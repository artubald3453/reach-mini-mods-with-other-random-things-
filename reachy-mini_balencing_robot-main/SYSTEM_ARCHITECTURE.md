# System Architecture

## High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    SELF-BALANCING ROBOT                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────┐         ┌──────────────┐       ┌──────────┐  │
│  │          │  USB    │              │  I2C  │          │  │
│  │ Computer ├────────►│   Arduino    ├───────┤ MPU6050  │  │
│  │  / Pi    │ Serial  │     Nano     │       │   IMU    │  │
│  │          │         │              │       │          │  │
│  └──────────┘         └──────┬───────┘       └──────────┘  │
│                              │                              │
│                              │ PWM + Direction              │
│                              │                              │
│                       ┌──────▼───────┐                      │
│                       │              │                      │
│                       │     L298N    │                      │
│                       │Motor Driver  │                      │
│                       │              │                      │
│                       └──────┬───────┘                      │
│                              │                              │
│                    ┌─────────┴─────────┐                    │
│                    │                   │                    │
│              ┌─────▼─────┐       ┌─────▼─────┐             │
│              │           │       │           │             │
│              │  Motor A  │       │  Motor B  │             │
│              │  (Left)   │       │  (Right)  │             │
│              │           │       │           │             │
│              └─────┬─────┘       └─────┬─────┘             │
│                    │                   │                    │
│              ┌─────▼─────┐       ┌─────▼─────┐             │
│              │  Wheel A  │       │  Wheel B  │             │
│              └───────────┘       └───────────┘             │
│                                                             │
│  Power: ◄────────── Battery (7.4V-12V) ────────────►       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Control Flow Diagram

```
START
  │
  ├─► Initialize Hardware
  │     ├─ Setup motor pins
  │     ├─ Initialize I2C
  │     ├─ Connect to MPU6050
  │     └─ Calibrate sensors
  │
  ├─► MAIN LOOP (100 Hz)
  │     │
  │     ├─► Read MPU6050 Sensors
  │     │     ├─ Read accelerometer (current angle)
  │     │     ├─ Read gyroscope (rate of change)
  │     │     └─ Apply complementary filter
  │     │
  │     ├─► Safety Check
  │     │     └─ If |angle| > 45° → EMERGENCY STOP
  │     │
  │     ├─► Read Serial Commands
  │     │     ├─ Check for 'f', 'b', 'l', 'r', 's'
  │     │     └─ Apply timeout (500ms)
  │     │
  │     ├─► Calculate PID Output
  │     │     ├─ Error = currentAngle - targetAngle
  │     │     ├─ P = Kp × Error
  │     │     ├─ I = Ki × ∫Error
  │     │     ├─ D = Kd × (dError/dt)
  │     │     └─ Output = P + I + D
  │     │
  │     ├─► Apply Commands
  │     │     ├─ baseSpeed (forward/backward)
  │     │     └─ turnSpeed (left/right)
  │     │
  │     ├─► Calculate Motor Speeds
  │     │     ├─ motorA = pidOutput + baseSpeed - turnSpeed
  │     │     └─ motorB = pidOutput + baseSpeed + turnSpeed
  │     │
  │     ├─► Drive Motors
  │     │     ├─ Set direction (IN1-IN4)
  │     │     └─ Set speed (PWM on ENA/ENB)
  │     │
  │     └─► Debug Output (optional)
  │           └─ Print angle, PID, speeds to Serial
  │
  └─► Loop back (every 10ms)
```

## Data Flow - Balancing

```
  MPU6050 Sensors
        │
        ├─► Accelerometer → Angle (from gravity)
        │         │
        │         └─► accAngle
        │
        └─► Gyroscope → Rate of change
                  │
                  └─► gyroRate
                        │
                        ▼
              Complementary Filter
           (98% gyro + 2% accelerometer)
                        │
                        ▼
                 filteredAngle ──┐
                                 │
                   targetAngle ──┤
                                 │
                                 ▼
                          angleError = filteredAngle - targetAngle
                                 │
                    ┌────────────┼────────────┐
                    │            │            │
                    ▼            ▼            ▼
              P = Kp×Error  I = Ki×∫Error  D = Kd×dError/dt
                    │            │            │
                    └────────────┼────────────┘
                                 │
                                 ▼
                            pidOutput
                                 │
              ┌──────────────────┼──────────────────┐
              │                                     │
        ┌─────▼─────┐                         ┌─────▼─────┐
        │ Motor A   │                         │ Motor B   │
        │ Speed     │                         │ Speed     │
        └───────────┘                         └───────────┘
```

## Data Flow - Movement Commands

```
  Serial Input (USB)
        │
        ├─► 'f' → baseSpeed = +30, turnSpeed = 0
        ├─► 'b' → baseSpeed = -30, turnSpeed = 0
        ├─► 'l' → baseSpeed = 0,   turnSpeed = +25
        ├─► 'r' → baseSpeed = 0,   turnSpeed = -25
        └─► 's' → baseSpeed = 0,   turnSpeed = 0
              │
              ▼
        Combined with PID output
              │
        ┌─────┴─────┐
        │           │
        ▼           ▼
  motorSpeedA    motorSpeedB
  = pidOutput    = pidOutput
  + baseSpeed    + baseSpeed
  - turnSpeed    + turnSpeed
        │           │
        └─────┬─────┘
              │
              ▼
     Drive Motors (PWM)
```

## Sensor Fusion (Complementary Filter)

```
Accelerometer                      Gyroscope
(Measures angle)                   (Measures rate)
      │                                  │
      │ Pros:                            │ Pros:
      │ - Accurate long-term             │ - Fast response
      │ Cons:                            │ Cons:
      │ - Noisy short-term               │ - Drifts over time
      │                                  │
      └───────────┬──────────────────────┘
                  │
                  ▼
         Complementary Filter
         α = 0.98 (trust gyro)
         1-α = 0.02 (trust accel)
                  │
   angle = α×(angle + gyro×dt) + (1-α)×accel
                  │
                  ▼
           Filtered Angle
        (Best of both sensors!)
```

## PID Controller Behavior

```
                        Target Angle (setpoint)
                               │
                               ▼
     Current Angle ────► [Compare] ────► Error
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
        ▼                      ▼                      ▼
  [P: React NOW]      [I: Fix Steady]     [D: Dampen Change]
        │                      │                      │
   Kp × Error         Ki × ∫Error dt       Kd × dError/dt
        │                      │                      │
        └──────────────────────┼──────────────────────┘
                               │
                               ▼
                          PID Output
                               │
                               ▼
                         Motor Control


Kp too high  → Oscillation, instability
Kp too low   → Slow response, falls over

Ki too high  → Overshoot, oscillation
Ki too low   → Steady-state error (drift)

Kd too high  → Sluggish, overdamped
Kd too low   → Oscillation, underdamped
```

## Motor Control Strategy

```
Balancing (PID)        Forward/Backward (baseSpeed)
      │                         │
      │                         │
      ▼                         ▼
  pidOutput                baseSpeed = +30 (forward)
      │                    baseSpeed = -30 (backward)
      │                         │
      └──────────┬──────────────┘
                 │
                 │        Turning (turnSpeed)
                 │               │
                 ▼               ▼
           motorSpeedA      turnSpeed = +25 (left)
           motorSpeedB      turnSpeed = -25 (right)
                 │               │
                 └───────┬───────┘
                         │
                         ▼
            
   Motor A (Left)              Motor B (Right)
   = pid + base - turn         = pid + base + turn
   
   Examples:
   
   Balancing only:
   A = pid + 0 - 0 = pid
   B = pid + 0 + 0 = pid
   (Both motors same speed)
   
   Forward:
   A = pid + 30 - 0 = pid + 30
   B = pid + 30 + 0 = pid + 30
   (Both motors faster forward)
   
   Left turn:
   A = pid + 0 - 25 = pid - 25
   B = pid + 0 + 25 = pid + 25
   (Right motor faster → turns left)
```

## Physical Robot Layout

```
    TOP VIEW
    
    ┌───────────────┐
    │   Battery     │  ← Heavy (on bottom or centered)
    │   ∿∿∿∿∿∿∿     │
    └───────┬───────┘
            │
    ┌───────┴───────┐
    │   Arduino     │  ← Brain (top level)
    │   + MPU6050   │
    └───────┬───────┘
            │
    ┌───────┴───────┐
    │    L298N      │  ← Motor driver (middle level)
    └───┬───────┬───┘
        │       │
    ┌───▼───┐ ┌───▼───┐
    │Motor │ │Motor │  ← Motors on sides
    │   A  │ │   B  │
    └───┬───┘ └───┬───┘
        │       │
      ┌─▼─┐   ┌─▼─┐
      │ ○ │   │ ○ │    ← Wheels
      │   │   │   │
      └───┘   └───┘
    
    
    SIDE VIEW
    
            Arduino + MPU6050 (must be level!)
                 ┌─────┐
                 │▓▓▓▓▓│
    Battery      ├─────┤      ← Center of gravity here
    ┌──────┐     │  ○  │        (adjust targetAngle)
    │∿∿∿∿∿ │     │     │
    └──────┘     │L298N│
         ▲       ├─────┤
         │       │  │  │
    Heavy!       └──┼──┘
                    │
               ┌────┴────┐
               │ Motor   │
               └────┬────┘
                    │
                  ┌─▼─┐
                  │ ○ │  Wheel
                  │   │
                  └───┘
```

## Power Distribution

```
Battery (7.4V - 12V LiPo)
    │
    ├─► Switch (safety cutoff)
    │
    └─► L298N Motor Driver
          │
          ├─► Motor A (12V)
          │
          ├─► Motor B (12V)
          │
          └─► 5V Regulator (on L298N)
                │
                ├─► Arduino Nano (5V)
                │     │
                │     └─► MPU6050 (5V)
                │
                └─► Common Ground (critical!)
```

## Communication Protocol

```
Computer/Pi                    Arduino
    │                             │
    ├─────── USB Serial ──────────┤
    │       (115200 baud)          │
    │                             │
    ├──► Send: 'f'                │
    │                             ├──► Receive: 'f'
    │                             ├──► Set baseSpeed = 30
    │                             └──► Drive motors
    │                             │
    │◄─── Debug: "Angle: 2.3..." ─┤
    │                             │
    │                             │
    ├──► Send: 's'                │
    │                             ├──► Receive: 's'
    │                             ├──► Set baseSpeed = 0
    │                             └──► Balance only
    │                             │
    
    Timeout: If no command for 500ms → auto-stop
```

## Timing Diagram

```
Time (ms)  0    10   20   30   40   50   60   70   80   90   100
           │    │    │    │    │    │    │    │    │    │    │
Main Loop: ├────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤
           └►Loop1   Loop2   Loop3   Loop4   Loop5  etc...
              ▲       ▲       ▲       ▲       ▲
              │       │       │       │       │
Each loop:    │       │       │       │       │
  1. Read sensor (2ms)
  2. Calculate PID (1ms)
  3. Drive motors (1ms)
  4. Debug output (conditional)
  5. Delay to maintain 10ms cycle
  
Update rate: 100 Hz (fast enough for balancing)
```

## Summary

- **Input:** MPU6050 angle + Serial commands
- **Processing:** PID control algorithm
- **Output:** Motor speeds (PWM)
- **Goal:** Keep robot upright while following commands
- **Loop Rate:** 100 Hz (10ms per cycle)
- **Response Time:** < 100ms
- **Control Method:** PID with complementary filter

