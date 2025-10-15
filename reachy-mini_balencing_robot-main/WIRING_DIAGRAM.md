# Wiring Diagram for Self-Balancing Robot

## Connection Overview

```
Battery --> L298N Motor Driver --> Motors
         |
         --> Arduino Nano (via 5V regulator on L298N)
         
Arduino Nano <--> MPU6050 (I2C)
Arduino Nano <--> L298N Motor Driver (control pins)
Arduino Nano <--> Computer (via USB)
```

## Detailed Pin Connections

### 1. MPU6050 to Arduino Nano

| MPU6050 Pin | Arduino Nano Pin | Description |
|-------------|------------------|-------------|
| VCC         | 5V               | Power supply |
| GND         | GND              | Ground |
| SCL         | A5               | I2C Clock |
| SDA         | A4               | I2C Data |
| XDA         | (not connected)  | Optional |
| XCL         | (not connected)  | Optional |
| AD0         | GND              | I2C Address Select (low) |
| INT         | (not connected)  | Optional interrupt pin |

**Note:** On Arduino Uno, SCL = A5, SDA = A4. Some boards have dedicated SCL/SDA pins.

### 2. L298N Motor Driver to Arduino Nano

| L298N Pin   | Arduino Nano Pin | Description |
|-------------|------------------|-------------|
| IN1         | D7               | Motor A direction control 1 |
| IN2         | D8               | Motor A direction control 2 |
| ENA         | D9 (PWM)         | Motor A speed control (PWM) |
| IN3         | D4               | Motor B direction control 1 |
| IN4         | D5               | Motor B direction control 2 |
| ENB         | D10 (PWM)        | Motor B speed control (PWM) |
| GND         | GND              | Common ground |
| 5V OUT      | 5V               | 5V output from L298N regulator |

**Important Notes:**
- Remove the jumpers from ENA and ENB on the L298N if they have them
- The 5V OUT can power the Arduino (connect to 5V pin, NOT VIN)
- Ensure common ground between Arduino and L298N

### 3. Motors to L298N

| L298N Terminal | Connection |
|----------------|------------|
| Motor A (+)    | Left Motor Wire 1 |
| Motor A (-)    | Left Motor Wire 2 |
| Motor B (+)    | Right Motor Wire 1 |
| Motor B (-)    | Right Motor Wire 2 |

**Note:** If motor spins in wrong direction, swap the two wires for that motor.

### 4. Battery to L298N

| Battery     | L298N Pin |
|-------------|-----------|
| Positive (+)| +12V      |
| Negative (-)| GND       |

**Power Switch:** Install switch on positive wire between battery and L298N.

## Complete Wiring Diagram (ASCII Art)

```
                    SELF-BALANCING ROBOT WIRING
                    
                         [Battery 7.4V-12V]
                               |
                          [ON/OFF Switch]
                               |
                    +----------+----------+
                    |                     |
                    |                     |
             +------+------+              |
             |             |              |
             |   L298N     |              |
             |   Motor     |              |
             |   Driver    |              |
             |             |              |
             +------+------+              |
                    |                     |
         +----------+----------+          |
         |          |          |          |
    [Motor A]  [Motor B]   [5V Out]      |
     (Left)     (Right)       |          |
         |          |          |          |
         |          |      +---+---+      |
         |          |      |Arduino|      |
         |          |      | Nano  |      |
         |          |      |       |      |
         |          |      +---+---+      |
         |          |          |          |
         |          |      [I2C Bus]      |
         |          |          |          |
         |          |      +---+---+      |
         |          |      |MPU6050|      |
         |          |      +-------+      |
         |          |                     |
        [Wheel]   [Wheel]            [USB Cable]
                                          |
                                    [Computer/Pi]
```

## Detailed Component Layout

### Power Distribution

```
Battery (7.4V - 12V)
    |
    +--> Power Switch
           |
           +--> L298N +12V Input
                 |
                 +--> L298N 5V Regulator Output
                       |
                       +--> Arduino Nano 5V pin
                       +--> MPU6050 VCC
```

### Signal Flow

```
Computer/Pi (USB Serial)
    |
    +--> Arduino Nano
           |
           +--> MPU6050 (I2C: reads angle data)
           |
           +--> L298N Motor Driver (PWM + Direction)
                 |
                 +--> Motor A (Left)
                 +--> Motor B (Right)
```

## Step-by-Step Assembly Instructions

### Step 1: Prepare Components
1. Lay out all components
2. Test Arduino with USB (upload blink sketch)
3. Test MPU6050 (use I2C scanner sketch)

### Step 2: Mount Components on Chassis
1. Mount motors to chassis
2. Attach wheels to motor shafts
3. Mount Arduino on top plate (use standoffs)
4. Mount L298N on middle plate
5. Mount MPU6050 centered and level (critical!)
6. Mount battery holder/battery

### Step 3: Wiring - Power Connections First
1. Connect battery to L298N (+12V and GND)
2. Install power switch in positive wire
3. Connect L298N 5V OUT to Arduino 5V
4. Connect all grounds together (Arduino, L298N, MPU6050)

### Step 4: Wiring - Motor Connections
1. Connect motors to L298N output terminals
2. Test motor direction with simple Arduino sketch
3. Swap wires if motors spin wrong direction

### Step 5: Wiring - Control Signals
1. Connect Arduino D7, D8, D9 to L298N IN1, IN2, ENA
2. Connect Arduino D4, D5, D10 to L298N IN3, IN4, ENB
3. Double-check PWM pins (D9 and D10)

### Step 6: Wiring - MPU6050
1. Connect MPU6050 VCC to Arduino 5V
2. Connect MPU6050 GND to Arduino GND
3. Connect MPU6050 SCL to Arduino A5
4. Connect MPU6050 SDA to Arduino A4
5. **IMPORTANT:** Mount MPU6050 perfectly level and centered!

### Step 7: Cable Management
1. Use zip ties or tape to secure wires
2. Keep wires away from wheels
3. Ensure no wires can get caught in motors
4. Use heat shrink on any exposed connections

## Troubleshooting Common Wiring Issues

### Motors Don't Spin
- Check battery voltage (should be > 7V)
- Check L298N power LED
- Verify motor connections are tight
- Remove ENA/ENB jumpers on L298N
- Check Arduino is sending PWM signals

### Motors Spin Wrong Direction
- Swap the two wires for that motor on L298N terminals
- Or modify code to invert the direction signals

### MPU6050 Not Found
- Check I2C connections (A4=SDA, A5=SCL)
- Try I2C scanner sketch
- Check 5V and GND connections
- Some MPU6050 modules need pull-up resistors (usually built-in)

### Robot Falls Over Immediately
- Check MPU6050 mounting (must be level!)
- Verify MPU6050 orientation in code
- Tune PID values (start with lower Kp)
- Check targetAngle setting

### Arduino Resets When Motors Start
- Battery voltage too low (charge battery)
- Add capacitor across motor terminals (100µF)
- Ensure solid ground connections
- Use separate power for motors and logic (if needed)

## Safety Warnings

⚠️ **IMPORTANT SAFETY NOTES:**

1. **LiPo Batteries:** Never over-discharge (< 3.0V per cell). Use low-voltage alarm!
2. **Short Circuits:** Double-check all connections before powering on
3. **Motor Wires:** Can cause shorts - use heat shrink or electrical tape
4. **Moving Parts:** Keep fingers away from wheels and gears when powered
5. **Start Slow:** Test at low speed first, increase gradually
6. **Emergency Stop:** Have power switch easily accessible
7. **Heat:** L298N can get hot - ensure ventilation

## Optional Improvements

### Add Voltage Display
```
Battery --> Voltage Display Module --> L298N
              (3-wire, DC 2.5-30V)
```

### Add Buzzer for Alerts
```
Arduino D12 --> (+) Buzzer --> GND
```

### Add LED Indicators
```
Arduino D13 --> 330Ω Resistor --> LED (+) --> GND
```

## Next Steps

1. Complete all wiring per diagram above
2. Triple-check all connections
3. Power on and verify LEDs light up
4. Upload Arduino code
5. Test MPU6050 readings via Serial Monitor
6. Calibrate MPU6050
7. Test motor control
8. Tune PID values
9. Test balancing!

## Testing Checklist

- [ ] Power LED on Arduino lights up
- [ ] Power LED on L298N lights up
- [ ] MPU6050 detected via I2C
- [ ] Both motors spin forward together
- [ ] Both motors spin backward together
- [ ] Left turn works (motors spin opposite)
- [ ] Right turn works (motors spin opposite)
- [ ] Serial monitor shows angle readings
- [ ] Tilt robot - angle changes correctly
- [ ] Ready for balancing tests!

