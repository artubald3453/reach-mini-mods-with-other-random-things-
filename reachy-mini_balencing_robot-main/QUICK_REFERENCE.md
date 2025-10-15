# Quick Reference Card

## Pin Connections

### MPU6050 → Arduino
```
VCC → 5V    |    GND → GND
SCL → A5    |    SDA → A4
```

### L298N → Arduino
```
IN1 → D7    |    IN2 → D8    |    ENA → D9 (PWM)
IN3 → D4    |    IN4 → D5    |    ENB → D10 (PWM)
GND → GND   |    5V OUT → 5V
```

### Battery → L298N
```
Battery + → L298N +12V (through switch)
Battery - → L298N GND
```

## Serial Commands
```
f = Forward  |  b = Backward
l = Left     |  r = Right
s = Stop
```

## Default PID Values
```arduino
Kp = 40.0    // Proportional gain
Ki = 0.8     // Integral gain
Kd = 1.2     // Derivative gain
targetAngle = -2.0  // Balance point
```

## Tuning Quick Guide

### Step 1: Tune Kp
```
Set Ki=0, Kd=0
Start Kp=10
Increase until oscillates
Reduce by 20%
```

### Step 2: Tune Kd
```
Keep Kp from step 1
Start Kd=0.5
Increase until smooth
```

### Step 3: Tune Ki
```
Keep Kp and Kd
Start Ki=0.1
Increase slowly
Stop if unstable
```

## Troubleshooting

| Problem | Quick Fix |
|---------|-----------|
| Falls immediately | Lower Kp, check targetAngle |
| Oscillates | Increase Kd |
| Drifts slowly | Increase Ki |
| MPU6050 not found | Check A4/A5 connections |
| Motors don't work | Remove ENA/ENB jumpers |
| Weak motors | Charge battery (>7.4V) |

## Python Control Example
```python
import serial, time
robot = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)
robot.write(b'f')  # Forward
time.sleep(2)
robot.write(b's')  # Stop
robot.close()
```

## Safety Checklist
- [ ] Test elevated first
- [ ] Power switch accessible
- [ ] Battery charged (>7V)
- [ ] MPU6050 mounted level
- [ ] All wires secured
- [ ] Test on flat surface

## Software Setup
```bash
1. Install Arduino IDE
2. Tools → Manage Libraries → Install "MPU6050"
3. Select Board: Arduino Nano
4. Select Port
5. Upload code
```

## Parts Quick List
- Arduino Nano
- MPU6050 module
- L298N motor driver
- 2x DC motors + wheels
- 7.4V-12V battery
- Chassis
- Wiring

**Total: $50-120**

---

📖 **Full docs:** See README.md, PARTS_LIST.md, WIRING_DIAGRAM.md, SETUP_AND_TUNING_GUIDE.md

