# Self-Balancing Two-Wheel Robot

A complete Arduino-based self-balancing robot that can be controlled via USB serial commands. Perfect for learning about PID control, sensor fusion, and robotics!

![Robot Type: Two-Wheel Balancing Robot](https://img.shields.io/badge/Type-Self--Balancing-blue)
![Platform: Arduino](https://img.shields.io/badge/Platform-Arduino-green)
![Control: USB Serial](https://img.shields.io/badge/Control-USB%20Serial-orange)

## 📋 Overview

This project implements a two-wheel self-balancing robot (similar to a Segway) that:
- Automatically maintains balance using PID control
- Uses MPU6050 IMU for tilt sensing
- Accepts serial commands for movement (forward, backward, left, right)
- Can be controlled by a computer or Raspberry Pi via USB

## 🎯 Features

- ✅ **Self-balancing** - Uses PID control algorithm
- ✅ **Real-time control** - Responds to serial commands instantly
- ✅ **Sensor fusion** - Combines gyroscope and accelerometer data
- ✅ **Safety features** - Emergency stop on excessive tilt
- ✅ **Tunable parameters** - Easily adjust PID gains
- ✅ **Serial debugging** - Real-time monitoring via Serial Monitor
- ✅ **Timeout protection** - Stops if no command received
- ✅ **Well-documented** - Extensive comments and guides

## 📁 Project Files

```
balencing_robot/
├── self_balancing_robot.ino    # Main Arduino code
├── README.md                    # This file
├── PARTS_LIST.md               # Complete shopping list
├── WIRING_DIAGRAM.md           # Detailed wiring instructions
└── SETUP_AND_TUNING_GUIDE.md   # Setup, calibration, and tuning
```

## 🛠️ Required Components

### Core Components
- **Arduino Nano** or Uno
- **MPU6050** Gyroscope/Accelerometer module
- **L298N** Motor Driver
- **2x DC Geared Motors** (with encoders recommended)
- **2x Wheels** (65-80mm diameter)
- **Battery** (7.4V-12V LiPo or 18650 cells)
- **Chassis** (acrylic, 3D printed, or pre-made)

### Total Cost
- **Budget build:** $50-70
- **Recommended build:** $80-120

👉 See [PARTS_LIST.md](PARTS_LIST.md) for complete details and purchase links

## 📊 Technical Specifications

- **Control System:** PID with complementary filter
- **Update Rate:** 100 Hz (10ms loop)
- **Communication:** 115200 baud serial
- **Maximum Tilt:** 45° (configurable)
- **Response Time:** <100ms
- **Balance Accuracy:** ±2° (when tuned)

## 🔌 Quick Start

### 1. Get the Parts
See [PARTS_LIST.md](PARTS_LIST.md) for what to buy

### 2. Wire Everything
Follow [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md) for connections

### 3. Install Software
```bash
# Install Arduino IDE from arduino.cc
# Install MPU6050 library via Library Manager
```

### 4. Upload Code
```bash
1. Open self_balancing_robot.ino
2. Select board: Arduino Nano
3. Select port
4. Upload!
```

### 5. Calibrate & Tune
Follow [SETUP_AND_TUNING_GUIDE.md](SETUP_AND_TUNING_GUIDE.md) for step-by-step tuning

## 🎮 How to Control

### Serial Commands
Send single characters via Serial Monitor (115200 baud):

| Command | Action |
|---------|--------|
| `f` | Move forward |
| `b` | Move backward |
| `l` | Turn left |
| `r` | Turn right |
| `s` | Stop |

### Example: Python Control

```python
import serial
import time

# Connect to Arduino
robot = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for initialization

# Move forward for 2 seconds
robot.write(b'f')
time.sleep(2)

# Stop
robot.write(b's')
time.sleep(1)

# Turn left
robot.write(b'l')
time.sleep(1)

# Stop and close
robot.write(b's')
robot.close()
```

### Example: Node.js Control

```javascript
const SerialPort = require('serialport');
const port = new SerialPort('/dev/ttyUSB0', { baudRate: 115200 });

// Move forward
port.write('f');

setTimeout(() => {
  // Stop
  port.write('s');
}, 2000);
```

## ⚙️ Configuration

### Adjust PID Values
Located in the main `.ino` file:

```arduino
float Kp = 40.0;   // Proportional gain
float Ki = 0.8;    // Integral gain
float Kd = 1.2;    // Derivative gain
```

### Adjust Target Angle
```arduino
float targetAngle = -2.0;  // Depends on center of gravity
```

### Adjust Drive Speeds
```arduino
const int DRIVE_SPEED = 30;  // Forward/backward speed
const int TURN_SPEED = 25;   // Turning speed
```

See [SETUP_AND_TUNING_GUIDE.md](SETUP_AND_TUNING_GUIDE.md) for complete tuning instructions

## 🔧 Troubleshooting

### Robot won't balance
- ✓ Check MPU6050 is mounted level
- ✓ Calibrate MPU6050 while upright
- ✓ Adjust targetAngle for your center of gravity
- ✓ Tune PID gains (start with Kp)

### Motors don't spin
- ✓ Check battery voltage (> 7V)
- ✓ Remove ENA/ENB jumpers on L298N
- ✓ Verify wiring connections

### MPU6050 not found
- ✓ Check I2C connections (A4=SDA, A5=SCL)
- ✓ Run I2C scanner sketch
- ✓ Check 5V and GND connections

### Serial commands don't work
- ✓ Verify baud rate is 115200
- ✓ Check USB cable connection
- ✓ Try sending with newline character

See [SETUP_AND_TUNING_GUIDE.md](SETUP_AND_TUNING_GUIDE.md) for more troubleshooting

## 📚 How It Works

### 1. Sensor Reading
The MPU6050 measures:
- **Accelerometer** - Current tilt angle
- **Gyroscope** - Rate of tilt change

### 2. Sensor Fusion
A complementary filter combines both sensors:
```
filteredAngle = 0.98 × (angle + gyroRate × dt) + 0.02 × accAngle
```

### 3. PID Control
The PID controller calculates motor speed:
```
Error = currentAngle - targetAngle
Output = Kp×Error + Ki×∫Error + Kd×(dError/dt)
```

### 4. Motor Control
Motor speeds are adjusted to:
- Correct the tilt (from PID)
- Apply movement commands (forward/backward)
- Apply turning commands (left/right)

## 🎓 Learning Objectives

This project teaches:
- **PID Control** - Tuning and implementation
- **Sensor Fusion** - Combining multiple sensors
- **Motor Control** - PWM and H-bridge drivers
- **Serial Communication** - Command protocols
- **Real-time Systems** - Timing and control loops
- **Embedded Programming** - Arduino/C++

## 🚀 Advanced Modifications

### Add More Features
1. **Bluetooth Control** - Add HC-05 module
2. **RC Control** - Add RC receiver
3. **Autonomous Navigation** - Add ultrasonic sensors
4. **Speed Control** - Add wheel encoders
5. **LED Indicators** - Add status LEDs
6. **Battery Monitor** - Add voltage display
7. **Data Logging** - Log to SD card

### Performance Improvements
1. **Better PID** - Implement advanced PID variants
2. **Kalman Filter** - Replace complementary filter
3. **Speed Feedback** - Use encoder feedback in control loop
4. **Adaptive Control** - Auto-tune PID parameters

## ⚠️ Safety Notes

- ⚠️ Test with robot elevated before placing on ground
- ⚠️ Keep power switch easily accessible
- ⚠️ Don't over-discharge LiPo batteries (< 3.0V per cell)
- ⚠️ Motor driver can get hot - ensure ventilation
- ⚠️ Test in open space away from obstacles
- ⚠️ Start at low speeds, increase gradually

## 📖 Additional Resources

### Documentation
- [Arduino Reference](https://www.arduino.cc/reference/en/)
- [MPU6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [L298N Datasheet](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf)

### Tutorials
- [PID Control Tutorial](https://www.youtube.com/results?search_query=pid+control+tutorial)
- [Self-Balancing Robot Theory](https://www.youtube.com/results?search_query=self+balancing+robot+theory)
- [Arduino Motor Control](https://www.arduino.cc/en/Tutorial/BuiltInExamples)

### Community
- [Arduino Forums](https://forum.arduino.cc/)
- [Reddit r/arduino](https://www.reddit.com/r/arduino/)
- [Stack Overflow - Arduino Tag](https://stackoverflow.com/questions/tagged/arduino)

## 🤝 Contributing

This is a personal project, but feel free to:
- Suggest improvements
- Report bugs
- Share your build
- Create variations

## 📄 License

This project is provided as-is for educational purposes. Feel free to modify and use for personal projects.

## 🙏 Acknowledgments

Based on classic self-balancing robot designs with improvements for:
- Better sensor fusion
- More robust PID control
- Easier serial integration
- Comprehensive documentation

## 📞 Support

If you encounter issues:
1. Check [SETUP_AND_TUNING_GUIDE.md](SETUP_AND_TUNING_GUIDE.md)
2. Review [WIRING_DIAGRAM.md](WIRING_DIAGRAM.md)
3. Verify all connections
4. Test components individually
5. Check Serial Monitor for debug output

---

**Happy Building! 🤖**

*Created for learning robotics, PID control, and embedded systems*

