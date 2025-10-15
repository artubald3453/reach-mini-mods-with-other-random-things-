# Setup, Calibration, and Tuning Guide

## Table of Contents
1. [Software Setup](#software-setup)
2. [Hardware Testing](#hardware-testing)
3. [Initial Calibration](#initial-calibration)
4. [PID Tuning Guide](#pid-tuning-guide)
5. [Serial Communication](#serial-communication)
6. [Troubleshooting](#troubleshooting)

---

## Software Setup

### 1. Install Arduino IDE

1. Download from [arduino.cc](https://www.arduino.cc/en/software)
2. Install for your operating system
3. Launch Arduino IDE

### 2. Install Required Library

**MPU6050 Library Installation:**

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for "MPU6050"
4. Install **"MPU6050" by Electronic Cats** or **"MPU6050_tockn"**
   - Alternative: Install "I2Cdevlib MPU6050" by Jeff Rowberg

**Note:** The code uses the Electronic Cats MPU6050 library. If you use a different library, you may need to adjust the sensor reading functions.

### 3. Install CH340 Drivers (if needed)

If using a clone Arduino with CH340 USB chip:
- **Windows:** Download from manufacturer website
- **Mac:** [CH340 Mac Driver](https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver)
- **Linux:** Usually works out of the box

### 4. Upload the Code

1. Connect Arduino via USB
2. Select **Tools → Board → Arduino Nano** (or your board)
3. Select **Tools → Processor → ATmega328P** (or ATmega328P Old Bootloader)
4. Select **Tools → Port** → Choose your Arduino port
5. Open `self_balancing_robot.ino`
6. Click **Upload** (→ button)
7. Wait for "Done uploading"

---

## Hardware Testing

### Test 1: Power Check

**Without battery connected:**
1. Connect Arduino to computer via USB
2. Arduino power LED should light up
3. Open **Tools → Serial Monitor**
4. Set baud rate to **115200**
5. You should see initialization messages

**Expected Output:**
```
Self-Balancing Robot Initializing...
MPU6050 connection successful
Calibrating MPU6050... Keep robot upright and still!
```

### Test 2: MPU6050 Test

**With Arduino still on USB:**
1. Keep robot upright and still during calibration
2. After calibration, tilt the robot forward/backward
3. Watch Serial Monitor - angle values should change
4. Tilt forward = positive angle
5. Tilt backward = negative angle

**If MPU6050 not found:**
```arduino
// Run this I2C Scanner sketch first
#include <Wire.h>
void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Scanner");
}
void loop() {
  byte error, address;
  int devices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device at 0x");
      Serial.println(address, HEX);
      devices++;
    }
  }
  if (devices == 0) Serial.println("No I2C devices found");
  delay(5000);
}
```

Expected: Should find device at `0x68` (MPU6050)

### Test 3: Motor Test (WITH ROBOT ELEVATED!)

**Important:** Prop up robot so wheels don't touch ground!

1. Disconnect USB
2. Connect battery to L298N
3. Turn on power switch
4. Reconnect USB (for serial communication)
5. Type commands in Serial Monitor:
   - Type `f` and press Enter → wheels spin forward
   - Type `b` → wheels spin backward
   - Type `l` → wheels spin opposite (left turn)
   - Type `r` → wheels spin opposite (right turn)
   - Type `s` → wheels stop

**If motors don't work:**
- Check battery voltage (> 7V)
- Check all motor driver connections
- Verify ENA/ENB jumpers removed
- Try simple motor test sketch

---

## Initial Calibration

### 1. MPU6050 Calibration

**Automatic Calibration (built into code):**
1. Place robot perfectly upright on a level surface
2. Keep completely still
3. Power on robot
4. Wait for calibration (2 seconds)
5. Calibration complete!

**Manual Calibration (if automatic fails):**

Uncomment this in the code to print offsets:
```arduino
Serial.print("AX Offset: "); Serial.println(ax_offset);
Serial.print("AY Offset: "); Serial.println(ay_offset);
Serial.print("AZ Offset: "); Serial.println(az_offset);
Serial.print("GX Offset: "); Serial.println(gx_offset);
Serial.print("GY Offset: "); Serial.println(gy_offset);
Serial.print("GZ Offset: "); Serial.println(gz_offset);
```

Then set these offsets manually in setup():
```arduino
mpu.setXAccelOffset(YOUR_VALUE);
mpu.setYAccelOffset(YOUR_VALUE);
// etc...
```

### 2. Target Angle Calibration

The `targetAngle` is the angle at which the robot is balanced.

**Finding the correct targetAngle:**

1. Build robot with all components mounted
2. Hold robot at what feels like perfect balance point
3. Check the "Angle" value in Serial Monitor
4. That value is your target angle
5. Update in code:
```arduino
float targetAngle = -2.0;  // Replace with your value
```

**Common values:**
- Heavy battery on top: targetAngle = -3.0 to -5.0
- Battery on bottom: targetAngle = 0.0 to -2.0
- Depends on center of gravity!

---

## PID Tuning Guide

### Understanding PID

**PID = Proportional + Integral + Derivative**

- **Kp (Proportional):** Reacts to current error - higher = stronger correction
- **Ki (Integral):** Fixes steady-state error - higher = faster to reach target
- **Kd (Derivative):** Dampens oscillations - higher = smoother response

### Default Starting Values (in code)

```arduino
float Kp = 40.0;
float Ki = 0.8;
float Kd = 1.2;
```

### Tuning Process (Ziegler-Nichols Method Simplified)

**IMPORTANT:** Start with robot elevated or held by hand!

#### Step 1: Tune Kp Only

1. Set Ki = 0 and Kd = 0
2. Start with Kp = 10
3. Increase Kp until robot responds to tilt
4. Keep increasing until robot starts to oscillate
5. Reduce Kp by 20% from oscillation point

**Example:**
```arduino
Kp = 20;  // Start here
Kp = 30;  // Getting responsive
Kp = 40;  // Good response
Kp = 50;  // Starting to oscillate
Kp = 40;  // Back off to 80% of 50
```

#### Step 2: Tune Kd

1. Keep Kp from Step 1
2. Start with Kd = 0.5
3. Increase Kd to reduce oscillations
4. Stop when robot is smooth but still responsive

**Example:**
```arduino
Kd = 0.5;  // Still oscillating
Kd = 1.0;  // Better
Kd = 1.5;  // Smoother
Kd = 2.0;  // Too sluggish, go back
Kd = 1.5;  // Good!
```

#### Step 3: Tune Ki

1. Keep Kp and Kd from previous steps
2. Start with Ki = 0.1
3. Increase Ki slowly to eliminate steady-state error
4. Stop if robot becomes unstable

**Example:**
```arduino
Ki = 0.1;  // Robot slowly drifts
Ki = 0.5;  // Better
Ki = 1.0;  // Stable
Ki = 2.0;  // Unstable, go back
Ki = 1.0;  // Perfect!
```

### Quick Tuning Reference

| Problem | Solution |
|---------|----------|
| Robot falls over immediately | Decrease Kp, check targetAngle |
| Robot oscillates back/forth | Increase Kd |
| Robot drifts slowly | Increase Ki (carefully) |
| Robot overshoots | Decrease Kp |
| Robot too slow to respond | Increase Kp |
| Robot unstable | Decrease Ki, increase Kd |

### Advanced Tuning Tips

**1. Test on flat, smooth surface** (tiles or hardwood, not carpet)

**2. Adjust drive speeds** if robot is too aggressive:
```arduino
const int DRIVE_SPEED = 30;  // Lower = gentler
const int TURN_SPEED = 25;   // Lower = slower turns
```

**3. Change complementary filter** if angle readings noisy:
```arduino
const float ALPHA = 0.98;  // Higher = trust gyro more (0-1)
```

**4. Adjust MAX_ANGLE** for safety:
```arduino
const int MAX_ANGLE = 45;  // Lower = stop sooner
```

### Tuning for Different Weights

**Lighter robot (< 500g):**
```arduino
Kp = 30.0;
Ki = 0.5;
Kd = 1.0;
```

**Medium robot (500g - 1kg):**
```arduino
Kp = 40.0;
Ki = 0.8;
Kd = 1.2;
```

**Heavy robot (> 1kg):**
```arduino
Kp = 50.0;
Ki = 1.0;
Kd = 1.5;
```

---

## Serial Communication

### Commands

| Command | Action | Example |
|---------|--------|---------|
| `f` | Move forward | Type `f` in Serial Monitor |
| `b` | Move backward | Type `b` in Serial Monitor |
| `l` | Turn left | Type `l` in Serial Monitor |
| `r` | Turn right | Type `r` in Serial Monitor |
| `s` | Stop | Type `s` in Serial Monitor |

### Using with Raspberry Pi

**Python Example:**
```python
import serial
import time

# Open serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

# Send commands
ser.write(b'f')  # Forward
time.sleep(2)
ser.write(b's')  # Stop
time.sleep(1)
ser.write(b'l')  # Left
time.sleep(1)
ser.write(b's')  # Stop

ser.close()
```

### Using with External Service

**Node.js Example:**
```javascript
const SerialPort = require('serialport');

const port = new SerialPort('/dev/ttyUSB0', {
  baudRate: 115200
});

// Send forward command
port.write('f', (err) => {
  if (err) {
    return console.log('Error:', err.message);
  }
  console.log('Command sent');
});
```

### Command Timeout

Robot automatically stops if no command received for 500ms:
```arduino
const unsigned long COMMAND_TIMEOUT = 500; // Adjust as needed
```

---

## Troubleshooting

### Problem: Robot won't balance

**Solutions:**
1. Check MPU6050 is mounted level and centered
2. Calibrate MPU6050 again
3. Adjust targetAngle
4. Reduce Kp, increase Kd
5. Verify motors spin correct direction
6. Check battery voltage (> 7.4V)

### Problem: Robot oscillates rapidly

**Solutions:**
1. Increase Kd (damping)
2. Decrease Kp
3. Check for loose connections
4. Verify MPU6050 mounting is solid
5. Lower ALPHA (trust accelerometer more)

### Problem: Robot falls in one direction

**Solutions:**
1. Adjust targetAngle
2. Check center of gravity (move battery)
3. Verify both motors have equal power
4. Check wheel alignment

### Problem: Motors are weak

**Solutions:**
1. Charge battery (should be > 7.4V)
2. Check motor driver connections
3. Verify ENA/ENB connected to PWM pins
4. Check motor driver isn't overheating

### Problem: Serial commands don't work

**Solutions:**
1. Check baud rate is 115200
2. Verify USB cable is connected
3. Try sending commands with newline/carriage return
4. Check Serial Monitor settings

### Problem: Arduino resets when motors start

**Solutions:**
1. Battery voltage too low - charge it
2. Add 100µF capacitor across motor terminals
3. Ensure solid common ground
4. Use higher capacity battery

---

## Performance Optimization

### 1. Faster Response

```arduino
Kp = 60.0;  // Increase proportional gain
DRIVE_SPEED = 40;  // Faster driving
```

### 2. Smoother Balance

```arduino
Kd = 2.0;  // Increase derivative gain
ALPHA = 0.99;  // Trust gyro more
```

### 3. Better Stability

```arduino
ALPHA = 0.96;  // Trust accelerometer more
Ki = 1.5;  // Stronger integral action
```

---

## Next Steps

1. ✅ Upload code
2. ✅ Test MPU6050
3. ✅ Test motors
4. ✅ Calibrate target angle
5. ✅ Tune PID (start with Kp)
6. ✅ Test on flat surface
7. ✅ Fine-tune for smooth operation
8. ✅ Test serial commands
9. ✅ Integrate with Raspberry Pi (optional)
10. ✅ Enjoy your self-balancing robot!

---

## Safety Reminders

- Always test with robot elevated first
- Keep power switch accessible
- Don't over-discharge LiPo batteries
- Watch for overheating motor driver
- Test on open floor away from obstacles
- Start with low speeds, increase gradually

---

## Need Help?

**Common Resources:**
- Arduino Forums: forum.arduino.cc
- MPU6050 Datasheet: Search "MPU6050 datasheet"
- PID Tuning: Search "PID tuning for self-balancing robot"
- Video Tutorials: YouTube "Arduino self balancing robot"

**Debugging Tips:**
1. Use Serial Monitor to print values
2. Test components individually
3. Check wiring against diagram
4. Verify power supply voltage
5. Take breaks and come back fresh!

Good luck with your build! 🤖

