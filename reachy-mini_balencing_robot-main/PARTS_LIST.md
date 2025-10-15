# Self-Balancing Robot - Complete Parts List
## Based on 3D Printed Design with Teensy 4.0 & Stepper Motors

---

## ⚠️ IMPORTANT NOTES

This parts list is specifically for the **3D printed self-balancing robot design** from Thingiverse. The design uses:
- **Teensy 4.0** microcontroller (600 MHz ARM Cortex-M7)
- **NEMA 17 Stepper motors**
- **DRV8825 stepper drivers**
- **HC-05 Bluetooth module** for wireless control
- **3D printed chassis** (STL files included)

**Original Design:** https://www.thingiverse.com/thing:4593553  
**Video Tutorial:** https://youtu.be/2k_8KneGMak

---

## Bill of Materials (From Original Design)

### 1. **Teensy 4.0 Microcontroller** ⭐ CRITICAL
- **NOT Arduino Mega/Nano/Uno** - Must be Teensy 4.0
- ARM Cortex-M7 at 600 MHz
- Multiple hardware serial ports
- USB programming
- **Amazon Link:** https://www.amazon.com/gp/product/B08259KDHY
- **Official Store:** https://www.pjrc.com/store/teensy40.html
- **Price:** ~$20-25
- **Quantity:** 1
- **Alternative:** Teensy 4.1 (more expensive, has extra features)

### 2. **MPU6050 IMU Module (GY-521)**
- 6-axis gyroscope + accelerometer
- I2C interface
- Must support DMP (Digital Motion Processor)
- **Amazon Link:** https://www.amazon.com/gp/product/B00LP25V1A
- **Price:** ~$6-10
- **Quantity:** 1
- **Search term:** "GY-521 MPU6050"

### 3. **HC-05 Bluetooth Module**
- For wireless control from Android phone
- Serial communication (3.3V logic compatible with Teensy)
- **Amazon Link:** https://www.amazon.com/gp/product/B07VL725T8
- **Price:** ~$8-12
- **Quantity:** 1
- **Note:** Can use HC-06 as alternative (slave-only mode)

### 4. **NEMA 17 Stepper Motors (x2)**
- Bipolar stepper motors
- 200 steps per revolution (1.8° step angle)
- Holding torque: ~40 Ncm (56 oz-in)
- **Amazon Link:** https://www.amazon.com/gp/product/B07LF898KN
- **Price:** ~$20-30 for pair
- **Quantity:** 2
- **Specs:** 12V, 1.5A per phase

### 5. **DRV8825 Stepper Motor Drivers (x2)**
- Up to 1/32 microstepping
- 2.2A max current per coil
- Includes heatsinks
- **Amazon Link:** https://www.amazon.com/gp/product/B07QJVCFRZ
- **Price:** ~$10-15 for pack
- **Quantity:** 2 (one per motor)
- **Important:** Must set current limit correctly!

### 6. **3S LiPo Battery**
- Voltage: 11.1V (3S)
- Capacity: 1300-2200mAh (1800mAh recommended)
- Discharge rate: 20C minimum
- XT60 connector
- **Amazon Option 1:** https://www.amazon.com/gp/product/B087CQR92J
- **Amazon Option 2:** https://www.amazon.com/gp/product/B0072AEYLW
- **Price:** ~$20-35
- **Quantity:** 1
- **Note:** Must fit inside chassis - check dimensions!

### 7. **5V/6V Step-Down Converter (UBEC)**
- Hobbywing 3A UBEC or similar
- Input: 7-26V (3S LiPo compatible)
- Output: 5V or 6V selectable
- 3A continuous output
- **Banggood Link:** https://www.banggood.com/Hobbywing-3A-UBEC-5V-6V-Switch-Mode-BEC-For-RC-Models-p-915037.html
- **Alternative:** Any 5V 3A buck converter
- **Price:** ~$8-15
- **Quantity:** 1
- **Purpose:** Powers Teensy 4.0 from battery

### 8. **XT60 Connectors**
- Male + Female pairs
- For battery connections
- **Amazon Link:** https://www.amazon.com/MCIGICM-Female-Bullet-Connectors-Battery/dp/B07DVDKL42
- **Price:** ~$8-12 for pack
- **Quantity:** 2-3 pairs (have spares)

### 9. **Breadboard**
- Half-size or full-size breadboard
- 400 or 830 tie points
- For prototyping connections
- **Amazon Link:** https://www.amazon.com/gp/product/B00LSG5BJK
- **Price:** ~$6-10
- **Quantity:** 1
- **Note:** Can be replaced with permanent soldering later

### 10. **Jumper Wires**
- **Breadboard Jumper Wires** (various lengths)
  - Male-to-Male
  - **Amazon Link:** https://www.amazon.com/MCIGICM-Breadboard-Jumper-Cables-Arduino/dp/B081GMJVPB
  - **Quantity:** 1 pack
  
- **Dupont Wires** (20-30cm)
  - Male-to-Male, Male-to-Female, Female-to-Female
  - **Amazon Link:** https://www.amazon.com/gp/product/B07GD2869Z
  - **Price:** ~$8-15 total
  - **Quantity:** 1 pack of each type

### 11. **M3 Screws and Nuts**
- Various lengths (6mm, 8mm, 10mm, 12mm, 16mm, 20mm)
- Assortment kit recommended
- **Amazon Link:** https://www.amazon.com/gp/product/B01J7NM9JA
- **Price:** ~$12-18
- **Quantity:** 1 assortment kit
- **Includes:** Screws, nuts, washers

### 12. **Wheels (x2)**
- Diameter: ~60-80mm
- Must fit 5mm motor shaft
- **Option A:** Bicycle training wheels (repurposed)
  - **Amazon Link:** https://www.amazon.com/Training-Reinforced-Prevent-Breaking-Bending/dp/B07RV5T18G
  - **Note:** May need modification
  
- **Option B:** 3D print your own wheels
  - Use `WheelDontPrint_OnlyForReference.STL` as guide
  - Add rubber O-rings or bands for traction
  - **Recommended:** Print wheel hubs, add 70mm rubber bands

- **Price:** ~$10-20
- **Quantity:** 2

### 13. **100µF Electrolytic Capacitors**
- Voltage rating: 16V or higher
- For stepper driver stability
- **Price:** ~$5 for pack
- **Quantity:** 2 minimum (one per driver)
- **Purpose:** Prevent voltage spikes, improve driver stability

---

## Additional Requirements (Tools & Materials)

### Essential Tools
- ✅ **Soldering iron** (for permanent connections)
- ✅ **Solder** (60/40 or lead-free)
- ✅ **Heat shrink tubing** (various sizes)
- ✅ **Wire strippers**
- ✅ **Screwdrivers** (Phillips #1, #2)
- ✅ **Allen keys/Hex keys** (for motor shaft set screws)
- ✅ **3D Printer** (or access to printing service)
- ✅ **Android Phone** (for control app)

### Recommended Tools
- **Multimeter** (for voltage/current testing)
- **Hot glue gun** (for securing components)
- **Needle-nose pliers**
- **Wire cutters/diagonal cutters**
- **Helping hands** (soldering assistant)
- **LiPo battery charger** (balance charger for 3S)
- **LiPo voltage alarm** (safety device)

### Additional Wiring/Materials (if needed)
- **22-24 AWG wire** (red/black for power)
- **Zip ties** (cable management)
- **Velcro straps** (battery mounting)
- **Double-sided tape**
- **Electrical tape**

---

## 3D Printing Requirements

### Files to Print
All STL files located in: `/Self-Balancing Two-Wheeled Robot - 4593553/files/`

1. ✅ `BackBody.STL` - Rear body panel
2. ✅ `FrontBody1.STL` - Front body panel  
3. ✅ `EyeHole.STL` - Decorative eye hole
4. ✅ `MotorCase.STL` - Motor mounting (print **2x**)
5. ✅ `MotorDriverCover.STL` - Driver protection cover
6. ✅ `MotorDriverCover2.STL` - Alternative driver cover
7. ✅ `WheelMount.STL` - Wheel shaft adapter (print **2x**)
8. ⚠️ `WheelDontPrint_OnlyForReference.STL` - Reference only (buy or design your own)

### Print Settings
- **Material:** PLA (recommended) or PETG
- **Layer Height:** 0.2mm
- **Infill:** 20-30%
- **Supports:** Yes (for some parts)
- **Bed Adhesion:** Brim or raft recommended
- **Estimated Material:** ~200-300g of filament
- **Print Time:** ~12-20 hours total

---

## Software Requirements

### 1. Arduino IDE with Teensyduino
- Download Arduino IDE: https://www.arduino.cc/en/software
- Download Teensyduino: https://www.pjrc.com/teensy/td_download.html
- **Install Teensyduino** (adds Teensy support to Arduino IDE)

### 2. Required Arduino Libraries
Install via Library Manager or manual download:

1. **I2Cdev** by Jeff Rowberg
   - GitHub: https://github.com/jrowberg/i2cdevlib
   
2. **MPU6050** (with DMP support)
   - From I2Cdevlib repository
   - Must support Digital Motion Processor (DMP)
   
3. **Wire Library** (built-in with Arduino IDE)

### 3. Optional: Edit MPU6050 Library
- The README mentions optional editing of MPU6050 library file
- This may improve performance but is not required
- See original documentation for details

### 4. Android Control App
- **RemoteXY** or similar Bluetooth control app
- Search "Bluetooth Arduino Controller" in Google Play Store
- Free apps available
- You'll create a custom control panel in the app

---

## Total Cost Breakdown

| Component | Price Range |
|-----------|-------------|
| Teensy 4.0 | $20-25 |
| MPU6050 | $6-10 |
| HC-05 Bluetooth | $8-12 |
| 2x NEMA 17 Steppers | $20-30 |
| 2x DRV8825 Drivers | $10-15 |
| 3S LiPo Battery | $20-35 |
| UBEC Step-Down | $8-15 |
| XT60 Connectors | $8-12 |
| Breadboard | $6-10 |
| Jumper Wires | $8-15 |
| M3 Hardware | $12-18 |
| Wheels | $10-20 |
| Capacitors | $5 |
| 3D Printing Filament | $5-15 |
| Tools (if needed) | $20-50 |
| LiPo Charger | $20-30 |
| **TOTAL** | **$180-280** |

**Minimum Build (if you have tools/printer):** ~$150-180  
**Complete Build (buy everything):** ~$250-300

---

## Assembly Steps (Overview)

From the README, the process is:

1. ✅ **Buy items** from this bill of materials
2. ✅ **3D Print** all STL files from files folder
3. ✅ **Wire it all up** (see wiring section below)
4. ✅ **Set current limit** on motor drivers (critical!)
5. ✅ **Download Arduino code**, libraries, optional MPU6050 library edit
6. ✅ **Download Android app**, create control panel
7. ✅ **Upload code** to Teensy 4.0
8. ✅ **Assemble** everything into chassis
9. ✅ **Test** and tune!

---

## Key Wiring Notes

### Power Distribution
```
3S LiPo (11.1V)
    |
    ├─► XT60 Connector
    |
    ├─► Power Switch (recommended)
    |
    ├─► DRV8825 Drivers (VMOT pins) - Motor power
    |
    └─► UBEC Step-Down Converter
          |
          ├─► 5V Output → Teensy 4.0 VIN
          └─► 5V Output → MPU6050, HC-05
```

### Critical Connections
- **100µF capacitors** across motor driver power pins (VMOT and GND)
- **Common ground** - All grounds must be connected together
- **HC-05 voltage** - Check if it needs 5V or 3.3V (some modules have regulators)
- **Teensy 4.0 is 3.3V logic** - Most modules are compatible but verify

---

## Setting Motor Driver Current Limit

⚠️ **CRITICAL STEP - Must be done before powering motors!**

### Why It's Important
- Prevents motor overheating
- Prevents driver damage
- Ensures optimal performance

### How to Set Current Limit (DRV8825)
1. Measure stepper motor rated current (usually 1.5A for these motors)
2. Calculate VREF: `VREF = Current × 2` (for DRV8825)
3. Connect driver to power (no motor yet)
4. Use multimeter to measure voltage on driver's potentiometer
5. Adjust tiny potentiometer with screwdriver until VREF is correct
6. Example: For 1.5A motor → VREF = 1.5 × 2 = 3.0V

**For your motors:** Likely VREF should be around 2.4-3.0V

---

## Important Compatibility Notes

### This Design REQUIRES:
- ✅ **Teensy 4.0** (NOT Arduino Mega/Nano/Uno)
- ✅ **Stepper motors** (NOT DC motors)
- ✅ **DRV8825 drivers** (A4988 may work but not recommended)
- ✅ **3S LiPo battery** (11.1V, not 7.4V 2S)
- ✅ **Android phone** (for Bluetooth control)

### Won't Work With:
- ❌ Arduino boards (code is Teensy-specific)
- ❌ DC motors (completely different control method)
- ❌ L298N motor driver (for DC motors, not steppers)
- ❌ 2S LiPo (7.4V too low for steppers)

---

## Safety Warnings

### ⚠️ LiPo Battery Safety
- **Never discharge below 3.0V per cell** (9.0V total)
- Always use a balance charger
- Use LiPo voltage alarm
- Store at 3.8V per cell (storage charge)
- Use fireproof LiPo bag for charging/storage
- Never leave charging unattended
- Check for puffing/damage before each use

### ⚠️ Stepper Driver Safety
- **Always attach heatsinks** before powering
- **Set current limit correctly** before connecting motors
- Never disconnect/connect motors while powered
- Ensure good ventilation
- Monitor temperature during operation

### ⚠️ Soldering Safety
- Work in ventilated area
- Use fume extractor if possible
- Don't breathe solder fumes
- Keep iron tip clean
- Unplug when not in use

### ⚠️ General Safety
- Double-check all wiring before powering on
- Test with robot elevated first (wheels off ground)
- Keep fingers away from wheels when powered
- Work in open area away from stairs/edges
- Have power switch easily accessible
- Start with low speeds, tune gradually

---

## Where to Buy - Quick Links

### All-in-One Options
- **Amazon** - Most items available, fast Prime shipping
- **Banggood** - UBEC and some electronics
- **PJRC** - Official Teensy 4.0 store

### Alternatives
- **AliExpress** - Cheaper prices, slower shipping (2-4 weeks)
- **eBay** - Individual components
- **Pololu** - Quality stepper motors and drivers
- **Adafruit** - Premium components
- **SparkFun** - Educational resources included

### 3D Printing
- **Own printer** - Most economical
- **Local makerspace** - Often free or low cost
- **3DHubs/Craftcloud** - Online printing service
- **Shapeways** - Professional printing (expensive)

---

## Pre-Purchase Checklist

Before ordering, ensure you have:

- [ ] Access to 3D printer or printing service
- [ ] Soldering equipment (iron, solder, etc.)
- [ ] Android phone for control app
- [ ] LiPo battery charger (balance charger)
- [ ] Basic hand tools
- [ ] Multimeter (for setting current limit)
- [ ] Computer with Arduino IDE
- [ ] Workspace for assembly
- [ ] Understanding of LiPo safety

---

## Next Steps After Receiving Parts

1. **3D print all chassis parts** (while waiting for deliveries)
2. **Install Arduino IDE + Teensyduino**
3. **Install required libraries** (I2Cdev, MPU6050)
4. **Test Teensy 4.0** (upload blink sketch)
5. **Set stepper driver current limits** (very important!)
6. **Test motors individually** (simple step test)
7. **Calibrate MPU6050** (IMU_Zero sketch)
8. **Wire everything on breadboard** (test before assembly)
9. **Upload balancing code**
10. **Setup Android app** control panel
11. **Assemble into chassis**
12. **Test and tune PID values**

---

## Support Resources

- **Original Thingiverse Page:** https://www.thingiverse.com/thing:4593553
- **YouTube Video Tutorial:** https://youtu.be/2k_8KneGMak
- **Teensy Forum:** https://forum.pjrc.com/
- **I2Cdevlib GitHub:** https://github.com/jrowberg/i2cdevlib
- **PJRC Teensy Documentation:** https://www.pjrc.com/teensy/
- **DRV8825 Datasheet:** Search "DRV8825 current limit setting"

---

## Troubleshooting Common Issues

### Teensy won't upload
- Install Teensyduino addon
- Press physical button on Teensy
- Check USB cable (must be data cable, not charge-only)

### Motors won't move
- Check current limit setting on drivers
- Verify power connections (VMOT, GND)
- Add 100µF capacitors
- Check step/direction pin connections

### MPU6050 not detected
- Check I2C wiring (SDA, SCL)
- Check 3.3V or 5V power
- Try I2C scanner sketch
- Verify AD0 pin (sets I2C address)

### Bluetooth won't connect
- Check HC-05 power (usually 5V)
- Pair phone with module first
- Check TX/RX connections (swap if needed)
- Verify baud rate matches code

### Robot falls over immediately
- MPU6050 not mounted level
- PID values need tuning
- Current limit too low (weak motors)
- Battery voltage too low

---

**Good luck with your build! 🤖**

*This is an advanced robotics project requiring experience with:*
- *Soldering and electronics*
- *3D printing*
- *Arduino/Teensy programming*
- *LiPo battery safety*
- *PID tuning*

*Take your time, double-check all connections, and don't hesitate to ask for help in the community!*
