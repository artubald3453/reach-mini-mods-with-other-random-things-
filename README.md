
<p align="center">
  <img src="https://img.shields.io/badge/Robotics-Reachy-blue" alt="Reachy Robotics">
  <img src="https://img.shields.io/badge/Platform-Multi--Platform-green" alt="Multi-Platform">
  <img src="https://img.shields.io/badge/License-Open%20Source-orange" alt="Open Source">
</p>

A comprehensive collection of robotics projects, libraries, and tools for building and controlling Reachy and Reachy Mini humanoid robots. This repository contains everything from advanced robotic hands to computer vision systems, kinematics solvers, and motor controllers.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Projects](#projects)
  - [AmazingHand](#1-amazinghand)
  - [Pollen Vision](#2-pollen-vision)
  - [Reachy Mini Dances Library](#3-reachy-mini-dances-library)
  - [Reachy Mini Rust Kinematics](#4-reachy-mini-rust-kinematics)
  - [Reachy Mini Toolbox](#5-reachy-mini-toolbox)
  - [Self-Balancing Robot](#6-self-balancing-robot)
  - [Reachy Mini Motor Controller](#7-reachy-mini-motor-controller)
- [Getting Started](#getting-started)
- [System Requirements](#system-requirements)
- [Contributing](#contributing)
- [Resources](#resources)
- [License](#license)

---

## 🎯 Overview

This repository houses a collection of interconnected projects that enable the development, control, and enhancement of Reachy humanoid robots. Whether you're building a complete Reachy robot from scratch, adding capabilities like advanced manipulation or computer vision, or experimenting with individual components, you'll find the tools and documentation you need here.

### What's Inside?

- **Hardware Designs**: 3D-printable robotic hand with full CAD files
- **Vision Systems**: Zero-shot computer vision models for robotics applications
- **Control Systems**: Kinematics solvers, motor controllers, and movement libraries
- **Educational Projects**: Self-balancing robot with comprehensive documentation
- **Utilities**: Toolboxes and helper libraries for robot development

---

## 🚀 Projects

### 1. AmazingHand

<img src="https://img.shields.io/badge/Type-Hardware-red" alt="Hardware"> <img src="https://img.shields.io/badge/DOF-8-blue" alt="8 DOF"> <img src="https://img.shields.io/badge/Cost-<$200-green" alt="Low Cost">

**An open-source, 8 DOF humanoid robotic hand designed for the Reachy2 robot.**

#### Key Features
- 8 degrees of freedom with 4 articulated fingers
- 2 phalanxes per finger with parallel mechanism control
- All actuators integrated inside the hand (no cables!)
- 3D printable design with flexible shells
- Weighs only 400g
- Low-cost build (<200€)
- Compatible with Arduino and Python control

#### What You Get
- Complete CAD files (STL and STEP formats)
- Onshape design document with predefined positions
- Bill of Materials (BOM) with sourcing links
- Detailed assembly guide with 3D printing tips
- Arduino and Python example code
- Calibration and control software

#### Control Options
- **Python**: Using Waveshare serial bus driver
- **Arduino**: Using Feetech TTL Linker

#### Perfect For
- Adding expressive hands to humanoid robots
- Research in manipulation and grasping
- Educational robotics projects
- Open-source robot builders

📁 **Directory**: `AmazingHand-main/`  
📖 **Documentation**: [Assembly Guide](AmazingHand-main/docs/AmazingHand_Assembly.pdf) | [BOM](https://docs.google.com/spreadsheets/d/1QH2ePseqXjAhkWdS9oBYAcHPrxaxkSRCgM_kOK0m52E/)  
🔗 **CAD Files**: [Onshape](https://cad.onshape.com/documents/430ff184cf3dd9557aaff2be/)

---

### 2. Pollen Vision

<img src="https://img.shields.io/badge/Type-Software-blue" alt="Software"> <img src="https://img.shields.io/badge/Models-Zero--Shot-purple" alt="Zero-Shot"> <img src="https://img.shields.io/badge/Python-3.10+-green" alt="Python">

**A unified interface to zero-shot computer vision models curated for robotics use cases.**

#### Key Features
- **Zero-shot object detection** with Owl-Vit
- **Zero-shot object segmentation** with Mobile-SAM
- **Monocular depth estimation** with Depth Anything
- **Object recognition** with Recognize-Anything
- Simple, unified API for all models
- Real-time performance optimized for robotics
- Luxonis camera wrappers included

#### Quick Start Example
```python
import cv2
from pollen_vision.vision_models.object_detection import OwlVitWrapper
from pollen_vision.vision_models.object_segmentation import MobileSamWrapper
from pollen_vision.utils import Annotator, get_bboxes

owl = OwlVitWrapper()
sam = MobileSamWrapper()
annotator = Annotator()

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    predictions = owl.infer(frame, ["paper cups"])  # Zero-shot detection!
    bboxes = get_bboxes(predictions)
    masks = sam.infer(frame, bboxes=bboxes)
    annotated_frame = annotator.annotate(frame, predictions, masks=masks)
    cv2.imshow("frame", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
```

#### Installation
```bash
pip install "pollen-vision[vision] @ git+https://github.com/pollen-robotics/pollen-vision.git@main"
```

#### Perfect For
- Robot perception systems
- Object detection and tracking
- Visual servoing applications
- 3D scene understanding

📁 **Directory**: `pollen-vision-develop/`  
🎮 **Try Online**: [HuggingFace Demo](https://huggingface.co/spaces/pollen-robotics/pollen-vision-demo)  
📓 **Examples**: Jupyter notebooks included in `examples/`

---

### 3. Reachy Mini Dances Library

<img src="https://img.shields.io/badge/Type-Software-blue" alt="Software"> <img src="https://img.shields.io/badge/Format-JSON-yellow" alt="JSON"> <img src="https://img.shields.io/badge/Python-3.x-green" alt="Python">

**A library of dance moves and choreographies for the Reachy Mini robot.**

#### Features
- Pre-programmed dance choreographies
- Interactive demo mode
- JSON-based choreography format
- Easy to create custom dances
- Keyboard control support

#### Installation
```bash
cd reachy_mini_dances_library-main
pip install -e .
```

#### Usage
```bash
# Interactive mode (cycles through all moves)
python examples/dance_demo.py

# Play a specific choreography
python examples/dance_demo.py --choreography ./examples/choreographies/another_one_bites_the_dust.json --no-keyboard
```

#### Perfect For
- Demonstrations and exhibitions
- Entertainment applications
- Testing robot coordination
- Creating engaging robot behaviors

📁 **Directory**: `reachy_mini_dances_library-main/`  
💃 **Included Dances**: "Another One Bites the Dust" and more

---

### 4. Reachy Mini Rust Kinematics

<img src="https://img.shields.io/badge/Type-Software-blue" alt="Software"> <img src="https://img.shields.io/badge/Language-Rust-orange" alt="Rust"> <img src="https://img.shields.io/badge/Performance-High-red" alt="High Performance">

**High-performance kinematics solver for Reachy Mini written in Rust with Python bindings.**

#### Features
- **Analytical Inverse Kinematics** - Fast and accurate
- **Numerical Forward Kinematics** - Robust solutions
- Python bindings via Maturin
- Optimized for real-time control
- Translation of C++ kinematics library

#### Installation
```bash
cd reachy_mini_rust_kinematics-main
pip install maturin
pip install -e . --verbose
```

#### Perfect For
- Real-time motion planning
- Trajectory generation
- Joint angle calculations
- Performance-critical applications

📁 **Directory**: `reachy_mini_rust_kinematics-main/`  
🔗 **Original**: Based on [reachy_mini_cpp_kinematics](https://github.com/pollen-robotics/reachy_mini_cpp_kinematics)

---

### 5. Reachy Mini Toolbox

<img src="https://img.shields.io/badge/Type-Software-blue" alt="Software"> <img src="https://img.shields.io/badge/Category-Utilities-lightblue" alt="Utilities">

**Collection of utility tools and helpers for Reachy Mini development.**

#### Features
- Vision tools for camera integration
- Common utilities for robot control
- Helper functions for development
- Integration with `reachy_mini` package

#### Installation
```bash
cd reachy_mini_toolbox-main
pip install -e .

# Install with vision support
pip install -e .[vision]
```

#### Requirements
- Requires `reachy_mini` package for examples

#### Perfect For
- Simplifying common robot tasks
- Quick prototyping
- Development workflows

📁 **Directory**: `reachy_mini_toolbox-main/`  
🔧 **Tools**: Multiple helper scripts in `tools/` directory

---

### 6. Self-Balancing Robot

<img src="https://img.shields.io/badge/Type-Hardware+Software-red" alt="Hardware+Software"> <img src="https://img.shields.io/badge/Platform-Arduino-teal" alt="Arduino"> <img src="https://img.shields.io/badge/Education-Friendly-green" alt="Educational">

**A complete Arduino-based self-balancing two-wheel robot with USB serial control.**

#### Key Features
- Automatic balance using PID control
- MPU6050 IMU for tilt sensing
- Serial command interface (forward/backward/left/right)
- Complementary filter for sensor fusion
- Real-time debugging via Serial Monitor
- Comprehensive documentation

#### Components
- Arduino Nano or Uno
- MPU6050 Gyroscope/Accelerometer
- L298N Motor Driver
- 2x DC Geared Motors
- 7.4V-12V Battery
- **Total Cost**: $50-120

#### Control Example (Python)
```python
import serial
import time

robot = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)

robot.write(b'f')  # Move forward
time.sleep(2)
robot.write(b's')  # Stop
robot.close()
```

#### Serial Commands
- `f` - Move forward
- `b` - Move backward
- `l` - Turn left
- `r` - Turn right
- `s` - Stop

#### Perfect For
- Learning PID control
- Sensor fusion education
- Embedded systems projects
- Robotics fundamentals

📁 **Directory**: `reachy-mini_balencing_robot-main/`  
📖 **Guides**: [Parts List](reachy-mini_balencing_robot-main/PARTS_LIST.md) | [Wiring Diagram](reachy-mini_balencing_robot-main/WIRING_DIAGRAM.md) | [Setup & Tuning](reachy-mini_balencing_robot-main/SETUP_AND_TUNING_GUIDE.md)

---

### 7. Reachy Mini Motor Controller

<img src="https://img.shields.io/badge/Type-Software-blue" alt="Software"> <img src="https://img.shields.io/badge/Language-Rust-orange" alt="Rust"> <img src="https://img.shields.io/badge/Servos-Dynamixel-red" alt="Dynamixel">

**Motor controller for Reachy Mini's Stewart platform and servo system with Python bindings.**

#### Features
- Controls Stewart platform (6x XL330 servos)
- Base rotation control (XC330 servo)
- Antenna control (XL330 servos)
- Python bindings for easy integration
- Used by the official Reachy Mini project

#### Installation
```bash
cd reachy-mini-motor-controller-main
pip install maturin
pip install -e . --verbose
```

#### Perfect For
- Reachy Mini robot control
- Stewart platform applications
- Dynamixel servo management

📁 **Directory**: `reachy-mini-motor-controller-main/`  
🔗 **Used By**: [Reachy Mini](https://github.com/pollen-robotics/reachy_mini)

---

## 🛠️ Getting Started

### Prerequisites

Depending on which projects you want to use, you'll need:

#### Software Development
- **Python**: 3.10 or higher
- **Git LFS**: For large files (required for pollen-vision)
  ```bash
  # Ubuntu
  sudo apt-get install git-lfs
  
  # macOS
  brew install git-lfs
  ```
- **Rust & Cargo**: For Rust-based projects
  ```bash
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  ```
- **Maturin**: For Python-Rust bindings
  ```bash
  pip install maturin
  ```

#### Hardware Development
- **Arduino IDE**: For Arduino-based projects
- **3D Printer**: For printing hardware components
- **Soldering Equipment**: For electronics assembly

### Quick Start Guide

1. **Clone this repository**
   ```bash
   git clone <your-repository-url>
   cd Reachy_plans
   ```

2. **Choose your project** and navigate to its directory

3. **Follow the project-specific README** for installation and setup

4. **Explore the examples** in each project's `examples/` directory

---

## 💻 System Requirements

### Tested Platforms
- **Ubuntu 22.04 LTS**
- **macOS** (M1 Pro and Intel)
- **Windows 10/11** (for Arduino projects)

### Minimum Hardware
- **RAM**: 4GB (8GB+ recommended for vision projects)
- **Storage**: 10GB free space
- **CPU**: Multi-core processor (ARM or x86)
- **GPU**: Optional but recommended for vision models

---

## 🤝 Contributing

These projects are primarily maintained by [Pollen Robotics](https://www.pollen-robotics.com/) and the open-source robotics community.

### How to Contribute
- Report bugs and issues
- Suggest improvements
- Share your builds and modifications
- Create new choreographies or demos
- Improve documentation

### Community Resources
- [Pollen Robotics Discord](https://discord.com/channels/519098054377340948/)
- [GitHub Issues](../../issues)
- [Pollen Robotics Website](https://www.pollen-robotics.com/)

---

## 📚 Resources

### Official Documentation
- [Reachy Documentation](https://docs.pollen-robotics.com/)
- [Pollen Robotics Blog](https://www.pollen-robotics.com/blog/)
- [HuggingFace Space](https://huggingface.co/pollen-robotics)

### Learning Resources
- **PID Control**: Essential for balancing robot and motor control
- **Computer Vision**: For understanding pollen-vision capabilities
- **Kinematics**: For robot motion planning
- **3D Printing**: For hardware fabrication
- **Arduino**: For embedded control systems

### Videos & Tutorials
- [Pollen Robotics YouTube](https://www.youtube.com/@PollenRobotics)
- Arduino self-balancing robot tutorials
- Hand tracking demo videos in AmazingHand project

---

## 📄 License

Projects in this repository use various open-source licenses:

- **AmazingHand**: Apache 2.0 (software) / CC BY 4.0 (hardware)
- **Pollen Vision**: Apache 2.0
- **Other Projects**: See individual project directories for license information

All mechanical designs are generally licensed under [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).

---

## 🙏 Acknowledgments

Special thanks to:
- **Pollen Robotics Team** - For creating and maintaining these amazing projects
- **Open Source Community** - For contributions and support
- **Reachy Robot Users** - For feedback and improvements

### Key Contributors
- Steve N'Guyen - Rust integration, demos
- Pierre Rouanet - Python libraries
- Augustin Crampette & Matthieu Lapeyre - Mechanical design

---

## 🎯 Project Status

| Project | Status | Maintenance | Documentation |
|---------|--------|-------------|---------------|
| AmazingHand | ✅ Stable | Active | Excellent |
| Pollen Vision | ✅ Stable | Active | Excellent |
| Dances Library | ✅ Stable | Active | Good |
| Rust Kinematics | ✅ Stable | Active | Good |
| Toolbox | ✅ Stable | Active | Good |
| Balancing Robot | ✅ Stable | Maintenance | Excellent |
| Motor Controller | ✅ Stable | Active | Good |

---

<p align="center">
  <b>🤖 Happy Building! 🤖</b>
  <br><br>
</p>

---

**Last Updated**: October 2025  
**Repository Maintained By**: Finn Cullen  
**Creadits to**: Pollen-robotics, https://www.thingiverse.com/thing:4593553 (WorkerCraftsman)

