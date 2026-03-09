# LYCO: TOMI Autonomous Greenhouse Robot

### Project Status: Active Development (Spring 2026 Senior Design)
*Note: This repository serves as a portfolio showcase of the core navigation and vision logic. As this is an ongoing Senior Design project at the UA Controlled Environment Agriculture Center, some modules are currently in active debugging and testing phases.*

## Overview
The TOMI Autonomous Greenhouse Robot is a mobile robotic platform designed to navigate complex, controlled agricultural environments. The system relies on a dual-processor architecture to divide high-level computer vision processing from low-level, real-time physical actuation, allowing for autonomous line-following and obstacle detection.

## Hardware Architecture
This project utilizes a heterogeneous hardware stack to handle both intensive data processing and precise motor control:
* **High-Level Processing:** Raspberry Pi
* **Vision & Depth:** OAK-D Lite Camera
* **Low-Level Actuation:** Arduino Nano ESP32
* **Motor Control:** Sabertooth Motor Drivers

## Software & Firmware Stack
* **Python / Computer Vision:** Utilizes OpenCV and the DepthAI API on the Raspberry Pi to process real-time spatial data, identify pathways, and detect obstacles.
* **Embedded C++:** Bare-metal firmware on the Arduino Nano ESP32 handles the translation of navigation commands into precise PWM signals.
* **Serial Communication:** Custom serial protocols ensure reliable, low-latency command transfer between the Raspberry Pi and the Arduino.

## System Workflow
1. **Perception:** The OAK-D Lite captures RGB and depth data.
2. **Processing:** The Raspberry Pi runs OpenCV scripts to filter the environment and calculate trajectory vectors.
3. **Command:** Vectors are translated into motor speeds and sent via serial connection to the Arduino.
4. **Actuation:** The Arduino triggers the Sabertooth drivers to adjust the DC motors accordingly.

## How to Run (Core Modules)

**1. Firmware Deployment (Run First)**
* Connect the **Arduino Nano ESP32** to your machine via USB-C.
* Open and flash `Motor.ino` to the board using the Arduino IDE.
* *Important:* This firmware must be running on the ESP32 prior to launching the Python scripts to successfully establish the serial listening protocol.

**2. Vision Execution**
* Establish a remote Wi-Fi connection to the **Raspberry Pi** using Pi Connect.
* From the Pi's terminal, execute the main vision logic:
  ```bash
  python3 Obstacle_Logic.py
