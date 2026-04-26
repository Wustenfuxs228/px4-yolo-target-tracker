# PX4 Autonomous Target Tracker (SITL)

![Python](https://img.shields.io/badge/Python-3.10-blue.svg)
![PX4](https://img.shields.io/badge/PX4-Autopilot-0B28A1.svg)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange.svg)
![YOLOv8](https://img.shields.io/badge/Ultralytics-YOLOv8-yellow.svg)
![MAVSDK](https://img.shields.io/badge/MAVSDK-Python-lightgrey.svg)

![Target Tracking Demo](<img width="800" height="450" alt="Screencastfrom2026-04-2613-56-20-ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/ad0dd804-ce34-430d-8a6a-effcb4bbc40e" />)
> 🎥 **[Watch the full 45-second demonstration video here (All Flight Modes)](https://youtu.be/20aRas9gDrE)**

## Overview
This project implements a Software-In-The-Loop (SITL) simulation for an autonomous UAV capable of visually detecting and dynamically tracking moving targets (vehicles, trucks, boats). 
The system integrates the PX4 Autopilot, the Gazebo physics simulator, and a YOLOv8 object detection model. Flight control and camera gimbal stabilization are managed via a custom Finite State Machine (FSM) utilizing MAVSDK.

## Key Features
* **Real-time Computer Vision:** Object detection and classification using YOLOv8 via a GStreamer UDP feed from the Gazebo virtual camera.
* **Finite State Machine (FSM):** Seamless switching between operational modes:
  * `MANUAL`: Full pilot control over flight and gimbal.
  * `ASSISTED`: AI controls Gimbal (Pitch) and Drone Heading (Yaw) for cinematic tracking, while the pilot controls 3D movement (strafing/altitude).
  * `FULL AUTO`: AI fully manages pursuit, maintaining a predefined safe distance using Proportional (P) control loops.
* **Tactical HUD:** Custom OpenCV-based On-Screen Display featuring real-time MAVLink telemetry (Altitude, Velocity, Yaw, Gimbal Pitch) and target lock status.
* **Robust Failsafes:** Immediate emergency manual-override ("Kill Switch") and handling of pre-flight EKF2 sensor calibration delays.

## System Architecture
1. **Gazebo Simulator:** Renders the physical environment and streams the camera feed via GStreamer (`udpsrc port=5600`).
2. **Vision Node (YOLOv8):** Processes the video stream, extracts bounding boxes, and calculates the positional error (X, Y) relative to the frame center.
3. **Control Loop:** Calculates appropriate velocity setpoints (`VelocityBodyYawspeed`) and Gimbal angles based on the visual error using tuned P-Gains.
4. **MAVSDK / PX4:** Receives commands via MAVLink (`udp://:14540`) and executes the physical movement in the simulation.
   
## Running the Simulation

*(This project is designed to run inside the standard PX4 SITL Docker container.)*

**1. Clean up and Start the Gazebo World:**
```bash
# Kill any zombie processes and reset the environment
pkill -9 px4; pkill -9 gzserver; pkill -9 gzclient; pkill -9 python3

# Launch the Typhoon H480 SITL model
export DISPLAY=:0
cd /root/projects/PX4-Autopilot
make px4_sitl gazebo-classic_typhoon_h480

2. Launch the Tracking Node:
In a new terminal attached to the same container:
Bash

export DISPLAY=:0
NO_AT_BRIDGE=1 python3 /root/projects/threaded_fly.py

Flight Controls

The system utilizes a "Cruise Control" style input method.

    T - Toggle Flight Modes (Manual -> Assisted -> Full Auto)

    SPACE - Emergency Brake (Halts all movement and forces Manual Mode)

    W / S - Forward / Backward (Max 5.0 m/s)

    A / D - Strafe Left / Right

    Q / E - Rotate Yaw (Manual mode only)

    R / F - Altitude Up / Down

    G - Hold Altitude (Zero vertical velocity)

    I / K / J / L - Manual Gimbal Control (Pitch/Yaw)

🗺️ Future Work / Roadmap

    ROS 2 Migration: Port the Python MAVSDK script to C++ using ROS 2 nodes (MicroXRCE-DDS).

    PID Controllers: Upgrade the current P-Controllers to full PID loops to handle simulated wind resistance and reduce tracking oscillation.

    Edge AI Optimization: Export the yolov8s.pt model to TensorRT format to simulate deployment constraints on edge devices like the Jetson Nano.
