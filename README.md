# Robotic Arm Controller (ESP32)

6-DOF robotic arm firmware for ESP32. Servo positions are fetched from a Firebase Realtime Database and applied with staggered movement to avoid current spikes.

## Hardware
- ESP32
- M1 Base, M2 Shoulder, M3 Elbow — MG996R servos
- M4 Wrist pitch — MG90S servo
- M5 Wrist roll, M6 Gripper — SG90 servos

## Features
- WiFiManager captive portal for WiFi setup (no hardcoded credentials)
- Polls Firebase every 100ms for target servo angles
- Staggers servo moves 50ms apart to reduce power draw spikes
- Per-servo angle clamping for safety

## Setup
1. Flash to ESP32 via Arduino IDE (install `WiFiManager`, `ESP32Servo`, `ArduinoJson` libraries)
2. On first boot, connect to the `RoboticArm-Setup` WiFi hotspot and enter your WiFi credentials
3. Update `FB_URL` with your own Firebase Realtime Database URL
