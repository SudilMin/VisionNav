# VisionNav: Comprehensive Product & Architecture Overview
*This document provides full product and technical context for LLMs to understand the scope, features, and architecture of VisionNav.*

## 1. The Core Problem & Current Industry Flaws
Current assistive technologies for the visually impaired suffer from severe limitations that VisionNav was built to solve:
- **Spatial Amnesia:** Standard wearables are strictly reactive. They process an isolated camera frame, output a warning, and immediately forget it. Because they lack a persistent map (SLAM), the system loses all environmental awareness the second the user turns their head.
- **Cognitive Load Saturation:** Systems like Biped.ai (NOA) use continuous 3D spatial audio, which overwhelms the user's auditory cortex and actively masks crucial real-world sounds (like approaching traffic). Devices like Strap Tech (Ara) use complex haptic "languages" that require intense focus to decode.
- **Form Factor Constraints:** Robotic canes (Glidance) occupy the dominant hand, restricting biomechanics. Smart glasses (OrCam) cause cervical spine fatigue and suffer from severe thermal throttling.

## 2. The VisionNav Solution & Distributed Architecture
VisionNav acts as a context-aware robotic state machine. To solve battery, thermal, and weight constraints, it utilizes a **Distributed Split-Node Architecture** connected via high-speed Wi-Fi (`ROS 2 FastDDS`):
* **Sensor Node (On-Body Chest Rig):** A lightweight, unencumbering rig containing a Raspberry Pi, 2D LiDAR, Smartphone Camera, and an ESP32. It performs zero heavy computing; it simply captures, compresses, and streams sensor data.
* **Compute Node (Edge/Laptop):** A powerful host device (carried in a backpack or running nearby) that handles all heavy lifting: YOLOv5 object detection, Semantic SLAM mapping, A* Navigation, and Local LLM inference.
* **Zero-Spam Philosophy:** VisionNav employs algorithmic filtering. It remains completely silent to preserve the user's peace of mind, only interrupting if a critical hazard breaches a proximity threshold, or if the user actively asks for help.

## 3. Core Operational Modes
VisionNav adapts to its environment to provide the safest experience.

### Indoor Mode (Semantic SLAM & Pathfinding)
* **Mapping:** The 1D LiDAR distance data is fused with 2D Camera bounding boxes to project physical objects into 3D space. These objects are dropped as persistent markers into a live ROS 2 SLAM map.
* **Autonomous Navigation:** The user can command the system to take them to a previously seen object. A custom A* pathfinding algorithm (with Chaikin smoothing) calculates the safest route avoiding walls and obstacles, updating in real-time.

### Outdoor Mode (Collision Corridors & Hazards)
* **Hazard Avoidance:** Because SLAM mapping an entire city street is impossible, Outdoor Mode disables persistent mapping. Instead, it projects a virtual "Collision Corridor" directly in front of the user.
* **Moving Threats:** It actively monitors the velocity vectors of dynamic objects (people, bicycles, cars). If an obstacle rapidly breaches the safety threshold in the center of the frame, it triggers an immediate emergency alert.

## 4. Navigation & Feedback Mechanisms
VisionNav offers dual-modality feedback to ensure the user is safely guided to their destination:
* **Voice Navigation:** A voice assistant utilizing Piper TTS provides clean, turn-by-turn auditory directions (e.g., "Bear slightly right. 3 feet remaining.", "You have arrived at the chair.").
* **Vibration Navigation (The "Last Inch" Grasping System):** While voice navigation gets the user to the correct side of the room, grasping the actual object is notoriously difficult for the blind. VisionNav pairs with a Bluetooth-enabled haptic wristband. As the user reaches out, the system calculates the Euclidean distance between their hand and the target object, modulating the vibration frequency to peak exactly when their hand touches the object.

## 5. Hardware Interface: ESP32 Smart Buttons
To completely eliminate the need for a screen or keyboard, the user controls the entire VisionNav system seamlessly via physical ESP32 push buttons mounted on the chest rig:
* **Button 1 (AI Voice Navigation):** Triggers the `find_object.py` node. The user holds this button to speak spatial commands like *"Find the chair"* or *"Take me to the door"*.
* **Button 2 (Moondream Scene Describer):** Triggers the `scene_describer.py` node. When pressed, the system instantly grabs the latest camera frame and feeds it into the Moondream2 Vision Language Model (running via Ollama). The VLM analyzes the image and speaks a highly detailed, natural language description of everything in front of the user, helping them understand complex environments (e.g., reading signs, recognizing room layouts).
