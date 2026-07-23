# VisionNav: Wearable AI Guide for the Visually Impaired
*(Project CV Description)*

*Below are various ways to frame the project on your CV depending on the space you have available. It is written as a fully completed project encompassing both the software simulation and hardware deployment.*

---

## Option 1: Detailed Resume Entry (Recommended for Software/Robotics Roles)

**VisionNav: Wearable AI Navigation & Perception System** | *Lead Developer* 
*Engineered a wearable, multi-sensory robotics system integrating computer vision and IoT to provide macro-navigation and micro-grasping assistance for the visually impaired.*
* **Multi-Layer AI Perception:** Architected a 100% offline, dual-tier AI stack utilizing **YOLOv5** for real-time 30-FPS spatial mapping and **Moondream2 (Ollama 4-bit)** for deep scene understanding and native text recognition (OCR).
* **Semantic SLAM & Navigation:** Built a ROS 2 autonomous navigation framework leveraging 2D LiDAR, a Pinhole Camera Model for depth estimation, and A* pathfinding. System recalculates routes every 1.5 seconds, delivering continuous turn-by-turn voice guidance via **Piper TTS**.
* **Dynamic Hazard & Surface Tracking:** Programmed velocity-tracking algorithms to detect rapidly approaching hazards (vehicles, pedestrians) and downward drop-offs, instantly overriding pathfinding to trigger an emergency haptic brake on the user’s chest rig.
* **Micro-Navigation & IoT Integration:** Developed a "Last Inch" hand-tracking grasping system using stereophonic chest haptics (ESP32). Integrated a Bluetooth-beacon IoT system for smart cane tracking and an IMU/Barometer for multi-floor elevation mapping.
* **Cognitive Memory Architecture:** Engineered a temporal SQLite database to passively log GPS coordinates of personal items for instant "lost keys" retrieval, and integrated OpenCV facial recognition for real-time social memory.

---

## Option 2: Short Bullet Points (For a General Tech CV)

**VisionNav: Wearable AI Guide**
* Developed a wearable assistive device leveraging **ROS 2**, **YOLOv5**, and offline **Vision Language Models (VLMs)** to provide spatial awareness and voice-guided indoor navigation.
* Optimized a 1.6 Billion parameter AI model to run locally on a 4GB GPU via **Ollama 4-bit quantization**, reducing query response times from 52s to 2s for real-time scene description and OCR.
* Engineered a dual-input command router processing both terminal typing and **Whisper AI** voice transcription to seamlessly query spatial maps and SQLite temporal memory databases.
* Integrated custom hardware (ESP32) featuring dual chest-mounted haptic motors for micro-navigation (grasping assistance) and dynamic hazard braking, alongside IMU/Barometric sensors for multi-floor awareness.

---

## Option 3: "Skills Used" Summary section
If your CV has a specific "Projects" block where you just list the technologies used:

**VisionNav: Wearable AI Guide for the Visually Impaired**
**Technologies:** Python, C++, ROS 2 (Jazzy), Gazebo Harmonic, OpenCV, PyTorch, YOLOv5, Ollama (GGUF Quantization), Whisper AI, Piper TTS, SQLite, ESP32 Microcontrollers, Bluetooth/IoT, IMU/Barometric Sensors.
**Summary:** Built a complete wearable robotics platform that acts as a cognitive co-pilot for the blind. Merged real-time semantic SLAM with offline generative VLMs to provide voice-guided pathfinding, lost-item temporal memory, and haptic-steered object grasping.
