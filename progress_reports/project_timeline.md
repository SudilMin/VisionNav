# VisionNav - 10-Week Master Project Timeline

This timeline outlines the end-to-end development of the VisionNav wearable system, transitioning from the foundational ROS 2 simulation into a fully realized physical hardware prototype with advanced micro-navigation and cognitive AI features.

---

## Phase 1: Foundation and Simulation (Completed)

### Week 1: Core Perception and Spatial Mapping (Completed)
* Built the core ROS 2 Jazzy and Gazebo simulation environment.
* Implemented the primary AI object detection pipeline (YOLOv5) and Semantic SLAM.
* **Outcome:** The system successfully detects objects and drops 3D semantic markers into the RViz map.

### Week 2: Advanced Navigation, Dynamic Hazards, & Fast AI (Completed)
* Upgraded to continuous A* turn-by-turn voice navigation (Piper TTS).
* Implemented emergency braking for rapidly approaching dynamic hazards.
* Integrated the lightning-fast 4-bit Ollama Moondream engine for native scene description and text recognition.
* **Outcome:** A unified, two-tier perception stack capable of macro-navigation and open-ended scene understanding.

---

## Phase 2: Hardware Transition & Micro-Navigation

### Week 3: Physical Hardware Integration (The Chest Rig)
* **Hardware:** Assemble the physical chest rig using an ESP32 micro-controller, wide-angle RGB Camera, and dual chest vibration motors (Left/Right).
* **Software:** Bridge the ESP32 camera feed to the existing ROS 2 YOLO pipeline via Wi-Fi/Serial. Ensure the hardware battery system is stable.
* **Goal:** Successfully run the current YOLO and Moondream models on live video from the physical chest camera.

### Week 4: The "Last Inch" Grasping System (Micro-Navigation)
* **Feature:** Implement hand-tracking in YOLO alongside object tracking.
* **Software:** Calculate the X/Y offset between the user's hand and the target object (e.g., a coffee cup).
* **Hardware:** Program the ESP32 to pulse the left/right chest haptic motors to steer the user's hand.
* **Goal:** The user can flawlessly reach out and grab a small object guided purely by chest vibrations and short voice cues ("Move right... Stop. Grab.").

---

## Phase 3: Advanced Perception & Memory

### Week 5: Surface Hazards & Dynamic Danger Translation
* **Feature:** Implement "Surface Hazard Detection".
* **Software:** Train YOLO on specialized classes: `puddle`, `wet_floor_sign`, `stair_edge`, `platform_edge`. Override A* pathfinding to avoid these zones.
* **Hardware:** Link the "Velocity Tracking" (from Week 2) directly to the ESP32 to trigger maximum-power haptic feedback for moving hazards.
* **Goal:** The physical rig actively protects the user from both high-speed collisions and ground-level drop-offs.

### Week 6: Social Memory & Multi-Modal "Hold & Ask"
* **Feature 1:** Integrate a lightweight facial recognition model (FaceNet/OpenCV).
* **Feature 2:** Cloud LLM Integration (Gemini/ChatGPT API) for complex reasoning ("When does this milk expire?").
* **Goal:** The system can reliably recognize friends entering the room and can answer high-fidelity questions about objects held up to the camera.

### Week 7: Temporal Object Memory (The "Lost Keys" Tracker)
* **Feature:** Build a lightweight SQLite database to log the GPS/Map coordinates and timestamps of small movable items (wallet, keys, phone).
* **Software:** When the user asks "Where is my wallet?", the system queries the database, reads the last known timestamp, and calculates an A* route to that exact coordinate.
* **Goal:** A perfect passive tracking system for misplaced personal items.

---

## Phase 4: Expansion & Real-World Deployment

### Week 8: Smart Cane Integration & Multi-Floor Mapping
* **Hardware (Cane):** Attach a $2 Bluetooth beacon to the user's white cane. Program the ESP32 to track its RSSI signal strength to help the user find it if dropped.
* **Hardware (IMU/Barometer):** Wire a BME280 (Barometer) and IMU to the ESP32.
* **Software:** Map atmospheric pressure drops to elevation changes to create separate "Map Layers" for different floors (Floor 1, Floor 2) and warn of approaching stairs.
* **Goal:** True multi-floor awareness and physical tool (cane) recovery.

### Week 9: Full System Field Testing
* **Goal:** Take the physical rig out of the lab.
* **Tasks:** Test in varied lighting conditions, noisy environments (to test the dual terminal/voice interface), and crowded areas. Calibrate haptic motor intensity and A* pathfinding tolerance based on real blind user feedback (or blindfolded testing).

### Week 10: Final Polish, Optimization & Presentation
* **Tasks:** Clean up code, optimize ESP32 power consumption to maximize battery life, and finalize the project presentation/demo video.
* **Goal:** Deliver a complete, revolutionary product that eclipses existing market solutions (NOA, Ara, Glide) by successfully solving both macro and micro navigation.
