# VisionNav: Wearable Assistive Navigation System

![Final Hardware Prototype](123.png)

## Product Overview
VisionNav is an intelligent, wearable assistive device engineered to act as a digital guide for individuals suffering from severe visual impairment or total blindness. The physical device consists of a comfortable, tactical-style chest harness equipped with environmental sensors, which connects to a discreet wearable processing unit and a wireless earpiece. By constantly scanning the surrounding environment, identifying everyday household objects, and measuring distances, the system can speak directly to the user to guide them safely through complex indoor spaces.

## Primary Purpose
Navigating unfamiliar indoor environments can be highly disorienting and hazardous for visually impaired individuals. While traditional tools like white canes provide immediate tactile feedback regarding the ground directly in front of the user, they cannot tell a person where a specific piece of furniture is located or the best path to walk across a room. 

The primary purpose of VisionNav is to restore a sense of spatial independence. Instead of relying on a human guide or physical trial-and-error, the user can request to find a specific object in a room. The system will then mathematically calculate a safe walking route that actively avoids walls and obstacles, leading the user directly to their destination.

## User Experience and Operational Workflow
The system is designed to be entirely hands-free and operates in a continuous, intuitive loop:

1. System Initialization: The user straps on the lightweight chest rig and inserts their wireless earpiece. The device powers on and immediately begins processing its surroundings.
2. Environmental Scanning: As the user walks, the front-facing camera and distance sensors quietly observe the environment. The system identifies furniture, structural obstacles, and open pathways, gradually building a mental map of the room in its memory.
3. Voice Command Activation: When the user needs to locate something, they press a tactile button located on the chest rig and issue a verbal command, such as "Find a chair."
4. Natural Language Guidance: Once the target object is located, the system does not simply point in a straight line. It calculates the safest walking route to avoid any hazards. It then speaks directly into the user's ear using an intuitive clock-face directional system. For example, it will clearly instruct the user: "Turn right to your 2 o'clock direction, then walk 15 feet."
5. Active Safety and Haptic Feedback: While the user is walking, the system continuously monitors the area directly in front of them. If they deviate from the path and get too close to a wall or obstacle, vibration motors embedded in the chest straps will trigger, providing an immediate physical warning to stop.

## Hardware and Software Composition
This project bridges the gap between physical embedded hardware and advanced Artificial Intelligence. The complete architecture relies on several interconnected systems:

* The Core Processing Unit: A small, high-performance computer housed in a lightweight backpack. This acts as the brain of the device, running all the artificial intelligence models and performing the heavy mathematical calculations required for navigation.
* Environmental Sensors: The "eyes" of the system. This includes a high-definition RGB camera used exclusively to identify objects, paired with a laser-based distance sensor that continuously measures the exact distance in feet between the user and surrounding walls.
* Peripheral Microcontroller: A small electronic chip that acts as the physical bridge between the user and the core computer. It manages the physical push-buttons, handles the audio stream to the Bluetooth earpiece, and regulates the vibration motors.
* Artificial Intelligence Engine: The software layer of the project features advanced object-recognition programming trained to instantly recognize dozens of common household items, paired with navigation software that calculates safe walking routes.

## Key Project Objectives
1. Enhance User Independence: To engineer a reliable system that allows visually impaired users to confidently navigate unknown indoor environments without requiring physical human assistance.
2. Prioritize Physical Safety: To build a system that actively steers users away from structural hazards and provides immediate physical warnings before collisions occur.
3. Implement Natural Human Interaction: To avoid complicated technological interfaces by utilizing natural, human-like voice instructions (such as feet and clock-directions) that can be understood instantly without a steep learning curve.
4. Ensure Comfort and Discretion: To design a wearable hardware system that is lightweight, ergonomically sound, and visually discreet enough to be worn comfortably in public spaces.
