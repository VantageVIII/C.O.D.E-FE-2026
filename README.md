# Team C.O.D.E. - WRO Future Engineers 2026

**Team Name:** Cognitive Operations & Digital Engineering (C.O.D.E.)
**Team Members:** Jadon Steele & Mischa Miller
**Robot Name:** MSE-6 (Alias: "Mouse")

Welcome to the official GitHub repository for Team C.O.D.E. This repository contains all the source code, 3D CAD models, documentation, and media for our autonomous vehicle designed for the World Robot Olympiad (WRO) Future Engineers 2026 competition. 

Our engineering philosophy this season focuses on **simplification and consistency**. By upgrading to an RDK X5 edge computer and a tri-camera HuskyLens vision system, we shifted the complexity from hardware sensor arrays to software processing, resulting in a highly reliable, reproducible autonomous system.

---

## 1. Repository Structure

Our repository is organized to ensure full reproducibility for any team wishing to recreate the MSE-6:

* `/Codes/`: Contains all software required to run the robot.
  * `/Base Codes/`: Contains low-level hardware interfacing scripts (e.g., pin definitions, bit-bashing logic, motor controller initialization).
  * `/Main Codes/`: Contains the high-level logic, including the finite state machine, mapping algorithms, and obstacle avoidance logic for the competition runs.
  * `/Test Codes/`: Contains isolated scripts used during development to test individual components (e.g., servo calibration, camera frame testing).
* `/models/`: Contains all 3D-printable `.STL` files for our custom ABS chassis, including the upper plate, lower plate, ribs, and camera mounting brackets.
* `/Documentation/`: Contains our Engineering Journal, component comparisons, color value logs, and problem-solving logs.
* `/media/`: Contains photos and videos of the robot in action, including official qualification run recordings.
* `/other/`: Contains reference materials, rulebooks, and legacy code from the 2025 season for comparative analysis.

---

## 2. Electromechanical Components & Hardware Architecture

The MSE-6 is built using carefully selected commercial off-the-shelf components combined with a custom-fabricated chassis.

### Main Controller
* **RDK X5 SBC:** The brain of our robot. Featuring an octa-core ARM Cortex-A55 processor, 8 GB of LPDDR4 memory, and a 10 TOPS NPU (Neural Processing Unit). We selected the RDK X5 over alternatives like the Raspberry Pi 5 because of its superior AI acceleration capabilities, which are essential for processing three simultaneous camera feeds without latency. 

### Vision & Sensor System
* **3x HuskyLens V1 AI Cameras:** We utilize a tri-camera setup. One camera faces downward to replace traditional color sensors, utilizing advanced pattern recognition to map the track lines (blue/orange) at high speeds. Two forward-facing cameras are angled to provide a combined 120° field of view, detecting track boundaries and red/green traffic pillars without blind spots.
* **DFRobot 6-Axis Accelerometer/Gyro:** Mounted precisely on the chassis center axis. This provides crucial heading data to maintain perfectly straight trajectories between turns.

### Actuation & Movement
* **Drive Motor:** A 12V 250 RPM Planetary DC motor. We opted for a high-torque ratio rather than a high-speed ratio to ensure smooth, predictable acceleration.
* **Steering:** A DFRobot DS-R001 6Kg Clutch Servo. The internal clutch mechanism is a critical reliability upgrade—if the robot collides with a wall, the clutch slips instead of stripping the servo gears, preventing catastrophic mechanical failure.
* **Motor Driver:** L298N Dual H-Bridge, chosen for its simplicity, thermal robustness, and reliability in driving our 12V planetary motor.

### Power System
* **Battery:** 3S1P 18650 Lithium-ion pack (11.1V nominal). 
* **Power Regulation:** The 11.1V line runs through a 5A inline blade fuse for short-circuit protection, then into an XL4015 Buck Converter which steps the voltage down to a stable 5V (5A max) to power the RDK X5 and logic components safely.

---

## 3. Step-by-Step Build & Assembly Guide

To reproduce the MSE-6, follow these mechanical and electrical assembly instructions.

### Mechanical Assembly
1. **Print the Chassis:** Navigate to the `/models/` directory and 3D print the upper and lower chassis plates using ABS filament. ABS is strictly recommended over PLA due to the high operating temperatures of the RDK X5 (approx. 60°C idle).
2. **Mount the Drivetrain:** Secure the 12V Planetary DC motor to the rear motor bracket of the lower chassis. Attach the rear solid axle and wheels.
3. **Install the Steering System:** Mount the DFRobot 6Kg Clutch Servo to the front of the lower chassis. Connect the steering linkage to the front wheel hubs, ensuring a maximum steering angle of 50° is achievable.
4. **Assemble the Tiers:** Use the provided nylon spacers to mount the upper chassis plate above the lower plate. Ensure the structural ribs are aligned to distribute torque stress.
5. **Mount the Sensors:** Attach the custom 3-camera bracket to the upper chassis. Angle the forward cameras outwards at 30° each from the center line. Mount the Gyro perfectly parallel to the chassis centerline.

### Electrical Wiring & Integration
*[PLACEHOLDER: Insert Markdown Image Link to Wiring Diagram here. E.g., `![Wiring Diagram](./media/wiring_diagram.png)`]*

1. **Power Routing:** Connect the 3S1P battery to the 5A inline fuse. Split the output: route one parallel line directly to the 12V input of the L298N motor driver. Route the other line into the XL4015 Buck Converter.
2. **Logic Power:** Tune the XL4015 output to exactly 5.0V using a multimeter. Connect the 5V output to the RDK X5 power input pins.
3. **Motor Control:** Connect the RDK X5 GPIO pins to the IN1, IN2, and ENA pins on the L298N. (Note: We use software bit-bashing for control rather than hardware PWM).
4. **Sensor Comms:** Connect the three HuskyLens cameras and the DFRobot Gyro to the I2C/UART interface pins on the RDK X5. Ensure common ground across all components.

---

## 4. Software Architecture & Execution

Our software is written in Python and is designed to run on the RDK OS (Ubuntu 22.04 base). It heavily utilizes modular programming to separate hardware control from high-level decision-making.

### Module Breakdown
* **Hardware Interface Module:** Handles the direct GPIO bit-bashing. Because the RDK X5 requires specific pin mappings, this module manually toggles pins to simulate PWM for the drive motor and translates angle requests into pulse widths for the clutch servo.
* **Vision Processing Module:** Interfaces with the HuskyLens cameras via UART/I2C. It pulls bounding box data, color IDs, and block coordinates, filtering out false positives.
* **Navigation & State Machine Module:** The core logic. It merges gyro heading data with camera vision data to determine the robot's current state.

### State Machine Logic
1. **State 1: Search & Map (Lap 1):** The robot drives forward using the gyro to maintain heading. The downward camera looks for orange or blue markers. Upon detecting a color, it assigns the turning sequence (e.g., Blue = Counterclockwise track). The robot maps the location of red/green pillars.
2. **State 2: Lane Follow (Laps 2 & 3):** The robot increases speed, relying on the mapped data and forward cameras to maintain its position relative to the inner wall.
3. **State 3: Obstacle Avoidance:** If a green pillar is detected, the software calculates a proportional steering offset to safely pass the pillar on the left. If a red pillar is detected, it calculates an offset to pass on the right. Once the bounding box of the pillar passes the camera's FOV, the gyro re-establishes the straight-line heading.

### Installation & Execution Guide

To run the software on a fresh RDK X5 setup:

1. **Flash the OS:** Flash RDK OS Linux (Ubuntu 22.04 compatible) onto the RDK X5 using RDK Studio.
2. **Clone the Repository:**
   ```bash
   git clone https://github.com/VantageVIII/C.O.D.E-FE-2026.git
   cd C.O.D.E-FE-2026