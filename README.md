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
  * `/Test Codes/`: Contains isolated scripts used during development to test individual components.
* `/models/`: Contains all 3D-printable `.STL` files for our custom ABS chassis, including the upper plate, lower plate, ribs, and camera mounting brackets.
* `/Documentation/`: Contains our Engineering Journal, component comparisons, color value logs, and problem-solving logs.
* `/media/`: Contains photos and videos of the robot in action, including official qualification run recordings.
* `/other/`: Contains reference materials, rulebooks, and legacy code from the 2025 season for comparative analysis.

---

## 2. Electromechanical Components & Hardware Architecture

The MSE-6 is built using carefully selected commercial off-the-shelf components combined with a custom-fabricated chassis.

### Main Controller
* **RDK X5 SBC:** The brain of our robot. Featuring an octa-core ARM Cortex-A55 processor, 8 GB of LPDDR4 memory, and a 10 TOPS NPU (Neural Processing Unit). We selected the RDK X5 over alternatives because of its superior AI capabilities, which are essential for processing three simultaneous camera feeds. 

### Vision & Sensor System
* **3x HuskyLens V1 AI Cameras:** We utilize a tri-camera setup. One camera faces downward to read mat colors. Two forward-facing cameras are angled to provide a combined 120° field of view, detecting track boundaries and traffic pillars without blind spots.
* **DFRobot 6-Axis Accelerometer/Gyro:** Mounted precisely on the chassis center axis. This provides crucial heading data to maintain perfectly straight trajectories between turns.

### Actuation & Movement
* **Drive Motor:** A 12V 250 RPM Planetary DC motor. We opted for a high-torque ratio rather than a high-speed ratio. Because our code relies on GPIO bit-bashing for motor control, a torque-oriented layout allows for much smoother, less twitchy movement.
* **Steering:** A DFRobot DS-R001 6Kg Clutch Servo. The internal clutch mechanism is a critical reliability upgrade—if the robot collides with a wall, the clutch slips instead of stripping the servo gears.
* **Motor Driver:** L298N Dual H-Bridge.

### Power System
* **Battery:** 3S1P 18650 Lithium-ion pack (11.1V nominal). 
* **Power Regulation:** The 11.1V line runs into an XL4015 Buck Converter which steps the voltage down to a stable 5V. From the buck converter, the 5V line passes through a **5A inline blade fuse** before routing to the RDK X5 and logic components. This specific topology ensures that our 5V circuit is strictly protected from drawing more than the buck converter's maximum rating if a short occurs.

---

## 3. Step-by-Step Build & Assembly Guide

### Mechanical Assembly
1. **Print the Chassis:** Navigate to the `/models/` directory and 3D print the upper and lower chassis plates using ABS filament. ABS is required due to the operating temperatures of the RDK X5.
2. **Mount the Drivetrain:** Secure the 12V Planetary DC motor to the rear motor bracket of the lower chassis. Attach the rear solid axle and wheels.
3. **Install the Steering System:** Mount the DFRobot 6Kg Clutch Servo to the front of the lower chassis. Connect the steering linkage to the front hubs.
4. **Assemble the Tiers:** Use the provided nylon spacers to mount the upper chassis plate above the lower plate.
5. **Mount the Sensors:** Attach the custom 3-camera bracket to the upper chassis. Angle the forward cameras outwards at 30° each from the center line. 

### Electrical Wiring & Integration
*[PLACEHOLDER: Insert Markdown Image Link to Wiring Diagram here. E.g., `![Wiring Diagram](./media/wiring_diagram.png)`]*

1. **Power Routing:** Connect the 3S1P battery to the XL4015 Buck Converter. Route the 5V output of the buck converter through the 5A inline fuse.
2. **Logic Power:** Connect the output of the 5A fuse to the RDK X5 power input pins. Route a parallel 12V line directly from the battery to the L298N motor driver.
3. **Motor Control:** Connect the RDK X5 GPIO pins to the IN1, IN2, and ENA pins on the L298N. 
4. **Sensor Comms:** Connect the three HuskyLens cameras and the DFRobot Gyro to the I2C/UART interface pins on the RDK X5. 

---

## 4. Software Architecture & Game Strategy

Our software is written in Python and runs on the RDK OS. We utilize a highly optimized Finite State Machine (FSM) to separate our strategies for Round 1 (Open Challenge) and Round 2 (Obstacle Challenge).

### Round 1: Open Challenge Strategy
In Round 1, there are no traffic pillars, so our goal is maximum speed and efficiency.
1. **Orientation Logic:** The downward-facing camera scans the mat to identify the starting color (blue or orange), determining if the robot must travel clockwise or counterclockwise.
2. **Heading & Execution:** The robot relies entirely on the DFRobot Gyro for its heading. It is programmed to hug the **outer wall** as closely as possible, allowing for a wider turning radius and maximizing our top speed while remaining stable.

### Round 2: Obstacle Challenge Strategy
In Round 2, the robot must obey the red and green traffic pillars. To guarantee reliability, we split the logic between a "Mapping Lap" and "Execution Laps".
1. **Orientation Logic:** Similar to Round 1, the downward camera sets the initial orientation.
2. **Lap 1 (Corner Mapping & Evaluation):** The robot drives forward using gyro heading. Crucially, the robot is programmed to **stop** at the start of a corner turn. It uses the forward-facing HuskyLens to evaluate the traffic pillar situated at the corner. Based on the pillar's color and the robot's current orientation (CW/CCW), the algorithm determines if it needs to execute a **"narrow"** or **"sharp"** turn.
3. **Memory Storage:** The robot saves this specific corner turn type (narrow or sharp) into an array sequence.
4. **Laps 2 & 3 (Corner Execution & Real-Time Avoidance):** For the remaining two laps, the robot utilizes a hybrid approach:
   * **Corner Turns (Memory):** When the robot reaches the corners, it does not rely on the cameras. Instead, it reads the saved turn sequence (narrow or sharp) from memory and executes it "blindly." This is a vital engineering decision: by navigating corners purely from memory on Laps 2 and 3, the robot mitigates the risk of missing a pillar or executing a wrong turn due to minor offsets or imperfections that accumulate over the run.
   * **Straightaway Pillars (Real-Time):** For pillars located on the straight sides of the track, the robot actively uses the forward HuskyLens cameras to detect and avoid them in real-time, safely shifting lanes as necessary.

---

## 5. Installation & Execution Guide

To run the software on a fresh RDK X5 setup:
Application Download List:
[RDK Studio](https://d-robotics.github.io/rdk_x_doc/en/RDK/)
[Thonny](https://thonny.org/)
[Github Desktop](https://desktop.github.com/download/)
[Google Antigravity](https://antigravity.google/download)

1. **Flash the OS:** Download the official RDK Studio and flash RDK OS Linux onto the RDK X5.

2. **Clone the Repository:**
   ```bash
   git clone https://github.com/VantageVIII/C.O.D.E-FE-2026.git
   cd C.O.D.E-FE-2026

3.**Install Dependencies: Ensure Python 3 is installed. Then install the required libraries:**
    ```bash
    pip install matplotlib numpy smbus smbus2
    sudo apt-get install i2c-tools

4.**Run the Code: Navigate to the Main Codes directory and execute the competition script:**
    ```bash
    cd Codes/Main\ Codes/
    python3 main_run.py

6. **License and Credits**
Designed and programmed by Team C.O.D.E. (Jadon Steele & Mischa Miller) for the WRO Future Engineers 2026 Season. Special thanks to HelderBerg Robotics Club for their continued support, and Prints by Paul for 3D manufacturing assistance.