# WRO FE Monthly Log  – July 
(Code Consistency & Hardware Adjustments)

### July 1 – Code Consistency & Hardware Adjustments

- Achieved **major consistency** in Round 1 code execution.
- Modified **wheel gear ratios**: switched from a speed‑oriented ratio to a torque‑oriented ratio for better control.
- Adjusted **colour sensor positioning**: added a **6 mm spacer** to lower the sensor, resulting in more accurate readings.

### July 2 – Refinement & Sensor Issue

- Focused on improving **overall consistency** of runs.
- Identified a **gyro sensor drifting issue** that required correction.

### July 3 – Team Sync

- Held a **team meeting** to review progress and align on outstanding challenges.
- Discussed gyro drift and sensor calibration as next steps.

### July 25 – OpenGL Course & Route Mapping

- Began an **OpenGL course** to integrate advanced visualization and mapping into the robot’s navigation system.
    
- **Goal of OpenGL integration:**
    
    - Use **gyro + accelerometer data** in combination with mat mapping.
    - Detect **pillar positions** and **colour IDs** using Huskylens.
    - Filter sensor data through an **ID set**, then through a **route set**.
    
- **Planned workflow:**
    - **Lap 1:** Robot maps the board, calibrates speed, and identifies obstacle layout IDs on each side.
    - **Lap 2 & 3:** Robot replays Lap 1 route with improved speed and consistency.