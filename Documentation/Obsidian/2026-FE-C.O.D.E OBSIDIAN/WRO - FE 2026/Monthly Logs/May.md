
# WRO FE Monthly Log – May 
(System Integration & Round 1 Coding)

### Major Milestones

- **May 2**: Achieved **full robot movement**:
    - Steering system operational.
    - Motors successfully controlled.
    - **Colour sensor** integrated and functioning.

- **May 18**: Received and successfully set up the **Huskylens V1 camera**.
    
- **May 23**: Made a major design decision:
    - Removed the **time-of-flight sensor layer** entirely.
    - Transitioned to a simplified sensor suite: **Huskylens camera**, **colour sensor**, and an **additional gyro sensor**.
    
- **May 26**: Recieved, installed and configured the **gyro sensor**, ensuring stable readings and integration with the control system.
    
- **May 27**: Began developing the **full Round 1 code** and conducted initial testing.

### Control Method – Bit-Bashing

- Continued using **bit-bashing** instead of PWM for servo steering and motor controller ENA pin control.
    
- **Bit-bashing explained**:
    - A software-driven method of manually toggling GPIO pins to simulate control signals.
    - Provides flexibility and direct control without relying on hardware PWM modules.
    - Requires precise timing loops in code to maintain accuracy and avoid drift.

### Round 1 Code Plan

- Robot runs using **only the gyro and colour sensor**.
- **Forward movement** until detecting either **blue** or **orange**.
- Rotation logic:
    - Maintains an **array of rotation values** for orientation.
    - If **blue** is detected first → assigns **counterclockwise array order**.
    - If **orange** is detected first → assigns **clockwise array order**.
- **Gyro sensor** ensures straight-line stability.
- **Colour sensor** acts as the trigger for turns:
    - When the robot sees the same colour it detected at the start, it executes the corresponding turn.
- **Testing results**:
    
    - Right turns are smooth and stable.
    - Left turns show issues when synchronized with the gyro, requiring further refinement.

### Mechanical Adjustments

- Adjusted the **steering linkage** for proper alignment.
- Added a **2° toe-in angle** on the wheels to improve stability and responsiveness.