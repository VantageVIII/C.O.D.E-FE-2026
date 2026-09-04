The **RDK X5** was chosen as the main controller for the robot because it offers the best balance of **AI acceleration, connectivity, and GPIO support** compared to other SBCs in the comparison:

- **AI Acceleration:**
    - Equipped with a **10 TOPS BPU (Neural Processing Unit)**, the RDK X5 is far more capable for edge AI and robotics tasks than the Raspberry Pi 5 (limited AI acceleration) and Jetson Nano (fixed 4 GB RAM, older GPU).

- **Connectivity:**
    - Built‑in **Wi‑Fi 6 + Bluetooth 5.4** ensures modern wireless standards, unlike the Jetson Nano which lacks onboard Wi‑Fi.
- **GPIO & Interfaces:**
    - Provides **28 GPIOs** with support for **SPI, I²C, I²S, PWM, and UART**, covering all the communication protocols needed for sensors and actuators in the robot.
        
- **Form Factor:**
    - Compact dimensions (**85 × 56 × 20 mm**) make it stackable and ideal for embedded robotics, unlike the larger Jetson Nano or LattePanda Delta 3.
        
- **Power Input:**
    - Runs on a simple **5V/5A USB‑C supply**, compatible with the robot’s buck converter and battery system.
        
- **OS Support:**
    - Supports **Ubuntu 22.04 and RDK OS Linux**, giving flexibility for robotics frameworks like ROS2.

### Why Not the Others?

- **Raspberry Pi 5:** Excellent community support and dual 4K display, but limited AI acceleration and requires active cooling.
- **Jetson Nano:** Strong AI features with CUDA, but restricted by fixed 4 GB RAM and no onboard Wi‑Fi.
- **LattePanda Delta 3:** Runs Windows/Linux with powerful x86 CPU, but higher power draw and larger form factor make it less suited for compact robotics.

### Conclusion

The **RDK X5** was selected because it combines **AI performance, modern connectivity, versatile GPIO support, and compact design**, making it the most suitable SBC for an **edge AI robotics platform**.