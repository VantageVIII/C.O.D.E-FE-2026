
# WRO FE Monthly Log – January 
(Research and Slow Start)

### Repository & Documentation

- Created a GitHub repository to serve as a centralized backup and version control system for all project files throughout the year.

### Power System Development

- Experimented with designing a custom power pack for the RDK X5.
- Ultimately selected a standard configuration: **3× 18650 lithium cells in series**, mounted on the designated battery sled.
- Integrated a **5A 5V buck converter** and a **5A inline blade fuse** to ensure stable voltage regulation and protection for the RDK.

### Software & Environment Setup

- Conducted research on **ROS2** for potential integration.
- Set up the RDK X5 environment:
    - Learned to flash NAND using **X Burn** and to flash the RDK using **RDK Studio**.
    - Navigated the **Ubuntu GNOME environment**.
    - Installed essential libraries and tools: `matplotlib`, `numpy`, `smbus`, `smbus2`, `pip`, `git`, `VL53L0x`, `DFRobot I2C-Multiplexer`, `xfce`, `TCS34725`, `I2C-tools`.
        

### Hardware Research & Design

- **3D Printing Materials**: Researched filament options and selected **ABS** for chassis printing due to durability and heat resistance.
- **Camera Selection**: Chose **Huskylens V1** for vision capabilities.
- **Chassis Improvements compared to 2025 robot**:
    
    - Shortened wheelbase.
    - Lowered center of gravity by relocating battery pack to the bottom.
    - Added a separate layer for I2C lines.
    - Aligned time-of-flight sensors above wheels for improved responsiveness.

### Sensor & Input Enhancements compared to 2025 robot

- Upgraded front sensor to **VL53L1x** for better responsiveness.
- Replaced mechanical button with **TP223 Capacitive Touch Sensor** to reduce mechanical failure risk and wiring complexity.

### Thermal Management

- Investigated cooling solutions after observing RDK idle temperatures averaging **60°C**, which could interfere with I2C performance.
- Considered adding heatsinks to improve thermal stability.

### Progress Challenges

- Development pace slowed due to participation in **FTC (FIRST Tech Challenge)** during the month, which required time and focus alongside research activities.