# Inter‑Integrated Circuit (I²C)

**Definition:** I²C is a synchronous, multi‑master, multi‑slave, serial communication protocol developed by Philips (now NXP). It enables short‑distance communication between integrated circuits using only two bidirectional lines: **SDA (Serial Data)** and **SCL (Serial Clock)**.

### Key Characteristics

- **Synchronous Communication:** Data transfer is synchronized with a shared clock (SCL).
- **Two‑Wire Interface:** Only SDA and SCL lines are required, plus ground.
- **Multi‑Device Support:** Multiple masters and slaves can share the same bus.
- **Addressing:** Each slave device has a unique 7‑bit or 10‑bit address.
- **Open‑Drain Design:** Devices pull the line low; external pull‑up resistors are required.

### Typical Parameters

- **Clock Speed:** Standard (100 kHz), Fast (400 kHz), Fast Plus (1 MHz), High‑Speed (3.4 MHz).
- **Data Frame Structure:**
    
    - **Start condition** (SDA pulled low while SCL is high)
    - **Address frame** (7/10 bits + R/W bit)
    - **ACK/NACK bit** (acknowledgment from receiver)
    - **Data frames** (8 bits each, followed by ACK/NACK)
    - **Stop condition** (SDA released high while SCL is high)

### Applications

- Communication between microcontrollers and peripherals (EEPROMs, sensors, displays).
- Configuration of ICs in embedded systems.
- Power management ICs and real‑time clocks.
- Consumer electronics (smartphones, TVs, cameras).

### Hardware Interface

- **SDA (Serial Data):** Transfers data between devices.
- **SCL (Serial Clock):** Synchronizes data transfer.
- **Pull‑Up Resistors:** Required on both lines to maintain logic high when idle.

### Advantages

- Simple wiring (two lines for many devices).
- Supports multiple masters and slaves.
- Well‑suited for low‑speed, short‑distance communication.

### Limitations

- Limited speed compared to SPI or UART.
- Bus capacitance restricts maximum distance and number of devices.
- Requires careful pull‑up resistor sizing for reliable operation.

# Components Using I2C:
[[TCS34725 - Colour Sensor]]