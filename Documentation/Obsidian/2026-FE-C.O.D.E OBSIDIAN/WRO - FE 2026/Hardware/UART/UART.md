## Universal Asynchronous Receiver/Transmitter (UART)

**Definition:** UART is a hardware communication protocol that enables asynchronous serial data transmission between devices. It converts parallel data from a microcontroller or computer into serial form for transmission, and reconverts received serial data back into parallel form.

### Key Characteristics

- **Asynchronous Communication:** No shared clock signal; synchronization is achieved using start and stop bits.
- **Full Duplex:** Supports simultaneous transmission (TX) and reception (RX).
- **Frame Structure:** Data is sent in packets consisting of:
    
    - **Start bit** (signals beginning of data frame)
    - **Data bits** (usually 7, 8, or 9 bits)
    - **Optional parity bit** (error detection)
    - **Stop bit(s)** (signals end of frame)

### Typical Parameters

- **Baud Rate:** Speed of transmission (e.g., 9600, 115200 bps).
- **Data Bits:** Commonly 8 bits per frame.
- **Parity:** None, even, or odd.
- **Stop Bits:** 1 or 2 bits.

### Applications

- Microcontroller ↔ Sensor communication (e.g., accelerometers, GPS modules).
- Debugging and logging via serial consoles.
- Embedded systems interfacing (Arduino, Raspberry Pi, etc.).
- Peripheral connections (Bluetooth modules, GSM modems).

### Hardware Interface

- **TX (Transmit):** Sends data out.
- **RX (Receive):** Reads incoming data.
- **GND:** Common ground reference.
- Optional **RTS/CTS** lines for hardware flow control.

### Advantages

- Simple and widely supported.
- Requires only two data lines (TX, RX).
- Flexible baud rate configuration.

### Limitations

- Point‑to‑point only (not multi‑drop like RS‑485).
- Limited distance and speed compared to modern protocols (e.g., USB, SPI).
- No inherent clock synchronization — requires precise baud rate matching.
# Components Using UART
[[HUSKYLENS V1]]
[[GYRO]]