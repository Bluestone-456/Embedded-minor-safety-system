# Miners’ Environmental Safety and Detection (ESD) System

![Platform](https://img.shields.io/badge/Platform-STM32F407VG-blue)
![Language](https://img.shields.io/badge/Language-Embedded%20C-green)
![Communication](https://img.shields.io/badge/Communication-ZigBee%20%7C%20UART-orange)
![Wireless](https://img.shields.io/badge/Wireless-XBee-informational)
![IDE](https://img.shields.io/badge/IDE-Keil%20uVision-purple)
![Status](https://img.shields.io/badge/Project%20Status-Completed-brightgreen)

The **Miners’ Environmental Safety and Detection (ESD) Project** is a wireless, sensor-based embedded system designed to enhance **underground miner safety** by continuously monitoring environmental and physical conditions that may indicate hazardous situations.

The system uses a **dual-node STM32 architecture**, where a **sensor node** acquires real-time data from multiple safety-critical sensors and transmits it wirelessly using **ZigBee (XBee modules)** to a **monitoring node**, which displays the information on an OLED screen. This enables early detection of dangerous conditions and supports timely intervention to prevent accidents.

---

## System Overview

### Node 1 – Sensor & Transmission Node
- Interfaces with multiple environmental and physical sensors
- Continuously samples air quality,temperature, sound, vibration, and pressure data
- Formats sensor readings into human-readable ASCII packets
- Transmits data wirelessly via **ZigBee (XBee) module over UART**

### Node 2 – Monitoring & Display Node
- Receives sensor data wirelessly through ZigBee
- Uses **interrupt-driven UART reception**
- Parses incoming data fields
- Displays real-time readings on an **SSD1306 OLED display**
- Feature to display warnings on the OLED screen for abnormal readings

> ZigBee modules are configured in **transparent (AT) mode**, allowing the STM32 microcontrollers to treat the wireless link as a standard UART interface while the ZigBee hardware manages the wireless communication layer.

---

## Sensors & Safety Parameters

| Sensor | Interface | Purpose |
|------|----------|--------|
| MQ-series Gas Sensor | ADC | Detects hazardous gases such as methane and carbon monoxide |
| BMP280 | I2C | Monitors air pressure to detect ventilation or structural anomalies and temperature for heat or fire hazards |
| LM393 Sound Sensor | ADC | Detects abnormal noise (falling rocks, machinery faults) |
| Vibration Sensor | GPIO | Detects ground tremors or equipment instability |

Together, these sensors provide a **multi-layered safety monitoring system** suitable for harsh underground environments.

---

## Wireless Communication

- **Technology:** ZigBee (XBee modules)
- **MCU Interface:** UART
- **Operating Mode:** Transparent (AT mode)
- **Reception Method:** UART interrupt-based
- **Data Format:** ASCII text stream
- **Direction:** Node 1 → Node 2

### Example Transmitted Data
```
T:26.1 H:58 P:1009 G:342 S:215 V:1 D:124
```
> ZigBee protocol handling (routing, addressing, error correction) is managed entirely by the XBee modules, keeping the STM32 firmware lightweight and modular.

This human-readable format simplifies debugging, validation, and future system expansion.

---

## Hardware Components

- STM32F407VG Development Boards (×2)
- ZigBee (XBee) Wireless Modules (×2)
- MQ-series Gas Sensor
- BMP280 Pressure and Temperature Sensor
- LM393 Sound Detection Module
- Vibration Sensor
- SSD1306 OLED Display (I2C)

---

## Software & Tools

- **IDE:** Keil uVision
- **Programming Language:** Embedded C
- **Drivers / Libraries:** CMSIS, STM32 Standard Peripheral Drivers
- **Display Driver:** SSD1306 OLED (custom driver)
- **Communication:** UART with interrupt handling

---

## Project Structure
```
Project/
│── NODE1/ # Sensor & transmitter node
│ ├── main.c
│ └── peripheral drivers
│
│── NODE2/ # Receiver & display node
│ ├── main.c
│ ├── ssd1306.c / .h
│ ├── fonts.c / .h
│ └── display utilities
```
(Keil-generated build, RTE, and object files are included.)

---

## Build & Run Instructions

1. Open **NODE1** project in Keil uVision
2. Select target device `STM32F407VGTx`
3. Build and flash firmware to Node 1
4. Open **NODE2** project in Keil uVision
5. Build and flash firmware to Node 2
6. Connect STM32 UART TX/RX to ZigBee modules
7. Power both nodes and ensure ZigBee pairing
8. Observe live environmental data on the OLED display

---

## Key Features

- Wireless environmental monitoring using ZigBee
- Multi-sensor safety data acquisition
- Interrupt-driven UART communication
- Real-time OLED visualization
- Modular and scalable embedded architecture
- Cost-effective and practical for underground deployment

---

## Design Insights

- Using ZigBee in transparent mode avoids the overhead of implementing a full wireless protocol stack on the MCU.
- UART abstraction allows easy replacement of ZigBee with other wireless technologies (LoRa, BLE, ESP-NOW).
- Multi-sensor fusion improves reliability and reduces false positives in hazard detection.

---

## Future Enhancements

- Add CRC/checksum for data integrity
- Implement bidirectional communication and alerts
- Introduce packet framing and acknowledgements
- Migrate to ZigBee API mode for advanced networking
- Enable low-power modes for battery-powered deployment
- Add mobile or control-room monitoring interface

---

## Author

**Aastha Sinha**  
BITS Pilani

---

## License

This project is intended for academic and learning purposes. Free to use and modify with proper attribution.

