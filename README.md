# Gas Sensor EHMR Firmware

This repository contains the firmware for the **Elastomeric Half Mask Respirator (EHMR)** with integrated environmental sensing. The system runs on an **ATmega328PB** microcontroller and communicates with multiple I²C and analog sensors to monitor real-time air quality, transmitting results to a connected Bluetooth mobile application.

---

## About This Project

This project was developed as part of the San Diego State University (SDSU) Senior Design capstone course through a collaboration between the Department of Electrical and Computer Engineering and the Department of Mechanical Engineering. The system represents a multidisciplinary effort to design, build, and test a smart elastomeric half-mask respirator (EHMR) that integrates real-time environmental sensing, embedded firmware, Bluetooth communication, and ergonomic design to support first responder safety.

---

## Supported Sensors

| Sensor               | Function                             | Interface | I²C Address |
|----------------------|--------------------------------------|-----------|-------------|
| SCD41                | CO₂, Temperature, Humidity           | I²C       | 0x62        |
| SPS30                | Particulate Matter (PM1.0–PM10.0)    | I²C       | 0x69        |
| XGZP6847A            | Pressure (analog, -40–40 kPa)        | ADC       | N/A         |
| HM-10 BLE Module     | Wireless Bluetooth Communication     | UART      | N/A         |

---

## Wiring Guide

Connect the following components to the ATmega328PB:

| Pin     | Signal        | Description                         |
|---------|---------------|-------------------------------------|
| PC3     | ADC3          | Pressure sensor analog input        |
| SDA/SCL | TWI0 / TWI1   | SCD41 Internal/External, SPS30      |
| PD0/PD1 | UART0         | HM-10 UART interface (TX/RX)        |
| VCC     | Power         | 3.3V or 5V regulated supply          |
| GND     | Ground        | Common ground                       |

---
