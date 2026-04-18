# Grefur I2C Ecosystem

This repository contains the core communication framework for **grefur AS** hardware. The system is designed to provide high-quality, plug-and-play simplicity for SaaS-connected sensors.

## Architecture Overview

The system operates on a specialized I2C protocol that separates the logic into two distinct roles:

1.  **Master (Gateway)**: Typically a powerful microcontroller (e.g., ESP32-S3-WROOM, STM32). It manages the I2C bus, discovers connected sensors dynamically, and bridges the data to the Grefur ecosystem, making it ready for production use.
2.  **Slave (Sensor Node)**: Low-power microcontrollers (e.g., ESP32-C3, ATTiny) that handle physical sensing. They expose their data via a standardized register-map that the Master can read without any prior manual configuration.

## Folder Structure

- `/master`: Contains the `Grefur_Master_Driver` library.
- `/slave`: Contains the `Grefur_Slave_Driver` library.
- `/examples`: Implementation examples for various sensor types.

## Why this protocol?

Traditional I2C implementations require the Master to know exactly what is connected at which address. This **protocol** implements a proprietary **Metadata Discovery** overhead:

- **Plug & Play**: Connect any Grefur Sensor Module to the bus; the Master automatically identifies its name, data types, and capabilities.
- **Scalability**: Add new sensor types to your product line without ever updating the Master's core communication firmware.
- **Reliability**: Built-in error handling and retries ensure data integrity across different operating environments.

## How to Implement

### 1. Creating a Sensor (Slave)
Use the `Grefur_Slave_Driver`. Map your local variables to I2C registers using `addRegister()`. This makes your sensor data instantly visible to the Master.

### 2. Creating a Gateway (Master)
Use the `Grefur_Master_Driver`. Run `scanAndDiscover()` to populate a list of all active sensors on the bus. Use `pollDevice()` to fetch live data for further processing or cloud uplink.

---

### License
This project is licensed under the **MIT License**. You are free to use, modify, and distribute – just give credit.

---
Made with ❤️ by **Grefur AS** *Quality and Simplicity for the End User.*
