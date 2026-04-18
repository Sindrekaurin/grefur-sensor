# Grefur_Module_Driver

A lightweight, high-performance I2C Slave protocol driver for **grefur** sensor nodes. This library enables plug-and-play metadata discovery and live data streaming between controllers.

## Features

- **Automatic Discovery**: Master nodes can query register names, types, and permissions without prior configuration.
- **Typed Data Handling**: Built-in support for `U8`, `I16`, `U16`, and `I32` data types.
- **Pointer-Based Mapping**: Link your local C++ variables directly to I2C registers. The driver handles the byte-shifting and communication in the background.
- 
## Installation

1. Create a folder named `Grefur_Module_Driver` in your `libraries` directory.
2. Place `Grefur_Module_Driver.h`, `Grefur_Module_Driver.cpp`, and `library.properties` inside that folder.
3. Restart your IDE.

## Quick Start

```cpp
#include <Grefur_Module_Driver.h>

// Initialize with a unique Device ID (e.g., 5001 for Pressure Node)
GrefurModule node(5001);

// Local variables to be exposed via I2C
int32_t currentPressure = 101325;
uint8_t sensorStatus = 0;

void setup() {
  // Map variables to registers
  // node.addRegister(Addr, Type, Writable, "Name", &Variable);
  node.addRegister(0x12, GREFUR_T_I32, false, "pres", &currentPressure);
  node.addRegister(0x20, GREFUR_T_U8,  false, "stat", &sensorStatus);
  
  // Start I2C slave on address 0x09
  node.begin(0x09);
}

void loop() {
  // Your sensor acquisition logic here
  // The GrefurModule handles all I2C requests via interrupts
  currentPressure = readPressureSensor(); 
  delay(100);
}
