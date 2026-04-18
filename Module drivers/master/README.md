# Grefur_Master_Driver

The orchestration engine for *Grefur-sensor*. This library manages the communication bus, discovers connected sensors (slaves), and provides high-level methods to poll data.

## Features

- **Bus Discovery**: Automatically scans the I2C bus and identifies grefur-compatible nodes.
- **Cluster Polling**: Update all registers of a device with a single command.
- **Robustness**: Built-in retries for noisy I2C environments.
- **Type Awareness**: Automatically handles byte-shifting for U8, U16, I16, and I32.

## Usage

```cpp
#include <Grefur_Master_Driver.h>

GrefurMaster grefur;

void setup() {
    Serial.begin(115200);
    grefur.begin(6, 7); // SDA, SCL
    
    // Discover all sensors on the bus
    uint8_t found = grefur.scanAndDiscover();
    Serial.printf("Found %u grefur devices\n", found);
}

void loop() {
    // Cluster Poll: Refresh all data from the first found device
    if (grefur.pollDevice(0)) {
        GrefurDevice* dev = grefur.getDevices();
        Serial.print("Sensor Value: ");
        Serial.println(dev[0].registers[0].lastValue);
    }
    delay(1000);
}
