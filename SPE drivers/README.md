
# ESP32 ADIN1110 Driver
## 10BASE-T1L Single Pair Ethernet over SPI

This ESP-IDF component provides a full Ethernet driver for the **Analog Devices ADIN1110** 10BASE-T1L MAC/PHY, connecting an ESP32 to a single twisted-pair network.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  ESP32                                                          │
│                                                                 │
│  ┌──────────┐    SPI (up to 25 MHz)    ┌────────────────────┐  │
│  │  lwIP    │◄──────────────────────── │   adin1110.c       │  │
│  │ TCP/IP   │                          │   Driver           │  │
│  └──────────┘   INT# (active-low GPIO) └────────┬───────────┘  │
│                                                  │              │
└──────────────────────────────────────────────────┼─────────────┘
                                                   │ SPI + GPIO
                                          ┌────────▼───────────┐
                                          │   ADIN1110         │
                                          │   MAC + PHY        │
                                          │   10BASE-T1L       │
                                          └────────┬───────────┘
                                                   │ 100 Ω twisted pair
                                                   │ (up to 1000 m @ 2.4 V)
                                          ┌────────▼───────────┐
                                          │  Remote Device     │
                                          │  (PLC, sensor,     │
                                          │   another ESP32…)  │
                                          └────────────────────┘
```

---

## Hardware Wiring

### Minimal connection (no hardware reset)

| ESP32 GPIO | ADIN1110 Pin | Function             | Notes                      |
|-----------|-------------|----------------------|----------------------------|
| 18        | SCLK        | SPI clock            | Up to 25 MHz               |
| 23        | MOSI / SDI  | SPI data to device   |                            |
| 19        | MISO / SDO  | SPI data from device |                            |
| 5         | /CS         | Chip select          | Active-low                 |
| 4         | /INT        | Interrupt            | Active-low, needs pull-up  |
| 3V3       | VDD         | Power                | 100 nF decoupling cap      |
| GND       | GND         | Ground               |                            |

### With hardware reset (recommended)

| ESP32 GPIO | ADIN1110 Pin | Notes                                |
|-----------|-------------|--------------------------------------|
| 2         | /RESET      | Active-low; 10 kΩ pull-up to 3V3    |

### Single Pair Ethernet cable

Connect the ADIN1110 **PA+** and **PA−** pads to a standard **100 Ω STP or UTP** twisted pair cable. Termination resistors (100 Ω differential) should be placed at **both** ends of the cable.

```
ADIN1110                              Remote Device
  PA+ ──────────────┐ ┌─────────────── PA+
                    │ │   100Ω term
  PA− ──────────────┘ └─────────────── PA−
```

---

## Quick Start

### 1. Add the component

Copy the `adin1110_driver/` folder into your ESP-IDF project's `components/` directory:

```
my_project/
├── components/
│   └── adin1110_driver/
│       ├── CMakeLists.txt
│       ├── include/
│       │   ├── adin1110.h
│       │   └── adin1110_regs.h
│       └── src/
│           └── adin1110.c
├── main/
│   └── main.c
└── CMakeLists.txt
```

### 2. Add to your project CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_app)
```

### 3. Initialize the driver

```c
#include "adin1110.h"

adin1110_handle_t adin_dev;

adin1110_config_t cfg = {
    .spi_host     = SPI2_HOST,
    .pin_miso     = 19,
    .pin_mosi     = 23,
    .pin_sclk     = 18,
    .pin_cs       = 5,
    .pin_int      = 4,
    .pin_rst      = 2,           // -1 if not connected
    .spi_clock_hz = 12000000,    // 12 MHz (safe default)
    .tx_level     = ADIN1110_TX_LEVEL_2V4,  // 2.4 V for long cables
    .spi_prot     = ADIN1110_SPI_PROT_NONE,
    .append_crc   = true,
    .mac_addr     = {0x02, 0xAD, 0x1E, 0x10, 0x00, 0x01},
};

ESP_ERROR_CHECK(adin1110_init(&cfg, &adin_dev));
ESP_ERROR_CHECK(adin1110_start(adin_dev));
```

### 4. Attach to lwIP

```c
esp_netif_init();

esp_netif_inherent_config_t base = ADIN1110_NETIF_DEFAULT_CONFIG();
esp_netif_config_t netif_cfg = {
    .base  = &base,
    .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
};
esp_netif_t *netif = esp_netif_new(&netif_cfg);

adin1110_attach_netif(adin_dev, netif);
esp_netif_set_mac(netif, cfg.mac_addr);
```

---

## Configuration Options

| Option        | Values                                     | Description                                |
|---------------|--------------------------------------------|--------------------------------------------|
| `tx_level`    | `ADIN1110_TX_LEVEL_1V0` / `_2V4`          | 1.0 V = cables <15 m; 2.4 V = up to 1 km  |
| `spi_prot`    | `ADIN1110_SPI_PROT_NONE` / `_CRC`         | CRC-8 adds SPI frame integrity checking    |
| `append_crc`  | `true` / `false`                           | Let the ADIN1110 MAC append FCS to TX      |
| `spi_clock_hz`| 1 MHz … 25 MHz                             | Start at 12 MHz; increase if signal is clean |

---

## Files

| File                        | Purpose                                             |
|-----------------------------|-----------------------------------------------------|
| `include/adin1110.h`        | Public API                                          |
| `include/adin1110_regs.h`   | Register map and bit definitions                    |
| `src/adin1110.c`            | Driver implementation                               |
| `example/main.c`            | Full working example with lwIP integration          |
| `CMakeLists.txt`            | ESP-IDF component build file                        |

---

## API Reference

```c
// Initialize hardware and verify chip
esp_err_t adin1110_init(const adin1110_config_t *cfg, adin1110_handle_t *out_dev);

// Attach to esp_netif for lwIP integration
esp_err_t adin1110_attach_netif(adin1110_handle_t dev, esp_netif_t *netif);

// Start: enable interrupts, launch RX task
esp_err_t adin1110_start(adin1110_handle_t dev);

// Stop driver gracefully
esp_err_t adin1110_stop(adin1110_handle_t dev);

// Free all resources
esp_err_t adin1110_deinit(adin1110_handle_t dev);

// Send a raw Ethernet frame (no FCS needed)
esp_err_t adin1110_send(adin1110_handle_t dev, const uint8_t *buf, size_t len);

// Check physical link state
esp_err_t adin1110_get_link(adin1110_handle_t dev, bool *link_up);

// Direct register access
esp_err_t adin1110_reg_read(adin1110_handle_t dev, uint16_t reg, uint32_t *out);
esp_err_t adin1110_reg_write(adin1110_handle_t dev, uint16_t reg, uint32_t val);

// MDIO (internal PHY registers)
esp_err_t adin1110_mdio_read(adin1110_handle_t dev, uint8_t phy, uint8_t reg, uint16_t *out);
esp_err_t adin1110_mdio_write(adin1110_handle_t dev, uint8_t phy, uint8_t reg, uint16_t data);
```

---

## Troubleshooting

| Symptom                         | Likely Cause                                    | Fix                                              |
|---------------------------------|-------------------------------------------------|--------------------------------------------------|
| IDVER read returns 0x00000000   | SPI wiring incorrect or bad solder joint        | Check MISO/MOSI/SCLK/CS connections              |
| Reset timeout                   | /RESET held low or VDD issue                    | Check power supply, add 100 nF decoupling cap    |
| Link never comes up             | Termination resistors missing                   | Add 100 Ω across PA+/PA− at each cable end       |
| CRC errors on SPI               | Clock too fast for trace length                 | Reduce `spi_clock_hz` to 4–8 MHz                |
| Frames received but no IP       | esp_netif not attached                          | Call `adin1110_attach_netif()` before `start()`  |
| TX works, RX drops frames       | RX task priority too low                        | Increase `ADIN1110_RX_TASK_PRIORITY`             |

---

## Notes on 10BASE-T1L Cable Distance

| TX Voltage | Typ. Max Distance | Use Case                    |
|------------|-------------------|-----------------------------|
| 1.0 Vpp    | ~15 m             | Same panel / cabinet        |
| 2.4 Vpp    | ~1000 m           | Field sensors, IIOT, APL    |

The ADIN1110 is IEC 63171-6 (APL) compatible for use in industrial and process automation (e.g., PROFINET over SPE, EtherNet/IP over SPE).

---

## License

MIT – free to use in commercial and open-source projects.
