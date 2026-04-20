# Grefur Sensor Backplate

This is the main repository for the **Grefur Sensor Backplate** – an open-source, modular IoT controller platform designed for flexibility, extensibility, and seamless integration into MQTT-based ecosystems. Whether you're building environmental monitors, automation controllers, or custom input/output devices – the **Sensor Backplate** is your universal starting point.

---

## What is the Grefur Sensor Backplate?

The Sensor Backplate is more than a sensor platform. It is a **programmable IoT controller** with:

- **Physical I/O** – Read digital and analog inputs, drive digital and analog outputs
- **Built-in logic engine** – Write expressions directly on the device to create automation rules without external tools
- **MQTT integration** – Publish sensor values, receive external signals, and bind remote values into local logic
- **Web UI** – Configure sensors, outputs, and logic from a browser with no additional software

> Developed and maintained by **Grefur AS**. The platform specification is open and public. Anyone can design compatible modules.

<img width="1298" height="766" alt="image" src="https://github.com/user-attachments/assets/4c39ff73-0cbe-4872-82f3-1b469f8e5125" />


---

## Backplate – The Heart of the System

The standardized **Sensor Backplate** is designed for universal mounting on walls, enclosures, HVAC systems, outdoor installations, and more:

- **Modular Expansion** – Any sensor or I/O module can be designed for easy integration
- **Technology Agnostic** – Compatible with ESP8266, ESP32, and more
- **IoT-First** – Simplified publishing of measured and calculated values into any MQTT ecosystem

---

## I/O Capabilities

The Sensor Backplate supports direct connection of physical hardware:

**Inputs**
- NTC temperature sensors
- Analog sensors (voltage, pressure, current, etc.)
- Digital inputs (buttons, switches, flow sensors, Hall-effect sensors)

**Outputs**
- Digital outputs (relays, solenoids, indicators)
- Analog/PWM outputs (variable speed drives, control valves, dimmers)

---

## Grefur Module Bus – I2C Expansion

The Sensor Backplate exposes a dedicated **I2C expansion bus**, enabling seamless integration of purpose-built add-on modules. This transforms the backplate from a fixed-function device into an open, scalable platform.

### How it works

The backplate acts as the **I2C master**. Each connected module runs the **Grefur Slave Driver** and responds to a standardized discovery and data protocol. On startup, the backplate scans the bus, reads each module's register map, and makes all discovered values immediately available to the logic engine and MQTT pipeline — with no manual configuration required.

```
Grefur Sensor Backplate (master)
    │
    ├── SDA ──────────────────────────────────┐
    ├── SCL ──────────────────────────────────┤
    │                                         │
    ├─── Module 0x10 ── Module 0x11 ── Module 0x12 ── ...
         (e.g. relay)   (e.g. CO₂)    (e.g. PWM out)
```

### What the bus gives you

| Capability | Detail |
|---|---|
| **Up to 16 modules** per bus | Each module gets a unique I2C address |
| **Up to 16 registers per module** | Mix of readable inputs and writable outputs |
| **Auto-discovery** | Backplate enumerates all modules and their register maps at startup |
| **Typed data** | Registers carry a data type (`U8`, `I16`, `U16`, `I32`, `F32`) — no guessing |
| **Named registers** | Each register has a human-readable name, surfaced directly in the web UI |
| **Bidirectional** | Modules can expose both sensor inputs (read) and actuator outputs (write) |
| **Hot-swap friendly** | Online status is tracked per module; offline detection built in |
| **Open specification** | Anyone can build a compatible module using the published slave driver |

### Why this matters

The I2C bus turns the Sensor Backplate into a **hub**, not just a controller. Instead of designing one device that does everything, you design one backplate that handles the essentials and expand it with modules purpose-built for your application:

- Need CO₂ measurement? Plug in a CO₂ module.
- Need 4-channel relay output? Plug in a relay module.
- Need a PWM motor driver? Plug in a PWM module.

Each module's registers appear automatically in the web UI as sensors or actuators — readable values feed directly into the logic engine, and writable registers become actuator targets. The full power of the expression engine (`HYST`, `PID`, `RAMP`, etc.) is immediately available against any module value.

### Designing a compatible module

Any microcontroller with I2C slave capability can become a Grefur module. The **Grefur Slave Driver** library handles the entire protocol:

```cpp
GrefurSlaveModule mod(0x1234);        // device ID

float temperature = 0.0f;
uint8_t fanSpeed  = 0;

mod.addRegister(0x0010, GREFUR_T_F32, false, "temperature", &temperature);
mod.addRegister(0x0020, GREFUR_T_U8,  true,  "fanSpeed",    &fanSpeed);

mod.begin(0x10);                      // I2C address on the bus
```

That's it. The backplate will discover both registers, expose `temperature` as a readable sensor, and `fanSpeed` as a writable actuator — all without any additional configuration on the host side.

---

## Logic Engine – Programming on the Device

The Sensor Backplate includes a built-in expression engine that lets you define automation rules directly on the controller. No external PLC or scripting environment needed.

**Supported expression types**
- `CALC_EXPRESSION` – Event-based logic, triggers on value change
- `CALC_CONTINUOUS` – Time-based logic, evaluates at a fixed interval
- `CALC_EXTERNAL` – Binds to an incoming MQTT value with optional timeout/fallback

**Built-in functions**

| Function | Description |
|---|---|
| `HYST(val, high, low)` | Schmitt-trigger hysteresis to prevent output flapping |
| `DELAY(input, time, mode, unit)` | Delayed on/off with ON, OFF, or symmetric modes |
| `RAMP(target, rateUp, rateDown)` | Smooth ramp toward a target value |
| `CYCLE(on, off)` | Asymmetric timer toggling between 1 and 0 |
| `PID(input, setpoint, Kp, Ki, Kd)` | PID controller |
| `CURVE(val, x1, y1, x2, y2, ...)` | Linear interpolation lookup table |
| `AVG(a, b, ...)` | Arithmetic average |
| `LIMIT(val, min, max)` | Clamp a value to a range |
| `ROUND(val, decimals)` | Round to N decimal places |
| `ABS(val)` | Absolute value |
| `SQRT(val)` | Square root |

**Supported operators:** `+` `-` `*` `/` `==` `!=` `>=` `<=` `>` `<` `&&` `||` `!` and ternary `? :`

**Example expressions**
```
HYST(RoomTemp, 24, 22)
DELAY(Button, 30, OFF, s) ? 1 : 0
RAMP(TargetSpeed, 10, 5)
PID(FlowSensor, 50, 1.2, 0.3, 0.05)
```

**Documentation**
https://docs.google.com/document/d/1KdkB8BtieTk5un02kl3CuF5Xg3J6uxHhwuBDASOmFMo/edit?usp=sharing


---

## Publish Policies

Each sensor, calculation, and output can be configured with an independent publish policy:

- **On change** – Publish only when the value changes beyond a threshold
- **On interval** – Publish at a fixed time interval
- **On change or interval** – Publish on either condition

Hardware outputs are always driven immediately on value change, independent of publish policy.

---

## MQTT Integration

All values are published and received through an MQTT broker, enabling integration with:

- Home automation systems (Home Assistant, OpenHAB, etc.)
- Industrial control systems
- IoT dashboards and data lakes

The Sensor Backplate supports both flat and nested JSON payloads, configurable per device.

---

## Web Configuration UI

The Sensor Backplate hosts a web interface directly on the device. From a browser you can:

- Add, edit, and remove sensors and outputs
- Write and validate logic expressions
- Configure MQTT broker settings and publish policies
- Monitor live values from all inputs, calculations, and outputs

---

## General Specifications

### MUX Support (ESP8266)
- **4 analog channels** via multiplexer IC
- **Control pins:** A (GPIO2), B (GPIO14)
- Only active if enabled in configuration

### Supported Platforms
- ESP8266
- ESP32

---

## Open Source

The Grefur Sensor Backplate is fully open source:

- Hardware specifications
- Firmware
- Configuration tools and APIs

Join the community to build your own modules or contribute to the ecosystem.

---

## License

This project is licensed under the **MIT License**. You are free to use, modify, and distribute – just give credit.

---

Made with ❤️ by [Grefur AS](https://grefur.com)
