# Grefur – Open Hardware IoT Controller Platform

This is the main repository for the **Grefur** project – an open-source, modular IoT controller platform designed for flexibility, extensibility, and seamless integration into MQTT-based ecosystems. Whether you're building environmental monitors, automation controllers, or custom input/output devices – **Grefur** is your universal starting point.

---

## What is Grefur Sensor?

Grefur is more than a sensor platform. It is a **programmable IoT controller** with:

- **Physical I/O** – Read digital and analog inputs, drive digital and analog outputs
- **Built-in logic engine** – Write expressions directly on the device to create automation rules without external tools
- **MQTT integration** – Publish sensor values, receive external signals, and bind remote values into local logic
- **Web UI** – Configure sensors, outputs, and logic from a browser with no additional software

> Developed and maintained by **Grefur AS**. The platform specification is open and public. Anyone can design compatible modules.

<img width="1298" height="766" alt="image" src="https://github.com/user-attachments/assets/4c39ff73-0cbe-4872-82f3-1b469f8e5125" />


---

## Backplate – The Heart of the System

The standardized **Grefur Backplate** is designed for universal mounting on walls, enclosures, HVAC systems, outdoor installations, and more:

- **Modular Expansion** – Any sensor or I/O module can be designed for easy integration
- **Technology Agnostic** – Compatible with ESP8266, ESP32, and more
- **IoT-First** – Simplified publishing of measured and calculated values into any MQTT ecosystem

---

## I/O Capabilities

Grefur supports direct connection of physical hardware:

**Inputs**
- NTC temperature sensors
- Analog sensors (voltage, pressure, current, etc.)
- Digital inputs (buttons, switches, flow sensors, Hall-effect sensors)

**Outputs**
- Digital outputs (relays, solenoids, indicators)
- Analog/PWM outputs (variable speed drives, control valves, dimmers)

---

## Logic Engine – Programming on the Device

Grefur includes a built-in expression engine that lets you define automation rules directly on the controller. No external PLC or scripting environment needed.

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

Grefur supports both flat and nested JSON payloads, configurable per device.

---

## Web Configuration UI

Grefur hosts a web interface directly on the device. From a browser you can:

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

The Grefur platform is fully open source:

- Hardware specifications
- Firmware
- Configuration tools and APIs

Join the community to build your own modules or contribute to the ecosystem.

---

## License

This project is licensed under the **MIT License**. You are free to use, modify, and distribute – just give credit.

---

Made with ❤️ by [Grefur AS](https://grefur.com)
