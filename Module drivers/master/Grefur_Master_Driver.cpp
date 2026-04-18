#include "Grefur_Master_Driver.h"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

GrefurModuleMaster::GrefurModuleMaster()
    : _deviceCount(0),
      _changeCallback(nullptr),
      _scanState(GREFUR_SCAN_IDLE),
      _scanAddr(0x08),
      _scanRegIdx(0),
      _scanWaitingRegs(false)
{}

// ─────────────────────────────────────────────────────────────────────────────
// Begin
// ─────────────────────────────────────────────────────────────────────────────

void GrefurModuleMaster::begin(int sda, int scl, uint32_t frequency) {
    Wire.begin(sda, scl);
    Wire.setClock(frequency);
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal Helpers
// ─────────────────────────────────────────────────────────────────────────────

uint8_t GrefurModuleMaster::_getTypeSize(uint8_t type) {
    switch (type) {
        case GREFUR_T_U8:  return 1;
        case GREFUR_T_I16:
        case GREFUR_T_U16: return 2;
        case GREFUR_T_I32:
        case GREFUR_T_F32: return 4;
        default:           return 0;
    }
}

// Store raw bytes into a GrefurValue union with correct type interpretation
void GrefurModuleMaster::_storeValue(GrefurMasterReg& reg, uint8_t* buf, uint8_t size) {
    reg.prevValue = reg.lastValue;

    uint32_t raw = 0;
    for (uint8_t i = 0; i < size; i++) raw = (raw << 8) | buf[i];

    switch (reg.dataType) {
        case GREFUR_T_U8:  reg.lastValue.asU8    = (uint8_t)raw;  break;
        case GREFUR_T_U16: reg.lastValue.asU16   = (uint16_t)raw; break;
        case GREFUR_T_I16: reg.lastValue.asI16   = (int16_t)raw;  break;
        case GREFUR_T_I32: reg.lastValue.asI32   = (int32_t)raw;  break;
        case GREFUR_T_F32:
            // Reinterpret raw bits as IEEE 754 float
            memcpy(&reg.lastValue.asFloat, &raw, 4);
            break;
    }

    // Detect change (compare raw bits)
    reg.changed = (memcmp(&reg.lastValue, &reg.prevValue, sizeof(GrefurValue)) != 0);
}

// Read bytes from a device register into a buffer — shared by poll and other reads
bool GrefurModuleMaster::_readRegisterBytes(uint8_t i2cAddr, uint8_t regAddr, uint8_t size, uint8_t* outBuf) {
    Wire.beginTransmission(i2cAddr);
    Wire.write(regAddr);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(i2cAddr, size) != size) return false;
    for (uint8_t i = 0; i < size; i++) outBuf[i] = Wire.read();
    return true;
}

// Write a GrefurValue to a device register
bool GrefurModuleMaster::_writeRegisterBytes(uint8_t i2cAddr, uint8_t regAddr, uint8_t type, GrefurValue value) {
    Wire.beginTransmission(i2cAddr);
    Wire.write(regAddr);

    uint8_t size = _getTypeSize(type);
    uint32_t raw = 0;

    if (type == GREFUR_T_F32) {
        memcpy(&raw, &value.asFloat, 4);
    } else {
        raw = value.asU32;
    }

    for (int8_t i = (size - 1); i >= 0; i--) {
        Wire.write((raw >> (i * 8)) & 0xFF);
    }

    return (Wire.endTransmission() == 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// Discovery — Blocking
// ─────────────────────────────────────────────────────────────────────────────

uint8_t GrefurModuleMaster::scanAndDiscover() {
    _deviceCount = 0;

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() != 0) continue;
        if (_deviceCount >= GREFUR_MAX_DEVICES) break;

        GrefurDevice& dev = _devices[_deviceCount];
        dev.i2cAddress = addr;
        dev.regCount   = 0;
        dev.active     = false;
        dev.online     = false;

        // Request metadata: index 0 to get total register count
        Wire.beginTransmission(addr);
        Wire.write(0x00);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) continue;

        // Read exactly 1 byte: totalRegs
        // (previously read 6, discarding 5 — fixed here)
        if (Wire.requestFrom(addr, (uint8_t)1) < 1) continue;
        uint8_t totalRegs = Wire.read();
        if (totalRegs > GREFUR_MAX_REGISTERS) totalRegs = GREFUR_MAX_REGISTERS;

        for (uint8_t i = 0; i < totalRegs; i++) {
            Wire.beginTransmission(addr);
            Wire.write(0x00);
            Wire.write(i);
            Wire.endTransmission();

            // Expect: [totalRegs(1), regAddr(1), dataType(1), writable(1), nameLen(1), name(up to 15)]
            if (Wire.requestFrom(addr, (uint8_t)20) < 5) continue;

            Wire.read(); // Discard totalRegs echo
            GrefurMasterReg& r = dev.registers[dev.regCount];
            r.address  = Wire.read();
            r.dataType = Wire.read();
            r.writable = (Wire.read() == 1);
            uint8_t nameLen = Wire.read();
            if (nameLen >= GREFUR_NAME_LEN) nameLen = GREFUR_NAME_LEN - 1;

            memset(r.name, 0, GREFUR_NAME_LEN);
            for (uint8_t c = 0; c < nameLen; c++) {
                if (Wire.available()) r.name[c] = Wire.read();
            }
            // Drain any remaining bytes to avoid buffer pollution
            while (Wire.available()) Wire.read();

            memset(&r.lastValue, 0, sizeof(GrefurValue));
            memset(&r.prevValue, 0, sizeof(GrefurValue));
            r.valid   = true;
            r.changed = false;
            dev.regCount++;
        }

        dev.active = true;
        dev.online = true;
        _deviceCount++;
    }

    _scanState = GREFUR_SCAN_DONE;
    return _deviceCount;
}

// ─────────────────────────────────────────────────────────────────────────────
// Discovery — Non-blocking (call in loop())
// ─────────────────────────────────────────────────────────────────────────────

bool GrefurModuleMaster::scanAndDiscoverAsync() {
    if (_scanState == GREFUR_SCAN_DONE) return true;
    _scanState = GREFUR_SCAN_RUNNING;

    // Process one address per call to avoid blocking
    if (_scanAddr >= 0x78) {
        _scanState = GREFUR_SCAN_DONE;
        return true;
    }

    Wire.beginTransmission(_scanAddr);
    if (Wire.endTransmission() == 0 && _deviceCount < GREFUR_MAX_DEVICES) {
        GrefurDevice& dev = _devices[_deviceCount];
        dev.i2cAddress = _scanAddr;
        dev.regCount   = 0;
        dev.active     = false;
        dev.online     = false;

        Wire.beginTransmission(_scanAddr);
        Wire.write(0x00);
        Wire.write(0x00);
        if (Wire.endTransmission() == 0 && Wire.requestFrom(_scanAddr, (uint8_t)1) >= 1) {
            uint8_t totalRegs = Wire.read();
            if (totalRegs > GREFUR_MAX_REGISTERS) totalRegs = GREFUR_MAX_REGISTERS;

            for (uint8_t i = 0; i < totalRegs; i++) {
                Wire.beginTransmission(_scanAddr);
                Wire.write(0x00);
                Wire.write(i);
                Wire.endTransmission();

                if (Wire.requestFrom(_scanAddr, (uint8_t)20) < 5) continue;

                Wire.read();
                GrefurMasterReg& r = dev.registers[dev.regCount];
                r.address  = Wire.read();
                r.dataType = Wire.read();
                r.writable = (Wire.read() == 1);
                uint8_t nameLen = Wire.read();
                if (nameLen >= GREFUR_NAME_LEN) nameLen = GREFUR_NAME_LEN - 1;

                memset(r.name, 0, GREFUR_NAME_LEN);
                for (uint8_t c = 0; c < nameLen; c++) {
                    if (Wire.available()) r.name[c] = Wire.read();
                }
                while (Wire.available()) Wire.read();

                memset(&r.lastValue, 0, sizeof(GrefurValue));
                memset(&r.prevValue, 0, sizeof(GrefurValue));
                r.valid   = true;
                r.changed = false;
                dev.regCount++;
            }

            dev.active = true;
            dev.online = true;
            _deviceCount++;
        }
    }

    _scanAddr++;
    return (_scanAddr >= 0x78);
}

void GrefurModuleMaster::resetScan() {
    _deviceCount      = 0;
    _scanAddr         = 0x08;
    _scanRegIdx       = 0;
    _scanWaitingRegs  = false;
    _scanState        = GREFUR_SCAN_IDLE;
    memset(_devices, 0, sizeof(_devices));
}

// ─────────────────────────────────────────────────────────────────────────────
// Online Check
// ─────────────────────────────────────────────────────────────────────────────

bool GrefurModuleMaster::isModuleOnline(uint8_t devIdx) {
    if (devIdx >= _deviceCount) return false;
    Wire.beginTransmission(_devices[devIdx].i2cAddress);
    bool online = (Wire.endTransmission() == 0);
    _devices[devIdx].online = online;
    return online;
}

void GrefurModuleMaster::checkAllModulesOnline() {
    for (uint8_t i = 0; i < _deviceCount; i++) {
        isModuleOnline(i);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Polling
// ─────────────────────────────────────────────────────────────────────────────

bool GrefurModuleMaster::pollRegister(uint8_t devIdx, uint8_t regIdx) {
    if (devIdx >= _deviceCount) return false;
    GrefurDevice& dev = _devices[devIdx];
    if (!dev.active || !dev.online) return false;
    if (regIdx >= dev.regCount) return false;

    GrefurMasterReg& reg = dev.registers[regIdx];
    uint8_t size = _getTypeSize(reg.dataType);
    if (size == 0) return false;

    uint8_t buf[4] = {0};
    for (int retry = 0; retry < GREFUR_I2C_RETRIES; retry++) {
        if (_readRegisterBytes(dev.i2cAddress, reg.address, size, buf)) {
            _storeValue(reg, buf, size);

            // Fire callback if value changed
            if (reg.changed && _changeCallback != nullptr) {
                _changeCallback(devIdx, regIdx, reg.lastValue);
            }
            return true;
        }
    }

    // Mark device offline after exhausting retries
    dev.online = false;
    return false;
}

bool GrefurModuleMaster::pollDevice(uint8_t devIdx) {
    if (devIdx >= _deviceCount) return false;  // Fixed: was missing bounds check
    bool success = true;
    for (uint8_t r = 0; r < _devices[devIdx].regCount; r++) {
        if (!pollRegister(devIdx, r)) success = false;
    }
    return success;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write
// ─────────────────────────────────────────────────────────────────────────────

// Write by device index — consistent with poll API
bool GrefurModuleMaster::writeToModule(uint8_t devIdx, uint8_t regIdx, GrefurValue value) {
    if (devIdx >= _deviceCount) return false;
    GrefurDevice& dev = _devices[devIdx];
    if (!dev.active || regIdx >= dev.regCount) return false;
    GrefurMasterReg& reg = dev.registers[regIdx];
    if (!reg.writable) return false;
    return _writeRegisterBytes(dev.i2cAddress, reg.address, reg.dataType, value);
}

// Write raw — by I2C address directly (advanced/escape-hatch use)
bool GrefurModuleMaster::writeRaw(uint8_t devAddr, uint8_t regAddr, uint8_t type, GrefurValue value) {
    return _writeRegisterBytes(devAddr, regAddr, type, value);
}

// ─────────────────────────────────────────────────────────────────────────────
// Register Lookup by Name
// ─────────────────────────────────────────────────────────────────────────────

int8_t GrefurModuleMaster::findRegisterByName(uint8_t devIdx, const char* name) {
    if (devIdx >= _deviceCount) return -1;
    GrefurDevice& dev = _devices[devIdx];
    for (uint8_t i = 0; i < dev.regCount; i++) {
        if (strncmp(dev.registers[i].name, name, GREFUR_NAME_LEN) == 0) return (int8_t)i;
    }
    return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
// Change Callback
// ─────────────────────────────────────────────────────────────────────────────

void GrefurModuleMaster::onValueChanged(GrefurChangeCallback callback) {
    _changeCallback = callback;
}

// ─────────────────────────────────────────────────────────────────────────────
// Typed Value Getters
// ─────────────────────────────────────────────────────────────────────────────

float GrefurModuleMaster::getFloat(uint8_t devIdx, uint8_t regIdx) {
    if (devIdx >= _deviceCount || regIdx >= _devices[devIdx].regCount) return 0.0f;
    return _devices[devIdx].registers[regIdx].lastValue.asFloat;
}

int32_t GrefurModuleMaster::getInt(uint8_t devIdx, uint8_t regIdx) {
    if (devIdx >= _deviceCount || regIdx >= _devices[devIdx].regCount) return 0;
    return _devices[devIdx].registers[regIdx].lastValue.asI32;
}

uint32_t GrefurModuleMaster::getUInt(uint8_t devIdx, uint8_t regIdx) {
    if (devIdx >= _deviceCount || regIdx >= _devices[devIdx].regCount) return 0;
    return _devices[devIdx].registers[regIdx].lastValue.asU32;
}
