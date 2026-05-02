#include "Grefur_Master_Driver.h"

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

GrefurModuleMaster::GrefurModuleMaster()
    : _deviceCount(0),
      _changeCallback(nullptr),
      _writeCallback(nullptr),
      _readCallback(nullptr),
      _errorCallback(nullptr),
      _discoveryCallback(nullptr),
      _activityCallback(nullptr),
      _offlineCallback(nullptr),
      _onlineCallback(nullptr),
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
// Callback Setters
// ─────────────────────────────────────────────────────────────────────────────

void GrefurModuleMaster::onValueChanged(GrefurChangeCallback callback) {
    _changeCallback = callback;
}

void GrefurModuleMaster::onWrite(GrefurMasterWriteCallback callback) {
    _writeCallback = callback;
}

void GrefurModuleMaster::onRead(GrefurMasterReadCallback callback) {
    _readCallback = callback;
}

void GrefurModuleMaster::onError(GrefurMasterErrorCallback callback) {
    _errorCallback = callback;
}

void GrefurModuleMaster::onDiscovery(GrefurDeviceCallback callback) {
    _discoveryCallback = callback;
}

void GrefurModuleMaster::onActivity(GrefurDeviceCallback callback) {
    _activityCallback = callback;
}

void GrefurModuleMaster::onDeviceOffline(GrefurDeviceCallback callback) {
    _offlineCallback = callback;
}

void GrefurModuleMaster::onDeviceOnline(GrefurDeviceCallback callback) {
    _onlineCallback = callback;
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

void GrefurModuleMaster::_storeValue(GrefurMasterReg& reg, uint8_t* buf, uint8_t size) {
    reg.prevValue = reg.lastValue;

    uint32_t raw = 0;
    for (uint8_t i = 0; i < size; i++) raw = (raw << 8) | buf[i];

    switch (reg.dataType) {
        case GREFUR_T_U8:  reg.lastValue.asU8  = (uint8_t)raw;  break;
        case GREFUR_T_U16: reg.lastValue.asU16 = (uint16_t)raw; break;
        case GREFUR_T_I16: reg.lastValue.asI16 = (int16_t)raw;  break;
        case GREFUR_T_I32: reg.lastValue.asI32 = (int32_t)raw;  break;
        case GREFUR_T_F32: memcpy(&reg.lastValue.asFloat, &raw, 4); break;
    }

    reg.changed = (memcmp(&reg.lastValue, &reg.prevValue, sizeof(GrefurValue)) != 0);
}

bool GrefurModuleMaster::_readRegisterBytes(uint8_t i2cAddr, uint16_t regAddr, uint8_t size, uint8_t* outBuf) {
    Wire.beginTransmission(i2cAddr);
    Wire.write((regAddr >> 8) & 0xFF);
    Wire.write(regAddr & 0xFF);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(i2cAddr, size) != size) return false;
    for (uint8_t i = 0; i < size; i++) outBuf[i] = Wire.read();
    return true;
}

bool GrefurModuleMaster::_writeRegisterBytes(uint8_t i2cAddr, uint16_t regAddr, uint8_t type, GrefurValue value) {
    Wire.beginTransmission(i2cAddr);
    Wire.write((regAddr >> 8) & 0xFF);
    Wire.write(regAddr & 0xFF);

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
// Internal: ping a device and restore online flag if reachable
// ─────────────────────────────────────────────────────────────────────────────

bool GrefurModuleMaster::_tryReconnect(uint8_t devIdx) {
    GrefurDevice& dev = _devices[devIdx];
    if (dev.online) return true;

    Wire.beginTransmission(dev.i2cAddress);
    if (Wire.endTransmission() == 0) {
        dev.online = true;
        if (_onlineCallback) _onlineCallback(devIdx);
        return true;
    }
    return false;
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

        Wire.beginTransmission(addr);
        Wire.write((uint8_t)0x00);
        Wire.write((uint8_t)0x00);
        Wire.write((uint8_t)0x00);
        if (Wire.endTransmission() != 0) continue;

        if (Wire.requestFrom(addr, (uint8_t)1) < 1) continue;
        uint8_t totalRegs = Wire.read();
        if (totalRegs > GREFUR_MAX_REGISTERS) totalRegs = GREFUR_MAX_REGISTERS;

        for (uint8_t i = 0; i < totalRegs; i++) {
            Wire.beginTransmission(addr);
            Wire.write((uint8_t)0x00);
            Wire.write((uint8_t)0x00);
            Wire.write(i);
            Wire.endTransmission();

            if (Wire.requestFrom(addr, (uint8_t)21) < 6) continue;

            Wire.read();

            GrefurMasterReg& r = dev.registers[dev.regCount];
            uint8_t addrHi = Wire.read();
            uint8_t addrLo = Wire.read();
            r.address  = ((uint16_t)addrHi << 8) | addrLo;
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
        if (_discoveryCallback) _discoveryCallback(_deviceCount);
        if (_activityCallback)  _activityCallback(_deviceCount);
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
        Wire.write((uint8_t)0x00);
        Wire.write((uint8_t)0x00);
        Wire.write((uint8_t)0x00);
        if (Wire.endTransmission() == 0 && Wire.requestFrom(_scanAddr, (uint8_t)1) >= 1) {
            uint8_t totalRegs = Wire.read();
            if (totalRegs > GREFUR_MAX_REGISTERS) totalRegs = GREFUR_MAX_REGISTERS;

            for (uint8_t i = 0; i < totalRegs; i++) {
                Wire.beginTransmission(_scanAddr);
                Wire.write((uint8_t)0x00);
                Wire.write((uint8_t)0x00);
                Wire.write(i);
                Wire.endTransmission();

                if (Wire.requestFrom(_scanAddr, (uint8_t)21) < 6) continue;

                Wire.read();

                GrefurMasterReg& r = dev.registers[dev.regCount];
                uint8_t addrHi = Wire.read();
                uint8_t addrLo = Wire.read();
                r.address  = ((uint16_t)addrHi << 8) | addrLo;
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
            if (_discoveryCallback) _discoveryCallback(_deviceCount);
            if (_activityCallback)  _activityCallback(_deviceCount);
            _deviceCount++;
        }
    }

    _scanAddr++;
    return (_scanAddr >= 0x78);
}

void GrefurModuleMaster::resetScan() {
    _deviceCount     = 0;
    _scanAddr        = 0x08;
    _scanRegIdx      = 0;
    _scanWaitingRegs = false;
    _scanState       = GREFUR_SCAN_IDLE;
    memset(_devices, 0, sizeof(_devices));
}

// ─────────────────────────────────────────────────────────────────────────────
// Online Check
// ─────────────────────────────────────────────────────────────────────────────

bool GrefurModuleMaster::isModuleOnline(uint8_t devIdx) {
    if (devIdx >= _deviceCount) {
        _notifyError(GREFUR_ERR_BAD_DEVICE_IDX, devIdx);
        return false;
    }
    GrefurDevice& dev = _devices[devIdx];
    bool wasOnline = dev.online;

    Wire.beginTransmission(dev.i2cAddress);
    dev.online = (Wire.endTransmission() == 0);

    if (!wasOnline && dev.online) {
        if (_onlineCallback) _onlineCallback(devIdx);
    } else if (wasOnline && !dev.online) {
        if (_offlineCallback) _offlineCallback(devIdx);
    }

    return dev.online;
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
    if (devIdx >= _deviceCount) {
        _notifyError(GREFUR_ERR_BAD_DEVICE_IDX, devIdx);
        return false;
    }
    GrefurDevice& dev = _devices[devIdx];
    if (!dev.active) return false;

    if (!dev.online && !_tryReconnect(devIdx)) {
        _notifyError(GREFUR_ERR_DEVICE_OFFLINE, devIdx);
        return false;
    }

    if (regIdx >= dev.regCount) {
        _notifyError(GREFUR_ERR_BAD_REGISTER_IDX, devIdx);
        return false;
    }

    GrefurMasterReg& reg = dev.registers[regIdx];
    uint8_t size = _getTypeSize(reg.dataType);
    if (size == 0) return false;

    uint8_t buf[4] = {0};
    for (int retry = 0; retry < GREFUR_I2C_RETRIES; retry++) {
        if (_readRegisterBytes(dev.i2cAddress, reg.address, size, buf)) {
            _storeValue(reg, buf, size);
            if (_activityCallback) _activityCallback(devIdx);
            if (_readCallback)     _readCallback(devIdx, regIdx, reg.lastValue);
            if (reg.changed && _changeCallback) {
                _changeCallback(devIdx, regIdx, reg.lastValue);
            }
            return true;
        }
    }

    // All retries exhausted — mark offline
    bool wasOnline = dev.online;
    dev.online = false;
    if (wasOnline && _offlineCallback) _offlineCallback(devIdx);
    _notifyError(GREFUR_ERR_READ_FAILED, devIdx);
    return false;
}

bool GrefurModuleMaster::pollDevice(uint8_t devIdx) {
    if (devIdx >= _deviceCount) {
        _notifyError(GREFUR_ERR_BAD_DEVICE_IDX, devIdx);
        return false;
    }
    GrefurDevice& dev = _devices[devIdx];
    if (!dev.active) return false;

    if (!dev.online && !_tryReconnect(devIdx)) {
        _notifyError(GREFUR_ERR_DEVICE_OFFLINE, devIdx);
        return false;
    }

    bool success = true;
    for (uint8_t r = 0; r < dev.regCount; r++) {
        if (!pollRegister(devIdx, r)) success = false;
    }
    return success;
}

bool GrefurModuleMaster::pollAddress(uint8_t i2cAddr, uint16_t regAddr) {
    for (uint8_t i = 0; i < _deviceCount; i++) {
        if (_devices[i].i2cAddress != i2cAddr) continue;

        if (!_devices[i].online && !_tryReconnect(i)) {
            _notifyError(GREFUR_ERR_DEVICE_OFFLINE, i);
            return false;
        }

        for (uint8_t r = 0; r < _devices[i].regCount; r++) {
            if (_devices[i].registers[r].address == regAddr) {
                return pollRegister(i, r);
            }
        }
    }
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write
// ─────────────────────────────────────────────────────────────────────────────

bool GrefurModuleMaster::writeToModule(uint8_t devIdx, uint8_t regIdx, GrefurValue value) {
    if (devIdx >= _deviceCount) {
        _notifyError(GREFUR_ERR_BAD_DEVICE_IDX, devIdx);
        return false;
    }
    GrefurDevice& dev = _devices[devIdx];
    if (!dev.active) return false;

    if (!dev.online && !_tryReconnect(devIdx)) {
        _notifyError(GREFUR_ERR_DEVICE_OFFLINE, devIdx);
        return false;
    }

    if (regIdx >= dev.regCount) {
        _notifyError(GREFUR_ERR_BAD_REGISTER_IDX, devIdx);
        return false;
    }

    GrefurMasterReg& reg = dev.registers[regIdx];
    if (!reg.writable) {
        _notifyError(GREFUR_ERR_NOT_WRITABLE, devIdx);
        return false;
    }

    if (!_writeRegisterBytes(dev.i2cAddress, reg.address, reg.dataType, value)) {
        _notifyError(GREFUR_ERR_WRITE_FAILED, devIdx);
        return false;
    }

    if (_activityCallback) _activityCallback(devIdx);
    if (_writeCallback)    _writeCallback(devIdx, regIdx, value);
    return true;
}

bool GrefurModuleMaster::writeRaw(uint8_t devAddr, uint16_t regAddr, uint8_t type, GrefurValue value) {
    if (!_writeRegisterBytes(devAddr, regAddr, type, value)) {
        _notifyError(GREFUR_ERR_WRITE_FAILED, 0xFF);  // 0xFF = ukjent devIdx
        return false;
    }
    if (_activityCallback) _activityCallback(0xFF);
    return true;
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
// Typed Value Getters (by device/register index)
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

// ─────────────────────────────────────────────────────────────────────────────
// Value Getters by Address
// ─────────────────────────────────────────────────────────────────────────────

float GrefurModuleMaster::getValueAsFloat(uint8_t i2cAddr, uint16_t regAddr) {
    for (uint8_t i = 0; i < _deviceCount; i++) {
        if (_devices[i].i2cAddress == i2cAddr) {
            for (uint8_t r = 0; r < _devices[i].regCount; r++) {
                GrefurMasterReg& reg = _devices[i].registers[r];
                if (reg.address == regAddr) {
                    switch (reg.dataType) {
                        case GREFUR_T_U8:  return (float)reg.lastValue.asU8;
                        case GREFUR_T_I16: return (float)reg.lastValue.asI16;
                        case GREFUR_T_U16: return (float)reg.lastValue.asU16;
                        case GREFUR_T_I32: return (float)reg.lastValue.asI32;
                        case GREFUR_T_F32: return reg.lastValue.asFloat;
                        default:           return 0.0f;
                    }
                }
            }
        }
    }
    return 0.0f;
}

int32_t GrefurModuleMaster::getIntByAddress(uint8_t i2cAddr, uint16_t regAddr) {
    for (uint8_t i = 0; i < _deviceCount; i++) {
        if (_devices[i].i2cAddress == i2cAddr) {
            for (uint8_t r = 0; r < _devices[i].regCount; r++) {
                if (_devices[i].registers[r].address == regAddr)
                    return _devices[i].registers[r].lastValue.asI32;
            }
        }
    }
    return 0;
}

float GrefurModuleMaster::getFloatByAddress(uint8_t i2cAddr, uint16_t regAddr) {
    for (uint8_t i = 0; i < _deviceCount; i++) {
        if (_devices[i].i2cAddress == i2cAddr) {
            for (uint8_t r = 0; r < _devices[i].regCount; r++) {
                if (_devices[i].registers[r].address == regAddr)
                    return _devices[i].registers[r].lastValue.asFloat;
            }
        }
    }
    return 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Metadata Getters
// ─────────────────────────────────────────────────────────────────────────────

uint8_t GrefurModuleMaster::getDeviceAddress(uint8_t devIdx) const {
    if (devIdx >= _deviceCount) return 0;
    return _devices[devIdx].i2cAddress;
}

uint8_t GrefurModuleMaster::getRegisterCount(uint8_t devIdx) const {
    if (devIdx >= _deviceCount) return 0;
    return _devices[devIdx].regCount;
}

const char* GrefurModuleMaster::getRegisterName(uint8_t devIdx, uint8_t regIdx) const {
    if (devIdx >= _deviceCount || regIdx >= _devices[devIdx].regCount) return "";
    return _devices[devIdx].registers[regIdx].name;
}

uint16_t GrefurModuleMaster::getRegisterAddress(uint8_t devIdx, uint8_t regIdx) const {
    if (devIdx >= _deviceCount || regIdx >= _devices[devIdx].regCount) return 0;
    return _devices[devIdx].registers[regIdx].address;
}

uint8_t GrefurModuleMaster::getRegisterDataType(uint8_t devIdx, uint8_t regIdx) const {
    if (devIdx >= _deviceCount || regIdx >= _devices[devIdx].regCount) return 0;
    return _devices[devIdx].registers[regIdx].dataType;
}
