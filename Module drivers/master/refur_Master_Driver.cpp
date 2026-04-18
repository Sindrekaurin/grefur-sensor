#include "Grefur_Master_Driver.h"

GrefurMaster::GrefurMaster() : _deviceCount(0) {}

void GrefurMaster::begin(int sda, int scl, uint32_t frequency) {
    Wire.begin(sda, scl);
    Wire.setClock(frequency);
}

uint8_t GrefurMaster::_getTypeSize(uint8_t type) {
    if (type == GREFUR_T_U8) return 1;
    if (type == GREFUR_T_I16 || type == GREFUR_T_U16) return 2;
    if (type == GREFUR_T_I32) return 4;
    return 0;
}

uint8_t GrefurMaster::scanAndDiscover() {
    _deviceCount = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            if (_deviceCount >= GREFUR_MAX_DEVICES) break;

            GrefurDevice& dev = _devices[_deviceCount];
            dev.i2cAddress = addr;
            dev.regCount = 0;

            // 1. Ask for Register Count
            Wire.beginTransmission(addr);
            Wire.write(0x00); // Metadata Discovery Mode
            Wire.write(0x00); // Index 0
            if (Wire.endTransmission() != 0) continue;

            if (Wire.requestFrom(addr, (uint8_t)6) >= 5) {
                uint8_t totalRegs = Wire.read();
                if (totalRegs > GREFUR_MAX_REGISTERS) totalRegs = GREFUR_MAX_REGISTERS;

                // 2. Discover each register
                for (uint8_t i = 0; i < totalRegs; i++) {
                    Wire.beginTransmission(addr);
                    Wire.write(0x00);
                    Wire.write(i);
                    Wire.endTransmission();

                    if (Wire.requestFrom(addr, (uint8_t)24) > 5) {
                        Wire.read(); // Discard totalCount
                        GrefurMasterReg& r = dev.registers[dev.regCount];
                        r.address = Wire.read();
                        r.dataType = Wire.read();
                        r.writable = (Wire.read() == 1);
                        uint8_t nameLen = Wire.read();

                        memset(r.name, 0, GREFUR_NAME_LEN);
                        for (uint8_t c = 0; c < nameLen; c++) {
                            char ch = Wire.read();
                            if (c < GREFUR_NAME_LEN - 1) r.name[c] = ch;
                        }
                        r.valid = true;
                        dev.regCount++;
                    }
                }
                dev.active = true;
                _deviceCount++;
            }
        }
    }
    return _deviceCount;
}

bool GrefurMaster::pollRegister(uint8_t devIdx, uint8_t regIdx) {
    if (devIdx >= _deviceCount) return false;
    GrefurDevice& dev = _devices[devIdx];
    if (regIdx >= dev.regCount) return false;
    GrefurMasterReg& reg = dev.registers[regIdx];

    for (int retry = 0; retry < GREFUR_I2C_RETRIES; retry++) {
        Wire.beginTransmission(dev.i2cAddress);
        Wire.write(reg.address);
        if (Wire.endTransmission(false) != 0) continue;

        uint8_t size = _getTypeSize(reg.dataType);
        if (Wire.requestFrom(dev.i2cAddress, size) == size) {
            if (size == 1) reg.lastValue = Wire.read();
            else if (size == 2) reg.lastValue = (Wire.read() << 8) | Wire.read();
            else if (size == 4) {
                reg.lastValue = ((uint32_t)Wire.read() << 24) | ((uint32_t)Wire.read() << 16) |
                               ((uint32_t)Wire.read() << 8)  | Wire.read();
            }
            return true;
        }
    }
    return false;
}

bool GrefurMaster::pollDevice(uint8_t devIdx) {
    bool success = true;
    for (uint8_t r = 0; r < _devices[devIdx].regCount; r++) {
        if (!pollRegister(devIdx, r)) success = false;
    }
    return success;
}

bool GrefurMaster::writeValue(uint8_t devAddr, uint8_t regAddr, uint8_t type, uint32_t value) {
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    uint8_t size = _getTypeSize(type);
    if (size == 1) Wire.write((uint8_t)value);
    else if (size == 2) { Wire.write(value >> 8); Wire.write(value & 0xFF); }
    else if (size == 4) {
        Wire.write(value >> 24); Wire.write(value >> 16);
        Wire.write(value >> 8);  Wire.write(value & 0xFF);
    }
    return (Wire.endTransmission() == 0);
}
