#include "Grefur_module_driver.h"

GrefurModule* GrefurModule::_instance = nullptr;

GrefurModule::GrefurModule(uint16_t deviceId) : _deviceId(deviceId) {
  _instance = this;
}

void GrefurModule::addRegister(uint8_t addr, uint8_t type, bool writable, const char* name, void* ptr) {
  if (_regCount < GREFUR_MAX_REGS) {
    _regs[_regCount++] = {addr, type, writable, name, ptr};
  }
}

void GrefurModule::begin(uint8_t i2cAddr) {
  Wire.begin(i2cAddr);
  Wire.onReceive(_handleReceive);
  Wire.onRequest(_handleRequest);
}

void GrefurModule::_handleReceive(int n) {
  if (n < 1 || !_instance) return;
  _instance->_selectedAddr = Wire.read();

  if (n > 1) {
    if (_instance->_selectedAddr == 0x00) {
      _instance->_discoveryIdx = Wire.read();
    } else {
      /* Search for writable register */
      for (int i = 0; i < _instance->_regCount; i++) {
        GrefurReg& r = _instance->_regs[i];
        if (r.addr == _instance->_selectedAddr && r.writable) {
          if (r.type == GREFUR_T_U8) {
            *(uint8_t*)r.dataPtr = Wire.read();
          } else if (r.type == GREFUR_T_U16 || r.type == GREFUR_T_I16) {
            if (Wire.available() >= 2) {
               uint16_t val = (Wire.read() << 8) | Wire.read();
               if (r.type == GREFUR_T_U16) *(uint16_t*)r.dataPtr = val;
               else *(int16_t*)r.dataPtr = (int16_t)val;
            }
          }
          // Note: Add I32 write if needed for production
        }
      }
    }
  }
  while (Wire.available()) Wire.read();
}

void GrefurModule::_handleRequest() {
  if (!_instance) return;

  if (_instance->_selectedAddr == 0x00) {
    /* Metadata Discovery */
    if (_instance->_discoveryIdx < _instance->_regCount) {
      GrefurReg& r = _instance->_regs[_instance->_discoveryIdx];
      uint8_t nLen = strlen(r.name);
      Wire.write(_instance->_regCount);
      Wire.write(r.addr);
      Wire.write(r.type);
      Wire.write((uint8_t)r.writable);
      Wire.write(nLen);
      Wire.write((const uint8_t*)r.name, nLen);
    } else {
      Wire.write(0x00);
    }
  } 
  else if (_instance->_selectedAddr == 0x01) {
    /* Global Device ID */
    Wire.write(_instance->_deviceId >> 8);
    Wire.write(_instance->_deviceId & 0xFF);
  }
  else {
    /* Data Polling */
    for (int i = 0; i < _instance->_regCount; i++) {
      GrefurReg& r = _instance->_regs[i];
      if (r.addr == _instance->_selectedAddr) {
        if (r.type == GREFUR_T_U8) {
          Wire.write(*(uint8_t*)r.dataPtr);
        } else if (r.type == GREFUR_T_U16 || r.type == GREFUR_T_I16) {
          uint16_t val = *(uint16_t*)r.dataPtr;
          Wire.write(val >> 8); Wire.write(val & 0xFF);
        } else if (r.type == GREFUR_T_I32) {
          int32_t val = *(int32_t*)r.dataPtr;
          Wire.write(val >> 24); Wire.write(val >> 16);
          Wire.write(val >> 8);  Wire.write(val & 0xFF);
        }
        return;
      }
    }
    Wire.write(0xFF); /* Unknown register */
  }
}
