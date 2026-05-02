#include "Grefur_Slave_Driver.h"

// ─────────────────────────────────────────────────────────────────────────────
// Static instance pointer (required for Wire callbacks)
// ─────────────────────────────────────────────────────────────────────────────
GrefurSlaveModule* GrefurSlaveModule::_instance = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
GrefurSlaveModule::GrefurSlaveModule(uint16_t deviceId)
    : _deviceId(deviceId),
      _regCount(0),
      _writeCallback(nullptr),
      _readCallback(nullptr),
      _errorCallback(nullptr),
      _discoveryCallback(nullptr),
      _activityCallback(nullptr),
      _selectedAddr16(0),
      _discoveryIdx(0),
      _pendingWrite(false),
      _pendingRegAddr(0),
      _pendingLen(0)
{
    _instance = this;
    memset(_pendingBuf, 0, sizeof(_pendingBuf));
}

// ─────────────────────────────────────────────────────────────────────────────
// Begin
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::begin(uint8_t i2cAddr) {
    Wire.begin(i2cAddr);
    Wire.onReceive(_handleReceive);
    Wire.onRequest(_handleRequest);
}

// ─────────────────────────────────────────────────────────────────────────────
// Callback Setters
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::onWrite(GrefurWriteCallback callback) {
    _writeCallback = callback;
}

void GrefurSlaveModule::onRead(GrefurReadCallback callback) {
    _readCallback = callback;
}

void GrefurSlaveModule::onError(GrefurErrorCallback callback) {
    _errorCallback = callback;
}

void GrefurSlaveModule::onDiscovery(GrefurSimpleCallback callback) {
    _discoveryCallback = callback;
}

void GrefurSlaveModule::onActivity(GrefurSimpleCallback callback) {
    _activityCallback = callback;
}

// ─────────────────────────────────────────────────────────────────────────────
// Add Register
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::addRegister(uint16_t addr, uint8_t type, bool writable, const char* name, void* ptr) {
    if (_regCount >= GREFUR_MAX_REGS) return;
    GrefurSlaveReg& r = _regs[_regCount++];
    r.addr     = addr;
    r.type     = type;
    r.writable = writable;
    r.dataPtr  = ptr;
    memset(r.name, 0, GREFUR_NAME_LEN);
    strncpy(r.name, name, GREFUR_NAME_LEN - 1);
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal: type size helper
// ─────────────────────────────────────────────────────────────────────────────
uint8_t GrefurSlaveModule::_getTypeSize(uint8_t type) {
    switch (type) {
        case GREFUR_T_U8:  return 1;
        case GREFUR_T_I16:
        case GREFUR_T_U16: return 2;
        case GREFUR_T_I32:
        case GREFUR_T_F32: return 4;
        default:           return 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Apply a pending write to the register's dataPtr
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::_applyPendingWrite() {
    if (!_pendingWrite) return;
    _pendingWrite = false;

    for (uint8_t i = 0; i < _regCount; i++) {
        GrefurSlaveReg& r = _regs[i];
        if (r.addr != _pendingRegAddr) continue;

        if (!r.writable) {
            _notifyError(GREFUR_ERR_NOT_WRITABLE);
            return;
        }

        uint8_t size = _getTypeSize(r.type);
        if (_pendingLen < size) {
            _notifyError(GREFUR_ERR_BAD_LENGTH);
            return;
        }

        uint32_t raw = 0;
        for (uint8_t b = 0; b < size; b++) raw = (raw << 8) | _pendingBuf[b];

        switch (r.type) {
            case GREFUR_T_U8:  *(uint8_t*)r.dataPtr  = (uint8_t)raw;  break;
            case GREFUR_T_U16: *(uint16_t*)r.dataPtr = (uint16_t)raw; break;
            case GREFUR_T_I16: *(int16_t*)r.dataPtr  = (int16_t)raw;  break;
            case GREFUR_T_I32: *(int32_t*)r.dataPtr  = (int32_t)raw;  break;
            case GREFUR_T_F32: memcpy(r.dataPtr, &raw, 4);            break;
        }

        if (_writeCallback) _writeCallback(r.addr, _pendingBuf, size);
        return;
    }

    // No matching register found
    _notifyError(GREFUR_ERR_UNKNOWN_REG);
}

// ─────────────────────────────────────────────────────────────────────────────
// Wire ISR: Receive (master writes to us)
// Expects: [addrHi(1), addrLo(1), ...payload...]
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::_handleReceive(int n) {
    if (!_instance) return;

    if (n < 2) {
        _instance->_notifyError(GREFUR_ERR_SHORT_PACKET);
        while (Wire.available()) Wire.read();
        return;
    }

    // Any valid I2C hit — fire activity callback
    if (_instance->_activityCallback) _instance->_activityCallback();

    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    uint16_t addr = ((uint16_t)hi << 8) | lo;
    _instance->_selectedAddr16 = addr;

    if (n > 2) {
        if (addr == 0x0000) {
            // Discovery mode: next byte is register index to describe
            _instance->_discoveryIdx = Wire.read();
            if (_instance->_discoveryCallback) _instance->_discoveryCallback();
        } else {
            // Write to a data register — buffer the payload bytes
            uint8_t avail = Wire.available();
            if (avail > 4) avail = 4;
            for (uint8_t i = 0; i < avail; i++) {
                _instance->_pendingBuf[i] = Wire.read();
            }
            _instance->_pendingLen     = avail;
            _instance->_pendingRegAddr = addr;
            _instance->_pendingWrite   = true;
            _instance->_applyPendingWrite();
        }
    }

    // Always drain — prevents buffer pollution between transactions
    while (Wire.available()) Wire.read();
}

// ─────────────────────────────────────────────────────────────────────────────
// Wire ISR: Request (master reads from us)
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::_handleRequest() {
    if (!_instance) return;

    // Any valid I2C hit — fire activity callback
    if (_instance->_activityCallback) _instance->_activityCallback();

    uint16_t sel = _instance->_selectedAddr16;

    // ── 0x0000: Metadata Discovery ────────────────────────────────────────────
    if (sel == 0x0000) {
        uint8_t idx = _instance->_discoveryIdx;
        if (idx < _instance->_regCount) {
            GrefurSlaveReg& r = _instance->_regs[idx];
            uint8_t nLen = strnlen(r.name, GREFUR_NAME_LEN);
            Wire.write(_instance->_regCount);
            Wire.write((r.addr >> 8) & 0xFF);
            Wire.write(r.addr & 0xFF);
            Wire.write(r.type);
            Wire.write((uint8_t)r.writable);
            Wire.write(nLen);
            Wire.write((const uint8_t*)r.name, nLen);
            if (_instance->_discoveryCallback) _instance->_discoveryCallback();
        } else {
            _instance->_notifyError(GREFUR_ERR_BAD_DISCOVERY_IDX);
            Wire.write((uint8_t)0x00);
        }
        return;
    }

    // ── 0x0001: Device ID ─────────────────────────────────────────────────────
    if (sel == 0x0001) {
        Wire.write(_instance->_deviceId >> 8);
        Wire.write(_instance->_deviceId & 0xFF);
        if (_instance->_readCallback) _instance->_readCallback(sel);
        return;
    }

    // ── Data Poll ─────────────────────────────────────────────────────────────
    for (uint8_t i = 0; i < _instance->_regCount; i++) {
        GrefurSlaveReg& r = _instance->_regs[i];
        if (r.addr != sel) continue;

        uint8_t size = _instance->_getTypeSize(r.type);

        uint32_t raw = 0;
        if (r.type == GREFUR_T_F32) {
            memcpy(&raw, r.dataPtr, 4);
        } else {
            switch (size) {
                case 1: raw = *(uint8_t*)r.dataPtr;  break;
                case 2: raw = *(uint16_t*)r.dataPtr; break;
                case 4: raw = *(uint32_t*)r.dataPtr; break;
            }
        }

        for (int8_t b = size - 1; b >= 0; b--) {
            Wire.write((raw >> (b * 8)) & 0xFF);
        }

        if (_instance->_readCallback) _instance->_readCallback(sel);
        return;
    }

    // Unknown register — send 0xFF sentinel and notify error
    _instance->_notifyError(GREFUR_ERR_UNKNOWN_REG);
    Wire.write((uint8_t)0xFF);
}
