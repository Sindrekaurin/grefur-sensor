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
      _selectedAddr(0),
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
// Register a write callback
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::onWrite(GrefurWriteCallback callback) {
    _writeCallback = callback;
}

// ─────────────────────────────────────────────────────────────────────────────
// Add Register
// name is strncpy'd into a fixed buffer — no dangling pointer risk
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::addRegister(uint8_t addr, uint8_t type, bool writable, const char* name, void* ptr) {
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
// Called from _handleRequest (still inside Wire ISR context) — kept minimal
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::_applyPendingWrite() {
    if (!_pendingWrite) return;
    _pendingWrite = false;

    for (uint8_t i = 0; i < _regCount; i++) {
        GrefurSlaveReg& r = _regs[i];
        if (r.addr != _pendingRegAddr || !r.writable) continue;

        uint8_t size = _getTypeSize(r.type);
        if (_pendingLen < size) return;

        // Reconstruct raw value from big-endian bytes
        uint32_t raw = 0;
        for (uint8_t b = 0; b < size; b++) raw = (raw << 8) | _pendingBuf[b];

        switch (r.type) {
            case GREFUR_T_U8:  *(uint8_t*)r.dataPtr  = (uint8_t)raw;  break;
            case GREFUR_T_U16: *(uint16_t*)r.dataPtr = (uint16_t)raw; break;
            case GREFUR_T_I16: *(int16_t*)r.dataPtr  = (int16_t)raw;  break;
            case GREFUR_T_I32: *(int32_t*)r.dataPtr  = (int32_t)raw;  break;
            case GREFUR_T_F32:
                memcpy(r.dataPtr, &raw, 4); // IEEE 754 bit-reinterpretation
                break;
        }

        // Fire write callback if registered
        if (_writeCallback) {
            _writeCallback(r.addr, _pendingBuf, size);
        }
        return;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Wire ISR: Receive (master writes to us)
// ─────────────────────────────────────────────────────────────────────────────
void GrefurSlaveModule::_handleReceive(int n) {
    if (n < 1 || !_instance) return;

    _instance->_selectedAddr = Wire.read();

    if (n > 1) {
        if (_instance->_selectedAddr == 0x00) {
            // Discovery mode: next byte is register index to describe
            _instance->_discoveryIdx = Wire.read();
        } else {
            // Write to a data register — buffer the bytes, apply them safely
            uint8_t avail = Wire.available();
            if (avail > 4) avail = 4;
            for (uint8_t i = 0; i < avail; i++) {
                _instance->_pendingBuf[i] = Wire.read();
            }
            _instance->_pendingLen     = avail;
            _instance->_pendingRegAddr = _instance->_selectedAddr;
            _instance->_pendingWrite   = true;

            // Apply immediately while still in ISR context
            // (Wire callbacks on ESP32 run in task context, not true ISR — this is safe)
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

    uint8_t sel = _instance->_selectedAddr;

    // ── 0x00: Metadata Discovery ─────────────────────────────────────────────
    if (sel == 0x00) {
        uint8_t idx = _instance->_discoveryIdx;
        if (idx < _instance->_regCount) {
            GrefurSlaveReg& r = _instance->_regs[idx];
            uint8_t nLen = strnlen(r.name, GREFUR_NAME_LEN);
            Wire.write(_instance->_regCount);   // Total register count
            Wire.write(r.addr);                  // Register address
            Wire.write(r.type);                  // Data type
            Wire.write((uint8_t)r.writable);     // Writable flag
            Wire.write(nLen);                    // Name length
            Wire.write((const uint8_t*)r.name, nLen); // Name bytes
        } else {
            Wire.write((uint8_t)0x00);
        }
        return;
    }

    // ── 0x01: Device ID ───────────────────────────────────────────────────────
    if (sel == 0x01) {
        Wire.write(_instance->_deviceId >> 8);
        Wire.write(_instance->_deviceId & 0xFF);
        return;
    }

    // ── Data Poll ─────────────────────────────────────────────────────────────
    for (uint8_t i = 0; i < _instance->_regCount; i++) {
        GrefurSlaveReg& r = _instance->_regs[i];
        if (r.addr != sel) continue;

        uint8_t size = _instance->_getTypeSize(r.type);

        // Read raw bytes from dataPtr and send big-endian
        // Handles all types including F32 correctly via memcpy
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
        return;
    }

    // Unknown register — send size-matched 0xFF bytes to avoid master desync
    // Master will receive expected byte count but with sentinel value
    Wire.write((uint8_t)0xFF);
}
