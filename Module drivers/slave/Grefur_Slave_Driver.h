/* {Summary: Grefur Slave Module Driver - I2C Slave Protocol} */

#ifndef GREFUR_SLAVE_DRIVER_H
#define GREFUR_SLAVE_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

// ─── Data Types (must match Master Driver) ────────────────────────────────────
#define GREFUR_T_U8   0x01
#define GREFUR_T_I16  0x02
#define GREFUR_T_U16  0x03
#define GREFUR_T_I32  0x04
#define GREFUR_T_F32  0x05   // Float32 — matches Master Driver

// ─── Limits ───────────────────────────────────────────────────────────────────
#define GREFUR_MAX_REGS  16
#define GREFUR_NAME_LEN  16  // Matches Master Driver

// ─── Write Callback Type ──────────────────────────────────────────────────────
// Fired when master writes to a writable register: (regAddr, newRawBytes, numBytes)
typedef void (*GrefurWriteCallback)(uint8_t regAddr, const uint8_t* data, uint8_t len);

// ─── Register Descriptor ──────────────────────────────────────────────────────
struct GrefurSlaveReg {
    uint8_t addr;
    uint8_t type;
    bool    writable;
    char    name[GREFUR_NAME_LEN];  // Local copy — no dangling pointer risk
    void*   dataPtr;
};

// ─────────────────────────────────────────────────────────────────────────────
class GrefurSlaveModule {
public:
    GrefurSlaveModule(uint16_t deviceId);

    // Add a local variable to the I2C register map
    // name is copied into fixed-size buffer — safe for string literals and stack strings
    void addRegister(uint8_t addr, uint8_t type, bool writable, const char* name, void* ptr);

    // Start I2C slave on given address
    void begin(uint8_t i2cAddr);

    // Register a callback fired when master writes to any writable register
    void onWrite(GrefurWriteCallback callback);

private:
    static GrefurSlaveModule* _instance;

    uint16_t       _deviceId;
    GrefurSlaveReg _regs[GREFUR_MAX_REGS];
    uint8_t        _regCount;

    // Write callback
    GrefurWriteCallback _writeCallback;

    // ISR-safe state — access only inside Wire callbacks
    volatile uint8_t _selectedAddr;
    volatile uint8_t _discoveryIdx;

    // Pending write buffer — filled in _handleReceive, acted on in _handleRequest
    // Using a small staging buffer ensures we never block inside the ISR
    volatile bool    _pendingWrite;
    volatile uint8_t _pendingRegAddr;
    uint8_t          _pendingBuf[4];
    volatile uint8_t _pendingLen;

    static void _handleReceive(int n);
    static void _handleRequest();

    // Internal helpers
    uint8_t _getTypeSize(uint8_t type);
    void    _applyPendingWrite();
};

#endif
