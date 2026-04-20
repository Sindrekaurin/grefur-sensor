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
#define GREFUR_T_F32  0x05

// ─── Limits ───────────────────────────────────────────────────────────────────
#define GREFUR_MAX_REGS  16
#define GREFUR_NAME_LEN  16

// ─── Write Callback Type ──────────────────────────────────────────────────────
typedef void (*GrefurWriteCallback)(uint16_t regAddr, const uint8_t* data, uint8_t len);

// ─── Register Descriptor ──────────────────────────────────────────────────────
struct GrefurSlaveReg {
    uint16_t addr;      // ← uint16_t
    uint8_t  type;
    bool     writable;
    char     name[GREFUR_NAME_LEN];
    void*    dataPtr;
};

// ─────────────────────────────────────────────────────────────────────────────
class GrefurSlaveModule {
public:
    GrefurSlaveModule(uint16_t deviceId);

    void addRegister(uint16_t addr, uint8_t type, bool writable, const char* name, void* ptr); // ← uint16_t
    void begin(uint8_t i2cAddr);
    void onWrite(GrefurWriteCallback callback);

private:
    static GrefurSlaveModule* _instance;

    uint16_t       _deviceId;
    GrefurSlaveReg _regs[GREFUR_MAX_REGS];
    uint8_t        _regCount;

    GrefurWriteCallback _writeCallback;

    volatile uint16_t _selectedAddr16;  // ← var _selectedAddr (uint8_t)
    volatile uint16_t _discoveryIdx;
    volatile bool     _pendingWrite;
    volatile uint16_t _pendingRegAddr;  // ← var uint8_t
    uint8_t           _pendingBuf[4];
    volatile uint8_t  _pendingLen;

    static void _handleReceive(int n);
    static void _handleRequest();

    uint8_t _getTypeSize(uint8_t type);
    void    _applyPendingWrite();
};

#endif
