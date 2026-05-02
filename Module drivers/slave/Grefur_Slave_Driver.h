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

// ─── Error Codes ──────────────────────────────────────────────────────────────
#define GREFUR_ERR_SHORT_PACKET      0x01
#define GREFUR_ERR_NOT_WRITABLE      0x02
#define GREFUR_ERR_BAD_LENGTH        0x03
#define GREFUR_ERR_UNKNOWN_REG       0x04
#define GREFUR_ERR_BAD_DISCOVERY_IDX 0x05

// ─── Callback Types ───────────────────────────────────────────────────────────
typedef void (*GrefurWriteCallback)(uint16_t regAddr, const uint8_t* data, uint8_t len);
typedef void (*GrefurReadCallback)(uint16_t regAddr);   // ← moved here, global scope
typedef void (*GrefurSimpleCallback)(void);
typedef void (*GrefurErrorCallback)(uint8_t errorCode);

// ─── Register Descriptor ──────────────────────────────────────────────────────
struct GrefurSlaveReg {
    uint16_t addr;
    uint8_t  type;
    bool     writable;
    char     name[GREFUR_NAME_LEN];
    void*    dataPtr;
};

// ─────────────────────────────────────────────────────────────────────────────
class GrefurSlaveModule {
public:
    GrefurSlaveModule(uint16_t deviceId);

    void addRegister(uint16_t addr, uint8_t type, bool writable, const char* name, void* ptr);
    void begin(uint8_t i2cAddr);

    void onWrite(GrefurWriteCallback callback);
    void onRead(GrefurReadCallback callback);
    void onError(GrefurErrorCallback callback);
    void onDiscovery(GrefurSimpleCallback callback);
    void onActivity(GrefurSimpleCallback callback);

private:
    static GrefurSlaveModule* _instance;

    uint16_t       _deviceId;
    GrefurSlaveReg _regs[GREFUR_MAX_REGS];
    uint8_t        _regCount;

    volatile uint16_t _selectedAddr16;
    volatile uint8_t  _discoveryIdx;    // ← uint8_t, maks 16
    volatile bool     _pendingWrite;
    volatile uint16_t _pendingRegAddr;
    uint8_t           _pendingBuf[4];
    volatile uint8_t  _pendingLen;

    GrefurWriteCallback  _writeCallback;    // ← kun én gang
    GrefurReadCallback   _readCallback;
    GrefurErrorCallback  _errorCallback;
    GrefurSimpleCallback _discoveryCallback;
    GrefurSimpleCallback _activityCallback;

    static void _handleReceive(int n);
    static void _handleRequest();

    uint8_t _getTypeSize(uint8_t type);
    void    _applyPendingWrite();
    void    _notifyError(uint8_t code) {
        if (_errorCallback) _errorCallback(code);
    }
};

#endif
