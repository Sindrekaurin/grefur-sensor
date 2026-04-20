/* {Summary: Grefur Master Driver - I2C Orchestration & Discovery} */

#ifndef GREFUR_MASTER_DRIVER_H
#define GREFUR_MASTER_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

// ─── Limits ───────────────────────────────────────────────────────────────────
#define GREFUR_MAX_DEVICES   16
#define GREFUR_MAX_REGISTERS 16
#define GREFUR_NAME_LEN      16
#define GREFUR_I2C_RETRIES   3

// ─── Data Types (must match Slave Driver) ─────────────────────────────────────
#define GREFUR_T_U8   0x01
#define GREFUR_T_I16  0x02
#define GREFUR_T_U16  0x03
#define GREFUR_T_I32  0x04
#define GREFUR_T_F32  0x05

// ─── Register Value Union ─────────────────────────────────────────────────────
union GrefurValue {
    uint32_t asU32;
    int32_t  asI32;
    uint16_t asU16;
    int16_t  asI16;
    uint8_t  asU8;
    float    asFloat;
};

// ─── Register Descriptor ──────────────────────────────────────────────────────
struct GrefurMasterReg {
    uint16_t    address;
    uint8_t     dataType;
    char        name[GREFUR_NAME_LEN];
    bool        writable;
    GrefurValue lastValue;
    GrefurValue prevValue;
    bool        valid;
    bool        changed;
};

// ─── Device Descriptor ────────────────────────────────────────────────────────
struct GrefurDevice {
    uint8_t         i2cAddress;
    uint8_t         regCount;
    GrefurMasterReg registers[GREFUR_MAX_REGISTERS];
    bool            active;
    bool            online;
};

// ─── Change Callback Type ─────────────────────────────────────────────────────
typedef void (*GrefurChangeCallback)(uint8_t devIdx, uint8_t regIdx, GrefurValue newValue);

// ─── Async Scan State ─────────────────────────────────────────────────────────
enum GrefurScanState {
    GREFUR_SCAN_IDLE,
    GREFUR_SCAN_RUNNING,
    GREFUR_SCAN_DONE
};

// ─────────────────────────────────────────────────────────────────────────────
class GrefurModuleMaster {
public:
    GrefurModuleMaster();

    void begin(int sda, int scl, uint32_t frequency = 100000);

    // ── Discovery ─────────────────────────────────────────────────────────────
    uint8_t scanAndDiscover();
    bool    scanAndDiscoverAsync();
    void    resetScan();

    // ── Online Check ──────────────────────────────────────────────────────────
    bool isModuleOnline(uint8_t devIdx);
    void checkAllModulesOnline();

    // ── Polling ───────────────────────────────────────────────────────────────
    bool pollDevice(uint8_t devIdx);
    bool pollRegister(uint8_t devIdx, uint8_t regIdx);
    bool pollAddress(uint8_t i2cAddr, uint16_t regAddr);

    // ── Write ─────────────────────────────────────────────────────────────────
    bool writeToModule(uint8_t devIdx, uint8_t regIdx, GrefurValue value);
    bool writeRaw(uint8_t devAddr, uint16_t regAddr, uint8_t type, GrefurValue value);

    // ── Register Lookup ───────────────────────────────────────────────────────
    int8_t findRegisterByName(uint8_t devIdx, const char* name);

    // ── Change Callback ───────────────────────────────────────────────────────
    void onValueChanged(GrefurChangeCallback callback);

    // ── Data Access ───────────────────────────────────────────────────────────
    uint8_t      getDeviceCount()  const { return _deviceCount; }
    GrefurDevice* getDevices()           { return _devices; }
    GrefurDevice* getDevice(uint8_t idx) { return (idx < _deviceCount) ? &_devices[idx] : nullptr; }

    uint8_t     getDeviceAddress(uint8_t devIdx) const;
    uint8_t     getRegisterCount(uint8_t devIdx) const;
    const char* getRegisterName(uint8_t devIdx, uint8_t regIdx) const;
    uint16_t    getRegisterAddress(uint8_t devIdx, uint8_t regIdx) const;
    uint8_t     getRegisterDataType(uint8_t devIdx, uint8_t regIdx) const;

    // Typed getters by device/register index
    float    getFloat(uint8_t devIdx, uint8_t regIdx);
    int32_t  getInt(uint8_t devIdx, uint8_t regIdx);
    uint32_t getUInt(uint8_t devIdx, uint8_t regIdx);

    // Typed getters by raw I2C + register address
    float   getValueAsFloat(uint8_t i2cAddr, uint16_t regAddr);
    int32_t getIntByAddress(uint8_t i2cAddr, uint16_t regAddr);
    float   getFloatByAddress(uint8_t i2cAddr, uint16_t regAddr);

private:
    GrefurDevice         _devices[GREFUR_MAX_DEVICES];
    uint8_t              _deviceCount;
    GrefurChangeCallback _changeCallback;

    GrefurScanState _scanState;
    uint8_t         _scanAddr;
    uint8_t         _scanRegIdx;
    bool            _scanWaitingRegs;

    void _storeValue(GrefurMasterReg& reg, uint8_t* buf, uint8_t size);
    bool _readRegisterBytes(uint8_t i2cAddr, uint16_t regAddr, uint8_t size, uint8_t* outBuf);
    bool _writeRegisterBytes(uint8_t i2cAddr, uint16_t regAddr, uint8_t type, GrefurValue value);
    bool _tryReconnect(uint8_t devIdx);  // Ping offline device and restore online flag
    uint8_t _getTypeSize(uint8_t type);
};

#endif
