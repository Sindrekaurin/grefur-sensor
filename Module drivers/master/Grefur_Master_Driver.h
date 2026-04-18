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
#define GREFUR_T_F32  0x05   // NEW: 32-bit float support

// ─── Register Value Union ─────────────────────────────────────────────────────
// Allows correct signed/float interpretation without casting hacks
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
    uint8_t    address;
    uint8_t    dataType;
    char       name[GREFUR_NAME_LEN];
    bool       writable;
    GrefurValue lastValue;   // Correct typed union instead of raw uint32_t
    GrefurValue prevValue;   // Tracks previous value for change detection
    bool       valid;
    bool       changed;      // Set true when lastValue differs from prevValue
};

// ─── Device Descriptor ────────────────────────────────────────────────────────
struct GrefurDevice {
    uint8_t        i2cAddress;
    uint8_t        regCount;
    GrefurMasterReg registers[GREFUR_MAX_REGISTERS];
    bool           active;
    bool           online;   // Tracks live reachability separate from discovery
};

// ─── Change Callback Type ─────────────────────────────────────────────────────
// Called when a register value changes: (deviceIdx, regIdx, newValue)
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

    // Initialise I2C bus as module master
    void begin(int sda, int scl, uint32_t frequency = 100000);

    // ── Discovery ─────────────────────────────────────────────────────────────

    // Blocking full scan — returns number of modules found
    uint8_t scanAndDiscover();

    // Non-blocking scan — call repeatedly in loop(), returns true when done
    bool scanAndDiscoverAsync();

    // Reset scan so it can be run again
    void resetScan();

    // ── Online Check ──────────────────────────────────────────────────────────

    // Ping a single module — updates device.online, returns reachability
    bool isModuleOnline(uint8_t devIdx);

    // Ping all known modules — updates online flag for each
    void checkAllModulesOnline();

    // ── Polling ───────────────────────────────────────────────────────────────

    // Poll all registers on one module
    bool pollDevice(uint8_t devIdx);

    // Poll a single register on one module
    bool pollRegister(uint8_t devIdx, uint8_t regIdx);

    // ── Write ─────────────────────────────────────────────────────────────────

    // Write to a module by device index (consistent with poll API)
    bool writeToModule(uint8_t devIdx, uint8_t regIdx, GrefurValue value);

    // Write raw — by I2C address and register address directly (advanced use)
    bool writeRaw(uint8_t devAddr, uint8_t regAddr, uint8_t type, GrefurValue value);

    // ── Register Lookup ───────────────────────────────────────────────────────

    // Find a register by name on a specific device — returns regIdx or -1
    int8_t findRegisterByName(uint8_t devIdx, const char* name);

    // ── Change Callback ───────────────────────────────────────────────────────

    // Register a callback fired when any polled register changes value
    void onValueChanged(GrefurChangeCallback callback);

    // ── Data Access ───────────────────────────────────────────────────────────

    uint8_t      getDeviceCount()          { return _deviceCount; }
    GrefurDevice* getDevices()             { return _devices; }
    GrefurDevice* getDevice(uint8_t idx)   { return (idx < _deviceCount) ? &_devices[idx] : nullptr; }

    // Typed getters for last polled value
    float    getFloat(uint8_t devIdx, uint8_t regIdx);
    int32_t  getInt(uint8_t devIdx, uint8_t regIdx);
    uint32_t getUInt(uint8_t devIdx, uint8_t regIdx);

private:
    GrefurDevice         _devices[GREFUR_MAX_DEVICES];
    uint8_t              _deviceCount;
    GrefurChangeCallback _changeCallback;

    // Async scan state
    GrefurScanState _scanState;
    uint8_t         _scanAddr;        // Current address being scanned
    uint8_t         _scanRegIdx;      // Current register being discovered
    bool            _scanWaitingRegs; // True while fetching registers for a found device

    uint8_t _getTypeSize(uint8_t type);
    void    _storeValue(GrefurMasterReg& reg, uint8_t* buf, uint8_t size);
    bool    _readRegisterBytes(uint8_t i2cAddr, uint8_t regAddr, uint8_t size, uint8_t* outBuf);
    bool    _writeRegisterBytes(uint8_t i2cAddr, uint8_t regAddr, uint8_t type, GrefurValue value);
};

#endif
