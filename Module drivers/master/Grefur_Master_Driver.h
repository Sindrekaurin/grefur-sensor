/* {Summary: Grefur Master Driver - I2C Orchestration & Discovery} */

#ifndef GREFUR_MASTER_DRIVER_H
#define GREFUR_MASTER_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

#define GREFUR_MAX_DEVICES   16
#define GREFUR_MAX_REGISTERS 16
#define GREFUR_NAME_LEN      16
#define GREFUR_I2C_RETRIES   3

/* Data Types matching Slave Driver */
#define GREFUR_T_U8   0x01
#define GREFUR_T_I16  0x02
#define GREFUR_T_U16  0x03
#define GREFUR_T_I32  0x04

struct GrefurMasterReg {
    uint8_t address;
    uint8_t dataType;
    char name[GREFUR_NAME_LEN];
    bool writable;
    uint32_t lastValue;
    bool valid;
};

struct GrefurDevice {
    uint8_t i2cAddress;
    uint8_t regCount;
    GrefurMasterReg registers[GREFUR_MAX_REGISTERS];
    bool active;
};

class GrefurMaster {
public:
    GrefurMaster();
    void begin(int sda, int scl, uint32_t frequency = 100000);
    
    /* Discovery */
    uint8_t scanAndDiscover();
    
    /* Polling Logic */
    bool pollDevice(uint8_t deviceIdx);              // Cluster Poll: All regs on one device
    bool pollRegister(uint8_t devIdx, uint8_t regIdx); // Single Data Point Poll
    
    /* Write Logic */
    bool writeValue(uint8_t devAddr, uint8_t regAddr, uint8_t type, uint32_t value);

    /* Data Access */
    uint8_t getDeviceCount() { return _deviceCount; }
    GrefurDevice* getDevices() { return _devices; }

private:
    GrefurDevice _devices[GREFUR_MAX_DEVICES];
    uint8_t _deviceCount = 0;
    uint8_t _getTypeSize(uint8_t type);
};

#endif
