/* {Summary: Grefur Module Driver - I2C Slave Protocol} */

#ifndef GREFUR_MODULE_DRIVER_H
#define GREFUR_MODULE_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

/* Grefur Datatypes */
#define GREFUR_T_U8   0x01
#define GREFUR_T_I16  0x02
#define GREFUR_T_U16  0x03
#define GREFUR_T_I32  0x04

#define GREFUR_MAX_REGS 16

struct GrefurReg {
  uint8_t addr;
  uint8_t type;
  bool writable;
  const char* name;
  void* dataPtr;
};

class GrefurModule {
public:
  GrefurModule(uint16_t deviceId);
  
  /* {Summary: Add a local variable to the I2C register table} */
  void addRegister(uint8_t addr, uint8_t type, bool writable, const char* name, void* ptr);
  
  /* {Summary: Initialize I2C slave with address} */
  void begin(uint8_t i2cAddr);

private:
  static void _handleReceive(int n);
  static void _handleRequest();
  
  uint16_t _deviceId;
  static GrefurModule* _instance;
  GrefurReg _regs[GREFUR_MAX_REGS];
  uint8_t _regCount = 0;
  
  volatile uint8_t _selectedAddr = 0;
  volatile uint8_t _discoveryIdx = 0;
};

#endif
