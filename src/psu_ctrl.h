#ifndef PSU_CTRL_H
#define PSU_CTRL_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

int iPSUCtrl_init(uint8_t i2cbus, uint8_t address);
int PSUCtrl_deviceIsReady();
int PSUCtrl_readEEPROM();
int PSUCtrl_readVar(uint8_t address, uint8_t* writeInts, uint32_t writeLen, uint8_t* data, uint32_t readCount);
int PSUCtrl_writeVar(uint8_t address, uint8_t* data, uint32_t len);
int PSUCtrl_readDPS1200(uint8_t reg, uint8_t* data, uint32_t count);
int PSUCtrl_readDPS1200Register(uint8_t reg, int *value);
int PSUCtrl_writeDPS1200(uint8_t reg, int value);
int PSUCtrl_forceFanRPM(int rpm);
int PSUCtrl_deinit(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
