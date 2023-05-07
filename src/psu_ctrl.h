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

int psuctrl_init(void);
int PSUCtrl_forceFanRPM(int rpm);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
