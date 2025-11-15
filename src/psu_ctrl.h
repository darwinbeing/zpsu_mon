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
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

int psuctrl_init(void);
int PSUCtrl_forceFanRPM(int rpm);
void PSUCtrl_ONOFF(lv_event_t * e);
void PSUCtrl_CVCC(lv_event_t * e);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
