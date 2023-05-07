#ifndef __DISPLAY_CONTROL_H_
#define __DISPLAY_CONTROL_H_
#include <inttypes.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void display_control_init(void);
void display_control_power_on(bool on);
void display_control_set_brightness(uint8_t percent);
void lvgl_update(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
