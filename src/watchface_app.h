#pragma once

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

void watchface_app_start(lv_group_t *group);
void watchface_app_stop(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif
