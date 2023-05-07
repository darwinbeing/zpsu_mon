#include <math.h>
#include <watchface_ui.h>
#include <lvgl.h>
#include <general_ui.h>
#include "ui.h"
#ifdef __ZEPHYR__
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(watchface_ui, LOG_LEVEL_WRN);
#endif


#define SMALL_WATCHFACE_CENTER_OFFSET 38

#define USE_SECOND_HAND

const lv_img_dsc_t *get_icon_from_weather_code(int code);

void watchface_init(void)
{
}

void page_event_cb(lv_event_t *e)
{
}

void watchface_show(void)
{
	ui_init();
}

void watchface_remove(void)
{
}

void watchface_set_battery_percent(int32_t percent, int32_t value)
{
}

void watchface_set_hrm(int32_t value)
{
}

void watchface_set_step(int32_t value)
{
}

void watchface_set_time(int32_t hour, int32_t minute, int32_t second)
{
}

void watchface_set_num_notifcations(int32_t value)
{
}

void watchface_set_ble_connected(bool connected)
{
}

void watchface_set_weather(int8_t temperature, int weather_code)
{
}

void watchface_set_date(int day_of_week, int date)
{
}

const lv_img_dsc_t *get_icon_from_weather_code(int code)
{
}

void watchface_set_ep(struct psuctrl_data_event *evt)
{
        LOG_WRN("PSU: %s %s %s %s %d", evt->volts, evt->amps, evt->watts, evt->energy, evt->is_kWh);

        lv_label_set_text(ui_LabelVoltage, evt->volts);
        lv_label_set_text(ui_LabelCurrent, evt->amps);
        lv_label_set_text(ui_LabelPower, evt->watts);
        lv_label_set_text(ui_LabelEnergy, evt->energy);

        if(evt->is_kWh) {
                lv_label_set_text(ui_Label8, "kWh");
        } else {
                lv_label_set_text(ui_Label8, "Wh");
        }
}
