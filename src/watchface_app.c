#include <watchface_ui.h>

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/logging/log.h>
#include <lvgl.h>
#include <clock.h>
#include <battery.h>
#include <heart_rate_sensor.h>
#include <accelerometer.h>
#include <vibration_motor.h>
#include <ram_retention_storage.h>
#include <events/ble_data_event.h>
#include <events/accel_event.h>
#include <notification_manager.h>
#include <zephyr/zbus/zbus.h>
#include <zsw_charger.h>
#include <events/chg_event.h>
#include <events/psuctrl_event.h>

LOG_MODULE_REGISTER(watcface_app, LOG_LEVEL_WRN);

static void zbus_ble_comm_data_callback(const struct zbus_channel *chan);
static void zbus_accel_data_callback(const struct zbus_channel *chan);
static void zbus_chg_state_data_callback(const struct zbus_channel *chan);
static void zbus_psuctrl_data_callback(const struct zbus_channel *chan);

ZBUS_CHAN_DECLARE(ble_comm_data_chan);
ZBUS_LISTENER_DEFINE(watchface_ble_comm_lis, zbus_ble_comm_data_callback);

ZBUS_CHAN_DECLARE(accel_data_chan);
ZBUS_LISTENER_DEFINE(watchface_accel_lis, zbus_accel_data_callback);

ZBUS_CHAN_DECLARE(chg_state_data_chan);
ZBUS_LISTENER_DEFINE(watchface_chg_event, zbus_chg_state_data_callback);

ZBUS_CHAN_DECLARE(psuctrl_data_chan);
ZBUS_LISTENER_DEFINE(watchface_psuctrl_event, zbus_psuctrl_data_callback);

#define WORK_STACK_SIZE 3000
#define WORK_PRIORITY   5

#define RENDER_INTERVAL_LVGL    K_MSEC(100)
#define ACCEL_INTERVAL          K_MSEC(100)
#define BATTERY_INTERVAL        K_MINUTES(1)
#define SEND_STATUS_INTERVAL    K_SECONDS(30) // TODO move out from here
#define DATE_UPDATE_INTERVAL    K_MINUTES(1)

typedef enum work_type {
    UPDATE_CLOCK,
    OPEN_WATCHFACE,
    BATTERY,
    SEND_STATUS_UPDATE,
    PSU_STATUS_UPDATE,
    UPDATE_DATE
} work_type_t;

typedef struct delayed_work_item {
    struct k_work_delayable work;
    work_type_t             type;
} delayed_work_item_t;

void general_work(struct k_work *item);

static void check_notifications(void);
static int read_battery(int *batt_mV, int *percent);
static void update_ui_from_event(struct k_work *item);

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

static delayed_work_item_t battery_work =   { .type = BATTERY };
static delayed_work_item_t clock_work =     { .type = UPDATE_CLOCK };
static delayed_work_item_t status_work =    { .type = SEND_STATUS_UPDATE };
static delayed_work_item_t date_work =      { .type = UPDATE_DATE };
static delayed_work_item_t psuctrl_work =    { .type = PSU_STATUS_UPDATE };

static delayed_work_item_t general_work_item;
static struct k_work_sync canel_work_sync;

static K_WORK_DEFINE(update_ui_work, update_ui_from_event);
static ble_comm_cb_data_t last_data_update;

static bool running;

static int watchface_app_init(const struct device *arg)
{
    k_work_init_delayable(&general_work_item.work, general_work);
    k_work_init_delayable(&battery_work.work, general_work);
    k_work_init_delayable(&clock_work.work, general_work);
    k_work_init_delayable(&status_work.work, general_work);
    k_work_init_delayable(&date_work.work, general_work);
    /* k_work_init_delayable(&psuctrl_work.work, general_work); */
    running = false;

    return 0;
}

void watchface_app_start(lv_group_t *group)
{
    general_work_item.type = OPEN_WATCHFACE;
    // __ASSERT(0 <= k_work_schedule(&general_work_item.work, K_MSEC(100)), "FAIL schedule");
    __ASSERT(0 <= k_work_schedule(&general_work_item.work, K_NO_WAIT), "FAIL schedule");
}

void watchface_app_stop(void)
{
    running = false;
    k_work_cancel_delayable_sync(&battery_work.work, &canel_work_sync);
    k_work_cancel_delayable_sync(&clock_work.work, &canel_work_sync);
    k_work_cancel_delayable_sync(&date_work.work, &canel_work_sync);
    watchface_remove();
}

void general_work(struct k_work *item)
{
    delayed_work_item_t *the_work = CONTAINER_OF(item, delayed_work_item_t, work);
    switch (the_work->type) {
        case OPEN_WATCHFACE: {
            running = true;
            watchface_show();
            lvgl_update();
            /* __ASSERT(0 <= k_work_schedule(&battery_work.work, K_NO_WAIT), "FAIL battery_work"); */
            /* __ASSERT(0 <= k_work_schedule(&clock_work.work, K_NO_WAIT), "FAIL clock_work"); */
            /* __ASSERT(0 <= k_work_schedule(&date_work.work, K_SECONDS(1)), "FAIL clock_work"); */
            break;
        }
        case UPDATE_CLOCK: {
            struct tm *time = clock_get_time();
            LOG_INF("%d, %d, %d\n", time->tm_hour, time->tm_min, time->tm_sec);
            watchface_set_time(time->tm_hour, time->tm_min, time->tm_sec);

            // TODO move from this file
            /* retained.current_time_seconds = clock_get_time_unix(); */
            /* retained_update(); */
            check_notifications();
            __ASSERT(0 <= k_work_schedule(&clock_work.work, K_SECONDS(1)), "FAIL clock_work");
            break;
        }
        case UPDATE_DATE: {
            struct tm *time = clock_get_time();
            watchface_set_date(time->tm_wday, time->tm_mday);
            __ASSERT(0 <= k_work_schedule(&date_work.work, DATE_UPDATE_INTERVAL), "FAIL date_work");
        }
        case BATTERY: {
            int batt_mv;
            int batt_percent;
            static uint32_t count;

            if (read_battery(&batt_mv, &batt_percent) == 0) {
                watchface_set_battery_percent(batt_percent, batt_mv);
            }
            watchface_set_hrm(count % 220);
            //heart_rate_sensor_fetch(&hr_sample);
            count++;
            __ASSERT(0 <= k_work_schedule(&battery_work.work, BATTERY_INTERVAL),
                     "FAIL battery_work");
            break;
        }
        case SEND_STATUS_UPDATE: {
            // TODO move to main
            int batt_mv;
            int batt_percent;
            int msg_len;
            int is_charging;
            char buf[100];
            memset(buf, 0, sizeof(buf));

            /* if (read_battery(&batt_mv, &batt_percent) == 0) { */
            /*     is_charging = zsw_charger_is_charging(); */
            /*     msg_len = snprintf(buf, sizeof(buf), "{\"t\":\"status\", \"bat\": %d, \"volt\": %d, \"chg\": %d} \n", batt_percent, */
            /*                        batt_mv, is_charging); */
            /*     ble_comm_send(buf, msg_len); */
            /* } */
            __ASSERT(0 <= k_work_schedule(&status_work.work, SEND_STATUS_INTERVAL),
                     "Failed schedule status work");
            break;
        }
    }
}

/** A discharge curve specific to the power source. */
static const struct battery_level_point levels[] = {
    /*
    Battery supervisor cuts power at 3500mA so treat that as 0%
    TODO analyze more to get a better curve.
    */
    { 10000, 4150 },
    { 0, 3500 },
};

static int read_battery(int *batt_mV, int *percent)
{
#if 0
    int rc = battery_measure_enable(true);
    if (rc != 0) {
        LOG_ERR("Failed initialize battery measurement: %d\n", rc);
        return -1;
    }
    // From https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/boards/nrf/battery/src/main.c
    *batt_mV = battery_sample();

    if (*batt_mV < 0) {
        LOG_ERR("Failed to read battery voltage: %d\n", *batt_mV);
        return -1;
    }

    unsigned int batt_pptt = battery_level_pptt(*batt_mV, levels);

    LOG_DBG("%d mV; %u pptt\n", *batt_mV, batt_pptt);
    *percent = batt_pptt / 100;

    rc = battery_measure_enable(false);
    if (rc != 0) {
        LOG_ERR("Failed disable battery measurement: %d\n", rc);
        return -1;
    }
#endif
    return 0;
}

static void check_notifications(void)
{
    uint32_t num_unread = notification_manager_get_num();
    watchface_set_num_notifcations(num_unread);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (!running) return;
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    __ASSERT(0 <= k_work_schedule(&status_work.work, K_MSEC(1000)), "FAIL status");

    watchface_set_ble_connected(true);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (!running) return;
    watchface_set_ble_connected(false);
}

static void update_ui_from_event(struct k_work *item)
{
    if (running) {
        if (last_data_update.type == BLE_COMM_DATA_TYPE_WEATHER) {
            LOG_DBG("Weather: %s t: %d hum: %d code: %d wind: %d dir: %d", last_data_update.data.weather.report_text,
                    last_data_update.data.weather.temperature_c, last_data_update.data.weather.humidity, last_data_update.data.weather.weather_code,
                    last_data_update.data.weather.wind,
                    last_data_update.data.weather.wind_direction);
            watchface_set_weather(last_data_update.data.weather.temperature_c, last_data_update.data.weather.weather_code);
        } else if (last_data_update.type == BLE_COMM_DATA_TYPE_SET_TIME) {
            k_work_reschedule(&date_work.work, K_SECONDS(1));
        }
        return;
    }
}

static void zbus_ble_comm_data_callback(const struct zbus_channel *chan)
{
	if (running) {
        struct ble_data_event *event = zbus_chan_msg(chan);
        // TODO getting this callback again before workqueue has ran will
        // cause previous to be lost.
        memcpy(&last_data_update, &event->data, sizeof(ble_comm_cb_data_t));
        k_work_submit(&update_ui_work);
    }
}

static void zbus_accel_data_callback(const struct zbus_channel *chan)
{
	if (running) {
        struct accel_event *event = zbus_chan_msg(chan);
        if (event->data.type == ACCELEROMETER_EVT_TYPE_STEP) {
            watchface_set_step(event->data.data.step.count);
        }
    }
}

static void zbus_chg_state_data_callback(const struct zbus_channel *chan)
{
    if (running) {
        struct chg_state_event *event = zbus_chan_msg(chan);
        // TODO Show some nice animation or similar
        LOG_WRN("CHG: %d", event->is_charging);
        __ASSERT(0 <= k_work_reschedule(&status_work.work, K_MSEC(10)),
                     "Failed schedule status work");
    }
}

static void zbus_psuctrl_data_callback(const struct zbus_channel *chan)
{
    if (running) {
        struct psuctrl_data_event *event = zbus_chan_msg(chan);
        // TODO Show some nice animation or similar
        // LOG_WRN("PSU: %s %s %s %s", event->volts, event->amps, event->watts, event->energy);
        /* __ASSERT(0 <= k_work_reschedule(&psuctrl_work.work, K_MSEC(10)), */
        /*              "Failed schedule status work"); */
        watchface_set_ep(event->volts, event->amps, event->watts, event->energy);
        lvgl_update();
    }
}

SYS_INIT(watchface_app_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
