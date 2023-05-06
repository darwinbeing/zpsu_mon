#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/zbus/zbus.h>
#include <lvgl.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <math.h>
#include "ui.h"

#include "button_handler.h"
#include "button_assignments.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app);

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));


static uint32_t count;
static uint8_t brightness = 0;
static uint32_t fanRPM = 3200;

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS		1000

// The devicetree node identifier for the "led0" alias.
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define GPIO_BL_EN	20
static int set_backlight(uint8_t brightness);

ZBUS_CHAN_DECLARE(button_chan);
ZBUS_OBS_DECLARE(button_sub);

#define CONFIG_BUTTON_MSG_SUB_THREAD_PRIO 5
#define CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE 4
#define CONFIG_BUTTON_MSG_SUB_STACK_SIZE 1024
ZBUS_SUBSCRIBER_DEFINE(button_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);
static struct k_thread button_msg_sub_thread_data;
static k_tid_t button_msg_sub_thread_id;

K_THREAD_STACK_DEFINE(button_msg_sub_thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);

class PowerSupply {
 public:
	PowerSupply(uint8_t i2cbus = 0, uint8_t address = 7) : address_(0x58 + address), EEaddress_(0x50 + address) {
		const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
		if (!dev) {
			printf("Failed to get I2C device binding\n");
			return;
		}

		this->i2cdev_ = dev;
		this->numReg_ = 0x58 / 2;
		// this->lastReg_ = (uint16_t *)k_malloc(this->numReg_);
		// memset(this->lastReg_, 0, this->numReg_ * sizeof(uint16_t));
		// this->minReg_ = (uint16_t*) k_malloc(this->numReg_);
		// memset(this->minReg_, 0xff, this->numReg_ * sizeof(uint16_t));
		// this->maxReg_ = (uint16_t*)k_malloc(this->numReg_);
		// memset(this->maxReg_, 0, this->numReg_ * sizeof(uint16_t));

		this->lastReg_ = new uint16_t[this->numReg_];
		memset(this->lastReg_, 0, this->numReg_ * sizeof(uint16_t));
		this->minReg_ = new uint16_t[this->numReg_];
		memset(this->minReg_, 0xff, this->numReg_ * sizeof(uint16_t));
		this->maxReg_ = new uint16_t[this->numReg_];
		memset(this->maxReg_, 0, this->numReg_ * sizeof(uint16_t));

		k_mutex_init(&mutex_);

	}
	int deviceIsReady() {
		if (!device_is_ready(this->i2cdev_)) {
			printf("I2C device is not ready\n");
			return 0;
		}
		return 1;
	}

	~PowerSupply() {
		delete[] lastReg_;
		delete[] minReg_;
		delete[] maxReg_;

		// free(lastReg_);
		// free(minReg_);
		// free(maxReg_);
	}

	int readEEPROM() {
		uint8_t data[256];

		// k_mutex_lock(&this->mutex_, K_FOREVER);
		if(k_mutex_lock(&this->mutex_, K_MSEC(100))) {
			printf("i2c mutex lock failed\n");
			return -1;
		}

		if (i2c_burst_read(this->i2cdev_, this->EEaddress_, 0, data, 256)) {
			printf("Failed to read EEPROM\n");
			k_mutex_unlock(&this->mutex_);
			return -2;
		}

		k_mutex_unlock(&this->mutex_);
		printf("%02x", data[0]);
		for (int i = 1; i < 256; i++) {
			printf(" %02x", data[i]);
		}
		printf("\n");
		return 0;
	}

	int readVar(uint8_t address, uint8_t* writeInts, uint32_t writeLen, uint8_t* data, uint32_t readCount) {
		struct i2c_msg msgs[] = {
			{
				.buf = writeInts,
				.len = writeLen,
				.flags = I2C_MSG_WRITE,
			},
			{
				.buf = data,
				.len = readCount,
				.flags = I2C_MSG_READ | I2C_MSG_RESTART | I2C_MSG_STOP,
			}
		};
		// k_mutex_lock(&this->mutex_, K_FOREVER);
		if(k_mutex_lock(&this->mutex_, K_MSEC(100))) {
			printf("i2c mutex lock failed\n");
			return -1;
		}
		if (i2c_transfer(this->i2cdev_, &msgs[0], 2, address)) {
			printf("Failed to read variable\n");
			k_mutex_unlock(&this->mutex_);
			return -1;
		}

		k_mutex_unlock(&this->mutex_);

		return 0;
	}

	int writeVar(uint8_t address, uint8_t* data, uint32_t len) {
		uint8_t reg = data[0];
		uint8_t *buf = &data[1];

		// k_mutex_lock(&this->mutex_, K_FOREVER);
		if(k_mutex_lock(&this->mutex_, K_MSEC(100))) {
			printf("i2c mutex lock failed\n");
			return -1;
		}

		if (i2c_burst_write(this->i2cdev_, address, reg, (uint8_t *)buf, len - 1)) {
			k_mutex_unlock(&this->mutex_);
			return -1;
		}
		k_mutex_unlock(&this->mutex_);

		return 0;
	}

	int readDPS1200(uint8_t reg, uint8_t* data, uint32_t count) {
		uint8_t cs = reg + (this->address_ << 1);
		uint8_t regCS = ((0xff - cs) + 1) & 0xff;

		uint8_t writeInts[] = { reg, regCS };
		return this->readVar(this->address_, writeInts, 2, data, count);
	}

	int readDPS1200Register(uint8_t reg, int *value) {
		uint8_t data[3];
		int ret = this->readDPS1200(reg<<1, data, 3);	 // if low bit set returns zeros (so use even # cmds)
		if(ret < 0) {
			return -1;
		}

		int replyCS = 0;
		for(int i=0; i < sizeof(data); i++) {
			replyCS+=data[i];
		}

		replyCS=((0xff-replyCS)+1)&0xff;	//check reply checksum (not really reqd)
		if(replyCS!=0) {
			printf("Read error");
			return -2;
		}
		*value=data[0] | data[1]<<8;
		return 0;

	}

	int writeDPS1200(uint8_t reg, int value) {
		uint8_t valLSB = value&0xff;
		uint8_t valMSB=value>>8;
		uint8_t cs=(this->address_<<1)+reg+valLSB+valMSB;
		uint8_t regCS=((0xff-cs)+1)&0xff;
		uint8_t writeInts[] = {reg,valLSB,valMSB,regCS};
		return this->writeVar(this->address_, writeInts, 4);
	}

	int forceFanRPM(int rpm) {
		this->writeDPS1200(0x40,rpm);
		return 0;
	}

 private:
	const struct device *i2cdev_;
	int i2cbus_;
	int address_;
	int EEaddress_;
	int numReg_;
	uint16_t* lastReg_;
	uint16_t* minReg_;
	uint16_t* maxReg_;
	struct k_mutex mutex_;
};

static PowerSupply ps=PowerSupply(0, 7);

static int bl_init()
{
	uint32_t period = PWM_USEC(10);
	int ret;

	printk("bl_init\n");
	printk("PWM-based backlight\n");

	if (!device_is_ready(pwm_led1.dev)) {
		printk("Error: PWM device %s is not ready\n",
					 pwm_led1.dev->name);
		return -1;
	}

	ret = pwm_set_dt(&pwm_led1, period, period / 2);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return -3;
	}

	return 0;
}

static int set_backlight(uint8_t brightness) {
	uint32_t period = PWM_USEC(100);
	int ret;

	if (!device_is_ready(pwm_led1.dev)) {
		printk("Error: PWM device %s is not ready\n",
					 pwm_led1.dev->name);
		return -1;
	}

	ret = pwm_set_dt(&pwm_led1, period, period * brightness / 255);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return -2;
	}

	return 0;
}

/* size of stack area used by each thread */
#define STACKSIZE 1024
static K_THREAD_STACK_DEFINE(psu_mon_stack, STACKSIZE);
static struct k_thread psu_mon_thread;

static K_THREAD_STACK_DEFINE(led_task_stack, STACKSIZE);
static struct k_thread led_thread;

static K_THREAD_STACK_DEFINE(pwm_led_stack, STACKSIZE);
static struct k_thread pwm_led_thread;

static void led_entry(void *p1, void *p2, void *p3)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return ;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}

struct electricity_msg {
  float volts;
  float amps;
  float watts;
  float energy;
};

ZBUS_CHAN_DECLARE(electricity_chan);
ZBUS_OBS_DECLARE(electricity_sub);

ZBUS_CHAN_DEFINE(electricity_chan, struct electricity_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));
#define CONFIG_ELECTRICITY_MSG_SUB_QUEUE_SIZE 4
ZBUS_SUBSCRIBER_DEFINE(electricity_sub, CONFIG_ELECTRICITY_MSG_SUB_QUEUE_SIZE);

static void psu_mon(void *p1, void *p2, void *p3)
{

	// ps.readEEPROM();
	float Gain=1.14;
	uint32_t start_time_ms = k_uptime_get();
  float volts = 0;
  float amps = 0;
  float watts = 0;
  float energy = 0;

	while(1) {
		int val;
		int ret;

		ret=ps.readDPS1200Register(7, &val);
		if(!ret) {
			volts = val;
			volts = volts*Gain/253.9;
		}
		ret=ps.readDPS1200Register(8, &val);
		if(!ret) {
			amps = val;
			amps = amps/64.0;
		}

		ret=ps.readDPS1200Register(9, &val);
		if(!ret) {
			watts = val;
		}
		uint32_t end_time_ms = k_uptime_get();
		uint32_t elapsed_time = end_time_ms - start_time_ms;
		start_time_ms = end_time_ms;

		float power = volts * amps;
		energy += power * elapsed_time / 1000.0 / 3600;

    struct electricity_msg msg;
    msg.volts = volts;
    msg.amps = amps;
    msg.watts = watts;
    msg.energy = energy;

		ret = zbus_chan_pub(&electricity_chan, &msg, K_NO_WAIT);
		if (ret) {
			LOG_ERR("Failed to publish button msg, ret: %d", ret);
		}
		k_sleep(K_MSEC(500));
	}
}

#define max(a,b) ((a) >= (b) ? (a) : (b))

static int format_val(float num, char *buf) {
	if(num <= 0) {
		sprintf(buf, "%s", "00.00");
	} else if(0 < num && num < 1) {
		sprintf(buf, "%.3f", num);
	} else {
		sprintf(buf, "%.*f", max(0, 3 - (int)(floor(log10(abs(num))))), num);
	}

	return 0;
}

static const struct pwm_dt_spec red_pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led));
static const struct pwm_dt_spec green_pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led));
static const struct pwm_dt_spec blue_pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led));

#define STEP_SIZE PWM_USEC(2000)

int pwm_rgb_led_init(void)
{
	uint32_t pulse_red = PWM_MSEC(20), pulse_green = PWM_MSEC(20), pulse_blue = PWM_MSEC(20); /* pulse widths */
	int ret;

	printk("PWM-based RGB LED control\n");

	if (!device_is_ready(red_pwm_led.dev) ||
	    !device_is_ready(green_pwm_led.dev) ||
	    !device_is_ready(blue_pwm_led.dev)) {
		printk("Error: one or more PWM devices not ready\n");
		return 0;
	}

  ret = pwm_set_pulse_dt(&red_pwm_led, pulse_red);
  if (ret != 0) {
    printk("Error %d: red write failed\n", ret);
    return 0;
  }

  ret = pwm_set_pulse_dt(&green_pwm_led, pulse_green);
  if (ret != 0) {
    printk("Error %d: green write failed\n", ret);
    return 0;
  }
  ret = pwm_set_pulse_dt(&blue_pwm_led, pulse_blue);
  if (ret != 0) {
    printk("Error %d: blue write failed\n", ret);
    return 0;
  }
	return 0;
}

static int pwm_led_init()
{
	uint32_t period = PWM_SEC(500);
	int ret;

	printk("PWM-based blinky\n");

	if (!device_is_ready(pwm_led0.dev)) {
		printk("Error: PWM device %s is not ready\n",
					 pwm_led0.dev->name);
		return -1;
	}

	ret = pwm_set_dt(&pwm_led0, period, period / 2U);
	if (ret) {
		printk("Error %d: failed to set pulse width\n", ret);
		return -2;
	}
	return 0;
}

/* Handle button activity */
static void button_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;

	while (1) {
		ret = zbus_sub_wait(&button_sub, &chan, K_FOREVER);

		struct button_msg msg;

		ret = zbus_chan_read(chan, &msg, K_MSEC(100));

		LOG_DBG("Got btn evt from queue - id = %d, action = %d", msg.button_pin,
			msg.button_action);

		if (msg.button_action != BUTTON_PRESS) {
			LOG_WRN("Unhandled button action");
			return;
		}

		switch (msg.button_pin) {
		case BUTTON_A:
      fanRPM += 1000;
      ps.forceFanRPM(fanRPM);
			break;
		case BUTTON_B:
      fanRPM -= 1000;
      ps.forceFanRPM(fanRPM);
			break;
		case BUTTON_X:
      brightness += 2;
      set_backlight(brightness);
			break;
		case BUTTON_Y:
      brightness -= 2;
      set_backlight(brightness);
			break;
		default:
			printf("Unexpected/unhandled button id: %d\n", msg.button_pin);
		}
	}
}


int main(void)
{
	int ret;
	const struct device *display_dev;

	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Device not ready, aborting test");
		return -1;
	}

  pwm_rgb_led_init();
	// pwm_led_init();
	bl_init();
	ui_init();
	lv_task_handler();
	display_blanking_off(display_dev);


  ret = zbus_chan_add_obs(&button_chan, &button_sub, K_MSEC(200));
  ret = zbus_chan_add_obs(&electricity_chan, &electricity_sub, K_MSEC(200));

	k_thread_create(&led_thread, led_task_stack, STACKSIZE, led_entry, NULL, NULL,
									NULL, K_PRIO_COOP(10), 0, K_NO_WAIT);
	// k_thread_create(&psu_mon_thread, psu_mon_stack, STACKSIZE, psu_mon, NULL, NULL,
	// 								NULL, K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
	k_thread_create(&psu_mon_thread, psu_mon_stack, STACKSIZE, psu_mon, NULL, NULL,
									NULL, K_PRIO_PREEMPT(0), 0, K_MSEC(1));

  button_handler_init();
	button_msg_sub_thread_id =
		k_thread_create(&button_msg_sub_thread_data, button_msg_sub_thread_stack,
				CONFIG_BUTTON_MSG_SUB_STACK_SIZE,
				(k_thread_entry_t)button_msg_sub_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(button_msg_sub_thread_id, "BUTTON_MSG_SUB");

	char buf[32];
	const struct zbus_channel *chan;
  float volts = 0;
  float amps = 0;
  float watts = 0;
  float energy = 0;

	while (1) {
		ret = zbus_sub_wait(&electricity_sub, &chan, K_FOREVER);
		struct electricity_msg msg;
		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
    if (ret == 0) {
      volts = msg.volts;
      amps = msg.amps;
      watts = msg.watts;
      energy = msg.energy;

      format_val(volts, buf);
      lv_label_set_text(ui_LabelVoltage, buf);

      format_val(amps, buf);
      lv_label_set_text(ui_LabelCurrent, buf);

      format_val(watts, buf);
      lv_label_set_text(ui_LabelPower, buf);

      if(energy >= 1000) {
        format_val(energy/1000, buf);
        lv_label_set_text(ui_LabelEnergy, buf);
        lv_label_set_text(ui_Label8, "kWh");
      } else {
        format_val(energy, buf);
        lv_label_set_text(ui_LabelEnergy, buf);
        lv_label_set_text(ui_Label8, "Wh");
      }
      lv_task_handler();
      // k_sleep(K_MSEC(100));
    }
	}

}
