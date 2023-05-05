/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BUTTON_HANDLER_H_
#define _BUTTON_HANDLER_H_

#include <stdint.h>
#include <zephyr/drivers/gpio.h>

#define CONFIG_MODULE_BUTTON_HANDLER_LOG_LEVEL 3
#define CONFIG_BUTTON_PUBLISH_STACK_SIZE 1024
#define CONFIG_BUTTON_PUBLISH_THREAD_PRIO 7
#define CONFIG_BUTTON_DEBOUNCE_MS   10

enum button_action {
        BUTTON_PRESS,
        BUTTON_ACTION_NUM,
};

struct button_msg {
        uint32_t button_pin;
        enum button_action button_action;
};

struct btn_config {
	const char *btn_name;
	uint8_t btn_pin;
	uint32_t btn_cfg_mask;
};

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initialize button handler, with buttons defined in button_assignments.h.
 *
 * @note This function may only be called once - there is no reinitialize.
 *
 * @return 0 if successful.
 * @return -ENODEV	gpio driver not found
 */
int button_handler_init(void);

/** @brief Check button state.
 *
 * @param[in] button_pin Button pin
 * @param[out] button_pressed Button state. True if currently pressed, false otherwise
 *
 * @return 0 if success, an error code otherwise.
 */
int button_pressed(gpio_pin_t button_pin, bool *button_pressed);

void button_process(int pin);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* _BUTTON_HANDLER_H_ */
