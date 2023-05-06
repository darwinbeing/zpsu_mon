/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Button assignments
 *
 * Button mappings are listed here.
 *
 */

#ifndef _BUTTON_ASSIGNMENTS_H_
#define _BUTTON_ASSIGNMENTS_H_

#include <zephyr/drivers/gpio.h>

/** @brief List of buttons and associated metadata
 */
enum button_pin_names {
	BUTTON_A = DT_GPIO_PIN(DT_ALIAS(sw0), gpios),
	BUTTON_B = DT_GPIO_PIN(DT_ALIAS(sw1), gpios),
	BUTTON_X = DT_GPIO_PIN(DT_ALIAS(sw2), gpios),
	BUTTON_Y = DT_GPIO_PIN(DT_ALIAS(sw3), gpios),
};

#endif /* _BUTTON_ASSIGNMENTS_H_ */
