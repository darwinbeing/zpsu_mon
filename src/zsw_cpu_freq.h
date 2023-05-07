/*
 * This file is part of ZSWatch project <https://github.com/jakkra/ZSWatch/>.
 * Copyright (c) 2023 Jakob Krantz.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ZSW_CPU_FREQ_H_
#define __ZSW_CPU_FREQ_H_
#include <stdbool.h>

typedef enum zsw_cpu_freq_t {
    ZSW_CPU_FREQ_DEFAULT,
    ZSW_CPU_FREQ_FAST,
} zsw_cpu_freq_t;

void zsw_cpu_set_freq(zsw_cpu_freq_t freq, bool wait);

zsw_cpu_freq_t zsw_cpu_get_freq(void);

#endif // __ZSW_CPU_FREQ_H_

