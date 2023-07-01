# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zpsu-mon)

add_compile_definitions(LV_LVGL_H_INCLUDE_SIMPLE)

if(CONFIG_POSIX_API OR CONFIG_PTHREAD_IPC OR CONFIG_POSIX_CLOCK OR
  CONFIG_POSIX_MQUEUE OR CONFIG_POSIX_FS OR CONFIG_EVENTFD OR CONFIG_GETOPT)
  # This is a temporary workaround so that Newlib declares the appropriate
  # types for us. POSIX features to be formalized as part of #51211
  zephyr_compile_options($<$<COMPILE_LANGUAGE:C>:-DPATH_MAX=256>)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/ui
  )

set(app_SRCS
  ${PROJECT_SOURCE_DIR}/src/led.c
  ${PROJECT_SOURCE_DIR}/src/ui/watchface_ui.c
  ${PROJECT_SOURCE_DIR}/src/ui/screens/ui_Screen1.c
  ${PROJECT_SOURCE_DIR}/src/ui/ui_helpers.c
  ${PROJECT_SOURCE_DIR}/src/ui/ui.c
  ${PROJECT_SOURCE_DIR}/src/ui/fonts/ui_font_DSEG7ClassicBold36.c
  ${PROJECT_SOURCE_DIR}/src/watchface_app.c
  ${PROJECT_SOURCE_DIR}/src/psu_ctrl.c
  ${PROJECT_SOURCE_DIR}/src/buttons.c
  ${PROJECT_SOURCE_DIR}/src/button_handler.c
  ${PROJECT_SOURCE_DIR}/src/clock.c
  ${PROJECT_SOURCE_DIR}/src/notification_manager.c
  ${PROJECT_SOURCE_DIR}/src/main.c
  ${PROJECT_SOURCE_DIR}/src/display_control.c
  )

target_sources(app PRIVATE ${app_SRCS})

# add_subdirectory(src/ui)
