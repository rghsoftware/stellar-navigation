cmake_minimum_required(VERSION 3.5)
project(CFS_STARNAV C)

set(APP_SRC_FILES
  fsw/src/starnav_app.c
  fsw/src/starnav_device.c
)

add_cfe_app(starnav ${APP_SRC_FILES})

target_include_directories(starnav PUBLIC
  fsw/mission_inc
  fsw/platform_inc
)


