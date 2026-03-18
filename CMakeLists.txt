cmake_minimum_required(VERSION 3.20.0)

# Use the exact absolute path to your cloned module
set(ZEPHYR_EXTRA_MODULES "/home/sudha/zephyer_rtos/modules/HX711_zephyr_driver")

# Boot up Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(hoist_app)
target_sources(app PRIVATE src/main.c)