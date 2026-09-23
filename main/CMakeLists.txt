idf_component_register(
       SRCS
              "main.c"
              "eg915_selftest.c"
              "self_test.c"
              "motion_sensor.c"
              "gnss_module.c"
              "mqtt_module.c"
              "battery_module.c"
              "gpio_module.c"
              "button_module.c"
              "timer_module.c"
       INCLUDE_DIRS ""
)

# Cattle_T 没有 CAN / RS485 硬件，can_module.c 与 rs485_module.c 已从构建中移除。
# pe_original.c 仅作为 pe_board 的历史参考，不参与编译。
