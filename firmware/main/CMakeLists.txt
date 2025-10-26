set(src app_main.c
        time.c
        mqtt.c
        state.c
        pms.c
        measurement.c
        dallas.c
        net.c
        co2.c
        ram.c
        crc16.c
        temp.c
        timer.c
        queue.c)

set(mod esp_driver_gpio
        esp_driver_tsens
        esp_driver_uart
        esp_wifi
        mqtt
        onewire_bus)

idf_component_register(
        SRCS ${src}
        PRIV_REQUIRES ${mod}
        INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)

