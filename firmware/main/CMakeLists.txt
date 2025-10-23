set(src app_main.cpp
        time.cpp
        mqtt.cpp
        state.cpp
        pms.cpp
        measurement.cpp
        dallas.cpp
        net.cpp
        co2.cpp
        crc16.cpp
        timer.cpp)

set(mod esp_driver_gpio
        esp_driver_uart
        esp_wifi
        mqtt
        onewire_bus)

idf_component_register(
        SRCS ${src}
        PRIV_REQUIRES ${mod}
        INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers -std=gnu++17)

