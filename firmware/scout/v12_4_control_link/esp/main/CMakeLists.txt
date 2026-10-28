idf_component_register(SRCS "app_main.c" "sensors.c" "control.c" "actuators.c" "transport.c" "usb_tx.c" "wire.c" "capture.c" "protocol.c" "network.c" "urgent.c" "settings.c"
    INCLUDE_DIRS "." ".."
    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_driver_pcnt
        esp_driver_usb_serial_jtag esp_adc esp_timer esp_partition esp_wifi esp_netif nvs_flash lwip json icm45605)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
