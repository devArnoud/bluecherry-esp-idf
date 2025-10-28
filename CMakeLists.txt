idf_component_register(
    SRCS "src/bluecherry.c"
    INCLUDE_DIRS "src"
    REQUIRES bootloader_support spi_flash app_update fatfs vfs esp_timer
    PRIV_REQUIRES esp_netif mbedtls esp_partition
)