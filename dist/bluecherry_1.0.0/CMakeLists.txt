idf_component_register(
    SRCS "src/bluecherry.c"
    INCLUDE_DIRS "src"
    PRIV_REQUIRES esp_netif mbedtls
)