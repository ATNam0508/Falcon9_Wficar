idf_component_register(
    SRCS "src/espnow.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_wifi nvs_flash
)