set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_SRCS  src/espnow.c)

set(COMPONENT_REQUIRES driver esp_wifi nvs_flash)

register_component()