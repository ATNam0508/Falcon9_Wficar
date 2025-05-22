set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_SRCS  src/imu.c)

set(COMPONENT_REQUIRES driver)

register_component()