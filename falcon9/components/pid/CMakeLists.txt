set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_SRCS  src/pid.c)

set(COMPONENT_REQUIRES driver)

register_component()