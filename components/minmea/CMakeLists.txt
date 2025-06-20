idf_component_register(SRCS "minmea.c"
                       INCLUDE_DIRS ".")

target_compile_definitions(${COMPONENT_LIB} PRIVATE timegm=mktime)