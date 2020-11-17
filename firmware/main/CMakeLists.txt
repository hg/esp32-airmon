idf_component_register(
        SRCS app_main.cc time.cc mqtt.cc settings.cc state.cc pms.cc measurement.cc dallas.cc net.cc commands.cc
        INCLUDE_DIRS "."
        EMBED_TXTFILES "ca.pem"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)

