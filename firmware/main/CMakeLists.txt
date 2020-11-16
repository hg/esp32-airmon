idf_component_register(
        SRCS "app_main.cc"
        INCLUDE_DIRS "."
        EMBED_TXTFILES "ca.pem"
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)

