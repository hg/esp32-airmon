idf_component_register(
	SRCS "app_main.cc"
	INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)

