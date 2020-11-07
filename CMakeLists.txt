# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(airmon LANGUAGES C)

set(PROJECT_ELF ${project_elf})

# Ignore false clang warnings about `struct foo = { 0 }`
target_compile_options(${PROJECT_ELF} PRIVATE -Wno-missing-braces -Wmissing-field-initializers)

