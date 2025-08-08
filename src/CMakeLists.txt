# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(
    SRCS "main.cpp" # Aquí se incluye tu código fuente principal
    "fondo.cpp"

    # "components/ssd1306/ssd1306_i2c_legacy.c"

    REQUIRES driver # Si el driver es necesario
)