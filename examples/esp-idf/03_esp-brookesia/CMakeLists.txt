cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp-brookesia)

if(IDF_VERSION_MAJOR GREATER_EQUAL 6)
    idf_component_get_property(esp_boost_lib espressif__esp-boost COMPONENT_LIB)
    target_compile_options(
        ${esp_boost_lib}
        PRIVATE -Wno-error=deprecated-variadic-comma-omission
    )
endif()
