# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(test_app_components "$ENV{IDF_PATH}/tools/test_apps/components")
set(legacy_test_app_components "$ENV{IDF_PATH}/tools/unit-test-app/components")
if(EXISTS "${test_app_components}")
    list(APPEND EXTRA_COMPONENT_DIRS "${test_app_components}")
elseif(EXISTS "${legacy_test_app_components}")
    list(APPEND EXTRA_COMPONENT_DIRS "${legacy_test_app_components}")
else()
    message(FATAL_ERROR "ESP-IDF test application components not found under IDF_PATH")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_esp_brookesia)
