# The minimum version of CMake that is required to build the project. The line must be the first line.
cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{ESP_MATTER_PATH})
    message(FATAL_ERROR "Please set ESP_MATTER_PATH to the path of esp-matter repo")
endif(NOT DEFINED ENV{ESP_MATTER_PATH})

# The set() commands should be placed after the cmake_minimum() line but before the include() line.
set(PROJECT_VER "1.0")
set(PROJECT_VER_NUMBER 1)
set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)
set(ENV{PATH} "$ENV{PATH}:${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip/.environment/cipd/packages/pigweed")

# Pulls in the rest of the CMake functionality to configure the project, discover all the components, etc.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Include common component dependencies and configurations from ESP-Matter examples
include(${ESP_MATTER_PATH}/examples/common/cmake_common/components_include.cmake)

# Optional list of additional directories to search for components.
set(EXTRA_COMPONENT_DIRS
        "${MATTER_SDK_PATH}/config/esp32/components"
        "${ESP_MATTER_PATH}/components"
        ${extra_components_dirs_append})

# Creates the project named matter_bmp280
project(matter_bmp280)

# WARNING: This is just an example for using key for decrypting the encrypted OTA image
# Please do not use it as is.
if(CONFIG_ENABLE_ENCRYPTED_OTA)
    target_add_binary_data(matter-sensor.elf "esp_image_encryption_key.pem" TEXT)
endif()

# Set the C++ standard to C++17
idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
# For RISCV chips, project_include.cmake sets -Wno-format, but does not clear various
# flags that depend on -Wformat
idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)
# Enable colored output in ninja builds
add_compile_options(-fdiagnostics-color=always -Wno-write-strings)