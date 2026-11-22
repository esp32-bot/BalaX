cmake_minimum_required(VERSION 3.22)

# This robot's own settings, not committed: sdkconfig.local overrides sdkconfig.defaults,
# e.g. CONFIG_BALAX_DEFAULT_VOLUME=70.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.local")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.local")
endif()

# Shared esp32-bot components (m5stickc_plus, gfx, settings, console_commands) live in
# ../tools/components when checked out under ~/gh/esp32-bot/.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../tools/components")
    list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../tools/components")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Build only the components main depends on, not every component in ESP-IDF.
idf_build_set_property(MINIMAL_BUILD ON)
project(balax)
