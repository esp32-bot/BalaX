idf_component_register(
    SRCS "main.cpp" "board.cpp" "app_settings.cpp" "gain_settings.cpp" "imu_calibration.cpp"
         "status.cpp" "status_view.cpp" "cpu_stats.cpp" "commands.cpp" "balance.cpp"
         "robot.cpp" "telemetry_stream.cpp" "remote_protocol.cpp" "remote.cpp" "wifi.cpp"
         "prog_vm.cpp" "lan_server.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES m5stickc_plus balac gfx settings console console_commands esp_pm esp_timer esp_hw_support
                  nvs_flash esp_wifi esp_netif esp_event spi_flash esp_http_server app_update
                  bootloader_support esp_partition
                  espcoredump  # nothing calls it, but the minimal build leaves out what isn't required
    # Settings register themselves and nothing refers to them by name, so without this the
    # linker would leave them out.
    WHOLE_ARCHIVE)

# The Wi-Fi networks, from the gitignored secrets/wifi.json (tools/wifi.py). The header is
# regenerated at every build, but only rewritten when it changes, so it only rebuilds then.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    idf_build_get_property(python PYTHON)
    idf_build_get_property(project_dir PROJECT_DIR)
    set(generated "${CMAKE_CURRENT_BINARY_DIR}/generated")
    add_custom_target(wifi_networks
        COMMAND "${python}" "${project_dir}/tools/wifi.py" header "${generated}/wifi_networks.h"
        BYPRODUCTS "${generated}/wifi_networks.h"
        VERBATIM)
    add_dependencies(${COMPONENT_LIB} wifi_networks)
    target_include_directories(${COMPONENT_LIB} PRIVATE "${generated}")
endif()
