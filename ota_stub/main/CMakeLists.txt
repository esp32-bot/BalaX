idf_component_register(
    SRCS "ota_stub.c"
    INCLUDE_DIRS "." "../../main"
    PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash
                  esp_driver_gpio esp_driver_i2c esp_driver_spi
                  esp_timer app_update bootloader_support esp_partition lwip
)

if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    idf_build_get_property(python PYTHON)
    idf_build_get_property(project_dir PROJECT_DIR)
    set(generated "${CMAKE_CURRENT_BINARY_DIR}/generated")
    add_custom_target(stub_wifi_networks
        COMMAND "${python}" "${project_dir}/../tools/wifi.py" c-header "${generated}/stub_wifi.h"
        BYPRODUCTS "${generated}/stub_wifi.h"
        VERBATIM)
    add_dependencies(${COMPONENT_LIB} stub_wifi_networks)
    target_include_directories(${COMPONENT_LIB} PRIVATE "${generated}")
endif()
