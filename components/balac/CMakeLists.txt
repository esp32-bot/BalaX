idf_component_register(
    SRCS "motors.cpp"
    INCLUDE_DIRS "include"
    REQUIRES m5stickc_plus esp_driver_i2c
    PRIV_REQUIRES freertos)
