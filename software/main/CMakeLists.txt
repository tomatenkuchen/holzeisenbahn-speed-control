file(GLOB_RECURSE srcs "src/*.cpp")

idf_component_register(SRCS "${srcs}"
                       PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_ledc esp_driver_mcpwm esp_adc
                       REQUIRES driver
                       INCLUDE_DIRS "./src")
