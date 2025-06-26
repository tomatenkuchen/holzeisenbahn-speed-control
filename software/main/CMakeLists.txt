file(GLOB_RECURSE srcs "main.cpp" "src/*.cpp")

idf_component_register(SRCS "${srcs}"
                       PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_ledc
                       REQUIRES driver
                       INCLUDE_DIRS "./include")
