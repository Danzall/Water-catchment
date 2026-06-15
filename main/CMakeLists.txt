idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES json esp_driver_mcpwm esp_driver_pcnt esp_driver_gpio mqtt nvs_flash esp_netif driver esp_wifi
                       INCLUDE_DIRS ".")
