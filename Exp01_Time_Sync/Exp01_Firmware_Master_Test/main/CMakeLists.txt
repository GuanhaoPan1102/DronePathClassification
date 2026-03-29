idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES drv_4g_gnss_sync esp_timer freertos esp_wifi nvs_flash driver)
