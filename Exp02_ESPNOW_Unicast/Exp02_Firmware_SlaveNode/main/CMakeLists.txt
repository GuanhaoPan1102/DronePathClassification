idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES espnow_experiment_defs drv_gnss_sync freertos esp_wifi nvs_flash driver)
