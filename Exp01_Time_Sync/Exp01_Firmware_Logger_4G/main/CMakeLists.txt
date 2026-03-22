idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES sync_experiment_defs drv_4g_gnss_sync esp_timer freertos esp_wifi nvs_flash driver)
