idf_component_register(SRCS "drv_ble_scan.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt sys_defs nvs_flash freertos)
