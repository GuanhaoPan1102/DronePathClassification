idf_component_register(SRCS "drv_ble_adv.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt sys_defs drv_gps_read nvs_flash)
