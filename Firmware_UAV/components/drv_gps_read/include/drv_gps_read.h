#ifndef COMMON_DRV_GPS_READ_H  // Include Guard
#define COMMON_DRV_GPS_READ_H

#include <stdint.h>
#include <stdbool.h>
#include "sys_defs.h"

void gnss_init(int uart_num, int tx_pin, int rx_pin, int baud_rate);

gps_data_t gnss_get_data(void);

#endif // COMMON_DRV_GPS_READ_H