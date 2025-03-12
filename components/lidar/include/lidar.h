#ifndef LIDAR_H_
#define LIDAR_H_

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "esp_log.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "driver/uart.h"
#include "UDP_TCP.h"


void  lidar_task(void);

#endif /* COMPONENTS_SENSOR_HMC5883L_HMC5883L_H_ */

