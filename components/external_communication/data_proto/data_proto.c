
#include "data_proto.h"

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "esp_system.h"
#include "esp_log.h"
#include "string.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "freertos/queue.h"



SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
