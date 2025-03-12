
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/time.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "driver/gpio.h"
#include "config.h"


void app_main(void)
{
    printf("Hello 米醋!\n");

    config_init();
    if (hardware_init() == false)
    {
        while (1);
    }
    
    task_init();

}
