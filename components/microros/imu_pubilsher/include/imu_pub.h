#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdbool.h"
#include "stdint.h"

extern proto_data_imu_t proto_imu_data;
void imu_pub_task(void);

#ifdef __cplusplus
}
#endif
