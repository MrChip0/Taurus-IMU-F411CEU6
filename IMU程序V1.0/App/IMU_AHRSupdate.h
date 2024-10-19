#ifndef __IMU_AHRSUPDATE_H
#define __IMU_AHRSUPDATE_H

#include "BMI088_Read.h"
#include "IST8310_Read.h"
#include "bsp_imu.h"
#include "bsp_bmi088.h"

extern uint8_t angle_tx_data[12];
void IMU_AHRSupdate_task(void);

#endif


