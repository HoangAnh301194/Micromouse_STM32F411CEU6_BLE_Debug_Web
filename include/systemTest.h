/**
 * @file systemTest.h
 * @brief System Test Menu Interface
 * @author HoangAnh & Shinochan
 * @date 2025
 */

#ifndef SYSTEM_TEST_H
#define SYSTEM_TEST_H

#include <stdint.h>
#include "mpu6050.h"
#include "motion_controller.h"
extern Motion_Controller_t motion_ctrl;

/**
 * @brief Run system test menu (infinite loop)
 * @param mpu: Pointer to MPU6050 handle
  * @note This function never returns - contains while(1) loop
 */
void SystemTest_Run(MPU6050_Handle_t *mpu);


#endif /* SYSTEM_TEST_H */
