/*
 * imu_task.cpp -- IMU physics and logo animation
 */

#include "globals.h"

void imuTask(void* p) {
    for (;;) {
        float targetAngle = 0.0f;
#if ENABLE_IMU
        if (imuPresent) {
            float ax, ay, az;
            M5.Imu.getAccel(&ax, &ay, &az);
            targetAngle = atan2f(ax, ay);
        }
#endif
        float diff = targetAngle - logoAngle;
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        logoVel   = logoVel * IMU_DAMPING + diff * IMU_SPRING;
        logoAngle += logoVel;

        if (currentScreen == SCREEN_LOGO) drawLogoScreen(logoAngle);

        vTaskDelay(pdMS_TO_TICKS(IMU_POLL_MS));
    }
}
