#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

#include "vector2d.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

class IMU
{
public:
    static void setup();

    static void getData( float *orientation );
    
    static float getYaw()
    { 
        float ret; 

        xSemaphoreTake( qSemaphore, portMAX_DELAY );

        ret = atan2(( 2 * q.x * q.y ) - ( 2 * q.w * q.z ), ( 2 * q.w * q.w ) + ( 2 * q.x * q.x ) - 1 );

        xSemaphoreGive( qSemaphore );

        return ret;
    }

    static void setMotionVector( const Vector2d& v ) { motionVector = v; }

private:
    static inline MPU6050 mpu;
    static inline uint8_t fifoBuffer[64];
    static inline Quaternion q{ 0, 0, 0, 0 };
    static inline unsigned long lastDataTimestamp = 0;
    static inline unsigned long lastTelemetryTimestamp = 0;
    static inline VectorInt16 maxAccel{ 0, 0, 0 };
    static inline uint32_t maxAccelMag = 0;
    static inline TaskHandle_t taskHandle = nullptr;
    static inline SemaphoreHandle_t qSemaphore = nullptr;
    static inline StaticSemaphore_t qMutexBuffer;
    static inline Vector2d motionVector;

    static void mpuInterrupt();

    [[noreturn]] static void task( void *arg );
    
#if IMU_CALIBRATION
    static void dump();
#endif
};

#endif /* IMU_H */