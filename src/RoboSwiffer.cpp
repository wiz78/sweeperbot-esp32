#include <Arduino.h>

#include "motors.h"
#include "serverComm.h"
#include "lidar.h"
#include "imu.h"

void setup()
{
#if IMU_CALIBRATION
    Serial.begin(115200);
#endif
    // this is the first one because it writes to the serial... which is then used by the LIDAR
    IMU::setup();

#if !IMU_CALIBRATION
    ServerComm::setup();
    Motors::setup();
    Lidar::setup();
#endif
}

void loop()
{
    vTaskDelay( 10000 / portTICK_PERIOD_MS );
}
