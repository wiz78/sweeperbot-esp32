#include "imu.h"
#include "motors.h"
#include "serverComm.h"
#include "pins.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

void IMU::setup()
{
    static StaticTask_t taskBuffer;
    static StackType_t stack[ 4096 ];
    uint8_t devStatus;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    qSemaphore = xSemaphoreCreateMutexStatic( &qMutexBuffer );

    pinMode( PIN_IMU_INTERRUPT, INPUT_PULLUP );
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

#if IMU_CALIBRATION
    Serial.printf("devStatus = %d\n", devStatus);
#endif

    mpu.setXGyroOffset(-42);
    mpu.setYGyroOffset(-7);
    mpu.setZGyroOffset(-36);
    mpu.setXAccelOffset(1208);
    mpu.setYAccelOffset(1996);
    mpu.setZAccelOffset(1008);

    // make sure it worked (returns 0 if so)
    if( devStatus == 0 ) {

        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(30);
        mpu.CalibrateGyro(30);
        mpu.CalibrateAccel(30);
        mpu.CalibrateGyro(30);

#if IMU_CALIBRATION
        Serial.println();
        mpu.PrintActiveOffsets();

        Serial.println(F("\n\n[Press Any Key]"));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again
#endif

         mpu.setDMPEnabled( true );

        attachInterrupt( digitalPinToInterrupt( PIN_IMU_INTERRUPT ), mpuInterrupt, RISING );
    }

    taskHandle = xTaskCreateStatic( task, "IMU_task", sizeof( stack ) / sizeof( stack[0] ), NULL, 3, stack, &taskBuffer );
}

void IMU::mpuInterrupt()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR( taskHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

[[noreturn]] void IMU::task( [[maybe_unused]] void *arg )
{
    for(;;) {
        xTaskNotifyWait( 0, 0, nullptr, portMAX_DELAY );

        if( mpu.getFIFOCount() >= mpu.dmpGetFIFOPacketSize() ) {
            unsigned long now = millis();

            mpu.dmpGetCurrentFIFOPacket( fifoBuffer );

            xSemaphoreTake( qSemaphore, portMAX_DELAY );
            mpu.dmpGetQuaternion( &q, fifoBuffer );
            xSemaphoreGive( qSemaphore );

            if( lastDataTimestamp ) {
                VectorFloat gravity;
                VectorInt16 aa;
                VectorInt16 accel;
                uint32_t accelMag;
    
                mpu.dmpGetGravity( &gravity, &q );
                mpu.dmpGetAccel( &aa, fifoBuffer );
                mpu.dmpGetLinearAccel( &accel, &aa, &gravity );

                accelMag = ( accel.x * accel.x ) + ( accel.y * accel.y );

                if( accelMag > maxAccelMag ) {
                    const float projection = Vector2d( -static_cast<float>( accel.x ), -static_cast<float>( accel.y )).project( motionVector );
                    
                    maxAccelMag = accelMag;
                    maxAccel = accel;

                    if(( motionVector.x() || motionVector.y() ) && ( projection > 2000 ))
                        ServerComm::log( "accelMag = %d (%hd, %hd, %hd), motion = (%f, %f), proj = %f",
                                         accelMag, accel.x, accel.y, accel.z, motionVector.x(), motionVector.y(), projection );

                    if( projection > 4000 )
                        Motors::stop( true );
                }

                if( now - lastTelemetryTimestamp >= 250 ) {
                    struct __attribute__((packed))
                    {
                        int8_t packetType;
                        float euler[3];
                        int16_t accelX;
                        int16_t accelY;
                        int16_t accelZ;
                    } data;

                    mpu.dmpGetEuler( data.euler, &q );

                    data.accelX = maxAccel.x;
                    data.accelY = maxAccel.y;
                    data.accelZ = maxAccel.z;

                    ServerComm::sendIMUData( reinterpret_cast<uint8_t *>( &data ), sizeof( data ));

                    lastTelemetryTimestamp = now;
                    maxAccelMag = 0;
                }
            }

            lastDataTimestamp = now;
        }

#if IMU_CALIBRATION
        dump();
#endif
    }
}

#if IMU_CALIBRATION
void IMU::dump()
{
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[2] * 180 / M_PI);

    Serial.print("  ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);

    Serial.print("  a\t");
    Serial.print(aa.x);
    Serial.print("\t");
    Serial.print(aa.y);
    Serial.print("\t");
    Serial.print(aa.z);

    Serial.print("  areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
}
#endif

void IMU::getData( float orientation[3] )
{
    mpu.dmpGetEuler( orientation, &q );
}
