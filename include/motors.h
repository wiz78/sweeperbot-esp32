#ifndef MOTORS_H
#define MOTORS_H

#include "motor.h"
#include "encoder.h"
#include "pins.h"

class Motors
{
public:
    static void setup();
   
    static void stop( bool byObstacle );
    static void move( int mm );
    static void rotate( int direction, float angle );

    static constexpr double toRadians( double deg ) { return deg * M_PI / 180; }

private:
    enum class MotorCmd { Stop, Move, Rotate, NextRotationStep };

    struct motorCmdMsg
    {
        MotorCmd cmd;
        union {
            int direction;
            int distance;
            Encoder *encoder;
        };
        bool byObstacle;
        float angle;
    };

    static inline Motor leftMotor{ 0, PIN_MOTOR1_ENABLE, PIN_MOTOR1_PIN1, PIN_MOTOR1_PIN2 };
    static inline Encoder leftEncoder{ PCNT_UNIT_0, PIN_MOTOR1_ENCODER };
    static inline Motor rightMotor{ 1, PIN_MOTOR2_ENABLE, PIN_MOTOR2_PIN1, PIN_MOTOR2_PIN2 };
    static inline Encoder rightEncoder{ PCNT_UNIT_1, PIN_MOTOR2_ENCODER };
    static inline QueueHandle_t commandsQueue;
    static inline uint8_t cmdQueueStorageBuffer[ 2 * sizeof( motorCmdMsg ) ];
    static inline StaticQueue_t cmdQueueBuffer;
    static inline StaticTask_t taskBuffer;
    static inline StackType_t taskStack[ 8192 ];
    static inline float startYaw;
    static inline int rotationDirection = 0;
    static inline uint32_t targetRotationPulses = 0;
    static inline float targetRotationAngle = 0;
    static inline float currentRotationAngle = 0;
    static inline bool rotateUsingGyro = false;
    static inline bool movingStraight = false;
    static inline float previousMotionError = 0;
    static inline float totalMotionError = 0;

    [[noreturn]] static void motorTask( void *arg );

    static void handleMove( int distance );
    static void straightPID();

    static void checkStatus();

    static void stopAndSendMsg( bool byObstacle = false );

    static void handleRotate( const struct motorCmdMsg& msg );
    static void startRotating( float angle );
    static void sendNextRotationStepMsg( Encoder *encoder );
    static void nextRotationStep( Encoder *encoder );

    static constexpr float angleDiff( float a, float b )
    {
        float diff = fabs( a - b );

        if( diff > M_PI )
            diff = M_PI + M_PI - diff;

        return diff;
    }

    static constexpr float toDegrees( float rad ) { return static_cast<float>( rad * 180 / M_PI ); }

    static void stopFromEncoder( Encoder *encoder );
};

#endif