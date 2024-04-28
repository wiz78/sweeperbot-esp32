#include <Arduino.h>

#include "motors.h"
#include "imu.h"
#include "lidar.h"
#include "serverComm.h"
#include "vector2d.h"

#include <algorithm>
#include <cfloat>

static constexpr double WHEEL_CIRCUMFERENCE = 70 * M_PI;
static constexpr double PULSES_PER_REVOLUTION = 60;
static constexpr double PULSES_PER_MM = PULSES_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;
static constexpr double WHEELBASE = 207;
static constexpr double PULSES_PER_RADIANT = ( WHEELBASE * M_PI * PULSES_PER_MM ) / ( 2 * M_PI );
static constexpr double MIN_ANGLE_TO_ROTATE_VIA_ENCODERS = 0.508421841913725;
static const uint32_t MIN_ROTATION_PULSES = static_cast<uint32_t>( ceil( MIN_ANGLE_TO_ROTATE_VIA_ENCODERS * PULSES_PER_RADIANT ));
static constexpr double ROTATION_INERTIA = Motors::toRadians( 7 );
static constexpr double ROTATION_MIN_ANGLE_WAIT_FOR_LIDAR = Motors::toRadians( 1.5 );
static constexpr float BLOCKED_SPEED_THRESHOLD = 0.018; // experimental value

void Motors::setup()
{
    leftMotor.setup( leftEncoder );
    rightMotor.setup( rightEncoder );
    Encoder::setup();

    commandsQueue = xQueueCreateStatic( sizeof( cmdQueueStorageBuffer ) / sizeof( motorCmdMsg ), sizeof( motorCmdMsg ), cmdQueueStorageBuffer, &cmdQueueBuffer );

    xTaskCreateStaticPinnedToCore( motorTask, "motors_task", sizeof( taskStack ) / sizeof( taskStack[ 0 ] ), nullptr, 4, taskStack, &taskBuffer, 1 );
}

void Motors::motorTask( [[maybe_unused]] void *arg )
{
    motorCmdMsg msg{};

    for(;;) {

        if( xQueueReceive( commandsQueue, &msg, 5 / portTICK_PERIOD_MS )) {

            switch( msg.cmd ) {

                case MotorCmd::Stop:
                    stopAndSendMsg( msg.byObstacle );
                    break;

                case MotorCmd::Move:
                    handleMove( msg.distance );
                    break;

                case MotorCmd::Rotate:
                    handleRotate( msg );
                    break;

                case MotorCmd::NextRotationStep:
                    nextRotationStep( msg.encoder );
                    break;
            }
        }

        checkStatus();

        if( movingStraight && !leftMotor.isAccelerating() && !rightMotor.isAccelerating() )
            straightPID();
    }
}

void Motors::handleMove( int distance )
{
    auto pulses = static_cast<uint32_t>( std::abs( distance ) * PULSES_PER_MM );

    movingStraight = true;
    startYaw = round( IMU::getYaw() * 100 );
    previousMotionError = 0;
    totalMotionError = 0;

    ServerComm::log( "Moving %d mm (%d pulses) - yaw: %f (%f°)", distance, pulses, startYaw / 100, toDegrees( startYaw ));

    leftEncoder.waitForPulses( std::max( 2U, pulses - 5 ), Motors::stopFromEncoder );
    rightEncoder.waitForPulses( std::max( 2U, pulses - 5 ), Motors::stopFromEncoder );
    leftMotor.setTargetSpeed(( distance > 0 ) ? 100 : -100 );
    rightMotor.setTargetSpeed(( distance > 0 ) ? 100 : -100 );

    IMU::setMotionVector( Vector2d( 0, static_cast<float>( sign( distance ))));
}

void Motors::straightPID()
{
    static constexpr float KP = 10;
    static constexpr float KD = 6;
    static constexpr float KI = 2.5;
    float yaw = round( IMU::getYaw() * 100 );
    float direction = leftMotor.getDirection();
    float error = ( startYaw - yaw ) * direction;
    float correction;

    totalMotionError += error;

    correction = ( error * KP ) + (( error - previousMotionError ) * KD ) + ( totalMotionError * KI );

    if( std::fabs( correction ) > 1 ) {
        float leftSpeed = 100, rightSpeed = 100;

        if( correction > 0 )
            rightSpeed -= std::min( correction, 98.0F );
        else
            leftSpeed += std::max( correction, -98.0F );

        leftMotor.changeSpeed( leftSpeed * direction );
        rightMotor.changeSpeed( rightSpeed * direction );

        leftMotor.setTargetSpeed( leftMotor.getSpeed() );
        rightMotor.setTargetSpeed( rightMotor.getSpeed() );
    }

    previousMotionError = error;
}

void Motors::checkStatus()
{
    bool leftRunning = leftMotor.isRunning();
    bool rightRunning = rightMotor.isRunning();

    if( leftRunning || rightRunning ) {
        bool stop = false;
        
        if( leftRunning && rightRunning ) {

            if( rotateUsingGyro ) {

                if( angleDiff( IMU::getYaw(), startYaw ) + ROTATION_INERTIA + currentRotationAngle >= targetRotationAngle )
                    stop = true;

            } else if( leftMotor.isGoingForward() && rightMotor.isGoingForward() ) {
                
                if( Lidar::getMinDistance( 0 ) < 190 ) {
                    stop = true;
                    ServerComm::log( "Motors - stop by proximity (%d)", Lidar::getMinDistance( 0 ));
                }

            } else if( !leftMotor.isGoingForward() && !rightMotor.isGoingForward() ) {
                
                if( Lidar::getMinDistance( 2 ) < 90 ) {
                    stop = true;
                    ServerComm::log( "Motors - stop by proximity (%d)", Lidar::getMinDistance( 2 ));
                }

            /*} else if(( Lidar::getMinDistance( 1 ) < 100 ) || ( Lidar::getMinDistance( 3 ) < 100 )) {
                stop = true;
                ServerComm::log( "Motors - stop by proximity (%d; %d)", Lidar::getMinDistance( 1 ), Lidar::getMinDistance( 3 ));*/
            }
        }

        if(( leftMotor.getActualSpeed() < BLOCKED_SPEED_THRESHOLD ) || ( rightMotor.getActualSpeed() < BLOCKED_SPEED_THRESHOLD )) {
            ServerComm::log( "Motors - blocked? left speed: %f, right speed: %f", leftMotor.getActualSpeed(), rightMotor.getActualSpeed() );
            stopAndSendMsg( true );
        } else if( stop )
            stopAndSendMsg();

        leftMotor.update();
        rightMotor.update();
    }
}

void Motors::stopAndSendMsg( bool byObstacle )
{
    bool resumeLidar = !Lidar::isRunning();

    leftMotor.stop();
    rightMotor.stop();

    if( resumeLidar )
        vTaskDelay( 200 / portTICK_PERIOD_MS );

    if( rotateUsingGyro )
        ServerComm::sendRotation( angleDiff( IMU::getYaw(), startYaw ) * static_cast<float>( rotationDirection ));

    if( resumeLidar ) {

        Lidar::resume();

        if( targetRotationAngle > ROTATION_MIN_ANGLE_WAIT_FOR_LIDAR )
            ServerComm::waitForLIDARScan();
    }

    targetRotationAngle = 0;
    targetRotationPulses = 0;
    rotateUsingGyro = false;
    movingStraight = false;

    IMU::setMotionVector( Vector2d() );

    ServerComm::sendStopped( byObstacle );
}

void Motors::handleRotate( const struct motorCmdMsg& msg )
{
    if( msg.angle > 0 ) {

        rotationDirection = msg.direction;
        targetRotationAngle = msg.angle;
        currentRotationAngle = 0;
        targetRotationPulses = static_cast<uint32_t>( ceil( PULSES_PER_RADIANT * targetRotationAngle ));

        IMU::setMotionVector( Vector2d( static_cast<float>( msg.direction ), 0 ));
        startRotating( msg.angle );

        // give time to actually start the motors (otherwise checkStatus() will stop immediately)
        if( rotateUsingGyro && ( msg.angle <= ROTATION_INERTIA ))
            vTaskDelay( 15 / portTICK_PERIOD_MS );
    }
}

void Motors::startRotating( float angle )
{
    float speed;

    Lidar::stop();

    ServerComm::log( "startRotating( %f [%f°] ) - direction: %d", angle, toDegrees( angle ), rotationDirection );

    if( targetRotationPulses > MIN_ROTATION_PULSES ) {
        uint32_t pulses = targetRotationPulses - MIN_ROTATION_PULSES;

//        ServerComm::log( "rotating up to pulses: %d, target: %d", pulses, targetRotationPulses );

        leftEncoder.waitForPulses( pulses, Motors::sendNextRotationStepMsg );
        rightEncoder.waitForPulses( pulses, Motors::sendNextRotationStepMsg );

        rotateUsingGyro = false;
        speed = ( rotationDirection > 0 ) ? 100 : -100;

    } else {

//        ServerComm::log( "rotating up to angle: %f (%f°), target: %f (%f°)",
//                         angle, toDegrees( angle * static_cast<float>( rotationDirection )), targetRotationAngle,
//                         toDegrees( targetRotationAngle * static_cast<float>( rotationDirection )));

        speed = ( rotationDirection > 0 ) ? 10 : -10;
        rotateUsingGyro = true;
    }

    startYaw = IMU::getYaw();

    leftMotor.setTargetSpeed( speed );
    rightMotor.setTargetSpeed( -speed );
}

void Motors::sendNextRotationStepMsg( Encoder *encoder )
{
    motorCmdMsg msg { .cmd = MotorCmd::NextRotationStep, .encoder = encoder };

    xQueueSendToBack( commandsQueue, &msg, portMAX_DELAY );
}

void Motors::nextRotationStep( Encoder *encoder )
{
    float diff;

    leftMotor.stop();
    rightMotor.stop();

    diff = angleDiff( IMU::getYaw(), startYaw );
    currentRotationAngle += diff;

    ServerComm::sendRotation( diff * static_cast<float>( rotationDirection ));
//    ServerComm::log( "nextRotationStep() - targetRotationPulses: %d, encoder unit: %d, pulses: %d, limit: %d",
//                     targetRotationPulses, encoder->getUnit(), encoder->getPulses(), encoder->getLimit() );

    targetRotationPulses -= std::min( targetRotationPulses, encoder->getLimit() );

//    ServerComm::log( "nextRotationStep() - rotation so far: %f (%f°) - targetRotationPulses: %d", diff, toDegrees( diff ), targetRotationPulses );

    if( targetRotationPulses )
        diff = targetRotationAngle - currentRotationAngle;
    else
        diff = 0;

    if( diff > 0 )
        startRotating( diff );
    else
        stopAndSendMsg();
}

void Motors::stopFromEncoder( Encoder *encoder )
{
    motorCmdMsg msg { .cmd = MotorCmd::Stop, .encoder = encoder };

    xQueueSendToBack( commandsQueue, &msg, portMAX_DELAY );
}

void Motors::stop( bool byObstacle )
{
    motorCmdMsg msg { .cmd = MotorCmd::Stop, .byObstacle = byObstacle };

    if( byObstacle )
        xQueueReset( commandsQueue );

    xQueueSendToBack( commandsQueue, &msg, portMAX_DELAY );
}

void Motors::move( int mm )
{
    motorCmdMsg msg { .cmd = MotorCmd::Move, .distance = std::max( std::abs( mm ), 19 ) * sign( mm ) };

    xQueueSendToBack( commandsQueue, &msg, portMAX_DELAY );
}

void Motors::rotate( int direction, float angle )
{
    motorCmdMsg msg { .cmd = MotorCmd::Rotate, .direction = sign( direction ), .angle = angle };

    xQueueSendToBack( commandsQueue, &msg, portMAX_DELAY );
}
