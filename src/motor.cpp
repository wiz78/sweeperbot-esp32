#include <Arduino.h>

#include "motor.h"
#include "encoder.h"
#include "serverComm.h"

#include <algorithm>

// Motor PWM properties
constexpr int motorResolution = 11;
constexpr int motorMaxPWM = pow( 2, motorResolution );

constexpr int minSpeed = 56;
constexpr int minDuty = ( minSpeed * motorMaxPWM ) / 100;

void Motor::setup( Encoder& enc )
{
    constexpr int pwmFreq = 30000;

    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html#supported-range-of-frequency-and-duty-resolutions
    static_assert( pwmFreq * motorMaxPWM < 80000000 );

    pinMode( pin1, OUTPUT );
    pinMode( pin2, OUTPUT );

    ledcSetup( pwmChannel, pwmFreq, motorResolution );
    ledcAttachPin( pinEnable, pwmChannel );

    stop();

    encoder = &enc;
}

void Motor::stop()
{
    ledcWrite( pwmChannel, 0 );

    // needed before changing other pins
    digitalWrite( pinEnable, LOW );
    digitalWrite( pin1, LOW );
    digitalWrite( pin2, LOW );

    currentSpeed = 0;
    targetSpeed = 0;
}

void Motor::run( float speed )
{
    // needed before changing other pins
    digitalWrite( pinEnable, LOW );

    digitalWrite( pin1, speed > 0 );
    digitalWrite( pin2, speed < 0 );

    changeSpeed( speed );

    speedometerTime = millis();
    speedometerPulses = 0;
    actualSpeed = infinityf();
}

void Motor::changeSpeed( float speed )
{
    speed = std::clamp( speed, -100.0f, 100.0f );

    if( speed != currentSpeed ) {
        const auto rise = static_cast<float>( motorMaxPWM - minDuty );
        int duty = static_cast<int>( std::round( std::fabs( speed ) * rise ) / 100.f ) + minDuty;

        ledcWrite( pwmChannel, duty );

        currentSpeed = speed;
    }
}

void Motor::setTargetSpeed( float speed )
{ 
    if(( speed != currentSpeed ) && ( speed != targetSpeed )) {

        targetSpeed = std::clamp( speed, -100.f, 100.f );
        accelerationStartTime = millis();

        if( currentSpeed == 0 )
            run( getDirection() );
    }
}

void Motor::update()
{
    unsigned long now = millis();
    unsigned long deltaTime;

    if( isAccelerating() ) {

        deltaTime = now - accelerationStartTime;

        if( deltaTime >= 5 ) {
            static constexpr float accel = .1;
            int direction = static_cast<int>( sign( targetSpeed - currentSpeed ));
            float newSpeed = currentSpeed + ( static_cast<float>( deltaTime ) * accel * static_cast<float>( direction ));

            if((( direction < 0 ) && ( newSpeed < targetSpeed )) ||
               (( direction > 0 ) && ( newSpeed > targetSpeed )))
                newSpeed = targetSpeed;

            accelerationStartTime = now;

            if( newSpeed != 0 ) {
                
                if( sign( newSpeed ) != sign( currentSpeed ))
                    run( newSpeed );
                else
                    changeSpeed( newSpeed );

            } else
                stop();
        }
    }

    if( isRunning() ) {

        deltaTime = now - speedometerTime;

        if( deltaTime >= 1000 ) {
            uint32_t pulses = encoder->getPulses();

            actualSpeed = static_cast<float>( pulses - speedometerPulses ) / static_cast<float>( deltaTime );

            speedometerTime = now;
            speedometerPulses = pulses;

            //ServerComm::log( "Motor speed %f", actualSpeed );
        }
    }
}