#ifndef MOTOR_H
#define MOTOR_H

enum class Direction { Forwards, Backwards, CW, CCW };

template <typename T> T sign(T val)
{
    return (static_cast<T>(0) <= val) - (val < static_cast<T>(0));
}

class Encoder;

class Motor
{
public:
    Motor( int pwmChannel, int pinEnable, int pin1, int pin2 ) : pwmChannel{ pwmChannel }, pinEnable{ pinEnable }, pin1{ pin1 }, pin2{ pin2 }
    {
    }

    void setup( Encoder& enc );

    void stop();
    
    void run( float speed );
    void changeSpeed( float speed );
    void setTargetSpeed( float speed );
    void update();

    [[nodiscard]] bool isRunning() const { return ( currentSpeed != 0 ) || isAccelerating(); }
    [[nodiscard]] bool isGoingForward() const { return currentSpeed > 0; }
    [[nodiscard]] bool isAccelerating() const { return currentSpeed != targetSpeed; }
    [[nodiscard]] float getSpeed() const { return currentSpeed; };
    [[nodiscard]] float getActualSpeed() const { return actualSpeed; };
    [[nodiscard]] float getDirection() const { return sign( targetSpeed ); };

private:
    int pwmChannel;
    int pinEnable;
    int pin1;
    int pin2;
    Encoder *encoder = nullptr;
    float currentSpeed = 0;
    float targetSpeed = 0;
    unsigned long accelerationStartTime = 0;
    unsigned long speedometerTime = 0;
    uint32_t speedometerPulses = 0;
    float actualSpeed = 0;
};

#endif