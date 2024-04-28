#ifndef ENCODER_H
#define ENCODER_H

#include <cstdint>

#include <Arduino.h>
#include <driver/pcnt.h>

class Encoder
{
public:
    Encoder( pcnt_unit_t unit, int pin );

    static void setup();

    void waitForPulses( uint32_t count, void ( *f )( Encoder * ) );

    void reset();
    [[nodiscard]] uint32_t getPulses() const;
    [[nodiscard]] uint32_t getLimit() const { return pulsesLimit; }
    [[nodiscard]] pcnt_unit_t getUnit() const { return unit; }

private:
    static constexpr int MAX_ENCODERS = 2;

    typedef struct {
        int unit;
        uint32_t status;
    } pcnt_evt_t;

    static inline Encoder *encoders[ MAX_ENCODERS ]{};
    static inline pcnt_isr_handle_t isrHandle = nullptr;
    static inline QueueHandle_t evtQueue;
    static inline uint8_t evtQueueStorageBuffer[ 10 * sizeof( pcnt_evt_t ) ];
    static inline StaticQueue_t evtQueueBuffer;
    static inline StaticTask_t taskBuffer;
    static inline StackType_t taskStack[ 8192 ];
    pcnt_unit_t unit;
    uint32_t overflowCounter = 0;
    uint32_t pulsesLeft = 0;
    uint32_t pulsesLimit = 0;
    void ( *onCountReached )( Encoder * ) = nullptr;

    [[noreturn]] static void encodersTask( [[maybe_unused]] void *arg );
    static void IRAM_ATTR interruptHandler( [[maybe_unused]] void *arg );
    void handleEvent( uint32_t status );
};

#endif /* ENCODER_H */