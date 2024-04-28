#include "encoder.h"
#include "serverComm.h"

#include <soc/pcnt_reg.h>
#include <soc/pcnt_struct.h>

static constexpr uint16_t MAX_PULSES = 32767;

// https://docs.espressif.com/projects/esp-idf/en/v4.4.7/esp32/api-reference/peripherals/pcnt.html

void Encoder::setup()
{
    evtQueue = xQueueCreateStatic( sizeof( evtQueueStorageBuffer ) / sizeof( pcnt_evt_t ), sizeof( pcnt_evt_t ), evtQueueStorageBuffer, &evtQueueBuffer );

    xTaskCreateStaticPinnedToCore( encodersTask, "encoders_task", sizeof( taskStack ) / sizeof( taskStack[ 0 ] ), nullptr, 5, taskStack, &taskBuffer, 1 );
}

Encoder::Encoder( pcnt_unit_t unit, int pin ) : unit{ unit }
{
    pcnt_config_t config{
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .pos_mode = PCNT_COUNT_INC,
        .counter_h_lim = MAX_PULSES,
        .unit = unit, 
        .channel = PCNT_CHANNEL_0
    };

    if( pcnt_unit_config( &config ) != ESP_OK )
        ServerComm::log( "Couldn't configure PCNT unit %d, pin %d", unit, pin );

    pcnt_set_filter_value( unit, 1000 ); // in 80MHz click ticks
    pcnt_filter_enable( unit );
  
    pcnt_counter_pause( unit );
    pcnt_counter_clear( unit );

    pcnt_event_disable( unit, PCNT_EVT_THRES_0 );
    pcnt_event_disable( unit, PCNT_EVT_THRES_1 );
    pcnt_event_disable( unit, PCNT_EVT_L_LIM );
    pcnt_event_disable( unit, PCNT_EVT_ZERO );
  
    pcnt_event_enable( unit, PCNT_EVT_H_LIM );   
    pcnt_intr_enable( unit );
  
    Encoder::encoders[ unit ] = this;

    if( pcnt_counter_resume( unit ) != ESP_OK )
        ServerComm::log( "Couldn't resume PCNT unit %d, pin %d after setup", unit, pin );
}

void Encoder::encodersTask( [[maybe_unused]] void *arg )
{
    pcnt_isr_register( interruptHandler, nullptr, 0, &isrHandle );

    for(;;) {
        pcnt_evt_t msg;

        if(( xQueueReceive( evtQueue, &msg, portMAX_DELAY ) == pdTRUE ) && encoders[ msg.unit ] )
            encoders[ msg.unit ]->handleEvent( msg.status );
    }
}

void Encoder::interruptHandler( [[maybe_unused]] void *arg )
{
    uint32_t intr_status = PCNT.int_st.val;

    for( int i = 0; i < MAX_ENCODERS; i++ ) {

        if( intr_status & BIT( i )) {
            portBASE_TYPE taskAwoken = pdFALSE;
            pcnt_evt_t evt;

            evt.unit = i;
            evt.status = PCNT.status_unit[ i ].val;
            PCNT.int_clr.val = BIT( i );

            xQueueSendFromISR( evtQueue, &evt, &taskAwoken );
            
            if( taskAwoken == pdTRUE )
                portYIELD_FROM_ISR();
        }
    }
}

void Encoder::handleEvent( uint32_t status )
{
    if( status & PCNT_STATUS_H_LIM_M ) {

        if( pulsesLeft ) {

            pulsesLeft -= std::min( static_cast<uint32_t>( MAX_PULSES ), pulsesLeft );

            if( pulsesLeft <= 0 ) {

                for( auto& encoder : encoders ) {
                    encoder->pulsesLeft = 0;
                    pcnt_set_event_value( encoder->unit, PCNT_EVT_H_LIM, MAX_PULSES );
                }

                ( *onCountReached )( this );

            } else if( pulsesLeft < static_cast<uint32_t>( MAX_PULSES ))
                pcnt_set_event_value( unit, PCNT_EVT_H_LIM, static_cast<int16_t>( pulsesLeft ));

        } else {

            overflowCounter++;

            pcnt_counter_clear( unit );
        }
    }
}

void Encoder::waitForPulses( uint32_t count, void ( *f )( Encoder * ) )
{
    pcnt_counter_pause( unit );
    pcnt_counter_clear( unit );

    overflowCounter = 0;
    pulsesLimit = count;
    pulsesLeft = count;
    onCountReached = f;

    pcnt_set_event_value( unit, PCNT_EVT_H_LIM, static_cast<int16_t>( std::min( count, static_cast<uint32_t>( MAX_PULSES ))));
    pcnt_counter_resume( unit );

    // otherwise the interrupt handler won't be called - not sure why
    pcnt_counter_pause( unit );
    pcnt_counter_clear( unit );
    pcnt_counter_resume( unit );
}

void Encoder::reset()
{
    pcnt_counter_pause( unit );
    pcnt_counter_clear( unit );

    overflowCounter = 0;
    pulsesLimit = 0;

    pcnt_set_event_value( unit, PCNT_EVT_H_LIM, MAX_PULSES );
    pcnt_counter_resume( unit );
}

uint32_t Encoder::getPulses() const
{
    int16_t value;

    if( pcnt_get_counter_value( unit, &value ) == ESP_OK )
        return static_cast<uint32_t>( value ) + ( overflowCounter * 32767U );

    ServerComm::log( "Couldn't get PCNT unit %d value", unit );

    return 0;
}