#include "serverComm.h"
#include "lidar.h"
#include "motors.h"

#include <hal/uart_types.h>
#include <driver/uart.h>
#include <Update.h>

static constexpr uart_port_t uart_num = UART_NUM_2;

static constexpr unsigned char CMD_RPI_READY = 0;
static constexpr unsigned char CMD_ROTATE = 1;
static constexpr unsigned char CMD_MOVE = 2;
static constexpr unsigned char CMD_STOP = 3;
static constexpr unsigned char CMD_UPDATE = 4;
static constexpr unsigned char CMD_MIN_DISTANCES = 5;

void ServerComm::setup()
{
    static StaticTask_t taskBuffer;
    static StackType_t stack[ 4096 ];
    static constexpr int uart_buffer_size = 1024;
    static const uart_config_t uart_config = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK( uart_driver_install( uart_num, uart_buffer_size, uart_buffer_size, 0, nullptr, intr_alloc_flags ));
    ESP_ERROR_CHECK( uart_param_config( uart_num, &uart_config ));
    ESP_ERROR_CHECK( uart_set_pin( uart_num, PIN_UART2_TX, PIN_UART2_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ));

    uartSemaphore = xSemaphoreCreateRecursiveMutexStatic( &uartMutexBuffer );

    esp_log_set_vprintf( vlog );

    xTaskCreateStatic( task, "Comm_task", sizeof( stack ) / sizeof( stack[ 0 ] ), nullptr, 2, stack, &taskBuffer );
}

void ServerComm::task( [[maybe_unused]] void *arg )
{
    unsigned char buffer[ 10 ];

    writeEmptyPacket( PacketType::ESPReady );

    for(;;) {
        int len = uart_read_bytes( uart_num, &buffer, 1, portMAX_DELAY );

        if( len == 1 ) {

            switch( buffer[ 0 ] ) {

                case CMD_RPI_READY:
                    rpiReady = true;
                    log( "Starting transmissions" );
                    break;

                case CMD_ROTATE:
                    if( uart_read_bytes( uart_num, &buffer, 5, portMAX_DELAY ) == 5 ) {
                        float angle;
                        auto *direction = reinterpret_cast<int8_t *>( &buffer[ 0 ] );
                        auto *anglePtr = reinterpret_cast<unsigned int *>( &angle );

                        *anglePtr = buffer[ 1 ] | ( buffer[ 2 ] << 8 ) | ( buffer[ 3 ] << 16 ) | ( buffer[ 4 ] << 24 );

                        Motors::rotate( *direction, angle );
                    }
                    break;

                case CMD_MOVE:
                    if( uart_read_bytes( uart_num, &buffer, 4, portMAX_DELAY ) == 4 )
                        Motors::move( buffer[ 0 ] | ( buffer[ 1 ] << 8 ) | ( buffer[ 2 ] << 16 ) | ( buffer[ 3 ] << 24 ));
                    break;

                case CMD_STOP:
                    Motors::stop( false );
                    break;

                case CMD_UPDATE:
                    firmwareUpdate();
                    break;

                case CMD_MIN_DISTANCES: {
                    uint16_t dists[4];

                    if( uart_read_bytes( uart_num, dists, sizeof( dists ), portMAX_DELAY ) == sizeof( dists ))
                        Lidar::setMinDistances( dists );

                    if( TaskHandle_t task = taskToNotifyForLIDAR.load( std::memory_order_relaxed ))
                        xTaskNotify( task, 0, eNoAction );
                }   break;
            }
        }        
    }
}

void ServerComm::log( const char *format, ... )
{
    va_list arg;

    va_start( arg, format );
    (void)vlog( format, arg );
    va_end( arg );
}

int ServerComm::vlog( const char *format, va_list arg )
{
    static char buffer[ 512 ];
    int ret;

    xSemaphoreTakeRecursive( uartSemaphore, portMAX_DELAY );

    ret = vsnprintf( &buffer[ 1 ], sizeof( buffer ) - 1, format, arg );

    buffer[ 0 ] = static_cast<char>( PacketType::LOG );

    writePacket( reinterpret_cast<uint8_t *>( buffer ), strlen( &buffer[ 1 ] ) + 1 );

    xSemaphoreGiveRecursive( uartSemaphore );

    return ret;
}

void ServerComm::writePacket( const void *data, size_t size, bool force )
{
    if( rpiReady || force ) {
        auto dataLen = static_cast<uint32_t>( size );

        xSemaphoreTakeRecursive( uartSemaphore, portMAX_DELAY );

        uart_write_bytes( uart_num, &dataLen, sizeof( dataLen ));
        uart_write_bytes( uart_num, data, size );

        xSemaphoreGiveRecursive( uartSemaphore );
    }
}

void ServerComm::writeEmptyPacket( PacketType msg )
{
    uint8_t packet[] = { static_cast<uint8_t>( msg ) };

    writePacket( packet, sizeof( packet ), msg == PacketType::ESPReady );
}

void ServerComm::sendLIDARData( uint8_t *data, size_t size )
{
    writePacket( data, size );
}

void ServerComm::sendIMUData( uint8_t *data, size_t size )
{
    *data = static_cast<uint8_t>( PacketType::IMU );

    writePacket( data, size );
}

void ServerComm::sendRotation( float angle )
{
    struct __attribute__((packed))
    {
        uint8_t packetType;
        float angle;
    } data = {
        static_cast<uint8_t>( PacketType::Rotation ),
        angle
    };

    writePacket( &data, sizeof( data ));
}

void ServerComm::firmwareUpdate()
{
    static unsigned char buffer[ 1024 ];

    xSemaphoreTakeRecursive( uartSemaphore, portMAX_DELAY );

    if( uart_read_bytes( uart_num, buffer, 4, portMAX_DELAY ) == 4 ) {
        size_t len = buffer[ 0 ] | ( buffer[ 1 ] << 8 ) | ( buffer[ 2 ] << 16 ) | ( buffer[ 3 ] << 24 );
        bool aborted = false;

        Update.begin( len );

        while( len > 0 ) {
            ssize_t block = uart_read_bytes( uart_num, buffer, std::min( sizeof( buffer ), len ), portMAX_DELAY );

            if( block > 0 ) {

                Update.write( buffer, block );
                writeEmptyPacket( PacketType::ESPFirmwareGotPacket );

                len -= block;

            } else {
                Update.abort();
                len = 0;
                aborted = true;
            }
        }

        writeEmptyPacket( PacketType::ESPFirmwareStop );

        if( aborted )
            log( "Firmware update aborted" );
        else {

            log( "Completing firmware update..." );

            Update.end( true );
        }

        log( "Rebooting" );
        ESP.restart();
    }

    xSemaphoreGiveRecursive( uartSemaphore );
}

void ServerComm::waitForLIDARScan()
{
    TaskHandle_t handle = xTaskGetCurrentTaskHandle();

    xTaskNotifyStateClear( handle );

    taskToNotifyForLIDAR.store( handle, std::memory_order_relaxed );

    xTaskNotifyWait( 0, 0, nullptr, portMAX_DELAY );

    taskToNotifyForLIDAR.store( nullptr, std::memory_order_relaxed );
}
