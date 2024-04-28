#include "lidar.h"
#include "serverComm.h"

#include <hal/uart_types.h>
#include <driver/uart.h>

static constexpr uint32_t LIDAR_TASK_STACK_SIZE = 4096;
static constexpr uart_port_t uart_num = UART_NUM_0;

void Lidar::setup()
{
    static StaticTask_t lidarTaskBuffer;
    static StackType_t lidarStack[ LIDAR_TASK_STACK_SIZE ];
    static constexpr int uart_buffer_size = sizeof( LiDARFrame ) * 4;
    static const uart_config_t uart_config = {
        .baud_rate = 230400,
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

    ESP_ERROR_CHECK( uart_driver_install( uart_num, uart_buffer_size, 0, 0, nullptr, intr_alloc_flags ));
    ESP_ERROR_CHECK( uart_param_config( uart_num, &uart_config ));
    ESP_ERROR_CHECK( uart_set_pin( uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ));

    resume();

    //xTaskCreateStatic( task, "LIDAR_task", LIDAR_TASK_STACK_SIZE, NULL, 2, lidarStack, &lidarTaskBuffer );
    xTaskCreateStaticPinnedToCore( task, "LIDAR_task", LIDAR_TASK_STACK_SIZE, nullptr, 2, lidarStack, &lidarTaskBuffer, 1 );
}

void Lidar::stop()
{
    flushData.store( true, std::memory_order_relaxed );
    running.store( false, std::memory_order_relaxed );
}

void Lidar::resume()
{
    resetDistances();

    flushData.store( false, std::memory_order_relaxed );
    running.store( true, std::memory_order_relaxed );
}

void Lidar::task( [[maybe_unused]] void *arg )
{
    static LiDARFrame frame;

    for(;;) {
        int len = uart_read_bytes( uart_num, &frame, 1, portMAX_DELAY );

        if( len && ( frame.measure.header == LIDAR_HEADER )) {

            len = uart_read_bytes( uart_num, reinterpret_cast<uint8_t *>( &frame ) + 1, 1, portMAX_DELAY );

            if( len == 1 ) {
                size_t  bytesToRead = 0;

                switch( frame.measure.ver_len ) {

                    case LIDAR_DATA_PKG_INFO:
                        bytesToRead = sizeof( frame.measure ) - 2;
                        break;

                    case LIDAR_HEALTH_PKG_INFO:
                        bytesToRead = sizeof( frame.health ) - 2;
                        break;

                    case LIDAR_MANUFACT_PKG_INF:
                        bytesToRead = sizeof( frame.manufacture ) - 2;
                        break;
                }

                if( bytesToRead ) {

                    len = uart_read_bytes( uart_num, reinterpret_cast<uint8_t *>( &frame ) + 2, bytesToRead, portMAX_DELAY );

                    if( running.load( std::memory_order_relaxed )) {

                        if(( len == bytesToRead ) && ( frame.measure.ver_len == LIDAR_DATA_PKG_INFO ))
                            ServerComm::sendLIDARData( reinterpret_cast<uint8_t *>( &frame ), bytesToRead + 2 );

                    } else if( flushData.load( std::memory_order_relaxed )) {

                        flushData.store( false, std::memory_order_relaxed );
                        ServerComm::sendLIDARFlush();
                        resetDistances();
                    }
                }
            }
        }        
    }
}

void Lidar::resetDistances()
{
    for( auto& minDistance : minDistances )
        minDistance.store( 32000, std::memory_order_relaxed );
}
