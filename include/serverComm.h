#ifndef SERVERCOMM_H
#define SERVERCOMM_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

class ServerComm
{
public:
    static void setup();

    static void log( const char *format, ... )  __attribute__ ((format (printf, 1, 2)));

    static void sendLIDARData( uint8_t *data, size_t size );
    static void sendLIDARFlush() { writeEmptyPacket( PacketType::LIDARFlush ); }
    static void sendIMUData( uint8_t *data, size_t size );
    static void sendStopped( bool byObstacle ) { writeEmptyPacket( byObstacle ? PacketType::StoppedByObstacle : PacketType::Stopped ); }
    static void sendRotation( float angle );

    static void waitForLIDARScan();

private:
    enum class PacketType
    {
        LIDAR = 0,
        IMU = 1,
        LIDARFlush = 2,
        Stopped = 3,
        StoppedByObstacle = 4,
        Rotation = 5,
        ESPFirmwareGotPacket = 252,
        ESPFirmwareStop = 253,
        ESPReady = 254,
        LOG = 255,
    };

    static inline SemaphoreHandle_t uartSemaphore = nullptr;
    static inline StaticSemaphore_t uartMutexBuffer;
    static inline bool rpiReady = false;
    static inline std::atomic<TaskHandle_t> taskToNotifyForLIDAR = nullptr;

    [[noreturn]] static void task( [[maybe_unused]] void *arg );

    static int vlog( const char *format, va_list arg );

    static void writePacket( const void *data, size_t size, bool force = false );
    static void writeEmptyPacket( PacketType msg );

    static void firmwareUpdate();
};

#endif /* SERVERCOMM_H */