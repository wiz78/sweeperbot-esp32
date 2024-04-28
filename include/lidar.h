#ifndef LIDAR_H
#define LIDAR_H

#include <cmath>
#include <cstring>
#include <atomic>

#define LIDAR_POINT_PER_PACK    12
#define LIDAR_HEADER            0x54
#define LIDAR_DATA_PKG_INFO     0x2C
#define LIDAR_HEALTH_PKG_INFO   0xE0
#define LIDAR_MANUFACT_PKG_INF  0x0F

typedef struct  __attribute__((packed)) {
    uint8_t header; 
    uint8_t information; 
    uint16_t speed;
    uint16_t product_version;   
    uint32_t sn_high;
    uint32_t sn_low;
    uint32_t hardware_version;
    uint32_t firmware_version;
    uint8_t crc8;
} LiDARManufactureInfoType;

typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t intensity;
} LidarPointStructType;

typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructType point[ LIDAR_POINT_PER_PACK ];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LiDARMeasureDataType;

typedef struct __attribute__((packed)) {
    uint8_t  header;
    uint8_t  information;
    uint8_t error_code;
    uint8_t  crc8;
} LiDARHealthInfoType;

typedef union __attribute__((packed)) {
    LiDARManufactureInfoType manufacture;
    LiDARMeasureDataType measure;
    LiDARHealthInfoType health;
} LiDARFrame;

class Lidar
{
public:
    static void setup();

    static void stop();
    static void resume();

    static bool isRunning() { return running.load( std::memory_order_relaxed ); }

    static void setMinDistances( uint16_t dist[4] ) { for( int i = 0; i < 4; i++ ) minDistances[ i ].store( dist[ i ], std::memory_order_relaxed ); }
    static uint16_t getMinDistance( int side ) { return minDistances[ side ].load( std::memory_order_relaxed ); }

private:
    static inline std::atomic<bool> running = false;
    static inline std::atomic<bool> flushData = false;
    static inline std::atomic<uint16_t> minDistances[4];

    [[noreturn]] static void task( void *arg );

    static void resetDistances();
};

#endif