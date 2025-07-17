#ifndef __PACKET_H__
#define __PACKET_H__

#include <Arduino.h>
// #include <cstdint>
// #include <cstring>
#include "ubx_gps.h"

template <typename T, size_t Size>

class CircularBuffer
{
public:
    CircularBuffer() : head_(0), tail_(0), full_(false) {}

    bool write(const T& item) {
        if (isFull()) {
            return false;
        }
        buffer_[head_] = item;
        head_ = (head_ + 1) % Size;
        full_ = (head_ == tail_);
        return true;
    }

    bool read(T& item) {
    if (isEmpty()) {
        return false;
    }
    item = buffer_[tail_];
    tail_ = (tail_ + 1) % Size;
    full_ = false;
    return true;
    }

    bool isEmpty() const {
        return (!full_ && (head_ == tail_)); // NOT full이면서 맨앞과 맨뒤가 일치하면 empty 버퍼;
    }

    bool isFull() const {
        return full_;
    }

private:
    T buffer_[Size];
    size_t head_;
    size_t tail_;
    bool full_;
};

#pragma pack(push, 1) // 1바이트 단위로 메모리 정렬
struct OptimizedImuPayload { // 총 33Byte - Only IMU
    int16_t acc[3]; // 2 + 2 + 2 = 6
    int16_t gyro[3]; // 2 + 2 + 2 = 6
    int16_t mag[3]; // 2 + 2 + 2 = 6
    int16_t euler[3]; // 2 + 2 + 2 = 6
    uint16_t temperature; // 2
    uint32_t pressure; // 4
    uint16_t P_alt; // 2
    // IMU 32Byte
    
    uint8_t ejection;
    // 1Byte
};
#pragma pack(pop)

// #pragma pack(push, 1)
// struct OptimizedGpsPayload { // 총 19Byte - Only GPS
//     int32_t lon; // 4
//     int32_t lat; // 4
//     int32_t alt; // 4
//     int16_t velN; // 2
//     int16_t velE; // 2
//     int16_t velD; // 2
//     uint8_t fixType; // 1
//     // GPS 19Byte
// };
// #pragma pack(pop)

#pragma pack(push, 1)
struct OptimizedImuGpsPayload { // 총 52Byte - IMU & GPS
    int16_t acc[3]; // 2 + 2 + 2 = 6
    int16_t gyro[3]; // 2 + 2 + 2 = 6
    int16_t mag[3]; // 2 + 2 + 2 = 6
    int16_t euler[3]; // 2 + 2 + 2 = 6
    uint16_t temperature; // 2
    uint32_t pressure; // 4
    uint16_t P_alt; // 2
    // IMU 32Byte
    
    uint32_t lon; // 4
    uint32_t lat; // 4
    uint32_t alt; // 4
    int16_t velN; // 2
    int16_t velE; // 2
    int16_t velD; // 2
    uint8_t fixType; // 1
    // GPS 19Byte
    
    uint8_t ejection;
    // 1Byte
};
#pragma pack(pop)

class Packet
{
private:
    char buf[512];
    void add_chksum();

public:
    static const uint8_t HEADER1 = 0xAA;
    static const uint8_t HEADER2 = 0xBB;

    enum class MsgID : uint8_t
    {
        IMU = 0x01,
        GPS = 0x02,
        IMU_GPS = 0x03,
        // STATE = 0x10
    };

    Packet();
    int get_gps_packet(char* packet, uint32_t timestamp, GpsData& gps);
    int get_imu_packet(char* packet, uint32_t timestamp, float* acc, float* gyro, float* mag, float* euler, float* press, uint8_t ejection);
    int get_imu_gps_packet(char* packet, uint32_t timestamp, float* acc, float* gyro, float* mag, float* euler, float* press, GpsData& gps, uint8_t ejection);
};

// 사용 예시 (Packet::MsgID::IMU 처럼 사용)
// buf[2] = static_cast<uint8_t>(Packet::MsgID::IMU_GPS);

// namespace IMU{
//     enum imu_data
//     {
//         SIZE = 65,

//         TIMESTAMP = 4,
//         AX = TIMESTAMP + 4,
//         AY = AX + 4, 
//         AZ = AY + 4, 
//         GX = AZ + 4, 
//         GY = GX + 4, 
//         GZ = GY + 4, 
//         MX = GZ + 4, 
//         MY = MX + 4, 
//         MZ = MY + 4,
//         ROLL = MZ + 4,
//         PITCH = ROLL + 4,
//         YAW = PITCH + 4,
        
//         TEMP = YAW + 4,
//         P  = TEMP + 4, 
//         P_ALT = P + 4,

//         EJECT = P_ALT + 4
//         // EJECT -> 1
//     };
// }

// namespace GPS
// {
//     enum gps_data
//     {
//         SIZE = 29,

//         TIMESTAMP = 4,
//         LON = TIMESTAMP + 4, 
//         LAT = LON + 4, 
//         ALT = LAT + 4,
//         VN  = ALT + 4, 
//         VE  = VN + 4, 
//         VD  = VE + 4,
//         FIX = VD + 4
//         // FIX -> 1
//     };
// }

// namespace IMUGPS
// {
//     enum imu_gps_data
//     {
//         SIZE = 90,

//         TIMESTAMP = 4,
//         AX = TIMESTAMP + 4,
//         AY = AX + 4,
//         AZ = AY + 4,
//         GX = AZ + 4, 
//         GY = GX + 4, 
//         GZ = GY + 4, 
//         MX = GZ + 4, 
//         MY = MX + 4, 
//         MZ = MY + 4,
//         ROLL = MZ + 4,
//         PITCH = ROLL + 4,
//         YAW = PITCH + 4,

//         TEMP = YAW + 4,
//         P  = TEMP + 4, 
//         P_ALT = P + 4,
        
//         LON = P_ALT + 4, 
//         LAT = LON + 4, 
//         ALT = LAT + 4,
//         VN  = ALT + 4, 
//         VE  = VN + 4, 
//         VD  = VE + 4,
//         FIX = VD + 4,

//         EJECT = FIX + 1
//         // EJECT -> 1
//     };
// }

// namespace STATE
// {
//     enum state
//     {
//         SIZE = 4,

//         EJECT = 4,
//         // EJECT -> 1
//     };
// }

    
#endif