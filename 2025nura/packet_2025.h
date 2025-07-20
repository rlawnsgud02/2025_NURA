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
struct OptimizedImuPayload { // 총 34Byte - Only IMU
    int16_t acc[3]; // 2 + 2 + 2 = 6
    int16_t gyro[3]; // 2 + 2 + 2 = 6
    int16_t mag[3]; // 2 + 2 + 2 = 6
    int16_t euler[3]; // 2 + 2 + 2 = 6
    uint16_t temperature; // 2
    uint32_t pressure; // 4
    uint16_t P_alt; // 2
    // IMU 32Byte
    
    uint8_t ejection; // 1Byte

    uint8_t launch; // 1Byte
};
#pragma pack(pop)

#pragma pack(push, 1)
struct OptimizedGpsPayload { // 총 19Byte - Only GPS
    uint32_t lon; // 4
    uint32_t lat; // 4
    uint32_t alt; // 4
    int16_t velN; // 2
    int16_t velE; // 2
    int16_t velD; // 2
    uint8_t fixType; // 1
    // GPS 19Byte
};
#pragma pack(pop)

#pragma pack(push, 1)
struct OptimizedImuGpsPayload { // 총 53Byte - IMU & GPS
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
    
    uint8_t ejection; // 1Byte

    uint8_t launch; // 1Byte
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
    int get_imu_packet(char* packet, uint32_t timestamp, float* acc, float* gyro, float* mag, float* euler, float* press, uint8_t ejection, uint8_t launch);
    int get_gps_packet(char* packet, uint32_t timestamp, GpsData& gps);
    int get_imu_gps_packet(char* packet, uint32_t timestamp, float* acc, float* gyro, float* mag, float* euler, float* press, GpsData& gps, uint8_t ejection, uint8_t launch);
};
    
#endif