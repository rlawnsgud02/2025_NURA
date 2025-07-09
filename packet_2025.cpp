#include "packet.h"

Packet::Packet()
{
    buf[0] = Packet::HEADER1;
    buf[1] = Packet::HEADER2;
}

void Packet::add_chksum()
{
    int len = (unsigned char)buf[3] + 5; // payload_size + 5
    char chksum = 0;
    for (int i = 2; i < len - 1; i++) {
        chksum ^= buf[i];
    }
    buf[len - 1] = chksum;
}

int Packet::get_imu_packet(char * packet, uint32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * press, uint8_t ejection)
{
    buf[2] = MSG_IMU;
    buf[3] = sizeof(OptimizedImuPayload) + 4; // payload_size(27) + timestamp_size(4)
    
    memcpy(buf + 4, &timestamp, 4);

    OptimizedImuGpsPayload payload;
    for (int i = 0; i < 3; ++i) {
        payload.acc[i]   = static_cast<int16_t>(acc[i] * 100.0f);
        payload.gyro[i]  = static_cast<int16_t>(gyro[i] * 10.0f);
        payload.mag[i]   = static_cast<int16_t>(mag[i] * 10.0f);
        payload.euler[i] = static_cast<int16_t>(euler[i] * 100.0f);
    }
    payload.pressure = static_cast<int16_t>(press * 10.0f);
    
    payload.ejection = ejection;

    memcpy(buf + 8, &payload, sizeof(OptimizedImuPayload)); // Payload 구조체에 모두 저장 후 한 번에 memcpy
    // buf[0] = HEADER1; buf[1] = HEADER2; buf[2] = MsgID; buf[3] = Length(= payload_size 패킷의 크기) -> 27; buf[4 ~ 7] = timestamp;
    // 그래서 (buf + 8)로 버퍼 시작점으로부터 8번째부터 구조체 내용 COPY;

    add_chksum(); // 여기까지 36Byte

    int final_size = sizeof(OptimizedImuPayload) + 4 + 5; // payload + timestamp + (header,id,len,chksum)
    memcpy(packet, buf, final_size);
    return final_size; // 36Byte
}

int Packet::get_gps_packet(char * packet, uint32_t timestamp, GpsData &gps)
{
    buf[2] = MSG_GPS;
    buf[3] = sizeof(OptimizedGpsPayload) + 4; // payload_size(19) + timestamp_size(4)
    
    memcpy(buf + 4, &timestamp, 4);

    OptimizedGpsPayload payload;
    payload.lon = static_cast<int32_t>(gps.lon * 1e7);
    payload.lat = static_cast<int32_t>(gps.lat * 1e7);
    payload.alt = static_cast<int32_t>(gps.height * 100.0f);
    payload.velN = static_cast<int16_t>(gps.velN * 100.0f);
    payload.velE = static_cast<int16_t>(gps.velE * 100.0f);
    payload.velD = static_cast<int16_t>(gps.velD * 100.0f);
    payload.fixType = gps.fixType;
    
    memcpy(buf + 8, &payload, sizeof(OptimizedGpsPayload)); // Payload 구조체에 모두 저장 후 한 번에 memcpy
    // buf[0] = HEADER1; buf[1] = HEADER2; buf[2] = MsgID; buf[3] = Length(= payload_size 패킷의 크기) -> 19; buf[4 ~ 7] = timestamp;
    // 그래서 (buf + 8)로 버퍼 시작점으로부터 8번째부터 구조체 내용 COPY;

    add_chksum(); // 여기까지 28Byte

    int final_size = sizeof(OptimizedGpsPayload) + 4 + 5; // payload + timestamp + (header,id,len,chksum)
    memcpy(packet, buf, final_size);
    return final_size; // 28Byte
}

int Packet::get_imu_gps_packet(char* packet, uint32_t timestamp, float* acc, float* gyro, float* mag, float* euler, float press, GpsData& gps, uint8_t ejection)
{
    buf[2] = MSG_IMU_GPS;
    buf[3] = sizeof(OptimizedImuGpsPayload) + 4; // payload_size(46) + timestamp_size(4)
    
    memcpy(buf + 4, &timestamp, 4);

    OptimizedImuGpsPayload payload;
    for (int i = 0; i < 3; ++i) {
        payload.acc[i]   = static_cast<int16_t>(acc[i] * 100.0f);
        payload.gyro[i]  = static_cast<int16_t>(gyro[i] * 10.0f);
        payload.mag[i]   = static_cast<int16_t>(mag[i] * 10.0f);
        payload.euler[i] = static_cast<int16_t>(euler[i] * 100.0f);
    }
    payload.pressure = static_cast<int16_t>(press * 10.0f);

    payload.lon = static_cast<int32_t>(gps.lon * 1e7);
    payload.lat = static_cast<int32_t>(gps.lat * 1e7);
    payload.alt = static_cast<int32_t>(gps.height * 100.0f);
    payload.velN = static_cast<int16_t>(gps.velN * 100.0f);
    payload.velE = static_cast<int16_t>(gps.velE * 100.0f);
    payload.velD = static_cast<int16_t>(gps.velD * 100.0f);
    payload.fixType = gps.fixType;
    payload.ejection = ejection;
    
    memcpy(buf + 8, &payload, sizeof(OptimizedImuGpsPayload)); // Payload 구조체에 모두 저장 후 한 번에 memcpy
    // buf[0] = HEADER1; buf[1] = HEADER2; buf[2] = MsgID; buf[3] = Length(= payload_size 패킷의 크기) -> 46; buf[4 ~ 7] = timestamp;
    // 그래서 (buf + 8)로 버퍼 시작점으로부터 8번째부터 구조체 내용 COPY;

    add_chksum(); // 여기까지 55Byte

    int final_size = sizeof(OptimizedImuGpsPayload) + 4 + 5; // payload + timestamp + (header,id,len,chksum)
    memcpy(packet, buf, final_size);
    return final_size; // 55Byte
}

// int Packet::get_state_packet(uint8_t ejection)
// {
//     buf[2] = MSG_STATE;
//     buf[3] = STATE::SIZE;

//     memcpy(buf+STATE::EJECT, &ejection, 1);

//     add_chksum();

//     memcpy(packet, buf, STATE::SIZE+5);
//     return STATE::SIZE+5;
// }