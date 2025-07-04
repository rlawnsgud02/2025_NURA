#include "packet.h"

Packet::Packet()
{
    buf[0] = Packet::HEADER1;
    buf[1] = Packet::HEADER2;
}

void Packet::add_chksum()
{
    int len = buf[3] + 5;
    char chksum = 0;
    for(int i=2; i<len-1; i++)
    {
        chksum ^= buf[i];
    }
    buf[len-1] = chksum;
}

int Packet::get_gps_packet(char * packet, uint32_t timestamp, GpsData &gps)
{
    buf[2] = MSG_GPS;
    buf[3] = GPS::SIZE;
    
    memcpy(buf+GPS::TIMESTAMP, &timestamp, 4);
    memcpy(buf+GPS::LON, &gps.lon, 4);
    memcpy(buf+GPS::LAT, &gps.lat, 4);
    memcpy(buf+GPS::ALT, &gps.height, 4);
    memcpy(buf+GPS::VN, &gps.velN, 4);
    memcpy(buf+GPS::VE, &gps.velE, 4);
    memcpy(buf+GPS::VD, &gps.velD, 4);
    buf[GPS::FIX] = gps.fixType;

    add_chksum();

    memcpy(packet, buf, GPS::SIZE+5);
    return GPS::SIZE+5;
}

int Packet::get_imu_packet(char * packet, uint32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * press, uint8_t ejection)
{
    buf[2] = MSG_IMU;
    buf[3] = IMU::SIZE;

    memcpy(buf+IMU::TIMESTAMP, &timestamp, 4);
    memcpy(buf+IMU::AX, acc, 12);
    memcpy(buf+IMU::GX, gyro, 12);
    memcpy(buf+IMU::MX, mag, 12);
    memcpy(buf+IMU::ROLL, euler, 12);
    memcpy(buf+IMU::TEMP, press, 12);

    buf[IMU::EJECT] = ejection;
    
    add_chksum();

    memcpy(packet, buf, IMU::SIZE+5);

    return IMU::SIZE+5;
}

int Packet::get_imu_gps_packet(char * packet, uint32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * press,  GpsData &gps, uint8_t ejection)
{
    buf[2] = MSG_IMU_GPS;
    buf[3] = IMUGPS::SIZE;
    
    memcpy(buf+IMUGPS::TIMESTAMP, &timestamp, 4);
    memcpy(buf+IMUGPS::AX, acc, 12);
    memcpy(buf+IMUGPS::GX, gyro, 12);
    memcpy(buf+IMUGPS::MX, mag, 12);
    memcpy(buf+IMU::ROLL, euler, 12);
    memcpy(buf+IMUGPS::TEMP, press, 12);

    memcpy(buf+IMUGPS::LON, &gps.lon, 4);
    memcpy(buf+IMUGPS::LAT, &gps.lat, 4);
    memcpy(buf+IMUGPS::ALT, &gps.height, 4);
    memcpy(buf+IMUGPS::VN, &gps.velN, 4);
    memcpy(buf+IMUGPS::VE, &gps.velE, 4);
    memcpy(buf+IMUGPS::VD, &gps.velD, 4);
    buf[IMUGPS::FIX] = gps.fixType;

    buf[IMUGPS::EJECT] = ejection;
    
    add_chksum();

    memcpy(packet, buf, IMUGPS::SIZE+5);

    return IMUGPS::SIZE+5;
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