#ifndef __PACKET_H__
#define __PACKET_H__

#include <Arduino.h>
#include "ubx_gps.h"

/* Packet Structure
/ HEADER1 / HEADER2 / Type / Payload Length / ... Data ... / Chksum /
*/

class Packet
{
private:
    char buf[100];
    void add_chksum();

public:
    Packet();
    int get_gps_packet(char * packet, uint32_t timestamp, GpsData &gps);
    int get_imu_packet(char * packet, uint32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * press, uint8_t ejection);
    int get_imu_gps_packet(char * packet, uint32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * press, GpsData &gps, uint8_t ejection);
    // int get_state_packet(uint8_t ejection);

    enum id
    {
        // HEADER1 = 0xB5,
        // HEADER2 = 0x62,

        HEADER1 = 0xFF,
        HEADER2 = 0xFE
    };

    enum msgs
    {
        MSG_IMU = 0x01,
        MSG_GPS = 0x02,
        MSG_IMU_GPS = 0x03,
        // MSG_STATE = 0x10
    };
    
};

namespace IMU{
    enum imu_data
    {
        SIZE = 65,

        TIMESTAMP = 4,
        AX = TIMESTAMP + 4,
        AY = AX + 4, 
        AZ = AY + 4, 
        GX = AZ + 4, 
        GY = GX + 4, 
        GZ = GY + 4, 
        MX = GZ + 4, 
        MY = MX + 4, 
        MZ = MY + 4,
        ROLL = MZ + 4,
        PITCH = ROLL + 4,
        YAW = PITCH + 4,
        TEMP = YAW + 4,
        P  = TEMP + 4, 
        P_ALT = P + 4,

        EJECT = P_ALT + 4
        // EJECT -> 1
    };
}

namespace GPS
{
    enum gps_data
    {
        SIZE = 29,

        TIMESTAMP = 4,
        LON = TIMESTAMP + 4, 
        LAT = LON + 4, 
        ALT = LAT + 4,
        VN  = ALT + 4, 
        VE  = VN + 4, 
        VD  = VE + 4,
        FIX = VD + 4
        // FIX -> 1
    };
}

namespace IMUGPS
{
    enum imu_gps_data
    {
        SIZE = 90,

        TIMESTAMP = 4,
        AX = TIMESTAMP + 4,
        AY = AX + 4,
        AZ = AY + 4,
        GX = AZ + 4, 
        GY = GX + 4, 
        GZ = GY + 4, 
        MX = GZ + 4, 
        MY = MX + 4, 
        MZ = MY + 4,
        ROLL = MZ + 4,
        PITCH = ROLL + 4,
        YAW = PITCH + 4,
        TEMP = YAW + 4,
        P  = TEMP + 4, 
        P_ALT = P + 4,
        
        LON = P_ALT + 4, 
        LAT = LON + 4, 
        ALT = LAT + 4,
        VN  = ALT + 4, 
        VE  = VN + 4, 
        VD  = VE + 4,
        FIX = VD + 4,

        EJECT = FIX + 1
        // EJECT -> 1
    };
}

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