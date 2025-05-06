#ifndef __UBX_GPS_H__
#define __UBX_GPS_H__

#include <Arduino.h>
#include <SPI.h>
#include "ubx_config.h"

struct GpsData {
    unsigned int iTOW;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
    
    unsigned char fixType;
    float lon, lat, height, hMSL;
    float velN, velE, velD;
};

namespace UBX_ID {
    enum Header { HEADER1 = 0xB5, HEADER2 = 0x62 };
    enum CFG { CFG = 0x06, MSG = 0x01, NAV5 = 0x24, PRT = 0x00, RATE = 0x08 };
    enum NAV { NAV = 0x01, PVT = 0x07, POSLLH = 0x02 };
}

class UbxGPS {
private:
    int csPin;
    char buffer[100];
    GpsData gps;
    bool new_update_flag;
    char byte;

    void decode(char byte);
    void parse_PVT(char *packet);
    void parse_POSLLH(char *packet);
    int byte_to_int(const char *ptr, int len);
    void select();
    void deselect();
    char spi_transfer_byte(char out = 0xFF);

public:
    UbxGPS(int csPin);
    void initialize();
    bool get_gps_data(GpsData &data);
    bool is_updated();
    char is_fixed();
    void set_config(const char *cmd);
    void disable_all_nmea(bool disable=true);
    void enable_ubx(char id_);
    void printGps();
};

#endif
