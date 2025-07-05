#ifndef __UBX_GPS_H__
#define __UBX_GPS_H__

#include <Arduino.h>
#include "ubx_config.h"

// #include <SoftwareSerial.h>
#include <Wire.h> 

struct GpsData {
    // unsigned int iTOW;
    // unsigned short year;
    // unsigned char month;
    // unsigned char day;
    // unsigned char hour;
    // unsigned char min;
    // unsigned char sec;
    
    unsigned char fixType;
    float lon, lat, height;
    float velN, velE, velD;
};

namespace UBX_ID // UBX Payload ID
{
    enum Header
    {
        HEADER1 = 0xB5,
        HEADER2 = 0x62
    };

    enum CFG
    {
        CFG = 0x06,

        MSG = 0x01,
        NAV5 = 0x24,
        PRT = 0x00,
        RATE = 0x08,

    };

    enum NAV
    {
        NAV = 0x01,
        NMEA_NAV2 = 0xF7,

        CLOCK = 0x22,
        POSECEF = 0x01,
        POSLLH = 0x02,
        STATUS = 0x03,
        PVT = 0x07,
        TIMEGPS = 0x16,
        TIMEUTC = 0x21,
        VELECEF = 0x11,
        VELNED = 0x12

    };

    enum NMEA
    {
        NMEA = 0xF0,

        GPGGA = 0x00,
        GPGLL = 0x01,
        GPGSA = 0x02,
        GPGSV = 0x03,
        GPRMC = 0x04
    };
}

class UbxGPS {
private:
    // SoftwareSerial 사용 버전 코드. 주석처리 //
    // SoftwareSerial& GPSserial;
    // int rxPin, txPin; // GPS의 RX, TX 핀
    // char buffer[100];  // 수신 버퍼

    // GpsData gps; // GPS 데이터 저장할 구조체 변수
    // bool new_update_flag;
    // void decode(char byte);
    // void parse_PVT(char *packet);
    // void parse_POSLLH(char *packet);
    // int byte_to_int(const char *ptr, int len); 
    // char byte;

    // I2C 사용 버전 코드 //
    TwoWire& gpsPort; // 제어할 I2C 포트 객체를 직접 참조
    int gpsSDA;
    int gpsSCL;
    char buffer[100];
    GpsData gps;
    bool new_update_flag;

    void decode(char byte);
    void parse_PVT(char *packet);
    void parse_POSLLH(char *packet);
    int byte_to_int(const char *ptr, int len); 
    char byte;

public:
    // UbxGPS(SoftwareSerial& serial);
    UbxGPS(TwoWire& port, int sda, int scl); 
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