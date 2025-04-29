#ifndef SDFATLOGGER_H
#define SDFATLOGGER_H

#include <SdFat.h>
#include "ubx_gps.h"

namespace DATA {
    extern int32_t timestamp;
    extern float* acc;
    extern float* gyro;
    extern float* mag;
    extern float* euler;
    extern float maxG;
    extern float* baro;
    extern GpsData gps;
    extern int eject;
}
// namespace DATA {
//     struct SensorData {
//         int32_t timestamp;
//         float acc[3];
//         float gyro[3];
//         float mag[3];
//         float euler[3];
//         float maxG;
//         float baro[3];
//         int fixType;
//         double lon;
//         double lat;
//         float height;
//         float velN, velE, velD;
//         int eject;
//     };

//     extern SensorData current;
// }

// 164 + GPS -> 512제한

class SDFatLogger {
private:
    SdFs sd;
    FsFile dataFile;
    int CS_pin;
    int file_num;
    char file_name[25];
    bool init;

    // int32_t timestamp;
    // float* acc;
    // float* gyro;
    // float* mag;
    // float* euler;
    // float maxG;
    // float* baro;
    // GpsData gps;
    // int eject;

public:
    SDFatLogger(int cs);
    int initialize();
    void show_file_list();
    int create_new_data_file();
    bool isInit();
    void setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, GpsData &gps, int eject);
    // void setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, int eject);
    const char* getFileName();
    bool openFile();
    bool closeFile();
    bool flushFile();
    bool write_data();
    void print();
    // bool write_one_line();
};

#endif // SDFATLOGGER_H