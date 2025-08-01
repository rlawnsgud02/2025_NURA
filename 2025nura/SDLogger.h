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
    extern int8_t eject;
    extern int8_t launch;
    extern int* control_log;
}

// namespace DATA {
//     extern int32_t timestamp;
//     extern float* acc;
//     extern float* gyro;
//     extern float* mag;
//     extern float* euler;
//     extern float maxG;
//     extern float* baro;
//     extern int eject;
// }

class SDFatLogger {
private:
    SdFs sd;
    FsFile dataFile;
    int CS_pin;
    int file_num;
    char file_name[25];
    bool init;

public:
    SDFatLogger(int cs);
    int initialize();
    void show_file_list();
    int create_new_data_file();
    // int write_data(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, GpsData &gps, int eject);
    int write_data();
    bool isInit();
    const char* getFileName();
    bool openFile();
    bool closeFile();
    bool flushFile();
    void setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, GpsData &gps, int eject, int launch, int * control_log);
    // void setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, int eject);
    void print();
    bool write_one_line();
};

#endif // SDFATLOGGER_H