#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <SPI.h>
#include <SD.h> // SD library for Arduino
#include "ubx_gps.h"

class SDLogger {
private:
    int CS_pin;
    File *fp;

    int file_num;
    char file_name[25];

    int check_new_data_file_number();
 
public:
    SDLogger(int cs);
    ~SDLogger();

    int initialize();

    void show_file_list();
    int create_new_data_file();

    int write_data(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * baro, GpsData &gps, int eject);
    void save_data_file(); // 완료 후 파일 저장 외에도 중간 저장 가능
    int open_data_file(); // 중간 저장 시 사용
};

#endif // SDLOGGER_H
