#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <SPI.h>
#include <SD.h>

class SDLogger {
private:
    int cs_pin;
    char file_name[20];
    File fp;

public:
    SDLogger(int cs);
    ~SDLogger();

    int initialize();
    void show_file_list();
    int create_new_data_file();
    int write_data(uint32_t timestamp, float *acc, float *gyro, float *mag, float *euler, float *baro);
    void save_data_file();
    int open_data_file();
};

#endif // SDLOGGER_H
