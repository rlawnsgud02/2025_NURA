#ifndef EBIMU_AHRS_H
#define EBIMU_AHRS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <string.h> // strtok, memcpy를 위해 추가
#include <stdlib.h> // atof를 위해 추가

class EBIMU_AHRS {
public:
    EBIMU_AHRS(HardwareSerial& serial, int rxPin, int txPin, long baudRate = 115200);
    
    void initialize(); // IMU 초기화
    void sendCommand(const char* command); // 명령어 전송
    void parseData(); // IMU 데이터 파싱
    
    /*
    Calibration은 구체적으로 구현해두지 않았는데, EBIMU에서 공식으로 제공하는 터미널을 사용하여 Calibration을 더 편하게 할 수 있고
    Calibration한 data는 EEPROM에 저장하여 사용할 수 있도록 구현되어 있기에 힘들게 전자모듈 상에서 할 필요가 없음
    따라서 미리 Calibration하고 조립할 것.
    */
    void calibrateGyro();    // 자이로 캘리브레이션
    void calibrateAccel();   // 가속도 캘리브레이션
    void calibrateMagneto(); // 지자기 캘리브레이션
    void calibrateAll();     // 모든 센서 캘리브레이션

    String readData(); // IMU 데이터 읽기
    // void readData(char* buffer, int bufferSize); // IMU 데이터 읽기
    void printData();  // IMU 데이터 출력
    void getRPY(float &r, float &p, float &y);
    void getAccelGyroMagFloat(float* accel, float* gyro, float* mag);
    

    float get_anglegro();

private:
    HardwareSerial& IMU_Serial;
    int rxPin;
    int txPin;
    long baudRate;

    float data[12]; // accel, gyro, mag, rpy를 저장하는 배열 (각각 3개씩)
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float roll, pitch, yaw;
    // float maxG;

};

#endif // EBIMU_AHRS_H
