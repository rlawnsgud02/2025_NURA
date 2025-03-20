#ifndef EBIMU_AHRS_H
#define EBIMU_AHRS_H

#include <Arduino.h>
#include <HardwareSerial.h>

class EBIMU_AHRS {
public:
    EBIMU_AHRS(HardwareSerial& serial, int rxPin, int txPin, long baudRate = 115200);
    
    void initialize(); // IMU 초기화
    void sendCommand(const char* command); // 명령어 전송
    void parseData(); // IMU 데이터 파싱
    
    void calibrateGyro();    // 자이로 캘리브레이션
    void calibrateAccel();   // 가속도 캘리브레이션
    void calibrateMagneto(); // 지자기 캘리브레이션
    void calibrateAll();     // 모든 센서 캘리브레이션

    String readData(); // IMU 데이터 읽기
    void printData();  // IMU 데이터 출력
    void getRPY(float &r, float &p, float &y);
    float getRoll();
    float getPitch();
    float getYaw();
    float getAccelZ();

private:
    HardwareSerial& IMU_Serial;
    int rxPin;
    int txPin;
    long baudRate;

    float data[6]; // RPY, AccelX, AccelY, AccelZ를 저장하는 배열
    float roll, pitch, yaw;
    float accelZ; // 발사 직후 max g값 구하기
};

#endif // EBIMU_AHRS_H
