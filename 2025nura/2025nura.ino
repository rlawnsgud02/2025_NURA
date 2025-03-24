///////////////////2025 NURA AHRS & Wireless Communication/////////////////////
// Last update: 2025.03.20

#include "EBIMU_AHRS.h"
#include "ubx_gps.h"

// 핀 설정
#define GPS_TX 11 // GPS TX핀 11번
#define GPS_RX 12 // GPS RX핀 12번
#define IMU_TX 9 // IMU TX핀 9번
#define IMU_RX 10 // IMU RX핀 10번
#define RF_TX 7 // RF TX핀 7번
#define RF_RX 8 // RF RX핀 8번

// Debuging pins
#define threadPin1 2 // 스레드 확인용 디버깅 핀. LED를 연결하여 깜빡이도록 구현 가능. D2에 해당

// 객체 생성
// UbxGPS gps(GPS_RX, GPS_TX); // GPS는 SoftwareSerial 사용
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
UbxGPS gps(gpsSerial);
EBIMU_AHRS imu(Serial2, IMU_RX, IMU_TX);

GpsData gpsdata; // GPS 데이터 저장할 구조체 변수

// 변수 선언
float acc[3], gyro[3], mag[3], baro[3];
float roll, pitch, yaw; // AHRS의 Roll, Pitch, Yaw
float accelZ; // Z축 가속도
float maxG = 0; // 발사 직후의 최대 G값

bool threadFlag1 = false; // 스레드 시작을 알리는 플래그

void setup()
{
    Serial.begin(115200);
    while (!Serial); // Serial 초기화 대기
    Serial.println("-----| Serial Ready! |-----"); 

    // 센서 초기화
    imu.initialize();

    gpsSerial.begin(19200);
    delay(2000);
    gps.initialize();        // initialize 안에서 9600bps로 PRT 설정 전송
    
    // 디버깅 핀 설정
    // pinMode(threadPin1, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    if (!threadFlag1)
    {
        // digitalWrite(threadPin1, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        threadFlag1 = true;
    }
    else if (threadFlag1)
    {
        // digitalWrite(threadPin1, LOW);
        digitalWrite(LED_BUILTIN, LOW);
        threadFlag1 = false;
    }

    // AHRS 데이터 업데이트
    imu.parseData();
    roll = imu.getRoll(); // Roll 업데이트
    pitch = imu.getPitch(); // Pitch 업데이트
    yaw = imu.getYaw(); // Yaw 업데이트
    accelZ = imu.getAccelZ(); // Z축 가속도 업데이트
    maxG = max(maxG, accelZ);

    gps.get_gps_data(gpsdata); // GPS 데이터 업데이트. gpsdata에 구조체로 저장


    // // 디버깅용 print
    // // imu.printData();
    // // Serial.print("Max G: "); Serial.println(maxG); // 최대 G값 출력
    gps.printGps(); // GPS 데이터 출력
    
}