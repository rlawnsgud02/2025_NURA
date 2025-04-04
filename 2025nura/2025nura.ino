///////////////////2025 NURA AHRS & Wireless Communication/////////////////////
// Last update: 2025.04.03

#include "EBIMU_AHRS.h"
#include "ubx_gps.h"
#include "BMP390L.h"
#include "SDCard.h"
#include "SPI.h"
#include "SDLogger.h"

// 핀 설정
#define GPS_TX 6 // GPS TX핀 11번
#define GPS_RX 7 // GPS RX핀 12번
#define IMU_TX 8 // IMU TX핀 9번
#define IMU_RX 9 // IMU RX핀 10번
#define BARO_SDA A0
#define BARO_SCL A1
#define RF_TX 4 // RF TX핀 7번
#define RF_RX 5 // RF RX핀 8번
#define CS_PIN 10

// Debuging pins
#define threadPin1 2 // 스레드 확인용 디버깅 핀. LED를 연결하여 깜빡이도록 구현 가능. D2에 해당
#define safeyPin 3

// 객체 생성
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
UbxGPS gps(gpsSerial);
EBIMU_AHRS imu(Serial2, IMU_RX, IMU_TX);
BMP390L Baro;
// SDLogger sd(CS_PIN);
SDFatLogger sd(CS_PIN); 

GpsData gpsdata; // GPS 데이터 저장할 구조체 변수

// 변수 선언
float acc[3], gyro[3], mag[3], RPY[3], baro[3];
float maxG = 0; // 발사 직후의 최대 G값
int chute_eject = 0; // 낙하산 사출 여부

static uint32_t timeStamp = 0;
static uint32_t Timer = 0;

bool threadFlag1 = false; // 스레드 시작을 알리는 플래그
bool isLaunched = false;


void setup()
{
    Serial.begin(115200);
    while (!Serial); // Serial 초기화 대기
    Serial.println("-----| Serial Ready! |-----"); 

    sd.initialize();

    // 센서 초기화
    imu.initialize();

    gpsSerial.begin(19200);
    delay(1000);
    gps.initialize();
    delay(500);
    
    Baro.begin_I2C(BMP3XX_DEFAULT_ADDRESS, BARO_SDA, BARO_SCL);
    delay(500);
    
    // 디버깅 핀 설정
    // pinMode(threadPin1, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(safeyPin, INPUT);

    Serial.println("-----| START! |-----");
    Timer = millis(); // 타이머 시작
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

    timeStamp = millis() - Timer;

    // 센서 데이터 업데이트
    if(Serial2.available()) {
        imu.parseData();
        imu.getRPY(RPY[0], RPY[1], RPY[2]);
        imu.getAccelGyroMagFloat(acc, gyro, mag);
        // maxG = max(maxG, acc[2]); // acc 값이 한 번씩 튀는 문제 발생 중...
    }

    if(gpsSerial.available()) {
      gps.get_gps_data(gpsdata); // gpsdata에 구조체에 데이터 저장
    }

    if (Baro.isDataReady()) {
        if (Baro.performReading()) {
            Baro.getTempPressAlt(baro[0], baro[1], baro[2]);
        }
    }

    sd.write_data(timeStamp, acc, gyro, mag, RPY, maxG, baro, gpsdata, chute_eject);
    // Serial.print("현재 사용중인 파일 이름: ");
    // Serial.println(sd.getFileName());

    // 디버깅용 print
    // imu.printData();
    // Serial.print("Max G: "); Serial.println(maxG); // 최대 G값 출력
    // gps.printGps(); // GPS 데이터 출력
    // Serial.print("Baro Temp: "); Serial.print(baro[0]); Serial.print(" C, ");
    // Serial.print("Baro Press: "); Serial.print(baro[1]); Serial.print(" hPa, ");
    // Serial.print("Baro Alt: "); Serial.print(baro[2]); Serial.println(" m");
    // delay(500);
}