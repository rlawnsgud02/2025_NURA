/* ----------------------------------------
    2025 NURA AHRS & Wireless Communication 
    - Last update: 2025.05.19
   ---------------------------------------- */

#include "EBIMU_AHRS.h"
#include "ubx_gps.h"
#include "BMP390L.h"
#include "SDLogger.h"
#include "NMT.h"
#include "packet.h"

// 핀 설정
// #define GPS_TX 6 // GPS TX핀
// #define GPS_RX 7 // GPS RX핀
#define GPS_SDA 6 // GPS SDA핀
#define GPS_SCL 7 // GPS SCL핀
#define GPS_INT A7 // GPS 인터럽트 핀
#define IMU_TX 8 // IMU TX핀
#define IMU_RX 9 // IMU RX핀
#define BARO_SDA A0
#define BARO_SCL A1
#define RF_TX 4 // RF TX핀
#define RF_RX 5 // RF RX핀
#define CS_PIN 10

// Debuging pins
#define threadPin1 2 // 스레드 확인용 디버깅 핀. LED를 연결하여 깜빡이도록 구현 가능. D2에 해당
#define safeyPin 3

// 객체 생성
// SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
// UbxGPS gps(gpsSerial);
UbxGPS gps(Wire1, GPS_SDA, GPS_SCL); // I2C를 사용한 GPS 객체 생성
EBIMU_AHRS imu(Serial2, IMU_RX, IMU_TX);
BMP390L Baro;
SDFatLogger sd(CS_PIN);
NMT rf(Serial1, RF_RX, RF_TX, 9600);
Packet data;

GpsData gpsdata; // GPS 데이터 저장할 구조체 변수
char packet[100];

// 변수 선언
float acc[3], gyro[3], mag[3], RPY[3], baro[3];
float maxG = 0; // 발사 직후의 최대 G값
int chute_eject = 0; // 낙하산 사출 여부

volatile bool gpsReady = false;
static uint32_t timeStamp = 0;
static uint32_t Timer = 0;

bool threadFlag1 = false; // 스레드 시작을 알리는 플래그
bool isLaunched = false;
bool sd_init = true; // SD 카드 초기화 여부

// Interrupt가 들어오면 실행
void gpsInterrupt() {
  gpsReady = true;
}

void setup()
{
    Serial.begin(115200);
    // while (!Serial); // Serial 초기화 대기. USB로 연결하지 않은 아두이노 단독 실행의 경우 반드시 주석!!
    Serial.println("-----| Serial Ready! |-----");

    rf.initialize();
    delay(5000);

    imu.initialize();
    rf.print("IMU Ready!");

    // gpsSerial.begin(19200);
    gps.initialize();
    delay(500);
    
    Baro.begin_I2C(BMP3XX_DEFAULT_ADDRESS, BARO_SDA, BARO_SCL);
    delay(500);
    
    pinMode(GPS_INT, INPUT); // GPS 인터럽트 핀 설정
    attachInterrupt(digitalPinToInterrupt(GPS_INT), gpsInterrupt, RISING);

    // 디버깅 핀 설정
    // pinMode(threadPin1, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(safeyPin, INPUT);

    Serial.println("-----| START! |-----");
    rf.print("Avionics Ready!");
}

void loop()
{
    if(sd_init)
    {
        sd.initialize();
        Timer = millis(); // 타이머 시작
        sd_init = false;
    }

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
        maxG = max(maxG, acc[2]); 
    }

    if (Baro.isDataReady()) {
        if (Baro.performReading()) {
            Baro.getTempPressAlt(baro[0], baro[1], baro[2]);
        }
    }

    // if (gpsReady) {
    //     gpsReady = false; 
    //     gps.get_gps_data(gpsdata);
    // }
    gps.get_gps_data(gpsdata);

    // sd 카드 데이터 저장
    sd.openFile();
    // sd.setData(timeStamp, acc, gyro, mag, RPY, maxG, baro, chute_eject);
    sd.setData(timeStamp, acc, gyro, mag, RPY, maxG, baro, gpsdata, chute_eject);
    sd.write_data();
    sd.closeFile();

    // RF 데이터 전송
    // int packet_len = 0;
    // if(gps.is_updated()){
        // packet_len = data.get_imu_gps_packet(packet, timeStamp, acc, gyro, mag, RPY, baro, gpsdata, chute_eject);
    // } 
    // else{
        // packet_len = data.get_imu_packet(packet, timeStamp, acc, gyro, mag, RPY, baro, chute_eject);
    // }
    // rf.transmit_packet(packet, packet_len);
    
    // 디버깅용 print
    // imu.printData();
    // Serial.print("Max G: "); Serial.println(maxG); // 최대 G값 출력
    // gps.printGps(); // GPS 데이터 출력
    Serial.print("Lat: "); Serial.print(gpsdata.lat, 7);
    Serial.print(", Lon: "); Serial.print(gpsdata.lon, 7);
    Serial.print(", Height: "); Serial.print(gpsdata.height);
    Serial.print(", FixType: "); Serial.println(gpsdata.fixType);
    // Serial.print("Baro Temp: "); Serial.print(baro[0]); Serial.print(" C, ");
    // Serial.print("Baro Press: "); Serial.print(baro[1]); Serial.print(" hPa, ");
    // Serial.print("Baro Alt: "); Serial.print(baro[2]); Serial.println(" m");
    // delay(500);
}