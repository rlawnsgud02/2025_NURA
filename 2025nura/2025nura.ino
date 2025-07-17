/* ----------------------------------------
    2025 NURA AHRS & Wireless Communication 
    - Last update: 2025.07.17
   ---------------------------------------- */

#include "EBIMU_AHRS.h"
#include "ubx_gps.h"
#include "BMP390L.h"
#include "SDLogger.h"
#include "NMT.h"
// #include "packet.h"
#include "packet_2025.h"
#include "ejection.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define RF_RX 4 // RF RX핀
#define RF_TX 5 // RF TX핀
// #define GPS_TX 6 // GPS TX핀
// #define GPS_RX 7 // GPS RX핀
#define GPS_SDA 6 // GPS SDA핀
#define GPS_SCL 7 // GPS SCL핀
#define IMU_RX 8 // IMU RX핀
#define IMU_TX 9 // IMU TX핀
#define CS_PIN 10
#define BARO_SDA A0
#define BARO_SCL A1
#define CANARD1 A2
#define CANARD2 A3
#define CANARD3 A4
#define CANARD4 A5
#define CHUTE A6
#define SAFETY_PIN 2 // RBF
#define LAUNCH_PIN 3 // 발사 감지 핀. 제거 시 부저 울리게 설계

#define CH1 0 // Canard 1
#define CH2 1 // Canard 2
#define CH3 2 // Canard 3
#define CH4 3 // Canard 4
#define CH5 4 // Ejection Servo

// 객체 생성
// SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
// UbxGPS gps(gpsSerial);
UbxGPS gps(Wire1, GPS_SDA, GPS_SCL); // I2C를 사용한 GPS 객체 생성
EBIMU_AHRS imu(Serial2, IMU_RX, IMU_TX);
BMP390L Baro;
SDFatLogger sd(CS_PIN);
NMT rf(Serial1, RF_RX, RF_TX, 9600);
Packet payload;
ejection chute(CHUTE, CH5, false); // 사출 객체 생성

typedef struct {
  float yaw; // 또는 제어에 필요한 roll 값
} ControlData_t;

typedef struct {
  uint32_t timestamp;
  float acc[3], gyro[3], mag[3], RPY[3];
  float baro[3];
  float maxG;
} BlackBoxData_t;

typedef struct {
  float roll, pitch;
  float altitude;
  uint32_t timestamp;
} ParachuteData_t;

typedef struct {
    int eject_type;
} EjectionData_t;

QueueHandle_t ControlQueue;
QueueHandle_t BlackBoxQueue;
QueueHandle_t ParachuteQueue;
QueueHandle_t EjectionQueue;

void FlightControl(void *pvParameters);
void ATTALT(void *pvParameters);
void Parachute(void *pvParameters);
void SRG(void *pvParameters);

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

    sd.initialize();

    chute.servo_init();
    rf.print("eject servo Ready!");

    // 디버깅 핀 설정
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SAFETY_PIN, INPUT_PULLDOWN);
    pinMode(LAUNCH_PIN, INPUT_PULLDOWN);

    // Queue 생성
    ControlQueue = xQueueCreate(1, sizeof(ControlData_t));
    BlackBoxQueue = xQueueCreate(5, sizeof(BlackBoxData_t));
    ParachuteQueue = xQueueCreate(1, sizeof(ParachuteData_t));
    EjectionQueue = xQueueCreate(3, sizeof(EjectionData_t));

    // RTOS 설정. Function, Name, Stack Size, Parameter, Priority, Handle, Core
    xTaskCreatePinnedToCore(FlightControl, "Control Loop", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ATTALT, "IMU, Barometric Loop", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(Parachute, "Chute Ejcetion Loop", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(SRG, "SD, RF, GPS LETSGO", 4096, NULL, 1, NULL, 0);

    Serial.println("-----| START! |-----");
    rf.print("Avionics Ready!");
}

void FlightControl(void *pvParameters)
{
    ControlData_t parachute_data;
    while(true) {
        // 제어 큐에서 데이터를 받아옴. 10ms 동안 데이터가 없으면 그냥 넘어감
        if (xQueueReceive(ControlQueue, &parachute_data, pdMS_TO_TICKS(10)) == pdPASS) {
            // 여기에 parachute_data.yaw 값을 이용한 카나드 제어 로직 구현
        }
            // vTaskDelay를 별도로 두지 않음. xQueueReceive가 최대 10ms 대기하므로 100Hz 주기가 맞춰짐.
    }
}

void ATTALT(void *pvParameters)
{
    uint32_t Timer = millis();
    float maxG = 0;

    // 데이터 전송용 구조체 인스턴스 생성
    ControlData_t control_data;
    BlackBoxData_t blackbox_data;
    ParachuteData_t parachute_data;

    while(true) {
        // IMU, 기압계 데이터 읽기
        if(Serial2.available()) {
            imu.parseData();
            imu.getRPY(blackbox_data.RPY[0], blackbox_data.RPY[1], blackbox_data.RPY[2]);
            imu.getAccelGyroMagFloat(blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag);
            maxG = max(maxG, blackbox_data.acc[2]);
        }
        if (Baro.isDataReady()) {
            if (Baro.performReading()) {
                Baro.getTempPressAlt(blackbox_data.baro[0], blackbox_data.baro[1], blackbox_data.baro[2]);
            }
        }

        control_data.yaw = blackbox_data.RPY[2];

        blackbox_data.timestamp = millis() - Timer;
        blackbox_data.maxG = maxG;

        parachute_data.timestamp = blackbox_data.timestamp;
        parachute_data.roll = blackbox_data.RPY[0];
        parachute_data.pitch = blackbox_data.RPY[1];
        parachute_data.altitude = blackbox_data.baro[2];

        // --- 큐로 데이터 전송 ---
        // xQueueOverwrite: 큐가 꽉 차있으면, 가장 오래된 데이터를 덮어쓰고 최신 데이터 삽입 (제어용으로 적합)
        xQueueOverwrite(ControlQueue, &control_data);
        xQueueOverwrite(ParachuteQueue, &parachute_data);

        // xQueueSend: 큐에 공간이 있을 때만 데이터 삽입.
        xQueueSend(BlackBoxQueue, &blackbox_data, pdMS_TO_TICKS(5)); // 5ms 동안 큐에 공간이 나길 기다림

        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz 유지
    }
}

void Parachute(void *pvParameters)
{
    ParachuteData_t parachute_data;
    EjectionData_t ejection_data;

    while(true) {
        // 사출 판단 큐에서 데이터를 받아옴
        if (xQueueReceive(ParachuteQueue, &parachute_data, portMAX_DELAY) == pdPASS) { // 새 데이터가 올 때까지 무한정 대기
            if(digitalRead(SAFETY_PIN) == LOW) { // 1차 안전장치
                if(digitalRead(LAUNCH_PIN) == LOW) { // 2차 안전장치
                    ejection_data.eject_type = chute.eject(sqrt(pow(parachute_data.roll, 2) + pow(parachute_data.pitch, 2)), parachute_data.altitude, parachute_data.timestamp, 0); // 0은 메시지 타입. 필요시 변경 가능
                    xQueueSend(EjectionQueue, &ejection_data, pdMS_TO_TICKS(5));
                }
            }
        }
    }
}

void SRG(void *pvParameters)
{
    // sd.initialize();
    BlackBoxData_t blackbox_data;
    GpsData gpsData;
    EjectionData_t ejection_data;

    while(true) {
        gps.get_gps_data(gpsData);

        if (xQueueReceive(BlackBoxQueue, &blackbox_data, 0) == pdPASS || xQueueReceive(EjectionQueue, &ejection_data, 0) == pdPASS) {
            sd.setData(blackbox_data.timestamp, blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag, blackbox_data.RPY, blackbox_data.maxG, blackbox_data.baro, gpsData, ejection_data.eject_type); // setData 함수를 구조체를 받도록 수정 필요
            sd.write_data();

            // RF로 데이터 전송
            char packet[100];
            int packet_len = 0;

            if(gps.is_updated) {
                packet_len = payload.get_imu_gps_packet(packet, blackbox_data.timestamp, blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag, blackbox_data.RPY, blackbox_data.baro, gpsData, ejection_data.eject_type); // get...packet 함수를 구조체를 받도록 수정 필요
            }
            else{
                packet_len = payload.get_imu_gps_packet(packet, blackbox_data.timestamp, blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag, blackbox_data.RPY, blackbox_data.baro, ejection_data.eject_type); // get...packet 함수를 구조체를 받도록 수정 필요
            }
            rf.transmit_packet(packet, packet_len);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // 5Hz 유지
    }
}

void loop() {}