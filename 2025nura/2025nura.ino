/* ----------------------------------------
    2025 NURA Avionics
    - Avionics Team: K. JunHyeong, K. RangHyeon, K. YongJin, S. SeungMin
    - Last update: 2025.07.22
   ---------------------------------------- */

#include "EBIMU_AHRS.h"
#include "ubx_gps.h"
#include "BMP390L.h"
#include "SDLogger.h"
#include "NMT.h"
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

#define Kp 4
#define Ki 0.1
#define Kd 1

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

// 서보 선언
const int SERVO_FREQUENCY = 50;
const int PWM_RESOLUTION_BITS = 12;
const uint32_t MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION_BITS) - 1;

// bool sd_init = false;
bool set_launch_time = false;

struct ControlData_t{
  float yaw;
};

struct BlackBoxData_t{
  uint32_t timestamp;
  float acc[3], gyro[3], mag[3], RPY[3];
  float baro[3];
  float maxG;
};

struct ParachuteData_t{
  float roll, pitch;
  float altitude;
  uint32_t timestamp;
};

struct EjectionData_t{
    int8_t eject_type;
    int8_t launch_status; // 발사 상태 (0: 미발사, 1: 발사)
    // EjectionData_t(): eject_type(0), launch_status(0) {} // 생성자를 통한 초기화
};

QueueHandle_t ControlQueue;
QueueHandle_t BlackBoxQueue;
QueueHandle_t ParachuteQueue;
QueueHandle_t EjectionQueue;

void FlightControl(void *pvParameters);
void ATTALT(void *pvParameters);
void Parachute(void *pvParameters);
void SRG(void *pvParameters);

// TaskHandle_t task1;
// TaskHandle_t task2;
// TaskHandle_t task3;
// TaskHandle_t task4;

void servo_write_us(int channel, int pulse_us) {
    double period_us = 1000000.0 / SERVO_FREQUENCY;
    uint32_t duty = (uint32_t)(((double)pulse_us / period_us) * (double)MAX_DUTY_CYCLE);
    ledcWrite(channel, duty);
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

    ledcSetup(CH1, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcSetup(CH2, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcSetup(CH3, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
    ledcSetup(CH4, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);

    ledcAttachPin(CANARD1, CH1);
    ledcAttachPin(CANARD2, CH2);
    ledcAttachPin(CANARD3, CH3);
    ledcAttachPin(CANARD4, CH4);

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
    xTaskCreatePinnedToCore(SRG, "SD, RF, GPS LETSGO", 5120, NULL, 1, NULL, 0);
    // xTaskCreatePinnedToCore(printWatermark, "Watermark", 2048, NULL, 1, NULL, 0); // 스레드별 메모리 사용량 확인

    Serial.println("-----| START! |-----");
    rf.print("Avionics Ready!");
}

// void printWatermark(void *pvParameters){
//     while(1){
//         delay(2000);
//         Serial.print("TASK: ");
//         Serial.print(pcTaskGetName(task1)); // Get task name with handler
//         Serial.print(", High Watermark: ");
//         Serial.print(uxTaskGetStackHighWaterMark(task1));
//         Serial.println();
//         Serial.print("TASK: ");
//         Serial.print(pcTaskGetName(task2)); // Get task name with handler
//         Serial.print(", High Watermark: ");
//         Serial.print(uxTaskGetStackHighWaterMark(task2));
//         Serial.println();
//         Serial.print("TASK: ");
//         Serial.print(pcTaskGetName(task3)); // Get task name with handler
//         Serial.print(", High Watermark: ");
//         Serial.print(uxTaskGetStackHighWaterMark(task3));
//         Serial.println();
//         Serial.print("TASK: ");
//         Serial.print(pcTaskGetName(task4)); // Get task name with handler
//         Serial.print(", High Watermark: ");
//         Serial.print(uxTaskGetStackHighWaterMark(task4));
//         Serial.println();
//     }
// }

// PWM 1us 당 0,09도 회전
void FlightControl(void *pvParameters)
{
    ControlData_t control_data;

    double setpoint = 90.0; 
    double integral = 0.0;
    double previous_error = 0.0;
    uint32_t last_time = 0;

    last_time = micros();
    
    servo_write_us(CH1, 1500);
    servo_write_us(CH2, 1500);
    servo_write_us(CH3, 1500);
    servo_write_us(CH4, 1500);

    while(true) {
        // 제어 큐에서 데이터를 받아옴. 10ms 동안 데이터가 없으면 그냥 넘어감
        if (xQueueReceive(ControlQueue, &control_data, pdMS_TO_TICKS(10)) == pdPASS) {
            uint32_t current_time = micros();
            double dt = (current_time - last_time) / 1000000.0;
            if (dt <= 0) {
                dt = 0.01; // 최소 10ms로 설정
            }
            last_time = current_time;

            // P 제어
            double current_yaw = control_data.yaw;
            double error = setpoint - current_yaw;

            // I 제어
            integral += error * dt;
            integral = constrain(integral, -100, 100); // 파라미터(I 범위) 튜닝 필요. -> PD 제어를 사용하므로 일단 패스

            // D 제어
            double derivative = (error - previous_error) / dt;

            // PID 제어
            // double pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);

            // PD 제어
            double pid_output = (Kp * error) + (Kd * derivative);

            previous_error = error;

            // 서보 제어값 연산
            double control_angle = constrain(pid_output, -111, 111); // +- 9.99도 제한. 1us 당 0.09도 회전이므로, 111us가 최대 회전값

            servo_write_us(CH1, 1500 + control_angle);
            servo_write_us(CH2, 1500 + control_angle);
            servo_write_us(CH3, 1500 + control_angle);
            servo_write_us(CH4, 1500 + control_angle);

            // Serial.print("Control Angle: "); Serial.println(control_angle);
            // Serial.print(" | PID Output: "); Serial.println(pid_output);
        }
            // vTaskDelay를 별도로 두지 않음. xQueueReceive가 최대 10ms 대기하므로 100Hz 주기가 맞춰짐.
    }
}

void ATTALT(void *pvParameters)
{
    uint32_t Timer = millis();
    float maxG = 0;

    // 데이터 전송용 구조체 인스턴스 생성
    static ControlData_t control_data = {0};
    static BlackBoxData_t blackbox_data = {0};
    static ParachuteData_t parachute_data = {0};

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

    vTaskDelay(pdMS_TO_TICKS(7000)); // SD카드를 위해서 정지


    while(true) {
        // 사출 판단 큐에서 데이터를 받아옴
        if (xQueueReceive(ParachuteQueue, &parachute_data, portMAX_DELAY) == pdPASS) { // 새 데이터가 올 때까지 무한정 대기
            // if(digitalRead(SAFETY_PIN) == LOW) { // 1차 안전장치
            //     if(digitalRead(LAUNCH_PIN) == LOW) { // 2차 안전장치
                        if (!set_launch_time) {
                            ejection_data.launch_status = 1;
                            set_launch_time = true;
                            chute.set_launch_time(parachute_data.timestamp);
                        }
                        ejection_data.eject_type = chute.eject(sqrt(pow(parachute_data.roll, 2) + pow(parachute_data.pitch, 2)), parachute_data.altitude, parachute_data.timestamp, 0); // 0은 메시지 타입. 필요시 변경 가능
                        xQueueSend(EjectionQueue, &ejection_data, pdMS_TO_TICKS(5));
                // }
            // }
        }
    }
}

void SRG(void *pvParameters)
{
    sd.initialize();

    static BlackBoxData_t blackbox_data = {0};
    static GpsData gpsData;
    static EjectionData_t ejection_data = {0};


    while(true) {
        gps.get_gps_data(gpsData);

        xQueueReceive(BlackBoxQueue, &blackbox_data, 0);
        xQueueReceive(EjectionQueue, &ejection_data, 0);

        sd.setData(blackbox_data.timestamp, blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag, blackbox_data.RPY, blackbox_data.maxG, blackbox_data.baro, gpsData, ejection_data.eject_type, ejection_data.launch_status); // setData 함수를 구조체를 받도록 수정 필요
        sd.write_data();

        // RF로 데이터 전송
        char packet[100];
        int packet_len = 0;

        if(gps.is_updated()) {
            packet_len = payload.get_imu_gps_packet(packet, blackbox_data.timestamp, blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag, blackbox_data.RPY, blackbox_data.baro, gpsData, ejection_data.eject_type, ejection_data.launch_status); // get...packet 함수를 구조체를 받도록 수정 필요
        }
        else{
            packet_len = payload.get_imu_packet(packet, blackbox_data.timestamp, blackbox_data.acc, blackbox_data.gyro, blackbox_data.mag, blackbox_data.RPY, blackbox_data.baro, ejection_data.eject_type); // get...packet 함수를 구조체를 받도록 수정 필요
        }
        rf.transmit_packet(packet, packet_len);

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz 유지
    }
}

void loop() {
    // ESP32 남은 메모리 용량 확인용...
    // Serial.print("Current Free Heap: ");
    // Serial.print(ESP.getFreeHeap());
    // Serial.print(" bytes  |  ");

    // Serial.print("Minimum Free Heap since boot: ");
    // Serial.print(ESP.getMinFreeHeap());
    // Serial.println(" bytes");

    // delay(5000);
}