#include "EBIMU_AHRS.h"

//생성자
EBIMU_AHRS::EBIMU_AHRS(HardwareSerial& serial, int rxPin, int txPin, long baudRate)
    : IMU_Serial(serial), rxPin(rxPin), txPin(txPin), baudRate(baudRate), roll(0), pitch(0), yaw(0), accelZ(0) {memset(data, 0, sizeof(data));}

void EBIMU_AHRS::initialize() {
    Serial.println("\n\n------| Initializing IMU... |------\n");

    IMU_Serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    delay(2000); // 안정적인 시작을 위한 대기

    Serial.println("\n------| Configuring IMU... |------\n");
    sendCommand("<soc1>"); // ASCII 모드 설정
    sendCommand("<sor10>"); // 100Hz 출력 속도 설정
    sendCommand("<sof1>"); // Euler Angle(RPY) 출력
    sendCommand("<soa1>"); // 가속도 데이터 출력 활성화 (X, Y, Z 전체). RPY값 뒤에 출력됨.


    Serial.println("\n------| IMU Initialize Done! |------\n");
}

void EBIMU_AHRS::sendCommand(const char* command) {
    IMU_Serial.println(command);
    delay(100);
    while (IMU_Serial.available()) {
        String response = IMU_Serial.readStringUntil('\n');
        Serial.println("IMU Response: " + response);
    }
}

String EBIMU_AHRS::readData() {
    if (IMU_Serial.available()) {
        return IMU_Serial.readStringUntil('\n');
    }
    return "";
}

void EBIMU_AHRS::parseData() {
    String imuData = readData();
    if (imuData.length() > 0 && imuData.startsWith("*")) {
        imuData.remove(0, 1); // '*' 제거

        int lastComma = 0;
        int nextComma;

        // 데이터 파싱 (쉼표 단위로 끊어서 배열에 저장)
        for (int index = 0; index < 5; index++) {
            nextComma = imuData.indexOf(',', lastComma);
            if (nextComma == -1) break; // 더 이상 쉼표가 없으면 종료
            data[index] = imuData.substring(lastComma, nextComma).toFloat();
            lastComma = nextComma + 1;
        }
        data[5] = imuData.substring(lastComma).toFloat();

        //데이터 저장
        roll = data[0];
        pitch = data[1];
        yaw = data[2];
        accelZ = data[5]; // 마지막 값이 Z축 가속도
    }
}

void EBIMU_AHRS::printData() {
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.print(yaw);
    Serial.print(" | AccelZ: "); Serial.println(accelZ);
}

void EBIMU_AHRS::getRPY(float &r, float &p, float &y) {
    r = roll;
    p = pitch;
    y = yaw;
}

float EBIMU_AHRS::getRoll() {
    return roll;
}

float EBIMU_AHRS::getPitch() {
    return pitch;
}

float EBIMU_AHRS::getYaw() {
    return yaw;
}

float EBIMU_AHRS::getAccelZ() {
    return accelZ;
}

//주의! 캘리브레이션은 아직 완성도가 낮기에 사용에 주의가 필요함.
// 자이로 캘리브레이션
void EBIMU_AHRS::calibrateGyro() {
    Serial.println("Calibrating Gyro...");
    sendCommand("<cg>");
    Serial.println("Gyro Calibration Complete.");
}

// 가속도 캘리브레이션
void EBIMU_AHRS::calibrateAccel() {
    Serial.println("Calibrating Accelerometer...");
    sendCommand("<caf>");
    Serial.println("Accelerometer Calibration Complete.");
}

// 지자기 캘리브레이션
void EBIMU_AHRS::calibrateMagneto() {
    Serial.println("Calibrating Magnetometer...");

    sendCommand("<cmf>");
    delay(20000); // 지자기 캘리브레이션은 20초 정도 소요. 
    sendCommand(">"); // 캘리브레이션 종료

    Serial.println("Magnetometer Calibration Complete.");
}

// 모든 센서 캘리브레이션
void EBIMU_AHRS::calibrateAll() {
    Serial.println("Starting Full Sensor Calibration...");
    calibrateGyro();
    delay(1000);
    calibrateAccel();
    delay(1000);
    calibrateMagneto();
    Serial.println("All Sensor Calibration Complete.");
}