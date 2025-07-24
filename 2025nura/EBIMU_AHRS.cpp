#include "EBIMU_AHRS.h"

//생성자
EBIMU_AHRS::EBIMU_AHRS(HardwareSerial& serial, int rxPin, int txPin, long baudRate)
    : IMU_Serial(serial), rxPin(rxPin), txPin(txPin), baudRate(baudRate), roll(0), pitch(0), yaw(0), accelZ(0) {memset(data, 0, sizeof(data));}

void EBIMU_AHRS::initialize() {
    // Serial.println("\n\n-----| Initializing IMU... |-----\n");

    // IMU_Serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    // delay(2000); // 안정적인 시작을 위한 대기

    // Serial.println("\n-----| Configuring IMU... |-----\n");
    // // sendCommand("<soc1>"); // ASCII 모드 설정
    // // sendCommand("<soc2>"); // Binary 모드 설정
    // // sendCommand("<sor10>"); // 100Hz 출력 속도 설정
    // // sendCommand("<sof1>"); // Euler Angle(RPY) 출력
    // // sendCommand("<sog1>"); // 자이로 출력 활성화
    // // sendCommand("<soa1>"); // 가속도 출력 활성화 (X, Y, Z 전체). RPY값 뒤에 출력됨.
    // // sendCommand("<som1>"); // 지자기 출력 활성화
    // sendCommand("<cmoz>"); // YAW 오프셋 제거

    // Serial.println("\n-----| IMU Initialize Done! |-----\n");

    Serial.println("\n\n-----| Initializing IMU... |-----\n");

    IMU_Serial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
    delay(1000); // 시리얼 포트 안정화를 위한 대기

    Serial.println("\n-----| Stopping IMU stream for configuration... |-----\n");
    IMU_Serial.println("<stop>"); // 데이터 스트림 중지
    clearIMURxBuffer();           // <ok> 응답 및 버퍼에 쌓인 데이터 모두 비우기

    Serial.println("\n-----| Configuring IMU... |-----\n");
    // 이제 IMU는 데이터 스트림 없이 <ok> 응답만 보냅니다.
    sendCommand("<soc2>");   // Binary 모드 설정
    sendCommand("<sor10>");  // 10Hz 출력 속도 설정 (데이터시트상 10ms 주기)
    sendCommand("<sof1>");   // Euler Angle(RPY) 출력
    sendCommand("<sog1>");   // 자이로 출력 활성화
    sendCommand("<soa1>");   // 가속도 출력 활성화
    sendCommand("<som1>");   // 지자기 출력 활성화
    sendCommand("<cmoz>");   // YAW 오프셋 제거

    Serial.println("\n-----| Restarting IMU stream... |-----\n");
    IMU_Serial.println("<start>"); // 설정 완료 후 데이터 스트림 재시작
    clearIMURxBuffer();            // 마지막 <ok> 응답 비우기

    Serial.println("\n-----| IMU Initialize Done! |-----\n");
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

        // 데이터의 출력 순서는 rpy, acc, gyro, mag. 데이터 시트에 적혀 있음.
        for (int i = 0; i < 11; i++) {
            nextComma = imuData.indexOf(',', lastComma);
            if (nextComma == -1) return; // 파싱 중 오류 발생 시 종료
            data[i] = imuData.substring(lastComma, nextComma).toFloat();
            lastComma = nextComma + 1;
        }

        // 마지막 값 (magZ)은 쉼표 없이 줄 끝에 있으므로 따로 처리
        data[11] = imuData.substring(lastComma).toFloat();

        // 데이터 분리. 데이터 시트에 명신된 데이터 출력 순서임.
        roll    = data[0];
        pitch   = data[1];
        yaw     = data[2];

        gyroX  = data[3];
        gyroY  = data[4];
        gyroZ  = data[5];

        accelX  = data[6];
        accelY  = data[7];
        accelZ  = data[8];

        magX    = data[9];
        magY    = data[10];
        magZ    = data[11];
    }
}

void EBIMU_AHRS::parseDataBinary() {
    // 데이터시트(p.13) 기반 상수 정의
    const int PACKET_SIZE = 28;
    const uint8_t SOP = 0x55;  

    // 시리얼 버퍼에 완전한 패킷이 수신될 때까지 대기
    while (IMU_Serial.available() >= PACKET_SIZE) {

        // 1. 패킷 시작점(SOP) 찾기: 0x55 0x55
        // 데이터 스트림 동기화를 위해 SOP를 먼저 찾습니다.
        if (IMU_Serial.read() == SOP && IMU_Serial.read() == SOP) {
            
            // 2. 패킷 데이터 읽기
            // SOP를 제외한 나머지 데이터(값, 체크섬)를 버퍼로 읽어옵니다.
            uint8_t buffer[PACKET_SIZE - 2];
            IMU_Serial.readBytes(buffer, sizeof(buffer));

            // 3. 체크섬(Checksum) 검증
            // 데이터시트(p.13)에 따라 SOP를 포함한 모든 데이터의 합을 계산합니다[cite: 266].
            uint16_t calculatedChecksum = SOP + SOP;
            for (int i = 0; i < (PACKET_SIZE - 4); i++) { // 데이터 부분(24바이트)만 더함
                calculatedChecksum += buffer[i];
            }

            // 수신된 체크섬 값을 16비트 정수로 변환
            uint16_t receivedChecksum = ((uint16_t)buffer[24] << 8) | buffer[25];

            // 계산된 체크섬과 수신된 체크섬이 다르면 데이터 오류로 판단하고 패킷을 버립니다.
            if (calculatedChecksum != receivedChecksum) {
                continue; // 다음 패킷을 찾아 계속 진행
            }

            // 4. 데이터 파싱 및 스케일링
            // 체크섬이 유효하면, 2바이트씩 묶어 16비트 정수로 변환 후,
            // 데이터시트의 스케일링 팩터에 따라 실수(float)로 변환합니다.
            int16_t rawValue;
            
            // [데이터 순서] 기존 parseData와 동일하게 RPY -> Gyro -> Accel -> Mag 순으로 처리
            for(int i = 0; i < 3; i++) {
                rawValue = (int16_t)(((uint16_t)buffer[i*2] << 8) | buffer[i*2 + 1]);
                data[i] = rawValue / 100.0f;
            }
            for(int i = 3; i < 6; i++) {
                rawValue = (int16_t)(((uint16_t)buffer[i*2] << 8) | buffer[i*2 + 1]);
                data[i] = rawValue / 10.0f;
            }
            for(int i = 6; i < 9; i++) {
                rawValue = (int16_t)(((uint16_t)buffer[i*2] << 8) | buffer[i*2 + 1]);
                data[i] = rawValue / 1000.0f;
            }
            for(int i = 9; i < 12; i++) {
                rawValue = (int16_t)(((uint16_t)buffer[i*2] << 8) | buffer[i*2 + 1]);
                data[i] = rawValue / 10.0f;
            }

            // 5. 파싱된 데이터를 클래스 멤버 변수에 할당
            roll    = data[0]; pitch   = data[1]; yaw     = data[2];
            gyroX   = data[3]; gyroY   = data[4]; gyroZ   = data[5];
            accelX  = data[6]; accelY  = data[7]; accelZ  = data[8];
            magX    = data[9]; magY    = data[10]; magZ    = data[11];
            
            // 성공적으로 패킷 하나를 파싱했으므로 함수 종료
            return; 
        }
    }
}


void EBIMU_AHRS::printData() {
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.print(yaw);
    Serial.print(" | AccelX: "); Serial.print(accelX);
    Serial.print(" | AccelY: "); Serial.print(accelY);
    Serial.print(" | AccelZ: "); Serial.print(accelZ);
    Serial.print(" | GyroX: "); Serial.print(gyroX);
    Serial.print(" | GyroY: "); Serial.print(gyroY);
    Serial.print(" | GyroZ: "); Serial.print(gyroZ);
    Serial.print(" | MagX: "); Serial.print(magX);
    Serial.print(" | MagY: "); Serial.print(magY);
    Serial.print(" | MagZ: "); Serial.println(magZ);
}

void EBIMU_AHRS::getRPY(float &r, float &p, float &y) {
    r = roll;
    p = pitch;
    y = yaw;
}

void EBIMU_AHRS::getAccelGyroMagFloat(float* accel, float* gyro, float* mag) {
    accel[0] = accelX;
    accel[1] = accelY;
    accel[2] = accelZ;

    gyro[0] = gyroX;
    gyro[1] = gyroY;
    gyro[2] = gyroZ;

    mag[0] = magX;
    mag[1] = magY;
    mag[2] = magZ;
}

// 주의! 캘리브레이션은 아직 완성도가 낮기에 사용에 주의가 필요함.
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

float EBIMU_AHRS::get_anglegro() {
    return sqrt(pow(roll, 2) + pow(pitch, 2));
}

void EBIMU_AHRS::clearIMURxBuffer() {
  delay(100); // 응답이 도착할 시간을 잠시 기다립니다.
  while (IMU_Serial.available() > 0) {
    IMU_Serial.read();
  }
}