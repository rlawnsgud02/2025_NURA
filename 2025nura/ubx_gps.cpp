#include "ubx_gps.h"
#include <Wire.h>

// UbxGPS 클래스 생성자. SoftwareSerial을 사용한 버전은 주석처리
// UbxGPS::UbxGPS(SoftwareSerial& serial)
//     : GPSserial(serial), new_update_flag(false) {}

// I2C를 사용한 버전
UbxGPS::UbxGPS(TwoWire& port, int sda, int scl)
    : gpsPort(port), gpsSDA(sda), gpsSCL(scl), new_update_flag(false) {}

void UbxGPS::initialize() {
    Serial.println("\n-----| GPS Initializing.. |-----\n");

    // set_config(UBX_config::PRT);
    gpsPort.begin(gpsSDA, gpsSCL);

    Serial.println("Setting NAV config");
    set_config(UBX_config::NAV5);
    delay(50);
    
    Serial.println("Setting RATE config");
    set_config(UBX_config::RATE);
    delay(50);

    Serial.println("Setting PMS config");
    set_config(UBX_config::PMS);
    delay(50);

    disable_all_nmea(true);
    enable_ubx(UBX_ID::PVT);
    // enable_ubx(UBX_ID::POSLLH);

    Serial.println("\n-----| GPS Initialized! |-----\n");
    delay(500);
}

// bool UbxGPS::get_gps_data(GpsData &data) { // GPS 데이터를 업데이트하고 인자로 받은 변수에 구조체로 저장한다.   
//     bool packetReceived = false;

//     while (GPSserial.available()) {
//         byte = GPSserial.read();
//         decode(byte);
//         if (new_update_flag) {
//             new_update_flag = false;
//             data = gps;
//             packetReceived = true;
//         }
//     }

//     return packetReceived;
// }

bool UbxGPS::get_gps_data(GpsData &data) {
    bool packetReceived = false;
    uint16_t bytesAvailable = 0;

    // 1. u-blox 모듈의 0xFD 레지스터를 읽어 수신 가능한 데이터 바이트 수를 확인합니다.
    gpsPort.beginTransmission(UBLOX_ADDR);
    gpsPort.write(0xFD);
    if (gpsPort.endTransmission(false) != 0) {
        return false; // I2C 통신 오류
    }

    if (gpsPort.requestFrom((uint8_t)UBLOX_ADDR, (uint8_t)2) == 2) {
        uint8_t msb = gpsPort.read();
        uint8_t lsb = gpsPort.read();
        bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    // 2. 수신할 데이터가 있으면 0xFF 레지스터에서 데이터를 읽어옵니다.
    if (bytesAvailable > 0) {
        gpsPort.requestFrom((uint8_t)UBLOX_ADDR, bytesAvailable);

        while (gpsPort.available()) {
            byte = gpsPort.read();
            decode(byte); // 읽어온 바이트를 디코더로 넘깁니다.
            if (new_update_flag) {
                new_update_flag = false;
                data = gps;
                packetReceived = true;
            }
        }
    }

    return packetReceived;
}

bool UbxGPS::is_updated() {
    return new_update_flag;
}

char UbxGPS::is_fixed() {
    return gps.fixType;
}

void UbxGPS::set_config(const char *cmd) {
    // int len = cmd[4] + 8; // UBX 메시지 길이 계산
    // for (int i = 0; i < len; i++) {
    //     GPSserial.write(cmd[i]);
    // }
    // delay(30);

    uint16_t payload_len = (uint16_t)cmd[5] << 8 | (uint8_t)cmd[4];
    int total_len = payload_len + 8;

    gpsPort.beginTransmission(UBLOX_ADDR);
    gpsPort.write((const uint8_t*)cmd, total_len);
    if (gpsPort.endTransmission() != 0) {
        Serial.println("GPS Set Config Failed!");
    }
    delay(100);
}

void UbxGPS::disable_all_nmea(bool disable) {
    char packet[16] = {0xB5, 0x62};
    int idx = 2;

    for (int id = 0x00; id <= 0x0F; id++) {
        packet[idx++] = 0x06;  // UBX_CLASS_CFG
        packet[idx++] = 0x01;  // UBX_ID_MSG
        packet[idx++] = 0x08;  // Length LSB
        packet[idx++] = 0x00;  // Length MSB
        packet[idx++] = 0xF0;  // Payload: NMEA Class
        packet[idx++] = id;    // Payload: NMEA ID

        char config = disable ? 0x00 : 0x01;
        for (int i = 0; i < 6; i++) {
            packet[idx++] = config;
        }

        // 체크섬 계산
        char ck_a = 0, ck_b = 0;
        for (int i = 2; i < 14; i++) {
            ck_a = ck_a + packet[i];
            ck_b = ck_b + ck_a;
        }
        packet[14] = ck_a;
        packet[15] = ck_b;

        set_config(packet);
        idx = 2; // 다음 루프를 위해 인덱스 초기화
    }
}

void UbxGPS::enable_ubx(char id_) {
    char packet[16] = {0xB5, 0x62};
    int idx = 2;

    packet[idx++] = 0x06;  // UBX_CLASS_CFG
    packet[idx++] = 0x01;  // UBX_ID_MSG
    packet[idx++] = 0x08;  // Length LSB
    packet[idx++] = 0x00;  // Length MSB
    packet[idx++] = 0x01;  // Payload: UBX-NAV Class
    packet[idx++] = id_;   // Payload: Target UBX ID

    for (int i = 0; i < 6; i++) {
        packet[idx++] = 0x01;
    }

    // 체크섬 계산
    char ck_a = 0, ck_b = 0;
    for (int i = 2; i < 14; i++) {
        ck_a = ck_a + packet[i];
        ck_b = ck_b + ck_a;
    }
    packet[14] = ck_a;
    packet[15] = ck_b;

    set_config(packet);
}

// decode는 while문에서 반복적으로 실행되며 바이트 단위로 decoding을 진행한다.
void UbxGPS::decode(char byte) 
{
    static int idx = 0;  // 진행도 저장 변수. static으로 선언되었기에 재호출되어도 값이 초기화되진 않음!
    static short payload_len = 0;  
    static bool flag_comp = false;  
    static char CK_A = 0, CK_B = 0;  // Checksum 계산

    switch(idx)
    {
    case 0: // check header1
        if(byte == UBX_ID::HEADER1) {
            buffer[idx++] = byte;
        } else {
            idx = 0;
        }
        break;
    case 1: // check header2
        if(byte == UBX_ID::HEADER2) {
            buffer[idx++] = byte;
        } else {
            idx = 0;
        }
        break;
    case 2: // save packet class. ex) NAV
    case 3: // save packet id
        buffer[idx++] = byte;
        CK_A += byte;
        CK_B += CK_A;
        break;
    case 4: // save length (low byte)
        buffer[idx++] = byte;
        CK_A += byte;
        CK_B += CK_A;
        payload_len = byte;
        break;
    case 5: // save length2 (high byte)
        buffer[idx++] = byte;
        CK_A += byte;
        CK_B += CK_A;
        payload_len |= ((uint16_t)byte << 8);
        break;

    default:
        buffer[idx++] = byte;

        // 체크섬을 제외한 데이터만 계산
        if(idx <= payload_len + 6) {
            CK_A += byte;
            CK_B += CK_A;
        }
    }
    
    if(idx >= payload_len + 8) { // 패킷 수신 완료
        flag_comp = (buffer[payload_len+6] == CK_A && buffer[payload_len+7] == CK_B);

        if (!flag_comp) {
            Serial.println("Checksum Error!"); // 디버깅용 로그
        }
        else 
        {
            char packet_class = buffer[2];
            char packet_id = buffer[3];
    
            if(packet_class == UBX_ID::NAV) 
            {
                if(packet_id == UBX_ID::PVT)
                {
                    parse_PVT(buffer);
                }
                else if(packet_id == UBX_ID::POSLLH) 
                {
                    parse_POSLLH(buffer);
                }
            }
            new_update_flag = true;
        }

        idx = 0; // 패킷 하나에 대한 수신을 완료했으므로 진행도 초기화.
        payload_len = 0;
        CK_A = 0;
        CK_B = 0;
        flag_comp = false;
    }
}


void UbxGPS::parse_PVT(char *packet) { //PVT 데이터 파싱. PVT는 100바이트.
    // gps.iTOW = byte_to_int(packet+6, 4);
    // gps.year = (short)byte_to_int(packet+10, 2);
    // gps.month = packet[12];
    // gps.day = packet[13];
    // gps.hour = packet[14];
    // gps.min = packet[15];
    // gps.sec = packet[16];
    gps.fixType = packet[26]; //
    gps.lon = byte_to_int(packet + 30, 4) / 1e7; //
    gps.lat = byte_to_int(packet + 34, 4) / 1e7; //
    gps.height = byte_to_int(packet + 38, 4) / 1000.0; //
    gps.velN = byte_to_int(packet + 54, 4) / 1000.0; //
    gps.velE = byte_to_int(packet + 58, 4) / 1000.0; //
    gps.velD = byte_to_int(packet + 62, 4) / 1000.0; //
}

void UbxGPS::parse_POSLLH(char *packet) { //POSLLH 데이터 파싱
    gps.lon = byte_to_int(packet + 10, 4) / 1e7;
    gps.lat = byte_to_int(packet + 14, 4) / 1e7;
    gps.height = byte_to_int(packet + 18, 4) / 1000.0;
}

int UbxGPS::byte_to_int(const char *ptr, int len) {
    int val = 0;
    for (int i = 0; i < len; i++) {
        val |= ((uint8_t)ptr[i]) << (8 * i);
    }
    return val;
}

// 디버깅용 GPS 데이터 출력
void UbxGPS::printGps() {
    if(get_gps_data(gps)){ // print 전에 GPS 데이터 업데이트. 유효한 경우에만 출력력
        Serial.print("위도: "); Serial.print(gps.lat, 7); // 소수점 7자리까지 출력
        Serial.print(", 경도: "); Serial.print(gps.lon, 7); // 소수점 7자리까지 출력
        Serial.println(); // 그냥 줄바꾸기
    } 
}