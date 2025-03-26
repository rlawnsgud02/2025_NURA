#include "ubx_gps.h"

// Ublox M9N을 이용하여 UBX-NAV-PVT 데이터를 받는 클래스
// Last update: 2025.03.24

UbxGPS::UbxGPS(SoftwareSerial& serial)
    : GPSserial(serial), new_update_flag(false) {}


void UbxGPS::initialize() {
    Serial.println("\n-----| GPS Initializing.. |-----\n");

    set_config(UBX_config::PRT);
    set_config(UBX_config::NAV5);
    set_config(UBX_config::RATE);
    set_config(UBX_config::PMS);

    disable_all_nmea(true);
    enable_ubx(UBX_ID::PVT);
    // enable_ubx(UBX_ID::POSLLH);

    Serial.println("\n\n-----| GPS Initialized! |-----\n");
    delay(500);
}

bool UbxGPS::get_gps_data(GpsData &data) { // GPS 데이터를 업데이트하고 인자로 받은 변수에 구조체로 저장한다.   
    if(!GPSserial.available()) return false; // 데이터가 없는 경우 false 반환하고 바로 종료

    while(GPSserial.available()) { // 데이터가 존재할 경우 1 바이트씩 decoding을 진행한다.
        char byte = GPSserial.read();
        decode(byte);
    }
    data = gps;
    return true;
}

bool UbxGPS::is_updated() {
    return new_update_flag;
}

char UbxGPS::is_fixed() {
    return gps.fixType;
}

void UbxGPS::set_config(const char *cmd) {
    int len = cmd[4] + 8; // UBX 메시지 길이 계산
    for (int i = 0; i < len; i++) {
        GPSserial.write(cmd[i]);
    }
    delay(30);
    // 일단 다른 문제가 없으면 아래 코드 시도해보기 (더 효율적적)
    // serial.write((const uint8_t*)cmd, len); // 바이트 배열을 한번에 전송
    // serial.flush(); // 모든 데이터가 전송될 때까지 대기
}

void UbxGPS::disable_all_nmea(bool disable) { //모든 NMEA 비활성화. UBX 프로토콜만 사용할 것이므로 NMEA는 비활성화
    char packet[16] = {0xB5, 0x62};
    int idx = 2;

    for (int id = 0x00; id <= 0x0F; id++) {
        packet[idx++] = 0x06;  // UBX_CLASS_CFG
        packet[idx++] = 0x01;  // UBX_ID_MSG
        packet[idx++] = 0x08;
        packet[idx++] = 0x00;
        packet[idx++] = 0xF0;
        packet[idx++] = id;

        char config = disable ? 0x00 : 0x01;
        for (int i = 0; i < 5; i++) {
            packet[idx++] = config;
        }
        packet[idx++] = 0x01;

        set_config(packet);
        idx = 2;
    }
}

void UbxGPS::enable_ubx(char id_) {
    char packet[16] = {0xB5, 0x62};
    int idx = 2;

    packet[idx++] = 0x06;
    packet[idx++] = 0x01;
    packet[idx++] = 0x08;
    packet[idx++] = 0x00;
    packet[idx++] = 0x01;
    packet[idx++] = id_;

    for (int i = 0; i < 5; i++) {
        packet[idx++] = 0x01;
    }
    packet[idx++] = 0x01;

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
    gps.iTOW = byte_to_int(packet+6, 4);
    gps.year = (short)byte_to_int(packet+10, 2);
    gps.month = packet[12];
    gps.day = packet[13];
    gps.hour = packet[14];
    gps.min = packet[15];
    gps.sec = packet[16];
    gps.fixType = packet[26];
    gps.lon = byte_to_int(packet + 30, 4) / 1e7;
    gps.lat = byte_to_int(packet + 34, 4) / 1e7;
    gps.height = byte_to_int(packet + 38, 4) / 1000.0;
    gps.velN = byte_to_int(packet + 54, 4) / 1000.0;
    gps.velE = byte_to_int(packet + 58, 4) / 1000.0;
    gps.velD = byte_to_int(packet + 62, 4) / 1000.0;
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