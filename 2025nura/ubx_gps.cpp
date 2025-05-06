#include "ubx_gps.h"

UbxGPS::UbxGPS(int cs) : csPin(cs), new_update_flag(false) {}

void UbxGPS::initialize() {
    pinMode(csPin, OUTPUT);
    select();  // CS LOW
    delay(500);

    set_config(UBX_config::NAV5);
    set_config(UBX_config::RATE);
    set_config(UBX_config::PMS);

    disable_all_nmea(true);
    enable_ubx(UBX_ID::PVT);

    deselect();

    Serial.println("GPS SPI initialized!");
}

void UbxGPS::select() {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
}

void UbxGPS::deselect() {
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
}

char UbxGPS::spi_transfer_byte(char out) {
    return SPI.transfer(out);
}

bool UbxGPS::get_gps_data(GpsData &data) {
    bool packetReceived = false;
    int attempts = 0;  // 시도 횟수 초기화

    select();  // SPI 통신 시작

    while (attempts < 5) {
        byte = spi_transfer_byte();  // SPI에서 1바이트 읽기
        decode(byte);  // 디코딩 처리

        if (new_update_flag) {  // 새로운 업데이트가 있으면
            new_update_flag = false;
            data = gps;  // gps 데이터 업데이트
            packetReceived = true;
            break;  // 데이터가 있으면 빠져나옴
        }

        attempts++;  // 시도 횟수 증가
    }

    deselect();  // SPI 통신 끝

    return packetReceived;  // 성공 여부 반환
}



bool UbxGPS::is_updated() { return new_update_flag; }
char UbxGPS::is_fixed() { return gps.fixType; }

void UbxGPS::set_config(const char *cmd) {
    int len = cmd[4] + 8;
    for (int i = 0; i < len; i++) {
        spi_transfer_byte(cmd[i]);
    }
    delay(30);
}

void UbxGPS::disable_all_nmea(bool disable) {
    char packet[16] = {0xB5, 0x62};
    int idx = 2;
    for (int id = 0x00; id <= 0x0F; id++) {
        packet[idx++] = 0x06;
        packet[idx++] = 0x01;
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

void UbxGPS::decode(char byte) {
    static int idx = 0;
    static short payload_len = 0;
    static bool flag_comp = false;
    static char CK_A = 0, CK_B = 0;

    switch (idx) {
    case 0:
        if (byte == UBX_ID::HEADER1) buffer[idx++] = byte;
        else idx = 0;
        break;
    case 1:
        if (byte == UBX_ID::HEADER2) buffer[idx++] = byte;
        else idx = 0;
        break;
    case 2: case 3: case 4: case 5:
        buffer[idx++] = byte;
        CK_A += byte;
        CK_B += CK_A;
        if (idx == 5) payload_len = buffer[4];
        if (idx == 6) payload_len |= ((uint16_t)byte << 8);
        break;
    default:
        buffer[idx++] = byte;
        if (idx <= payload_len + 6) {
            CK_A += byte;
            CK_B += CK_A;
        }
    }

    if (idx >= payload_len + 8) {
        flag_comp = (buffer[payload_len+6] == CK_A && buffer[payload_len+7] == CK_B);
        if (flag_comp) {
            if (buffer[2] == UBX_ID::NAV) {
                if (buffer[3] == UBX_ID::PVT) parse_PVT(buffer);
                else if (buffer[3] == UBX_ID::POSLLH) parse_POSLLH(buffer);
            }
            new_update_flag = true;
        } else {
            Serial.println("Checksum Error!");
        }
        idx = 0; payload_len = 0; CK_A = 0; CK_B = 0; flag_comp = false;
    }
}

void UbxGPS::parse_PVT(char *packet) {
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

void UbxGPS::parse_POSLLH(char *packet) {
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

void UbxGPS::printGps() {
    if (get_gps_data(gps)) {
        Serial.print("위도: "); Serial.print(gps.lat, 7);
        Serial.print(", 경도: "); Serial.print(gps.lon, 7);
        Serial.println();
    }
}
