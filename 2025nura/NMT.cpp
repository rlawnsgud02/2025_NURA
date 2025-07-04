#include "NMT.h"

NMT::NMT(HardwareSerial& serial_port, int rx_pin, int tx_pin, unsigned long baud)
    : RFserial(serial_port), _rx_pin(rx_pin), _tx_pin(tx_pin), _baud(baud) {
}

void NMT::initialize() {
    RFserial.begin(_baud, SERIAL_8N1, _rx_pin, _tx_pin);
    delay(200);

    if (Serial) {
        Serial.println("RF Initializing...");
    }

    sendCommand("++++\r\n", 500);
    sendCommand("AT+ACODE=00000000\r\n", 200);
    sendCommand("AT+FBND=1\r\n", 200);    // RF 주파수 대역 설정
    sendCommand("AT+CHN=2\r\n", 200);     // RF 채널 설정
    sendCommand("AT+HPERF=1\r\n", 200);   // 고성능 수신 모드 활성화
    // sendCommand("AT+ID?\r\n", 200);
    sendCommand("AT+RST=1\r\n", 500);

    if (Serial) {
        Serial.println("RF Initialize Complete");
        RFserial.print("RF Initialize Complete\r\n");
    }
}

// 데이터 패킷 전송 함수 (String 버전)
// void NMT::transmit_packet(const String& data) {
//     RFserial.print(data + "\r\n");
// }

// 데이터 패킷 전송 함수 (const char* 버전). 좀 더 고인물 버전. 잘 활용 시 메모리 절약 가능.
// void NMT::transmit_packet(const char* data) {
//     RFserial.print(data);
//     RFserial.print("\r\n");
// }

void NMT::transmit_packet(const char* data, int len) {
    RFserial.write(data, len);
    RFserial.print("\r\n");
}


void NMT::sendCommand(const char* cmd, int wait_ms) {
    RFserial.print(cmd);
    delay(wait_ms);     

    if (Serial) {
        while (RFserial.available()) {
            char c = RFserial.read();
            Serial.write(c);
        }
    } else {
        while (RFserial.available()) {
            RFserial.read();
        }
    }
}

void NMT::print(const char* str) {
    RFserial.print(str);
    // RFserial.print("\r\n");
}