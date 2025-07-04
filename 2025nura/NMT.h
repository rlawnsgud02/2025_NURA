#ifndef NMT_H
#define NMT_H

#include <Arduino.h>

class NMT {
public:
    NMT(HardwareSerial& serial_port, int rx_pin, int tx_pin, unsigned long baud = 9600);

    void initialize();

    void transmit_packet(const String& data);    
    void transmit_packet(const char* data);
    void transmit_packet(const char* data, int len);

    void print(const char* str);

private:
    void sendCommand(const char* cmd, int wait_ms);

    HardwareSerial& RFserial; 
    unsigned long _baud;
    int _tx_pin;
    int _rx_pin; 
};

#endif // NMT_H