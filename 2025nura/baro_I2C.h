#ifndef I2C_H
#define I2C_H

#include <Wire.h>
#include <Arduino.h>

class I2C {
public:
    I2C(uint8_t addr, TwoWire *wire);
    bool begin(bool addr_detect = true);
    bool detected();
    bool read(uint8_t *buffer, size_t len, bool stop = true);
    bool _read(uint8_t *buffer, size_t len, bool stop = true);
    bool write(const uint8_t *buffer, size_t len, bool stop = true, const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0);
    bool write_then_read(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len, bool stop = false);
    size_t maxBufferSize();


private:
    TwoWire *_wire;
    uint8_t _addr;
    int _SCL, _SDA;
    bool _begun;
    size_t _maxBufferSize;
};

#endif