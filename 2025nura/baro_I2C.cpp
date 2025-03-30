#include "baro_I2C.h"


I2C::I2C(uint8_t addr, TwoWire *wire) {
    // _SCL = SCL;
    // _SDA = SDA;
    _addr = addr;
    _wire = wire;
    _begun = false;
    #ifdef ARDUINO_ARCH_SAMD
        _maxBufferSize = 250; // as defined in Wire.h's RingBuffer
    #elif defined(ESP32)
        _maxBufferSize = I2C_BUFFER_LENGTH;
    #else
        _maxBufferSize = 32;
    #endif
}

bool I2C::begin(bool addr_detect) {
    // if (_SCL >= 0 && _SDA >= 0) {
    //     _wire->begin(_SDA, _SCL);
    // }
    _begun = true;

    if (addr_detect) {
        return detected();
    }
    return true;
}

bool I2C::detected() {
    if (!_begun && !begin()) {
        return false;
    }

    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() == 0) {
        return true;
    }
    return false;
}

bool I2C::write(const uint8_t *buffer, size_t len, bool stop, const uint8_t *prefix_buffer, size_t prefix_len) {
    if ((len + prefix_len) > maxBufferSize()) {
        return false;
    }

    _wire->beginTransmission(_addr);

    if ((prefix_len != 0) && (prefix_buffer != nullptr)) {
        if (_wire->write(prefix_buffer, prefix_len) != prefix_len) {
            return false;
        }
    }

    if (_wire->write(buffer, len) != len) {
        return false;
    }

    if (_wire->endTransmission(stop) == 0) {
        return true;
    }

    else {
        return false;
    }
}

bool I2C::read(uint8_t *buffer, size_t len, bool stop) {
    size_t pos = 0;
    while (pos < len) {
        size_t read_len = ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
        bool read_stop = (pos < (len - read_len)) ? false : stop;
        if (!_read(buffer + pos, read_len, read_stop)) {
            return false;
        }
        pos += read_len;
    }
    return true;
}

bool I2C::_read(uint8_t *buffer, size_t len, bool stop) {

    size_t recv = _wire->requestFrom((uint8_t)_addr, (uint8_t)len, (uint8_t)stop);

    if (recv != len) {
        return false;
    }

    for (uint16_t i = 0; i < len; i++) {
        buffer[i] = _wire->read();
    }

    return true;
}

bool I2C::write_then_read(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len, bool stop) {
    if (!write(write_buffer, write_len, stop)) {
        return false;
    }
    return read(read_buffer, read_len);
}

size_t I2C::maxBufferSize() {
    return _maxBufferSize;
}