#include "BMP390L.h"
#include "Arduino.h"

Adafruit_I2CDevice *g_i2c_dev = NULL; ///< Global I2C interface pointer

static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void delay_usec(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);

BMP390L::BMP390L() {
    _meas_end = 0;
    _filterEnabled = _tempOSEnabled = _presOSEnabled = false;
    AltitudeBias = 0.0f;
}

bool BMP390L::begin_I2C(uint8_t addr, int SDA, int SCL) {
    if (i2c_dev) {
        delete i2c_dev;
    }

    // Wire 포인터를 로컬에서 새로 선언 (기본은 Wire)
    TwoWire *wire = &Wire;

    if (SDA >= 0 && SCL >= 0) {
        wire->begin(SDA, SCL);
    }

    g_i2c_dev = i2c_dev = new Adafruit_I2CDevice(addr, wire);

    if (!i2c_dev->begin()) {
        return false;
    }

    the_sensor.chip_id = addr;
    the_sensor.intf = BMP3_I2C_INTF;
    the_sensor.read = &i2c_read;
    the_sensor.write = &i2c_write;
    the_sensor.intf_ptr = i2c_dev;
    the_sensor.dummy_byte = 0;

    return _init();
}


bool BMP390L::_init() {
    g_i2c_dev = i2c_dev;
    the_sensor.delay_us = delay_usec;
    int8_t rslt = BMP3_OK;

    rslt = bmp3_soft_reset(&the_sensor);

    if (rslt != BMP3_OK) {
        return false;
    }

    rslt = bmp3_init(&the_sensor);

    rslt = validate_trimming_param(&the_sensor);

    if (rslt != BMP3_OK) {
        return false;
    }

    setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    setPressureOversampling(BMP3_NO_OVERSAMPLING);
    setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    setOutputDataRate(BMP3_ODR_25_HZ);
    the_sensor.settings.op_mode = BMP3_MODE_FORCED;

    return true;
}

static int8_t validate_trimming_param(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;

    rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);

    if (rslt == BMP3_OK) {
        for (i = 0; i < 21; i++) {
            crc = (uint8_t)cal_crc(crc, trim_param[i]);
        }
        crc = (crc^0xFF);

        rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);

        if (stored_crc != crc) {
           rslt = -1;
        }
    }

    return rslt;
}

static int8_t cal_crc(uint8_t seed, uint8_t data) {
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if ((seed & 0x80) ^ (data & 0x80)) {
            var2 = 1;
        } else {
            var2 = 0;
        }
        
        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t)(poly * var2);
    }

    return (int8_t)seed;
}

bool BMP390L::setTemperatureOversampling(uint8_t oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X) {
        return false;
    }

    the_sensor.settings.odr_filter.temp_os = oversample;

    if (oversample == BMP3_NO_OVERSAMPLING) {
        _tempOSEnabled = false;
    } else {
        _tempOSEnabled = true;
    }

    return true;
}

bool BMP390L::setPressureOversampling(uint8_t oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X) {
        return false;
    }

    the_sensor.settings.odr_filter.press_os = oversample;

    if (oversample == BMP3_NO_OVERSAMPLING) {
        _presOSEnabled = false;
    } else {
        _presOSEnabled = true;
    }

    return true;
}

bool BMP390L::setIIRFilterCoeff(uint8_t filtercoeff) {
    if (filtercoeff > BMP3_IIR_FILTER_COEFF_127) {
        return false;
    }

    the_sensor.settings.odr_filter.iir_filter = filtercoeff;

    if (filtercoeff == BMP3_IIR_FILTER_DISABLE) {
        _filterEnabled = false;
    } else {
        _filterEnabled = true;
    }

    return true;
}

bool BMP390L::setOutputDataRate(uint8_t odr) {
    if (odr > BMP3_ODR_0_001_HZ) {
        return false;
    }

    the_sensor.settings.odr_filter.odr = odr;

    _ODREnabled = true;

    return true;
}

bool BMP390L::performReading() {
    g_i2c_dev = i2c_dev;
    int8_t rslt;
    struct bmp3_data data;
    uint16_t setting_sel = 0;
    uint8_t sensor_comp = 0;

    the_sensor.settings.temp_en = BMP3_ENABLE;
    setting_sel |= BMP3_SEL_TEMP_EN;
    sensor_comp |= BMP3_TEMP;

    if (_tempOSEnabled) {
        setting_sel |= BMP3_SEL_TEMP_OS;
    }

    the_sensor.settings.press_en = BMP3_ENABLE;
    setting_sel |= BMP3_SEL_PRESS_EN;
    sensor_comp |= BMP3_PRESS;

    if(_presOSEnabled) {
        setting_sel |= BMP3_SEL_PRESS_OS;
    }

    if(_filterEnabled) {
        setting_sel |= BMP3_SEL_IIR_FILTER;
    }

    if(_ODREnabled) {
        setting_sel |= BMP3_SEL_ODR;
    }

    rslt = bmp3_set_sensor_settings(setting_sel, &the_sensor);
    
    if(rslt != BMP3_OK) {
        return false;
    }

    the_sensor.settings.op_mode = BMP3_MODE_FORCED;
    rslt = bmp3_set_op_mode(&the_sensor);

    if(rslt != BMP3_OK) {
        return false;
    }
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);
    if(rslt != BMP3_OK) {
        return false;
    }
    Temperature = data.temperature;
    Pressure = data.pressure;

    return true;
}

float BMP390L::readTemperature() {
    return Temperature;
}

float BMP390L::readPressure() {
    return Pressure;
}

void BMP390L::set_Altitude(float seaLevel) {
    float atmospheric = Pressure/100.0F;
    Altitude = 44330 * (1.0 - powf(atmospheric / seaLevel, 0.1903));    
}

float BMP390L::readAltitude(int8_t portion) {
    set_Altitude(1013.25);
    if (portion == 0) {
        return Altitude;
    }
    else {
        return Altitude - AltitudeBias;
    }
    return 0.0f;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    if (!g_i2c_dev->write((uint8_t *)reg_data, len, true, &reg_addr, 1)) {
        return 1;
    }

    return 0;
}

int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {

    if (!g_i2c_dev->write_then_read(&reg_addr, 1, reg_data, len)) {
        return 1;
    }
    return 0;
}

static void delay_usec(uint32_t us, void *intf_ptr) { 
    delayMicroseconds(us); 
}

bool BMP390L::set_AltitudeBias() {
    int count = 0;
    Serial.println("Setting Altitude Bias");
    
    for (int i = 0; i < 1000; i++) {
        if (performReading()) {
            if (i >= 400) {
                AltitudeBias += readAltitude(0);
                    count++;
            }
        } 
        delay(5);
    }
    if (count > 0) {
        AltitudeBias /= count;
        return true;
    }

    return false;
}

float BMP390L::readAltitudeBias() {
    return AltitudeBias;
}

void BMP390L::getTempPressAlt(float &temp, float &press, float &altitude)
{
    temp = readTemperature();
    press = readPressure();
    altitude = readAltitude(1);
}