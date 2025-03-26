#ifndef BMP390L_H
#define BMP390L_H

#include "bmp3.h"
#include <Adafruit_I2CDevice.h>

#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address

class BMP390L {
private:

    Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

    float Temperature;
    float Pressure;
    float Altitude;
    float AltitudeBias;
    

    bool _filterEnabled, _tempOSEnabled, _presOSEnabled, _ODREnabled;

    unsigned long _meas_end;

    struct bmp3_dev the_sensor;

    bool _init();

    

public:
 
    BMP390L();
    
    bool begin_I2C(uint8_t addr = BMP3XX_DEFAULT_ADDRESS, TwoWire *theWire = &Wire);

    bool setTemperatureOversampling(uint8_t os);
    bool setPressureOversampling(uint8_t os);
    bool setIIRFilterCoeff(uint8_t fs);
    bool setOutputDataRate(uint8_t odr);

    bool performReading();

    float readTemperature();
    float readPressure();
    float readAltitude(int8_t portion = 1);
    float readAltitudeBias(); //실제로는 필요 없음
    void set_Altitude(float seaLevel);

    bool set_AltitudeBias();

};



#endif // BMP390L_H