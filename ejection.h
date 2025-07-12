#ifndef __EJECTION_H__
#define __EJECTION_H__

#define MAX_BUF 50

#include <math.h>
#include <ESP32Servo.h>

#define MIN 800
#define MAX 2200

enum EjectionType {
    NO_EJECTION = 0,
    GYRO_EJECTION = 1,
    ALT_EJECTION = 2,
    TIME_EJECTION = 3,
    MANUAL_EJECTION = 4
};

class ejection {
private:


public:
    ejection(int servopin, bool safetypin, bool launchpin, bool is_ejected = false);

    int8_t type;
    int8_t count;

    int servopin;
    bool is_ejected;
    bool safetypin;
    bool launchpin;

    double clac_BUF[MAX_BUF] = {0};
    double anglegro;
    double max_avg_alt;
    double avg_alt;
    int32_t timer;

    Servo *Eject_servo;


    int eject(float anglegro, double alt, int32_t time, int8_t msg = 0);
    void SERVO();
    // void message();

    bool eject_gyro(float anglegro);
    bool eject_alt(double alt);
    bool eject_time();
    int8_t eject_manual();

    void BUF_avg(double alt);

};

#endif