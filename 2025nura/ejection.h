#ifndef __EJECTION_H__
#define __EJECTION_H__

#include <math.h>
#include <Arduino.h>

#define MAX_BUF 50

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
    int8_t type;
    int8_t count;

    int8_t servopin;
    int8_t CH;
    bool safetypin; //int8_t
    bool launchpin; //int8_t
    bool is_ejected;

    double clac_BUF[MAX_BUF] = {0};
    double anglegro;
    double max_avg_alt;
    double avg_alt;
    int32_t timer;

    int8_t servo_frequency;
    int8_t pwm_bits;
    int16_t max_duty;


    bool eject_gyro(float anglegro);
    bool eject_alt(double alt);
    bool eject_time();
    int8_t eject_manual();

    void BUF_avg(double alt);

    void servo_write(int pulse);

public:
    ejection(int8_t servopin, int8_t ch, bool safetypin, bool launchpin, bool is_ejected = false); 

    int eject(float anglegro, double alt, int32_t time, int8_t msg = 0);
    void servo_init();
};

#endif