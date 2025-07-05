#ifndef __EJECTION_H__
#define __EJECTION_H__

#define MAX_BUF 50

class ejection {
private:


public:
    ejection(bool is_ejected = false, bool safetypin = true);

    int type;

    bool is_ejected;
    bool safetypin;

    double anglegro;
    double max_avg_alt;
    double avg_alt;
    double time;

    void eject(double *euler, double alt);
    void message();

    bool eject_gyro(double *euler);
    bool eject_alt(double alt);
    bool eject_time();

};

#endif