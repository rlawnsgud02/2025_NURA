#include "SDCard.h"

SDLogger::SDLogger(int cs)
    : CS_pin(cs), fp(nullptr), file_num(0) {}

SDLogger::~SDLogger() {
    if (fp) {
        fp->close();
        delete fp;
    }
}

int SDLogger::initialize() {
    if (!SD.begin(CS_pin)) {
        Serial.println("SD 카드 초기화 실패!");
        return -1;
    }

    Serial.println("SD 카드 초기화 완료!");
    show_file_list();
    
    return create_new_data_file();
}

void SDLogger::show_file_list() {
    File root = SD.open("/");
    if (!root) {
        Serial.println("SD 카드 읽기 실패!");
        return;
    }

    Serial.println("\n<File List>");
    File entry = root.openNextFile();
    while (entry) {
        Serial.println(entry.name());
        entry = root.openNextFile();
    }
    Serial.println();
}

int SDLogger::create_new_data_file() {
    file_num = 1;
    sprintf(file_name, "/output_%03d.csv", file_num);

    while (SD.exists(file_name)) {
        file_num++;
        sprintf(file_name, "/output_%03d.csv", file_num);
    }

    fp = new File(SD.open(file_name, FILE_WRITE));
    if (!fp || !(*fp)) {
        Serial.println("파일 생성 실패!");
        return -1;
    }

    fp->println("[Data]");
    fp->println("time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,T,P,P_alt,fix,lon,lat,alt,vn,ve,vd,eject");
    fp->close();
    delete fp;
    fp = nullptr;

    Serial.print("새로운 데이터 파일 생성: ");
    Serial.println(file_name);
    return 1;
}

int SDLogger::write_data(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float * baro, GpsData &gps, int eject) {
    fp = new File(SD.open(file_name, FILE_WRITE));
    if (!fp || !(*fp)) {
        Serial.println("파일 열기 실패!");
        return -1;
    }

    fp->print(timestamp);
    fp->print(",");

    for (int i = 0; i < 3; i++) { fp->print(acc[i]); fp->print(","); }
    for (int i = 0; i < 3; i++) { fp->print(gyro[i]); fp->print(","); }
    for (int i = 0; i < 3; i++) { fp->print(mag[i]); fp->print(","); }
    for (int i = 0; i < 3; i++) { fp->print(euler[i]); fp->print(","); }
    for (int i = 0; i < 3; i++) { fp->print(baro[i]); fp->print(","); }

    // GPS
    fp->print(gps.fixType); fp->print(",");
    fp->print(gps.lon, 7); fp->print(",");
    fp->print(gps.lat, 7); fp->print(",");
    fp->print(gps.height); fp->print(",");
    fp->print(gps.velN); fp->print(",");
    fp->print(gps.velE); fp->print(",");
    fp->print(gps.velD); fp->print(",");

    // Eject 상태
    fp->println(eject);

    fp->close();
    delete fp;
    fp = nullptr;

    return 1;
}

void SDLogger::save_data_file() {
    if (fp) {
        fp->flush();
        fp->close();
        delete fp;
        fp = nullptr;
    }
}

int SDLogger::open_data_file() {
    fp = new File(SD.open(file_name, FILE_WRITE));
    if (!fp || !(*fp)) {
        Serial.println("파일 열기 실패!");
        return -1;
    }
    return 1;
}
