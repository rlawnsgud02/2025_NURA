#include "SDCard.h"

SDLogger::SDLogger(int cs) : cs_pin(cs) {}

SDLogger::~SDLogger() {
    if (fp) {
        fp.close();
    }
}

int SDLogger::initialize() {
    if (!SD.begin(cs_pin)) {
        Serial.println("SD 카드 초기화 실패!");
        return -1;
    }
    Serial.println("SD 카드 초기화 완료!");
    show_file_list();
    return 1;
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
    int file_num = 1;
    sprintf(file_name, "/output_%03d.csv", file_num);

    while (SD.exists(file_name)) {
        file_num++;
        sprintf(file_name, "/output_%03d.csv", file_num);
    }

    fp = SD.open(file_name, FILE_WRITE);
    if (!fp) {
        Serial.println("파일 생성 실패!");
        return -1;
    }

    fp.println("[Data]");
    fp.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,T,P,P_alt,fix,lon,lat,alt,vn,ve,vd,eject");
    fp.close();

    Serial.print("새로운 데이터 파일 생성: ");
    Serial.println(file_name);
    return 1;
}

int SDLogger::write_data(uint32_t timestamp, float *acc, float *gyro, float *mag, float *euler, float *baro) {
    fp = SD.open(file_name, FILE_WRITE);
    if (!fp) {
        Serial.println("파일 열기 실패!");
        return -1;
    }

    fp.print(timestamp);
    fp.print(",");
    for (int i = 0; i < 3; i++) {
        fp.print(acc[i]);
        fp.print(",");
    }
    for (int i = 0; i < 3; i++) {
        fp.print(gyro[i]);
        fp.print(",");
    }
    for (int i = 0; i < 3; i++) {
        fp.print(mag[i]);
        fp.print(",");
    }
    for (int i = 0; i < 3; i++) {
        fp.print(euler[i]);
        fp.print(",");
    }
    for (int i = 0; i < 3; i++) {
        fp.print(baro[i]);
        if (i < 2) fp.print(",");
    }
    fp.println();
    fp.close();

    return 1;
}

void SDLogger::save_data_file() {
    if (fp) {
        fp.close();
    }
}

int SDLogger::open_data_file() {
    fp = SD.open(file_name, FILE_WRITE);
    if (!fp) {
        Serial.println("파일 열기 실패!");
        return -1;
    }
    return 1;
}
