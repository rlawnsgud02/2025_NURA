#include "SDLogger.h"

SDFatLogger::SDFatLogger(int cs) : CS_pin(cs), file_num(1), init(false) {
    memset(file_name, 0, sizeof(file_name));
}

int SDFatLogger::initialize() {
    if (!sd.begin(CS_pin, SD_SCK_MHZ(25))) {
        Serial.println("SdFat 카드 초기화 실패!");
        return -1;
    }

    Serial.println("SdFat 카드 초기화 완료!");
    show_file_list();
    return create_new_data_file();
}

void SDFatLogger::show_file_list() {
    FsFile root = sd.open("/");
    if (!root) {
        Serial.println("루트 디렉토리 열기 실패!");
        return;
    }

    Serial.println("<File List>");
    FsFile entry;
    char nameBuf[32];

    while ((entry = root.openNextFile())) {
        entry.getName(nameBuf, sizeof(nameBuf));
        Serial.println(nameBuf);
        entry.close();
    }

    root.close();
}

int SDFatLogger::create_new_data_file() {
    sprintf(file_name, "/output_%03d.csv", file_num);
    while (sd.exists(file_name)) {
        file_num++;
        sprintf(file_name, "/output_%03d.csv", file_num);
    }

    dataFile = sd.open(file_name, FILE_WRITE);
    if (!dataFile) {
        Serial.println("파일 생성 실패!");
        return -1;
    }

    dataFile.println("[Data]");
    dataFile.println("time,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw,maxG,T,P,P_alt,fix,lon,lat,alt,vn,ve,vd,eject");
    dataFile.close();

    init = true;
    Serial.print("-----| New File Created : "); Serial.println(file_name);
    return 1;
}

int SDFatLogger::write_data(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, GpsData &gps, int eject) {
    dataFile = sd.open(file_name, FILE_WRITE);
    if (!dataFile) {
        Serial.println("파일 열기 실패!");
        return -1;
    }

    dataFile.print(timestamp); dataFile.print(",");

    for (int i = 0; i < 3; i++) { dataFile.print(acc[i]); dataFile.print(","); }
    for (int i = 0; i < 3; i++) { dataFile.print(gyro[i]); dataFile.print(","); }
    for (int i = 0; i < 3; i++) { dataFile.print(mag[i]); dataFile.print(","); }
    for (int i = 0; i < 3; i++) { dataFile.print(euler[i]); dataFile.print(","); }
    dataFile.print(maxG); dataFile.print(",");

    for (int i = 0; i < 3; i++) { dataFile.print(baro[i]); dataFile.print(","); }

    dataFile.print(gps.fixType); dataFile.print(",");
    dataFile.print(gps.lon, 7); dataFile.print(",");
    dataFile.print(gps.lat, 7); dataFile.print(",");
    dataFile.print(gps.height); dataFile.print(",");
    dataFile.print(gps.velN); dataFile.print(",");
    dataFile.print(gps.velE); dataFile.print(",");
    dataFile.print(gps.velD); dataFile.print(",");

    dataFile.println(eject);
    dataFile.close();

    return 1;
}

bool SDFatLogger::isInit() {
    return init;
}

const char* SDFatLogger::getFileName() {
    return file_name;
}
