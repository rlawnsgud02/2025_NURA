#include "SDLogger.h"

// namespace DATA {
//     int32_t timestamp;
//     float* acc;
//     float* gyro;
//     float* mag;
//     float* euler;
//     float maxG;
//     float* baro;
//     GpsData gps;
//     int eject;
// }

namespace DATA {
    int32_t timestamp;
    float* acc;
    float* gyro;
    float* mag;
    float* euler;
    float maxG;
    float* baro;
    int eject;
}

SDFatLogger::SDFatLogger(int cs) : CS_pin(cs), file_num(1), init(false) {
    memset(file_name, 0, sizeof(file_name));
}

int SDFatLogger::initialize() {
    if (!sd.begin(CS_pin, SD_SCK_MHZ(4))) {    
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

    // dataFile = sd.open(file_name, FILE_WRITE);
    dataFile = sd.open(file_name, O_WRITE | O_APPEND | O_CREAT); // 정상코드 방식임
    if (!dataFile) {
        Serial.println("파일 생성 실패!");  
        return -1;
    }

    dataFile.println("[Data]");
    dataFile.println("TimeStamp,ax,ay,az,gx,gy,gz,mx,my,mz,Roll,Pitch,Yaw,maxG,T,P,P_alt,fix,Lon,Lat,Alt,VN,VE,VD,Chute"); // GPS 포함 버전
    // dataFile.println("TimeStamp,ax,ay,az,gx,gy,gz,mx,my,mz,Roll,Pitch,Yaw,maxG,T,P,P_alt,Chute");
    // dataFile.flush();
    dataFile.close();
    
    init = true;
    Serial.print("-----| New File Created : "); Serial.println(file_name);
    return 1;
}

int SDFatLogger::write_data() {
    // dataFile = sd.open(file_name, O_WRITE | O_APPEND);
    // openFile();
    if (dataFile.getWriteError()) {
        Serial.println("❌ writeError flag set!");
        dataFile.clearWriteError();  // 꼭 리셋해야 다음 print가 먹음
    }

    char buf[256];
    // GPS 없는 버전
    int len = snprintf(buf, sizeof(buf),
        "%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",
        DATA::timestamp,
        DATA::acc[0], DATA::acc[1], DATA::acc[2],
        DATA::gyro[0], DATA::gyro[1], DATA::gyro[2],
        DATA::mag[0],  DATA::mag[1],  DATA::mag[2],
        DATA::euler[0], DATA::euler[1], DATA::euler[2],
        DATA::maxG,
        DATA::baro[0], DATA::baro[1], DATA::baro[2],
        DATA::eject);

    // GPS 포함 버전
    // int len = snprintf(buf, sizeof(buf),
    //     "%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
    //     "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%d\n",
    //     DATA::timestamp,
    //     DATA::acc[0], DATA::acc[1], DATA::acc[2],
    //     DATA::gyro[0], DATA::gyro[1], DATA::gyro[2],
    //     DATA::mag[0],  DATA::mag[1],  DATA::mag[2],
    //     DATA::euler[0], DATA::euler[1], DATA::euler[2],
    //     DATA::maxG,
    //     DATA::baro[0], DATA::baro[1], DATA::baro[2],
    //     DATA::gps.fixType, DATA::gps.lon, DATA::gps.lat, DATA::gps.height,
    //     DATA::gps.velN, DATA::gps.velE, DATA::gps.velD,
    //     DATA::eject);

    if (len < 0 || len >= (int)sizeof(buf)) {
        Serial.println("❌ snprintf overflow");
        closeFile();
        return -2;
    }

    size_t n = dataFile.write(buf, len);
    if (n != (size_t)len) {
        Serial.printf("❌ write failed: wrote %u/%d bytes\n", n, len);
        Serial.printf("errCode = 0x%02X / 0x%02X\n", sd.sdErrorCode(), sd.sdErrorData());
        closeFile();
        return -3;
    }

    // closeFile();
    flushFile(); 
    
    return 1;
}

// GPS 데이터 제외 버전전
void SDFatLogger::setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, int eject) {
    DATA::timestamp = timestamp;
    DATA::acc = acc;
    DATA::gyro = gyro;
    DATA::mag = mag;
    DATA::euler = euler;
    DATA::maxG = maxG;
    DATA::baro = baro;
    DATA::eject = eject;
}

// GPS 데이터 저장 버전
// void SDFatLogger::setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, GpsData &gps, int eject) {
//     DATA::timestamp = timestamp;
//     DATA::acc = acc;
//     DATA::gyro = gyro;
//     DATA::mag = mag;
//     DATA::euler = euler;
//     DATA::maxG = maxG;
//     DATA::baro = baro;
//     DATA::gps = gps;
//     DATA::eject = eject;
// }

bool SDFatLogger::isInit() {
    return init;
}

const char* SDFatLogger::getFileName() {
    return file_name;
}

bool SDFatLogger::openFile() {
    // dataFile = sd.open(file_name, O_WRITE | O_APPEND);
    dataFile = sd.open(file_name, FILE_WRITE);

    if (!dataFile) {
        Serial.println("파일 열기 실패!");
        return false;
    }
    return true;
}

bool SDFatLogger::closeFile() {
    if (dataFile) {
        dataFile.close();
        return true;
    }
    return false;
}

bool SDFatLogger::flushFile() {
    if (dataFile) {
        dataFile.flush();
        return true;
    }
    return false;
}

void SDFatLogger::print() {
    Serial.print(DATA::timestamp); Serial.print(", ");
    for (int i = 0; i < 3; i++){Serial.print(DATA::acc[i]);Serial.print(", ");}
    for (int i = 0; i < 3; i++){Serial.print(DATA::gyro[i]);Serial.print(", ");}
    for (int i = 0; i < 3; i++){Serial.print(DATA::mag[i]);Serial.print(", ");}
    for (int i = 0; i < 3; i++){Serial.print(DATA::euler[i]);Serial.print(", ");}
    Serial.print(DATA::maxG);Serial.print(", ");
    for (int i = 0; i < 3; i++){Serial.print(DATA::baro[i]);Serial.print(", ");}

    Serial.print(DATA::eject);Serial.println("");
}

bool SDFatLogger::write_one_line() {
    if (!dataFile) return false;

    for (int i = 0; i < 10; ++i) {
        dataFile.print("1");
        if (i < 9) dataFile.print(',');
    }
    dataFile.println();
    dataFile.flush();  // 매번 flush()해서 파일 손상 방지
    return true;
}