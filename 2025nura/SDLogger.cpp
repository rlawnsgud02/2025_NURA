    #include "SDLogger.h"

    SDFatLogger::SDFatLogger(int cs) : CS_pin(cs), file_num(1), init(false) {
        memset(file_name, 0, sizeof(file_name));
    }

    namespace DATA {
        int32_t timestamp;
        float* acc;
        float* gyro;
        float* mag;
        float* euler;
        float maxG;
        float* baro;
        GpsData gps;
        int eject;
    }

    int SDFatLogger::initialize() {
        pinMode(CS_pin, OUTPUT);
        // select(); // 이거 추가하면 sd.begin과 꼬임

        if (!sd.begin(CS_pin, SD_SCK_MHZ(4))) {
            Serial.println("SdFat 카드 초기화 실패!");
            deselect();
            return -1;
        }

        Serial.println("SdFat 카드 초기화 시작!");
        show_file_list();
        return create_new_data_file();
    }

    void SDFatLogger::show_file_list() {
        FsFile root = sd.open("/");
        if (!root) {
            Serial.println("루트 디렉토리 열기 실패!");
            deselect();
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
        // dataFile.flush();
        init = true;

        deselect();
        Serial.print("-----| New File Created : "); Serial.println(file_name);
        return 1;
    }

    bool SDFatLogger::write_data() {
        // select();

        if (dataFile.getWriteError()) {
            Serial.println("❌ writeError flag set!");
            dataFile.clearWriteError(); 
        }

        openFile();
        char buf[256];
        int len = snprintf(buf, sizeof(buf),
            "%ld,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
            "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.7f,%.7f,%.2f,%.2f,%.2f,%.2f,%d\n",

            DATA::timestamp,
            DATA::acc[0], DATA::acc[1], DATA::acc[2],
            DATA::gyro[0], DATA::gyro[1], DATA::gyro[2],
            DATA::mag[0],  DATA::mag[1],  DATA::mag[2],
            DATA::euler[0], DATA::euler[1], DATA::euler[2],
            DATA::maxG,
            DATA::baro[0], DATA::baro[1], DATA::baro[2],
            DATA::gps.fixType, DATA::gps.lon, DATA::gps.lat, DATA::gps.height,
            DATA::gps.velN, DATA::gps.velE, DATA::gps.velD,
            DATA::eject);

        if (len < 0 || len >= (int)sizeof(buf)) {
            Serial.println("❌ snprintf overflow");
            closeFile();
            deselect();
            return -2;
        }

        size_t n = dataFile.write(buf, len);
        if (n != (size_t)len) {
            Serial.printf("❌ write failed: wrote %u/%d bytes\n", n, len);
            Serial.printf("errCode = 0x%02X / 0x%02X\n", sd.sdErrorCode(), sd.sdErrorData());
            closeFile();
            deselect();
            return -3;
        }

        // closeFile();
        dataFile.flush();

        // deselect();

        return 1;
    }

    // void SDFatLogger::setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, GpsData &gps, int eject) {
    //     select();
    //     DATA::timestamp = timestamp;
    //     DATA::acc = acc;
    //     DATA::gyro = gyro;
    //     DATA::mag = mag;
    //     DATA::euler = euler;
    //     DATA::maxG = maxG;
    //     DATA::baro = baro;
    //     DATA::gps = gps;
    //     DATA::eject = eject;
    //     deselect();
    // }

    void SDFatLogger::setData(int32_t timestamp, float * acc, float * gyro, float * mag, float * euler, float maxG, float * baro, int eject) {
        // select();
        DATA::timestamp = timestamp;
        DATA::acc = acc;
        DATA::gyro = gyro;
        DATA::mag = mag;
        DATA::euler = euler;
        DATA::maxG = maxG;
        DATA::baro = baro;
        DATA::eject = eject;
        // deselect();
    }

    bool SDFatLogger::openFile() {
        dataFile = sd.open(file_name, O_WRITE | O_APPEND);
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

    bool SDFatLogger::isInit() {
        return init;
    }

    const char* SDFatLogger::getFileName() {
        return file_name;
    }

    void SDFatLogger::print() {
        // select();
        Serial.print(DATA::timestamp); Serial.print(", ");
        for (int i = 0; i < 3; i++){Serial.print(DATA::acc[i]);Serial.print(", ");}
        for (int i = 0; i < 3; i++){Serial.print(DATA::gyro[i]);Serial.print(", ");}
        for (int i = 0; i < 3; i++){Serial.print(DATA::mag[i]);Serial.print(", ");}
        for (int i = 0; i < 3; i++){Serial.print(DATA::euler[i]);Serial.print(", ");}
        Serial.print(DATA::maxG);Serial.print(", ");
        for (int i = 0; i < 3; i++){Serial.print(DATA::baro[i]);Serial.print(", ");}
        Serial.print(DATA::gps.fixType); Serial.print(", ");
        Serial.print(DATA::gps.lon); Serial.print(", ");
        Serial.print(DATA::gps.lat); Serial.print(", ");
        Serial.print(DATA::gps.height); Serial.print(", ");
        Serial.print(DATA::gps.velN); Serial.print(", ");
        Serial.print(DATA::gps.velE); Serial.print(", ");
        Serial.print(DATA::gps.velD); Serial.print(", ");

        Serial.print(DATA::eject);Serial.println("");
        // deselect();
    }

    void SDFatLogger::select() {
        digitalWrite(CS_pin, LOW);

        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    }

    void SDFatLogger::deselect() {
        SPI.endTransaction();

        digitalWrite(CS_pin, HIGH);
    }