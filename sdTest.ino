#include "SDCard.h"

#define CS_PIN 10
SDLogger sd(CS_PIN);


void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    while (!Serial); // Serial 초기화 대기
    sd.initialize(); 

}

void loop() {
  // put your main code here, to run repeatedly:

}
