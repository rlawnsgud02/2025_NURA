#include "ejection.h"

// ejection 객체 사용 방법
/*
객체 생성 시 is_ejected safetypin은 기본 파라미터로 초기화됨
사출 type, anglegyro, max_avg_alt, avg_alt, time 모두 0으로 초기화

메인 파일에서 객체 생성 후, 객체이름.eject(euler, alt, mes); 함수만 호출하면 작동함 -> mes는 강제 사출을 위해 추가함

현재 25.07.05 16:20에
코드의 기본적인 틀만 작성했으며 세부적인 코드를 추가하여 구현해야함

클래스로 정의한 이유
24nura의 사출 코드가 너무 방대하여 가독성이 떨어지며 추후 코드 
수정이나 설명, 이해의 측면에서 더욱 효과적이기에 클래스로 구현함

모든 데이터를 pointer로 구현할 수도 있지만(데이터 안정화 보장 가능) 굳이..?


기록 25.07.10
수정 사항
eject_alt의 avg_alt 구현
lunchpin 선언
eject에서 safetypin, lunchpin 조건문 추가

고민 사항
사출 여부와 종류 메시지를 지상국에 보내는데 NMT헤더를 추가하느냐(통신 문제가 생길까봐 안하는게 좋을 것 같음)
아니면 eject이 char pointer를 반환하도록 수정하여 rf.print(문자열);를 구현하느냐
@@@@ 가능한 방안 @@@@
1. 쓰레드끼리 같은 변수를 사용하여 eject의 반환값을 저장하고 rf.print에 해당 변수를 보낸다 -> 전역 변수 사용
2. 쓰레드를 3개로 구현하여 사출과 sd, 통신을 같이 처리한다 -> 쓰레드 3, 4의 메모리를 전부 할당해야 함.

사용 방법
객체 생성 후, eject()가 int를 반환함 -> main파일에서 type에 따라 메시지 작성
eject(servopin, is_ejected, safetypin, launchpin);

type 0 -> 사출 X | type 1 -> 자세 | type 2 -> 고도 | type 3 -> 시간 | type 4 -> 강제 사출

*/


/*
// 실제
ejection::ejection(int servopin, int8_t safetypin, int8_t launchpin, bool is_ejected): safetypin(safetypin), launchpin(launchpin), is_ejected(is_ejected) {
    type = NO_EJECTION; count = 0; anglegro = 0; max_avg_alt = 0; avg_alt = 0; timer = 0; 
    Eject_servo.attach(servopin, MIN, MAX);
    Eject_servo.writeMicroseconds(MIN);
}
*/

// 임시 테스트용
ejection::ejection(int servopin, bool safetypin, bool launchpin, bool is_ejected): servopin(servopin), safetypin(safetypin), launchpin(launchpin), is_ejected(is_ejected) {
    type = NO_EJECTION; count = 0; anglegro = 0; max_avg_alt = 0; avg_alt = 0; timer = 0; 
}

void ejection::SERVO() {
    Servo S;
    Eject_servo = &S;
    Eject_servo->attach(servopin, MIN, MAX); //SDLogger참고해서 수정하기!
    Eject_servo->writeMicroseconds(MIN);
}


bool ejection::eject_gyro(float anglegro) {
    if (anglegro > 70) {
        //사출 코드 추가
        Eject_servo->writeMicroseconds(MAX);
           
        type = GYRO_EJECTION;
        return true;
    }
    return false;
}

bool ejection::eject_alt(double alt) {
    BUF_avg(alt);

    max_avg_alt = (avg_alt >= max_avg_alt) ? avg_alt : max_avg_alt;

    if (max_avg_alt - avg_alt > 3) {
        //사출 코드 추가
        Eject_servo->writeMicroseconds(MAX);

        type = ALT_EJECTION;
        return true;
    }
    return false;
}

bool ejection::eject_time() {
    if (timer >= 9000) {
        //사출 코드 추가
        Eject_servo->writeMicroseconds(MAX);

        type = TIME_EJECTION;
        return true;
    }
    return false;
}

int8_t ejection::eject_manual() {
    Eject_servo->writeMicroseconds(MAX);

    return MANUAL_EJECTION;
}

int ejection::eject(float anglegro, double alt, int32_t time, int8_t msg) { // mes -> GCS에서 통신으로 사출하기 위해
    timer = time;
    if (safetypin == false) {
        if (launchpin == false) {
            if (timer % 1000 < 200) {
                is_ejected = (msg == 1) ? (type = eject_manual(), true) : ((type = 0), eject_gyro(anglegro) || eject_alt(alt) || eject_time());
            }
            else {
                is_ejected = (msg == 1) ? (type = eject_manual(), true) : ((type = 0), eject_alt(alt) || eject_time());
            }
        }
    }   
    if (is_ejected) {
        return type; 
    }
    return NO_EJECTION;
}

// void ejection::message() {
//     if (is_ejected) {
//         switch(type) {
//             case 1:
//                 //printf("자세"); //더 작성해야함
//                 message = 
//                 break;
            
//             case 2:
//                 printf("고도"); //더 작성해야함
//                 break;

//             case 3:
//                 printf("시간"); //더 작성해야함
//                 break;

//             case 4:
//                 printf("강제");
//                 break;
//         }
//         printf("사출 성공"); //더 작성해야함
//         //더 구현해야함
//     }
// }

void ejection::BUF_avg(double alt) {
    avg_alt = 0;
    clac_BUF[count] = alt;
    count++;
    for (int i = 0; i < MAX_BUF; i++) {
        avg_alt += clac_BUF[i];
    }
    avg_alt /= MAX_BUF;
    count = (count == MAX_BUF) ? 0 : count;
}