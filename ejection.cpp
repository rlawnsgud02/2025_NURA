#include "ejection.h"

// ejection 객체 사용 방법
/*
객체 생성 시 is_ejected safetypin은 기본 파라미터로 초기화됨
사출 type, anglegyro, max_avg_alt, avg_alt, time 모두 0으로 초기화

메인 파일에서 객체 생성 후, 객체이름.eject(euler, alt, msg); 함수만 호출하면 작동함 -> msg는 강제 사출을 위해 추가함

현재 25.07.05 16:20에
코드의 기본적인 틀만 작성했으며 세부적인 코드를 추가하여 구현해야함

클래스로 정의한 이유
24nura의 사출 코드가 너무 방대하여 가독성이 떨어지며 추후 코드 
수정이나 설명, 이해의 측면에서 더욱 효과적이기에 클래스로 구현함

모든 데이터를 pointer로 구현할 수도 있지만(데이터 안정화 보장 가능) 굳이..?
*/


ejection::ejection(bool is_ejected, bool safetypin): is_ejected(is_ejected), safetypin(safetypin) {
    type = 0; anglegro = 0; max_avg_alt = 0; avg_alt = 0; time = 0;
}

bool ejection::eject_gyro(double *euler) {
    anglegro = sqrt(pow(euler[0], 2) + pow(euler[1], 2));
    if (anglegro > 70) {
        //사출 코드 추가
        type = 1;
        return true;
    }
    return false;
}

bool ejection::eject_alt(double alt) {
    avg_alt += alt; //alt_avg를 구하는 방법이 생각 안남 ㅅㅂㅠ
    max_avg_alt = (avg_alt > max_avg_alt) ? avg_alt : max_avg_alt;

    if (max_avg_alt - avg_alt > 3) {
        //사출 코드 추가
        type = 2;
        return true;
    }
    return false;
}

bool ejection::eject_time() {
    if (time >= 9) {
        //사출 코드 추가
        type = 3;
        return true;
    }
    return false;
}

void ejection::eject(double *euler, double alt, int mes = 0) { // mes -> GCS에서 통신으로 사출하기 위해
    is_ejected = (mes == 1) ? (type = 4, true) : ((type = 0), eject_gyro(euler) || eject_alt(alt) || eject_time());
    message();
}

void ejection::message() {
    if (is_ejected) {
        switch(type) {
            case 1:
                printf("자세"); //더 작성해야함
                break;
            
            case 2:
                printf("고도"); //더 작성해야함
                break;

            case 3:
                printf("시간"); //더 작성해야함
                break;

            case 4:
                printf("강제");
                break;
        }
        printf("사출 성공"); //더 작성해야함
        //더 구현해야함
    }
}