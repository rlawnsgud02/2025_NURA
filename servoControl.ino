/*
 * ESP32 LEDC API 직접 제어를 통한 서보 제어 최종 해결책 (안정화 버전)
 * - PWM 해상도를 16비트에서 12비트로 변경하여 하드웨어 호환성 및 안정성 확보
 * - 펄스 폭 계산 로직을 더 명확하게 수정
 */

// 서보 핀 할당
const int CANARD1 = A2;
const int CANARD2 = A3;
const int CANARD3 = A4;
const int CANARD4 = A5;
const int CHUTE = A6;

// LEDC 채널 할당 (0~15까지 사용 가능)
const int servo1Channel = 0;
const int servo2Channel = 1;
const int servo3Channel = 2;
const int servo4Channel = 3;
const int servo5Channel = 4;

// 서보 제어 파라미터
const int SERVO_FREQUENCY = 50;      // 50Hz (또는 고성능을 위해 330Hz) -> 카나드는 330, 사출은 50
const int PWM_RESOLUTION_BITS = 12;  // 16비트 -> 12비트로 변경 (안정성 확보)
const int MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION_BITS) - 1; // 4095

// 서보 펄스 폭 범위 (마이크로초 단위)
const int MIN_PULSE_US = 1000;
const int MAX_PULSE_US = 2000;

/**
 * @brief 마이크로초(us) 단위의 펄스 폭을 LEDC duty 값으로 변환하여 서보를 제어하는 헬퍼 함수
 * @param channel 제어할 LEDC 채널 (0-15)
 * @param pulse_us 목표 펄스 폭 (마이크로초 단위, 예: 1500)
 */
void servo_write_us(int channel, int pulse_us) {
  // 1초를 마이크로초로 나눈 값 (1,000,000)을 주파수로 나누어 1주기의 길이를 계산
  double period_us = 1000000.0 / SERVO_FREQUENCY;
  
  // (목표 펄스 폭 / 전체 주기) * 최대 해상도 값 으로 duty cycle 계산
  uint32_t duty = (uint32_t)(((double)pulse_us / period_us) * (double)MAX_DUTY_CYCLE);
  
  ledcWrite(channel, duty);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("Initializing servos with direct LEDC control (v2)...");

  // 각 서보 채널에 대한 LEDC 설정 (12비트 해상도)
  ledcSetup(servo1Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo2Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo3Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo4Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo5Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);

  // 설정된 LEDC 채널을 실제 GPIO 핀에 연결
  ledcAttachPin(CANARD1, servo1Channel);
  ledcAttachPin(CANARD2, servo2Channel);
  ledcAttachPin(CANARD3, servo3Channel);
  ledcAttachPin(CANARD4, servo4Channel);
  ledcAttachPin(CHUTE, servo5Channel);

  Serial.println("LEDC setup complete. Starting main loop.");
}

void loop() {
  // 모든 서보를 동시에 최소 -> 최대로 스위핑
  // Serial.println("Sweeping from MIN to MAX...");
  // for (int pulse = MIN_PULSE_US; pulse <= MAX_PULSE_US; pulse += 10) {
  //   servo_write_us(servo1Channel, pulse);
  //   servo_write_us(servo2Channel, pulse);
  //   servo_write_us(servo3Channel, pulse);
  //   servo_write_us(servo4Channel, pulse);
  //   servo_write_us(servo5Channel, pulse);
  //   delay(15);
  // }

  // // 모든 서보를 동시에 최대 -> 최소로 스위핑
  // Serial.println("Sweeping from MAX to MIN...");
  // for (int pulse = MAX_PULSE_US; pulse >= MIN_PULSE_US; pulse -= 10) {
  //   servo_write_us(servo1Channel, pulse);
  //   servo_write_us(servo2Channel, pulse);
  //   servo_write_us(servo3Channel, pulse);
  //   servo_write_us(servo4Channel, pulse);
  //   servo_write_us(servo5Channel, pulse);
  //   delay(15);
  // }
  servo_write_us(servo5Channel, 2200);


  delay(1000);

  for(int pulse=1500;pulse<=1700;pulse+=10){
    servo_write_us(servo1Channel, pulse);
    servo_write_us(servo2Channel, pulse);
    servo_write_us(servo3Channel, pulse);
    servo_write_us(servo4Channel, pulse);
    delay(10);
  }

  for(int pulse=1700;pulse>=1500;pulse-=10){
    servo_write_us(servo1Channel, pulse);
    servo_write_us(servo2Channel, pulse);
    servo_write_us(servo3Channel, pulse);
    servo_write_us(servo4Channel, pulse);
    delay(10);
  }

  delay(1000);

  for(int pulse=1500;pulse>=1300;pulse-=10){
    servo_write_us(servo1Channel, pulse);
    servo_write_us(servo2Channel, pulse);
    servo_write_us(servo3Channel, pulse);
    servo_write_us(servo4Channel, pulse);
    delay(10);
  }

  for(int pulse=1300;pulse<=1500;pulse+=10){
    servo_write_us(servo1Channel, pulse);
    servo_write_us(servo2Channel, pulse);
    servo_write_us(servo3Channel, pulse);
    servo_write_us(servo4Channel, pulse);
    delay(10);
  }

  servo_write_us(servo5Channel, 800);
  delay(1000);
}
