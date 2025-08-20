/*
  ESP32Servo.h는 버전 충돌로 인해서 정상 작동하지 않습니다...
  대신에 ESP32의 PWM 기능을 사용하여 서보를 제어하는 방식을 사용합니다.
  조립상의 이유 등으로 카나드의 정렬이 맞지 않는다면, PWM을 직접 조정하며 맞춰주세요.
  그 외에도 서보의 동작을 테스트하기 위한 다양한 기능들이 있습니다.
*/

const int CANARD1 = A2;
const int CANARD2 = A3;
const int CANARD3 = A4;
const int CANARD4 = A5;
const int CHUTE = A6;

const int servo1Channel = 0;
const int servo2Channel = 1;
const int servo3Channel = 2;
const int servo4Channel = 3;
const int servo5Channel = 4;

const int SERVO_FREQUENCY = 50;
const int PWM_RESOLUTION_BITS = 12;
const int MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION_BITS) - 1;

const int MIN_PULSE_US = 500;
const int MAX_PULSE_US = 2500;


void servo_write_us(int channel, int pulse_us) {
  double period_us = 1000000.0 / SERVO_FREQUENCY;
  uint32_t duty = (uint32_t)(((double)pulse_us / period_us) * (double)MAX_DUTY_CYCLE);
  ledcWrite(channel, duty);
}

void setup() {
  Serial.begin(115200);

  ledcSetup(servo1Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo2Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo3Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo4Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcSetup(servo5Channel, SERVO_FREQUENCY, PWM_RESOLUTION_BITS);

  ledcAttachPin(CANARD1, servo1Channel);
  ledcAttachPin(CANARD2, servo2Channel);
  ledcAttachPin(CANARD3, servo3Channel);
  ledcAttachPin(CANARD4, servo4Channel);
  ledcAttachPin(CHUTE, servo5Channel);
}

void loop() {
  // 모든 서보를 동시에 최소 -> 최대로 스위핑
  // for (int pulse = MIN_PULSE_US; pulse <= MAX_PULSE_US; pulse += 10) {
  //   servo_write_us(servo1Channel, pulse);
  //   servo_write_us(servo2Channel, pulse);
  //   servo_write_us(servo3Channel, pulse);
  //   servo_write_us(servo4Channel, pulse);
  //   servo_write_us(servo5Channel, pulse);
  //   delay(10);
  // }

  // 모든 서보를 동시에 최대 -> 최소로 스위핑
  // Serial.println("Sweeping from MAX to MIN...");
  // for (int pulse = MAX_PULSE_US; pulse >= MIN_PULSE_US; pulse -= 10) {
  //   servo_write_us(servo1Channel, pulse);
  //   servo_write_us(servo2Channel, pulse);
  //   servo_write_us(servo3Channel, pulse);
  //   servo_write_us(servo4Channel, pulse);
  //   servo_write_us(servo5Channel, pulse);
  //   delay(10);
  // }
  // delay(1000);

  // 서보 초기화 및 정렬 테스트

  // servo_write_us(servo1Channel, 1500);
  // servo_write_us(servo2Channel, 1500);
  // servo_write_us(servo3Channel, 1560);
  // servo_write_us(servo4Channel, 1530);
  // delay(5000);

  // servo_write_us(servo1Channel, 1611);
  // servo_write_us(servo2Channel, 1611);
  // servo_write_us(servo3Channel, 1611);
  // servo_write_us(servo4Channel, 1611);
  // delay(5000);

  // servo_write_us(servo1Channel, 1480);
  // servo_write_us(servo2Channel, 1440);
  // servo_write_us(servo3Channel, 1520);
  // servo_write_us(servo4Channel, 1540);
  // delay(5000);

  // servo_write_us(servo1Channel, 500);
  // servo_write_us(servo2Channel, 1389);
  // servo_write_us(servo3Channel, 1389);
  // servo_write_us(servo4Channel, 1389);
  // delay(1500);

  // servo_write_us(servo1Channel, 2500);
  // servo_write_us(servo2Channel, 2000);
  // servo_write_us(servo3Channel, 2000);
  // servo_write_us(servo4Channel, 2000);
  // delay(1500);

  // servo_write_us(servo5Channel, 800);
  // delay(1000);
}
