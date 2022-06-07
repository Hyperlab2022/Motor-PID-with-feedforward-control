#include "pidmotor.h"
/**************motor pins****************************/
#define ENCA 1
#define ENCB 1
#define PWM 1
#define IN2 1
#define IN1 1
#define H 1 //motor homing sensor
Encoder ENC(ENCA, ENCB);
pidmotor Motor(PWM, IN1, IN2, H, &ENC);
float f, g;
/**************Task Schedular Code****************************/
//yesss!!! the pid compute function will run in background in another core simultaneously by its status from (Ready,Running,Blocked,Suspended,Deleted)
TaskHandle_t pid_compute;
inline void Task1code(void * parameter) __attribute__((always_inline));
void Task1code( void * parameter ) {
  for (;;) {
    Motor.compute(f);
    vTaskDelay(1);
  }
}
void setup() {
  xTaskCreatePinnedToCore(Task1code, "pid_compute", 10000, NULL, 2, &pid_compute, 0);
  vTaskSuspend(pid_compute);
  delay(50);
  Serial.println("setup");
  Motor.calibrate();
  Motor.test_cal();
  vTaskResume(pid_compute);
}
//use serial motor to test the motor
//example command {m 90}
void loop() {
  vTaskResume(pid_compute);
  //  vTaskSuspend(pid_compute);
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'm': {
          int x = Serial.parseInt();
          f = x / 1.63;
          break;
        }
      case 'X': {
          while (digitalRead(H)) Motor.move(0, 100);
          Serial.printf("motor homed");
          break;
        }
      case 'T': {
          vTaskSuspend(pid_compute);
          Motor.test_sys();
          vTaskResume(pid_compute);
          break;
        }
      case 'c': {
          vTaskSuspend(pid_compute);
          Motor.calibrate();
          Motor.test_cal();
          vTaskResume(pid_compute);
          break;
        }
      case 'j': {
          int xkp = Serial.parseInt();
          int xki = Serial.parseInt();
          int xkd = Serial.parseInt();
          Motor.set_coeffs(xkp, xki, xkd);
          break;
        }
      default: {
          Serial.printf("Input any command");
          delay(150);
        }
    }
  }
  yield();
}
