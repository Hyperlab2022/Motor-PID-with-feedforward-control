/*
   pidmotor.cpp
   Created on: Jun 01, 2022
   Author: yashmewada9618
   -><-
*/
#include "pidmotor.h"

pidmotor::pidmotor(byte _PWM, byte _IN1, byte _IN2, byte _limit, Encoder *enc) {
  Serial.begin(115200);
  //EEPROM.begin(EEPROM_SIZE);
  //  Encoder _myenc(_ENCA, _ENCB);
  //  myenc = _myenc;
  this->enc = enc;
  this->PWM = _PWM;
  this->IN1 = _IN1;
  this->IN2 = _IN2;
  limit = _limit;
  pinMode(_PWM, OUTPUT);
  pinMode(_IN1, OUTPUT);
  pinMode(_IN2, OUTPUT);
  pinMode(limit, INPUT_PULLUP);
  speed = 255;
}
void pidmotor::setinputlimits(double mininput, double maxinput) {
  mmin = mininput;
  mmax = maxinput;
}
void pidmotor::move(int dir, byte pwmval) {
  analogWrite(PWM, pwmval);
  if (dir == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (dir == -1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
  }
}
void pidmotor::set_coeffs(float _kp, float _ki, float _kd) {
  //  kp = _kp * 2;
  //  ki = _ki * 1.4;
  //  kd = _kd * 1.4;
  kp = _kp;
  ki = _ki;
  kd = _kd;
  //  EEPROM.writeFloat(address, kp);
  //  address += sizeof(kp);
  //  EEPROM.writeFloat(address, ki);
  //  address += sizeof(ki);
  //  EEPROM.writeFloat(address, kd);
  //  address += sizeof(kd);
  //  EEPROM.commit();
  //    Serial.printf("%.2f\t%.2f\t%.2f\n", kp, ki, kd);
  //  return;
  //  return kp, ki, kd;
}
void pidmotor::get_coeffs() {
  float kpp, kii, kdd;
  int ad = 0;
  //  EEPROM.get(ad, kpp);
  //  ad += sizeof(kpp);
  //  EEPROM.get(ad, kii);
  //  ad += sizeof(kii);
  //  EEPROM.get(ad, kdd);
  //  kp = kpp;
  //  ki = kii;
  //  kd = kdd;
  //  Serial.printf("%.2f\t%.2f\t%.2f\n", kpp, kii, kdd);
}
bool pidmotor::calibrate(int input_target) {
  while (cal_flag == false) {
    enc->write(0);
    PIDAutotuner tuner = PIDAutotuner();
    tuner.setTargetInputValue(input_target);
    tuner.setLoopInterval(loopInterval);
    tuner.setOutputRange(-255, 255);
    tuner.setZNMode(PIDAutotuner::ZNModeNoOvershoot);
    tuner.startTuningLoop(micros());
    long microseconds;
    while (!tuner.isFinished()) {
      //    Serial.println("hyeyheyhe");
      long prevMicroseconds = microseconds;
      microseconds = micros();
      double input = enc->read();
      double output = tuner.tunePID(input, microseconds);
      if (output > 0) move(1, abs(output));
      else move(-1, abs(output));
      while (micros() - microseconds < loopInterval) delayMicroseconds(1);
    }
    move(0, 0);
    kp = tuner.getKp();
    ki = tuner.getKi();
    kd = tuner.getKd();
    set_coeffs(tuner.getKp(), tuner.getKi(), tuner.getKd());
    Serial.printf("Calibration done!! New co-effs are kp : %.2f\tki : %.2f\tkd : %.2f\n", kp, ki, kd);
    move(0, 0);
    enc->write(0);
    cal_flag = true;
  }
  return 1;
}
void IRAM_ATTR pidmotor::compute(volatile int target) {
  long current_time = micros();
  float time_change = ((float) (current_time - previous_time)) / ( 1.0e6 );
  previous_time = current_time;
  int e = enc->read() - target;
  float dedt = (e - previous_error) / time_change;
  eintegral += (e * time_change);
  if (eintegral > 255) eintegral = 255;
  else if (eintegral < 0) eintegral = 0;
  //  float u = kp * e + ki * eintegral + kd * dedt - target * (kd / kp);
  float u = kp * e + ki * eintegral + kd * dedt + target * 0.011;
  float pwr = fabs(u);
  previous_error = e;
  if ( pwr > 255 ) pwr = 255;
  int dir = -1;
  if (u < 0) dir = 1;
  move(dir, pwr);
  //  Serial.printf("%.2f\t%.2f\t%.2f\n", kp, ki, kd);
}
bool pidmotor::test_sys(int tar) {
  while (enc->read() <= tar && sys_flag == false) {
    move(1, 150);
    Serial.printf("encoder reading CW - %d\n", enc->read());
  }
  while (enc->read() >= 0) {
    move(-1, 150);
    Serial.printf("encoder reading CCW - %d\n", enc->read());
    sys_flag = true;
  }
  move(0, 255);
  Serial.printf("System Tested Successfully\n");
  return 1;
}
bool pidmotor::test_cal(int tar) {
  if (!cal_flag) {
    Serial.printf("Error testing the calibration, First calibrate the motor and then run this function"); return 0;
  }
  else {
    compute(tar);
    return 1;
  }
}
void pidmotor::set_motor_limit(int min, int max) {
  mmin = min;
  mmax = max;
}
