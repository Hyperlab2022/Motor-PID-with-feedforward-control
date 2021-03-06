/*
   pidmotor.h

    Created on: Jun 01, 2022
        Author: yashmewada9618
        This is the header file for pid motor calibration and test calibration routines, this library is intended to use
        changelog v1 -> Inplemented Integral windup logic.
        changelog v1.1 -> Inplemented feedforward control logic. (See changed defination of PID output in .cpp file)
        for DC motors with a 3 pin motor drivers only. It cytron in serial simplified mode is to be used refer GTU ROBOTICS CLUB github repos
    To-Do:
        1. Make this library compatible with any stream specified driver and 2 pin motor drivers
        2. Store P,I,D co effs onto eeprom so that we don't need to recalibrate the motor every time we power on the driver.
    Note:
        -> Donot forget to test the CCW and CW direction of the motor and/or encoder for proper functionalities.
*/
#ifndef pidmotor_h
#define pidmotor_h
#include "Arduino.h"
#include <Encoder.h>
#include <pidautotuner.h>
#define loopInterval 5000
class pidmotor {
  private:
    Encoder *enc;
  public:
    //    3.56  ki : 0.14 kd : 11.47
    long previous_time = 0; double mmin, mmax, totalerror = 0;
    float previous_error = 0;
    float eintegral = 0;
    byte enca, encb;
    double last_input, Pterm, Iterm, Dterm;
    //    int address = 0;
    pidmotor(byte _PWM, byte _IN1, byte _IN2, byte _limit, Encoder *enc);
    void setinputlimits(double mininput = 0, double maxinput = 720);
    byte limit, PWM, IN1, IN2, speed;
    bool cal_flag = false, sys_flag = false;
    float kp, ki, kd;
    void get_coeffs();
    void move(int dir, byte pwm);
    //    inline void Task1code(void * parameter) __attribute__((always_inline));
    void IRAM_ATTR compute(volatile int target = 0);
    bool test_sys(int tar = 1320);
    bool calibrate(int input_target = 1000);
    bool test_cal(int tar = 1320); void set_motor_limit(int min, int max);
    void set_coeffs(float _kp = 3.56, float _ki = 3.14, float kd = 11.47);
    inline int32_t readenc() {
      return enc->read();
    }
    inline void encwrite(int32_t pos) {
      enc->write(pos);
    }
};
#endif
