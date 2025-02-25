#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <stdio.h>
#include "pico_copter.hpp"
#include "pico/stdlib.h"
#include "sensor.hpp"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <Eigen/Dense>
#include "ekf.hpp"
#include <math.h>
#include "radio.hpp"

typedef unsigned char byte;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

#define BATTERY_VOLTAGE (11.1)
#define BUFFER_SIZE 16


//グローバル関数の宣言
void loop_400Hz(void);
void control_init();
void rate_control(void);
void angle_control(void);
void gyro_calibration(void);
void variable_init(void);
void log_output(void);

//グローバル変数
extern uint8_t LockMode;
extern volatile uint8_t Logoutputflag;
extern float ideal;
extern float input;
extern float lotated_distance;
extern float z_acc;
extern float func_time;
extern float T_ref;
extern uint64_t count_up;
// extern char buffer[BUFFER_SIZE];
// extern uint8_t print_flag;
class PID
{
  private:
    float m_kp;
    float m_ti;
    float m_td;
    float m_filter_time_constant;
    float m_err,m_err2,m_err3;
    float m_h;
  public:
    float m_filter_output;
    float m_integral;
    PID();
    void set_parameter(
        float kp, 
        float ti, 
        float td,
        float filter_time_constant, 
        float h);
    void reset(void);
    void i_reset(void);
    void printGain(void);
    float filter(float x);
    float update(float err);
};

class Filter
{
  private:
    float m_state;
    float m_T;
    float m_h;
  public:
    float m_out;
    Filter();
    void set_parameter(
        float T,
        float h);
    void reset(void);
    float update(float u);
};


#endif
