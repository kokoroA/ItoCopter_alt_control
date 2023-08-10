#include "control.hpp"

//Sensor data 1
float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;

//Times
float Elapsed_time=0.0;
uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
float FR_duty, FL_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;  
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
float Pref=0.0,Qref=0.0,Rref=0.0;
const float Phi_trim   = 0.01;
const float Theta_trim = 0.02;
const float Psi_trim   = 0.0;

//Extended Kalman filter 
Matrix<float, 7 ,1> Xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> Xe = MatrixXf::Zero(7,1);
Matrix<float, 6 ,1> Z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 6, 6> Q;// = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R;// = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7 ,6> G;
Matrix<float, 3 ,1> Beta;

//Auto_fly
uint64_t count_up = 0;
float auto_mode_count = 0;
float start_time;
float current_time;
float func_time;
float z_acc;
float base_dis_count = 0;
uint16_t Range;
int8_t status;
uint8_t Temp2;
uint8_t Temp;
uint8_t IntPol;
uint8_t val = 0;
uint8_t range_gbuf[16];
float range_flag = 0;
uint16_t altitude_count = 0;
float stick;
float last_stick;
float hov_distance_flag = 0;
float auto_mode = 0;
float ideal;
float takeoff_flag = 0;
float landing_flag = 0;
float initialize_flag =1;
float hovering_flag = 0;
float hove_distance;
float hove_time = 0.0;
float flying_mode = 0;
float input= 0;
Matrix<float, 3 ,3> lotate_mat = MatrixXf::Zero(3,3);
float f_distance = 0;
float f_distance2 = 0;
float f_distance3 = 0;
float lotated_distance = 0;
Matrix<float, 3 ,1> distance_mat = MatrixXf::Zero(3,1);
Matrix<float, 3 ,1> f_distance_mat = MatrixXf::Zero(3,1);

//Log
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
const uint8_t DATANUM=45; //Log Data Number
const uint32_t LOGDATANUM=48000;
float Logdata[LOGDATANUM]={0.0};

//State Machine
uint8_t LockMode=0;
float Disable_duty =0.10;
float Flight_duty  =0.18;//0.2/////////////////
uint8_t OverG_flag = 0;

//PID object and etc.
Filter acc_filter;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void kalman_filter(void);
void logging(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void printPQR(void);
void Auto_fly(void);
void Auto_fly_initialize(void);
void altitude_rate_control(void);
void Auto_takeoff(void);
void Auto_landing(void);
void Hovering(void);
float lotate_altitude(float l_distance);
void lotate_altitude_init(float Theta,float Psi,float Phi);
void test_Hovering(void);

#define AVERAGE 2000
#define KALMANWAIT 6000

//Main loop
//This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  static uint8_t led=1;
  S_time=time_us_32();
  
  //割り込みフラグリセット
  pwm_clear_irq(2);


  if (Arm_flag==0)
  {
      //motor_stop();
      Elevator_center = 0.0;
      Aileron_center = 0.0;
      Rudder_center = 0.0;
      Pbias = 0.0;
      Qbias = 0.0;
      Rbias = 0.0;
      Phi_bias = 0.0;
      Theta_bias = 0.0;
      Psi_bias = 0.0;
      return;
  }
  else if (Arm_flag==1)
  {
    motor_stop();
    //Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      //Sensor Read
      sensor_read();
      Aileron_center  += Chdata[3];
      Elevator_center += Chdata[1];
      Rudder_center   += Chdata[0];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if(BiasCounter<KALMANWAIT)
    {
      //Sensor Read
      sensor_read();
      if(BiasCounter == AVERAGE)
      {
        Elevator_center = Elevator_center/AVERAGE;
        Aileron_center  = Aileron_center/AVERAGE;
        Rudder_center   = Rudder_center/AVERAGE;
        Pbias = Pbias/AVERAGE;
        Qbias = Qbias/AVERAGE;
        Rbias = Rbias/AVERAGE;
        Mx_ave = Mx_ave/AVERAGE;
        My_ave = My_ave/AVERAGE;
        Mz_ave = Mz_ave/AVERAGE;

        Xe(4,0) = Pbias;
        Xe(5,0) = Qbias;
        Xe(6,0) = Rbias;
        Xp(4,0) = Pbias;
        Xp(5,0) = Qbias;
        Xp(6,0) = Rbias;
        MN = Mx_ave;
        ME = My_ave;
        MD = Mz_ave;
      }
      
      AngleControlCounter++;
      if(AngleControlCounter==4)
      {
        AngleControlCounter=0;
        sem_release(&sem);
      
      }
      Phi_bias   += Phi;
      Theta_bias += Theta;
      Psi_bias   += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Phi_bias   = Phi_bias/KALMANWAIT;
      Theta_bias = Theta_bias/KALMANWAIT;
      Psi_bias   = Psi_bias/KALMANWAIT;
      return;
    }
  }
  else if( Arm_flag==2)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        led=0;
        gpio_put(LED_PIN,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Arm_flag=3;
      }
      return;
    }
    //LED Blink
    gpio_put(LED_PIN, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
   
    //Rate Control (400Hz)
    rate_control();
   
    if(AngleControlCounter==4)
    {
      AngleControlCounter=0;
      //Angle Control (100Hz)
      sem_release(&sem);
    }
    AngleControlCounter++;
  }
  else if(Arm_flag==3)
  {
    motor_stop();
    OverG_flag = 0;
    if(LedBlinkCounter<10){
      gpio_put(LED_PIN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      gpio_put(LED_PIN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
    
    //Get Stick Center 
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
  
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait  output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        LockMode=2;//Enable Flight
        Arm_flag=2;
      }
      return;
    }

    if(logdata_out_com()==1)
    {
      Arm_flag=4;
      return;
    }
  }
  else if(Arm_flag==4)
  {
    motor_stop();
    Logoutputflag=1;
    //LED Blink
    gpio_put(LED_PIN, led);
    if(LedBlinkCounter<400){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
  E_time=time_us_32();
  D_time=E_time-S_time;
}

void control_init(void)
{
  acc_filter.set_parameter(0.005, 0.0025);
  //Rate control
  p_pid.set_parameter( 2.0, 0.145, 0.028, 0.015, 0.0025);//3.4
  q_pid.set_parameter( 2.1, 0.125, 0.028, 0.015, 0.0025);//3.8
  r_pid.set_parameter(12.0, 0.5, 0.008, 0.015, 0.0025);//9.4
  //Angle control
  phi_pid.set_parameter  ( 5.5, 9.5, 0.025, 0.018, 0.01);//6.0
  theta_pid.set_parameter( 5.5, 9.5, 0.025, 0.018, 0.01);//6.0
  psi_pid.set_parameter  ( 0.0, 10.0, 0.010, 0.03, 0.01);
  //Rate control
  //p_pid.set_parameter(3.3656, 0.1, 0.0112, 0.01, 0.0025);
  //q_pid.set_parameter(3.8042, 0.1, 0.0111, 0.01, 0.0025);
  //r_pid.set_parameter(9.4341, 0.11, 0.0056, 0.01, 0.0025);
  //Angle control
  //phi_pid.set_parameter  ( 9.0   , 0.07, 0.0352,  0.01, 0.01);
  //theta_pid.set_parameter( 8.5583, 0.1 , 0.0552,  0.01, 0.01);
  //psi_pid.set_parameter  ( 9.0256, 0.11, 0.0034,  0.01, 0.01);
}

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[2]<CH3MIN+80 
   && Chdata[0]>CH1MAX-80
   && Chdata[3]<CH4MIN+80 
   && Chdata[1]>CH2MAX-80)
  { 
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;

}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[4]<(CH5MAX+CH5MIN)*0.5 
   && Chdata[2]<CH3MIN+80 
   && Chdata[0]<CH1MIN+80
   && Chdata[3]>CH4MAX-80 
   && Chdata[1]>CH2MAX-80)
  {
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

void lotate_altitude_init(float Theta,float Psi,float Phi){
  lotate_mat(0,0) = cos(Theta)*cos(Psi);
  lotate_mat(0,1) = -sin(Psi)*cos(Theta);
  lotate_mat(0,2) = sin(Theta);
  lotate_mat(1,0) = (sin(Phi)*sin(Theta)*cos(Psi))+(cos(Phi)*sin(Psi));
  lotate_mat(1,1) = (-sin(Phi)*sin(Theta)*sin(Psi)) + (cos(Phi) * cos(Psi));
  lotate_mat(1,2) = -sin(Phi)*cos(Theta);
  lotate_mat(2,0) = (-sin(Theta)*cos(Phi)*cos(Psi)) + (sin(Phi)*sin(Psi));
  lotate_mat(2,1) = (sin(Theta)*cos(Phi)*sin(Psi)) + (sin(Phi)*cos(Psi));
  lotate_mat(2,2) = cos(Phi)*cos(Theta);
}

float lotate_altitude(float l_distance){
  // distance_mat(0,0) = 0;
  // distance_mat(0,1) = 0;
  distance_mat(0,2) = l_distance;
  f_distance_mat = lotate_mat * distance_mat;
  f_distance = f_distance_mat(0,1);
  f_distance2 = f_distance_mat(0,2);
  f_distance3 = f_distance_mat(0,3);

  return f_distance;
}

// //自動離着陸モード
// void Auto_fly(void){
//   test_Hovering();
//   // if (flying_mode == 1){
//   //   Auto_takeoff();
//   // }

//   // else if(flying_mode == 2){
//   //   Hovering();
//   // }

//   // else if (flying_mode == 3){
//   //   Auto_landing();
//   // }

// }

// void test_Hovering(void){
//   u = Kalman_PID(lotated_distance,hove_distance);
// }

// //ホバリング
// void Hovering(void){
//   //実験なので4秒
//   //本番はゴールを見つけたら着陸モード
//   if (hove_time < 4.0)
//   {
//     u = Kalman_PID(lotated_distance,hove_distance);
//     hove_time + 0.03;
//   }
//   else{
//     // hovering_flag = 0;
//     // landing_flag = 1;
//     flying_mode = 3;
//   }
// }

// //自動着陸
// void Auto_landing(void){
//   ideal = lotated_distance - 3;//高度の目標値更新のコード
//   u = Kalman_PID(lotated_distance,ideal);
//   if (lotated_distance < 90)//自動離着陸終了の処理(ある高度まで行ったらモーターを止める)
//   {
//     motor_stop();
//     // landing_flag = 0;
//     flying_mode = 4;
//   }
// }

// //自動離陸
// void Auto_takeoff(void){
//   ideal = lotated_distance + 3;
//   u = Kalman_PID(lotated_distance,ideal);
//   if (lotated_distance >= 400)
//   {
//     // takeoff_flag = 0;
//     // hovering_flag = 1;
//     hove_distance = lotated_distance;
//     flying_mode = 2;
//   }
// }

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  //  if (flying_mode == 0){
  //   stick = Chdata[2];
  //   flying_mode = 1;
  // }

  //Read Sensor Value
  sensor_read();

  // lotate_altitude_init(Theta,Psi,Phi);
  // //lotate_altitudeもfloat型に変換して計算しているか確認
  // lotated_distance = lotate_altitude(distance);

  // auto_mode = 0;
  // hov_distance_flag = 0;
  // last_stick = stick;

  // if((Chdata[2]-last_stick) < 13 || (Chdata[2]-last_stick) > 13)
  // {
  //   motor_stop();
  // }

  //通常モード
  // if (Chdata[4]<(CH5MAX+CH5MIN)*0.5){
  //   auto_mode = 1;
  //   if (base_dis_count == 0){
  //     stick = Chdata[2];
  //     base_dis_count = 1;
  //   }
  //   // //スティック入力を理想値とする。スティック入力の値の変更が無ければ、その値の高度を保つ
  //   // if (Chdata[2] >= 690 && Chdata[2] <= 715)
  //   // {
  //   //   ideal = lotated_distance;
  //   //   u = Kalman_PID(lotated_distance,ideal);
  //   // }
  //   // else if(Chdata[2] < 690)
  //   // {
  //   //   if(Chdata[2] >= CH3MIN && Chdata[2] <= 350){
  //   //     motor_stop();
  //   //   }
  //   //   ideal = lotated_distance - 0.1 * (2*(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN) - 1);
  //   //   u = Kalman_PID(lotated_distance,ideal);
  //   // }
  //   // else if(Chdata[2] > 715)
  //   // {
  //   //   ideal = lotated_distance + 0.1 * (2*(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN) - 1);
  //   //   u = Kalman_PID(lotated_distance,ideal);
  //   // }

  //   //高度を保つ処理
  //   if(Chdata[2] >= (stick+15) && Chdata[2] >= (stick-15)){
  //     //ideal = lotated_distance;
  //     ideal = distance;
  //     u = Kalman_PID(distance,ideal);
  //   }
  //   //スティックの変化量に比例して高度上昇の処理
  //   else if (Chdata[2] > stick + 15)
  //   {
  //     ideal = distance + 0.1 * (2*(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN) - 1);
  //     u = Kalman_PID(distance,ideal);
  //   }
  //   //スティックの変化量に比例して高度を下げる処理
  //   else if (Chdata[2] < stick + 15)
  //   {
  //     if(Chdata[2] >= CH3MIN && Chdata[2] <= 350){     
  //       motor_stop();
  //     }

  //     ideal = distance - 0.1 * (2*(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN) - 1);
  //     u = Kalman_PID(distance,ideal);
  //   }
    
  // }
  // else {
  //   base_dis_count = 0;
  // }

  // ideal = (float)distance - 0.1 * (2.0*(500-(float)CH3MIN)/((float)CH3MAX-(float)CH3MIN) - 1.0f);
  // ideal = 5;
  // input = Kalman_PID((float)distance,(float)distance);



  // altitude_count += 1;
  // if(altitude_count == 21){
  //   get_Altitude();
  //   altitude_count = 0;
  // }

  //Get Bias
  //Pbias = Xe(4, 0);
  //Qbias = Xe(5, 0);
  //Rbias = Xe(6, 0);

  //Control angle velocity
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  //Get reference
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;
  T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);

  if(Chdata[4] > (CH5MAX + CH5MIN)*0.5){
    auto_mode =1;
  }
  else{
    auto_mode =0;
    auto_mode_count = 0;
    T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  }

  if (auto_mode ==1){

    if(auto_mode_count ==0){
      auto_mode_count = 1;
      //ideal = Chada[2]
      ideal = lotated_distance;
    }

    input = Kalman_PID(lotated_distance,ideal,z_acc);
    // T_ref = 4.25 + (input/11.1);
    T_ref = 4.25 - (input/11.1);
    //釣り合いのT = 0.5と仮定
    // T_ref = 0.6 * BATTERY_VOLTAGE*( 0.5 - (input*0.001));
    // T_ref = 0.6 * BATTERY_VOLTAGE*input*0.001;
    //T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  }


  // if (auto_mode == 0){
  //   T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  // }
  // else if(auto_mode ==1){
  //     T_ref = 0.6 * BATTERY_VOLTAGE*u;//値を１の間にするために0.6をかける
  // }

  //Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  //PID
  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  //Motor Control
  // 1250/11.1=112.6
  // 1/11.1=0.0901
  
  // FR_duty = (T_ref +(-P_com +Q_com -R_com)*0.25)*0.0901;
  // FL_duty = (T_ref +( P_com +Q_com +R_com)*0.25)*0.0901;
  // RR_duty = (T_ref +(-P_com -Q_com +R_com)*0.25)*0.0901;
  // RL_duty = (T_ref +( P_com -Q_com -R_com)*0.25)*0.0901;
    FR_duty = (T_ref)*0.0901;
    FL_duty = (T_ref )*0.0901;
    RR_duty = (T_ref)*0.0901;
    RL_duty = (T_ref)*0.0901;
  //FR_duty = (T_ref)*0.0901;
  //FL_duty = (T_ref)*0.0901;
  //RR_duty = (T_ref)*0.0901;
  //RL_duty = (T_ref)*0.0901;
  
  float minimum_duty=0.1;
  const float maximum_duty=0.95;
  minimum_duty = Disable_duty;

  if (FR_duty < minimum_duty) FR_duty = minimum_duty;
  if (FR_duty > maximum_duty) FR_duty = maximum_duty;

  if (FL_duty < minimum_duty) FL_duty = minimum_duty;
  if (FL_duty > maximum_duty) FL_duty = maximum_duty;

  if (RR_duty < minimum_duty) RR_duty = minimum_duty;
  if (RR_duty > maximum_duty) RR_duty = maximum_duty;

  if (RL_duty < minimum_duty) RL_duty = minimum_duty;
  if (RL_duty > maximum_duty) RL_duty = maximum_duty;

  //Duty set
  if(T_ref/BATTERY_VOLTAGE < Disable_duty)
  {
    motor_stop();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    Pref=0.0;
    Qref=0.0;
    Rref=0.0;
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
    Phi_bias   = Phi;
    Theta_bias = Theta;
    Psi_bias   = Psi;
  }
  else
  {
    if (OverG_flag==0){
      set_duty_fr(FR_duty);
      set_duty_fl(FL_duty);
      set_duty_rr(RR_duty);
      set_duty_rl(RL_duty);
    }
    else motor_stop();
    //printf("%12.5f %12.5f %12.5f %12.5f\n",FR_duty, FL_duty, RR_duty, RL_duty);
  }
 
  //printf("\n");

  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, fr_duty, fl_duty, rr_duty, rl_duty, p_rate, q_rate, r_rate);
  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, p_com, q_com, r_com, p_ref, q_ref, r_ref);
  //printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n", 
  //    Elapsed_time, Phi, Theta, Psi, Phi_bias, Theta_bias, Psi_bias);
  //Elapsed_time = Elapsed_time + 0.0025;
  //Logging
  //logging();
}

void angle_control(void)
{
  float phi_err,theta_err,psi_err;
  float q0,q1,q2,q3;
  float e23,e33,e13,e11,e12;
  while(1)
  {
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    S_time2=time_us_32();
    kalman_filter();
    q0 = Xe(0,0);
    q1 = Xe(1,0);
    q2 = Xe(2,0);
    q3 = Xe(3,0);
    e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    e12 = 2*(q1*q2 + q0*q3);
    e13 = 2*(q1*q3 - q0*q2);
    e23 = 2*(q2*q3 + q0*q1);
    e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    Phi = atan2(e23, e33);
    Theta = atan2(-e13, sqrt(e23*e23+e33*e33));
    Psi = atan2(e12,e11);

    //Get angle ref 
    Phi_ref   = Phi_trim   + 0.3 *M_PI*(float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5)*2/(CH4MAX-CH4MIN);
    Theta_ref = Theta_trim + 0.3 *M_PI*(float)(Chdata[1] - (CH2MAX+CH2MIN)*0.5)*2/(CH2MAX-CH2MIN);
    Psi_ref   = Psi_trim   + 0.8 *M_PI*(float)(Chdata[0] - (CH1MAX+CH1MIN)*0.5)*2/(CH1MAX-CH1MIN);

    //Error
    phi_err   = Phi_ref   - (Phi   - Phi_bias);
    theta_err = Theta_ref - (Theta - Theta_bias);
    psi_err   = Psi_ref   - (Psi   - Psi_bias);
    
    //PID Control
    if (T_ref/BATTERY_VOLTAGE < Flight_duty)
    {
      Pref=0.0;
      Qref=0.0;
      Rref=0.0;
      phi_pid.reset();
      theta_pid.reset();
      psi_pid.reset();
      Aileron_center  = Chdata[3];
      Elevator_center = Chdata[1];
      Rudder_center   = Chdata[0];
      /////////////////////////////////////
      Phi_bias   = Phi;
      Theta_bias = Theta;
      Psi_bias   = Psi;
      /////////////////////////////////////
    }
    else
    {
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = Psi_ref;//psi_pid.update(psi_err);//Yawは角度制御しない
    }

    //Logging
    logging();

    E_time2=time_us_32();
    D_time2=E_time2-S_time2;

  }
}

void logging(void)
{  
  //Logging
  if(Chdata[4]>(CH5MAX+CH5MIN)*0.5)
  { 
    if(Logflag==0)
    {
      Logflag=1;
      LogdataCounter=0;
    }
    if(LogdataCounter+DATANUM<LOGDATANUM)
    {
      Logdata[LogdataCounter++]=Xe(0,0);                  //1
      Logdata[LogdataCounter++]=Xe(1,0);                  //2
      Logdata[LogdataCounter++]=Xe(2,0);                  //3
      Logdata[LogdataCounter++]=Xe(3,0);                  //4
      Logdata[LogdataCounter++]=Xe(4,0);                  //5
      Logdata[LogdataCounter++]=Xe(5,0);                  //6
      Logdata[LogdataCounter++]=Xe(6,0);                  //7
      Logdata[LogdataCounter++]=Wp;//-Pbias;              //8
      Logdata[LogdataCounter++]=Wq;//-Qbias;              //9
      Logdata[LogdataCounter++]=Wr;//-Rbias;              //10

      Logdata[LogdataCounter++]=Ax;                       //11
      Logdata[LogdataCounter++]=Ay;                       //12
      Logdata[LogdataCounter++]=Az;                       //13
      Logdata[LogdataCounter++]=Mx;                       //14
      Logdata[LogdataCounter++]=My;                       //15
      Logdata[LogdataCounter++]=Mz;                       //16
      Logdata[LogdataCounter++]=Pref;                     //17
      Logdata[LogdataCounter++]=Qref;                     //18
      Logdata[LogdataCounter++]=Rref;                     //19
      Logdata[LogdataCounter++]=Phi-Phi_bias;             //20

      Logdata[LogdataCounter++]=Theta-Theta_bias;         //21
      Logdata[LogdataCounter++]=Psi-Psi_bias;             //22
      Logdata[LogdataCounter++]=Phi_ref;                  //23
      Logdata[LogdataCounter++]=Theta_ref;                //24
      Logdata[LogdataCounter++]=Psi_ref;                  //25
      Logdata[LogdataCounter++]=P_com;                    //26
      Logdata[LogdataCounter++]=Q_com;                    //27
      Logdata[LogdataCounter++]=R_com;                    //28
      Logdata[LogdataCounter++]=p_pid.m_integral;//m_filter_output;    //29
      Logdata[LogdataCounter++]=q_pid.m_integral;//m_filter_output;    //30

      Logdata[LogdataCounter++]=r_pid.m_integral;//m_filter_output;    //31
      Logdata[LogdataCounter++]=phi_pid.m_integral;//m_filter_output;  //32
      Logdata[LogdataCounter++]=theta_pid.m_integral;//m_filter_output;//33
      Logdata[LogdataCounter++]=Pbias;                    //34
      Logdata[LogdataCounter++]=Qbias;                    //35

      Logdata[LogdataCounter++]=Rbias;                    //36
      Logdata[LogdataCounter++]=T_ref;                    //37
      Logdata[LogdataCounter++]=Acc_norm;                 //38
      Logdata[LogdataCounter++]=distance;                 //39
      Logdata[LogdataCounter++]=mu_Yn_est(1,0);           //40
      Logdata[LogdataCounter++]=mu_Yn_est(0,0);           //41
      Logdata[LogdataCounter++]=FR_duty;                   //42
      Logdata[LogdataCounter++]=FL_duty;                    //43
      Logdata[LogdataCounter++]=RR_duty;                  //44
      Logdata[LogdataCounter++]=RL_duty;                  //45
    }
    else Logflag=2;
  }
  else
  { 
    if(Logflag>0)
    {
      Logflag=0;
      LogdataCounter=0;
    }
  }
}

void log_output(void)
{
  if(LogdataCounter==0)
  {
    printPQR();
    printf("#Roll rate PID gain\n");
    p_pid.printGain();
    printf("#Pitch rate PID gain\n");
    q_pid.printGain();
    printf("#Yaw rate PID gain\n");
    r_pid.printGain();
    printf("#Roll angle PID gain\n");
    phi_pid.printGain();
    printf("#Pitch angle PID gain\n");
    theta_pid.printGain();
  }
  if(LogdataCounter+DATANUM<LOGDATANUM)
  {
    //LockMode=0;
    printf("%10.2f ", Log_time);
    Log_time=Log_time + 0.01;
    for (uint8_t i=0;i<DATANUM;i++)
    {
      printf("%12.5f",Logdata[LogdataCounter+i]);
    }
    printf("\n");
    LogdataCounter=LogdataCounter + DATANUM;
  }
  else 
  {
    Arm_flag=3;
    Logoutputflag=0;
    LockMode=0;
    Log_time=0.0;
    LogdataCounter=0;
  }
}


void gyroCalibration(void)
{
  float wp,wq,wr;
  float sump,sumq,sumr;
  uint16_t N=400;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump=sump+Wp;
    sumq=sumq+Wq;
    sumr=sumr+Wr;
  }
  Pbias=sump/N;
  Qbias=sumq/N;
  Rbias=sumr/N;
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read();
  Ax =-acceleration_mg[0]*GRAV*0.001;
  Ay =-acceleration_mg[1]*GRAV*0.001;
  Az = acceleration_mg[2]*GRAV*0.001;
  Wp = angular_rate_mdps[0]*M_PI*5.55555555e-6;//5.5.....e-6=1/180/1000
  Wq = angular_rate_mdps[1]*M_PI*5.55555555e-6;
  Wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6;
  Mx0 =-magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 =-magnetic_field_mgauss[2];

  
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>250.0) OverG_flag = 1;
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrt(Wp*Wp + Wq*Wq + Wr*Wr);
  if (rate_norm > 6.0) OverG_flag =1;

/*地磁気校正データ
回転行列
[[ 0.65330968  0.75327755 -0.07589064]
 [-0.75666134  0.65302622 -0.03194321]
 [ 0.02549647  0.07829232  0.99660436]]
中心座標
122.37559195017053 149.0184454603531 -138.99116060635413
W
-2.432054387460946
拡大係数
0.003077277151877191 0.0031893151610213463 0.0033832794976645804

//回転行列
const float rot[9]={0.65330968, 0.75327755, -0.07589064,
                   -0.75666134, 0.65302622, -0.03194321,
                    0.02549647, 0.07829232,  0.99660436};
//中心座標
const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
//拡大係数
const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};
*/
  //回転行列
  const float rot[9]={-0.78435472, -0.62015392, -0.01402787,
                       0.61753358, -0.78277935,  0.07686857,
                      -0.05865107,  0.05162955,  0.99694255};
  //中心座標
  const float center[3]={-109.32529343620176, 72.76584808916506, 759.2285249891385};
  //拡大係数
  const float zoom[3]={0.002034773458122364, 0.002173892202021849, 0.0021819494099235273};

//回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*Mx0 +rot[1]*My0 +rot[2]*Mz0 -center[0]);
  my1 = zoom[1]*( rot[3]*Mx0 +rot[4]*My0 +rot[5]*Mz0 -center[1]);
  mz1 = zoom[2]*( rot[6]*Mx0 +rot[7]*My0 +rot[8]*Mz0 -center[2]);
//逆回転
  Mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  My = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  Mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 

  mag_norm=sqrt(Mx*Mx +My*My +Mz*Mz);
  Mx/=mag_norm;
  My/=mag_norm;
  Mz/=mag_norm;
 
  // start_time = time_us_64();
  if(isDataReady == 0)
  {
    Status = VL53L1X_CheckForDataReady(dev,&isDataReady);
  }
  else if (isDataReady == 1)
  {
    //data_count = data_count + 1;
    isDataReady = 0;
    Status = VL53L1X_GetRangeStatus(dev,&rangeStatus);
    Status = VL53L1X_GetDistance(dev,&distance);
    Status = VL53L1X_ClearInterrupt(dev);
    // z_acc  = Az-9.80665;
    //何故 gを引くのか
    z_acc = Az - 9.76548;
    lotate_altitude_init(Theta,Psi,Phi);
    lotated_distance = lotate_altitude(distance);
    // z_acc  = Az-9.80665;
    //input = Kalman_PID(lotated_distance,z_acc);
  }
  
  // current_time = time_us_64();
  // func_time = (current_time - start_time)/1000000.0;
  // count_up = count_up + 1;


  // if(range_flag==0)
  // {
  //   //write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);
  //   VL53L1_WrByte(dev,VL53L0X_REG_SYSRANGE_START,0x01);
  //   range_flag = 1;
  // }
  //val = read_byte_data_at(VL53L0X_REG_RESULT_RANGE_STATUS);
  // val = VL53L1_RdByte(dev,VL53L0X_REG_RESULT_RANGE_STATUS);

  // VL53L1_RdByte(dev,VL53L0X_REG_RESULT_RANGE_STATUS,&val);
  // if (val & 0x01)
  // {
  //   range_flag = 0;
  //read_block_data_at(dev,0x14, 12,&range_gbuf);
  // Range = makeuint16(range_gbuf[11], range_gbuf[10]);
  // } 

  // altitude_count = altitude_count + 1;
  // if(altitude_count == 20){
  //   altitude_count = 0;
  //   if(isDataReady == 0)
  //   {
  //     Status = VL53L1X_CheckForDataReady(dev,&isDataReady);
  //     // uint8_t Temp;
  //     // uint8_t IntPol;
  //     // Temp = 0;
  //     // IntPol = 0;

  //     //status |= VL53L1X_GetInterruptPolarity(dev, &IntPol);
  //     //VL53L1X_GetInterruptPolarityの処理
  //     // uint8_t Temp2;

  //     // status |= VL53L1_RdByte(dev, GPIO_HV_MUX__CTRL, &Temp2);
  //     // Temp2 = Temp2 & 0x10;
  //     // IntPol = !(Temp2>>4);
  //     //printf("Temp(getInterrupt) : %d\n",pInterruptPolarity);
  //     //ここまでVL53L1X_GetInterruptPolarityの処理

  //     //status |= VL53L1_RdByte(dev, GPIO__TIO_HV_STATUS, &Temp);
  //     /* Read in the register to check if a new value is available */
  //     //printf("status (CheckForDataReady) : %d\n" , status);
  //     //printf("Temp and IntPol : %d , %d \n", (Temp&1),IntPol);
  //     // if (status == 0){
  //     //   if ((Temp & 1) == IntPol)
  //     //     isDataReady = 1;
  //     //   else
  //     //     isDataReady = 0;
  //     // }
  //   }
  //   else if (isDataReady == 1)
  //   {
  //     //data_count = data_count + 1;
  //     isDataReady = 0;
  //     Status = VL53L1X_GetRangeStatus(dev,&rangeStatus);
  //     Status = VL53L1X_GetDistance(dev,&distance);
  //     Status = VL53L1X_ClearInterrupt(dev);
  //     // z_acc  = Az;
  //     // input = Kalman_PID((float)distance,Ax);
  //   }
  // }
}

void variable_init(void)
{
  //Variable Initalize
  Xe << 1.00, 0.0, 0.0, 0.0,0.0,0.0, 0.0;
  Xp =Xe;

  Q <<  6.0e-5, 0.0    , 0.0    ,  0.0    , 0.0    , 0.0   ,
        0.0   , 5.0e-5 , 0.0    ,  0.0    , 0.0    , 0.0   ,
        0.0   , 0.0    , 2.8e-5 ,  0.0    , 0.0    , 0.0   ,
        0.0   , 0.0    , 0.0    ,  5.0e-5 , 0.0    , 0.0   ,
        0.0   , 0.0    , 0.0    ,  0.0    , 5.0e-5 , 0.0   ,
        0.0   , 0.0    , 0.0    ,  0.0    , 0.0    , 5.0e-5;

  R <<  1.701e0, 0.0     , 0.0     , 0.0   , 0.0   , 0.0   ,
        0.0     , 2.799e0, 0.0     , 0.0   , 0.0   , 0.0   ,
        0.0     , 0.0     , 1.056e0, 0.0   , 0.0   , 0.0   ,
        0.0     , 0.0     , 0.0     , 2.3e-1, 0.0   , 0.0   ,
        0.0     , 0.0     , 0.0     , 0.0   , 1.4e-1, 0.0   ,
        0.0     , 0.0     , 0.0     , 0.0   , 0.0   , 0.49e-1;
          
  G <<   1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 
        -1.0, 1.0,-1.0, 0.0, 0.0, 0.0, 
        -1.0,-1.0, 1.0, 0.0, 0.0, 0.0, 
         1.0,-1.0,-1.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  
  G=G*0.01;

  Beta << 0.0, 0.0, 0.0;
  
  P <<  1e0,  0,   0,   0,   0,  0,   0,  
        0  ,1e0,   0,   0,   0,  0,   0,
        0  ,  0, 1e0,   0,   0,  0,   0,  
        0  ,  0,   0, 1e0,   0,  0,   0, 
        0  ,  0,   0, 0  , 1e0,  0,   0,  
        0  ,  0,   0, 0  ,   0,1e0,   0,  
        0  ,  0,   0, 0  ,   0,  0, 1e0;
}

void printPQR(void)
{
  volatile int m=0;
  volatile int n=0;
  //Print P
  printf("#P\n");
  for (m=0;m<7;m++)
  {
    printf("# ");
    for (n=0;n<7;n++)
    {
      printf("%12.4e ",P(m,n));
    }
    printf("\n");
  }
  //Print Q
  printf("#Q\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",Q(m,n));
    }
    printf("\n");
  }
  //Print R
  printf("#R\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",R(m,n));
    }
    printf("\n");
  }
}

void output_data(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6lu,%6lu,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         //"%13.8f"
         "\n"
            ,Elapsed_time//1
            ,Xe(0,0), Xe(1,0), Xe(2,0), Xe(3,0)//2~5 
            ,Xe(4,0), Xe(5,0), Xe(6,0)//6~8
            //,Phi-Phi_bias, Theta-Theta_bias, Psi-Psi_bias//6~8
            ,D_time, D_time2//10,11
            ,Ax, Ay, Az//11~13
            ,Wp, Wq, Wr//14~16
            ,Mx, My, Mz//17~19
            //,mag_norm
        ); //20
}
void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n"
            ,Elapsed_time//1
            ,Ax, Ay, Az//2~4
            ,Wp, Wq, Wr//5~7
            ,Mx, My, Mz//8~10
        ); //20
}

void kalman_filter(void)
{
  //Kalman Filter
  float dt=0.01;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G*dt, Beta, dt);
}


PID::PID()
{
  m_kp=1.0e-8;
  m_ti=1.0e8;
  m_td=0.0;
  m_integral=0.0;
  m_filter_time_constant=0.01;
  m_filter_output=0.0;
  m_err=0.0;
  m_h=0.01;
}

void PID::set_parameter(
    float kp, 
    float ti, 
    float td,
    float filter_time_constant, 
    float h)
{
  m_kp=kp;
  m_ti=ti;
  m_td=td;
  m_filter_time_constant=filter_time_constant;
  m_h=h;
}

void PID::reset(void)
{
  m_integral=0.0;
  m_filter_output=0.0;
  m_err=0.0;
  m_err2=0.0;
  m_err3=0.0;
}

void PID::i_reset(void)
{
  m_integral=0.0;
}
void PID::printGain(void)
{
  printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Filter T:%8.4f h:%8.4f\n",m_kp,m_ti,m_td,m_filter_time_constant,m_h);
}

float PID::filter(float x)
{
  m_filter_output = m_filter_output * m_filter_time_constant/(m_filter_time_constant + m_h) 
                  + x * m_h/(m_filter_time_constant + m_h);   
  return m_filter_output;
}

float PID::update(float err)
{
  float d;
  m_integral = m_integral + m_h * err;
  if(m_integral> 30000.0)m_integral = 30000.0;
  if(m_integral<-30000.0)m_integral =-30000.0;
  m_filter_output = filter((err-m_err3)/m_h);
  m_err3 = m_err2;
  m_err2 = m_err;
  m_err  = err;
  return m_kp*(err + m_integral/m_ti + m_td * m_filter_output); 
}

Filter::Filter()
{
  m_state = 0.0;
  m_T = 0.0025;
  m_h = 0.0025;
}

void Filter::reset(void)
{
  m_state = 0.0;
}

void Filter::set_parameter(float T, float h)
{
  m_T = T;
  m_h = h;
}

float Filter::update(float u)
{
  m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
  m_out = m_state;
  return m_out;
}
