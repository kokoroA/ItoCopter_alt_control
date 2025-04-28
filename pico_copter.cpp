#include "pico_copter.hpp"

//グローバル変数
uint8_t Arm_flag=0;
semaphore_t sem;
float budget_s;
float budget_e;
float diff;
float diff_sum;
float start_t = time_us_64();
float current_t = start_t;

int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  //Initialize stdio for Pico
  stdio_init_all();

  //Initialize LSM9DS1
  imu_mag_init();

  //Initialize Radio
  radio_init();

  //Initialize Variavle
  variable_init();

  //Initilize Control
  control_init();
  Kalman_init();
  initialize_Altitude();

  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib=0;
  pwm_init();
  
  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
 
  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  Arm_flag=1;
  
  while(1)
  {
    //printf("Arm_flag:%d LockMode:%d\n",Arm_flag, LockMode);
    tight_loop_contents();

    while (Logoutputflag==1){
      log_output();
    }

    // start_time = start_time + 1;


    // budget_s = time_us_64();
    // if (diff_sum >= 5000)
    // {
    //   // start_time = 0;
    //   diff_sum = 0;
    //   // printf("%9.6f %4d %9.6f %9.6f\n",(current_t-start_t)/1000000.0,distance,lotated_distance,mu_Yn_est(1,0));
    //   //printf("distance : %4d\n",distance);
    //   // printf("ideal : %9.6f\n",ideal);
    //   //printf("lotated_distance : %9.6f\n",lotated_distance);
    //   //printf("Altitude  : %9.6f\n",mu_Yn_est(1,0));
    //   // printf("velocity  : %9.6f\n",mu_Yn_est(0,0));
    //   //printf("%9.6f %4d \n",(current_t-start_t)/1000000.0,distance);
    //   // printf("Ax  : %9.6f\n",z_acc+ 9.80665);
    //   // printf("z_acc  : %9.6f\n",z_acc);
    //   // printf("%9.6f %9.6f\n",(current_t-start_t)/1000000.0,input);
    //   // if (print_flag == 1){
    //   //   printf("%9.6f %s\n",(current_t-start_t)/1000000.0,buffer);
    //   // }
    //   //printf("T_ref  : %9.6f\n",T_ref);
    //   // printf("func_time %9.6f \n",func_time);
    //   // printf("count %llu \n", count_up);
    //   current_t = time_us_64();
    // }
    // else{
    //   budget_e = time_us_64();
    //   diff = budget_e - budget_s;
    //   diff_sum = diff_sum + diff;
    // }

  }

  return 0;
}
