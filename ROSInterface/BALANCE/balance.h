#ifndef __BALANCE_H
#define __BALANCE_H		

#define MIN_POWER_VOLTAGE 7	  
#define MAX_PWN_VALUE 5500

#include "sys.h"
#include "system.h"
#define BALANCE_TASK_PRIO		4     //Task priority //�������ȼ�
#define BALANCE_STK_SIZE 		512   //Task stack size //�����ջ��С

//Parameter of kinematics analysis of omnidirectional trolley
//ȫ����С���˶�ѧ��������
#define X_PARAMETER    (sqrt(3)/2.f)               
#define Y_PARAMETER    (0.5f)    
#define L_PARAMETER    (1.0f)



extern int A,B,C,DD;
extern u8 command_lost_count;//���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
void Balance_task(void *pvParameters);
void Limit_Pwm(int amplitude);
float target_limit_float(float insert,float low,float high);
int target_limit_int(int insert,int low,int high);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
float float_abs(float insert);
int Incremental_PI_A (float Encoder,float Target);
int Incremental_PI_B (float Encoder,float Target);
int Incremental_PI_C (float Encoder,float Target);
int Incremental_PI_D (float Encoder,float Target);

void Get_RC(void);
void PS2_control(void);
void Remote_Control(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
void Drive_Motor(float Vx,float Vy,float Vz);
void Key(void);
void Get_Velocity_Form_Encoder(void);
void robot_mode_check(void);
void Smooth_control(float vx,float vy,float vz);
void auto_pwm_clear(void);
void robot_slefcheck(void);
// #define getAbs(x) _Generic((x),\
//     int:int_abs(x,y),\
//     float:float_abs(x,y),\
//     u32:myabs(x),\

//     default:unsupport())
#endif  

