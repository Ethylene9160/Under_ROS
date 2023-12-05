#include "balance.h"
#include"usartx.h"

//Whether the robot model is incorrectly marked
//�������ͺ��Ƿ�����־λ
int robot_mode_check_flag=0;
int A=1,B=1,C=1,DD=1; //Used for testing //���ڲ���
int Time_count=0; //Time variable //��ʱ����
u8 command_lost_count=0;//���ڡ�CAN�������ʧʱ���������ʧһ���ֹͣ����

//========== PWM���ʹ�ñ��� ==========//
u8 start_check_flag = 0;//����Ƿ���Ҫ���PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //��ǿ�ʼ���PWM
u8 clear_done_once = 0; //�����ɱ�־λ
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

//�Լ����
int check_a,check_b,check_c,check_d;
u8 check_end=0;
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
    static float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�
    //Ŀ���ٶ��޷�
    Vx=target_limit_float(Vx,-amplitude,amplitude);
    Vy=target_limit_float(Vy,-amplitude,amplitude);
    Vz=target_limit_float(Vz,-amplitude,amplitude);
    Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //�������ٶȽ���ƽ������

    //Get the smoothed data
    //��ȡƽ������������
    Vx=smooth_control.VX;
    Vy=smooth_control.VY;
    Vz=smooth_control.VZ;

#if _4WD //4WD wheel car //����С��

    //Inverse kinematics //�˶�ѧ���
    MOTOR_A.Target = +0+Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_B.Target = -0+Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_C.Target = +0+Vx+Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_D.Target = -0+Vx+Vz*(Wheel_axlespacing+Wheel_spacing);

    //Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
    MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
    MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
    MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
    MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude);

#elif Omni //Omni wheel Car //ȫ���ֳ�

    //Inverse kinematics //�˶�ѧ���
    MOTOR_A.Target =  Vy + Omni_turn_radiaus*Vz;
    MOTOR_B.Target = -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
    MOTOR_C.Target = +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;

    //Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
    MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
    MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
    MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
    MOTOR_D.Target=0;	//Out of use //û��ʹ�õ�

#endif
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{
    static int my_counter = 0;
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        // This task runs at a frequency of 100Hz (10ms control once)
        //��������100Hz��Ƶ�����У�10ms����һ�Σ�
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        //Time count is no longer needed after 30 seconds
        //ʱ�������30�������Ҫ
        if(Time_count<3000)Time_count++;

        //Get the encoder data, that is, the real time wheel speed,
        //and convert to transposition international units
        //��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
        Get_Velocity_Form_Encoder();

		//Do not enter the control before the end of self-check to prevent the PID control from starting integration
		//�Լ����ǰ��������ƣ���ֹPID���ƿ�ʼ����
		if(Time_count>CONTROL_DELAY+380) 
		{
//				command_lost_count++;//���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
//				if(command_lost_count>RATE_100_HZ&&APP_ON_Flag==0&&Remote_ON_Flag==0&&PS2_ON_Flag==0)
//					Move_X=0,Move_Y=0,Move_Z=0;
			
			if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //����APPң������
			else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //����ģң������
			else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //����PS2�ֱ���������
			
			//CAN, Usart 1, Usart 3 control can directly get the 2 axis target speed, 
			//without additional processing
			//CAN������1������3(ROS)����ֱ�ӵõ�2��Ŀ���ٶȣ�������⴦��
			else                      Drive_Motor(Move_X,0,Move_Z);  //CAN������1������3(ROS)����
		}

        Key();
		
        //�������ǳ�ʼ����ɺ�,���������ͺ��Ƿ�ѡ�����
        //When the gyroscope is initialized, check whether the robot model is selected incorrectly
		robot_slefcheck();//���A\B����C\D�����Դ�ӷ������߱������߽ӷ���������ж�
        if(CONTROL_DELAY<Time_count && Time_count<CONTROL_DELAY+350) //Advance 1 seconds to test //ǰ��1����в���
        {
			if( Time_count>CONTROL_DELAY+200) Drive_Motor(0,0,0);
			else  Drive_Motor(0.15f,0, 0);
            robot_mode_check(); //Detection function //��⺯��
        }
        else if(CONTROL_DELAY+350<Time_count && Time_count<CONTROL_DELAY+380) 
		{

			check_end=1;
			Drive_Motor(0,0,0); //The stop forward control is completed //������ֹͣǰ������
		}
		
        //If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0, or the model detection marker is 0
        //�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ���������ʧ�ܱ�־λΪ0�������ͺż���־λΪ0
        OLED_ShowNumber(20,10,Turn_Off(Voltage),5,12);
        OLED_ShowNumber(20,20,robot_mode_check_flag,5,12);
        //robot_mode_check_flag=0;
        if(Turn_Off(Voltage)==0&&robot_mode_check_flag==0)
        {
            //Speed closed-loop control to calculate the PWM value of each motor,
            //PWM represents the actual wheel speed
            //�ٶȱջ����Ƽ�������PWMֵ��PWM������ʵ��ת��
            MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
            MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
            MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
            MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
			
			//����Ƿ���Ҫ���PWM���Զ�ִ������
			auto_pwm_clear();

#if _4WD
            //Set different PWM control polarity according to different car models
            //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
            // my_counter += 1;
            if (Car_Mode==0||Car_Mode==1||Car_Mode==2||Car_Mode==3)
                Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm);  //MD36���ϵ��
            else
                Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,-MOTOR_C.Motor_Pwm,-MOTOR_D.Motor_Pwm);  //MD60���ϵ��
            // OLED_ShowNumber(20,30,my_counter,5,12);



#elif Omni
            //Set different PWM control polarity according to different car models
            //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
            if (Car_Mode==0)                           Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm,0); //����ȫ���������μ���      SENIOR_OMNI MD36N_5_18
            else if (Car_Mode==1||Car_Mode==2)                        Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm,0); //����ȫ����Բ��/�����γ��� SENIOR_OMNI MD36N_27
            else if (Car_Mode==3)	                          Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm,0); //����ȫ����Բ������        SENIOR_OMNI MD36N_51
            else if (Car_Mode==4||Car_Mode==5||Car_Mode==6)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,-MOTOR_C.Motor_Pwm,0); //����ȫ��������            TOP_OMNI    MD60N_*
#endif
        }
        //If Turn_Off(Voltage) returns to 1, or the model detection marker is 1, the car is not allowed to move, and the PWM value is set to 0
        //���Turn_Off(Voltage)����ֵΪ1�������ͺż���־λΪ1�����������С�������˶���PWMֵ����Ϊ0
        else	Set_Pwm(0,0,0,0);
    }
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
@Param  : motor_a, motor_b, motor_c, motor_d
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
    //Forward and reverse control of motor
    //�������ת����
    if(motor_a>0)			AIN1=0,		AIN2=1;
    else 	            AIN1=1,		AIN2=0;
    //Motor speed control
    //���ת�ٿ���
    // OLED_ShowNumber(20,30,motor_a*A,5,12);
    // TIM_SetCompare4(TIM8,2000);
    // return;
    TIM_SetCompare4(TIM8,myabs(motor_a*A));

    if(motor_b>0)			BIN1=0,		BIN2=1;
    else 	            BIN1=1,			BIN2=0;
    //Motor speed control
    //���ת�ٿ���
    TIM_SetCompare3(TIM8,myabs(motor_b*B));

    //Forward and reverse control of motor
    //�������ת����
    if(motor_c>0)			CIN2=0,		CIN1=1;
    else 	            CIN2=1,			CIN1=0;
    //Motor speed control
    //���ת�ٿ���
    TIM_SetCompare2(TIM8,myabs(motor_c*C));

    if(motor_d>0)			DIN2=0,		DIN1=1;
    else 	           DIN2=1,			DIN1=0;
    //Motor speed control
    //���ת�ٿ���
    TIM_SetCompare1(TIM8,myabs(motor_d*DD));
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{
    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
    MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬�����ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ�������ƣ�1��������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
    //return 0;
    //return Flag_Stop;
    u8 temp;
    //static int stop_count, enable_count;

    if(voltage < MIN_POWER_VOLTAGE||EN==0||Flag_Stop==1)
    // if(0||EN==0||Flag_Stop==1)
    {
        temp=1;
        PWMA=0;
        PWMB=0;
        PWMC=0;
        PWMD=0;
        AIN1=0;
        AIN2=0;
        BIN1=0;
        BIN2=0;
        CIN1=0;
        CIN2=0;
        DIN1=0;
        DIN2=0;
    }
    else
        temp=0;
    return temp;
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{
    u32 temp;
    if(a<0)  temp=-a;
    else temp=a;
    return temp;
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
    if(insert>=0) return insert;
    else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ��
e(k-1)������һ�ε�ƫ��  �Դ�����
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation //����ƫ��
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;
    Last_bias=Bias; //Save the last deviation //������һ��ƫ��
	
	//���PWM��־λ����λΪ1ʱ������Ҫ���PWM
	if( start_clear ) 
	{
		//PWM�𽥵ݼ��ķ�ʽ���������С�����ڵ���ͷŶ������΢�ƶ���Ӱ��
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//�������ϣ����Ǳ�־λ��4������ֱ���4��bit��ʾ
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
    return Pwm;
}
int Incremental_PI_B (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation //����ƫ��
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;
    Last_bias=Bias; //Save the last deviation //������һ��ƫ��
	
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<1;
		else clear_state &= ~(1<<1);
	}
	
    return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation //����ƫ��
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;
    Last_bias=Bias; //Save the last deviation //������һ��ƫ��

	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<2;
		else clear_state &= ~(1<<2);
	}
	
    return Pwm;
}
int Incremental_PI_D (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation //����ƫ��
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    // if(Pwm>16800)Pwm=16800;
    // if(Pwm<-16800)Pwm=-16800;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;

    Last_bias=Bias; //Save the last deviation //������һ��ƫ��
	
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//4������������ϣ���ر��������
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	
    return Pwm;
}

/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
    switch(Flag_Direction) //Handle direction control commands //�������������
    {
    case 1:
        Move_X=RC_Velocity;
        Move_Z=0;
        break;
    case 2:
        Move_X=RC_Velocity;
        Move_Z=-PI/4;
        break;
    case 3:
        Move_X=0;
        Move_Z=-PI/4;
        break;
    case 4:
        Move_X=-RC_Velocity;
        Move_Z=-PI/4;
        break;
    case 5:
        Move_X=-RC_Velocity;
        Move_Z=0;
        break;
    case 6:
        Move_X=-RC_Velocity;
        Move_Z=PI/4;
        break;
    case 7:
        Move_X=0;
        Move_Z=PI/4;
        break;
    case 8:
        Move_X=RC_Velocity;
        Move_Z=PI/4;
        break;
    default:
        Move_X=0;
        Move_Z=0;
        break;
    }
  

    if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //����ת
    else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //����ת

#if _4WD
    if(Move_X<0) Move_Z = -Move_Z;
#endif

    //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s
    Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;
    Move_Z=Move_Z;

    //Control target value is obtained and kinematics analysis is performed
    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(Move_X,Move_Y,Move_Z);

}
/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
    int LX,LY,RY;
    int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���

    //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
    //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
    LY=-(PS2_LX-128);
    LX=-(PS2_LY-128);
    RY=-(PS2_RX-128);

    //Ignore small movements of the joystick //����ҡ��С���ȶ���
    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;
    if(LX==0) Move_X=Move_X/1.2f;
    if(RY==0) Move_Z=Move_Z/1.2f;

    if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//����
    else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //����

    if(RC_Velocity<0)   RC_Velocity=0;

    //Handle PS2 controller control commands
    //��PS2�ֱ�����������д���
    Move_X=LX;
    Move_Y=LY;
    Move_Z=RY;
    Move_X=Move_X*RC_Velocity/128;
    Move_Y=Move_Y*RC_Velocity/128;
    Move_Z=Move_Z*(PI/4)*(RC_Velocity/500)/128;

    //Z������ת��
#if _4WD
    if(Move_X<0) Move_Z = -Move_Z;
    Move_Y = 0;
#endif

    //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s
    Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;

    //Control target value is obtained and kinematics analysis is performed
    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
    //Data within 1 second after entering the model control mode will not be processed
    //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=100;
    int Threshold=100;

    //limiter //�޷�
    int LX,LY,RY,RX,Remote_RCvelocity;
    static float Target_LX,Target_LY,Target_RY;
    Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
    Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
    Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
    Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

    // Front and back direction of left rocker. Control forward and backward.
    //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX=Remoter_Ch2-1500;

//		//Left joystick left and right. Control left and right movement.
//	  //��ҡ�����ҷ��򡣿��������ƶ���
//    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
    //��ҡ��ǰ��������/�Ӽ��١�
    RX=Remoter_Ch3-1500;		//

    //Right stick left and right. To control the rotation.
    //��ҡ�����ҷ��򡣿�����ת��
    RY=Remoter_Ch1-1500;

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;


    if(LX==0) Target_LX=Target_LX/1.2f;
    if(LY==0) Target_LY=Target_LY/1.2f;
    if(RY==0) Target_RY=Target_RY/1.2f;


    //Throttle related //�������
    Remote_RCvelocity=RC_Velocity+RX;
    if(Remote_RCvelocity<0)Remote_RCvelocity=0;

    //The remote control command of model aircraft is processed
    //�Ժ�ģң�ؿ���������д���
    Move_X= LX;
    Move_Y=-LY;
    Move_Z=-RY;
    Move_X= Move_X*Remote_RCvelocity/500;
    Move_Y= Move_Y*Remote_RCvelocity/500;
    Move_Z= Move_Z*(PI/4)/500;

    //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s
    Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;

    //Z������ת��
#if _4WD
    if(Move_X<0) Move_Z = -Move_Z;
    Move_Y = 0;
#endif

    //Data within 1 second after entering the model control mode will not be processed
    //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;

    //Control target value is obtained and kinematics analysis is performed
    //�õ�����Ŀ��ֵ�������˶�ѧ����
    Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
    u8 tmp;
    tmp=click();
    if(tmp==1)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro)),memcpy(Deviation_accel,Original_accel,sizeof(accel));
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
    float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; //���ڻ�ȡ��������ԭʼ����

    //Obtain the original data of the encoder, and the polarity of different models of cars is also different
    //��ȡ��������ԭʼ���ݣ�ͬʱ��ͬ�ͺŵ�С������Ҳ����ͬ
#if _4WD
    Encoder_A_pr= Read_Encoder(2);
    Encoder_B_pr= Read_Encoder(3);
    Encoder_C_pr=-Read_Encoder(4);
    Encoder_D_pr=-Read_Encoder(5);
#elif Omni
    Encoder_A_pr=-Read_Encoder(2);
    Encoder_B_pr=-Read_Encoder(3);
    Encoder_C_pr=-Read_Encoder(4);
#endif

	//δ����Լ�ʱ�ռ�����������
	if( check_end==0 )
	{
		check_a+=Encoder_A_pr;
		check_b+=Encoder_B_pr;
		check_c+=Encoder_C_pr;
		check_d+=Encoder_D_pr;
	}
	
    //The encoder converts the raw data to wheel speed in m/s
    //������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
    MOTOR_A.Encoder = Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
    MOTOR_B.Encoder = Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
    MOTOR_C.Encoder = Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
    MOTOR_D.Encoder = Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.02; //ƽ��������ֵ
	
	//X���ٶ�ƽ��
	if(vx>smooth_control.VX)
	{
		smooth_control.VX+=step;
		if(smooth_control.VX>vx) smooth_control.VX=vx;
	}
	else if (vx<smooth_control.VX)
	{
		smooth_control.VX-=step;
		if(smooth_control.VX<vx) smooth_control.VX=vx;
	}
	else
		 smooth_control.VX =vx;
	
	//Y���ٶ�ƽ��
	if(vy>smooth_control.VY)
	{
		smooth_control.VY+=step;
		if(smooth_control.VY>vy) smooth_control.VY=vy;
	}
	else if (vy<smooth_control.VY)
	{
		smooth_control.VY-=step;
		if(smooth_control.VY<vy) smooth_control.VY=vy;
	}
	else
		 smooth_control.VY =vy;
	
	//Z���ٶ�ƽ��
	if(vz>smooth_control.VZ)
	{
		smooth_control.VZ+=step;
		if(smooth_control.VZ>vz) smooth_control.VZ=vz;
	}
	else if (vz<smooth_control.VZ)
	{
		smooth_control.VZ-=step;
		if(smooth_control.VZ<vz) smooth_control.VZ=vz;
	}
	else
		 smooth_control.VZ =vz;
	
	//0��ʱ��֤��ֹ�ȶ�
	if(vx==0&&smooth_control.VX<0.05f&&smooth_control.VX>-0.05f) smooth_control.VX=0;
	if(vy==0&&smooth_control.VY<0.05f&&smooth_control.VY>-0.05f) smooth_control.VY=0;
	if(vz==0&&smooth_control.VZ<0.05f&&smooth_control.VZ>-0.05f) smooth_control.VZ=0;
}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.
Input   : none
Output  : none
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	#define ERROR_PWM 7000 //pwmԤ��ֵ
	static u8 once=1;
	static int last_a,last_b,last_c,last_d;
	static u8 times;
	
	if( once ) check_a=0,check_b=0,check_c=0,check_d=0,once=0;
	
	if( EN==1 && robot_mode_check_flag==0) //��������ʹ�ü�ͣ���������Լ�Ĺ���
	{
		if(Time_count<CONTROL_DELAY+200) //С��ֹͣ�ڼ䲻����������ݣ���ֹ��Ϊ����
		{
			//�ۼ�ֵ�����˵��ˣ�˵��������Ƴ����쳣��ͨ������ǵ��������Դ������������Ӧ+���������߲���Ӧ ���ִ�����ӵĽ��
			if( check_a<last_a-1000 || check_b<last_b-1000 || check_c < last_c-1000 || check_d <last_d-1000 )	
			{
                
				times++;
				if( times>2 ) robot_mode_check_flag=1,LED_G=0;
			}
			last_a = check_a,last_b = check_b,last_c = check_c,last_d = check_d;
			
			//�����ڸ�����˵�����ڷ����෴���������������Ϊ������ѡ���������ߴ������������ߴ���
            //to check
			//if( check_a<-3000 ||check_b<-3000 ||check_c<-3000 ||check_d<-3000 ) robot_mode_check_flag=1,LED_G=0;
		}

		//ӵ��һ��pwm����ֵ�������������ݲ��䡣��������Ϊ��������δ���ߡ�����δ���߻�����
		// if(float_abs(MOTOR_A.Motor_Pwm)>5500 && check_a<500) robot_mode_check_flag=1,LED_B=0;	
		// if(float_abs(MOTOR_B.Motor_Pwm)>5500 && check_b<500) robot_mode_check_flag=1,LED_B=0;	
		// if(float_abs(MOTOR_C.Motor_Pwm)>5500 && check_c<500) robot_mode_check_flag=1,LED_B=0;
		// if(float_abs(MOTOR_D.Motor_Pwm)>5500 && check_d<500) robot_mode_check_flag=1,LED_B=0;

        
		if(float_abs(MOTOR_A.Motor_Pwm)>ERROR_PWM && check_a<500) robot_mode_check_flag=1,LED_B=0;	
		if(float_abs(MOTOR_B.Motor_Pwm)>ERROR_PWM && check_b<500) robot_mode_check_flag=1,LED_B=0;	
		if(float_abs(MOTOR_C.Motor_Pwm)>ERROR_PWM && check_c<500) robot_mode_check_flag=1,LED_B=0;
		if(float_abs(MOTOR_D.Motor_Pwm)>ERROR_PWM && check_d<500) robot_mode_check_flag=1,LED_B=0;
		
		//�����ߣ�����0.1m/s�ٶ��޷������PWM��ֵ���������ͣ�������A��B�ӷ�����C��D�ӷ������߸����Ѿ���������ܳ��ܵķ�Χ
		if( float_abs(MOTOR_A.Motor_Pwm)>ERROR_PWM||float_abs(MOTOR_B.Motor_Pwm)>ERROR_PWM||\
			 float_abs(MOTOR_C.Motor_Pwm)>ERROR_PWM||float_abs(MOTOR_D.Motor_Pwm)>ERROR_PWM )
		{
			robot_mode_check_flag = 1;
			LED_B=1,LED_G=1;
		}
		
	}
}

void robot_slefcheck(void)
{
	if( smooth_control.VX==0&&smooth_control.VZ==0 )
	{
		if( MOTOR_A.Motor_Pwm> MAX_PWM_VALUE && MOTOR_B.Motor_Pwm<-MAX_PWM_VALUE ||\
            MOTOR_A.Motor_Pwm<-MAX_PWM_VALUE && MOTOR_B.Motor_Pwm> MAX_PWM_VALUE ||\
			MOTOR_C.Motor_Pwm> MAX_PWM_VALUE && MOTOR_D.Motor_Pwm<-MAX_PWM_VALUE ||\
            MOTOR_C.Motor_Pwm<-MAX_PWM_VALUE && MOTOR_D.Motor_Pwm> MAX_PWM_VALUE  )
		{
			robot_mode_check_flag = 1;
		}
	}
}

//PWM��������
void auto_pwm_clear(void)
{
	//С����̬�����ж�
	float y_accle = (float)(accel[1]/1671.84f);//Y����ٶ�ʵ��ֵ
	float z_accle = (float)(accel[2]/1671.84f);//Z����ٶ�ʵ��ֵ
	float diff;
	
	//����Y��Z���ٶ��ں�ֵ����ֵԽ�ӽ�9.8����ʾС����̬Խˮƽ
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM�������
	if( smooth_control.VX !=0.0f || smooth_control.VZ != 0.0f)
	{
		start_check_flag = 1;//�����Ҫ���PWM
		wait_clear_times = 0;//��λ��ռ�ʱ
		start_clear = 0;     //��λ�����־
		
		
		//�˶�ʱб�¼������ݸ�λ
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //��Ŀ���ٶ��ɷ�0��0ʱ����ʼ��ʱ 2.5 �룬��С������б��״̬�£����pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//С����ˮƽ����ʱ�ű�����pwm����ֹС����б�����˶���������
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//�������pwm
				else clear_done_once = 1;//С����б���ϣ������������
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//�����������������Ƴ���Ϊ��pwm����һ����ֵ����10����ٴ����
	if( clear_done_once )
	{
		//С���ӽ���ˮƽ��ʱ����������������ֹС����б�����ﳵ
		if( diff > 8.8f )
		{
			//��������pwm�ٴλ��ۣ��������
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//�������pwm
					clear_state = 0;
				}
			}
			else
			{
				clear_again_times = 0;
			}
		}
		else
		{
			clear_again_times = 0;
		}

	}
}


