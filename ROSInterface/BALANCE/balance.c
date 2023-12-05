#include "balance.h"
#include"usartx.h"

//Whether the robot model is incorrectly marked
//机器人型号是否错误标志位
int robot_mode_check_flag=0;
int A=1,B=1,C=1,DD=1; //Used for testing //用于测试
int Time_count=0; //Time variable //计时变量
u8 command_lost_count=0;//串口、CAN控制命令丢失时间计数，丢失一秒后停止控制

//========== PWM清除使用变量 ==========//
u8 start_check_flag = 0;//标记是否需要清空PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //标记开始清除PWM
u8 clear_done_once = 0; //清除完成标志位
u16 clear_again_times = 0;
float debug_show_diff = 0;
void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/

//自检参数
int check_a,check_b,check_c,check_d;
u8 check_end=0;
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
    static float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
    //目标速度限幅
    Vx=target_limit_float(Vx,-amplitude,amplitude);
    Vy=target_limit_float(Vy,-amplitude,amplitude);
    Vz=target_limit_float(Vz,-amplitude,amplitude);
    Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理

    //Get the smoothed data
    //获取平滑处理后的数据
    Vx=smooth_control.VX;
    Vy=smooth_control.VY;
    Vz=smooth_control.VZ;

#if _4WD //4WD wheel car //四驱小车

    //Inverse kinematics //运动学逆解
    MOTOR_A.Target = +0+Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_B.Target = -0+Vx-Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_C.Target = +0+Vx+Vz*(Wheel_axlespacing+Wheel_spacing);
    MOTOR_D.Target = -0+Vx+Vz*(Wheel_axlespacing+Wheel_spacing);

    //Wheel (motor) target speed limit //车轮(电机)目标速度限幅
    MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
    MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
    MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
    MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude);

#elif Omni //Omni wheel Car //全向轮车

    //Inverse kinematics //运动学逆解
    MOTOR_A.Target =  Vy + Omni_turn_radiaus*Vz;
    MOTOR_B.Target = -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
    MOTOR_C.Target = +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;

    //Wheel (motor) target speed limit //车轮(电机)目标速度限幅
    MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude);
    MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude);
    MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude);
    MOTOR_D.Target=0;	//Out of use //没有使用到

#endif
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{
    static int my_counter = 0;
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        // This task runs at a frequency of 100Hz (10ms control once)
        //此任务以100Hz的频率运行（10ms控制一次）
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

        //Time count is no longer needed after 30 seconds
        //时间计数，30秒后不再需要
        if(Time_count<3000)Time_count++;

        //Get the encoder data, that is, the real time wheel speed,
        //and convert to transposition international units
        //获取编码器数据，即车轮实时速度，并转换位国际单位
        Get_Velocity_Form_Encoder();

		//Do not enter the control before the end of self-check to prevent the PID control from starting integration
		//自检结束前不进入控制，防止PID控制开始积分
		if(Time_count>CONTROL_DELAY+380) 
		{
//				command_lost_count++;//串口、CAN控制命令丢失时间计数，丢失1秒后停止控制
//				if(command_lost_count>RATE_100_HZ&&APP_ON_Flag==0&&Remote_ON_Flag==0&&PS2_ON_Flag==0)
//					Move_X=0,Move_Y=0,Move_Z=0;
			
			if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控命令
			else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控命令
			else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄控制命令
			
			//CAN, Usart 1, Usart 3 control can directly get the 2 axis target speed, 
			//without additional processing
			//CAN、串口1、串口3(ROS)控制直接得到2轴目标速度，无须额外处理
			else                      Drive_Motor(Move_X,0,Move_Z);  //CAN、串口1、串口3(ROS)控制
		}

        Key();
		
        //等陀螺仪初始化完成后,检测机器人型号是否选择错误
        //When the gyroscope is initialized, check whether the robot model is selected incorrectly
		robot_slefcheck();//针对A\B或者C\D电机电源接反、或者编码器线接反现象进行判断
        if(CONTROL_DELAY<Time_count && Time_count<CONTROL_DELAY+350) //Advance 1 seconds to test //前进1秒进行测试
        {
			if( Time_count>CONTROL_DELAY+200) Drive_Motor(0,0,0);
			else  Drive_Motor(0.15f,0, 0);
            robot_mode_check(); //Detection function //检测函数
        }
        else if(CONTROL_DELAY+350<Time_count && Time_count<CONTROL_DELAY+380) 
		{

			check_end=1;
			Drive_Motor(0,0,0); //The stop forward control is completed //检测完成停止前进控制
		}
		
        //If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0, or the model detection marker is 0
        //如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0，或者型号检测标志位为0
        OLED_ShowNumber(20,10,Turn_Off(Voltage),5,12);
        OLED_ShowNumber(20,20,robot_mode_check_flag,5,12);
        //robot_mode_check_flag=0;
        if(Turn_Off(Voltage)==0&&robot_mode_check_flag==0)
        {
            //Speed closed-loop control to calculate the PWM value of each motor,
            //PWM represents the actual wheel speed
            //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
            MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
            MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
            MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
            MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
			
			//检测是否需要清除PWM并自动执行清理
			auto_pwm_clear();

#if _4WD
            //Set different PWM control polarity according to different car models
            //根据不同小车型号设置不同的PWM控制极性
            // my_counter += 1;
            if (Car_Mode==0||Car_Mode==1||Car_Mode==2||Car_Mode==3)
                Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm);  //MD36电机系列
            else
                Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm,-MOTOR_C.Motor_Pwm,-MOTOR_D.Motor_Pwm);  //MD60电机系列
            // OLED_ShowNumber(20,30,my_counter,5,12);



#elif Omni
            //Set different PWM control polarity according to different car models
            //根据不同小车型号设置不同的PWM控制极性
            if (Car_Mode==0)                           Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm,0); //高配全向轮三角形极速      SENIOR_OMNI MD36N_5_18
            else if (Car_Mode==1||Car_Mode==2)                        Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm,0); //高配全向轮圆形/三角形常规 SENIOR_OMNI MD36N_27
            else if (Car_Mode==3)	                          Set_Pwm(-MOTOR_A.Motor_Pwm,-MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm,0); //高配全向轮圆形重载        SENIOR_OMNI MD36N_51
            else if (Car_Mode==4||Car_Mode==5||Car_Mode==6)	Set_Pwm( MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,-MOTOR_C.Motor_Pwm,0); //顶配全向轮重载            TOP_OMNI    MD60N_*
#endif
        }
        //If Turn_Off(Voltage) returns to 1, or the model detection marker is 1, the car is not allowed to move, and the PWM value is set to 0
        //如果Turn_Off(Voltage)返回值为1，或者型号检测标志位为1，不允许控制小车进行运动，PWM值设置为0
        else	Set_Pwm(0,0,0,0);
    }
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
@Param  : motor_a, motor_b, motor_c, motor_d
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
    //Forward and reverse control of motor
    //电机正反转控制
    if(motor_a>0)			AIN1=0,		AIN2=1;
    else 	            AIN1=1,		AIN2=0;
    //Motor speed control
    //电机转速控制
    // OLED_ShowNumber(20,30,motor_a*A,5,12);
    // TIM_SetCompare4(TIM8,2000);
    // return;
    TIM_SetCompare4(TIM8,myabs(motor_a*A));

    if(motor_b>0)			BIN1=0,		BIN2=1;
    else 	            BIN1=1,			BIN2=0;
    //Motor speed control
    //电机转速控制
    TIM_SetCompare3(TIM8,myabs(motor_b*B));

    //Forward and reverse control of motor
    //电机正反转控制
    if(motor_c>0)			CIN2=0,		CIN1=1;
    else 	            CIN2=1,			CIN1=0;
    //Motor speed control
    //电机转速控制
    TIM_SetCompare2(TIM8,myabs(motor_c*C));

    if(motor_d>0)			DIN2=0,		DIN1=1;
    else 	           DIN2=1,			DIN1=0;
    //Motor speed control
    //电机转速控制
    TIM_SetCompare1(TIM8,myabs(motor_d*DD));
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值
入口参数：幅值
返回  值：无
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
函数功能：限幅函数
入口参数：幅值
返回  值：无
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
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
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
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
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
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
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
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;
    Last_bias=Bias; //Save the last deviation //保存上一次偏差
	
	//清除PWM标志位，该位为1时代表需要清除PWM
	if( start_clear ) 
	{
		//PWM逐渐递减的方式清除，减缓小车由于电机释放而造成轻微移动的影响
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		//若清除完毕，则标记标志位，4个电机分别用4个bit表示
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<0;
		else clear_state &= ~(1<<0);
	}
	
    return Pwm;
}
int Incremental_PI_B (float Encoder,float Target)
{
    static float Bias,Pwm,Last_bias;
    Bias=Target-Encoder; //Calculate the deviation //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;
    Last_bias=Bias; //Save the last deviation //保存上一次偏差
	
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
    Bias=Target-Encoder; //Calculate the deviation //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;
    Last_bias=Bias; //Save the last deviation //保存上一次偏差

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
    Bias=Target-Encoder; //Calculate the deviation //计算偏差
    Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;
    // if(Pwm>16800)Pwm=16800;
    // if(Pwm<-16800)Pwm=-16800;
    if(Pwm>MAX_PWM_VALUE)Pwm=MAX_PWM_VALUE;
    if(Pwm<-MAX_PWM_VALUE)Pwm=-MAX_PWM_VALUE;

    Last_bias=Bias; //Save the last deviation //保存上一次偏差
	
	if( start_clear ) 
	{
		if(Pwm>0) Pwm--;
		if(Pwm<0) Pwm++;
		
		if( Pwm<2.0f&&Pwm>-2.0f ) Pwm=0,clear_state |= 1<<3;
		else clear_state &= ~(1<<3);
		
		//4个电机均清除完毕，则关闭清除任务
		if( (clear_state&0xff)==0x0f ) start_clear = 0,clear_done_once=1,clear_state=0;
	}
	
    return Pwm;
}

/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
    switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
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
  

    if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //左自转
    else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //右自转

#if _4WD
    if(Move_X<0) Move_Z = -Move_Z;
#endif

    //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
    Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;
    Move_Z=Move_Z;

    //Control target value is obtained and kinematics analysis is performed
    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X,Move_Y,Move_Z);

}
/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
函数功能：对PS2手柄控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
    int LX,LY,RY;
    int Threshold=20; //Threshold to ignore small movements of the joystick //阈值，忽略摇杆小幅度动作

    //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
    //128为中值。PS2坐标系与ROS坐标系对X、Y的定义不一样
    LY=-(PS2_LX-128);
    LX=-(PS2_LY-128);
    RY=-(PS2_RX-128);

    //Ignore small movements of the joystick //忽略摇杆小幅度动作
    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;
    if(LX==0) Move_X=Move_X/1.2f;
    if(RY==0) Move_Z=Move_Z/1.2f;

    if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//加速
    else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //减速

    if(RC_Velocity<0)   RC_Velocity=0;

    //Handle PS2 controller control commands
    //对PS2手柄控制命令进行处理
    Move_X=LX;
    Move_Y=LY;
    Move_Z=RY;
    Move_X=Move_X*RC_Velocity/128;
    Move_Y=Move_Y*RC_Velocity/128;
    Move_Z=Move_Z*(PI/4)*(RC_Velocity/500)/128;

    //Z轴数据转化
#if _4WD
    if(Move_X<0) Move_Z = -Move_Z;
    Move_Y = 0;
#endif

    //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
    Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;

    //Control target value is obtained and kinematics analysis is performed
    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
函数功能：对航模遥控控制命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
    //Data within 1 second after entering the model control mode will not be processed
    //对进入航模控制模式后1秒内的数据不处理
    static u8 thrice=100;
    int Threshold=100;

    //limiter //限幅
    int LX,LY,RY,RX,Remote_RCvelocity;
    static float Target_LX,Target_LY,Target_RY;
    Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
    Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
    Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
    Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

    // Front and back direction of left rocker. Control forward and backward.
    //左摇杆前后方向。控制前进后退。
    LX=Remoter_Ch2-1500;

//		//Left joystick left and right. Control left and right movement.
//	  //左摇杆左右方向。控制左右移动。
//    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
    //右摇杆前后方向。油门/加减速。
    RX=Remoter_Ch3-1500;		//

    //Right stick left and right. To control the rotation.
    //右摇杆左右方向。控制自转。
    RY=Remoter_Ch1-1500;

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RY>-Threshold&&RY<Threshold)RY=0;


    if(LX==0) Target_LX=Target_LX/1.2f;
    if(LY==0) Target_LY=Target_LY/1.2f;
    if(RY==0) Target_RY=Target_RY/1.2f;


    //Throttle related //油门相关
    Remote_RCvelocity=RC_Velocity+RX;
    if(Remote_RCvelocity<0)Remote_RCvelocity=0;

    //The remote control command of model aircraft is processed
    //对航模遥控控制命令进行处理
    Move_X= LX;
    Move_Y=-LY;
    Move_Z=-RY;
    Move_X= Move_X*Remote_RCvelocity/500;
    Move_Y= Move_Y*Remote_RCvelocity/500;
    Move_Z= Move_Z*(PI/4)/500;

    //Unit conversion, mm/s -> m/s
    //单位转换，mm/s -> m/s
    Move_X=Move_X/1000;
    Move_Y=Move_Y/1000;

    //Z轴数据转化
#if _4WD
    if(Move_X<0) Move_Z = -Move_Z;
    Move_Y = 0;
#endif

    //Data within 1 second after entering the model control mode will not be processed
    //对进入航模控制模式后1秒内的数据不处理
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;

    //Control target value is obtained and kinematics analysis is performed
    //得到控制目标值，进行运动学分析
    Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
函数功能：单击用户按键更新陀螺仪零点
入口参数：无
返回  值：无
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
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
    float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; //用于获取编码器的原始数据

    //Obtain the original data of the encoder, and the polarity of different models of cars is also different
    //获取编码器的原始数据，同时不同型号的小车极性也不相同
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

	//未完成自检时收集编码器数据
	if( check_end==0 )
	{
		check_a+=Encoder_A_pr;
		check_b+=Encoder_B_pr;
		check_c+=Encoder_C_pr;
		check_d+=Encoder_D_pr;
	}
	
    //The encoder converts the raw data to wheel speed in m/s
    //编码器原始数据转换为车轮速度，单位m/s
    MOTOR_A.Encoder = Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
    MOTOR_B.Encoder = Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
    MOTOR_C.Encoder = Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
    MOTOR_D.Encoder = Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.02; //平滑处理步进值
	
	//X轴速度平滑
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
	
	//Y轴速度平滑
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
	
	//Z轴速度平滑
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
	
	//0速时保证静止稳定
	if(vx==0&&smooth_control.VX<0.05f&&smooth_control.VX>-0.05f) smooth_control.VX=0;
	if(vy==0&&smooth_control.VY<0.05f&&smooth_control.VY>-0.05f) smooth_control.VY=0;
	if(vz==0&&smooth_control.VZ<0.05f&&smooth_control.VZ>-0.05f) smooth_control.VZ=0;
}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.
Input   : none
Output  : none
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转。
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	#define ERROR_PWM 7000 //pwm预警值
	static u8 once=1;
	static int last_a,last_b,last_c,last_d;
	static u8 times;
	
	if( once ) check_a=0,check_b=0,check_c=0,check_d=0,once=0;
	
	if( EN==1 && robot_mode_check_flag==0) //保留可以使用急停开关跳过自检的功能
	{
		if(Time_count<CONTROL_DELAY+200) //小车停止期间不检测以下内容，防止人为干扰
		{
			//累计值出现了倒退，说明电机控制出现异常，通常情况是电机驱动电源接线与电机不对应+编码器接线不对应 两种错误叠加的结果
			if( check_a<last_a-1000 || check_b<last_b-1000 || check_c < last_c-1000 || check_d <last_d-1000 )	
			{
                
				times++;
				if( times>2 ) robot_mode_check_flag=1,LED_G=0;
			}
			last_a = check_a,last_b = check_b,last_c = check_c,last_d = check_d;
			
			//若存在负数，说明存在方向相反的情况：错误类型为：车型选错、驱动接线错误或编码器接线错误
            //to check
			//if( check_a<-3000 ||check_b<-3000 ||check_c<-3000 ||check_d<-3000 ) robot_mode_check_flag=1,LED_G=0;
		}

		//拥有一定pwm参数值后，若编码器数据不变。错误类型为：编码器未接线、驱动未接线或负责超重
		// if(float_abs(MOTOR_A.Motor_Pwm)>5500 && check_a<500) robot_mode_check_flag=1,LED_B=0;	
		// if(float_abs(MOTOR_B.Motor_Pwm)>5500 && check_b<500) robot_mode_check_flag=1,LED_B=0;	
		// if(float_abs(MOTOR_C.Motor_Pwm)>5500 && check_c<500) robot_mode_check_flag=1,LED_B=0;
		// if(float_abs(MOTOR_D.Motor_Pwm)>5500 && check_d<500) robot_mode_check_flag=1,LED_B=0;

        
		if(float_abs(MOTOR_A.Motor_Pwm)>ERROR_PWM && check_a<500) robot_mode_check_flag=1,LED_B=0;	
		if(float_abs(MOTOR_B.Motor_Pwm)>ERROR_PWM && check_b<500) robot_mode_check_flag=1,LED_B=0;	
		if(float_abs(MOTOR_C.Motor_Pwm)>ERROR_PWM && check_c<500) robot_mode_check_flag=1,LED_B=0;
		if(float_abs(MOTOR_D.Motor_Pwm)>ERROR_PWM && check_d<500) robot_mode_check_flag=1,LED_B=0;
		
		//最后防线，正常0.1m/s速度无法到达的PWM数值，错误类型：编码器A、B接反或者C、D接反、或者负载已经超出电机能承受的范围
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

//PWM消除函数
void auto_pwm_clear(void)
{
	//小车姿态简易判断
	float y_accle = (float)(accel[1]/1671.84f);//Y轴加速度实际值
	float z_accle = (float)(accel[2]/1671.84f);//Z轴加速度实际值
	float diff;
	
	//计算Y、Z加速度融合值，该值越接近9.8，表示小车姿态越水平
	if( y_accle > 0 ) diff  = z_accle - y_accle;
	else diff  = z_accle + y_accle;
	
//	debug_show_diff = diff;
	
	//PWM消除检测
	if( smooth_control.VX !=0.0f || smooth_control.VZ != 0.0f)
	{
		start_check_flag = 1;//标记需要清空PWM
		wait_clear_times = 0;//复位清空计时
		start_clear = 0;     //复位清除标志
		
		
		//运动时斜坡检测的数据复位
		clear_done_once = 0;
		clear_again_times=0;
	}
	else //当目标速度由非0变0时，开始计时 2.5 秒，若小车不在斜坡状态下，清空pwm
	{
		if( start_check_flag==1 )
		{
			wait_clear_times++;
			if( wait_clear_times >= 250 )
			{
				//小车在水平面上时才标记清空pwm，防止小车在斜坡上运动出现溜坡
				if( diff > 8.8f )	start_clear = 1,clear_state = 0;//开启清除pwm
				else clear_done_once = 1;//小车在斜坡上，标记已完成清除
				
				start_check_flag = 0;
			}
		}
		else
		{
			wait_clear_times = 0;
		}
	}

	//完成了清除后，若出现推车行为，pwm积累一定数值后将在10秒后再次清空
	if( clear_done_once )
	{
		//小车接近于水平面时才作积累消除，防止小车在斜坡上溜车
		if( diff > 8.8f )
		{
			//完成清除后pwm再次积累，重新清除
			if( int_abs(MOTOR_A.Motor_Pwm)>300 || int_abs(MOTOR_B.Motor_Pwm)>300 || int_abs(MOTOR_C.Motor_Pwm)>300 || int_abs(MOTOR_D.Motor_Pwm)>300 )
			{
				clear_again_times++;
				if( clear_again_times>1000 )
				{
					clear_done_once = 0;
					start_clear = 1;//开启清除pwm
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


