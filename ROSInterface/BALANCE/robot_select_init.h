#ifndef __ROBOTSELECTINIT_H
#define __ROBOTSELECTINIT_H
#include "sys.h"
#include "system.h"

//Parameter structure of robot
//�����˲����ṹ��
typedef struct  
{
  float WheelSpacing;      //Wheelspacing, 4wd_Car is half wheelspacing //�־� ������Ϊ���־�
  float AxleSpacing;       //Axlespacing, 4wd_Car is half axlespacing //��� ������Ϊ�����	
  int GearRatio;           //Motor_gear_ratio //������ٱ�
  int EncoderAccuracy;     //Number_of_encoder_lines //����������(����������)
  float WheelDiameter;     //Diameter of driving wheel //������ֱ��	
  float OmniTurnRadiaus;   //Rotation radius of omnidirectional trolley //ȫ����С����ת�뾶
}Robot_Parament_InitTypeDef;


//half wheelspacing //���־�
#define SENIOR_4WD_BS_wheelspacing   0.176 
//#define SENIOR_4WD_DL_wheelspacing   0.247 
#define SENIOR_4WD_DL_wheelspacing   0.230
#define TOP_4WD_BS_wheelspacing      0.311
#define TOP_4WD_DL_wheelspacing      0.295
#define FlagShip_4WD_BS_wheelspacing 0.286
//#define FlagShip_4WD_DL_wheelspacing 0.298  ԭ�Ա�����
#define FlagShip_4WD_DL_wheelspacing 0.24

//half axlespacing //�����
//#define SENIOR_4WD_BS_axlespacing   0.156
#define SENIOR_4WD_BS_axlespacing   0.150
//#define SENIOR_4WD_DL_axlespacing   0.214 
#define SENIOR_4WD_DL_axlespacing   0.20 
#define TOP_4WD_BS_axlespacing      0.308
#define TOP_4WD_DL_axlespacing      0.201
#define FlagShip_4WD_BS_axlespacing 0.167
//#define FlagShip_4WD_DL_axlespacing 0.200   ԭ�Ա�����
#define FlagShip_4WD_DL_axlespacing 0.14

//Motor_gear_ratio
//������ٱ�
#define   MD36N_5_18  5.18
#define   MD36N_27    27
#define   MD36N_51    51
#define   MD36N_71    71
#define   MD60N_18    18
#define   MD60N_47    47

//Number_of_encoder_lines
//����������
#define		Photoelectric_500 500
#define	  Hall_13           13

//4WD wheel tire diameter series
//��������ֱ̥��
#define		_4WD_225  0.225f
#define		_4WD_250  0.250f
#define   _4WD_152  0.152f

 
//Omni wheel tire diameter series
//�־�ȫ����ֱ��ϵ��
#define	  FullDirecion_75  0.075
#define	  FullDirecion_127 0.127
#define	  FullDirecion_152 0.152
#define	  FullDirecion_203 0.203
#define	  FullDirecion_217 0.217

//Rotation radius of omnidirectional trolley
//ȫ����С����ת�뾶
#define   Omni_Turn_Radiaus_164 0.164
#define   Omni_Turn_Radiaus_180 0.160 //����Ϊ160
#define   Omni_Turn_Radiaus_290 0.280 //����Ϊ280

//The encoder octave depends on the encoder initialization Settings
//��������Ƶ����ȡ���ڱ�������ʼ������
#define   EncoderMultiples 4
//Encoder data reading frequency
//���������ݶ�ȡƵ��
#define CONTROL_FREQUENCY 100

//#define PI 3.1415f  //PI //Բ����

void Robot_Select(void);
#if _4WD
void Robot_Init(float wheelspacing, float axlespacing, int gearratio, int Accuracy, float tyre_diameter);
#elif Omni
void Robot_Init(float omni_turn_radiaus, int gearratio, int Accuracy, float tyre_diameter);
#endif

#endif
