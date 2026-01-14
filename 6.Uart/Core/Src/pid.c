#include "pid.h"
#include "stm32f1xx_it.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"

//传感器数据变量
int Encoder_Left,Encoder_Right;
float pitch, yaw,roll;
short gyrox,gyroy,gyroz;
short aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,velocity_out,Turn_out,Target_speed,Target_Turn,MOTO1,MOTO2;
float Med_Angle=0;//平衡时角度值偏移量

float Vertical_kp=-300,Vertical_kd=-2;//直立环 数量级 （kp； 0~1000，kd；0~10）
float Velocity_kp=0.4,Velocity_ki;//速度环 数量级 （kp； 0~1）
float Turn_kp,Turn_kd;//转向环
uint8_t stop;
 
extern TIM_HandleTypeDef htim2,htim4;
extern uint8_t rx_buf[2],Bluetooth_date;
extern uint8_t  Fore,Back,Left,Right;
#define Speed_Y 12 //俯仰（前后）最大规定速度
#define Speed_Z 150//偏航（左右）最大规定速度

//直立PD控制器
//输入：期望角度，真实角度，角速度
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_kp*(Angle-Med)+Vertical_kd*gyro_Y;
	return temp;
}

//速度PI控制器
//输入：期望速度，左编码器，右编码器
 int velocity(int Target,int encoder_L,int encoder_R)
 {

	 static int Err_Lowout_last,Encoder_S;
	 static float a=0.3;
	 int Err,Err_Lowout,temp;
	 Velocity_ki=Velocity_kp/200;
	//1计算偏差值
	 Err=(encoder_R+encoder_L)-Target;
	 //2低通滤波
	 Err_Lowout=(1-a)*Err+a*Err_Lowout_last;
	 Err_Lowout_last=Err_Lowout;
	 //3积分
	 Encoder_S+=Err_Lowout;
	 //4积分限幅(-20000~20000)
	 Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	 if(stop==1)Encoder_S=0,stop=0;
	 //速度环计算
	 temp=Velocity_kp*Err_Lowout+Velocity_ki*Encoder_S;
	 return temp;
 }
 
 //转向环PD控制器
//输入：偏航角速度，角度值
 int Turn(int gyro_Z,int Target_Turn)
 {	
	 int temp;
	 temp=Turn_kp*Target_Turn+Turn_kd*gyro_Z;
	 return temp;
 
 }	  
 
 void Control(void) //每隔10ms调用一次
 {
	int PWM_out;
	 //1.读取编码器和陀螺仪的数据
	 Encoder_Left=Read_Speed(&htim2);
	 Encoder_Right=-Read_Speed(&htim4);
	 mpu_dmp_get_data(&pitch,&roll,&yaw);
	 MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	 MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
 
	 //2.将数据传入到PID控制器中，计算输出结果，左右电机转速值
	 velocity_out=velocity(Target_speed,Encoder_Left,Encoder_Right);
	 Vertical_out=Vertical(velocity_out+Med_Angle,roll,gyrox);
	 Turn_out=Turn(gyroz,Target_Turn);
	 PWM_out=Vertical_out;
	 MOTO1=PWM_out-Turn_out;
	 MOTO2=PWM_out+Turn_out;
	 Limit(&MOTO1,&MOTO2);
	 Load(MOTO1,MOTO2);
 }
 
 
