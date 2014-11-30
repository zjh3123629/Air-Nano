#include "include.h"
#include "control.h"


struct _ctrl ctrl;
int16_t Moto_duty[4];

extern float GY;

/***************************************************/
/*void CONTROL(float rol, float pit, float yaw)    */
/*输入：rol   横滚角                               */
/*      pit   俯仰角                               */
/*			yaw   航向                                 */
/*输出：                                           */
/*备注：串级PID 控制   外环（角度环）采用PID调节    */
/*                     内环（角速度环）采用PD调节  */
/***************************************************/
void CONTROL(float rol, float pit, float yaw)   
{
	static float roll_old,pitch_old;
	if(ctrl.ctrlRate >= 2)
	{
		//*****************外环(角度环)PID**************************//
		//横滚计算///////////////
	  pit = pit + RC_Data.PITCH/1.4;
		ctrl.pitch.shell.increment += pit;
		
		//limit for the max increment
		if(ctrl.pitch.shell.increment > ctrl.pitch.shell.increment_max)  	ctrl.pitch.shell.increment = ctrl.pitch.shell.increment_max;
		else if(ctrl.pitch.shell.increment < -ctrl.pitch.shell.increment_max)		ctrl.pitch.shell.increment = -ctrl.pitch.shell.increment_max;
		
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * pit + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment + ctrl.pitch.shell.kd * (pit - pitch_old);
		pitch_old = pit;
		
		//俯仰计算//////////////
		rol = rol + RC_Data.ROLL/1.4;
		ctrl.roll.shell.increment += rol;
		
		//limit for the max increment
		if(ctrl.roll.shell.increment > ctrl.roll.shell.increment_max)  	ctrl.roll.shell.increment = ctrl.roll.shell.increment_max;
		else if(ctrl.roll.shell.increment < -ctrl.roll.shell.increment_max)		ctrl.roll.shell.increment = -ctrl.roll.shell.increment_max;

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * rol + ctrl.roll.shell.ki * ctrl.roll.shell.increment + ctrl.roll.shell.kd * (rol - roll_old);
		roll_old = rol;
		
		//航向计算/////////////
	  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * RC_Data.YAW + ctrl.yaw.shell.kd * sensor.gyro.origin.z;
    ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
  //********************内环(角速度环)PD*********************************//
	ctrl.roll.core.kp_out = ctrl.roll.core.kp * (ctrl.roll.shell.pid_out + sensor.gyro.radian.x * RtA);
	ctrl.roll.core.kd_out = ctrl.roll.core.kd * (sensor.gyro.origin.x - sensor.gyro.histor.x);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * (ctrl.pitch.shell.pid_out + sensor.gyro.radian.y * RtA);
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.origin.y - sensor.gyro.histor.y);
	
	ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * (ctrl.yaw.shell.pid_out + sensor.gyro.radian.z * RtA);
	ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (sensor.gyro.origin.z - sensor.gyro.histor.z);
	
	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.kd_out;
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.kd_out;
	ctrl.yaw.core.pid_out = ctrl.yaw.core.kp_out + ctrl.yaw.core.kd_out;

	sensor.gyro.histor.x = sensor.gyro.origin.x;   //储存历史值
	sensor.gyro.histor.y = sensor.gyro.origin.y;
  sensor.gyro.histor.z = sensor.gyro.origin.z;
	
	
	if(RC_Data.THROTTLE>40)
	{
		int date_throttle	= RC_Data.THROTTLE/cos(angle.roll/RtA)/cos(angle.pitch/RtA);
		Moto_duty[0] = date_throttle + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
		Moto_duty[1] = date_throttle + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
		Moto_duty[2] = date_throttle - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
		Moto_duty[3] = date_throttle - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
	}
	else
	{	
    Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] =0;	
		ctrl.pitch.shell.increment = 0;
		ctrl.roll.shell.increment= 0;		

	}
	
	if(RC_Data.ARMED)  moto_PwmRflash(&Moto_duty[0]);		
	else       moto_STOP();		
}



void Deblocking(void)
{
	 static vs8 flag=1;
	 static vs16 time1=0,time2=0;
   if(!RC_Data.ARMED && RC_Data.ROLL >= 9 && RC_Data.PITCH >= 9 && RC_Data.THROTTLE <= 40)		{  time1++; }	
	 else time1=0;
	 if(time1>20 && !RC_Data.ARMED) { RC_Data.ARMED = 1; time1 = 0;}
  		
   if(RC_Data.ARMED && RC_Data.ROLL <= -14 && RC_Data.PITCH >= 10 && RC_Data.THROTTLE <= 40)		{  time2++; }	
	 else time2=0;
	 if(time2>20 && RC_Data.ARMED)   {  RC_Data.ARMED = 0; time2 = 0;  }
}

