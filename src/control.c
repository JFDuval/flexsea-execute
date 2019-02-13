/****************************************************************************
	[Project] FlexSEA: Flexible & Scalable Electronics Architecture
	[Sub-project] 'flexsea-execute' Advanced Motion Controller
	Copyright (C) 2016 Dephy, Inc. <http://dephy.com/>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************
	[Lead developper] Jean-Francois (JF) Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors] Elliott Rouse, Luke Mooney
*****************************************************************************
	[This file] control: Control Loops
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "main_fsm.h"
#include "control.h"
#include "motor.h"
#include "ext_input.h"
#include "user-ex.h"
#include "calibration_tools.h"
#include "sensor_commut.h"
#include "trapez.h"
#include "flexsea_global_structs.h"
#include "mag_encoders.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Main data structure for all the controllers:
struct ctrl_s ctrl[2];

//In Control tool:
struct in_control_s in_control;

//****************************************************************************
// Function(s)
//****************************************************************************

//Use this function to change the control strategy
void control_strategy(uint8_t strat, uint8_t ch)
{
	//Are we already using this controller?
	//Are we currently running a calibration routine?
	if(calibrationFlags || ctrl[ch].active_ctrl == strat)
	{
		//Yes. Nothing to do, exit.
		return;
	}
	else
	{
		//Different controller
	
		//By default we place the gains to 0 before we change the strategy:
						
		//Position controller
		ctrl[ch].position.gain.P_KP = 0;
		ctrl[ch].position.gain.P_KI = 0;
		ctrl[ch].position.gain.P_KD = 0;
		
		//Impedance controller
		ctrl[ch].impedance.gain.Z_K = 0;
		ctrl[ch].impedance.gain.Z_B = 0;
		ctrl[ch].impedance.gain.Z_I = 0;
		
		//Current controller
		ctrl[ch].current.gain.I_KP = 0;
		ctrl[ch].current.gain.I_KI = 0;
		ctrl[ch].current.gain.I_KD = 0;
		ctrl[ch].current.setpoint_val = 0;
		ctrl[ch].current.error_sum = 0;
		
		//To avoid a huge startup error on the Position-based controllers:
		if(strat == CTRL_POSITION)
		{
			ctrl[ch].position.setp = refresh_enc_control(ch);
			steps = trapez_gen_motion_1(ctrl[ch].position.setp, ctrl[ch].position.setp, 1, 1);
		}
		else if(strat == CTRL_IMPEDANCE)
		{
			ctrl[ch].impedance.setpoint_val = refresh_enc_control(ch);
			steps = trapez_gen_motion_1(ctrl[ch].impedance.setpoint_val, ctrl[ch].impedance.setpoint_val, 1, 1);
		}
		
		//ToDo: pretty sure we won't use that as a control mode anymore. Remove.
		if (strat != CTRL_MEASRES)
		{
			measure_motor_resistance = 0;
		}
		else
		{
			measure_motor_resistance = 1;
		}
		
		ctrl[ch].active_ctrl = strat;
		in_control.controller = ctrl[ch].active_ctrl;
		
		//The user should call a set gain function at this point.
	}
}

//Starts all the parameters at 0 or Default values
void init_ctrl_data_structure(void)
{
	uint8_t ch = 0;
	
	for(ch = 0; ch < 2; ch++)
	{
		//No active controller:
		ctrl[ch].active_ctrl = CTRL_NONE;
		
		//Generic controller
		ctrl[ch].generic.gain.g0 = 0;
		ctrl[ch].generic.gain.g1 = 0;
		ctrl[ch].generic.gain.g2 = 0;
		ctrl[ch].generic.gain.g3 = 0;
		ctrl[ch].generic.gain.g4 = 0;
		ctrl[ch].generic.gain.g5 = 0;
		ctrl[ch].generic.actual_val = 0;
		ctrl[ch].generic.setpoint_val = 0;
		ctrl[ch].generic.error = 0;
		ctrl[ch].generic.error_sum = 0;
		ctrl[ch].generic.error_dif = 0;
		
		//Position controller
		ctrl[ch].position.gain.g0 = 0;
		ctrl[ch].position.gain.g1 = 0;
		ctrl[ch].position.gain.g2 = 0;
		ctrl[ch].position.gain.g3 = 0;
		ctrl[ch].position.gain.g4 = 0;
		ctrl[ch].position.gain.g5 = 0;
		ctrl[ch].position.pos = 0;
		ctrl[ch].position.setp = 0;
		ctrl[ch].position.error = 0;
		ctrl[ch].position.error_sum = 0;
		ctrl[ch].position.error_dif = 0;
		
		//Impedance controller
		ctrl[ch].impedance.gain.g0 = 0;
		ctrl[ch].impedance.gain.g1 = 0;
		ctrl[ch].impedance.gain.g2 = 0;
		ctrl[ch].impedance.gain.g3 = 0;
		ctrl[ch].impedance.gain.g4 = 0;
		ctrl[ch].impedance.gain.g5 = 0;
		ctrl[ch].impedance.actual_val = 0;
		ctrl[ch].impedance.actual_vel = 0;
		ctrl[ch].impedance.setpoint_val = 0;
		ctrl[ch].impedance.error = 0;
		ctrl[ch].impedance.error_sum = 0;
		ctrl[ch].impedance.error_dif = 0;
		
		//Current controller
		ctrl[ch].current.gain.g0 = 0;
		ctrl[ch].current.gain.g1 = 0;
		ctrl[ch].current.gain.g2 = 0;
		ctrl[ch].current.gain.g3 = 0;
		ctrl[ch].current.gain.g4 = 0;
		ctrl[ch].current.gain.g5 = 0;
		ctrl[ch].current.actual_val = 0;
		ctrl[ch].current.setpoint_val = 0;
		ctrl[ch].current.error = 0;
		ctrl[ch].current.error_sum = 0;
		ctrl[ch].current.error_dif = 0;
	}
}

//Motor position controller - non blocking
int32 motor_position_pid(int32 wanted_pos, int32 actual_pos, uint8_t ch)
{
	int32 p = 0, i = 0, d = 0;
	int32 pwm = 0;

	//Position values:
	ctrl[ch].position.pos = actual_pos;
	ctrl[ch].position.setp = wanted_pos;
	in_control.actual_val = ctrl[ch].position.pos;
	in_control.setp = ctrl[ch].position.setp;
	
	//Errors:
	ctrl[ch].position.error_prev = ctrl[ch].position.error;
	ctrl[ch].position.error = ctrl[ch].position.setp - ctrl[ch].position.pos;
	in_control.error = ctrl[ch].position.error;
	ctrl[ch].position.error_sum = ctrl[ch].position.error_sum + ctrl[ch].position.error;
	ctrl[ch].position.error_dif = (ctrl[ch].position.error_dif << 1) + \
							6*(ctrl[ch].position.error - ctrl[ch].position.error_prev);
	ctrl[ch].position.error_dif = ctrl[ch].position.error_dif >> 3;	
	
	//Saturate cumulative error
	if(ctrl[ch].position.error_sum >= MAX_ERR_SUM)
		ctrl[ch].position.error_sum = MAX_ERR_SUM;
	if(ctrl[ch].position.error_sum <= -MAX_ERR_SUM)
		ctrl[ch].position.error_sum = -MAX_ERR_SUM;
	
	//Proportional term
	p = (int32_t)((int32_t)ctrl[ch].position.gain.P_KP * ctrl[ch].position.error) / 100;
	in_control.r[0] = p;
	//Integral term
	i = (int32_t)((int32_t)ctrl[ch].position.gain.P_KI * (ctrl[ch].position.error_sum/10)) / 250;
	in_control.r[1] = i;
	//Differential term:
	d = (int32_t)((int32_t)ctrl[ch].position.gain.P_KD * ctrl[ch].position.error_dif) / 100;
	in_control.r[2] = d;
	
	//Output
	pwm = (p + i + d);		//
	
	//(Saturation happens in setMotorVoltage(), if needed)
	
	setMotorVoltage(pwm, ch);
	in_control.output = pwm;
	
	return ctrl[ch].position.error;
}

//Motor position controller - non blocking. PID + Feed Forward (FF)
//The FF term comes from the calling function, it's added to the output.
int32 motor_position_pid_ff_1(int32 wanted_pos, int32 actual_pos, int32 ff, uint8_t ch)
{
	int32 p = 0, i = 0, d = 0;
	int32 pwm = 0;

	//Position values:
	ctrl[ch].position.pos = actual_pos;
	ctrl[ch].position.setp = wanted_pos;
	
	//Errors:
	ctrl[ch].position.error = ctrl[ch].position.pos - ctrl[ch].position.setp;
	ctrl[ch].position.error_sum = ctrl[ch].position.error_sum + ctrl[ch].position.error;
	//ctrl[ch].position.error_dif ToDo
	
	//Saturate cumulative error
	if(ctrl[ch].position.error_sum >= MAX_CUMULATIVE_ERROR)
		ctrl[ch].position.error_sum = MAX_CUMULATIVE_ERROR;
	if(ctrl[ch].position.error_sum <= -MAX_CUMULATIVE_ERROR)
		ctrl[ch].position.error_sum = -MAX_CUMULATIVE_ERROR;
	
	//Proportional term
	p = (int32_t)((int32_t)ctrl[ch].position.gain.P_KP * ctrl[ch].position.error) / 100;
	//Integral term
	i = (int32_t)((int32_t)ctrl[ch].position.gain.P_KI * ctrl[ch].position.error_sum) / 100;
	//Derivative term
	d = 0;	//ToDo
	
	//Output
	pwm = (p + i + d) + ff;		//
	
	//Saturates PWM to low values
	if(pwm >= POS_PWM_LIMIT)
		pwm = POS_PWM_LIMIT;
	if(pwm <= -POS_PWM_LIMIT)
		pwm = -POS_PWM_LIMIT;
	
	setMotorVoltage(pwm, ch);
	
	return ctrl[ch].position.error;
}

//PI Current controller #3: for sinusoidal commutation
//'wanted_curr' & 'measured_curr' are centered at zero and are in the ±CURRENT_SPAN range
//The sign of 'wanted_curr' will change the rotation direction, not the polarity of the current (I have no control on this)
inline int32 motor_current_pid_3(int32 wanted_curr, int32 measured_curr, uint8_t ch)
{
	int32_t sign = 0;
	
	//Error and integral of errors:
	ctrl[ch].current.error = (wanted_curr - measured_curr);	//Actual error
	ctrl[ch].current.error_sum += ctrl[ch].current.error;	//Cumulative error
	
	//Saturate cumulative error
	if(ctrl[ch].current.error_sum >= MAX_CUM_CURRENT_ERROR)
		ctrl[ch].current.error_sum = MAX_CUM_CURRENT_ERROR;
	if(ctrl[ch].current.error_sum <= -MAX_CUM_CURRENT_ERROR)
		ctrl[ch].current.error_sum = -MAX_CUM_CURRENT_ERROR;

	//Proportional term
	volatile int32 curr_p = (int)((ctrl[ch].current.gain.I_KP * ctrl[ch].current.error)>>8);
	//Integral term
	volatile int32 curr_i = (int)((ctrl[ch].current.gain.I_KI * ctrl[ch].current.error_sum)>>13);
	//Add differential term here if needed
	//In both cases we divide to get a finer gain adjustement w/ integer values.

	//Output
	volatile int32 curr_pwm = curr_p + curr_i + as5047.signed_ang_vel*37 + wanted_curr/10;
	
	#if(MOTOR_COMMUT == COMMUT_SINE) 

		setMotorVoltage(curr_pwm, ch);
	
	#endif
		
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
		//Sign extracted from wanted_curr:
		if(wanted_curr < 0)
		{
			curr_pwm = -curr_pwm;
			MotorDirection_Control = 0;		//MotorDirection_Write(0);
		}
		else
		{
			MotorDirection_Control = 1;		//MotorDirection_Write(1);
		}

		//Saturates PWM
		if(curr_pwm >= POS_PWM_LIMIT)
			curr_pwm = POS_PWM_LIMIT;
		if(curr_pwm <= -POS_PWM_LIMIT)
			curr_pwm = POS_PWM_LIMIT;

		//Write duty cycle to PWM module (avoiding double function calls)
		curr_pwm = PWM1DC(curr_pwm);

		CY_SET_REG16(PWM_1_COMPARE1_LSB_PTR, (uint16)curr_pwm);	
		CY_SET_REG16(PWM_1_COMPARE2_LSB_PTR, (uint16)(PWM2DC(curr_pwm)));
		//Compare 2 can't be 0 or the ADC won't trigger => that's why I'm adding 1
	#endif
		
	return ctrl[ch].current.error;	
}

//Impedance controller
void impedance_controller(uint8_t ch)
{
	int32 spring_torq; 
	int32 damping_torq;

	spring_torq = ((ctrl[ch].impedance.setpoint_val-ctrl[ch].impedance.actual_val)*ctrl[ch].impedance.gain.g0)>>9;
	damping_torq = (-1*(ctrl[ch].impedance.actual_vel)*ctrl[ch].impedance.gain.g1)>>6;
	
	ctrl[ch].current.setpoint_val = (spring_torq+damping_torq);
}

//in_control.combined = [CTRL2:0][MOT_DIR][PWM]
void in_control_combine(void)
{
	uint16_t tmp = 0;
	
	tmp = ((in_control.controller & 0x03) << 13) | ((in_control.mot_dir & 0x01) << 12) | (in_control.pwm & 0xFFF);
	in_control.combined = tmp;
}

//Reads the PWM and MOTOR_DIR values from hardware:
void in_control_get_pwm_dir(void)
{
	#if(MOTOR_COMMUT == COMMUT_BLOCK)
	in_control.pwm = PWM_1_ReadCompare1();
	in_control.mot_dir = MotorDirection_Read();
	#else
	//ToDo Sine
	in_control.pwm = 0;
	in_control.mot_dir = 0;
	#endif	//#if(MOTOR_COMMUT == COMMUT_BLOCK)
}
