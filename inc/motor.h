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
	[Contributors] Luke Mooney
*****************************************************************************
	[This file] motor: motor control functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_MOTOR_H
#define INC_MOTOR_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************
	
//****************************************************************************
// Prototype(s):
//****************************************************************************

void init_motor(void);	
void motor_open_speed_1(int32 pwm_duty);
void motor_open_speed_2(int16 pwm_duty, int sign);
void setMotorVoltage(int32 mV, uint8_t ch);

void motor_fixed_pwm_test_code_blocking(int spd);
void motor_fixed_pwm_test_code_non_blocking(int spd);

void initDmaPwmCompare(void);
void setDmaPwmCompare(uint16_t a, uint16_t b, uint16_t c);

void initDmaPwmCompare(void);
void setDmaPwmCompare(uint16_t a, uint16_t b, uint16_t c);

//****************************************************************************
// Definition(s):
//****************************************************************************

//PWM limits
#define MAX_PWM						1000
#define MIN_PWM						(-MAX_PWM)
#define P1_DEADTIME					30					//Make sure that it matches the hardware setting!
#define PWM1DC(x)					MAX(x, (P1_DEADTIME+2))
#define PWM2DC(x)					MAX(((x - P1_DEADTIME)>>1), 10)

//Motor voltage vs PWM:
//Actual ouput of mosfet V = pwm*Vb/1000
//If the amp of the sin wave is V then the line-to-line voltage is = Vll = V*(3^.5) = (3^.5)*pwm*Vb/1000 = pwm*Vb/577
//If you want the pwm for the corresponding Vll then pwm = 577*Vll/Vb
#define GET_PWM_FROM_V(v,vb) 		(((int32_t)v * 577)/((int32_t)vb))
#define GET_V_FROM_PWM(pwm,vb)		(((int32_t)pwm * (int32_t)vb)/577)

//Common definitions, DMA_Px:
#define DMA_PX_BYTES_PER_BURST 		2
#define DMA_PX_REQUEST_PER_BURST 	1
#define DMA_PX_SRC_BASE 			(CYDEV_SRAM_BASE)
#define DMA_PX_DST_BASE 			(CYDEV_PERIPH_BASE)
	
//****************************************************************************
// Structure(s)
//****************************************************************************	

#endif	//INC_MOTOR_H
