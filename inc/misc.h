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
	[Contributors]
*****************************************************************************
	[This file] misc: when it doesn't belong in any another file, it ends up 
	here...
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/
	
#ifndef INC_MISC_H
#define INC_MISC_H

//****************************************************************************
// Include(s)
//****************************************************************************
	
#include "main.h"
#include "flexsea_global_structs.h"
	
//****************************************************************************
// Shared variable(s)
//****************************************************************************
	
extern volatile uint8_t t1_100us_flag;
extern volatile uint8_t t1_time_share, t1_new_value;
extern volatile uint8_t activeFSM;
extern volatile int8_t timingError[10];
#define FSMS_INACTIVE UINT8_MAX
// MACRO for incrementing and limiting counter values (making them restart at 0 after hitting a value)
#define TICK_COUNTER(x, lim) do { if((++x) >= lim) x = 0; } while(0)
	
extern volatile uint8_t adc_sar1_flag;	
extern volatile uint8_t adc_delsig_flag;

//extern uint16 last_as5047_word;	    
//extern int32 angle_read_counter; 
//extern int32 last_angle_read_gap;
    
//****************************************************************************
// Public Function Prototype(s):
//****************************************************************************

void test_code_blocking(void);	
void test_code_non_blocking(void);
uint8_t timebase_1s(void);
uint8_t timebase_100ms(void);
void refreshExStructureData(void);
void decodeExData(struct execute_s *exPtr);
uint8_t unwrap_buffer(uint8_t *array, uint8_t *new_array, uint32_t len);
void bootManage(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

#define SDELAY	5

//PSoC 5 ADC conversions:
#define P5_ADC_SUPPLY				5.0
#define P5_ADC_MAX					4096

//PSoC 4 ADC conversions:
#define P4_ADC_SUPPLY				5.0
#define P4_ADC_MAX					2048
#define P4_T0						0.5
#define P4_TC						0.01
	
#endif	//INC_MISC_H
