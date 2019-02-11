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

//Note: It's a sign that code is misplaced when this file gets long.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "misc.h"
#include "user-ex.h"
#include <flexsea_comm.h>
#include "imu.h"
#include "control.h"
#include "analog.h"
#include "safety.h"
#include "strain.h"
#include "flexsea_global_structs.h"
//(Add #include as needed when running test code)

//****************************************************************************
// Variable(s)
//****************************************************************************

//Timers:
volatile uint8_t t1_100us_flag = 0;
volatile uint8_t t1_time_share = 0, t1_new_value = 0;
volatile uint8_t activeFSM = FSMS_INACTIVE;
volatile int8_t timingError[10] = {0};

//int32_t angle_read_counter = 0, last_angle_read_gap = 0;

//ADC:
volatile uint8_t adc_sar1_flag = 0;
volatile uint8_t adc_delsig_flag = 0, adc_delsig_lastCh = 0;

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Call this function in the 1kHz FSM. It will return 1 every second.
uint8_t timebase_1s(void)
{
	static uint16 time = 0;
	
	time++;
	if(time >= 999)
	{
		time = 0;
		return 1;
	}
	
	return 0;
}

//Call this function in the 1kHz FSM. It will return 1 every 100ms.
uint8_t timebase_100ms(void)
{
	static uint16 time = 0;
	
	time++;
	if(time >= 99)
	{
		time = 0;
		return 1;
	}
	
	return 0;
}

//Fill exec1 with latest sensor values:
void refreshExStructureData(void)
{
	#ifdef USE_IMU
	exec1.accel.x = imu.accel.x;
	exec1.accel.y = imu.accel.y;
	exec1.accel.z = imu.accel.z;
	exec1.gyro.x = imu.gyro.x;
	exec1.gyro.y = imu.gyro.y;
	exec1.gyro.z = imu.gyro.z;
	#endif

    #ifdef USE_STRAIN
	exec1.strain = strain_read();
    #endif

	exec1.analog[0] = read_analog(0);
	exec1.analog[1] = read_analog(1);
	
	exec1.current = ctrl[0].current.actual_val;
	
	exec1.volt_batt = safety_cop.v_vb;
	exec1.volt_int = safety_cop.v_vg;
	exec1.temp = safety_cop.temperature;
	exec1.status1 = safety_cop.status1;
	exec1.status2 = safety_cop.status2;
	
	//Decode some values:
	decodeExData(&exec1);
}

//ToDo: replace float calculation by integer math
void decodeExData(struct execute_s *exPtr)
{
	//Accel in mG
	exPtr->decoded.accel.x = (1000*exPtr->accel.x)/8192;
	exPtr->decoded.accel.y = (1000*exPtr->accel.y)/8192;
	exPtr->decoded.accel.z = (1000*exPtr->accel.z)/8192;

	//Gyro in degrees/s
	exPtr->decoded.gyro.x = (100*exPtr->gyro.x)/164;
	exPtr->decoded.gyro.y = (100*exPtr->gyro.y)/164;
	exPtr->decoded.gyro.z = (100*exPtr->gyro.z)/164;

	exPtr->decoded.strain = 0;

	exPtr->decoded.current = exPtr->current;   //1mA/bit for sine comm.

	exPtr->decoded.volt_batt = (int32_t)1000*P4_ADC_SUPPLY*((16*\
						(float)exPtr->volt_batt/3 + 302 ) \
						/P4_ADC_MAX) / 0.0738;          //mV

	exPtr->decoded.volt_int = (int32_t)1000*P4_ADC_SUPPLY*((26*\
						(float)exPtr->volt_int/3 + 440 ) \
						/P4_ADC_MAX) / 0.43;            //mV

	exPtr->decoded.temp = ((1300*exPtr->temp + 20500)/2048) - 50;

	exPtr->decoded.analog[0] = 0;
	exPtr->decoded.analog[1] = 0;
	exPtr->decoded.analog[2] = 0;
	exPtr->decoded.analog[3] = 0;
	exPtr->decoded.analog[4] = 0;
	exPtr->decoded.analog[5] = 0;
	exPtr->decoded.analog[6] = 0;
	exPtr->decoded.analog[7] = 0;
}

void test_code_blocking(void)
{
	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//Blocking Test code - enable one and only one for special 
	//debugging. Normal code WILL NOT EXECUTE when this is enabled!
	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
	//strain_test_blocking();
	//safety_cop_comm_test_blocking();
	//imu_test_code_blocking();
	//motor_fixed_pwm_test_code_blocking(200);
	//wdclk_test_blocking();
	//timing_test_blocking();
	//test_uart_dma_xmit();
	//motor_cancel_damping_test_code_blocking();
	//test_pwro_output_blocking();
	//strain_amp_6ch_test_code_blocking();
	//as5048b_test_code_blocking();
	//rgbLedRefresh_testcode_blocking();
	//compress6chTestCodeBlocking();
	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=	
}

void test_code_non_blocking(void)
{
	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
	//Non-Blocking Test code
	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
	//motor_fixed_pwm_test_code_non_blocking(125);
	//pwro_output(245);	
	//test_angle_eeprom();
	//test_angle_flash();
	//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=		
}

//If the Header isn't in [0] we 'unwrap' the array and save it in 'new_array'
//Note: this function used to be in the flexsea-comm stack, but only Execute was 
//using it. I think that the circular buffers will render it useless (ToDo)
uint8_t unwrap_buffer(uint8_t *array, uint8_t *new_array, uint32_t len)
{
	uint8_t i = 0, j = 0, retval = 0, idx = 0;

	if(array[0] != HEADER)	//Quick check
	{
		for(i = 1; i < len; i++)
		{
			if(array[i] == HEADER)
			{
				//We found the header
				idx = i;
				for(j = 0; j < len; j++)
				{
					new_array[j] = array[idx];
					idx++;
					idx %= len;
				}

				retval = i;
				break;
			}
		}
	}
	else
	{
		//No need to unwrap, copy & exit
		for(i = 0; i < len; i++)
		{
			//new_array = array
			new_array[i] = array[i];
		}
		retval = 0;
	}

	return retval;
}

//Boot Manage for setups that include it
void bootManage(void)
{
	EX15_Write(1);
	CyDelay(1);
	EX15_Write(0);
}
