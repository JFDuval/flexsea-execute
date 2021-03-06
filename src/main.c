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
	[This file] main: FlexSEA-Execute
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

// FlexSEA: Flexible & Scalable Electronics Architecture

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main_fsm.h"
#include "main.h"
#include "misc.h"
#include "ui.h"
#include "local_comm.h"
#include "peripherals.h"
#include "main_fsm.h"
#include "user-ex.h"
#include "cyapicallbacks.h"
#include "flexsea_system.h"
#include "misc.h"
#include "flexsea_user_structs.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//Map fsm case to an index:
void (*fsmCases[10])(void) = {&mainFSM0, &mainFSM1, &mainFSM2, &mainFSM3, \
			&mainFSM4, &mainFSM5, &mainFSM6, &mainFSM7,	&mainFSM8, &mainFSM9};

//****************************************************************************
// Function(s)
//****************************************************************************

// attribute guarantees inlining unless inlining "causes a problem"
// for example: inlining a recursive function into itself is only done once
__attribute__((always_inline)) __STATIC_INLINE void runFSM(uint8_t index)
{
	activeFSM = index;
	//DEBUG_H2(1);
	fsmCases[index]();
	//DEBUG_H2(0);
	activeFSM = FSMS_INACTIVE;
}

int main(void)
{
	//Power on delay with LEDs
	power_on();	     
	
	//Prepare FlexSEA Stack & communication:
	init_flexsea_payload_ptr();
	initLocalComm();

	//Initialize all the peripherals
	init_peripherals();
	
	initializeGlobalStructs();
	initializeUserStructs();

	#ifdef FINDPOLES
		calibrationFlags |= CALIBRATION_FIND_POLES;
	#endif
	
	//Test code, use with care. Normal code might NOT run when enabled!
	//test_code_blocking();
	//test_code_non_blocking();
	
	//Project specific initialization code
	init_user();

    //Turn on manage
   	bootManage();
	
	//Main loop
	while(1)
	{
		if(t1_new_value == 1)
		{
			//If the time share slot changed we run the timing FSM. Refer to
			//timing.xlsx for more details. 't1_new_value' updates at 10kHz,
			//each slot at 1kHz.			
			
			t1_new_value = 0;			
			
            runFSM(t1_time_share);

			//Increment value, limits to 0-9
			TICK_COUNTER(t1_time_share, 10);
            
			//The code below is executed every 100us, after the previous slot. 
			//Keep it short! (<10us if possible)
			mainFSM10kHz();
		}
		else
		{
			//Asynchronous code goes here.
			mainFSMasynchronous();
		}
	}
}
