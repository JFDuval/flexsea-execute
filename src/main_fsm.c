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
	[This file] main_fsm: Contains all the case() code for the main FSM
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "i2c.h"
#include "main_fsm.h"
#include "misc.h"
#include "ui.h"
#include "motor.h"
#include "peripherals.h"
#include "analog.h"
#include "control.h"
#include "sensor_commut.h"
#include "usb.h"
#include "safety.h"
#include "ext_input.h"
#include "flexsea_global_structs.h"
#include "flexsea_interface.h"
#include "calibration_tools.h"
#include "flexsea_cmd_stream.h"
#include "flexsea.h"
#include "trapez.h"
#include "i2t-current-limit.h"
#include "flexsea_board.h"
#include "local_comm.h"
#include "strain.h"
#include "mag_encoders.h"
#include "user-ex.h"
#include <flexsea_board.h>
#include "flexsea_comm_multi.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t eL0 = 0, eL1 = 0, eL2 = 0;
uint16 safety_delay = 0;
uint8_t new_cmd_led = 0;
uint8_t toggle_wdclk = 0;
uint8_t readyToDecodeEXI2C = 0;
volatile uint8_t suppressMotor = 0;

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

uint16_t computeFsmStatus(volatile int8_t *timingError);
void transmitMultiFrame();

//****************************************************************************
// Public Function(s)
//****************************************************************************

//1kHz time slots:
//================

//Case 0: I2C_0
void mainFSM0(void)
{
	i2c_0_fsm();
}

//Case 1: I2C_1
void mainFSM1(void)
{
	refreshExStructureData();
	
	//Read from Safety Co-Processor
	#ifdef USE_I2C_1
	
	safety_cop_read_all();
	
	#endif 	//USE_I2C_1
}

//Case 2: some safety features & 100ms timebase
void mainFSM2(void)
{
	#ifdef USE_I2T_LIMIT
	//Sample current (I2t limit):
	i2t_sample(ctrl[0].current.actual_vals.avg);
	#endif	//USE_I2T_LIMIT
			
	//100ms timebase:
	if(timebase_100ms())
	{
		#ifdef USE_I2T_LIMIT
		//Is the current in range?
		currentLimit = i2t_compute();
		#endif	//USE_I2T_LIMIT
	} 
}

//Case 3: Strain Gauge DelSig ADC, SAR ADC
void mainFSM3(void)
{
	#ifdef USE_STRAIN
	//Start a new conversion
	ADC_DelSig_1_StartConvert();
	
	//Filter the previous results
	strain_filter_dma();
	#endif
}

//Case 4: User Interface & Communication
static uint8_t autoParsed;
void mainFSM4(void)
{
	//Alive LED
	alive_led();
	
	//UI RGB LED:
	
	rgbLedRefreshFade();
	
	if(safety_delay > SAFETY_DELAY)
	{
		//status_error_codes(safety_cop.status1, safety_cop.status2, &eL0, &eL1, &eL2); 
	}
	else
	{
		safety_delay++;
	}
	
	//Display temperature status on RGB	
	overtemp_error(&eL1, &eL2);					//Comment this line if safety code is problematic
	rgb_led_ui(eL0, eL1, eL2, new_cmd_led);		//ToDo add more error codes
	if(new_cmd_led)
	{
		new_cmd_led = 0;
	}
	
	//Communication:
	autoParsed = 0;
	if(receiveFxPacketByPeriph(comm_multi_periph + PORT_USB) && comm_multi_periph[PORT_USB].out.unpackedIdx > 0)
	{
		autoParsed++;
	}
	
	//LED turns green when commands are received
	if(autoParsed)
	{
		new_cmd_led = 1;
	}
}

//Case 5: Position sensors & Position setpoint, and Comm
void mainFSM5(void)
{
	//Refresh encoder readings (ENC_CONTROL only)
	refresh_enc_control(0);
	
	#ifdef USE_TRAPEZ	

	//Trapezoidal trajectories (can be used for both Position & Impedance)	
	if((ctrl[0].active_ctrl == CTRL_POSITION) || (ctrl[0].active_ctrl == CTRL_IMPEDANCE))
	{
		ctrl[0].position.trap_t++;
		ctrl[0].impedance.trap_t++;
		ctrl[0].position.setp = trapez_get_pos(steps);	//New setpoint
		ctrl[0].impedance.setpoint_val = trapez_get_pos(steps);	//New setpoint
	}
	
	#endif	//USE_TRAPEZ
	
	//Communication:
	if(!autoParsed)
	{
		autoStream();
	}
}

//Case 6: P & Z controllers, 0 PWM
void mainFSM6(void)
{
	uint8_t ch = 0;
	
	//Comm:
	int i;
	for(i = 0; i < NUMBER_OF_PORTS; ++i)
	{
		if(comm_multi_periph[i].out.unpackedIdx)
		{
			packMultiPacket(&(comm_multi_periph[i].out));
			comm_multi_periph[i].out.unpackedIdx = 0;
		}
	}
	
	// If we are running a calibration test, all controllers should be disabled anyways.
	// Also we should be in CTRL_NONE, but that should be handled elsewhere
	if(calibrationFlags != 0)
	{
		return;
	}
	
	if(ctrl[ch].active_ctrl == CTRL_POSITION)
	{
		motor_position_pid(ctrl[ch].position.setp, ctrl[ch].position.pos, ch);
	}
	else if(ctrl[ch].active_ctrl == CTRL_IMPEDANCE)
	{
		impedance_controller(ch);
	}
	
	//If no controller is used the PWM should be 0:
	if(ctrl[ch].active_ctrl == CTRL_NONE)
	{
		setMotorVoltage(0, ch);
	}
	
	//If we have a communication problem we kill the motor:
	if(suppressMotor)
	{
		ctrl[ch].active_ctrl = CTRL_NONE;
		setMotorVoltage(0, ch);
	}
}

//Case 7:
void mainFSM7(void)
{
	//Timestamp needed by GUI:
	rigid1.ctrl.timestamp++;
}

//Case 8: SAR ADC filtering
void mainFSM8(void)
{
	update_diffarr_avg(&ctrl[0].current.actual_vals,50);
	calc_motor_L();
	if(adc_sar1_flag)
	{
		filter_sar_adc();
		adc_sar1_flag = 0;
	}	
}

//Case 9: User functions & 1s timebase	
void mainFSM9(void)
{
	if(calibrationFlags & CALIBRATION_FIND_POLES)
	{
		find_poles();
		if(!findingpoles)
		{
			calibrationFlags = 0;
		}
	}
	else
	{
		#if(RUNTIME_FSM == ENABLED)
			user_fsm();
		#endif
	}
	
	//1s timebase:
	if(timebase_1s())
	{
		#ifdef USE_USB
		//Tries to connect to USB:
		usbRuntimeConnect();
		#endif
	}
}

//10kHz time slot:
//================

void mainFSM10kHz(void)
{
	//FlexSEA Network Communication
	#ifdef USE_COMM
		
		//Did we receive new bytes from a master?
		flexsea_receive_from_master();
		
		//Did we receive new commands? Can we parse them?
		parseMasterCommands(&new_cmd_led);
		
		//Time to reply - RS-485?
		sendMasterDelayedResponse();
		
		//Monitor comm:
		//suppressMotor = detectMnCommError(new_cmd_led);
		
		// Multi Packet stuff
		static uint8_t flip = 0;
		flip ^= 1;
		if(flip)
		{
			transmitMultiFrame();
		}
	
	#endif	//USE_COMM 
	
	#if(((MOTOR_COMMUT == COMMUT_BLOCK) && (CURRENT_SENSING != CS_LEGACY)) || \
		(MOTOR_COMMUT == COMMUT_SINE))
		
		if((calibrationFlags == 0) && ((ctrl[0].active_ctrl == CTRL_CURRENT) || (ctrl[0].active_ctrl == CTRL_IMPEDANCE)))
		{
			//Current controller
			motor_current_pid_3(ctrl[0].current.setpoint_val, ctrl[0].current.actual_vals.avg, 0);
		}
		else
		{
			ctrl[0].current.error_sum = 0;
		}
		
	#endif
	
	//RGB LED:
	rgbLedRefresh();
}

//Asynchronous time slots:
//========================

void mainFSMasynchronous(void)
{
	//WatchDog Clock (Safety-CoP)
	toggle_wdclk ^= 1;
	WDCLK_Write(toggle_wdclk);
}

uint16_t computeFsmStatus(volatile int8_t *timingError)
{
	int8_t mostOffendingFSM = -1;
	uint8_t numOffenses = 0;
	int i;
	for(i = 0; i < 10; ++i)
	{
		if(timingError[i] > numOffenses)
		{
			mostOffendingFSM = i;
			numOffenses = timingError[i];
		}
	}

	uint16_t fsmStatus = 0;
	if(mostOffendingFSM != -1)
		fsmStatus = (mostOffendingFSM << 4) | (numOffenses & 0x0F);

	return fsmStatus;
}

void transmitMultiFrame()
{
	transmitFxPacket(PORT_USB);
}
