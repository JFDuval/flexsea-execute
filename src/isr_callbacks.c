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
	[This file] isr_callbacks: Implementation of the ISR functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//Important: this files goes with "cyapicallbacks.h". PSoC Creator 3.3 doesn't 
//include "cyapicallbacks.h" if I place it in /inc/... or I'm doing something
//wrong. Solution was to place it in /execute_1_0.cydsn/.

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "cyapicallbacks.h"
#include "misc.h"
#include "analog.h"
#include "current_sensing.h"
#include "control.h"
#include "serial.h"
#include "flexsea_board.h"
#include "flexsea_buffers.h"
#include "mag_encoders.h"
#include "sensor_commut.h"
#include "user-ex.h"

//****************************************************************************
// Public Function(s)
//****************************************************************************

void isr_t1_Interrupt_InterruptCallback()
{
	//Timer 1: 100us
	
	//Clear interrupt
	Timer_1_ReadStatusRegister();
	isr_t1_ClearPending();

	//All the timings are based on 100us slots
	//10 slots form the original 1ms timebase
	//'t1_time_share' is from 0 to 9, it controls the main FSM
	
	t1_new_value = 1;
	
	//Flag for the main code
	t1_100us_flag = 1;
}

void isr_t2_Interrupt_InterruptCallback()
{
	//Timer 2: 10us delay. RS-485 transceiver.
	
	//Clear interrupt
	Timer_2_ReadStatusRegister();
	isr_t2_ClearPending();	
	
	//Transfer is over, enable Receiver & disable Emitter
	UART_DMA_XMIT_Write(0);		//No transmission
	DE_Write(0);
	NOT_RE_Write(0);
	
	T2_RESET_Write(1);	
}

//General ADC - we get here after N samples have been taken
void isr_sar1_dma_Interrupt_InterruptCallback()
{
	volatile int8_t ch = 0;
	volatile static int8_t chMem[10];
	volatile static uint8_t idx = 0;
	
	//What was the last channel?
	ch = AMuxSeq_1_GetChannel();
	if(ch < 0 || ch >= ADC1_CHANNELS)
		ch = 0;	//Error

	//Debugging:
	chMem[idx] = ch;
	idx++;
	idx %= 10;
	
	adc1_res[ch][0] = adc_sar1_dma_array[0];
	adc1_res[ch][1] = adc_sar1_dma_array[1];
	adc1_res[ch][2] = adc_sar1_dma_array[2];
	adc1_res[ch][3] = adc_sar1_dma_array[3];
	
	//Next:
	AMuxSeq_1_Next();

	adc_sar1_flag = 1;
}

//Current sensing:
static uint8 update_current_flag = 0;
void isr_sar2_dma_Interrupt_InterruptCallback()
{	
	#if((MOTOR_COMMUT == COMMUT_BLOCK) && (CURRENT_SENSING == CS_LEGACY))
		
		volatile int32 adc_sum = 0;
		volatile int32 adc_avg = 0;
			
		//Read last ADC value
		adc_sum = (int32)(adc_dma_array[0] + adc_dma_array[1] + adc_dma_array[2] + \
					adc_dma_array[3] + adc_dma_array[4]);
		adc_avg = (adc_sum / 5);
		
		ctrl[0].current.actual_val = (int32)(adc_avg - CURRENT_ZERO);	
		//Used by the current controller, 0 centered.
			
		if((ctrl[0].active_ctrl == CTRL_CURRENT) || (ctrl[0].active_ctrl == CTRL_IMPEDANCE))
		{
			//Current controller
			motor_current_pid_2(ctrl[0].current.setpoint_val, ctrl[0].current.actual_val);
		}
	
	#else		
		update_current_flag = 1;		
	#endif
}

void isr_dma_uart_rx_Interrupt_InterruptCallback()
{
	//Update rx_buf with the latest DMA data:
	//unwrap_buffer(uart_dma_rx_buf, uart_dma_rx_buf_unwrapped, 48);
	//update_rx_buf_array_485(uart_dma_rx_buf_unwrapped, 48);		//ToDo shouldn't be harcoded. Buffer name?
	update_rx_buf_485(uart_dma_rx_buf, 48);
	commPeriph[PORT_RS485_1].rx.bytesReadyFlag++;
}

void isr_dma_uart_tx_Interrupt_InterruptCallback()
{
	T2_RESET_Write(0);
	Timer_2_Start();		
}

void isr_dma_uart_bt_rx_Interrupt_InterruptCallback()
{
	//Update rx_buf with the latest DMA data:
	//unwrap_buffer(uart_dma_bt_rx_buf, uart_dma_bt_rx_buf_unwrapped, 48);
	//update_rx_buf_array_wireless(uart_dma_bt_rx_buf_unwrapped, 48);		//ToDo shouldn't be harcoded. Buffer name?
	update_rx_buf_wireless(uart_dma_bt_rx_buf, 48);
	commPeriph[PORT_WIRELESS].rx.bytesReadyFlag++;
}

void isr_delsig_Interrupt_InterruptCallback()
{
	ADC_DelSig_1_StopConvert();
	adc_delsig_flag = 1;	
}

void ADC_SAR_1_ISR_InterruptCallback()
{
	//Not used anymore
}

//SPI - AS5047 position sensor
void isr_spi_tx_Interrupt_InterruptCallback()
{
	#ifdef USE_AS5047

	//static volatile uint16 frame_errors = 0, parity_errors = 0, man_test = 0;
	volatile uint8_t tx_status_isr = 0;
	static uint16 counter = 0;
	static uint8 velcounter = 0;
	
	//Read status to clear flag:
	tx_status_isr = SPIM_1_ReadTxStatus();
	
	if(spi_isr_state < SPI_TX_MAX_INDEX)
	{
		//Read last word received:
		spidata_miso[spi_isr_state] = SPIM_1_ReadRxData();
		//Next transfer:
		spi_isr_state++;
		//SPIM_1_WriteTxData(as5047_empty_read);
		CY_SET_REG16(SPIM_1_TXDATA_PTR, as5047_empty_read);
	}
	else
	{
		//Transfer complete, decode answer:
		spidata_miso[spi_isr_state] = SPIM_1_ReadRxData();
		as5047_angle = (spidata_miso[spi_isr_state] & 0x3FFF);
		spi_read_flag = 1;
		update_as504x_absang(as5047_angle, &as5047);
		sensor_sin_commut(as5047.ang_comp_clks >> 3, exec1.sine_commut_pwm);

		if(update_current_flag)
		{
			update_current_arrays();
			update_current_flag = 0;
		}
		
		if(velcounter >= 20)
		{
			//Encoder velocity estimation:
			//update_as504x_contang(&as5047);
			update_as504x_vel(&as5047);
			velcounter = 0;		   
		}
		velcounter++;

		if(counter < 20000)
		{
			if(counter > 14000){set_current_zero();}
			counter++;
		}
		
		/* Partially developped error testing code:
		
		//Error in last command frame?
		if(spidata_miso[spi_isr_state] & AS5047_ERR_FRAME)
		{
			frame_errors++;
		}
		
		if(as5047_angle == 0)
		{
			man_test++;
			//EX15_Write(1);
			//EX15_Write(0);
		}
		//Wrong parity?		
		//...
		*/		
	}   
	#endif	//USE_AS5047
	//EX2_Write(0);
}

//Interrupt triggers when PWM A reloads
void isr_mot_Interrupt_InterruptCallback()
{
	//Encoder, sine commutation:
	#if(MOTOR_COMMUT == COMMUT_SINE) 

		#if(ENC_COMMUT == ENC_AS5047)
			//Start reading, result via ISR 
			as5047_read_single_isr(AS5047_REG_ANGLECOM); 
		#endif //ENC_AS5047

	#endif	//(MOTOR_COMMUT == COMMUT_SINE)
}
