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
	[This file] analog: ADC configurations, read & filter functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/
	
#ifndef INC_ANALOG_H
#define INC_ANALOG_H

//****************************************************************************
// Include(s)
//****************************************************************************	
	
#include "main.h"
#include <flexsea_user_structs.h>
		
//****************************************************************************
// Prototype(s):
//****************************************************************************

void init_analog(void);
uint16 adc_avg8(uint16 new_data);
void filter_sar_adc(void);
int16 read_analog(uint8_t ch);
void adc_sar1_dma_config(void);
void adc_sar2_dma_config(void);
void double_buffer_adc(void);

//****************************************************************************
// Definition(s):
//****************************************************************************

//General ADC:
#define ADC1_CHANNELS				4
#define ADC1_BUF_LEN				4
#define ADC1_SHIFT					2
//Shift is used for averaging, match with BUF_LEN

//DMA ADC SAR 1 (General)
#define DMA_5_BYTES_PER_BURST 		2
#define DMA_5_REQUEST_PER_BURST 	1
#define DMA_5_SRC_BASE 				(CYDEV_PERIPH_BASE)
#define DMA_5_DST_BASE 				(CYDEV_SRAM_BASE)

//****************************************************************************
// Shared variable(s)
//****************************************************************************	
	
volatile extern uint16 adc1_res[ADC1_CHANNELS][ADC1_BUF_LEN];
volatile extern uint16 adc1_res_filtered[ADC1_CHANNELS];
volatile extern uint16 adc1_dbuf[ADC1_CHANNELS][ADC1_BUF_LEN];
extern volatile uint16 adc_sar1_dma_array[ADC1_BUF_LEN + 1];

//****************************************************************************
// Structure(s):
//****************************************************************************

#endif	//INC_ANALOG_H
