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
	[Lead developper] Jean-Francois Duval, jfduval at dephy dot com.
	[Origin] Based on Jean-Francois Duval's work at the MIT Media Lab 
	Biomechatronics research group <http://biomech.media.mit.edu/>
	[Contributors]
*****************************************************************************
	[This file] usb: USB CDC
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "user-ex.h"

#ifdef USE_USB

#include "main.h"
#include "usb.h"
#include "flexsea_board.h"
#include <flexsea_comm.h>
#include <flexsea_comm_multi.h>

//****************************************************************************
// Variable(s)
//****************************************************************************

uint8_t buffer[RX_BUF_LEN];
uint8_t usbConnected = 0;

//****************************************************************************
// Function(s)
//****************************************************************************

//Initialize the USB peripheral
//Returns 0 is success, 1 if timeout (happens when the cable is unplugged)
uint8_t init_usb(void)
{
	uint16 cnt = 0, flag = 0;
	
	//Start USBFS Operation with 5V operation
	USBUART_1_Start(0u, USBUART_1_5V_OPERATION);
	
	//Wait for Device to enumerate
	for(cnt = 0; cnt < USB_ENUM_TIMEOUT; cnt++)
	{
		if(USBUART_1_GetConfiguration())
		{
			flag = 1;
			break;
		}
		CyDelay(1);
	}

	if(flag)
	{
		//Enumeration is done, enable OUT endpoint for receive data from Host
		USBUART_1_CDC_Init();
		usbConnected = 1;
		return 1;		//Success
	}
	
	return 0;	//Timeout
}

//Call this function periodically to see if USB is ready to be connected.
void usbRuntimeConnect(void)
{
	//Skip if it's already working:
	if(usbConnected) {return;}
	
	if(USBUART_1_GetConfiguration())
	{
		//Enumeration is done, enable OUT endpoint for receive data from Host
		USBUART_1_CDC_Init();
		usbConnected = 1;
	}
}

void get_usb_data(void)
{
	static 	int16 count = 0;
	
	//USB Data
	if(USBUART_1_DataIsReady() != 0u)			   	//Check for input data from PC
	{
		count = USBUART_1_GetAll(buffer);		   	//Read received data and re-enable OUT endpoint
		if(count != 0u)
		{
			//Store all bytes in rx buf:
			//update_rx_buf_usb(buffer, count+1);
			//commPeriph[PORT_USB].rx.bytesReadyFlag++;
			copyIntoMultiPacket(comm_multi_periph + PORT_USB, buffer, count+1);
		}
	}
}

//Sends a fixed length packet over USB. Discarded if USB isn't ready.
uint8_t usb_puts(uint8_t *buf, uint32 len)
{
	if(USBUART_1_CDCIsReady() != 0)
	{
		USBUART_1_PutData(( const uint8_t*)buf, len);
		return 1;
	}
	
	return 0;
}

#endif	//USE_USB
