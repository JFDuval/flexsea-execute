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
	[This file] flexsea_board: configuration and functions for this 
	particular board
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

#ifndef INC_FLEXSEA_BOARD_H
#define INC_FLEXSEA_BOARD_H

//****************************************************************************
// Include(s)
//****************************************************************************

#include <stdint.h>
#include <flexsea.h>
#include <flexsea_comm.h>
	
//****************************************************************************
// Prototype(s):
//****************************************************************************

void flexsea_send_serial_slave(PacketWrapper* p);
void flexsea_send_serial_master(PacketWrapper* p);
void flexsea_receive_from_master(void);
uint8_t getBoardID(void);
void setBoardID(uint8_t bid);
uint8_t getBoardUpID(void);
uint8_t getBoardSubID(uint8_t sub, uint8_t idx);
uint8_t getSlaveCnt(uint8_t sub);

uint8_t getDeviceId();
uint8_t getDeviceType();

//****************************************************************************
// Definition(s):
//****************************************************************************

//<FlexSEA User>
//==============

//How many slave busses?
#define COMM_SLAVE_BUS				2

//How many slaves on this bus?
#define SLAVE_BUS_1_CNT				0
#define SLAVE_BUS_2_CNT				0
//Note: only Manage can have a value different than 0 or 1

//How many possible masters?
#define COMM_MASTERS				3

//Overload buffer & function names (for user convenience):

//#define comm_str_485_1					comm_str_1
//#define unpack_payload_485				unpack_payload_1
//#define rx_command_485					rx_command_1
//#define update_rx_buf_byte_485			update_rx_buf_byte_1
//#define update_rx_buf_array_485			update_rx_buf_array_1
#define update_rx_buf_485(x, y)			circ_buff_write(commPeriph[PORT_RS485_1].rx.circularBuff, (x), (y))

//#define comm_str_usb					comm_str_2
//#define unpack_payload_usb				unpack_payload_2
//#define rx_command_usb					rx_command_2
//#define update_rx_buf_byte_usb			update_rx_buf_byte_2
//#define update_rx_buf_array_usb			update_rx_buf_array_2
#define update_rx_buf_usb(x, y)			circ_buff_write(commPeriph[PORT_USB].rx.circularBuff, (x), (y))

//#define comm_str_wireless				comm_str_3
//#define unpack_payload_wireless			unpack_payload_3
//#define rx_command_wireless				rx_command_3
//#define update_rx_buf_byte_wireless		update_rx_buf_byte_3
//#define update_rx_buf_array_wireless	update_rx_buf_array_3
#define update_rx_buf_wireless(x, y)		circ_buff_write(commPeriph[PORT_WIRELESS].rx.circularBuff, (x), (y))

//===============
//</FlexSEA User>

//****************************************************************************
// Structure(s):
//****************************************************************************

//****************************************************************************
// Shared variable(s)
//****************************************************************************

extern PortType masterSlave[NUMBER_OF_PORTS];

#endif	//INC_FLEXSEA_BOARD_H
