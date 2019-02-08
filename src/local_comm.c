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
	[This file] local_comm: Communication functions, board specific
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2017-03-06 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "local_comm.h"
#include "serial.h"
#include "misc.h"
#include <flexsea_payload.h>
#include <flexsea_board.h>
#include "flexsea_cmd_stream.h"
#include "flexsea_sys_def.h"
#include <flexsea_comm.h>
#include "flexsea_comm_multi.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************	

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Prepares the structures:
void initLocalComm(void)
{
	uint8_t i;
	for(i = 0; i < NUMBER_OF_PORTS; i++)
	{
		//Multi-packet:
		//initMultiPeriph(comm_multi_periph+i, i, masterSlave[i]);

		//Single packets:
		initCommPeriph(&commPeriph[i], i, masterSlave[i], \
				comm_str[i], rx_command[i], &rx_buf_circ[i], \
				&packet[i][INBOUND], &packet[i][OUTBOUND]);
	}
}

//Did we receive new commands? Can we parse them?
void parseMasterCommands(uint8_t *new_cmd)
{
	uint8_t parseResult = 0, newCmdLed = 0;
	
	//RS-485
	if(commPeriph[PORT_RS485_1].rx.unpackedPacketsAvailable > 0)
	{
		commPeriph[PORT_RS485_1].rx.unpackedPacketsAvailable = 0;
		parseResult = payload_parse_str(&packet[PORT_RS485_1][INBOUND]);
		newCmdLed += (parseResult == PARSE_SUCCESSFUL) ? 1 : 0;
		CyDmaClearPendingDrq(DMA_3_Chan);
	}

	//USB
	if(commPeriph[PORT_USB].rx.unpackedPacketsAvailable > 0)
	{
		commPeriph[PORT_USB].rx.unpackedPacketsAvailable = 0;
		parseResult = payload_parse_str(&packet[PORT_USB][INBOUND]);
		newCmdLed += (parseResult == PARSE_SUCCESSFUL) ? 1 : 0;
	}

	//Wireless
	if(commPeriph[PORT_WIRELESS].rx.unpackedPacketsAvailable > 0)
	{
		commPeriph[PORT_WIRELESS].rx.unpackedPacketsAvailable = 0;
		parseResult = payload_parse_str(&packet[PORT_WIRELESS][INBOUND]);
		newCmdLed += (parseResult == PARSE_SUCCESSFUL) ? 1 : 0;
	}

	if(newCmdLed > 0) {*new_cmd = 1;}
}

//Call this to send any pending delayed reply on RS-485
void sendMasterDelayedResponse(void)
{
	Port port = PORT_RS485_1;
	
	if(commPeriph[port].tx.packetReady == 1)
	{
		//We never replied in the same time slot:
		if(t1_time_share == commPeriph[port].tx.timeStamp)
		{
			//rs485_puts(reply_ready_buf, reply_ready_len);
			rs485_puts(packet[port][OUTBOUND].packed, \
						packet[port][OUTBOUND].numb);
			
			//Drop flag
			commPeriph[port].tx.packetReady = 0;
		}		
	}
}

uint8_t isMultiAutoStream(uint8_t cmdCode) {
	return cmdCode == CMD_SYSDATA;
}

static MultiPacketInfo pInfo;
static int sinceLastStreamSend[MAX_STREAMS] = {0};

void autoStream(void)
{

	if(isStreaming)
	{
		int i;
		for(i = 0; i < isStreaming; i++)
		{
			sinceLastStreamSend[i]++;
		}

		for(i = 0; i < isStreaming; i++)
		{
			if(sinceLastStreamSend[i] >= streamPeriods[i])
			{
				if(isMultiAutoStream(streamCmds[i]))
				{

					MultiCommPeriph *cp = comm_multi_periph + streamPortInfos[i];
					pInfo.xid = streamReceivers[i];
					pInfo.rid = getDeviceId();
					pInfo.portIn = streamPortInfos[i];

					// following line is a bad-practice-band-aid!
					// we should enforce that auto-streamable commands do not read from the unpacked buffer
					// this line forces sysdata to respond with data and not metadata regardless of the status of the comm periph
					// TODO: resolvable by changing sysdata metadata to be a response to a write command instead of a read command
					cp->in.unpacked[0] = 0;
					uint8_t error = receiveAndFillResponse(streamCmds[i], RX_PTYPE_READ, &pInfo, cp);
					if(error)
						cp->out.unpackedIdx = 0;

				}
				else
				{

					//Determine what offset to use:
					streamCurrentOffset[i]++;
					if(streamCurrentOffset[i] > streamIndex[i][1])
					{
						streamCurrentOffset[i] = streamIndex[i][0];
					}

					uint8_t cp_str[256] = {0};
					cp_str[P_XID] = streamReceivers[i];
					cp_str[P_DATA1] = streamCurrentOffset[i];
					(*flexsea_payload_ptr[streamCmds[i]][RX_PTYPE_READ]) (cp_str, &streamPortInfos[i]);
				}

				sinceLastStreamSend[i] -= streamPeriods[i];

				//we return to avoid sending two msgs in one cycle
				//since counters were already incremented, we will still try to hit other stream frequencies
				return;
			}
		}
	}
}

//****************************************************************************
// Private Function(s)
//****************************************************************************
