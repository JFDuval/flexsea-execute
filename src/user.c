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
	[This file] user: User Projects & Functions
*****************************************************************************
	[Change log] (Convention: YYYY-MM-DD | author | comment)
	* 2016-09-29 | jfduval | Released under GPL-3.0 release
	*
****************************************************************************/

/*Important: we reached a point where we couldn't support all configurations
  without changing the TopDesign (we ran out of ressources). You might have
  to select a different TopDesign file than the one included by default (check
  the folded, there is more than one included) */

//****************************************************************************
// Include(s)
//****************************************************************************

#include "main.h"
#include "user.h"

//****************************************************************************
// Variable(s)
//****************************************************************************

//****************************************************************************
// Private Function Prototype(s):
//****************************************************************************

static void init_barebone(void);

//****************************************************************************
// Public Function(s)
//****************************************************************************

//Initialization function - call once in main.c, before while()
void init_user(void)
{	
	//Barebone:
	#if(ACTIVE_PROJECT == PROJECT_BAREBONE)
	init_barebone();
	#endif	//PROJECT_EXOCUTE
	
	//ExoBoots:
	#if(ACTIVE_PROJECT == PROJECT_EXOCUTE)
	init_exo();
	#endif	//PROJECT_EXOCUTE
	
	//MIT CSEA Knee:
	#if(ACTIVE_PROJECT == PROJECT_CSEA_KNEE)
	init_csea_knee();
	#endif	//PROJECT_CSEA_KNEE
	
	//RIC/NU Knee:
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
	init_ricnu_knee();
	#endif	//PROJECT_RICNU_KNEE
	
	//MIT Ankle 2-DoF:
	#if(ACTIVE_PROJECT == PROJECT_ANKLE_2DOF)
	init_ankle_2dof();
	#endif	//PROJECT_ANKLE_2DOF
	
	//MIT d'Arbeloff Dual-Speed Dual-Motor:
	#if(ACTIVE_PROJECT == PROJECT_DSDM)
	init_dsdm();
	#endif	//PROJECT_DSDM
	
	//Dephy's Gravity Boot v1:
	#if(ACTIVE_PROJECT == PROJECT_DEPHY_GRAVITY_1)
	init_gravity_1();
	#endif	//PROJECT_DEPHY_GRAVITY_1  
}

//Call this function in one of the main while time slots.
void user_fsm(void)
{
	//ExoBoot code
	#if(ACTIVE_PROJECT == PROJECT_EXOCUTE)
	exo_fsm();
	#endif	//PROJECT_EXOCUTE
	
	//CSEA Knee code
	#if(ACTIVE_PROJECT == PROJECT_CSEA_KNEE)
	csea_knee_fsm();
	#endif	//PROJECT_CSEA_KNEE
	
	//RIC/NU Knee code
	#if(ACTIVE_PROJECT == PROJECT_RICNU_KNEE)
		ricnu_knee_fsm();
	#endif	//PROJECT_RICNU_KNEE
	
	//MIT d'Arbeloff Dual-Speed Dual-Motor:
	#if(ACTIVE_PROJECT == PROJECT_DSDM)
	dsdm_fsm();
	#endif	//PROJECT_DSDM
	
	//Dephy's Gravity Boot v1:
	#if(ACTIVE_PROJECT == PROJECT_DEPHY_GRAVITY_1)
	gravity_1_fsm();
	#endif	//PROJECT_DEPHY_GRAVITY_1  
}

//****************************************************************************
// Private Function(s)
//****************************************************************************

static void init_barebone(void)
{
	//Barebone:
	#if(ACTIVE_PROJECT == PROJECT_BAREBONE)
	board_id = SLAVE_ID;
	#endif	//PROJECT_BAREBONE
}
