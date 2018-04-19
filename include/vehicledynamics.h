//////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: vehicledynamics.h,v 1.10 2012/04/13 17:54:42 iowa\dheitbri Exp $
//
// Author(s):   Gustavo Ordaz Hernandez
// Date:        August, 1999
//
// Description:	Vehicle dynamics interface functions.
//
//////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#pragma warning(disable:4786)
#endif

#ifndef __VEHICLE_DYNAMICS_H
#define __VEHICLE_DYNAMICS_H

#include "cvedpub.h"
#include "cvedstrc.h"
#include "objlayout.h"

using namespace CVED;

void 
InitFourWheelVehState( 
	CCved&                     cved,
	cvTObjAttr&                attr,
	cvTObjState::VehicleState* pState
	);

void 
DoVehicleDynamics(
	int                    cvedId,
	CCved&                 cved,	
	double                  delta,
	const cvTObjAttr*      pAttr,
	const TVehicleState*   pCurState,
	const TVehicleContInp* pContInp,
	TVehicleState*         pFutState,
    bool updateSteering    = true
	);

#endif	// __VEHICLE_DYNAMICS_H
