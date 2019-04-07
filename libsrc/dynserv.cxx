//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:		$Id: dynserv.cxx,v 1.68 2018/07/16 14:06:26 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		January, 1999
//
// Implementation of dynamic model functions for the various
// object types.
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"
#include "vehicledynamics.h"
#include "odeDynamics.h"

const double FEET_TO_METERS = 0.3048;

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: Dynamic model for the vehicle object.
//
// Remarks:  This object is an autonomous vehicle and, therefore, this
//   function calls DoVehicleDynamics.  
//
// Arguments:
//	 cCved - The CCved instance.
//   delta  - The time increment.
//   cpAttr - Pointer to object's attributes.
//   pCurState - Pointer to object's current state.
//   cpContInp  - Pointer to object's control inputs.
//   pFutState - Pointer to object's future state.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
static void
VehicleDynamics(
	int                    cvedId,
	CCved&                 cCved,
	double                 delta,
	const cvTObjAttr*      cpAttr,
	const TVehicleState*   cpCurState,
	const TVehicleContInp* cpContInp,
	TVehicleState*         pFutState,
    bool                   updateSteering
	)
{

	DoVehicleDynamics(
				cvedId,
				cCved, 
				delta, 
				cpAttr,
				cpCurState, 
				cpContInp, 
				pFutState,
                updateSteering
				);
	//For external driver/ODE interactions not ready yet.
	//cCved.SetupVehODEObject(cpAttr, cvedId, cCved, pFutState, false );
} // end of VehicleDynamics

//////////////////////////////////////////////////////////////////////////////
//
// Description: TrajFollowerDynamics
// 	Dynamic model for trajectory followers
//
// Remarks: This function implements a simple integrator model that linearly 
// 	extrapolates the state of an object based on its current position and 
// 	velocity.
//
// Arguments:
//  cvedId - the cved id of the object
//	cCved - the CCved instance
//  delta  - the time increment
//  cpAttr - pointer to object's attributes
//  cpCurState - pointer to object's current state
//  cpContInp  - pointer to object's control inputs
//  cParentType - parent object type
//  cpParentState - parent object state
//  pFutState - pointer to object's future state
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
static void
TrajFollowerDynamics(
	int                                       cvedId,
	CCved&                                    cCved,
	double                                    delta,
	const cvTObjAttr*                         cpAttr,
	const cvTObjState::TrajFollowerState*     cpCurState,
	const cvTObjContInp::TrajFollowerContInp* cpContInp,
	const cvEObjType						  cParentType,
	const cvTObjState*	                      cpParentState,
	cvTObjState::TrajFollowerState*           pFutState
	)
{
	int i;
	unsigned char mode;
	bool DebugFlag = false;
	bool DebugModeFlag = false;

	if ( DebugFlag ) {

		gout << "->Enter TrajFollower dynamics " << endl;
		gout << "Obj len/wid: " << cpAttr->xSize << ", ";
		gout << cpAttr->ySize << " delta = " << delta << endl;
		gout << "  CurState: pos= " << cpCurState->position << endl;
		gout << "            tang = " << cpCurState->tangent;
		gout << " lat = " << cpCurState->lateral << endl;

		gout << "  ContInputs: targVel=" << cpContInp->targVel;
		gout << ", targDir=" << cpContInp->targDir << endl;

	}

	mode = cpCurState->curMode; // use the previous mode 

	// check to see if the object mode has just been transitioned
	if ( cpCurState->curMode != cpCurState->prevMode )
	// recent transition
	{
		bool noTransition = false;

		switch ( cpCurState->curMode ) {
		case eCV_FREE_MOTION:
		{
			// initialize position, rotation and velocities
			// right now just copy from the current values of the object 
			// or use the pre-assigned values 
			double initPosRot[6], initVel[6], angVel_g[3];

			if ( !cpCurState->useAsAbsoluteValue ) 
			{
				// values have not been pre-assigned, calculate them now
				initPosRot[0] = cpCurState->position.x;
				initPosRot[1] = cpCurState->position.y;
				initPosRot[2] = cpCurState->position.z;
				initPosRot[3] = 
					atan2( -cpCurState->lateral.k, cpCurState->lateral.i * cpCurState->tangent.j 
					- cpCurState->lateral.j * cpCurState->tangent.i ); // roll
				initPosRot[4] = asin( cpCurState->tangent.k );  // pitch
				initPosRot[5] = atan2( cpCurState->tangent.j, cpCurState->tangent.i ); // yaw

				switch ( cpCurState->prevMode ) {
				case eCV_COUPLED_OBJ:
				case eCV_OBJ_REL_TRAJ:
				{
					TVector3D parentUp; 

					parentUp.i = cpParentState->anyState.lateral.j * cpParentState->anyState.tangent.k - 
						cpParentState->anyState.lateral.k * cpParentState->anyState.tangent.j;
					parentUp.j = cpParentState->anyState.lateral.k * cpParentState->anyState.tangent.i - 
						cpParentState->anyState.lateral.i * cpParentState->anyState.tangent.k;
					parentUp.k = cpParentState->anyState.lateral.i * cpParentState->anyState.tangent.j - 
						cpParentState->anyState.lateral.j * cpParentState->anyState.tangent.i;

					double velTan, velLat, velNorm, rollRate, pitchRate, yawRate;

					if ( cpCurState->prevMode == eCV_COUPLED_OBJ )
					// Convert the current velocities of the parent vehicle, which are in 
					// the local coordinate system of the vehicle, to global coordinates
					{
						if ( cParentType == eCV_VEHICLE )
						// parent is a vehicle, all the velocities are available. 
						// Add angular velocity to simulate object rolling off the edge of the 
						// parent. Assume it rolls off the back edge of the parent, then this 
						// corresponds to a positive value added to pitch rate ( negative angluar 
						// velocity about y axis ). It should be assigned in init velocities 
						// during authorization.
						{
							velTan = cpParentState->vehicleState.vehState.vel + cpCurState->initVel[0];
							velLat = cpParentState->vehicleState.vehState.velLat - cpCurState->initVel[1];
							velNorm = cpParentState->vehicleState.vehState.velNorm + cpCurState->initVel[2];
							rollRate = cpParentState->vehicleState.vehState.rollRate + cpCurState->initVel[3];
							pitchRate = cpParentState->vehicleState.vehState.pitchRate + cpCurState->initVel[4]; 
							yawRate = cpParentState->vehicleState.vehState.yawRate + cpCurState->initVel[5]; 
						}
						else
						// parent is not a vehicle and only has tangent velocity. No lateral, normal,
						// or angular velocities are available. Assume them to be 0.
						{
							velTan = cpParentState->vehicleState.vehState.vel * FEET_TO_METERS + cpCurState->initVel[0];
							velLat = -cpCurState->initVel[1];
							velNorm = cpCurState->initVel[2];
							rollRate = cpCurState->initVel[3];
							pitchRate = cpCurState->initVel[4]; 
							yawRate = cpCurState->initVel[5]; 
						}
					}
					else 
					// Object relative trajectory mode.
					// Initial linear velocity is the velocity along the relative path plus 
					// the linear velocity of the parent. Initial angular velocity is the 
					// angular velocity of the parent, plus a negative pitch rate to simulate 
					// the object rolling of the edge of the parent, as in the coupled obj 
					// mode.
					{
						if ( cParentType == eCV_VEHICLE )
						{
							velTan = cpParentState->vehicleState.vehState.vel + 
								cpContInp->targDir.i * cpCurState->vel + cpCurState->initVel[0];
							velLat = cpParentState->vehicleState.vehState.velLat - 
								cpContInp->targDir.j * cpCurState->vel - cpCurState->initVel[1];
							velNorm = cpParentState->vehicleState.vehState.velNorm + cpCurState->initVel[2];
							rollRate = cpParentState->vehicleState.vehState.rollRate + cpCurState->initVel[3];
							pitchRate = cpParentState->vehicleState.vehState.pitchRate + cpCurState->initVel[4]; 
							yawRate = cpParentState->vehicleState.vehState.yawRate + cpCurState->initVel[5]; 
						}
						else
						{
							velTan = cpParentState->vehicleState.vehState.vel * FEET_TO_METERS +
								cpContInp->targDir.i * cpCurState->vel + cpCurState->initVel[0];
							velLat = -cpContInp->targDir.j * cpCurState->vel - cpCurState->initVel[1];
							velNorm = cpCurState->initVel[2];
							rollRate = cpCurState->initVel[3];
							pitchRate = cpCurState->initVel[4]; 
							yawRate = cpCurState->initVel[5]; 
						}
					}

					// V_gl = v_x * TAN_p - v_y * LAT_p + v_z * UP_p
					//      = v_tan * TAN_p + v_lat * LAT_p + v_norm * UP_p 
					initVel[0] = 
						velTan * cpParentState->anyState.tangent.i + 
						velLat * cpParentState->anyState.lateral.i + 
						velNorm * parentUp.i;
					initVel[1] =  
						velTan * cpParentState->anyState.tangent.j + 
						velLat * cpParentState->anyState.lateral.j + 
						velNorm * parentUp.j;
					initVel[2] = 
						velTan * cpParentState->anyState.tangent.k + 
						velLat * cpParentState->anyState.lateral.k + 
						velNorm * parentUp.k;

					// V_ga = v_rx * TAN_p -v_ry * LAT_p + v_rz * UP_p
					//      = rollRate * TAN_p + pitchRate * LAT_p + yawRate * UP_p
					angVel_g[0] =
						rollRate * cpParentState->anyState.tangent.i + 
						pitchRate * cpParentState->anyState.lateral.i + 
						yawRate * parentUp.i;
					angVel_g[1] =
						rollRate * cpParentState->anyState.tangent.j + 
						pitchRate * cpParentState->anyState.lateral.j + 
						yawRate * parentUp.j;
					angVel_g[2] =
						rollRate * cpParentState->anyState.tangent.k + 
						pitchRate * cpParentState->anyState.lateral.k + 
						yawRate * parentUp.k;
					
					/*
					double rotMatI[3][3]; // Inverse matrix of the global rotation , R^i = R^t
					                      // Used to convert global angular velocity 
					                      // to body frame angular velocity
					double cr, cp, cy, sr, sp, sy;

					cr = cos( initPosRot[3] ); 
					sr = sin( initPosRot[3] );
					cp = cos( initPosRot[4] ); 
					sp = sin( initPosRot[4] );
					cy = cos( initPosRot[5] ); 
					sy = sin( initPosRot[5] );
					rotMatI[0][0] = cy * cp;
					rotMatI[1][0] = -sy * cr - cy * sp *sr;
					rotMatI[2][0] = sy * sr - cy * sp *cr;
					rotMatI[0][1] = sy * cp;
					rotMatI[1][1] = cy * cr - sy * sp * sr;
					rotMatI[2][1] = -cy * sr - sy * sp * cr;
					rotMatI[0][2] = sp;
					rotMatI[1][2] = cp * sr;
					rotMatI[2][2] = cp * cr;

					initVel[3] = rotMatI[0][0] * angVel_g[0] + rotMatI[0][1] * angVel_g[1] +
						rotMatI[0][2] * angVel_g[2];
					initVel[4] = -(rotMatI[1][0] * angVel_g[0] + rotMatI[1][1] * angVel_g[1] +
						rotMatI[1][2] * angVel_g[2]);  // negate to reverse y angular velocity to pitch rate
					initVel[5] = rotMatI[2][0] * angVel_g[0] + rotMatI[2][1] * angVel_g[1] +
						rotMatI[2][2] * angVel_g[2];
					*/
					initVel[3] = angVel_g[0];
					initVel[4] = -angVel_g[1];
					initVel[5] = angVel_g[2];

					if ( DebugModeFlag )
					{
						printf("FM Object #%d Init posrot %.4f %.4f %.4f %.4f %.4f %.4f vels %.4f %.4f %.4f %.4f %.4f %.4f\n", 
							cvedId,
							initPosRot[0], initPosRot[1], initPosRot[2], 
							initPosRot[3], initPosRot[4], initPosRot[5],
							initVel[0], initVel[1], initVel[2],
							initVel[3], initVel[4], initVel[5] );
						printf("AV w.r.t. parent (%.4f %.4f %.4f) w.r.t. world (%.4f %.4f %.4f)\n",
							rollRate, -pitchRate, yawRate, angVel_g[0], angVel_g[1], angVel_g[2]);
					}
					
					break;
				}
				case eCV_GROUND_TRAJ:
				{
					TVector3D up; 
					double velTan, velLat, velNorm;

					up.i = cpCurState->lateral.j * cpCurState->tangent.k - 
						cpCurState->lateral.k * cpCurState->tangent.j;
					up.j = cpCurState->lateral.k * cpCurState->tangent.i - 
						cpCurState->lateral.i * cpCurState->tangent.k;
					up.k = cpCurState->lateral.i * cpCurState->tangent.j - 
						cpCurState->lateral.j * cpCurState->tangent.i;

					velTan = cpCurState->vel * FEET_TO_METERS + cpCurState->initVel[0];
					velLat = -cpCurState->initVel[1];
					velNorm = cpCurState->initVel[2];

					initVel[0] = 
						velTan * cpCurState->tangent.i + 
						velLat * cpCurState->lateral.i + 
						velNorm * up.i;
					initVel[1] =  
						velTan * cpCurState->tangent.j + 
						velLat * cpCurState->lateral.j + 
						velNorm * up.j;
					initVel[2] = 
						velTan * cpCurState->tangent.k + 
						velLat * cpCurState->lateral.k + 
						velNorm * up.k;

					initVel[3] = 
						cpCurState->initVel[3] * cpCurState->tangent.i + 
						cpCurState->initVel[4] * cpCurState->lateral.i + 
						cpCurState->initVel[5] * up.i;
					initVel[4] = 
						-(cpCurState->initVel[3] * cpCurState->tangent.j + 
						cpCurState->initVel[4] * cpCurState->lateral.j + 
						cpCurState->initVel[5] * up.j); // pitch rate, negative of vel_roty
					initVel[5] = 
						cpCurState->initVel[3] * cpCurState->tangent.k + 
						cpCurState->initVel[4] * cpCurState->lateral.k + 
						cpCurState->initVel[5] * up.k;

					break;
				}
				default:
					// should not happen
					for ( i=0; i<6; ++i )
						initVel[i] = 0;
					break;
				}
			}
			else
			// use initial pos, rot and velocities stored in the state as absolute values
			{
				for ( i=0; i<6; ++i )
				{
					initVel[i] = cpCurState->initVel[i];
					initPosRot[i] = cpCurState->offset[i];
				}
			}

			CODEObject* pOdeObj = new CODEObject;
			double cgoffset[3] = {0, 0, 0}, 
				dimension[3] = {cpAttr->xSize, cpAttr->ySize, cpAttr->zSize},
				visOrigOffset[3] = {0, 0, -cpAttr->zSize*0.5}; // assuming the vis model origin is at the bottom
			odePublic::SBodyProps bodyProps;
			bodyProps.cvedId = cvedId;
			bodyProps.solId = cpAttr->solId;
			bodyProps.frictionCoeff = cCved.GetSol().GetObj(cpAttr->solId)->GetFrictionCoeff();
			bodyProps.bounceEnergyLoss = cCved.GetSol().GetObj(cpAttr->solId)->GetBounceEnergyLoss();
			bodyProps.contactSoftCFM = (dimension[2]>3.1)?1e-03:1e-10;
			double mass = (dimension[2]>3.1)?1200:80;
			odePublic::EODEObjType type;
			bodyProps.cigiId = cCved.GetSol().GetObj(cpAttr->solId)->GetVisModelCigiId();
			// Temporary code, use cigi id to identify ode object type.
			// In the future a field should be added to the sol obj to identify its type
			switch ( bodyProps.cigiId ) {
			// box 
			case 512:  // suit case 
			case 514:  // desk
				type = odePublic::e_BOX;
				break;
			// sphere 
			case 513:  // ball
				type = odePublic::e_SPHERE;
				mass = 1.0;
				break;
			// cylinder
			case 509:  // traf cone 12"
				type = odePublic::e_CYLINDER;
				mass = 2.0;
				break;
			case 500:  // traf cone
			case 510:  // traf cone 18"
				type = odePublic::e_CYLINDER;
				mass = 4.5;
				break;
			case 515:  // traf cone 36"
				type = odePublic::e_CYLINDER;
				mass = 18.0;
				break;
			case 505:  // drum
				type = odePublic::e_COMPOSITE;
				mass = 10.0;
				break;
			case 508:  // crash barrel
				type = odePublic::e_CAPSULE;
				mass = 100.0;
				break;
			default:
				type = odePublic::e_BOX;
			}

			cCved.CreateODEObject( type, pOdeObj, initPosRot, initVel, 
				mass, cgoffset, dimension, visOrigOffset, bodyProps );
			pFutState->pODEObj = (void*)pOdeObj;

			break;
		}
			
		case eCV_COUPLED_OBJ:
		{
			double cr, cp, cy, sr, sp, sy;

			cr = cos( cpCurState->offset[3] ); 
			sr = sin( cpCurState->offset[3] );
			cp = cos( cpCurState->offset[4] ); 
			sp = sin( cpCurState->offset[4] );
			cy = cos( cpCurState->offset[5] ); 
			sy = sin( cpCurState->offset[5] );
			pFutState->rotMat[0][0] = cy * cp;
			pFutState->rotMat[0][1] = -sy * cr - cy * sp *sr;
			pFutState->rotMat[0][2] = sy * sr - cy * sp *cr;
			pFutState->rotMat[1][0] = sy * cp;
			pFutState->rotMat[1][1] = cy * cr - sy * sp * sr;
			pFutState->rotMat[1][2] = -cy * sr - sy * sp * cr;
			pFutState->rotMat[2][0] = sp;
			pFutState->rotMat[2][1] = cp * sr;
			pFutState->rotMat[2][2] = cp * cr;

			break;
		}

		case eCV_OBJ_REL_TRAJ:
		{
			if ( cpContInp->targDir.i == 0 && cpContInp->targDir.i == 0 )
				// relative path hasn't been passed in yet, there is no 
				// way to compensate for the initial rotation of the 
				// relative path, so don't transition yet.
			{
				noTransition = true;
				mode = cpCurState->prevMode; // use the previous mode 
				break;
			}

			double cr, cp, cy, sr, sp, sy, rotMat[3][3];

			cr = cos( cpCurState->offset[3] ); 
			sr = sin( cpCurState->offset[3] );
			cp = cos( cpCurState->offset[4] ); 
			sp = sin( cpCurState->offset[4] );
			cy = cos( cpCurState->offset[5] ); 
			sy = sin( cpCurState->offset[5] );
			rotMat[0][0] = cy * cp;
			rotMat[0][1] = -sy * cr - cy * sp *sr;
			rotMat[0][2] = sy * sr - cy * sp *cr;
			rotMat[1][0] = sy * cp;
			rotMat[1][1] = cy * cr - sy * sp * sr;
			rotMat[1][2] = -cy * sr - sy * sp * cr;
			rotMat[2][0] = sp;
			rotMat[2][1] = cp * sr;
			rotMat[2][2] = cp * cr;

			// offset the initial rotation of the relative trajectory so that the object 
			// will start sliding at the current relative rotation instead of the initial 
			// rotation of the trajectory 
			// The offset rotation matrix R_-y is 
			//          cos_y  sin_y  0
			//          -sin_y cos_y  0 
			//            0      0    1 
			// where tangent_relPath_0 = ( cos_y, sin_y, 0 ) 
			cy = cpContInp->targDir.i; 
			sy = cpContInp->targDir.j;

			pFutState->rotMat[0][0] = 
				cy * rotMat[0][0] + sy * rotMat[1][0]; 
			pFutState->rotMat[0][1] = 
				cy * rotMat[0][1] + sy * rotMat[1][1]; 
			pFutState->rotMat[0][2] = 
				cy * rotMat[0][2] + sy * rotMat[1][2]; 
			pFutState->rotMat[1][0] = 
				-sy * rotMat[0][0] + cy * rotMat[1][0]; 
			pFutState->rotMat[1][1] = 
				-sy * rotMat[0][1] + cy * rotMat[1][1]; 
			pFutState->rotMat[1][2] = 
				-sy * rotMat[0][2] + cy * rotMat[1][2]; 
			pFutState->rotMat[2][0] = 
				rotMat[2][0]; 
			pFutState->rotMat[2][1] = 
				rotMat[2][1]; 
			pFutState->rotMat[2][2] = 
				rotMat[2][2]; 				

			break;
		}

		case eCV_GROUND_TRAJ:
			// should not happen

			break;

		default:
			break;
		}

		if ( !noTransition )
		{
//			fprintf(stdout, "Object #%d transitioning from %d to %d\n", 
//				cvedId, cpCurState->prevMode, cpCurState->curMode);
			gout << "Object #" << cvedId << " transitioning from mode " << (int)(cpCurState->prevMode) 
				<< " to " << (int)(cpCurState->curMode) << endl;
			if ( cpCurState->prevMode == eCV_FREE_MOTION )
			{
				// remove entities in ODE
				cCved.DeleteODEObj( (CODEObject*)(cpCurState->pODEObj) );
			}
			pFutState->prevMode = cpCurState->curMode; 
			// may need write protection here since potentially 
			// multiple threads can write it
		}
	}

	switch ( mode ) {
	case eCV_FREE_MOTION:
	{
		
		if ( !pFutState->pODEObj ) 
		{
			cCved.SetupODEObject( cpAttr, cvedId, cCved, cpCurState, pFutState, DebugModeFlag );
		}

		// the object pos, rot and vel information will be obtained later after the ode sim step
		break;
	}

	case eCV_COUPLED_OBJ: 
	{
		TVector3D parentUp; 

		parentUp.i = cpParentState->anyState.lateral.j * cpParentState->anyState.tangent.k - 
			cpParentState->anyState.lateral.k * cpParentState->anyState.tangent.j;
		parentUp.j = cpParentState->anyState.lateral.k * cpParentState->anyState.tangent.i - 
			cpParentState->anyState.lateral.i * cpParentState->anyState.tangent.k;
		parentUp.k = cpParentState->anyState.lateral.i * cpParentState->anyState.tangent.j - 
			cpParentState->anyState.lateral.j * cpParentState->anyState.tangent.i;

		// P_c = P_p + x_offset * TAN_p - y_offset * LAT_p + z_offset * UP_p
		pFutState->position.x = cpParentState->anyState.position.x + 
			cpCurState->offset[0] * cpParentState->anyState.tangent.i - 
			cpCurState->offset[1] * cpParentState->anyState.lateral.i + 
			cpCurState->offset[2] * parentUp.i;
		pFutState->position.y = cpParentState->anyState.position.y + 
			cpCurState->offset[0] * cpParentState->anyState.tangent.j - 
			cpCurState->offset[1] * cpParentState->anyState.lateral.j + 
			cpCurState->offset[2] * parentUp.j;
		pFutState->position.z = cpParentState->anyState.position.z + 
			cpCurState->offset[0] * cpParentState->anyState.tangent.k - 
			cpCurState->offset[1] * cpParentState->anyState.lateral.k + 
			cpCurState->offset[2] * parentUp.k;

		// Tan_c = R_p * R_offset * [1 0 0]'
		// Lat_c = R_p * R_offset * [0 -1 0]'
		// Where R_p = [Tan_p -Lat_p Up_p]
		pFutState->tangent.i = 
			cpParentState->anyState.tangent.i * pFutState->rotMat[0][0] - 
			cpParentState->anyState.lateral.i * pFutState->rotMat[1][0] + 
			parentUp.i * pFutState->rotMat[2][0]; 
		pFutState->tangent.j = 
			cpParentState->anyState.tangent.j * pFutState->rotMat[0][0] - 
			cpParentState->anyState.lateral.j * pFutState->rotMat[1][0] + 
			parentUp.j * pFutState->rotMat[2][0]; 
		pFutState->tangent.k = 
			cpParentState->anyState.tangent.k * pFutState->rotMat[0][0] - 
			cpParentState->anyState.lateral.k * pFutState->rotMat[1][0] + 
			parentUp.k * pFutState->rotMat[2][0]; 

		pFutState->lateral.i =  
			-cpParentState->anyState.tangent.i * pFutState->rotMat[0][1] + 
			cpParentState->anyState.lateral.i * pFutState->rotMat[1][1] - 
			parentUp.i * pFutState->rotMat[2][1]; 
		pFutState->lateral.j = 
			-cpParentState->anyState.tangent.j * pFutState->rotMat[0][1] + 
			cpParentState->anyState.lateral.j * pFutState->rotMat[1][1] - 
			parentUp.j * pFutState->rotMat[2][1]; 
		pFutState->lateral.k = 
			-cpParentState->anyState.tangent.k * pFutState->rotMat[0][1] + 
			cpParentState->anyState.lateral.k * pFutState->rotMat[1][1] -
			parentUp.k * pFutState->rotMat[2][1]; 

		if ( DebugModeFlag )
			printf("CO parent (%.4f %.4f %.4f) tan(%.4f %.4f %.4f) lat(%.4f %.4f %.4f) child #%d (%.4f %.4f %.4f) tan (%.4f %.4f %.4f) lat (%.4f %.4f %.4f)\n", 
				cpParentState->anyState.position.x, cpParentState->anyState.position.y,
				cpParentState->anyState.position.z,
				cpParentState->anyState.tangent.i, cpParentState->anyState.tangent.j, 
				cpParentState->anyState.tangent.k, cpParentState->anyState.lateral.i,
				cpParentState->anyState.lateral.j, cpParentState->anyState.lateral.k, 
				cvedId, 
				pFutState->position.x, pFutState->position.y, pFutState->position.z,
				pFutState->tangent.i, pFutState->tangent.j, pFutState->tangent.i, 
				pFutState->lateral.i, pFutState->lateral.j, pFutState->lateral.k);

		pFutState->vel = cpParentState->anyState.vel; 
		// really it should be the parent velocity vector (vel, -velLat, velNorm) rotated

		break;
	}

	case eCV_OBJ_REL_TRAJ:
	{
		double x, y, z;	
		CVector3D  tangent, tangent1, lateral, lateral1;
		TVector3D parentUp; 			

		// calculate relatvie position and rotation
		// the control input will be relative values
		// perfect aceleration

		pFutState->vel = cpContInp->targVel; // future state stores relative velocity along relative path 

		// simple integrator for relative position
		x = cpCurState->offset[0] + cpContInp->targDir.i * pFutState->vel * delta;
		y = cpCurState->offset[1] + cpContInp->targDir.j * pFutState->vel * delta;
		z = cpCurState->offset[2];

		tangent.m_i = cpContInp->objOri.i;
		tangent.m_j = cpContInp->objOri.j;
		//tangent.m_k = 0;
		lateral.m_i = tangent.m_j;
		lateral.m_j = -tangent.m_i;
		//lateral.m_k = 0;
		//up.m_i = 0;
		//up.m_j = 0;
		//up.m_k = 1;

		
		// If the object has stopped moving, 
		//	then the tangent and lateral vectors
		//	will be set to (0,0,0).  This would
		//	cause problems with the bounding box 
		//	calculation, so just set tangent and
		//	lateral back to their previous values
		if (tangent.Normalize() == 0.0)  tangent = cpCurState->tangent;
		if (lateral.Normalize() == 0.0)  lateral = cpCurState->lateral;
		

		// update the new relative position, store it in 
		// offset[0] to [2]. New relative orientation info is in objOri of the control input.
		pFutState->offset[0] = x;
		pFutState->offset[1] = y;
		pFutState->offset[2] = z;

		// apply relative rotation from the path to tangent and lateral of the child, 
		// which should have already gone through a rotation defined in rotMat
		//   Tan_1 = R_relPath * R_offset * [1 0 0]'
		//   Lat_1 = R_relPath * R_offset * [0 -1 0]'
		// where R_relPath = [ Tan -Lat Up] as defined in tangent, lateral ( and up which 
		//     is (0, 0, 1) ), and R_offset as in rotMat
		tangent1.m_i = 
			tangent.m_i * pFutState->rotMat[0][0] -
			lateral.m_i * pFutState->rotMat[1][0]; 
		tangent1.m_j = 
			tangent.m_j * pFutState->rotMat[0][0] - 
			lateral.m_j * pFutState->rotMat[1][0]; 
		tangent1.m_k = 
			pFutState->rotMat[2][0]; 
		lateral1.m_i =  
			-tangent.m_i * pFutState->rotMat[0][1] + 
			lateral.m_i * pFutState->rotMat[1][1]; 
		lateral1.m_j = 
			-tangent.m_j * pFutState->rotMat[0][1] + 
			lateral.m_j * pFutState->rotMat[1][1]; 
		lateral1.m_k = 
			-pFutState->rotMat[2][1]; 

		// Apply rotation from parent to tangent and lateral of child
		//   Tan_c = R_p * Tan_1
		//   Lat_c = R_p * Lat_1
		// Where R_p = [Tan_p -Lat_p Up_p]
		parentUp.i = cpParentState->anyState.lateral.j * cpParentState->anyState.tangent.k - 
			cpParentState->anyState.lateral.k * cpParentState->anyState.tangent.j;
		parentUp.j = cpParentState->anyState.lateral.k * cpParentState->anyState.tangent.i - 
			cpParentState->anyState.lateral.i * cpParentState->anyState.tangent.k;
		parentUp.k = cpParentState->anyState.lateral.i * cpParentState->anyState.tangent.j - 
			cpParentState->anyState.lateral.j * cpParentState->anyState.tangent.i;

		pFutState->tangent.i = 
			cpParentState->anyState.tangent.i * tangent1.m_i - 
			cpParentState->anyState.lateral.i * tangent1.m_j + 
			parentUp.i * tangent1.m_k; 
		pFutState->tangent.j = 
			cpParentState->anyState.tangent.j * tangent1.m_i - 
			cpParentState->anyState.lateral.j * tangent1.m_j + 
			parentUp.j * tangent1.m_k; 
		pFutState->tangent.k = 
			cpParentState->anyState.tangent.k * tangent1.m_i - 
			cpParentState->anyState.lateral.k * tangent1.m_j + 
			parentUp.k * tangent1.m_k; 

		pFutState->lateral.i =  
			cpParentState->anyState.tangent.i * lateral1.m_i - 
			cpParentState->anyState.lateral.i * lateral1.m_j + 
			parentUp.i * lateral1.m_k; 
		pFutState->lateral.j = 
			cpParentState->anyState.tangent.j * lateral1.m_i - 
			cpParentState->anyState.lateral.j * lateral1.m_j + 
			parentUp.j * lateral1.m_k; 
		pFutState->lateral.k = 
			cpParentState->anyState.tangent.k * lateral1.m_i - 
			cpParentState->anyState.lateral.k * lateral1.m_j + 
			parentUp.k * lateral1.m_k; 

		pFutState->position.x = cpParentState->anyState.position.x + 
			x * cpParentState->anyState.tangent.i - 
			y * cpParentState->anyState.lateral.i + 
			z * parentUp.i;
		pFutState->position.y = cpParentState->anyState.position.y + 
			x * cpParentState->anyState.tangent.j - 
			y * cpParentState->anyState.lateral.j + 
			z * parentUp.j;
		pFutState->position.z = cpParentState->anyState.position.z + 
			x * cpParentState->anyState.tangent.k - 
			y * cpParentState->anyState.lateral.k + 
			z * parentUp.k;

		if ( DebugModeFlag )
		{
			printf("Obj #%d targ vel %.4f targDir (%.4f %.4f) objOri ( %.4f %.4f)\n", 
				cvedId, cpContInp->targVel, cpContInp->targDir.i, cpContInp->targDir.j, 
				cpContInp->objOri.i, cpContInp->objOri.j);
			printf("OR parent (%.4f %.4f %.4f) tan(%.4f %.4f %.4f) lat(%.4f %.4f %.4f) child #%d (%.4f %.4f %.4f) tan (%.4f %.4f %.4f) lat (%.4f %.4f %.4f)\n", 
				cpParentState->anyState.position.x, cpParentState->anyState.position.y,
				cpParentState->anyState.position.z,
				cpParentState->anyState.tangent.i, cpParentState->anyState.tangent.j, 
				cpParentState->anyState.tangent.k, cpParentState->anyState.lateral.i,
				cpParentState->anyState.lateral.j, cpParentState->anyState.lateral.k, 
				cvedId, 
				pFutState->position.x, pFutState->position.y, pFutState->position.z,
				pFutState->tangent.i, pFutState->tangent.j, pFutState->tangent.i, 
				pFutState->lateral.i, pFutState->lateral.j, pFutState->lateral.k);
		}

		break;
	}

	case eCV_GROUND_TRAJ:
	{
		double x, y, z;	// future position

		// perfect aceleration
		pFutState->vel = cpContInp->targVel;

		// simple integrator for (x,y)
		x = cpCurState->position.x + cpContInp->targDir.i * pFutState->vel * delta;
		y = cpCurState->position.y + cpContInp->targDir.j * pFutState->vel * delta;

		if ( DebugFlag ) {
			gout << "  fut position: " << x << ", " << y << endl;
		}

		// query a tricycle area to obtain orientation
		TVector2D forw, rght;
		double x1, y1, x2, y2, x3, y3;
		double z1, z2, z3;
		CCved::EQueryCode  rcode1, rcode2, rcode3;
		CVector3D  norm1, norm2, norm3;
		CVector3D  tangent, lateral;

		// forward and right vectors
		forw.i = cpContInp->objOri.i;
		forw.j = cpContInp->objOri.j;
		rght.i = forw.j;
		rght.j = -forw.i;

		// first point is straigh ahead
		x1 = x + 0.5 * cpAttr->xSize * forw.i;
		y1 = y + 0.5 * cpAttr->xSize * forw.j;

		// second point is rear right
		x2 = x + 0.5 * cpAttr->ySize * rght.i - 0.5 * cpAttr->xSize * forw.i;
		y2 = y + 0.5 * cpAttr->ySize * rght.j - 0.5 * cpAttr->xSize * forw.j;

		// third point is rear left
		x3 = x - 0.5 * cpAttr->ySize * rght.i - 0.5 * cpAttr->xSize * forw.i;
		y3 = y - 0.5 * cpAttr->ySize * rght.j - 0.5 * cpAttr->xSize * forw.j;

		if ( DebugFlag ) {
			gout << "  query points : " << x1 << ", " << y1 << ";  ";
			gout << x2 << ", " << y2 << ";  " << x3 << ", " << y3 << endl;
		}

		CCved::CTerQueryHint lastRLDPosition;

		lastRLDPosition.CopyFromStruct(cpCurState->posHint[0]);
		rcode1 = cCved.QryTerrain(
								x1, 
								y1, 
								cpCurState->position.z, 
								z1, 
								norm1, 
								&lastRLDPosition
								);
		lastRLDPosition.CopyToStruct( pFutState->posHint[0] );

		lastRLDPosition.CopyFromStruct( cpCurState->posHint[1] );
		rcode2 = cCved.QryTerrain(
								x2, 
								y2, 
								cpCurState->position.z, 
								z2, 
								norm2, 
								&lastRLDPosition
								);
		lastRLDPosition.CopyToStruct( pFutState->posHint[1] );

		lastRLDPosition.CopyFromStruct( cpCurState->posHint[2] );
		rcode3 = cCved.QryTerrain(
								x3, 
								y3, 
								cpCurState->position.z, 
								z3, 
								norm3, 
								&lastRLDPosition
								);
		lastRLDPosition.CopyToStruct( pFutState->posHint[2] );

		bool AllAnsGood =  rcode1 != CCved::eCV_OFF_ROAD 
							&& rcode2 != CCved::eCV_OFF_ROAD 
							&& rcode3 != CCved::eCV_OFF_ROAD;

		if ( AllAnsGood ) {
			z = ( z1 + z2 + z3) * 0.3333333f;
		}
		else {
			// Because the current position is raised off the
			// 	ground by 1/2*zHeight of the object, we need 
			// 	to subtract that from the current z to get
			// 	the off-road z value.
			z = cpCurState->position.z; // - 0.5*cpAttr->zSize;
			z1 = z;
			z2 = z;
			z3 = z;
		}
		if ( DebugFlag ) {
			gout << "  AllAnswersGood = " << AllAnsGood << " average z = ";
			gout << z << endl;
		}

		if ( z > cCV_ZERO )  gout << "";


		pFutState->position.x = x;
		pFutState->position.y = y;
		// To make sure that the position of the object is
		//  at the center of the bounding block, push the z
		//  value up by 1/2*zHeight of the object.
		pFutState->position.z = z;  // + 0.5*cpAttr->zSize;



		const CSolObj* cpSolObj = cCved.GetSol().GetObj( cCved.GetObjSolId(cvedId) );
		const CSolObjVehicle* cpSolObjVeh = dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
		const CDynaParams& dynaParams = cpSolObjVeh->GetDynaParams();

		if(cpSolObjVeh != NULL){
			float tireCircumference;
			tireCircumference = (float)(2*3.14159*dynaParams.m_TireRadius);
	  
			float tireDist;
			tireDist = float(sqrt(pow((cpCurState->position.x - pFutState->position.x),2) +
				pow((cpCurState->position.y - pFutState->position.y),2)));

			float frontLeftTireRot = cpCurState->tireRot[0];
			float frontRightTireRot = cpCurState->tireRot[1];
			float rearTireRot = cpCurState->tireRot[2];

			frontLeftTireRot = (float)fmod((((tireDist/tireCircumference)*360.0f) + pFutState->tireRot[0]),360.0f);
			frontRightTireRot =(float)fmod((((tireDist/tireCircumference)*360.0f) + pFutState->tireRot[1]),360.0f);
			rearTireRot = (float)fmod((((tireDist/tireCircumference)*360.0) + pFutState->tireRot[2]),360);

			pFutState->tireRot[0] = frontLeftTireRot;
			pFutState->tireRot[1] = frontRightTireRot;
			pFutState->tireRot[2] = rearTireRot;
		}



		tangent.m_i = x1 - x;
		tangent.m_j = y1 - y;
		tangent.m_k = z1 - z;

		lateral.m_i = x2 - x3;
		lateral.m_j = y2 - y3;
		lateral.m_k = z2 - z3;

		// If the object has stopped moving, 
		//	then the tangent and lateral vectors
		//	will be set to (0,0,0).  This would
		//	cause problems with the bounding box 
		//	calculation, so just set tangent and
		//	lateral back to their previous values
		if (tangent.Normalize() == 0.0)  tangent = cpCurState->tangent;

		if (lateral.Normalize() == 0.0)  lateral = cpCurState->lateral;

		pFutState->tangent.i = tangent.m_i;
		pFutState->tangent.j = tangent.m_j;
		pFutState->tangent.k = tangent.m_k;

		pFutState->lateral.i = lateral.m_i;
		pFutState->lateral.j = lateral.m_j;
		pFutState->lateral.k = lateral.m_k;

		if ( DebugModeFlag )
			printf("GT object #%d (%.4f %.4f %.4f) tan (%.4f %.4f %.4f) lat (%.4f %.4f %.4f)\n", 
				cvedId, 
				pFutState->position.x, pFutState->position.y, pFutState->position.z,
				pFutState->tangent.i, pFutState->tangent.j, pFutState->tangent.i, 
				pFutState->lateral.i, pFutState->lateral.j, pFutState->lateral.k);

		break;
	}
	default:
		break;
	} // end of switch( mode )
} // end of TrajFollowerDynamics


//////////////////////////////////////////////////////////////////////////////
//
// Description:  DynamicModel (private)
// 	Dispatcher for the various dynamic models
//
// Remarks: This function selects the appropriate dynamic model based on the
// 	object's type and sol identifier.
//
// Arguments:
//  type  - the type of the object
//  cpAttr - pointer to object's attributes
//  pCurState - pointer to object's current state
//  cpContInp  - pointer to object's control inputs
//  pFutState - pointer to object's future state
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::DynamicModel(
		 int				 cvedId,
         cvEObjType          type,
         const cvTObjAttr*    cpAttr,
         const cvTObjState*   pCurState,
         const cvTObjContInp* cpContInp,
         cvTObjState*         pFutState
		 )
{
	if ( m_pHdr->dynaMult <= 0 ) {
		cvCInternalError err("Dynamics multiplier is 0", __FILE__, __LINE__);
		throw err;
	}

	switch ( type ) {
		case eCV_TRAJ_FOLLOWER  : 
		{
			TObj* pParentObj;
			cvTObjState* pParentState = NULL;
			cvEObjType  parentType = eCV_INVALID;

			if ( pCurState->trajFollowerState.curMode == eCV_OBJ_REL_TRAJ || 
				pCurState->trajFollowerState.curMode == eCV_COUPLED_OBJ || 
				pCurState->trajFollowerState.prevMode == eCV_OBJ_REL_TRAJ || 
				pCurState->trajFollowerState.prevMode == eCV_COUPLED_OBJ )
			{
				if ( pCurState->trajFollowerState.parentId < 0 )
				{
					// parent not found yet
					break;
				}
				else
				{
					pParentObj = BindObj( pCurState->trajFollowerState.parentId ); 
					parentType = pParentObj->type;
					if( (m_pHdr->frame & 1) == 0 ) 
					{
						// even frame
						pParentState    = &(pParentObj->stateBufB.state);
					}
					else 
					{
						// odd frame
						pParentState    = &(pParentObj->stateBufA.state);
					}
				}
			}

			TrajFollowerDynamics(
					cvedId,
					*this,
					m_pHdr->deltaT / m_pHdr->dynaMult,
					cpAttr,
					&pCurState->trajFollowerState,
					&cpContInp->trajFollowerContInp,
					parentType,
					pParentState,
					&pFutState->trajFollowerState);
			break;
		}

		case eCV_VEHICLE :       // vehicle dynamics
			VehicleDynamics(
				    cvedId,
					*this,
					m_pHdr->deltaT / m_pHdr->dynaMult,
					cpAttr,
					&pCurState->vehicleState.vehState,
					&cpContInp->vehicleContInp.contInp,
					&pFutState->vehicleState.vehState,
                    m_pHdr->frame%2 == 0
					);
			break;

		case eCV_TRAILER :
			assert( 0 );
			break;

		case eCV_RAIL_VEH :
			assert( 0 );
			break;

		case eCV_TERRAIN :
			assert( 0 );
			break;

		case eCV_TRAFFIC_LIGHT :
			assert( 0 );
			break;

		case eCV_TRAFFIC_SIGN :
			assert( 0 );
			break;

		case eCV_COMPOSITE_SIGN :
			assert( 0 );
			break;

		case eCV_OBSTACLE :
			assert( 0 );
			break;

		case eCV_POI :
			assert( 0 );
			break;

		case eCV_COORDINATOR :
			assert( 0 );
			break;

		case eCV_EXTERNAL_DRIVER: // vehicle dynamics if not on simulator
			VehicleDynamics(
				    cvedId,
					*this,
					m_pHdr->deltaT / m_pHdr->dynaMult,
					cpAttr,
					&pCurState->vehicleState.vehState,
					&cpContInp->vehicleContInp.contInp,
					&pFutState->vehicleState.vehState,
                    m_pHdr->frame%2 > 0
					);
			break;

		case eCV_EXTERNAL_TRAILER :
			assert( 0 );
			break;

		case eCV_WALKER :
			assert( 0 );
			break;

		default : 
			assert( 0 );
	}

	// update the bounding box
	CPoint3D ll, ur;

	UpdateBBox(cpAttr, 
			pFutState->anyState.tangent, 
			pFutState->anyState.lateral,
			pFutState->anyState.position,
			ll,
			ur);

	pFutState->anyState.boundBox[0].x = ll.m_x;
	pFutState->anyState.boundBox[0].y = ll.m_y;
	pFutState->anyState.boundBox[0].z = ll.m_z;

	pFutState->anyState.boundBox[1].x = ur.m_x;
	pFutState->anyState.boundBox[1].y = ur.m_y;
	pFutState->anyState.boundBox[1].z = ur.m_z;
	
} // end of DynamicModel

//////////////////////////////////////////////////////////////////////////////
//
// Description:  SetupODEObject
// 	
//
// Remarks: 
//
// Arguments:

//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetupODEObject( const cvTObjAttr* cpAttr,
					   int cvedId,
			 		   CCved& cCved,
			   		   const cvTObjState::TrajFollowerState* cpCurState,
					   cvTObjState::TrajFollowerState* pFutState,
					   bool DebugModeFlag )
{
	CODEObject* pOdeObj;

	pOdeObj = new CODEObject;
	double cgoffset[3] = {0, 0, 0}, 
		dimension[3] = {cpAttr->xSize, cpAttr->ySize, cpAttr->zSize},
		visOrigOffset[3] = {0, 0, -cpAttr->zSize*0.5}; // assuming the vis model origin is at the bottom
	odePublic::SBodyProps bodyProps;
	bodyProps.cvedId = cvedId;
	bodyProps.solId = cpAttr->solId;
	bodyProps.frictionCoeff = cCved.GetSol().GetObj(cpAttr->solId)->GetFrictionCoeff();
	bodyProps.bounceEnergyLoss = cCved.GetSol().GetObj(cpAttr->solId)->GetBounceEnergyLoss();
	bodyProps.contactSoftCFM = (dimension[2]>3.1)?1e-03:1e-09;
	double mass = (dimension[2]>3.1)?1200:80;
	odePublic::EODEObjType type;
	bodyProps.cigiId = cCved.GetSol().GetObj(cpAttr->solId)->GetVisModelCigiId();
	// Temporary code, use cigi id to identify ode object type.
	// In the future a field should be added to the sol obj to identify its type
	switch ( bodyProps.cigiId ) {
	// box 
	case 512:  // suit case 
	case 514:  // desk
		type = odePublic::e_BOX;
		break;
	// sphere 
	case 513:  // ball
		type = odePublic::e_SPHERE;
		mass = 1.0;
		break;
	// cylinder
	case 509:  // traf cone 12"
		type = odePublic::e_CYLINDER;
		mass = 2.0;
		break;
	case 500:  // traf cone
	case 510:  // traf cone 18"
		type = odePublic::e_CYLINDER;
		mass = 4.5;
		break;
	case 515:  // traf cone 36"
		type = odePublic::e_CYLINDER;
		mass = 18.0;
		break;
	case 505:  // drum
		type = odePublic::e_COMPOSITE;
		mass = 10.0;
		break;
	case 508:  // crash barrel
		type = odePublic::e_CAPSULE;
		mass = 100.0;
		break;
	default:
		type = odePublic::e_BOX;
	}

	double initVel[6]; 
	if(cpCurState != NULL && pFutState != NULL){
		// initial velocities are in local coord system, convert them to world coords
		initVel[0] = cpCurState->rotMat[0][0] * cpCurState->initVel[0] + 
			cpCurState->rotMat[0][1] * cpCurState->initVel[1] + 
			cpCurState->rotMat[0][2] * cpCurState->initVel[2];
		initVel[1] = cpCurState->rotMat[1][0] * cpCurState->initVel[0] + 
			cpCurState->rotMat[1][1] * cpCurState->initVel[1] + 
			cpCurState->rotMat[1][2] * cpCurState->initVel[2];
		initVel[2] = cpCurState->rotMat[2][0] * cpCurState->initVel[0] + 
			cpCurState->rotMat[2][1] * cpCurState->initVel[1] + 
			cpCurState->rotMat[2][2] * cpCurState->initVel[2];
			
		// remember initVel[4] and cpCurState->initVel[4] are pitch rates, the negative of 
		// vel roty.
		initVel[3] = cpCurState->rotMat[0][0] * cpCurState->initVel[3] - 
			cpCurState->rotMat[0][1] * cpCurState->initVel[4] + 
			cpCurState->rotMat[0][2] * cpCurState->initVel[5];
		initVel[4] = -(cpCurState->rotMat[1][0] * cpCurState->initVel[3] - 
			cpCurState->rotMat[1][1] * cpCurState->initVel[4] + 
			cpCurState->rotMat[1][2] * cpCurState->initVel[5]);
		initVel[5] = cpCurState->rotMat[2][0] * cpCurState->initVel[3] - 
			cpCurState->rotMat[2][1] * cpCurState->initVel[4] + 
			cpCurState->rotMat[2][2] * cpCurState->initVel[5];

		cCved.CreateODEObject( type, pOdeObj, 
			cpCurState->offset, cpCurState->initVel, mass, 
			cgoffset, dimension, visOrigOffset, bodyProps );
		pFutState->pODEObj = (void*)pOdeObj;
	}
	else{
		const dReal posRot[6] = {0,0,0,0,0,0};
		const dReal vels[6] = {0,0,0,0,0,0};
		cCved.CreateODEObject( type, pOdeObj, 
			posRot, vels, mass, 
			cgoffset, dimension, visOrigOffset, bodyProps );
	}

	if ( DebugModeFlag )
		printf("FM Object #%d Init posrot %.4f %.4f %.4f %.4f %.4f %.4f vels %.4f %.4f %.4f %.4f %.4f %.4f\n", 
			cvedId,
			cpCurState->offset[0], cpCurState->offset[1], cpCurState->offset[2], 
			cpCurState->offset[3], cpCurState->offset[4], cpCurState->offset[5],
			initVel[0], initVel[1], initVel[2],
			initVel[3], initVel[4], initVel[5] );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:  SetupODEObject
// 	
//
// Remarks: 
//
// Arguments:

//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetupVehODEObject( const cvTObjAttr* cpAttr,
					   int cvedId,
			 		   CCved& cCved,
					   TVehicleState* pFutState,
					   bool DebugModeFlag )
{
	CODEObject* pOdeObj;

	pOdeObj = new CODEObject;
	double cgoffset[3] = {0, 0, 0}, 
		dimension[3] = {cpAttr->xSize, cpAttr->ySize, cpAttr->zSize},
		visOrigOffset[3] = {0, 0, -cpAttr->zSize*0.5}; // assuming the vis model origin is at the bottom
	odePublic::SBodyProps bodyProps;
	bodyProps.cvedId = cvedId;
	bodyProps.solId = cpAttr->solId;
	bodyProps.frictionCoeff = cCved.GetSol().GetObj(cpAttr->solId)->GetFrictionCoeff();
	bodyProps.bounceEnergyLoss = cCved.GetSol().GetObj(cpAttr->solId)->GetBounceEnergyLoss();
	bodyProps.contactSoftCFM = (dimension[2]>3.1)?1e-03:1e-09;
	double mass = (dimension[2]>3.1)?1200:80;
	odePublic::EODEObjType type;
	bodyProps.cigiId = cCved.GetSol().GetObj(cpAttr->solId)->GetVisModelCigiId();
	// Temporary code, use cigi id to identify ode object type.
	// In the future a field should be added to the sol obj to identify its type
	switch ( bodyProps.cigiId ) {
	// box 
	case 512:  // suit case 
	case 514:  // desk
		type = odePublic::e_BOX;
		break;
	// sphere 
	case 513:  // ball
		type = odePublic::e_SPHERE;
		mass = 1.0;
		break;
	// cylinder
	case 509:  // traf cone 12"
		type = odePublic::e_CYLINDER;
		mass = 2.0;
		break;
	case 500:  // traf cone
	case 510:  // traf cone 18"
		type = odePublic::e_CYLINDER;
		mass = 4.5;
		break;
	case 515:  // traf cone 36"
		type = odePublic::e_CYLINDER;
		mass = 18.0;
		break;
	case 505:  // drum
		type = odePublic::e_COMPOSITE;
		mass = 10.0;
		break;
	case 508:  // crash barrel
		type = odePublic::e_CAPSULE;
		mass = 100.0;
		break;
	default:
		type = odePublic::e_BOX;
	}

	dReal posRot[6] = {0,0,0,0,0,0};
	dReal vels[6] = {0,0,0,0,0,0};

	posRot[0] = pFutState->position.x;
	posRot[1] = pFutState->position.y;
	posRot[2] = pFutState->position.z;
	posRot[3] = atan2( -pFutState->lateral.k, pFutState->lateral.i * pFutState->tangent.j 
					- pFutState->lateral.j * pFutState->tangent.i ); // roll
	posRot[4] = asin( pFutState->tangent.k );  // pitch
	posRot[5] = atan2( pFutState->tangent.j, pFutState->tangent.i ); // yaw

	cCved.CreateODEObject( type, pOdeObj, 
		posRot, vels, mass, 
		cgoffset, dimension, visOrigOffset, bodyProps );

	/*if ( DebugModeFlag )
		printf("FM Object #%d Init posrot %.4f %.4f %.4f %.4f %.4f %.4f vels %.4f %.4f %.4f %.4f %.4f %.4f\n", 
			cvedId,
			cpCurState->offset[0], cpCurState->offset[1], cpCurState->offset[2], 
			cpCurState->offset[3], cpCurState->offset[4], cpCurState->offset[5],
			initVel[0], initVel[1], initVel[2],
			initVel[3], initVel[4], initVel[5] );*/
}


} // namespace CVED
