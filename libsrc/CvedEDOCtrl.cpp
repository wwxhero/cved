#include "CvedEDOCtrl.h"
#include "cvedpub.h"
#include "cvedstrc.h"
#include "objreflistUtl.h"
#include "EnvVar.h"
#include <algorithm>
#include "hcsmobject.h"
#include "ExternalControlInterface.h"
#include "polygon2d.h"

#include <filename.h>
#include "hcsmobject.h"
#include "odeDynamics.h"
namespace CVED {

CCvedEDOCtrl::CCvedEDOCtrl(IExternalObjectControl* pCtrl) : CCvedDistri(pCtrl)
{
}

CCvedEDOCtrl::~CCvedEDOCtrl(void)
{
}

CDynObj*	CCvedEDOCtrl::DistriCreateADO(const string&		cName,
								const cvTObjAttr&	cAttr,
								const CPoint3D*		cpInitPos,
								const CVector3D*	cpInitTan,
								const CVector3D*	cpInitLat)
{
	assert(0); //edo ctrl should not create a distributed driving object currently
	return NULL;
}
void		CCvedEDOCtrl::DistriDeleteADO( CDynObj* )
{
	assert(0); //edo ctrl should not create a distributed driving object currently
}

void CCvedEDOCtrl::ExecuteDynamicModels(void)
{
	TObjectPoolIdx id;
	TObj*          pO;
	int            count = 0; // optimization
	TObjectPoolIdx attachedObjIds[cNUM_DYN_OBJS]; // list of objects that are attached to others
	                                              // they need to be processed after their parents
	                                              // are processed.
	TObjectPoolIdx freeMotionObjIds[cNUM_DYN_OBJS]; // list of objects in free motion. The main
	                                                // loop will create the ode objects for them
	                                                // if they are newly transitioned to free motion
	                                                // mode, while the second loop processes them
	                                                // after the ode simstep so that updated pos,
	                                                // rot, vel can be obtained. This way newly
	                                                // transitioned free motion objects won't lose
	                                                // one frame of dynamics.
	int            attachedObjCount = 0, freeMotionObjCount = 0, i;
//	static short timer = 0;
//	double currentSecs;

	/*
	if ( !timer )
	{
		timer   = hrt_timer_alloc(HRT_MICROSECOND, "");
		hrt_timer_start( timer );
	}
	printf("ExecDyna: current sec %f simstep size %f\n",
		hrt_timer_currentsecs( timer ), m_pHdr->deltaT / m_pHdr->dynaMult * 2);
	*/
	IExternalObjectControl* ctrl = m_pCtrl;
	if (NULL != ctrl)
		ctrl->PreUpdateDynamicModels();
	if( m_haveFakeExternalDriver
	|| NULL != ctrl )
	{
		id = 0;
	}
	else
	{
		id = cMAX_EXT_CNTRL_OBJS;
	}
	pO = BindObj( id );

	while( id < cNUM_DYN_OBJS )
	{
		bool executeDynaModel = (
					(
						pO->type == eCV_TRAJ_FOLLOWER ||
						pO->type == eCV_VEHICLE ||
						pO->type == eCV_EXTERNAL_DRIVER
						)
					&&
					( pO->phase == eALIVE || pO->phase == eDYING )
					);
		bool localSim = (0 == id);
		bool RemoteOwn = (0 != id
					&& (pO->type == eCV_VEHICLE));

//		if ( pO->type == eCV_TRAJ_FOLLOWER )
//			fprintf(stdout, " traj follower %d phase %d\n", id, pO->phase);

		if( executeDynaModel )
		{
			cvTObjState currState;
			const cvTObjState* pCurrState = &currState;
			cvTObjState* pFutState;
			const cvTObjContInp* pCurrContInp;

			// on even frames, dynamic servers use buffer A for
			// control inputs and buffer B for state.  Vice versa
			// for odd frames.  Same attributes are used as
			// attributes don't change based on the frame.

			if( (m_pHdr->frame & 1) == 0 )
			{
				// even frame
				pCurrContInp = &pO->stateBufA.contInp;
				currState    = pO->stateBufB.state;
				pFutState    = &pO->stateBufB.state;
			}
			else
			{
				// odd frame
				pCurrContInp = &pO->stateBufB.contInp;
				currState    = pO->stateBufA.state;
				pFutState    = &pO->stateBufA.state;
			}

			if ( pO->type == eCV_TRAJ_FOLLOWER )
			{
				if ( currState.trajFollowerState.curMode == eCV_FREE_MOTION )
				// free motion object, its pos, rot, vel information has to
				// be obtained in a separate loop after the ode sim step.
				{
					freeMotionObjIds[freeMotionObjCount] = id;
					++freeMotionObjCount;
				}

				if ( currState.trajFollowerState.curMode != eCV_GROUND_TRAJ )
				// traj follower that was or is attached to a parent object, delay the dynamics computation
				{
					attachedObjIds[attachedObjCount] = id;
					++attachedObjCount;
				}
				else{
					CVED::CCved &me = *this;
					CVED::CObj* obj = BindObjIdToClass2(id);
					if ( !RemoteOwn
					 ||	(NULL == ctrl
					 ||	!ctrl->OnGetUpdate(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState)))
					{
						if (!localSim
					 		|| m_haveFakeExternalDriver) //local simulator has its own dynamic
							*pFutState = *pCurrState;
					}
					if (localSim
						&& NULL != ctrl)
						ctrl->OnPushUpdate(0, pCurrContInp, pFutState);

				}
			}
			else
			{
				CVED::CCved &me = *this;
				CVED::CObj* obj = BindObjIdToClass2(id);
				if ( !RemoteOwn
					 ||	(NULL == ctrl
					 ||	!ctrl->OnGetUpdate(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState)))
				{
					if (!localSim
					 || m_haveFakeExternalDriver) //local simulator has its own dynamic
						*pFutState = *pCurrState;
				}
				if (localSim
					&& NULL != ctrl)
						ctrl->OnPushUpdate(0, pCurrContInp, pFutState);
			}

			// This is an optimization.
			// We know the total number of dynamic objects because
			// is stored in the header so once we have processed
			// as many objects, there is no reason to keep going through
			// the object pool since the rest of the objects will
			// certainly be in the TDead state
			count++;
			if( count == m_pHdr->dynObjectCount )  break;
		}

		id++;
		pO++;
	}

//	fprintf(stdout, "processing %d attached objs now\n", attachedObjCount);
	// now process the child objects
	for ( i=0; i<attachedObjCount; ++i )
	{
//		fprintf(stdout, "attached obj %d\n", attachedObjIds[i]);
		pO = BindObj( attachedObjIds[i] );

//		fprintf(stdout, "bind ok\n");
		cvTObjState currState;
		const cvTObjState* pCurrState = &currState;
		cvTObjState* pFutState;
		const cvTObjContInp* pCurrContInp;

		// on even frames, dynamic servers use buffer A for
		// control inputs and buffer B for state.  Vice versa
		// for odd frames.  Same attributes are used as
		// attributes don't change based on the frame.

		if( (m_pHdr->frame & 1) == 0 )
		{
			// even frame
			pCurrContInp = &pO->stateBufA.contInp;
			currState    = pO->stateBufB.state;
			pFutState    = &pO->stateBufB.state;
		}
		else
		{
			// odd frame
			pCurrContInp = &pO->stateBufB.contInp;
			currState    = pO->stateBufA.state;
			pFutState    = &pO->stateBufA.state;
		}

		CVED::CCved &me = *this;
		bool localSim = (0 == attachedObjIds[i]);
		bool RemoteOwn = (0 != attachedObjIds[i]
					&& pO->type == eCV_EXTERNAL_DRIVER);
		if ( !RemoteOwn
		||	(NULL == ctrl
		||	!ctrl->OnGetUpdate(attachedObjIds[i], const_cast<cvTObjContInp*>(pCurrContInp), pFutState)))
		{
			if (!localSim
			 || m_haveFakeExternalDriver) //local simulator has its own dynamic
				*pFutState = *pCurrState;
		}
		if (localSim
		&& NULL != ctrl)
			ctrl->OnPushUpdate(0, pCurrContInp, pFutState);
	}

	// execute ode dynamics from objects in free motion mode
	m_pOde->SimStep( (float)m_pHdr->deltaT / m_pHdr->dynaMult );

	// get updated information of free motion objects
	for ( i=0; i<freeMotionObjCount; ++i )
	{
//		fprintf(stdout, "free motion obj %d\n", freeMotionObjIds[i]);
		pO = BindObj( freeMotionObjIds[i] );

//		fprintf(stdout, "bind ok\n");
		cvTObjState* pFutState;
		double position[3], tangent[3], lateral[3];
		CODEObject* pOdeObj;

		// on even frames, dynamic servers use buffer A for
		// control inputs and buffer B for state.  Vice versa
		// for odd frames.  Same attributes are used as
		// attributes don't change based on the frame.

		if( (m_pHdr->frame & 1) == 0 )
		{
			// even frame
			pOdeObj = (CODEObject*)(pO->stateBufB.state.trajFollowerState.pODEObj);
			pFutState = &pO->stateBufB.state;
		}
		else
		{
			// odd frame
			pOdeObj = (CODEObject*)(pO->stateBufA.state.trajFollowerState.pODEObj);
			pFutState = &pO->stateBufA.state;
		}

		pOdeObj->GetPosRot( position, tangent, lateral );
		pFutState->anyState.position.x = position[0];
		pFutState->anyState.position.y = position[1];
		pFutState->anyState.position.z = position[2];
		pFutState->anyState.tangent.i = tangent[0];
		pFutState->anyState.tangent.j = tangent[1];
		pFutState->anyState.tangent.k = tangent[2];
		pFutState->anyState.lateral.i = lateral[0];
		pFutState->anyState.lateral.j = lateral[1];
		pFutState->anyState.lateral.k = lateral[2];

		bool DebugModeFlag = false;

		if ( DebugModeFlag )
			printf("FM object #%d (%.4f %.4f %.4f) tan (%.4f %.4f %.4f) lat (%.4f %.4f %.4f)\n",
				freeMotionObjIds[i],
				position[0], position[1], position[2],
				tangent[0], tangent[1], tangent[2],
				lateral[0], lateral[1], lateral[2]);


	}
	if (NULL != ctrl)
		ctrl->PostUpdateDynamicModels();
}

//it creates a mock edo
CDynObj* CCvedEDOCtrl::LocalCreateEDO(CHeaderDistriParseBlock& blk)
{
	const double cMETER_TO_FEET = 3.2808; // feet
	//
	// Get the SOL object.
	//
	const string cSolName = blk.GetSolName();

	const CSolObj* cpSolObj = this->GetSol().GetObj( cSolName );
	if( !cpSolObj )
	{
		assert(0);
		return NULL;
	}

	cvEObjType type = eCV_VEHICLE;
	//
	// Initialize the attributes.
	//
	cvTObjAttr attr = { 0 };
	attr.solId = cpSolObj->GetId();
	attr.xSize = cpSolObj->GetLength();
	attr.ySize = cpSolObj->GetWidth();
	attr.zSize = cpSolObj->GetHeight();
	attr.colorIndex = blk.GetColorIndex();
	attr.hcsmId = 0;



	double initVel = 0;
	this->GetOwnVehicleVel(initVel);
	//
	// Get the starting location and add the vehicle's CG in the z-axis.
	//
	CPoint3D cartPos = blk.GetOwnVehPos();
	CRoadPos roadPos(*this, cartPos);
	CDynObj* pObj = NULL;
	if( !roadPos.IsValid() )
	{
		//
		// The vehicle is currently off-road...use the offroad point.
		//

		CPoint3D targPos = roadPos.GetXYZ();
		CVector3D tan = targPos - cartPos;
		CVector3D lat( 0.0, 0.0, 0.0 );

		//
		// Create the CVED object.
		//
		pObj = this->CreateDynObj(
							blk.GetSimName()
							, type
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end if vehicle offroad
	else
	{
		//
		// The vehicle is on the road.
		//
		cartPos = roadPos.GetXYZ();
        CRoadPos roadPosN = roadPos;
        if (initVel > 0 ){
            float distLookAhead = initVel * cMETER_TO_FEET;
            if (roadPosN.Travel(distLookAhead) == CRoadPos::eERROR){
                roadPosN = roadPos;
            }
        }
        CVector3D rotVec;
        if (initVel > 0){
            CVector3D vec1 = roadPos.GetTangentInterpolated();
            CVector3D vec2 = roadPosN.GetTangentInterpolated();
            CVector3D vec3 = vec1-vec2;

            auto targPosRp = roadPosN.GetBestXYZ();
            auto currPosRp = roadPos.GetBestXYZ();
            CVector3D targPos(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            CVector3D currPos(currPosRp.m_x,currPosRp.m_y,currPosRp.m_z);
	        CVector3D targetSeg(targPos - currPos);
            float laneWidth = 0;

            float currentMaxOffset = 6.0f;

	        //Scaling this by 1/3 the segment length
	        //was found through experimentations
	        //to work well, this should be parmaiterized
	        vec3.Scale(targetSeg.Length()/3);


            if (vec3.Length() > currentMaxOffset){
                vec3.Scale(currentMaxOffset/vec3.Length());
            }
            CPoint3D projectedXYZ  = roadPos.GetBestXYZ();
	        CRoadPos tempRoadPos(roadPos);
	        tempRoadPos.SetXYZ(projectedXYZ + vec3);
            targPosRp = tempRoadPos.GetBestXYZ();
            CVector3D vec4(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            rotVec = vec4 - currPos;
            rotVec.Normalize();
        }
		//
		// Get the staring tangent and lateral vectors.
		//
		CVector3D tan = roadPos.GetTangent();
		CVector3D lat = roadPos.GetRightVec();
        if (initVel > 0){
            double yaw = acos(rotVec.DotP(tan) );
            if (/*diff.Length()*/ yaw > 0.01){
                auto axis = tan.CrossP(rotVec);
                if (axis.m_k<0) yaw*=-1;
                tan.RotZ(yaw);
                lat.RotZ(yaw);
            }
        }
		//
		// Create the CVED object.
		//
		pObj = this->CreateDynObj(
							blk.GetSimName()
							, type
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end else vehicle on the road

	//
	// Get a pointer to the vehicle object.
	//
	CVehicleObj* pVehicleObj = dynamic_cast<CVehicleObj *>( pObj );

	//
	// Set the initial velocity.
	//


	bool haveInitVel = initVel >= 0.0;
	if( haveInitVel )
	{
		// set velocity to both buffers
		pObj->SetVel( initVel, false );
	}

	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	const CSolObjVehicle* cpSolVeh =
				dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
	if( !cpSolVeh )
	{
		assert(0);
		this->DeleteDynObj(pObj);
		return NULL;
	}

	const CDynaParams& dynaParams = cpSolVeh->GetDynaParams();

	double suspStif =
			( dynaParams.m_SuspStifMin + dynaParams.m_SuspStifMax ) / 2;
	pVehicleObj->SetSuspStif( suspStif );
	pVehicleObj->SetSuspStifImm( suspStif );
	double suspDamp =
			( dynaParams.m_SuspDampMin + dynaParams.m_SuspDampMax ) / 2;
	pVehicleObj->SetSuspDamp( suspDamp );
	pVehicleObj->SetSuspDampImm( suspDamp );

	double tireStif =
			( dynaParams.m_TireStifMin + dynaParams.m_TireStifMax ) / 2;
	pVehicleObj->SetTireStif( tireStif );
	pVehicleObj->SetTireStifImm( tireStif );
	double tireDamp =
			( dynaParams.m_TireDampMin + dynaParams.m_TireDampMax ) / 2;
	pVehicleObj->SetTireDamp( tireDamp );
	pVehicleObj->SetTireDampImm( tireDamp );
	//pVehicleObj->SetDynaFidelity( m_pI->m_dynModel );
	//pVehicleObj->SetDynaFidelityImm( m_pI->m_dynModel );
	pVehicleObj->SetQryTerrainErrCount( 0 );
	pVehicleObj->SetQryTerrainErrCountImm( 0 );
	pVehicleObj->SetDynaInitComplete( 0 );
	pVehicleObj->SetDynaInitCompleteImm( 0 );

	//
	// Set the initial audio and visual state.
	//
	pVehicleObj->SetAudioState( blk.GetAudioState() );
	pVehicleObj->SetVisualState( blk.GetVisualState() );

	// //
	// // If the audio or visual has been set then assign these values
	// // to the dials.  This way the ADO will play those sounds and
	// // display those lights until they are explicity turned off.
	// //
	// if( blk.GetVisualState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetVisualState() );
	// 	string str( buf );
	// 	m_dialVisualState.SetValue( str );
	// }

	// if( blk.GetAudioState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetAudioState() );
	// 	string str( buf );
	// 	m_dialAudioState.SetValue( str );
	// }

    //init control
    pVehicleObj->SetTargPos(cartPos);
    pVehicleObj->StoreOldTargPos();

	return pObj;
}

CDynObj* CCvedEDOCtrl::LocalCreatePDO(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos,
					const CVector3D*	cpInitTan,
					const CVector3D*	cpInitLat)
{
	return CCvedDistri::CreateDynObj(cName, eCV_VEHICLE, cAttr, cpInitPos, cpInitTan, cpInitLat);
}

};