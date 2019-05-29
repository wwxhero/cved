#include "CvedADOCtrl.h"
#include "cvedpub.h"
#include "cvedstrc.h"
#include "objreflistUtl.h"
#include "EnvVar.h"
#include <algorithm>
#include "ExternalControlInterface.h"
#include "polygon2d.h"

#include <filename.h>
#include "odeDynamics.h"
namespace CVED {

CCvedADOCtrl::CCvedADOCtrl(IExternalObjectControl* pCtrl) : CCvedDistri(pCtrl)
{
}

CCvedADOCtrl::~CCvedADOCtrl(void)
{
}

void CCvedADOCtrl::ExecuteDynamicModels(void)
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
						pO->type == eCV_EXTERNAL_DRIVER ||
						pO->type == eCV_AVATAR
						)
					&&
					( pO->phase == eALIVE || pO->phase == eDYING )
					);
		bool localObj = (pO->type == eCV_VEHICLE);
		bool remoteVeh = (pO->type == eCV_EXTERNAL_DRIVER);
		bool remotePed = (pO->type == eCV_AVATAR);

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

					bool received = ( !remoteVeh || ctrl->OnGetUpdate(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState))
								 && ( !remotePed || ctrl->OnGetUpdateArt(id, pFutState));
					if (!received)
						*pFutState = *pCurrState;
					if (localObj)
					{
						DynamicModel(
								id,
								pO->type,
								&pO->attr,
								pCurrState,
								pCurrContInp,
								pFutState
								);
						ctrl->OnPushUpdate(id, pCurrContInp, pFutState);
					}

				}
			}
			else
			{
				CVED::CCved &me = *this;
				bool received = ( !remoteVeh || ctrl->OnGetUpdate(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState))
							 && ( !remotePed || ctrl->OnGetUpdateArt(id, pFutState));
				if (!received)
					*pFutState = *pCurrState;

				if (localObj) //local simulator has its own dynamic
				{
					DynamicModel(
								id,
								pO->type,
								&pO->attr,
								pCurrState,
								pCurrContInp,
								pFutState
								);
					ctrl->OnPushUpdate(id, pCurrContInp, pFutState);
				}
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
		bool localObj = (0 == attachedObjIds[i]);
		bool remoteVeh = (0 != attachedObjIds[i]
					&& pO->type == eCV_EXTERNAL_DRIVER);
		bool remotePed = (0 != attachedObjIds[i]
					&& pO->type == eCV_AVATAR);

		bool received = ( !remoteVeh || ctrl->OnGetUpdate(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState))
					 && ( !remotePed || ctrl->OnGetUpdateArt(id, pFutState));
		if (!received)
			*pFutState = *pCurrState;

		if (localObj) //local simulator has its own dynamic
		{
			DynamicModel(
					attachedObjIds[i],
					pO->type,
					&pO->attr,
					pCurrState,
					pCurrContInp,
					pFutState
					);
			ctrl->OnPushUpdate(id, pCurrContInp, pFutState);
		}
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

CDynObj* CCvedADOCtrl::LocalCreateEDO(
					bool				own,
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos,
					const CVector3D*	cpInitTan,
					const CVector3D*	cpInitLat)
{
	return CCvedDistri::CreateDynObj(cName, eCV_EXTERNAL_DRIVER, cAttr, cpInitPos, cpInitTan, cpInitLat);
}


void CCvedADOCtrl::Maintainer(void)
{
	int    i;
	TObj   *pO;

	if ( m_debug > 0 ) {
		gout << "->Enter CCved::Maintainer, frm=" << m_pHdr->frame << endl;
	}

	// implement the object phase transition diagram
	LockObjectPool();
	for (i=0, pO = BindObj(0); i<cNUM_DYN_OBJS; i++, pO++) {
		if ( pO->phase == eBORN ) {
			if ( m_debug > 2 ) {
				gout << "Object " << i << " from Born->Alive " << endl;
			}
			pO->phase = eALIVE;
			if (eCV_VEHICLE == pO->type) //an ado object has state transfered from eBORN->eALIVE
			{
				const CDynObj* pVehicle = BindObjIdToClass(i);
				CPoint3D pos = pVehicle->GetPos();
				CVector3D tan = pVehicle->GetTan();
				CVector3D lat = pVehicle->GetLat();
				m_pCtrl->OnCreateADO(i, pO->name, pO->attr, pos, tan, lat);
			}

		}
		else
		if ( pO->phase == eDYING ) {
			if (eCV_VEHICLE == pO->type) //an ado object has state transfered from eDYING->eDEAD
				m_pCtrl->OnDeleteADO(i);

			if ( m_debug > 2 ) {
				gout << "Object " << i << " from Dying->Dead " << endl;
			}
			pO->phase = eDEAD;
			m_pHdr->dynObjectCount--;

		}
	}

	//
	// Copy dynamic object state between buffers.
	//
	for( i = 0, pO = BindObj( 0 ); i < cNUM_DYN_OBJS; i++, pO++ )
	{
		if( pO->phase == eALIVE )
		{
			bool avatar_state = (pO->type == eCV_AVATAR
							|| pO->type == eCV_EXTERNAL_AVATAR);
			TAvatarJoint* j_a = NULL;
			TAvatarJoint* j_b = NULL;
			if (avatar_state)
			{
				j_a = pO->stateBufA.state.avatarState.child_first;
				j_b = pO->stateBufB.state.avatarState.child_first;
			}
			if( (m_pHdr->frame & 1) == 0 )
			{		// even frame
				pO->stateBufA.state = pO->stateBufB.state;
			}
			else {
				pO->stateBufB.state = pO->stateBufA.state;
			}
			if (avatar_state)
			{
				pO->stateBufA.state.avatarState.child_first = j_a;
				pO->stateBufB.state.avatarState.child_first = j_b;
			}

		}
	}
	m_pHdr->frame++;
	UnlockObjectPool();

	FillByRoadDynObjList();

#ifdef DEBUG_MAINTAINER
if( m_pHdr->frame > 0 ) {

	gout << "<<<<< *************** Maintainer [frame=" <<  m_pHdr->frame;
	gout << "] *************** >>>>>" << endl;

	int s;
	for( s = 0; s < m_pHdr->roadRefCount; s++ )
	{
		if( m_pRoadRefPool[s].objIdx != 0 )
		{
			gout << "RoadRefPool[" << s << "].objIdx = ";
			gout << m_pRoadRefPool[s].objIdx << endl;

		}
	}
	for( s = 0; s < m_pHdr->intrsctnRefCount; s++ )
	{
		if( m_pIntrsctnRefPool[s].objIdx != 0 )
		{
			gout << "IntrsctnRefPool[" << s << "].objIdx = ";
			gout << m_pIntrsctnRefPool[s].objIdx << endl;
		}
	}

	for( s = 0; s < m_pHdr->dynObjRefCount; s++ )
	{
		if( m_pDynObjRefPool[s].objId == 0 ) continue;

		gout << "DynObjRefPool[" << s << "]:" << endl;

		cvTDynObjRef* pDorNode = &( m_pDynObjRefPool[s] );
		if( pDorNode->objId != 0 )
			gout << "  objId = "<< pDorNode->objId << endl;
		if( (int)pDorNode->terrain != 0 )
			gout << "  terrain = "<< (int)pDorNode->terrain << endl;
		if( pDorNode->next != 0 )
			gout<< "  next = " << pDorNode->next << endl;
		if( pDorNode->roadId != 0 )
			gout<< "  roadId = " << pDorNode->roadId << endl;
		if( pDorNode->lanes != 0 )
			gout << "  lanes = " << pDorNode->lanes << endl;
		if( pDorNode->distance != 0.0f )
			gout << "  distance = " << pDorNode->distance << endl;
		if( (int)pDorNode->direction != 0 )
			gout << "  direction = " << (int)pDorNode->direction << endl;
		if( pDorNode->roadCntrlPnt != 0 )
			gout << "  roadCntrlPnt = " << pDorNode->roadCntrlPnt << endl;
		if( pDorNode->intrsctnId != 0 )
			gout << "  intrsctnId = " << pDorNode->intrsctnId << endl;
		if( pDorNode->corridors != 0 )
			gout << "  corridors = " << pDorNode->corridors << endl;

		int t;
		for( t = 0; t < cCV_MAX_CRDRS; t++ )
		{
			if( pDorNode->crdrDistances[t] != 0.0f )
				gout <<  "  crdrDistances[t] = " << pDorNode->crdrDistances[t] << endl;
		}
	}

	// print out objs on road 2
	int curRoadId = 14;
	int curRoadIdx = m_pRoadRefPool[curRoadId].objIdx;
	gout << "objs on road " << curRoadId << " are:  " ;
	while( curRoadIdx != 0 )
	{
		gout << m_pDynObjRefPool[curRoadIdx].objId << ",";
		curRoadIdx = m_pDynObjRefPool[curRoadIdx].next;
	}
	gout << endl;


	// print out objs on intersection 2
	int curIntrsctnId = 37;
	int curIntrsctnIdx = m_pIntrsctnRefPool[curIntrsctnId].objIdx;
	gout << "objs on intersection " << curIntrsctnId << " are:  ";
	while( curIntrsctnIdx != 0 )
	{
		gout << m_pDynObjRefPool[curIntrsctnIdx].objId << ",";
		curIntrsctnIdx = m_pDynObjRefPool[curIntrsctnIdx].next;
	}
	gout << endl;
}


#endif


	if ( m_debug > 0 ) {
		gout << "<-Exit CCved::Maintainer." << endl;
	}
} // end of Maintainer

CDynObj*	CCvedADOCtrl::DistriCreateADO(const string&		cName,
								const cvTObjAttr&	cAttr,
								const CPoint3D*		cpInitPos,
								const CVector3D*	cpInitTan,
								const CVector3D*	cpInitLat)
{
	CDynObj* obj =  CCvedDistri::CreateDynObj(cName, eCV_VEHICLE, cAttr, cpInitPos, cpInitTan, cpInitLat);
	return obj;
}

void CCvedADOCtrl::DistriDeleteADO( CDynObj* obj )
{
	CCvedDistri::DeleteDynObj(obj);
}

CDynObj* CCvedADOCtrl::LocalCreatePDO(
					bool 				own,
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos,
					const CVector3D*	cpInitTan,
					const CVector3D*	cpInitLat)
{
	cvEObjType type = (own?eCV_EXTERNAL_AVATAR:eCV_AVATAR);
	return CCvedDistri::CreateDynObj(cName, type, cAttr, cpInitPos, cpInitTan, cpInitLat);
}

}