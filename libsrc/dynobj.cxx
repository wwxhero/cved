//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: dynobj.cxx,v 1.80 2016/07/15 14:36:05 IOWA\dheitbri Exp $
//
// Author(s):
// Date:		September, 1998
//
// Description:	The implementation of the CDynObj class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/

extern cvTFourWheelVeh	g_stateVector[cNUM_DYN_OBJS];
extern cvTVehLin		g_vehLin[cNUM_DYN_OBJS];


namespace CVED{

/////////////////////////////////////////////////////////////////////////////
//
// Description: ~CDynObj
// 	Destructor.
//
// Remarks:  If the current instance is not marked "read only", then set
// 	the associated object's phase to "dying", indicating that the object
// 	should be deleted.  See MakeReadOnly() for a description of the reason
// 	for "read only" CDynObj instances.
//
// Arguments:
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
CDynObj::~CDynObj()
{
	if( !m_readOnly )
		m_pObj->phase = eDYING;
} // end of ~CDynObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the position of a dynamic object.
//
// Remarks: This function sets the current position of the current dynamic
// 	object.  The specified position won't be visible to queries until after
// 	the next execution of the maintainer.
//
// 	This function is only to be used in place of a dynamic server.  If it is
// 	called for an object that also has an automatic dynamic model within CVED,
// 	then the results are unpredictable.
//
// Arguments:
// 	cPos - The position.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CDynObj::SetPos( const CPoint3D& cPos, bool useDoubleBuffer )
{
	AssertValid();

	cvTObjState::AnyObjState* pSt;
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( useDoubleBuffer )
	{
		if( (pH->frame & 1) == 0 )
		{
			pSt = &m_pObj->stateBufB.state.anyState;
		}
		else
		{
			pSt = &m_pObj->stateBufA.state.anyState;
		}

		pSt->position.x = cPos.m_x;
		pSt->position.y = cPos.m_y;
		pSt->position.z = cPos.m_z;
	}
	else
	{
		pSt = &m_pObj->stateBufA.state.anyState;
		pSt->position.x = cPos.m_x;
		pSt->position.y = cPos.m_y;
		pSt->position.z = cPos.m_z;

		pSt = &m_pObj->stateBufB.state.anyState;
		pSt->position.x = cPos.m_x;
		pSt->position.y = cPos.m_y;
		pSt->position.z = cPos.m_z;
	}

	// update the bounding box
	CPoint3D ll, ur;
	CCved::UpdateBBox(
				&m_pObj->attr,
				pSt->tangent,
				pSt->lateral,
				pSt->position,
				ll,
				ur
				);

	pSt->boundBox[0].x = ll.m_x;
	pSt->boundBox[0].y = ll.m_y;
	pSt->boundBox[0].z = ll.m_z;

	pSt->boundBox[1].x = ur.m_x;
	pSt->boundBox[1].y = ur.m_y;
	pSt->boundBox[1].z = ur.m_z;

} // end of SetPos

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the tangent direction of a dynamic object.
//
// Remarks: This function sets the tangent direction of the orientation for
// 	the current dynamic object.  The specified data won't be visible to
// 	queries, until after the next execution of the maintainer.
//
// 	This function is only to be used in place of a dynamic server.  If it is
// 	called for an object that also has an automatic dynamic model within CVED,
// 	then the results are unpredictable.
//
// 	The function expects the vector to be normalized and non-zero.  If it is
// 	not, a lot of calculations could be erroneous.
//
// Arguments:
// 	cTan - The tangent vector, normalized.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CDynObj::SetTan( const CVector3D& cTan, bool useDoubleBuffer )
{
	AssertValid();

	cvTObjState::AnyObjState* pSt;
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( useDoubleBuffer )
	{
		if( (pH->frame & 1) == 0 )
		{		// even frame
			pSt = &m_pObj->stateBufB.state.anyState;
		}
		else
		{
			pSt = &m_pObj->stateBufA.state.anyState;
		}

		pSt->tangent.i = cTan.m_i;
		pSt->tangent.j = cTan.m_j;
		pSt->tangent.k = cTan.m_k;
	}
	else
	{
		pSt = &m_pObj->stateBufA.state.anyState;
		pSt->tangent.i = cTan.m_i;
		pSt->tangent.j = cTan.m_j;
		pSt->tangent.k = cTan.m_k;

		pSt = &m_pObj->stateBufB.state.anyState;
		pSt->tangent.i = cTan.m_i;
		pSt->tangent.j = cTan.m_j;
		pSt->tangent.k = cTan.m_k;
	}

	// update the bounding box
	CPoint3D ll, ur;
	CCved::UpdateBBox(
				&m_pObj->attr,
				pSt->tangent,
				pSt->lateral,
				pSt->position,
				ll,
				ur
				);

	pSt->boundBox[0].x = ll.m_x;
	pSt->boundBox[0].y = ll.m_y;
	pSt->boundBox[0].z = ll.m_z;

	pSt->boundBox[1].x = ur.m_x;
	pSt->boundBox[1].y = ur.m_y;
	pSt->boundBox[1].z = ur.m_z;
} // end of SetTan

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the lateral direction of a dynamic object.
//
// Remarks: This function sets the lateral direction of the orientation for
// 	the current dynamic object.  The specified data won't be visible to
// 	queries, until after the next execution of the maintainer.
//
// 	This function is only to be used in place of a dynamic server.  If it is
// 	called for an object that also has an automatic dynamic model within CVED,
// 	then the results are unpredictable.
//
// 	The function expects the vector to be normalized and non-zero.  If it is
// 	not, a lot of calculations could be erroneous.
//
// Arguments:
// 	cLat - The lateral vector, normalized.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CDynObj::SetLat( const CVector3D& cLat, bool useDoubleBuffer )
{
	AssertValid();

	cvTObjState::AnyObjState* pSt;
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( useDoubleBuffer )
	{
		if( (pH->frame & 1) == 0 )
		{
			pSt = &m_pObj->stateBufB.state.anyState;
		}
		else
		{
			pSt = &m_pObj->stateBufA.state.anyState;
		}

		pSt->lateral.i = cLat.m_i;
		pSt->lateral.j = cLat.m_j;
		pSt->lateral.k = cLat.m_k;
	}
	else
	{
		pSt = &m_pObj->stateBufA.state.anyState;
		pSt->lateral.i = cLat.m_i;
		pSt->lateral.j = cLat.m_j;
		pSt->lateral.k = cLat.m_k;

		pSt = &m_pObj->stateBufB.state.anyState;
		pSt->lateral.i = cLat.m_i;
		pSt->lateral.j = cLat.m_j;
		pSt->lateral.k = cLat.m_k;
	}

	// update the bounding box
	CPoint3D ll, ur;
	CCved::UpdateBBox(
				&m_pObj->attr,
				pSt->tangent,
				pSt->lateral,
				pSt->position,
				ll,
				ur
				);

	pSt->boundBox[0].x = ll.m_x;
	pSt->boundBox[0].y = ll.m_y;
	pSt->boundBox[0].z = ll.m_z;

	pSt->boundBox[1].x = ur.m_x;
	pSt->boundBox[1].y = ur.m_y;
	pSt->boundBox[1].z = ur.m_z;
} // end of SetLat

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the velocity of a dynamic object.
//
// Remarks:  This function sets the velocity of a dynamic object. This new
//   velocity becomes visible during the next execution of CVED.
//
// Arguments:
//   vel - The new velocity.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CDynObj::SetVel( double vel, bool useDoubleBuffer )
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader *>( GetInst() );

	if( useDoubleBuffer )
	{
		if( ( pH->frame & 1 ) == 0 )
		{
			m_pObj->stateBufB.state.anyState.vel = vel;
		}
		else
		{
			m_pObj->stateBufA.state.anyState.vel = vel;
		}
	}
	else
	{
		m_pObj->stateBufA.state.anyState.vel = vel;
		m_pObj->stateBufB.state.anyState.vel = vel;
	}
} // end of SetVel


//////////////////////////////////////////////////////////////////////////////
//              TRAJECTORY FOLLOWER MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: InitModeValues
// 	   Initialize traj follower state values that are related to modes
//
// Remarks:
//    Default values include ground traj object mode, no parent, and
//    not using posrot(offset) and velocities as absolute values, which
//    indicates the pos, rot and vel values are not assigned, so that
//    the dynamics will know to calculate them when transitioned to the
//    proper modes.
//
// Arguments:
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::InitModeValues( void )
{
	if ( m_pObj )
	{
		m_pObj->stateBufA.state.trajFollowerState.curMode = eCV_GROUND_TRAJ;
		m_pObj->stateBufA.state.trajFollowerState.prevMode = eCV_GROUND_TRAJ;
		m_pObj->stateBufA.state.trajFollowerState.parentId = -1;
		m_pObj->stateBufA.state.trajFollowerState.offset[0] = 0;
		m_pObj->stateBufA.state.trajFollowerState.offset[1] = 0;
		m_pObj->stateBufA.state.trajFollowerState.offset[2] = 0;
		m_pObj->stateBufA.state.trajFollowerState.offset[3] = 0;
		m_pObj->stateBufA.state.trajFollowerState.offset[4] = 0;
		m_pObj->stateBufA.state.trajFollowerState.offset[5] = 0;
		m_pObj->stateBufA.state.trajFollowerState.initVel[0] = 0;
		m_pObj->stateBufA.state.trajFollowerState.initVel[1] = 0;
		m_pObj->stateBufA.state.trajFollowerState.initVel[2] = 0;
		m_pObj->stateBufA.state.trajFollowerState.initVel[3] = 0;
		m_pObj->stateBufA.state.trajFollowerState.initVel[4] = 0;
		m_pObj->stateBufA.state.trajFollowerState.initVel[5] = 0;
		m_pObj->stateBufA.state.trajFollowerState.useAsAbsoluteValue = false;

		m_pObj->stateBufB.state.trajFollowerState.curMode = eCV_GROUND_TRAJ;
		m_pObj->stateBufB.state.trajFollowerState.prevMode = eCV_GROUND_TRAJ;
		m_pObj->stateBufB.state.trajFollowerState.parentId = -1;
		m_pObj->stateBufB.state.trajFollowerState.offset[0] = 0;
		m_pObj->stateBufB.state.trajFollowerState.offset[1] = 0;
		m_pObj->stateBufB.state.trajFollowerState.offset[2] = 0;
		m_pObj->stateBufB.state.trajFollowerState.offset[3] = 0;
		m_pObj->stateBufB.state.trajFollowerState.offset[4] = 0;
		m_pObj->stateBufB.state.trajFollowerState.offset[5] = 0;
		m_pObj->stateBufB.state.trajFollowerState.initVel[0] = 0;
		m_pObj->stateBufB.state.trajFollowerState.initVel[1] = 0;
		m_pObj->stateBufB.state.trajFollowerState.initVel[2] = 0;
		m_pObj->stateBufB.state.trajFollowerState.initVel[3] = 0;
		m_pObj->stateBufB.state.trajFollowerState.initVel[4] = 0;
		m_pObj->stateBufB.state.trajFollowerState.initVel[5] = 0;
		m_pObj->stateBufB.state.trajFollowerState.useAsAbsoluteValue = false;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Initialize the mode of the traj follower
//
// Remarks: Sets both previous mode and current mode to the specified value.
//
// Arguments: mode to set to
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::InitializeMode( unsigned char initMode )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	//if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.trajFollowerState.curMode = initMode;
		m_pObj->stateBufB.state.trajFollowerState.prevMode = initMode;
	}
	//else
	{
		m_pObj->stateBufA.state.trajFollowerState.curMode = initMode;
		m_pObj->stateBufA.state.trajFollowerState.prevMode = initMode;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Get the current mode of the traj follower
//
// Remarks:In combination of the previous mode, the dynamics can tell
//          if a traj follower has just changed mode and perform proper
//          initialization for the new mode. After this, the dynamics
//          needs to set the previous mode to be the same as the current
//          mode.
//
// Arguments: void
//
// Returns: current mode of the traj follower
//
/////////////////////////////////////////////////////////////////////////////
unsigned char
CTrajFollowerObj::GetCurrentMode( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.trajFollowerState.curMode;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.trajFollowerState.curMode;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Set the current mode of the traj follower
//
// Remarks:
//
// Arguments: mode to set to
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetCurrentMode( unsigned char curMode )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.trajFollowerState.curMode = curMode;
	}
	else
	{
		m_pObj->stateBufA.state.trajFollowerState.curMode = curMode;
	}
}

/*
/////////////////////////////////////////////////////////////////////////////
//
// Description: Get the previous mode of the traj follower
//
// Remarks:
//
// Arguments: void
//
// Returns: previous mode of the traj follower
//
/////////////////////////////////////////////////////////////////////////////
unsigned char
CTrajFollowerObj::GetPreviousMode( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.trajFollowerState.prevMode;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.trajFollowerState.prevMode;
	}
}
*/

/*
/////////////////////////////////////////////////////////////////////////////
//
// Description: Set the previous mode of the traj follower
//
// Remarks: This function should only be used for initialization, right
//          after an traj follower object is created, and by the
//          dynamics, after finishing the initialization work for the
//          new mode of an newly transitioned object.
//
// Arguments: previous mode to set to
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetPreviousMode( unsigned char prevMode )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.trajFollowerState.prevMode = prevMode;
	}
	else
	{
		m_pObj->stateBufA.state.trajFollowerState.prevMode = prevMode;
	}
}
*/

/////////////////////////////////////////////////////////////////////////////
//
// Description: Get the parent cved id of the traj follower
//
// Remarks: parent id is used when a traj follower is in coupled obj mode
//          or obj relative traj mode
//
// Arguments: void
//
// Returns: parent cved id of the traj follower
//
/////////////////////////////////////////////////////////////////////////////
int
CTrajFollowerObj::GetParentId( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.trajFollowerState.parentId;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.trajFollowerState.parentId;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Set the parent cved id of the traj follower
//
// Remarks:
//
// Arguments: parent id
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetParentId( int parentId )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.trajFollowerState.parentId = parentId;
	}
	else
	{
		m_pObj->stateBufA.state.trajFollowerState.parentId = parentId;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Get the offset of the traj follower w.r.t. its parent
//
// Remarks: Offset is used in the coupled obj mode and obj relative
//          traj follower mode.
//
// Arguments: Pointer to the offset array to write to
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::GetOffset( double offset[6] )
{
	int i;

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		for ( i=0; i<6; ++i )
			offset[i] = m_pObj->stateBufA.state.trajFollowerState.offset[i];
	}
	else
	{
		// odd frame
		for ( i=0; i<6; ++i )
			offset[i] = m_pObj->stateBufB.state.trajFollowerState.offset[i];
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Set the offset of the traj follower w.r.t. its parent
//
// Remarks: Used in the coupled object and object relative trajectory modes.
//
// Arguments: pointer to the input offset array
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetOffset( const double offset[6] )
{
	int i;
	double cr, cp, cy, sr, sp, sy;

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	cr = cos( offset[3] );
	sr = sin( offset[3] );
	cp = cos( offset[4] );
	sp = sin( offset[4] );
	cy = cos( offset[5] );
	sy = sin( offset[5] );

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		for ( i=0; i<6; ++i )
			m_pObj->stateBufB.state.trajFollowerState.offset[i] = offset[i];
		m_pObj->stateBufB.state.trajFollowerState.rotMat[0][0] = cy * cp;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[0][1] = -sy * cr - cy * sp *sr;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[0][2] = sy * sr - cy * sp *cr;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[1][0] = sy * cp;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[1][1] = cy * cr - sy * sp * sr;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[1][2] = -cy * sr - sy * sp * cr;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[2][0] = sp;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[2][1] = cp * sr;
		m_pObj->stateBufB.state.trajFollowerState.rotMat[2][2] = cp * cr;

	}
	else
	{
		for ( i=0; i<6; ++i )
			m_pObj->stateBufA.state.trajFollowerState.offset[i] = offset[i];
		m_pObj->stateBufA.state.trajFollowerState.rotMat[0][0] = cy * cp;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[0][1] = -sy * cr - cy * sp *sr;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[0][2] = sy * sr - cy * sp *cr;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[1][0] = sy * cp;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[1][1] = cy * cr - sy * sp * sr;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[1][2] = -cy * sr - sy * sp * cr;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[2][0] = sp;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[2][1] = cp * sr;
		m_pObj->stateBufA.state.trajFollowerState.rotMat[2][2] = cp * cr;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Get the initial position, rotation and velocity of the object
//
// Remarks: Used in the free motion mode. Position and rotation are stored in
//          the member variable offset[6], which is also used to store the
//          offset in coupled obj and obj relative traj follower modes.
//
// Arguments: pointers to init pos rot array and velocity array to write to
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::GetInitPosRotVel( double initPosRot[6], double initVel[6] )
{
	int i;

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		for ( i=0; i<6; ++i )
		{
			initPosRot[i] = m_pObj->stateBufA.state.trajFollowerState.offset[i];
			initVel[i] = m_pObj->stateBufA.state.trajFollowerState.initVel[i];
		}
	}
	else
	{
		// odd frame
		for ( i=0; i<6; ++i )
		{
			initPosRot[i] = m_pObj->stateBufB.state.trajFollowerState.offset[i];
			initVel[i] = m_pObj->stateBufB.state.trajFollowerState.initVel[i];
		}
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Set the initial position, rotation and velocity of the object
//
// Remarks: Used in the free motion mode
//
// Arguments: pointers to input initial pos rot array and velocity array
//
// Returns: void
//
/////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetInitPosRotVel( const double initPosRot[6], const double initVel[6] )
{
	int i;

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		for ( i=0; i<6; ++i )
		{
			m_pObj->stateBufB.state.trajFollowerState.offset[i] = initPosRot[i];
			m_pObj->stateBufB.state.trajFollowerState.initVel[i] = initVel[i];
		}
	}
	else
	{
		for ( i=0; i<6; ++i )
		{
			m_pObj->stateBufA.state.trajFollowerState.offset[i] = initPosRot[i];
			m_pObj->stateBufA.state.trajFollowerState.initVel[i] = initVel[i];
		}
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target direction of the trajfollower object.
//
// Remarks: This function sets the target direction of the current
// 	trajfollower object.  The specified data won't be visible to queries
// 	until after the next execution of the maintainer.
//
// 	The function expects the vector to be normalized and non-zero.  If it is
// 	not, a lot of calculations could be erroneous.
//
// Arguments:
// 	cDir - The direction.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetTargDir( const CVector2D& cDir )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.trajFollowerContInp.targDir.i = cDir.m_i;
		m_pObj->stateBufB.contInp.trajFollowerContInp.targDir.j = cDir.m_j;
	}
	else
	{
		m_pObj->stateBufA.contInp.trajFollowerContInp.targDir.i = cDir.m_i;
		m_pObj->stateBufA.contInp.trajFollowerContInp.targDir.j = cDir.m_j;
	}
} // end of SetTargDir

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the orienation of the trajfollower object.
//
// Remarks: The orientation is independent of the object direction.
//
// 	The function expects the vector to be normalized and non-zero.  If it is
// 	not, a lot of calculations could be erroneous.
//
// Arguments:
// 	cOri - The orienation.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetObjOri( const CVector2D& cOri )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.trajFollowerContInp.objOri.i = cOri.m_i;
		m_pObj->stateBufB.contInp.trajFollowerContInp.objOri.j = cOri.m_j;
	}
	else
	{
		m_pObj->stateBufA.contInp.trajFollowerContInp.objOri.i = cOri.m_i;
		m_pObj->stateBufA.contInp.trajFollowerContInp.objOri.j = cOri.m_j;
	}
} // end of SetObjOri

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target velocity of the trajfollower object.
//
// Remarks: This function sets the target velocity of the current
// 	trajfollower object.  The specified data won't be visible to queries
// 	until after the next execution of the maintainer.
//
// Arguments:
// 	vel - The velocity.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetTargVel( double vel )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());

	if( (pH->frame & 1) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.trajFollowerContInp.targVel = vel;
	}
	else
	{
		m_pObj->stateBufA.contInp.trajFollowerContInp.targVel = vel;
	}
} // end of SetTargVel

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the visual state of the current trajfollower.
//
// Remarks:
//
// Arguments:
//
// Returns: An unsigned 8-bit integer containing the visual state bits.
//
//////////////////////////////////////////////////////////////////////////////
TU16b
CTrajFollowerObj::GetVisualState( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.trajFollowerState.visualState;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.trajFollowerState.visualState;
	}
} // end of GetVisualState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the visual state of the current trajfollower.
//
// Remarks: This function sets the visual state of the current
// 	trajfollower object.  The specified data won't be visible to queries
// 	until after the next execution of the maintainer.
//
// Arguments:
// 	visualState - An unsigned 8-bit integer containing the desired visual
//                state.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetVisualState( TU16b visualState )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.trajFollowerState.visualState = visualState;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.trajFollowerState.visualState = visualState;
	}
} // end of SetVisualState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the audio state of the current trajfollower.
//
// Remarks:
//
// Arguments:
//
// Returns: An unsigned 16-bit integer containing the audio state bits.
//
//////////////////////////////////////////////////////////////////////////////
TU16b
CTrajFollowerObj::GetAudioState( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.trajFollowerState.audioState;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.trajFollowerState.audioState;
	}
} // end of GetAudioState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the audio state of the current trajfollower.
//
// Remarks: This function sets the audio state of the current
// 	trajfollower object.  The specified data won't be visible to queries
// 	until after the next execution of the maintainer.
//
// Arguments:
// 	audioState - An unsigned 8-bit integer containing the desired audio state.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrajFollowerObj::SetAudioState( TU16b audioState )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.trajFollowerState.audioState = audioState;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.trajFollowerState.audioState = audioState;
	}
} // end of SetAudioState

bool
CTrajFollowerObj::GetAnimationState(void) const{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return 0 < (m_pObj->stateBufB.state.trajFollowerState.visualState & 0x8000);
	}
	else
	{
		// odd frame
		return 0 < (m_pObj->stateBufB.state.trajFollowerState.visualState & 0x8000);
	}
}
void
CTrajFollowerObj::SetAnimationState(bool isOn){
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		if ( isOn )
			m_pObj->stateBufB.state.trajFollowerState.visualState |= 0x8000;
		else
			m_pObj->stateBufB.state.trajFollowerState.visualState &= ~(0x8000);
	}
	else
	{
		// odd frame
		if ( isOn )
			m_pObj->stateBufA.state.trajFollowerState.visualState |= 0x8000;
		else
			m_pObj->stateBufA.state.trajFollowerState.visualState &= ~(0x8000);
	}
}

void
CWalkerObj::SetAnimationState(bool isOn){
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.walkerState.animationOn = isOn;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.walkerState.animationOn = isOn;
	}
}
//////////////////////////////////////////////////////////////////////////////
//              TWO-WHEELED VEHICLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              MOTORCYCLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              BICYCLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              VEHICLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target distance of the vehicle object.
//
// Remarks: This function sets the target distance of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cDist - The target distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTargDist( const double& cDist )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targDist = cDist;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targDist = cDist;
	}
} // end of SetTargDist

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target position of the vehicle object.
//
// Remarks: This function sets the target position of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cPos - The target position.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTargPos( const CPoint3D& cPos )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos.x = cPos.m_x;
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos.y = cPos.m_y;
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos.z = cPos.m_z;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos.x = cPos.m_x;
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos.y = cPos.m_y;
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos.z = cPos.m_z;
	}
} // end of SetTargPos
//////////////////////////////////////////////////////////////////////////////
///\brief
///   store the old target point so dyna can iterate between current and
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::StoreOldTargPos(){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.oldtargPos.x = m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos.x;
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.oldtargPos.y = m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos.y;
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.oldtargPos.z = m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos.z;
	}
	else
	{
		// odd frame
        m_pObj->stateBufA.contInp.vehicleContInp.contInp.oldtargPos.x = m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos.x;
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.oldtargPos.y = m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos.y;
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.oldtargPos.z = m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos.z;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target velocity of the vehicle object.
//
// Remarks: This function sets the target velocity of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cVel - The target velocity.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTargVel( const double& cVel )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targVel = cVel;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targVel = cVel;
	}
} // end of SetTargVel

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the lead object id of the vehicle object.
//
// Remarks: This function sets the lead object id of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cLeadObj - The lead object's id.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetLeadObj( const int cLeadObj )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.leadObj = cLeadObj;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.leadObj = cLeadObj;
	}
} // end of SetLeadObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target following distance of the vehicle object.
//
// Remarks: This function sets the target following distance of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cTargDist - The following distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetLeadTargDist( const double& cTargDist )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.leadTargDist = cTargDist;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.leadTargDist = cTargDist;
	}
} // end of SetLeadTargDist

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the actual following distance of the vehicle object.
//
// Remarks: This function sets the actual following distance of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cActualDist - The actual distance to the lead object.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetLeadActualDist( const double& cActualDist )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.leadActualDist = cActualDist;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.leadActualDist = cActualDist;
	}
} // end of SetLeadActualDist

//////////////////////////////////////////////////////////////////////////////
//
// Description: Indicate if the vehicle object has a target steering input.
//
// Remarks: The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 haveSteer - Does the vehicle have a valid steering input.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetHaveSteer( bool haveSteer )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.haveSteer = haveSteer;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.haveSteer = haveSteer;
	}
} // end of SetHaveSteer

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target steering of the vehicle object.
//
// Remarks: This function sets the target steering of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cSteer - The target steering.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTargSteer( const double& cSteer )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targSteer = (float)cSteer;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targSteer = (float)cSteer;
	}
} // end of SetTargSteer

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the max steering rate (Rads per Sec) of the vehicle object.
//
// Remarks: This function sets the max steering rate for the
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cSteerMax - The target steering.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetSteerMax( const double& cSteerMax )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.maxSteerRateRadpS = (float)cSteerMax;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.maxSteerRateRadpS = (float)cSteerMax;
	}
} // end of SetTargSteer
//////////////////////////////////////////////////////////////////////////////
//
// Description: Indicate if the vehicle object has a target acceleration
//   input.
//
// Remarks: The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 haveSteer - Does the vehicle have a valid acceleration input.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetHaveAccel( bool haveAccel )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.haveAccel = haveAccel;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.haveAccel = haveAccel;
	}
} // end of SetHaveAccel


//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the target acceleration of the vehicle object.
//
// Remarks: This function sets the target acceleration of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cAccel - The target acceleration.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTargAccel( const double& cAccel )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.contInp.vehicleContInp.contInp.targAccel = cAccel;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.contInp.vehicleContInp.contInp.targAccel = cAccel;
	}
} // end of SetTargAccel

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the suspension dampening of the vehicle object.
//
// Remarks:  This function gets the susupension dampening of the
//   vehicle object.
//
// Arguments:
//
// Returns: A double containing the suspension dampening.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetSuspDamp( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.suspDamp;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.suspDamp;
	}

} // end of GetSuspDamp

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the immediate suspension dampening of the
//   vehicle object.
//
// Remarks:  This function gets the immediate susupension dampening
//   of the vehicle object, bypassing the double-buffering.
//
// 	 This function may return inconsistant values if it is called multiple
// 	 times within a frame.
//
// Arguments:
//
// Returns: A double containing the immediate suspension dampening.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetSuspDampImm( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.suspDamp;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.suspDamp;
	}

} // end of GetSuspDampImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the suspension dampening of the vehicle object.
//
// Remarks: This function sets the susp damp of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cSuspDamp - The suspension dampening.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetSuspDamp( const double& cSuspDamp )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.suspDamp = cSuspDamp;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.suspDamp = cSuspDamp;
	}

} // end of SetSuspDamp

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the suspension dampening of the vehicle object in
//   current frame.
//
// Remarks: This function sets the suspension dampening of the current
// 	 vehicle object.  The values are written to the current frame,
//   bypassing the double-buffering.
//
// Arguments:
// 	 cSuspDamp - The suspension dampening.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetSuspDampImm( const double& cSuspDamp )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.suspDamp = cSuspDamp;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.suspDamp = cSuspDamp;
	}

} // end of SetSuspDampImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the suspension steffness of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the suspension stiffness.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetSuspStif( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.suspStif;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.suspStif;
	}

} // end of GetSuspStif

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the immediate suspension stiffness of the
//   vehicle object.
//
// Remarks:  This function gets the immediate susupension stiffness
//   of the vehicle object, bypassing the double-buffering.
//
// 	 This function may return inconsistant values if it is called multiple
// 	 times within a frame.
//
// Arguments:
//
// Returns: A double containing the immediate suspension stiffness.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetSuspStifImm( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.suspStif;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.suspStif;
	}

} // end of GetSuspStifImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the suspension stiffness of the vehicle object.
//
// Remarks: This function sets the susp stif of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cSuspStif - The suspension stiffness.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetSuspStif( const double& cSuspStif )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.suspStif = cSuspStif;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.suspStif = cSuspStif;
	}

} // end of SetSuspStif

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the suspension stiffness of the vehicle object in
//   current frame.
//
// Remarks: This function sets the suspension stiffness of the current
// 	 vehicle object.  The values are written to the current frame,
//   bypassing the double-buffering.
//
// Arguments:
// 	 cSuspDamp - The suspension stiffness.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetSuspStifImm( const double& cSuspStif )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.suspStif = cSuspStif;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.suspStif = cSuspStif;
	}

} // end of SetSuspStifImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the tire dampening of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the tire dampening.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetTireDamp( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.tireDamp;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.tireDamp;
	}

} // end GetTireDamp

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the immediate tire dampening of the
//   vehicle object.
//
// Remarks:  This function gets the immediate tire dampening
//   of the vehicle object, bypassing the double-buffering.
//
// 	 This function may return inconsistant values if it is called multiple
// 	 times within a frame.
//
// Arguments:
//
// Returns: A double containing the immediate tire dampening.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetTireDampImm( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.tireDamp;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.tireDamp;
	}

} // end of GetTireDampImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the tire dampening of the vehicle object.
//
// Remarks: This function sets the tire damp of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cTireDamp - The tire dampening.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTireDamp( const double& cTireDamp )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.tireDamp = cTireDamp;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.tireDamp = cTireDamp;
	}

} // end of SetTireDamp

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the tire dampening of the vehicle object in
//   current frame.
//
// Remarks: This function sets the tire dampening of the current
// 	 vehicle object.  The values are written to the current frame,
//   bypassing the double-buffering.
//
// Arguments:
// 	 cTireDamp - The tire dampening.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTireDampImm( const double& cTireDamp )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.tireDamp = cTireDamp;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.tireDamp = cTireDamp;
	}

} // end of SetTireDampImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the tire stiffness of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the tire stiffness.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetTireStif( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.tireStif;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.tireStif;
	}

} // end of GetTireStif

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the immediate tire stiffness of the
//   vehicle object.
//
// Remarks:  This function gets the immediate tire stiffness
//   of the vehicle object, bypassing the double-buffering.
//
// 	 This function may return inconsistant values if it is called multiple
// 	 times within a frame.
//
// Arguments:
//
// Returns: A double containing the immediate tire stiffness.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetTireStifImm( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.tireStif;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.tireStif;
	}

} // end of GetTireStifImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the tire stiffness of the vehicle object.
//
// Remarks: This function sets the tire stiffness of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cTireStif - The tire stiffness.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTireStif( const double& cTireStif )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.tireStif = cTireStif;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.tireStif = cTireStif;
	}

} // end of SetTireStif

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the tire stiffness of the vehicle object in
//   current frame.
//
// Remarks: This function sets the tire stiffness of the current
// 	 vehicle object.  The values are written to the current frame,
//   bypassing the double-buffering.
//
// Arguments:
// 	 cTireStif - The tire stiffness.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetTireStifImm( const double& cTireStif )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.tireStif = cTireStif;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.tireStif = cTireStif;
	}

} // end of SetTireStifImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetTireRotation
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
float
CVehicleObj::GetTireRotation( int tire ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.tireRot[tire];
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.tireRot[tire];
	}
} // end of GetTireRotation

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the dynaInitComplete of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: An integer: 0 --> not complete, 1 --> complete
//
//////////////////////////////////////////////////////////////////////////////
int
CVehicleObj::GetDynaInitComplete( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.dynaInitComplete;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.dynaInitComplete;
	}

} // end of GetDynaInitComplete

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the dynaInitComplete of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: An integer: 0 --> not complete, 1 --> complete
//
//////////////////////////////////////////////////////////////////////////////
int
CVehicleObj::GetDynaInitCompleteImm( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.dynaInitComplete;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.dynaInitComplete;
	}

} // end of GetDynaInitCompleteImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the dynaInitComplete of the vehicle object.
//
// Remarks: This function sets the dynaInitComplete of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 dynaInitComplete - Has the dynamics been initalized?
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetDynaInitComplete( int dynaInitComplete )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.dynaInitComplete = dynaInitComplete;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.dynaInitComplete = dynaInitComplete;
	}

} // end of SetDynaInitComplete

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the dynaInitComplete of the vehicle object in the
//  current frame.
//
// Remarks: This function sets the dynaInitComplete of the current
// 	 vehicle object.  The values are written to the current frame,
//   bypassing the double-buffering.
//
// Arguments:
// 	 dynaInitComplete - Has the dynamics been initalized?
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetDynaInitCompleteImm( int dynaInitComplete )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.dynaInitComplete = dynaInitComplete;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.dynaInitComplete = dynaInitComplete;
	}

} // end of SetDynaInitCompleteImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the velocity brake of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the velocity brake.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetVelBrake( void ) const
{

	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.velBrake;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.velBrake;
	}

} // end of GetVelBrake

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the velocity brake of the vehicle object.
//
// Remarks: This function sets the vel brake of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cVelBrake - The velocity brake.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetVelBrake( const double& cVelBrake )
{

	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.velBrake = cVelBrake;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.velBrake = cVelBrake;
	}

} // end of SetVelBrake

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the state vector of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A cvTFourWheelVeh containing the tire information.
//
//////////////////////////////////////////////////////////////////////////////
cvTFourWheelVeh
CVehicleObj::GetStateVector( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );
	return g_stateVector[m_pObj->myId];
} // end of GetStateVector

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the state vector of the vehicle object.
//
// Remarks: This function sets the tire info of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
//   cStateVector - the tire info
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetStateVector( const cvTFourWheelVeh& cStateVector )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );
	g_stateVector[m_pObj->myId] = cStateVector;
} // end of SetStateVector

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the tire info of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A cvTFourWheelVeh containing the tire info.
//
//////////////////////////////////////////////////////////////////////////////
cvTVehLin
CVehicleObj::GetVehLin( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );
	return g_vehLin[m_pObj->myId];
} // end of GetVehLin

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the tire info of the vehicle object.
//
// Remarks: This function sets the tire info of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 vehLin - the tire info
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetVehLin( const cvTVehLin& cVehLin )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );
	g_vehLin[m_pObj->myId] = cVehLin;
} // end of SetVehLin


//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the vehicle dynamics fidelity of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: An enumeration containing the dynamics fidelity.
//
//////////////////////////////////////////////////////////////////////////////
EDynaFidelity
CVehicleObj::GetDynaFidelity( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.dynaFidelity;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.dynaFidelity;
	}
} // end of GetDynaFidelity

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the vehicle dynamics fidelity of the vehicle object.
//
// Remarks: This function sets the dynaFidelity of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 dynaFidelity - The vehicle dynamics fidelity.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetDynaFidelity( const EDynaFidelity dynaFidelity )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.dynaFidelity = dynaFidelity;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.dynaFidelity = dynaFidelity;
	}
} // end of SetDynaFidelity

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the vehicle dynamics fidelity of the vehicle
//   object in the current frame.
//
// Remarks: This function sets the dynaFidelity of the current
// 	 vehicle object.  The values are written to the current frame,
//   bypassing the double-buffering.
//
// Arguments:
// 	 dynaFidelity - The vehicle dynamics fidelity.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetDynaFidelityImm( const EDynaFidelity dynaFidelity )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.dynaFidelity = dynaFidelity;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.dynaFidelity = dynaFidelity;
	}
} // end of SetDynaFidelityImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the visual state of the current trajfollower.
//
// Remarks:
//
// Arguments:
//
// Returns: An unsigned 8-bit integer containing the visual state bits.
//
//////////////////////////////////////////////////////////////////////////////
TU16b
CVehicleObj::GetVisualState( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.visualState;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.visualState;
	}
} // end of GetVisualState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the visual state of the vehicle.
//
// Remarks:
//
// Arguments:
//
// Returns: An unsigned 8-bit integer containing the visual state bits.
//
//////////////////////////////////////////////////////////////////////////////
TU16b
CVehicleObj::GetVisualStateImm( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.visualState;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.visualState;
	}
} // end of GetVisualState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the visual state of the current vehicle.
//
// Remarks: This function sets the visual state of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 visualState - An unsigned 8-bit integer containing the
//                 desired visual state.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetVisualState( TU16b visualState )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.visualState = visualState;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.visualState = visualState;
	}
} // end of SetVisualState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the audio state of the current vehicle.
//
// Remarks:
//
// Arguments:
//
// Returns: An unsigned 16-bit integer containing the audio state bits.
//
//////////////////////////////////////////////////////////////////////////////
TU16b
CVehicleObj::GetAudioState( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.audioState;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.audioState;
	}
} // end of GetAudioState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the audio state of the current vehicle.
//
// Remarks: This function sets the audio state of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 visualState - An unsigned 8-bit integer containing the
//                 desired audio state
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetAudioState( TU16b audioState )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.audioState = audioState;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.audioState = audioState;
	}
} // end of SetAudioState

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the lateral acceleration of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the lateral acceleration.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetLatAccel( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.latAccel;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.latAccel;
	}
} // end of GetLatAccel
double  CVehicleObj::GetSteeringAngle(void) const{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
        return m_pObj->stateBufA.state.vehicleState.vehState.steeringWheelAngle;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.steeringWheelAngle;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the acceleration of the vehicle object.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the acceleration.
//
//////////////////////////////////////////////////////////////////////////////
double
CVehicleObj::GetAccel( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.acc;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.acc;
	}
} // end of GetAccel

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the number of times that QryTerrain has continuously
//   failed.
//
// Remarks:
//
// Arguments:
//
// Returns: A integer containing the number of times.
//
//////////////////////////////////////////////////////////////////////////////
int
CVehicleObj::GetQryTerrainErrCount( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.qryTerrainErrCount;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.qryTerrainErrCount;
	}
} // end of GetQryTerrainErrCount

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the velocity brake of the vehicle object.
//
// Remarks: This function sets the vel brake of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cVelBrake - The velocity brake.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetQryTerrainErrCount( const int cQryTerrainErrCount )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.vehicleState.vehState.qryTerrainErrCount =
														cQryTerrainErrCount;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.vehicleState.vehState.qryTerrainErrCount =
														cQryTerrainErrCount;
	}
} // end of SetQryTerrainErrCount


//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the velocity brake of the vehicle object in the current
//   state.
//
// Remarks: This function sets the vel brake of the current
// 	 vehicle object.  The specified data won't be visible to queries
// 	 until after the next execution of the maintainer.
//
// Arguments:
// 	 cVelBrake - The velocity brake.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CVehicleObj::SetQryTerrainErrCountImm( const int cQryTerrainErrCount )
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.vehicleState.vehState.qryTerrainErrCount =
														cQryTerrainErrCount;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.vehicleState.vehState.qryTerrainErrCount =
														cQryTerrainErrCount;
	}
} // end of SetQryTerrainErrCountImm

//////////////////////////////////////////////////////////////////////////////
//              EMERGENCY VEHICLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              BUS MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              UTILITY VEHICLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              TRUCK MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              TRAILER MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              RAIL VEHICLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              TERRAIN OBJECT MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              TRAFFIC LIGHT MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetState
// 	Get the state of the traffic light object.
//
// Remarks:
//
// Arguments:
//
// Returns: a eCVTrafficLightState containing the state
//
//////////////////////////////////////////////////////////////////////////////
eCVTrafficLightState
CTrafficLightObj::GetState( void ) const
{
	AssertValid();

	return m_pObj->stateBufA.state.trafficLightState.state;
} // end of GetState

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetSuspDamp
// 	Set the state of the traffic light object.
//
// Remarks: This function sets the state of the current traffic light.
// 	The specified data won't be visible to queries until after the next
// 	execution of the maintainer.
//
// Arguments:
// 	cState - the state
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CTrafficLightObj::SetState( const eCVTrafficLightState& cState )
{
	AssertValid();

	// Set both buffers with the understanding that traffic lights
	// state is not propagated by the maintainer.
	m_pObj->stateBufB.state.trafficLightState.state = cState;
	m_pObj->stateBufA.state.trafficLightState.state = cState;
} // end of SetState


//////////////////////////////////////////////////////////////////////////////
//              TRAFFIC SIGN MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              SIGN MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              OBSTACLE MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              ENVIRONMENT OBJECT MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              SPECIAL EFFECT OBJECT MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              RAILWAY OBJECT MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//              EXTERNAL DRIVER MEMBER FUNCTIONS
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the acceleration of a dynamic object.
//
// Remarks:  This function sets the acceleration of a dynamic object. This new
//   acceleration becomes visible during the next execution of CVED.
//
// Arguments:
//   accel - The new acceleration.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CExternalDriverObj::SetAccel( double accel, bool useDoubleBuffer )
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader *>( GetInst() );

	if( useDoubleBuffer )
	{
		if( ( pH->frame & 1 ) == 0 )
		{
			m_pObj->stateBufB.state.externalDriverState.acc = accel;
		}
		else
		{
			m_pObj->stateBufA.state.externalDriverState.acc = accel;
		}
	}
	else
	{
		m_pObj->stateBufA.state.externalDriverState.acc = accel;
		m_pObj->stateBufB.state.externalDriverState.acc = accel;
	}
} // end of SetAccel
double
CExternalDriverObj::GetAccel( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.externalDriverState.acc;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.externalDriverState.acc;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:  Sets the angular velocity of a dynamic object.
//
// Remarks:  This function sets the angular velocity of a dynamic object.
//   This new angular velocity becomes visible during the next execution of
//   CVED.
//
// Arguments:
//   angularVel - The new angularVelocity.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CExternalDriverObj::SetAngularVel(
			const CVector3D& angularVel,
			bool useDoubleBuffer
			)
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader *>( GetInst() );

	if( useDoubleBuffer )
	{
		if( ( pH->frame & 1 ) == 0 )
		{
			m_pObj->stateBufB.state.externalDriverState.angularVel.i = angularVel.m_i;
			m_pObj->stateBufB.state.externalDriverState.angularVel.j = angularVel.m_j;
			m_pObj->stateBufB.state.externalDriverState.angularVel.k = angularVel.m_k;
		}
		else
		{
			m_pObj->stateBufA.state.externalDriverState.angularVel.i = angularVel.m_i;
			m_pObj->stateBufA.state.externalDriverState.angularVel.j = angularVel.m_j;
			m_pObj->stateBufA.state.externalDriverState.angularVel.k = angularVel.m_k;
		}
	}
	else
	{
		m_pObj->stateBufB.state.externalDriverState.angularVel.i = angularVel.m_i;
		m_pObj->stateBufB.state.externalDriverState.angularVel.j = angularVel.m_j;
		m_pObj->stateBufB.state.externalDriverState.angularVel.k = angularVel.m_k;
		m_pObj->stateBufA.state.externalDriverState.angularVel.i = angularVel.m_i;
		m_pObj->stateBufA.state.externalDriverState.angularVel.j = angularVel.m_j;
		m_pObj->stateBufA.state.externalDriverState.angularVel.k = angularVel.m_k;
	}
} // end of SetAngularVel


//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the position of the driver bypassing double buffering.
//
// Remarks: This function returns the position of the driver object.  The
// 	difference between this function and the general CObj::GetPos() function
// 	is which of the two buffers is used to obtain the data.  This function
// 	bypasses the double buffering scheme which ensures that all queries
// 	return the same state, independnent of their relative execution with the
// 	dynamic servers.
//
// 	When the dynamic servers and the behaviors are running at different
// 	rates, the regular CObj::GetPos() returns the state of an object
// 	as defined in the previous frame.  The indended use of the
// 	GetPosHighFreq is to return the state of an object as defined
// 	in the current frame.
// 	Keep in mind that when using this function, unless externally
// 	synchronized with any client who is modifying the state
// 	of the object, i.e., the dynamic servers or any client
// 	calling the SetPos() functions, it is possible to
// 	get different data depending on the relative execution of
// 	the SetPos function with the GetPosHighFreq.
//
// Returns: The 3D position of the object.
//
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CExternalDriverObj::GetPosHighFreq(void) const
{
	AssertValid();

//	gout << "->Enter CExternalDriverObj::GetPos()" << endl;

	cvTHeader *pH = static_cast<cvTHeader *>(GetInst());
	if( (pH->frame & 1) == 0 )
	{
		// even frame
		CPoint3D pos( m_pObj->stateBufB.state.anyState.position );
		return pos;
	}
	else
	{
		// odd frame
		CPoint3D pos( m_pObj->stateBufA.state.anyState.position );
		return pos;
	}
} // end of GetPosHighFreq

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the tangent of the driver bypassing double buffering.
//
// Remarks: This function returns the tangent vector of the driver object.
// 	For details on the difference between this function and CDynObj::GetTan(),
// 	see the GetPosHighFreq() function.
//
// Returns: The 3D tangent direction of the object.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CExternalDriverObj::GetTanHighFreq(void) const
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader *>(GetInst());
	if( (pH->frame & 1) == 0 )
	{
		// even frame
		CVector3D vec( m_pObj->stateBufB.state.anyState.tangent );
		return vec;
	}
	else
	{
		// odd frame
		CVector3D vec( m_pObj->stateBufA.state.anyState.tangent );
		return vec;
	}
} // end of GetTanHighFreq

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the lateral vector of the driver bypassing double
//  buffering.
//
// Remarks: This function returns the lateral vector of the driver object.
// 	For details on the difference between this function and CDynObj::GetTan(),
// 	see the GetPosHighFreq() function.
//
// Returns: The 3D tangent direction of the object.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CExternalDriverObj::GetLatHighFreq(void) const
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader *>(GetInst());
	if( (pH->frame & 1) == 0 )
	{
		// even frame
		CVector3D vec( m_pObj->stateBufB.state.anyState.lateral );
		return vec;
	}
	else
	{
		// odd frame
		CVector3D vec( m_pObj->stateBufA.state.anyState.lateral );
		return vec;
	}
} // end of GetLatHighFreq

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the angular velocity vector of the driver bypassing double
//  buffering.
//
// Remarks: This function returns the angular velocity vector of the driver
//  object.
//
// Returns: The angular velocity vector of the object.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CExternalDriverObj::GetAngularVelHighFreq( void ) const
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader *>(GetInst());
	if( (pH->frame & 1) == 0 )
	{
		// even frame
		CVector3D vec( m_pObj->stateBufB.state.externalDriverState.angularVel );
		return vec;
	}
	else
	{
		// odd frame
		CVector3D vec( m_pObj->stateBufA.state.externalDriverState.angularVel );
		return vec;
	}
} // end of GetLatHighFreq

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the bounding box of the driver bypassing double
//  buffering.
//
// Remarks: This function returns the bounding box of the driver object.
// 	For details on the difference between this function and CDynObj::GetTan(),
// 	see the GetPosHighFreq() function.
//
// Returns: The bounding box of the object.
//
//////////////////////////////////////////////////////////////////////////////
CBoundingBox
CExternalDriverObj::GetBoundBoxHighFreq(void) const
{
	AssertValid();

	cvTHeader *pH = static_cast<cvTHeader *>(GetInst());
	if( (pH->frame & 1) == 0 )
	{
		// even frame
		CBoundingBox bb(
					m_pObj->stateBufB.state.anyState.boundBox[0].x,
					m_pObj->stateBufB.state.anyState.boundBox[0].y,
					m_pObj->stateBufB.state.anyState.boundBox[1].x,
					m_pObj->stateBufB.state.anyState.boundBox[1].y
					);
		return bb;
	}
	else
	{
		// odd frame
		CBoundingBox bb(
					m_pObj->stateBufA.state.anyState.boundBox[0].x,
					m_pObj->stateBufA.state.anyState.boundBox[0].y,
					m_pObj->stateBufA.state.anyState.boundBox[1].x,
					m_pObj->stateBufA.state.anyState.boundBox[1].y
					);
		return bb;
	}
} // end of GetBoundBoxHighFreq

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the acceleration of the external driver object bypassing
//  double buffering.
//
// Remarks:
//
// Arguments:
//
// Returns: A double containing the acceleration.
//
//////////////////////////////////////////////////////////////////////////////
double
CExternalDriverObj::GetAccelHighFreq( void ) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.externalDriverState.acc;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.externalDriverState.acc;
	}
} // end of GetAccelHighFreq

void CVisualObjectObj::SetColor(float R,float G, float B, float A, bool doublebuffer){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer && ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.color[0] = R;
		m_pObj->stateBufB.state.virtualObjectState.color[1] = G;
		m_pObj->stateBufB.state.virtualObjectState.color[2] = B;
		m_pObj->stateBufB.state.virtualObjectState.color[3] = A;
	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.color[0] = R;
		m_pObj->stateBufA.state.virtualObjectState.color[1] = G;
		m_pObj->stateBufA.state.virtualObjectState.color[2] = B;
		m_pObj->stateBufA.state.virtualObjectState.color[3] = A;
	}
}
void CVisualObjectObj::SetRotation(float Rotation, bool doublebuffer){
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer || ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.rotation = Rotation;
	}
	if( !doublebuffer || ( pH->frame & 1 ) > 0 )
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.rotation = Rotation;
	}
}
void CVisualObjectObj::SetTargetId(int id, bool doublebuffer){
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer  ||   ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.parentId = id;

	}
	if(!doublebuffer  ||    ( pH->frame & 1 ) > 0 )
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.parentId = id;
	}
}
int CVisualObjectObj::GetTargetId() const{

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.virtualObjectState.parentId;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.virtualObjectState.parentId;
	}
}
TU16b CVisualObjectObj::GetStateIndex() const{
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if(  ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.virtualObjectState.StateIndex;
	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.virtualObjectState.StateIndex;
	}
}
void CVisualObjectObj::SetStateIndex(TU16b id, bool doublebuffer){
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer  ||   ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.StateIndex = id;
	}
	if(!doublebuffer  ||    ( pH->frame & 1 ) > 0 )
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.StateIndex = id;
	}
}
void CVisualObjectObj::SetBoarderColor(float R,float G, float B, float A, bool doublebuffer){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer  ||   ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.color[0] = R;
		m_pObj->stateBufB.state.virtualObjectState.color[1] = G;
		m_pObj->stateBufB.state.virtualObjectState.color[2] = B;
		m_pObj->stateBufB.state.virtualObjectState.color[3] = A;
	}
	if(!doublebuffer  ||    ( pH->frame & 1 ) > 0 )
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.colorBorder[0] = R;
		m_pObj->stateBufA.state.virtualObjectState.colorBorder[1] = G;
		m_pObj->stateBufA.state.virtualObjectState.colorBorder[2] = B;
		m_pObj->stateBufA.state.virtualObjectState.colorBorder[3] = A;
	}
}
CPoint3D CVisualObjectObj::GetDrawPosition() const{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );
	CPoint3D ret;
	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		ret.m_x = m_pObj->stateBufA.state.virtualObjectState.overlayPosition.x;
		ret.m_y = m_pObj->stateBufA.state.virtualObjectState.overlayPosition.y;
		ret.m_z = m_pObj->stateBufA.state.virtualObjectState.overlayPosition.z;
	}
	else
	{
		// odd frame
		ret.m_x = m_pObj->stateBufB.state.virtualObjectState.overlayPosition.x ;
		ret.m_y = m_pObj->stateBufB.state.virtualObjectState.overlayPosition.y ;
		ret.m_z = m_pObj->stateBufB.state.virtualObjectState.overlayPosition.z ;
	}
	return ret;
}

void CVisualObjectObj::GetColor(float &R,float &G, float &B, float &A) const{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		R = m_pObj->stateBufA.state.virtualObjectState.color[0];
		G = m_pObj->stateBufA.state.virtualObjectState.color[1];
		B = m_pObj->stateBufA.state.virtualObjectState.color[2];
		A = m_pObj->stateBufA.state.virtualObjectState.color[3];
	}
	else
	{
		// odd frame
		R = m_pObj->stateBufB.state.virtualObjectState.color[0];
		G = m_pObj->stateBufB.state.virtualObjectState.color[1];
		B = m_pObj->stateBufB.state.virtualObjectState.color[2];
		A = m_pObj->stateBufB.state.virtualObjectState.color[3];
	}
}
float CVisualObjectObj::GetRotation() const{
	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return (float)m_pObj->stateBufA.state.virtualObjectState.rotation;
	}
	else
	{
		// odd frame
		return (float)m_pObj->stateBufB.state.virtualObjectState.rotation;
	}
}
void CVisualObjectObj::GetBoarderColor(float &R,float &G, float &B, float &A) const{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufA.state.virtualObjectState.color[0] = R;
		m_pObj->stateBufA.state.virtualObjectState.color[1] = G;
		m_pObj->stateBufA.state.virtualObjectState.color[2] = B;
		m_pObj->stateBufA.state.virtualObjectState.color[3] = A;
	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.virtualObjectState.color[0] = R;
		m_pObj->stateBufB.state.virtualObjectState.color[1] = G;
		m_pObj->stateBufB.state.virtualObjectState.color[2] = B;
		m_pObj->stateBufB.state.virtualObjectState.color[3] = A;
	}
}
void CVisualObjectObj::SetDrawPosition(const CPoint3D &pos, bool doublebuffer){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer || ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.overlayPosition.x = pos.m_x;
		m_pObj->stateBufB.state.virtualObjectState.overlayPosition.y = pos.m_y;
		m_pObj->stateBufB.state.virtualObjectState.overlayPosition.z = pos.m_z;
	}
	if(!doublebuffer ||  ( pH->frame & 1 ) > 0 )
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.overlayPosition.x = pos.m_x;
		m_pObj->stateBufA.state.virtualObjectState.overlayPosition.y = pos.m_y;
		m_pObj->stateBufA.state.virtualObjectState.overlayPosition.z = pos.m_z;
	}
}
void CVisualObjectObj::SetDrawType(TU8b objType, bool doublebuffer){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer && ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.objType = objType;

	}
	else
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.objType = objType;
	}
}
TU8b CVisualObjectObj::GetDrawType() const{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.virtualObjectState.objType;

	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.virtualObjectState.objType;
	}
}

void CVisualObjectObj::SetDrawScreen(TU8b objType, bool doublebuffer){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer && ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.DrawScreen = objType;

	}
	else
	{
		// odd frame
		m_pObj->stateBufB.state.virtualObjectState.DrawScreen = objType;
	}
}

TU8b CVisualObjectObj::GetDrawScreen() const{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		return m_pObj->stateBufA.state.virtualObjectState.DrawScreen;

	}
	else
	{
		// odd frame
		return m_pObj->stateBufB.state.virtualObjectState.DrawScreen;
	}
}

void CVisualObjectObj::SetDrawSize(float x, float y, bool doublebuffer){
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( !doublebuffer  ||   ( pH->frame & 1 ) == 0 )
	{
		// even frame
		m_pObj->stateBufB.state.virtualObjectState.scale[0] = x;
		m_pObj->stateBufB.state.virtualObjectState.scale[1] = y;
	}
	if(!doublebuffer  ||    ( pH->frame & 1 ) > 0 )
	{
		// odd frame
		m_pObj->stateBufA.state.virtualObjectState.scale[0] = x;
		m_pObj->stateBufA.state.virtualObjectState.scale[1] = y;
	}
}
void CVisualObjectObj::GetDrawSize(float &x, float &y) const {
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>( GetInst() );

	if( ( pH->frame & 1 ) == 0 )
	{
		// even frame
		x = m_pObj->stateBufA.state.virtualObjectState.scale[0];
		y = m_pObj->stateBufA.state.virtualObjectState.scale[1];
	}
	else
	{
		// odd frame
		x = m_pObj->stateBufB.state.virtualObjectState.scale[0];
		y = m_pObj->stateBufB.state.virtualObjectState.scale[1];
	}
}
} // end namespace CVED