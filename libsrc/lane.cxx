//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: lane.cxx,v 1.49 2018/12/07 21:05:19 IOWA\dheitbri Exp $
//
// Author(s):   Yiannis Papelis
// Date:		September, 1998
//
// Description:	Implementation of the lane class.
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h" 

namespace CVED {

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator<<
// 	Prints the contents of cLane to the ostream out.
//
// Remarks:
// 
// Arguments:
// 	out - ostream to print the cLane to
// 	cLane - lane to print
// 
// Returns: reference to the modified ostream
//
//////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream &out, const CLane &cLane)
{
	out << "(" << cLane.GetName() << ")";
	return out;
} // end of operator<<

} // namespace CVED

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

bool
CLane::operator==(const CLane& cRhs)
{
	AssertValid();
	
	return ( 
		m_pLane->roadId == cRhs.m_pLane->roadId &&
		m_pLane->laneNo == cRhs.m_pLane->laneNo &&
		m_pLane->direction == cRhs.m_pLane->direction
		);
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLane
// 	This constructor takes a CCved instance, a CRoad instance, and a laneId.
//
// Remarks:
// 
// Arguments:
// 	cCved - a CCved reference
// 	cRoad - the road containing the lane
// 	laneId - the numeric lane identifier on the specified road
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLane::CLane(const CCved& cCved, const CRoad& cRoad, int laneId)
	:
	CCvedItem(&cCved),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*>       (GetInst()) + pH->laneOfs;

	m_pLane = (reinterpret_cast<cvTLane*>(pOfs)) + cRoad.GetLaneIdx() + laneId; 
} // end of CLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLane
// 	This constructor takes a CRoad instance and a laneId.
//
// Remarks: The CCvedItem superclass is intialized with the CRoad
// 
// Arguments:
// 	cRoad - the road containing the lane
// 	laneId - the numeric lane identifier on the specified road
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLane::CLane(const CRoad& cRoad, int laneId)
	: 
	CCvedItem(cRoad), 
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*>       (GetInst()) + pH->laneOfs;
	m_pLane = (reinterpret_cast<cvTLane*>(pOfs)) + cRoad.GetLaneIdx() + laneId; 
} // end of CLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLane
// 	This constructor takes a CCved instance, a road id, and a laneId.
//
// Remarks: The CCvedItem superclass is intialized with the CRoad
// 
// Arguments:
//  cCved - CCved instance
// 	rd - identifier of the road containing the lane
// 	laneId - the numeric lane identifier on the specified road
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLane::CLane(const CCved& cCved, int rd, int laneId)
	: 
	CCvedItem(&cCved),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	cvTHeader*  pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*>       (GetInst()) + pH->roadOfs;
	cvTRoad*	pRd	 = (reinterpret_cast<TRoad*>(pOfs)) + rd;

	pOfs = static_cast<char*>       (GetInst()) + pH->laneOfs;
	m_pLane = (reinterpret_cast<cvTLane*>(pOfs)) + pRd->laneIdx + laneId;
} // end of CLane		

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLane
// 	This constructor takes a CCved instance and a cvTLane structure
//
// Remarks:
// 
// Arguments:
//  cCved - CCved instance
//	pLane - a pointer to a cvTLane structure
//	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLane::CLane(const CCved& cCved, TLane* pLane)
	: 
	CCvedItem(&cCved), m_pLane(pLane),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{} // end of CLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLane
// 	This copy constructor assigns the contents of the parameter to the current
// 	instance.
//
// Remarks: 
// 
// Arguments:
// 	cSrc - a CLane instance
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLane::CLane(const CLane& cSrc)
{
	*this = cSrc;
} // end of CLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLeft
// 	Return lane to the left of current
//
// Remarks: If there is no lane to the left, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//
// Returns: The lane to the left of the current.
//
//////////////////////////////////////////////////////////////////////////////
CLane
CLane::GetLeft(void) const
{
	AssertValid();

	int			numOfLanes = GetRoad().GetNumLanes();
	cvELnDir	dir = GetDirection();
	bool		haveLeftLane;

	// when direction is positive
	haveLeftLane = ((dir==ePOS) && (m_pLane->laneNo < (numOfLanes-1)));
	if( haveLeftLane )
	{
		CLane leftLane(GetCved(), m_pLane+1);
		return leftLane;
	}
	// when direction is negative
	haveLeftLane = ((dir==eNEG) && (m_pLane->laneNo > 0));
	if( haveLeftLane )
	{
		CLane leftLane(GetCved(), m_pLane-1);
		return leftLane;
	}

	// throw an exception here
	string msg("No left lane exist");
	cvCInternalError e(msg, __FILE__, __LINE__);
	CLane dummy;
	throw e;
	return dummy;
} // end of GetLeft

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRight
// 	Return lane to the right of current
//
// Remarks: If there is no lane to the right, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//
// Returns: The lane to the right of the current.
//
//////////////////////////////////////////////////////////////////////////////
CLane
CLane::GetRight(void) const
{
	AssertValid();
   
	int			numOfLanes = GetRoad().GetNumLanes();
	cvELnDir	dir = GetDirection();
	bool		ifRightLane;

	// when direction is positive
	ifRightLane = ((dir==ePOS) && (m_pLane->laneNo > 0));
	if (ifRightLane){
		CLane rightLane(GetCved(), m_pLane-1);
		return rightLane;
	}
	// when direction is negative
	ifRightLane = ((dir==eNEG) && (m_pLane->laneNo < (numOfLanes-1)));
	if (ifRightLane){
		CLane rightLane(GetCved(), m_pLane+1);
		return rightLane;
	}
	
	// throw an exception here
	string			msg("No right lane exist");
	cvCInternalError	e(msg, __FILE__, __LINE__);
	CLane dummy;
	throw e;
	return dummy;
} // end of GetRight
 
/////////////////////////////////////////////////////////////////////////////
//
// Description: Determine if this is the left-most on the road.
//
// Remarks: If the current lane is not bound, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//
// Returns: If the lane is the left-most, the function returns true, 
// 	 otherwise it returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsLeftMost( void ) const
{

	AssertValid();

	cvELnDir dir = GetDirection();

	if ( dir == ePOS ) {

		int numOfLanes = GetRoad().GetNumLanes();
		return ( m_pLane->laneNo == ( numOfLanes - 1 ) );

	}
	else {

		return ( m_pLane->laneNo == 0 ); 
		
	}  
} // end of IsLeftMost

/////////////////////////////////////////////////////////////////////////////
//
// Description: Determine if this is the left-most lane traveling in
//   this direction on the road.
//
// Remarks: If the current lane is not bound, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//
// Returns: If the lane is the left-most, the function returns true, 
// 	 otherwise it returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsLeftMostAlongDir( void ) const
{

	AssertValid();

	//
	// I can use the IsLeftOpposite function to figure out if I'm the
	// left-most lane moving in my direction.  However, I have to first
	// check to see if there even exists to my left.
	//
	cvELnDir dir = GetDirection();
	bool moreLanesToLeft;

	if ( dir == ePOS ) {

		// add 1 for positive direction lanes
		int numOfLanes = GetRoad().GetNumLanes();
		moreLanesToLeft = !( m_pLane->laneNo == numOfLanes - 1 );
	}
	else {

		// subtract 1 for negative direction lanes
		moreLanesToLeft = ( m_pLane->laneNo != 0 );

	}
	
	if ( moreLanesToLeft ) {

		//
		// If the lane to my left is moving in the opposite direction
		// then I'm the left-most lane.
		//
		return IsLeftOpposite();

	}
	else {

		//
		// This is already the left-most lane.
		//
		return true;

	}

} // end of IsLeftMostAlongDir


  /////////////////////////////////////////////////////////////////////////////
  //
  // Description: Determine if this is the left-most lane traveling in
  //   this direction on the road.
  //
  // Remarks: If the current lane is not bound, the function throws an 
  // 	exception of type TBD.
  //
  // Arguments:
  //
  // Returns: If the lane is the left-most, the function returns true, 
  // 	 otherwise it returns false.
  //
  //////////////////////////////////////////////////////////////////////////////
bool
CLane::IsRightMostAlongDir( void ) const
{

    AssertValid();

    //
    // I can use the IsLeftOpposite function to figure out if I'm the
    // right-most lane moving in my direction.  However, I have to right
    // check to see if there even exists to my left.
    //
    cvELnDir dir = GetDirection();
    bool moreLanesToRight = false;
    if ( dir == ePOS ) {

        moreLanesToRight = ( m_pLane->laneNo != 0 );             

    }
    else {

        int numOfLanes = GetRoad().GetNumLanes();
        moreLanesToRight = !( m_pLane->laneNo == ( numOfLanes - 1 ) );

    }

    if ( moreLanesToRight ) {

        //
        // If the lane to my right is moving in the opposite direction
        // then I'm the right-most lane.
        //
        
        return IsRightOpposite();

    }
    else {

        //
        // This is already the left-most lane.
        //
        return true;

    }

} // end of IsLeftMostAlongDir

/////////////////////////////////////////////////////////////////////////////
//
// Description: Determine if this is the right most lane on the road.
//
// Remarks: If the current lane is not bound, the function throws an 
// 	 exception of type TBD.
//
// Arguments:
//
// Returns: If the lane is the right most, the function returns true, 
// 	 otherwise it returns false.
/////////////////////////////////////////////////////////////////////////////
bool
CLane::IsRightMost( void ) const
{
	AssertValid();
	
	cvELnDir dir = GetDirection();

	if ( dir == ePOS ) {

		return ( m_pLane->laneNo == 0 );             

	}
	else {

		int numOfLanes = GetRoad().GetNumLanes();
		return ( m_pLane->laneNo == ( numOfLanes - 1 ) );

	}
} // end of IsRightMost

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsLeftOpposite
// 	Determine if this lane to the left is opposite direction
//
// Remarks: If the current lane is not bound, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//
// Returns: If the lane to the left is opposite direction, the function 
// 	returns true else it returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsLeftOpposite(void) const
{
	AssertValid();

	if(	GetLeft().GetDirection()!=GetDirection() )
		return true;
	else
		return false;
} // end of IsLeftOpposite
///////////////////////////////////////////////////////////////////////////////
///
/// Description: IsLeftOpposite
/// 	Determine if this lane to the right is opposite direction
///
/// Remarks: If the current lane is not bound, the function throws an 
/// 	exception of type TBD.
///
/// Arguments:
///
/// Returns: If the lane to the right is opposite direction, the function 
/// 	returns true else it returns false.
///
///////////////////////////////////////////////////////////////////////////////
bool
CLane::IsRightOpposite(void) const
{
    AssertValid();
    if (GetRoad().GetNumLanes() == 1) {
        return false;
    }
    if(	GetRight().GetDirection()!=GetDirection() )
        return true;
    else
        return false;
} // end of IsLeftOpposite

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetOppositeLane
// 	Get a lane in the opposite direction immediately to the left of the
//  current lane.
//
// Remarks: If the current lane is not bound, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//
// Returns: If the lane to the left is opposite direction, the function 
// 	returns true else it returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::GetOppositeLane( CLane& oppositeLane ) const
{
	AssertValid();

	if( !IsLeftMost() )
	{
		if( IsLeftOpposite() )
		{
			oppositeLane = GetLeft();
			return true;
		}
	}

	return false;
} // end of IsLeftOpposite

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetOppositeLane
// 	Get all lanes in the opposite direction
//
// Remarks: If the current lane is not bound, the function throws an 
// 	exception of type TBD.
//
// Arguments:
//  (outputs) oppositeLanes - All lanes in the opposite direction
//  (input) includeVehicleRestrictedLanes - Include vehicle restricted lanes.
//  (input) includeTurnLanes - Include turn lanes.
//
// Returns: none
//
//////////////////////////////////////////////////////////////////////////////
void
CLane::GetOppositeLane( vector<CLane>& oppositeLanes, bool includeVehicleRestrictedLanes, bool includeTurnLanes ) const
{
	AssertValid();

	cvELnDir currDirection = m_pLane->direction;
	int numOfLanes = GetRoad().GetNumLanes();

	int i;
	for( i = 0; i < numOfLanes; i++ )
	{
		CLane laneToCheck( GetCved(), m_pLane - m_pLane->laneNo + i );
		if( laneToCheck.GetDirection() != currDirection )
		{
			if( !includeVehicleRestrictedLanes && laneToCheck.IsVehicleRestrictedLane() )
			{
				continue;
			}

			if( !includeTurnLanes && laneToCheck.IsTurnLane() )
			{
				continue;
			}

			oppositeLanes.push_back( laneToCheck );
		}
	}
} // end of GetOppositeLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsVehicleRestrictedLane
// 	Is this a lane that's restricted to vehicles?
//
// Remarks: Checks the attributes to see if this has been tagged as a 
//  vehicle restricted lane.  A bike lane would be an example of a vehicle
//  restricted lane.  This function doesn't check distance along the road
//  and assumes that the entire lane has been marked as restricted lane.
//
// Arguments:
//
// Returns: A boolean indicating if it's a vehicle restricted lane.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsVehicleRestrictedLane( void ) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pLane->attrIdx;

	// First check the road attributes
	CAttr attr;
	int laneMask = 1 << (m_pLane->laneNo % cCV_MAX_LANES);
	bool foundAttrOnRoad = GetRoad().QryAttr( 
				cCV_VEHICLE_RESTRICTION_LANE_ATTR, attr, laneMask 
				);
	if( foundAttrOnRoad )
	{
		return true;
	}
	else
	{
		// For each attribute on the lane
		int i;
		for( i = 0; i < m_pLane->numAttr; i++, pAttr++ )
		{
			// If the attr ids match ...
			bool match = pAttr->myId == cCV_VEHICLE_RESTRICTION_LANE_ATTR;
			if( match )  return true;
		}
	}

	// No matching attribute found
	return false;
} // end of IsVehicleRestrictedLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsTurnLane
// 	Is this a lane that's only for turning vehicles?
//
// Remarks: Checks the attributes to see if this has been tagged as a 
//  turn lane.  This function doesn't check distance along the road
//  and assumes that the entire lane has been marked as restricted lane.
//
// Arguments:
//
// Returns: A boolean indicating if it's a turn lane.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsTurnLane( void ) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pLane->attrIdx;

	// First check the road attributes
	CAttr attr;
	int laneMask = 1 << (m_pLane->laneNo % cCV_MAX_LANES);
	bool foundAttrOnRoad = GetRoad().QryAttr( 
				cCV_TURN_LANE_ATTR, attr, laneMask 
				);
	if( foundAttrOnRoad )
	{
		return true;
	}
	else
	{
		// For each attribute on the lane
		int i;
		for( i = 0; i < m_pLane->numAttr; i++, pAttr++ )
		{
			// If the attr ids match ...
			bool match = pAttr->myId == cCV_TURN_LANE_ATTR;
			if( match )  return true;
		}
	}

	// No matching attribute found
	return false;
} // end of IsTurnLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsOnRamp
// 	Is this a lane that leads onto an interstate?
//
// Remarks: Checks the attributes to see if this has been tagged as an 
//  interstate on ramp.  This function doesn't check distance along the road
//  and assumes that the entire lane has been marked as a ramp.
//
// Arguments:
//
// Returns: A boolean indicating if it's an on ramp.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsOnRamp( void ) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pLane->attrIdx;

	// First check the road attributes
	CAttr attr;
	int laneMask = 1 << (m_pLane->laneNo % cCV_MAX_LANES);
	bool foundAttrOnRoad = GetRoad().QryAttr( 
				cCV_INTERSTATE_ON_RAMP, attr, laneMask 
				);
	if( foundAttrOnRoad )
	{
		return true;
	}
	else
	{
		// For each attribute on the lane
		int i;
		for( i = 0; i < m_pLane->numAttr; i++, pAttr++ )
		{
			// If the attr ids match ...
			bool match = pAttr->myId == cCV_INTERSTATE_ON_RAMP;
			if( match )  return true;
		}
	}

	// No matching attribute found
	return false;
} // end of IsTurnLane
///////////////////////////////////////////////////////////////////////////////
///
/// Description: IsOnRamp
/// 	Is this a lane that leads onto an interstate?
///
/// Remarks: Checks the attributes to see if this has been tagged as an 
///  interstate on ramp.  This function doesn't check distance along the road
///  and assumes that the entire lane has been marked as a ramp.
///
/// Arguments:
///
/// Returns: A boolean indicating if it's an on ramp.
///
///////////////////////////////////////////////////////////////////////////////
bool
CLane::IsOffRamp( void ) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pLane->attrIdx;

	// First check the road attributes
	CAttr attr;
	int laneMask = 1 << (m_pLane->laneNo % cCV_MAX_LANES);
	bool foundAttrOnRoad = GetRoad().QryAttr( 
				cCV_INTERSTATE_ON_RAMP, attr, laneMask 
				);
	if( foundAttrOnRoad )
	{
		return true;
	}
	else
	{
		// For each attribute on the lane
		int i;
		for( i = 0; i < m_pLane->numAttr; i++, pAttr++ )
		{
			// If the attr ids match ...
			bool match = pAttr->myId == cCV_INTERSTATE_OFF_RAMP;
			if( match )  return true;
		}
	}

	// No matching attribute found
	return false;
} // end of IsTurnLane

 ///////////////////////////////////////////////////////////////////////////////
 ///
 /// Description: IsInterstate
 /// 	Is this a lane on a interstate
 ///
 /// Remarks: Checks the attributes to see if this has been tagged as an 
 ///  interstate  This function doesn't check distance along the road
 ///  and assumes that the entire lane as interstate
 ///
 /// Arguments:
 ///
 /// Returns: A boolean indicating if it's an on interstate.
 ///
 ///////////////////////////////////////////////////////////////////////////////
bool CLane::IsInterstate(void) const {
    AssertValid();

    cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
    char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
    cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pLane->attrIdx;

    // First check the road attributes
    CAttr attr;
    int laneMask = 1 << (m_pLane->laneNo % cCV_MAX_LANES);
    bool foundAttrOnRoad = GetRoad().QryAttr( 
        cCV_INTERSTATE_ATTR, attr, laneMask 
    );
    if( foundAttrOnRoad )
    {
        return true;
    }
    else
    {
        // For each attribute on the lane
        int i;
        for( i = 0; i < m_pLane->numAttr; i++, pAttr++ )
        {
            // If the attr ids match ...
            bool match = pAttr->myId == cCV_INTERSTATE_OFF_RAMP;
            if( match )  return true;
        }
    }

    // No matching attribute found
    return false;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: IsDrivingLane
// 	Is this a lane that a vehicle can drive on?
//
// Remarks: Checks the attributes to see if this has been tagged as a 
//  vehicle restricted lane or a turn lane.  This function doesn't check 
//	distance along the road and assumes that the entire lane has been 
//	marked as a restricted lane or a turn lane.
//
// Arguments:
//
// Returns: A boolean indicating if it's a drivable lane.
//
//////////////////////////////////////////////////////////////////////////////
bool
CLane::IsDrivingLane(void) const {
	
	AssertValid();

	if (IsVehicleRestrictedLane()) {
		return false;
	}

	if (IsTurnLane()) {
		return false;
	}

	return true;
} // end of IsDrivingLane

/////////////////////////////////////////////////////////////////////////////
//
// Description: GetDirection
// 	Returns the direction of the current lane.
//
// Remarks:
//
// Arguments:
//
// Returns: a cvELnDir containing the direction of the current lane
//
//////////////////////////////////////////////////////////////////////////////
cvELnDir 
CLane::GetDirection(void) const
{
	AssertValid();
	return m_pLane->direction;
} // end of GetDirection

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetOffset
// 	Determine the offset of a lane
//
// Remarks: The offset of a lane is the perpendicular distance between the 
// 	road centerline and the center of the lane.  Lanes whose centerline is 
// 	to the right of the road have a positive offset while lanes whose
// 	centerline is to the left of the road centerline have a negative offset.  
// 	Left and right are defined when looking along the road's centerline from 
// 	the source to the destination intersection.
//
// 	The function returns the offset of a lane at a particular point
// 	along the road.  It is possible for the offset of a lane to change
// 	along a road so do not keep this value and use it for a whole road.
//
// Arguments:
// 	dist - (optional) distance along the road at which point the offset is 
// 		returned.  Default value is 0.0.
//
// Returns: The offset of the lane at the particular point on the road
//
//////////////////////////////////////////////////////////////////////////////
double 
CLane::GetOffset(double dist) const
{
	AssertValid();

	cvTHeader*	pH   = static_cast<cvTHeader*>	(GetInst());
	char*		pOfs = static_cast<char*>		(GetInst()) + pH->laneOfs;
	TLane*		pLn  = reinterpret_cast<TLane*>	(pOfs);

	pOfs	= static_cast<char*>	(GetInst()) + pH->longitCntrlOfs;
	cvTCntrlPnt*	pCp =	reinterpret_cast<cvTCntrlPnt*> (pOfs) + 
							GetRoad().GetCntrlPntIdx(); 
		
	int whichPt = Search(dist);
	pLn+=((pCp+whichPt)->laneIdx + m_pLane->laneNo);
	return pLn->offset;
} // GetOffset

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetWidth
// 	Determine the width of a lane
//
// Remarks: It is possible for the width of a lane to change along a road so 
// 	do not keep this value and use it for a whole road.
//
// Arguments:
// 	dist - (optional) distance along the road at which point the offset is 
// 		returned.  Default value is 0.0.
//
// Returns: The width of the lane at the particular point on the road
//
//////////////////////////////////////////////////////////////////////////////
double 
CLane::GetWidth(double dist) const
{
	AssertValid();

	cvTHeader*	pH   = static_cast<cvTHeader*>	(GetInst());
    char*		pOfs = static_cast<char*>		(GetInst()) + pH->laneOfs;
    TLane*		pLn  = reinterpret_cast<TLane*>	(pOfs);

	pOfs	= static_cast<char*>   (GetInst()) + pH->longitCntrlOfs;
	cvTCntrlPnt*	pCp =	reinterpret_cast<cvTCntrlPnt*> (pOfs) +
							GetRoad().GetCntrlPntIdx();
     
	int whichPt = Search(dist);
	pLn+=((pCp+whichPt)->laneIdx + m_pLane->laneNo);
	return pLn->width;
} // end of GetWidth

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetName
// 	Return the road's name along with the lane id.
//
// Remarks: This function returns a string holding the road's name along
//   with the lane's id.
//
// Arguments:
//
// Returns: A STL string containing the road's name and the lane's id.
//
//////////////////////////////////////////////////////////////////////////////
string 
CLane::GetName(void) const
{
	AssertValid();
	string s;
	THeader *pH 	= static_cast<THeader*> (GetInst());
	char* pCharPool	= static_cast<char*> (GetInst()) + pH->charOfs;

	// assemble the road name and the id
	string rd(GetRoad().GetName());
	s = rd + ':';

	char ln = m_pLane->laneNo + '0';
	s += ln;

	return s;
} // end of getName

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoad
// 	Get the road containing this lane
//
// Remarks:
//
// Arguments:
//
// Returns: A CRoad instance representing the road containing the lane.
//
//////////////////////////////////////////////////////////////////////////////
CRoad 
CLane::GetRoad(void) const
{
	AssertValid();
	CRoad road(GetCved(), m_pLane->roadId);
	return road;
} // end of GetRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextIntrsctn
// 	Gets the next intersection.
//
// Remarks: This function gets the next intersection following the
//   lane's direction.
//
// Arguments:
//
// Returns: An CIntrsctn instance representing the next intersection.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CLane::GetNextIntrsctn(void) const
{
	AssertValid();
	CRoad road(GetCved(), m_pLane->roadId);

	CIntrsctn nextIntrsctn;

	if (m_pLane->direction == ePOS) {
		nextIntrsctn = road.GetDestIntrsctn();
	}
	else if (m_pLane->direction == eNEG) {
		nextIntrsctn = road.GetSourceIntrsctn();
	}
	else {
		string	msg("lane direction has invalid value");
		cvCInternalError	e(msg, __FILE__, __LINE__);
		throw e;
	}

	return nextIntrsctn;
} // end of GetNextIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPrevIntrsctn
// 	Gets the previous intersection.
//
// Remarks: This function gets the previous intersection following the
//   lane's direction.
//
// Arguments:
//
// Returns: An CIntrsctn instance representing the previous intersection.
//
/////////////////////////////////////////////////////////////////////////////
CIntrsctn
CLane::GetPrevIntrsctn(void) const
{
	AssertValid();
	CRoad road(GetCved(), m_pLane->roadId);

	CIntrsctn prevIntrsctn;

	if (m_pLane->direction == ePOS) {
		prevIntrsctn = road.GetSourceIntrsctn();
	}
	else if (m_pLane->direction == eNEG) {
		prevIntrsctn = road.GetDestIntrsctn();
	}
	else {
		string	msg("lane direction has invalid value");
		cvCInternalError	e(msg, __FILE__, __LINE__);
		throw e;
	}

	return prevIntrsctn;
} // end of GetPrevIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLateralDistance
//	Returns the lateral distance between the center of the current lane and
//	the center of the parameter.
//
// Remarks: If the current lane and the parameter do not lie on the same road, 
//	or if either lane is invalid, this function will cause a failed assertion.
//
// Arguments:
//	lane - a valid CLane instance that lies on the same road as the current 
//		lane.
//
// Returns: The difference between the offset of the parameter and the offset
//	of the current lane.  A positive value indicates that the parameter lies
//	in the direction of the right vector with respect to the current lane, and
//	a negative value indicates that the parameter lies in the opposite 
//	direction of the right vector with respect to the current lane.
//
//////////////////////////////////////////////////////////////////////////////
double
CLane::GetLateralDistance(const CLane& lane) const
{
	AssertValid();
	lane.AssertValid();
	assert(m_pLane->roadId == lane.m_pLane->roadId);

	return (lane.m_pLane->offset - m_pLane->offset);
} // end of GetLateralDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
//	Performs a deep copy of the parameter to the current instance
//	
// Remarks:
//
// Arguments:
//  icRhs - a reference to an object intended to be to the right of the =
//
// Returns: a reference to the current instance
//
//////////////////////////////////////////////////////////////////////////////
CLane&
CLane::operator=( const CLane& cRhs )
{
	if ( this != &cRhs )
	{
		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pLane = cRhs.m_pLane;
		m_pRng = cRhs.m_pRng;
		m_rngStreamId = cRhs.m_rngStreamId;
	}
	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This mutator sets the random number generator info.
//
// Remarks: If a random number generator isn't assigned then the Path
//   uses Rand instead.
//
// Arguments:
//   pRng - A pointer to an instance of CRandNumGen.
//   streamId - A stream id.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CLane::SetRandNumGen( CRandNumGen* pRng, int streamId )
{
	m_pRng = pRng;
	m_rngStreamId = streamId;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Return the lane number with respect to the road 
//
// Remarks:
//
// Arguments:
//
// Returns: The lane number with respect to the road
//
//////////////////////////////////////////////////////////////////////////////
TU8b
CLane::GetId( void ) const
{
    AssertValid();
	return m_pLane->laneNo;
} // end of GetId

//////////////////////////////////////////////////////////////////////////////
//
// Description: Return the lane array index  
//
// Remarks:
//
// Arguments:
//
// Returns: The lane number with respect to the road
//
//////////////////////////////////////////////////////////////////////////////
TU32b
CLane::GetIndex( void ) const
{
    AssertValid();
	return m_pLane->myId;
} // end of GetId

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRelativeId 
// 	Return the lane number with respect to the road.
//
// Remarks:
//
// Arguments:
//
// Returns: The lane number with respect to the road
//
//////////////////////////////////////////////////////////////////////////////
TU8b
CLane::GetRelativeId(void) const
{
    AssertValid();
	//TRoad* pRoad = BindRoad(m_pLane->roadId);
    //return m_pLane->myId - pRoad->laneIdx;
    return m_pLane->laneNo;
} // end of GetRelativeId

//////////////////////////////////////////////////////////////////////////////
//
// Description: Search (private)
// 	Search the right control point containing the lane informaiton
//
// Remarks:
//
// Arguments:
// 	dist - distance along the current lane to search
//
// Return: A number representing the right control point (with respect to the
// 	road).
//
//////////////////////////////////////////////////////////////////////////////
int 
CLane::Search(double dist) const
{
	AssertValid();

	cvELnDir	dir = GetDirection();
	CRoad		rd(GetRoad());
	int			numOfLanes = rd.GetNumLanes();
	//int			laneIdx = rd.GetLaneIdx();
	int			numOfCntrlPt = rd.GetCntrlPntCount();
	int			cntrlPntIdx = rd.GetCntrlPntIdx();

	cvTHeader*	pH   = static_cast<cvTHeader*>    (GetInst());
	char*		pOfs = static_cast<char*>  (GetInst()) + pH->longitCntrlOfs;
	cvTCntrlPnt*	pCp  =	reinterpret_cast<cvTCntrlPnt*> (pOfs) + cntrlPntIdx;

	if (dir == ePOS);
	else if (dir == eNEG)
		dist = rd.GetLinearLength() - dist;

		
	int begin	= 0;
	int end		= numOfCntrlPt - 1;
	int middle	= (begin+end)/2;;

	while(middle!=begin){
		if ( dist < (pCp+middle)->distance )
			end = middle;
		else
			begin = middle;
		middle = (begin+end)/2;
	}
	return begin;
} // end of Search 

} // namespace CVED
