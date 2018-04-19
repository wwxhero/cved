//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: roadpos.cxx,v 1.150 2016/10/28 15:58:21 IOWA\dheitbri Exp $
//
// Author(s):   Yiannis Papelis
// Date:		September, 1998
//
// Description:	Implementation of the road position class.
//
//////////////////////////////////////////////////////////////////////////////
#include <time.h>
#include "cvedpub.h"
#include "cvedstrc.h"	// private CVED data structs
#include "splineHermite.h"
#include "cubicsplinepos.h"
#include "splineHermiteNonNorm.h"

#define	ROAD_POS_DEBUG	0
namespace CVED {

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator<<
// 	Prints the contents of the CRoadPos parameter to the ostream parameter.
//
// Remarks: The ostream&<< has to appear within the namespace, otherwise
//	it won't be properly assiciated.
//
// Arguments:
// 	out - an ostream reference to which the CRoadPos will be printed
// 	cRoadPos - a const reference to a CRoadPos instance.
//
// Returns: the ostream reference so that the operator<< may be nested.
//
//////////////////////////////////////////////////////////////////////////////
ostream& 
operator<<(ostream& out, const CRoadPos& cRoadPos)
{
	out << cRoadPos.GetString();
	return out;
} // end of operator<<

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator<<
// 	Prints the contents of the TSegment parameter to the ostream parameter.
//
// Remarks: The ostream&<< has to appear within the namespace, otherwise
//	it won't be properly assiciated.
//
// This function is used to debug the Travel function.
//
// Arguments:
// 	out - an ostream reference to which the TSegment will be printed
// 	cSeg - a const reference to a TSegment instance.
//
// Returns: the ostream reference so that the operator<< may be nested.
//
//////////////////////////////////////////////////////////////////////////////
ostream&
operator<<(ostream& out, const CRoadPos::TSegment& cSeg) 
{
	if (cSeg.isRoad) {
		
		out << "Road segment: " << endl;
		out << "  cp: [" 
			<< cSeg.begCntrlPntIdx << ", "
			<< cSeg.endCntrlPntIdx << "]" << endl;
		out << "  Road[" << cSeg.pRoad->myId << "]" << endl;
		out << "    cp: [" 
			<< cSeg.pRoad->cntrlPntIdx << ", "
			<< cSeg.pRoad->cntrlPntIdx + 
			   cSeg.pRoad->numCntrlPnt << "]"
			<< endl;
		out << "    isecs: " 
			<< cSeg.pRoad->srcIntrsctnIdx << " to "
			<< cSeg.pRoad->dstIntrsctnIdx << endl;
		out << "    lanes: "
			<< cSeg.pRoad->numOfLanes << endl;
	}
	else {

		out << "Isec segment: " << endl;
		out << "  cp: [" 
			<< cSeg.begCntrlPntIdx << ", "
			<< cSeg.endCntrlPntIdx << "]" << endl;
		out << "  Isec[" << cSeg.pCrdr->intrsctnId << "]" << endl;
		out << "  Crdr[" << cSeg.pCrdr->myId << "]" << endl;
		out << "    cp: [" 
			<< cSeg.pCrdr->cntrlPntIdx << ", "
			<< cSeg.pCrdr->cntrlPntIdx + 
			   cSeg.pCrdr->numCntrlPnt << "]"
			<< endl;
		out << "    src: [" 
			<< cSeg.pCrdr->srcRdIdx << ", "
			<< cSeg.pCrdr->srcLnIdx << "]" << endl;
		out << "    dst: [" 
			<< cSeg.pCrdr->dstRdIdx << ", "
			<< cSeg.pCrdr->dstLnIdx << "]" << endl;
	}
	return out;

} // end of operator<<

} // end of namespace CVED

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//		Assignment op and destructor
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy of the CRoadPos parameter to the current object.
//
// Remarks: 
//
// Arguments:
//  cRhs - a reference to an object intended to be to the right of the =
//
// Returns: a reference to the current object
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos&
CRoadPos::operator=( const CRoadPos& cRhs )
{
	if (&cRhs != this) 
	{
		// Assign superclass members
		this->CCvedItem::operator=( cRhs );

		m_isRoad		= cRhs.m_isRoad;
		
		m_pRoad			= cRhs.m_pRoad;
		m_pLane			= cRhs.m_pLane;
		m_dist			= cRhs.m_dist;
		m_ofs			= cRhs.m_ofs;
		m_cntrlPntIdx 	= cRhs.m_cntrlPntIdx;

		m_pIntrsctn		= cRhs.m_pIntrsctn;

		m_cdo.clear();
		m_cdo			= cRhs.m_cdo;

		m_pRng          = cRhs.m_pRng;
		m_rngStreamId   = cRhs.m_rngStreamId;
	}
	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CRoadPos
// 	The default destructor does nothing
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::~CRoadPos()
{} // end of ~CRoadPos


//////////////////////////////////////////////////////////////////////////////
//		Road constructors
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes a CRoad instance and a lane id.
// 
// Remarks: Creates a CRoadPos that is on a road.
//
// Arguments:
//  cRoad	- a valid CRoad instance
//  cLane	- id of the lane with respect to the road
//  dist 	- the distance component
//  ofs  	- the offset component
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos(
			const CRoad&	cRoad,
			int				lane,
			double			dist,
			double			ofs
			) 
	: 
	CCvedItem( cRoad ), 
	m_isRoad( true ), 
	m_dist( dist ),
	m_ofs( ofs ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	m_pRoad = BindRoad(cRoad.GetId());
	m_pLane = BindLane(m_pRoad->laneIdx + lane);

	// Find the control point associated with the 
	// 	given distance along the road.
	m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

	TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
	m_pLane = BindLane(pCurCP->laneIdx + lane);

} // end of CRoadPos
		
//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes a CRoad instance and a CLane instance.
// 
// Remarks: Creates a CRoadPos that is on a road.
//
// Arguments:
//  cRoad	- a valid CRoad instance
//  cLane	- a valid CLane instance
//  dist 	- the distance component
//  ofs  	- the offset component
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos(
			const CRoad&	cRoad,
			const CLane&	cLane,
			double			dist,
			double			ofs
			) 
	: 
	CCvedItem( cRoad ), 
	m_isRoad( true ), 
	m_dist( dist ),
	m_ofs( ofs ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	m_pRoad = BindRoad(cRoad.GetId());
	m_pLane = BindLane(cLane.GetRelativeId()); //GetRID_

	// Find the control point associated with the 
	// 	given distance along the road.
	m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
	TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
	m_pLane = BindLane(cLane.GetRelativeId() + pCurCP->laneIdx);
}


//////////////////////////////////////////////////////////////////////////////
//		Intersection constructors
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes a CIntrsctn instance and a corridor id.
// 
// Remarks: Creates a CRoadPos that is on an intersection.
//
// Arguments:
//  cIntrsctn- a valid CIntrsctn instance
//  crdr	- id of the corridor with respect to the intrsctn  
//  dist 	- the distance component
//  ofs  	- the offset component
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos(
			const CIntrsctn& cIntrsctn,
			int				 crdr,
			double			 dist,
			double			 ofs
			) 
	: 
	CCvedItem( cIntrsctn ), 
	m_isRoad( false ),
	m_pRng( NULL ),
	m_rngStreamId( -1 ) 
{
	TCdo		tmpCdo;

	m_pIntrsctn = BindIntrsctn(cIntrsctn.GetId());
	tmpCdo.pCrdr = BindCrdr(m_pIntrsctn->crdrIdx + crdr); 
	tmpCdo.dist = dist;
	tmpCdo.ofs = ofs;

	// Find the control point associated with the 
	// 	given distance along the crdr.
	tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
		+ tmpCdo.pCrdr->cntrlPntIdx;

	// Make tmpCdo the only member of the m_cdo vector
	m_cdo.push_back(tmpCdo);
} // end of CRoadPos
		
//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes a CIntrsctn instance and a CCrdr instance.
//
// Remarks: Creates a CRoadPos that is on an intersection.
// 
// Arguments:
//  cIntrsctn- a valid CIntrsctn instance
//  cCrdr	- a valid CCrdr instance  
//  dist 	- the distance component
//  ofs  	- the offset component
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos(
			const CIntrsctn& cIntrsctn,
			const CCrdr&	 cCrdr,
			double			 dist,
			double			 ofs
			) 
	: 
	CCvedItem( cIntrsctn ), 
	m_isRoad( false ), 
	m_dist( dist ),
	m_ofs( ofs ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	TCdo		tmpCdo;

	m_pIntrsctn = BindIntrsctn(cIntrsctn.GetId());
	tmpCdo.pCrdr = BindCrdr(cCrdr.GetId()); 
	tmpCdo.dist = dist;
	tmpCdo.ofs = ofs;

	// Find the control point associated with the 
	// 	given distance along the crdr.
	tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
		+ tmpCdo.pCrdr->cntrlPntIdx;

	// Make tmpCdo the only member of the m_cdo vector
	m_cdo.push_back(tmpCdo);
} // end of CRoadPos


//////////////////////////////////////////////////////////////////////////////
//		Constructors for road or intersection
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes the string name of a road/intrsctn, the int id of 
// 	the lane/crdr, a distance, and an offset.
//
// Remarks: The default usage of this function is with a road name and 
// 	lane id, rather than an intrsctn name and crdr id.
//
// Arguments:
//  cCved 	- a reference to the cved item the object belongs to
//  name 	- if isRoad, name of road component, otherwise name of intrsctn
//  id		- if isRoad, id of lane, otherwise id of crdr
//  dist 	- the distance component
//  ofs  	- the offset component
//	isRoad  - (optional) default value is true, indicating that the parameters
//				should be used to construct a road CRoadPos.  False indicates
//				that an intersection CRoadPos should be constructed.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos(
			const CCved&	cCved,
			const string&	name,
			const int		id,
			const double	dist,
			const double	ofs, 
			const bool		isRoad
			) 
	: 
	CCvedItem( &cCved ), 
	m_isRoad( isRoad ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	void*		pEnv = GetInst();
	THeader*	pH   = static_cast<THeader*>(pEnv);
	char*		pCharPool = static_cast<char*>(pEnv) + pH->charOfs;
	unsigned int i;
	TCdo		tmpCdo;

	// If the parameters are related to roads, 
	// 	assign the road data.
	if (m_isRoad) {

		for (i = 1; i < pH->roadCount; i++) {
			m_pRoad = BindRoad(i);
			if ( name == (pCharPool + m_pRoad->nameIdx) )
				break;
		}
		if (i == pH->roadCount){
			//there's no road with this name
			string msg("Invalid road name");
			cvCInternalError e(msg, __FILE__, __LINE__);
			throw e;
		}

		m_pLane = BindLane(m_pRoad->laneIdx + id);

		m_dist = dist;
		m_ofs = ofs;

		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = Search(dist) + m_pRoad->cntrlPntIdx;
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
        m_pLane = BindLane(pCurCP->laneIdx + id);
	}

	// Otherwise, assign the intersection data.
	else {

		for (i = 1; i < pH->intrsctnCount; i++) {
			m_pIntrsctn = BindIntrsctn(i);
			if ( name == (pCharPool + m_pIntrsctn->nameIdx) )
				break;
		}
		if (i == pH->intrsctnCount){
			//there's no intrsctn with this name
			string msg("Invalid intrsctn name");
			cvCInternalError e(msg, __FILE__, __LINE__);
			throw e;
		}

		tmpCdo.pCrdr = BindCrdr(m_pIntrsctn->crdrIdx + id);
		tmpCdo.dist = dist;
		tmpCdo.ofs = ofs;

		// Find the control point associated with the 
		// 	given distance along the crdr.
		tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
			+ tmpCdo.pCrdr->cntrlPntIdx;
	
		// Make tmpCdo the only member of the m_cdo vector
		m_cdo.push_back(tmpCdo);
	}
} // end of CRoadPos

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes a geometric position
//
// Remarks: Utilizes the SetXYZ() method.
//
// Arguments:
// 	pEnv - pointer to the CCved instance
// 	point - geometric point in space for which to find a matching road pos
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos( const CCved& cCved, const CPoint3D& cPoint) 
	: 
	CCvedItem(&cCved), 
	m_isRoad(true), 
	m_pRoad(0), 
	m_pLane(0), 
	m_dist(0.0), 
	m_ofs(0.0), 
	m_cntrlPntIdx(0),
	m_pIntrsctn(0),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	SetXYZ( cPoint );
} // end of CRoadPos


//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPos
// 	Constructor that takes a string
//
// Remarks: See SetString() for format of name parameter.
//
// Arguments:
// 	pEnv - pointer to the CCved instance
// 	name - string describing the road, lane, distance, offset
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::CRoadPos( const CCved& cCved, const string& cName ) 
	: 
	CCvedItem(&cCved), 
	m_isRoad(true), 
	m_pRoad(0), 
	m_pLane(0), 
	m_dist(0.0), 
	m_ofs(0.0), 
	m_cntrlPntIdx(0),
	m_pIntrsctn(0),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{
	if (cName == "") return;
	SetString(cName);
} // end of CRoadPos


//////////////////////////////////////////////////////////////////////////////
//		Accessors
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsRoad
// 	Indicate whether the current CRoadPos represents a road or an intersection 
// 	position.
//
// Remarks: Does not check that CRoadPos is valid
//
// Arguments:
//
// Returns: true if the CRoadPos is a road position, or false if it's an 
// 	intrsctn position.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::IsRoad(void) const
{
	return m_isRoad;
} // end of IsRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLane
// 	Return the lane component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on a road, then the lane component
// 	is returned.  Otherwise, an invalid CLane instance is returned.
//
//////////////////////////////////////////////////////////////////////////////
CLane  
CRoadPos::GetLane(void) const
{
	AssertValid();
	
	if (m_isRoad) {
		CLane l(GetCved(), m_pLane);
		return l;
	}
	else {
		CLane l;
		return l;
	}
} // end of GetLane
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLane
// 	Return the lane component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on a road, then the lane component
// 	is returned.  Otherwise, an invalid CLane instance is returned.
//
//////////////////////////////////////////////////////////////////////////////
CLane  
CRoadPos::GetSplineLane(void) const
{
	AssertValid();
	
	if (m_isRoad) {
		CLane l(GetCved(), m_pSplineLane);
		return l;
	}
	else {
		CLane l;
		return l;
	}
} // end of GetLane


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoad
// 	Return the road component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on a road, then the road component
// 	is returned.  Otherwise, an invalid CRoad instance is returned.
//
//////////////////////////////////////////////////////////////////////////////
CRoad  
CRoadPos::GetRoad(void) const
{
	AssertValid();

	if (m_isRoad) {
		CRoad r(GetCved(), m_pRoad->myId);
		return r;
	}
	else {
		CRoad r;
		return r;
	}
} // end of GetRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCorridor
// 	Return the crdr component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	id - (optional) Identifier, with respect to the intersection, of the 
//		corridor to get.  The default value is -1, which indicates that the
//		first corridor should be returned.
//
// Returns: If the current CRoadPos lies on an intrsctn, then the crdr 
// 	component is returned.  Otherwise, an invalid CCrdr instance is returned.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CRoadPos::GetCorridor(int id) const
{
	AssertValid();
	
	if (!m_isRoad) {
		
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {

			if ( (id < 0) || 
				 ((itr->pCrdr->myId - m_pIntrsctn->crdrIdx) == id) ) {
				CCrdr c(GetCved(), itr->pCrdr->myId);
				return c;
			}
		}
	}
	CCrdr c;
	return c;
} // end of GetCorridor


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextCrdr
// 	Returns the next oncoming corridor based on a direction and a lane index
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion. 
//
//	Occasionally the next intersection may have multiple corridors with
//	the same direction (straight). For these cases, the function will
//	obtain all control points of each vector and compare the direction
//	of each corridor to its current direction by taking the cross product
//	of its current right normal vector with the cross product of every
//	right normal vector along the corridor. The function then computes the
//	average of all cross products along the corridor.
//	Straight corridors will have an average cross product close to 0, 
//	right corridors will have the lowest average cross product and left
//	corridors will have the highest average cross product.
//
// Arguments:
//	crdrId - (output) the relative id of the next corridor
//	dirOut - (output) the direction of the next corridor
//	eTurnDir - (optional) if specified, will attempt to find a corridor 
//		going the specified direction. Default is straight
//	laneIdx - (optional) the index (absolute id) of the lane. If the CRoadPos
//		is on a road or if a lane index is not provided then the function
//		will use CRoadPos.GetLane(). If the position is on an intersection then
//		it will use the provided lane index to determine which corridor to use
//
// Returns: the next corridor or an invalid corridor upon failure
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CRoadPos::GetNextCrdr(int& crdrId, ETravelTurnDir& dirOut, ETravelTurnDir eTurnDir, const int laneIdx) const{

	AssertValid();

	TCrdrVec crdrVec;
	CIntrsctn intr = GetNextIntrsctn();
	if (!intr.IsValid()){
		return CCrdr();
	}

	// denotes whether or not we need to check if corridors contain this point
	bool checkPoint = false;

	// get set of corridors to check
	if (m_isRoad){
		intr.GetCrdrsStartingFrom(GetLane(), crdrVec);
	} else {
		if (laneIdx != -1){
			CLane lane(GetCved(), BindLane(laneIdx));
			intr.GetCrdrsStartingFrom(lane, crdrVec);
		} else {
			intr.GetAllCrdrs(crdrVec);
			checkPoint = true;
		}
	}

	// intersection has no corridors
	if (crdrVec.size() == 0){
		return CCrdr();
	}

	// default outputs (if no corridor is found, use first corridor)
	crdrId = crdrVec[0].GetRelativeId();
	if (crdrVec[0].GetCrdrDirection() == CCrdr::eSTRAIGHT){
		dirOut = CRoadPos::eSTRAIGHT;
	} else if (crdrVec[0].GetCrdrDirection() == CCrdr::eLEFT){
		dirOut = CRoadPos::eLEFT;
	} else if (crdrVec[0].GetCrdrDirection() == CCrdr::eRIGHT){
		dirOut = CRoadPos::eRIGHT;
	}

	int index = 0;
	TCrdrVec straightCrdrVec;
	for(size_t i=0; i<crdrVec.size(); i++){
		if ( (crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT  && eTurnDir == CRoadPos::eLEFT ) ||
			 (crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT && eTurnDir == CRoadPos::eRIGHT)) {
			
			// check if the corridor contains the point (only if lane index not provided & point is on intersection)
			if (checkPoint && !HasCorridor(crdrVec[i].GetRelativeId())){
				continue;
			}

			// found a suitable corridor
			crdrId = crdrVec[i].GetRelativeId();
			dirOut = eTurnDir;
			return crdrVec[i];
		}
		if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
			if (checkPoint && !HasCorridor(crdrVec[i].GetRelativeId())){
				continue;
			}
			// will return a straight corridor if no suitable corridor is found
			crdrId = crdrVec[i].GetRelativeId();
			dirOut = CRoadPos::eSTRAIGHT;
			index = (int)i;
			straightCrdrVec.push_back(crdrVec[i]);
		}
	}

	// intersection has multiple straight corridors
	// also could not find any left or right corridors (if searching for them)
	if (straightCrdrVec.size() > 1) {
		// set default corridor (this will be corridor with lowest ID)
		int curCrdr = 0; 

		// in intersections with 2+ straight corridors for one lane, a turn signal
		// is not necessary so the corridor is technically a straight corridor
		dirOut = CRoadPos::eSTRAIGHT;	

		// get right normal vector at current position
		CVector3D curCPRVec;
		if (m_isRoad) {
			TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);

			curCPRVec.m_i = pCurCP->rightVecLinear.i;
			curCPRVec.m_j = pCurCP->rightVecLinear.j;
			curCPRVec.m_k = 0.0;
		} else {
			vector<TCdo>::const_iterator cdo = m_cdo.begin();
			TCrdrPnt* pCurCP = BindCrdrPnt(cdo->cntrlPntIdx);

			curCPRVec.m_i = pCurCP->rightVecLinear.i;
			curCPRVec.m_j = pCurCP->rightVecLinear.j;
			curCPRVec.m_k = 0.0;
		} 

		curCPRVec.Normalize();

		// variables to test
		CCrdr::TCntrlPntVec crdrPntVec;
		CVector3D testCPRVec;
		double currentAvg = 0;

		// set initial average cross product for comparisons
		straightCrdrVec[0].GetCntrlPnts(crdrPntVec);
		for (size_t i=0; i<crdrPntVec.size(); i++) {

			// get right vector at each control point
			testCPRVec.m_i = crdrPntVec[i].rightVecLinear.i;
			testCPRVec.m_j = crdrPntVec[i].rightVecLinear.j;
			testCPRVec.m_k = 0.0;

			testCPRVec.Normalize();

			// do 2D cross product
			currentAvg += curCPRVec.m_i * testCPRVec.m_j - curCPRVec.m_j * testCPRVec.m_i;
		}

		// since corridors can have a differing number of control points, we need to
		// take the average
		currentAvg /= crdrPntVec.size();

		// test all other corridors 
		double testAvg;
		for(size_t i=1; i<straightCrdrVec.size(); i++) {
			straightCrdrVec[i].GetCntrlPnts(crdrPntVec);

			for (size_t j=0; j<crdrPntVec.size(); j++) {

				testCPRVec.m_i = crdrPntVec[j].rightVecLinear.i;
				testCPRVec.m_j = crdrPntVec[j].rightVecLinear.j;
				testCPRVec.m_k = 0.0;

				testCPRVec.Normalize();

				// do 2D cross product
				testAvg += curCPRVec.m_i * testCPRVec.m_j - curCPRVec.m_j * testCPRVec.m_i;
			}

			testAvg /= crdrPntVec.size();

			// depending on the value of the test average and the turn parameter
			// update the current corridor and current average accordingly
			if ( (testAvg > currentAvg           && eTurnDir == CRoadPos::eLEFT    ) || 
				 (testAvg < currentAvg           && eTurnDir == CRoadPos::eRIGHT   ) || 
				 (abs(testAvg) < abs(currentAvg) && eTurnDir == CRoadPos::eSTRAIGHT)) {

				currentAvg = testAvg;
				curCrdr = (int)i;
			}
		}

		// output the corridor ID and return the corridor
		crdrId = straightCrdrVec[curCrdr].GetRelativeId();
		return straightCrdrVec[curCrdr];
	}

	return crdrVec[index];

} // end of GetNextCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextCrdrById
// 	Returns the next oncoming corridor based on the provided relative ID
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion. 
//
//
// Arguments:
//	crdrId - the relative id of the next corridor
//	dirOut - (output) the direction of the next corridor
//
// Returns: the next corridor or an invalid corridor upon failure
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CRoadPos::GetNextCrdrById(int crdrId, ETravelTurnDir& out) const{

	AssertValid();

	TCrdrVec crdrVec;
	CIntrsctn intr = GetNextIntrsctn();
	if (!intr.IsValid()){
		return CCrdr();
	}

	intr.GetAllCrdrs(crdrVec);

	// look for corridor with matching id
	for(size_t i=0; i<crdrVec.size(); i++) {
		if (crdrVec[i].GetRelativeId() == crdrId) {

			// set output (direction of corridor)
			if (crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT) {
				out = eLEFT;
			} else if (crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT) {
				out = eRIGHT;
			} else {
				out = eSTRAIGHT;
			}

			return crdrVec[i];
		}
	}

	// return invalid corridor upon failure
	return CCrdr();

} // end of GetNextCrdrById


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextCrdr
// 	Returns the next oncoming corridor based on a direction and a lane index
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion. 
//
//	Occasionally the next intersection may have multiple corridors with
//	the same direction (straight). For these cases, the function will
//	obtain all control points of each vector and compare the direction
//	of each corridor to its current direction by taking the cross product
//	of its current right normal vector with the cross product of every
//	right normal vector along the corridor. The function then computes the
//	average of all cross products along the corridor.
//	Straight corridors will have an average cross product close to 0, 
//	right corridors will have the lowest average cross product and left
//	corridors will have the highest average cross product.
//
// Arguments:
//	crdrId - (output) the relative id of the next corridor
//	dirOut - (output) the direction of the next corridor
//	eTurnDir - (optional) if specified, will attempt to find a corridor 
//		going the specified direction. Default is straight
//	laneIdx - (optional) the index (absolute id) of the lane. If the CRoadPos
//		is on a road or if a lane index is not provided then the function
//		will use CRoadPos.GetLane(). If the position is on an intersection then
//		it will use the provided lane index to determine which corridor to use
//
// Returns: the next corridor or an invalid corridor upon failure
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CRoadPos::GetPrevCrdr(int& crdrId, ETravelTurnDir& dirOut, ETravelTurnDir eTurnDir, const int laneIdx) const{

	AssertValid();

	TCrdrVec crdrVec;
	CIntrsctn intr = GetPrevtIntrsctn();
	if (!intr.IsValid()){
		return CCrdr();
	}

	// denotes whether or not we need to check if corridors contain this point
	bool checkPoint = false;

	// get set of corridors to check
	if (m_isRoad){
        intr.GetCrdrsLeadingTo(GetLane(), crdrVec);
	} else {
		if (laneIdx != -1){
			CLane lane(GetCved(), BindLane(laneIdx));
			intr.GetCrdrsLeadingTo(lane, crdrVec);
		} else {
			intr.GetAllCrdrs(crdrVec);
			checkPoint = true;
		}
	}

	// intersection has no corridors
	if (crdrVec.size() == 0){
		return CCrdr();
	}

	// default outputs (if no corridor is found, use first corridor)
	crdrId = crdrVec[0].GetRelativeId();
	if (crdrVec[0].GetCrdrDirection() == CCrdr::eSTRAIGHT){
		dirOut = CRoadPos::eSTRAIGHT;
	} else if (crdrVec[0].GetCrdrDirection() == CCrdr::eLEFT){
		dirOut = CRoadPos::eLEFT;
	} else if (crdrVec[0].GetCrdrDirection() == CCrdr::eRIGHT){
		dirOut = CRoadPos::eRIGHT;
	}

	int index = 0;
	TCrdrVec straightCrdrVec;
	for(size_t i=0; i<crdrVec.size(); i++){
		if ( (crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT  && eTurnDir == CRoadPos::eLEFT ) ||
			 (crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT && eTurnDir == CRoadPos::eRIGHT)) {
			
			// check if the corridor contains the point (only if lane index not provided & point is on intersection)
			if (checkPoint && !HasCorridor(crdrVec[i].GetRelativeId())){
				continue;
			}

			// found a suitable corridor
			crdrId = crdrVec[i].GetRelativeId();
			dirOut = eTurnDir;
			return crdrVec[i];
		}
		if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
			if (checkPoint && !HasCorridor(crdrVec[i].GetRelativeId())){
				continue;
			}
			// will return a straight corridor if no suitable corridor is found
			crdrId = crdrVec[i].GetRelativeId();
			dirOut = CRoadPos::eSTRAIGHT;
			index = (int)i;
			straightCrdrVec.push_back(crdrVec[i]);
		}
	}

	// intersection has multiple straight corridors
	// also could not find any left or right corridors (if searching for them)
	if (straightCrdrVec.size() > 1) {
		// set default corridor (this will be corridor with lowest ID)
		int curCrdr = 0; 

		// in intersections with 2+ straight corridors for one lane, a turn signal
		// is not necessary so the corridor is technically a straight corridor
		dirOut = CRoadPos::eSTRAIGHT;	

		// get right normal vector at current position
		CVector3D curCPRVec;
		if (m_isRoad) {
			TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);

			curCPRVec.m_i = pCurCP->rightVecLinear.i;
			curCPRVec.m_j = pCurCP->rightVecLinear.j;
			curCPRVec.m_k = 0.0;
		} else {
			vector<TCdo>::const_iterator cdo = m_cdo.begin();
			TCrdrPnt* pCurCP = BindCrdrPnt(cdo->cntrlPntIdx);

			curCPRVec.m_i = pCurCP->rightVecLinear.i;
			curCPRVec.m_j = pCurCP->rightVecLinear.j;
			curCPRVec.m_k = 0.0;
		} 

		curCPRVec.Normalize();

		// variables to test
		CCrdr::TCntrlPntVec crdrPntVec;
		CVector3D testCPRVec;
		double currentAvg = 0;

		// set initial average cross product for comparisons
		straightCrdrVec[0].GetCntrlPnts(crdrPntVec);
		for (size_t i=0; i<crdrPntVec.size(); i++) {

			// get right vector at each control point
			testCPRVec.m_i = crdrPntVec[i].rightVecLinear.i;
			testCPRVec.m_j = crdrPntVec[i].rightVecLinear.j;
			testCPRVec.m_k = 0.0;

			testCPRVec.Normalize();

			// do 2D cross product
			currentAvg += curCPRVec.m_i * testCPRVec.m_j - curCPRVec.m_j * testCPRVec.m_i;
		}

		// since corridors can have a differing number of control points, we need to
		// take the average
		currentAvg /= crdrPntVec.size();

		// test all other corridors 
		double testAvg;
		for(size_t i=1; i<straightCrdrVec.size(); i++) {
			straightCrdrVec[i].GetCntrlPnts(crdrPntVec);

			for (size_t j=0; j<crdrPntVec.size(); j++) {

				testCPRVec.m_i = crdrPntVec[j].rightVecLinear.i;
				testCPRVec.m_j = crdrPntVec[j].rightVecLinear.j;
				testCPRVec.m_k = 0.0;

				testCPRVec.Normalize();

				// do 2D cross product
				testAvg += curCPRVec.m_i * testCPRVec.m_j - curCPRVec.m_j * testCPRVec.m_i;
			}

			testAvg /= crdrPntVec.size();

			// depending on the value of the test average and the turn parameter
			// update the current corridor and current average accordingly
			if ( (testAvg > currentAvg           && eTurnDir == CRoadPos::eLEFT    ) || 
				 (testAvg < currentAvg           && eTurnDir == CRoadPos::eRIGHT   ) || 
				 (abs(testAvg) < abs(currentAvg) && eTurnDir == CRoadPos::eSTRAIGHT)) {

				currentAvg = testAvg;
				curCrdr = (int)i;
			}
		}

		// output the corridor ID and return the corridor
		crdrId = straightCrdrVec[curCrdr].GetRelativeId();
		return straightCrdrVec[curCrdr];
	}

	return crdrVec[index];

} // end of GetNextCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextCrdrById
// 	Returns the next oncoming corridor based on the provided relative ID
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion. 
//
//
// Arguments:
//	crdrId - the relative id of the next corridor
//	dirOut - (output) the direction of the next corridor
//
// Returns: the next corridor or an invalid corridor upon failure
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CRoadPos::GetPrevCrdrById(int crdrId, ETravelTurnDir& out) const{

	AssertValid();

	TCrdrVec crdrVec;
    CIntrsctn intr = GetPrevtIntrsctn();
	if (!intr.IsValid()){
		return CCrdr();
	}

	intr.GetAllCrdrs(crdrVec);

	// look for corridor with matching id
	for(size_t i=0; i<crdrVec.size(); i++) {
		if (crdrVec[i].GetRelativeId() == crdrId) {

			// set output (direction of corridor)
			if (crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT) {
				out = eLEFT;
			} else if (crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT) {
				out = eRIGHT;
			} else {
				out = eSTRAIGHT;
			}

			return crdrVec[i];
		}
	}

	// return invalid corridor upon failure
	return CCrdr();

} // end of GetNextCrdrById

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCorridor
// 	Return the crdr component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	id - (optional) Identifier, with respect to the intersection, of the 
//		corridor to get.  The default value is -1, which indicates that the
//		first corridor should be returned.
//
// Returns: If the current CRoadPos lies on an intrsctn, then the crdr 
// 	component is returned.  Otherwise, an invalid CCrdr instance is returned.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CRoadPos::GetCorridorWithAbsoluteId(int id) const
{
	AssertValid();
	
	if (!m_isRoad) {
		
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {

			if ( (id < 0) || 
				 (itr->pCrdr->myId == id) ){
				CCrdr c(GetCved(), itr->pCrdr->myId);
				return c;
			}
		}
	}
	CCrdr c;
	return c;
} // end of GetCorridor
//////////////////////////////////////////////////////////////////////////////
///\brief
///		Returns of the list of crds the roadpos contains
///\remark 	
///		This function copies a list on crd id's. Will cause assert failure for invalide
///		intersections
///	
///\param	crds -destination to copy crds to (absolute id)
///\return		true -we copied the contents
///\return		false, either we are on a road, 
//////////////////////////////////////////////////////////////////////////////
bool CRoadPos::GetCorridors(vector<int> &crds) const{
	AssertValid();
	crds.clear();
	if (!m_isRoad) {
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
			crds.push_back(itr->pCrdr->myId);
		}
		return true;
	}
	return false;
}
//////////////////////////////////////////////////////////////////////////////
///\brief
///		Returns of the list of crds the roadpos contains
///\remark 	
///		This function copies a list on crd id's. Will cause assert failure for invalide
///		intersections
///	
///\param	crds -destination to copy crds to (absolute id)
///\return		true -we copied the contents
///\return		false, either we are on a road, 
//////////////////////////////////////////////////////////////////////////////
bool CRoadPos::GetCorridors(vector< pair<int,double> > &crds) const{
	AssertValid();
	crds.clear();
	if (!m_isRoad) {
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
            crds.push_back(make_pair(itr->pCrdr->myId,itr->dist));
		}
		return true;
	}
	return false;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: HasCorridor
// 	Indicates whether the given corridor is contained in the current CRoadPos
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	crdrId - Identifier of the corridor to find, with respect to the current
//	intersection.
//
// Returns: If the current CRoadPos lies on an intrsctn, then the list of 
//	corridors is examined to find a match to the parameter.  Otherwise, the
//	function returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::HasCorridor(int crdrId) const
{
	AssertValid();
	
	if (!m_isRoad) {
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {

			if ( (itr->pCrdr->myId - m_pIntrsctn->crdrIdx) == crdrId )
				return true;
		}
	}

	return false;
} // end of HasCorridor
//////////////////////////////////////////////////////////////////////////////
//
// Description: HasCorridor
// 	Indicates whether the given corridor is contained in the current CRoadPos
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	crdrId - Identifier of the corridor to find, with respect to the current
//	intersection.
//
// Returns: If the current CRoadPos lies on an intrsctn, then the list of 
//	corridors is examined to find a match to the parameter.  Otherwise, the
//	function returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool			
CRoadPos::HasCorridorWithAbsoluteId(int crdrId) const{
	AssertValid();
	
	if (!m_isRoad) {
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {

			if ( itr->pCrdr->myId == crdrId )
				return true;
		}
	}

	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetIntrsctn
// 	Return the intrsctn component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on an intrsctn, then the intrsctn 
// 	component is returned.  Otherwise, an invalid CIntrsctn instance is 
// 	returned.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn   
CRoadPos::GetIntrsctn(void) const
{
	AssertValid();

	if (!m_isRoad) {
		CIntrsctn i(GetCved(), m_pIntrsctn->myId);
		return i;
	}
	else {
		CIntrsctn i;
		return i;
	}
} // end of GetIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextIntrsctn
// 	Return the next intersection component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on an intrsctn, then the intrsectionn 
// 	component is returned.  If the current CRoadPos lies on a road, then either
//  the next intersection component or an invalid intersection is returned
//
//////////////////////////////////////////////////////////////////////////////

CIntrsctn
CRoadPos::GetNextIntrsctn(void) const
{
	AssertValid();

	if (!m_isRoad) {
		return GetIntrsctn();
	}

	CLane lane(GetCved(), m_pLane);
	if(lane.IsValid()){
		return lane.GetNextIntrsctn();
	} else {
		return CIntrsctn();
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextIntrsctn
// 	Return the next intersection component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on an intrsctn, then the intrsectionn 
// 	component is returned.  If the current CRoadPos lies on a road, then either
//  the next intersection component or an invalid intersection is returned
//
//////////////////////////////////////////////////////////////////////////////

CIntrsctn
CRoadPos::GetPrevtIntrsctn(void) const
{
	AssertValid();

	if (!m_isRoad) {
		return GetIntrsctn();
	}

	CLane lane(GetCved(), m_pLane);
	if(lane.IsValid()){
        return lane.GetPrevIntrsctn();
	} else {
		return CIntrsctn();
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetDistanceToNextIntrsctn
// 	Return the distance to the next intersection
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//
// Returns: If the current CRoadPos lies on an intrsctn, 0 is returned.
//  If the current CRoadPos lies on a road, then the distance to
//  the next intersection component is returned
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetDistanceToNextIntrsctn(void) const
{
	AssertValid();

	if (!m_isRoad) {
		return 0;
	}

	return GetRoad().GetLinearLength() - GetDistanceOnLane();
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the distance to the next halt line based on some 
//	specified starting position. If a halt line is found, the relative id
//	of the corridor the halt line is on is also outputted. 
//
// Remarks: CRoadPos can be on a road or an intersection. If lookahead is larger than
//	1 then subsequent intersections will always be intersections that can be
//	reached by going straight or by taking the first corridor.
//
// Arguments:
//  cRoadPos - position to search for the next halt line from
// 	lookahead - (optional) specifies the number of subsequent intersections to check
//		if the first intersection does not contain a halt line. GetNextHldOfs() will
//		stop looking for subsequent halt lines if a halt line is found.
//
// Returns: the distance to the next halt line or -1 upon failure
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetDistToNextHldOfs( int lookahead ) const{
	CIntrsctn intr;
	vector<CHldOfs> hldOfsList;
	TCrdrVec crdrVec;

	// get the next or current intersection
	intr = GetNextIntrsctn();
	if (!intr.IsValid()){
		return -1;
	}

	CLane lane = GetLane();
	if (IsRoad() && !lane.IsValid()){
		return -1;
	}

	int n = 0;
	if (lookahead < 1){
		lookahead = 1;
	}

	double dist = 0;
	
	// check if point is on a road or intersection
	// may return the wrong haltline if a point lies on two different corridors
	if (!IsRoad()){
		// query the next few intersections for haltlines
		while( n < lookahead ){
			// check if any corridors contain the point and if so, return the haltline
			intr.GetAllCrdrs(crdrVec);

			if (crdrVec.size() == 0){
				return -1;
			}

			for(size_t i=0; i<crdrVec.size(); i++){
				if (HasCorridor(crdrVec[i].GetRelativeId())){
					crdrVec[i].GetAllHldOfs(hldOfsList);
					if (hldOfsList.size() > 0){
						if (n == 0){
							return hldOfsList.begin()->GetDistance() - GetDistanceOnLane(crdrVec[i].GetRelativeId());
						} else {
							return hldOfsList.begin()->GetDistance() + dist;
						}
					}
				}
			}

			// no haltlines found
			int crdr = -1;
			for(size_t i=0; i<crdrVec.size(); i++){
				if (HasCorridor(crdrVec[i].GetRelativeId())){
					if (crdr == -1){
						crdr = (int)i;
					}
					if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
						crdr = (int)i;
						break;
					}
				}
			}

			// no corridors found
			if (crdr == -1){
				return -1;
			}

			// find the next lane & intersection
			lane = crdrVec[crdr].GetDstntnLn();
			if (!lane.IsValid()){
				return -1;
			}
			intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return -1;
			}

			if (n == 0){
				dist += lane.GetRoad().GetLinearLength() + crdrVec[crdr].GetLength() - GetDistanceOnLane(crdrVec[crdr].GetRelativeId());
			} else {
				dist += lane.GetRoad().GetLinearLength() + crdrVec[crdr].GetLength();
			}
			n++;
		}
	} else {
		// query the next few intersections for haltlines
		dist += GetRoad().GetLinearLength() - GetDistanceOnLane();
		while(n < lookahead){
			// get all corridors connected to the lane given by the position
			intr.GetCrdrsStartingFrom(lane, crdrVec);

			// no corridors in intersection
			if (crdrVec.size() == 0){
				return -1;
			}

			for(size_t i=0; i<crdrVec.size(); i++){
				crdrVec[i].GetAllHldOfs(hldOfsList);
				if (hldOfsList.size() > 0){
					return hldOfsList.begin()->GetDistance() + dist;
				}
			}

			// no haltlines found, look for straight corridor so we can find the next intersection
			int crdr = 0;
			for(size_t i=0; i<crdrVec.size(); i++){
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					crdr = (int)i;
					break;
				}
			}
			// find the next lane & intersection
			lane = crdrVec[crdr].GetDstntnLn();
			if (!lane.IsValid()){
				return -1;
			}
			intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return -1;
			}

			dist += lane.GetRoad().GetLinearLength() + crdrVec[crdr].GetLength();
			n++;
		}
	}

	// failed to find a suitable corridor with a haltline
	return -1;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the next halt line based on some specified starting
//	position. If a halt line is found, the relative id of the corridor the
//	halt line is on is also outputted. 
//
// Remarks: CRoadPos can be on a road or an intersection. If lookahead is larger than
//	1 then subsequent intersections will always be intersections that can be
//	reached by going straight or by taking the first corridor.
//
// Arguments:
//  cRoadPos - position to search for the next halt line from
//  crdrIdOut - (output) the relative id of the corridor the halt line is located on
// 	lookahead - (optional) specifies the number of subsequent intersections to check
//		if the first intersection does not contain a halt line. GetNextHldOfs() will
//		stop looking for subsequent halt lines if a halt line is found.
//
// Returns: the next halt line or an invalid halt line upon failure
//
//////////////////////////////////////////////////////////////////////////////
CHldOfs			
CRoadPos::GetNextHldOfs( int* crdrIdOut , int lookahead ) const{
	CIntrsctn intr;
	vector<CHldOfs> hldOfsList;
	TCrdrVec crdrVec;

	// get the next or current intersection
	intr = GetNextIntrsctn();
	if (!intr.IsValid()){
		return CHldOfs();
	}

	CLane lane = GetLane();
	if (IsRoad() && !lane.IsValid()){
		return CHldOfs();
	}

	int n = 0;
	if (lookahead < 1){
		lookahead = 1;
	}
	
	// check if point is on a road or intersection
	// may return the wrong haltline if a point lies on two different corridors
	if (!IsRoad()){
		while( n < lookahead ){
			// check if any corridors contain the point and if so, return the haltline
			intr.GetAllCrdrs(crdrVec);

			// no corridors in intersection
			if (crdrVec.size() == 0){
				return CHldOfs();
			}

			for(size_t i=0; i<crdrVec.size(); i++){
				if (HasCorridor(crdrVec[i].GetRelativeId())){
					crdrVec[i].GetAllHldOfs(hldOfsList);
					if (hldOfsList.size() > 0){
						if (crdrIdOut){
							*crdrIdOut = crdrVec[i].GetId();
						}
						return *hldOfsList.begin();
					}
				}
			}

			// no haltlines found
			int crdr = -1;
			for(size_t i=0; i<crdrVec.size(); i++){
				if (HasCorridor(crdrVec[i].GetRelativeId())){
					if (crdr == -1){
						crdr = (int)i;
					}
					if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
						crdr = (int)i;
						break;
					}
				}
			}

			// no corridors found
			if (crdr == -1){
				return CHldOfs();
			}

			// find the next lane & intersection
			lane = crdrVec[crdr].GetDstntnLn();
			if (!lane.IsValid()){
				return CHldOfs();
			}
			intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return CHldOfs();
			}

			n++;
		}
	} else {
		// query the next few intersections for haltlines
		while(n < lookahead ){
			// get all corridors connected to the lane given by the position
			intr.GetCrdrsStartingFrom(lane, crdrVec);

			//no corridors in intersection
			if (crdrVec.size() == 0){
				return CHldOfs();
			}

			for(size_t i=0; i<crdrVec.size(); i++){
				crdrVec[i].GetAllHldOfs(hldOfsList);
				if (hldOfsList.size() > 0){
					if (crdrIdOut){
						*crdrIdOut = crdrVec[i].GetId();
					}
					return *hldOfsList.begin();
				}
			}

			// no haltlines found, look for straight corridor so we can find the next intersection
			int crdr = 0;
			for(size_t i=0; i<crdrVec.size(); i++){
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					crdr = (int)i;
					break;
				}
			}

			// find the next lane & intersection
			lane = crdrVec[crdr].GetDstntnLn();
			if (!lane.IsValid()){
				return CHldOfs();
			}
			intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return CHldOfs();
			}

			n++;
		}
	}

	// failed to find a suitable corridor with a haltline
	return CHldOfs();
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetDistance
// 	Return the distance component
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	id - If the current CRoadPos is on a road, this parameter is ignored.  
//		Otherwise, it is an optional parameter specifying the identifier, 
//		with respect to the current intersection, of the corridor whose
//		distance should be returned.  The default value is -1, indicating
//		that the first corridor should be used. 
//
// Returns: The distance component.
//
//////////////////////////////////////////////////////////////////////////////
double  
CRoadPos::GetDistance(int id) const
{
	AssertValid();

	if (m_isRoad)
		return m_dist;
	else {

		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
			
			if ( (id < 0) ||
				 ((itr->pCrdr->myId - m_pIntrsctn->crdrIdx) == id) )
				return itr->dist;
		}
		return 0.0;
	}
} // end of GetDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetDistance
// 	Return the distance from the start of the lane.
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	id - If the current CRoadPos is on a road, this parameter is ignored.  
//		Otherwise, it is an optional parameter specifying the identifier, 
//		with respect to the current intersection, of the corridor whose
//		distance should be returned.  The default value is -1, indicating
//		that the first corridor should be used. 
//
// Returns: The distance from the start of the lane.
//
//////////////////////////////////////////////////////////////////////////////
double  
CRoadPos::GetDistanceOnLane(int id) const
{
	AssertValid();

	if (m_isRoad) {

		CLane curLane = GetLane();
		if ( curLane.GetDirection() == eNEG ) {

			CRoad road = GetRoad();
			return ( road.GetLinearLength() - m_dist );

		}
		else {

			return m_dist;

		}

	}
	else {

		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
			
			if ( (id < 0) ||
				 ((itr->pCrdr->myId - m_pIntrsctn->crdrIdx) == id) )
				return itr->dist;
		}
		return 0.0;

	}

} // end of GetDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLaneLength
// 	Return the length of the current lane or corridor.
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	id - If the current CRoadPos is on a road, this parameter is ignored.  
//		Otherwise, it is an optional parameter specifying the identifier, 
//		with respect to the current intersection, of the corridor whose
//		length should be returned.  The default value is -1, indicating
//		that the first corridor should be used. 
//
// Returns: The distance from the start of the lane or 0 upon failure.
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetLaneLength(int id) const {
	AssertValid();

	if (m_isRoad){
		if (!GetLane().IsValid() || !GetLane().GetRoad().IsValid()){
			return 0;
		}
		return GetLane().GetRoad().GetLinearLength();
	} else {
		CIntrsctn intr = GetIntrsctn();
		if (!intr.IsValid()){
			return 0;
		}
		TCrdrVec crdrVec;
		intr.GetAllCrdrs(crdrVec);

		if (crdrVec.size() == 0){
			return 0;
		}

		// search for corridor with matching id
		if (id != -1){
			for(size_t i=0; i<crdrVec.size(); i++){
				if (crdrVec[i].GetRelativeId() == id){
					return crdrVec[i].GetLength();
				}
			}
			// no corridor with matching ID found
			return 0;
		}

		// if no corridor found or if id == -1, return the first corridor's length by default
		return crdrVec[0].GetLength();
	}
}

///////////////////////////////////////////////////////////////////////////////
///
///\brief GetOffset, Return the offset component
///
///\remark If the CRoadPos is invalid, this function will cause a failed
/// 	assertion.
///
///\param	id - If the current CRoadPos is on a road, this parameter is ignored.  
///			Otherwise, it is an optional parameter specifying the identifier, 
///			with respect to the current intersection, of the corridor whose
///			offset should be returned.  The default value is -1, indicating
///			that the first corridor should be used.
///	
///\param	cpPath - the path to use to find the corridor, each roadpos may have
///			multiple corridors, use the corridor the is in the path, if there is 
///			no over lap between the corridors in the path and the roadpos, just use
///			use the first corridor we have
///
///\param	pCorrUsed - optional return value for the corridor relative id we  
///			used to calulate the offset, if we are on a roadsegment this will 
///			not be set 
///
///\return The offset component.
///
//////////////////////////////////////////////////////////////////////////////
double  
CRoadPos::GetOffset(int id, const CPath *cpPath, int* pCorrUsed ) const
{
	AssertValid();
	if (m_isRoad)
		return m_ofs;
	else { //We have Crd, if we have a path we need to get the offset to crdr pnt, else
		   //the first valid crdr will do
		return ComputerCrdrOffset(id,cpPath,pCorrUsed);
	}
} // end of GetOffset

//////////////////////////////////////////////////////////////////////////////
///
///\brief Get Offset Using a Spline
/// 	
///
///\remark 
///	If the CRoadPos is invalid, this function will cause a failed
/// 	assertion. Currently if we are in an intersection this will be the same
///		as our reg offset.
///
/// 
///\par	id - If the current CRoadPos is on a road, this parameter is ignored.  
///		Otherwise, it is an optional parameter specifying the identifier, 
///		with respect to the current intersection, of the corridor whose
///		offset should be returned.  The default value is -1, indicating
///		that the first corridor should be used. 
///
///\return 
///		The offset component.
///
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetSplineOffset(int id, const CPath *cpPath, int* pCorrUsed ) const
{
	AssertValid();
	if (m_isRoad)
		return m_splineOfs;
	else { //We have Crd, if we have a path we need to get the offset to crdr pnt, else
		   //the first valid crdr will do
		return ComputerCrdrOffset(id,cpPath,pCorrUsed);
	}
} // end of GetOffset
//////////////////////////////////////////////////////////////////////////////
///
///\brief Get Offset of a crdr
/// 	
///
///\remark 
///	If the CRoadPos is invalid, this function will cause a failed
/// 	assertion. This function is a helper function for the get lane offset 
///		functions
///
/// 
///\par	id - If the current CRoadPos is on a road, this parameter is ignored.  
///		Otherwise, it is an optional parameter specifying the identifier, 
///		with respect to the current intersection, of the corridor whose
///		offset should be returned.  The default value is -1, indicating
///		that the first corridor should be used. 
///
///\return 
///		The offset component.
///
//////////////////////////////////////////////////////////////////////////////
double	CRoadPos::ComputerCrdrOffset(int id, const CPath *cpPath,int* pCorrUsed) const{
	if (!cpPath || !cpPath->IsValid() || id >= 0 || !cpPath->Contains(*this)){ 
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
			if ( (id < 0) || //if id < 0, just pick the first one....
				( (itr->pCrdr->myId - m_pIntrsctn->crdrIdx) == id) ){
					if (pCorrUsed){
						*pCorrUsed = id;
					}
					return itr->ofs;
			}
		}
		return 0.0;
	}else{
		int targCrd = -1;
		//since a path may have multiple crds for a intersection, and a roadpos
		//can have multiple crdr, find the corridor that intersects the two sets
		//since we already know the path contains our target we should have at 
		//one crdr that intersects
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
			CVED::CRoad srcRoad(GetCved(), itr->pCrdr->srcRdIdx);
			if (cpPath->GetCrdrFromIntrscn(m_pIntrsctn->myId,targCrd,NULL,itr->pCrdr->srcLnIdx - srcRoad.GetLaneIdx()))
				break;
		}
		double ofs = 0.0;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++) {
				ofs = itr->ofs;
				if ((itr->pCrdr->myId - m_pIntrsctn->crdrIdx) == targCrd){
					if (pCorrUsed){
						*pCorrUsed = targCrd;
					}						
					return ofs;
				}
		}
		return ofs;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: 
// 	Return a string describing the road position
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
//	This function returns a string that contains a textual description
// 	of the road position.  The string has the format: XXX:YY:ZZ:WW where
// 	XXX is the road/intrsctn name, YY is the lane/crdr, 
// 	ZZ is the distance and WW is the offset.
//
// Arguments:
//
// Returns: A string describing the component.
//
//////////////////////////////////////////////////////////////////////////////
string 
CRoadPos::GetName(void) const
{
	AssertValid();
	string s;
	THeader *pH 	= static_cast<THeader*> (GetInst());
	char* pCharPool	= static_cast<char*> (GetInst()) + pH->charOfs;
	char dist[10];
	char ofs[10];

	// If current CRoadPos is on a road, assemble
	// 	the road name and lane Id.
	if (m_isRoad) {
		string rd(pCharPool + m_pRoad->nameIdx);
		s = rd + ':';
	
		char ln[10];
		sprintf_s(ln, "%d", m_pLane->laneNo);
		s += ln;
		s += ':';

		sprintf_s(dist, "%.2f", m_dist);
		s += dist;
		s += ':';
	
		sprintf_s(ofs, "%.2f", m_ofs);
		s += ofs;
	}
	// 	Otherwise, assemble the intrsctn name and crdr Id.
	else {
		string in(pCharPool + m_pIntrsctn->nameIdx);
		s = in + ':';
		
		vector<TCdo>::const_iterator cdo = m_cdo.begin();
		char cr[10];
		sprintf_s(cr, "%d", cdo->pCrdr->myId - m_pIntrsctn->crdrIdx);
		
		s += cr;
		s += ':';

		sprintf_s(dist, "%.2f", cdo->dist);
		s += dist;
		s += ':';
	
		sprintf_s(ofs, "%.2f", cdo->ofs);
		s += ofs;
	}

	return s;
} // end of GetName

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetString
// 	Returns a string containing the CRoadPos member data
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
//	This function returns the same value as GetName(), but is provided for 
//	consistency with the smparser code, so that CRoadPos may be used as a dial 
//	or monitor type.
//
// Arguments:
//
// Returns: a string representation of current instance
//
//////////////////////////////////////////////////////////////////////////////
string
CRoadPos::GetString() const
{
	return GetName();
} // end of GetString

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetAttribute
// 	Retrieves the attribute on the current road and lane that has the given 
// 	id.
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// Arguments:
//	id - identifier for an attribute type
//	attr - output parameter, contains the desired CAttr if the function 
//			returns true.
//
// Returns: If an attribute is found of the desired id on the current road, 
// 	lane, and distance, then attr is assigned that attribute and the functon
// 	returns true.  Otherwise, it returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::GetAttribute(const int id, CAttr& attr) const 
{
	AssertValid();

	if (m_isRoad) {
		CRoad road = GetRoad();

		bitset<cCV_MAX_LANES> lanes;
		lanes.set(m_pLane->laneNo);

		return road.QryAttr(id, attr, lanes.to_ulong(), m_dist);
	}
	else {

		CIntrsctn isec = GetIntrsctn();

		bitset<cCV_MAX_CRDRS> crdrs;
		vector<TCdo>::const_iterator itr;
		for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++)
			crdrs.set(itr->pCrdr->myId - m_pIntrsctn->crdrIdx);

	//	return isec.QryAttr(id, attr, crdrs.to_ulong(), 
	//		m_cdo.begin()->dist);
		return isec.QryAttr(id, attr);

	}
} // end of GetAttribute
//////////////////////////////////////////////////////////////////////////////
///\brief
/// 	Determines is the roadpos is on, a on ramp for a freeway 
/// 
///\remark
///     Crdrs are not marked as on ramps, we are assuming the section of 
///     crdr that does not over lap, and is connected to a lane that is an
///     on ramp, is a on ramp. Once the crdr intersects with the interstate
///     its no longer considered on, the on ramp
///////////////////////////////////////////////////////////////////////////////
bool 
CRoadPos::GetIsOnRamp(){
	AssertValid();

	if (m_isRoad) {
        CVED::CLane ln(GetCved(),m_pLane);
        if (!ln.IsValid())
            return false;
        return ln.IsOnRamp();
	}
	else {
        if (m_cdo.size() != 1)
            return false;
        CVED::CCrdr crdr (GetCved(),m_cdo[0].pCrdr);
        if (!crdr.IsValid())
            return false;
        auto lane = crdr.GetSrcLn();
        if (!lane.IsValid())
            return false;
		return lane.IsOnRamp();
	}
}
//////////////////////////////////////////////////////////////////////////////
///\brief
/// 	Determines is the roadpos is on, a on ramp for a freeway 
/// 
///\remark
///     Crdrs are not marked as on ramps, we are assuming the section of 
///     crdr that does not over lap, and is connected to a lane that is an
///     on ramp, is a on ramp. Once the crdr intersects with the interstate
///     its no longer considered on, the on ramp
///////////////////////////////////////////////////////////////////////////////
bool 
CRoadPos::GetIsOffRamp(){
	AssertValid();

	if (m_isRoad) {
        CVED::CLane ln(GetCved(),m_pLane);
        if (!ln.IsValid())
            return false;
        return ln.IsOffRamp();
	}
	else {
        if (m_cdo.size() != 1)
            return false;
        CVED::CCrdr crdr (GetCved(),m_cdo[0].pCrdr);
        if (!crdr.IsValid())
            return false;
        auto lane = crdr.GetDstntnLn();
        if (!lane.IsValid())
            return false;
		return lane.IsOffRamp();
	}
}

////////////////////////////////////////////////////////////////////////////////
///\brief
///     This actually is the same as getXYZ
///\remark
///     as the spline generation was incorrect for years, 
///     we have no way to tell if GetXYZ and GetBestXYZ was incorrectly intermixed
///		if they are intermixed you can have objects that slowly shift in position
///		as the difference between the two is added together.
///
///     We need to carefully move GetBestXYZ refrences to a corrected 
///		GetVeryBestXYZ
///////////////////////////////////////////////////////////////////////////////
CPoint3D
CRoadPos::GetBestXYZ() const 
{
    return GetXYZ();   
}
//////////////////////////////////////////////////////////////////////////////
///
///\brief GetVeryBestXYZ
/// 	Returns best (x,y,z) that corresponds to current road position. 
///		
///
///\remark
///		If the CRoadPos is invalid, this function will cause a failed
/// 	assertion.
///
/// 	Uses the pre-computed right vectors for the road spline to determine where 
/// 	(x,y,z) should be offset from the curve.  By contrast, GetXYZ(), GetBestXYZ 
///		uses an orthogonal vector to the vector between the two control points.
///		This function still uses a linear aproximator for T, this should use
///		an interative process such as newton raphson to get a better estimate
/// Arguments:
///
///\return 
///		The 3D point corresponding to the current road position.
///
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CRoadPos::GetVeryBestXYZ(bool useoffset) const 
{
	AssertValid();
	
	double t;

	CPoint3D curCPLoc, nexCPLoc;
	CVector3D curCPRVec, nexCPRVec;

	CVector3D rT;
	CPoint3D pT;
	double offset;
    double	relT;
	CPoint3D result;

	// If the current point is on a road
	if (m_isRoad) {
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		TCntrlPnt* pNexCP = pCurCP+1;
	
		curCPLoc.m_x = pCurCP->location.x;
		curCPLoc.m_y = pCurCP->location.y;
		curCPLoc.m_z = pCurCP->location.z;

		nexCPLoc.m_x = pNexCP->location.x;
		nexCPLoc.m_y = pNexCP->location.y;
		nexCPLoc.m_z = pNexCP->location.z;

		curCPRVec.m_i = pCurCP->rightVecLinear.i;
		curCPRVec.m_j = pCurCP->rightVecLinear.j;
		curCPRVec.m_k = pCurCP->rightVecLinear.k;
		
		nexCPRVec.m_i = pNexCP->rightVecLinear.i;
		nexCPRVec.m_j = pNexCP->rightVecLinear.j;
		nexCPRVec.m_k = pNexCP->rightVecLinear.k;
		
		// Calculate the total offset of the point 
		//	from the road segment
		/*
		int laneNo = m_pLane->laneNo;
        CRoadPos* const localThis = const_cast<CRoadPos* const>(this);
        localThis->m_pLane = BindLane(pCurCP->laneIdx + laneNo);
		*/
		if (useoffset)
		{
			if ( ePOS == m_pLane->direction )
				offset = m_ofs + m_pLane->offset;
			else
				offset = -m_ofs + m_pLane->offset;
		}
		else
		{
			offset = m_pLane->offset;
		}

		// Find t
		t = (m_dist - pCurCP->cummulativeLinDist) / 
			/*(pCurCP->distToNextLinear)*/ 1;

		relT = t/(pCurCP->distToNextLinear);
	} // If the current point is on a road

	// If the current point is on an intersection
	else {
		vector<TCdo>::const_iterator cdo = m_cdo.begin();

		TCrdrPnt* pCurCP = BindCrdrPnt(cdo->cntrlPntIdx);
		TCrdrPnt* pNexCP = pCurCP+1;
	
		curCPLoc.m_x = pCurCP->location.x;
		curCPLoc.m_y = pCurCP->location.y;
		curCPLoc.m_z = 0.0;

		nexCPLoc.m_x = pNexCP->location.x;
		nexCPLoc.m_y = pNexCP->location.y;
		nexCPLoc.m_z = 0.0;

		curCPRVec.m_i = pCurCP->rightVecLinear.i;
		curCPRVec.m_j = pCurCP->rightVecLinear.j;
		curCPRVec.m_k = 0.0;
		
		nexCPRVec.m_i = pNexCP->rightVecLinear.i;
		nexCPRVec.m_j = pNexCP->rightVecLinear.j;
		nexCPRVec.m_k = 0.0;
		
		// Offset is simply the offset from the
		//	center of the corridor
		if (useoffset)
			offset = cdo->ofs;
		else
			offset = 0.0;
		
		// Find t
		t = (cdo->dist - pCurCP->distance) /
			/*(pNexCP->distance - pCurCP->distance)*/1;
		relT = (cdo->dist - pCurCP->distance) /
    			(pNexCP->distance - pCurCP->distance);

	} // If the current point is on an intersection


	// Find right vector at t.
	rT.m_i = (1-relT)*curCPRVec.m_i + relT*nexCPRVec.m_i;
	rT.m_j = (1-relT)*curCPRVec.m_j + relT*nexCPRVec.m_j;
	rT.m_k = (1-relT)*curCPRVec.m_k + relT*nexCPRVec.m_k;
	rT.Normalize(); 
	
	// Find the point on the road segment at t
	if ( m_isRoad ) {
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		double tt = t * t;
		double ttt = tt * t;
#ifdef _DEBUG
        cvTSplCoef tempcv; //for the debugger
#endif
		pT.m_x = ttt * pCurCP->hermite[0].A + tt * pCurCP->hermite[0].B +
					t * pCurCP->hermite[0].C + pCurCP->hermite[0].D;
		pT.m_y = ttt * pCurCP->hermite[1].A + tt * pCurCP->hermite[1].B +
					t * pCurCP->hermite[1].C + pCurCP->hermite[1].D;
		pT.m_z = ttt * pCurCP->hermite[2].A + tt * pCurCP->hermite[2].B +
					t * pCurCP->hermite[2].C + pCurCP->hermite[2].D;
	}
	else {
		pT.m_x = (1-relT)*curCPLoc.m_x + relT*nexCPLoc.m_x;
		pT.m_y = (1-relT)*curCPLoc.m_y + relT*nexCPLoc.m_y;
		pT.m_z = 0.0;
	}

	// The answer is pT + offset*(ortho)
	result.m_x = pT.m_x + offset*rT.m_i;
	result.m_y = pT.m_y + offset*rT.m_j;
	result.m_z = pT.m_z + offset*rT.m_k;

	if (!m_isRoad) {
        if (!m_pIntrsctn->elevMap){
            result.m_z =  m_pIntrsctn->elevation;
        }else{
			const CTerrainGrid<Post>* pGrid = NULL;
            pGrid = GetIntrsctnGrid(m_pIntrsctn->myId);
            Post gridOut;
            if (pGrid && pGrid->QueryElev(result, gridOut) )
                result.m_z = gridOut.z;
            else
                result.m_z = m_pIntrsctn->elevation;
        }
	}

	return result;

} // end of GetVeryBestXYZ
//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Returns (x,y,z) that corresponds to current road position
///\remark
///     If the CRoadPos is invalid, this function will cause a failed
/// 	assertion.
///
/// 	This function reverses what was done with the SetXYZ function.  If the 
/// 	CRoadPos was created with a CPoint3D or if SetXYZ determined the current 
/// 	position, then GetXYZ is guaranteed to return the same point that was used.
///
/// 	The road segment is assumed to be a straight line, and the resulting
/// 	point is along the vector perpendicular to that line.
///
///\return The 3D point corresponding to the current road position.
///
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CRoadPos::GetXYZ() const 
{
	AssertValid();
	if (!IsValid()){
		return CPoint3D(0,0,0);
	}
	
	double dx, dy;
	CPoint3D curCPLoc, nexCPLoc;
	CVector3D ortho;
	double t;
	CPoint3D pT;
	double offset;
	CPoint3D result;
	
	// If the current point is on a road
	if (m_isRoad) {
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		TCntrlPnt* pNexCP = pCurCP+1;

		curCPLoc.m_x = pCurCP->location.x;
		curCPLoc.m_y = pCurCP->location.y;
		curCPLoc.m_z = pCurCP->location.z;

		nexCPLoc.m_x = pNexCP->location.x;
		nexCPLoc.m_y = pNexCP->location.y;
		nexCPLoc.m_z = pNexCP->location.z;

		// Calculate the total offset of the point from 
		//	the road segment
		//int laneNo = m_pLane->laneNo;
		//CRoadPos* const localThis = const_cast<CRoadPos* const>(this);
		//localThis->m_pLane = BindLane(pCurCP->laneIdx + laneNo);
 		if ( ePOS == m_pLane->direction )
			offset = m_ofs + m_pLane->offset;
		else
			offset = -m_ofs + m_pLane->offset;
		
		// Find t
		t = (m_dist - pCurCP->cummulativeLinDist) / 
			(pCurCP->distToNextLinear);
	
	} // If the current point is on a road
	
	// If the current point is on an intersection
	else {
		vector<TCdo>::const_iterator cdo = m_cdo.begin();
        TCrdrPnt debug;
		TCrdrPnt* pCurCP = BindCrdrPnt(cdo->cntrlPntIdx);
		TCrdrPnt* pNexCP = pCurCP+1;
        TCrdrPnt debugP;

		curCPLoc.m_x = pCurCP->location.x;
		curCPLoc.m_y = pCurCP->location.y;		
        nexCPLoc.m_x = pNexCP->location.x;
		nexCPLoc.m_y = pNexCP->location.y;

		// Offset is simply the offset from the
		//	center of the corridor
		offset = cdo->ofs;

		// Find t
		t = (cdo->dist - pCurCP->distance) /
			(pNexCP->distance - pCurCP->distance);

	} // If the current point is on an intersection

	// Find slope of the line segment 
	//	between the current CP
	//	and the next CP
	dx = curCPLoc.m_x - nexCPLoc.m_x;
	dy = curCPLoc.m_y - nexCPLoc.m_y;

	// Create a vector orthoganal to the 
	//	line segment between the current CP
	//	and the next CP
	ortho.m_i = -dy;
	ortho.m_j = dx;
	ortho.m_k = 0.0;
	ortho.Normalize();

	// Find the point on the road segment at t
	pT.m_x = (1-t)*curCPLoc.m_x + t*nexCPLoc.m_x;
	pT.m_y = (1-t)*curCPLoc.m_y + t*nexCPLoc.m_y;
	pT.m_z = (1-t)*curCPLoc.m_z + t*nexCPLoc.m_z;

	// The answer is pT + offset*(ortho)
	result.m_x = pT.m_x + offset*ortho.m_i;
	result.m_y = pT.m_y + offset*ortho.m_j;
	result.m_z = pT.m_z + offset*ortho.m_k;
    if (!m_isRoad) {
        const CTerrainGrid<Post>* pGrid = NULL;
        if (!m_pIntrsctn->elevMap){
            result.m_z =  m_pIntrsctn->elevation;
        }else{
            pGrid = GetIntrsctnGrid(m_pIntrsctn->myId);
            Post gridOut;
            if (pGrid->QueryElev(result, gridOut) )
                result.m_z = gridOut.z;
            else
                result.m_z = m_pIntrsctn->elevation;
        }
    }
	return result;

} // end of GetXYZ
//////////////////////////////////////////////////////////////////////////////
///
///\brief
/// 	Returns the tangent at the current road pos
///\remark
///     If the CRoadPos is invalid, this function will cause a failed
/// 	assertion.
///
/// 	Uses the current control point to retrieve the linear or cubic 
/// 	tangent of the road at that point.  This function for a road segment
/// 	is interpolated between the two control points.  Unlike GetTangent where 
/// 	the tangent returned by this function is merely the tangent at one control 
/// 	point.
///
///	Note that only the linear tangent is returned if the CRoadPos is on a
///	corridor.  And since no z information is stored in the corridor, the
///	z-value is set to 0.
///
///	Note that only the linear tangent is returned if the CRoadPos is on a
///	corridor.  And since no z information is stored in the corridor, the
///	z-value is set to 0.
///
///\par
/// 	cubic - optional boolean parameter indicating whether the cubic or 
/// 		linear tangent should be returned.  The default value is false, 
/// 		meaning that the linear tangent will be returned.
///
///\return 
///		a CVector3D containing the tangent vector
/// 
//////////////////////////////////////////////////////////////////////////////
CVector3D
CRoadPos::GetTangentInterpolated(bool cubic) const 
{
	AssertValid();
	CVector3D tan1;
	CVector3D tan2;

	if (m_isRoad) {
		TCntrlPnt *cntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		int factor = ((m_pLane->direction == ePOS)? 1 : -1);
		TCntrlPnt* pNexCP = cntrlPnt+1;

		double t = (m_dist - cntrlPnt->cummulativeLinDist) / 
			(cntrlPnt->distToNextLinear);

		if (cubic) {
			tan1.m_i = factor*cntrlPnt->tangVecCubic.i;
			tan1.m_j = factor*cntrlPnt->tangVecCubic.j;
			tan1.m_k = factor*cntrlPnt->tangVecCubic.k;

			tan2.m_i = factor*pNexCP->tangVecCubic.i;
			tan2.m_j = factor*pNexCP->tangVecCubic.j;
			tan2.m_k = factor*pNexCP->tangVecCubic.k;
		}
		else {
			tan1.m_i = factor*cntrlPnt->tangVecLinear.i;
			tan1.m_j = factor*cntrlPnt->tangVecLinear.j;
			tan1.m_k = factor*cntrlPnt->tangVecLinear.k;

			tan2.m_i = factor*pNexCP->tangVecCubic.i;
			tan2.m_j = factor*pNexCP->tangVecCubic.j;
			tan2.m_k = factor*pNexCP->tangVecCubic.k;
		}
		tan1.Scale(1-t);
		tan2.Scale(t);
		tan1 = tan1 + tan2;
        tan1.Normalize();
	}
	else {
		// Since the tangent is not stored in the corridor control 
		// 	point, compute the linear tangent by subtracting the 
		// 	position of the current control point from the position
		// 	of the next control point, and normalize.
		
		const auto &crd = m_cdo.begin();
		
		TCrdrPnt *pCurCP = BindCrdrPnt((crd->cntrlPntIdx));
		TCrdrPnt* pNexCP = pCurCP+1;
		int currOffset = 1;
		double curr_distance =  pNexCP->distance - pCurCP->distance;
		int currCount =  crd->cntrlPntIdx - crd->pCrdr->cntrlPntIdx;
		if (curr_distance < 10 &&  currCount < (int)crd->pCrdr->numCntrlPnt - 1){
			pNexCP++;
			currOffset++;
		}

		tan1.m_i = pNexCP->location.x - pCurCP->location.x;
		tan1.m_j = pNexCP->location.y - pCurCP->location.y;
		tan1.m_k = 0.0;
		//if we are not at the last control point, calculate the tan for the next 
		//CP
		
		if ((int)crd->pCrdr->numCntrlPnt - 1> currCount + currOffset){ 
			TCrdrPnt* pNexCP2 = pNexCP + 1;
			curr_distance =  pNexCP->distance - pCurCP->distance;
			currOffset++;
			if (curr_distance < 10 &&  currCount + currOffset < (int)crd->pCrdr->numCntrlPnt - 1){
				pNexCP2++;
				currOffset++;
			}

			double t = (crd->dist - pCurCP->distance) / 
				(pNexCP->distance - pCurCP->distance);

			

			tan2.m_i = pNexCP2->location.x - pNexCP->location.x;
			tan2.m_j = pNexCP2->location.y - pNexCP->location.y;
			tan2.m_k = 0.0;

			tan1.Scale(1-t);
			tan2.Scale(t);
			tan1 = tan1 + tan2;
		}
		tan1.Normalize();

	}

	return tan1;

} // end of GetTangent
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetTangent
// 	Returns the tangent at the current road pos
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
// 	Uses the current control point to retrieve the linear or cubic 
// 	tangent of the road at that point.  Note that the real tangent of the 
// 	road segment is interpolated between the two control points.  The 
// 	tangent returned by this function is merely the tangent at one control 
// 	point.
//
//	Note that only the linear tangent is returned if the CRoadPos is on a
//	corridor.  And since no z information is stored in the corridor, the
//	z-value is set to 0.
//
// Arguments:
//	Note that only the linear tangent is returned if the CRoadPos is on a
//	corridor.  And since no z information is stored in the corridor, the
//	z-value is set to 0.
//
// Argument:
// 	cubic - optional boolean parameter indicating whether the cubic or 
// 		linear tangent should be returned.  The default value is false, 
// 		meaning that the linear tangent will be returned.
//
// Returns: a CVector3D containing the tangent vector
// 
//////////////////////////////////////////////////////////////////////////////
CVector3D
CRoadPos::GetTangent(bool cubic) const 
{
	AssertValid();
	CVector3D tan;

	if (m_isRoad) {
		TCntrlPnt *cntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		int factor = ((m_pLane->direction == ePOS)? 1 : -1);

		if (cubic) {
			tan.m_i = factor*cntrlPnt->tangVecCubic.i;
			tan.m_j = factor*cntrlPnt->tangVecCubic.j;
			tan.m_k = factor*cntrlPnt->tangVecCubic.k;
		}
		else {
			tan.m_i = factor*cntrlPnt->tangVecLinear.i;
			tan.m_j = factor*cntrlPnt->tangVecLinear.j;
			tan.m_k = factor*cntrlPnt->tangVecLinear.k;
		}
	}
	else {
		// Since the tangent is not stored in the corridor control 
		// 	point, compute the linear tangent by subtracting the 
		// 	position of the current control point from the position
		// 	of the next control point, and normalize.
		TCrdrPnt *pCurCP = BindCrdrPnt((m_cdo.begin()->cntrlPntIdx));
		TCrdrPnt* pNexCP = pCurCP+1;

		tan.m_i = pNexCP->location.x - pCurCP->location.x;
		tan.m_j = pNexCP->location.y - pCurCP->location.y;
		tan.m_k = 0.0;
		tan.Normalize();
	}

	return tan;

} // end of GetTangent

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRightVec
// 	Returns the right vector at the current road pos
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
//	Because only the 2D linear right vector is stored in the crdr cntrl point, 
//	the 2D linear right vector will be returned if the CRoadPos lies on an
//	intersection.  The k-coordinate is set to 0.
//
// 	Uses the current control point to retrieve the linear or cubic 
// 	right vec of the road at that point.  Note that the real right vec of 
// 	the road segment is interpolated between the two control points.  The 
// 	right vec returned by this function is merely the right vec at one 
// 	control point.
//
// Arguments:
// 	cubic - optional boolean parameter indicating whether the cubic or 
// 		linear vector should be returned.  The default value is false, 
// 		meaning that the linear vector will be returned.
//
// Returns: a CVector3D containing the right vector
// 
//////////////////////////////////////////////////////////////////////////////
CVector3D
CRoadPos::GetRightVec(bool cubic) const 
{
	AssertValid();
	CVector3D rht;

	if (m_isRoad) {
		TCntrlPnt *cntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		int factor = ((m_pLane->direction == ePOS)? 1 : -1);

		if (cubic) {
			rht.m_i = factor*cntrlPnt->rightVecCubic.i;
			rht.m_j = factor*cntrlPnt->rightVecCubic.j;
			rht.m_k = factor*cntrlPnt->rightVecCubic.k;
		}
		else {
			rht.m_i = factor*cntrlPnt->rightVecLinear.i;
			rht.m_j = factor*cntrlPnt->rightVecLinear.j;
			rht.m_k = factor*cntrlPnt->rightVecLinear.k;
		}
	}
	else {

		TCrdrPnt *cntrlPnt = BindCrdrPnt((m_cdo.begin()->cntrlPntIdx));
		rht.m_i = cntrlPnt->rightVecLinear.i;
		rht.m_j = cntrlPnt->rightVecLinear.j;
		rht.m_k = 0.0;
	}

	return rht;

} // end of GetRightVec
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRightVec
// 	Returns the right vector at the current road pos
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
// 	assertion.
//
//	Because only the 2D linear right vector is stored in the crdr cntrl point, 
//	the 2D linear right vector will be returned if the CRoadPos lies on an
//	intersection.  The k-coordinate is set to 0.
//
// 	Uses the current control point to retrieve the linear or cubic 
// 	right vec of the road at that point.  Note that the real right vec of 
// 	the road segment is interpolated between the two control points.  The 
// 	right vec returned by this function is merely the right vec at one 
// 	control point.
//
// Arguments:
// 	cubic - optional boolean parameter indicating whether the cubic or 
// 		linear vector should be returned.  The default value is false, 
// 		meaning that the linear vector will be returned.
//
// Returns: a CVector3D containing the right vector
// 
//////////////////////////////////////////////////////////////////////////////
CVector3D
CRoadPos::GetRightVecInterpolated(bool cubic) const
{
	AssertValid();
	CVector3D rht1;
    CVector3D rht2;
    CVector3D res;
	if (m_isRoad) {
		TCntrlPnt *cntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
        TCntrlPnt* pNexCP = cntrlPnt+1;
		int factor = ((m_pLane->direction == ePOS)? 1 : -1);

		double t = (m_dist - cntrlPnt->cummulativeLinDist) / 
			(cntrlPnt->distToNextLinear);

		if (cubic) {
			rht1.m_i = factor*cntrlPnt->rightVecCubic.i;
			rht1.m_j = factor*cntrlPnt->rightVecCubic.j;
			rht1.m_k = factor*cntrlPnt->rightVecCubic.k;

			rht2.m_i = factor*pNexCP->rightVecCubic.i;
			rht2.m_j = factor*pNexCP->rightVecCubic.j;
			rht2.m_k = factor*pNexCP->rightVecCubic.k;
		}
		else {
			rht1.m_i = factor*cntrlPnt->rightVecLinear.i;
			rht1.m_j = factor*cntrlPnt->rightVecLinear.j;
			rht1.m_k = factor*cntrlPnt->rightVecLinear.k;

        	rht2.m_i = factor*pNexCP->rightVecLinear.i;
			rht2.m_j = factor*pNexCP->rightVecLinear.j;
			rht2.m_k = factor*pNexCP->rightVecLinear.k;
		}

		rht1.Scale(1-t);
		rht2.Scale(t);
		res = rht1 + rht2;
        res.Normalize();
	}
	else {

		TCrdrPnt *cntrlPnt = BindCrdrPnt((m_cdo.begin()->cntrlPntIdx));
        TCrdrPnt* pNexCP = cntrlPnt+1;

		rht1.m_i = cntrlPnt->rightVecLinear.i;
		rht1.m_j = cntrlPnt->rightVecLinear.j;
		rht1.m_k = 0.0;

		rht2.m_i = pNexCP->rightVecLinear.i;
		rht2.m_j = pNexCP->rightVecLinear.j;
		rht2.m_k = 0.0;

		//rht1.Scale(1-t);
		//rht2.Scale(t);
		res = rht1;// + rht2;

	}

	return res;

} // end of GetRightVec
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetElevation
// 	Gets the elevation at the current CRoadPos. 
//
// Remarks: If the current CRoadPos is on a road, the elevation is taken from
// 	the z value of the current control point.  If the current CRoadPos is on
// 	an intersection, the elevation is either flat or stored in a CTerrainGrid
// 	within CCved.
//
// Arguments:
//
// Returns: A double value containing the elevation.
//
//////////////////////////////////////////////////////////////////////////////
double 
CRoadPos::GetElevation() const
{
	double elevation = 0.0;
	if (m_isRoad) {
		TCntrlPnt* pCntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		elevation = pCntrlPnt->location.z;
	}
	else {
		const CTerrainGrid<Post>* pGrid = 
			GetIntrsctnGrid(m_pIntrsctn->myId);
		if (pGrid) {
			Post gridOut;
			CPoint3D point3d = GetBestXYZ();
			if (pGrid->QueryElev(point3d, gridOut) )
				elevation = gridOut.z;
		}
		else
			elevation = m_pIntrsctn->elevation;
	}

	return elevation;
}// end of GetElevation

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetGrade
// 	Gets the grade of the road at this point. 
//
// Remarks: 
//
// Arguments: bool bInDegrees    if true, returns grade in degrees
//                               if false, returns grade in radians
//
// Returns: A double value containing the grade in specified units.
//
//////////////////////////////////////////////////////////////////////////////
double CRoadPos::GetGrade(bool bInDegrees/*=FALSE*/) const
{
	CVector3D tangent = GetTangent(true);
	double length = tangent.Length();
	double gradeRadians = asin(tangent.m_k/length);
	if( !bInDegrees )
		return gradeRadians;
	return (180*gradeRadians)/3.14; //convert to degrees
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLateralDistance
//	Returns the lateral distance between the current CRoadPos and the 
//	parameter.
//
// Remarks: If the current roadpos and the parameter do not lie on the same 
//	road, or if either is invalid, this function will cause a failed assertion.
//
// Arguments:
//	roadPos - a valid CRoadPos instance that lies on the same road as the 
//		current CRoadPos.
//
// Returns: The difference between the offset of the parameter and the offset
//	of the current lane.  A positive value indicates that the parameter lies
//	in the direction of the right vector with respect to the current roadPos, 
//	and a negative value indicates that the parameter lies in the opposite 
//	direction of the right vector with respect to the current roadPos.
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetLateralDistance(const CRoadPos& roadPos) const
{
	AssertValid();
	roadPos.AssertValid();
	assert(m_isRoad && roadPos.m_isRoad);
	assert(m_pLane->roadId == roadPos.m_pLane->roadId);

	TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);

	/*
	int laneNo = m_pLane->laneNo;
	CRoadPos* const localThis = const_cast<CRoadPos* const>(this);
	localThis->m_pLane = BindLane(pCurCP->laneIdx + laneNo);

	CRoadPos* const localRoadPos = const_cast<CRoadPos* const>(&roadPos);
	pCurCP = BindCntrlPnt(roadPos.m_cntrlPntIdx);
	laneNo = roadPos.m_pLane->laneNo;
	localRoadPos->m_pLane = BindLane(pCurCP->laneIdx + laneNo);
	*/
	/*	
	return ( (roadPos.m_pLane->offset+roadPos.m_ofs) - 
			 (m_pLane->offset+m_ofs) );
	*/
	
	double dist1 = 0;
	double dist2 = 0;

	if ( ePOS == roadPos.m_pLane->direction )
		dist1 = roadPos.m_pLane->offset + roadPos.m_ofs;
	else
		dist1 = roadPos.m_pLane->offset - roadPos.m_ofs;

	if ( ePOS == m_pLane->direction )
		dist2 = m_pLane->offset + m_ofs;
	else
		dist2 = m_pLane->offset - m_ofs;

	return (dist1 - dist2);

} // end of GetLateralDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLateralDistance
//	Returns the lateral distance between the current CRoadPos and the 
//	center of the parameter.
//
// Remarks: If the current roadpos and the parameter do not lie on the same 
//	road, or if either is invalid, this function will cause a failed assertion.
//
// Arguments:
//	lane - a valid CLane instance that lies on the same road as the current 
//		CRoadPos.
//
// Returns: The difference between the offset of the parameter and the offset
//	of the current lane.  A positive value indicates that the parameter lies
//	in the direction of the right vector with respect to the current roadPos, 
//	and a negative value indicates that the parameter lies in the opposite 
//	direction of the right vector with respect to the current roadPos.
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetLateralDistance(const CLane& lane) const
{
	AssertValid();
	assert(m_isRoad);
	CLane curLane = GetLane();
	
	double retValue = 0;
	if ( ePOS == m_pLane->direction )
		retValue = curLane.GetLateralDistance(lane) - m_ofs;
	else
		retValue = curLane.GetLateralDistance(lane) + m_ofs;
	return retValue;
} // end of GetLateralDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoadPosInOppositeDir
//	Returns the given road position on a lane in the opposite direction 
//  if one exists.  It randomly picks the lane of the road position if more 
//  than one opposite direction exists.
//
// Remarks: If the current roadpos and the parameter do not lie on the same 
//	road, or if either is invalid, this function will cause a failed assertion.
//
// Arguments:
//	oppositeRoadPos - (output) the roadPos in the opposite direction.
//
// Returns: True if a roadPos in the opposite direction exists
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::GetRoadPosInOppositeDir( CRoadPos& oppositeRoadPos ) const
{
	AssertValid();
	assert(m_isRoad);

	// get all lanes in the opposite direction
	CLane curLane = GetLane();
	vector<CLane> oppositeLanes;
	curLane.GetOppositeLane( oppositeLanes );

	if( oppositeLanes.size() > 0 )
	{
		// randomly select the lane if more than one lane was returned
		int index;
		if( oppositeLanes.size() == 1 )
		{
			index = 0;
		}
		else // size > 1 
		{
			if( m_pRng )
			{
				// have a pointer to random number generator
				index = (int)m_pRng->RandomLongRange( 0, (long)(oppositeLanes.size() - 1) , m_rngStreamId );
			}
			else
			{
				// generate a number from 0 to opppositeLanes.size() - 1
				index = rand() % oppositeLanes.size();
			}
		}

		assert( index >= 0 && index < oppositeLanes.size() );

		CLane oppositeLane = oppositeLanes[index];
		oppositeRoadPos = *this;
		oppositeRoadPos.SetLane( oppositeLane );
		return oppositeRoadPos.IsValid();
	}

	return false;
} // end of GetRoadPosInOppositeDir

//////////////////////////////////////////////////////////////////////////////
// 
// Description: GetCurvature
// 	Returns the radius of curvature at the current road/corridor position.
//
// Remarks: The curvature returned is the stored value at the control point
// 	previous to the current position.
//
// 	If this function is called on an invalid CRoadPos instance, a failed
// 	assertion will occur.
//
// Arguments: 
//
// Returns: the double radius of curvature from 0 to cCV_INFINITY.
// 	
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetCurvature(void) const
{
	AssertValid();
	
	// If the current instance lies on a road
	if (m_isRoad) {

		TCntrlPnt* pCntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		return fabs(pCntrlPnt->radius);
	}

	// Else it lies on an intersection
	else {
		TCrdrPnt* pCntrlPnt = BindCrdrPnt(m_cdo.begin()->cntrlPntIdx);
		return fabs(pCntrlPnt->radius);
	}
	
} // end of GetCurvature
double	CRoadPos::GetCurvatureInterpolated(void) const{
	AssertValid();

	if (m_isRoad) {
		TCntrlPnt *cntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		int factor = ((m_pLane->direction == ePOS)? 1 : -1);
		TCntrlPnt* pNexCP = cntrlPnt+1;

		double t = (m_dist - cntrlPnt->cummulativeLinDist) / 
			(cntrlPnt->distToNextLinear);

		TCntrlPnt* pCntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		double current = fabs(pCntrlPnt->radius);
		double next = fabs(pNexCP->radius);
		return (1-t)*current + next*(t);
	}
	// Else it lies on an intersection
	else {
		TCrdrPnt* pCntrlPnt = BindCrdrPnt(m_cdo.begin()->cntrlPntIdx);
		return fabs(pCntrlPnt->radius);
	}
}
//////////////////////////////////////////////////////////////////////////////
// 
// Description: GetExpandedCurvature
// 	Returns the expanded radius of curvature at the current road/corridor 
//  position.
//
// Remarks: The curvature returned is the stored value at the control point
// 	previous to the current position.
//
// 	If this function is called on an invalid CRoadPos instance, a failed
// 	assertion will occur.
//
// Arguments: 
//
// Returns: the expanded double radius of curvature from -cCV_INFINITY to 
//  cCV_INFINITY. A negative value indicates the road/corridor is curving
//  to the left, while a positive value indicates a curve to the right.
// 	
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetExpandedCurvature(void) const
{
	AssertValid();
	
	// If the current instance lies on a road
	if (m_isRoad) {

		TCntrlPnt* pCntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		return pCntrlPnt->radius;
	}
	// Else it lies on an intersection
	else {
		TCrdrPnt* pCntrlPnt = BindCrdrPnt(m_cdo.begin()->cntrlPntIdx);
		return pCntrlPnt->radius;
	}
	
} // end of GetExpandedCurvature
//////////////////////////////////////////////////////////////////////////////
///\brief	
///		Finds the current road marking for the current position
///\remark 
///		This function looks for the cCV_ROAD_ATTR_LSTYLE, or cCV_CRD_ATTR_LSTYLE
///		and returns the the corrispoding values with the current roadpos
///		currently it uses the first crd
///
///\param	leftMarking  (OUT)	-marking to the left of the driver
///\param	rightMarking (OUT)	-marking to the left of the driver
///\param	leftAtrib    (OUT)	-marking to the left of the driver
///\param	rightAtrib   (OUT)	-marking to the left of the driver
//////////////////////////////////////////////////////////////////////////////

void CRoadPos::GetRoadMarking(int &leftMarking, int &rightMarking, int &rightAtrib, int &leftAtrib) const{
	CVED::CAttr attrs; //attributes
	if (!m_isRoad){
		if (m_cdo.size() < 1)
			return; //we have a problem
		CVED::CCrdr myCrd( GetCved(),m_cdo[0].pCrdr);
		if (!myCrd.IsValid())
			return;
		vector<CAttr> attrs;
		myCrd.QryAttr(attrs);
		for (unsigned int i=0; i < attrs.size(); i++){
			if (attrs[i].GetId() == cCV_ATTR_LSTYLE || attrs[i].GetId() == cCV_ATTR_RSTYLE){
				float start =(float)attrs[i].GetFrom();
				float end  = (float)attrs[i].GetTo();
				if ((start < m_cdo[0].dist || start < 0) &&
					(end   > m_cdo[0].dist || end < 0) ){
						leftMarking = int(attrs[i].GetVal1())/100;
						rightMarking= int(attrs[i].GetVal1())%100;
						rightAtrib = 0;
						leftAtrib = 0;
						return;
				}
			}
		}
		leftMarking = -1;
		rightMarking= -1;
		rightAtrib = 0;
		leftAtrib = 0;
	}
	else{
		CAttr attr;
		
		if ( m_pLane && ( ( CRoadPos::GetAttribute(cCV_ATTR_LSTYLE,attr)
						    && attr.GetId() == cCV_ATTR_LSTYLE )
					   || ( CRoadPos::GetAttribute(cCV_ATTR_RSTYLE,attr)
						    && attr.GetId() == cCV_ATTR_RSTYLE ) 
						)
			){
			//if we are going in the negitive dirrection, right and left are flipped
			if (m_pLane->direction == eNEG){
				rightMarking = int(attr.GetVal1())/100;
				leftMarking= int(attr.GetVal1())%100;
			}
			else{
				leftMarking = int(attr.GetVal1())/100;
				rightMarking= int(attr.GetVal1())%100;
			}
			rightAtrib = 0;
			leftAtrib = 0;
			return;
		}
		leftMarking = -1;
		rightMarking= -1;
		rightAtrib = 0;
		leftAtrib = 0;
	}
	return;
	
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: operator==
// 	Comparsison operator returns true if the parameter is the same as the 
// 	current instance
//
// Remarks:
//
// Arguments:
// 	cRhs - right-hand side of the comparison
//
// Returns: true or false
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::operator==(const CRoadPos& cRhs) const
{
	AssertValid();
	cRhs.AssertValid();

	// If they're both on a road 
	if ( (m_isRoad) && (cRhs.m_isRoad) ) {

		// If the roads match
		if (m_pRoad->myId == cRhs.m_pRoad->myId) {

			// If any lanes match 
			if (m_pLane->myId == cRhs.m_pLane->myId) {

				if ( (fabs(m_dist - cRhs.m_dist) < cCV_ZERO) &&
					 (fabs(m_ofs - cRhs.m_ofs) < cCV_ZERO) )
					return true;

			} // If any lanes match

		} // If the roads match

	} // If they're both on a road

	// Else if they're both on an intersection
	else if ( (!m_isRoad) && (!cRhs.m_isRoad) ) {

		// If the intersections match
		if (m_pIntrsctn->myId == cRhs.m_pIntrsctn->myId) {

			// For each corridor in the vector
			vector<TCdo>::const_iterator itrC, itrP;
			for (itrC = m_cdo.begin(), itrP = cRhs.m_cdo.begin();
				 ( (itrC != m_cdo.end()) && (itrP != cRhs.m_cdo.end()) );
				 itrC++, itrP++) {

				if ( (itrC->pCrdr->myId != itrP->pCrdr->myId) ||
					 (fabs(itrC->dist - itrP->dist) > cCV_ZERO) ||
					 (fabs(itrC->ofs - itrP->ofs) > cCV_ZERO) )
					return false;

			} // For each corridor in the vector
			return true;

		} // If the intersections match

	} // Else if they're both on an intersection

	// If control makes it here, they're not the same.
	return false;

} // end of operator==


//////////////////////////////////////////////////////////////////////////////
//		Mutators
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetRoad
// 	Sets the road and lane to the parameters.  The offset and distance remain 
// 	unchanged.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	or either of the parameters is invalid 
//
// 	If the current CRoadPos contained an intersection, then it is modified
// 	to contain a road.
//
//	If the lane parameter is on the road parameter, then the local road
//	and lane variables are changed, regardless of the distance.
//
// Arguments: 
// 	cRoad - a valid CRoad instance
// 	cLane - a valid CLane instance
//
// Returns: True if the lane parameter is on the road parameter and the 
// 	current distance is within the bounds of the road.  False otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::SetRoad(const CRoad& cRoad, const CLane& cLane) 
{
	CCvedItem::AssertValid();

	//int laneId = cLane.GetId();
	//int laneIdx = cRoad.GetLaneIdx();

	if ( cRoad.GetId() == cLane.GetRoad().GetId()){ 
		//if ( (laneId >= laneIdx) &&
		//		 (laneId < (laneIdx + cRoad.GetNumLanes())) ) {
		if (cLane.GetRelativeId() < cRoad.GetNumLanes()){

			m_isRoad = true;
			m_cdo.clear();

			m_pRoad = BindRoad(cRoad.GetId());
			m_pLane = BindLane(cLane.GetRelativeId());
			m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

			TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
			m_pLane = BindLane(cLane.GetRelativeId() + pCurCP->laneIdx ); //GetRID_


			if (m_dist > cRoad.GetLinearLength())
				return false;

			else
				return true;
		}
	}
	return false;
} // end of SetRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetLane
// 	Sets the lane to the parameter.  The offset and distance remain unchanged.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	or either of the parameters is invalid 
//
//	If the lane parameter is not on the current road, then the current
//	road is changed to the road that the lane lies on.   As with SetRoad, 
//	if the current distance is outside the bounds of the new road, the
//	current road and lane are still changed.
//
// 	If the current CRoadPos contained an intersection, then it is modified
// 	to contain a road.
//
// Arguments: 
// 	cLane - a valid CLane instance
//
// Returns: 
// 	True if the current distance is within the bounds of the road. 
// 	False otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::SetLane(const CLane& cLane) 
{
	return(SetRoad(cLane.GetRoad(), cLane));
} // end of SetLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetIntrsctn
// 	Sets the intersection and corridor to the parameters.  The distance and
// 	offset are set to the parameters.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	or either of the parameters is invalid 
//
// 	If the current CRoadPos contained a road, then it is modified to contain
// 	an intersection.
//
//	If the crdr parameter is on the intrsctn parameter, then the local crdr
//	and intrsctn variables are changed, regardless of the distance.
//
// Arguments: 
// 	cIntrsctn - a valid CIntrsctn instance
// 	cCrdr - a valid CCrdr instance
//	dist - (optional) Distance along the corridor.  Default value is 0.0.
//	offset - (optional) Offset from center of the corridor.  Default 
//		value is 0.0.
//
// Returns: True if the crdr parameter is on the intrsctn parameter, 
// 	false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::SetIntrsctn(const CIntrsctn& cIntrsctn, 
					  const CCrdr& cCrdr, 
					  double dist, double offset) 
{
	CCvedItem::AssertValid();

	int crdrId = cCrdr.GetId();
	int crdrIdx = cIntrsctn.GetCrdrIdx();

	if ( (crdrId >= crdrIdx) &&
		 (crdrId < (crdrIdx + cIntrsctn.GetNumCrdrs())) ) {

		m_isRoad = false;
		m_cdo.clear();

		m_pIntrsctn = BindIntrsctn(cIntrsctn.GetId());

		TCdo tmpCdo;
		tmpCdo.pCrdr = BindCrdr(cCrdr.GetId());
		tmpCdo.dist = dist;
		tmpCdo.ofs = offset;
		tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr) + 
			tmpCdo.pCrdr->cntrlPntIdx;

		m_cdo.push_back(tmpCdo);

		return true;
	}
	else
		return false;
} // end of SetIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetCorridor
// 	Sets the crdr to the parameter.  The offset and distance remain unchanged.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	or either of the parameters is invalid 
//
//	If the crdr parameter is not on the current intrsctn, then the current
//	intrsctn is changed to the intrsctn that the crdr lies on.   As with 
//	
//	SetIntrsctn, if the current distance is outside the bounds of the new 
//	intersection, the current intrsctn and crdr are still changed.
//
// 	If the current CRoadPos contained an intersection, then it is modified
// 	to contain a intrsctn.
//
// Arguments: 
// 	cCrdr - a valid CCrdr instance
//	dist - (optional) Distance along the corridor.  Default value is 0.0.
//	offset - (optional) Offset from center of the corridor.  Default 
//
// Returns: 
// 	True if the current distance is within the bounds of the intrsctn. 
// 	False otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::SetCorridor(const CCrdr& cCrdr, 
					  double dist, double offset) 
{
	return SetIntrsctn(cCrdr.GetIntrsctn(), cCrdr, dist, offset);
} // end of SetCorridor

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetCorridors
// 	Sets the intersection and corridors to the parameters.  The distances are
// 	set to the corresponding distance parameter values.  The offsets remain
// 	unchanged.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	or either of the parameters is invalid 
//
// 	If the current CRoadPos contained a road, then it is modified to contain
// 	an intersection.
//
// Arguments: 
// 	cIntrsctn - a valid CIntrsctn instance
// 	crdrs - a bitset containing corridor ids, with respect to the cIntrsctn
// 	distances - an array of distances along each corridor
//
// Returns: True.  There's a return value to maintain consistancy with the
// 	other Set* functions.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::SetCorridors(
			const CIntrsctn& cIntrsctn, 
			const bitset<cCV_MAX_CRDRS>& crdrs, 
			const double distances[cCV_MAX_CRDRS], 
			double offset
			) 
{
	CCvedItem::AssertValid();

	int crdrId;
	int crdrIdx = cIntrsctn.GetCrdrIdx();
	int numCrdrs = cIntrsctn.GetNumCrdrs();
	TCrdr*	pCrdr = BindCrdr(crdrIdx);
	TCdo tmpCdo;

	// Set the m_isRoad flag to false
	m_isRoad = false;

	// Set local intrsctn field to parameter
	m_pIntrsctn = BindIntrsctn(cIntrsctn.GetId());

	// First empty out any previous cdo info
	m_cdo.clear();

	for (crdrId = 0; crdrId < numCrdrs; crdrId++, pCrdr++) {

		// If this bit is set, add this corridor.
		if (crdrs.test(crdrId)) {

			tmpCdo.pCrdr = pCrdr;
			tmpCdo.dist = distances[crdrId];
			tmpCdo.ofs = offset;
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr) + 
				tmpCdo.pCrdr->cntrlPntIdx;

			m_cdo.push_back(tmpCdo);
		}
	}
	return true;
} // end of SetCorridors

//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the distance component.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	 is invalid.
//
// Arguments: 
// 	 dist - The value to set distance to.
//
// Returns: If the current CRoadPos is on a road and the new distance 
//   keeps it within the road, then the function will return eWITHIN_ROAD.
//   If the	current CRoadPos is on an intersection and the new distance 
//   keeps it within the corridor, then the function will return 
//   eWITHIN_CRDR.  Otherwise, the function returns ePAST_ROAD.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::SetDistance( double dist ) 
{
	AssertValid();

	if (m_isRoad) {
		// Find the control point associated with the 
		// 	given distance along the road.
		m_dist = dist;
		m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		int laneNo = m_pLane->laneNo;
		m_pLane = BindLane(pCurCP->laneIdx + laneNo);
	
		if ( (m_dist<0) || (m_dist>m_pRoad->roadLengthLinear) )
			return ePAST_ROAD;
		else
			return eWITHIN_ROAD;
	}
	else {
		// Find the control point associated with the 
		// 	given distance along the corridor.
		vector<TCdo>::iterator cdo = m_cdo.begin();

		cdo->dist = dist;
		cdo->cntrlPntIdx = Search(dist) + 
			cdo->pCrdr->cntrlPntIdx;

		TCrdrPnt* pLastCrdrPnt = BindCrdrPnt(cdo->pCrdr->cntrlPntIdx +
											 cdo->pCrdr->numCntrlPnt -
											 1);
		if ( (dist < 0) || (dist > pLastCrdrPnt->distance) )
			return ePAST_ROAD;
		else
			return eWITHIN_CRDR;
	}
} // end of SetDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: Increments the distance component based on the direction 
//   of the lane or corridor the current CRoadPos lies on.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	 is invalid 
//
// Arguments: 
// 	 dist - The value to increment distance by.
//
// Returns: If the current CRoadPos is on a road and the new distance 
//   keeps it within the road, then the function will return eWITHIN_ROAD.  
//   If the	current CRoadPos is on an intersection and the new distance 
//   keeps it within the corridor, then the function will return 
//   eWITHIN_CRDR.  Otherwise, the function returns ePAST_ROAD.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::IncrDistance( double dist ) 
{
	AssertValid();

	if (m_isRoad) {
		if (m_pLane->direction == ePOS)
			m_dist += dist;
		else
			m_dist -= dist;

		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = m_pRoad->cntrlPntIdx + Search(m_dist);
	
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		int laneNo = m_pLane->laneNo;
		m_pLane = BindLane(pCurCP->laneIdx + laneNo);
	
		if ( (m_dist<0) || (m_dist>m_pRoad->roadLengthLinear) )
			return ePAST_ROAD;
		else
			return eWITHIN_ROAD;
	}
	else {
		vector<TCdo>::iterator cdo = m_cdo.begin();

		cdo->dist += dist;
		cdo->cntrlPntIdx = Search(cdo->dist) + 
			cdo->pCrdr->cntrlPntIdx;

		// Find the control point associated with the 
		// 	given distance along the corridor.
		TCrdrPnt* pLastCrdrPnt = BindCrdrPnt(cdo->pCrdr->cntrlPntIdx +
											 cdo->pCrdr->numCntrlPnt -
											 1);
		if ( (cdo->dist<0) || (cdo->dist>pLastCrdrPnt->distance) )
			return ePAST_ROAD;
		else
			return eWITHIN_CRDR;
	}
} // end of IncrDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: Decrements the distance component based on the direction 
//   of the lane or corridor the current CRoadPos lies on.
//
// Remarks: This function will throw an exception if the current CRoadPos
// 	 is invalid 
//
// Arguments: 
// 	 dist - The value to increment distance by.
//
// Returns: If the current CRoadPos is on a road and the new distance 
//   keeps it within the road, then the function will return eWITHIN_ROAD.  
//   If the	current CRoadPos is on an intersection and the new distance 
//   keeps it within the corridor, then the function will return 
//   eWITHIN_CRDR.  Otherwise, the function returns ePAST_ROAD.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::DecrDistance( double dist ) 
{
	AssertValid();

	if( m_isRoad ) 
	{
		if( m_pLane->direction == ePOS )
			m_dist -= dist;
		else
			m_dist += dist;

		// Find the control point associated with the given distance along the road.
		m_cntrlPntIdx = m_pRoad->cntrlPntIdx + Search(m_dist);
	
		TCntrlPnt* pCurCP = BindCntrlPnt( m_cntrlPntIdx );
		int laneNo = m_pLane->laneNo;
		m_pLane = BindLane( pCurCP->laneIdx + laneNo );
	
		if( (m_dist<0) || (m_dist>m_pRoad->roadLengthLinear) )
			return ePAST_ROAD;
		else
			return eWITHIN_ROAD;
	}
	else 
	{
		vector<TCdo>::iterator cdo = m_cdo.begin();

		cdo->dist -= dist;
		cdo->cntrlPntIdx = cdo->pCrdr->cntrlPntIdx + Search(cdo->dist);

		// Find the control point associated with the given distance along the corridor.
		TCrdrPnt* pLastCrdrPnt = BindCrdrPnt(cdo->pCrdr->cntrlPntIdx + cdo->pCrdr->numCntrlPnt - 1);
		if ( (cdo->dist<0) || (cdo->dist>pLastCrdrPnt->distance) )
			return ePAST_ROAD;
		else
			return eWITHIN_CRDR;
	}
} // end of IncrDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set the offset component.
//
// Remarks: If the current position is on an intersection, this function
//   sets the variables that keep offsets for both intersections and roads.
//   If the CRoadPos is invalid, this function will cause a failed
// 	 assertion.
//
// Arguments:
// 	 offset - Offset to set the current component to.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CRoadPos::SetOffset( double offset )
{

	AssertValid();

	m_ofs = offset;
	if ( !m_isRoad ) {

		//
		// Set all of the corridors' offsets.
		//
		vector<TCdo>::iterator i;
		for ( i = m_cdo.begin(); i != m_cdo.end(); i++ ) {
			(m_cdo.begin())->ofs = offset;
		}
		
	}

} // end of SetOffset

//////////////////////////////////////////////////////////////////////////////
//
// Description: Changes the lane of the current CRoadPos to the lane 
//   at the left of the current lane.  
//
// Remarks: This function will throw an exception if the current 
//   CRoadPos is invalid.
//
//   The lane is changed with respect to the direction of the current lane.
//
// Arguments:
//		center - denotes whether or not the function should return a point
//			in the center of the lane.
//
// Returns: 
// 	 If the current CRoadPos instance is on a road:
// 		If the current lane is the leftmost lane, then the function returns 
// 		false and no change is made.  Otherwise, the lane is changed and the 
// 		function returns true.
// 	 If the current CRoadPos instance is on an intersection:
//		Nothing is changed and the function returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::ChangeLaneLeft(bool center) 
{
	AssertValid();

	if ( m_isRoad ) {
		CLane lane( GetCved(), m_pLane );
		int diff = 0;

		if ( lane.IsLeftMostAlongDir() )  return false;

		lane = lane.GetLeft();
	
		diff = lane.GetRelativeId() - m_pLane->laneNo; //GetRID_
		m_pLane += diff;
	
		if (center) {
			m_ofs = 0;
		}	

		return true;
	}
	else {
		return false;
	}
} // end of ChangeLaneLeft

//////////////////////////////////////////////////////////////////////////////
//
// Description: Changes the lane of the current CRoadPos to the lane 
//   at the right of the current lane.  
//
// Remarks: This function will throw an exception if the current 
//   CRoadPos is invalid.
//
// 	 The lane is changed with respect to the direction of the current lane.
//
// Arguments:
//
// Returns: 
// 	 If the current CRoadPos instance is on a road:
// 		If the current lane is the leftmost lane, then the function returns 
// 		false and no change is made.  Otherwise, the lane is changed and the 
// 		function returns true.
// 	 If the current CRoadPos instance is on an intersection:
//		Nothing is changed and the function returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::ChangeLaneRight(bool center) 
{
	AssertValid();

	if ( m_isRoad ) {
		CLane lane( GetCved(), m_pLane );
		int diff = 0;
	
		if ( lane.IsRightMost() )  return false;
	
		lane = lane.GetRight();
	
		diff = lane.GetRelativeId() - m_pLane->laneNo;
		m_pLane += diff;

		if (center) {
			m_ofs = 0;
		}	

		return true;
	}
	else {
		return false;
	}
} // end of ChangeLaneRight

//////////////////////////////////////////////////////////////////////////////
//
// Description: Initializes the current CRoadPos with the string parameter.
//
// Remarks: The parameter should contain a string with the same format 
//   as the string returned by GetString().  The format is XXXX:YYY:ZZZ:WWW 
//   where XXX is the name of the road/intrsctn, YYY is the lane/crdr id 
//   with respect to the road/intrsctn, ZZZ is the distance along the road, 
//   and WWW is the offset from the center of the lane.
//
// Arguments:
// 	 cName - String containing the formatted CRoadPos name.
//
// Returns: false if there are any format errors or if the name is invalid, 
//	 true otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::SetString( const string& cName )
{
	CCvedItem::AssertValid();

	string strName;
	int id;
	int buffSize = (int)strlen(cName.c_str())+1;
	char *pBuf = new char[buffSize];
    char *pTokDel = " \t:";
    char *pTok;
	char *pCurrPos = NULL;
    strcpy_s(pBuf,buffSize, cName.c_str());
    pTok = strtok_s(pBuf, pTokDel,&pCurrPos);
	//if (!pTok){ //if a ADO object is pasted outside of a road it will cause the program to crash DAH
	//	return false;
	//}
	strName = pTok;
    pTok = strtok_s(NULL, pTokDel,&pCurrPos);
    id = atoi(pTok);
    pTok = strtok_s(NULL, pTokDel,&pCurrPos);
    m_dist = atof(pTok);
    pTok = strtok_s(NULL, pTokDel,&pCurrPos);
    m_ofs = atof(pTok);
	delete [] pBuf;

	THeader*	pH   = static_cast<THeader*>(GetInst());
	char* pCharPool	= static_cast<char*>(GetInst()) + pH->charOfs;
	unsigned int i;

	m_isRoad = true;
	for (i = 1; i < pH->roadCount; i++){
		m_pRoad = BindRoad(i);
		if ( strName == (pCharPool + m_pRoad->nameIdx) )
			break;
	}

	// If there's no road with this name, then try the intrsctn pool.
	if (i == pH->roadCount) {
		m_pRoad = 0;
		m_pLane = 0;
		m_isRoad = false;
	
		for (i = 1; i < pH->intrsctnCount; i++){
			m_pIntrsctn = BindIntrsctn(i);
			if ( strName == (pCharPool + m_pIntrsctn->nameIdx) )
				break;
		}
		if (i == pH->intrsctnCount) {
			// There's no road or intrsctn by this name, 
			// 	so return false.
			m_pIntrsctn = 0;
			m_cdo.clear();
			return false;
		}
		// Valid intrsctn found, so set up crdr.
		else {
			TCdo tmpCdo;

			tmpCdo.dist = m_dist;
			tmpCdo.ofs = m_ofs;

			tmpCdo.pCrdr= BindCrdr(m_pIntrsctn->crdrIdx + id); 
	
			// Find the control point associated with the 
			// 	given distance along the road.
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;

			// Make tmpCdo the only member of the m_cdo vector
			m_cdo.clear();
			m_cdo.push_back(tmpCdo);
		}
	} // If there's no road with this name, then try the intrsctn pool.

	// Valid road found, so set up lane.
	else {
		m_pLane = BindLane(m_pRoad->laneIdx + id);
	
		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
        m_pLane = BindLane(pCurCP->laneIdx + id);
	}

	return true;
} // end of SetString

//////////////////////////////////////////////////////////////////////////////
//
// Description: Advance the position on a road by the specified amount.
//
// Remarks: This function calls Travel with its parameter.
//
// Arguments: 
// 	 inc - Amount to advance the current road position.
//
// Returns: one of { eWITHIN_ROAD, eWITHIN_CRDR, ePAST_ROAD } indicating 
// 	 whether the new position is still on the road/crdr, or if it has 
//   passed the end.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::operator+(double inc) 
{
	return Travel(inc);
} // end of operator+

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retreat the position on a road by the specified amount.
//
// Remarks: This function calls Travel with (-1)*parameter.
//
// Arguments: 
// 	 dec - Amount to retreat the current road position.
//
// Returns: one of { eWITHIN_ROAD, eWITHIN_CRDR, ePAST_ROAD } indicating 
// 	 whether the new position is still on the road/crdr, or if it has 
//   passed the end.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::operator-(double dec) 
{
	return Travel(-dec);
} // end of operator-

//////////////////////////////////////////////////////////////////////////////
//
// Description: Return the amount a point has gone over the end of a road.
//
// Remarks: Used to correct the Trigger pasting error
//
// Arguments: None
//
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetRoadLengthLinear() const
{
	if( IsRoad() ) 
	{
		if( m_pLane->direction == ePOS )
		{
			return m_pRoad->roadLengthLinear - m_dist;
		}
		else if( m_pLane->direction == eNEG )
		{
			return m_pRoad->roadLengthLinear + m_dist;
		}
		else
		{
			// this should never happen as roads can either be positive or negative
			gout << "CVED::CRoadPos:GetRoadLenghLinear: unknown condition "
				<< "direction = " << m_pLane->direction << endl;
			return 0.0;
		}
	} 
	else 
	{
		return 0.0;
	}
}

//////////////////////////////////////////////////////////////////////////////
// Description: Travel
///\brief
/// 	Advance the position along the road network by a specified amount.
///\remark
/// This function will throw an exception if the current CRoadPos is 
/// 	invalid.
///
///	This function updates the distance component of the current instance
///	by advancing it along the direction of the lane or corridor by the
///	amount specified in the first parameter.  
///
///	If distance specified moves the current position beyond the road, 
///	and a destination lane was specified, then the next intersection 
///	is scanned for a corridor that connects the current lane with the 
///	lane specified in the parameter.  If a corridor is found, then the 
///	current CRoadPos instance is set to lie on that corridor, with the
///	same offset as before, and a distance that indicates the distance
///	traveled into the corridor.
///
///	If distance specified moves the current position beyond the road, 
///	but no destination lane is given, then ePAST_ROAD is returned, and
///	the current CRoadPos instance remains on the road.
///
///	If the current CRoadPos lies on a corridor, the distance may advance
///	along the corridor or into the connecting lane.
///
/// Arguments:
///\param	dist - the amount by which to change the distance component
///\param	cpDstLane - (optional) used to determine which corridor should 
///                     be selected if the Travel function moves the current
///                     position off the road and into a connecting 
///                     intersection.
///
///\param   eTurnDir -  (optional) superseded by cpDstLane 
///                     used to determine which corridor should 
///                     be selected if the Travel function moves the current
///                     position off the road and into a connecting 
///                     intersection.
/// 
/// Returns:
///	ePAST_ROAD: If the distance traveled moves the current position beyond
///				the bounds of the road, and no adjoining corridor was 
///				specified or found.
///
///	eWITHIN_ROAD: If the distance traveled results in a position on a road.
///
///	eWITHIN_CRDR: If the distance traveled results in a position on a 
///				  corridor.
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::Travel(double dist, const CLane* cpDstLane, const ETravelTurnDir eTurnDir)
{
	AssertValid();

	// If currently on a road
	if (m_isRoad) {

		if (m_pLane->direction == ePOS)
			m_dist += dist;
		else if (m_pLane->direction == eNEG)
			m_dist -= dist;

		//if (m_dist < 0.0) m_dist = 0.0;
	
		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		m_pLane = BindLane(pCurCP->laneIdx + m_pLane->laneNo);
		
		// If travel distance remains within the road
		if ( (m_dist >= 0) && 
			 (m_dist < m_pRoad->roadLengthLinear) ) 
			return eWITHIN_ROAD;
		
		// If Travel() moves into an intersection
		else {
			CLane lane(GetCved(), m_pLane);
			if (!lane.IsValid()){
				return eERROR;
			}

			CIntrsctn intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return eERROR;
			}

			TCrdrVec crdrVec;
			intr.GetCrdrsStartingFrom(lane, crdrVec);
			int index = 0;

			if (crdrVec.size() > 0){
				bool found = false;
				for (unsigned int i = 0; i < crdrVec.size(); i++){
					// look for a corridor following the specified direction
					if (cpDstLane != 0 && crdrVec[i].GetDstntnLn().GetRelativeId() == cpDstLane->GetRelativeId()) {
						index = i;
						break;
					}
					if (index == 0 && crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
						// take straight corridor if no corridor is picked
						index = i;
					}
					if (eTurnDir != eDEFAULT && (eTurnDir == eSTRAIGHT && crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT || 
												 eTurnDir == eRIGHT    && crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT    ||
												 eTurnDir == eLEFT     && crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT   ) ){
						// if a turn direction is specified and the corridor's direction is the same
						index = i;
					}
				}
			} else {
				// intersection has no corridors (should not happen)
				return eERROR;
			}

			// make some pointers to store as member variables so we can continue traveling
			TIntrsctn* pIntrsctn = BindIntrsctn(intr.GetId());
			TCrdr* pCrdr = BindCrdr(crdrVec[index].GetId());

			m_isRoad = false;
			m_pIntrsctn = pIntrsctn;
			m_cdo.clear();
			TCdo tmpCdo;
			tmpCdo.pCrdr = pCrdr;
			tmpCdo.ofs = m_ofs;
			tmpCdo.dist = m_dist > m_pRoad->roadLengthLinear ? m_dist - m_pRoad->roadLengthLinear : -(m_dist);	// change to 0 to start at beginning of intersection
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;
			m_cdo.push_back(tmpCdo);

			// if we overshoot the intersection
			if (tmpCdo.dist > crdrVec[index].GetLength()) {
				Travel(0, cpDstLane, eTurnDir);
			}

			return eWITHIN_CRDR;
		} // If travel distance moved beyond road

	} // If currently on a road

	// If currently on a corridor
	else {

		// Increment the distance on the current corridor
		vector<TCdo>::iterator cdo = m_cdo.begin();

		// tries to handle case when Travel() is supplied a CRoadPos that begins on an intersection (need to find the correct corridor first)
		if (cpDstLane != 0 && m_cdo.size() > 1 || m_cdo.size() == 0){
			TCrdrVec crdrVec;
			CIntrsctn intr = GetIntrsctn();
			if (!intr.IsValid()){
				return eERROR;
			}

			bool hasCorridors = true;

			intr.GetCrdrsLeadingTo(*cpDstLane, crdrVec);
			if (crdrVec.size() == 0 ){
				// intersection has no corridors leading to the destination lane meaning we can no longer use the destination lane parameter to pick a corridor
				hasCorridors = false;
			
				intr.GetAllCrdrs(crdrVec);
				if (crdrVec.size() == 0) {
					return eERROR;
				}
			}

			int index = 0;
			for(unsigned int i=0; i<crdrVec.size(); i++){
				if ( (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT && (eTurnDir == CRoadPos::eSTRAIGHT || eTurnDir == CRoadPos::eDEFAULT)) || 
					 (crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT     &&  eTurnDir == CRoadPos::eLEFT) ||
					 (crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT    &&  eTurnDir == CRoadPos::eRIGHT) ) {
		
					index = i;

					if (hasCorridors && HasCorridor(crdrVec[i].GetRelativeId()) || !hasCorridors) { 
						// if the corridor contains the point or if the destination lane parameter is invalid, we are done
						break;
					}
				}
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					index = i; // take a straight corridor if a corridor with the specified direction cannot be found
				}
			}

			// store corridor data
			m_isRoad = false;
			m_pIntrsctn = BindIntrsctn(intr.GetId());
			TCdo tmpCdo;
			tmpCdo.pCrdr = BindCrdr(crdrVec[index].GetId());
			tmpCdo.ofs = m_ofs;
			tmpCdo.dist = GetDistanceOnLane(crdrVec[index].GetRelativeId());
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;
			m_cdo.clear();
			m_cdo.push_back(tmpCdo);
			cdo = m_cdo.begin();
			// if we overshoot the intersection
			if (tmpCdo.dist > crdrVec[index].GetLength()) {
				Travel(0, cpDstLane, eTurnDir);
			}
		}

		TCrdrPnt *pLastPnt = BindCrdrPnt(cdo->pCrdr->cntrlPntIdx +
										 cdo->pCrdr->numCntrlPnt - 1);
		double length = pLastPnt->distance;

		cdo->dist += dist;

		// get the index of the point; used in GetVeryBestXYZ()
		cdo->cntrlPntIdx = Search(cdo->dist, cdo->pCrdr) + cdo->pCrdr->cntrlPntIdx;

		// If distance is within the corridor
		if ( (cdo->dist >= 0) &&
			 (cdo->dist < length) ){

			return eWITHIN_CRDR;
		}

		// If distance is beyond the corridor
		else {

			// Move current position to the connecting road/lane
			m_isRoad = true;

			// If we've advanced past the beginning of the corridor
			if (cdo->dist < 0) {
				//bind our new lane + road
				m_pRoad = BindRoad(cdo->pCrdr->srcRdIdx);
				m_pLane = BindLane(cdo->pCrdr->srcLnIdx);

				
				if (m_pLane->direction == eNEG){
					m_dist = - cdo->dist;
				} else {
					m_dist = m_pRoad->roadLengthLinear + cdo->dist;
				}

				m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

				TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
				int id = cdo->pCrdr->srcLnIdx - m_pRoad->laneIdx;
				m_pLane = BindLane(pCurCP->laneIdx + id);

			}
			// If we've advanced beyond the end of the corridor
			else {
				m_dist = cdo->dist - length;

				m_pRoad = BindRoad(cdo->pCrdr->dstRdIdx);
				m_pLane = BindLane(cdo->pCrdr->dstLnIdx);

				if (m_pLane->direction == eNEG){
					m_dist = m_pRoad->roadLengthLinear - m_dist;
				}

				m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
				TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
				int id = cdo->pCrdr->dstLnIdx - m_pRoad->laneIdx;
				m_pLane = BindLane(pCurCP->laneIdx + id);
			}

			m_ofs = cdo->ofs;

			// if we overshoot the road
			if (cdo->dist < 0 || cdo->dist >= m_pRoad->roadLengthLinear) {
				Travel(0, cpDstLane, eTurnDir);
			}

			return eWITHIN_ROAD;
				
		} // If distance is beyond the corridor

	} // If currently on a corridor

} // end of Travel

//////////////////////////////////////////////////////////////////////////////
// Description: TravelDir
///\brief
/// 	Advance the position along the road network by a specified amount.
///\remark
/// This function will throw an exception if the current CRoadPos is 
/// 	invalid.
///
///	This function updates the distance component of the current instance
///	by advancing it along the direction of the lane or corridor by the
///	amount specified in the first parameter.  
///
/// If the traveler begins on an intersection, then the traveler picks
/// a corridor based on either the lane index parameter or it selects
/// the first corridor that contains its starting position.
///
///	If distance specified moves the current position beyond the road, 
///	but no destination lane is given, then ePAST_ROAD is returned, and
///	the current CRoadPos instance remains on the road.
///
///	If the current CRoadPos lies on a corridor, the distance may advance
///	along the corridor or into the connecting lane.
///
/// Arguments:
///\param	dist - the amount by which to change the distance component
///\param	eTurnDir - (optional) used to determine which corridor should 
///                     be selected if the Travel function moves the current
///                     position off the road and into a connecting 
///                     intersection.
///
///\param   idx -  (optional) the absolute id of the source lane. 
///						Used to determine which corridor to travel along
///						if the travel function is started on an intersection.
/// 
/// Returns:
///	ePAST_ROAD: If the distance traveled moves the current position beyond
///				the bounds of the road, and no adjoining corridor was 
///				specified or found.
///
///	eWITHIN_ROAD: If the distance traveled results in a position on a road.
///
///	eWITHIN_CRDR: If the distance traveled results in a position on a 
///				  corridor.
///
/// eERROR:	generic error
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::TravelDir(double dist, const ETravelTurnDir eTurnDir, const int idx)
{
	AssertValid();

	// If currently on a road
	if (m_isRoad) {

		if (m_pLane->direction == ePOS)
			m_dist += dist;
		else if (m_pLane->direction == eNEG)
			m_dist -= dist;

		//if (m_dist < 0.0) m_dist = 0.0;
	
		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		m_pLane = BindLane(pCurCP->laneIdx + m_pLane->laneNo);
		
		// If travel distance remains within the road
		if ( (m_dist >= 0) && 
			 (m_dist < m_pRoad->roadLengthLinear) ) 
			return eWITHIN_ROAD;
		
		// If Travel() moves into an intersection
		else {
			CLane lane(GetCved(), m_pLane);
			if (!lane.IsValid()){
				return eERROR;
			}

			CIntrsctn intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return eERROR;
			}

			int crdrId;
			ETravelTurnDir dirOut;
			CCrdr crdr = GetNextCrdr(crdrId, dirOut, eTurnDir, idx);

			// make some pointers to store as member variables so we can continue traveling
			TIntrsctn* pIntrsctn = BindIntrsctn(intr.GetId());
			TCrdr* pCrdr = BindCrdr(crdr.GetId());

			m_isRoad = false;
			m_pIntrsctn = pIntrsctn;
			m_cdo.clear();
			TCdo tmpCdo;
			tmpCdo.pCrdr = pCrdr;
			tmpCdo.ofs = m_ofs;
			tmpCdo.dist = m_dist > m_pRoad->roadLengthLinear ? m_dist - m_pRoad->roadLengthLinear : -(m_dist);	// change to 0 to start at beginning of intersection

			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;
			m_cdo.push_back(tmpCdo);

			// if we overshoot the intersection
			if (tmpCdo.dist > crdr.GetLength()) {
				TravelDir(0, eTurnDir, idx);
			}

			return eWITHIN_CRDR;
		} // If travel distance moved beyond road

	} // If currently on a road

	// If currently on a corridor
	else {

		// Increment the distance on the current corridor
		vector<TCdo>::iterator cdo = m_cdo.begin();

		// tries to handle case when Travel() is supplied a CRoadPos that begins on an intersection (need to find the correct corridor first)
		if (idx != -1 && (idx != cdo->pCrdr->srcLnIdx || cdo->pCrdr->direction != eTurnDir) ){
			TCrdrVec crdrVec;
			CIntrsctn intr = GetIntrsctn();
			if (!intr.IsValid()){
				return eERROR;
			}

			CLane lane (GetCved(), BindLane(idx));
			if (!lane.IsValid()){
				return eERROR;
			}
			
			intr.GetCrdrsStartingFrom(lane, crdrVec);
			if (crdrVec.size() == 0 ){
				// intersection has no corridors, should not happen
				return eERROR;
			}

			int index = 0;
			for(unsigned int i=0; i<crdrVec.size(); i++){
				if ( (crdrVec[i].GetCrdrDirection()    == CCrdr::eSTRAIGHT && (eTurnDir == CRoadPos::eSTRAIGHT || eTurnDir == CRoadPos::eDEFAULT)) || 
						(crdrVec[i].GetCrdrDirection() == CCrdr::eLEFT     &&  eTurnDir == CRoadPos::eLEFT) ||
						(crdrVec[i].GetCrdrDirection() == CCrdr::eRIGHT    &&  eTurnDir == CRoadPos::eRIGHT) ) {
								
					index = i;
					break;
				}
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					index = i; // take a straight corridor if a corridor with the specified direction cannot be found
				}
			}

			// store corridor data
			m_isRoad = false;
			m_pIntrsctn = BindIntrsctn(intr.GetId());
			TCdo tmpCdo;
			tmpCdo.pCrdr = BindCrdr(crdrVec[index].GetId());
			tmpCdo.ofs = m_ofs;
			tmpCdo.dist = GetDistanceOnLane(crdrVec[index].GetRelativeId());
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;
			m_cdo.clear();
			m_cdo.push_back(tmpCdo);
			cdo = m_cdo.begin(); 
			// if we overshoot the intersection, this should never happen
			if (tmpCdo.dist > crdrVec[index].GetLength()) {
				TravelDir(0, eTurnDir, idx);
			}
		}

		TCrdrPnt *pLastPnt = BindCrdrPnt(cdo->pCrdr->cntrlPntIdx +
										 cdo->pCrdr->numCntrlPnt - 1);
		double length = pLastPnt->distance;

		cdo->dist += dist;

		// get the index of the point; used in GetVeryBestXYZ()
		cdo->cntrlPntIdx = Search(cdo->dist, cdo->pCrdr) + cdo->pCrdr->cntrlPntIdx;

		// If distance is within the corridor
		if ( (cdo->dist >= 0) &&
			 (cdo->dist < length) ){

			return eWITHIN_CRDR;
		}

		// If distance is beyond the corridor
		else {

			// Move current position to the connecting road/lane
			m_isRoad = true;

			// If we've advanced past the beginning of the corridor
			if (cdo->dist < 0) {
				//bind our new lane + road
				m_pRoad = BindRoad(cdo->pCrdr->srcRdIdx);
				m_pLane = BindLane(cdo->pCrdr->srcLnIdx);

				
				if (m_pLane->direction == eNEG){
					m_dist = - cdo->dist;
				} else {
					m_dist = m_pRoad->roadLengthLinear + cdo->dist;
				}

				m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

				TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
				int id = cdo->pCrdr->srcLnIdx - m_pRoad->laneIdx;
				m_pLane = BindLane(pCurCP->laneIdx + id);

			}
			// If we've advanced beyond the end of the corridor
			else {
#ifdef _DEBUG
				cvTCrdr tcvtcrd; //for the benifit of the debugger
#endif
				m_dist = cdo->dist - length;

				m_pRoad = BindRoad(cdo->pCrdr->dstRdIdx);
				m_pLane = BindLane(cdo->pCrdr->dstLnIdx);

				if (m_pLane->direction == eNEG){
					m_dist = m_pRoad->roadLengthLinear - m_dist;
				}

				m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
				TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
				int id = cdo->pCrdr->dstLnIdx - m_pRoad->laneIdx;
				m_pLane = BindLane(pCurCP->laneIdx + id);
			}

			m_ofs = cdo->ofs;

			// if we overshoot the road
			if (cdo->dist < 0 || cdo->dist >= m_pRoad->roadLengthLinear) {
				TravelDir(0, eTurnDir, idx);
			}

			//m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
			return eWITHIN_ROAD;
				
		} // If distance is beyond the corridor

	} // If currently on a corridor

} // end of Travel

//////////////////////////////////////////////////////////////////////////////
// Description: TravelCrdr
///\brief
/// 	Advance the position along the road network by a specified amount.
///\remark
/// This function will throw an exception if the current CRoadPos is 
/// 	invalid.
///
///	This function updates the distance component of the current instance
///	by advancing it along the direction of the lane or corridor by the
///	amount specified in the first parameter.  
///
/// If the traveler begins on an intersection, then the traveler picks
/// a corridor based on the corridor ID parameter.
///
///	If distance specified moves the current position beyond the road, 
///	but no destination lane is given, then ePAST_ROAD is returned, and
///	the current CRoadPos instance remains on the road.
///
///	If the current CRoadPos lies on a corridor, the distance may advance
///	along the corridor or into the connecting lane.
///
/// Arguments:
///\param	dist - the amount by which to change the distance component
///\param	crdrId - relative ID of the corridor to take when travelling
///				along an intersection
/// 
/// Returns:
///	ePAST_ROAD: If the distance traveled moves the current position beyond
///				the bounds of the road, and no adjoining corridor was 
///				specified or found.
///
///	eWITHIN_ROAD: If the distance traveled results in a position on a road.
///
///	eWITHIN_CRDR: If the distance traveled results in a position on a 
///				  corridor.
///
/// eERROR:	generic error
//////////////////////////////////////////////////////////////////////////////
CRoadPos::ETravelResult
CRoadPos::TravelCrdr(double dist, const int crdrId)
{
	AssertValid();

	// If currently on a road
	if (m_isRoad) {

		if (m_pLane->direction == ePOS)
			m_dist += dist;
		else if (m_pLane->direction == eNEG)
			m_dist -= dist;

		//if (m_dist < 0.0) m_dist = 0.0;
	
		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		m_pLane = BindLane(pCurCP->laneIdx + m_pLane->laneNo);
		
		// If travel distance remains within the road
		if ( (m_dist >= 0) && 
			 (m_dist < m_pRoad->roadLengthLinear) ) 
			return eWITHIN_ROAD;
		
		// If Travel() moves into an intersection
		else {
			CLane lane(GetCved(), m_pLane);
			if (!lane.IsValid()){
				return eERROR;
			}

			CIntrsctn intr = lane.GetNextIntrsctn();
			if (!intr.IsValid()){
				return eERROR;
			}

			TCrdrVec crdrVec;
			intr.GetCrdrsStartingFrom(lane, crdrVec);
			int index = 0;

			if (crdrVec.size() > 0){
				bool found = false;
				for (unsigned int i = 0; i < crdrVec.size(); i++){
					// look for a corridor following the specified direction
					if (crdrVec[i].GetRelativeId() == crdrId) {
						index = i;
						break;
					}
					if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
						// take straight corridor if the above condition is never met
						index = i;
					}
				}
			} else {
				// intersection has no corridors (should not happen)
				return eERROR;
			}

			// make some pointers to store as member variables so we can continue traveling
			TIntrsctn* pIntrsctn = BindIntrsctn(intr.GetId());
			TCrdr* pCrdr = BindCrdr(crdrVec[index].GetId());

			m_isRoad = false;
			m_pIntrsctn = pIntrsctn;
			m_cdo.clear();
			TCdo tmpCdo;
			tmpCdo.pCrdr = pCrdr;
			tmpCdo.ofs = m_ofs;
			tmpCdo.dist = m_dist > m_pRoad->roadLengthLinear ? m_dist - m_pRoad->roadLengthLinear : -(m_dist);	// change to 0 to start at beginning of intersection
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;
			m_cdo.push_back(tmpCdo);

			// if we overshoot the intersection
			if (tmpCdo.dist > crdrVec[index].GetLength()) {
				TravelCrdr(0, crdrId);
			}

			return eWITHIN_CRDR;
		} // If travel distance moved beyond road

	} // If currently on a road

	// If currently on a corridor
	else {

		// Increment the distance on the current corridor
		vector<TCdo>::iterator cdo = m_cdo.begin();

		// tries to handle case when Travel() is supplied a CRoadPos that begins on an intersection (need to find the correct corridor first)
		if (GetCorridor(crdrId).IsValid() && cdo->pCrdr->myId != GetCorridor(crdrId).GetId()){
			TCrdrVec crdrVec;
			CIntrsctn intr = GetIntrsctn();
			if (!intr.IsValid()){
				return eERROR;
			}

			intr.GetAllCrdrs(crdrVec);
			if (crdrVec.size() == 0 ){
				// intersection has no corridors, should not happen
				return eERROR;
			}

			int index = 0;
			for(unsigned int i=0; i<crdrVec.size(); i++){
				if ( crdrVec[i].GetRelativeId() == crdrId ) {			
					index = i;
					break;
				}
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					index = i; // take a straight corridor if a corridor with the specified direction cannot be found
				}
			}

			// store corridor data
			m_isRoad = false;
			m_pIntrsctn = BindIntrsctn(intr.GetId());
			TCdo tmpCdo;
			tmpCdo.pCrdr = BindCrdr(crdrVec[index].GetId());
			tmpCdo.ofs = m_ofs;
			tmpCdo.dist = GetDistanceOnLane(crdrVec[index].GetRelativeId());
			tmpCdo.cntrlPntIdx = Search(tmpCdo.dist, tmpCdo.pCrdr)
				+ tmpCdo.pCrdr->cntrlPntIdx;
			m_cdo.clear();
			m_cdo.push_back(tmpCdo);
			cdo = m_cdo.begin();
			// if we overshoot the intersection, this should never happen
			if (tmpCdo.dist > crdrVec[index].GetLength()) {
				TravelCrdr(0, crdrId);
			}
		}

		TCrdrPnt *pLastPnt = BindCrdrPnt(cdo->pCrdr->cntrlPntIdx +
										 cdo->pCrdr->numCntrlPnt - 1);
		double length = pLastPnt->distance;

		cdo->dist += dist;

		// get the index of the point; used in GetVeryBestXYZ()
		cdo->cntrlPntIdx = Search(cdo->dist, cdo->pCrdr) + cdo->pCrdr->cntrlPntIdx;

		// If distance is within the corridor
		if ( (cdo->dist >= 0) &&
			 (cdo->dist < length) ){

			return eWITHIN_CRDR;
		}

		// If distance is beyond the corridor
		else {

			// Move current position to the connecting road/lane
			m_isRoad = true;

			// If we've advanced past the beginning of the corridor
			if (cdo->dist < 0) {
				//bind our new lane + road
				m_pRoad = BindRoad(cdo->pCrdr->srcRdIdx);
				m_pLane = BindLane(cdo->pCrdr->srcLnIdx);

				
				if (m_pLane->direction == eNEG){
					m_dist = - cdo->dist;
				} else {
					m_dist = m_pRoad->roadLengthLinear + cdo->dist;
				}

				m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;

				TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
				int id = cdo->pCrdr->srcLnIdx - m_pRoad->laneIdx;
				m_pLane = BindLane(pCurCP->laneIdx + id);

			}
			// If we've advanced beyond the end of the corridor
			else {
				m_dist = cdo->dist - length;

				m_pRoad = BindRoad(cdo->pCrdr->dstRdIdx);
				m_pLane = BindLane(cdo->pCrdr->dstLnIdx);

				if (m_pLane->direction == eNEG){
					m_dist = m_pRoad->roadLengthLinear - m_dist;
				}

				m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
				TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
				int id = cdo->pCrdr->dstLnIdx - m_pRoad->laneIdx;
				m_pLane = BindLane(pCurCP->laneIdx + id);
			}

			m_ofs = cdo->ofs;

			// if we overshoot the road
			if (cdo->dist < 0 || cdo->dist >= m_pRoad->roadLengthLinear) {
				TravelCrdr(0, crdrId);
			}

			return eWITHIN_ROAD;
				
		} // If distance is beyond the corridor

	} // If currently on a corridor

} // end of TravelCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: Travel
// 	Check where the changes of the attribe happen along the lane
//
// Remarks: This function will throw an exception if the current CRoadPos is 
// 	invalid.
//
//	This function explores the roadpos object along the direction of the
// 	lane. Once it find out any change along the direction of the lane, it
// 	will set the flag in the proper bit in the argument "posMak" which is
// 	an argument of type bitmask, and also return the distance from the 
// 	position where it is located right now.
//  
//	NOTE:  This function doesn't seem to do what it's supposed to do 
//	(e.g., posMask isn't used or modified anywhere), and I'm not exactly 
//	clear about what its supposed to do in the first place.  So, if the 
//	current CRoadPos is on an intersection, the function just returns 0. 
//	  -jvogel
//
// Arguments:
// 	posMask - a variable of type bitmask used to track what changes have
// 		been made. the index to the bit in the mask is of a enumeration type
//
//
// Returns: a distance of type double to indicate how far it would be if any 
// 	attribute along the direction ever changes toward which it is moving. 
// 	The function returns 0 if no changes exist within the road
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::Travel(bitset<cCV_NUM_ATTR_CHANGE>& posMask) const
{
	AssertValid();

	if (m_isRoad) {

		int			numOfCntrlPnt = m_pRoad->numCntrlPnt;
		int			cntrlPntIdx = m_pRoad->cntrlPntIdx;
		int			attrIdx = m_pRoad->attrIdx;

		THeader*	pH   = static_cast<THeader*>    (GetInst());
		char*		pOfs = static_cast<char*>(GetInst()) + pH->longitCntrlOfs;
		TCntrlPnt*	pCp  = reinterpret_cast<TCntrlPnt*>(pOfs) + cntrlPntIdx;
					pOfs = static_cast<char*>(GetInst()) + pH->attrOfs;
		TAttr*		pAttr= reinterpret_cast<TAttr*>(pOfs) + attrIdx;
		

		int			whichPnt = Search(m_dist);
		TCntrlPnt*	beginPnt = pCp + whichPnt;
		int			fromAttrPnt	= Search(pAttr->from);
		int			toAttrPnt	= Search(pAttr->to); 
		int			i;
		int			changedPnt = 0;
		int			dirFlag = 1;

		// below is the code to check where the change will hapen
		// at this point, it doesn't set the right flag in the input 
		// argument posMask

		if (m_pLane->direction == ePOS){
			dirFlag = 1;
			for (i = whichPnt+1; i < numOfCntrlPnt; i++){
				TCntrlPnt*	currentPnt = pCp + i;
				// check for lane width
				if (currentPnt->laneIdx != beginPnt->laneIdx){
					changedPnt = i;
					break;
				}
				// check for lateral profile
				if (currentPnt->latCntrlPntIdx != beginPnt->latCntrlPntIdx){
					changedPnt = i;
					break;
				}
				// check for attr	
				if ((i==toAttrPnt) || (i==fromAttrPnt)){
					changedPnt = i;
					break;
				}
			}
		}else if (m_pLane->direction == eNEG){
			dirFlag = -1;
			for (i=whichPnt-1; i>=0; i--){
				TCntrlPnt*	currentPnt = pCp + i;
				// check for lane width
				if (currentPnt->laneIdx != beginPnt->laneIdx){
					changedPnt = i+1;
					break;
				}
				// check for lateral profile
				if (currentPnt->latCntrlPntIdx != beginPnt->latCntrlPntIdx){
					changedPnt = i;
					break;
				}	
				// check for attr	
				if ((i==toAttrPnt) || (i==fromAttrPnt)){
					changedPnt = i+1;
					break;
				}
			}
		}

		return dirFlag*((pCp+changedPnt)->distance - (pCp+whichPnt)->distance);

	}
	else
		return 0;
} // end of Travel

//////////////////////////////////////////////////////////////////////////////
//
// Description: FindNext
// 	Advance the position on a road if any change of attribe exist along the 
// 	lane. 
//
// Remarks: This function will throw an exception if the current CRoadPos is 
// 	invalid.
//
// 	This function advances the roadpos object along the direction of the
// 	lane. Once it find out any change along the direction of the lane, it
// 	will set the flag in the proper bit in the argument "posMak" which is
// 	an argument of type bitmask, and also update road position to the postition
// 	where where the change of attribute happens.
//  
//	NOTE:  This function doesn't seem to do what it's supposed to do 
//	(e.g., posMask isn't used or modified anywhere), and I'm not exactly 
//	clear about what its supposed to do in the first place.  So, if the 
//	current CRoadPos is on an intersection, the function just returns 0. 
//	  -jvogel
//
// Arguments:
// 	posMask - a variable of type bitmask used to track what changes have
// 		been made. the index to the bit in the mask is of a enumeration type
//
//
// Returns: a distance of type double to indicate how far it would be if any 
// 	attribute along the direction ever changes toward which it is moving. 
// 	The function returns 0 if no changes exist within the road
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::FindNext(bitset<cCV_NUM_ATTR_CHANGE>& posMask)
{
	AssertValid();

	if (m_isRoad) {
		int			numOfCntrlPnt = m_pRoad->numCntrlPnt;
		int			cntrlPntIdx = m_pRoad->cntrlPntIdx;
		int			attrIdx = m_pRoad->attrIdx;

		THeader*	pH   = static_cast<THeader*>    (GetInst());
		char*		pOfs = static_cast<char*>(GetInst()) + pH->longitCntrlOfs;
		TCntrlPnt*	pCp  = reinterpret_cast<TCntrlPnt*>(pOfs) + cntrlPntIdx;
						pOfs = static_cast<char*>(GetInst()) + pH->attrOfs;
		TAttr*		pAttr= reinterpret_cast<TAttr*>(pOfs) + attrIdx;
		

		int			whichPnt = Search(m_dist);
		TCntrlPnt*	beginPnt = pCp + whichPnt;
		int			fromAttrPnt	= Search(pAttr->from);
		int			toAttrPnt	= Search(pAttr->to); 
		int			i;
		int			changedPnt = 0;
		int			dirFlag = 1;

		// below is the code to check where the change will hapen
		// at this point, it doesn't set the right flag in the input 
		// argument posMask

		if (m_pLane->direction == ePOS){
			dirFlag = 1;
			for (i = whichPnt+1; i < numOfCntrlPnt; i++){
				TCntrlPnt*	currentPnt = pCp + i;
				// check for lane width
				if (currentPnt->laneIdx != beginPnt->laneIdx){
					changedPnt = i;
					break;
				}
				// check for lateral profile
				if (currentPnt->latCntrlPntIdx != beginPnt->latCntrlPntIdx){
					changedPnt = i;
					break;
				}
				// check for attr	
				if ((i==toAttrPnt) || (i==fromAttrPnt)){
					changedPnt = i;
					break;
				}
			}
		}else if (m_pLane->direction == eNEG){
			dirFlag = -1;
			for (i=whichPnt-1; i>=0; i--){
				TCntrlPnt*	currentPnt = pCp + i;
				// check for lane width
				if (currentPnt->laneIdx != beginPnt->laneIdx){
					changedPnt = i+1;
					break;
				}
				// check for lateral profile
				if (currentPnt->latCntrlPntIdx != beginPnt->latCntrlPntIdx){
					changedPnt = i;
					break;
				}	
				// check for attr	
				if ((i==toAttrPnt) || (i==fromAttrPnt)){
					changedPnt = i+1;
					break;
				}
			}
		}

		m_dist = (pCp+changedPnt)->distance;

		// Find the control point associated with the 
		// 	given distance along the road.
		m_cntrlPntIdx = Search(m_dist) + m_pRoad->cntrlPntIdx;
		TCntrlPnt* pCurCP = BindCntrlPnt(m_cntrlPntIdx);
		int laneNo = m_pLane->laneNo;
		m_pLane = BindLane(pCurCP->laneIdx + laneNo);

		return dirFlag*((pCp+changedPnt)->distance - (pCp+whichPnt)->distance);
	}

	else
		return 0;
} // end of FindNext

//////////////////////////////////////////////////////////////////////////////
//
// Description: Updates the current road position with the geometric 
//  position parameter.
//
// Remarks:  If the current CRoadPos is valid, then the previous data is
// 	used to search for the new position more quickly.  If the current 
// 	CRoadPos is invalid, then the road piece quadtree is searched to find
// 	the corresponding road position.
//
// Arguments:
// 	cPoint - geometric point in space for which to find a matching road pos
//	cpCrdr - (optional) a destination corridor
//	pColdSearch - (optional) an output parameter that is set to true if a 
//		cold search was required, false otherwise.
//
// Returns: True if parameter is on a road, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CRoadPos::SetXYZ(
			const CPoint3D& cPoint,
			const CCrdr* cpCrdr,
			bool* pColdSearch
			) 
{
	vector<TSegment> segments;
	TSegment tmpSeg;
	bool found = false;

	if( pColdSearch )  *pColdSearch = false;

	// If the current CRoadPos contains valid data, then
	// 	search around that point first.
	//
	// If current CRoadPos is on a valid road
	if( (m_isRoad) && (m_pRoad!=0) && (m_pLane!=0) ) 
	{
		// Add current road segment to the vector
		tmpSeg.isRoad = true;
		tmpSeg.begCntrlPntIdx = m_cntrlPntIdx;
		tmpSeg.endCntrlPntIdx = m_cntrlPntIdx+1;
		tmpSeg.pRoad = m_pRoad;
		tmpSeg.direction = m_pLane->direction;
		segments.push_back( tmpSeg );

		GetRoadSegments( segments, 0.0, cpCrdr );

#if ROAD_POS_DEBUG
		gout << "Warm search on road: ";
#endif
		m_cdo.clear();
		found = FindPoint( cPoint, segments );

#if ROAD_POS_DEBUG
		gout << (found? "worked" : "failed") << endl;
#endif
	} // If current CRoadPos is on a valid road

	// Otherwise, current CRoadPos is on an intersection
	else if( (!m_isRoad) && (m_pIntrsctn!=0) && (!m_cdo.empty()) ) 
	{
		// Add current intrsctn segment to the vector
		tmpSeg.isRoad = false;
		tmpSeg.pIntrsctn = m_pIntrsctn;

		vector<TCdo>::const_iterator pCdo;
		vector<TCdo> tmpCdo = m_cdo;
		m_cdo.clear();
		for( pCdo = tmpCdo.begin(); pCdo != tmpCdo.end(); pCdo++ ) 
		{
			segments.clear();
			tmpSeg.begCntrlPntIdx = pCdo->cntrlPntIdx;
			tmpSeg.endCntrlPntIdx = pCdo->cntrlPntIdx+1;
			tmpSeg.pCrdr = pCdo->pCrdr;
			segments.push_back( tmpSeg );

			// Search forward from the current segment
			GetCrdrSegments( segments, 0.0, cpCrdr );

			found |= FindPoint( cPoint, segments );
		}

#if ROAD_POS_DEBUG
		gout << "Warm search on isec: "
			 << (found? "worked" : "failed") << endl;
#endif
	} // If current CRoadPos is on a valid intersection

	// If the previous search was unsuccessful, or if the current
	// 	road pos does not contain valid data, then found would be
	// 	false.  Search both quadtrees.
	if( !found ) 
	{
		if( pColdSearch )  *pColdSearch = true;

		vector<int> quadTreeIds;

		///////////////////////////////////////////////////////////////
		// Get the roadpieces overlapping the point
		TRoadPiece* pRdPc;

		TRoad* pRoadPool = BindRoad( 0 );
		TRoadPiece* pRoadPiecePool = BindRoadPiece( 0 );

		GetCved().SearchRdPcQuadTree(
							cPoint.m_x,
							cPoint.m_y,
							cPoint.m_x,
							cPoint.m_y,
							quadTreeIds
							);

		// Assemble the vector of USegments
		segments.clear();

		// For each road piece in result
		vector<int>::const_iterator cItr;
		for( cItr = quadTreeIds.begin(); cItr != quadTreeIds.end(); cItr++ ) 
		{
			// Add a segment for that road piece.
			pRdPc = &pRoadPiecePool[*cItr];

			tmpSeg.isRoad = true;
			tmpSeg.pRoad = &pRoadPool[pRdPc->roadId];
			tmpSeg.begCntrlPntIdx = pRdPc->first + tmpSeg.pRoad->cntrlPntIdx;
			tmpSeg.endCntrlPntIdx = pRdPc->last + tmpSeg.pRoad->cntrlPntIdx;

			segments.push_back( tmpSeg );
		} // For each road piece in result

			
		////////////////////////////////////////////////////////////
		// Get the intersections overlapping the point.
		//
		TIntrsctn* pIsecPool = BindIntrsctn( 0 );
		TCrdr*	   pCrdrPool = BindCrdr( 0 );

		// If a corridor was passed to this function, use it.
		if( cpCrdr ) 
		{
			tmpSeg.isRoad = false;
			tmpSeg.pCrdr = &pCrdrPool[cpCrdr->GetId()];
			tmpSeg.pIntrsctn = &pIsecPool[cpCrdr->GetIntrsctnId()];
            CCved::TIntVec pnts;
            GetCved().GetCrdrsCntrlPointsNear(cPoint,cpCrdr->GetId(),pnts);

            for (unsigned int i = 0; i < pnts.size(); i++){
                tmpSeg.begCntrlPntIdx = pnts[i];
                tmpSeg.endCntrlPntIdx = pnts[i]+1;
			    segments.push_back( tmpSeg );
            }
		} // if a corridor was given

		// If no corridor was given, do a cold search on the intersection
		// 	quad tree.
		else 
		{
			// Note that this line may be deleted, once Concurrent 
			// 	fixes the STL set<>::clear() method.
			GetCved().SearchIntrsctnQuadTree(
								cPoint.m_x,
								cPoint.m_y,
								cPoint.m_x,
								cPoint.m_y,
								quadTreeIds
								);
  //      
			TIntrsctn* pIntrsctn = pIsecPool;
			TCrdr* pCrdrPool = BindCrdr( 0 );
			TCrdr* pCrdr = pCrdrPool;
			int crdrItr, startCrdrId, endCrdrId;
			// For each intrsctn in result
			for( cItr = quadTreeIds.begin(); cItr != quadTreeIds.end(); cItr++ )
			{	
				// Add a segment for each corridor
				pIntrsctn = &pIsecPool[*cItr];

				try {
					CVED::CIntrsctn intObj(GetCved(),*cItr);
					CPolygon2D border;
					intObj.GetBorder(border);
					if (border.Contains(cPoint))
					{
						startCrdrId = pIntrsctn->crdrIdx;
						endCrdrId = startCrdrId + pIntrsctn->numOfCrdrs;
						for(
							crdrItr = startCrdrId, 
				 			pCrdr = &pCrdrPool[crdrItr];
				 			crdrItr < endCrdrId;
				 			crdrItr++, pCrdr++
							)
						{	
							tmpSeg.isRoad = false;
							tmpSeg.pCrdr = pCrdr;
							tmpSeg.pIntrsctn = pIntrsctn;
                            CCved::TIntVec pnts;
                            GetCved().GetCrdrsCntrlPointsNear(cPoint,pCrdr->myId,pnts);
                            for (unsigned int i = 0; i < pnts.size(); i++){
                                tmpSeg.begCntrlPntIdx = pnts[i];
                                tmpSeg.endCntrlPntIdx = pnts[i]+1;
                                segments.push_back( tmpSeg );
                            }
						}
					}
				} catch (cvCInternalError) {
					// skip over problematic intersections
				}
			} // For each intrsctn in result
		} // If no corridor was given

#if ROAD_POS_DEBUG
		gout << "Cold search: ";
#endif
		m_cdo.clear();
		found = FindPoint(cPoint, segments);

#if ROAD_POS_DEBUG
		gout << (found? "worked" : "failed") << endl;
#endif	
	} // if (!found) 

	if( found )
	{
		//
		// For roadPos on road, make sure that the distance lies between
		// 0 and the max road length.
		//
		if( m_isRoad && m_pRoad )
		{
			if( m_dist < 0.0 )  m_dist = 0.0;
			else if( m_dist > m_pRoad->roadLengthLinear )  m_dist = m_pRoad->roadLengthLinear;
		}
	}
	else
	{	
		// Make current CRoadPos invalid
		m_pRoad = 0;
		m_pLane = 0;
		m_pIntrsctn = 0;
		m_cdo.clear();
	}

	return found;

} // end of SetXYZ

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetXY
// 	Updates the current road position with the 2D geometric position parameter.
//
// Remarks: The current CRoadPos must be valid so that the missing elevation 
//	information can be taken from the current point.
//
// Arguments:
// 	cPoint - geometric point in space for which to find a matching road pos
//	cpCrdr - (optional) a destination corridor
//	pColdSearch - (optional) an output parameter that is set to true if a 
//		cold search was required, false otherwise.
//  cHeightThresh - height threshold within which to look for points
//
// Returns: True if parameter is on a road, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CRoadPos::SetXY(
			const CPoint2D& cPoint,
			const CCrdr* cpCrdr,
			bool* pColdSearch,
			const double cHeightThresh
			) 
{
	vector<TSegment> segments;
	TSegment tmpSeg;
	bool found = false;
	CPoint3D point3d( cPoint.m_x, cPoint.m_y, 0.0 );

	if( pColdSearch )  *pColdSearch = false;

	// If the current CRoadPos contains valid data, then
	// 	search around that point first.
	//
	// If current CRoadPos is on a valid road
	if ( (m_isRoad) && (m_pRoad!=0) && (m_pLane!=0) ) {
	
		// Use the elevation of the current point
		TCntrlPnt* pCntrlPnt = BindCntrlPnt(m_cntrlPntIdx);
		point3d.m_z = pCntrlPnt->location.z;
		
		// Add current road segment to the vector
		tmpSeg.isRoad = true;
		tmpSeg.begCntrlPntIdx = m_cntrlPntIdx;
		tmpSeg.endCntrlPntIdx = m_cntrlPntIdx+1;
		tmpSeg.pRoad = m_pRoad;
		tmpSeg.direction = m_pLane->direction;
		segments.push_back( tmpSeg );

		GetRoadSegments( segments, 0.0, cpCrdr );

#if ROAD_POS_DEBUG
		gout << "Warm search on road: ";
#endif
		m_cdo.clear();
		found = FindPoint( point3d, segments, cHeightThresh );

#if ROAD_POS_DEBUG
		gout << (found? "worked" : "failed") << endl;
#endif

	} // If current CRoadPos is on a valid road

	// Otherwise, current CRoadPos is on an intersection
	else if( (!m_isRoad) && (m_pIntrsctn!=0) && (!m_cdo.empty()) ) 
	{
		// Use the elevation of the current point
		const CTerrainGrid<Post>* pGrid = 
			GetIntrsctnGrid( m_pIntrsctn->myId );
		if( pGrid ) 
		{
			Post gridOut;
			CPoint3D qryPnt = GetBestXYZ();
			if( pGrid->QueryElev(qryPnt, gridOut) )
			{
				point3d.m_z = gridOut.z;
			}
		}
		else
		{
			point3d.m_z = m_pIntrsctn->elevation;
		}

		// Add current intrsctn segment to the vector
		tmpSeg.isRoad = false;
		tmpSeg.pIntrsctn = m_pIntrsctn;

		vector<TCdo> tmpCdoVec = m_cdo;
		vector<TCdo>::const_iterator pCdo;
		m_cdo.clear();
		for( 
			pCdo = tmpCdoVec.begin(); 
			pCdo != tmpCdoVec.end();
			pCdo++
			)
		{
			segments.clear();
			tmpSeg.begCntrlPntIdx = pCdo->cntrlPntIdx;
			tmpSeg.endCntrlPntIdx = pCdo->cntrlPntIdx+1;
			tmpSeg.pCrdr = pCdo->pCrdr;
			segments.push_back( tmpSeg );

			// Search forward from the current segment
			GetCrdrSegments( segments, 0.0, cpCrdr );

			found |= FindPoint( point3d, segments, cHeightThresh );
		}

#if ROAD_POS_DEBUG
		gout << "Warm search on isec: "
			 << (found? "worked" : "failed") << endl;
#endif
	} // If current CRoadPos is on a valid intersection

	// If the previous search was unsuccessful, or if the current
	// 	road pos does not contain valid data, then found would be
	// 	false.  Search both quadtrees.
	if( !found ) 
	{
		if( pColdSearch )  *pColdSearch = true;

		vector<int> quadTreeIds;
		vector<int>::const_iterator itr;

		///////////////////////////////////////////////////////////////
		// Get the roadpieces overlapping the point
		TRoadPiece* pRdPc;

		TRoad* pRoadPool = BindRoad(0);
		TRoadPiece* pRoadPiecePool = BindRoadPiece(0);

		GetCved().SearchRdPcQuadTree(
							cPoint.m_x,
							cPoint.m_y,
							cPoint.m_x,
							cPoint.m_y,
							quadTreeIds
							);

		// Assemble the vector of USegments
		segments.clear();

		// For each road piece in result
		for( itr = quadTreeIds.begin(); itr != quadTreeIds.end(); itr++ )
		{
			// Add a segment for that road piece.
			pRdPc = &pRoadPiecePool[*itr];

			tmpSeg.isRoad = true;
			tmpSeg.pRoad = &pRoadPool[pRdPc->roadId];
			tmpSeg.begCntrlPntIdx = pRdPc->first + 
				tmpSeg.pRoad->cntrlPntIdx;
			tmpSeg.endCntrlPntIdx = pRdPc->last + 
				tmpSeg.pRoad->cntrlPntIdx;

			segments.push_back( tmpSeg );
		} // For each road piece in result

			
		////////////////////////////////////////////////////////////
		// Get the intersections overlapping the point.
		//
		TIntrsctn* pIsecPool = BindIntrsctn( 0 );
		TCrdr*	   pCrdrPool = BindCrdr( 0 );

		// If a corridor was passed to this function, use it.
		if( cpCrdr ) 
		{
			tmpSeg.isRoad = false;
			tmpSeg.pCrdr = &pCrdrPool[cpCrdr->GetId()];
			tmpSeg.pIntrsctn = &pIsecPool[cpCrdr->GetIntrsctnId()];
			tmpSeg.begCntrlPntIdx = tmpSeg.pCrdr->cntrlPntIdx;
			tmpSeg.endCntrlPntIdx = tmpSeg.pCrdr->cntrlPntIdx +
				tmpSeg.pCrdr->numCntrlPnt - 1;

			segments.push_back( tmpSeg );
		} // if a corridor was given

		// If no corridor was given, do a cold search on the intersection
		// 	quad tree.
		else 
		{
			// Note that this line may be deleted, once Concurrent 
			// 	fixes the STL set<>::clear() method.
			GetCved().SearchIntrsctnQuadTree(
								cPoint.m_x,
								cPoint.m_y,
								cPoint.m_x,
								cPoint.m_y,
								quadTreeIds
								);

			TIntrsctn* pIntrsctn = pIsecPool;
			TCrdr* pCrdrPool = BindCrdr( 0 );
			TCrdr* pCrdr = pCrdrPool;
			int crdrItr, startCrdrId, endCrdrId;

			// For each intrsctn in result
			for( 
				itr = quadTreeIds.begin(); 
			 	itr != quadTreeIds.end(); 
				itr++
				) 
			{
				// Add a segment for each corridor
				pIntrsctn = &pIsecPool[*itr];
				startCrdrId = pIntrsctn->crdrIdx;
				endCrdrId = startCrdrId + pIntrsctn->numOfCrdrs;
	
				for(
					crdrItr = startCrdrId, 
				 	pCrdr = &pCrdrPool[crdrItr];
				 	crdrItr < endCrdrId;
				 	crdrItr++, pCrdr++
					)
				{	
					tmpSeg.isRoad = false;
					tmpSeg.pCrdr = pCrdr;
					tmpSeg.pIntrsctn = pIntrsctn;
					tmpSeg.begCntrlPntIdx = pCrdr->cntrlPntIdx;
					tmpSeg.endCntrlPntIdx = pCrdr->cntrlPntIdx +
						pCrdr->numCntrlPnt - 1;
	
					segments.push_back( tmpSeg );
				}
			} // For each intrsctn in result
		} // If no corridor was given

#if ROAD_POS_DEBUG
		gout << "Cold search: ";
#endif
		m_cdo.clear();
		found = FindPoint(point3d, segments, cHeightThresh);

#if ROAD_POS_DEBUG
		gout << (found? "worked" : "failed") << endl;
#endif
	} // if (!found) 

	return found;

} // end of SetXY

//////////////////////////////////////////////////////////////////////////////
//		Protected functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: Search (protected)
// 	Protected method that searches the right control point containing the 
// 	lane/crdr information
//
// Remarks: This function searches the right control point corresponding to 
// 	the argument dist.  If the current CRoadPos is a road, then the road 
// 	control points are searched.  Otherwise, the crdr cntrl points are 
// 	searched.
//
// Arguments:
// 	dist - distance along current road or corridor to search
// 	cpCrdr - optional parameter, if specified, then search along that corridor.
//
// Returns: A number representing the index to right control point containing 
// 	the lane/crdr control point index with respect to the road/crdr
//
//////////////////////////////////////////////////////////////////////////////
int
CRoadPos::Search(double dist, 
				 const TCrdr* cpCrdr) const
{
#ifdef _DEBUG
	cvTRoad tempcv; //for the debugger
#endif
	// Search road if on road.
	if (m_isRoad) {

		AssertValid();

		int			numOfCntrlPt = m_pRoad->numCntrlPnt;
		int			cntrlPntIdx = m_pRoad->cntrlPntIdx;
		TCntrlPnt*	pCp  =  BindCntrlPnt(cntrlPntIdx);

		int			begin   = 0;
		int			end     = numOfCntrlPt - 1;
		int			middle  = (begin+end)/2;;

		while(middle!=begin){
			if ( dist < (pCp+middle)->distance )
				end = middle;
			else
				begin = middle;
			middle = (begin+end)/2;
		}

		return begin;
	}
	// Search crdr if on intrsctn
	else {
		if (!cpCrdr) {
			AssertValid();
			cpCrdr = m_cdo.begin()->pCrdr;
		}

		int			numOfCntrlPt = cpCrdr->numCntrlPnt;
		int			cntrlPntIdx = cpCrdr->cntrlPntIdx;
		TCrdrPnt*	pCp  =  BindCrdrPnt(cntrlPntIdx);

		int			begin   = 0;
		int			end     = numOfCntrlPt - 1;
		int			middle  = (begin+end)/2;;

		while(middle!=begin){
			if ( dist < (pCp+middle)->distance )
				end = middle;
			else
				begin = middle;
			middle = (begin+end)/2;
		}

		return begin;
	}
} // end of Search

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoadSegments (protected)
// 	Assembles the road segments from the last segment in the vector for a 
// 	distance of cSEARCH_DISTANCE. 
//
// Remarks: This function is called recursively until cSEARCH_DISTANCE is
// 	covered.  It is used by SetXYZ to collect a set of road segments to search.
//
// Arguments:
// 	segments - vector to place the segments in
// 	distCovered - total distance covered so far
// 	cpCrdr - (optional) specifies a destination corridor for the function to
// 		search.
//
// 	Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CRoadPos::GetRoadSegments(vector<TSegment>& segments, 
						  double 			distCovered, 
						  const CCrdr* 		cpCrdr) 
{
	TSegment	tmpSeg;
	vector<TSegment>::iterator pFinalSeg;
	TIntrsctn*	pIntrsctn;
	unsigned int crdrId;
	TCrdr*		pCrdr;
	int			roadId;

	double		newDist = distCovered;

	// If the search distance hasn't been covered yet
	// 	and there is a segment in the vector to start from, 
	// 	then continue search.
	if ( (distCovered < cSEARCH_DISTANCE) &&
		 (!segments.empty()) ) {

		pFinalSeg = segments.end();
		pFinalSeg--;

		if (!pFinalSeg->isRoad) {
			// Wrong function was called, but it's ok
			// Call GetCrdrSegments.
			GetCrdrSegments(segments, distCovered, cpCrdr);
		}
		
		// If final segment is on a road, 
		// 	figure out whether to increment the end control
		// 	point of the current road segment, or add some 
		// 	corridor segments.

		pFinalSeg = segments.end();
		pFinalSeg--;

		// If the road direction is positive, then increment
		// 	segment to next control point.
		if (pFinalSeg->direction == ePOS) {

			// The distance covered is the distance to the 
			// 	next control point of the road.
			TCntrlPnt* pCurCP = BindCntrlPnt(pFinalSeg->endCntrlPntIdx);
			newDist += pCurCP->distToNextLinear;

			// If not at the penultimate cntrl pnt on road 
			// (at the end), increment endCntrlPntIdx and recur.
			if (pFinalSeg->endCntrlPntIdx + 1 <
				pFinalSeg->pRoad->cntrlPntIdx + 
				pFinalSeg->pRoad->numCntrlPnt) {

				pFinalSeg->endCntrlPntIdx++;
				GetRoadSegments(segments, newDist, cpCrdr);
			}

			// If at the penultimate cntrl pnt on the road, 
			//	then look at destination intersection.
			else {
				
				// Find connecting (destination) intersection
				pIntrsctn = BindIntrsctn(pFinalSeg->pRoad->dstIntrsctnIdx);

				// If a corridor was passed to this function, use it.
				if (cpCrdr) {
					
					// If the corridor belongs to the destination intrsctn
					// 	and the corridor begins at the current road
					if ( (cpCrdr->GetIntrsctnId() == pIntrsctn->myId) &&
						 (cpCrdr->GetSrcRdIdx() == pFinalSeg->pRoad->myId) ) {

						// For each corridor in the intersection
						// 	(that runs from the current road), 
						// 	add a segment.
						pCrdr = BindCrdr(cpCrdr->GetId());
		
						// Add this corridor to the list of segments
						tmpSeg.isRoad = false;
						tmpSeg.pCrdr = pCrdr;
						tmpSeg.pIntrsctn = pIntrsctn;
						tmpSeg.begCntrlPntIdx = pCrdr->cntrlPntIdx;
						tmpSeg.endCntrlPntIdx = tmpSeg.begCntrlPntIdx+1;
						segments.push_back(tmpSeg);

						// The distance covered is the distance previously
						//	calculated, plus the distance between the two
						//	corridor control points.
						TCrdrPnt* pCurCP = 
							BindCrdrPnt(tmpSeg.begCntrlPntIdx);
						TCrdrPnt* pNexCP = pCurCP+1;
						double crdrDist = 0.0;
						if ( (pCurCP != 0) && (pNexCP != 0) )
							crdrDist = pNexCP->distance - pCurCP->distance;

						GetCrdrSegments(segments, newDist+crdrDist, cpCrdr);
						
					} // If the destination corridor can be reached from here

					// The given corridor does not connect to the current
					// 	road/lane, so return without adding any points.
					else {
						return;
					}
					
				} // if a destination corridor was provided
				
				// No destination corridor was specified, so perform
				// 	a normal search on all corridors in the intersection.
				else {

					// For each corridor in the intersection
					// 	(that runs from the current road), 
					// 	add a segment.
					pCrdr = BindCrdr(pIntrsctn->crdrIdx);
	
					// Store roadId, because pFinalSeg will become
					//	invalid once a new segment is pushed onto
					//	the vector.
					roadId = pFinalSeg->pRoad->myId;
	
					for (crdrId = 0; 
				 	 	crdrId < pIntrsctn->numOfCrdrs; 
				 	 	crdrId++, pCrdr++) {
					
						// If the corridor begins at the current road
						if (pCrdr->srcRdIdx == roadId) {
	
							tmpSeg.isRoad = false;
							tmpSeg.pCrdr = pCrdr;
							tmpSeg.pIntrsctn = pIntrsctn;
							tmpSeg.begCntrlPntIdx = pCrdr->cntrlPntIdx;
							tmpSeg.endCntrlPntIdx = tmpSeg.begCntrlPntIdx+1;
							segments.push_back(tmpSeg);
							
							// The distance covered is the distance previously
							//	calculated, plus the distance between the two
							//	corridor control points.
							TCrdrPnt* pCurCP = 
								BindCrdrPnt(tmpSeg.begCntrlPntIdx);
							TCrdrPnt* pNexCP = pCurCP+1;
							double crdrDist = 0.0;
							if ( (pCurCP != 0) && (pNexCP != 0) )
								crdrDist = pNexCP->distance - pCurCP->distance;
	
							GetCrdrSegments(segments, newDist+crdrDist, cpCrdr);
						}
	
					} // For each corridor
	
				} // else no destination corridor was provided

			} // At end of road
			
		} // direction is positive

		// If the lane direction is negative with respect to the road
		else {

			// The distance covered is the distance to the 
			//  beginning control point of the segment.
			TCntrlPnt* pCurCP = BindCntrlPnt(pFinalSeg->begCntrlPntIdx-1);
			if (pCurCP != 0)
				newDist += pCurCP->distToNextLinear;

			// If not at the beginning of the road, 
			// decrement begCntrlPntIdx and recur.
			if (pFinalSeg->begCntrlPntIdx >
				pFinalSeg->pRoad->cntrlPntIdx) {

				pFinalSeg->begCntrlPntIdx--;
				GetRoadSegments(segments, newDist);
			}

			// If at the first cntrl pnt on the road, 
			//	then look at source intersection.
			else {

				// Find connecting (source) intersection
				pIntrsctn = BindIntrsctn(pFinalSeg->pRoad->srcIntrsctnIdx);
	
				// If a corridor was passed to this function, use it.
				if (cpCrdr) {
					
					// For each corridor in the intersection
					// 	(that runs to the current road), 
					// 	add a segment.
					pCrdr = BindCrdr(cpCrdr->GetId());

					// If the corridor belongs to the source intrsctn
					// 	and the corridor begins at the current road
					if ( (cpCrdr->GetIntrsctnId() == pIntrsctn->myId) &&
						 (cpCrdr->GetSrcRdIdx() == pFinalSeg->pRoad->myId) ) {

						// Add this corridor to the list of segments
						tmpSeg.isRoad = false;
						tmpSeg.pCrdr = pCrdr;
						tmpSeg.pIntrsctn = pIntrsctn;
						tmpSeg.begCntrlPntIdx = pCrdr->cntrlPntIdx;
						tmpSeg.endCntrlPntIdx = tmpSeg.begCntrlPntIdx+1;
						segments.push_back(tmpSeg);

						// The distance covered is the distance previously
						//	calculated, plus the distance between the two
						//	corridor control points.
						TCrdrPnt* pCurCP = 
							BindCrdrPnt(tmpSeg.begCntrlPntIdx);
						TCrdrPnt* pNexCP = pCurCP+1;
						double crdrDist = 0.0;
						if ( (pCurCP != 0) && (pNexCP != 0) )
							crdrDist = pNexCP->distance - pCurCP->distance;

						GetCrdrSegments(segments, newDist+crdrDist, cpCrdr);
						
					} // If the destination corridor can be reached from here

					// The given corridor does not connect to the current
					// 	road/lane, so return without adding any points.
					else {
						return;
					}
					
				} // if a destination corridor was provided
				
				// No destination corridor was specified, so perform
				// 	a normal search on all corridors in the intersection.
				else {

					// Find connecting (source) intersection
					pIntrsctn = BindIntrsctn(pFinalSeg->pRoad->srcIntrsctnIdx);
	
					// For each corridor in the intersection
					// 	(that runs to the current road), 
					// 	add a segment.
					pCrdr = BindCrdr(pIntrsctn->crdrIdx);
	
					// Store roadId, because pFinalSeg will become
					//	invalid once a new segment is pushed onto
					//	the vector.
					roadId = pFinalSeg->pRoad->myId;
	
					for (crdrId = 0; 
					 	crdrId < pIntrsctn->numOfCrdrs; 
					 	crdrId++, pCrdr++) {
						
						// If the corridor begins at the current road
						if (pCrdr->srcRdIdx == roadId) {
	
							tmpSeg.isRoad = false;
							tmpSeg.pCrdr = pCrdr;
							tmpSeg.pIntrsctn = pIntrsctn;
							tmpSeg.begCntrlPntIdx = pCrdr->cntrlPntIdx;
							tmpSeg.endCntrlPntIdx = tmpSeg.begCntrlPntIdx + 1;
							segments.push_back(tmpSeg);
							
							// The distance covered is the distance previously
							//	calculated, plus the distance between the two
							//	corridor control points.
							TCrdrPnt* pCurCP = BindCrdrPnt(
														tmpSeg.begCntrlPntIdx);
							TCrdrPnt* pNexCP = pCurCP+1;
							double crdrDist = 0;
							if ( (pCurCP != 0) && (pNexCP != 0) )
								crdrDist = pNexCP->distance - pCurCP->distance;
	
							GetCrdrSegments(segments, newDist+crdrDist, cpCrdr);
	
						} // If the corridor begins at the current road
	
					} // For each corridor
	
				} // No destination corridor was specified

			} // At beginning of road

		} // direction is negative
		
	} // If the search distance hasn't been covered yet and
	  // there is a segment to start with.

} // end of GetRoadSegments

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdrSegments (protected)
// 	Assembles the corridor segments from the last segment in the vector for a 
// 	distance of cSEARCH_DISTANCE. 
//
// Remarks: This function is called recursively until cSEARCH_DISTANCE is
// 	covered.  It is used by SetXYZ to collect a set of corridor segments to 
// 	search.
//
// Arguments:
// 	segments - vector to place the segments in
// 	distCovered - total distance covered so far
// 	cpCrdr - (optional) Used by this function only to pass to GetRoadSegments.
// 		Once this function is called, a corridor has already been selected, so
// 		the parameter is not used.
//
// 	Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CRoadPos::GetCrdrSegments(vector<TSegment>& segments, 
						  double 			distCovered,
						  const CCrdr*		cpCrdr)
{
	TSegment	tmpSeg;
	vector<TSegment>::iterator pFinalSeg;

	double		newDist = distCovered;

	TLane*		pLane;
	TRoad*		pRoad;

	// If the search distance hasn't been covered yet
	// 	and there is a segment in the vector to start from, 
	// 	then continue search.
	if ( (distCovered < cSEARCH_DISTANCE) &&
		 (!segments.empty()) ) {

		pFinalSeg = segments.end();
		pFinalSeg--;

		// If the final segment is on a road, then
		// the wrong function was called, but it's ok.
		// Call GetRoadSegments.
		if (pFinalSeg->isRoad) {
			GetRoadSegments(segments, distCovered, cpCrdr);
			return;
		}

		// If the final segment is on an intersection, 
		// 	figure out whether to increment the end cntrl pnt idx
		// 	or add the connecting road segment.

		// The distance covered is the distance to the 
		// 	next control point of the crdr.
		TCrdrPnt* pCurCP = BindCrdrPnt(pFinalSeg->endCntrlPntIdx);
		TCrdrPnt* pNexCP = pCurCP+1;
		if ( (pCurCP != 0) && (pNexCP != 0) ) 
			newDist += (pNexCP->distance - pCurCP->distance);

		// If not at the pentuntimate cntrl pnt on crdr
		// (at the end), increment endCntrlPntIdx and recur.
		if (pFinalSeg->endCntrlPntIdx + 2 <
			pFinalSeg->pCrdr->cntrlPntIdx +
			pFinalSeg->pCrdr->numCntrlPnt) {

			pFinalSeg->endCntrlPntIdx++;
			GetCrdrSegments(segments, newDist, cpCrdr);
		}

		// If at the penultimate cntrl pnt on the crdr,
		//	then look at connecting road.
		else {

			// Find connecting (destination) road and lane
			pRoad = BindRoad(pFinalSeg->pCrdr->dstRdIdx);
			pLane = BindLane(pFinalSeg->pCrdr->dstLnIdx);

			tmpSeg.isRoad = true;
			tmpSeg.pRoad = pRoad;
			tmpSeg.direction = pLane->direction;

			// If the connecting lane is moving in a positive 
			// 	direction w/r/t the road, then set control points
			// 	to first two on the road and calculate the new
			// 	distance covered.
			if (tmpSeg.direction == ePOS) {

				tmpSeg.begCntrlPntIdx = pRoad->cntrlPntIdx;
				tmpSeg.endCntrlPntIdx = tmpSeg.begCntrlPntIdx + 1;
				segments.push_back(tmpSeg);

				// The distance covered is the distance previously
				//	calculated, plus the distance between the two
				//	road control points.
				TCntrlPnt* pCurCP = BindCntrlPnt(tmpSeg.begCntrlPntIdx);
				if (pCurCP != 0) 
					newDist += pCurCP->distToNextLinear;
			}

			// If the connecting lane is moving in a positive 
			// 	direction w/r/t the road, then set control points
			// 	to first two on the road and calculate the new
			// 	distance covered.
			else {

				tmpSeg.begCntrlPntIdx = pRoad->cntrlPntIdx +
					pRoad->numCntrlPnt - 2;
				tmpSeg.endCntrlPntIdx = tmpSeg.begCntrlPntIdx + 1;
				segments.push_back(tmpSeg);

				// The distance covered is the distance previously
				//	calculated, plus the distance between the two
				//	road control points.
				TCntrlPnt* pCurCP = BindCntrlPnt(tmpSeg.begCntrlPntIdx);
				if (pCurCP != 0) 
					newDist += pCurCP->distToNextLinear;
			}

			// Call GetRoadSegments on the current segment
			GetRoadSegments(segments, newDist, cpCrdr);

		} // At the end of the corridor

	} // If the search distance hasn't been covered yet and
	  // there is a segment to start with.

} // end of GetCrdrSegments

//////////////////////////////////////////////////////////////////////////////
///
/// Description: FindPoint (protected)
/// 	Searches a vector of TSegments for the given point.
///
/// Remarks: If the TSegment is a road segment, FindRoadPoint is called.
/// 	Otherwise, FindIntrsctnPoint is called.
///
/// 	The segments will be searched in the order they are in the vector.  If
///  the point is found on a road, then no other segments are searched.  If
///  the point is found on an intersection, then all corridor segments are
///  searched in order to find every corridor the point lies on.
///
/// Arguments:
///\param[in]	cPoint - geometric point for which to search
///\param[in]	cSegments - vector of USegments to search
///\param[in]   cHeightThresh - height threshold within which to look for points
///
///\return True if the point is found, false otherwise.
///
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::FindPoint(
			const CPoint3D& cPoint,
		 	const vector<TSegment>& cSegments,
			const double cHeightThresh
			) 
{
	
	vector<TSegment>::const_iterator pSeg;
	bool found = false;

	double splineOfs = -1; 
	double prevT = 0.0;	

	// For each segment in the vector
	for( pSeg = cSegments.begin(); pSeg != cSegments.end(); pSeg++ ) 
	{
		// If the current segment is on a road
		if( pSeg->isRoad )
		{
			// If the point has already been found
			//	(from previous crdr searches)
			//	then return true.
			if( found )  return true;

			// Search for the point on the road segment
			found |= FindRoadPoint( cPoint, *pSeg, cHeightThresh );

			// If found there, return true.
			if( found )  return true;
		
		}
		else 
		{

			// Look for all crdrs that the point is on, 
			//	so don't return until all cSegments are 
			//	searched *or* a road segment is encountered.
			found |= FindIntrsctnPoint( cPoint, *pSeg, cHeightThresh );

		}
	}

	return found;
} // end of FindPoint
//////////////////////////////////////////////////////////////////////////////
///\brief
/// 	Searches an Intersection For a give point.
///
///\remark 
/// 	The segments will be searched in the order they are in the vector.  If
///  the point is found on a road, then no other segments are searched.  If
///  the point is found on an intersection, then all corridor segments are
///  searched in order to find every corridor the point lies on.
///
/// Arguments:
///\param[in]	cPoint - geometric point for which to search
///\param[in]	id     - id of the intersection to search
///\param[in]   cHeightThresh - height threshold within which to look for points
///
///\return True if the point is found, false otherwise.
///
//////////////////////////////////////////////////////////////////////////////
bool	
CRoadPos::FindPointInIntrsctn(
    const CPoint3D&, 
	int id,
	const double cHeightThresh)
{
    return true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: FindRoadPoint (protected)
// 	Find the geometric point on the road segment.
// 	
// Remarks:  This function uses the linear approximation of the curve between
// 	the consecutive control points on the road.  
//
// 	If found, place the road, lane, distance, offset, and cntrl pnt 
// 	index in the parameters and return true.
//
// 	Note that If the point is found on the shoulder, then the lane is set to
// 	the closest lane.
//
// Arguments:
//	cPoint - geometric point for which to search
//	cRoadSeg - road segment to search
//  cHeightThresh - height threshold within which to look for points
//
// Returns: If point is found on the given road segment, then
//	the function returns true and the local variables are set.  If point
//	is not found, then the function returns false and changes nothing.
//
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::FindRoadPoint(
			const CPoint3D& cPoint,
		 	const TSegment& cRoadSeg,
			const double cHeightThresh
			) 
{
	
	TCntrlPnt*	pCurCP;
	TCntrlPnt*	pNexCP;
	int 		cntrlPntIdx;

	double 		prevT = 0.0;	// Used to determine if the point
								//  lies on the "no man's land"
								//	where t(i) > 1 and t(i+1) < 0 
	double 		ofs2;
	double		splineOfs;
	double 		tValue;
	int 		laneIdx;
	double 		upper, lower;
	bool 		found;

	int endCntrlPntIdx = cRoadSeg.endCntrlPntIdx;
	if (cRoadSeg.pRoad->cntrlPntIdx+cRoadSeg.pRoad->numCntrlPnt-1 != endCntrlPntIdx)
		endCntrlPntIdx++;


	// For each control point in the segment
	for (cntrlPntIdx = cRoadSeg.begCntrlPntIdx, 
		 pCurCP = BindCntrlPnt(cntrlPntIdx);
		 cntrlPntIdx < endCntrlPntIdx;
		 cntrlPntIdx++, pCurCP++) {

		pNexCP = pCurCP+1;

		// Check to see if point is far above or below the 
		// 	current cntrl pnt
		

		if (  !((cPoint.m_z > (pCurCP->location.z - 0.5*cHeightThresh)) &&
				(cPoint.m_z < (pNexCP->location.z + 0.5*cHeightThresh))
				||
				(cPoint.m_z < (pCurCP->location.z + 0.5*cHeightThresh)) &&
				(cPoint.m_z > (pNexCP->location.z - 0.5*cHeightThresh))) ){
			continue;
		}
		
		

		// Create a line segment between the current cntrl pnt and the
		// 	next one on the road.
		CLineSeg2D lineSeg(pCurCP->location.x, 
						   pCurCP->location.y, 
						   pNexCP->location.x, 
						   pNexCP->location.y);

		// Find the offset squared of point from lineSeg, as well as the 
		// 	t-value for the projection of point onto lineSeg.
		ofs2 = lineSeg.PerpDistSquare(cPoint.m_x, 
									  cPoint.m_y,
									  &tValue);

		// if tValue < 0 and we're on the first control point, calculate the
		// t-value of the previous control point (if one exists).  This is needed
		// for when the "no man's land" is between two road segments
		if ((tValue-cCV_ZERO < 0.0f) && (cntrlPntIdx == cRoadSeg.begCntrlPntIdx)) {
			if (cRoadSeg.pRoad->cntrlPntIdx != cntrlPntIdx) {
				TCntrlPnt* pPrevCP = pCurCP - 1;
				CLineSeg2D prevSeg(pPrevCP->location.x,
								   pPrevCP->location.y,
								   pCurCP->location.x,
								   pCurCP->location.y);
				prevSeg.PerpDistSquare( cPoint.m_x, cPoint.m_y, &prevT );
			}
		}
	
		// If the tValue is on [0,1] or if it falls
		//	in the no man's land and
		//	if the offset falls within the width of the
		//	road + the shoulder, then the point lies on
		//	the road.
		//if ( ( (tValue >= 0.0) && (tValue <= 1.0) ) ||
		//	 ( (prevT > 1.0) && (tValue < 0.0) ) 
		//	 &&	 
		//	 (ofs2 < 0.5*(pCurCP->physicalWidth *
		//	  		 	  pCurCP->physicalWidth)) ) {

		if ( ( (tValue+cCV_ZERO >= 0.0f) && (tValue-cCV_ZERO <= 1.0f) ) ||
			 ( (prevT+cCV_ZERO > 1.0f) && (tValue-cCV_ZERO < 0.0f) ) 
			 &&	 
			 (ofs2 < 0.5*(pCurCP->physicalWidth *
			  		 	  pCurCP->physicalWidth)) ) {

			// Set all the member varibles.
			m_isRoad = true;
			m_cdo.clear();
			m_pRoad = cRoadSeg.pRoad;
			m_cntrlPntIdx = cntrlPntIdx;
	
			m_dist = pCurCP->cummulativeLinDist +
			   	 	pCurCP->distToNextLinear * tValue;
			cvTCntrlPnt*	ptCurCP;
			int 		cntrlPntIdx;

			double 		prevT = 0.0;	
			int endCntrlPntIdx;

			CSplineHermiteNonNorm m_spline;
			vector<TSegment> tSegments;
	
			CCrdr* cpCrdr = NULL;
			double maxWidth = 0;
			
			CPoint3D tPoint; //temp point
			CVector3D tNorm;
			CPoint2D m_RightOffset; //The control point for Crdr is in the middle of the lane, we need to adjust the position
							// to match the road control point being in middle of the road.

			//now lets get the points before and after our first point.
			int first_ctrlPnt = -1;
			int last_ctrlPnt = -1;
			int laneCnt  = cRoadSeg.pRoad->numOfLanes;	//if our lane cnt is 0 than the ctrl pnts are in middle of the road
			
			first_ctrlPnt = cRoadSeg.pRoad->cntrlPntIdx;
					
			last_ctrlPnt = cRoadSeg.pRoad->cntrlPntIdx + cRoadSeg.pRoad->numCntrlPnt;
					
								
            //make sure we have enough space before our pnts
			if (first_ctrlPnt <= int(cRoadSeg.begCntrlPntIdx) - 2){
					ptCurCP = BindCntrlPnt(cRoadSeg.begCntrlPntIdx - 2);
					m_spline.addPoint(ptCurCP->location.x, ptCurCP->location.y, ptCurCP->location.z );
					ptCurCP = BindCntrlPnt(cRoadSeg.begCntrlPntIdx - 1);
					m_spline.addPoint(ptCurCP->location.x, ptCurCP->location.y, ptCurCP->location.z );
			}else{
				if (first_ctrlPnt <= int(cRoadSeg.begCntrlPntIdx) - 1){
					//plus the last control point of our road segment
						ptCurCP = BindCntrlPnt(cRoadSeg.begCntrlPntIdx - 1);
						m_spline.addPoint(ptCurCP->location.x, ptCurCP->location.y, ptCurCP->location.z );
				}
							
			}
					
					
						
			//lets get the points for the best guess of the road segments we are on	
						
			endCntrlPntIdx = cRoadSeg.endCntrlPntIdx;
			for (cntrlPntIdx = cRoadSeg.begCntrlPntIdx; cntrlPntIdx < endCntrlPntIdx; cntrlPntIdx++) {
					ptCurCP = BindCntrlPnt(cntrlPntIdx);
					if (ptCurCP->physicalWidth > maxWidth)
						 maxWidth = ptCurCP->physicalWidth;
					m_spline.addPoint(ptCurCP->location.x, ptCurCP->location.y, ptCurCP->location.z );
			}
					
			
					 //now we need +2 after the best guess segments
			if ( last_ctrlPnt > (int)cRoadSeg.endCntrlPntIdx + 2){
					ptCurCP = BindCntrlPnt(cRoadSeg.endCntrlPntIdx);
            		m_spline.addPoint(ptCurCP->location.x, ptCurCP->location.y, ptCurCP->location.z );

					ptCurCP = BindCntrlPnt(cRoadSeg.endCntrlPntIdx + 1);
					m_spline.addPoint(ptCurCP->location.x, ptCurCP->location.y, ptCurCP->location.z );
			}else{
			//are we going from road to crdr, or crdr to road?
				 if (last_ctrlPnt > (int)cRoadSeg.endCntrlPntIdx){
						//plus the last control point of our road segment
						ptCurCP = BindCntrlPnt(cRoadSeg.endCntrlPntIdx);
				 }	
			}
			double splineOfs2;
			CCubicSplinePos pos;
			CPoint3D splinePoint;
			splineOfs = -9999999;
			splineOfs2 = -9999999;;
			//we need at least 4 points for a good spline
			m_spline.calc();
			ofs2 = sqrt(ofs2);
			if (m_spline.getCount() > 3 && m_spline.locate(cPoint.m_x,cPoint.m_y,cPoint.m_z,maxWidth+5.0f,pos))
			{
				m_spline.eval(pos,splinePoint);
				//now lets get the distance between or spline projection, and our current location
				double xDiff = cPoint.m_x - splinePoint.m_x;
				double yDiff = cPoint.m_y - splinePoint.m_y;
				double zDiff = cPoint.m_z - splinePoint.m_z;
				splineOfs = sqrt(xDiff*xDiff + yDiff*yDiff /*+ zDiff*zDiff*/);
			}else{
				splineOfs = ofs2;
			}
				 
			CVector3D rightVector(pCurCP->rightVecLinear);
			CVector3D vertVector(cPoint.m_x - pCurCP->location.x, 
							 	cPoint.m_y - pCurCP->location.y, 
							 	cPoint.m_z - pCurCP->location.z); 
			if (rightVector.DotP(vertVector) < 0){
						ofs2 *= -1;
						splineOfs*= -1;

			}
				
			// Calculate the width of the shoulder.  Use this to extend 
			//	the width of the extreme right and left lanes.
			double shoulderWidth = 0.5*(pCurCP->physicalWidth - 
									   pCurCP->logicalWidth);
	
			// Calculate the lane on which point lies
			m_pLane = BindLane(pCurCP->laneIdx);
			laneIdx = 0; 
			found = false;
			do {
				upper = m_pLane->offset + m_pLane->width*0.5;
				lower = m_pLane->offset - m_pLane->width*0.5;
	
				// If the current lane is the leftmost or rightmost
				//	lane on the road, add the calculated shoulder width
				//	to the upper or lower bound.
				if ( (laneIdx == 0) || 
				 	 (laneIdx == pCurCP->numLanes-1) ) {
					if (upper > 0)
						upper += shoulderWidth;
					if (lower < 0)
						lower -= shoulderWidth;
				}
	
				// If the offset is within [lower, upper), then
				// 	the lane has been found.
				if ( (ofs2 >= lower) &&
				 	 (ofs2 < upper) ) {
	
					found = true;
	
					// Find m_ofs within the lane
					if (m_pLane->direction == ePOS){
						m_ofs = ofs2 - m_pLane->offset;
					} else {
						m_ofs = m_pLane->offset - ofs2;
			 		}
				}
				else {
					laneIdx++;
					m_pLane++;
				}
	
			} while ( (laneIdx < pCurCP->numLanes) &&
				  	  (!found) );

			// If the lane was not found, then the point is not
			// 	on the road.  I don't think this will ever happen.
			if (!found) {
				m_pSplineLane = 0;
				m_pLane = 0;
				m_pRoad = 0;
				return false;
			}

				// Calculate the lane on which point lies
			m_pSplineLane = BindLane(pCurCP->laneIdx);
			laneIdx = 0; 
			found = false;
			do {
				upper = m_pSplineLane->offset + m_pSplineLane->width*0.5;
				lower = m_pSplineLane->offset - m_pSplineLane->width*0.5;
	
				// If the current lane is the leftmost or rightmost
				//	lane on the road, add the calculated shoulder width
				//	to the upper or lower bound.
				if ( (laneIdx == 0) || 
				 	 (laneIdx == pCurCP->numLanes-1) ) {
					if (upper > 0)
						upper += shoulderWidth;
					if (lower < 0)
						lower -= shoulderWidth;
				}
	
				// If the offset is within [lower, upper), then
				// 	the lane has been found.
				if ( (splineOfs >= lower) &&
				 	 (splineOfs < upper) ) {
	
					found = true;
	
					// Find m_ofs within the lane
					if (m_pLane->direction == ePOS){
						m_splineOfs = splineOfs - m_pSplineLane->offset;
					} else {
						m_splineOfs = m_pSplineLane->offset - splineOfs;
			 		}
				}
				else {
					laneIdx++;
					m_pSplineLane++;
				}
	
			} while ( (laneIdx < pCurCP->numLanes) &&
				  	  (!found) );
		
			// If the lane was not found, then the point is not
			// 	on the road.  I don't think this will ever happen.
			if (!found) {
				m_pSplineLane = 0;
				return false;
			}
			else return true;
	
		} // If the point lies on the road

		prevT = tValue;

	} // For each control point in the segment
	
	return false;

} // end of FindRoadPoint

//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Find the geometric point on the intrsctn segment.(protected)
///\remark
///     This function uses the linear approximation of the curves 
/// 	created by the corridor control points.
///\remark
/// 	If found, place the intrsctn, crdr, distance, offset, and cntrl pnt 
/// 	index in the parameters and return true.
///
///
///\param[in]	cPoint - geometric point for which to search
///\param[in]	cIsecSeg - intrsctn segment to search
///\param[in]   cHeightThresh - height threshold within which to look for points
///
//\return If point is found on the given intrsctn segment, then
///	the function returns true and the local variables are set.  If point
///	is not found, then the function returns false and changes nothing.
///
//////////////////////////////////////////////////////////////////////////////
bool
CRoadPos::FindIntrsctnPoint(
			const CPoint3D& cPoint,
		 	const TSegment& cIsecSeg,
			const double cHeightThresh
			) 
{
	
    double ofs;                        
	double tValue;                     
	
	TCrdrPnt*	pCurCP;
	TCrdrPnt*	pNextCP;
	unsigned int cntrlPntIdx;
	double prevT = 0.0;		// Used to determine if the point
							//  lies on the "no man's land"
							//	where t(i) > 1 and t(i+1) < 0 
    const double cAtCtrlPoint = 0.1; //in feet -tolerence for our "no man's land"
	
    // For each control point pair on the corridor
	for (cntrlPntIdx = cIsecSeg.begCntrlPntIdx, 
		 pCurCP = BindCrdrPnt(cntrlPntIdx);
		 cntrlPntIdx < cIsecSeg.endCntrlPntIdx;
		 cntrlPntIdx++, pCurCP++) {

		pNextCP = pCurCP + 1;
		// Check to see if point is far above or below the cntrl pnt
		double zout;
		CVector3D norm;
		
		((CCved&)GetCved()).QryTerrainIntersection(cIsecSeg.pIntrsctn, cPoint, zout, norm, NULL, NULL);

		if ( (cPoint.m_z < zout - 0.5*cHeightThresh) ||
			 (cPoint.m_z > zout + 0.5*cHeightThresh) ) {
			continue;
		}

		// Create a line segment between the current 
		// 	cntrl pnt and the next one on the corridor.
		CLineSeg2D lineSeg(pCurCP->location.x, 
						   pCurCP->location.y,
						   pNextCP->location.x,
						   pNextCP->location.y);
	
		// Find the offset squared of point from lineSeg, 
		// 	as well as the t-value for the projection of 
		// 	point onto lineSeg.
		ofs = lineSeg.PerpDistSquare(cPoint.m_x, 
									 cPoint.m_y,
								  	 &tValue);
        
		// If the tValue is on [0,1], or if the point
		//	lies on the no-man's land between segments, 
		//	put point on the current control point.
		//if ( ((tValue >= 0.0) && (tValue <= 1.0)) ||
		//     ((prevT  >  1.0) && (tValue <  0.0)) ) {
        double tolerance = cAtCtrlPoint/(pNextCP->distance - pCurCP->distance);

		if ( ((tValue+tolerance >= 0.0) && (tValue-tolerance <= 1.0)) ||
		     ((prevT+tolerance  >  1.0) && (tValue-tolerance <  0.0)) ) {

			ofs = sqrt(ofs);

			// If the point is inside the width of the corridor, 
			// 	and tValue is between [0, 1], then the point lies 
			// 	on this corridor, near this control point.
			if (ofs < 0.5*pCurCP->width) {
			
				// Set all the output varibles.
				m_isRoad = false;
				m_pIntrsctn = cIsecSeg.pIntrsctn;
				TCdo tmpCdo;
				tmpCdo.pCrdr = cIsecSeg.pCrdr;
				tmpCdo.cntrlPntIdx = cntrlPntIdx;
		
				tmpCdo.dist = pCurCP->distance +
			   		(pNextCP->distance-pCurCP->distance) * tValue;
				if (lineSeg.IsPtOnRightSide(cPoint)){
					tmpCdo.ofs = ofs;
				} else {
					tmpCdo.ofs = -ofs;
				}

				if (tmpCdo.pCrdr->intrsctnId == m_pIntrsctn->myId){
					// Add tmpCdo to list of corridors
					m_cdo.push_back(tmpCdo);
				}

				return true;

			} // If within width of corridor

		} // If tValue is on [0,1] or in no man's land.

		prevT = tValue;

	} // For each control point pair

	return false;

} // end of FindIntrsctnPoint
//////////////////////////////////////////////////////////////////////////////
///\brief
///		This function gives a CCrd priority over other CCrd's that it is on
///\remark
///		This function sorts through m_crds if we are on a Crd, and if we are
///		and it finds the passed in corridor is in m_crds, it places it at the 
///		element 0 in m_crds
///\par
///		This function is mostly a work around, due to alot of procedures just
///		use the first corridor avialible and do not check to see if its on 
///		their path
///
///\return	true if the crd passed in was founds
///		false the roadpos is currently not on give corridor
//////////////////////////////////////////////////////////////////////////////
bool CRoadPos::SetCrdPriority(const CCrdr* cpCrdr){
	if (!cpCrdr)
		return false;
	int targId = cpCrdr->GetId();
    return SetCrdPriority(targId);
	return false; //should be a dead line.....
}
//////////////////////////////////////////////////////////////////////////////
///\brief
///		This function gives a CCrd priority over other CCrd's that it is on
///\remark
///		This function sorts through m_crds if we are on a Crd, and if we are
///		and it finds the passed in corridor is in m_crds, it places it at the 
///		element 0 in m_crds
///\par
///		This function is mostly a work around, due to alot of procedures just
///		use the first corridor avialible and do not check to see if its on 
///		their path
///
///\return	true if the crd passed in was founds
///		false the roadpos is currently not on give corridor
//////////////////////////////////////////////////////////////////////////////
bool CRoadPos::SetCrdPriority(int crdrId){
	int targId = crdrId;
	if (m_isRoad)
		return false; //we are on a road..........
	if (m_cdo.size() == 0)
		return false; //something bad.....
	else if (m_cdo.size() > 1){
		if (m_cdo[0].pCrdr->myId == targId){
			return true; //our first element is our target.......... 
		}
		//now we need to see if we have the element
		//unsigned int cnt = m_cdo.size();
		vector<TCdo>::iterator itr;
		itr = m_cdo.begin();

		for (; itr !=  m_cdo.end(); itr++){
			if (itr->pCrdr->myId == targId)
				break;
		}
		if (itr == m_cdo.end())
			return false; //we do not have the target.....
		TCdo tempCdo = *itr; //make a temp copy of cdo
		m_cdo.erase(itr); //remove the old cdo
		m_cdo.insert(m_cdo.begin(),tempCdo);
		
	}
	return false; //should be a dead line.....
}
bool CRoadPos::SetCrdPriorityByDir(CCrdr::ECrdrDirection dir){
	
   int targetDir =  -2;
   //eLEFT, eSTRAIGHT, eRIGHT 

   if (dir == CCrdr::eLEFT)
	   targetDir = cCV_CRDR_LEFT_DIR; 
   if (dir == CCrdr::eSTRAIGHT)
	   targetDir = cCV_CRDR_STRAIGHT_DIR; 
   if (dir == CCrdr::eRIGHT)
	   targetDir = cCV_CRDR_RIGHT_DIR; 

	if (m_isRoad)
		return false; //we are on a road..........
	int targId = 0;
	if (m_cdo.size() == 0)
		return false; //something bad.....
	else if (m_cdo.size() == 1){
		if (targetDir != m_cdo[0].pCrdr->direction){
			return false;
		}else{
			return true;
		}
	}
	else if (m_cdo.size() > 1){
		if (m_cdo[0].pCrdr->direction == targetDir){
			return true; //our first element is our target.......... 
		}
		//now we need to see if we have the element
		//unsigned int cnt = m_cdo.size();
		vector<TCdo>::iterator itr;
		itr = m_cdo.begin();

		for (; itr !=  m_cdo.end(); itr++){
			if (itr->pCrdr->direction == targetDir)
				break;
		}
		if (itr == m_cdo.end())
			return false; //we do not have the target.....
		TCdo tempCdo = *itr; //make a temp copy of cdo
		m_cdo.erase(itr); //remove the old cdo
		m_cdo.insert(m_cdo.begin(),tempCdo);
		return true;
	}
    return true;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetSpeedLimit
//  Retrieves the limit on the current road and lane that has the given
//  id.
//
// Remarks: If the CRoadPos is invalid, this function will cause a failed
//  assertion.
//
// Arguments:
//	id - (optional) relative ID of a corridor. If the CRoadPos is on a
//	road, this parameter is ignored. Otherwise, if specified, the function
//	will look for a speed limit attribute in the destination road.
//
// Returns: If an attribute is found of SpeedLimits on the current road,
//  lane, and distance, then a double representing the speedlimit will be 
//  returned, a negative(-1) will be returned if there's no speedlimit
//  attribute.
//
//////////////////////////////////////////////////////////////////////////////
double
CRoadPos::GetSpeedLimit(int id) const
{

	if (m_isRoad){

		vector<CAttr> attrVec;

    	bitset<cCV_MAX_LANES> lanes;
    	lanes.set(m_pLane->laneNo);
		CRoad road = GetRoad();
		vector<CAttr> tmpVec;
		road.QryAttr(tmpVec, m_dist, m_dist);
		unsigned int i = 0;
		for(; i < tmpVec.size(); i++){
			if (tmpVec[i].GetLaneMask() & lanes.to_ulong()){
				attrVec.push_back(tmpVec[i]);
			}
		}
		if (attrVec.size() == 0){
			return -1;
		} else {
			i = 0;
			for(; i < attrVec.size(); i++){
				if (attrVec[i].GetId() == cCV_SPEED_LIMIT_ATTR){
					return attrVec[i].GetVal1();
				}
			}
		}


	} else {
		/*
        CIntrsctn intrsctn = GetIntrsctn();
        bitset<cCV_MAX_CRDRS> crdrs;
        vector<TCdo>::const_iterator itr;
        for (itr = m_cdo.begin(); itr != m_cdo.end(); itr++){
            crdrs.set(itr->pCrdr->myId - m_pIntrsctn->crdrIdx);
		}
        intrsctn.QryAttr(attrVec, crdrs.to_ulong(), m_cdo.begin()->dist);
		*/

		CCrdr crdr = GetCorridor(id);
		CRoad road = crdr.GetSrcRd();
		CLane lane = crdr.GetSrcLn();
		int laneId = lane.GetRelativeId();
		double dist = -1;
		if (lane.GetDirection() == ePOS){
			// make sure it won't past the road into intrsctn
			dist = road.GetLinearLength() - 1.0;
		} else {
			dist = 1.0;
		}
		CRoadPos rp(road, laneId, dist, 0);
		return rp.GetSpeedLimit();
			
	}

	return -1;
}

////////////////////////////////////////////////////////////////////////////////
////
//// Description: CalculateRoute
////  Creates a tree of all possible directions the driver can take and 
////	calculates the first or shortest route between two points
////	Uses a breadth-first search and does not use any greedy algorithms
////
//// Remarks: both the CRoadPos and the destination must be on a road
////
//// Remarks on optimizations: The first optimization implemented was to
////	keep track of how many times a route traveled away from the destination
////	point in a row. If the route travels away from the destination point
////	too many times, then the route finder will stop expanding upon that
////	route. In order to implement this, each node contains a counter that
////	increments every time a route travels away from the destination point
////	and resets every time a route travels towards the destination point. 
////	Whether or not the route travels towards or away from the destination 
////	point is decided by computing the linear distance to the destination
////	and comparing it with the previous linear distance to the destination.
////	This optimization has a significant effect on certain routes where there
////	are few paths that lead to the destination. In some cases, the route
////	calculation time dropped from several seconds to almost instantaneous.
////
////	The second optimization implemented deals with routes that contain
////	many nodes. As the number of nodes continues to grow, the calculation
////	time increases exponentially. The optimization periodically checks
////	the route tree when it becomes large and eliminates all nodes except
////	the closest several nodes. This implementation uses the functions
////	GetAllNodesAtLevel() and DiscardFarNodes(). It has no noticeable
////	effect on routes with a small number of nodes, but dramatically
////	decreases the calculation time of routes with 18+ nodes.
////
//// Arguments:
////	end	- destination point
////	out - vector holding route info (road index, lane id, crdr id, length)
////		for each segment of the route
////	totalDist - total distance of the route in feet
////	shortest - (default = false), flag to find the shortest route
////		note: finding shortest route takes significantly longer for complex routes
////	maxHeight - (default = 25), maximum height of the tree before giving up
////
//// Returns: true if route was found, otherwise false
////
////////////////////////////////////////////////////////////////////////////////
//bool
//CRoadPos::CalculateRoute(CRoadPos& end, vector<TRouteInfo>& out, double& totalDist, bool shortest, int maxHeight) {
//	
//	AssertValid();
//
//	if (!end.IsValid()) {
//		return false;
//	}
//
//	CRoad road = GetRoad();
//	CIntrsctn intr = GetLane().GetNextIntrsctn();
//	CRoad dest = end.GetRoad();
//
//	if (!road.IsValid() || !intr.IsValid() || !dest.IsValid()) {
//		return false;
//	}
//
//	// used to store valid routes if searching for shortest route
//	vector< vector<TRouteInfo> > routes;
//	vector<double> routeDistances;
//	
//	// set up head node
//	TRouteNode head;
//	head.head = true;
//	head.found = true;	// head always included in route
//	head.searched = false;
//	head.srcCrdrId = -1; // head has no source
//	head.srcLaneId = -1;
//	head.roadIdx = road.GetId();
//	head.roadDir = GetLane().GetDirection();
//	head.length = 0; // length is based on source lane and corridor
//
//	CPoint3D initLoc = GetVeryBestXYZ();
//	CPoint3D destLoc = end.GetVeryBestXYZ();
//	head.linearDistSquared = (initLoc.m_x - destLoc.m_x) * (initLoc.m_x - destLoc.m_x) +
//							 (initLoc.m_y - destLoc.m_y) * (initLoc.m_y - destLoc.m_y) +
//							 (initLoc.m_z - destLoc.m_z) * (initLoc.m_z - destLoc.m_z) ;
//	head.awayNodes = 0;
//
//	// trivial case where initial and destination are on the same road
//	// and the destination is ahead of the initial position
//	if (road.GetId() == dest.GetId() && GetLane().GetDirection() == end.GetLane().GetDirection() && GetDistanceOnLane() < end.GetDistanceOnLane()) {
//		TRouteInfo info;
//		info.roadIdx = dest.GetId();
//		info.crdrId = -1;
//		info.laneId = end.GetLane().GetRelativeId();
//		totalDist = end.GetDistanceOnLane() - GetDistanceOnLane();
//		info.length = totalDist;
//		out.push_back(info);
//		return true;
//	}
//	
//	// the maxAway counter is the max number of times a route can move away from the destination point in a row
//	for(int maxAway=1; maxAway<=8; maxAway*=2) {
//		// resets the tree if a route was not found
//		head.children.clear();
//		head.searched = false;
//
//		totalDist = 0; // keeps track of total route distance
//		int height = 1;	// height of the route tree
//		bool done = false; // keeps track of whether a route or set of routes has been found
//
//		while(height <= maxHeight){
//			// check if a valid route was found
//			if (GetRouteChildren(road.GetId(), GetLane().GetDirection(), end, height, &head , maxAway)){
//
//				TRouteNode* ptr = &head;
//				vector<TRouteInfo> temp;
//
//				// the first node's length is calculated differently from all other nodes
//				bool first = true;
//					
//				// store route data
//				while(true) {
//					// get the first node in the route (after the head)
//					bool found = false;
//					for(int i=0; i<ptr->children.size(); i++) {
//						if (ptr->children[i].found) {
//							ptr = &ptr->children[i];
//							found = true;
//							break;
//						}
//					}
//
//					if (found) {
//						// set up a TRouteInfo struct and store
//						// important data from the TRouteNode
//						TRouteInfo info;
//						info.roadIdx = ptr->parent->roadIdx;
//						info.crdrId = ptr->srcCrdrId;
//						info.laneId = ptr->srcLaneId;
//
//						// adjust length of first segment since the initial position
//						// starts in the middle of the road
//						info.length = first ? ptr->length - GetDistanceOnLane() : ptr->length;
//
//						// add segment length to the total distance
//						totalDist += info.length;
//
//						if (!shortest) {
//							out.push_back(info);
//						} else {
//							ptr->found = false;
//							temp.push_back(info);
//						}
//
//						// need to change a few things for the last node
//						if (ptr->children.size() == 0) {
//							TRouteInfo last;
//							last.roadIdx = dest.GetId();
//							last.crdrId = -1;	// no corridor
//							last.laneId = end.GetLane().GetRelativeId();
//							last.length = end.GetDistanceOnLane();	// does not include corridor
//
//							totalDist += last.length;
//						
//							if (!shortest) {
//								out.push_back(last);
//							} else {
//								temp.push_back(last);
//								routes.push_back(temp);
//								routeDistances.push_back(totalDist);
//
//								// reset distance tracker
//								totalDist = 0;
//
//								// since a route has been found, we don't have to search much more
//								// this is not strictly necessary, but it will speed up things by a lot
//								// this is used only if calculating the shortest route
//								if (maxHeight > height + 2 && height < 10) {
//									maxHeight = height + 2;
//								} else if (maxHeight > height + 1 && height < 15) {
//									maxHeight = height + 1;
//								} else {
//									maxHeight = height;
//								}
//							}
//							break;
//						}
//					} else {
//						// should not happen--occurs when GetRouteChildren() returns true & either the route was constructed improperly or there is no route
//						// add a breakpoint to debug GetRouteChildren()
//						return false;
//					}
//
//					first = false;
//				}
//
//				// if not searching for the shortest route, return the output
//				if (!shortest) {
//					return true;
//				} else {
//					// otherwise we need to find if there are any other routes with the same number of segments
//					// however, we do not need to remake the tree with a higher max away node limit
//					done = true; 
//					continue; 
//				}
//
//			}
//
//			// no route found; increment the height of the tree
//			height++;
//
//			// periodically scan the tree for nodes that are too far away from the destination and discard these nodes
//			if (height % 8 == 0) {
//				std::vector<TRouteNode*> nodes;
//				GetAllNodesAtLevel(&head, height, nodes); 
//				DiscardFarNodes(nodes, 8);
//			}
//		}
//
//		// if finding the shortest route and a route has been found, do not reset the route tree
//		if (done) {
//			break;
//		}
//	}
//
//
//	// if looking for shortest route, iterate through all routes and find the shortest one
//	if (shortest && routes.size() > 0) {
//		int shortestRoute = 0;
//		double min = routeDistances[0];
//		for(int i=1; i<routeDistances.size(); i++){
//			if (routeDistances[i] < min) {
//				shortestRoute = i;
//				min = routeDistances[i];
//			}
//		}
//
//		totalDist = min;
//		for(int i=0; i<routes[shortestRoute].size(); i++){
//			out.push_back(routes[shortestRoute][i]);
//		}
//		return true;
//	}
//
//	// if the function reaches this point then no routes were found and the optimizations probably need some fine-tuning
//	return false;
//}
//
////////////////////////////////////////////////////////////////////////////////
////
//// Description: GetRouteChildren
////  Searches the provided node and populates its children vector. If a child
////	leads to the destination road, the function will return true.
////
//// Arguments: 
////	roadIdx - initial road index
////	dir	- initial road direction
////	end - destination position
////	height - level of tree to search on
////	parent - pointer to node to search
////
//// Returns: true if valid route found, otherwise false
////
////////////////////////////////////////////////////////////////////////////////
//bool
//CRoadPos::GetRouteChildren(int roadIdx, cvELnDir dir, CRoadPos& end, int height, TRouteNode* parent, int maxAway){
//
//	// if the node has been searched, descend farther down the tree
//	if (parent->searched) {
//		for(int i=0; i<parent->children.size(); i++){
//			if (GetRouteChildren(parent->children[i].roadIdx, parent->children[i].roadDir, end, height-1, &parent->children[i], maxAway)) {
//				// found a valid route
//				return true;
//			}
//		}
//	} else if (height == 1){ // if the node is at the correct height and has not been searched
//		// prevents this node from being searched again
//		parent->searched = true;
//
//		TCrdrVec crdrVec;
//		CRoad road(GetCved(), roadIdx);
//		CIntrsctn intr;
//
//		// check which direction we are traveling and get the correct intersection
//		if (dir == eNEG) {
//			intr = road.GetSourceIntrsctn();
//		} else {
//			intr = road.GetDestIntrsctn();
//		}
//
//		// get all corridors
//		intr.GetCrdrsStartingFrom(road, crdrVec);
//
//		for(int i=0; i<crdrVec.size(); i++) {
//			if (!crdrVec[i].GetSrcLn().IsDrivingLane()) {
//				// cars cannot drive on this lane
//				continue;
//			}
//
//			// store route info
//			TRouteNode node;
//			node.head = false;
//			node.parent = parent;
//			node.found = false;
//			node.searched = false;
//			node.srcCrdrId = crdrVec[i].GetRelativeId();
//			node.srcLaneId = crdrVec[i].GetSrcLn().GetRelativeId();
//			node.roadIdx = crdrVec[i].GetDstntnRdIdx();
//			node.roadDir = crdrVec[i].GetDstntnLn().GetDirection();
//			node.length = crdrVec[i].GetSrcLn().GetRoad().GetLinearLength() + crdrVec[i].GetLength();
//
//			// calculate the linear distance from the current node to the destination point
//			TCntrlPnt* cntrlPnt;
//			if (maxAway != -1) {
//				// determine which side of the road to use (use either the beginning or the end)
//				if (dir == eNEG ) {
//					cntrlPnt = BindCntrlPnt(road.GetCntrlPntIdx());
//				} else {
//					cntrlPnt = BindCntrlPnt(road.GetCntrlPntIdx() + road.GetCntrlPntCount());
//				}
//
//				// since distance is used only for comparison, no need to square root
//				node.linearDistSquared = (cntrlPnt->location.x - end.GetVeryBestXYZ().m_x) * (cntrlPnt->location.x - end.GetVeryBestXYZ().m_x) +
//										 (cntrlPnt->location.y - end.GetVeryBestXYZ().m_y) * (cntrlPnt->location.y - end.GetVeryBestXYZ().m_y) +
//										 (cntrlPnt->location.z - end.GetVeryBestXYZ().m_z) * (cntrlPnt->location.z - end.GetVeryBestXYZ().m_z) ;
//
//				if (node.linearDistSquared > parent->linearDistSquared) { 
//					// increment awayNodes counter
//					node.awayNodes = parent->awayNodes + 1;
//
//					if (node.awayNodes > maxAway) {
//						node.searched = true; // prevent node from being searched again
//					}
//				} else {
//					node.awayNodes = 0; // reset awayNodes counter
//				}
//			} 
//			
//			// found a valid path
//			if (node.roadIdx == end.GetRoad().GetId() && node.roadDir == end.GetLane().GetDirection()) {
//				node.found = true;
//				node.searched = true;	// prevents this node from being searched again if searching for shortest route
//				parent->children.push_back(node);
//
//				// mark each parent node as part of the route
//				TRouteNode* ptr = parent;
//				while(!ptr->head) {
//					ptr->found = true;
//					ptr = ptr->parent;
//				}
//
//				return true;
//			}
//
//			// did not find valid path; add to parent's children
//			parent->children.push_back(node);
//		}
//	}
//
//	// unable to find a route from this node
//	return false;
//}
//
////////////////////////////////////////////////////////////////////////////////
////
//// Description: GetAllNodesAtLevel
////  Recursively descends the route tree until it reaches the specified level.
////	Once at the specified level the function populates the output vector
////	with the route nodes.
////
//// Arguments: 
////	parent - current node
////	level - level of the tree to search for
////	out - (output) vector containing a pointer to all nodes at the specified
////			level
////
////////////////////////////////////////////////////////////////////////////////
//void CRoadPos::GetAllNodesAtLevel(TRouteNode* parent, int level, std::vector<TRouteNode*>& out) {
//	if (level > 0) {
//		// descend
//		for(int i=0; i<parent->children.size(); i++){ 
//			GetAllNodesAtLevel(&parent->children[i], level-1, out);
//		}
//	} else {
//		// at the correct level
//		out.push_back(parent);
//	}
//}
//
////////////////////////////////////////////////////////////////////////////////
////
//// Description: DiscardFarNodes
////  Searches a vector of nodes and discards all but a user-specified number of
////	nodes. Nodes are discarded based on their linear distance to the
////	destination point.
////
//// Arguments: 
////	nodes - all nodes to search
////	numToKeep - number of nodes to keep
////
////////////////////////////////////////////////////////////////////////////////
//void CRoadPos::DiscardFarNodes(std::vector<TRouteNode*>& nodes, int numToKeep) {
//
//	// invalid inputs
//	if (numToKeep > nodes.size() || numToKeep < 1) {
//		return;
//	}
//
//	// indices of nodes to keep
//	std::vector<int> keep;
//	
//	// set default indices
//	for(int i=0; i<numToKeep; i++) {
//		keep.push_back(i) ;
//	}
//
//	// keep track of which index has the maximum distance
//	int max = 0;
//
//	// find the largest distance amongst the initial nodes
//	for(int i=1; i<keep.size()-1; i++) {
//		if (nodes[i]->linearDistSquared > nodes[max]->linearDistSquared) {
//			max = i;
//		}
//	}
//
//	// search all nodes for smaller distances
//	for(int i=numToKeep; i<nodes.size(); i++) {
//		if (nodes[i]->linearDistSquared < nodes[ keep[max] ]->linearDistSquared) {
//			//insert node index here
//			keep[max] = i;
//
//			// find new max
//			for(int j=0; j<keep.size(); j++) {
//				if ( nodes[ keep[j] ]->linearDistSquared > nodes[ keep[max] ]->linearDistSquared ) {
//					max = j;
//				}
//			}
//		}
//	}
//
//	// stop searching in nodes that are too far away
//	for(int i=0; i<nodes.size(); i++) {
//		nodes[i]->searched = true;
//	}
//	for(int i=0; i<numToKeep; i++) {
//		nodes[ keep[i] ]->searched = false;
//	}
//
//}

} // namespace CVED
