//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: pathnetwork.cxx,v 1.11 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Jillian Vogel
// Date:		October, 1999
//
// Description:	The implementation of the CPathNetwork class.  A CPathNetwork
// 	is an immutable collection of CPaths and their interconnections 
// 	over a specified area of the road network.  
//
//////////////////////////////////////////////////////////////////////////////

#include "cvedpub.h"
#include "cvedstrc.h"		// private CVED data structs

#define PATH_NETWORK_DEBUG 0

namespace CVED {

//////////////////////////////////////////////////////////////////////////////
//	Operator functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator<<
//  Prints the contents of the cPathNetwork to the ostream parameter.
//
// Remarks: The ostream&<< has to appear within the namespace, otherwise
//  it won't be properly associated.
//
// Arguments:
//  out - ostream to print to
//  cPathNetwork - CPathNetwork to print
//
// Returns: A reference to the ostream parameter so that the << operations
//  can be nested.
//////////////////////////////////////////////////////////////////////////////
ostream&
operator<<(ostream& out, const CPathNetwork& cPP) 
{
	// Print out distance covered
	out << "Distance covered: " << cPP.m_covered << endl;

	// Print out cPP.m_intrsctnPnts
	CPathNetwork::TIsecPointMap::const_iterator pI;
	out << "Intrsctn Points: " << endl;
	for (pI = cPP.m_intrsctnPnts.begin(); 
		 pI != cPP.m_intrsctnPnts.end(); pI++)
		out << "  " << pI->first << endl << "    " << pI->second << endl;
	
	// Print out cPP.m_roadPnts
	CPathNetwork::TRoadPointMap::const_iterator pR;
	out << endl << "Road Points: " << endl;
	for (pR = cPP.m_roadPnts.begin(); 
		 pR != cPP.m_roadPnts.end(); pR++) {

		out << "  " << pR->first << endl;
		out << "    POS: src = " << pR->second.posPoint.srcIntrsctnId
			<< ", dst = " << pR->second.posPoint.dstIntrsctnId << endl
			<< "    " << pR->second.posPoint.pathPoint << endl;
		out << "    NEG: src = " << pR->second.negPoint.srcIntrsctnId
			<< ", dst = " << pR->second.negPoint.dstIntrsctnId << endl
			<< "    " << pR->second.negPoint.pathPoint << endl;
	}
	return out;
} // end of operator<<

} // end namespace CVED

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
//	The assignment operator assigns the contents of the parameter to the 
//	current instance.
//
// Remarks: 
//
// Arguments: 
//	cRhs - a CPathNetwork to assign to the current instance
//
// Returns: A reference to the current instance to allow for nested assignment
//	statements.
//
//////////////////////////////////////////////////////////////////////////////
CPathNetwork&
CPathNetwork::operator=(const CPathNetwork& cRhs)
{
	if (this != &cRhs) {
		// Copy superclass data
		this->CCvedItem::operator=(cRhs);

		m_covered = cRhs.m_covered;
	} 
	return *this;	
} // end of operator=


//////////////////////////////////////////////////////////////////////////////
//		Constructors and destructor
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: CPathNetwork
// 	The default constructor initializes the local variables to default values.
//
// Remarks: This constructor creates an invalid CPathNetwork instance.
//
// Arguments:
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathNetwork::CPathNetwork()
	: CCvedItem(), 
	  m_covered(0.0f)
{} // end of CPathNetwork

//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CPathNetwork
// 	The default destructor frees the data allocated for the class.
//
// Remarks:
//
// Arguments:
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathNetwork::~CPathNetwork() 
{} // end of ~CPathNetwork

//////////////////////////////////////////////////////////////////////////////
//
// Description: CPathNetwork
// 	The copy constructor calls the assignment operator.
//
// Remarks:
//
// Arguments:
//	cCopy - CPathNetwork instance to copy to the current instance
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathNetwork::CPathNetwork(const CPathNetwork& cCopy)
{
	*this = cCopy;
} // end of CPathNetwork

///////////////////////////////////////////////////////////////////////////////
//
// Description: CPathNetwork
//  Initializes the CPathNetwork superclass with the CCved instance.
//
// Remarks: This constructor creates an invalid CPathNetwork instance.
//
// Arguments:
//	cCved - a valid CCved instance
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathNetwork::CPathNetwork(const CCved& cCved)
	: CCvedItem(&cCved), 
	  m_covered(0.0f)
{} // end of CPathNetwork
	
/////////////////////////////////////////////////////////////////////////////
//
// Description: CPathNetwork
//  Initializes the CPathNetwork instance with the roads and intersections 
//  that lie within a square with the given center and sides at 2*dist units.
//
// Remarks: 
//
// Arguments:
//	cCved - a valid CCved instance
//	center - a CPoint2D indicating the center of the rectangle to search
//	dist - a double value indicating the distance from center to search
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathNetwork::CPathNetwork(const CCved& cCved, 
						   const CPoint2D& center, 
						   double dist)
	: CCvedItem(&cCved)
{
	SetRange(center, dist);
} // end of CPathNetwork


//////////////////////////////////////////////////////////////////////////////
//	Functions
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: SetRange
//  Initializes the CPathNetwork instance with the roads and intersections 
//  that lie within a square with the given center and sides at 2*dist units.
//
// Remarks: 
//
// Arguments:
//	center - a CPoint2D indicating the center of the rectangle to search
//	dist - a double value indicating the distance from center to search
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPathNetwork::SetRange(const CPoint2D& center, 
					   double dist)
{
	// Clear out any previous data
	m_intrsctnPnts.clear();
	m_roadPnts.clear();

	// Issue calls to the CCved quadtrees to find out which roads and 
	// 	intersections lie within the square defined by center and 2*dist.
	double minX, minY, maxX, maxY;
	minX = center.m_x - dist;
	minY = center.m_y - dist;
	maxX = center.m_x + dist;
	maxY = center.m_y + dist;
	vector<int> rdPcIds;
	vector<int> isecIds;
	vector<int>::const_iterator pId;
	const CCved& cved = GetCved();
	cvTRoadPiece* rdPcPool = BindRoadPiece(0), *pRdPc;
	cvTRoad* roadPool = BindRoad(0), *pRoad;
	cvTCntrlPnt* cntrlPntPool = BindCntrlPnt(0), *pCntrlPnt;
	cvTLane* lanePool = BindLane(0), *pLane;
	TRoadPointMap::iterator pRdPnt;
	TIsecPointMap::iterator pIsecPnt;
	int laneId;
	double start, end;
	
	cved.SearchRdPcQuadTree(minX, minY, maxX, maxY, rdPcIds);
	cved.SearchIntrsctnQuadTree(minX, minY, maxX, maxY, isecIds);

	// For each intersection in the result, create a CPathPoint and
	// 	place it into m_intrsctnPnts.
	for (pId = isecIds.begin(); pId != isecIds.end(); pId++) {
		TIsecPointPair isecPoint;
		isecPoint.first = *pId;
		isecPoint.second.SetCved(&cved);
		isecPoint.second.m_isRoad = false;
		isecPoint.second.m_intrsctn = CIntrsctn(cved, *pId);
		isecPoint.second.m_laneCrdrMask.set();
		isecPoint.second.SetRangeToEnd(false, true);

		m_intrsctnPnts.insert(isecPoint);
	} // For each intersection

	// For each roadPiece in the result, connect it to other road pieces
	// 	on the same road by using the m_roadPnts map.  If no entry exists
	// 	for the current road id, then create one and initialize its 
	// 	pathPoints with the roadPiece.  If an entry already exists, extend 
	// 	the pathPoints with the roadPiece.
	for (pId = rdPcIds.begin(); pId != rdPcIds.end(); pId++) {

		pRdPc = &rdPcPool[*pId];
		pRoad = &roadPool[pRdPc->roadId];

		pCntrlPnt = &cntrlPntPool[pRdPc->first+pRoad->cntrlPntIdx];
		start = pCntrlPnt->cummulativeLinDist;

		pCntrlPnt = &cntrlPntPool[pRdPc->last+pRoad->cntrlPntIdx];
		end = pCntrlPnt->cummulativeLinDist;
		
		pRdPnt = m_roadPnts.find(pRoad->myId);

		// If there is no entry for this road
		if (pRdPnt == m_roadPnts.end()) {
	
			TRoadPointPair rdPnt;
			rdPnt.first = pRoad->myId;
			
			// Initialize the road for both rdPnt.second entries.
			rdPnt.second.posPoint.pathPoint.SetCved(&cved);
			rdPnt.second.posPoint.pathPoint.m_isRoad = true;
			rdPnt.second.posPoint.pathPoint.m_road = CRoad(cved, pRoad->myId);
			rdPnt.second.negPoint.pathPoint = rdPnt.second.posPoint.pathPoint;

			// Find the pos and/or neg lanes on this road
			for (laneId = 0, pLane = &lanePool[pRoad->laneIdx]; 
				 laneId < pRoad->numOfLanes; laneId++, pLane++) {

				// If the current lane is positive, add it to the lane
				// 	mask of rdPnt.second.posPoint
				if (pLane->direction == ePOS) {
					
					rdPnt.second.posPoint.pathPoint.m_laneCrdrMask.set(laneId);
					rdPnt.second.posPoint.pathPoint.m_startDist[laneId] = start;
					rdPnt.second.posPoint.pathPoint.m_endDist[laneId] = end;
				}
				// Else the current lane is negative, add it to the lane
				// 	mask of rdPnt.second.negPoint
				else {
					
					rdPnt.second.negPoint.pathPoint.m_laneCrdrMask.set(laneId);
					rdPnt.second.negPoint.pathPoint.m_startDist[laneId] = end;
					rdPnt.second.negPoint.pathPoint.m_endDist[laneId] = start;
				}
			}

			// Set the src/dstIntrsctnId if they are found in the
			// 	m_intrsctnPnts vector.
			pIsecPnt = m_intrsctnPnts.find(pRoad->srcIntrsctnIdx);
			if (pIsecPnt != m_intrsctnPnts.end()) {
				rdPnt.second.posPoint.srcIntrsctnId = pRoad->srcIntrsctnIdx;
				rdPnt.second.negPoint.dstIntrsctnId = pRoad->srcIntrsctnIdx;
			}
			pIsecPnt = m_intrsctnPnts.find(pRoad->dstIntrsctnIdx);
			if (pIsecPnt != m_intrsctnPnts.end()) {
				rdPnt.second.posPoint.dstIntrsctnId = pRoad->dstIntrsctnIdx;
				rdPnt.second.negPoint.srcIntrsctnId = pRoad->dstIntrsctnIdx;
			}

			// Add the entry into the map
			m_roadPnts.insert(rdPnt);
		}

		// Otherwise, there is an entry for this road
		else {

			// Extend the range of distance covered by the CPathPoints
			// 	in the entry, so that the range covered by the current
			// 	roadPiece is included.
			for (laneId = 0; laneId < 
					pRdPnt->second.posPoint.pathPoint.m_road.GetNumLanes(); 
				 laneId++) {

				// Adjust start/end dist on ePOS dirPoint
				if (start < 
					pRdPnt->second.posPoint.pathPoint.m_startDist[laneId]) {
					pRdPnt->second.posPoint.pathPoint.m_startDist[laneId] = 
						start;
				}
				if (end > 
					pRdPnt->second.posPoint.pathPoint.m_endDist[laneId]) {
					pRdPnt->second.posPoint.pathPoint.m_endDist[laneId] = 
						end;
				}		

				// Adjust start/end dist on eNEG dirPoint
				if (end < 
					pRdPnt->second.negPoint.pathPoint.m_startDist[laneId]) {
					pRdPnt->second.negPoint.pathPoint.m_startDist[laneId] = end;
				}
				if (start > 
					pRdPnt->second.negPoint.pathPoint.m_endDist[laneId]) {
					pRdPnt->second.negPoint.pathPoint.m_endDist[laneId] = start;
				}		
			}
		}
		
	} // For each roadPiece

	// Compute m_covered
	TRoadPointMap::const_iterator pR;
	TIsecPointMap::const_iterator pI;
	m_covered = 0.0f;

	// For each roadPoint in the map, add its length to the set
	for (pR = m_roadPnts.begin(); pR != m_roadPnts.end(); pR++) {
		if (pR->second.posPoint.pathPoint.IsValid())
			m_covered += pR->second.posPoint.pathPoint.GetLength();
		else if (pR->second.negPoint.pathPoint.IsValid())
			m_covered += pR->second.negPoint.pathPoint.GetLength();
	}

	// For each CPathPoint and in the vector, add its length to the set
	for (pI = m_intrsctnPnts.begin(); pI != m_intrsctnPnts.end(); pI++)
		m_covered += pI->second.GetLength();

} // end of SetRange
	
//////////////////////////////////////////////////////////////////////////////
//	
// Description: GetNumObjs
// 	Returns the number of dynamic CVED objects which lie on the current
// 	CPathNetwork.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CPathNetwork instance.
//
// Arguments:
//
// Returns: The number of objects found on the network.
// 
//////////////////////////////////////////////////////////////////////////////
int
CPathNetwork::GetNumObjs(void) const
{
	AssertValid();

	TRoadPointMap::const_iterator pR;
	TIsecPointMap::const_iterator pI;
	set<int> objIds;
	vector<int> tmpObjIds;
	vector<int>::const_iterator pId;

	objIds.clear();

	// For each roadPoint in the map, add the objects to the set
	for (pR = m_roadPnts.begin(); pR != m_roadPnts.end(); pR++) {
		if (pR->second.posPoint.pathPoint.IsValid())
			pR->second.posPoint.pathPoint.GetObjectsOnPoint(tmpObjIds);
		if (pR->second.negPoint.pathPoint.IsValid())
			pR->second.negPoint.pathPoint.GetObjectsOnPoint(tmpObjIds);
	}

	// For each intrsctnPoint in the map, add the objects to the set
	for (pI = m_intrsctnPnts.begin(); pI != m_intrsctnPnts.end(); pI++)
		pI->second.GetObjectsOnPoint(tmpObjIds);

	// Add into set to remove duplicates
	for (pId = tmpObjIds.begin(); pId != tmpObjIds.end(); pId++)
		objIds.insert(*pId);

	return (int) objIds.size();
} // end of GetNumObjs

//////////////////////////////////////////////////////////////////////////////
//	
// Description: GetDistanceCovered
// 	Returns the total linear distance covered by the roads and intersections
// 	in the current CPathNetwork.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CPathNetwork instance.
//
// Arguments:
//
// Returns: A double value containing the total linear distance covered by the
// 	CPathNetwork.
// 
//////////////////////////////////////////////////////////////////////////////
double
CPathNetwork::GetDistanceCovered() const
{
	// Return the contents of the m_covered variable
	return m_covered;
} // end of GetDistanceCovered

//////////////////////////////////////////////////////////////////////////////
//	
// Description: GetPathsTowards
// 	Returns the CPaths in the current CPathNetwork that terminate at the given
// 	CRoadPos.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CPathNetwork instance.
//
// Arguments:
//	pathVec - Will contain the CPaths that terminate at the given CRoadPos.  
//		This vector is cleared before use.
//	laneCrdrVec - For each CPath in pathVec, will contain a vector of 
//		relative identifiers for the lanes/crdrs included in the first 
//		CPathPoint on the CPath.
//	roadPos - Position at which to terminate the CPaths.
//	
// Returns: The number of paths found.
// 
//////////////////////////////////////////////////////////////////////////////
int
CPathNetwork::GetPathsTowards(vector<CPath>& pathVec,
							  TLaneCrdrVec& laneCrdrVec,
							  const CRoadPos& roadPos) const
{
	pathVec.clear();
	laneCrdrVec.clear();
	if (!roadPos.IsValid())
		return 0;

	CPath path(GetCved());
	int srcIsec = -1;
	int dstRoad = -1;
	// If the roadPos lies on a road, 
	//	search within the m_roadPnts map
	if (roadPos.IsRoad()) {
		TRoadPointMap::const_iterator pRd;
		for (pRd = m_roadPnts.begin(); pRd != m_roadPnts.end(); pRd++) {

			if ( (pRd->second.posPoint.pathPoint.IsValid()) &&
				 (pRd->second.posPoint.pathPoint.Contains(roadPos)) ) {
				path.Prepend(pRd->second.posPoint.pathPoint);
				srcIsec = pRd->second.posPoint.srcIntrsctnId;
				dstRoad = pRd->first;
				break;
			}
			else if ( (pRd->second.negPoint.pathPoint.IsValid()) &&
					  (pRd->second.negPoint.pathPoint.Contains(roadPos)) ) {
				path.Prepend(pRd->second.negPoint.pathPoint);
				srcIsec = pRd->second.negPoint.srcIntrsctnId;
				dstRoad = pRd->first;
				break;
			} 
		}
	}

	// Else, the roadPos lies on an intersection, 
	// 	so search within m_intrsctnPnts map
	else {

		TIsecPointMap::const_iterator pIs;
		for (pIs = m_intrsctnPnts.begin(); 
			 pIs != m_intrsctnPnts.end(); pIs++) {
			
			if (pIs->second.Contains(roadPos)) {
				srcIsec = pIs->first;
				break;
			}
		}
	}
	
	// Call recursive function to get all paths headed towards roadPos
	if ( (!RecursiveTowards(srcIsec, dstRoad, path, pathVec)) &&
		 (path.Size() > 0) )
		 pathVec.push_back(path);

#if PATH_NETWORK_DEBUG
	vector<CPath>::const_iterator pP;
	gout << pathVec.size() << " paths found." << endl;
	for (pP = pathVec.begin(); pP != pathVec.end(); pP++)
		gout << "  " << *pP << endl;
#endif

	// Initialize laneCrdrVec vector
	vector<CPath>::const_iterator pPath;
	vector<int> laneCrdrs;
	for (pPath = pathVec.begin(); pPath != pathVec.end(); pPath++) {

		laneCrdrs.clear();
		CPath::cTPathIterator pPoint = pPath->Begin();
		for (int i = 0; i < cCV_MAX_CRDRS; i++) {

			if (pPath->m_points[pPoint].m_laneCrdrMask.test(i))
				laneCrdrs.push_back(i);
		}

		laneCrdrVec.push_back(laneCrdrs);
	}
	
	return (int) pathVec.size();
} // end of GetPathsTowards


//////////////////////////////////////////////////////////////////////////////
//	Private helper functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: RecursiveTowards (private)
// 	This is a recursive helper function for GetPathsTowards.  
//
// Remarks: This function creates a vector of CPaths starting from the 
// 	curPath and srcIsec, and terminating at the end of the CPathNetwork.
//
// Arguments:
// 	srcIsec - identifier of the source intersection of the current road.
//		If this value is -1, it indicates that the source isec is not in
//		the CPathNetwork, and therefore the recursion terminates.
// 	dstRoad - identifier of the destination road of the srcIsec.  If this
// 		value is -1, then all corridors in the srcIsec are used, instead of
// 		only the ones terminating at dstRoad.
//	curPath - current CPath to prepend successive CPathPoints to.
//	pathVec - cumulative vector of CPaths radiating from the original curPath
//
// Returns: True if the curPath parameter was modified (i.e. found to connect
// 	to something), false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool 
CPathNetwork::RecursiveTowards(int srcIsec, int dstRoad,
							   CPath& curPath,
							   vector<CPath>& pathVec) const
{
	// BASE: unknown srcIsec
	TIsecPointMap::const_iterator pIsMap;
	pIsMap = m_intrsctnPnts.find(srcIsec);
	if (pIsMap == m_intrsctnPnts.end()) 
		return false;

	// RECURSIVE: valid srcIsec
	else {

		CIntrsctn isec = pIsMap->second.GetIntrsctn();

		// If the current path already contains the
		//	given intersection, return.
		if (AlreadyContains(curPath, pIsMap->second.m_intrsctn))
			return true;

		// Prepend the srcIsec
		if (!curPath.Prepend(pIsMap->second))
			return false;

#if PATH_NETWORK_DEBUG
		gout << curPath.Size() 
			 << " elements in curPath." << endl;
		gout << "  " << curPath << endl << endl;;
#endif

		// Before prepending to the current path, 
		// 	store a copy of it, including the current
		// 	intersection, so that each possible 
		// 	route may be found
		CPath origPath = curPath;
		
		// Get all connecting roads in srcIsec
		vector<CRoad> roads;
		vector<CRoad>::const_iterator pRd;

		// If there was a valid dstRoad passed, use it
		if (dstRoad > 0) {
			CRoad rd(GetCved(), dstRoad);
			isec.GetReverseRoadAccessListThrghIntrsctn(rd, roads);
		}

		// Otherwise, get all roads accessible through srcIsec
		else 
			isec.GetRoads(roads);

		// For each connecting road from srcIsec
		TRoadPointMap::const_iterator pRdMap;
		bool first = true;
		for (pRd = roads.begin(); pRd != roads.end(); pRd++) {
			
			// If the path already contains the given road, 
			//	then continue onto the next road
			if (AlreadyContains(curPath, *pRd))
				continue;

			// If the road exists in the map
			pRdMap = m_roadPnts.find(pRd->GetId());
			if (pRdMap != m_roadPnts.end()) {
	
				// Figure out whether the corridor connects to the positive,
				// 	the negative, or both types of lanes.
				vector<CCrdr> crdrs;
				vector<CCrdr>::const_iterator pCr;
				isec.GetCrdrsStartingFrom(*pRd, crdrs);
				bool pos = false, 
				 	 neg = false;
				cvTLane* pLanePool = BindLane(0);
				
				// For each corridor originating at the current road
				for (pCr = crdrs.begin(); pCr != crdrs.end(); pCr++) {
	
					int srcLnIdx = pCr->GetSrcLnIdx();
					// If the source lane is in the positive direction
					// 	and we haven't processed that direction yet
					if ( !pos && 
						 (pLanePool[srcLnIdx].direction == ePOS) ) {
						
						pos = true;
						
						if (!curPath.Prepend(
								pRdMap->second.posPoint.pathPoint))
							return false;
						
#if PATH_NETWORK_DEBUG
						gout << curPath.Size() 
							 << " elements in curPath." << endl;
						gout << "  " << curPath << endl << endl;;
#endif

						// If recuring with the current path is successful, 
						// 	push it back onto the pathVec
						if ( (RecursiveTowards(
								pRdMap->second.posPoint.srcIntrsctnId,
								pRdMap->first,
								curPath, 
								pathVec)) ||
								(first) ) {
							first = false;
							pathVec.push_back(curPath);
						}
					}

					// If the source lane is in the negative direction
					// 	and we haven't processed that direction yet
					else if ( !neg && 
							  (pLanePool[srcLnIdx].direction == eNEG) ) {
						
						neg = true;

						if (!curPath.Prepend(
								pRdMap->second.negPoint.pathPoint))
							return false;
						
						// If recuring with the current path is successful, 
						// 	push it back onto the pathVec
						if ( (RecursiveTowards(
								pRdMap->second.negPoint.srcIntrsctnId,
								pRdMap->first,
								curPath, 
								pathVec)) ||
								(first) ) {
							first = false;
							pathVec.push_back(curPath);
						}
					}

					// Otherwise, break out of loop
					else
						break;

				} // For each corridor originating at the current road

			} // If the road exists in the map

			// For the next iteration, reset curPath back to the original
			curPath = origPath;
			
		} // For each connecting road from srcIsec

		return true;
	} // RECURSIVE: valid srcIsec

} // end of RecursiveTowards

//////////////////////////////////////////////////////////////////////////////
//
// Description: AlreadyContains (private)
// 	This is a static helper function for RecursiveTowards.  
//
// Remarks: This function returns true if the given path already contains a 
//	node on the given intersection.
//
// Arguments:
//	path - a const CPath instance to check for isec
//	isec - a const CIntrsctn instance to search for in path
//
// Returns: True if the given path already contains a node on the given 
//	intersection, false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CPathNetwork::AlreadyContains(const CPath& path, const CIntrsctn& isec) 
{
	CPath::cTPathIterator itr;

	for (itr = path.Begin(); itr != path.End(); itr++) {
		if ( (!path.m_points[itr].m_isRoad) &&
			 (path.m_points[itr].m_intrsctn.GetId() == isec.GetId()) )
			return true;
	}
	return false;
} // end of AlreadyContains

//////////////////////////////////////////////////////////////////////////////
//
// Description: AlreadyContains (private)
// 	This is a static helper function for RecursiveTowards.  
//
// Remarks: This function returns true if the given path already contains a 
//	node on the given road.
//
// Arguments:
//	path - a const CPath instance to check for road
//	road - a const CRoad instance to search for in path
//
// Returns: True if the given path already contains a node on the given 
//	road, false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CPathNetwork::AlreadyContains(const CPath& path, const CRoad& road) 
{
	CPath::cTPathIterator itr;

	for (itr = path.Begin(); itr != path.End(); itr++) {
		if ( (path.m_points[itr].m_isRoad) &&
			 (path.m_points[itr].m_road.GetId() == road.GetId()) )
			return true;
	}
	return false;
} // end of AlreadyContains

} // namespace CVED
