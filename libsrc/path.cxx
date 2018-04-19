//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: path.cxx,v 1.92 2015/11/20 20:31:26 iowa\oahmad Exp $
//
// Author(s):	Jillian Vogel, Omar Ahmad
// Date:		October, 1999
//
// Description:	The implementation of the CPath class
//
//////////////////////////////////////////////////////////////////////////////

#include "cvedpub.h"
#include "cvedstrc.h"		// private CVED data structs

#define PATH_DEBUG	0
namespace CVED {
	
//////////////////////////////////////////////////////////////////////////////
//
// Description: Sends the contents of the current CPath to the ostream 
//   parameter.
//
// Remarks: The ostream&<< has to appear within the namespace, otherwise
//   it won't be properly assiciated.
//
// Arguments:
//   out - ostream instance to print to.
//   cPath - CPath instance to print.
//
// Returns: A reference to the ostream parameter so that the << operations
//  can be nested.
//
//////////////////////////////////////////////////////////////////////////////
ostream&
operator<<( ostream& out, const CPath& cPath )
{
	vector<string> pathStr;
	cPath.GetString( pathStr );

	vector<string>::const_iterator itr;
	for( itr = pathStr.begin(); itr != pathStr.end(); itr++ ) 
	{
		out << *itr << endl;
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
//		Constructors, destructor, and assignment operator
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: Default constructor does nothing.
//
// Remarks:  
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPath::CPath() 
	: 
	CCvedItem(), 
	m_curLaneId( -1 ), 
	m_pathPointIdx( 0 ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{

} // end of CPath

//////////////////////////////////////////////////////////////////////////////
//
// Description:	 Destructor does nothing.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
CPath::~CPath()
{

} // end of ~CPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: Copy constructor uses the assignment operator.
//
// Remarks:  
//
// Arguments:
// 	cCopy - CPath instance to copy to the current instance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPath::CPath( const CPath& cCopy )
{

	*this = cCopy;

} // end of CPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: Assignment operator performs a deep copy from the 
//   parameter to the current instance.
//
// Remarks:  
//
// Arguments:
// 	 cRhs - CPath on the right-hand side of the assignment.
//
// Returns: A reference to the current instance, so that the assignments can
// 	 be nested.
//
//////////////////////////////////////////////////////////////////////////////
CPath&
CPath::operator=( const CPath& cRhs ) 
{
	if ( this != &cRhs ) 
	{
		// Assign superclass items
		this->CCvedItem::operator=( cRhs );

		m_points.clear();
		m_points = cRhs.m_points;
		m_curLaneId = cRhs.m_curLaneId;
	  	m_pathPointIdx = cRhs.m_pathPointIdx;
		m_pRng = cRhs.m_pRng;
		m_rngStreamId = cRhs.m_rngStreamId;
	}

	return *this;

} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: CCved constructor initializes the CCvedItem superclass.
//
// Remarks:  This constructor creates an invalid CPath instance.  Use 
//	 Append/Prepend(const CPoint2D&),
//	 Append/Prepend(const CRoadPos&);
//	 Append/Prepend(const CPathPoint&); or
//	 SetString(const vector<string>&); to create initialize the path.
//
// Arguments:
//	 cCved - A valid CCved instance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPath::CPath( const CCved& cCved ) 
	: 
	CCvedItem( &cCved ), 
	m_curLaneId( -1 ), 
	m_pathPointIdx( 0 ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{

} // end of CPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: This constructor takes a CPoint3D instance as the 
//   starting point.
//
// Remarks: The CPoint3D is used to construct a CRoadPos, which is used to
// 	 construct a CPathPoint, which is added to the list.
//
// Arguments:
// 	 cCved - A reference to the CCved instance.
// 	 cPoint - 3D point near the road network.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPath::CPath( const CCved& cCved, const CPoint3D& cPoint )
	: 
	CCvedItem( &cCved ), 
	m_curLaneId( -1 ), 
	m_pathPointIdx( 0 ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{

	CRoadPos roadPos( cCved, cPoint );
	if( roadPos.IsValid() ) 
	{
		CPathPoint pathPoint( roadPos );
		if( pathPoint.IsValid() )  m_points.push_front( pathPoint );
	}

} // end of CPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: This constructor takes a CRoadPos instance as the 
//   starting point.
//
// Remarks: 
//
// Arguments:
// 	 cRoadPos - A road or intersection position on the road network.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPath::CPath( const CRoadPos& cRoadPos )
	: 
	CCvedItem( cRoadPos ), 
	m_curLaneId( -1 ), 
	m_pathPointIdx( 0 ),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{

	if( !cRoadPos.IsValid() )  return;
	
	CPathPoint pathPoint( cRoadPos );
	m_points.push_front( pathPoint );

} // end of CPath

//////////////////////////////////////////////////////////////////////////////
//		Accessor functions	
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Returns an iterator pointing to the beginning of the CPath.
//
// Remarks: The beginning of the CPath is represented by a CPathPoint.
//
// Arguments:
//
// Returns: A cTPathIterator at the beginning of the list of points.
//
//////////////////////////////////////////////////////////////////////////////
CPath::cTPathIterator
CPath::Begin() const
{

	return m_points.begin();

} // end of Begin

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Returns an iterator pointing to the end of the CPath.
//
// Remarks: The end of the CPath is represented by a CPathPoint.
//
// Arguments:
//
// Returns: A cTPathIterator at the end of the list of points.
//
//////////////////////////////////////////////////////////////////////////////
CPath::cTPathIterator
CPath::End() const
{

	return m_points.end();

} // end of End

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Returns the number of roads and intersections in the 
//  current CPath.
//
// Remarks:
//
// Arguments:
//
// Returns: An integer size of the current CPath.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::Size() const
{

	return m_points.size();

} // end of Size

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Returns whether or not the path contains any points
//
// Remarks:
//
// Arguments:
//
// Returns: True if the path contains 1 or more points, and false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::IsEmpty() const
{

	return m_points.empty();

} // end of IsEmpty

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the length covered from the parameter to the 
//  end of the current CPath.
//
// Remarks: The lengths of each CPointPath instance in the CPath are 
// 	accumulated to get the result.  If a CPathPoint has more than one lane
//  or corridor in it, the length of the lane/corridor with the lowest cved
//  id is used.  Note that the current CPath need not be valid to run this
//  function.
//
// Arguments:
//	cpStart - (Optional) Pointer to a valid CRoadPos instance.  Default
//      value is 0, indicating that the length of the entire path should 
//      be returned. 
//
// Returns: A double value indicating the length of the current CPath.
//
//////////////////////////////////////////////////////////////////////////////
double
CPath::GetLength( const CRoadPos* cpStart ) const 
{

	double length = 0.0;
	
	cTPathIterator itr = m_points.begin();

	if( cpStart ) 
	{
		//
		// Find the pathpoint corresponding to the cpStart parameter.
		//
		int startIdx = m_pathPointIdx;
		itr = FindPathPoint( *cpStart, startIdx );

		//
		// If FindPathPoint returned 0, then no corresponding
		// point was found, so return 0.0f.
		//
		if( itr == m_points.end() ) 
		{
			return 0.0f;
		}
		// 
		// Otherwise, get information from the parameter.
		//
		else 
		{
			// 
			// Find which lane or corridor the parameter lies on.
			//
			int laneCrdrId = -1;
			if( cpStart->IsRoad() ) 
			{
				laneCrdrId = cpStart->GetLane().GetRelativeId();
			}
			else 
			{
				laneCrdrId = cpStart->GetCorridor().GetRelativeId();
			}

			//
			// The initial length is affected by the road pos distance.
			// The distance to the end of the node from the current road pos
			// is | endDist - curDist |.
			//
			double endDist = m_points[itr].GetEndDist( laneCrdrId );
			double startDist = cpStart->GetDistance();
			length += fabs( endDist - startDist );

			// Add in lengths for the rest of the nodes.
			itr++;
		}
	}

	// 
	// Add up the lengths of the remaining points.
	//
	for(; itr != m_points.end(); itr++) 
	{
		length += m_points[itr].GetLength();
	}

	return length;

} // end of GetLength

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the length covered from the start parameter 
//  to the end parameter.
//
// Remarks: The lengths of each CPointPath instance in the CPath from cpStart
//  to cpEnd are accumulated to get the result.  If a CPathPoint has more than
//  one lane or corridor in it, the length of the lane/corridor with the
//  lowest cved id is used.  Note that the current CPath need not be valid to
//  run this function.
//
// Arguments:
//	cpStart - Pointer to a valid CRoadPos instance.
//  cpEnd   - Pointer to a valid CRoadPos instance located after cpStart.
//
// Returns: A double value indicating the length of the path segment.
//
//////////////////////////////////////////////////////////////////////////////
double
CPath::GetLength( const CRoadPos* cpStart, const CRoadPos* cpEnd ) const 
{

	double length = 0.0;
	
	//
	// Both arguments should be valid.
	//
	if( !cpStart || !cpEnd )  return length;

	//
	// Find the pathPoint corresponding to the cpStart parameter.
	//
	cTPathIterator startItr = m_points.begin();
	int startIdx = m_pathPointIdx;
	startItr = FindPathPoint( *cpStart, startIdx );

	//
	// Find the pathPoint corresponding to the cpEnd parameter.
	//
	cTPathIterator endItr = m_points.begin();
	int endIdx = m_pathPointIdx;
	endItr = FindPathPoint( *cpEnd, endIdx );

	//
	// If FindPathPoint returned 0, then no corresponding point was 
	// found, so return 0.0f.
	//
	if( startItr == m_points.end() || endItr == m_points.end() ) 
	{
		return 0.0f;
	}
	//
	// Otherwise, get information from the parameter.
	//
	else 
	{
		if( endIdx < startIdx ) 
		{
			length = -1.0;
		}
		else if( endIdx == startIdx ) 
		{
			length = cpEnd->GetDistanceOnLane() - cpStart->GetDistanceOnLane();
		}
		else 
		{
			// 
			// Find which lane or corridor the parameters lie on.
			//
			int startLaneCrdrId = -1;
			if( cpStart->IsRoad() ) 
			{
				startLaneCrdrId = cpStart->GetLane().GetRelativeId();
			}
			else 
			{
				startLaneCrdrId = cpStart->GetCorridor().GetRelativeId();
			}

			int endLaneCrdrId = -1;
			if( cpEnd->IsRoad() ) 
			{
				endLaneCrdrId = cpEnd->GetLane().GetRelativeId();
			}
			else 
			{
				endLaneCrdrId = cpEnd->GetCorridor().GetRelativeId();
			}

			// The initial length is affected by the road pos distance.
			// The distance to the end of the node from the current road pos
			// is | endDist - curDist |.
			double startDist = cpStart->GetDistance();
			double endDist = m_points[startItr].GetEndDist( startLaneCrdrId );
			length += fabs( endDist - startDist );

			// Add in lengths for the rest of the nodes.
			startItr++;

			// Add up the lengths of the in between points.
			for( ; startItr != endItr; startItr++ ) 
			{
				length += m_points[startItr].GetLength();
			}

			// Add the length of the last point
			startDist = m_points[endItr].GetStartDist( endLaneCrdrId );
            if (!cpEnd->IsRoad() && cpEnd->HasCorridor(endLaneCrdrId)){
                auto mask = m_points[endItr].GetLaneCrdrMask();
                if (mask.test(endLaneCrdrId)){
                    endDist   = cpEnd->GetDistance(endLaneCrdrId);
                }else{
                    //the path does not have the crdr, try to match the crdr
                    vector< pair<int,double> > crds;
                    cpEnd->GetCorridors(crds);
                    int idx = cpEnd->GetIntrsctn().GetCrdrIdx();
                    bool foundOverLap = false;
                    for (auto itr = crds.begin(); itr != crds.end(); itr++){
                        int id = itr->first-idx;
                        if (mask.test(id)){
                            endDist   = cpEnd->GetDistance(id); 
                            startDist = m_points[endItr].GetStartDist(id);
                            foundOverLap = true;
                            break;
                        }
                    }
                    if (!foundOverLap){
                        //our end point is not on our path.....return -1;
                        return -1.0;
                    }
                }
            }else{
                endDist   = cpEnd->GetDistance();
            }
			
			length += fabs( endDist - startDist );
		}
	}

	return length;

} // end of GetLength


//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the number of intersections covered from the 
//  parameter to the end of the path.
//
// Remarks: Note that the current CPath need not be valid to run this
//  function.
//
// Arguments:
//	cpStart - (Optional) Pointer to a valid CRoadPos instance.  Default value
//		is 0, indicating that the number of intersections in the entire path 
//		should be returned. 
//
// Returns: A int value indicating the number of intersections in the current
//	CPath.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetNumIntrsctns( const CRoadPos* cpStart ) const 
{
	int numIntrsctns = 0;
	
	cTPathIterator itr = m_points.begin();
	if( cpStart ) 
	{
		int curIdx = m_pathPointIdx;

		// Find the pathPoint corresponding to the parameter
		itr = FindPathPoint( *cpStart, curIdx );
	}

	for( ; itr != m_points.end(); itr++ )
	{
		if( !( m_points[itr].m_isRoad ) ) numIntrsctns++;
	}

	return numIntrsctns;
} // end of GetNumIntrsctns

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the number of roads covered from the parameter 
//  to the end of the path.
//
// Remarks: Note that the current CPath need not be valid to run this 
//  function.
//
// Arguments:
//	cpStart - (Optional) Pointer to a valid CRoadPos instance.  Default value
//		is 0, indicating that the number of roads in the entire path 
//		should be returned. 
//
// Returns: A int value indicating the number of roads in the current
//	CPath.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetNumRoads( const CRoadPos* cpStart ) const 
{
	int numRoads = 0;
	
	cTPathIterator itr = m_points.begin();
	if( cpStart ) 
	{
		int curIdx = m_pathPointIdx;

		// Find the pathPoint corresponding to the parameter
		itr = FindPathPoint( *cpStart, curIdx );
	}

	for ( ; itr != m_points.end(); itr++ )
	{
		if( m_points[itr].m_isRoad ) numRoads++;
	}

	return numRoads;
} // end of GetNumRoads

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the id of the current lane used by Travel to 
//   determine whether a lane change is needed.
//
// Remarks: The lane id is with respect to the current road.
//
// Arguments:
//
// Returns: An integer.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetCurLaneId() const 
{
	return m_curLaneId;
} // end of GetCurLaneId

///////////////////////////////////////////////////////////////////////////////
///
/// Description: GetString
/// 	Sets each element in the vector parameter to a string representing each 
/// 	CPathPoint in the path.
///
/// Remarks:  The format of each CPathPoint is as follows:
/// 	If the CPathPoint lies on a road:
/// 		R:Road_Name:Lane_Id[Start_dist:End_dist]:Lane_Id[Start_dist:...
/// 	Else if the CPathPoint lies on an intersection
/// 		I:Intrsctn_Name:Crdr_Id[Start_dist:End_dist]:Crdr_Id[Start_dist:...
///
/// Arguments:
/// 	pathStr - (Output) Upon return, will contain a string for each node in the
/// 		current path.
///
/// Returns: void
/// 	
///////////////////////////////////////////////////////////////////////////////
void
CPath::GetString(vector<string>& pathStr) const
{
	pathStr.clear();
	
	cTPathIterator itr;
	for (itr = m_points.begin(); itr != m_points.end(); itr++) {
		string point = m_points[itr].GetString();
		pathStr.push_back(point);
	}

} // end of GetString
///////////////////////////////////////////////////////////////////////////////
///
///\brief
///     Check every path point to see if it is valid
///\remark
///     This function differs from isValid, in that isValid just checks to make
///     sure the path has a good CVED reference, and other shallow checks. This 
///     is a deeper check to look at every path point
///
///\return true is every path point is valid
/// 	
///////////////////////////////////////////////////////////////////////////////

bool  
CPath::ValidatePath() const{
	cTPathIterator itr;
    cTPathIterator begin =  m_points.begin();
	bool isValid = true;
    vector<string> pathStr;
    GetString(pathStr);
    for (itr = begin; itr != m_points.end(); itr++) {
		if (!m_points[itr].IsValid() ){
            isValid = false;
            break;
        }
        if (itr != begin){
            //every road must be followed by a intersection, and vice versa
           if ( m_points[itr].IsRoad() == m_points[itr-1].IsRoad()){
               isValid = false;
               break;
           }
           //we must make sure each road is connected to the next intersection
           bool isConnected;
           isConnected = false;
           if (m_points[itr-1].IsRoad()){
               CVED::CRoad road = m_points[itr-1].GetRoad();
               auto mask = m_points[itr-1].GetLaneCrdrMask();
               for (int i = 0; i<mask.size(); i++){
                   if (mask.test(i)){ 
                       CLane currLane(road,i);
                       CIntrsctn intrer = m_points[itr].GetIntrsctn();
                       auto crdrMask = m_points[itr].GetLaneCrdrMask();
                       for (int j = 0; j < crdrMask.size(); j++){
                           if (crdrMask.test(j)){ 
                               CCrdr crdr(intrer,j);
                               if (crdr.GetSrcLn() == currLane){
                                   isConnected = true;
                                   break;
                               }
                           }
                       }
                       if (isConnected)
                           break;
                   }
               } 
           }//end if Road
           //we must make sure each intersection is connected to the next road
           else{
               CVED::CRoad road = m_points[itr].GetRoad();
               auto mask = m_points[itr].GetLaneCrdrMask();
               for (int i = 0; i<mask.size(); i++){
                   if (mask.test(i)){ 
                       CLane currLane(road,i);
                       CIntrsctn intrer = m_points[itr-1].GetIntrsctn();
                       auto crdrMask = m_points[itr-1].GetLaneCrdrMask();
                       for (int j = 0; j < crdrMask.size(); j++){
                           if (crdrMask.test(j)){ 
                               CCrdr crdr(intrer,j);
                               if (crdr.GetDstntnLn() == currLane){
                                   isConnected = true;
                                   break;
                               }
                           }
                       }
                       if (isConnected)
                           break;
                   }
               } 
           }//end if Road
           if (!isConnected){
            isValid =  false;
            break;
           }

        }

	}
    return isValid; 
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetStart
//  Gets the first RoadPos in the path.
//
// Remarks:	 Useful when using Travel() to get a starting position.
//
// Arguments: 
//	id - (Optional) Indicates the relative identifier of the lane or corridor
//		to place the returned CRoadPos on.  The default value is -1, indicating
//		that the first valid lane or corridor should be used.
//
// Returns:	 CRoadPos containing the first path position.  If there is no
//     first point, the CRoadPos will be invalid.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CPath::GetStart(int id) const
{
	cTPathIterator iter = Begin();
	return m_points[iter].GetRoadPos(0.0f, id);
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoadPos
//  Gets the specified RoadPos in the path.
//
// Remarks:	
//
// Arguments: 
//	num - index of the road position to retrieve
//	id - (Optional) Indicates the relative identifier of the lane or corridor
//		to place the returned CRoadPos on.  The default value is -1, indicating
//		that the first valid lane or corridor should be used.
//
// Returns:	 CRoadPos of the specified path position. If num is invalid, an
//		invalid CRoadPos will be returned.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CPath::GetRoadPos(int num, int id) const
{
	if (num < 0 || num >= m_points.size()) {
		return CRoadPos();
	}
	return m_points[num].GetRoadPos(0.0f, id);
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetEnd
//  Gets the last RoadPos in the path.
//
// Remarks:	 Used to get the CRoadPos of the end of the path
//
// Arguments: 
//	id - (Optional) Indicates the relative identifier of the lane or corridor
//		to place the returned CRoadPos on.  The default value is -1, indicating
//		that the first valid lane or corridor should be used.
//
// Returns:	 CRoadPos containing the last path position.  If there is no
//     last point, the CRoadPos will be invalid.
//
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CPath::GetEnd(int id) const
{
	return m_points[m_points.size()-1].GetRoadPos(1.0f, id);
}


//////////////////////////////////////////////////////////////////////////////
//		Mutator functions	
//////////////////////////////////////////////////////////////////////////////

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
CPath::SetRandNumGen( CRandNumGen* pRng, int streamId )
{
	m_pRng = pRng;
	m_rngStreamId = streamId;
}

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Initialize
// 	Sets the initial point of the CPath to the parameter, if it lies on a
//	valid road or intersection.
//
// Remarks: This function clears out any previous path information and sets
//	the initial point to the parameter. 
// 
//	If the current CPath was not created with a valid CCved instance, this 
//	function causes a failed assertion.
//
// Argument:
// 	point - a CPoint3D instance
//
// Returns: true if the point lies on a road or intersection and the 
//	 initialization was successful, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Initialize( const CPoint3D& cPoint )
{
	CRoadPos roadPos( GetCved(), cPoint );
	return Initialize( roadPos );
} // end of Initialize

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Initialize
// 	Sets the initial point of the CPath to the parameter, if it lies on a
//	valid road or intersection.
//
// Remarks: This function clears out any previous path information and sets
//	the initial point to the parameter. 
// 
//	If the current CPath was not created with a valid CCved instance, this 
//	function causes a failed assertion.
//
// Argument:
// 	cRoadPos - a CRoadPos instance
//
// Returns: true if the point lies on a road or intersection and the 
//	 initialization was successful, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Initialize( const CRoadPos& cRoadPos )
{
	CCvedItem::AssertValid();
	m_points.clear();
	if( cRoadPos.IsValid() ) 
	{
		CPathPoint pathPoint(cRoadPos);
		if( !pathPoint.IsValid() )  return false;

		m_points.push_front( pathPoint );
		return true;
	}
	else
	{
		return false;
	}
} // end of Initialize

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Randomly extends the end of the CPath by a given distance.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints.
//
// Argument:
//   distance - A double value to extend the path.
//   stayOnHighway - A boolean that indicates if append should stay on
//	     roads that have been marked as being highways.
//
// Returns: true if the extension was successful, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Append( double distance, bool stayOnHighway )
{
	/*static*/ CFastDeque<CPathPoint> extension;
	extension.clear();
	extension.Reserve(30);
	bool connected = false;

	if( IsValid() ) 
	{
		connected = m_points.back().Append( 
										distance, 
										m_pRng, 
										m_rngStreamId,
										extension, 
										stayOnHighway
										);

		//
		// If Append returned one or more points, append them to the 
		// current path.
		//
		if( connected )
		{
			cTPathIterator i;
			for( i = extension.begin(); i != extension.end(); i++ )
			{
				m_points.push_back( extension[i] );
			}
		}
	}

	return connected;
} // end of Append

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Appends the end of the CPath by a turn in the indicated 
//   direction at the next intersection.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints
//
//   If the Append was successful, the resulting CPath will run through the 
//   next intersection to the beginning of the next road.
//
// Argument:
//   turnAngle - Defines the clockwise angle (in radians) made between the 
// 	    direction of the current lane(s) and the destination lane(s).
// 	    Note some useful predefined constants can be used for this parameter:
// 	    cCV_STRAIGHT_TURN - go straight through the intersection
// 	    cCV_RIGHT_TURN - turn right
// 	    cCV_U_TURN - make a U-turn
// 	    cCV_LEFT_TURN - turn left 
// 	 epsilon - The +/- tolerance of the TurnAngle when finding the desired
//      destination lane(s).  Note that there is a predefined constant for 
//      this parameter: cCV_EPSILON = 1/8 * pi
//   dist - Extend the path only to the next intersection or the distance 
//      specified; whichever is smaller.
//  laneChangeAllowed - (optional, default true) If specified, determines
//      if path is only built from the current roadlane or if it's okay to 
//      to pick a corridor on the next intersection which requires a lane
//      change from teh current lane.
//
// Returns: The distance appended to the given path.
//
//////////////////////////////////////////////////////////////////////////////
double
CPath::Append( double turnAngle, double eps, double dist, bool laneChangeAllowed )
{
	/*static*/ CFastDeque<CPathPoint> extension;
	extension.clear();
	extension.Reserve(30);
	double distAppended = 0.0;

	if( IsValid() ) 
	{
		distAppended = m_points.back().Append( 
											turnAngle, 
											eps, 
											extension, 
											dist,
											laneChangeAllowed
											);

		// 
		// If Append returned one or more points, append them to the 
		// current path.
		//
		bool extend = distAppended > 0.0;
		if( extend )
		{
			cTPathIterator i;
			for( i = extension.begin(); i != extension.end(); i++ )
			{
				m_points.push_back( extension[i] );
			}
		}
	}

	return distAppended;
} // end of Append

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Appends the end of the CPath by connecting the parameter 
//   to the end of the CPath.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints
//
// Argument:
//   cPoint2d - An x,y value to connect the current path to.
//   cZThresh - The given point must have a z-value within this threshold
//      before SetXY succeeds.
//
// Returns: True if the extension creates a valid path, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Append( const CPoint2D& cPoint2d )
{
	AssertValid();

	//
	// Convert the given 2D point into a roadpos and then call the
	// Append that takes a roadPos as input.
	//
	CRoadPos roadPos = m_points.back().GetRoadPos( 1.0 );
	if( roadPos.SetXY( cPoint2d, 0,0, 1000 ) )
	{
		return Append( roadPos );
	}
	else
	{
		return false;
	}
} // end of Append

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Appends the end of the CPath by connecting the parameter 
//   to the end of the current CPath.
//
// Remarks: If the CPath contains a CPathPoint, then the underlying 
//   functionality of the CPathPoint is used.  Otherwise, the point is 
//   simply added onto the list.
//
//   Note that because the constructors for the CPathPoint class are 
//   protected, the only way to get an instance of the class is to access
//   one of the elements of an existing CPath through one of the iterator
//   functions.  This function is useful for concatenating two CPaths.
//
// Argument:
//   cRoadPos - Instance of the CRoadPos class to add to the path.
// 
// Returns: True if the extension creates a valid path, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Append( const CRoadPos& cRoadPos )
{
	CCvedItem::AssertValid();

	if( cRoadPos.IsValid() )
	{
		//
		// Create a PathPoint that represents the given cRoadPos.  Call
		// the Append that accepts a PathPoint as input.
		//
		CPathPoint pathPoint( cRoadPos );
		return Append( pathPoint );
	}
	else
	{
		return false;
	}
} // end of Append

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Appends the end of the CPath by connecting the parameter 
//   to the end of the current CPath.
//
// Remarks: If the CPath contains a CPathPoint, then the underlying 
//   functionality of the CPathPoint is used.  Otherwise, the point is 
//   simply added onto the list.
//
//   Note that because the constructors for the CPathPoint class are 
//   protected, the only way to get an instance of the class is to access
//   one of the elements of an existing CPath through one of the iterator
//   functions.  This function is useful for concatenating two CPaths and
//   is used by the CPath::Append(CPoint2D) function.
//
// Argument:
//   cPathPoint - Instance of the CPathPoint class to add to the path.
// 
// Returns: True if the extension creates a valid path, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Append( const CPathPoint& cPathPoint )
{
	CCvedItem::AssertValid();

	/*static*/ CFastDeque<CPathPoint> extension;
	extension.clear();
	extension.Reserve(30);

	bool connected = false;

	// If the current CPath and the parameter are valid
	if( cPathPoint.IsValid() )   
	{
		//
		// If the current CPath contains points, then attempt to extend 
		// the end of the CPath to connect with the parameter.
		//
		if( !m_points.empty() )
		{
			// There's got to be a better way than this.
			connected = m_points.back().Append( cPathPoint, extension );

			// If Append returned one or more points, append them to 
			// the current path.
			if( connected )
			{
				cTPathIterator i;
				for( i = extension.begin(); i != extension.end(); i++ )
				{
					m_points.push_back( extension[i] );
				}

				RemoveAnyExtraCorridors();
			}
		}
		// 
		// If the current path contains no points, simply add it to the list.
		//
		else
		{
			connected = true;
			m_points.push_front( cPathPoint );
		}
	}

	return connected;
} // end of Append

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Appends CPath through the next intersection such that no 
//   lane changes are needed on the road prior to the intersection.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints
//
//   If the Append was successful, the resulting CPath will run through the 
//   end of the next intersection (when on a road) or to the end of the 
//   current intersection.
//
// Argument:
//
// Returns: The distance appended to the given path.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::AppendNoLaneChange( const bitset<cCV_MAX_CRDRS>& cLaneCrdrMask )
{
	CFastDeque<CPathPoint> extension;
	extension.clear();
	extension.Reserve(30);
	double distAppended = 0.0;

	if( IsValid() ) 
	{
		distAppended = m_points.back().AppendNoLaneChange( 
											extension, 
											cLaneCrdrMask
											);

		// 
		// If Append returned one or more points, append them to the 
		// current path.
		//
		bool extend = distAppended > 0.0;
		if( extend )
		{
			cTPathIterator i;
			for( i = extension.begin(); i != extension.end(); i++ )
			{
				m_points.push_back( extension[i] );
			}
		}

		return extend;
	}

	return false;
} // end of Append

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Randomly extends the beginning of the CPath by a given 
//   distance.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints.
//
// Argument:
//   distance - A double value to extend the path.
//   stayOnHighway - A boolean that indicates if prepend should stay on
//	     roads that have been marked as being highways.
//
// Returns: true if the extension was successful, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Prepend( double distance, bool stayOnHighway )
{
	CFastDeque<CPathPoint> extension;
	bool connected = false;

	if( IsValid() ) 
	{
		connected = m_points.front().Prepend(
										distance, 
										m_pRng, 
										m_rngStreamId, 
										extension,
										stayOnHighway
										);

		//
		// If Prepend returned one or more points, prepend them 
		// to the current path.
		//
		if( connected ) 
		{
			cTPathIterator i;
			for( i = extension.end() - 1; i >= extension.begin(); i-- )
			{
				m_points.push_front( extension[i] );
			}
		}
	}
	return connected;
} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Prepends the end of the CPath by a turn in the 
//   indicated direction at the next intersection.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints
//
//   If the Prepend was successful, the resulting CPath will run through the 
//   next intersection to the beginning of the next road.
//
// Argument:
//   turnAngle - Defines the clockwise angle (in radians) made between the 
// 	    direction of the current lane(s) and the destination lane(s).
// 	    Note some useful predefined constants can be used for this parameter:
// 	    cCV_STRAIGHT_TURN - go straight through the intersection
// 	    cCV_RIGHT_TURN - turn right
// 	    cCV_U_TURN - make a U-turn
// 	    cCV_LEFT_TURN - turn left 
//   epsilon - The +/- tolerance of the TurnAngle when finding the desired
// 	    destination lane(s).  Note that there is a predefined constant for 
// 	    this parameter: cCV_EPSILON = 1/8 * pi
//   extension - output parameter, will contain any CPathPoints needed to 
//	    extend the current CPathPoint through the given turn.
//
// Returns: True if the extension was successful, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Prepend( double turnAngle, double eps )
{
	CFastDeque<CPathPoint> extension;
	bool connected = false;

	if( IsValid() ) 
	{
		connected = m_points.front().Prepend( turnAngle, eps, extension );

		//
		// If Prepend returned one or more points, prepend them to the 
		// current path.
		//
		if( connected )
		{
			cTPathIterator i;
			for( i = extension.end() - 1; i >= extension.begin(); i-- )
			{
				m_points.push_front( extension[i] );
			}
		}
	}

	return connected;
} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Prepends the end of the CPath by connecting the 
//   parameter to the end of the CPath.
//
// Remarks: This function uses the functionality of the underlying 
//   CPathPoint to perform the extend.  Therefore, the CPath must 
//   contain one or more CPathPoints
//
// Argument:
//   cPoint2d - x,y value to connect the current path to.
//
// Returns: True if the extension creates a valid path, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Prepend( const CPoint2D& cPoint2d )
{
	AssertValid();

	CRoadPos roadPos = m_points.front().GetRoadPos( 0.0 );
	if( roadPos.SetXY( cPoint2d ) )
	{
		return Prepend( roadPos );
	}
	else
	{
		return false;
	}
} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Prepends the end of the CPath by connecting the parameter 
//   to the end of the current CPath.
//
// Remarks: If the CPath contains a CPathPoint, then the underlying 
//   functionality of the CPathPoint is used.  Otherwise, the point is 
//   simply added onto the list.
//
//   Note that because the constructors for the CPathPoint class are 
//   protected, the only way to get an instance of the class is to access
//   one of the elements of an existing CPath through one of the iterator
//   functions.  This function is useful for concatenating two CPaths.
//
// Argument:
//   cRoadPos - Prepend this roadPos to the path.
// 
// Returns: True if the extension creates a valid path, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Prepend( const CRoadPos& cRoadPos )
{
	CCvedItem::AssertValid();

	if( cRoadPos.IsValid() )
	{
		CPathPoint pathPoint( cRoadPos );
		return Prepend( pathPoint );
	}
	else
	{
		return false;
	}
} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Prepends the end of the CPath by connecting the parameter 
//   to the end of the current CPath.
//
// Remarks: If the CPath contains a CPathPoint, then the underlying 
//   functionality of the CPathPoint is used.  Otherwise, the point is 
//   simply added onto the list.
//
//   Note that because the constructors for the CPathPoint class are 
//   protected, the only way to get an instance of the class is to access
//   one of the elements of an existing CPath through one of the iterator
//   functions.  This function is useful for concatenating two CPaths and
//   is used by the CPath::Prepend(CPoint2D) function.
//
// Argument:
//   cPathPoint - Instance of the CPathPoint class to add to the path.
// 
// Returns: True if the extension creates a valid path, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Prepend( const CPathPoint& cPathPoint )
{
	CCvedItem::AssertValid();

	CFastDeque<CPathPoint> extension;
	bool connected = false;

	// If the current CPath and the parameter are valid.
	if( cPathPoint.IsValid() )
	{
		// If the current CPath contains points, then attempt to extend 
		// the end of the CPath to connect with the parameter.
		if( !m_points.empty() ) 
		{
			// There's got to be a better way than this.
			connected = m_points.front().Prepend( cPathPoint, extension );

			// If Prepend returned one or more points, prepend them to 
			// the current path.
			if( connected )
			{
				cTPathIterator i;
				for( i = extension.end() - 1; i >= extension.begin(); i-- )
				{
					m_points.push_front( extension[i] );
				}
				RemoveAnyExtraCorridors();
			}
		}
		// If the current path contains no points, simply add it to the list.
		else 
		{
			connected = true;
			m_points.push_front( cPathPoint );
		}
	}

	return connected;
} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Pops the first point off the CPath.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::PopFront()
{
	m_points.pop_front();
} // end of PopFront

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Pops the last point off the CPath.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::PopBack()
{
	m_points.pop_back();
} // end of PopBack

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Deletes all the points in the path.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::Clear()
{
	m_points.clear();
} // end of Clear

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetString
// 	Sets each element in the current CPath with the strings in the parameter.
// 	Any points previously present in the path will be cleared out. 
//
// Remarks:  The format of each element in the parameter is as follows:
// 	If the node lies on a road:
// 		R:Road_Name:Lane_Id[Start_dist:End_dist]:Lane_Id[Start_dist:...
// 	Else if the node lies on an intersection
// 		I:Intrsctn_Name:Crdr_Id[Start_dist:End_dist]:Crdr_Id[Start_dist:...
//
// Arguments:
// 	cPathStr - (Input) Contains a string for each node desired in the current 
// 		path.
//
// Returns: True if a valid CPath is created from the parameter, false 
// 	otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CPath::SetString( const vector<string>& cPathStr ) 
{
	CPathPoint pathPoint( GetCved() );
	m_points.clear();

	if( !cPathStr.empty() ) 
	{
		bool returnValue = true;

		vector<string>::const_iterator cItr;
		for( cItr = cPathStr.begin(); cItr != cPathStr.end(); cItr++ ) 
		{
			pathPoint.SetString( *cItr );
			if( pathPoint.IsValid() )
			{
				m_points.push_back( pathPoint );
			}
			else
			{
				returnValue = false;
			}
		}

		return returnValue;
	}

	// There were no strings in the vector, so return false, 
	// indicating that the current CPath is now invalid.
	return false;

} // end of SetString


//////////////////////////////////////////////////////////////////////////////
//		Other functions	
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator==
//	Indicates whether the current CPath is the same as the paramter.
//
// Remarks:
//
// Arguments:
//	cPath - a CPath to compare to the current one.
//
// Returns: True if the current CPath is exactly the same as the parameter, 
//	false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::operator==( const CPath& cPath ) const 
{
	if( m_points.size() != cPath.m_points.size() )  return false;

	cTPathIterator curItr;
	cTPathIterator parItr;
	for( 
		curItr = m_points.begin(), 
		parItr = cPath.m_points.begin();
		( (curItr != m_points.end()) && (parItr != cPath.m_points.end()) );
		curItr++, parItr++
		)
	{
		if( !( m_points[curItr] == cPath.m_points[parItr] ) )
		{
			return false;
		}
	}

	return true;
} // end of operator==

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Indicated whether the current CPath intersects with 
//   the given parameter.  
//
// Remarks: Two CPaths intersect if they have one or more similar 
//   CPathPoints.
//
// Argument: 
// 	 cPath - Reference to a CPath.
//
// Returns:	True if the two paths intersect, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Intersect( const CPath& cPath ) const
{
	//
	// Iterate through the current instance's points
	//
	cTPathIterator curItr;
	cTPathIterator parItr;
	for( curItr = m_points.begin(); curItr != m_points.end(); curItr++ )
	{
		// Iterate through the parameter's points
		for( parItr = m_points.begin(); parItr != m_points.end(); parItr++ )
		{
			// If one pair of points is similar, then
			// 	the two paths intersect
			if( m_points[curItr].Similar( cPath.m_points[parItr] ) )
			{
				return true;
			}
		}
	}

	return false;
} // end of Intersect

//////////////////////////////////////////////////////////////////////////////
// 
// Description:  Indicates whether the current CPath is adjacent to 
//   the parameter.  
//
// Remarks: Two CPaths intersect if all points in the path are adjacent.
//
// Argument: 
// 	 cPath - Reference to a CPath.
//
// Returns:	True if the two paths are adjacent, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Adjacent( const CPath& cPath ) const
{
	// 
	// Iterate through the current instance's points.
	//
	cTPathIterator curItr;
	cTPathIterator parItr;
	for( curItr = m_points.begin(); curItr != m_points.end(); curItr++ )
	{
		// Iterate through the parameter's points
		for( parItr = m_points.begin(); parItr != m_points.end(); parItr++ )
		{
			// If one pair of points is adjacent, then
			// 	the two paths are adjacent
			if( !m_points[curItr].Adjacent( cPath.m_points[parItr] ) )
			{
				return false;
			}
		}
	}

	return true;
} // end of Adjacent

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Indicates whether the current CPath is contains the 
//   parameter.  
//
// Remarks: A CPath contains a CRoadPos if it runs over that position.
//
// Argument: 
// 	 cRoadPos - A reference to a valid CRoadPos.
//
// Returns:	True if the path contains the roadpos, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::Contains( const CRoadPos& cRoadPos ) const
{
	cTPathIterator curItr;

	// Iterate through the current instance's points
	for( curItr = m_points.begin(); curItr != m_points.end(); curItr++ )
	{
		if( m_points[curItr].Contains( cRoadPos ) )  return true;
	}
	return false;
} // end of Contains


//////////////////////////////////////////////////////////////////////////////
//
// Description: Given a starting road position and a distance, this
//   function traverses the path for the specified distance and returns 
//   the resulting road position.
//
// Remarks: Sets the input parameter 'end' to the result of travelling 
//   'dist' units from the point on the current CPath corresponding to 
//   'cStart'.  If 'cStart' lies on a road and GetRoadPos does not put the 
//   'end' point onto the connecting intersection, then the lane 
//   component of the 'end' point will be the same as the 'cStart' 
//   point.  The offset field of 'end' will always be the same as 
//   'cStart'.
//
//	 This function will cause a failed assertion if it is called on an 
//   invalid CPath instance.
//
// Arguments:
//	dist  - Distance to travel along the path starting from cStart.
//	cStartRoadPos - A CRoadPos corresponding to a point along the 
//          current CPath.
//	endRoadPos - (output) A CRoadPos corresponding to a point along the 
//          current CPath that is 'dist' units away from 'cStart', or as 
//          far from 'cStart' as it can get and still remain on the path.
//  strictConnectivity - Indicates if this function should enforce
//          connectivity between the given roadPos and corridors along
//          the path.
//
// Returns:  This function returns one of these enumerated values: 
//	eCV_TRAVEL_ERROR: Returned if an error is encountered.
//	eCV_TRAVEL_NOT_FOUND:  Returned if 'cStart' cannot be found on the 
//      current path
//	eCV_TRAVEL_LANE_CHANGE: Travel uses the lane set by the SwitchLane 
//		function to determine whether a lane change is needed to cross 
//      through the next intersection.  Note that the 'cStart' road pos 
//      does not indicate whether the user has completed a lane change, 
//      because the object whose position is represented by 'cStart' may 
//      not be completely within the lane.  Therefore the user must 
//      change the lane manually once the lane change has been 
//      completed.  If Travel is called before SwitchLane has been 
//      called on the current road, then the lane is taken from 'cStart'.
//	eCV_TRAVEL_OK: Returned if there's no error and no lane change is 
//      needed.
//	eCV_TRAVEL_END_OF_PATH: Returned if the resulting CRoadPos is at 
//      the end of the path.  Note that the returned 'endRoadPos' roadpos is 
//      valid and placed at the end of the path.
//
//////////////////////////////////////////////////////////////////////////////
CPath::ETravelCode
CPath::GetRoadPos( 
			double dist, 
			const CRoadPos& cStartRoadPos, 
			CRoadPos& endRoadPos, 
			bool strictConnectivity
			) 
{
	AssertValid();
	if( !cStartRoadPos.IsValid() )  return eCV_TRAVEL_ERROR;

	int pathPointIdx = m_pathPointIdx;
	
	//
	// If the 'cStartRoadPos' CRoadPos is not found along the current CPath, 
	// then return an error.
	//
	cTPathIterator curItr = FindPathPoint( cStartRoadPos, pathPointIdx );
	if( curItr == m_points.end() )  return eCV_TRAVEL_NOT_FOUND;

	endRoadPos = cStartRoadPos;
	double distTraveled = endRoadPos.GetDistance();
	
	do 
	{
		//
		// One these will be invalid, but there's no need to 
		// check that right now.
		//
		CLane prevLane = endRoadPos.GetLane();
		CCrdr prevCrdr = endRoadPos.GetCorridor();

		endRoadPos.SetDistance( distTraveled );

		//
		// Travel to the end of the current CPathPoint, or to the 
		// end of dist, whichever comes first.  If the current 
		// point lies on a road
		//
		if( m_points[curItr].m_isRoad ) 
		{
			//
			// Currently on a road coming from an intersection.
			//
			if( prevCrdr.IsValid() ) 
			{
				TLane* pCurLane = BindLane( prevCrdr.GetDstntnLnIdx() );

				//
				// Make sure that the previous corridor leads a lane
				// on this road that is on list of valid lanes.
				//
				if( m_points[curItr].m_laneCrdrMask.test( pCurLane->laneNo ) ) 
				{	
					int prevCrdrId = ( prevCrdr.GetId() - 
									   prevCrdr.GetIntrsctn().GetCrdrIdx()
									   );
					double crdrDist = endRoadPos.GetDistance( prevCrdrId );
					CLane dstLane( GetCved(), pCurLane );
					
					endRoadPos.SetLane( dstLane );
					endRoadPos.SetDistance( crdrDist );

				}
				else 
				{
					//
					// Error, previous corridor does not connect to 
					// the current lane.
					//
					return eCV_TRAVEL_ERROR;
				}
			}  // On a road, coming from an intersection.
			
			// else lane has already been set by cStartRoadPos

			// 
			// Compute distance to increment.
			//
			cvELnDir dir = m_points[curItr].GetLane().GetDirection();
			if( dir == ePOS ) 
			{
				distTraveled = m_points[curItr].GetEndDist() - endRoadPos.GetDistance();
			}
			else 
			{
				distTraveled = endRoadPos.GetDistance() - m_points[curItr].GetEndDist();
			}

			if( dist > distTraveled ) 
			{
				curItr++;
				pathPointIdx++;
			}
			else 
			{
				distTraveled = dist;
			}
				
		} // If the current point lies on a road

		else 
		{
			//
			// In an intersection, coming from a road.
			//
			int crdrUsed = -1;
			if( prevLane.IsValid() ) 
			{
				// 
				// Make sure there's a corridor whose source lane = prevLane.
				//
				int prevLnId = prevLane.GetRoad().GetLaneIdx();
				prevLnId += prevLane.GetRelativeId();


				int crdrId = m_points[curItr].GetFirstLaneCrdrIdx();
				TCrdr* pCrdr = BindCrdr( 
									crdrId + 
									m_points[curItr].m_intrsctn.GetCrdrIdx() 
									);
				for(
					; 
					( crdrId < cCV_MAX_CRDRS && crdrUsed < 0 );
					crdrId++, pCrdr++
					) 
				{
					if( m_points[curItr].m_laneCrdrMask.test( crdrId ) ) 
					{
						if( !strictConnectivity || pCrdr->srcLnIdx == prevLnId ) 
						{
							crdrUsed = crdrId;
							// Sets corridor and intersection
							endRoadPos.SetCorridor( 
									CCrdr( GetCved(), pCrdr ), 
									endRoadPos.GetDistance(), 
									endRoadPos.GetOffset() 
									);
						}
					}
				}

				if( crdrUsed < 0 ) 
				{
					// Error, no corridor on the path connects
					// the current corridor(s) to the previous lane.
					return eCV_TRAVEL_ERROR;
				}

			} // If coming from a road
			// else crdr has already been set by cStartRoadPos
			
			if( crdrUsed < 0 ) 
			{
				crdrUsed = endRoadPos.GetCorridor().GetId() - 
						   endRoadPos.GetIntrsctn().GetCrdrIdx();
			}

			// Compute distance to increment
			distTraveled = (
						m_points[curItr].GetEndDist( crdrUsed ) - 
						endRoadPos.GetDistance()
						);
			bool allDistTraveled = dist <= distTraveled;
			if( allDistTraveled )
			{	
				distTraveled = dist;
			}
			else 
			{
				curItr++;
				pathPointIdx++;
			}
		} // Else if the current point lies on an intersection 

		// Increment endRoadPos.distance
		endRoadPos.IncrDistance( distTraveled );

		// Decrement dist counter
		dist -= distTraveled;

		// Reset distTraveled for next iteration
		if( curItr != m_points.end() ) 
		{
			distTraveled = m_points[curItr].GetStartDist();
		}
		else 
		{
			return eCV_TRAVEL_END_OF_PATH;
		}
		
	} while( (dist > 0.0) && (curItr != m_points.end()) );

	//
	// If control reaches this point, then no error occurred and no 
	// lane change is necessary.  Return OK.
	//
	return eCV_TRAVEL_OK;

} // end of GetRoadPos

//////////////////////////////////////////////////////////////////////////////
// 
// Description: Indicates whether the input road is the next road 
//  on the path.  
//
// Remarks: If the current path point is a road, then the function looks
//  to see if the input road is the next road after the next intersection.
//
// Argument: 
// 	cRoad - A reference to a valid CRoad.
//
// Returns:	A boolean indicating if the input road is the next road on
//  the path.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::IsNextRoad( const CRoad& cRoad ) const
{
	// Find the path-point iterator for the current position in the path.
	cTPathIterator itr = FindPathPoint( m_pathPointIdx );

	// Skip to the intersection if the current point is road.
	if ( m_points[itr].IsRoad() )  itr++;

	// Skip to the road after the intersection.
	itr++;

	if( itr < m_points.size() ) 
	{
		return m_points[itr].Contains( cRoad ); 
	}
	else 
	{
		return false;
	}
}  // end of IsNextRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: Travel
// 	 This function is used to travel forward along a previously created CPath 
//   to a point the given distance away from a given starting point.
//
// Remarks: Sets the input parameter 'end' to the result of travelling 
//   'dist' units from the point on the current CPath corresponding to 
//   'cStart'.  If 'cStart' lies on a road and Travel does not put the 
//   'end' point onto the connecting intersection, then the lane 
//   component of the 'end' point will be the same as the 'cStart' 
//   point.  The offset field of 'end' will always be the same as 
//   'cStart'.
//
//	 Note that though this function modifies some internal data, it 
//   leaves the core path intact.  Therefore, the user may Travel over 
//   the path as many times as needed.
//
//	 This function will cause a failed assertion if it is called on an 
//   invalid CPath instance.
//
// Arguments:
//	dist  - Distance from 'cStart' to travel.
//	cStart - A CRoadPos corresponding to a point along the current CPath.
//	end   - (output) A CRoadPos corresponding to a point along the 
//          current CPath that is 'dist' units away from 'cStart', or as 
//          far from 'cStart' as it can get and still remain on the path.
//	changeLane  - (output) if the function returns eCV_TRAVEL_LANE_CHANGE, 
//          then this is the lane that the user should change to.
//
// Returns:  This function returns one of three enumerated values: 
//	eCV_TRAVEL_ERROR: Returned if an error is encountered.
//	eCV_TRAVEL_NOT_FOUND:  Returned if 'cStart' cannot be found on the 
//      current path
//	eCV_TRAVEL_LANE_CHANGE: Travel uses the lane set by the SwitchLane 
//		function to determine whether a lane change is needed to cross 
//      through the next intersection.  Note that the 'cStart' road pos 
//      does not indicate whether the user has completed a lane change, 
//      because the object whose position is represented by 'cStart' may 
//      not be completely within the lane.  Therefore the user must 
//      change the lane manually once the lane change has been 
//      completed.  If Travel is called before SwitchLane has been 
//      called on the current road, then the lane is taken from 'cStart'.
//	eCV_TRAVEL_OK: Returned if there's no error and no lane change is 
//      needed.
//	eCV_TRAVEL_END_OF_PATH: Returned if the resulting CRoadPos is at 
//      the end of the path.  Note that the returned 'end' roadpos is 
//      valid and placed at the end of the path.
//
//////////////////////////////////////////////////////////////////////////////
CPath::ETravelCode
CPath::Travel(
			double dist,
			const CRoadPos& cStart,
			CRoadPos& end,
			CLane& changeLane
			) 
{

	AssertValid();
	if( !cStart.IsValid() )  return eCV_TRAVEL_ERROR;

	//
	// If the current road pos is on a road, and if no current lane 
	// has been set, set it based on the current lane.
	// We do not update the current lane every time Travel is called, 
	// because a lane change may be in progress, and the current road 
	// pos may indicate a new lane, when the vehicle has not yet 
	// finished its lane change.
	//
	if( cStart.IsRoad() )
	{ 
		bool noCurrentLane = m_curLaneId < 0;
		if( noCurrentLane )
		{
			m_curLaneId = cStart.GetLane().GetRelativeId();
		}
	}
	else 
	{
		//
		// If the current road pos is on an intersection, then reset 
		// the current lane to an invalid -1, so that once we're back 
		// on a road, we can update the current lane.
		//
		m_curLaneId = -1;
	}

	//
	// If the 'cStart' CRoadPos is not found along the current CPath, 
	// then return an error.
	//
	cTPathIterator curItr = FindPathPoint( cStart, m_pathPointIdx );
	bool travelNotFound = curItr == m_points.end();
	if( travelNotFound )  return eCV_TRAVEL_NOT_FOUND;

	end = cStart;
	double distTraveled = end.GetDistance();

	//
	// Execute this loop until the given distance has been traveled 
	// or the end of the path has been reached.
	//
	do 
	{
		//
		// One of these will be invalid, but there's no need to 
		// check that right now.
		//
		CLane prevLane = end.GetLane();
		CCrdr prevCrdr = end.GetCorridor();

		end.SetDistance( distTraveled );

		//
		// Travel to the end of the current CPathPoint, or to the 
		// end of dist, whichever comes first.  If the current 
		// point lies on a road
		//
		if( m_points[curItr].m_isRoad )
		{
			// If coming from an intersection
			if( prevCrdr.IsValid() ) 
			{
				int crdrId = (
					prevCrdr.GetId() - prevCrdr.GetIntrsctn().GetCrdrIdx()
					);
		
				TLane* pCurLane = BindLane( prevCrdr.GetDstntnLnIdx() );

				//
				// Make sure that dstLane of prevCrdr is on the list 
				// of valid lanes.
				//
				if ( m_points[curItr].m_laneCrdrMask.test( pCurLane->laneNo ) ) 
				{
					double crdrDist = end.GetDistance( crdrId );
					CLane dstLane( GetCved(), pCurLane );
					// Sets lane and road
					end.SetLane( dstLane );
					end.SetDistance( crdrDist );
				}
				else 
				{
					//
					// Error, previous corridor does not connect to 
					// the current lane.
					//
					return eCV_TRAVEL_ERROR;
				}
			} // If coming from an intersection
			// else lane has already been set by cStart

			// 
			// Compute the distance from here to the end of the road.
			//
			cvELnDir dir = m_points[curItr].GetLane().GetDirection();
			if( dir == ePOS ) 
			{
				distTraveled = (
					m_points[curItr].GetEndDist() - end.GetDistance()
					);
			}
			else 
			{
				distTraveled = (
					end.GetDistance() - m_points[curItr].GetEndDist()
					);
			}

			bool allDistTraveled = dist <= distTraveled;
			if( allDistTraveled )
			{	
				distTraveled = dist;
			}
			else 
			{
				curItr++;
			}
		} // If the current point lies on a road
		else 
		{
			// 
			// Coming from a road onto an intersection.
			//
			int crdrUsed = -1;
			if( prevLane.IsValid() )
			{
				//
				// Make sure there's a corridor whose source lane = prevLane.
				//
				int prevLnId = prevLane.GetRoad().GetLaneIdx();
				bool noCurrentLane = m_curLaneId < 0;
				if( noCurrentLane )
				{
					prevLnId += prevLane.GetRelativeId();
				}
				else
				{
					prevLnId += m_curLaneId;
				}

				int crdrId = m_points[curItr].GetFirstLaneCrdrIdx();
				TCrdr* pCrdr = BindCrdr( 
									crdrId + 
									m_points[curItr].m_intrsctn.GetCrdrIdx()
									);

				for(
					; 
					( crdrId < cCV_MAX_CRDRS && crdrUsed < 0 );
					crdrId++, pCrdr++
					) 
				{
					if( m_points[curItr].m_laneCrdrMask.test( crdrId ) )
					{
						if( pCrdr->srcLnIdx == prevLnId ) 
						{
							crdrUsed = crdrId;
							// Sets corridor and intersection
							end.SetCorridor(
									CCrdr( GetCved(), pCrdr ), 
									end.GetDistance(), 
									end.GetOffset()
									);
						}
					}
				}

				if( crdrUsed < 0 ) 
				{
					bool tryAnotherSourceLane = prevLane.GetId() != m_curLaneId;
					if( tryAnotherSourceLane )
					{
						//
						// If we're in the middle of a lane change, then it makes sense
						// to check connectivity against the other possible source lane.
						//
						int prevLnId = prevLane.GetRoad().GetLaneIdx();
						prevLnId += prevLane.GetRelativeId();

						int crdrId = m_points[curItr].GetFirstLaneCrdrIdx();
						TCrdr* pCrdr = BindCrdr( 
											crdrId + 
											m_points[curItr].m_intrsctn.GetCrdrIdx()
											);

						for(
							; 
							( crdrId < cCV_MAX_CRDRS && crdrUsed < 0 );
							crdrId++, pCrdr++
							) 
						{
							if( m_points[curItr].m_laneCrdrMask.test( crdrId ) )
							{
								if( pCrdr->srcLnIdx == prevLnId ) 
								{
									crdrUsed = crdrId;
									// Sets corridor and intersection
									end.SetCorridor(
											CCrdr( GetCved(), pCrdr ), 
											end.GetDistance(), 
											end.GetOffset()
											);
								}
							}
						}

						if( crdrUsed < 0 )
						{
							// Error, no corridor on the path connects the current 
							// corridor(s) to the previous lane.
							return eCV_TRAVEL_ERROR;
						}
					}
					else
					{
						// Error, no corridor on the path connects the current 
						// corridor(s) to the previous lane.
						return eCV_TRAVEL_ERROR;
					}
				}
			} // If coming from a road
			// else crdr has already been set by cStart
			
			if( crdrUsed < 0 )
			{
				crdrUsed = (
					end.GetCorridor().GetId() - 
					end.GetIntrsctn().GetCrdrIdx()
					);
			}

			// Compute distance to increment
			distTraveled = (
				m_points[curItr].GetEndDist( crdrUsed ) - 
				end.GetDistance()
				);
			bool allDistTraveled = dist <= distTraveled;
			if( allDistTraveled )
			{	
				distTraveled = dist;
			}
			else 
			{
				curItr++;
			}

			//
			// Reset current lane index, because it's invalid now that 
			// we're on an intersection.
			//
			m_curLaneId = -1;

		} // Else if the current point lies on an intersection 

		// Increment end.distance
		end.IncrDistance( distTraveled );

		// Decrement dist counter
		dist -= distTraveled;

		// Reset distTraveled for next iteration
		if( curItr != m_points.end() )
		{
			distTraveled = m_points[curItr].GetStartDist();
		}
		else 
		{
			return eCV_TRAVEL_END_OF_PATH;
		}
		
	} while ( (dist > 0.0) && (curItr != m_points.end()) );

	//
	// Figure out if a lane change is needed.
	//
	changeLane = CLane();
	
	// 
	// If there is a current lane.
	//
	if( m_curLaneId >= 0 )
	{
		//
		// If the next point along the path is an intersection.
		//
		cTPathIterator pNextPathPoint = curItr;
		pNextPathPoint++;
		bool isIntersection = ( 
					pNextPathPoint != m_points.end() &&
					!m_points[pNextPathPoint].m_isRoad
					);

		if( isIntersection )
		{
			//
			// For each available corridor, if one is found that begins
			// at the current lane, return OK.
			//
			int crdrId = m_points[pNextPathPoint].GetFirstLaneCrdrIdx();
			TCrdr* pCrdr = BindCrdr( 
								crdrId + 
								m_points[pNextPathPoint].m_intrsctn.GetCrdrIdx() 
								);
			int firstLaneIdx = m_points[curItr].m_road.GetLaneIdx();

			//
			// Iterate through all the relevant corridors in the upcoming
			// intersection to see if there exists a corridor that begins
			// at the current lane.
			//
			for( ; crdrId < cCV_MAX_CRDRS; crdrId++, pCrdr++ ) 
			{
				//
				// Check to see if this corridor is in the mask.  If not
				// then skip it.
				//
				bool crdrInMask = m_points[pNextPathPoint].m_laneCrdrMask.test( crdrId );
				if( !crdrInMask )  continue;

				//
				// If this corridor begins at a lane that has
				// the same id (relative to the road) as the
				// current lane id, return OK.
				//
				int srcLaneId = pCrdr->srcLnIdx - firstLaneIdx;
				if( srcLaneId == m_curLaneId )  return eCV_TRAVEL_OK;

				//
				// If the current lane to return is valid.
				//
				bool noPreviousChangeLane = !changeLane.IsValid();
				if( noPreviousChangeLane ) 
				{
					//
					// This the lane to change to....unless we can find
					// a closer source lane from the remaining corridors.
					//
					TLane* pLane = BindLane( pCrdr->srcLnIdx );
					changeLane = CLane( GetCved(), pLane );
				}
				// If the current lane to return is not valid.
				else 
				{
					//
					// Check to see if the source lane of the current
					// corridor is closer to the current lane.  If it
					// is, then set lane to the source lane.
					// int returnLaneIdx = lane.GetId() - firstLaneIdx;
					//
					int returnLaneId = changeLane.GetRelativeId();
					bool closer = ( 
								abs( m_curLaneId - srcLaneId ) <
								abs( m_curLaneId - returnLaneId )
								);
					if( closer )
					{
						//
						// This source lane is closer than the one
						// previously chosen.  Use this one unless a
						// closer one is found from the remaining 
						// corridors.
						//
						changeLane = CLane( 
										GetCved(), 
										changeLane.GetRoad(), 
										srcLaneId
										);
					}
				}
			} // For each corridor

			//
			// If control made it here, then there was no corridor 
			// connecting the current lane to the next lane.  
			// Therefore, a lane change is required.  
			// Note that newLane should contain a lane adjacent 
			// to the current lane (or at least one that connects 
			// to the next corridor).  If it doesn't, then there's
			// an error.
			//
			if( changeLane.IsValid() ) 
			{
				return eCV_TRAVEL_LANE_CHANGE;
			}
			else 
			{
				return eCV_TRAVEL_ERROR;
			}

		} // If the next point along the path is an intersection

	} // If there is a current lane

	//
	// If control reaches this point, then no error occurred and no 
	// lane change is necessary.  Return OK.
	//
	return eCV_TRAVEL_OK;

} // end of Travel

//////////////////////////////////////////////////////////////////////////////
//
// Description: TravelBack
// 	 This function is used to travel backwards along a previously created CPath 
//   to a point the given distance away from a given starting point.
//
// Remarks: Sets the input parameter 'end' to the result of travelling 
//   'dist' units from the point on the current CPath corresponding to 
//   'cStart'.  If 'cStart' lies on a road and Travel does not put the 
//   'end' point onto the connecting intersection, then the lane 
//   component of the 'end' point will be the same as the 'cStart' 
//   point.  The offset field of 'end' will always be the same as 
//   'cStart'.
//
//	 Note that though this function modifies some internal data, it 
//   leaves the core path intact.  Therefore, the user may Travel over 
//   the path as many times as needed.
//
//	 This function will cause a failed assertion if it is called on an 
//   invalid CPath instance.
//
// Arguments:
//	dist  - Distance from 'cStart' to travel backwards.  The assumption
//		is that we are traveling backwards so any negative distances will
//		'fabs'ed.
//	cStart - A CRoadPos corresponding to a point along the current CPath.
//	pathTravelRoadPos - (output) A CRoadPos corresponding to a point along 
//      the current CPath that is 'dist' units away from 'cStart', or as 
//      far from 'cStart' as it can get and still remain on the path.
//	changeLane  - (output) if the function returns eCV_TRAVEL_LANE_CHANGE, 
//      then this is the lane that the user should change to.
//
// Returns:  This function returns one of three enumerated values: 
//	eCV_TRAVEL_ERROR: Returned if an error is encountered.
//	eCV_TRAVEL_NOT_FOUND:  Returned if 'cStart' cannot be found on the 
//      current path
//	eCV_TRAVEL_LANE_CHANGE: Travel uses the lane set by the SwitchLane 
//		function to determine whether a lane change is needed to cross 
//      through the next intersection.  Note that the 'cStart' road pos 
//      does not indicate whether the user has completed a lane change, 
//      because the object whose position is represented by 'cStart' may 
//      not be completely within the lane.  Therefore the user must 
//      change the lane manually once the lane change has been 
//      completed.  If Travel is called before SwitchLane has been 
//      called on the current road, then the lane is taken from 'cStart'.
//	eCV_TRAVEL_OK: Returned if there's no error and no lane change is 
//      needed.
//	eCV_TRAVEL_END_OF_PATH: Returned if the resulting CRoadPos is at 
//      the end of the path.  Note that the returned 'end' roadpos is 
//      valid and placed at the end of the path.
//
//////////////////////////////////////////////////////////////////////////////
CPath::ETravelCode
CPath::TravelBack(
			double dist,
			const CRoadPos& cInitial,
			CRoadPos& pathTravelRoadPos,
			bool strictConnectivity 
			) 
{

	AssertValid();
	if( !cInitial.IsValid() )  return eCV_TRAVEL_ERROR;

	//
	// If the current road pos is on a road, and if no current lane 
	// has been set, set it based on the current lane.
	// We do not update the current lane every time Travel is called, 
	// because a lane change may be in progress, and the current road 
	// pos may indicate a new lane, when the vehicle has not yet 
	// finished its lane change.
	//
	if( cInitial.IsRoad() )
	{ 
		bool noCurrentLane = m_curLaneId < 0;
		if( noCurrentLane )
		{
			m_curLaneId = cInitial.GetLane().GetRelativeId();
		}
	}
	else 
	{
		//
		// If the current road pos is on an intersection, then reset 
		// the current lane to an invalid -1, so that once we're back 
		// on a road, we can update the current lane.
		//
		m_curLaneId = -1;
	}

	//
	// If the 'cInitial' CRoadPos is not found along the current CPath, 
	// then return an error.
	//
	cTPathIterator curItr = FindPathPoint( cInitial, m_pathPointIdx );
	bool travelNotFound = curItr == m_points.end();
	if( travelNotFound )  return eCV_TRAVEL_NOT_FOUND;

	pathTravelRoadPos = cInitial;
	dist = fabs(dist);
//	double distOnThisSegment = pathTravelRoadPos.GetDistance();

	//
	// Execute this loop until the given distance has been traveled 
	// or the end of the path has been reached.
	//
	do 
	{
		//
		// One of these will be invalid, but there's no need to 
		// check that right now.
		//
		CLane nextLane = pathTravelRoadPos.GetLane();
		CCrdr nextCrdr = pathTravelRoadPos.GetCorridor();

//		pathTravelRoadPos.SetDistance( distOnThisSegment );

		//
		// Travel to the end of the current CPathPoint, or to the 
		// end of dist, whichever comes first.  If the current 
		// point lies on a road
		//
		if( m_points[curItr].m_isRoad )
		{
			// If coming from an intersection
			double segmentDist;
			if( nextCrdr.IsValid() ) 
			{
				int crdrId = (
					nextCrdr.GetId() - nextCrdr.GetIntrsctn().GetCrdrIdx()
					);
		
				TLane* pCurLane = BindLane( nextCrdr.GetSrcLnIdx() );

				// Make sure that srcLane of nextCrdr is on the list 
				// of valid lanes.
				if ( m_points[curItr].m_laneCrdrMask.test( pCurLane->laneNo ) ) 
				{
					double crdrDist = pathTravelRoadPos.GetDistance( crdrId );
					CLane srcLane( GetCved(), pCurLane );
					// Sets lane and road
					bool result = pathTravelRoadPos.SetLane( srcLane );
					if( !result )
					{
						return eCV_TRAVEL_ERROR;
					}
//					pathTravelRoadPos.SetDistance( crdrDist );
					CRoadPos::ETravelResult travelResult;
					double endDist = m_points[curItr].GetEndDist();
					if( srcLane.GetDirection() == ePOS )
						travelResult = pathTravelRoadPos.SetDistance( endDist );
					else
						travelResult = pathTravelRoadPos.SetDistance( 0.0 );
					if( travelResult == CRoadPos::ePAST_ROAD || travelResult == CRoadPos::eERROR )
					{
						return eCV_TRAVEL_ERROR;
					}
					segmentDist = m_points[curItr].GetEndDist() - m_points[curItr].GetStartDist();
				}
				else 
				{
					//
					// Error, previous corridor does not connect to 
					// the current lane.
					//
					return eCV_TRAVEL_ERROR;
				}
			} // If coming from an intersection
			// else lane has already been set by cInitial
			else
			{
				// Compute the distance from here to the end of the road.
				cvELnDir dir = m_points[curItr].GetLane().GetDirection();
				if( dir == ePOS ) 
				{
					segmentDist = pathTravelRoadPos.GetDistance();
				}
				else 
				{
					segmentDist = m_points[curItr].GetStartDist() - pathTravelRoadPos.GetDistance();
				}
			}

			bool completedTravel = dist <= segmentDist;
			if( completedTravel )
			{	
				CRoadPos::ETravelResult result = pathTravelRoadPos.DecrDistance( dist );
				if( result == CRoadPos::ePAST_ROAD || result == CRoadPos::eERROR )
					return eCV_TRAVEL_ERROR;
				else
					return eCV_TRAVEL_OK;			
			}
			else 
			{
				dist -= segmentDist;
				CRoadPos::ETravelResult result = pathTravelRoadPos.DecrDistance( segmentDist );
				if( result == CRoadPos::ePAST_ROAD || result == CRoadPos::eERROR )
				{
					return eCV_TRAVEL_ERROR;
				}
				else
				{
					if( curItr == m_points.begin() )
						return eCV_TRAVEL_END_OF_PATH;
					else
						curItr--;
				}
			}
		} // If the current point lies on a road
		else 
		{
			// 
			// Traveling back to an intersection from a road.
			//
			int crdrUsed = -1;
			double segmentDist;
			if( nextLane.IsValid() )
			{
				//
				// Make sure there's a corridor whose destination lane == nextLane.
				//
				int nextLnIdx = nextLane.GetRoad().GetLaneIdx();
				bool noCurrentLane = m_curLaneId < 0;
				if( noCurrentLane )
				{
					nextLnIdx += nextLane.GetRelativeId();
				}
				else
				{
					nextLnIdx += m_curLaneId;
				}

				int crdrId = m_points[curItr].GetFirstLaneCrdrIdx();
				TCrdr* pCrdr = BindCrdr( 
									crdrId + 
									m_points[curItr].m_intrsctn.GetCrdrIdx()
									);
				for(  ; crdrId < cCV_MAX_CRDRS && crdrUsed < 0; crdrId++, pCrdr++ ) 
				{
					if( m_points[curItr].m_laneCrdrMask.test( crdrId ) )
					{
						if( pCrdr->dstLnIdx == nextLnIdx ) 
						{
							crdrUsed = crdrId;
							double crdrDist = m_points[curItr].GetEndDist(crdrId);
							// Sets corridor and intersection
							pathTravelRoadPos.SetCorridor(
									CCrdr( GetCved(), pCrdr ), 
//									pathTravelRoadPos.GetDistance(), 
									crdrDist,
									pathTravelRoadPos.GetOffset()
									);
						}
					}
				}

				if( crdrUsed < 0 ) 
				{
					bool tryAnotherDstLane = (!strictConnectivity) && nextLane.GetId() != m_curLaneId;
					if( tryAnotherDstLane )
					{
						// If we're in the middle of a lane change, then it makes sense
						// to check connectivity against the other possible source lane.
						int nextLnIdx = nextLane.GetRoad().GetLaneIdx();
						nextLnIdx += nextLane.GetRelativeId();

						int crdrId = m_points[curItr].GetFirstLaneCrdrIdx();
						TCrdr* pCrdr = BindCrdr( 
											crdrId + 
											m_points[curItr].m_intrsctn.GetCrdrIdx()
											);

						for( ; crdrId < cCV_MAX_CRDRS && crdrUsed < 0; crdrId++, pCrdr++ ) 
						{
							if( m_points[curItr].m_laneCrdrMask.test( crdrId ) )
							{
								if( pCrdr->dstLnIdx == nextLnIdx ) 
								{
									crdrUsed = crdrId;
									double crdrDist = m_points[curItr].GetEndDist(crdrId);
									// Sets corridor and intersection
									pathTravelRoadPos.SetCorridor(
											CCrdr( GetCved(), pCrdr ), 
//											pathTravelRoadPos.GetDistance(), 
											crdrDist,
											pathTravelRoadPos.GetOffset()
											);
								}
							}
						}

						if( crdrUsed < 0 )
						{
							// Error, no corridor on the path connects the current 
							// corridor(s) to the previous lane.
							return eCV_TRAVEL_ERROR;
						}
					}
					else
					{
						// Error, no corridor on the path connects the current 
						// corridor(s) to the previous lane.
						return eCV_TRAVEL_ERROR;
					}
				}
			} // If coming from a road
			// else crdr has already been set by cInitial
			if( crdrUsed < 0 )
			{
				crdrUsed = (
					pathTravelRoadPos.GetCorridor().GetId() - 
					pathTravelRoadPos.GetIntrsctn().GetCrdrIdx()
					);
			}

			// Compute distance to increment
			segmentDist = pathTravelRoadPos.GetDistance();
			bool allDistTraveled = dist <= segmentDist;
			if( allDistTraveled )
			{	
//				distOnThisSegment = dist;
				CRoadPos::ETravelResult result = pathTravelRoadPos.DecrDistance( dist );
				if( result == CRoadPos::ePAST_ROAD || result == CRoadPos::eERROR )
					return eCV_TRAVEL_ERROR;
				else
					return eCV_TRAVEL_OK;			
			}
			else 
			{
				dist -= segmentDist;
				if( curItr == m_points.begin() )
				{
					return eCV_TRAVEL_END_OF_PATH;
				}
				else
					curItr--;
			}

			// Reset current lane index, because it's invalid now that 
			// we're on an intersection.
			m_curLaneId = -1;

		} // Else if the current point lies on an intersection 

		// Decrement pathTravelRoadPos.distance
//		pathTravelRoadPos.DecrDistance( distOnThisSegment );

		// Decrement dist counter
//		dist -= distOnThisSegment;
//		distOnThisSegment = m_points[curItr].GetStartDist();
	} while ( dist > 0.0 );

	//
	// If control reaches this point, then no error occurred and no 
	// lane change is necessary.  Return OK.
	//
	return eCV_TRAVEL_OK;

} // end of TravelBack

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function is used in conjunction with the Travel 
//  function to change the current lane id to the parameter.
//
// Remarks: If the user is using te Travel function to traverse the path, and 
// 	he wants to make a lane change, then he should indicate the completion of
// 	that lane change by calling this function.  Travel will return 
// 	eCV_TRAVEL_LANE_CHANGE as long as the current lane id is a lane that is 
// 	not connected to an available corridor in the next intersection.  Once
// 	the user calls this function with a connected lane, Travel will stop 
// 	returning eCV_TRAVEL_LANE_CHANGE.
//
// Arguments:
// 	laneId - Identifier of the lane with respect to the current road.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::SwitchLane( int laneId )
{
	m_curLaneId = laneId;
} // end of SwitchLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function is used in conjunction with the Travel 
//  function to change the current lane id to the parameter's id with 
//  respect to its road.
//
// Remarks: If the user is using te Travel function to traverse the path, and 
// 	he wants to make a lane change, then he should indicate the completion of
// 	that lane change by calling this function.  Travel will return 
// 	eCV_TRAVEL_LANE_CHANGE as long as the current lane id is a lane that is 
// 	not connected to an available corridor in the next intersection.  Once
// 	the user calls this function with a connected lane, Travel will stop 
// 	returning eCV_TRAVEL_LANE_CHANGE.
//
// Arguments:
// 	cLane - CLane instance of the lane to switch to.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::SwitchLane( const CLane& cLane )
{
	m_curLaneId = cLane.GetRelativeId();
} // end of SwitchLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the CVED object ids of the objects that lie 
//  on the current path.
//
// Remarks: The object ids are returned in order of their distance 
//  along the path.
//
// Arguments:
// 	objects - (output) Contains the object ids of the objects on the path
// 		Note: The vector is cleared at the beginning of the function
//
// Returns: The number of objects found on the path.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetObjectsOnPath( vector<int>& objects ) const
{
	objects.clear();
	cTPathIterator itr;
	for ( itr = m_points.begin(); itr != m_points.end(); itr++ ) 
	{
		m_points[itr].GetObjectsOnPoint( objects );
	}

	return (int) objects.size();
} // end of GetObjectsOnPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the CVED object ids of the objects that lie 
//  on the current path after the given road position.
//
// Remarks: The object ids are returned in order of their distance 
//  along the path.
//
// Arguments:
//  cStartRoadPos - The starting road position along the path.
// 	objects - (output) Contains the object ids of the objects on the path
// 		Note: The vector is cleared at the beginning of the function
//
// Returns: The number of objects found on the path.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetObjectsOnPath(
			const CRoadPos& cStartRoadPos, 
			vector<int>& objects
			) const
{

	objects.clear();

	//
	// Error checking.
	//
	if( !cStartRoadPos.IsValid() )  return 0;

	// 
	// Find the pathPoint corresponding to the cStartRoadPos parameter.
	//
	cTPathIterator itr = m_points.begin();
	int startIdx = m_pathPointIdx;
	itr = FindPathPoint( cStartRoadPos, startIdx );

	//
	// If FindPathPoint returned 0, then no corresponding point was 
	// found, so return 0.
	//
	if( itr == m_points.end() )  return 0;

	//
	// Find all objects on the current path point from cStartRoadPos
	// to its end.
	//
	m_points[itr].GetObjectsOnPoint( cStartRoadPos, objects );
	
	itr++;
	for( ; itr != m_points.end(); itr++) 
	{
		m_points[itr].GetObjectsOnPoint( objects );
	}

	return (int) objects.size();

} // end of GetObjectsOnPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the CVED object ids of the objects that lie on 
//  the current path after the given road position on its lane.
//
// Remarks: The object ids are returned in order of their distance 
//  along the path.
//
// Arguments:
//  cStartRoadPos - The starting road position along the path.
// 	objects - (output) Contains the object ids of the objects on the path
// 		Note: The vector is cleared at the beginning of the function
//
// Returns: The number of objects found on the path.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetObjectsOnPathLane(
			const CRoadPos& cStartRoadPos, 
			vector<int>& objects
			) const
{
	/*static*/ vector<int> scratch;			// to minimize memory alloc
	//static bool sInit = false;
	scratch.reserve(30);
/*
	if( sInit == false ) 
	{
		sInit = true;
		scratch.reserve( 30 );          // 30 is just an estimate of max size
	}
*/
	//
	// Error checking.
	//
	if( !cStartRoadPos.IsValid() )  return 0;

	// 
	// Find the pathPoint corresponding to the cStartRoadPos parameter.
	//
	cTPathIterator itr = m_points.begin();
	int startIdx = m_pathPointIdx;
	itr = FindPathPoint( cStartRoadPos, startIdx );

	//
	// If FindPathPoint returned 0, then no corresponding point was 
	// found, so return 0.
	//
	if( itr == m_points.end() )  return 0;

	//
	// Get the current lane.
	//
	int laneId;
	if( cStartRoadPos.IsRoad() ) 
	{
		laneId = cStartRoadPos.GetLane().GetRelativeId();
	}
	else 
	{
		laneId = -1;
	}

	//
	// Find all objects on the current path point from cStartRoadPos
	// to its end.
	//
	scratch.clear();
	m_points[itr].GetObjectsOnPoint( cStartRoadPos, laneId, scratch );
	
	itr++;
	for( ; itr != m_points.end(); itr++ ) 
	{
		m_points[itr].GetObjectsOnPoint( scratch );
	}

	objects = scratch;
	return (int) objects.size();

} // end of GetObjectsOnPathLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the CVED object ids of the objects that lie 
//  on the current path and a bitset containing the lanes/crdrs 
//  occupied by objects at the given point.
//
// Remarks: The object ids are returned in order of their distance 
//  along the path.
//
// Arguments:
//  cRoadPos - Position to get occupied bitset for
//  occupied - (output) Contains the ids of the lanes/crdrs that are occupied
//  	by objects on the point corresponding to cRoadPos.
// 	objects - (output) Contains the object ids of the objects on the path
// 		Note: The vector is cleared at the beginning of the function
//
// Returns: The number of objects found on the path.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetObjectsOnPath(
			const CRoadPos& cRoadPos, 
			bitset<cCV_MAX_CRDRS>& occupied,
			vector<int>& objects
			) const
{

	objects.clear();
	occupied.reset();

	cTPathIterator itr;
	for( itr = m_points.begin(); itr != m_points.end(); itr++ ) 
	{
		if( m_points[itr].Contains( cRoadPos ) ) 
		{
			m_points[itr].GetObjectsOnPoint( occupied, objects );
		}
		else 
		{
			m_points[itr].GetObjectsOnPoint( objects );
		}
	}

	return (int) objects.size();

} // end of GetObjectsOnPath

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
CPath::GetNextHldOfs( const CRoadPos& cPos, int* crdrIdOut , int lookahead) const{
	/* old code (does not work)
	//check to see if we in and intersection, if so check for halt line
    if (!cPos.IsRoad()){
        int id;
        if (GetCrdrFromIntrscn(cPos.GetIntrsctn().GetId(),id,&cPos)){
            CCrdr crdr(GetCved(),id);
            double dist = cPos.GetDistance(id);
            if (crdr.IsValid()){
                CHldOfs offset;
                if (crdr.GetHldOfByDist(offset,dist)){
                    return offset;
                }
            }
        }
    }
    CIntrsctn intr= GetNextIntrsctn(cPos);
    if (!intr.IsValid()){
        return CHldOfs();
    }
    int id;
    if (GetCrdrFromIntrscn(intr.GetId(),id,&cPos)){
        CCrdr crdr(GetCved(),id);
        vector<CHldOfs> hldOfsList;
        crdr.GetAllHldOfs(hldOfsList);
        if (crdrIdOut){
            *crdrIdOut = crdr.GetId();
        }
        if (hldOfsList.size() > 0){
            return *hldOfsList.begin();
        }else{
            return CHldOfs();
        }
    }else{
        return CHldOfs();
    }
	*/


	CIntrsctn intr;
	vector<CHldOfs> hldOfsList;
	TCrdrVec crdrVec;

	// get the next or current intersection
	intr = cPos.GetNextIntrsctn();
	if (!intr.IsValid()){
		return CHldOfs();
	}

	CLane lane = cPos.GetLane();
	if (cPos.IsRoad() && !lane.IsValid()){
		return CHldOfs();
	}

	int n = 0;
	if (lookahead < 1){
		lookahead = 1;
	}
	
	// check if point is on a road or intersection
	// may return the wrong haltline if a point lies on two different corridors
	if (!cPos.IsRoad()){
		while( n < lookahead ){
			// check if any corridors contain the point and if so, return the haltline
			intr.GetAllCrdrs(crdrVec);

			// no corridors in intersection
			if (crdrVec.size() == 0){
				return CHldOfs();
			}

			for(int i=0; i<crdrVec.size(); i++){
				if (cPos.HasCorridor(crdrVec[i].GetRelativeId())){
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
			for(int i=0; i<crdrVec.size(); i++){
				if (cPos.HasCorridor(crdrVec[i].GetRelativeId())){
					if (crdr == -1){
						crdr = i;
					}
					if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
						crdr = i;
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

			for(int i=0; i<crdrVec.size(); i++){
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
			for(int i=0; i<crdrVec.size(); i++){
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					crdr = i;
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
//  crdrIdOut - (output) the relative id of the corridor the halt line is located on
// 	lookahead - (optional) specifies the number of subsequent intersections to check
//		if the first intersection does not contain a halt line. GetNextHldOfs() will
//		stop looking for subsequent halt lines if a halt line is found.
//
// Returns: the distance to the next halt line or -1 upon failure
//
//////////////////////////////////////////////////////////////////////////////
double
CPath::GetDistToNextHldOfs( const CRoadPos& cPos , int lookahead ) const{
	/* old code (may not work)
	//check to see if we in and intersection, if so check for halt line
    if (!cPos.IsRoad()){
        int id;
        if (GetCrdrFromIntrscn(cPos.GetIntrsctn().GetId(),id,&cPos)){
            CCrdr crdr(GetCved(),id);
            double dist = cPos.GetDistance(id);
            if (crdr.IsValid()){
                CHldOfs offset;
                if (crdr.GetHldOfByDist(offset,dist)){
                    return offset.GetDistance() - dist;
                }
            }
        }
    }
	double distToNextIntrsctn = GetDistToNextIntrsctn(cPos);
    CIntrsctn intr= GetNextIntrsctn(cPos);
    if (!intr.IsValid()){
        return -1;
    }
    int id;
    if (GetCrdrFromIntrscn(intr.GetId(),id,&cPos)){
        CCrdr crdr(GetCved(),id);
        vector<CHldOfs> hldOfsList;
        crdr.GetAllHldOfs(hldOfsList);
        if (hldOfsList.size() > 0){
            double crdrDist = hldOfsList.begin()->GetDistance();
            return crdrDist + distToNextIntrsctn;
        }else{
            return -1;
        }
    }else{
        return -1;
    }
    return -1;
	*/

	CIntrsctn intr;
	vector<CHldOfs> hldOfsList;
	TCrdrVec crdrVec;

	// get the next or current intersection
	intr = cPos.GetNextIntrsctn();
	if (!intr.IsValid()){
		return -1;
	}

	CLane lane = cPos.GetLane();
	if (cPos.IsRoad() && !lane.IsValid()){
		return -1;
	}

	int n = 0;
	if (lookahead < 1){
		lookahead = 1;
	}

	double dist = 0;
	
	// check if point is on a road or intersection
	// may return the wrong haltline if a point lies on two different corridors
	if (!cPos.IsRoad()){
		// query the next few intersections for haltlines
		while( n < lookahead ){
			// check if any corridors contain the point and if so, return the haltline
			intr.GetAllCrdrs(crdrVec);

			if (crdrVec.size() == 0){
				return -1;
			}

			for(int i=0; i<crdrVec.size(); i++){
				if (cPos.HasCorridor(crdrVec[i].GetRelativeId())){
					crdrVec[i].GetAllHldOfs(hldOfsList);
					if (hldOfsList.size() > 0){
						if (n == 0){
							return hldOfsList.begin()->GetDistance() - cPos.GetDistanceOnLane(crdrVec[i].GetRelativeId());
						} else {
							return hldOfsList.begin()->GetDistance() + dist;
						}
					}
				}
			}

			// no haltlines found
			int crdr = -1;
			for(int i=0; i<crdrVec.size(); i++){
				if (cPos.HasCorridor(crdrVec[i].GetRelativeId())){
					if (crdr == -1){
						crdr = i;
					}
					if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
						crdr = i;
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
				dist += lane.GetRoad().GetLinearLength() + crdrVec[crdr].GetLength() - cPos.GetDistanceOnLane(crdrVec[crdr].GetRelativeId());
			} else {
				dist += lane.GetRoad().GetLinearLength() + crdrVec[crdr].GetLength();
			}
			n++;
		}
	} else {
		// query the next few intersections for haltlines
		dist += cPos.GetRoad().GetLinearLength() - cPos.GetDistanceOnLane();
		while(n < lookahead){
			// get all corridors connected to the lane given by the position
			intr.GetCrdrsStartingFrom(lane, crdrVec);

			// no corridors in intersection
			if (crdrVec.size() == 0){
				return -1;
			}

			for(int i=0; i<crdrVec.size(); i++){
				crdrVec[i].GetAllHldOfs(hldOfsList);
				if (hldOfsList.size() > 0){
					return hldOfsList.begin()->GetDistance() + dist;
				}
			}

			// no haltlines found, look for straight corridor so we can find the next intersection
			int crdr = 0;
			for(int i=0; i<crdrVec.size(); i++){
				if (crdrVec[i].GetCrdrDirection() == CCrdr::eSTRAIGHT){
					crdr = i;
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
// Description: Returns the next intersection from the given point.
//
// Remarks: If the cRoadPos lies on an intersection, then that 
//  intersection is	returned.  Otherwise, the next intersection 
//  on the path is returned.
//
// Arguments:
// 	cRoadPos - a valid CRoadPos that lies on the path.
//
// Returns: A CIntrsctn that is the intersection after the 
//  cRoadPos parameter. If cRoadPos does not lie on the path, or 
//  if there is no next intersection on the path, an invalid 
//  CIntrsctn instance will be returned.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CPath::GetNextIntrsctn( const CRoadPos& cRoadPos ) const
{
	int curIdx = m_pathPointIdx;
	cTPathIterator itr = FindPathPoint( cRoadPos, curIdx );

	// If a valid CPathPoint was returned
	bool validPathPoint = itr != m_points.end();
	if( validPathPoint ) 
	{
		// If that CPathPoint lies on a road
		if( m_points[itr].m_isRoad ) 
		{
			// Skip ahead to the next point on the path, which should be 
			// an intersection.
			itr++;
		}

		// If itr is pointing to a valid point,	return that intersection.
		validPathPoint = itr != m_points.end();
		if( validPathPoint )  return m_points[itr].m_intrsctn;
	}

	// Otherwise, return an invalid intersection.
	CIntrsctn intrsctn;
	return intrsctn;
} // end of GetNextIntrsctn



//////////////////////////////////////////////////////////////////////////////
//
// Description: Figures out if the current intersection is a two-road
//  intersection.  If the current path point is a road then the function
//  looks at the next intersection on the path.
//
// Remarks: 
//
// Arguments:
// 	cRoadPos - A valid CRoadPos that lies on the path.
//
// Returns: A boolean indicating if the current intersection is a two-
//  road intersection.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::IsIntrsctnTwoRoad( const CRoadPos& cRoadPos ) const
{
	int curIdx = m_pathPointIdx;
	cTPathIterator itr = FindPathPoint( cRoadPos, curIdx );

	// If a valid CPathPoint was returned
	bool validPathPoint = itr != m_points.end();
	if( validPathPoint ) 
	{
		// If that CPathPoint lies on a road
		if( m_points[itr].m_isRoad ) 
		{
			// Skip ahead to the next point on the path, which should be 
			// an intersection.
			itr++;
			validPathPoint = itr != m_points.end();
			if( !validPathPoint )  return false;
		}

		return m_points[itr].m_intrsctn.IsValid() && m_points[itr].m_intrsctn.IsTwoRoad();
	}

	return false;
} // end of GetNextIntrsctn


///////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the distance to the next intersection from 
//  the given point.
//
// Remarks: If the given cRoadPos lies on an intersection, or if 
//  there is no	next intersection, 0.0 will be returned.
//
// Arguments:
// 	cRoadPos - A valid CRoadPos that lies on the path.
//
// Returns: The distance to the next intersection on the path.
//
//////////////////////////////////////////////////////////////////////////////
double
CPath::GetDistToNextIntrsctn( const CRoadPos& cRoadPos ) const
{
	/* does not always work, sometimes returns 0 for a valid CRoadPos */

	int curIdx = m_pathPointIdx;
	cTPathIterator 	itr = FindPathPoint( cRoadPos, curIdx );
	double distance = 0.0f;

	// If a valid CPathPoint was returned
	bool validPathPoint = itr != m_points.end();
	if( validPathPoint ) 
	{
		// If that CPathPoint lies on a road
		if( m_points[itr].m_isRoad ) 
		{
			// Since the next node should be an intersection, the 
			// 	distance to the end of the current node is the 
			// 	distance to the next intersection.
			distance = fabs(
						m_points[itr].GetEndDist() - cRoadPos.GetDistance()
						);
		}
	}

	return distance;
} // end of GetDistToNextIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the next corridor from the given point.
//
// Remarks: If the cRoadPos lies on an intersection, then the current 
//   corridor is returned.  Otherwise, the next connecting corridor 
//   on the path is returned. 
//
// Arguments:
// 	 cRoadPos  - A valid CRoadPos that lies on the path.
//   checkLane - (optional) A boolean that indicates if the function 
//               should check to make sure if the lane in the given 
//               cRoadPos connects to the ensuing corridor.
//
// Returns: A CCrdr that is on the intersection after the cRoadPos 
//   parameter.	If the cRoadPos lies on a lane, the corridor is 
//   connected to that lane. If cRoadPos does not lie on the path, 
//   or if there is no next intersection on the path, an invalid 
//   CIntrsctn instance will be returned.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CPath::GetNextCrdr( const CRoadPos& cRoadPos, bool checkLane ) const
{

	int curIdx = m_pathPointIdx;
	cTPathIterator itr = FindPathPoint( cRoadPos, curIdx );

	//
	// Make sure input parameter lies on the path.
	//
	if( itr != m_points.end() ) 
	{
		if( !m_points[itr].m_isRoad ) 
		{
			// The input parameter lies on an intersection.
			// Return the parameter's corridor.
			//
            int idx = cRoadPos.GetIntrsctn().GetCrdrIdx();
            auto crdrBitSet = m_points[itr].GetLaneCrdrMask();
            vector< pair<int,double> > crds;
            cRoadPos.GetCorridors(crds);
            //we need to find the intersection between the crdrs the 
            //roadpos is on, and the path has
            for (auto citr = crds.begin(); citr != crds.end(); citr++){
                int id = citr->first - idx;
                if (crdrBitSet.test(id)){
                    return cRoadPos.GetCorridor(id);
                }
            }
            return cRoadPos.GetCorridor();
		}
		else 
		{
			//
			// The input parameter lies on a road.
			//

			//
			// Skip ahead to the next point on the path, 
			// which should be an intersection.
			//
			itr++;

			//
			// If itr is pointing to a valid point on an intersection, 
			// return the first corridor found whose source lane 
			// index matches the lane of the parameter and whose 
			// destination lane index matches the lane of the next
			// road on the path.  If there's no path beyond the 
			// intersection then simply check for the source lane
			// idx.
			//
			if( ( itr != m_points.end() ) && ( !m_points[itr].m_isRoad ) ) 
			{
				// 
				int crdrId = m_points[itr].GetFirstLaneCrdrIdx();
				int intrsctnCrdrIdx = crdrId + m_points[itr].m_intrsctn.GetCrdrIdx();
				TCrdr* pCrdr = BindCrdr( intrsctnCrdrIdx );

				// get the source lane index
				int srcLnIdx = cRoadPos.GetLane().GetRelativeId();
				srcLnIdx += cRoadPos.GetRoad().GetLaneIdx();

				int numCrdrs = m_points[itr].m_intrsctn.GetNumCrdrs();

				// get the destination road id and mask of lanes, if one exists
				bool haveDstRoad = false;
				int dstRoadIdx = -1;
				bitset<cCV_MAX_CRDRS> dstLaneMask;

				itr++;
				if ( itr != m_points.end() ) 
				{
					haveDstRoad = true;
					dstRoadIdx = m_points[itr].GetRoad().GetId();
					dstLaneMask = m_points[itr].GetLaneCrdrMask();
				}
                
				// iterate through the crdrs until a match is found
				for( ; crdrId < numCrdrs; crdrId++, pCrdr++ ) 
				{
					if( !checkLane || pCrdr->srcLnIdx == srcLnIdx )
					{
						if( haveDstRoad ) 
						{
							// make sure road idx match
							if( pCrdr->dstRdIdx == dstRoadIdx )
							{
								TLane* pCrdrDstLn = BindLane( pCrdr->dstLnIdx );
								if( dstLaneMask.test( pCrdrDstLn->laneNo ) ) 
								{
                                    CCrdr crdr( GetCved(), pCrdr );
									return crdr;
								}
							}
						}
						else 
						{
							CCrdr crdr( GetCved(), pCrdr );
							return crdr;
						}

					}
				}
			}
		} // If that CPathPoint lies on a road
	} // If a valid CPathPoint was returned

	// Otherwise, return an invalid corridor.
	CCrdr crdr;
	return crdr;
} // end of GetNextCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns a corridor associated with the given intersection
//  along the path.
//
// Remarks: If the cRoadPos lies on an intersection, then the current 
//  corridor is returned.  Otherwise, the next connecting corridor 
//  on the path is returned. 
//
// Arguments:
// 	cIntrsctnId - The intersection on which to find the corridor.
//  crdrId - (output) The corridor's identifier.
//	cpStart - (Optional) Pointer to a valid CRoadPos instance.  Default
//      value is 0, indicating that the path should be searched from the
//      beginning. 
//	srcLaneId - (Optional) Specifies the source lane, (Path may have 
//		multiple crds) 
//
// Returns: A boolean indicating if the given intersection was found
//  along the path.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::GetCrdrFromIntrscn( 
			int cIntrsctnId,
			int& crdrId,
			const CRoadPos* cpStart,
			int srcLaneId
			) const
{

	// 
	// Set the path iterator either to the start of the path or to 
	// the pathpoint associated with the given cpStart road position
	// if it is specified.
	//
	cTPathIterator itr = m_points.begin();
#ifdef _DEBUG
	TIntrsctn temp; //for the debugger
#endif
	if( cpStart ) 
	{
		int curIdx = m_pathPointIdx;
		itr = FindPathPoint( *cpStart, curIdx );
	}
    bool matchRoadPosWithCrdr = (cpStart != nullptr) && 
        (cpStart->IsValid() && !cpStart->IsRoad() && cpStart->GetIntrsctn().GetId() == cIntrsctnId);
	//
	// Find the intersection along the path that matches the given
	// intersection.  Return the relative id of the first corridor
	// on the intersection.
	//
	for ( ; itr != m_points.end(); itr++ )
	{
		bool foundIntrsctn = (
			!m_points[itr].m_isRoad && 
			cIntrsctnId == m_points[itr].GetIntrsctn().GetId()
			);
		if( foundIntrsctn )
		{
            //if our starting point is in the target intersection, we want to find the intersection
            //the passed in roadpos with the path, if one exist return the first one that satifies 
            //this condition
            if (matchRoadPosWithCrdr){
                auto bits = m_points[itr].GetLaneCrdrMask();
                vector<int> crdrs;
                cpStart->GetCorridors(crdrs);
                int idx = cpStart->GetIntrsctn().GetCrdrIdx();
                for (auto itrc = crdrs.begin(); itrc != crdrs.end(); itrc++){
                    if (bits.test(*itrc - idx)){
                        if (srcLaneId < 0){
                            crdrId  = *itrc - idx;
                            return true;
                        }
                        else{
                            CVED::CCrdr crd(cpStart->GetIntrsctn(), *itrc - idx);
                            if(crd.GetSrcLn().GetId() == srcLaneId){
                                crdrId  = *itrc - idx;
                                return true;
                            }
                        }
                    }
                }
            }
            //now find the crdr that matches 
			if (srcLaneId >= 0){
				int tempI;
				//CLane(m_cpCved,srcLaneId);
				tempI = m_points[itr].GetFirstCrdrWithLaneId(srcLaneId);
				if (tempI < 0)
					return false;
				else{
					crdrId = tempI;
					return true;
				}
			}else{
				crdrId = m_points[itr].GetFirstLaneCrdrIdx();
			}
			return true;
		}
	}

	return false;
} // end of GetCrdrFromIntrscn

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetOncomingPath
//	Generates a vector of CPaths that move in the opposite direction with respect
//	to the lane that the cStartRoadPos is located. And these CPaths are approaching
//	one lane which is the closest opposite direction lane. 
// 
// Remarks:  Note a cStartRoadPos contains implicit direction information in the 
//	 CLane() and CCrdr() member data.
//	
// Arguments:
//	cStartRoadPos - (Input) Contains the current road position. 	
//	paths - (Output) contains the CPaths that move in the opposite
//		direction from the cStartRoadPos upon return. Note: the vector
//		is cleared at the beginning of the function.
//
// Returns: The number of paths that move in the opposite direction from the
//	path where the cStartRoadPos is located.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetOncomingPath(
			const CRoadPos& cStartRoadPos, 
			vector<CPath>& paths
			) const
{
	
	paths.clear();
	//
	// Error checking for the parameter cStartRoadPos.
	//
	if( !cStartRoadPos.IsValid() )
	{
		return 0;
	}

	CIntrsctn intrsctn;
	//
	// If the starting position is on a road, then 
	// get the next intersection. If the starting 
	// position is on an intersection, then 
	// get the next intersection with respect to 
	// the corridor's destination lane.
	if( cStartRoadPos.IsRoad() )
	{	
		// On a road
		intrsctn = cStartRoadPos.GetLane().GetNextIntrsctn();	
	}
	else
	{
		// On an intersection.
		intrsctn = cStartRoadPos.GetCorridor().GetDstntnLn().GetNextIntrsctn();
		
	}

	//
	// Need to find the closest lane that moves in the opposite direction
	// from the lane where the cStartRoadPos is located. So keep moving
	// left from the cStartRoadPos lane until the lane in the opposite 
	// direction is reached.
	CLane closestOppositeDirLn;
	CLane leftLane;
	if( cStartRoadPos.IsRoad() )
	{

		// 
		// cStartRoadPos is on a road.
		//
		CLane currLane = cStartRoadPos.GetLane();
		
		//
		// If the cStartRoadPos' current lane is already
		// the leftmost lane, return.
		//
		if ( currLane.IsLeftMost() ) {

			return 0;

		}
		leftLane = cStartRoadPos.GetLane().GetLeft();

		if( !leftLane.IsValid() )
		{
			return 0;
		}
		else
		{

			cvELnDir leftLaneDir = leftLane.GetDirection();
			cvELnDir currLaneDir = currLane.GetDirection();
			 
			bool sameDirLanes = leftLaneDir == currLaneDir;

			while( sameDirLanes && !leftLane.IsLeftMost() )
			{
				
				leftLane = leftLane.GetLeft();
				if( !leftLane.IsValid() )
				{
					return 0;
				}
				leftLaneDir = leftLane.GetDirection();
				sameDirLanes = leftLaneDir == currLaneDir;
			}		
			if ( !sameDirLanes )
			{
				closestOppositeDirLn = leftLane;	;
			}
		}
	}
	else
	{
		// 
		// cStartRoadPos is on an intersection.
		//
		CLane currLane = cStartRoadPos.GetCorridor().GetDstntnLn();
		//
		// If the starRoadPos's destination lane is already
		// the leftmost lane, return.
		//
		if ( currLane.IsLeftMost() ) {

			return 0;

		}
		leftLane = cStartRoadPos.GetCorridor().GetDstntnLn().GetLeft();

		if( !leftLane.IsValid() )
		{
			return 0;
		}
		cvELnDir leftLaneDir = leftLane.GetDirection();
		cvELnDir currLaneDir = currLane.GetDirection();
		 
		bool sameDirLanes = leftLaneDir == currLaneDir;

		while( sameDirLanes && !leftLane.IsLeftMost() )
		{		
			leftLane = leftLane.GetLeft();
			if( !leftLane.IsValid() )
			{
		
				return 0;
			}
			leftLaneDir = leftLane.GetDirection();
			sameDirLanes = leftLaneDir == currLaneDir;
			
		}
		if ( !sameDirLanes )
		{
			closestOppositeDirLn = leftLane;	;
		}
		
	}
	
	vector<CCrdr> desiredCrdrs;

	// Hold all the corridors in the proper intersetion in a vector.
	// Find only those corridors that are approaching the closest opposite
	// direction lane and hold them in another vector.
	//
	vector<CCrdr> allCrdrs;
	intrsctn.GetAllCrdrs( allCrdrs );	
	vector<CCrdr>::iterator i;
	for ( i= allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr aCrdr = *i;
		if( aCrdr.GetDstntnLn() == closestOppositeDirLn )
		{
			desiredCrdrs.push_back( aCrdr );
		}
	}

	//
	// The number of corridors in the desired corridors vector is the 
	// number of the paths that move in the opposite direction from 
	// the cStartRoadPos. Now need to build a path based on each of 
	// the corridor and insert these paths in the output vectors.
	//
	vector<CCrdr>::iterator j;
	for( j = desiredCrdrs.begin(); j != desiredCrdrs.end(); j++ )
	{
		CCrdr dCrdr = *j;
		double dist;
		if( dCrdr.GetSrcLn().GetDirection() == ePOS )
		{
			dist = 0.0;
		}
		else
		{
			double lengthOfSrcLn = dCrdr.GetSrcLn().GetRoad().GetLinearLength();
			dist = lengthOfSrcLn;
		}
		
		CRoadPos srcRoadPos( 
					dCrdr.GetSrcLn().GetRoad(),
					dCrdr.GetSrcLn().GetRelativeId(),
					dist,
					0.0
					);
		CPath path( srcRoadPos );
		//
		// Find a roadPos to connect the path to. Can't use
		// cStartRoadPos directly here because the direction is
		// the opposite. If the cStartRoadPos is on a road, this
		// roadPos to be found is a corresponding road position 
		// of the cStartRoadPos, on the closest opposite direction lane. 
		// If the cStartRoadPos is on an intersection, this roadPos to 
		// be found is a corresponding road position of the cStartRoadPos
		// on the intersection.
		//
		if( cStartRoadPos.IsRoad() )
		{
			// On road.
			CRoadPos endRoadPos( 
						cStartRoadPos.GetRoad(),
						closestOppositeDirLn.GetRelativeId(),
						cStartRoadPos.GetDistance(),
						0.0
						);
			path.Append( endRoadPos );
			paths.push_back( path );
		}
		else
		{
			//
			// On intersection. Connect the path to a road position
			// that lies just before entering the intersection, then
			// exent the path further by the proper distance on the 
			// intersection. This ensures the objects on the intersection
			// get picked up.
			//
			CLane lane;
			double distance;
			lane = cStartRoadPos.GetCorridor().GetDstntnLn();

			if( lane.GetDirection() == ePOS )
			{

				distance = 0.0;
			}
			else
			{
				distance = lane.GetRoad().GetLinearLength();
			}
			CRoadPos endRoadPos(
						lane.GetRoad(),
						closestOppositeDirLn.GetRelativeId(),
						distance,
						0.0
						);
			path.Append( endRoadPos );
			double distOnIntrsctn = cStartRoadPos.GetCorridor().GetLength()  -
				cStartRoadPos.GetDistance() ;
			path.Append( distOnIntrsctn );
			paths.push_back( path );
		
		}
}

	return (int) paths.size();

} // end of GetOncomingPath

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetApproachingPaths
//	Generates a vector of CPaths that approach the cStartRoadPos parameter from 
//	behind. It will include those paths that are approaching the same direction
//  lanes of the cStartRoadPos lane.
//
// Remarks: Note that a cStartRoadPos contains implicit direction information in 
//	the CLane() and CCrdr() member data.  
//
// Arguments:
//	cStartRoadPos - (Input) Contains a road position to which the approaching  
//		paths should head towards.
//	paths - (Output) contains the approaching CPaths upon return. Note: the 
//		vector is cleared at the beginning of the function.
//
// Returns: The number of approaching paths found.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetApproachingPaths(
			const CRoadPos& cStartRoadPos, 
			vector<CPath>& paths 
			)const
{

	paths.clear();
	//
	// Error checking for the parameter cStartRoadPos.
	//
	if( !cStartRoadPos.IsValid() )
	{
		return 0;
	}
	
	CIntrsctn intrsctn;
	//
	// If the starting position is on a road, then 
	// get the previous intersection. If the starting 
	// position is on an intersection, then get 
	// the intersection.
	//
	if( cStartRoadPos.IsRoad() )
	{
		// On a road.
		intrsctn = cStartRoadPos.GetLane().GetPrevIntrsctn();

	}
	else
	{
		// On an intersection.
		intrsctn = cStartRoadPos.GetIntrsctn();
	}

	//
	// Get all the lanes that are running in the same direction
	// as the current lane where the cStartRoadPos is located. Hold
	// these lanes in a vector.
	//
	CRoad road;
	CLane currLane;
	if( cStartRoadPos.IsRoad() )
	{
		// On a road.
		road = cStartRoadPos.GetRoad();
		currLane = cStartRoadPos.GetLane();
	}
	else
	{
		// On an intersection.
		road = cStartRoadPos.GetCorridor().GetDstntnLn().GetRoad();
		currLane = cStartRoadPos.GetCorridor().GetDstntnLn();
	}
	int numLanes = road.GetNumLanes();	
	/*static*/ vector<cvTLane> sameDirLanes;
	/*if ( sameDirLanes.capacity() <= 0 )*/  sameDirLanes.reserve( 10 );
	sameDirLanes.clear();
	int firstLaneIdx = road.GetLaneIdx();
	cvTLane* pLane;
	for( int k = 0; k <= (numLanes - 1); k++ )
	{
		pLane = BindLane( firstLaneIdx + k );
		if( pLane->direction == currLane.GetDirection() )
		{
			sameDirLanes.push_back( *pLane );
		}
	}

	//
	// Hold all the corridors of the proper intersection in a vector.
	// Compare all the corridor's destination lanes 
	// with all the lanes that are running in the same direction 
	// as the current lane to see if they are equal. If so,
	// hold these corridors in another vector.
	//
	vector<CCrdr> allCrdrs;
	intrsctn.GetAllCrdrs( allCrdrs );
	/*static*/ vector<CCrdr> desiredCrdrs;
	/*if ( desiredCrdrs.capacity() <= 0 )*/  sameDirLanes.reserve( 20 );
	desiredCrdrs.clear();
	vector<CCrdr>::iterator i;
	for ( i= allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr aCrdr = *i;
		vector<cvTLane>::iterator j;
		for ( j = sameDirLanes.begin(); j != sameDirLanes.end(); j++ )
		{
			cvTLane dstntnLane = *j;
			if( aCrdr.GetDstntnLn( ).GetRoad().GetId() == dstntnLane.roadId &&
				aCrdr.GetDstntnLn( ).GetDirection() == dstntnLane.direction )
			{
				desiredCrdrs.push_back( aCrdr );
				break;
			}
		}		
	}
		
	//
	// Have found all the corridors that are approaching the cStartRoadPos 
	// from behind. Now need to build a path based on each corridor and 
	// insert these paths in the output vector.
	//	
	vector<CCrdr>::iterator m;
	for( m = desiredCrdrs.begin(); m != desiredCrdrs.end(); m++ )
	{
		CCrdr dCrdr = *m;
		double dist;
		if( dCrdr.GetSrcLn().GetDirection() == ePOS )
		{
			dist = 0.0;
		}
		else
		{
			double lengthOfSrcLn = dCrdr.GetSrcLn().GetRoad().GetLinearLength();
			dist = lengthOfSrcLn;
		}
		
		CRoadPos srcRoadPos( 
					dCrdr.GetSrcLn().GetRoad(),
					dCrdr.GetSrcLn().GetRelativeId(),
					dist,
					0.0 
					);
		CRoadPos crdrRoadPos( dCrdr.GetIntrsctn(), dCrdr );
		CPath path( srcRoadPos );
		path.Append( crdrRoadPos );
		path.Append( cStartRoadPos );
		if( path.GetLength( &srcRoadPos, &cStartRoadPos ) != 0.0 )
		{
			paths.push_back( path );
		}
		
	}
			
	return (int) paths.size();
} // end of GetApproachingPaths


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetApproachingPathsLane
//	Generates a vector of CPaths that approach the same lane that the cStartRoadPos 
//	parameter is located. Note that the CPaths approach the cStartRoadPos from behind.
//	 
// Remarks: This function is very similar to the GetApproachingPaths function.
//  The only difference is that the output CPaths are only the paths that are 
//  approaching the same lane where the cStartRoadPos is located. (The CPaths that
//  are approaching the same direction lanes of the cStartRoadPos lane are
//  not counted as in the GetApproachingPaths function). Also note that a 
//  cStartRoadPos contains implicit direction information in the CLane() and CCrdr() 
//  member data.  The search will extend only through the intersection previous 
//  to that point on the path.  
//
// Arguments:
//	cStartRoadPos - (Input) Contains a road position to which the approaching paths 
//		should head towards.
//	paths - (Output) contains the approaching CPaths upon return. Note: the vector
//		is cleared at the beginning of the function.
//
// Returns: The number of approaching paths found.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetApproachingPathsLane(
			const CRoadPos& cStartRoadPos, 
			vector<CPath>& paths
			) const
{

	paths.clear();
	//
	// Error checking for the parameter cStartRoadPos.
	//
	if( !cStartRoadPos.IsValid() )
	{
		return 0;
	}

	CIntrsctn intrsctn;
	//
	// If the starting position is on a road, then 
	// get the previous intersection. If the starting
	// position is on an intersection, then get the
	// intersection.
	//	
	if( cStartRoadPos.IsRoad() )
	{	
		// on a road.
		intrsctn = cStartRoadPos.GetLane().GetPrevIntrsctn();
	}
	else
	{
		// on an intersection.
		intrsctn = cStartRoadPos.GetIntrsctn();
	}

	//
	// Get the current lane where the cStartRoadPos is located.
	//
	CLane currLane;
	if( cStartRoadPos.IsRoad() )
	{
		// On a road.
		currLane = cStartRoadPos.GetLane();
	}
	else
	{
		// On an intersection.
		currLane = cStartRoadPos.GetCorridor().GetDstntnLn();
	}

	vector<CCrdr> desiredCrdrs;
	// 
	// Use a vector to hold all the corridors in the intersection.
	// Iterate the vector to compare if any of the corridors'
	// destintion lane is the same lane where the cStartRoadPos
	// is located. If so, put these corridors in another vector.
	//
	vector<CCrdr> allCrdrs;
	intrsctn.GetAllCrdrs( allCrdrs );	
	vector<CCrdr>::iterator i;
	for ( i= allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr aCrdr = *i;
		if( aCrdr.GetDstntnLn() == currLane )
		{
			desiredCrdrs.push_back( aCrdr );
		}
	}
	
	//
	// Have found all the corridors that are approaching the lane of the 
	// cStartRoadPos from behind. Now need to build a path based on each 
	// corridor and insert these paths in the output vector.
	//	
	vector<CCrdr>::iterator j;
	for( j = desiredCrdrs.begin(); j != desiredCrdrs.end(); j++ )
	{
		CCrdr dCrdr = *j;
		double dist;
		if( dCrdr.GetSrcLn().GetDirection() == ePOS )
		{
			dist = 0.0;
		}
		else
		{
			double lengthOfSrcLn = dCrdr.GetSrcLn().GetRoad().GetLinearLength();
			dist = lengthOfSrcLn;
		}
		
		CRoadPos srcRoadPos( 
					dCrdr.GetSrcLn().GetRoad(),
					dCrdr.GetSrcLn().GetRelativeId(),
					dist,
					0.0 
					);
		CPath path( srcRoadPos );
		path.Append( cStartRoadPos );
		if( path.GetLength( &srcRoadPos, &cStartRoadPos ) != 0.0 )
		{
			paths.push_back( path );
		}
		
	}
			
	return (int) paths.size();
} // end of GetApproachingPathsLane

///////////////////////////////////////////////////////////////////////////
double
CPath::GetNextTurnAngle( const CRoadPos& cRoadPos ) const
{

	//
	// Find the next intersection.  If already on an intersection then
	// return the next intersection on the path.
	//
	int curIdx = m_pathPointIdx;
	cTPathIterator itr = FindPathPoint( cRoadPos, curIdx );

	// If a valid CPathPoint was returned
	if ( itr != m_points.end() ) {

		// For roads, skip ahead to the next point on the path, which 
		// should be an intersection.  For intersections, skip ahead
		// two points.
		itr++;
		if (itr == m_points.end() ) return -1.0;

		if ( m_points[itr].m_isRoad )  itr++;

		// Make sure we have a valid intersection.
		if ( itr != m_points.end() ) {

			// The destination road is the next path point.
			double angle = m_points[itr].GetSrcDestAngle();
			return angle;

		}
		else {

			return -1.0;

		}

	}

	return -1.0;

} // end of GetNextTurnAngle


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets the lane that's closest clockwise to the current 
//  road path point after the upcoming intersection.
//
// Remarks:  If the current path point is an intersection then this
//   function treats the following path point as the road from which
//   to find the next clockwise lane.
//
// Arguments:
//  nextLane - The next lane closest clockwise.
//
// Returns: A boolean indicating if it was unables to find any lanes.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::GetNextLaneClockwise( CLane& nextLane )
{
	cTPathIterator itr = FindPathPoint( m_pathPointIdx );
	if( !m_points[itr].IsRoad() ) 
	{
		//
		// On an intersection.  Find the next lane clockwise from
		// the next road.
		//
		itr++;
		if( itr == m_points.end() )  return false;
	}

	return m_points[itr].GetNextLaneClockwise( nextLane );
}  // end of GetNextLaneClockwise


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets the lane that's closest counter-clockwise to the 
//  current road path point after the upcoming intersection.
//
// Remarks:  If the current path point is an intersection then this
//   function treats the following path point as the road from which
//   to find the next counter-clockwise lane.
//
// Arguments:
//  nextLane - The next lane closest counter-clockwise.
//
// Returns: A boolean indicating if it was unables to find any lanes.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::GetNextLaneCounterClockwise( CLane& nextLane )
{
	cTPathIterator itr = FindPathPoint( m_pathPointIdx );
	if( !m_points[itr].IsValid() )  return false;

	if( !m_points[itr].IsRoad() ) 
	{
		//
		// On an intersection.  Find the next lane counter-clockwise from
		// the next road.
		//
		itr++;
		if( itr == m_points.end() )  return false;
	}

	return m_points[itr].GetNextLaneCounterClockwise( nextLane );
}  // end of GetNextLaneCounterClockwise


/////////////////////////////////////////////////////////////////////////////
//		Inherited functions	
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// 
// Description: IsValid (protected)
// 	Returns true if the CPath is valid, false otherwise.
//
// Remarks:  A CPath is considered valid if its superclass is valid and
//  it contains one or more CPathPoints.
//
// Arguments:
//
// Returns: true or false
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::IsValid() const
{
	return ( CCvedItem::IsValid() && !m_points.empty() );
} // end of IsValid

//////////////////////////////////////////////////////////////////////////////
// 
// Description: AssertValid (private)
// 	Asserts false if the CPath is invalid.
//
// Remarks:  A CPath is considered valid if its superclass is valid and
//  it contains one or more CPathPoints.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::AssertValid() const
{
	CCvedItem::AssertValid();
	assert( !m_points.empty() );
} // end of AssertValid


//////////////////////////////////////////////////////////////////////////////
//		Private utility functions	
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns a cTPathIterator corresponding to the cRoadPos 
//   parameter.
//
// Remarks:  If a valid CPathPoint is passed in the pCurPoint parameter, 
//   the search is begun from that point.  Otherwise, the search is begun 
//   at the beginning of the current path.
//
// Arguments:
// 	cRoadPos - A valid CRoadPos instance.
//	start   - (Input)  Index of the CPathPoint from which to begin the search.
//			  (Output) Index of the returned CPathPoint.
//
// Returns:  An iterator pointing to a CPathPoint instance that the CRoadPos 
//   lies on.  If no such CPathPoint is found, the function returns 0.
//
//////////////////////////////////////////////////////////////////////////////
CPath::cTPathIterator
CPath::FindPathPoint( const CRoadPos& cRoadPos, int& start ) const
{
	//
	// If there are no points in the path, set start to -1 and
	// return 0.
	//
	if( m_points.empty() ) 
	{
		start = -1;
		return m_points.end();
	}

	cTPathIterator startOfSearch = FindPathPoint( start );

	//
	// If a valid startOfSearch was not found, set the start 
	// to the beginning of the list.
	//
	if( startOfSearch == m_points.end() ) 
	{
		startOfSearch = m_points.begin();
		start = 0;
	}

	cTPathIterator itr = startOfSearch;

	do 
	{
		//
		// If a point in the path is found that is similar to the 
		// CPathPoint constructed from the parameter, then take it.
		//
#if 0
		if( m_points[itr].IsRoad() )
		{
			gout << "    in road = " << m_points[itr].GetRoad() << endl;
			gout << "    laneMask = " << m_points[itr].GetLaneCrdrMask() << endl;
		}
		else
		{
			gout << "    in intr = " << m_points[itr].GetIntrsctn() << endl;
		}
#endif

		if( m_points[itr].Contains( cRoadPos ) )  
		{
			return itr;
		}
			
		// increment the iterator
		itr++; 
		start++;
		
		//
		// If we've reached the end of the list, return
		// to the beginning.
		//
		if( itr == m_points.end() ) 
		{
			itr = m_points.begin();
			start = 0;
		}
		
		//
		// Loop until we've reached the beginning of the search again.
		//
	} while( itr != startOfSearch );

	// no similar point was found, so return 0
	return m_points.end();

} // end of FindPathPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: (private) Returns a cTPathIterator that us at the given 
//  index in the list.
//
// Remarks: 
//
// Arguments:
//	idx - (Optional) Index of the cTPathIterator to return.  Default value 
//		is 0.
//
// Returns: An iterator pointing to a CPathPoint instance that the CRoadPos 
//	lies on.  If no such CPathPoint is found, the function returns 0.
//
//////////////////////////////////////////////////////////////////////////////
CPath::cTPathIterator
CPath::FindPathPoint( int idx ) const
{
	cTPathIterator itr;
	int curIdx;

	for( 
		itr = m_points.begin(), curIdx = 0; 
		itr != m_points.end(); 
		itr ++, curIdx++
		) 
	{
		if( curIdx == idx )  return itr;
	}
	return itr;
} // end of FindCurPathPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: RemoveAnyExtraCorridors (private)
// 	Utility function that scans over the current CPath and removes any 
// 	corridors from the intersection points that do not lead from the previous
// 	road to the next road.  
//
// Remarks: This is used to clean up any loose corridors left by the 
// 	CPathPoint::Append(CPathPoint) function.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::RemoveAnyExtraCorridors( void )
{
	int prevPoint = 0, curPoint = 0, nextPoint = 0;
	int crdrId, numCrdrs;
	TCrdr* pCrdr;
	int srcRoadId = 0, dstRoadId = 0;

#if PATH_DEBUG
	gout << "Before removal: " << endl;
	for (curPoint = m_points.begin(); curPoint != m_points.end(); curPoint++) {
		gout << "  " << *curPoint << endl;
		if (!curPoint->m_isRoad) {

			for (crdrId = 0, 
				 pCrdr = BindCrdr(crdrId+curPoint->m_intrsctn.GetCrdrIdx()); 
				 crdrId < curPoint->m_intrsctn.GetNumCrdrs(); crdrId++, pCrdr++) {

				if (curPoint->m_laneCrdrMask.test(crdrId)) {

					CRoad src(GetCved(), pCrdr->srcRdIdx);
					CRoad dst(GetCved(), pCrdr->dstRdIdx);
					gout << "    crdr[" << pCrdr->myId << "]: " << crdrId
						 << " runs from " << src.GetName() << "[" << src.GetId()
						 << "] to " << dst.GetName() << "[" << dst.GetId() << "]"
						 << endl;
				}
			}
		}
	}
#endif

	// If there's more than one point in the path
	if (m_points.size() > 1) {
	
		// For each point in the path
		for (curPoint = m_points.begin(), prevPoint = curPoint, nextPoint = curPoint;
			 curPoint != m_points.end(); curPoint++) {

			nextPoint++;

			//curStr = curPoint->GetString();

			// If the current point lies on an intersection
			if (!(m_points[curPoint].m_isRoad)) {

				// If prevPoint lies on a road
				if (m_points[prevPoint].m_isRoad) {
					// Get the desired source road Id
					srcRoadId = m_points[prevPoint].m_road.GetId();
					//prevStr = prevPoint->GetString();
				}
				else {
					srcRoadId = 0;
					//prevStr = "";
				}

				// If nextPoint lies on a road
				if ( (nextPoint != m_points.end()) && (m_points[nextPoint].m_isRoad) ) {
					// Get the desired destination road id
					dstRoadId = m_points[nextPoint].m_road.GetId();
					//nextStr = nextPoint->GetString();
				}
				else {
					dstRoadId = 0;
					//nextStr = "";
				}

				// For each corridor on the current intersection
				numCrdrs = m_points[curPoint].m_intrsctn.GetNumCrdrs();
				for (crdrId = m_points[curPoint].GetFirstLaneCrdrIdx(), 
					 pCrdr = BindCrdr(m_points[curPoint].m_intrsctn.GetCrdrIdx()+crdrId);
					 crdrId < numCrdrs;
					 crdrId++, pCrdr++) {
					
					// If the corridor does not lead from the srcRoadId
					// 	and to the dstRoadId, then remove it from the mask.
					if ( 
						 ( (srcRoadId != 0) && (srcRoadId != pCrdr->srcRdIdx) )
						 ||
						 ( (dstRoadId != 0) && (dstRoadId != pCrdr->dstRdIdx) ) 
					   )
					{
						m_points[curPoint].m_laneCrdrMask.reset(crdrId);
					}
					
				} // For each corridor on the current intersection

				// Clear out any other corridors set, because they're outside the
				//	max number of corridors in the intersection
				for (; crdrId < cCV_MAX_CRDRS; crdrId++)
					m_points[curPoint].m_laneCrdrMask.reset(crdrId);

			} // If the current point lies on an intersection

			prevPoint = curPoint;

		} // While nextPoint has not reached the end of the path
	} // If there's more than one point in the path

#if PATH_DEBUG
	gout << "After removal: " << endl;
	for (curPoint = m_points.begin(); curPoint != m_points.end(); curPoint++) {
		gout << "  " << *curPoint << endl;
		if (!curPoint->m_isRoad) {

			for (crdrId = 0, 
				 pCrdr = BindCrdr(crdrId+curPoint->m_intrsctn.GetCrdrIdx()); 
				 crdrId < curPoint->m_intrsctn.GetNumCrdrs(); crdrId++, pCrdr++) {

				if (curPoint->m_laneCrdrMask.test(crdrId)) {

					CRoad src(GetCved(), pCrdr->srcRdIdx);
					CRoad dst(GetCved(), pCrdr->dstRdIdx);
					gout << "    crdr[" << pCrdr->myId << "]: " << crdrId
						 << " runs from " << src.GetName() << "[" << src.GetId()
						 << "] to " << dst.GetName() << "[" << dst.GetId() << "]"
						 << endl;
				}
			}
		}
	}
#endif

} // end of RemoveAnyExtraCorridors


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets all intersections along the path that fall within
//   the given distance starting from the given road position.
//				
// Remarks:  Clears the given vector right away regardless of whether it
//   finds any intersections or not.  Starts search from the current 
//   intersection if the cStartRoadPos lies on an intersection.
//
// Arguments:
//   cStartRoadPos - Road position at which to start looking.
//   dist - Specifies how far to look ahead along the path for intersections.
//   intrsctns - (output) Write intersection identifiers to this vector.
//   skipDummyIntrsctns - (optional) If true, skip reporting dummy 
//       intersections--i.e. intersections with one source road and one
//       destination road.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CPath::GetIntrsctnsWithinDist( 
			const CRoadPos& cStartRoadPos, 
			double dist, 
			vector<int>& intrsctns,
			bool skipDummyIntrsctns,
			bool checkForLastMergeDist
			) const
{
	//
	// Clear the output intersection vector.
	//
	intrsctns.clear();

	//
	// Find the path point corresponding to the given road position.
	//
	int curIdx = m_pathPointIdx;
	cTPathIterator itr = FindPathPoint( cStartRoadPos, curIdx );

	//
	// Return if an invalid CPathPoint was returned.
	//
	bool invalidPathPoint = itr == m_points.end(); 
	if( invalidPathPoint )  return;

	//
	// This variable keeps track how far we've gone along the path.
	//
	double distCovered = 0.0;  // ft.

	//
	// Skip the current interection if needed.
	//
	bool onAnIntrsctn = !m_points[itr].m_isRoad;
	if( onAnIntrsctn )
	{
		//
		// The given position is on an intersection.  Add this intersection
		// to the output vector if it's not a dummy intersection and if the
		// cStartRoadPos's distance is not already beyond the distance where 
		// this corridor last intersets another corridor.
		//
		bool skipThisIntrsctn = 
				skipDummyIntrsctns && m_points[itr].GetIntrsctn().IsTwoRoad() &&
                !m_points[itr].GetIntrsctn().IsControlled();
       
		if( checkForLastMergeDist && !skipThisIntrsctn )
		{
			if( m_points[itr].GetIntrsctn().GetId() == cStartRoadPos.GetIntrsctn().GetId() )
			{
				CCrdr startCrdr = cStartRoadPos.GetCorridor();
				double dist;
				int crdrId;
				startCrdr.GetMrgDstLast( dist, crdrId );

//				gout << "last merge dist = " << dist << " with crdr " << crdrId;
//				gout << "  startDist = " << cStartRoadPos.GetDistance() << endl;

				if( dist > 1.0 && cStartRoadPos.GetDistance() > dist )
				{
					skipThisIntrsctn = true;
				}
			}
		}
		if( !skipThisIntrsctn )
		{
			intrsctns.push_back( m_points[itr].GetIntrsctn().GetId() );
		}

		distCovered = fabs( 
				cStartRoadPos.GetDistance() - 
				m_points[itr].GetEndDist()
				);

		itr++;
		bool invalidPathPoint = itr == m_points.end(); 
		if( invalidPathPoint )  return;

		distCovered += m_points[itr].GetLength();
	}	
	else
	{
		//
		// The given road position is on a road.
		//
		distCovered = fabs( 
				cStartRoadPos.GetDistance() -
				m_points[itr].GetEndDist()
				); 
	}

	while( distCovered < dist )
	{
		//
		// On a road, move to the intersection.
		//
		itr++;
		bool invalidPathPoint = itr == m_points.end(); 
		if( invalidPathPoint )  return;

		//
		// Add the intersection id to the output vector.
		//
		bool skipThisIntrsctn = 
				skipDummyIntrsctns && m_points[itr].GetIntrsctn().IsTwoRoad() &&
                !m_points[itr].GetIntrsctn().IsControlled();;
		if( !skipThisIntrsctn )
		{
			intrsctns.push_back( m_points[itr].GetIntrsctn().GetId() );
		}

		int crdrId = m_points[itr].GetFirstLaneCrdrIdx();
		distCovered += m_points[itr].GetLength( crdrId );

		//
		// On an intersection, move to the road.
		//
		itr++;
		invalidPathPoint = itr == m_points.end(); 
		if( invalidPathPoint )  return;

		distCovered += m_points[itr].GetLength();
	}

}  // GetIntrsctnsWithinDist


vector<CPoint2D>
CPath::GetOutline(double extra) const
{
	vector<CPoint2D> points;
	for (cTPathIterator i = m_points.begin(); i != m_points.end(); i++) {
		m_points[i].GetOutline( points, extra );
//		int s = newPoints.size();
//		points.insert( points.end(), &newPoints[s/2], newPoints.end() );
//		points.insert( points.begin(), newPoints.begin(), &newPoints[s/2-1] );
	}

	return points;
} // end of GetOutline

//////////////////////////////////////////////////////////////////////////////
//
// Description: CPath::SetLaneMask
//				
//
// Remarks:		
//
// Arguments: 
//   mask - the lane mask to use
//
// Returns:		bool
//
//////////////////////////////////////////////////////////////////////////////
bool
CPath::SetLaneMask( const bitset<cCV_MAX_CRDRS>& cMask )
{
	for ( int i = m_points.begin(); i != m_points.end(); i++) {
		m_points[i].SetLaneMask( cMask );
	}
	return true;
}

bool
CPath::SetLaneMask( const CRoadPos& cRoadPos, const bitset<cCV_MAX_CRDRS>& cMask )
{
	int startIdx = m_pathPointIdx;
	cTPathIterator startItr = FindPathPoint( cRoadPos, startIdx );
	if( startItr == m_points.end() )  return false;
    
    auto point = GetPoint(startItr);
    if (!point.IsValid())
        return false;
    point.SetLaneMask( cMask );
    return true;
}

const CPathPoint&
CPath::GetPoint( cTPathIterator itr ) const
{
	return m_points[itr];
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Given a road, this function returns the id of
//   of the lane that is on the same road along the path.
//				
// Remarks: The function starts looking at the path from the specified
//   road position which can be NULL.  In that case, it starts looking
//   from the start of the path.
//
// Arguments: 
//   cpStartRoadPos - Start looking at the path from this position.
//   cRoad - The road to find on the path.
//
// Returns:  An integer that represents the lane or corridor's id. It
//   returns -1 if it isn't able to find the given road on the path.
//
//////////////////////////////////////////////////////////////////////////////
int
CPath::GetLaneIdFromRoad( 
			const CRoadPos* cpStartRoadPos, 
			const CRoad& cRoad
			) const
{
	//
	// Get an iterator to the path point corresponding to the given
	// cpStartRoadPos.  If that argument is NULL then simply set the
	// iterator to the start of the path.
	//
	cTPathIterator startItr;
	if( cpStartRoadPos )
	{
		int startIdx = m_pathPointIdx;
		startItr = FindPathPoint( *cpStartRoadPos, startIdx );

		if( startItr == m_points.end() )  return -1;
	}
	else
	{
		startItr = m_points.begin();
	}

	//
	// Find the pathPoint corresponding to the given cRoad.
	//
	bool foundRoad;
	for( ; startItr != m_points.end(); startItr++ )
	{
		foundRoad = (
			m_points[startItr].IsRoad() && 
			m_points[startItr].GetRoad() == cRoad
			);
		if( foundRoad )
		{
            //if cpStartPos, is on a crdr that leads to the road, grab the lane that connects
            //from our the crdr on cpStartPos, to the target road.
            if (cpStartRoadPos && !cpStartRoadPos->IsRoad() && startItr > m_points.begin() && 
                m_points[startItr-1].GetIntrsctn().GetId() == cpStartRoadPos->GetIntrsctn().GetId()){
                auto crdBitSet = m_points[startItr-1].GetLaneCrdrMask();
                vector< pair<int,double> > crds;
                int crdrID = -1;
                cpStartRoadPos->GetCorridors(crds);
                int idx = cpStartRoadPos->GetIntrsctn().GetCrdrIdx();
                for (auto itr = crds.begin(); itr != crds.end(); itr++){
                    if (crdBitSet.test(itr->first-idx)){
                        crdrID = itr->first;
                        break;
                    }
                }
                if (crdrID > 0){
                    CVED::CCrdr crdr(GetCved(),crdrID);
                    int laneId = crdr.GetDstntnLn().GetId();
                    auto bitsRoad =   m_points[startItr].GetLaneCrdrMask();
                    if (bitsRoad.test(laneId)){
                        return laneId;
                    }
                }

            }else{
                //test if our starting point is the road we are on
                if (cpStartRoadPos && 
                    cpStartRoadPos->IsRoad() && 
                    m_points[startItr].IsRoad() &&
                    cpStartRoadPos->GetRoad().GetId() == m_points[startItr].GetRoad().GetId()){
                    //if our starting point is on the path, return the starting point's lane ID
                    int laneID = cpStartRoadPos->GetLane().GetRelativeId();
                    if (m_points[startItr].GetLaneCrdrMask().test(laneID)){
                        return laneID;
                    }
                }
                return m_points[startItr].GetLane().GetRelativeId();
            }
		}
	}

	return -1;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: CalculateRoute
//  Creates a tree of all possible directions the driver can take and 
//	calculates the first or shortest route between two points
//	Uses a breadth-first search and does not use greedy algorithms
//
// Remarks: both the start and the destination must be on a road. This method
//	differs from the method in roadpos.cxx in that it stores points in m_points
//	as path points rather than in a vector of TRouteInfo.
//
// Remarks on optimizations: The first optimization implemented was to
//	keep track of how many times a route traveled away from the destination
//	point in a row. If the route travels away from the destination point
//	too many times, then the route finder will stop expanding upon that
//	route. In order to implement this, each node contains a counter that
//	increments every time a route travels away from the destination point
//	and resets every time a route travels towards the destination point. 
//	Whether or not the route travels towards or away from the destination 
//	point is decided by computing the linear distance to the destination
//	and comparing it with the previous linear distance to the destination.
//	This optimization has a significant effect on certain routes where there
//	are few paths that lead to the destination. In some cases, the route
//	calculation time dropped from several seconds to almost instantaneous.
//
//	The second optimization implemented deals with routes that contain
//	many nodes. As the number of nodes continues to grow, the calculation
//	time increases exponentially. The optimization periodically checks
//	the route tree when it becomes large and eliminates all nodes except
//	the closest several nodes. This implementation uses the functions
//	GetAllNodesAtLevel() and DiscardFarNodes(). It has no noticeable
//	effect on routes with a small number of nodes, but dramatically
//	decreases the calculation time of routes with 18+ nodes.
//
// Arguments:
//	start - initial point
//	end	- destination point
//	totalDist - total distance of the route in feet
//	clear - (default = false) flag denoting whether or not to clear the current path
//	shortest - (default = false), flag to find the shortest route
//		note: finding shortest route takes significantly longer for complex routes
//	maxHeight - (default = 25), maximum height of the tree before giving up
//
// Returns: true if route was found, otherwise false
//
//////////////////////////////////////////////////////////////////////////////

bool
CPath::CalculateRoute(CRoadPos& start, CRoadPos& end, double& totalDist, bool clear, bool shortest, int maxHeight) {
	
	AssertValid();

	if (!start.IsValid() || !end.IsValid()) {
		return false;
	}

	CRoad road = start.GetRoad();
	CIntrsctn intr = start.GetLane().GetNextIntrsctn();
	CRoad dest = end.GetRoad();

	if (!road.IsValid() || !intr.IsValid() || !dest.IsValid()) {
		return false;
	}

	// used to store multiple valid routes if searching for shortest route
	vector< vector<CPathPoint> > routes;
	vector<double> routeDistances;
	
	// set up head node
	TRouteNode head;
	head.head = true;
	head.found = true;	// head always included in route
	head.searched = false;
	head.srcRoadIdx = -1; // head has no source
	head.srcCrdrId = -1; 
	head.srcLaneId = -1;
	head.roadIdx = road.GetId();
	head.roadDir = start.GetLane().GetDirection();
	head.length = 0; // length is based on source lane and corridor

	CPoint3D initLoc = start.GetVeryBestXYZ();
	CPoint3D destLoc = end.GetVeryBestXYZ();
	head.linearDistSquared = (initLoc.m_x - destLoc.m_x) * (initLoc.m_x - destLoc.m_x) +
							 (initLoc.m_y - destLoc.m_y) * (initLoc.m_y - destLoc.m_y) +
							 (initLoc.m_z - destLoc.m_z) * (initLoc.m_z - destLoc.m_z) ;
	head.awayNodes = 0;

	// trivial case where initial and destination are on the same road
	// and the destination is ahead of the initial position
	if (road.GetId() == dest.GetId() && start.GetLane().GetDirection() == end.GetLane().GetDirection() && start.GetDistanceOnLane() < end.GetDistanceOnLane()) {
		
		if (clear) {
			m_points.clear();
			m_points.push_back(CPathPoint(start));
		}
		
		m_points.push_back(CPathPoint(end));
		return true;
	}
	
	// the maxAway counter is the max number of times a route can move away from the destination point in a row
	for(int maxAway=1; maxAway<=8; maxAway*=2) {
		// resets the tree if a route was not found
		head.children.clear();
		head.searched = false;

		totalDist = 0; // keeps track of total route distance
		int height = 1;	// height of the route tree
		bool done = false; // keeps track of whether a route or set of routes has been found

		while(height <= maxHeight){
			// check if a valid route was found
			if (GetRouteChildren(road.GetId(), start.GetLane().GetDirection(), end, height, &head , maxAway)){

				TRouteNode* ptr = &head;
				vector<CPathPoint> temp;

				// the first node's length is calculated differently from all other nodes
				bool first = true;

				if (clear) {
					m_points.clear();
					if (!shortest) {
						m_points.push_back(CPathPoint(start));
					} else {
						ptr->found = false;
						temp.push_back(CPathPoint(start));
					}
				}
					
				// store route data
				while(true) {
					// get the first node in the route (after the head)
					bool found = false;
					for(int i=0; i<ptr->children.size(); i++) {
						if (ptr->children[i].found) {
							ptr = &ptr->children[i];
							found = true;
							break;
						}
					}

					if (found) {
						// add segment length to the total distance
						totalDist += first ? ptr->length - start.GetDistanceOnLane() : ptr->length;
						

						CRoad road(GetCved(), ptr->srcRoadIdx);
						// specified distance of 0.1 ft to add some padding
						CRoadPos pos(road, ptr->srcCrdrId, 0.1, 0);

						// if appending, do not add the first node
						if (clear || !first) {
							if (!shortest) {
								m_points.push_back(CPathPoint(pos));
							} else {
								ptr->found = false;
								temp.push_back(CPathPoint(pos));
							}
						}

						first = false;

						// need to change a few things for the last node
						if (ptr->children.size() == 0) {
							totalDist += end.GetDistanceOnLane();
						
							if (!shortest) {
								m_points.push_back(CPathPoint(end));
							} else {
								temp.push_back(CPathPoint(end));
								routes.push_back(temp);
								routeDistances.push_back(totalDist);

								// reset distance tracker
								totalDist = 0;

								// since a route has been found, we don't have to search much more
								// this is not strictly necessary, but it will speed up things by a lot
								// this is used only if calculating the shortest route
								if (maxHeight > height + 2 && height < 10) {
									maxHeight = height + 2;
								} else if (maxHeight > height + 1 && height < 15) {
									maxHeight = height + 1;
								} else {
									maxHeight = height;
								}
							}
							break;
						}
					} else {
						// should not happen--occurs when GetRouteChildren() returns true & either the route was constructed improperly or there is no route
						// add a breakpoint to debug GetRouteChildren()
						return false;
					}

					first = false;
				}

				// if not searching for the shortest route, return the output
				if (!shortest) {
					return true;
				} else {
					// otherwise we need to find if there are any other routes with the same number of segments
					// however, we do not need to remake the tree with a higher max away node limit
					done = true; 
					continue; 
				}

			}

			// no route found; increment the height of the tree
			height++;

			// periodically scan the tree for nodes that are too far away from the destination and discard these nodes
			if (height % 8 == 0) {
				std::vector<TRouteNode*> nodes;
				GetAllNodesAtLevel(&head, height, nodes); 
				DiscardFarNodes(nodes, 8);
			}
		}

		// if finding the shortest route and a route has been found, do not reset the route tree
		if (done) {
			break;
		}
	}


	// if looking for shortest route, iterate through all routes and find the shortest one
	if (shortest && routes.size() > 0) {
		int shortestRoute = 0;
		double min = routeDistances[0];
		for(int i=1; i<routeDistances.size(); i++){
			if (routeDistances[i] < min) {
				shortestRoute = i;
				min = routeDistances[i];
			}
		}

		totalDist = min;
		for(int i=0; i<routes[shortestRoute].size(); i++){
			m_points.push_back(routes[shortestRoute][i]);
		}
		return true;
	}

	// if the function reaches this point then no routes were found and the optimizations probably need some fine-tuning
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: AppendRoute
//  Finds a route from the last point in the current path to the specified 
//	position and combines both routes.
//
// Arguments:
//	end	- destination point
//	totalDist - total distance of the route in feet
//	shortest - (default = false), flag to find the shortest route
//		note: finding shortest route takes significantly longer for complex routes
//	maxHeight - (default = 25), maximum height of the tree before giving up
//
// Returns: true if route was found, otherwise false. Also eturns false if path 
//	has no points.
//
//////////////////////////////////////////////////////////////////////////////
bool CPath::AppendRoute(CRoadPos end, double& totalDist, bool shortest, int maxHeight) {
	if (m_points.size() == 0) {
		return false;
	}

	return CalculateRoute(m_points[m_points.size()-1].GetRoadPos(), end, totalDist, false, shortest, maxHeight);
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRouteChildren
//  Searches the provided node and populates its children vector. If a child
//	leads to the destination road, the function will return true.
//
// Arguments: 
//	roadIdx - initial road index
//	dir	- initial road direction
//	end - destination position
//	height - level of tree to search on
//	parent - pointer to node to search
//
// Returns: true if valid route found, otherwise false
//
//////////////////////////////////////////////////////////////////////////////
bool CPath::GetRouteChildren(int roadIdx, cvELnDir dir, CRoadPos& end, int height, TRouteNode* parent, int maxAway) {
	// if the node has been searched, descend farther down the tree
	if (parent->searched) {
		for(int i=0; i<parent->children.size(); i++){
			if (GetRouteChildren(parent->children[i].roadIdx, parent->children[i].roadDir, end, height-1, &parent->children[i], maxAway)) {
				// found a valid route
				return true;
			}
		}
	} else if (height == 1){ // if the node is at the correct height and has not been searched
		// prevents this node from being searched again
		parent->searched = true;

		TCrdrVec crdrVec;
		CRoad road(GetCved(), roadIdx);
		CIntrsctn intr;

		// check which direction we are traveling and get the correct intersection
		if (dir == eNEG) {
			intr = road.GetSourceIntrsctn();
		} else {
			intr = road.GetDestIntrsctn();
		}

		// get all corridors
		intr.GetCrdrsStartingFrom(road, crdrVec);

		for(int i=0; i<crdrVec.size(); i++) {
			if (!crdrVec[i].GetSrcLn().IsDrivingLane()) {
				// cars cannot drive on this lane
				continue;
			}

			// store route info
			TRouteNode node;
			node.head = false;
			node.parent = parent;
			node.found = false;
			node.searched = false;
			node.srcRoadIdx = road.GetId();
			node.srcCrdrId = crdrVec[i].GetRelativeId();
			node.srcLaneId = crdrVec[i].GetSrcLn().GetRelativeId();
			node.roadIdx = crdrVec[i].GetDstntnRdIdx();
			node.roadDir = crdrVec[i].GetDstntnLn().GetDirection();
			node.length = crdrVec[i].GetSrcLn().GetRoad().GetLinearLength() + crdrVec[i].GetLength();

			// calculate the linear distance from the current node to the destination point
			TCntrlPnt* cntrlPnt;
			if (maxAway != -1) {
				// determine which side of the road to use (use either the beginning or the end)
				if (dir == eNEG ) {
					cntrlPnt = BindCntrlPnt(road.GetCntrlPntIdx());
				} else {
					cntrlPnt = BindCntrlPnt(road.GetCntrlPntIdx() + road.GetCntrlPntCount());
				}

				// since distance is used only for comparison, no need to square root
				node.linearDistSquared = (cntrlPnt->location.x - end.GetVeryBestXYZ().m_x) * (cntrlPnt->location.x - end.GetVeryBestXYZ().m_x) +
										 (cntrlPnt->location.y - end.GetVeryBestXYZ().m_y) * (cntrlPnt->location.y - end.GetVeryBestXYZ().m_y) +
										 (cntrlPnt->location.z - end.GetVeryBestXYZ().m_z) * (cntrlPnt->location.z - end.GetVeryBestXYZ().m_z) ;

				if (node.linearDistSquared > parent->linearDistSquared) { 
					// increment awayNodes counter
					node.awayNodes = parent->awayNodes + 1;

					if (node.awayNodes > maxAway) {
						node.searched = true; // prevent node from being searched again
					}
				} else {
					node.awayNodes = 0; // reset awayNodes counter
				}
			} 
			
			// found a valid path
			if (node.roadIdx == end.GetRoad().GetId() && node.roadDir == end.GetLane().GetDirection()) {
				node.found = true;
				node.searched = true;	// prevents this node from being searched again if searching for shortest route
				parent->children.push_back(node);

				// mark each parent node as part of the route
				TRouteNode* ptr = parent;
				while(!ptr->head) {
					ptr->found = true;
					ptr = ptr->parent;
				}

				return true;
			}

			// did not find valid path; add to parent's children
			parent->children.push_back(node);
		}
	}

	// unable to find a route from this node
	return false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetAllNodesAtLevel
//  Recursively descends the route tree until it reaches the specified level.
//	Once at the specified level the function populates the output vector
//	with the route nodes.
//
// Arguments: 
//	parent - current node
//	level - level of the tree to search for
//	out - (output) vector containing a pointer to all nodes at the specified
//			level
//
//////////////////////////////////////////////////////////////////////////////
void CPath::GetAllNodesAtLevel(TRouteNode* parent, int level, std::vector<TRouteNode*>& out) {
	if (level > 0) {
		// descend
		for(int i=0; i<parent->children.size(); i++){ 
			GetAllNodesAtLevel(&parent->children[i], level-1, out);
		}
	} else {
		// at the correct level
		out.push_back(parent);
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: DiscardFarNodes
//  Searches a vector of nodes and discards all but a user-specified number of
//	nodes. Nodes are discarded based on their linear distance to the
//	destination point.
//
// Arguments: 
//	nodes - all nodes to search
//	numToKeep - number of nodes to keep
//
//////////////////////////////////////////////////////////////////////////////
void CPath::DiscardFarNodes(std::vector<TRouteNode*>& nodes, int numToKeep) {
	// invalid inputs
	if (numToKeep > nodes.size() || numToKeep < 1) {
		return;
	}

	// indices of nodes to keep
	std::vector<int> keep;
	
	// set default indices
	for(int i=0; i<numToKeep; i++) {
		keep.push_back(i) ;
	}

	// keep track of which index has the maximum distance
	int max = 0;

	// find the largest distance amongst the initial nodes
	for(int i=1; i<keep.size()-1; i++) {
		if (nodes[i]->linearDistSquared > nodes[max]->linearDistSquared) {
			max = i;
		}
	}

	// search all nodes for smaller distances
	for(int i=numToKeep; i<nodes.size(); i++) {
		if (nodes[i]->linearDistSquared < nodes[ keep[max] ]->linearDistSquared) {
			//insert node index here
			keep[max] = i;

			// find new max
			for(int j=0; j<keep.size(); j++) {
				if ( nodes[ keep[j] ]->linearDistSquared > nodes[ keep[max] ]->linearDistSquared ) {
					max = j;
				}
			}
		}
	}

	// stop searching in nodes that are too far away
	for(int i=0; i<nodes.size(); i++) {
		nodes[i]->searched = true;
	}
	for(int i=0; i<numToKeep; i++) {
		nodes[ keep[i] ]->searched = false;
	}
}

} // namespace CVED
