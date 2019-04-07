//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: pathpoint.cxx,v 1.68 2018/09/13 19:31:00 IOWA\dheitbri Exp $
//
// Author(s):	Jillian Vogel
// Date:		October, 1999
//
// Description:	The implementation of the CPathPoint class
//
//////////////////////////////////////////////////////////////////////////////

#include "cvedpub.h"
#include "cvedstrc.h"		// private CVED data structs
#include <randnumgen.h>

#define PATH_POINT_DEBUG 0
const double cPATH_POINT_ZERO = 0.01; // Allow for doubleing point discrepancies

namespace CVED {

#define	cTWO_TIMES_PI 6.28319530717958647692
	
//////////////////////////////////////////////////////////////////////////////
//	Operator functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: Prints the contents of the cPathPoint to the ostream 
//   parameter.
//
// Remarks: The ostream&<< has to appear within the namespace, otherwise
//   it won't be properly assiciated.
//
// Arguments:
//   out - ostream to print to.
//   cPathPoint - CPathPoint to print.
//
// Returns: A reference to the ostream parameter so that the << operations
//   can be nested.
//////////////////////////////////////////////////////////////////////////////
ostream&
operator<<( ostream& out, const CPathPoint& cPP ) 
{
	out << cPP.GetString();
	return out;
}  // end of operator<<

} // end namespace CVED

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: The assignment operator assigns the contents of the 
//   parameter to the current instance.
//
// Remarks: 
//
// Arguments: 
//	cRhs - A CPathPoint to assign to the current instance.
//
// Returns: A reference to the current instance to allow for nested 
//   assignment	statements.
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint&
CPathPoint::operator=( const CPathPoint& cRhs )
{
	if( this != &cRhs )
	{
		// Copy superclass data
		this->CCvedItem::operator=( cRhs );

		m_isRoad = cRhs.m_isRoad;
		m_road = cRhs.m_road;
		m_intrsctn = cRhs.m_intrsctn;
		m_laneCrdrMask = cRhs.m_laneCrdrMask;

		for( int i = 0; i < cCV_MAX_CRDRS; i++ )
		{
			m_startDist[i] = cRhs.m_startDist[i];
			m_endDist[i] = cRhs.m_endDist[i];
		}
	} 
	return *this;	
}  // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: The comparison operator returns true if the parameter 
//   is the same as the	current instance.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments: 
//	 cRhs - A CPathPoint to compare the current instance to.
//
// Returns: True if the parameter is the same as the current instance, 
//   false otherwise.
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::operator==( const CPathPoint& cRhs ) const
{
	AssertValid();
	cRhs.AssertValid();

	int laneCrdrId;
	bitset<cCV_MAX_CRDRS> overlap = m_laneCrdrMask;

	int roadIsecIdC;
	if( m_isRoad )
	{
		roadIsecIdC = m_road.GetId();
	}
	else
	{
		roadIsecIdC = m_intrsctn.GetId();
	}

	int roadIsecIdP;
	if( cRhs.m_isRoad )
	{
		roadIsecIdP = cRhs.m_road.GetId();
	}
	else
	{
		roadIsecIdP = cRhs.m_intrsctn.GetId();
	}

	// If they're both on a road or both on an intersection
	bool bothOnRoad = m_isRoad  == cRhs.m_isRoad;
	if( bothOnRoad )
	{

		// If the roads/intersections match
		bool match = roadIsecIdC == roadIsecIdP;
		if( match )
		{

			// If any lanes/corridors overlap
			overlap &= cRhs.m_laneCrdrMask;
			if( overlap.any() )
			{
				// For each overlapping lane
				for(laneCrdrId = 0; laneCrdrId < cCV_MAX_CRDRS; laneCrdrId++)
				{
					if( overlap.test( laneCrdrId ) )
					{
						if( ( fabs(m_startDist[laneCrdrId] - 
							  cRhs.m_startDist[laneCrdrId]) > cPATH_POINT_ZERO
							  ) ||
							( fabs(m_endDist[laneCrdrId] - 
							  cRhs.m_endDist[laneCrdrId]) > cPATH_POINT_ZERO
							  )
							)
						{
							return false;
						}
					}

				} // For each overlapping lane

				// If control makes it here, then they are the same
				return true;

			} // If any lanes overlap

		} // If the roads match

	} // If they're both on a road

	// If control makes it here, they're not the same.
	return false;

}  // end of operator==


//////////////////////////////////////////////////////////////////////////////
//		Constructor
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: The default constructor initializes the local variables.
//
// Remarks: This constructor creates an invalid CPointPath instance.
//
// Arguments:
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint::CPathPoint()
	: 
	CCvedItem()
{
	int n;
	for( n = 0; n < cCV_MAX_CRDRS; n++ )
	{
		m_startDist[n] = 0.0;
		m_endDist[n] = 0.0;
	}
}  // end of CPathPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: The default destructor does nothing.
//
// Remarks:
//
// Arguments:
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint::~CPathPoint() 
{}  // end of ~CPathPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: The copy destructor calls the assignment operator.
//
// Remarks:
//
// Arguments:
//	 cCopy - CPathPoint instance to copy to the current instance.
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint::CPathPoint( const CPathPoint& cCopy )
{
	*this = cCopy;
}  // end of CPathPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: The CCved destructor initializes the CCvedItem superclass.
//
// Remarks: This constructor creates an invalid CPointPath instance.
//
// Arguments:
//	 cCved - A valid CCved instance.
// 	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint::CPathPoint( const CCved& cCved )
	: 
	CCvedItem( &cCved )
{
	int n;
	for( n = 0; n < cCV_MAX_CRDRS; n++ )
	{
		m_startDist[n] = 0.0;
		m_endDist[n] = 0.0;
	}
}  // end of CPathPoint
	
//////////////////////////////////////////////////////////////////////////////
//
// Description: This constructor parses the string parameter and 
//   initializes its local variables.
//
// Remarks: The format of the string is as follows:
//   If the node lies on a road:
// 	   R:Road_Name:Lane_Id[Start_dist:End_dist]:Lane_Id[Start_dist:...
//   Else if the node lies on an intersection
//     I:Intrsctn_Name:Crdr_Id[Start_dist:End_dist]:Crdr_Id[Start_dist:...
//
// Arguments:
//	 cCved - A valid CCved instance.
// 	 cStr - String with the above format.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint::CPathPoint( const CCved& cCved, const string& cStr )
	: 
	CCvedItem( &cCved )
{
	int n;
	for( n = 0; n < cCV_MAX_CRDRS; n++ )
	{
		m_startDist[n] = 0.0;
		m_endDist[n] = 0.0;
	}
	SetString( cStr );
}  // end of CPathPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This constructor takes the information in the CRoadPos 
//   parameter and initializes its local variables.
//
// Remarks: If the given CRoadPos lies on a road, then the local data is 
//   initialized to all the lanes that run in the same direction as the lane
//   stored in the CRoadPos.  If the given CRoadPos lies on an intersection, 
//   then the local data is initialized to all corridors running from the 
//   source road to the destination road of the first corridor that the 
//   CRoadPos lies on.  Distance info is also taken from CRoadPos.
//
// Arguments:
//   cRoadPos - A Valid CRoadPos instance.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
CPathPoint::CPathPoint( const CRoadPos& cRoadPos )
	: 
	CCvedItem( cRoadPos )
{
	int n;
	for( n = 0; n < cCV_MAX_CRDRS; n++ )
	{
		m_startDist[n] = 0.0;
		m_endDist[n] = 0.0;
	}

	Initialize( cRoadPos );
}  // end of CPathPoint

//////////////////////////////////////////////////////////////////////////////
//		Accessor functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Indicates whether the current CPathPoint lies on a road 
//   on an intersection
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//
// Returns: True if the current CPathPoint lies on a road, false otherwise.
// 
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::IsRoad( void ) const
{
	AssertValid();
	return m_isRoad;
}  // end of IsRoad

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the road element of the current CPathPoint.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//
// Returns: A valid CRoad instance if the current CPathPoint lies on a 
//   road, an invalid CRoad instance otherwise.
// 
//////////////////////////////////////////////////////////////////////////////
CRoad
CPathPoint::GetRoad( void ) const
{
	AssertValid();
	return m_road;
}  // end of GetRoad

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the intersection element of the current CPathPoint.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//
// Returns: A valid CIntrsctn isntance if the current CPathPoint lies on an
// 	 intersection, an invalid CIntrsctn instance otherwise.
// 
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CPathPoint::GetIntrsctn( void ) const
{
	AssertValid();
	return m_intrsctn;
}  // end of GetIntrsctn

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the lane/crdr mask element of the current 
//   CPathPoint.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//
// Returns: A bitset whose bits represent the lanes the current CPathPoint 
// 	 lies on if it is on a road, or the corridors it lies on if it is on an
// 	 intersection.
// 
//////////////////////////////////////////////////////////////////////////////
bitset<cCV_MAX_CRDRS>
CPathPoint::GetLaneCrdrMask( void ) const
{
	AssertValid();
	return m_laneCrdrMask;
}  // end of GetLaneCrdrMask

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the first set lane/crdr index in the mask.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//
// Returns: The index of the first lane/crdr set in the mask.  
// 
//////////////////////////////////////////////////////////////////////////////
int
CPathPoint::GetFirstLaneCrdrIdx( void ) const
{
	
	AssertValid();

	for( int id = 0; id < cCV_MAX_CRDRS; id++ )
	{
		if( m_laneCrdrMask.test( id ) )  return id;
	}
	// If we get here, then there's an error.  A valid CPathPoint
	// 	must contain 1+ lanes or corridors.  So return -1.
	return -1;
}  // end of GetFirstLaneCrdrIdx
//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the first set source lane/crdr index in the mask.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//
// Returns: The index of the first crdr set in the mask
///			-1 if point not found  
// 
//////////////////////////////////////////////////////////////////////////////
int
CPathPoint::GetFirstCrdrWithLaneId(int id) const{
	
	AssertValid();
	if (!m_intrsctn.IsValid()){
		return -1;
	}

	for( int i = 0; i < cCV_MAX_CRDRS; i++ )
	{
		if( m_laneCrdrMask.test( i ) ){
			const CCrdr crd(m_intrsctn,i);
			if (crd.GetSrcLn().GetId() == id)
				return i;
		}
	}

	// If we get here, then there's an error.  A valid CPathPoint
	// 	must contain 1+ lanes or corridors.  So return -1.
	return -1;
}  // end of GetFirstLaneCrdrIdx     
int CPathPoint::GetFirstCrdrWithLaneIndex(int id) const{
	AssertValid();
	if (!m_intrsctn.IsValid()){
		return -1;
	}

	for( int i = 0; i < cCV_MAX_CRDRS; i++ )
	{
		if( m_laneCrdrMask.test( i ) ){
			const CCrdr crd(m_intrsctn,i);
			if (crd.GetSrcLn().GetIndex() == id)
				return i;
		}
	}

	// If we get here, then there's an error.  A valid CPathPoint
	// 	must contain 1+ lanes or corridors.  So return -1.
	return -1;
}
//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the starting distance of the indicated lane or 
//   corridor on the current CPathPoint.
//
// Remarks: This function will cause a failed assertion if it is called on an
//   invalid CPathPoint instance.
//
// Arguments:
//   id - (optional) Lane or corridor id whose starting distance is requested.
//        Default value is -1, which indicates that the first lane/crdr should
//        be used.
//		
// Returns: A double value representing the starting distance of the lane or
//   corridor.
// 
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::GetStartDist( int id ) const
{
	AssertValid();

	if( id < 0 )  id = GetFirstLaneCrdrIdx();

	return m_startDist[id];
} // end of GetStartDist

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the ending distance of the indicated lane or 
//   corridor on the current CPathPoint.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//   id - (optional) Lane or corridor id whose ending distance is requested.
//        Default value is -1, which indicates that the first lane/crdr should
//        be used.
//		
// Returns: A double value representing the ending distance of the lane or
//   corridor.
// 
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::GetEndDist( int id ) const
{
	AssertValid();

	if( id < 0 )  id = GetFirstLaneCrdrIdx();

	return m_endDist[id];
} // end of GetEndDist

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the starting distance all lanes/crdrs.
//
// Remarks: This function will cause a failed assertion if it is called on an
//   invalid CPathPoint instance.
//
// Arguments:
//		
// Returns: Doubles representing the starting distance of the lanes/crdrs.
// 
//////////////////////////////////////////////////////////////////////////////
const double*
CPathPoint::GetStartDists() const
{
	AssertValid();
	return m_startDist;
} // end of GetStartDists

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns the ending distance of the all lanes/crdrs.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
//		
// Returns: Doubles representing the ending distance of the lane/crdrs.
// 
//////////////////////////////////////////////////////////////////////////////
const double*
CPathPoint::GetEndDists() const
{
	AssertValid();
	return m_endDist;
} // end of GetEndDists

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns a CLane instance created from the indicated lane.  
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CPathPoint instance.
//
// Arguments:
// 	id - (optional) Lane id (relative to the current road) which is requested.
// 		Default value is -1, which indicates that the first lane should be
// 		returned.
//
// Returns: A CLane instance representing the indicated lane.  If the current
// 	CPathPoint instance lies on an intersection, the returned CLane will be
// 	invalid.
//
//////////////////////////////////////////////////////////////////////////////
CLane
CPathPoint::GetLane( int id ) const 
{
	AssertValid();

	if( id < 0 )  id = GetFirstLaneCrdrIdx();

	if( m_isRoad )
	{	
		CLane lane( m_road, id );
		return lane;
	}
	else 
	{
		CLane lane;
		return lane;
	}
}  // end of GetLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns a CCrdr instance created from the indicated lane.  
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments:
// 	 id - (optional) Crdr id (relative to the current intrsctn) which is 
// 	      requested.  Default value is -1, which indicates that the first 
//        crdr should be returned.
//
// Returns: A CCrdr instance representing the indicated crdr.  If the 
//   current CPathPoint instance lies on a road, the returned CCrdr will 
//   be invalid.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CPathPoint::GetCrdr( int id ) const 
{
	AssertValid();

	if( id < 0 )  id = GetFirstLaneCrdrIdx();

	if( !m_isRoad )
	{	
		CCrdr crdr( m_intrsctn, id );
		return crdr;
	}
	else 
	{
		CCrdr crdr;
		return crdr;
	}
} // end of GetCrdr

//////////////////////////////////////////////////////////////////////////////
//	
// Description: Returns a CRoadPos located at the given parametric 
//   distance along the current CPathPoint.  The distance is interpolated 
//   between the startDist and the endDist based on the t parameter.  
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance.
//
// Arguments: 
// 	 t  - (optional) Parametric value to interpolate distance with.  A 
//        value of 0 makes distance = startDist, and a value of 1 makes 
//        distance = endDist.  The default value is 0.0.
//   id - (optional) Identifier of the lane or corridor to use, relative 
//        to the current road or intersection.  The default value is -1, 
//        which indicates that the first lane should be used, or all 
//        corridors.
//		
// Returns: If the current CPathPoint lies on a road, the resulting 
//   CRoadPos lies on that road, on the specified lane, at the 
//   interpolated distance, and with an offset of 0.0.  If the current 
//   CPathPoint lies on an intersection, the resulting CRoadPos lies on 
//   that intersection, on the specified corridor, at the interpolated 
//   distance, and with an offset of 0.0.
// 
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CPathPoint::GetRoadPos( double t, int id ) const
{
	AssertValid();

	CRoadPos roadPos( GetCved() );

	if( m_isRoad )
	{
		if( id < 0 )  id = GetFirstLaneCrdrIdx();

		CLane lane( m_road, id );
		roadPos.SetLane( lane );

		double dist = ( 1.0f - t ) * m_startDist[id] + t * m_endDist[id];
		roadPos.SetDistance( dist );
		roadPos.SetOffset( 0.0f );
	}
	else 
	{
		// If id < 0, then use all corridors
		if( id < 0 )
		{
			double distances[cCV_MAX_CRDRS] = { 0 };
			for( int i = 0; i < cCV_MAX_CRDRS; i++ )
			{
				if( m_laneCrdrMask.test( i ) )
				{
					distances[i] = (
						( 1.0f - t ) *
						m_startDist[i] + 
						t *
						m_endDist[i]
						);
				}
			}
			roadPos.SetCorridors( 
						m_intrsctn, 
						m_laneCrdrMask, 
						distances, 
						0.0f
						);
		}
		// If a valid id was given, use it.
		else
		{
			CCrdr crdr( m_intrsctn, id );
			roadPos.SetCorridor( crdr );

			double dist = ( 1.0f - t ) * m_startDist[id] + t * m_endDist[id];
			roadPos.SetDistance( dist );
			roadPos.SetOffset( 0.0f );
		}
	}

	return roadPos;
}  // end of GetRoadPos

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the intersection following the current CPathPoint.
//   "Next" means the intersection in the direction of the current 
//   CPathPoint.
//
// Remarks: If the current CPathPoint lies on an intersection, then the 
// 	 current intersection is returned.
//
//   This function will cause a failed assertion if it is called
// 	 on an invalid CPathPoint instance
//
// Arguments:
//
// Returns: A CIntrsctn instance representing the next intersection in the 
//   direction of the current CPathPoint.  If there is no such intersection, 
//   the CIntrsctn is invalid.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CPathPoint::GetNextIntrsctn() const
{
	AssertValid();

	// If the current CPathPoint lies on a road
	if( m_isRoad )
	{
		// Return the next intersection in the 
		// 	direction of one current lane.
		cvELnDir dir = GetLane().GetDirection();
		if( dir == ePOS )
		{
			return m_road.GetDestIntrsctn();
		}
		else
		{
			return m_road.GetSourceIntrsctn();
		}
	}
	else 
	{
		// Return the current intersection
		return m_intrsctn;
	}
}  // end of GetNextIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the intersection preceding the current CPathPoint.
//   "Prev" means the intersection in the opposite direction of the current 
//   CPathPoint.
//
// Remarks: If the current CPathPoint lies on an intersection, then the 
//   current intersection is returned.
//
//   This function will cause a failed assertion if it is called
//   on an invalid CPathPoint instance
//
// Arguments:
//
// Returns: A CIntrsctn instance representing the previous intersection 
//   in the direction of the current CPathPoint.  If there is no such 
//   intersection, the CIntrsctn is invalid.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CPathPoint::GetPrevIntrsctn() const
{
	AssertValid();

	// If the current CPathPoint lies on a road
	if( m_isRoad )
	{
		// Return the prev intersection in the opposing direction of one 
		// current lane.
		cvELnDir dir = GetLane().GetDirection();
		if( dir == ePOS )
		{
			return m_road.GetSourceIntrsctn();
		}
		else
		{
			return m_road.GetDestIntrsctn();
		}
	}
	else 
	{
		// Return the current intersection
		return m_intrsctn;
	}
}  // end of GetPrevIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the road following the current CPathPoint.  
//   "Next" means the road in the direction of the current CPathPoint.
//
// Remarks: If the current CPathPoint lies on a road, then the current 
//    road is returned.
//
//    This function will cause a failed assertion if it is called
//    on an invalid CPathPoint instance
//
// Arguments:
//    id - If the current CPathPoint lies on an intersection, then this 
//         is the index of the corridor to use.  Default value is -1, 
//         indicating that the first valid corridor will be used.
//
// Returns: A CRoad instance representing the next road in the direction 
//    of the current CPathPoint.  If there is no such road, the CRoad is 
//    invalid. 
//
//////////////////////////////////////////////////////////////////////////////
CRoad
CPathPoint::GetNextRoad( int id ) const
{
	AssertValid();

	// If the current CPathPoint lies on an intersection
	if( !m_isRoad )
	{
		// Return the destination road of the current corridor.
		return GetCrdr( id ).GetDstntnRd();
	}
	else 
	{
		// Return the current road
		return m_road;
	}
}  // end of GetNextRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the road preceding the current CPathPoint.  
//   "Prev" means the road in the opposite direction of the current 
//    CPathPoint.
//
// Remarks: If the current CPathPoint lies on a road, then the current 
//    road is returned.
//
// 	  This function will cause a failed assertion if it is called
// 	  on an invalid CPathPoint instance
//
// Arguments:
//    id - If the current CPathPoint lies on an intersection, then this 
//         is the index of the corridor to use.  Default value is -1, 
//         indicating that the first valid corridor will be used.
//
// Returns: A CRoad instance representing the prevous road in the opposite
//    direction of the current CPathPoint.  If there is no such road, the 
//    CRoad is invalid. 
//
//////////////////////////////////////////////////////////////////////////////
CRoad
CPathPoint::GetPrevRoad( int id ) const
{
	AssertValid();

	// If the current CPathPoint lies on an intersection
	if( !m_isRoad )
	{
		// Return the source road of the current corridor.
		return GetCrdr( id ).GetSrcRd();
	}
	else 
	{
		// Return the current road
		return m_road;
	}
} // end of GetPrevRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the length of the current CPathPoint.
//	
// Remarks: The length is determined by the absolute value of the starting
//   distance minus the ending distance of the indicated lane/crdr.
//
//   This function will cause a failed assertion if it is called
//   on an invalid CPathPoint instance
//
// Arguments:
//   id - (optional) The index of the lane or corridor (with respect to the 
//        road or intersection) whose length is desired.
//        The default value is -1, indicating that the first valid lane or 
//        corridor will be used.
//
// Returns: A double value indicating the length of the current CPathPoint.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::GetLength( int id ) const
{
	AssertValid();

	if (id < 0)  id = GetFirstLaneCrdrIdx();

	return fabs( m_startDist[id] - m_endDist[id] );
}  // end of GetLength

//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets each element in the vector parameter to a string 
//   representing each node in the path.
//
// Remarks:  The format of each node is as follows:
//   If the node lies on a road:
//     R:Road_Name:Lane_Id[Start_dist:End_dist]:Lane_Id[Start_dist:...
//   Else if the node lies on an intersection
//     I:Intrsctn_Name:Crdr_Id[Start_dist:End_dist]:Crdr_Id[Start_dist:...
//
// Arguments:
//   pathStr - (Output) Upon return, will contain a string for each node 
//             in the current path.
//
// Returns: void
// 	
//////////////////////////////////////////////////////////////////////////////
string
CPathPoint::GetString( void ) const 
{
	string out;
	if( IsValid() ) 
	{
		// if cPathPoint lies on a road
		if( m_isRoad )
		{
			out = "R:";
			out += m_road.GetName();
		}
		// if cPathPoint lies on an intersection
		else
		{
			out = "I:";
			out += m_intrsctn.GetName();
		}

		// For each lane/crdr in the mask, print out the id and the dist range
		char laneCrdr[50];
		for( int i = 0; i < cCV_MAX_CRDRS; i++ )
		{
			if( m_laneCrdrMask.test( i ) )
			{
				sprintf_s( laneCrdr, ":%d[%.2f:%.2f]", 
					     i, m_startDist[i], m_endDist[i]
						 );
				out += laneCrdr;
			}
		}
	}
	else
	{
		out = "Bad Point";
	}

	return out;
}  // end of GetString


//////////////////////////////////////////////////////////////////////////////
//		Mutator functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward along the same 
//  road.  
//
// Remarks:  The path is extended, at maximum, to the end of the current
//  road or to the given distance if it is shorter than the distance to
//  the end of the road.
//
//  This function should only be called if the current CPathPoint is a
//  road.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
// 	dist - A double distance along the road network to extend.
//
// Returns:  The actual distance traveled.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::AppendOnSameRoad( double dist )
{
	// 
	// Make sure that the current path point is a road.
	//
	if( !m_isRoad )  return 0.0;

	//
	// Calculate the distance to the end of the road from the current
	// location.
	//
	double distToEnd;
	double direction;
	int firstLaneIdx = GetFirstLaneCrdrIdx();
	cvTLane* pLane = BindLane( m_road.GetLaneIdx() + firstLaneIdx );
	if (firstLaneIdx < 0){//we have failed to bind to a lane, 
		                  //bad stuff happens if we try to use -1 as an index
		return 0;
	}
	if( pLane->direction == ePOS )
	{
		distToEnd = m_road.GetLinearLength() - m_endDist[firstLaneIdx];
		direction = 1.0f;
	}
	else 
	{
		distToEnd = m_endDist[firstLaneIdx];
		direction = -1.0f;
	}
	double distTraveled = min( dist, distToEnd );

	//
	// Update the end distances of the lanes in the current 
	// road path point.
	//
	int laneId;
	for( laneId = firstLaneIdx; laneId < m_road.GetNumLanes(); laneId++ )
	{
		if( m_laneCrdrMask.test( laneId ) )
		{
			m_endDist[laneId] += direction * distTraveled;
		}
	}

	return distTraveled;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward along the same 
//  road to the end.  
//
// Remarks:  The path is extended to the end of the current road.
//
//  This function should only be called if the current CPathPoint is a
//  road.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
//
// Returns:  The actual distance traveled.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::AppendEntireRoad( )
{
	// 
	// Make sure that the current path point is a road.
	//
	if( !m_isRoad )  return 0.0;

	//
	// Calculate the distance to the end of the road from the current
	// location.
	//
	double distToEnd;
	double direction;
	int firstLaneIdx = GetFirstLaneCrdrIdx();
	cvTLane* pLane = BindLane( m_road.GetLaneIdx() + firstLaneIdx );
	if( pLane->direction == ePOS )
	{
		distToEnd = m_road.GetLinearLength() - m_endDist[firstLaneIdx];
		direction = 1.0f;
	}
	else 
	{
		distToEnd = m_endDist[firstLaneIdx];
		direction = -1.0f;
	}
//	double distTraveled = min( dist, distToEnd );

	//
	// Update the end distances of the lanes in the current 
	// road path point.
	//
	int laneId;
	for( laneId = firstLaneIdx; laneId < m_road.GetNumLanes(); laneId++ )
	{
		if( m_laneCrdrMask.test( laneId ) )
		{
			m_endDist[laneId] += direction * distToEnd;
		}
	}

	return distToEnd;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward along the same 
//  intersection.  
//
// Remarks:  The path is extended, at maximum, to the end of the current
//  corridor or to the given distance if it is shorter than the distance to
//  the end of the corridor.
//
//  This function should only be called if the current CPathPoint is an
//  intersection.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
// 	dist - A double distance along the road network to extend.
//
// Returns:  The actual distance traveled.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::AppendOnSameIntrsctn( double dist )
{
	//
	// Append any needed distance to the end of the current crdr.
	//
	int firstCrdr  = GetFirstLaneCrdrIdx();
	if (firstCrdr < 0) return 0;
	cvTCrdr* pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() + firstCrdr );
	cvTHeader* pH  = static_cast<cvTHeader*>  (GetInst());
	char* pOfs     = static_cast<char*> (GetInst()) + pH->crdrCntrlPntOfs;
	TCrdrCntrlPnt* pCrdrCntrlPnt = 
		(reinterpret_cast<TCrdrCntrlPnt*>( pOfs )) + pCrdr->cntrlPntIdx;
	pCrdrCntrlPnt += pCrdr->numCntrlPnt - 1;
	
	double distToEnd = pCrdrCntrlPnt->distance - m_endDist[firstCrdr];
	double distTraveled = min( dist, distToEnd );
	
	int crdrId;
	for( 
		crdrId = firstCrdr; 
		crdrId < m_intrsctn.GetNumCrdrs(); 
		crdrId++ 
		)
	{
		if( m_laneCrdrMask.test( crdrId ) )
		{
			m_endDist[crdrId] += distTraveled;
		}
	}

	return distTraveled;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward along the same 
//  intersection to the end of the current corridor.  
//
// Remarks:  The path is extended to the end of the current corridor.
//
//  This function should only be called if the current CPathPoint is an
//  intersection.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
//
// Returns:  The actual distance traveled.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::AppendEntireIntrsctn(  )
{
	//
	// Append any needed distance to the end of the current crdr.
	//
	int firstCrdr  = GetFirstLaneCrdrIdx();
	cvTCrdr* pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() + firstCrdr );
	cvTHeader* pH  = static_cast<cvTHeader*>  (GetInst());
	char* pOfs     = static_cast<char*> (GetInst()) + pH->crdrCntrlPntOfs;
	TCrdrCntrlPnt* pCrdrCntrlPnt = 
		(reinterpret_cast<TCrdrCntrlPnt*>( pOfs )) + pCrdr->cntrlPntIdx;
	pCrdrCntrlPnt += pCrdr->numCntrlPnt - 1;
	
	double distToEnd = pCrdrCntrlPnt->distance - m_endDist[firstCrdr];
//	double distTraveled = min( dist, distToEnd );
	
	int crdrId;
	for( 
		crdrId = firstCrdr; 
		crdrId < m_intrsctn.GetNumCrdrs(); 
		crdrId++ 
		)
	{
		if( m_laneCrdrMask.test( crdrId ) )
		{
			m_endDist[crdrId] += distToEnd;
		}
	}

	return distToEnd;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward randomly for 
//  the given distance along the road network.
//
// Remarks: This function extends the range of distances covered by the 
// 	current CPathPoint in an attempt to cover the desired distance.  If the 
// 	current CPathPoint is not enough to cover the distance, then a list of 
// 	CPathPoints is created that cover the distance, along the first path 
// 	found.  If the CPathPoint can be extended any distance at all, the 
//	function returns true.  Otherwise, it returns false.  Note that if 
//	there are multiple paths, only one is given.
// 
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
// 	dist - A double distance along the road network to extend.
//  pRng - A pointer to the random number generator.
//  rngStreamId - The rng stream id.
//	extension - output parameter, will contain any CPathPoints needed to 
//		extend the current CPathPoint by the given distance.
//  stayOnHighway - A boolean that indicates if prepend should stay on
//      roads that have been marked as being highways.
//
// Returns - True if the point can be extended, false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Append( 
			double dist, 
			CRandNumGen* pRng,
			int rngStreamId,
			CFastDeque<CPathPoint>& extension, 
			bool stayOnHighway
			)
{
	//
	//	BASE CASE
	//
	if( dist <= 0 )
	{
		return true;
	}

	//
	//	RECURSIVE CASE
	//
	else 
	{
		AssertValid();

		double distToTravel;

		// If the current CPathPoint is on a road
		if( m_isRoad )
		{
			int firstLaneIdx = GetFirstLaneCrdrIdx();
			distToTravel = AppendOnSameRoad( dist );

			//
			// Add another path point if the required distance still hasn't
			// been covered.
			//
			bool distLeftToAppend = dist > distToTravel;
			if( distLeftToAppend )
			{
				//
				// Add a new CPathPoint for the connecting intersection
				//
				CPathPoint isecPoint( GetCved() );
				CIntrsctn intrsctn = GetNextIntrsctn();

				if( stayOnHighway )
				{
					//
					// Pick the pathpoint so that it's on a highway.  Do
					// this by finding all the roads that the current road
					// leads thru the next intersection.  Pick
					// the first road that is a highway,  Add the corridor
					// that leads from the current road to that road.
					//
					TRoadVec roadVec;
					intrsctn.GetRoadAccessListThrghIntrsctn(
										m_road,
										roadVec
										);

					int numRoads = (int) roadVec.size();
					bool haveRoads = numRoads > 0;
					if( haveRoads )
					{
						TRoadVec::iterator i;
						bool foundHighway = false;
						for( i = roadVec.begin(); i != roadVec.end(); i++ )
						{
							bool isRoadHighway = i->ActiveAttr( 
													cCV_HIGHWAY_ATTR 
													);
							if( isRoadHighway )
							{
								//
								// This road is a highway.  Use this one.
								//
								foundHighway = true;
								break;
							}
						}

						if( foundHighway )
						{
							bool success = isecPoint.SetIntrsctnPoint(
															intrsctn, 
															m_road, 
															firstLaneIdx,
															*i
															);
							if( success )
							{
								isecPoint.SetRangeToStart( true, true );
								extension.push_back( isecPoint );
							}
							else
							{
								// Some error occurred, so stop recursion.
								distToTravel = dist;
							}
						}
						else
						{
							// 
							// No highway roads leading to current road thru
							// the previous intersection, so stop recursion.
							//
							distToTravel = dist;
						}
					}
					else
					{
						// 
						// No highway roads leading to current road thru
						// the previous intersection, so stop recursion.
						//
						distToTravel = dist;
					}
				}
				else
				{
					//
					// Get all the corridor that start from the current road.
					// 
					TCrdrVec crdrVec;
					intrsctn.GetCrdrsStartingFrom( m_road, crdrVec );

					int numCrdrs = (int) crdrVec.size();
					bool haveCrdrs = numCrdrs > 0;
					if( haveCrdrs )
					{
						int crdrIdx;
						if( pRng )
						{
							//
							// Have an assigned random number generator.
							//
							crdrIdx =  pRng->RandomLongRange(
													0, 
													numCrdrs, 
													rngStreamId
													);
						}
						else
						{
							//
							// No random number generator assigned.  Use 
							// Rand :(
							//
							srand( (unsigned) 1 );
							crdrIdx = static_cast<int>(rand() / ( (double) RAND_MAX ) * numCrdrs);
						}

						bool success = isecPoint.SetIntrsctnPoint(
													intrsctn, 
													m_road, 
													firstLaneIdx,
													crdrVec[crdrIdx].GetDstntnRd()
													);
						if( success ) 
						{
							isecPoint.SetRangeToStart( true, true );
							extension.push_back( isecPoint );
						}
						else
						{
							// Some error occurred, so stop recursion.
							distToTravel = dist;
						}
					}
					else
					{
						// No corridors leading from current road, 
						//	so stop recursion.
						distToTravel = dist;
					}
				}
			} // If there's distance left to append
		} // If the current CPathPoint is on a road
	
		// Else, the current CPathPoint is on an intersection
		else 
		{
			//
			// Append any needed distance to the end of the current crdr.
			//
			distToTravel = AppendOnSameIntrsctn( dist );

			//
			// If there's distance left to append then go the destination
			// road.
			//
			bool distLeftToAppend = dist > distToTravel;
			if( distLeftToAppend )
			{

				SetRangeToEnd( false, true );

				// Add a new CPathPoint for the connecting road
				CPathPoint roadPoint( GetCved() );
				int firstCrdr = GetFirstLaneCrdrIdx();
				cvTCrdr* pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() + firstCrdr );
				CCrdr crdr( GetCved(), pCrdr );
				if( roadPoint.SetRoadPoint( crdr ) )
				{
					roadPoint.SetRangeToStart( true, true );

					if( stayOnHighway )
					{
						//
						// Check to make sure that this road is a highway.
						//
						bool isRoadHighway = (
							roadPoint.GetRoad().ActiveAttr( cCV_HIGHWAY_ATTR )
							);
						if( isRoadHighway )
						{
							extension.push_back( roadPoint );
						}
						else
						{
							// This road is not a highway, so stop recursion.
							distToTravel = dist;
						}
					}
					else
					{
						extension.push_back( roadPoint );
					}
				}
				else 
				{
					// Some error occurred, so stop recursion.
					distToTravel = dist;
				}

			} // If there's distance left to append
		} // Else, the current CPathPoint is on an intersection
	
		// Recur 
		return( 
			extension.back().Append( 
								dist - distToTravel, 
								pRng,
								rngStreamId,
								extension,
								stayOnHighway
								) 
			);
	}

} // end of Append

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward along the turn 
//  defined by the given angle and epsilon.
//
// Remarks: This function extends the range of distances covered by the 
// 	current CPathPoint in an attempt to make the desired turn at the next 
// 	possible intersection.  If the current CPathPoint is not enough to cover 
// 	the turn, then a list of CPathPoints is created that make the turn,
// 	and it is returned in the output parameter.  If the turn can be made and 
// 	an appropriate path found, then the function returns true.  If there is
// 	no next intersection, or if no turn in the given direction can be found, 
// 	then the function returns false.
//
// 	If the current CPathPoint lies on an intersection, then the destination
// 	intersection of the connecting lanes is used.  If the current CPathPoint
// 	list on a road, then the destination intersection of the current lanes
// 	is used.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Argument:
// 	turnAngle - Defines the clockwise angle (in radians) made between the 
// 		direction of the current lane(s) and the destination lane(s).
// 		Note some useful predefined constants can be used for this parameter:
// 		cCV_STRAIGHT_TURN - go straight through the intersection
// 		cCV_RIGHT_TURN - turn right
// 		cCV_U_TURN - make a U-turn
// 		cCV_LEFT_TURN - turn left 
// 	epsilon - The +/- tolerance of the TurnAngle when finding the desired
// 		destination lane(s).  Note that there is a predefined constant for 
// 		this parameter: cCV_EPSILON = 1/8 * pi
//	extension - output parameter, will contain any CPathPoints needed to 
//		extend the current CPathPoint through the given turn.
//  dist - extend the path only to the next intersection or the distance 
//      specified; whichever is smaller.
//  laneChangeAllowed - (optional, default true) If specified, determines
//      if path is only built from the current roadlane or if it's okay to 
//      to pick a corridor on the next intersection which requires a lane
//      change from teh current lane.
//  cLaneCrdrMask - if laneChangeAllowed==false then this can be used to 
//      override the current mask which will have all lanes set to true on 
//      the current road.  The override mask should only have the one lane 
//      that we are currently on.
//
// Returns - The distance appended
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::Append(
			double turnAngle, 
			double epsilon,
			CFastDeque<CPathPoint>& extension,
			double dist,
			bool laneChangeAllowed
			)
{
	AssertValid();


	double distSoFar;

	// If the current CPathPoint lies on an intersection
	if( m_isRoad )
	{
		//
		// Append the speicified dist along the current road and
		// check to see if the road is long enough to satisfy
		// the dist requirement and quit.
		//
		distSoFar = AppendOnSameRoad( dist );
		if( dist - distSoFar > 0.1 )
		{
			//
			// Add the next intersection according to the specified
			// turn angle.
			// 
			CIntrsctn intrsctn = GetNextIntrsctn();

			vector<CRoad> roadVec;
			if( laneChangeAllowed )
			{
				intrsctn.GetRoadAccessListThrghIntrsctn( m_road, roadVec );
			}
			else
			{
				intrsctn.GetRoadAccessListThrghIntrsctn( m_road, m_laneCrdrMask, roadVec );
			}

			double bestDiffAngle = cTWO_TIMES_PI;
			double dstAngle, diffAngle;
			vector<CRoad>::const_iterator itr;
			vector<CRoad>::const_iterator pBestRoad = roadVec.end();
			// For each road accessible from roadPoint.m_road
			for( itr = roadVec.begin(); itr != roadVec.end(); itr++ ) 
			{
				// Get the angle between the source and destination roads
				dstAngle = GetAngleBetween( intrsctn, m_road, *itr );

				// Compute the difference between the dstAngle and the 
				// turn angle
				diffAngle = fabs(dstAngle - turnAngle);
				if ( diffAngle > cTWO_TIMES_PI/2.0 )
					diffAngle = cTWO_TIMES_PI - diffAngle;

				if( diffAngle < bestDiffAngle )
				{
					pBestRoad = itr;
					bestDiffAngle = diffAngle;
				}
			} // For each road accessible from roadPoint.m_road
			
			//
			// If we're taking the best we found, or if the bestDiffAngle
			// is within epsilon of the turnAngle, then make the turn.
			//
			bool foundAppropriateCrdr = (
						(bestDiffAngle <= epsilon + cPATH_POINT_ZERO) &&
						(pBestRoad != roadVec.end())
						);
			if( foundAppropriateCrdr )
			{
				// Create a CPathPoint on the intersection
				CPathPoint isecPoint( GetCved() );
				bool success = isecPoint.SetIntrsctnPoint(
												intrsctn, 
												m_road,
												GetFirstLaneCrdrIdx(),
												*pBestRoad
												);
				if( success ) 
				{	
					//
					// Set the current range to end.
					//
					SetRangeToEnd( false, true );

					//
					// Extend isecPoint range over full corridor 
					// and add it to the list
					//
					isecPoint.SetRangeToStart( true, true );
					extension.push_back( isecPoint );

					distSoFar += extension.back().Append( 
												turnAngle, 
												epsilon, 
												extension,
												dist - distSoFar 
												);
				}
			}
		}	
	}
	else
	{
		//
		// Extend isecPoint range over full corridor and add it to the list.
		//
		distSoFar = AppendOnSameIntrsctn( dist );

		if( dist - distSoFar > 0.1 )
		{
			// Create a CPathPoint on the next road
			CPathPoint nextRoadPoint( GetCved() );
			if( nextRoadPoint.SetRoadPoint( GetCrdr() ) )
			{
				//
				// Set the corridor's range to end.
				//
				SetRangeToEnd( false, true );

				//
				// Set distance range to the beginning of the road 
				// and add it to the list.  Extend the distance as far
				// along as needed.
				//
				nextRoadPoint.SetRangeToStart( true, true );
				extension.push_back( nextRoadPoint );

				distSoFar += extension.back().Append( 
												turnAngle, 
												epsilon, 
												extension, 
												dist - distSoFar 
												);
			}
		}
	} // If the current CPathPoint lies on an intersection
	
	return distSoFar;

} // end of Append

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward to connect 
//  itself with the parameter.
//
// Remarks: This function calls a recursive function that runs until it 
// 	connects the parameter to the current point, or until it has searched 2 
// 	intersections and their connecting roads.  The base case is where the 
// 	current point is on the same road or intersection as the user's parameter 
// 	point.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance.
//
// Argument:
// 	cPoint - A valid CPathPoint instance.
//	extension - (output) Contains any CPathPoints needed to extend the 
//      current CPathPoint to the given point. Note that this list will 
//      include the current point.
//
// Returns - True if the current CPathPoint connects with the parameter, false
// 	otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Append(
			const CPathPoint& cPoint,
			CFastDeque<CPathPoint>& extension
			)
{
	AssertValid();

	//
	// Use a temporary point so that if no connection is found, 
	// the current CPathPoint is not altered.
	//
	CPathPoint tmpPoint = *this;
	extension.clear();

	if( tmpPoint.RecursiveAppend( cPoint, &tmpPoint, extension, 0 ) )
	{
		*this = tmpPoint;
		return true;
	}
	else 
	{
		return false;
	}
} // end of Append

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint forward through the next 
//  intersection so that no lane changes are needed.
//
// Remarks: This function extends the range of distances covered by the 
// 	current CPathPoint in an attempt to make the desired turn at the next 
// 	possible intersection.  
//
// 	If the current CPathPoint lies on an intersection, then the destination
// 	intersection of the connecting lanes is used.  If the current CPathPoint
// 	list on a road, then the destination intersection of the current lanes
// 	is used.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Argument:
//	extension - output parameter, will contain any CPathPoints needed to 
//		extend the current CPathPoint through the given turn.
//  cLaneCrdrMask - A mask that determines which lanes are to be used to 
//      connect to the next intersection.
//
// Returns - The distance appended
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::AppendNoLaneChange(
			CFastDeque<CPathPoint>& extension,
			const bitset<cCV_MAX_CRDRS>& cLaneCrdrMask
			)
{
	AssertValid();


	double distSoFar;

	// If the current CPathPoint lies on an intersection
	if( m_isRoad )
	{
		//
		// Append the speicified dist along the current road and
		// check to see if the road is long enough to satisfy
		// the dist requirement and quit.
		//
		distSoFar = AppendEntireRoad();

		//
		// Add the next intersection according to the specified
		// turn angle.
		// 
		CIntrsctn intrsctn = GetNextIntrsctn();

		vector<CRoad> roadVec;
		intrsctn.GetRoadAccessListThrghIntrsctn( m_road, cLaneCrdrMask, roadVec );

		if( roadVec.size() > 0 )
		{
			// Create a CPathPoint on the intersection
			CPathPoint isecPoint( GetCved() );
			bool success = isecPoint.SetIntrsctnPoint(
											intrsctn, 
											m_road,
											GetFirstLaneCrdrIdx(),
											*roadVec.begin()
											);
			if( success ) 
			{	
				//
				// Set the current range to end.
				//
				SetRangeToEnd( false, true );

				//
				// Extend isecPoint range over full corridor 
				// and add it to the list
				//
				isecPoint.SetRangeToStart( true, true );
				extension.push_back( isecPoint );  // CFastDeque<CPathPoint>& 

				distSoFar += extension.back().AppendNoLaneChange( extension, cLaneCrdrMask );
			}
		}	
	}
	else
	{
		//
		// Extend isecPoint range over full corridor and add it to the list.
		//
		distSoFar = AppendEntireIntrsctn();
	} // If the current CPathPoint lies on an intersection
	
	return distSoFar;

} // end of Append

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint backward randomly for 
//  the given distance along the road network.
//
// Remarks: This function extends the range of distances covered by the 
// 	current CPathPoint in an attempt to cover the desired distance.  If the 
// 	current CPathPoint is not enough to cover the distance, then a list of 
// 	CPathPoints is created that cover the distance, along the first path 
// 	found.  If the CPathPoint can be extended any distance at all, the 
//	function returns true.  Otherwise, it returns false.  Note that if 
//	there are multiple paths, only one is given.
// 
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Argument:
// 	dist - A double distance along the road network to extend.
//  pRng - A pointer to the random number generator.
//  rngStreamId - The rng stream id.
//	extension - (output) Contains any CPathPoints needed to 
//		extend the current CPathPoint by the given distance.
//  stayOnHighway - A boolean that indicates if prepend should stay on
//      roads that have been marked as being highways.
//
// Returns - True if the point can be extended, false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Prepend( 
			double dist, 
			CRandNumGen* pRng,
			int rngStreamId,
			CFastDeque<CPathPoint>& extension,
			bool stayOnHighway
			)
{
	//
	//	BASE CASE
	//
	if( dist <= 0 )
	{
		return true;
	}
	//
	//	RECURSIVE CASE
	//
	else
	{
		AssertValid();

		double distToTravel;
		double distToStart;

		// If the current CPathPoint is on a road
		if( m_isRoad )
		{
			int firstLaneIdx = GetFirstLaneCrdrIdx();
			double direction;
			cvTLane* pLane = BindLane( m_road.GetLaneIdx() + firstLaneIdx );
			if( pLane->direction == eNEG )
			{
				distToStart = (
						m_road.GetLinearLength() - m_startDist[firstLaneIdx]
						);
				direction = -1.0f;
			}
			else 
			{
				distToStart = m_startDist[firstLaneIdx];
				direction = 1.0f;
			}
			distToTravel = min( dist, distToStart );

			int laneId;
			for( laneId = firstLaneIdx; laneId < m_road.GetNumLanes(); laneId++ )
			{
				if( m_laneCrdrMask.test( laneId ) )
				{
					m_startDist[laneId] -= direction * distToTravel;
				}
			}
				 
			// If there's distance left to prepend
			bool distLeftToPrepend = dist > distToTravel;
			if( distLeftToPrepend )
			{
				// Add a new CPathPoint for the connecting intersection
				CPathPoint isecPoint( GetCved() );
				CIntrsctn intrsctn = GetPrevIntrsctn();
				if( stayOnHighway )
				{
					//
					// Pick the pathpoint so that it's on a highway.  Do
					// this by finding all the roads that lead to the 
					// current road thru the previous intersection.  Pick
					// the first road that is a highway,  Add the corridor
					// that leads from that road to the current road.
					//
					TRoadVec roadVec;
					intrsctn.GetReverseRoadAccessListThrghIntrsctn(
										m_road,
										roadVec
										);

					int numRoads = (int) roadVec.size();
					bool haveRoads = numRoads > 0;
					if( haveRoads )
					{
						TRoadVec::iterator i;
						bool foundHighway = false;
						for( i = roadVec.begin(); i != roadVec.end(); i++ )
						{
							bool isRoadHighway = i->ActiveAttr( 
													cCV_HIGHWAY_ATTR 
													);
							if( isRoadHighway )
							{
								//
								// This road is a highway.  Use this one.
								//
								foundHighway = true;
								break;
							}
						}

						if( foundHighway )
						{
							bool success = isecPoint.SetIntrsctnPoint(
 															intrsctn, 
															*i, 
															m_road,
															firstLaneIdx
															);
							if( success )
							{
								isecPoint.SetRangeToEnd( true, true );
								extension.push_front( isecPoint );
							}
							else
							{
								// Some error occurred, so stop recursion.
								distToTravel = dist;
							}
						}
						else
						{
							// 
							// No highway roads leading to current road thru
							// the previous intersection, so stop recursion.
							//
							distToTravel = dist;
						}
					}
					else
					{
						// 
						// No highway roads leading to current road thru
						// the previous intersection, so stop recursion.
						//
						distToTravel = dist;
					}
				}
				else 
				{
					//
					// Get all the corridors that end at the current road.
					// 
					TCrdrVec crdrVec;
					intrsctn.GetCrdrsLeadingTo( m_road, crdrVec );

					int numCrdrs = (int) crdrVec.size();
					bool haveCrdrs = numCrdrs > 0;
					if( haveCrdrs )
					{
						int crdrIdx;
						if( pRng )
						{
							//
							// Have an assigned random number generator.
							//
							crdrIdx =  pRng->RandomLongRange(
													0, 
													numCrdrs, 
													rngStreamId
													);
						}
						else
						{
							//
							// No random number generator assigned.  Use 
							// Rand :(
							//
							srand( (unsigned) 1 );
							crdrIdx = static_cast<int>(rand() / ( (double) RAND_MAX ) * numCrdrs);
						}

						bool success = isecPoint.SetIntrsctnPoint(
												intrsctn, 
												crdrVec[crdrIdx].GetSrcRd(), 
												m_road,
												firstLaneIdx
												);
						if( success )
						{
							isecPoint.SetRangeToEnd( true, true );
							extension.push_front( isecPoint );
						}
						else
						{
							// Some error occurred, so stop recursion.
							distToTravel = dist;
						}
					}
					else
					{
						// 
						// No corridors leading to current road, so 
						// stop recursion.
						//
						distToTravel = dist;
					}
				}

			} // If there's distance left to append
		} // If the current CPathPoint is on a road
	
		// Else, the current CPathPoint is on an intersection
		else 
		{
			//
			// Prepend any needed distance from the start of the
			// corridor.
			//
			int crdrId, firstCrdr =  GetFirstLaneCrdrIdx();
			cvTCrdr* pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() + firstCrdr );
			CCrdr crdr( GetCved(), pCrdr );
			
			distToStart = m_startDist[firstCrdr];
			distToTravel = min( dist, distToStart );
			for( 
				crdrId = firstCrdr; 
				crdrId < m_intrsctn.GetNumCrdrs(); 
				crdrId++
				)
			{
				if( m_laneCrdrMask.test( crdrId ) )
				{
					m_startDist[crdrId] -= distToTravel;
				}
			}

			//
			// If there's distance left to prepend then go to the road
			// connecting to the corridor.
			//
			if( dist > distToTravel )
			{
				SetRangeToStart( true, false );

				// Add a new CPathPoint for the connecting road
				CPathPoint roadPoint( GetCved() );
				if( roadPoint.SetRoadPoint( crdr, true ) )
				{
					roadPoint.SetRangeToEnd( true, true );
					if( stayOnHighway )
					{
						//
						// Check to make sure that the previous road is
						// indeed a highway.
						//
						bool isRoadHighway = (
							roadPoint.GetRoad().ActiveAttr(	cCV_HIGHWAY_ATTR )
							);
						if( isRoadHighway )
						{
							extension.push_front( roadPoint );
						}
						else
						{
							// Previous road is not a highway.
							distToTravel = dist;
						}
					}
					else
					{
						extension.push_front( roadPoint );
					}
				}
				else 
				{
					// Some error occurred, so stop recursion.
					distToTravel = dist;
				}

			} // If there's distance left to append
		} // Else, the current CPathPoint is on an intersection
	
		// Recur 
		return(
			extension.front().Prepend(
								dist-distToTravel, 
								pRng,
								rngStreamId,
								extension,
								stayOnHighway
								)
			);
	}

} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint backward along the turn 
/// defined by the given angle and epsilon.
//
// Remarks: This function extends the range of distances covered by the 
// 	current CPathPoint in an attempt to make the desired turn at the previous 
// 	intersection.  If the current CPathPoint is not enough to cover 
// 	the turn, then a list of CPathPoints is created that make the turn,
// 	and it is returned in the output parameter.  If the turn can be made and 
// 	an appropriate path found, then the function returns true.  If there is
// 	no previous intersection, or if no turn in the given direction can be found, 
// 	then the function returns false.
//
// 	If the current CPathPoint lies on an intersection, then the source
// 	intersection of the connecting lanes is used.  If the current CPathPoint
// 	list on a road, then the source intersection of the current lanes
// 	is used.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance.
//
// Argument:
// 	turnAngle - Defines the clockwise angle (in radians) made between the 
// 		direction of the source lane(s) and the current lane(s).
// 		Note some useful predefined constants can be used for this parameter:
// 		cCV_STRAIGHT_TURN - go straight through the intersection
// 		cCV_RIGHT_TURN - turn right
// 		cCV_U_TURN - make a U-turn
// 		cCV_LEFT_TURN - turn left 
// 	epsilon - The +/- tolerance of the TurnAngle when finding the desired
// 		source lane(s).  Note that there is a predefined constant for 
// 		this parameter: cCV_EPSILON = 1/8 * pi
//	extension - (output) Contains any CPathPoints needed to extend the 
//	    current CPathPoint through the given turn.
//
// Returns - True if the turn can be made at the previous intersection, false 
// 	otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Prepend(
			double turnAngle, 
			double epsilon,
			CFastDeque<CPathPoint>& extension
			)
{
	AssertValid();

	CPathPoint roadPoint = *this;
	bool takeBest = false;
	extension.clear();

	// If the current CPathPoint lies on an intersection
	if( !m_isRoad )
	{	
		// Create a CPathPoint to cover the connecting road
		CCrdr crdr( m_intrsctn, GetFirstLaneCrdrIdx() );

		if( roadPoint.SetRoadPoint( crdr, true ) )
		{
			roadPoint.SetRangeToStart( true, false );
			roadPoint.SetRangeToEnd( false, true );

			extension.push_front( roadPoint );
		}
		else 
		{
			// There is no connecting road, so return false.
			return false;
		}
	} // If the current CPathPoint lies on an intersection

	CIntrsctn intrsctn = roadPoint.GetPrevIntrsctn();
	double bestDiffAngle = cTWO_TIMES_PI;
	double dstAngle, diffAngle;

	vector<CRoad> roadVec;
	intrsctn.GetReverseRoadAccessListThrghIntrsctn( 
										roadPoint.m_road, 
										roadVec 
										);
	vector<CRoad>::const_iterator pBestRoad = roadVec.end();

	// For each road accessible from roadPoint.m_road
	vector<CRoad>::const_iterator itr;
	for( itr = roadVec.begin(); itr != roadVec.end(); itr++ )
	{
		// Get the angle between the source and destination roads
		dstAngle = GetAngleBetween( intrsctn, *itr, roadPoint.m_road );

		// Compute the difference between the dstAngle and the 
		// turn angle
		diffAngle = fabs( dstAngle - turnAngle );
		if ( diffAngle > cTWO_TIMES_PI/2.0 )
			diffAngle = cTWO_TIMES_PI - diffAngle;

		if( diffAngle < bestDiffAngle )
		{
			pBestRoad = itr;
			bestDiffAngle = diffAngle;
		}
	} // For each road accessible from roadPoint.m_road
	
	// If we're taking the best we found, or if the bestDiffAngle
	//	is within epsilon of the turnAngle, then make the turn
	if( takeBest || ( bestDiffAngle <= epsilon + cPATH_POINT_ZERO ) )
	{
		if( pBestRoad != roadVec.end() )
		{
			// Create a CPathPoint on the intersection
			CPathPoint isecPoint( GetCved() );
			bool success = isecPoint.SetIntrsctnPoint(
											intrsctn, 
											*pBestRoad, 
											roadPoint.m_road,
											roadPoint.GetFirstLaneCrdrIdx()
											);
			if( success )
			{	
				// Extend isecPoint range over full corridor 
				// and add it to the list
				isecPoint.SetRangeToStart( true, false );
				isecPoint.SetRangeToEnd( false, true );
				extension.push_front( isecPoint );

				// Create a CPathPoint on the previous road
				CPathPoint prevRoadPoint( GetCved() );
				if( prevRoadPoint.SetRoadPoint( isecPoint.GetCrdr(), true ) )
				{
					// Set distance range to the end of the road 
					//	 and add it to the list
					prevRoadPoint.SetRangeToEnd( true, true );
					extension.push_front( prevRoadPoint );

					// Extend current CPathPoint to its end
					SetRangeToStart( true, false );

					return true;
				}
			}
		}
	}

	// If control makes it here, an error occurred in the processing or 
	//	no acceptable turn angle was found.
	return false;

} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
//
// Description: Extends the current CPathPoint backward to connect 
//  itself with the parameter.
//
// Remarks: This function extends the range of distances covered by the 
// 	current CPathPoint in an attempt to incorporate the CPathPoint in the
// 	parameter.  If the parameter cannot be included in the current 
// 	CPathPoint, then a list of CPathPoints is created that connects the 
// 	current point to the parameter, and it is returned in the output
// 	parameter.  Note that the returned list does not include the point 
// 	parameter.  If the points can be connected, then the function returns 
// 	true.  Otherwise, it returns false.
//
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Argument:
// 	cPoint - A valid CPathPoint instance.
//	extension - (output) Contains any CPathPoints needed to extend the
//		current CPathPoint to the given point.
//
// Returns - True if the current CPathPoint connects with the parameter, false
// 	otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Prepend( 
			const CPathPoint& cPoint, 
			CFastDeque<CPathPoint>& extension
			)
{
	AssertValid();

	// Use a temporary point so that if no connection is found, 
	//	the current CPathPoint is not altered.
	CPathPoint tmpPoint = *this;
	extension.clear();

	if( tmpPoint.RecursivePrepend( cPoint, &tmpPoint, extension, 0 ) )
	{
		*this = tmpPoint;
		return true;
	}
	else
	{
		return false;
	}
} // end of Prepend

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetString
// 	Initializes the current CPathPoint with the string parameter.
//
// Remarks:  The format of the string is as follows:
// 	If the node lies on a road:
// 		R:Road_Name:Lane_Id[Start_dist:End_dist]:Lane_Id[Start_dist:...
// 	Else if the node lies on an intersection
// 		I:Intrsctn_Name:Crdr_Id[Start_dist:End_dist]:Crdr_Id[Start_dist:...
//
//	Note that the current CPathPoint must have been initialized with a valid
//	CCved instance.
//
// Arguments:
// 	point - (Input) Contains a string representing the current CPathPoint
//
// Returns: True if a valid CPathPoint can be created from the parameter, 
// 	false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::SetString(const string& point) 
{
	CCvedItem::AssertValid();

	char type;
	string strName;
	int id;
	double startDist;
	double endDist;
	size_t buffSize = strlen(point.c_str())+1;
	char *pBuf = new char[buffSize];
    char *pTokDel = " :][";
    char *pTok;
	char *pCurPos = NULL;

	// Parse out the type of point:
	// 	R -> road, I -> intersection, 
	// 	else invalid.
    strcpy_s(pBuf,buffSize, point.c_str());
    pTok = strtok_s(pBuf, pTokDel,&pCurPos);
	type = pTok[0];
  
	// Parse out the name of the road/intersection
	pTok = strtok_s(NULL, pTokDel,&pCurPos);
    strName = pTok;

	if (type == 'R') {
		m_isRoad = true;
		m_road = GetCved().GetRoad(strName);		
	}
	else if (type == 'I') {
		m_isRoad = false;
		m_intrsctn = GetCved().GetIntersection(strName);		
	}
	else{
	    delete[] pBuf;
	    return false;
	}

	// Clear out the current lane/crdr mask
	m_laneCrdrMask.reset();

	// While there's still lane/crdrs left to parse,
	// 	parse them into id[startDist:endDist] chunks.
	while ( (pTok = strtok_s(NULL, pTokDel,&pCurPos)) != NULL ) {
    	id = atoi(pTok);

    	pTok = strtok_s(NULL, pTokDel,&pCurPos);
    	startDist = atof(pTok);
    	pTok = strtok_s(NULL, pTokDel,&pCurPos);
    	endDist = atof(pTok);

		// If it's a valid id, add it to the mask
		// 	and save the start/end distances
		if ( (id >= 0) && (id < cCV_MAX_CRDRS) ) {

			m_laneCrdrMask.set(id);
			m_startDist[id] = startDist;
			m_endDist[id] = endDist;
		}
	}

	// Delete the allocated buffer
	delete [] pBuf;

	// Return true if one or more lanes/crdrs was given, 
	// 	false otherwise.
	return (m_laneCrdrMask.any());

} // end of SetString


//////////////////////////////////////////////////////////////////////////////
//		Other functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: Similar
// 	Returns true if the parameter and the current instance share one or more 
// 	similar lanes or corridors and if the range of distances overlap.
//
// Remarks: This function will cause a failed assertion if it is called on an 
//	invalid CPathPoint instance
//
// Argument: 
// 	point - valid CPathPoint to check for similarity
//
// Returns: true or false
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Similar(const CPathPoint& point) const
{
	AssertValid();
	point.AssertValid();

	int roadIsecIdP, roadIsecIdC;
	int laneCrdrId;
	bitset<cCV_MAX_CRDRS> overlap = m_laneCrdrMask;
	double minP, maxP, minC, maxC;

	if (m_isRoad)
		roadIsecIdC = m_road.GetId();
	else
		roadIsecIdC = m_intrsctn.GetId();
	if (point.m_isRoad)
		roadIsecIdP = point.m_road.GetId();
	else
		roadIsecIdP = point.m_intrsctn.GetId();

	// If they're both on a road or both on an intersection
	if (m_isRoad  == point.m_isRoad) {

		// If the roads/intersections match
		if (roadIsecIdC == roadIsecIdP) {

			// If any lanes/corridors overlap
			overlap &= point.m_laneCrdrMask;
			if (overlap.any()) {

				// For each overlapping lane
				for (laneCrdrId = 0; laneCrdrId < cCV_MAX_CRDRS; laneCrdrId++) {

					if (overlap.test(laneCrdrId)) {

						
						if (point.m_startDist[laneCrdrId] < 
							point.m_endDist[laneCrdrId]) {

							minP = point.m_startDist[laneCrdrId];
							maxP = point.m_endDist[laneCrdrId];
						}
						else {
							minP = point.m_endDist[laneCrdrId];
							maxP = point.m_startDist[laneCrdrId];
						}
						if (m_startDist[laneCrdrId] < m_endDist[laneCrdrId]) {
							minC = m_startDist[laneCrdrId];
							maxC = m_endDist[laneCrdrId];
						}
						else {
							minC = m_endDist[laneCrdrId];
							maxC = m_startDist[laneCrdrId];
						}

						// If the range of distances for the parameter are 
						// 	disjoint from the range of distances for the 
						// 	current CPathPoint instance, then they are not 
						// 	similar.
						if ( (maxP < minC) || (maxC < minP) )
							return false;
					}

				} // For each overlapping lane

				// If control makes it here, then they are similar
				return true;

			} // If any lanes overlap

		} // If the roads match

	} // If they're both on a road

	// If control makes it here, they're not similar.
	return false;

} // end of Similar

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns true if the parameter and the current instance 
//   share one or more adjacent lanes or corridors.
//
// Remarks: Two lanes are considered adjacent if they are next to 
//   each other on the road and have the same direction on an 
//   overlapping range of distances.  Two corridors are considered 
//   adjacent if they have matching source and destination roads, 
//   on an overlapping range of distances.
//
// 	 This function will cause a failed assertion if it is called
// 	 on an invalid CPathPoint instance.
//
//   THIS FUNCTION IS NOT YET COMPLETE.
//
// Argument: 
// 	 cPoint - A valid CPathPoint to check for adjacency.
//
// Returns: A boolean.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Adjacent( const CPathPoint& cPoint ) const
{
	// TODO: Does anyone need this function?
	AssertValid();
	return false;

} // end of Adjacent

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns true if the current instance contains the 
//   road that the parameter lies on.
//
// Remarks: This function will cause a failed assertion if it is 
//   called on an invalid CPathPoint instance or if the parameter 
//   is invalid.
//
// Argument: 
// 	 cRoad - A valid CRoad to check.
//
// Returns: A boolean indicating if the path point contains the road.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Contains( const CRoad& cRoad ) const
{

	AssertValid();
	assert( cRoad.IsValid() );

	// This CPathPoint instance must be on a road.
	if ( m_isRoad ) {

		// If they're both on the same road
		if ( m_road.GetId() == cRoad.GetId() ) {

			return true;
			
		} // If they're both on the same road

	} // If this CPathPoint instance is on a road
		
	// If control makes it here, they're not similar.
	return false;

} // end of Contains

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns true if the current instance contains the road 
//   and lane (or intersection and corridor) that the parameter lies on, 
//   and if the distance of the parameter falls within the range of 
//   distances of the current instance.
//
// Remarks: This function will cause a failed assertion if it is called 
//   on an invalid CPathPoint instance or if the parameter is invalid.
//
// Argument: 
//   cRoadPos - A valid CRoadPos to check.
//
// Returns: A boolean.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Contains( const CRoadPos& cRoadPos ) const
{

	AssertValid();
	assert(cRoadPos.IsValid());
	if (!cRoadPos.IsValid()){
		return false;
	}
	// If they're both on a road
	bool bothOnRoad = m_isRoad && cRoadPos.IsRoad();
	if( bothOnRoad )
	{
		// If they're both on the same road
		bool sameRoad = m_road.GetId() == cRoadPos.GetRoad().GetId();
		if( sameRoad )
		{
			// If roadPos is on a lane in the mask
			int laneId = cRoadPos.GetLane().GetRelativeId();
			bool onLaneInMask = m_laneCrdrMask.test( laneId );
			if( onLaneInMask )
			{
				// If cRoadPos's distance is on the range, then return 
				// true (else false).
				double dist = cRoadPos.GetDistance();
//				gout << "      laneId = " << laneId << endl;
//				gout << "      m_startDist = " << m_startDist[laneId] << endl;
//				gout << "      m_endDist = " << m_endDist[laneId] << endl;
//				gout << "      dist = " << dist  << endl;
				bool posDirLane = m_startDist[laneId] < m_endDist[laneId];
				if( posDirLane )
				{
//					double distPlus = dist + cPATH_POINT_ZERO;
//					double distMinus = dist - cPATH_POINT_ZERO;
//					bool first = m_startDist[laneId] <= distPlus;
//					bool second = distMinus <= m_endDist[laneId];
//					gout << "      first = " << first << endl;
//					gout << "      second = " << second << endl;
//					gout << "      distMinus = " << distMinus << "   distPlus = " << distPlus << endl;
					
					return ( 
						( m_startDist[laneId] <= dist + cPATH_POINT_ZERO ) &&
						( dist - cPATH_POINT_ZERO <= m_endDist[laneId] ) 
						);
				} 
				else 
				{
				
					return ( 
						( m_endDist[laneId] <= dist + cPATH_POINT_ZERO ) &&
						( dist - cPATH_POINT_ZERO <= m_startDist[laneId] ) 
						);
				}
			} // If cRoadPos is on a lane in the mask
		} // If they're both on the same road
	} // If they're both on a road
	
	// If they're both on an intersection 
	bool bothOnIntrsctn = (!m_isRoad) && (!cRoadPos.IsRoad());
	if( bothOnIntrsctn )
	{
		// If they're both on the same intersection
		bool sameIntrsctn = m_intrsctn.GetId() == cRoadPos.GetIntrsctn().GetId();
		if( sameIntrsctn)
		{
			// If cRoadPos is on a lane in the mask
			vector<int> crds;
			cRoadPos.GetCorridors(crds);
			vector<int>::const_iterator itr;
			int intrCrdrIdx = m_intrsctn.GetCrdrIdx() ;
			for (itr = crds.begin(); itr != crds.end(); itr++){
				int crdrId = *itr - intrCrdrIdx; //convert from abs id to relative id
				if (crdrId >= cCV_MAX_CRDRS|| crdrId < 0 ){//added new bounds check to deal with crash
					return false;
				}
				bool onCrdrInMask = m_laneCrdrMask.test( crdrId );
				if( onCrdrInMask )
				{
					// If cRoadPos's distance is on the range, then return 
					// true (else false)
					double dist = cRoadPos.GetDistance();
					if( m_startDist[crdrId] < m_endDist[crdrId] )
					{
						if( ( (m_startDist[crdrId] <= dist+cPATH_POINT_ZERO) &&
							(dist-cPATH_POINT_ZERO <= m_endDist[crdrId]) )){
							return true;
						}

						//return ( (m_startDist[crdrId] <= dist+cPATH_POINT_ZERO) &&
						//		 (dist-cPATH_POINT_ZERO <= m_endDist[crdrId]) );
					} 
					else
					{	

						if ( ( (m_endDist[crdrId] <= dist+cPATH_POINT_ZERO) &&
							(dist-cPATH_POINT_ZERO <= m_startDist[crdrId]) ) ){
							return true;
						}
	
						//return ( (m_endDist[crdrId] <= dist+cPATH_POINT_ZERO) &&
						//		 (dist-cPATH_POINT_ZERO <= m_startDist[crdrId]) );
					}
				} // If cRoadPos is on a crdr in the mask
			}
		} // If they're both on the same intersection
	} // If they're both on an intersection
	
	// If control makes it here, they're not similar.
	return false;

} // end of Contains

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the CVED object ids of the objects that 
//   lie on the current point.
//
// Remarks: The object ids are placed into the vector in order 
//   along the current point.  Objects on all lanes/crdrs of the 
//   current point are returned.
//
//   The output vector in never cleared before information is 
//   written to it.  Thus, prior information is preserved.
//
// Arguments:
// 	objects - (output) Contains the object ids of the objects on 
//      the point.
//
// Returns: The number of objects on the current point.
//
//////////////////////////////////////////////////////////////////////////////
int
CPathPoint::GetObjectsOnPoint( vector<int>& objects ) const
{
	/*static*/ vector<int> tmpObjIds;
	//static bool        sInit = false;

	//if ( sInit == false ) {
	//	sInit = true;
		tmpObjIds.reserve(20);
	//}

	if ( m_isRoad ) 
	{
		// 
		// This path point lies on a road.
		//
		int laneId = GetFirstLaneCrdrIdx();
		if (laneId < 0) return 0;
		bitset<cCV_MAX_LANES> laneMask((int) m_laneCrdrMask.to_ulong() );

		bool posDirLane = m_startDist[laneId] < m_endDist[laneId];
		if( posDirLane ) 
		{
			GetCved().GetAllDynObjsOnRoadRange( 
						m_road.GetId(), 
						laneMask, 
						m_startDist[laneId], 
						m_endDist[laneId], 
						objects
						);
		}
		else 
		{
			tmpObjIds.clear();
			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						m_endDist[laneId], 
						m_startDist[laneId], 
						tmpObjIds
						);
		
			//
			// Need to insert objects in reverse order to maintain order
			// along the point.
			//
			vector<int>::reverse_iterator itr;
			for ( itr = tmpObjIds.rbegin(); itr < tmpObjIds.rend(); itr++ ) 
			{
				objects.push_back( *itr );
			}
		}
	} // If the point lies on a road
	else 
	{
		//
		// This path point lies on an intersection.
		//
		int crdrId = GetFirstLaneCrdrIdx();

		if( m_startDist[crdrId] < m_endDist[crdrId] )
		{
			GetCved().GetAllDynObjsOnIntrsctnRange(
						m_intrsctn.GetId(), 
						m_laneCrdrMask, 
						m_startDist, 
						m_endDist, 
						objects
						);
			
		}
		else 
		{
			tmpObjIds.clear();
			GetCved().GetAllDynObjsOnIntrsctnRange(
						m_intrsctn.GetId(), 
						m_laneCrdrMask, 
						m_endDist, 
						m_startDist, 
						tmpObjIds
						);
		
			//
			// Need to insert objects in reverse order to maintain order
			// along the point.
			//
			vector<int>::reverse_iterator itr;
			for( itr = tmpObjIds.rbegin(); itr < tmpObjIds.rend(); itr++ ) 
			{
				objects.push_back( *itr );
			}
		}
	} // Else the point lies on an intersection

	return (int) objects.size();
} // GetObjectsOnPoint


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets the angle between the source and destination road.
//
// Remarks:  The path point must be an intersection.  Throws an assertion
//   if the CPathPoint is invalid.
//
// Arguments:
//
// Returns: The angle between the source and destination roads.  A negative
//   number if it's unable to compute the angle.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::GetSrcDestAngle() const
{
	AssertValid();

	//
	// Return error for roads.
	//
	if ( m_isRoad )  return -1.0;

	// Get previous and next roads related to the intersection.
	CRoad prevRoad = GetPrevRoad();
	CRoad nextRoad = GetNextRoad();

	double angle = GetAngleBetween( m_intrsctn, prevRoad, nextRoad );

	return angle;
} // end of GetSrcDestAngle


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextLaneClockwise
// 	Gets the lane that's closest clockwise to the current road 
//  path point after the upcoming intersection.
//
// Remarks:  The path point must be a road.  This function finds the lane
//   that closest to the current road to the left or clockwise.
//   If there are no lanes to the left then it simply picks the one that's
//   the next closest.  As an example, consider a T-intersection where there
//   are only 2 lanes that connect to the current road: one goes straight
//   and the other one goes right.  This function would return the lane that
//   goes straight since that's closest to the left.
//
// Arguments:
//  nextLane - The next lane closest clockwise.
//
// Returns: A boolean indicating if it was unables to find any lanes.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::GetNextLaneClockwise( CLane& nextLane ) const
{

	AssertValid();

	//
	// Return false for intersections.
	//
	if ( !m_isRoad )  return false;

	CIntrsctn intrsctn = GetNextIntrsctn();
	if ( !intrsctn.IsValid() )  return false;

	//
	// Build a CRoadPos at the end of the source road into the
	// intersection.
	//
	CRoad srcRoad = GetRoad();
	CLane srcLane = GetLane();
	double endDist;
	if ( srcLane.GetDirection() == ePOS )
		endDist = srcRoad.GetLinearLength() - 1.0;
	else 
		endDist = 1.0;
	CRoadPos srcEndRoadPos( srcRoad, srcLane, endDist );
	if ( !srcEndRoadPos.IsValid() ) {

		cerr << "CPathPoint::GetNextLaneClockwise: ";
		cerr << "unable to build end of source road RoadPos" << endl;

		return false;

	}

	//
	// Get the tangent of this CRoadPos that has just been built.
	//
	CVector3D srcTan = srcEndRoadPos.GetTangent();

#if 0
	gout << "src = " << srcEndRoadPos << endl;
	gout << "srcTan = " << srcTan << endl;
#endif

	//
	// Get the list of roads that can be accessed thru the intersection.
	//
	TRoadVec destRoads;
	intrsctn.GetRoadAccessListThrghIntrsctn( srcRoad, destRoads );

	if ( destRoads.size() <= 0 )  return false;

	bool foundClockwise = false;
	double maxClockwise;
	CLane clockwiseLane;
	TRoadVec::iterator i;
	for ( i = destRoads.begin(); i != destRoads.end(); i++ ) {

		CRoad dstRoad = *i;

		//
		// Find one of the lanes on the destination road that the
		// source lane connects to.
		//
		TCrdrVec crdrs;
		intrsctn.GetCrdrsStartingFrom( srcRoad, crdrs );

		bool foundDstLane = false;
		TCrdrVec::iterator crdr;
		for ( crdr = crdrs.begin(); crdr != crdrs.end(); crdr++ ) {

			if ( dstRoad == (*crdr).GetDstntnRd() ) {

				foundDstLane = true;
				break;

			}

		}

		if ( !foundDstLane )  continue;

		//
		// Build a CRoadPos at the start of the destination road from the
		// intersection.
		//
		CLane dstLane = (*crdr).GetDstntnLn();
		double startDist;
		if ( dstLane.GetDirection() == ePOS )
			startDist = 1.0;
		else 
			startDist = dstRoad.GetLinearLength() - 1.0;
		CRoadPos dstEndRoadPos( dstRoad, dstLane, startDist );
		if ( !dstEndRoadPos.IsValid() ) {

			cerr << "CPathPoint::GetNextLaneClockwise: ";
			cerr << "unable to build start of destination road RoadPos";
			cerr << endl;

			return false;

		}

		//
		// Get the tangent of this CRoadPos that has just been built.
		//
		CVector3D dstTan = dstEndRoadPos.GetTangent();

#if 0
		gout << "dst = " << dstEndRoadPos << endl;
		gout << "dstTan = " << dstTan << endl;
#endif

		CVector3D crossVec = srcTan.CrossP( dstTan );

#if 0
		gout << "crossVec = " << crossVec << endl;
		gout << "Road '" << dstRoad << "' has result = " << crossVec.m_k << endl;
#endif

		if ( !foundClockwise ) {

			maxClockwise = crossVec.m_k;
			clockwiseLane = dstLane;
			foundClockwise = true;

		}
		else {
			
			if ( crossVec.m_k > maxClockwise ) {

				maxClockwise = crossVec.m_k;
				clockwiseLane = dstLane;

			}

		}

	}

	if ( foundClockwise )  nextLane = clockwiseLane;

	return foundClockwise;

}  // end of GetNextLaneClockwise


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNextLaneCounterClockwise
// 	Gets the lane that's closest counter-clockwise to the current road 
//  path point after the upcoming intersection.
//
// Remarks:  The path point must be a road.  This function finds the lane
//   that closest to the current road to the right or counter-clockwise.
//   If there are no lanes to the right then it simply picks the one that's
//   the next closest.  As an example, consider a T-intersection where there
//   are only 2 lanes that connect to the current road: one goes straight
//   and the other one goes left.  This function would return the lane that
//   goes straight since that's closest to the right.
//
// Arguments:
//  nextLane - The next lane closest counter-clockwise.
//
// Returns: A boolean indicating if it was unables to find any lanes.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::GetNextLaneCounterClockwise( CLane& nextLane ) const
{

	AssertValid();

	//
	// Return false for intersections.
	//
	if ( !m_isRoad )  return false;

	CIntrsctn intrsctn = GetNextIntrsctn();
	if ( !intrsctn.IsValid() )  return false;

	//
	// Build a CRoadPos at the end of the source road into the
	// intersection.
	//
	CRoad srcRoad = GetRoad();
	CLane srcLane = GetLane();
	double endDist;
	if ( srcLane.GetDirection() == ePOS )
		endDist = srcRoad.GetLinearLength() - 1.0;
	else 
		endDist = 1.0;
	CRoadPos srcEndRoadPos( srcRoad, srcLane, endDist );
	if ( !srcEndRoadPos.IsValid() ) {

		cerr << "CPathPoint::GetNextLaneCounterClockwise: ";
		cerr << "unable to build end of source road RoadPos" << endl;

		return false;

	}

	//
	// Get the tangent of this CRoadPos that has just been built.
	//
	CVector3D srcTan = srcEndRoadPos.GetTangent();

#if 0
	gout << "src = " << srcEndRoadPos << endl;
	gout << "srcTan = " << srcTan << endl;
#endif

	//
	// Get the list of roads that can be accessed thru the intersection.
	//
	TRoadVec destRoads;
	intrsctn.GetRoadAccessListThrghIntrsctn( srcRoad, destRoads );

	if ( destRoads.size() <= 0 )  return false;

	bool foundCounterClockwise = false;
	double minCounterClockwise;
	CLane counterClockwiseLane;
	TRoadVec::iterator i;
	for ( i = destRoads.begin(); i != destRoads.end(); i++ ) {

		CRoad dstRoad = *i;

		//
		// Find one of the lanes on the destination road that the
		// source lane connects to.
		//
		TCrdrVec crdrs;
		intrsctn.GetCrdrsStartingFrom( srcRoad, crdrs );

		bool foundDstLane = false;
		TCrdrVec::iterator crdr;
		for ( crdr = crdrs.begin(); crdr != crdrs.end(); crdr++ ) {

			if ( dstRoad == (*crdr).GetDstntnRd() ) {

				foundDstLane = true;
				break;

			}

		}

		if ( !foundDstLane )  continue;

		//
		// Build a CRoadPos at the start of the destination road from the
		// intersection.
		//
		CLane dstLane = (*crdr).GetDstntnLn();
		double startDist;
		if ( dstLane.GetDirection() == ePOS )
			startDist = 1.0;
		else 
			startDist = dstRoad.GetLinearLength() - 1.0;
		CRoadPos dstEndRoadPos( dstRoad, dstLane, startDist );
		if ( !dstEndRoadPos.IsValid() ) {

			cerr << "CPathPoint::GetNextLaneCounterClockwise: ";
			cerr << "unable to build start of destination road RoadPos";
			cerr << endl;

			return false;

		}

		//
		// Get the tangent of this CRoadPos that has just been built.
		//
		CVector3D dstTan = dstEndRoadPos.GetTangent();

#if 0
		gout << "dst = " << dstEndRoadPos << endl;
		gout << "dstTan = " << dstTan << endl;
#endif

		CVector3D crossVec = srcTan.CrossP( dstTan );

#if 0
		gout << "crossVec = " << crossVec << endl;
		gout << "Road '" << dstRoad << "' has result = " << crossVec.m_k << endl;
#endif

		if ( !foundCounterClockwise ) {

			minCounterClockwise = crossVec.m_k;
			counterClockwiseLane = dstLane;
			foundCounterClockwise = true;

		}
		else {
			
			if ( crossVec.m_k < minCounterClockwise ) {

				minCounterClockwise = crossVec.m_k;
				counterClockwiseLane = dstLane;

			}

		}

	}

	if ( foundCounterClockwise )  nextLane = counterClockwiseLane;

	return foundCounterClockwise;

}  // end of GetNextLaneCounterClockwise


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetObjectsOnPoint
// 	Returns the CVED object ids of the objects that lie on the current
//  point, starting from the given road position.
//
// Remarks: The object ids are placed into the vector in order along the 
// 	current point.  Objects on all lanes/crdrs of the current point are 
// 	returned.
//
// Arguments:
//  startRoadPos - The starting road position.
// 	objects - (output) Contains the object ids of the objects on the point
// 		Note: The set is never cleared, so prior information is preserved.
//
// Returns: The number of objects on the current point.
//
//////////////////////////////////////////////////////////////////////////////
int
CPathPoint::GetObjectsOnPoint(
			const CRoadPos& startRoadPos,
			vector<int>& objects
			) const
{

	/*static*/ vector<int> tmpObjIds;
	//static bool        sInit = false;

	double startDist = startRoadPos.GetDistance();
	double endDist;

	//if ( sInit == false ) {
	//	sInit = true;
		tmpObjIds.reserve(20);
	//}

	if ( m_isRoad ) {

		// 
		// This path point lies on a road.
		//
		CLane lane = startRoadPos.GetLane();
		int laneId = lane.GetRelativeId();
		cvELnDir laneDir = lane.GetDirection();
			;
		bitset<cCV_MAX_LANES> laneMask((int) m_laneCrdrMask.to_ulong() );

		if ( laneDir == ePOS ) {

			endDist = startRoadPos.GetRoad().GetLinearLength();
			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						startDist, 
						m_endDist[laneId], 
						objects
						);
		}
		else {
			tmpObjIds.clear();
			endDist = 0.0;
			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						m_endDist[laneId], 
						startDist, 
						tmpObjIds
						);
		
			// Need to insert objects in reverse order to maintain order
			// 	along the point.
			vector<int>::reverse_iterator itr;
			for ( itr = tmpObjIds.rbegin(); itr < tmpObjIds.rend(); itr++ ) {

				objects.push_back( *itr );

			}

		}

	} // If the point lies on a road
	else {

		// 
		// This path point lies on a intersection.
		//
		CCrdr crdr = startRoadPos.GetCorridor();
		int crdrId = crdr.GetRelativeId();
		endDist    = crdr.GetLength();

		GetCved().GetAllDynObjsOnCrdrRange(
					m_intrsctn.GetId(), 
					crdrId, 
					startDist, 
					endDist, 
					objects
					);
	} // Else the point lies on an intersection

	return (int) objects.size();

} // GetObjectsOnPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetObjectsOnPoint
// 	Returns the CVED object ids of the objects that lie on the current
//  point, starting from the given road position.
//
// Remarks: The object ids are placed into the vector in order along the 
// 	current point.  Objects on specified lanes (for roads) or the crdr
//  (for intersections) of the current point are returned..
//
// Arguments:
//  startRoadPos - The starting road position.
//  laneId       - The lane on which look for objects.
// 	objects      - (output) Contains the object ids of the objects 
//                 on the point.
//  Note: The set is never cleared, so prior information is preserved.
//
// Returns: The number of objects on the current point.
//
//////////////////////////////////////////////////////////////////////////////
int
CPathPoint::GetObjectsOnPoint(
			const CRoadPos& startRoadPos,
			const int laneId,
			vector<int>& objects
			) const
{

	/*static*/ vector<int> tmpObjIds;
	//static bool sInit = false;
	double startDist = startRoadPos.GetDistance();
	double endDist;

	//if ( sInit == false ) {
	//	sInit = true;
		tmpObjIds.reserve(20);
	//}

	if ( m_isRoad ) {

		// 
		// This path point lies on a road.
		//
		CLane lane = startRoadPos.GetLane();
		int laneId = lane.GetRelativeId();
		cvELnDir laneDir = lane.GetDirection();

		bitset<cCV_MAX_LANES> laneMask;
		laneMask.set( laneId );

		if ( laneDir == ePOS ) {

			endDist = startRoadPos.GetRoad().GetLinearLength();
			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						startDist, 
						m_endDist[laneId], 
						objects
						);
		}
		else {
			endDist = 0.0;
			tmpObjIds.clear();
			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						m_endDist[laneId], 
						startDist, 
						tmpObjIds
						);
		
			// Need to insert objects in reverse order to maintain order
			// 	along the point.
			vector<int>::reverse_iterator itr;
			for ( itr = tmpObjIds.rbegin(); itr < tmpObjIds.rend(); itr++ ) {

				objects.push_back( *itr );

			}

		}

	} // If the point lies on a road
	else {

		// 
		// This path point lies on a intersection.
		//
		CCrdr crdr = startRoadPos.GetCorridor();
		int crdrId = crdr.GetRelativeId();
		endDist    = crdr.GetLength();

		GetCved().GetAllDynObjsOnCrdrRange(
					m_intrsctn.GetId(), 
					crdrId, 
					startDist, 
					endDist, 
					objects
					);
	} // Else the point lies on an intersection

	return (int) objects.size();

} // GetObjectsOnPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetObjectsOnPoint
// 	Returns the CVED object ids of the objects that lie on the current point
// 	and a bitset containing the lanes/crdrs occupied by objects.
//
// Remarks: The object ids are placed into the vector in order along the 
// 	current point.  Objects on all lanes/crdrs of the current point are 
// 	returned.
//
// Arguments:
// 	objects - (output) Contains the object ids of the objects on the point
// 		Note: The set is never cleared, so prior information is preserved.
// 	occupied - (output) Contains the identifiers of the lanes/crdrs that are
// 		occupied by objects on the current point.
//
// Returns: The number of objects on the current point.
//
//////////////////////////////////////////////////////////////////////////////
int
CPathPoint::GetObjectsOnPoint(
			bitset<cCV_MAX_CRDRS>& occupied, 
			vector<int>& objects
			) const
{

	/*static*/ vector<int> tmpObjIds;
	//static bool        sInit = false;

	//if ( sInit == false ) {
	//	sInit = true;
		tmpObjIds.reserve(20);
	//}

	if ( m_isRoad ) {

		// 
		// This path point lies on a road.
		//
		int laneId = GetFirstLaneCrdrIdx();
		bitset<cCV_MAX_LANES> laneMask((int) m_laneCrdrMask.to_ulong() );
		bitset<cCV_MAX_LANES> occupiedLanes;

		if ( m_startDist[laneId] < m_endDist[laneId] ) {

			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						m_startDist[laneId], 
						m_endDist[laneId], 
						objects,
						occupiedLanes
						);
		}
		else {
			tmpObjIds.clear();
			GetCved().GetAllDynObjsOnRoadRange(
						m_road.GetId(), 
						laneMask, 
						m_endDist[laneId], 
						m_startDist[laneId], 
						tmpObjIds, 
						occupiedLanes
						);
		
			// 
			// Need to insert objects in reverse order to maintain order
			// along the point.
			//
			vector<int>::reverse_iterator itr;
			for ( itr = tmpObjIds.rbegin(); itr < tmpObjIds.rend(); itr++ ) {

				objects.push_back( *itr );

			}

		}

		// Set occupied bitset
		occupied = bitset<cCV_MAX_CRDRS>( (int) occupiedLanes.to_ulong() );

	} // If the point lies on a road
	else {

		// 
		// This path point lies on a intersection.
		//
		int crdrId = GetFirstLaneCrdrIdx();

		if ( m_startDist[crdrId] < m_endDist[crdrId] ) {

			GetCved().GetAllDynObjsOnIntrsctnRange(
						m_intrsctn.GetId(), 
						m_laneCrdrMask, 
						m_startDist, 
						m_endDist, 
						objects,
						occupied
						);
		}
		else {
			tmpObjIds.clear();
			GetCved().GetAllDynObjsOnIntrsctnRange(
						m_intrsctn.GetId(), 
						m_laneCrdrMask, 
						m_endDist, 
						m_startDist, 
						tmpObjIds, 
						occupied
						);
		
			// Need to insert objects in reverse order to maintain order
			// 	along the point.
			vector<int>::reverse_iterator itr;
			for ( itr = tmpObjIds.rbegin(); itr < tmpObjIds.rend(); itr++ ) {

				objects.push_back( *itr );

			}

		}

	} // Else the point lies on an intersection

	return (int) objects.size();

} // GetObjectsOnPoint


//////////////////////////////////////////////////////////////////////////////
//		Inherited functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsValid
// 	Returns whether the current CPathPoint instance is valid
//
// Remarks: A CPathPoint is considered valid if it was created with a valid
// 	CCved instance, it contains a valid road or intersection, and has at 
// 	least 1 lane or corridor.
//
// Arguments:
//
// Returns: true if valid, false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::IsValid() const
{
	if (CCvedItem::IsValid()) {
		if (m_isRoad)	return (m_road.IsValid() && m_laneCrdrMask.any());
		else			return (m_intrsctn.IsValid() && m_laneCrdrMask.any());
	}
	return false;
} // end of IsValid

//////////////////////////////////////////////////////////////////////////////
//
// Description: AssertValid
// 	Asserts whether the current CPathPoint instance is valid
//
// Remarks: A CPathPoint is considered valid if it was created with a valid
// 	CCved instance, it contains a valid road or intersection, and has at 
// 	least 1 lane or corridor.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPathPoint::AssertValid() const
{
	CCvedItem::AssertValid();

	if (m_isRoad) 
		assert((m_road.IsValid()) && (m_laneCrdrMask.any()));
	else 
		assert((m_intrsctn.IsValid()) && (m_laneCrdrMask.any()));
} // end of AssertValid 


//////////////////////////////////////////////////////////////////////////////
//	Private helper functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: (private) Takes the information in the CRoadPos 
//   parameter and initializes the local variables of the current 
//   instance.
//
// Remarks: If the given CRoadPos lies on a road, then the local data 
//   is initialized to all the lanes that run in the same direction as 
//   the lane stored in the CRoadPos.  If the given CRoadPos lies on 
//   an intersection, then the local data is initialized to all 
//   corridors running from the source road to the destination road of 
//   the first corridor that the CRoadPos lies on.  Distance info is 
//   also taken from CRoadPos.
//
// Arguments:
// 	 cRoadPos - A valid CRoadPos instance.
//
// Returns: True if the resulting CPathPoint is valid, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::Initialize( const CRoadPos& cRoadPos ) 
{
	if( !cRoadPos.IsValid() ) return false;

	m_isRoad = cRoadPos.IsRoad();

	// If CRoadPos lies on a road
	if( m_isRoad ) 
	{
		// Get current road
		m_road = cRoadPos.GetRoad();

		// Get all lanes in the correct direction
		cvTLane* pLane = BindLane( m_road.GetLaneIdx() );
		cvELnDir dir   = cRoadPos.GetLane().GetDirection();
		
		int numLanes = m_road.GetNumLanes();
		double dist  = cRoadPos.GetDistance();

		// For each lane on the road
		int laneId;
		for( laneId = 0; laneId < numLanes; laneId++, pLane++ )
		{
			//
			// If the lane's direction matches the parameter, 
			// then add it to the list of lanes.
			//
			if( pLane->direction == dir )
			{
				m_laneCrdrMask.set( laneId );
				m_startDist[laneId] = dist;
				m_endDist[laneId] = dist;
			}
		}

	} // If CRoadPos lies on a road

	// If CRoadPos lies on an intersection
	else 
	{
		// 
		// Get current intersection.
		//
		m_intrsctn = cRoadPos.GetIntrsctn();
		int numCrdrs = m_intrsctn.GetNumCrdrs();
		
		// 
		// Get all corridors in the direction of all corridors in cRoadPos.
		//
		typedef pair<int, int> srcDstRdPair;
		srcDstRdPair srcDst;

		//
		// Set up the crdrMap: a map of pairs of srcRdIdx and dstRdIdx and
		// a t-value for the distance along the corridor.
		//
		map<srcDstRdPair, double> crdrMap;
		int    crdrId;
		TCrdr* pCrdr;
		for(
			crdrId = 0, pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() ); 
			crdrId < numCrdrs; 
			crdrId++, pCrdr++
			)
		{
			srcDst.first = pCrdr->srcRdIdx;
			srcDst.second = pCrdr->dstRdIdx;
		
			CCrdr crdr = cRoadPos.GetCorridor( crdrId );

			// If the corridor is contained in cRoadPos
			if( crdr.IsValid() )
			{
				// Add an item to the map for this corridor
				pair<srcDstRdPair, double> mapItem;
				mapItem.first = srcDst;
				mapItem.second = (
							cRoadPos.GetDistance( crdrId ) / crdr.GetLength()
							);

				crdrMap.insert( mapItem );
			}
		} // For each corridor on the intersection

		//
		// Get the destination lane's direction of the given cRoadPos.
		//
		cvTLane* pInputDstLane = BindLane( cRoadPos.GetCorridor().GetDstntnLnIdx() );
		cvELnDir inputDstLaneDirection = pInputDstLane->direction;

		//
		// Add all the corridors that are in the cRoadPos or parallel
		// to a corridor in cRoadPos
		//
		// For roads that loop--i.e. the corridors at this intersection
		// go to both positive and negative direction lanes on the SAME
		// road, make sure that either only corridors going to the
		// positive OR negative lanes are inserted, but not both.
		//
		map<srcDstRdPair, double>::iterator mapItr;

		TCrdrPnt* pCntrlPntPool = BindCrdrPnt( 0 );
		TCrdrPnt* pCntrlPnt;
		bool haveRoad = false;
		int roadIdx;
		cvELnDir laneDir = eNONE;
		for(
			crdrId = 0, pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() ); 
			crdrId < numCrdrs; 
			crdrId++, pCrdr++
			)
		{
			srcDst.first = pCrdr->srcRdIdx;
			srcDst.second = pCrdr->dstRdIdx;
			mapItr = crdrMap.find( srcDst );

			//
			// If the corridor is in the direction of one
			// of the previously entered corridors
			//
			if( mapItr != crdrMap.end() )
			{
				cvTLane* pDstLane = BindLane( pCrdr->dstLnIdx );
				srcDstRdPair origSrcDst = mapItr->first;
				bool success = (
						origSrcDst.first == pCrdr->srcRdIdx &&
						pDstLane->direction == inputDstLaneDirection
						);
				if( success )
				{
					if( !haveRoad )
					{
						haveRoad = true;
						roadIdx = pCrdr->dstRdIdx;
						laneDir = pDstLane->direction;
					}
					else
					{
						if( roadIdx == pCrdr->dstRdIdx )
						{
							if( laneDir != pDstLane->direction )  continue;
						}
					}

					int cntrlPntIdx = pCrdr->cntrlPntIdx +	pCrdr->numCntrlPnt - 1;
					pCntrlPnt = &pCntrlPntPool[cntrlPntIdx];
					
					// Insert this corridor into the current CPathPoint
					m_laneCrdrMask.set( crdrId );
					m_startDist[crdrId] = mapItr->second * pCntrlPnt->distance;
					m_endDist[crdrId] = m_startDist[crdrId];
				}
			}
		} // For each corridor on the intersection
	} // If CRoadPos lies on an intersection

	return IsValid();
} // end of Initialize

//////////////////////////////////////////////////////////////////////////////
//
// Description: RecursiveAppend (private)
// 	This function recursively tries to connect the current CPathPoint to the
// 	parameter.  If the current CPathPoint lies on the same road/intersection
// 	and is headed in the same direction of the parameter, then they are
// 	connected.  Otherwise, the function looks ahead up to 2 intersections 
// 	ahead of the first CPathPoint.  As it recurs, it adds CPathPoints to the
// 	extension list.
//
// Remarks: This function is called by the Append(CPathPoint) function.
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
//	point - a CPathPoint to which the current CPathPoint should be connected
//	extension - If the function returns true, this contains all the 
//		CPathPoints, ending with the current one, that were used to connect
//		the first CPathPoint to the point parameter.
//	numIntrsctnsHit - An integer value that contains the number of 
//		intersections that have been traversed up to this point.  This 
//		function will search no more than 2.
//		 
// Returns: True if the current CPathPoint can be connected to the parameter
// 	in 2 intersections or less, false otherwise. 
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::RecursiveAppend(
			const CPathPoint& point, 
			const CPathPoint* cpOrigPoint, 
			CFastDeque<CPathPoint>& extension, 
			int numIntrsctnsHit, 
			int srcRoadIdx
			) 
{
	//
	// BASE CASE
	//

	// Is this directly connected to point?
	if( CheckAndMakeSamePoint( point ) ) 
	{
		// Add the current point to the extension list, if it is not 
		// the same point as the original.  The original should never 
		// be added to the list.
		if( this != cpOrigPoint )
		{
			extension.push_back( *this );
		}

		// A matching point was found, so return true.
		return true;
	}
	
	//
	// RECURSIVE CASE (with depth boundary)
	//

	// This not directly connected to point
	else if( numIntrsctnsHit < 2 )  
	{
		// Extend the current CPathPoint to the end of the
		// current road or corridor.
		SetRangeToEnd( false, true );

		// If this lies on an intersection, so increment numIntrsctnsHit.
		if( !m_isRoad )
		{
			if( !m_intrsctn.IsTwoRoad() )  numIntrsctnsHit++;
		}
		// Otherwise, store the current road id for later use with 
		// GetNextPathPoints
		else
		{
			srcRoadIdx = m_road.GetId();
		}

		//
		// Making a static vector for efficiency's sake.
		//
		vector<CPathPoint> nextPoints;
		GetNextPathPoints( nextPoints, srcRoadIdx );
		// For each connecting point
		vector<CPathPoint>::iterator itr;
		for( itr = nextPoints.begin(); itr != nextPoints.end(); itr++ ) 
		{
			// Run RecursiveAppend on the path point.  
			if( itr->RecursiveAppend(point, 
									 cpOrigPoint,
									 extension, 
									 numIntrsctnsHit, 
									 srcRoadIdx
									 )) 
			{
				// Add the current point to the extension list, if it is 
				// not the same point as the original.  The original 
				// should never be added to the list.
				if( this != cpOrigPoint )
				{
					extension.push_front( *this );
				}

				// Use the first connected path found.
				return true;
			}

		} // For each connecting point

	} // This not directly connected to point

	// No connecting points were found.
	return false;

} // end of RecursiveAppend

//////////////////////////////////////////////////////////////////////////////
//
// Description: RecursivePrepend (private)
// 	This function recursively tries to connect the current CPathPoint to the
// 	parameter.  If the current CPathPoint lies on the same road/intersection
// 	and is headed in the same direction of the parameter, then they are
// 	connected.  Otherwise, the function looks ahead up to 2 intersections 
// 	behind the first CPathPoint.  As it recurs, it adds CPathPoints to the
// 	extension list.
//
// Remarks: This function is called by the Prepend(CPathPoint) function.
// 	This function will cause a failed assertion if it is called
// 	on an invalid CPathPoint instance
//
// Arguments:
//	point - a CPathPoint to which the current CPathPoint should be connected
//	extension - If the function returns true, this contains all the 
//		CPathPoints, ending with the current one, that were used to connect
//		the first CPathPoint to the point parameter.
//	numIntrsctnsHit - An integer value that contains the number of 
//		intersections that have been traversed up to this point.  This 
//		function will search no more than 2.
//		 
// Returns: True if the current CPathPoint can be connected to the parameter
// 	in 2 intersections or less, false otherwise. 
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::RecursivePrepend(
			const CPathPoint& point, 
			const CPathPoint* cpOrigPoint, 
			CFastDeque<CPathPoint>& extension, 
			int numIntrsctnsHit, 
			int dstRoadIdx
			) 
{
	//
	// BASE CASE
	//

	// Is this directly connected to point?
	if( CheckAndMakeSamePoint( point, false ) ) 
	{
		// Add the current point to the extension list, 
		//	if it is not the same point as the original.
		//	The original should never be added to the list.
		if( this != cpOrigPoint )  extension.push_front( *this );

		// A matching point was found, so return true.
		return true;
	}
	
	//
	// RECURSIVE CASE (with depth boundary)
	//

	// This not directly connected to point
	else if( numIntrsctnsHit < 2 ) 
	{
		// Extend the current CPathPoint to the end of the
		//	current road or corridor.
		SetRangeToStart( true, false );

		// If this lies on an intersection, 
		//	so increment numIntrsctnsHit.
		if( !m_isRoad )
		{
			// ignore two-road intersections when checking for depth boundary
			if( !m_intrsctn.IsTwoRoad() )  numIntrsctnsHit++;
		}
		// Otherwise, store the current road id for later use with GetNextPathPoints
		else
		{
			dstRoadIdx = m_road.GetId();
		}

		vector<CPathPoint> prevPoints;
		GetPrevPathPoints( prevPoints, dstRoadIdx );

		// For each connecting point
		vector<CPathPoint>::reverse_iterator itr;
		for( itr = prevPoints.rbegin(); itr != prevPoints.rend(); itr++ ) 
		{
			// Run RecursivePrepend on the path point.  
			if (itr->RecursivePrepend(point, 
									  cpOrigPoint,
									  extension, 
									  numIntrsctnsHit, 
									  dstRoadIdx)) 
			{
				// Add the current point to the extension list, 
				//	if it is not the same point as the original.
				//	The original should never be added to the list.
				if( this != cpOrigPoint )  extension.push_back( *this );

				// Use the first connected path found.
				return true;
			}
		} // For each connecting point
	} // This not directly connected to point

	// No connecting points were found.
	return false;

} // end of RecursivePrepend

//////////////////////////////////////////////////////////////////////////////
//
// Description: CheckAndMakeSamePoint (private)
// 	Checks the current CPathPoint against the parameter to see if they lie on
// 	the same road/intersection and have lanes/corridors headed in the same
// 	direction.  If they do, then the current CPathPoint is altered to include
// 	distance range of the paramter.
//
// Remarks: This function both compares and alters the current CPathPoint, so
// 	it is both an Accessor and a Mutator function.
//
// 	This function will cause a failed assertion if3Y it is called
// 	on an invalid CPathPoint instance
//
// Arguments: 
// 	point - another CPathPoint with which to compare the current one
//	changeEnd - (Optional) If false, then the endDist of the current CPathPoint 
//		will be change, otherwise, the startDist will be changed.  This is
//		here so that both Append and Prepend can use this function.  The default
//		value is true.
//
// Returns: True if the current CPathPoint lies on the same road/intersection
// 	and in the same direction as the point parameter.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::CheckAndMakeSamePoint( const CPathPoint& point, bool changeEnd )
{
	// If this and point lie on a road
	if (m_isRoad && point.m_isRoad) {

#if PATH_POINT_DEBUG
		gout << "Both on a road." << endl;
		gout << "  current: " << m_road.GetName() << endl;
		gout << "  param  : " << point.m_road.GetName() << endl;
#endif

		// If this lies on same road as point
		if (m_road.GetId() == point.m_road.GetId()) {

			// If this lies on lanes in the same direction as point
			CLane thisLane(m_road, GetFirstLaneCrdrIdx());
			CLane pointLane(m_road, point.GetFirstLaneCrdrIdx());

#if PATH_POINT_DEBUG
			gout << "  Both on same road." << endl;
			gout << "    current: " << thisLane.GetDirection() << endl;
			gout << "    param  : " << pointLane.GetDirection() << endl;
#endif

			if (thisLane.GetDirection() == pointLane.GetDirection()) {

#if PATH_POINT_DEBUG
				gout << "    Both in same direction." << endl;
#endif
				// This and the point are essentially the same, this
				// 	just needs its distance range extended.

				// Set endDist or startDist of each lane in use to the 
				// 	end or start distance of the first lane in point.  
				// 	That way, the endDist or startDist for each lane is 
				// 	uniform.
				for (int id = 0; id < cCV_MAX_CRDRS; id++) {
					if (point.m_laneCrdrMask.test(id)) {
						if (changeEnd)
							m_endDist[id] = point.m_endDist[id];
						else
							m_startDist[id] = point.m_startDist[id];
					}
				}
				return true;
			}				
		}
	} // If this and point lie on a road

	// If current point and point lie on an intersection
	else if (!m_isRoad && !point.m_isRoad) {

#if PATH_POINT_DEBUG
		gout << "Both on an isec." << endl;
		gout << "  current: " << m_intrsctn.GetName() << endl;
		gout << "  param  : " << point.m_intrsctn.GetName() << endl;
#endif

		// If this lies on same intersection as point
		if (m_intrsctn.GetId() == point.m_intrsctn.GetId()) {

			// If this lies on some of the same corridors as point
			bitset<cCV_MAX_CRDRS> overlap = m_laneCrdrMask;
			overlap &= point.m_laneCrdrMask;
			if (overlap.any()) {

#if PATH_POINT_DEBUG
				gout << "Both on similar corridors." << endl;
#endif
				// This and the point are essentially the same, this
				// 	just needs its distance range extended.

				// Set endDist or startDist of each corridor in use to the 
				// 	end or start distance of the corridor in point.  
				// 	That way, the endDist or startDist for each lane is 
				// 	uniform.
				m_laneCrdrMask = overlap;
				for (int id = 0; id < cCV_MAX_CRDRS; id++) {
					if (m_laneCrdrMask.test(id)) {
						if (changeEnd)
							m_endDist[id] = point.m_endDist[id];
						else
							m_startDist[id] = point.m_startDist[id];
					}
				}
				return true;

			} // If this lies on some of the same corridors as point
		} // If this lies on same intrsctn as point
	} // If this and parameter lie on an intersection

#if PATH_POINT_DEBUG
	if (m_isRoad) gout << "Curr on road, param on isec." << endl;
	else gout << "Param on road, curr on isec." << endl;
#endif
	
	return false;
} // end of CheckAndMakeSamePoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get the next CPathPoint(s) ahead of the current CPathPoint.
//
// Remarks: If the current CPathPoint lies on a road, then 1 CPathPoint for
//   each road that can be reached from the current road through the next
//   intersection is added to the vector.  If the current CPathPoint lies on
//   an intersection, then 1 CPathPoint will be added to the vector 
//   representing the destination road and lane(s) of the current corridors.  
//
// Arguments:
//   nextPoints - A vector containing all the immediate next CPathPoints.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPathPoint::GetNextPathPoints(
			vector<CPathPoint>& nextPoints, 
			int srcRoadIdx
			) const
{
	// Just to be sure.
	AssertValid();

	// If this is on a road
	if( m_isRoad ) 
	{
		//
		// Create a CPathPoint for each road that can be reached
		// from the current road through the next intersection.
		//
		CPathPoint isecPoint( GetCved() );
		isecPoint.m_isRoad = false;
		isecPoint.m_intrsctn = GetNextIntrsctn();
		TCrdr* pCrdr = BindCrdr( isecPoint.m_intrsctn.GetCrdrIdx() );
		int crdrId;
		for( 
			crdrId = 0; 
			crdrId < isecPoint.m_intrsctn.GetNumCrdrs(); 
			crdrId++, pCrdr++ ) 
		{
			if( pCrdr->srcRdIdx == m_road.GetId() )
			{
				isecPoint.m_laneCrdrMask.set( crdrId );
			}
		}
		if( isecPoint.IsValid() ) 
		{
			isecPoint.SetRangeToStart( true, true );
			nextPoints.push_back( isecPoint );
		}
	} // If this is on a road

	// Else this is on an intersection
	else 
	{
		// Create a CPathPoint that for each connecting road.
		set<int> dstRoadIds;
		int crdrId = GetFirstLaneCrdrIdx();
		int numCrdrs = m_intrsctn.GetNumCrdrs();
		TCrdr* pCrdr = BindCrdr( m_intrsctn.GetCrdrIdx() + crdrId );
		for(; crdrId < numCrdrs; crdrId++, pCrdr++) 
		{
			// Look at only the corridors that in the current path point.
			if( m_laneCrdrMask.test( crdrId ) ) 
			{
				// If a source road was given, and the source road of 
				// the corridor matches it, then check the destination 
				// road.
				bool srcRoadMatches = (
							srcRoadIdx < 0 ||
							pCrdr->srcRdIdx == srcRoadIdx
							);
				if( srcRoadMatches ) 
				{
					bool dstRoadNotInSet = 
						dstRoadIds.find( pCrdr->dstRdIdx ) == dstRoadIds.end();
					if( dstRoadNotInSet )
					{
						CPathPoint roadPoint( GetCved() );
						CCrdr crdr( GetCved(), pCrdr );
						if( roadPoint.SetRoadPoint( crdr ) )
						{
							roadPoint.SetRangeToStart( true, true );
							nextPoints.push_back( roadPoint );
						}
						dstRoadIds.insert( pCrdr->dstRdIdx );
					}
				}
			}
		}
		
	} // Else this is on an intersection

} // end of GetNextPathPoints

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPrevPathPoints
//	Get the prev CPathPoint(s) ahead of the current CPathPoint.
//
// Remarks: If the current CPathPoint lies on a road, then 1 CPathPoint for
//	each road that can be reached from the current road through the prev
//	intersection is added to the vector.  If the current CPathPoint lies on
//	an intersection, then 1 CPathPoint will be added to the vector 
//	representing the source road and lane(s) of the current corridors.  
//
// Arguments:
//	nextPoints - a vector containing all the immediate prev CPathPoints.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPathPoint::GetPrevPathPoints(vector<CPathPoint>& prevPoints, 
							  int dstRoadIdx) const
{
	// Just to be sure.
	AssertValid();

	// If this is on a road
	if (m_isRoad) {

		CPathPoint isecPoint(GetCved());
		isecPoint.m_isRoad = false;
		isecPoint.m_intrsctn = GetPrevIntrsctn();
		
		int crdrId;
		TCrdr* pCrdr = BindCrdr(isecPoint.m_intrsctn.GetCrdrIdx());
		
		// For each corridor in the previous intersection
		for (crdrId = 0; crdrId < isecPoint.m_intrsctn.GetNumCrdrs();
			 crdrId++, pCrdr++) {

			// If the corridor terminates at the current road, 
			// 	add it to the mask.
			if (pCrdr->dstRdIdx == m_road.GetId()) 
				isecPoint.m_laneCrdrMask.set(crdrId);
			
		} // For each corridor in the previous intersection
		
		// If the resulting CPathPoint is valid, add it to the list.
		if (isecPoint.IsValid()) {
			// Make the current corridors begin and end at the
			// 	end of their corridors.
			isecPoint.SetRangeToEnd(true, true);
			prevPoints.push_back(isecPoint);
		}

	} // If this is on a road

	// Else this is on an intersection
	else {

		// Create a CPathPoint that for each connecting road.
		set<int> srcRoadIds;
		int crdrId = GetFirstLaneCrdrIdx();
		TCrdr* pCrdr = BindCrdr(m_intrsctn.GetCrdrIdx()+crdrId);
		for (; crdrId < m_intrsctn.GetNumCrdrs(); crdrId++, pCrdr++) {

			// If a destination road was given, and the dest. road
			//	of the corridor matches it, then check the source road
			if( m_laneCrdrMask.test( crdrId ) )
			{
				if ( (dstRoadIdx < 0) ||
					 (pCrdr->dstRdIdx == dstRoadIdx) ) {

					if (srcRoadIds.find(pCrdr->srcRdIdx) == srcRoadIds.end()) {

						CPathPoint roadPoint(GetCved());
						CCrdr crdr(GetCved(), pCrdr);
						if (roadPoint.SetRoadPoint(crdr, true)) {
							roadPoint.SetRangeToEnd(true, true);
							prevPoints.push_back(roadPoint);
						}
						srcRoadIds.insert(pCrdr->srcRdIdx);
					}
				}
			}
		}
		
	} // Else this is on an intersection

} // end of GetPrevPathPoints

//////////////////////////////////////////////////////////////////////////////
//
// Desciption: SetRangeToStart (private)
//	Sets the specified endpoints of the distance range to the start of the 
//	lane or corridor.
//
// Remarks: If the current CPathPoint lies on a lane, then the "start" of the
//	road is determined by the direction of the lane.  If the lane is ePOS, then
//	the start is 0.0.  Otherwise, the start is the length of the road.  The 
//	start of any corridor is 0.0.
//
// Arguments:
//	start - if true, then set m_startDist
//	end - if true, then set m_endDist
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPathPoint::SetRangeToStart(bool start, bool end) 
{
	if (m_isRoad) {

		cvTLane* pLanePool = BindLane(m_road.GetLaneIdx());
		double length = m_road.GetLinearLength();

		for (int id = 0; id < cCV_MAX_CRDRS; id++) {

			if (m_laneCrdrMask.test(id)) {

				if (pLanePool[id].direction == ePOS) {
					if (start) m_startDist[id] = 0.0f;
					if (end) m_endDist[id] = 0.0f;
				}
				else {
					if (start) m_startDist[id] = length;
					if (end) m_endDist[id] = length;
				}

			}	
		}
	}
	else {
		cvTCrdr* pCrdrPool = BindCrdr(m_intrsctn.GetCrdrIdx());

		for (int id = 0; id < cCV_MAX_CRDRS; id++) {
			if (m_laneCrdrMask.test(id)) {

				if (start) m_startDist[id] = 0.0f;
				if (end) m_endDist[id] = 0.0f;
			}
		}
	}

} // end of SetRangeToStart

//////////////////////////////////////////////////////////////////////////////
//
// Desciption: SetRangeToEnd (private)
//	Sets the specified endpoints of the distance range to the end of the 
//	lane or corridor.
//
// Remarks: If the current CPathPoint lies on a lane, then the "end" of the
//	road is determined by the direction of the lane.  If the lane is ePOS, then
//	the end is the length of the road.  Otherwise, the end is 0.0.  The 
//	end of any corridor is the length of that corridor.
//
// Arguments:
//	start - if true, then set m_startDist
//	end - if true, then set m_endDist
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CPathPoint::SetRangeToEnd(bool start, bool end) 
{
	if (m_isRoad) {

		cvTLane* pLanePool = BindLane(m_road.GetLaneIdx());
		double length = m_road.GetLinearLength();

		for (int id = 0; id < m_road.GetNumLanes(); id++) {

			if (m_laneCrdrMask.test(id)) {

				if (pLanePool[id].direction == ePOS) {
					if (start) m_startDist[id] = length;
					if (end) m_endDist[id] = length;
				}
				else {
					if (start) m_startDist[id] = 0.0f;
					if (end) m_endDist[id] = 0.0f;
				}

			}	
		}
	}
	else {
		cvTCrdr* pCrdrPool = BindCrdr(m_intrsctn.GetCrdrIdx());

		for (int id = 0; id < m_intrsctn.GetNumCrdrs(); id++) {
			if (m_laneCrdrMask.test(id)) {

				CCrdr crdr(GetCved(), &pCrdrPool[id]);
				double length = crdr.GetLength();

				if (start) m_startDist[id] = length;
				if (end) m_endDist[id] = length;
			}
		}
	}

} // end of SetRangeToEnd

//////////////////////////////////////////////////////////////////////////////
//
// Description: (private) Initializes the current instance with the 
//   given intersection and all corridors running from the source 
//   road, and in the same direction as the source lane,  to the 
//   destination road.
//
// Remarks: The start and end distance of the returned point are 0.
//	 The CPathPoint instance must have been created with a CCved 
//   instance, or a failed assertion will occur.
//
// Arguments: 
//	 cIntrsctn  - A valid CIntrsctn to use.
//	 cSrcRoad   - A valid CRoad which is the source of the corridors 
//       in pathPoint.
//   srcLaneIdx - The index of the lane associated with the src road.
//	 cDstRoad   - A valid CRoad which is the destination of the corridors
//		 in pathPoint.
//
// Returns: True if the resulting CPathPoint is valid, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::SetIntrsctnPoint(
			const CIntrsctn& cIntrsctn, 
			const CRoad& cSrcRoad,
			int srcLaneIdx,
			const CRoad& cDstRoad
			)
{
	CCvedItem::AssertValid();
	TCrdr* pCrdr = BindCrdr( cIntrsctn.GetCrdrIdx() );
	int crdrId;
	for( crdrId = 0; crdrId < cIntrsctn.GetNumCrdrs(); crdrId++, pCrdr++ )
	{
		//
		// Find a corridor that connects source road and lane to the
		// destination road.
		//
		bool match = (
				pCrdr->srcRdIdx == cSrcRoad.GetId() && 
				pCrdr->dstRdIdx == cDstRoad.GetId()
				);
;
		bool sameSrcDstRoad = match && pCrdr->srcRdIdx == pCrdr->dstRdIdx;
		if( sameSrcDstRoad )
		{
			//
			// The source and the destination roads are the same road.
			// Pick a corridor that has a source lane which moves in the
			// same direction as the given lane.
			//
			cvTLane* pLane = BindLane( cSrcRoad.GetLaneIdx() + srcLaneIdx );
			cvTLane* pCrdrSrcLane = BindLane( pCrdr->srcLnIdx );
			match = pLane->direction == pCrdrSrcLane->direction;
		}

		if( match ) 
		{
			CRoadPos roadPos( cIntrsctn, crdrId );
			return ( Initialize( roadPos ) );
		}
	}

	return false;
} // end of SetIntrsctnPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: (private) Initializes the current instance with the 
//   given intersection and all corridors running from the source 
//   road to the destination road and the lanes running in the same
//   direction as the given destination lane.
//
// Remarks: The start and end distance of the returned point are 0.
//	 The CPathPoint instance must have been created with a CCved 
//   instance, or a failed assertion will occur.
//
// Arguments: 
//	 cIntrsctn  - A valid CIntrsctn to use.
//	 cSrcRoad   - A valid CRoad which is the source of the corridors 
//       in pathPoint.
//	 cDstRoad   - A valid CRoad which is the destination of the corridors
//		 in pathPoint.
//   dstLaneIdx - The index of the lane associated with the dst road.
//
// Returns: True if the resulting CPathPoint is valid, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::SetIntrsctnPoint(
			const CIntrsctn& cIntrsctn, 
			const CRoad& cSrcRoad,
			const CRoad& cDstRoad,
			int dstLaneIdx
			)
{
	CCvedItem::AssertValid();
	TCrdr* pCrdr = BindCrdr( cIntrsctn.GetCrdrIdx() );
	int crdrId;
	for( crdrId = 0; crdrId < cIntrsctn.GetNumCrdrs(); crdrId++, pCrdr++ )
	{
		//
		// Find a corridor that connects source road and lane to the
		// destination road.
		//
		bool match = (
				pCrdr->srcRdIdx == cSrcRoad.GetId() && 
				pCrdr->dstRdIdx == cDstRoad.GetId()
				);
;
		bool sameSrcDstRoad = pCrdr->srcRdIdx == pCrdr->dstRdIdx;
		if( sameSrcDstRoad )
		{
			//
			// The source and the destination roads are the same road.
			// Pick a corridor that has a destination lane which moves in the
			// same direction as the given lane.
			//
			cvTLane* pLane = BindLane( cDstRoad.GetLaneIdx() + dstLaneIdx );
			cvTLane* pCrdrDstLane = BindLane( pCrdr->dstLnIdx );
			match = pLane->direction == pCrdrDstLane->direction;
		}

		if( match ) 
		{
			CRoadPos roadPos( cIntrsctn, crdrId );
			return ( Initialize( roadPos ) );
		}
	}

	return false;
} // end of SetIntrsctnPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: (private) Initializes the current instance with the 
//   given the source or destination road and lanes of the corridor 
//   parameter.
//
// Remarks: The start and end distance of the returned point are 0.
//	 The CPathPoint instance must have been created with a CCved 
//   instance, or a failed assertion will occur.
//
// Arguments: 
//	cCrdr  - Contains the source/destination road and lane to use.
//	source - (Optional) If true, then use the source road/lane of the 
//		     the corridor.  Otherwise, use the destination.  Default 
//           value is false.
//
// Returns: True if the resulting CPathPoint is valid, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CPathPoint::SetRoadPoint( const CCrdr& cCrdr, bool source )
{
	CCvedItem::AssertValid();
	CRoadPos roadPos( GetCved() );

	if( source )
	{
		roadPos.SetLane( cCrdr.GetSrcLn() );
	}
	else
	{
		roadPos.SetLane( cCrdr.GetDstntnLn() );
	}

	return Initialize( roadPos );

} // end of SetRoadPoint

//////////////////////////////////////////////////////////////////////////////
//
// Description: Calculates the angle between the source road and the 
//   destination road.  
//
// Remarks: The angle between the roads is determined solely by the 2 
// 	 control points nearest to the intersection on both roads. 
//
// Arguments: 
//	 cIntrsctn - A CIntrsctn that connects the two roads.
//	 cSrcRoad  - The source road.
//	 cDstRoad  - The destination road.
//
// Returns: A double containing the clockwise angle between srcRoad and dstRoad.
//
//////////////////////////////////////////////////////////////////////////////
double
CPathPoint::GetAngleBetween(
			const CIntrsctn& cIntrsctn, 
			const CRoad& cSrcRoad, 
			const CRoad& cDstRoad
			) const
{
	CVector3D srcVec, dstVec;

	TRoad* pSrcRd = BindRoad( cSrcRoad.GetId() );
	TRoad* pDstRd = BindRoad( cDstRoad.GetId() );

	TCntrlPnt* pCntrlPntPool = BindCntrlPnt( 0 );

	// If the source road heads away from the current intersection
	if( cIntrsctn.GetId() == pSrcRd->srcIntrsctnIdx )
	{
		// srcVec = 2nd cntrl pnt - 1st cntrl pnt
		srcVec.m_i = (
			pCntrlPntPool[pSrcRd->cntrlPntIdx].location.x -
			pCntrlPntPool[pSrcRd->cntrlPntIdx + 1].location.x
			);
		srcVec.m_j = (
			pCntrlPntPool[pSrcRd->cntrlPntIdx].location.y -
			pCntrlPntPool[pSrcRd->cntrlPntIdx + 1].location.y
			);
		srcVec.m_k = 0.0;
	}
	// Else if the source road heads towards the current intersection
	else if( cIntrsctn.GetId() == pSrcRd->dstIntrsctnIdx )
	{
		// srcVec = n-2 cntrl pnt - n-1 cntrl pnt
		srcVec.m_i = (
		   pCntrlPntPool[pSrcRd->cntrlPntIdx+pSrcRd->numCntrlPnt-1].location.x -
		   pCntrlPntPool[pSrcRd->cntrlPntIdx+pSrcRd->numCntrlPnt-2].location.x
		   );
		srcVec.m_j = (
		   pCntrlPntPool[pSrcRd->cntrlPntIdx+pSrcRd->numCntrlPnt-1].location.y -
		   pCntrlPntPool[pSrcRd->cntrlPntIdx+pSrcRd->numCntrlPnt-2].location.y
		   );
		srcVec.m_k = 0.0;
	}
	else 
	{
		// Error in the LRI file! 
		assert(0);
	}

	// If the destination road heads away from the current intersection
	if( cIntrsctn.GetId() == pDstRd->srcIntrsctnIdx )
	{
		// dstVec = 2nd cntrl pnt - 1st cntrl pnt
		dstVec.m_i = (
			pCntrlPntPool[pDstRd->cntrlPntIdx+1].location.x -
			pCntrlPntPool[pDstRd->cntrlPntIdx].location.x
			);
		dstVec.m_j = (
			pCntrlPntPool[pDstRd->cntrlPntIdx+1].location.y -
			pCntrlPntPool[pDstRd->cntrlPntIdx].location.y
			);
		dstVec.m_k = 0.0;
	}
	// Else if the destination road heads towards the current intersection
	else if( cIntrsctn.GetId() == pDstRd->dstIntrsctnIdx )
	{
		// dstVec = n-2 cntrl pnt - n-1 cntrl pnt
		dstVec.m_i = (
		   pCntrlPntPool[pDstRd->cntrlPntIdx+pDstRd->numCntrlPnt-2].location.x -
		   pCntrlPntPool[pDstRd->cntrlPntIdx+pDstRd->numCntrlPnt-1].location.x
		   );
		dstVec.m_j = (
		   pCntrlPntPool[pDstRd->cntrlPntIdx+pDstRd->numCntrlPnt-2].location.y -
		   pCntrlPntPool[pDstRd->cntrlPntIdx+pDstRd->numCntrlPnt-1].location.y
		   );
		dstVec.m_k = 0.0;
	}
	else 
	{
		// Error in the LRI file! 
		assert(0);
	}

	// 0. Normalize src and dst vectors
	srcVec.Normalize();
#if PATH_POINT_DEBUG
	gout << "  normalized srcVec: " << srcVec << endl;
	gout << "  non-normalized dstVec: " << dstVec << endl;
#endif

	// 1.  Rotate src and dst so that src = (0,1,0)
	// 	1a.	Find cos(R) between srcVec and (0,1,0)
	double cosR = srcVec.m_j;
#if PATH_POINT_DEBUG
	gout << "  cosR: " << cosR << endl;
#endif

	// 	1b.	Find sin(R) between srcVec and (0,1,0)
	//double sinR = sqrt(1.0 - (cosR*cosR));
	double sinR = srcVec.m_i;
	//if (srcVec.m_j < 0)
	//if (srcVec.m_i < 0)
		//sinR = -sinR;
#if PATH_POINT_DEBUG
	gout << "  sinR: " << sinR << endl;
#endif

	//  1c. Rotate src and dst with rotation matrix.
	//  | cosR -sinR 0 |   | x |
	//  | sinR  cosR 0 | * | y | = rotated vector
	//  |  0     0   1 |   | z |
	//
	srcVec.m_i = 0.0;
	srcVec.m_j = 1.0;
	srcVec.m_k = 0.0;

	CVector3D tempVec;
	/*
	dstVec.m_i = dstVec.m_i*cosR - dstVec.m_j*sinR;	
	dstVec.m_j = dstVec.m_i*sinR + dstVec.m_j*cosR;
	dstVec.m_k = dstVec.m_k;
	*/
	tempVec.m_i = dstVec.m_i*cosR - dstVec.m_j*sinR;	
	tempVec.m_j = dstVec.m_i*sinR + dstVec.m_j*cosR;
	tempVec.m_k = dstVec.m_k;
	dstVec = tempVec;
	
	
	// If resulting dstVec = (0,0,0), then make it (0,1,0), 
	// 	the same as srcVec.
	if( dstVec.Normalize() == 0 )  dstVec.m_j = 1.0;

#if PATH_POINT_DEBUG
	gout << "  normalized, rotated srcVec should be (0,1,0): " 
		 << srcVec << endl;
	gout << "  normalized, rotated dstVec:                   " 
		 << dstVec << endl;
#endif

	// 2.  Find close angle between rotated src and dst
	double cos = srcVec.DotP( dstVec );
	double angle = acos( cos );
#if PATH_POINT_DEBUG
	gout << "  close angle: " << angle;
#endif

	// 3.  Use j coordinate of rotated dst vector to determine
	// 	whether dst lies in the III or IV quadrants.  If it does, 
	// 	then the clockwise angle = 2*PI - angle.
	//if ( (dstVec.m_j < 0.0) || ( ( dstVec.m_j == 0) && (dstVec.m_i < 0) ) )
	if( dstVec.m_i > 0.0 )  angle = cTWO_TIMES_PI - angle;

#if PATH_POINT_DEBUG
	gout << ", clockwise angle: " << angle << endl;
#endif

	return angle;
} // GetAngleBetween


//////////////////////////////////////////////////////////////////////////////
//
// Description: Get a vector of points which outlines the area covered
//              by this pathpoint.
//				
// Remarks:		
//
// Arguments:
//   points - a vector of points.  The points on the right side will be appended
//            to the end, the points on the left will be inserted at the start.
//   extra  - A multiplier - how far beyond the border of the lane/corridor to
//            create the points.
//
// Returns:		
//
//////////////////////////////////////////////////////////////////////////////
void
CPathPoint::GetOutline(vector<CPoint2D>& points, double extra) const
{
	if (m_isRoad) {

		// The PathPoint represents a road segment
		int low = -1, high = -1;
		int roadDir = 0;
		for (int n=0; n < cCV_MAX_CRDRS; n++) {
			if (m_laneCrdrMask.test( n ) ) {
				if (!roadDir) {
					CRoadPos dirPos( m_road, n );
					roadDir = (dirPos.GetLane().GetDirection() == ePOS) ? 1 : -1;
				}
				high = n;
				if ( low < 0 ) low = n;
			}
		}
		if (roadDir == -1) {
			int temp = low;
			low = high;
			high = temp;
		}
		double dist = m_startDist[low];
		bool done = false;
		int dir = ( m_startDist[low] < m_endDist[low] ? 1 : -1 );
		CRoadPos rightSide( m_road, low, dist );
		CRoadPos leftSide( m_road, high, dist );
		CPoint2D leftPt, rightPt;
		rightSide.SetOffset( extra * rightSide.GetLane().GetWidth() / 2 );
		leftSide.SetOffset( extra * -leftSide.GetLane().GetWidth() / 2 );
		while ( !done ) {
			rightPt = rightSide.GetBestXYZ();
			leftPt  =  leftSide.GetBestXYZ();
			points.push_back( rightPt );
			points.insert( points.begin(), leftPt );

			dist += 20 * dir;
			if ( dir * (dist - m_endDist[low]) > 0) {
				done = true;
				dist = m_endDist[low];
			}
			leftSide.SetDistance( dist );
			rightSide.SetDistance( dist );
		}
		rightPt = rightSide.GetBestXYZ();
		leftPt  =  leftSide.GetBestXYZ();
		points.push_back( rightPt );
		points.insert( points.begin(), leftPt );

	} else {
		
		// The PathPoint represents a corridor or group of corridors
		int low = -1, high = -1;
		int roadDir = 0;
		int lowLane = -1, highLane = -1;
		// Find the leftmost and rightmost corridor, as defined by the lanes that the corridors
		// come from.
		for (int n=0; n < cCV_MAX_CRDRS; n++) {
			if (m_laneCrdrMask.test( n ) ) {
				CRoadPos pos( m_intrsctn, n );
				if (!roadDir) {
					roadDir = pos.GetCorridor().GetSrcLn().GetDirection() == ePOS ? 1 : -1;
					lowLane = n;
					highLane = n;
				}
				int rid = pos.GetCorridor().GetSrcLn().GetRelativeId();
				if (rid * roadDir < CCrdr(m_intrsctn, lowLane).GetSrcLn().GetRelativeId() * roadDir) lowLane = n;
				if (rid * roadDir > CCrdr(m_intrsctn, highLane).GetSrcLn().GetRelativeId() * roadDir) highLane = n;
			}
		}
		double dist = m_startDist[lowLane];
		bool done = false;
		CRoadPos rightSide( m_intrsctn, lowLane, dist );
		CRoadPos leftSide( m_intrsctn, highLane, dist );
		CPoint2D leftPt, rightPt;
		double lowLength = m_endDist[leftSide.GetCorridor().GetRelativeId()];
		double highLength = m_endDist[rightSide.GetCorridor().GetRelativeId()];
		double mult = lowLength / highLength;

//		rightSide.SetOffset( extra * 5 ); 
//		leftSide.SetOffset( extra * -5 );

		rightSide.SetOffset( extra * rightSide.GetCorridor().GetWidth(dist) / 2 );
		leftSide.SetOffset( extra * -leftSide.GetCorridor().GetWidth(dist) / 2 );

		while ( !done ) {
			rightPt = rightSide.GetBestXYZ();
			leftPt  =  leftSide.GetBestXYZ();
			points.push_back( rightPt );
			points.insert( points.begin(), leftPt );

			dist += 10;
			if ( dist - highLength > 0) {
				done = true;
				dist = highLength;
			}
			leftSide.SetDistance( dist * mult );
			rightSide.SetDistance( dist );
		}
		rightPt = rightSide.GetBestXYZ();
		leftPt  =  leftSide.GetBestXYZ();
		points.push_back( rightPt );
		points.insert( points.begin(), leftPt );
	}

} // GetOutline



//////////////////////////////////////////////////////////////////////////////
//
// Description: CPathPoint::SetLaneMask
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
CPathPoint::SetLaneMask( const bitset<cCV_MAX_CRDRS>& mask )
{
	m_laneCrdrMask = mask;
	return true;
}





#undef cPATH_POINT_ZERO
} // namespace CVED
