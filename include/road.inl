///////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: road.inl,v 1.5 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Yiannis Papelis
// Date:		October, 1998
//
// Description:	The inlin functions for the CRoad class
//
//////////////////////////////////////////////////////////////////////
#ifndef __ROAD_INL
#define __ROAD_INL		// {secret}

#include "road.h"

/////////////////////////////////////////////////////////////////////////////
//
// Description: AssertValid (protected)
// 	Assert that the instance is valid
//
// Remarks: This is a "guard" function that should be called before the
// 	class touches its internal data structures.  It verifies that
// 	the object is bound and has a valid pointer to a road in the
// 	virtual environment.  
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void
CRoad::AssertValid(void) const
{
	CCvedItem::AssertValid();
	if( m_pRoad == 0 ) 
	{
		string msg = "CRoad::AssertValid: null road";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}
} // end of AssertValid

//////////////////////////////////////////////////////////////////////////////
//
// Description: IsValid
// 	Indicates whether the current instance is valid
// 
// Remarks: This function may be used by outside elements which want to insure
// 	that the current instance is valid, without risking a failed assertion.
//
// Arguments:
//
// Returns: true or false  
// 	
///////////////////////////////////////////////////////////////////////////////
inline bool 
CRoad::IsValid(void) const
{
	return (CCvedItem::IsValid() && (m_pRoad != 0) );
} // end of IsValid

///////////////////////////////////////////////////////////////////////////////
//
// Description: CRoad
// 	Copy constructor creates a CRoad instance from the parameter.
//
// Remarks:
//
// Arguments:
//	cCopy - CRoad instance to copy
//	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline 
CRoad::CRoad(const CRoad &cCopy)
{
	*this = cCopy;
} // end of CRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoad
// 	Construts a CRoad instance that is bound but invalid.
//
// Remarks:
//
// Arguments:
//	cCved - CCved instance
//
// Returns: void
// 
//////////////////////////////////////////////////////////////////////////////
inline 
CRoad::CRoad(const CCved &cCved) 
	: CCvedItem(&cCved), 
	  m_pRoad(0) 
{} // end of CRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator==
// 	Return true if the current instance represents the same road as the 
// 	parameter.
//
// Remarks:
//
// Arguments:
//  cRhs - a reference to an object intended to be to the right of the ==
//
// Returns: true if they are equal, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
inline bool
CRoad::operator==(const CRoad& cRhs)
{
	AssertValid();
	return(m_pRoad==cRhs.m_pRoad);
} // end of operator==

//////////////////////////////////////////////////////////////////////////////
//
// Description: Get2CntrlPnts
// 	Acquires the indecs of the first and last controlpoints being bound by 
// 	the road piece
//
// Remarks:
//
// Arguments:
//	pFirst - a pointer to the index of the first control point being bound
//	pLast  - a pointer to the index of the last control point being bound
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void
CRoadPiece::Get2CntrlPnts(int* pFirst, int* pLast)
{
	*pFirst = static_cast<int>(m_first);
	*pLast	= static_cast<int>(m_last);
} // end of Get2CntrlPnts
 
//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPiece
// 	Default constructor creates an unbound, invalid instance.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CRoadPiece::CRoadPiece() 
	: CCvedItem(0),
	  m_road(0), 
	  m_x1(0.), 
	  m_y1(0.), 
	  m_x2(0.), 
	  m_y2(0.),
	  m_first(0),
	  m_last(0)
{} // end of CRoadPiece

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPiece
// 	Default constructor creates a bound but invalid instance.
//
// Remarks:
//
// Arguments:
// 	cCved - a CCved instance
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CRoadPiece::CRoadPiece(const CCved& cCved)
	: CCvedItem(&cCved),
	  m_road(0), 
	  m_x1(0.), 
	  m_y1(0.), 
	  m_x2(0.), 
	  m_y2(0.),
	  m_first(0),
	  m_last(0)
{} // end of CRoadPiece

//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CRoadPiece
// 	Default destructor does nothing.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CRoadPiece::~CRoadPiece()
{} // end of CRoadPiece

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy from the parameter to the current instance.
//
// Remarks:
//
// Arguments:
//  cRhs - a reference to an object intended to be to the right of the =
//
// Returns: a reference to the current instance so that the assignments can
// 	be nested.
//
//////////////////////////////////////////////////////////////////////////////
inline CRoadPiece&
CRoadPiece::operator=(const CRoadPiece& cRhs)
{
	if ( &cRhs != this ) {

		// Perform assignment of superclass items
		(CCvedItem)*this = (CCvedItem)cRhs;

		m_road	= cRhs.m_road;
		m_x1	= cRhs.m_x1;
		m_y1	= cRhs.m_y1;
		m_x2	= cRhs.m_x2;
		m_y2	= cRhs.m_y2;
		m_first	= cRhs.m_first;
		m_last	= cRhs.m_last;
	}
	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: CRoadPiece
// 	Copy constructor initializes the current instance with the parameter.
//
// Remarks:
//
// Arguments:
//	cCopy - the copy of a class to initialize the current instance
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CRoadPiece::CRoadPiece(const CRoadPiece &cCopy)
{
	*this = cCopy;
} // end of CRoadPiece


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoad
// 	Return the identifier of the parent road 
//
// Remarks: This function returns the identifier that this piece was
// 	extracted from.
//
// Arguments:
//
// Return value: The road identifier.
//
//////////////////////////////////////////////////////////////////////////////
inline int
CRoadPiece::GetRoad(void) const
{
	return static_cast<int>(m_road);
} // end of GetRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: HitTest
// 	Determine if a point is within the bounding box of the road piece.
// 
// Remarks:
//
// Arguments
// 	x - the x coordinate of the point
// 	y - the y coordinate of the point
//
// Returns: True if the point is within the bound box or false otherwise
//
//////////////////////////////////////////////////////////////////////////////
inline bool
CRoadPiece::HitTest(double x, double y) const
{
	if ( x >= m_x1 && x <= m_x2 && y >= m_y1 && y <= m_y2 )
		return true;
	else
		return false;
} // end of HitTest

#endif // __ROAD_INL

