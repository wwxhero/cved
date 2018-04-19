//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:		$Id: cntrlpnt.cxx,v 1.15 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):   Li-Jen Tsao
// Date:        December, 1998
//
// Description: The implementation of the CRoad class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: CCntrlPnt
// 	The constructor initializes an longitudinal control point instance.  
//
// Remarks: The index to control point is with respect to the road.
//
// Arguments:
//	cCved  - reference to a CCved instance
//	rdIdx - the index of the road where the control point is
//	ptIdx - the index of the control point being copied
//
// Returns: void 
//
//////////////////////////////////////////////////////////////////////////////
CCntrlPnt::CCntrlPnt(
			const CCved&	cCved, 
			int				rdIdx, 
			int				ptIdx) 
	: CCvedItem(&cCved)
{
	cvTHeader*	pH   = static_cast<cvTHeader*> (GetInst());
	char*		pOfs = static_cast<char*>	(GetInst()) + pH->roadOfs;
	cvTRoad*	pRoad = (reinterpret_cast<TRoad*>(pOfs)) + rdIdx;

	pOfs = static_cast<char*> (GetInst()) + pH->longitCntrlOfs;
	m_pCntrlPnt =	reinterpret_cast<TCntrlPnt*>(pOfs) + 
					pRoad->cntrlPntIdx + 
					ptIdx;
} // end of CCntrlPnt

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy of the parameter to the current instance.
//
// Remarks: 
//
// Arguments:
//  cRhs - a reference to an object intended to be to the right of the =
//
// Returns: a reference to the current instance
//
//////////////////////////////////////////////////////////////////////////////
CCntrlPnt&
CCntrlPnt::operator=(const CCntrlPnt& cRhs)
{
	if ( this != &cRhs ) {
		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pCntrlPnt = cRhs.m_pCntrlPnt;
	}

	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetName
// 	This function returns a string holding the name of the current control 
// 	point.
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//
// Returns: an STL string containing the control point's name.
//
//////////////////////////////////////////////////////////////////////////////
string 
CCntrlPnt::GetName(void) const
{
	AssertValid();

	char*		pCharPool;

	cvTHeader*	pH	= static_cast<cvTHeader*> (GetInst());
	pCharPool		= static_cast<char*> (GetInst()) + pH->charOfs;
	string s(pCharPool + m_pCntrlPnt->nameIdx);

	return s;
} // end of GetName

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetWidth
// 	This function returns the logical or physical width of the control point 
// 	depending on the passed parameter
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Argument:
//	logical - a bool value to indicate returning logical width when true
//
// Returns: the double width of the control point
//
//////////////////////////////////////////////////////////////////////////////
double
CCntrlPnt::GetWidth(bool logical) const
{
	AssertValid();

	if (logical)
		return m_pCntrlPnt->logicalWidth;
	else
		return m_pCntrlPnt->physicalWidth;
} // end of GetWidth

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetDistance
// 	This function returns the distance of the control point from the first 
// 	control point.
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//
// Returns: the double distance component of the control point
//
//////////////////////////////////////////////////////////////////////////////
double
CCntrlPnt::GetDistance(void) const
{
	AssertValid();
	
	return m_pCntrlPnt->distance;
} // end of GetDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRadius
// 	This function returns the radius of the control point  
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//
// Returns: the double radius of the control point
//
//////////////////////////////////////////////////////////////////////////////
double 
CCntrlPnt::GetRadius(void) const
{
	AssertValid();

	return m_pCntrlPnt->radius;
} // end of GetRadius

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNormal
// 	This function returns the radius of the control point  
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//
// Returns: the normal of the control point
//
///////////////////////////////////////////////////////////////////////////
CVector3D 
CCntrlPnt::GetNormal(void) const
{
	AssertValid();

	CVector3D vec(m_pCntrlPnt->normal);
	return vec;
} // end of GetNormal

///////////////////////////////////////////////////////////////////////////
//
// Description: GetLocation
// 	This function returns the location of the control point      
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//
// Returns: the location of the control point
//
///////////////////////////////////////////////////////////////////////////
CPoint3D
CCntrlPnt::GetLocation(void) const
{
	AssertValid();

	CPoint3D pt(m_pCntrlPnt->location);
	return pt;
} // end of GetLocation

///////////////////////////////////////////////////////////////////////////
//
// Description: GetRightVec
// 	This function returns the linear or cubic right vector of the control 
// 	point depending on the passed parameter cubic
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//	cubic - returning the cubic right vector when true, linear otherwise   
//
// Returns: the linear or cubic right CVector3D of the control point
//
///////////////////////////////////////////////////////////////////////////
CVector3D
CCntrlPnt::GetRightVec(bool cubic) const
{
	AssertValid();
    
	if (cubic) { 
		CVector3D rightVec(m_pCntrlPnt->rightVecCubic);
		return rightVec; 
	}
	else {
		CVector3D rightVec(m_pCntrlPnt->rightVecLinear);
		return rightVec; 
	}
} // end of GetRightVec
			
///////////////////////////////////////////////////////////////////////////
//
// Description: GetTangVec
// 	This function returns the linear or cubic tangent vector of the 
// 	control point depending on the passed parameter cubic
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	invalid CCntrlPnt instance.
//
// Arguments:
//  cubic - returning the cubic tangent vector when true, linear otherwise
//
// Returns: the linear or cubic tangent Cvector3D of the control point
//
///////////////////////////////////////////////////////////////////////////
CVector3D
CCntrlPnt::GetTangVec(bool cubic) const
{
	AssertValid();
    
	if (cubic) {
		CVector3D tVec(m_pCntrlPnt->tangVecCubic);
		return tVec;
	}
	else {
		CVector3D tVec(m_pCntrlPnt->tangVecLinear);
		return tVec;
	}
} // end of GetTangVec

} // namespace CVED
