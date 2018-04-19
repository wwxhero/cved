/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: cntrlpnt.h,v 1.16 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Li-Jen Tsao
// Date:        December, 1998
//
// Description: The definition of the CCrdr class
//
//////////////////////////////////////////////////////////////////////
#ifndef __CNTRLPNT_H
#define __CNTRLPNT_H    

#include "cvedpub.h"
#include <point3d.h>
#include <vector3d.h>

////////////////////////////////////////////////////////////////////////
//
// forward declarations for the road structure.  We define them here
// so we can reference them but we don't include the actual
// header file since it's private to CVED.
//

struct cvTCntrlPnt;

namespace CVED {

typedef struct cvTCntrlPnt TCntrlPnt;

/////////////////////////////////////////////////////////////////////////
//
// This class represents a logitudinal control point in the virtual
// environment. A road consists of a longitudinal spline that defines
// the centerline.
//
class CCntrlPnt : public CCvedItem {
public:
	CCntrlPnt();
	CCntrlPnt(const CCntrlPnt&);
	explicit CCntrlPnt(const CCved&);
	CCntrlPnt(const CCved&, int, int);
	CCntrlPnt&	operator=(const CCntrlPnt&);
	virtual ~CCntrlPnt();

	double		GetWidth(bool logical=false) const;
	double		GetDistance(void) const;
	double		GetRadius(void) const; 
	string		GetName(void) const;
	CVector3D	GetNormal(void) const;
	CPoint3D	GetLocation(void) const;
	CVector3D	GetRightVec(bool cubic=false) const;
	CVector3D	GetTangVec(bool cubic=false) const;

	bool		IsValid(void) const;

protected:
	void		AssertValid(void) const;

private:
	TCntrlPnt*	m_pCntrlPnt;
};

///////////////////////////////////////////////////////////////////
//
// Inline implementations
//
//////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: constructors.
//
// Remarks:
// The various constructors initialize an logitudinal control point
// 
// Arguments:
//	copy - the copy of a class to initialize the current instance
//
///////////////////////////////////////////////////////////////////////////
inline
CCntrlPnt::CCntrlPnt() : CCvedItem(0), m_pCntrlPnt(0)
{
}

inline
CCntrlPnt::CCntrlPnt(const CCntrlPnt& cCopy)
{
	*this = cCopy;
}

inline 
CCntrlPnt::~CCntrlPnt()
{
}

inline
CCntrlPnt::CCntrlPnt(const CCved& cCved) : CCvedItem(&cCved), m_pCntrlPnt(0)
{
}

/////////////////////////////////////////////////////////////////////////////
//
// {secret}
// Description: verify the class is valid
// Remarks:
// This is a "guard" function that should be called before the
// class touches its internal data structures.  It verifies that
// the object is bound and has a valid pointer to a road in the
// virtual environment.
//
inline void
CCntrlPnt::AssertValid(void) const
{
	CCvedItem::AssertValid();
    if( m_pCntrlPnt == 0 )
	{
		string msg = "CCntrlPnt::AssertValid: null control point";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is valid
// 
// Remarks: This function may be used by outside elements which want to insure
// 	that the current instance is valid, without risking a failed assertion.
//
// Returns: true or false  
// 	
inline bool
CCntrlPnt::IsValid(void) const
{
	return (CCvedItem::IsValid() && (m_pCntrlPnt != 0));
}
 
}		// namespace CVED
#endif  // #ifdef __CNTRLPNT_H

