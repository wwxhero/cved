//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: objattr.h,v 1.4 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Lijen Tsao	
// Date:		May 2000
//
// Description:	The implementation of the CObjAttr class
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __OBJ_ATTR_H
#define __OBJ_ATTR_H

#include "cvedpub.h"

struct cvTObjAttr;

namespace CVED {
typedef struct cvTAttr TAttr;	// {secret}

class CCved;

///////////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents an object attribute.  An object attribute is a general 
// purpose item that is used to associate user specific meaning to 
// an object. ObjAttr consists of solId hcsmId xSize ySize zSize
//
//
class CObjAttr : public CCvedItem  
{
public:
	CObjAttr();
	virtual ~CObjAttr();
	CObjAttr(const CObjAttr&);
	CObjAttr&	operator=(const CObjAttr&);

	explicit CObjAttr(const CCved&);
	CObjAttr(const CCved&, int);
	CObjAttr(const CCved&, cvTObjAttr*);

	int		GetSolId(void) const;
	int		GetHcsmId(void) const;
	double	GetXSize(void) const;
	double	GetYSize(void) const;
	double	GetZSize(void) const;

	bool	IsValid(void) const;

	friend ostream &operator<<(ostream &, const CObjAttr &);

protected:
	 void	AssertValid(void) const;

private:
	cvTObjAttr*	m_pObjAttr;
};

///////////////////////////////////////////////////////////////////////////////
//
// Description: constructors and destructor
//  
// Arguments: 
// 	cved - constant reference to the CCved instance
// 	
inline 
CObjAttr::CObjAttr() : CCvedItem(0) {}

inline
CObjAttr::~CObjAttr() {}

inline 
CObjAttr::CObjAttr(const CCved& cCved) : CCvedItem(&cCved) {}

inline 
CObjAttr::CObjAttr(const CObjAttr& cRhs) { *this = cRhs; }

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
CObjAttr::IsValid(void) const {
	return (CCvedItem::IsValid() && (m_pObjAttr != 0));
}

///////////////////////////////////////////////////////////////////////////////
//
// Description: Asserts whether the current instance is valid
// 
// Remarks: This function is used by local methods where a valid CObjAttr
//	instance is necessary for proper functionality.
// 	
inline void
CObjAttr::AssertValid(void) const
{
	CCvedItem::AssertValid();
	assert (m_pObjAttr != 0);
}

}		// namespace CVED
#endif // __ATTR_H

