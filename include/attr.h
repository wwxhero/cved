//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: attr.h,v 1.21 2019/07/10 15:27:48 IOWA\dheitbri Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	The implementation of the CAttr class
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __ATTR_H
#define __ATTR_H

#include "cvedpub.h"

struct cvTAttr;

namespace CVED {
typedef struct cvTAttr TAttr;	// {secret}

class CCved;

////////////////////////////////////////////////////////////////////////////////
///
/// Description:
/// This class represents an attribute.  An attribute is a general purpose
/// item that is used to associate user specific meaning to roads, lanes,
/// intersections and corridors.  On roads, lanes and corridors, an attribute
/// can be associated with part of the entity, for example, a particular 
/// attribute can be associated only with mile 2 to 5 of a 10 mile road.
///
/// Attributes are identified by a user specified id number.  If a dictionary
/// table is provided in the LRI file, each attribute can also be given a name.
/// Each attribute can also have two doubleing point values associated with
/// it.  The meaning of the value is completely up to the user.
///
class CAttr : public CCvedItem  
{
public:
	CAttr();
	virtual ~CAttr();
	CAttr(const CAttr&);
	CAttr&	operator=(const CAttr&);

	explicit CAttr(const CCved&);
	CAttr(const CCved&, int);
	CAttr(const CCved&, TAttr*);

	string	GetName(void) const;
	int		GetId(void) const;
	double	GetVal1(void) const;
	double	GetVal2(void) const;
	double	GetFrom(void) const;
	double	GetTo(void) const;
	int		GetLaneMask(void) const;	

	bool	IsValid(void) const;

	friend ostream &operator<<(ostream &, const CAttr &);

protected:
	 void	AssertValid(void) const;

private:
	TAttr*	m_pAttr;
};

////////////////////////////////////////////////////////////////////////////////
///
/// Description: constructors and destructor
///  
/// Arguments: 
/// 	cved - constant reference to the CCved instance
/// 	
inline 
CAttr::CAttr() : CCvedItem(0) {}

inline
CAttr::~CAttr() {}

inline 
CAttr::CAttr(const CCved& cCved) : CCvedItem(&cCved),m_pAttr(nullptr) {}

inline 
CAttr::CAttr(const CAttr& cRhs) { *this = cRhs; }

////////////////////////////////////////////////////////////////////////////////
///
/// Description: Indicates whether the current instance is valid
/// 
/// Remarks: This function may be used by outside elements which want to insure
/// 	that the current instance is valid, without risking a failed assertion.
///
/// Returns: true or false  
/// 	
inline bool
CAttr::IsValid(void) const {
	return (CCvedItem::IsValid() && (m_pAttr != 0));
}

////////////////////////////////////////////////////////////////////////////////
///
/// Description: Asserts whether the current instance is valid
/// 
/// Remarks: This function is used by local methods where a valid CAttr
///	instance is necessary for proper functionality.
/// 	
inline void
CAttr::AssertValid(void) const
{
	CCvedItem::AssertValid();
	assert (m_pAttr != 0);
}

}		// namespace CVED
#endif // __ATTR_H

