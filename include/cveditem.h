//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: cveditem.h,v 1.18 2001/05/30 15:52:51 shutters Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	The implementation of the CCvedItem class
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __CVED_ITEM_H
#define __CVED_ITEM_H 	// {secret}

#include "cvedpub.h"

struct cvTObjAttr;
struct cvTObj;
struct cvTCntrlPnt;
struct cvTCrdrCntrlPnt;
struct cvTRoadPiece;
struct cvTRoad;
struct cvTLane;
struct cvTIntrsctn;
struct cvTCrdr;

namespace CVED {
	
class CCved;

/////////////////////////////////////////////////////////////////////////////
//
// This class is the root of all classes representing elements that belong
// to an instance of a virtual environment represented by the CCved class.
// The class keeps track of which CCved instance a particular object belongs
// to and provides some basic functionality and debugging support.  
//
// In general, users of the CVED library should never have to deal 
// with this class directly.
//

class CCvedItem  
{
public:
	CCvedItem();
	CCvedItem(const CCvedItem&);
	CCvedItem&			operator=(const CCvedItem&);
	virtual ~CCvedItem();

	explicit CCvedItem(const CCved *);
	virtual bool		IsValid(void) const;

	void				SetCved(const CCved*);

protected:
	bool				IsBound(void) const;
	void*				GetInst(void) const;
	void*				GetInst(const CCved*) const;
	const CCved&		GetCved(void) const;
	virtual void		AssertValid(void) const;

	struct cvTObjAttr*	BindObjAttr(int) const;
	struct cvTObj*		BindObj(int) const;
	struct cvTCntrlPnt*	BindCntrlPnt(int) const;
	struct cvTCrdrCntrlPnt*	BindCrdrPnt(int) const;
	struct cvTRoadPiece*BindRoadPiece(int) const;
	struct cvTRoad*		BindRoad(int) const;
	struct cvTLane*		BindLane(int) const;
	struct cvTIntrsctn*	BindIntrsctn(int) const;
	struct cvTCrdr*		BindCrdr(int) const;

	const CTerrainGrid<Post>*
						GetIntrsctnGrid(int) const;

private:
	const CCved*		m_cpCved;

};


/////////////////////////////////////////////////////////////////////////////
// Inline functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Constructors.
//
// Arguments:
// ptr - a pointer to the CCved instance class that the object should
// be bound.
// copy - the object to duplicate
//
inline 
CCvedItem::CCvedItem()
	: m_cpCved(0) {}

inline 
CCvedItem::CCvedItem(const CCvedItem& cCopy) 
	: m_cpCved(cCopy.m_cpCved) {}

inline CCvedItem &
CCvedItem::operator=(const CCvedItem& cRhs)
{
	if (this != &cRhs) {
		m_cpCved = cRhs.m_cpCved;
	}
	return *this;
}

//////////////////////////////////////////////////////////////////////////////
// 
// Description: AssertValid (protected)
// 	Causes an assertion failure if there is no CCved instance bound to the
// 	current CCvedItem instance.
//
// Remarks: Used by all the subclasses of CCvedItem to insure the instance is
// 	valid before accessing the local variables.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CCvedItem::AssertValid(void) const
{
	assert(m_cpCved!=0);
} // end of AssertValid

/////////////////////////////////////////////////////////////////////////////
//
// Description: verify an object is bound to a CCved instance
//
// Remarks:
//
// Returns:
// The function returns true if the object is bound, else it returns false.
//
inline bool
CCvedItem::IsBound(void) const { return m_cpCved != 0; }

/////////////////////////////////////////////////////////////////////////////
//
// Description: verify if a class references a valid CVED entity
//
// Remarks:
// This function determines if the entity represented by the class
// is valid.  An entity is valid if it references some existing
// entity in a CVED instance. 
//
// Returns:
// The function returns true if the entity is valid or false.
//
inline bool 
CCvedItem::IsValid(void) const { return (m_cpCved != 0); }

inline
CCvedItem::~CCvedItem() { }

}		// namespace CVED

#endif // _CVED_ITEM_H

