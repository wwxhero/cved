//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: hldofs.h,v 1.6 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Lijen Tsao
// Date:		March, 2000
//
// Description:	The implementation of the CHldOfs class
//
//////////////////////////////////////////////////////////////////////////////

#ifndef __HLDOFS_H
#define __HLDOFS_H

#include "cvedpub.h"

struct cvTHldOfs;

namespace CVED {
typedef struct cvTHldOfs THldOfs;	// {secret}

class CCved;

///////////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents an holdoffset. 
//
class CHldOfs : public CCvedItem  
{
public:
	CHldOfs();
	virtual ~CHldOfs();
	CHldOfs(const CHldOfs&);
	CHldOfs&	operator=(const CHldOfs&);

	explicit CHldOfs(const CCved&);
	CHldOfs(const CCved&, int);
	CHldOfs(const CCved&, THldOfs*);

	int		GetObjId(void) const;
	int     GetSolId(void) const;
	string	GetName(void) const;
	char	GetReason(void) const;
	double	GetDistance(void) const;
	double	GetOrientation(void) const;
	double	GetThickness(void) const;

	bool	IsValid(void) const;

	friend ostream &operator<<(ostream &, const CHldOfs &);

protected:
	 void	AssertValid(void) const;

private:
	THldOfs*	m_pHldOfs;
};

///////////////////////////////////////////////////////////////////////////////
//
// Description: constructors and destructor
//  
// Arguments: 
// 	cved - constant reference to the CCved instance
// 	
inline 
CHldOfs::CHldOfs() : CCvedItem(0) {}

inline
CHldOfs::~CHldOfs() {}

inline 
CHldOfs::CHldOfs(const CCved& cCved) : CCvedItem(&cCved) {}

inline 
CHldOfs::CHldOfs(const CHldOfs& cRhs) { *this = cRhs; }

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
CHldOfs::IsValid(void) const {
	return (CCvedItem::IsValid() && (m_pHldOfs != 0));
}

///////////////////////////////////////////////////////////////////////////////
//
// Description: Asserts whether the current instance is valid
// 
// Remarks: This function is used by local methods where a valid CHldOfs
//	instance is necessary for proper functionality.
// 	
inline void
CHldOfs::AssertValid(void) const
{
	CCvedItem::AssertValid();
	assert (m_pHldOfs != 0);
}

}		// namespace CVED
#endif // __HLDOFS_H

