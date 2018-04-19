//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: reconfobj.h,v 1.8 1999/12/30 18:25:18 jvogel Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	The declaration for the CReconfObj class
//
/////////////////////////////////////////////////////////////////////////////
#ifndef __RECONF_OBJ_H
#define __RECONF_OBJ_H		// {secret}

#include "cvedpub.h"

namespace CVED {

/////////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents a reconfigurable object in the virtual environment.
// The dynamic object class derives from the CObj class and thus
// possesses all features of static objects.
//
// This class is an abstract class that cannot be instanced directly.
// However, there will be one class derived per each object type
// defined in the current version of CVED. 
//
// 
class CReconfObj : public CObj
{
public:
	CReconfObj();
	virtual ~CReconfObj();
	CReconfObj(const CReconfObj&);
	CReconfObj&	operator=(const CReconfObj&);

	int			GetNumOptions(void) const;
	void		SetOption(int);

protected:
	int			m_currentOption;
};


/////////////////////////////////////////////////////////////////////////////
//
// Inline functions
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: constructors.
//
// Remarks:
// The various constructors initialize an object instance.  The 
// default constructor builds an unbound object.  When a CCved pointer 
// is provided, the class
// is initialized as bound, but is not associated with an actual object
// in the virtual environment.  The constructor with a string searches
// all static and dynamic objects for one whose name matches the argument.
// If none is found, the object is bound but not associated.
//
// Arguments:
//  pEnv - pointer to CCved instance in which the object is bound
//  name - the name of the object in the environment to associate with this
//         instance
//  copy - the copy of a class to initialize the current instance
//
//
inline
CReconfObj::CReconfObj() : CObj() { }

inline
CReconfObj::CReconfObj(const CReconfObj &cCopy)
{
	*this = cCopy;
}


}		// namespace CVED

#endif // __RECONF_OBJ_H

