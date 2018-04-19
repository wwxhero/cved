/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: objmask.h,v 1.11 1999/12/30 18:25:18 jvogel Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	Declaration of the object type mask
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __OBJ_MASK_H
#define __OBJ_MASK_H		// {secret}

#include "cvedpub.h"

namespace CVED {

//
// This class provides an efficient representation of a set of object 
// types.  It also provides convenient converstions from/to
// integer types.  It is meant to allow filtering of the type
// of objects returned from the various object interrogation
// functions.
//
// Note that the class is designed explicitly for object
// types and makes key assumptions regarding the maximum number
// of objects.  
//
class CObjTypeMask {
public:
	CObjTypeMask();					// creates empty mask
	CObjTypeMask(const CObjTypeMask&);
	//fixme: shall it be done in derived class
	CObjTypeMask(bool singleExtern);
	virtual ~CObjTypeMask();
	const CObjTypeMask&	operator=(const CObjTypeMask&);
	CObjTypeMask(TU32b bits);

	static TU32b 	m_all;				// represents a full set

	void			SetAll(void);		// sets the set
	void			Clear(void);		// clears all types
	void			Clear(cvEObjType);	// removes specified type
	void			Set(cvEObjType);	// add specified type to the set
	bool			Has(cvEObjType) const;	// queries existense of type 
											// in set
	virtual bool	Has(cvEObjType, int objID) const;

protected:
	void			SelfTest(void);		// {secret} provides basic self tests

protected:
	TU8b			m_data[cNUM_OBJECT_TYPES/8+1];
	bool			m_bSingleExtern;
};

#include "objmask.inl"

}	// namespace CVED

#endif	// __OBJ_MASK_H

