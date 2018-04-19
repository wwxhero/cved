//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: objmask.cxx,v 1.8 2000/07/20 06:47:15 ltsao Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	This file contains a static def for the ObjMask class and 
// 	some template instantiation for concurrent compiling untill better 
// 	solution (is found if ever...)
//
//////////////////////////////////////////////////////////////////////////////
#ifdef _PowerMAXOS // {secret}
#include<sstream>
#include<iosfwd.H>
ostringstream mystring;
#endif

#include "cvedpub.h"

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

// Static Value definition
TU32b CObjTypeMask::m_all = 0xFFFFFFFF;

CObjTypeMask::CObjTypeMask(bool bSingleExtern) : m_bSingleExtern(bSingleExtern)
{

}

bool CObjTypeMask::Has(cvEObjType t, int objID) const
{
	if (!m_bSingleExtern)
		return CObjTypeMask::Has(t);
	else
	{
		bool has = false;
		bool peerSim = (eCV_EXTERNAL_DRIVER == t
					&& 0 != objID);
		if (peerSim) //peer simulator appears as a vehicle
			has = CObjTypeMask::Has(eCV_VEHICLE);
		else
			has = CObjTypeMask::Has(t);
		return has;
	}
}

} // namespace CVED
