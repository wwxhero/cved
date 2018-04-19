////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: reconfobj.cxx,v 1.6 2000/07/20 06:47:16 ltsao Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	The implementation of the CReconfObj class
//
////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CReconfObj
// 	The default destructor does nothing.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CReconfObj::~CReconfObj()
{} // end of ~CReconfObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNumOptions
// 	Return the number of options for this object
//
// Remarks: Reconfigurable objects can have an arbitrary number of attributes
// 	associated with them.  These alternative sets of attributes are called 
// 	options.  This function returns the number of options associated with 
// 	this object.
//
// Returns: An integer representing the number of options.  It will be 
// 	at least one.
//
//////////////////////////////////////////////////////////////////////////////
int   
CReconfObj::GetNumOptions(void) const
{
	// Needs to be completed.
	return 0;
} // end of GetNumOptions

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetOption
// 	Set the active option for the object
//
// Remarks: This function sets the currently active option for this object.
// 	Valid options are between 0 and N-1, where N is the value returned by the 
// 	GetNumOptions function.
//
// Arguments:
// 	opt - the option to set as active.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void  
CReconfObj::SetOption(int opt)
{} // end of SetOption

} // namespace CVED
