//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: lanemask.cxx,v 1.14 2018/07/16 14:04:35 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	The implementation of the lanemask class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

// for Docjet to recognize the namespace
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLaneMask
//	The default constructor initializes the mask to empty.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLaneMask::CLaneMask()
{
	int  i;
	for (i=0; i<cCV_MAX_LANES/8; i++)
		m_data[i] = 0;
} // end of CLaneMask

//////////////////////////////////////////////////////////////////////////////
//
// Description: CLaneMask
//	The copy constructor creates a replica of an existing class.
//
// Remarks:
// 
// Arguments: 
//   cCopy - object to be copied 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CLaneMask::CLaneMask(const CLaneMask& cCopy)
{
	*this = cCopy;
} // end of CLaneMask

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy from the parameter to the current instance.
//
// Remarks: 
//
// Arguments:
//    cRhs - the cvLaneMask object on the right hand side of assignment
//    
// Returns: a reference to the current instance (an l-value), to allow 
// 	cascaded assignments
//
//////////////////////////////////////////////////////////////////////////////
const CLaneMask& 
CLaneMask::operator=(const CLaneMask& cRhs)
{
	int i;
	for (i=0; i<cCV_MAX_LANES/8; i++)
		m_data[i] = cRhs.m_data[i];

	return *this;
} // end of operator=


//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CLaneMask
// 	Default destructor has very little work to do.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
CLaneMask::~CLaneMask()
{} // end of ~CLaneMask


//////////////////////////////////////////////////////////////////////////////
//
// Description: SetAllLanes
// 	This member function sets all lanes in the class.  
//
// Remarks: Following this call, the HasLane function will return true for all
// 	valid lane identifiers.
//
// Arguments:
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void CLaneMask::SetAllLanes(void)
{
	int i;
	for (i=0; i<cCV_MAX_LANES/8; i++)
		m_data[i] = 0xFF;
} // end of SetAllLanes

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetLane
// 	This member function adds the specified lane in the list of lanes 
// 	contained in the mask.  
//
// Remarks: The membership of the remaining lanes is not affected.
//
// Arguments:
// 	lane - which lane to add to the mask
// 	
// Returns: void
// 
//////////////////////////////////////////////////////////////////////////////
void CLaneMask::SetLane(unsigned lane)
{
	if ( lane < cCV_MAX_LANES )
		m_data[lane/8] |= (1 << (lane % 8));	
} // end of SetLane

//////////////////////////////////////////////////////////////////////////////
//
// Description: ClearAllLanes
// 	This function removes all lanes from the mask leaving the mask empty.
//
// Remarks: Following this call, the HasLane function will return false for
// 	all lane identifiers.
//
// Arguments: 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void CLaneMask::ClearAllLanes(void)
{
	for (int i=0; i<cCV_MAX_LANES/8; i++)
		m_data[i] = 0;
} // end of ClearAllLanes

//////////////////////////////////////////////////////////////////////////////
// 
// Description: ClearLane
// 	This function removes the specified lane from the list of lanes contained 
// 	in the mask.  
//
// Remarks: If the lane is invalid, the class remains unchanged
//
// Arguments:
// 	lane  - which lane to remove from the mask
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void CLaneMask::ClearLane(unsigned lane)
{
	if ( lane < cCV_MAX_LANES )
		m_data[lane/8] &= ~(1 << lane%8);
} // end of ClearLane


//////////////////////////////////////////////////////////////////////////////
//
// Description: HasLane
// 	This function indicates if the specified lane is contained in the mask.  
//
// Remarks:
//
// Arguments:
//    lane - which lane to consider
//
// Returns: If the lane is invalid or is not part of the mask, the function 
// 	returns false otherwise it returns true.
//
//////////////////////////////////////////////////////////////////////////////
bool CLaneMask::HasLane(unsigned lane) const
{
	if ( lane < cCV_MAX_LANES )
		return (m_data[lane/8] & (1 << (lane % 8))) != 0;
	else
		return false;
} // end of HasLane

#ifndef _PowerMAXOS
//////////////////////////////////////////////////////////////////////////////
// 
// Description: SelfTest (private)
// 	Internal self test function.  
//
// Remarks: Not to be used by users.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void CLaneMask::SelfTest(void)
{
	CLaneMask  lm;

	printf("Size of lane mask is %zu\n", sizeof(lm));

	lm.SetAllLanes();

	if ( lm.HasLane(3) == false ) {
		cvCTestError err("HasLane failed", __FILE__, __LINE__);
		throw err;
	}

	if ( lm.HasLane(15) == false ) {
		cvCTestError err("HasLane failed", __FILE__, __LINE__);
		throw err;
	}

	lm.ClearAllLanes();
	if ( lm.HasLane(15) == true ) {
		cvCTestError err("HasLane failed", __FILE__, __LINE__);
		throw err;
	}

	lm.SetLane(5);
	lm.SetLane(9);

	CLaneMask lm2(lm);
	CLaneMask lm3;
	lm3 = lm;

	if ( lm.HasLane(5) == false || lm2.HasLane(5) == false || lm3.HasLane(5) == false) {
		cvCTestError err("LaneMask tests failed", __FILE__, __LINE__);
		throw err;
	}
	if ( ! lm.HasLane(9) || ! lm2.HasLane(9) || ! lm3.HasLane(9) ) {
		cvCTestError err("LaneMask tests failed", __FILE__, __LINE__);
		throw err;
	}
	if ( lm.HasLane(10) == true ) {
		cvCTestError err("LaneMask tests failed", __FILE__, __LINE__);
		throw err;
	}
	if ( lm.HasLane(4) == true ) {
		cvCTestError err("LaneMask tests failed", __FILE__, __LINE__);
		throw err;
	}
	lm.ClearLane(9);
	if ( lm.HasLane(9) == true ) {
		cvCTestError err("LaneMask tests failed", __FILE__, __LINE__);
		throw err;
	}
} // end of SelfTest
#endif //_PowerMAXOS

} // namespace CVED
