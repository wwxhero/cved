//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// $Id: lanemask.h,v 1.9 1999/12/30 18:25:17 jvogel Exp $
//
// Description:	Header file for the lane mask class
//
/////////////////////////////////////////////////////////////////////////////
#ifndef __LANE_MASK_H
#define __LANE_MASK_H // {secret}

#include "cvedpub.h"

namespace CVED {

/////////////////////////////////////////////////////////////////////
//
// This class represents a set containing lanes.  It provides
// operations for adding/remove lanes from the set, and finding
// out if a particular lane is part of the set.
//
class CLaneMask {
public:
	CLaneMask();					// creates empty mask
	CLaneMask(const CLaneMask&);
	virtual ~CLaneMask();
	const CLaneMask& operator=(const CLaneMask&);

	void	SetAllLanes(void);			// sets all lanes in mask
	void	ClearAllLanes(void);		// clears all lanes in mask
	void	SetLane(unsigned);			// sets specified lane
	void	ClearLane(unsigned);		// removes specified lane
	bool	HasLane(unsigned) const;	// queries existence of lane

protected:
	void	SelfTest(void);			// provides basic self tests

protected:
	TU8b	m_data[cCV_MAX_LANES/8];
};

}		// namespace CVED

#endif		// #ifndef __LANE_MASK_H
