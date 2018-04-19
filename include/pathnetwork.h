//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:		$Id: pathnetwork.h,v 1.5 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Jillian Vogel
// Date:		October, 1999
//
// Description:	The definition of the CPathNetwork class.  A CPathNetwork is
// 	an immutable collection of Cpaths and their interconnections over a 
// 	specified area of the road network.  
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PATH_NETWORK_H
#define __PATH_NETWORK_H	// {secret}

#include "cvedpub.h"
#include "point2d.h"

#ifdef _WIN32
#include <set>
#include <vector>
#include <map>
#elif __sgi
#include <set.h>
#include <vector.h>
#include <map.h>
#elif _PowerMAXOS
#include <set>
#include <vector>
#include <map>
#endif

#include "pathpoint.h"

namespace CVED {
	
class CPathNetwork : public CCvedItem {

	public:
		// Types
		typedef vector<int> TLaneCrdrs;
		typedef vector<TLaneCrdrs> TLaneCrdrVec;

		// Public destructor
		~CPathNetwork(); 

		// Constructors and assignment operator
		CPathNetwork();
		CPathNetwork(const CPathNetwork&);
		CPathNetwork& operator=(const CPathNetwork &);
		explicit CPathNetwork(const CCved&);
		CPathNetwork(const CCved&, const CPoint2D&, double);

		// Functions
		void		SetRange(const CPoint2D&, double);
		
		int 		GetNumObjs() const;
		double		GetDistanceCovered() const;


		int			GetPathsTowards(vector<CPath>&,
									TLaneCrdrVec& laneCrdrVec,
									const CRoadPos&) const;

		// Miscellaneous
		friend ostream&	operator<<(ostream&, const CPathNetwork&);

	private:
		// Private helper functions
		bool RecursiveTowards(int srcIsec, int dstRoad,
							  CPath&, vector<CPath>&) const;
		static bool AlreadyContains(const CPath&, const CIntrsctn&);
		static bool AlreadyContains(const CPath&, const CRoad&);

		// Private data
		double			m_covered;
	
		typedef struct TDirPoint {
			CPathPoint pathPoint;	
			int srcIntrsctnId;	
			int dstIntrsctnId;	

			TDirPoint() { 
				srcIntrsctnId = -1; 
				dstIntrsctnId = -1; 
			};
		} TDirPoint;
		typedef struct TRoadPoint {
			TDirPoint	posPoint;
			TDirPoint	negPoint;
		} TRoadPoint;
		typedef pair<int, TRoadPoint> TRoadPointPair;
		typedef map<int, TRoadPoint> TRoadPointMap;
		typedef pair<int, CPathPoint> TIsecPointPair;
		typedef map<int, CPathPoint> TIsecPointMap;

		TRoadPointMap	m_roadPnts;		// Contains a CPathPoint for
										//	each direction of each 
										//	road covered by the square
										//	mapped to the road id.

		TIsecPointMap	m_intrsctnPnts;	// Contains a CPathPoint for
										//	each intersection covered
										//	by the square, mapped to
										//	the intrsctn id.
};

} // end namespace
#endif	// __PATH_NETWORK_H

