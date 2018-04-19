//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:		$Id: pathpoint.h,v 1.29 2013/10/31 17:28:41 iowa\oahmad Exp $
//
// Author(s):	Jillian Vogel
// Date:		October, 1999
//
// Description:	The definition of the abstract CPath class
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __PATH_POINT_H
#define __PATH_POINT_H	// {secret}

#include "cvedpub.h"
#include "point2d.h"
#include <bitset>

#ifdef _WIN32
#include <list>
#elif __sgi
#include <list.h>
#elif _PowerMAXOS
#include <list>
#endif

#define cCV_STRAIGHT_TURN	0.0
#define cCV_LEFT_TURN		1.57079632679489661923	// pi/2
#define cCV_U_TURN			3.14159265358979323846	// pi
#define cCV_RIGHT_TURN		4.71238898038468985769	// 3*pi/2
#define	cCV_EPSILON			0.78539816339744830961	// pi/4

namespace CVED {
	
class CPathPoint : public CCvedItem {

	friend class CPath;
	friend class CPathNetwork;

	public:
		// Public destructor
		virtual	~CPathPoint(); 

		// Constructors and assignment operator
		CPathPoint();
		CPathPoint( const CPathPoint& );
		CPathPoint& operator=( const CPathPoint& );

		// Accessor functions
		bool        IsRoad() const;
		CRoad       GetRoad() const;
		CIntrsctn   GetIntrsctn() const;
		bitset<cCV_MAX_CRDRS>
					GetLaneCrdrMask() const;
		int         GetFirstLaneCrdrIdx() const;
		int         GetFirstCrdrWithLaneId(int id) const;
		int         GetFirstCrdrWithLaneIndex(int id) const;
		double      GetStartDist( int id = -1 ) const;
		double      GetEndDist( int id = -1 ) const;
		const double* GetStartDists() const;
		const double* GetEndDists() const;
		CLane       GetLane( int id = -1 ) const;
		CCrdr       GetCrdr( int id = -1 ) const;
		
	protected:			// Functions used by CPath
		explicit CPathPoint( const CCved& );
		CPathPoint( const CCved&, const string& );
		explicit CPathPoint( const CRoadPos& );

		CRoadPos    GetRoadPos( double t = 0.0, int id = -1 ) const;
		CIntrsctn   GetNextIntrsctn() const;
		CIntrsctn   GetPrevIntrsctn() const;
		CRoad       GetNextRoad( int id=-1 ) const;
		CRoad       GetPrevRoad( int id=-1 ) const;
	
		double      GetLength( int id = -1 ) const;

		string      GetString() const;

		// Mutator functions
		bool        Append( 
						double, 
						CRandNumGen*, 
						int, 
						CFastDeque<CPathPoint>&, 
						bool stayOnHighway = false 
						);
		double      Append( 
						double turnAngle, 
						double epsilon, 
						CFastDeque<CPathPoint>& extension, 
						double dist,
						bool laneChangeAllowed = true
						);
		bool        Append( const CPathPoint&, CFastDeque<CPathPoint>& );
		double      AppendNoLaneChange(
						CFastDeque<CPathPoint>& extension,
						const bitset<cCV_MAX_CRDRS>& cLaneCrdrMask
						);
		bool        Prepend( 
						double,	
						CRandNumGen*, 
						int, 
						CFastDeque<CPathPoint>&, 
						bool stayOnHighway = false 
						);
		bool        Prepend( double, double, CFastDeque<CPathPoint>& );
		bool        Prepend( const CPathPoint&, CFastDeque<CPathPoint>& );

		bool        SetString( const string& );

		bool		SetLaneMask( const bitset<cCV_MAX_CRDRS>& mask );

		// Other functions
		bool        Similar( const CPathPoint& ) const;
		bool        Adjacent( const CPathPoint& ) const;
		bool        Contains( const CRoad& ) const;
		bool        Contains( const CRoadPos& ) const;
	
		int         GetObjectsOnPoint( vector<int>& ) const;
		int         GetObjectsOnPoint( const CRoadPos&, vector<int>& ) const;
		int         GetObjectsOnPoint( 
								const CRoadPos&, 
								const int laneId,
								vector<int>& 
								) const;
		int         GetObjectsOnPoint(
								bitset<cCV_MAX_CRDRS>&, 
								vector<int>&
								) const;
		double      GetSrcDestAngle() const;
		bool        GetNextLaneClockwise( CLane& ) const;
		bool        GetNextLaneCounterClockwise( CLane& ) const;

		void		GetOutline(vector<CPoint2D>& points, double extra = 1.0f) const;

		// Inherited functions 
		bool        IsValid() const;
		void        AssertValid() const;

		// Operator functions
		friend ostream&	operator<<( ostream&, const CPathPoint& );
		bool			operator==( const CPathPoint& ) const;

	private:
		// Member data
		bool        m_isRoad;
		CRoad       m_road;
		CIntrsctn   m_intrsctn;
		bitset<cCV_MAX_CRDRS>
					m_laneCrdrMask;
		double      m_startDist[cCV_MAX_CRDRS];
		double      m_endDist[cCV_MAX_CRDRS];

		// Helper functions
		bool        Initialize( const CRoadPos& );
		double		AppendOnSameRoad( double dist );
		double		AppendEntireRoad( void );
		double		AppendOnSameIntrsctn( double dist );
		double		AppendEntireIntrsctn( void );
		bool        RecursiveAppend(
								const CPathPoint&, 
								const CPathPoint*, 
								CFastDeque<CPathPoint>&, 
								int numIntrsctnsHit, 
								int srcRoadIdx = -1
								);
		bool        RecursivePrepend(
								const CPathPoint&, 
								const CPathPoint*, 
								CFastDeque<CPathPoint>&, 
								int numIntrsctnsHit, 
								int dstRoadIdx = -1
								);
		bool        CheckAndMakeSamePoint(
								const CPathPoint&, 
								bool changeEnd = true
								);
		void        GetNextPathPoints(
								vector<CPathPoint>&, 
								int srcRoadIdx = -1
								) const;
		void        GetPrevPathPoints(
								vector<CPathPoint>&, 
								int dstRoadIdx = -1
								) const;

		void        SetRangeToStart( bool start, bool end );
		void        SetRangeToEnd( bool start, bool end );

		bool        SetIntrsctnPoint(
								const CIntrsctn&, 
								const CRoad&, 
								int,
								const CRoad&
								);
		bool        SetIntrsctnPoint(
								const CIntrsctn&, 
								const CRoad&, 
								const CRoad&,
								int
								);
		bool        SetRoadPoint( const CCrdr&, bool source=false );

		double      GetAngleBetween(
								const CIntrsctn&, 
								const CRoad&, 
								const CRoad&
								) const;
};

} // end namespace
#endif	// __PATH_POINT_H

