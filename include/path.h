/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:		$Id: path.h,v 1.47 2015/11/11 01:23:11 iowa\oahmad Exp $
//
// Author(s):	Jillian Vogel, Omar Ahmad
// Date:		October, 1999
//
// Description:	The definition of the abstract CPath class
//
/////////////////////////////////////////////////////////////////////////////
#ifndef __PATH_H
#define __PATH_H	// {secret}

#include <randnumgen.h>
#include "cvedpub.h"
#ifdef _WIN32
#include <list>
#elif __sgi
#include <list.h>
#elif _PowerMAXOS
#include <list>
#endif

#include "fastdeque.h"

#include "pathpoint.h"

namespace CVED {
	
class CPath : public CCvedItem {

	friend class CPathNetwork;

	public:
		// Constructors, destructor
		CPath();
		~CPath();
		CPath( const CPath& );
		CPath& operator=( const CPath& );

		explicit CPath( const CCved& );
		CPath( const CCved&, const CPoint3D& );
		explicit CPath( const CRoadPos& );

		void SetRandNumGen( CRandNumGen* pRng, int streamId );
		
		// Useful types
		typedef	int cTPathIterator;
		typedef int cTPathReverseIterator;
		typedef enum {	
			eCV_TRAVEL_ERROR = -2,		// miscelaneous error
			eCV_TRAVEL_NOT_FOUND = -1,	// initial point not found 
			eCV_TRAVEL_OK = 0,			// Travel ok
			eCV_TRAVEL_LANE_CHANGE = 1,	// Lane change requested
			eCV_TRAVEL_END_OF_PATH = 2	// End of path found
		} ETravelCode;

		// Accessor functions
		cTPathIterator	Begin() const;
		cTPathIterator	End() const;
		int				Size() const;
		bool			IsEmpty() const;
		double			GetLength( const CRoadPos* cpStart=0 ) const;
		double			GetLength( const CRoadPos*, const CRoadPos* ) const;
		int				GetNumIntrsctns( const CRoadPos* cpStart=0 ) const;
		int				GetNumRoads( const CRoadPos* cpStart=0 ) const;
		int				GetCurLaneId() const;
		void			GetString( vector<string>& ) const;
		CRoadPos		GetStart( int id = -1 ) const;
		CRoadPos		GetRoadPos( int num, int id = -1 ) const;
		CRoadPos		GetEnd( int id = -1 ) const;

		const CPathPoint& GetPoint( cTPathIterator itr ) const;

		// Mutator functions
		bool			Initialize( const CPoint3D& );
		bool			Initialize( const CRoadPos& );

		void			LimitLaneChange( const CRoadPos& );

		bool			Append( double, bool stayOnHighway = false );
		double			Append( 
							double turnAngle, 
							double epsilon, 
							double dist, 
							bool laneChangeAllowed = true
							);
		bool			Append( const CPoint2D& );
		bool			Append( const CRoadPos& );
		bool			Append( const CPathPoint& );
		bool			AppendNoLaneChange( const bitset<cCV_MAX_CRDRS>& );

		bool			Prepend( double, bool stayOnHighway = false );
		bool			Prepend( double, double );
		bool			Prepend( const CPoint2D& );
		bool			Prepend( const CRoadPos& );
		bool			Prepend( const CPathPoint& );

		void			PopFront();
		void			PopBack();
		void			Clear();

		bool			SetString( const vector<string>& );

		bool			SetLaneMask( const bitset<cCV_MAX_CRDRS>& cMask );
		bool			SetLaneMask( const CRoadPos& cRoadPos, const bitset<cCV_MAX_CRDRS>& cMask );

		// Other functions
		bool			operator==( const CPath& ) const;
		bool			Intersect( const CPath& ) const;
		bool			Adjacent( const CPath& ) const;
		bool			Contains( const CRoadPos& ) const;
		
		ETravelCode     GetRoadPos( 
								double dist, 
								const CRoadPos& cStartRoadPos, 
								CRoadPos& endRoadPos, 
								bool strictConnectivity = true 
								);
		bool            IsNextRoad( const CRoad& ) const;

		ETravelCode		Travel( 
							double, 
							const CRoadPos&, 
							CRoadPos&, 
							CLane&
							);
		ETravelCode		TravelBack( 
							double, 
							const CRoadPos&, 
							CRoadPos&, 
							bool strictConnectivity = true 
							);
		void			SwitchLane( int );
		void			SwitchLane( const CLane& );
		
		int				GetObjectsOnPath( vector<int>&) const;
		int				GetObjectsOnPath( 
									const CRoadPos&, 
									vector<int>&
									) const;
		int				GetObjectsOnPath( 
									const CRoadPos&, 
									bitset<cCV_MAX_CRDRS>&,
									vector<int>&
									) const;
		int				GetObjectsOnPathLane( 
									const CRoadPos&, 
									vector<int>&
									) const;
	
		CIntrsctn		GetNextIntrsctn( const CRoadPos& ) const;
		bool            IsIntrsctnTwoRoad( const CRoadPos& cRoadPos ) const;
		double			GetDistToNextIntrsctn( const CRoadPos& ) const;
		CCrdr			GetNextCrdr( 
									const CRoadPos& cRoadPos, 
									bool checkLane = true 
									) const;
		CHldOfs			GetNextHldOfs( const CRoadPos&, int *crdrIdOut = NULL , int lookahead = 1) const;
		double			GetDistToNextHldOfs( const CRoadPos& , int lookahead = 1) const;
		bool            GetCrdrFromIntrscn( 
									int cIntrsctnId,
									int& crdrId,
									const CRoadPos* cpStart,
									int srcLaneId = -1
									) const;
		
		int 			GetOncomingPath(
									const CRoadPos&, 
									vector<CPath>&
									) const;
		int				GetApproachingPaths( 
									const CRoadPos&, 
									vector<CPath>&
									) const;

		int				GetApproachingPathsLane( 
									const CRoadPos&, 
									vector<CPath>&
									) const;
		
		double          GetNextTurnAngle( const CRoadPos& ) const;
		bool            GetNextLaneClockwise( CLane& );
		bool            GetNextLaneCounterClockwise( CLane& );
		void            GetIntrsctnsWithinDist( 
									const CRoadPos&, 
									double, 
									vector<int>&, 
									bool skipDummyIntrsctns = false,
									bool checkForLastMergeDist = false
									) const;

		int             GetLaneIdFromRoad( 
									const CRoadPos* cpStartRoadPos, 
									const CRoad& cRoad
									) const;


		vector<CPoint2D> GetOutline( double extra = 1.0f ) const;

		bool CalculateRoute(CRoadPos& start, CRoadPos& end, double& totalDist, bool clear = true, bool shortest = false, int maxHeight = 25) ;
		bool AppendRoute(CRoadPos end, double& totalDist, bool shortest = false, int maxHeight = 25) ;

        bool            ValidatePath() const;
		// Inherited functions 
		bool			IsValid() const override;          

		// used for route calculation
		struct TRouteNode {
			// IDs for navigation
		
			//absolute ID
			int srcRoadIdx;
			int roadIdx;
			cvELnDir roadDir;

			// relative IDs
			int srcCrdrId;
			int srcLaneId;

			// length of segment
			double length;

			// tree info
			bool head;
			TRouteNode* parent;
			std::vector<TRouteNode> children;
			bool searched; // denotes whether or not node has searched for children

			// used for optimization
			double linearDistSquared;	// square of the distance to the destination
			int awayNodes;	// number of nodes in the current route that go away from the destination
		
			// if valid route found
			bool found;
		} ;

		// final route info
		// only holds important information for route navigation
		struct TRouteInfo {
			int roadIdx;	// necessary since other IDs are relative
			int laneId;
			int crdrId;
			double length; // length of segment or distance along lane in feet for the last segment
		} ;

		bool GetRouteChildren(int roadIdx, cvELnDir dir, CRoadPos& end, int height, TRouteNode* parent, int maxAway = -1) ;
		void GetAllNodesAtLevel(TRouteNode* parent, int level, std::vector<TRouteNode*>& out) ;
		void DiscardFarNodes(std::vector<TRouteNode*>& nodes, int numToKeep) ;

	private:
		void			AssertValid() const;

		// Private utility functions
		cTPathIterator  FindPathPoint( int idx = 0 ) const;
		cTPathIterator	FindPathPoint( 
									const CRoadPos&, 
									int& start
									) const;
		void			RemoveAnyExtraCorridors();

		// Member data
		CFastDeque<CPathPoint> m_points;

		int				m_curLaneId;
		int				m_pathPointIdx;

		// random nubmer generator
		CRandNumGen*    m_pRng;
		int             m_rngStreamId;
};	// class CPath

ostream& operator<<( ostream&, const CPath& );
                    
} // end namespace

#endif	// __PATH_H

