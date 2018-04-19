//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: $Id: roadpos.h,v 1.66 2016/08/16 22:20:37 IOWA\dheitbri Exp $
//
// Description: Interface for the CRoadPos class.
//
//////////////////////////////////////////////////////////////////////
#ifndef __ROAD_POS_H
#define __ROAD_POS_H	// {secret}

#include <randnumgen.h>
#include "cvedpub.h"
#include "path.h"

////////////////////////////////////////////////////////////////////////
//
// forward declarations for the structure.  We define them here
// so we can reference them but we don't include the actual 
// header file since it's private to CVED.
struct cvTLane;
struct cvTRoad;
struct cvTCntrlPnt;
struct cvTHeader;
struct cvTAttr;

const double cROAD_POS_EPS = 16.0;

namespace CVED {

// {secret}
typedef struct cvTRoad TRoad;
typedef struct cvTLane TLane;	
typedef struct cvTCntrlPnt TCntrlPnt;
typedef struct cvTCrdrCntrlPnt TCrdrPnt;
typedef struct cvTHeader THeader;
typedef struct cvTAttr TAttr;
typedef TLongCntrlPntPoolIdx TCntrlPntIdx;

class CRoadPos  : public CCvedItem
{
public:
	// Common constructors, destructor, anad assignment op
	CRoadPos();
	CRoadPos(const CRoadPos&);
	CRoadPos& operator=(const CRoadPos&);
	virtual ~CRoadPos();
	explicit CRoadPos(const CCved&);

	// Road constructors
	CRoadPos(const CRoad&, 
			 int 	lane=0,
			 double	dist=0.0, 
			 double	ofs=0.0);

	CRoadPos(const CRoad&, 
			 const CLane&, 
			 double dst=0.0, 
			 double ofs=0.0);

	// Intersection constructors
	CRoadPos(const CIntrsctn&,
			 int	crdr = 0,
			 double 	dist = 0.0,
			 double 	ofs = 0.0); 

	CRoadPos(const CIntrsctn&intrsctn,
			 const CCrdr&	cCrdr,
			 double	dist = 0.0,
			 double	ofs = 0.0); 

	// Constructors for road or intersection
	CRoadPos(const CCved&	cCved,
			 const string&	cName,
			 int	id,
			 double	dist = 0.0,
			 double	ofs = 0.0, 
			 bool	isRoad = true); 

	CRoadPos(const CCved&, 
			 const CPoint3D&);

	CRoadPos(const CCved&, 
			 const string&);

	// Indicates the result of traveling along a road using the 
	// CRoadPos structure

	enum ETravelResult { eWITHIN_ROAD, ePAST_ROAD, eWITHIN_CRDR, eERROR };
    enum ETravelTurnDir{ eSTRAIGHT = 0, //take the straight crdr when travelling if you can 
                         eRIGHT = 1,    //take the right crdr when travelling if you can
                         eLEFT = -1,     //take the left crdr when travelling if you can
                         eDEFAULT   //currently does the same thing as eSTRAIGHT
                        };

	// Accessors	
	double			GetRoadLengthLinear() const;
	bool			IsRoad(void) const;
	CLane			GetLane(void) const;
	CLane			GetSplineLane(void) const;
	CRoad			GetRoad(void) const;
	CCrdr			GetCorridor(int id=-1) const;
	CCrdr			GetNextCrdr(int& crdrId, ETravelTurnDir& dirOut, ETravelTurnDir eTurnDir = eSTRAIGHT, const int laneIdx = -1) const;
	CCrdr			GetNextCrdrById(int crdrId, ETravelTurnDir& out) const;
	CCrdr			GetPrevCrdr(int& crdrId, ETravelTurnDir& dirOut, ETravelTurnDir eTurnDir = eSTRAIGHT, const int laneIdx = -1) const;
	CCrdr			GetPrevCrdrById(int crdrId, ETravelTurnDir& out) const;
    CCrdr           GetCorridorWithAbsoluteId(int id  = -1) const;
	bool			GetCorridors(vector<int> &crds) const;
    bool            GetCorridors(vector< pair<int,double> >& crds) const;	
	bool			HasCorridor(int) const;
    bool			HasCorridorWithAbsoluteId(int) const;
	CIntrsctn		GetIntrsctn(void) const;
	CIntrsctn		GetNextIntrsctn(void) const;
	CIntrsctn		GetPrevtIntrsctn(void) const;
	double			GetDistanceToNextIntrsctn(void) const;
	CHldOfs			GetNextHldOfs(int *crdrIdOut = NULL , int lookahead = 1) const;
	double			GetDistToNextHldOfs(int lookahead = 1) const;
	double			GetDistance(int id=-1) const;
	double          GetDistanceOnLane(int id=-1) const;
	double			GetLaneLength(int id=-1) const;
	double			GetOffset(int id=-1, const CPath *cpPath = NULL,int* pCorrUsed = NULL) const;
	double			GetSplineOffset(int id=-1, const CPath *cpPath = NULL,int* pCorrUsed = NULL) const;
	bool            GetRoadPosInOppositeDir( CRoadPos& ) const;

	string			GetName(void) const;
	string			GetString(void) const;

	bool			GetAttribute(const int, CAttr&) const;
    bool            GetIsOnRamp();
    bool            GetIsOffRamp();
    CPoint3D		GetVeryBestXYZ(bool useoffset = true) const;
    CPoint3D		GetBestXYZ(void) const;
	CPoint3D		GetXYZ(void) const;

	CVector3D		GetTangent(bool cubic = false) const;
	CVector3D       GetTangentInterpolated(bool cubic = false) const;
	CVector3D		GetRightVec(bool cubic = false) const;
	CVector3D		GetRightVecInterpolated(bool cubic = false) const;

	double			GetGrade(bool bInDegrees = false) const;

	double			GetElevation() const;

	double			GetLateralDistance(const CRoadPos&) const;
	double			GetLateralDistance(const CLane&) const;

	double			GetCurvature(void) const;
	double			GetCurvatureInterpolated(void) const;
	double			GetExpandedCurvature(void) const; // signed curvature
	void			GetRoadMarking(int &leftMarking, int &rightMarking, int &rightAtrib, int &leftAtrib) const;

	bool			IsValid(void) const;
	bool			operator==(const CRoadPos&) const;

	// Mutators
	bool			SetRoad(const CRoad&, const CLane&);
	bool			SetLane(const CLane&);
	bool			SetIntrsctn(
							const CIntrsctn&, 
							const CCrdr&, 
							double dist = 0.0,
							double offset = 0.0
							);
	bool			SetCorridor(
							const CCrdr&,
							double dist = 0.0,
							double offset = 0.0
							);
	bool			SetCorridors(
							const CIntrsctn&, 
							const bitset<cCV_MAX_CRDRS>&, 
							const double *, 
							double offset = 0.0
							);

	ETravelResult	SetDistance(double);
	ETravelResult	DecrDistance(double);
	ETravelResult	IncrDistance(double);
	void            SetOffset(double);

	bool			ChangeLaneLeft(bool center = false);
	bool			ChangeLaneRight(bool center = false);

	bool			SetString(const string&);

	ETravelResult	operator+(double inc);
	ETravelResult	operator-(double dec);

	ETravelResult	Travel(double dist, const CLane* pDstLane = 0, const ETravelTurnDir eTurnDir = eDEFAULT);
	ETravelResult	TravelDir(double dist, const ETravelTurnDir eTurnDir = eSTRAIGHT, const int id = -1);
	ETravelResult	TravelCrdr(double dist, const int crdrId);

	double			Travel(bitset<cCV_NUM_ATTR_CHANGE>&) const;
	double			FindNext(bitset<cCV_NUM_ATTR_CHANGE>&);

	bool			SetXYZ(
							const CPoint3D&,
							const CCrdr* cpCrdr = 0,
							bool* pColdSearch = 0
							);							
	bool			SetXY(
							const CPoint2D&,
							const CCrdr* cpCrdr = 0,
							bool* pColdSearch = 0,
							const double cHeightThresh = cROAD_POS_EPS
							);

	bool			SetCrdPriority(const CCrdr* cpCrdr);
    bool            SetCrdPriority(int crdrId);
	bool			SetCrdPriorityByDir(CCrdr::ECrdrDirection dir);

								 
	// Structures for storing road/crdr segments
	typedef struct {
		bool			isRoad;

		// If isRoad, cntrlPntIdx variables store
		// 	control point indexes relative to pRoad.
		// If !isRoad, cntrlPntIdx variables store
		// 	control point indexes relative to pCrdr.
		TLongCntrlPntPoolIdx  begCntrlPntIdx;
		TLongCntrlPntPoolIdx  endCntrlPntIdx;

		TRoad*			pRoad;
		cvELnDir		direction;

		TCrdr*			pCrdr;
		TIntrsctn*		pIntrsctn;

	} TSegment;

	double			GetSpeedLimit(int id = -1) const;

	//// used for route calculation
	//struct TRouteNode {
	//	// IDs for navigation
	//	
	//	//absolute ID
	//	int roadIdx;
	//	cvELnDir roadDir;

	//	// relative IDs
	//	int srcCrdrId;
	//	int srcLaneId;

	//	// length of segment
	//	double length;

	//	// tree info
	//	bool head;
	//	TRouteNode* parent;
	//	std::vector<TRouteNode> children;
	//	bool searched; // denotes whether or not node has searched for children

	//	// used for optimization
	//	double linearDistSquared;	// square of the distance to the destination
	//	int awayNodes;	// number of nodes in the current route that go away from the destination
	//	
	//	// if valid route found
	//	bool found;
	//} ;

	//// final route info
	//// only holds important information for route navigation
	//struct TRouteInfo {
	//	int roadIdx;	// necessary since other IDs are relative
	//	int laneId;
	//	int crdrId;
	//	double length; // length of segment or distance along lane in feet for the last segment
	//} ;

	//bool CalculateRoute(CRoadPos& end, vector<TRouteInfo>& out, double& totalDist, bool shortest = false, int maxHeight = 25) ;

protected:
	void			AssertValid(void) const;
	int				Search(
							double dist, 
							const TCrdr* pCrdr=0
							) const;
	void			GetRoadSegments(
							vector<TSegment>&,
							double distCovered, 
							const CCrdr* cpCrdr = 0
							);
	void			GetCrdrSegments(
							vector<TSegment>&,
							double distCovered,
							const CCrdr* cpCrdr = 0
							);

	bool			FindPointInIntrsctn(
							const CPoint3D&, 
							int id,
							const double cHeightThresh = cROAD_POS_EPS
							);

	bool			FindPoint(
							const CPoint3D&, 
							const vector<TSegment>&,
							const double cHeightThresh = cROAD_POS_EPS
							);
	bool			FindRoadPoint(
							const CPoint3D&, 
							const TSegment&,
							const double cHeightThresh = cROAD_POS_EPS
							);
	bool			FindIntrsctnPoint(
							const CPoint3D&, 
							const TSegment&,
							const double cHeightThresh = cROAD_POS_EPS
							);
	double			ComputerCrdrOffset(int id=-1, const CPath *cpPath = NULL,int* pCorrUsed = NULL) const;

	//bool GetRouteChildren(int roadIdx, cvELnDir dir, CRoadPos& end, int height, TRouteNode* parent, int maxAway = -1) ;
	//void GetAllNodesAtLevel(TRouteNode* parent, int level, std::vector<TRouteNode*>& out) ;
	//void DiscardFarNodes(std::vector<TRouteNode*>& nodes, int numToKeep) ;

private:
	bool			m_isRoad;		// If true, use road data
									// Else, use intersection data
	//  Road data
	TRoad*			m_pRoad;		// Pointer to the road structure
	TLane*			m_pLane;		// Pointer to the lane structure
	TLane*			m_pSplineLane;	// Pointer to the lane structure (when using a spline for the ofs)
	double			m_dist;			// Distance along road
	double			m_ofs;			// Offset from center of lane
	double			m_splineOfs;	// Offset from center of lane
	TCntrlPntIdx	m_cntrlPntIdx;	// Index of the road control point

	//  Intersection data
	typedef struct TCrdrDistOfs {
		TCrdr*			pCrdr;		// Pointer to the corridor structure
		double			dist;		// Distance along corridor
		double			ofs;		// Offset from center of corridor.
		TCntrlPntIdx	cntrlPntIdx;// Index of the crdr control point
	} TCdo;
	
	TIntrsctn*		m_pIntrsctn;	// Pointer to the intrsctn structure
	vector<TCdo>	m_cdo;			// Vector of corridor-dist-offsets.

#define cSEARCH_DISTANCE 20.0f
									// Distance along road/corridor to 
									//	search for the next point.
	// random nubmer generator
	CRandNumGen*    m_pRng;
	int             m_rngStreamId;
};

ostream&	operator<<(ostream&, const CRoadPos&);

/////////////////////////////////////////////////////////////////////////////
//
// Inline member functions
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: Default constructor
//
// Remarks: Creates an invalid CRoadPos instance.
// 
inline
CRoadPos::CRoadPos() 
	: 
	CCvedItem(0), 
	m_isRoad(true),
	m_pRoad(0), 
	m_pLane(0), 
	m_dist(0.0), 
	m_ofs(0.0), 
	m_splineOfs(0.0),
	m_cntrlPntIdx(0),
	m_pIntrsctn(0),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{}


//////////////////////////////////////////////////////////////////
//
// Description:  Constructor that initializes CCvedItem
//
// Remarks: Creates a CRoadPos with a valid, underlying CCvedItem
// 	instance, but with an invalid CRoadPos.
//
// Arguments:
//	cved - instance of CCved
//
//////////////////////////////////////////////////////////////////////
inline
CRoadPos::CRoadPos(const CCved& cCved) 
	: 
	CCvedItem(&cCved), 
	m_isRoad(true),
	m_pRoad(0), 
	m_pLane(0), 
	m_dist(0.0), 
	m_ofs(0.0), 
	m_splineOfs(0.0),
	m_cntrlPntIdx(0),
	m_pIntrsctn(0),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{}
		
//////////////////////////////////////////////////////////////////
//
// Description: copy constructor
//
// Remarks:
// This constructor initializes a CRoadPos
//
// Arguments:
//  copy - the copy of a class to initialize the current instance
//
//////////////////////////////////////////////////////////////////////
inline
CRoadPos::CRoadPos(const CRoadPos &cCopy) 
{
	*this = cCopy;
}
		
//////////////////////////////////////////////////////////////////
//
// Description: (private) Asserts that the local road and lane
// 	(or intersection and corridor) variables are set.
//
// Remarks:
// 	This function is used by the accessor functions of CRoadPos
//
//////////////////////////////////////////////////////////////////////
inline void
CRoadPos::AssertValid(void) const
{
    CCvedItem::AssertValid();
	if (m_isRoad) {
    	if ( (m_pRoad==0)||(m_pLane == 0) )
		{
			string msg = "CRoadPos::AssertValid: null road or lane";
			cvCInternalError e(msg, __FILE__, __LINE__);
			throw e;
		}
	}
	else {
    	if ( (m_pIntrsctn ==0)||(m_cdo.empty()) )
		{
			string msg = "CRoadPos::AssertValid: null intersection";
			cvCInternalError e(msg, __FILE__, __LINE__);
			throw e;
		}
	}
}


///////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is valid
// 
// Remarks: This function may be used by outside elements which want to 
// 	insure that the current instance is valid, without risking a failed 
// 	assertion.
//
// Returns: true or false  
// 	
inline bool
CRoadPos::IsValid( void ) const
{
    if( !CCvedItem::IsValid() )  return false;

	if( m_isRoad )
		return ( (m_pRoad != 0) && (m_pLane != 0) );
	else
		return ( (m_pIntrsctn != 0) && (!m_cdo.empty()) );
}

}	// namespace CVED

#endif // __ROAD_POS_H
