//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: road.h,v 1.45 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Yiannis Papelis
// Date:		October, 1998
//
// Description:	The implementation of the CRoad class
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __ROAD_H
#define __ROAD_H		// {secret}

#include "cvedpub.h"
#include <boundingbox.h>

////////////////////////////////////////////////////////////////////////
//
// forward declarations for the road structure.  We define them here
// so we can reference them but we don't include the actual 
// header file since it's private to CVED.
//
struct cvTCntrlPnt;
struct cvTRoad;
struct cvTRoadPiece;
struct cRoad;
/////////////////////////////////////////////////////////////////
///\brief Our Spatial DB system
///\remarks
///CVED this needs to be filled in.......................
///
///
///
///////////////////////////////////////////////////////////////
namespace CVED {

//typedef struct cvTCntrlPnt TCntrlPnt;
typedef struct cvTRoad TRoad;	///< {secret}
typedef struct cvTRoadPiece TRoadPiece;		///< {secret}

class CRoadPos;
class CLane;
class CIntrsctn;

/////////////////////////////////////////////////////////////////////////
/// \class CRoadPiece road.h
/// \author Yiannis Papelis
/// \date October, 1998
///
/// \brief This class represents a segment of a road
/// 
/// \remarks
///		This class represents a segment of a road whose geometry
///		is contained within a bounding rectangle.  It is a helper
///		class used that allow easier manipulation of roads by
///		breaking them up in smaller pieces that are geometrically
///		localized.
///\sa CRoad
////////////////////////////////////////////////////////////////////
class CRoadPiece : public CCvedItem {
public:
	CRoadPiece();
	~CRoadPiece();
	CRoadPiece( const CRoadPiece& );
	explicit CRoadPiece( const CCved& );
	CRoadPiece(
		const CCved&, 
		TRoadPoolIdx, 
		double, 
		double, 
		double, 
		double, 
		int, 
		int
		);
	CRoadPiece( const CCved&, TRoadPiecePoolIdx id );
	CRoadPiece&	operator=( const CRoadPiece& );

	void					Get2CntrlPnts( int* pFirst, int* pLast ); 
	int						GetRoad( void ) const;
	bool					HitTest( double x, double y ) const;
	CBoundingBox			GetBoundingBox( void ) const;

private:
	TRoadPoolIdx			m_road;   ///< identifier of road 
	TLongCntrlPntPoolIdx	m_first;  ///< indeces to first and last points
	TLongCntrlPntPoolIdx	m_last;   ///< road where the points are on
	double					m_x1;     ///< lower left of bounding box
	double					m_y1;     ///< lower left of bounding box
	double					m_x2;     ///< upper right of bounding box 
	double					m_y2;     ///< upper right of bounding box 
};
typedef vector<CRoadPiece> TRoadPieceVec;

/////////////////////////////////////////////////////////////////////////
/// \class	CRoad road.h
/// \author	Yiannis Papelis
/// \date	October, 1998
/// \brief	A logical rep of road in CVED
///\remarks 
///		This class represents a road in the virtual environment.
///		A road consists of a longitudinal spline that defines the
///		centerline.  Centered along the centerline are the lanes.
///		Each road must have one or more lanes.  The number of lanes
///		cannot change, but the actual width of the lanes can change
///		along the road.
///\par
///		Exactly two intersections are adjacent to each road, and they
///		are referred to as the source and destination intersections.
///\par
///		Each road can have attributes associated with it.  An attribute
///		is nothing more than a user defined code and two associated
///		doubleing point values.  Road attributes are used to specify
///		information about the whole, or part of a road.  Attributes
///		can be defined in the LRI file, or additional attributes 
///		can be added by the user through this class.
///		User specified attributes persist only for the duration
///		of execution of the program.
///
///\sa CRoadPos
///\sa CRoadPiece
/////////////////////////////////////////////////////////////////////
class CRoad  : public CCvedItem
{

public:
	CRoad();
	CRoad( const CRoad& );
	CRoad& operator=( const CRoad& rhs );
	virtual	~CRoad();
	explicit CRoad( const CCved& );

	CRoad( const CCved&, TU32b id );
	CRoad( const CCved&, const string& );

	double		GetLinearLength(void) const;
	double		GetCubicLength(void) const;

	int			GetNumLanes(void) const;
	int			GetLaneIdx(void) const;
	int			GetId(void) const;
	int			GetCntrlPntCount(void) const;
	int			GetCntrlPntIdx(void) const;
	///////////////////////////////////////////////////////
	///\brief data struct to hold control point info
	///////////////////////////////////////////////////////
	struct CtrlInfo {
		TPoint3D   pos;					///< position of control point
		TVector3D  norm;				///< normal at control point
		double      spline[4][3];		///< A,B,C,D for x, y, z
		double      cummLinDist;		///< cummulative road distance 
	};

	bool		GetCntrlPoint(int, int, vector<CtrlInfo> &) const;

	string		GetName(void) const;

	int			GetSourceIntrsctnIdx() const;
	int			GetDestIntrsctnIdx() const;

	CIntrsctn	GetSourceIntrsctn(void) const;
	CIntrsctn	GetDestIntrsctn(void) const;

	CRoadPos	Begin( const CLane* cpLane = 0 );
	CRoadPos	End( const CLane* cpLane = 0 );

	bool		ActiveAttr(
					const int id, 
					const double cDist = -1.0f
					) const;
	bool		QryAttr(
					const int id, 
					CAttr&,
					const int lanes = -1,
					const double cDist = -1.0f
					) const;
	bool		QryAttr(
					vector<CAttr>&	out, 
					const double	cD1=-1.0f, 
					const double	cD2=-1.0f
					) const;

	void		BreakUp(
					TRoadPieceVec &out, 
					double maxwid, 
					double maxhei
					) const;

	bool		IsHighway() const;

	bool		IsValid(void) const;

	friend 		ostream& operator<<( ostream&, const CRoad& );
	bool		operator==( const CRoad& );

private:		//use only BreakUp()
	void		FindBoundary(
					double*, 
					double*, 
					double*, 
					double*, 
					cvTCntrlPnt*
					) const; 

protected:
	void		AssertValid(void) const;

private:
	cvTRoad*	m_pRoad;///< This structure represents the road structure of lri file
};

typedef vector<CRoad> TRoadVec;
ostream& operator<<( ostream&, const CRoad& );

#include "road.inl"

}		// namespace CVED
#endif  // __ROAD_H
