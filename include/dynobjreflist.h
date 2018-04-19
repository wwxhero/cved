//////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: dynobjreflist.h,v 1.10 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description: The declaration of the CCved function related to the
// 	dynamic object reference list.
//
//////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#pragma warning(disable:4786)
#endif

#ifndef __DYN_OBJ_REF_LIST_H
#define __DYN_OBJ_REF_LIST_H

////////////////////  Data structures ////////////////////

//	This structure is used to store data from either a 
//	road or corridor control point, so that they may be
//	treated the same.
typedef struct {
	TPoint3D	location;
	TVector3D	rightVec;
	double		width;
	double		distance;
} TDorCntrlPnt;

// This data structure is used to store the previous position and 
//	orientation information for each live dynamic object.  This 
//	information is stored so that if an object does not move, it's
//	position with respect to the road database need not be recalculated.
//	Also, when objects do move, they don't tend to move far.  So the
//	search can be restricted to the area around where the object was
//	last found.
typedef struct {
	bool			valid;			// Is this a valid saved object?
	bool			same;			// Has the onject's location changed?
	CBoundingBox	boundBox;		// Object's previous bounding box
	cvTDynObjRef	*pDynObjRefs;	// Object's previous items in the 
									//  road reference list
} TSavedObjLoc;

TSavedObjLoc *m_pSavedObjLoc;

// These variables are pointers to the "dynamic object by road" 
//	efficienct data structures.  They help with making fast queries
//	for which objects are on a given road or intersection.
cvTRoadRef*		m_pRoadRefPool;
cvTIntrsctnRef*	m_pIntrsctnRefPool;
cvTDynObjRef*	m_pDynObjRefPool;

// This method is called by the Maintainer to refill the above
//	data structures with the current position of the obects.
void	FillByRoadDynObjList(void);

//  These are utility methods called by FillByRoadDynObjList()
int		FindDorIdx (TObjectPoolIdx objId);
void	DumpDynObjRefLists(ostream& out = cout);
void	LinkDorIntoList(TObjectPoolIdx dorIdx);
bool	IsObjOnRoadNetwork(const cvTObj* pObj);
int		SearchAroundPrevLocation(
			const string& spc,
			TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			double xSize, 
			double YSize, 
			double zSize,
			const CPoint3D cQuad[]
			);
int		SearchForward(
			const string& spc,
			TObjectPoolIdx objId,
			const CPoint3D&center,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPnt, 
			vector<TRoadPoolIdx>& foundRoads
			);
int		SearchBackward(
			const string& spc,
			TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPnt, 
			vector<TRoadPoolIdx>& foundRoads
			);
int		SearchRoadForward(
			const string& spc,
			TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPnt, 
			vector<TRoadPoolIdx>& foundRoads
			);
int		SearchRoadBackward(
			const string& spc,
			TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPnt, 
			vector<TRoadPoolIdx>& foundRoads
			);
int		SearchIsec(
			const string& spc,
			TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPnt, 
			vector<TRoadPoolIdx>& foundRoads
			);
bool	IsObjOnIntrsctn(
			const string& spc,
			const cvTIntrsctn* cpIntrsctn, 
			const CPoint3D& center, 
			double xSize, 
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			cvTDynObjRef& tmpDor,
			vector<TRoadPoolIdx>* pRoadSet = 0
			);
bool	IsObjOnCorridor(
			const string& spc,
			const cvTCrdr* cpCrdr,
			TCrdrCntrlPntPoolIdx	  startCntrlPntIdx,
			TCrdrCntrlPntPoolIdx   endCntrlPntIdx,
			const CPoint3D& cCenter, 
			double xSize, 
			double ySize, 
			const CPoint3D cQuad[],
			cvTDynObjRef& tmpDor, 
			TRoadPoolIdx& roadId
			);
bool	IsObjOnRoadPiece(
			const string& spc,
			const cvTRoadPiece* cpRdPc,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			cvTDynObjRef& tmpDor
			);
bool	IsObjOnRoadSegment(
			const string& spc,
			const cvTCntrlPnt* cpCurCp,
			const cvTCntrlPnt* cpNextCp,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			const CPoint3D cQuad[],
			cvTDynObjRef& tmpDor, 
			bool& nextSeg, 
			bool& prevSeg
			);
bool ObjectOverlapsSegmentOnRoad(
			const string& spc,
			const TDorCntrlPnt* cpCurCp, 
			const TDorCntrlPnt* cpNextCp,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			const CPoint3D cQuad[],
			double& distance, 
			double& minOffset2, 
			double& maxOffset2,
			bool& checkPrevSeg, 
			bool& checkNextSeg
			);
bool ObjectOverlapsSegmentOnCrdr(
			const string& spc,
			const TDorCntrlPnt* cpCurCp, 
			const TDorCntrlPnt* cpNextCp,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			const CPoint3D cQuad[],
			double& distance, 
			double& minOffset2, 
			double& maxOffset2,
			bool& checkPrevSeg, 
			bool& checkNextSeg
			);

#endif	// __DYN_OBJ_REF_LIST_H

