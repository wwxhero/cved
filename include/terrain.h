//////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: $Id: terrain.h,v 1.11 2003/12/04 16:32:41 schikore Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description: The declaration of the CCved function related to the
// 	terrain functions.
//
//////////////////////////////////////////////////////////////////////
void			GetFourCornerOfObj(
					TObjectPoolIdx			id, 
					CPoint2D*				pFourCorners,
					const CPoint3D*			cpPosition = 0,
					const CVector3D*		cpTangent = 0,
					const CVector3D*		cpLateral = 0) const;

void			GetFourCornerOfObj(
					TObjectPoolIdx			id, 
					CPoint3D*				pFourCorners,
					const CPoint3D*			cpPosition = 0,
					const CVector3D*		cpTangent = 0,
					const CVector3D*		cpLateral = 0) const;

bool			IfCollideWithStaticObj(
					double*                  pZ,
					CVector3D&              normal,
					const CPoint3D&         cIn,
					TRoadPoolIdx            roadId,
					TLongCntrlPntPoolIdx    cntrlPntIdx,
					const CVector3D&		cTangent,  //tan along the 
														//spline
					const CVector3D&		cLateral,	//lat along the 
														//spline on
					const CVector3D&		normalOnPoint,	 
					int*					pIfTrrnObjUsed,
					int*					pMaterial);

bool			IfCollideWithStaticObj(
					double*                  pZ,
					CVector3D&              normal,
					const CPoint3D&         cIn,
					const TIntrsctn*		cpInter,
					int*					pIfTrrnObjUsed,
					int*					pMaterial);

bool			IfCollideWithRepObj(
					double*					pZ,
					CVector3D&				normal,
					const CPoint3D&			cpIn,
					TRoadPoolIdx			roadId,
					TLongCntrlPntPoolIdx	cntrlPntIdx,
					const CPoint3D&			cPointOnSpline,
					double					tPar,
					const CVector3D&		cTangent,  //tan along the 
														//spline
					const CVector3D&		cLateral,	//lat along the 
														//spline on
					const CVector3D&		normalOnPoint,	 
					int*					pIfTrrnObjUsed,
					int*					pMaterial);

CPoint2D		GetLocalCoordinate(
					const CVector3D&		cTan,
					const CVector3D&		cLat,
					const CPoint3D&			cQueryPt,
					const CPoint3D&			cObjLocation);

bool QryTerrainRoadPiece( 
					TRoadPoolIdx			roadId, 
					TLongCntrlPntPoolIdx	cPta,
					TLongCntrlPntPoolIdx	cPtb, 
					const CPoint3D&			cIn,
					double&					zout, 
					CVector3D&				norm, 
					CTerQueryHint*			pHint,
					int*					pIfTrrnObjUsed,
					int*					pMaterial);

bool QryTerrainRoadPiece( 
					TRoadPoolIdx			roadId, 
					TLongCntrlPntPoolIdx	cPta,
					TLongCntrlPntPoolIdx	cPtb, 
					const CPoint3D&			cIn,
					double&					zout, 
					CVector3D&				norm, 
					CVector3D&				tan,
					CTerQueryHint*			pHint,
					int*                   pIfTrrnObjUsed,
					int*                    pMaterial);

bool QryTerrainRoadPieceUseHint(
					const CPoint3D&, 
					double&, 
					CVector3D&, 
					CTerQueryHint*,
					int*,
					int*);

/*
bool QryTerrainIntersection(
					const TIntrsctn*, // which intersection to search
					const CPoint3D&,  // query point
					double&,           // output z
					CVector3D&,       // normal
					int*,
					int*);            //material	
*/

bool QryTerrainOffRoad(
			        double*          pZ,
			        CVector3D&      normal,
			        const CPoint3D& cIn,
			        int*            pIfTrrnObjUsed,
			        int*            pMaterial);
bool IntersectWithTerrainObj(
					int				objId, 
					double*			pZ, 
					CVector3D&		normal, 
					const CPoint3D& cIn,
					int*			pIfTrrnObjUsed,
					int*			pMaterial);
bool IntersectWithTerrainObjOffRoad(
					int             objId,
					double*          pZ,
					CVector3D&      normal,
					const CPoint3D& cIn,
					int*            pIfTrrnObjUsed,
					int*            pMaterial,
					double*            pMaxElev);



TLongCntrlPntPoolIdx
	GetCntrlPntBehind(TRoadPoolIdx, TLongCntrlPntPoolIdx);
		
TLongCntrlPntPoolIdx
	GetCntrlPntAfter(TRoadPoolIdx, TLongCntrlPntPoolIdx);

void OrientVectorAlongXYZ(
	const CVector3D& tanVec, 
	const CVector3D& norVec, 
	CVector3D& vec) const;

void OrientVectorAlongXY(
	const CVector3D& norVec, 
	CVector3D& vec) const;
