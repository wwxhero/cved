//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: terrain.cxx,v 1.75 2016/07/15 14:32:53 IOWA\dheitbri Exp $
//
// Author(s):	Imran A. Pirwani
// Date:		January, 1999
//
// Description: The implementation of QryTerrain(...) related ideas that are
// 	part of the Cved class.
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
// {secret}
// This structure is used to keep track of potential results.  In order
// to deal with overlapping terrain, we keep track of all possible
// answers and then pick the best.
// We keep track of the output z, the output material, and the 
// hint that should be returned to the user if that choice is picked.
//
// The definition is private to this file
//////////////////////////////////////////////////////////////////////////////
struct Result {
	double					z;
	CVector3D				norm;
	int						mat;
	CCved::CTerQueryHint	hint;
	CCved::EQueryCode		code;
};


void 
CCved::SetNullTerrainQuery(
		const double&     zout,
		const double&  i,
		const double&  j,
		const double&  k,
		int*          pMaterial)
{
	m_NullTerrQuery = true;
	m_NullQueryZ = zout;
	m_NullQueryNorm.m_i = i;
	m_NullQueryNorm.m_j = j;
	m_NullQueryNorm.m_k = k;
	if ( pMaterial ) 
		m_NullQueryMaterial = *pMaterial;
	else
		m_NullQueryMaterial = 0;
}

void CCved::UnsetNullTerrainQuery(void)
{
	m_NullTerrQuery = false;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCntrlPntBehind (private)
// 	Find a control point located "behind" another one
//
// Remarks: This function finds a control point located at least X units
// 	behind another control point.  The function simply looks the the previous 
// 	control points while keeping track of the distance between the initial 
// 	and the current control point.  When that distance exceeds an arbitrary 
// 	constant, the function returns the new control point.  The first control 
// 	point of the road is returned if we hit the start of the road.
//
// 	NOTE:  I don't know why the constant 2.0 is used.  Maybe it should be a
// 	optional parameter?  - jvogel
//
// Arguments:
// 	roadId - identifier of road to search
// 	cntrlPntA - index of control point to start with
//
// Returns: The control point located at least 2.0 units behind cntrlPntA
//
//////////////////////////////////////////////////////////////////////////////
TLongCntrlPntPoolIdx
CCved::GetCntrlPntBehind( 
			TRoadPoolIdx			roadId,
			TLongCntrlPntPoolIdx	cntrlPntA)
{
	cvTRoad*				pRoad;
	cvTCntrlPnt*			pPnts;
	TLongCntrlPntPoolIdx	before;
	TLongCntrlPntPoolIdx	firstCp;

	pRoad = (cvTRoad *) ( ((char *)m_pHdr) + m_pHdr->roadOfs);
	pPnts = (cvTCntrlPnt *) ( ((char *)m_pHdr) + m_pHdr->longitCntrlOfs);
	firstCp = pRoad[roadId].cntrlPntIdx;

	for ( before = cntrlPntA; before > firstCp; before-- ) {
		if ( ( pPnts[cntrlPntA].cummulativeLinDist - 
			 pPnts[before].cummulativeLinDist ) > 2.0 ) {
			break;
		}
	}
	return before;
} // end of GetCntrlPntBehind

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCntrlPntAfter (private)
// 	Find a control point located "after" another one
//
// Remarks: This function finds a control point located at least X units
// 	after another control point.  The function simply looks the the next 
// 	control points while keeping track of the distance between the initial 
// 	and the current control point.  When that distance exceeds an arbitrary 
// 	constant, the function returns the new control point.  The last control 
// 	point of the road is returned if we hit the end of the road.
//
// 	NOTE:  I don't know why the constant 2.0 is used.  Maybe it should be a
// 	optional parameter?  - jvogel
//
// Arguments:
// 	roadId - identifier of the road to search
// 	cntrlPntB - control point before which to search
//
// Returns: The control point located at least 2.0 units after cntrlPntA
//
//////////////////////////////////////////////////////////////////////////////
TLongCntrlPntPoolIdx
CCved::GetCntrlPntAfter( 
			TRoadPoolIdx			roadId,
			TLongCntrlPntPoolIdx	cntrlPntB)
{
	cvTRoad*				pRoad;
	cvTCntrlPnt*			pPnts;
	TLongCntrlPntPoolIdx	after;
	TLongCntrlPntPoolIdx	lastCp;

	pRoad = (cvTRoad *) ( ((char *)m_pHdr) + m_pHdr->roadOfs);
	pPnts = (cvTCntrlPnt *) ( ((char *)m_pHdr) + m_pHdr->longitCntrlOfs);
	lastCp = pRoad[roadId].cntrlPntIdx + pRoad[roadId].numCntrlPnt - 1;	

	for ( after = cntrlPntB; after < lastCp; after++ ) {
		if ( ( pPnts[after].cummulativeLinDist - 
				pPnts[cntrlPntB].cummulativeLinDist ) > 2.0 ) {
            break;
        }
	}
	return after;
} // end of GetCntrlPntAfter

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrainRoadPieceUseHint (private)
// 	Compute terrain from roads using hint data
//
// Remarks: This function computes the terrain elevation as described
// 	by the road network (and objects), but it only looks near the location 
// 	specified by the hint.  If the road segment contained in the hint doesn't 
// 	work, the function tries segments located adjacent to the original hint.
//
// 	If the function succeeds, it updates the hint to contain the actual 
// 	control point that provided the answer
//
// Arguments:
// 	cIn - the 3D point to search for
// 	zout - the resulting elevation found at cIn
// 	norm - the resulting normal found at cIn
// 	pHint - pointer to the hint
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if the hint was useful, else false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::QryTerrainRoadPieceUseHint(
			const CPoint3D&	cIn,
			double&			zout,
			CVector3D&		norm,
			CTerQueryHint*	pHint,
			int*			pIfTrrnObjUsed,
			int*			pMaterial)
{
	TRoadPoolIdx			roadId;
	TLongCntrlPntPoolIdx	cPta;
	TLongCntrlPntPoolIdx	cPtb;

	//////////////////////////////////
	// Check to see if the pointer is valid.
	// Can't use hint so return false.
	//////////////////////////////////
	if (!pHint) {
		return false;
	}	
	
	//////////////////////////////////
	// if the hint is invalid then
	// return false.
	//////////////////////////////////
	if (pHint->m_hintState == CCved::eCV_OFF_ROAD) {
		return false;
	}

	//////////////////////////////////
	// Else pointer is good so, check 
	// its contents.  If the hint does
	// not point to a road piece, then
	// just return false.
	//////////////////////////////////
	if ( pHint->m_hintState != CCved::eCV_ON_ROAD ) {
		return false;
	}

	//////////////////////////////////
	// we know that the hint is pointing
	// to a road piece.  So, just
	// check to see if the point is
	// still inside that road piece. 
	//////////////////////////////////
	roadId = pHint->m_roadId;
	cPta = pHint->m_roadPiece;
	cPtb = pHint->m_roadPiece+1;
	if (QryTerrainRoadPiece(
						roadId, 
						cPta, 
						cPtb, 
						cIn, 
						zout, 
						norm, 
						pHint,
						pIfTrrnObjUsed,
						pMaterial)) {
		m_terQryRoadHits++;
		return true;
	}
	
	//////////////////////////////////
	// Could not succeed with the road
	// segment currently in the hint so,
	// search the segments in the 
	// neighborhood 
	//////////////////////////////////
	cPta = GetCntrlPntBehind( pHint->m_roadId, pHint->m_roadPiece );
	cPtb = GetCntrlPntAfter( pHint->m_roadId, pHint->m_roadPiece+1 );
	if (QryTerrainRoadPiece( 
						roadId, 
						cPta, 
						cPtb, 
						cIn,
						zout, 
						norm, 
						pHint,
						pIfTrrnObjUsed,
						pMaterial)) {
		m_terQryRoadHits++;
		return true;
	}

	//////////////////////////////////
	// could not succeed with the
	// road piece that the hint was
	// pointing to or for road pieces
	// in the neighbourhood. So, return
	// false.
	//////////////////////////////////
	return false;
} // end of QryTerrainRoadPieceUseHint

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrainIntersection (private)
// 	Find the terrain elevation and normal at the given point in the given
// 	intersection.
//
// Remarks:
//
// Arguments:
// 	cpIntrsctn - pointer to the intersection to search
// 	cIn - 3D point to search for
// 	zout - resulting elevation at cIn
// 	norm - resulting normal at cIn
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if the point was found, false otherwise
// 
//////////////////////////////////////////////////////////////////////////////
bool
CCved::QryTerrainIntersection( 
		const TIntrsctn*	cpIntrsctn,	// which intersection to check
		const CPoint3D&		cIn,	// query point
		double&				zout,	// output z
		CVector3D&			norm,
		int*				pIfTrrnObjUsed,
		int*				pMaterial)	// output normal
{
	// Determine if the query point is inside the border			
	if ( !m_intrsctnBndrs[cpIntrsctn->myId].Contains(cIn) ) 
		return false;

	// If it is flat, just use the data, otherwise query
	// the terrain elevation map.

	if ( cpIntrsctn->elevMap == 0 ) {
		zout     = cpIntrsctn->elevation;
		norm.m_i = norm.m_j = 0.0f;
		norm.m_k = m_pHdr->zdown ? -1.0f : 1.0f;
	}
	else {
		CTerrainGrid<Post> *pGrid = m_intrsctnGrids[cpIntrsctn->myId];
		if ( pGrid == 0 ) {
			cvCInternalError  err("Found NULL pointer.",
				__FILE__, __LINE__);
			throw err;
		}

		Post gridOut;		// the result of the query
		if ( pGrid->Query(cIn, gridOut, norm) ) {
			zout = gridOut.z;
			if (pMaterial)
				*pMaterial = gridOut.type;
		}
		else {
			return false;
		}
	}
#if 1
	if (cpIntrsctn->intrsctnFlag && eTERRN_OBJ) {
		try {
			IfCollideWithStaticObj(
							&zout, 
							norm, 
							cIn, 
							cpIntrsctn, 
							pIfTrrnObjUsed, 
							pMaterial);
		}
		catch( cvCInternalError ) { }
	}
#endif
	return true;
} // end of QryTerrainIntersection

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrainRoadPiece (private)
// 	Find elevation using data in a road piece.
// 
// Remarks: This function computes the terrain elevation using the road
// 	data contained in the road segments between cPta and cPta on the road 
// 	whose identifier is contained in roadId.
//
// 	The function computes the bounding box for all segments of the road piece 
// 	and tests the query point for containment in that segment.  Once the 
// 	segment that surrounds the query point is found, the function calculates 
// 	the elevation and normal vector, taking into account the profile, if one 
// 	is present.  Finally, the function sets the hint so next calls that are
// 	near the same query point don't have to search as far.
//
// Arguments:
//	roadId - the road to search
// 	cPta - the first control point in the range
// 	cPtb - the last control point
// 	cIn - the query point
// 	zout - the output elevation
// 	norm - the output normal vector
// 	pHint - pointer to the hint, could be NULL when no hint setting is needed
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if it found a segment that surrounds the query point, false 
// 	otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::QryTerrainRoadPiece( 
			TRoadPoolIdx			roadId, 
			TLongCntrlPntPoolIdx	cPta,
			TLongCntrlPntPoolIdx	cPtb, 
			const CPoint3D&			cIn,
			double&					zout, 
			CVector3D&				norm,
			CTerQueryHint*			pHint,
			int*					pIfTrrnObjUsed,
			int*					pMaterial)
{
	cvTCntrlPnt*			pPnts;
	TLongCntrlPntPoolIdx	pnt;
	bool					inside = false;
	bool					found = false;
	CPoint2D				segment[4];
	pPnts = (cvTCntrlPnt *) ( ((char *)m_pHdr) + m_pHdr->longitCntrlOfs);
	for ( pnt = cPta; pnt < cPtb; pnt++ ) {
		if ( pnt == cPta ) {
			GetSegment(&pPnts[pnt], &pPnts[pnt+1], segment);
		} else {
			segment[0] = segment[3];
			segment[1] = segment[2];	
			GetSegment(&pPnts[pnt+1], segment);
		}

    	///////////////////////////////////////////////////////
    	// Do point in sPoly for the point in and the polygon.
    	// if point in sPoly, then inside = true.
    	// WARNING:  This will not work for overlapping roads i.e.,
    	// overpasses.  For this to work for overpasses as well,
    	// we must search more and then do search in z to find the
    	// closest segment in space and not on the projection to
    	// ground.
    	///////////////////////////////////////////////////////

    	CPoint2D pt2D(cIn.m_x, cIn.m_y);
		inside = ( 
			pt2D.IsPointInTriangle(segment[0], segment[1], segment[2]) ||
			pt2D.IsPointInTriangle(segment[0], segment[2], segment[3]));

    	if ( inside ) {
      		found = true; 
    	}
    	if ( found ) {
      		break;
		}
	}
	if ( ! found ) {
		return false;
	}

  	///////////////////////////////////////////////////////////
  	// we must have found something.
  	// Determine the 't' parameter.
  	///////////////////////////////////////////////////////////

#if 1
	double tPar = ( pPnts[pnt].tangVecLinear.i * cIn.m_x -
         pPnts[pnt].tangVecLinear.i * pPnts[pnt].location.x +
         pPnts[pnt].tangVecLinear.j * cIn.m_y -
         pPnts[pnt].tangVecLinear.j * pPnts[pnt].location.y) /
         ( pPnts[pnt].tangVecLinear.i * pPnts[pnt].tangVecLinear.i +
         pPnts[pnt].tangVecLinear.j * pPnts[pnt].tangVecLinear.j);
#endif
#if 0
	// since the tangent is a unit vector, the dot product between
	// tangent and the vector from cur cntrlpnt to query point would
	// be the length along the tang direction
	double tPar = ( 
				pPnts[pnt].tangVecLinear.i * cIn.m_x -
       				pPnts[pnt].tangVecLinear.i * pPnts[pnt].location.x +
       			pPnts[pnt].tangVecLinear.j * cIn.m_y -
       				pPnts[pnt].tangVecLinear.j * pPnts[pnt].location.y +
       			pPnts[pnt].tangVecLinear.k * cIn.m_z -
       				pPnts[pnt].tangVecLinear.k * pPnts[pnt].location.z); 
#endif
	///////////////////////////////////////////////////////////
	// Normalize t to be in [0,1]...This can be removed if
	// natural cubic splines are used.
	///////////////////////////////////////////////////////////

	tPar /= pPnts[pnt].distToNextLinear;

	///////////////////////////////////////////////////////////
	// Next, get the point and the normal using this tPar.
	// We will use this point and normal to compute the point on
	// the base plane of the lateral curve and then compute the actual
	// height and the actual normal as a consequence of the non-flat
	// profile.
	///////////////////////////////////////////////////////////

	double tPar2 = tPar * tPar;
	double tPar3 = tPar2 * tPar;

#if 0
	CPoint3D cPointOnSpline(pPnts[pnt].hermite[0].A * tPar3 +
							pPnts[pnt].hermite[0].B * tPar2 +
                    		pPnts[pnt].hermite[0].C * tPar +
                    		pPnts[pnt].hermite[0].D,
                		pPnts[pnt].hermite[1].A * tPar3 +
                    		pPnts[pnt].hermite[1].B * tPar2 +
                    		pPnts[pnt].hermite[1].C * tPar +
                    		pPnts[pnt].hermite[1].D,
                		pPnts[pnt].hermite[2].A * tPar3 +
                    		pPnts[pnt].hermite[2].B * tPar2 +
                    		pPnts[pnt].hermite[2].C * tPar +
                    		pPnts[pnt].hermite[2].D);
#endif
#if 1
	CPoint3D cPointOnSpline( pPnts[pnt].location.x + tPar * 
							( pPnts[pnt+1].location.x - pPnts[pnt].location.x ),
							 pPnts[pnt].location.y + tPar * 
							( pPnts[pnt+1].location.y - pPnts[pnt].location.y ),
							 pPnts[pnt].location.z + tPar * 
							( pPnts[pnt+1].location.z - pPnts[pnt].location.z ) );
#endif

	CVector3D normalOnPoint( (pPnts[pnt].normal.i + tPar *
				( pPnts[pnt+1].normal.i - pPnts[pnt].normal.i ))/
				( pPnts[pnt].sn * tPar2 - pPnts[pnt].sn * tPar + 1 ),
				(pPnts[pnt].normal.j + tPar *
				( pPnts[pnt+1].normal.j - pPnts[pnt].normal.j ))/
				( pPnts[pnt].sn * tPar2 - pPnts[pnt].sn * tPar + 1 ),
				(pPnts[pnt].normal.k + tPar *
				( pPnts[pnt+1].normal.k - pPnts[pnt].normal.k ))/
				( pPnts[pnt].sn * tPar2 - pPnts[pnt].sn * tPar + 1 ));

	///////////////////////////////////////////////////////////
	// Right now, only the elevation and the normal on the plane is
	// returned.  At this point, I do not take the varying profile
	// into account...but this can be done soon.
	///////////////////////////////////////////////////////////

	double tVerticalDistance = cPointOnSpline.m_z -
          				( ( normalOnPoint.m_i * ( cIn.m_x - cPointOnSpline.m_x) +
            			normalOnPoint.m_j * ( cIn.m_y - cPointOnSpline.m_y ) ) /
         				normalOnPoint.m_k ) - cIn.m_z;

	zout = cIn.m_z + tVerticalDistance;
	norm = normalOnPoint;

	//////////////////////////////////////////////////////////////////////
	// !!! Calulation of the profile
	//////////////////////////////////////////////////////////////////////

	TLatCntrlPntPoolIdx FirstLatPnt = pPnts[pnt].latCntrlPntIdx;

	// calculate the tan and lat of the query point long the road
	CVector3D cTanVector(
				pPnts[pnt].hermite[0].A * tPar2 * 3 +
						pPnts[pnt].hermite[0].B * tPar * 2 +
                   		pPnts[pnt].hermite[0].C,
				pPnts[pnt].hermite[1].A * tPar2 * 3 +
                   		pPnts[pnt].hermite[1].B * tPar2 * 2 +
                   		pPnts[pnt].hermite[1].C,
              	pPnts[pnt].hermite[2].A * tPar2 * 3 +
                   		pPnts[pnt].hermite[2].B * tPar2 * 2 +
                   		pPnts[pnt].hermite[2].C);
	cTanVector.Normalize();
	CVector3D cLatVector( 
				pPnts[pnt].rightVecCubic.i + tPar *
					( pPnts[pnt+1].rightVecCubic.i - 
					pPnts[pnt].rightVecCubic.i ), 
				pPnts[pnt].rightVecCubic.j + tPar *
					( pPnts[pnt+1].rightVecCubic.j - 
					pPnts[pnt].rightVecCubic.j ),
				pPnts[pnt].rightVecCubic.k + tPar *
					( pPnts[pnt+1].rightVecCubic.k - 
					pPnts[pnt].rightVecCubic.k )); 
	cLatVector.Normalize();

	if ( FirstLatPnt != 0)
	{
		///////////////////////////////////////////////////////////////////
		// Calculation of the distance between the cIn point and the spline
		//////////////////////////////////////////////////////////////////

		double latDistance = sqrt(((cIn.m_x - cPointOnSpline.m_x) * 
			(cIn.m_x - cPointOnSpline.m_x) + 
			(cIn.m_y - cPointOnSpline.m_y) * (cIn.m_y - cPointOnSpline.m_y) + 
			(zout - cPointOnSpline.m_z) * (zout - cPointOnSpline.m_z)));
		
		//////////////////////////////////////////////////////////////////
		// Chek if the cIn point is on the left or on the right side
		// of the road.
		//////////////////////////////////////////////////////////////////

		CLineSeg2D CtrlPointLine(pPnts[pnt].location.x, pPnts[pnt].location.y,
			pPnts[pnt + 1].location.x, pPnts[pnt + 1].location.y);

		if (CtrlPointLine.IsPtOnLeftSide(cIn))
		{
			latDistance *= -1;
		}

		cvTLatCntrlPnt* pLatPnts;
		pLatPnts = (cvTLatCntrlPnt *) 
			(((char *)m_pHdr) + m_pHdr->latCntrlPntOfs);

		int numLatPnt = pPnts[pnt].numLatCntrlPnt;

		found = false;
		
		TLatCntrlPntPoolIdx LatPnt;

		//////////////////////////////////////////////////////////////////
		// Search in the profile between which two profile point the cIn 
		// point falls.
		//////////////////////////////////////////////////////////////////

		for ( LatPnt = FirstLatPnt;
			LatPnt < (numLatPnt + FirstLatPnt - 1);
			LatPnt++)
		{
			if ((latDistance >= pLatPnts[LatPnt].offset) && 
				(latDistance <= pLatPnts[LatPnt + 1].offset))
			{
				found = true;
				break;
			}
		}


		if (found)
		{
			if (pMaterial)
				*pMaterial = pLatPnts[LatPnt].material;
			//////////////////////////////////////////////////////////////
			// If the two profile points have the same high we simply 
			// add the elevation we don't recompute the normal.
			//////////////////////////////////////////////////////////////

			if ((fabs(pLatPnts[LatPnt + 1].height - 
				pLatPnts[LatPnt].height)) > 0.01)
			{
				//////////////////////////////////////////////////////////
				// Find the incline between the two profile points.
				//////////////////////////////////////////////////////////

				double incline = (pLatPnts[LatPnt + 1].height - 
					pLatPnts[LatPnt].height) / 
					(pLatPnts[LatPnt + 1].offset - pLatPnts[LatPnt].offset);

				double elevation = incline * latDistance - 
					incline * pLatPnts[LatPnt].offset + 
					pLatPnts[LatPnt].height;

				//////////////////////////////////////////////////////////
				// Computing the resulting elevation of the road.
				//////////////////////////////////////////////////////////

				zout += elevation;

				//////////////////////////////////////////////////////////
				// Compute the lateral vector. (lateral vector to the road)
				//////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////
				// Compute the vector between the two profile points.
				// To do that we find the exact position of the 
				// two profile points.
				// To find the coordinates of the two profile points we travel
				// from the center of the road in the direction of the lateral 
				// vector the distance D (distance between profile point and 
				// road center) and then we travel in the direction of 
				// the normal vector the distance H (elevation of the 
				// profile point).
				//////////////////////////////////////////////////////////

				CVector3D profVector((pLatPnts[LatPnt].offset - 
					pLatPnts[LatPnt + 1].offset) * cLatVector +
					(pLatPnts[LatPnt].height - 
					pLatPnts[LatPnt + 1].height) * normalOnPoint);

				//////////////////////////////////////////////////////////
				// Compute the tangent vector. (vector that goes in the 
				// direction of the road)
				// The tangent vector is the first derivate of the spline.
				//////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////
				// The result of the cross product of the tangent vector 
				// and the profile vector will be the new noprmal vector.
				//////////////////////////////////////////////////////////

				norm = cTanVector.CrossP(profVector);
				norm.Normalize();
			}
			else
			{
				//////////////////////////////////////////////////////////
				// The two profile points have the same elevation we only
				// compute the new elevation.
				// Is not necessaire to calculate the new normal vector.
				//////////////////////////////////////////////////////////
				zout += pLatPnts[LatPnt].height;
			}
		}
	}

	if ( (&pPnts[pnt])->cntrlPntFlag && eTERRN_OBJ){
		try {
			bool collideWithStaticObj	= IfCollideWithStaticObj(
													&zout, 
													norm, 
													cIn, 
													roadId, 
													pnt,
													cTanVector,
													cLatVector,
													normalOnPoint,
													pIfTrrnObjUsed,
													pMaterial);
		}
		catch( cvCInternalError ) { }
		try {
			bool collideWithRepObj		= IfCollideWithRepObj(
													&zout, 
													norm, 
													cIn, 
													roadId, 
													pnt,	
													cPointOnSpline, 
													tPar,
													cTanVector,
													cLatVector,
													normalOnPoint,
													pIfTrrnObjUsed,
													pMaterial);
		}
		catch( cvCInternalError ) { }
	}

	////////////////////////////////////////////////////////////
	// Now that we have found a road piece that the point
	// falls on, we must update the hint for subsequent calls.
	////////////////////////////////////////////////////////////

	if ( pHint ) {
		pHint->m_hintState = CCved::eCV_ON_ROAD;
		pHint->m_roadId = roadId;
		pHint->m_roadPiece = pnt;
	}

	return true;

} // end of QryTerrainRoadPiece

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrainRoadPiece (private)
// 	Find elevation using data in a road piece.  Also computes the tangent at
// 	cIn.
// 
// Remarks: This function computes the terrain elevation using the road
// 	data contained in the road segments between cPta and cPta on the road 
// 	whose identifier is contained in roadId.
//
// 	The function computes the bounding box for all segments of the road piece 
// 	and tests the query point for containment in that segment.  Once the 
// 	segment that surrounds the query point is found, the function calculates 
// 	the elevation and normal vector, taking into account the profile, if one 
// 	is present.  Finally, the function sets the hint so next calls that are
// 	near the same query point don't have to search as far.
//
// Arguments:
//	roadId - the road to search
// 	cPta - the first control point in the range
// 	cPtb - the last control point
// 	cIn - the query point
// 	zout - the output elevation
// 	norm - the output normal vector
// 	tan - the output tangent vector
// 	pHint - pointer to the hint, could be NULL when no hint setting is needed
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if it found a segment that surrounds the query point, false 
// 	otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::QryTerrainRoadPiece( 
			TRoadPoolIdx			roadId, 
			TLongCntrlPntPoolIdx	cPta,
			TLongCntrlPntPoolIdx	cPtb, 
			const CPoint3D&			cIn,
			double&					zout, 
			CVector3D&				norm,
			CVector3D&				tan,
			CTerQueryHint*			pHint,
			int*					pIfTrrnObjUsed,
			int*					pMaterial)
{
	cvTCntrlPnt*			pPnts;
	TLongCntrlPntPoolIdx	pnt;
	bool					inside = false;
	bool					found = false;
	CPoint2D				segment[4];
	pPnts = (cvTCntrlPnt *) ( ((char *)m_pHdr) + m_pHdr->longitCntrlOfs);
	for ( pnt = cPta; pnt < cPtb; pnt++ ) {
		if ( pnt == cPta ) {
			GetSegment(&pPnts[pnt], &pPnts[pnt+1], segment);
		} else {
			segment[0] = segment[3];
			segment[1] = segment[2];	
			GetSegment(&pPnts[pnt+1], segment);
		}

    	///////////////////////////////////////////////////////
    	// Do point in sPoly for the point in and the polygon.
    	// if point in sPoly, then inside = true.
    	// WARNING:  This will not work for overlapping roads i.e.,
    	// overpasses.  For this to work for overpasses as well,
    	// we must search more and then do search in z to find the
    	// closest segment in space and not on the projection to
    	// ground.
    	///////////////////////////////////////////////////////

    	CPoint2D pt2D(cIn.m_x, cIn.m_y);
		inside = ( 
			pt2D.IsPointInTriangle(segment[0], segment[1], segment[2]) ||
			pt2D.IsPointInTriangle(segment[0], segment[2], segment[3]));

    	if ( inside ) {
      		found = true; 
    	}
    	if ( found ) {
      		break;
		}
	}
	if ( ! found ) {
		return false;
	}

  	///////////////////////////////////////////////////////////
  	// we must have found something.
  	// Determine the 't' parameter.
  	///////////////////////////////////////////////////////////

#if 1
	double tPar = ( pPnts[pnt].tangVecLinear.i * cIn.m_x -
         pPnts[pnt].tangVecLinear.i * pPnts[pnt].location.x +
         pPnts[pnt].tangVecLinear.j * cIn.m_y -
         pPnts[pnt].tangVecLinear.j * pPnts[pnt].location.y) /
         ( pPnts[pnt].tangVecLinear.i * pPnts[pnt].tangVecLinear.i +
         pPnts[pnt].tangVecLinear.j * pPnts[pnt].tangVecLinear.j);
#endif
#if 0
	// since the tangent is a unit vector, the dot product between
	// tangent and the vector from cur cntrlpnt to query point would
	// be the length along the tang direction
	double tPar = ( 
				pPnts[pnt].tangVecLinear.i * cIn.m_x -
       				pPnts[pnt].tangVecLinear.i * pPnts[pnt].location.x +
       			pPnts[pnt].tangVecLinear.j * cIn.m_y -
       				pPnts[pnt].tangVecLinear.j * pPnts[pnt].location.y +
       			pPnts[pnt].tangVecLinear.k * cIn.m_z -
       				pPnts[pnt].tangVecLinear.k * pPnts[pnt].location.z); 
#endif
	///////////////////////////////////////////////////////////
	// Normalize t to be in [0,1]...This can be removed if
	// natural cubic splines are used.
	///////////////////////////////////////////////////////////

	tPar /= pPnts[pnt].distToNextLinear;

	///////////////////////////////////////////////////////////
	// Next, get the point and the normal using this tPar.
	// We will use this point and normal to compute the point on
	// the base plane of the lateral curve and then compute the actual
	// height and the actual normal as a consequence of the non-flat
	// profile.
	///////////////////////////////////////////////////////////

	double tPar2 = tPar * tPar;
	double tPar3 = tPar2 * tPar;

	CPoint3D cPointOnSpline(pPnts[pnt].hermite[0].A * tPar3 +
							pPnts[pnt].hermite[0].B * tPar2 +
                    		pPnts[pnt].hermite[0].C * tPar +
                    		pPnts[pnt].hermite[0].D,
                		pPnts[pnt].hermite[1].A * tPar3 +
                    		pPnts[pnt].hermite[1].B * tPar2 +
                    		pPnts[pnt].hermite[1].C * tPar +
                    		pPnts[pnt].hermite[1].D,
                		pPnts[pnt].hermite[2].A * tPar3 +
                    		pPnts[pnt].hermite[2].B * tPar2 +
                    		pPnts[pnt].hermite[2].C * tPar +
                    		pPnts[pnt].hermite[2].D);

	CVector3D normalOnPoint( (pPnts[pnt].normal.i + tPar *
				( pPnts[pnt+1].normal.i - pPnts[pnt].normal.i ))/
				( pPnts[pnt].sn * tPar2 - pPnts[pnt].sn * tPar + 1 ),
				(pPnts[pnt].normal.j + tPar *
				( pPnts[pnt+1].normal.j - pPnts[pnt].normal.j ))/
				( pPnts[pnt].sn * tPar2 - pPnts[pnt].sn * tPar + 1 ),
				(pPnts[pnt].normal.k + tPar *
				( pPnts[pnt+1].normal.k - pPnts[pnt].normal.k ))/
				( pPnts[pnt].sn * tPar2 - pPnts[pnt].sn * tPar + 1 ));

	///////////////////////////////////////////////////////////
	// Right now, only the elevation and the normal on the plane is
	// returned.  At this point, I do not take the varying profile
	// into account...but this can be done soon.
	///////////////////////////////////////////////////////////

	double tVerticalDistance = cPointOnSpline.m_z -
          				( ( normalOnPoint.m_i * ( cIn.m_x - cPointOnSpline.m_x) +
            			normalOnPoint.m_j * ( cIn.m_y - cPointOnSpline.m_y ) ) /
         				normalOnPoint.m_k ) - cIn.m_z;

	zout = cIn.m_z + tVerticalDistance;
	norm = normalOnPoint;

	//////////////////////////////////////////////////////////////////////
	// !!! Calulation of the profile
	//////////////////////////////////////////////////////////////////////

	TLatCntrlPntPoolIdx FirstLatPnt = pPnts[pnt].latCntrlPntIdx;

	// calculate the tan and lat of the query point long the road
	CVector3D cTanVector(
				pPnts[pnt].hermite[0].A * tPar2 * 3 +
						pPnts[pnt].hermite[0].B * tPar * 2 +
                   		pPnts[pnt].hermite[0].C,
				pPnts[pnt].hermite[1].A * tPar2 * 3 +
                   		pPnts[pnt].hermite[1].B * tPar2 * 2 +
                   		pPnts[pnt].hermite[1].C,
              	pPnts[pnt].hermite[2].A * tPar2 * 3 +
                   		pPnts[pnt].hermite[2].B * tPar2 * 2 +
                   		pPnts[pnt].hermite[2].C);
	cTanVector.Normalize();
	tan = cTanVector;
	CVector3D cLatVector( 
				pPnts[pnt].rightVecCubic.i + tPar *
					( pPnts[pnt+1].rightVecCubic.i - 
					pPnts[pnt].rightVecCubic.i ), 
				pPnts[pnt].rightVecCubic.j + tPar *
					( pPnts[pnt+1].rightVecCubic.j - 
					pPnts[pnt].rightVecCubic.j ),
				pPnts[pnt].rightVecCubic.k + tPar *
					( pPnts[pnt+1].rightVecCubic.k - 
					pPnts[pnt].rightVecCubic.k )); 
	cLatVector.Normalize();

	if (FirstLatPnt != 0)
	{
		//////////////////////////////////////////////////////////////////
		// Calculation of the distance between the in point
		// and the spline
		//////////////////////////////////////////////////////////////////

		double latDistance = sqrt(((cIn.m_x - cPointOnSpline.m_x) * 
			(cIn.m_x - cPointOnSpline.m_x) + 
			(cIn.m_y - cPointOnSpline.m_y) * (cIn.m_y - cPointOnSpline.m_y) + 
			(zout - cPointOnSpline.m_z) * (zout - cPointOnSpline.m_z)));
		
		//////////////////////////////////////////////////////////////////
		// Chek if the in point is on the left or on the right side
		// of the road.
		//////////////////////////////////////////////////////////////////

		CLineSeg2D CtrlPointLine(pPnts[pnt].location.x, pPnts[pnt].location.y,
			pPnts[pnt + 1].location.x, pPnts[pnt + 1].location.y);

		if (CtrlPointLine.IsPtOnLeftSide(cIn))
		{
			latDistance *= -1;
		}

		cvTLatCntrlPnt* pLatPnts;
		pLatPnts = (cvTLatCntrlPnt *) 
			(((char *)m_pHdr) + m_pHdr->latCntrlPntOfs);

		int numLatPnt = pPnts[pnt].numLatCntrlPnt;

		found = false;
		
		TLatCntrlPntPoolIdx LatPnt;

		//////////////////////////////////////////////////////////////////
		// Search in the profile between which two profile point the in point
		// falls.
		//////////////////////////////////////////////////////////////////

		for ( LatPnt = FirstLatPnt;
			LatPnt < (numLatPnt + FirstLatPnt - 1);
			LatPnt++)
		{
			if ((latDistance >= pLatPnts[LatPnt].offset) && 
				(latDistance <= pLatPnts[LatPnt + 1].offset))
			{
				found = true;
				break;
			}
		}


		if (found)
		{
			if (pMaterial)
				*pMaterial = pLatPnts[LatPnt].material;
			//////////////////////////////////////////////////////////////
			// If the two profile points have the same high we simply 
			// add the elevation we don't recompute the normal.
			//////////////////////////////////////////////////////////////

			if ((fabs(pLatPnts[LatPnt + 1].height - 
				pLatPnts[LatPnt].height)) > 0.01)
			{
				//////////////////////////////////////////////////////////
				// Find the incline between the two profile points.
				//////////////////////////////////////////////////////////

				double incline = (pLatPnts[LatPnt + 1].height - 
					pLatPnts[LatPnt].height) / 
					(pLatPnts[LatPnt + 1].offset - pLatPnts[LatPnt].offset);

				double elevation = incline * latDistance - 
					incline * pLatPnts[LatPnt].offset + 
					pLatPnts[LatPnt].height;

				//////////////////////////////////////////////////////////
				// Computing the resulting elevation of the road.
				//////////////////////////////////////////////////////////

				zout += elevation;

				//////////////////////////////////////////////////////////
				// Compute the lateral vector. (lateral vector to the road)
				//////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////
				// Compute the vector between the two profile points.
				// To do that we find the exact position of the 
				// two profile points.
				// To find the coordinates of the two profile points we travel
				// from the center of the road in the direction of the lateral 
				// vector the distance D (distance between profile point and 
				// road center) and then we travel in the direction of 
				// the normal vector the distance H (elevation of the 
				// profile point).
				//////////////////////////////////////////////////////////

				CVector3D profVector((pLatPnts[LatPnt].offset - 
					pLatPnts[LatPnt + 1].offset) * cLatVector +
					(pLatPnts[LatPnt].height - 
					pLatPnts[LatPnt + 1].height) * normalOnPoint);

				//////////////////////////////////////////////////////////
				// Compute the tangent vector. (vector that goes in the 
				// direction of the road)
				// The tangent vector is the first derivate of the spline.
				//////////////////////////////////////////////////////////

				//////////////////////////////////////////////////////////
				// The result of the cross product of the tangent vector 
				// and the profile vector will be the new noprmal vector.
				//////////////////////////////////////////////////////////

				norm = cTanVector.CrossP(profVector);
				norm.Normalize();
			}
			else
			{
				//////////////////////////////////////////////////////////
				// The two profile points have the same elevation we only
				// compute the new elevation.
				// Is not necessaire to calculate the new normal vector.
				//////////////////////////////////////////////////////////
				zout += pLatPnts[LatPnt].height;
			}
		}
	}

	if ((&pPnts[pnt])->cntrlPntFlag && eTERRN_OBJ){
		try {
			bool collideWithStaticObj	= IfCollideWithStaticObj(
													&zout, 
													norm, 
													cIn, 
													roadId, 
													pnt,
													cTanVector,
													cLatVector,
													normalOnPoint,
													pIfTrrnObjUsed,
													pMaterial);
		}
		catch( cvCInternalError ) { }
#if 0
		if (collideWithStaticObj){
			cerr<<"\n collid with staticObj\n";
		}else{
			cerr<<"\n don't collid with static Obj\n";
		}
#endif

		try {
			bool collideWithRepObj		= IfCollideWithRepObj(
													&zout, 
													norm, 
													cIn, 
													roadId, 
													pnt,	
													cPointOnSpline, 
													tPar,
													cTanVector,
													cLatVector,
													normalOnPoint,
													pIfTrrnObjUsed,
													pMaterial);
		}
		catch( cvCInternalError ) { }
#if 0
		if (collideWithRepObj){
			cerr<<"\n collid ewith repObj\n";
		}else{
			cerr<<"\n don't collid with repObj\n";
		}
#endif
	}

	////////////////////////////////////////////////////////////
	// Now that we have found a road piece that the point
	// falls on, we must update the hint for subsequent calls.
	////////////////////////////////////////////////////////////

	if ( pHint ) {
		pHint->m_hintState = CCved::eCV_ON_ROAD;
		pHint->m_roadId = roadId;
		pHint->m_roadPiece = pnt;
	}

	return true;
} // end of QryTerrainRoadPiece

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrain (private)
// 	Provide terrain elevation information
//
// Remarks: This function returns information about the terrain elevation
// 	in the virtual environment.  Terrain elevation is returned as an absolute 
// 	elevation (zout) along with the normal vector at the interrogation point 
// 	(norm).  The function searches all the roads, intersections and objects 
// 	that define terrain and if the interrogation point falls on any of these 
// 	elements it determines the elevation and normal vector.
//
// 	The function uses the following rules when searching for
// 	an element of the virtual environment to get terrain elevation.
// 	* The function first looks for any objects that affect terrain 
// 	elevation.  If the query point is within the outline of such
// 	an object, the function computes the elevation based on that
// 	object.  Then the function verifies that the vertical distance
// 	between the input z coordinate and output z is within a certain 
// 	tolerance and if that constraint is satisfied the function 
// 	returns that information.  If no object is found, or one (or more)
//	are found but are beyond the tolerance, the function proceeds with
// 	the road network.  Also, if an object is found but its elevation
// 	mode is _delta_, the function proceeds with looking for elevation
// 	from roads and intersections.  Keep in mind that repeated objects are 
// 	excluded from the search at this point.
// 
// 	* If no objects are found, the function looks on the road network and 
// 	intersections.  If the query is within an intersection or near 
//	enough to the road pavement, the elevation and normal are computed
// 	from that element.  If more than one road or intersection provide
// 	terrain information (as would be the case when having bridges or
// 	other overlapping terrain), the function first discards answers
// 	based on the vertical tolerance, then selects the answer that
// 	would yield the higher elevation.  The answer incorporates
// 	any modifications based on _delta_ terrain objects that were
// 	found in the earlier step.
//
// 	* As a last step, if the answer comes from the pavement of a road, the 
// 	function checks for any repeated objects that may affect the elevation.
// 	If any such object is found, the answer is modified based on the object's
// 	information and the final answer is returned.
//
// 	In addition to various internal data structures used to reduce the
// 	search time, the function can utilize a the pHing argument as a
// 	means to reduce even further the execution time of the function.
// 	The function stores critical information about the element that
// 	provided the answer given the current input.  This information
// 	can be used in later calls to provide a starting point for 
// 	the search and substantially reduce the search time, provided 
// 	that the new interrogation point is near the prior one.
// 	Keep in mind that in providing interrogations for the four wheels
// 	of a vehicle, using 4 different hint objects (one for each wheel) is
// 	more likely to have an impact than using a single hint for all wheels.
// 	In no case will use of a hint change the value that the function returns,
// 	in other words, the difference between using or not using a hint
// 	will only change the execution time of the function and not the
// 	results it returns.
//
// Arguments:
// 	cIn - the input x, y, and z coordinate of the query point.
// 		The input z coordinate is used to discriminate among vertically 
// 		overlaping terrain surfaces.
// 	zout - the output elevation
// 	norm - the normal vector at the interrogation point.  The normal vector
// 		is normalized (i.e., its length is 1.0)
// 	pHint - a pointer to a hint class.  On input, this class is consulted
// 		for the results of prior queries.  On output, the function updates 
// 		this class with information that would be useful to future queries.
// 		If the argument is 0, no hint is used.
// 	pIfTrrnObjUsed - (optional) contains the object id of the terrain 
//      object that was used to modify the terrain elevation, if any.
//      If no object was used for calculating the elevation, this argument
//      will be set to -1.
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: TOnRoad to indicate that a valid elevation was found based on 
// 	the road network.  The function returns TOnIntersect to indicate that a 
// 	valid elevation was found based on an intersection.  Finally, the function 
// 	returns TNoValue to indicate that no element of the virtual environment 
// 	could provide elevation for the specified point.  The value of the
// 	arguments zout, norm, and pHing are not changed when the function
// 	returns TNoValue.
//
//////////////////////////////////////////////////////////////////////////////
CCved::EQueryCode
CCved::QryTerrain(
			const CPoint3D&		cIn, 
			double&				zout, 
			CVector3D&			norm, 
			CTerQueryHint*		pHint,
			int*				pIfTrrnObjUsed,
			int*				pMaterial)
{

	if ( m_NullTerrQuery ) {
		zout = m_NullQueryZ;
		norm = m_NullQueryNorm;
		if ( pMaterial ) *pMaterial = m_NullQueryMaterial;
		if ( pIfTrrnObjUsed ) *pIfTrrnObjUsed = 0;
		return CCved::eCV_ON_ROAD;
	}

	if (pMaterial)
		*pMaterial      = 0;
	if (pIfTrrnObjUsed)
		*pIfTrrnObjUsed = -1;

	
	cvTRoad*		pRoad;
//	bool sInit = false;
	vector<Result>  possResults;	// possible results
	vector<int>  intrsctn;
	vector<int>  roadPieces;
	vector<int>::const_iterator  riter;
	vector<int>::const_iterator  iiter;
	//if (!sInit) {
		//possResults.reserve( 1000 );
		//intrsctn.reserve( 1000 );
		//roadPieces.reserve( 1000 );
		//sInit = true;
	//}
	//possResults.clear();
	//intrsctn.clear();
	//roadPieces.clear();

	// update performance data
	m_terQryCalls++;

	//////////////////////////////////////////
	// First see if the hint can be used to
	// speed up the process.  Check intersections first.
	////////////////////////////////////////
	if ( pHint && pHint->m_hintState == CCved::eCV_ON_INTRSCTN ) {
		cvTIntrsctn  *cpIntrsctn = BindIntrsctn(pHint->m_intersection);

		// if found, we simply return the answer
		if ( QryTerrainIntersection(
								cpIntrsctn, 
								cIn, 
								zout, 
								norm, 
								pIfTrrnObjUsed,
								pMaterial) ) {
			m_terQryInterHits++;
			return CCved::eCV_ON_INTRSCTN;
		}
	}

	//////////////////////////////////////////
	// Check use of hint for roads.
	//////////////////////////////////////////
	if (QryTerrainRoadPieceUseHint( 
								cIn, 
								zout, 
								norm, 
								pHint, 
								pIfTrrnObjUsed,
								pMaterial )){
		return CCved::eCV_ON_ROAD;
	}
	
	
	//////////////////////////////////////////
	// 
	// We were unable to use the hint on either a road or
	// an intersection so do a full search.  We start by looking 
	// at the intersections.  We use the quadtree to find
	// relevant intersections.
	//
	//////////////////////////////////////////

	m_intrsctnQTree.SearchRectangle(cIn.m_x-1, cIn.m_y-1, cIn.m_x+1, cIn.m_y+1,
				intrsctn);
	if ( intrsctn.size() != 0 ) {
		possResults.reserve( intrsctn.size() );
		for (iiter = intrsctn.begin(); iiter != intrsctn.end(); iiter++) {
			cvTIntrsctn  *cpIntrsctn = BindIntrsctn(*iiter);
			if ( QryTerrainIntersection(
									cpIntrsctn, 
									cIn, 
									zout, 
									norm, 
									pIfTrrnObjUsed,
									pMaterial) ) {
				Result ans;

				ans.z                   = zout;
				ans.norm                = norm;
				ans.mat                 = 0;		// needs fixin...
				ans.hint.m_hintState    = CCved::eCV_ON_INTRSCTN;
				ans.hint.m_intersection = *iiter;
				ans.code                = CCved::eCV_ON_INTRSCTN;
				possResults.push_back(ans);
			}
		}
	}



	m_rdPcQTree.SearchRectangle(cIn.m_x-1, cIn.m_y-1, cIn.m_x+1, cIn.m_y+1,
			roadPieces);
	if ( roadPieces.size() != 0 ) {
		possResults.reserve( roadPieces.size() );
		
		for (riter = roadPieces.begin(); riter != roadPieces.end(); riter++) {
			pRoad = (cvTRoad *) ( ((char *)m_pHdr) + m_pHdr->roadOfs);
			TLongCntrlPntPoolIdx firstCp, lastCp;
			TRoadPiece *pRoadPiece = BindRoadPiece(*riter);

			firstCp = pRoad[pRoadPiece->roadId].cntrlPntIdx+pRoadPiece->first;
			lastCp  = pRoad[pRoadPiece->roadId].cntrlPntIdx + pRoadPiece->last;

			if ( QryTerrainRoadPiece(
							pRoadPiece->roadId, 
							firstCp, 
							lastCp,
							cIn, 
							zout, 
							norm, 
							pHint,
							pIfTrrnObjUsed,
							pMaterial) ) {
				Result ans;

				ans.z          = zout;
				ans.norm       = norm;
				ans.mat        = 0;		// needs fixin...
				ans.code       = CCved::eCV_ON_ROAD;
				if ( pHint ) 
					ans.hint = *pHint;
				possResults.push_back(ans);
			}
		}
	}


	///////////////////////////////////////////////////////////////////
	//
	// Check on what options have been found and select the best one
	//

	if ( possResults.size() == 0 ) {
		if (QryTerrainOffRoad(&zout, norm, cIn, pIfTrrnObjUsed, pMaterial)){
			return eCV_ON_ROAD;	
		}
		if ( pHint ) {
			pHint->m_hintState = CCved::eCV_OFF_ROAD;
			pHint->m_roadId	= 0;
			pHint->m_roadPiece = 0;
		}
		return CCved::eCV_OFF_ROAD;
	}
	
	//
	// If there is only one answer, then just use it
	if ( possResults.size() == 1 ) {
		if ( pHint ) *pHint = possResults[0].hint;
		zout = possResults[0].z;
		norm = possResults[0].norm;
		return possResults[0].code;
	}

	//
	// If there are more than one answer we look for the z that is
	// closer, but below the input z.
	int  iter, best= -1;

	for (iter=0; iter != possResults.size(); iter++) {
		if ( fabs(cIn.m_z - possResults[iter].z) <= cQRY_TERRAIN_SNAP ) {
			if ( best == -1 ) {		// first candidate
				best = iter;
			}
			else {					// already have a candidate
				if ( possResults[best].z > cIn.m_z ) {
					if ( possResults[iter].z > cIn.m_z &&
								possResults[iter].z < possResults[best].z ) {
						best = iter;
					}
				}
				else {
					if ( possResults[iter].z > possResults[best].z ) {
						best = iter;
					}
				}
			}
		}
	}

	if ( best == -1 ) {
		if ( pHint ) {
			pHint->m_hintState = CCved::eCV_OFF_ROAD;
			pHint->m_roadId	= 0;
			pHint->m_roadPiece = 0;
		}
		return CCved::eCV_OFF_ROAD;
	}
	else {
		if ( pHint ) *pHint = possResults[best].hint;
		zout = possResults[best].z;
		norm = possResults[best].norm;
		return possResults[best].code;
	}
} // end of QryTerrain

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrain (private)
// 	Provide terrain elevation information
//
// Remarks: This function calls QryTerrain with a CPoint3D input.
//
// Arguments:
// 	x,y,z - coordinate of the query point.
// 		The input z coordinate is used to discriminate among vertically 
// 		overlaping terrain surfaces.
// 	zout - the output elevation
// 	norm - the normal vector at the interrogation point.  The normal vector
// 		is normalized (i.e., its length is 1.0)
// 	pHint - a pointer to a hint class.  On input, this class is consulted
// 		for the results of prior queries.  On output, the function updates 
// 		this class with information that would be useful to future queries.
// 		If the argument is 0, no hint is used.
// 	pIfTrrnObjUsed - (optional) contains the id of the terrain object
//      used to calcuate the elevation or -1 if no object was used.
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: TOnRoad to indicate that a valid elevation was found based on 
// 	the road network.  The function returns TOnIntersect to indicate that a 
// 	valid elevation was found based on an intersection.  Finally, the function 
// 	returns TNoValue to indicate that no element of the virtual environment 
// 	could provide elevation for the specified point.  The value of the
// 	arguments zout, norm, and pHing are not changed when the function
// 	returns TNoValue.
//
//////////////////////////////////////////////////////////////////////////////
CCved::EQueryCode 
CCved::QryTerrain(
			double			x, 
			double			y, 
			double			z, 
			double&			zout, 
			CVector3D&		norm,
			CTerQueryHint*	pHint,
			int*			pIfTrrnObjUsed,
			int*			pMaterial) 
{
	CPoint3D v(x, y, z);
	return QryTerrain(v, zout, norm, pHint, pIfTrrnObjUsed, pMaterial);
} // end of QryTerrain

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrain (private)
// 	Provide terrain elevation information
//
// Remarks: This function calls QryTerrain with a CPoint3D input.
//
// Arguments:
// 	roadPos - road coordinate of the query point.
// 		The input z coordinate is used to discriminate among vertically 
// 		overlaping terrain surfaces.
// 	zout - the output elevation
// 	norm - the normal vector at the interrogation point.  The normal vector
// 		is normalized (i.e., its length is 1.0)
// 	pHint - a pointer to a hint class.  On input, this class is consulted
// 		for the results of prior queries.  On output, the function updates 
// 		this class with information that would be useful to future queries.
// 		If the argument is 0, no hint is used.
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: TOnRoad to indicate that a valid elevation was found based on 
// 	the road network.  The function returns TOnIntersect to indicate that a 
// 	valid elevation was found based on an intersection.  Finally, the function 
// 	returns TNoValue to indicate that no element of the virtual environment 
// 	could provide elevation for the specified point.  The value of the
// 	arguments zout, norm, and pHing are not changed when the function
// 	returns TNoValue.
//
//////////////////////////////////////////////////////////////////////////////
CCved::EQueryCode 
CCved::QryTerrain(
			const CRoadPos&	roadPos, 
			double&			zout, 
			CVector3D&		norm, 
			CTerQueryHint*	pHint,
			int*			pIfTrrnObjUsed,
			int*			pMaterial) 
{
	CPoint3D v = roadPos.GetBestXYZ();
	return QryTerrain(v, zout, norm, pHint, pIfTrrnObjUsed, pMaterial);
} // end of QryTerrain

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetFourCornerOfObj (private)
// 	Returns the four corners of an object as determined by its position, 
// 	tangent, lateral, xSize, and ySize.
//
// Remarks: For convenience, the position, tangent, and lateral 
// 	values can be retrieved from the object's state if they are not
// 	provided by the user.
//
//	The function that takes a CPoint3D array for the fourCorners is
//	provided because of some inherent problem with polymorphism and
//	arrays.  If this function was not included, and a CPoint3D array
//	was passed into the CPoint2D array parameter, then the data would
//	come back incorrectly.  It would look like this:
//		3dCorners[0].m_z = real corner[1].m_x
//		3dCorners[1].m_x = real corner[1].m_y
//		3dCorners[1].m_y = real corner[2].m_x
//		3dCorners[1].m_z = real corner[2].m_y
//		...
//
// Parameters: 
// 	Input:
// 		objId - object identifier
// 		position - (optional) position of object
// 		tangent - (optional) normalized tangent vector of object
// 		lateral - (optional) normalized lateral vector of object
// 	Output:
// 		fourCorners - 4 points on the corners of the object
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetFourCornerOfObj(
			TObjectPoolIdx		objId,
			CPoint3D*			fourCorners,
			const CPoint3D*		position,
			const CVector3D*	tangent,
			const CVector3D*	lateral) const
{
	CPoint2D corners2D[4];
	int i;

	GetFourCornerOfObj(objId, corners2D, 
						position, tangent, lateral);
	
	// Assign the returned corner values to fourCorners
	//	If a 3D position was provided, then assign the
	//	z component of the position to fourCorners[i].m_z 
	for (i = 0; i < 4; i++) {

		fourCorners[i].m_x = corners2D[i].m_x;
		fourCorners[i].m_y = corners2D[i].m_y;

		if (position != 0) {
			fourCorners[i].m_z = position->m_z;
		}
		else {
			fourCorners[i].m_z = 0.0;
		}
	}
} // end of GetFourCornerOfObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the four corners of an object as determined by 
//  its position, tangent, lateral, xSize and ySize.
//
// Remarks: For convenience, the position, tangent, and lateral 
// 	values can be retrieved from the object's state if they are not
// 	provided by the user.
//
//	The function that takes a CPoint3D array for the fourCorners is
//	provided because of some inherent problem with polymorphism and
//	arrays.  If this function was not included, and a CPoint3D array
//	was passed into the CPoint2D array parameter, then the data would
//	come back incorrectly.  It would look like this:
//		3dCorners[1].m_x = real corner[1].m_y
//		3dCorners[1].m_y = real corner[2].m_x
//		...
//
// Arguments:
//  objId - object identifier
//	fourCorners - (output) 4 points on the corners of the object
// 	position - (optional) position of object
//  tangent - (optional) normalized tangent vector of object
//  lateral - (optional) normalized lateral vector of object
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetFourCornerOfObj(
			TObjectPoolIdx		objId,
			CPoint2D*			fourCorners,
			const CPoint3D*		position,
			const CVector3D*	tangent,
			const CVector3D*	lateral
			) const
{
	cvTObj*		pObj		= BindObj(objId);
	double		objLen		= pObj->attr.xSize * 0.5;
	double		objWid		= pObj->attr.ySize * 0.5;

	CPoint3D	objPos;
	CVector3D	objTan;
	CVector3D	objLat;

	if( position == 0 ) 
	{
		objPos = GetObjPos( objId );
	}
	else 
	{
		objPos = *position;
	}
		
	if( tangent == 0 ) 
	{
		objTan = GetObjTan( objId );
	}
	else 
	{
		objTan = *tangent;
	}

	if( lateral == 0 ) 
	{
		objLat = GetObjLat( objId );
	}
	else 
	{
		objLat = *lateral;
	}

	objTan.Scale( objLen );
	objLat.Scale( objWid );

	fourCorners[0] = objPos - objTan - objLat;
	fourCorners[1] = objPos - objTan + objLat;
	fourCorners[2] = objPos + objTan + objLat;
	fourCorners[3] = objPos + objTan - objLat;
} // end of GetFourCornerOfObj

//////////////////////////////////////////////////////////////////////////////
// 
// Description: GetLocalCoordinate (private)
// 	Return a point that is the queryPt translated into the object coordinate
// 	system.
//
// Remarks:
//
// Arguments:
// 	tan - tangent of the object
// 	right - right vector of the object
// 	queryPt - point to translate
// 	objLocation - center of object coordinate system
//
// Returns: a 2D point in the object coordinate system.
//
//////////////////////////////////////////////////////////////////////////////
CPoint2D 
CCved::GetLocalCoordinate(
		const CVector3D&		tan,
		const CVector3D&		right,
		const CPoint3D&			queryPt,
		const CPoint3D&			objLocation)
{
	CVector3D lat(right);
	lat.Scale(-1);
	// tan*v + lat*u= queryPt - (objLocation)
	//////////////////////////////////////////////////////////
	// ax + by = c
	// dx + ey = f
	// lat.m_i*u + tan.m_i*v = queryPt.m_x - objLocation.m_x
	// lat.m_j*u + tan.m_j*v = queryPt.m_y - objLocation.m_y
	/////////////////////////////////////////////////////////
	double c		= queryPt.m_x - objLocation.m_x;
	double f		= queryPt.m_y - objLocation.m_y;
	double abde	= lat.m_i*tan.m_j - lat.m_j*tan.m_i;
	double bcef	= tan.m_i*f - tan.m_j*c;
	double cafd	= c*lat.m_j - f*lat.m_i;
#if 0
	double u		= 0 - (bcef/abed);
	double v		= 0 - (cafd/abed);
#endif 
	
	//CPoint2D pt( -(bcef/abde), -(cafd/abde) );
	// i flipped u and v, coz tan is supposed to be along with x(u)
	// and the way i coded is the other way around
	CPoint2D pt( -(cafd/abde), -(bcef/abde) );
	return pt;
} // end of GetLocalCoordinate

//////////////////////////////////////////////////////////////////////////////
//
// Description: IntersectWithTerrainObj (private)
//  Find the elevation of the querried point. called by IfCollideWithStaticObj
//
// Remarks:
//
// Arguments:
//  objId - id of the object to be checked if the point falls within
//  pZ - output elevation of point on the static object
//  normal - output normal of point on the static object
//  cIn - point to query
//  pIfTrrnObjUsed - (optional) contains true if a terrain object was found
//      at cIn, false otherwise
//  pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if cIn falls on a terrain object
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IntersectWithTerrainObj(
					int				objId, 
					double*			pZ, 
					CVector3D&		normal, 
					const CPoint3D& cIn,
					int*			pIfTrrnObjUsed,
					int*			pMaterial
					)
{
	cvTObjState state;
	GetObjState( objId, state );
	CBoundingBox objBox(
				state.anyState.boundBox[0].x,
				state.anyState.boundBox[0].y,
				state.anyState.boundBox[1].x,
				state.anyState.boundBox[1].y
				);

	if( !objBox.Encloses( cIn ) )
	{
		return false;
	}

	CPoint2D  fourCorners[4];
	CPoint3D  objPos( state.anyState.position );
	CVector3D tanVec( state.anyState.tangent );
	CVector3D latVec( state.anyState.lateral );
	GetFourCornerOfObj( objId, fourCorners, &objPos, &tanVec, &latVec );

	/*static*/ CPolygon2D sPoly;
	sPoly.Clear();
	

	int i;
	for( i = 0; i < 4; i++ )
	{
		sPoly.AddPoint(fourCorners[i]);
	}
	if( sPoly.Contains( cIn ) )
	{
		// do something with pZ
		if( pIfTrrnObjUsed )  *pIfTrrnObjUsed = objId;

		CPoint2D localPnt;
		localPnt = GetLocalCoordinate( tanVec, latVec, cIn, objPos );

		int solId = GetObjSolId( objId );
		const CSolObj* cpSolObj = m_sSol.GetObj( solId );		
		const CSolObjTerrain* cpSolTerrainObj = 
						dynamic_cast<const CSolObjTerrain*>( cpSolObj );
		if (!cpSolTerrainObj)
			return false;
		const CTerrainGrid<Post>& grid = cpSolTerrainObj->GetMap();
		Post gridOut;       // the result of the query
		CPoint3D qryPnt( localPnt.m_x, localPnt.m_y, cIn.m_z );
		if( grid.Query( qryPnt, gridOut, normal ) ) 
		{
			CVector3D iVec( 1, 0, 0 );
			CVector3D jVec( 0, 1, 0 );
			double cz = tanVec.DotP( iVec );
			double sz = tanVec.DotP( jVec );
			normal.RotZ( cz, sz );

			if( cpSolTerrainObj->GetElevIsDelta() )
			{
				(*pZ) += gridOut.z;
			}else{
				*pZ = gridOut.z;
			}
			if( pMaterial )  *pMaterial = gridOut.type;
		}
		else 
		{
			cvCInternalError err(
								"Query outside of terrain grid map", 
								__FILE__, 
								__LINE__
								);
			throw err;
		}

		return true;
	}

	return false;
}

/////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// Description: IfCollideWithStaticObj (private)
// 	Determines whether a given point collides with a static object on a road.
//
// Remarks:
//
// Arguments:
// 	pZ - output elevation of point on the static object
// 	normal - output normal of point on the static object
// 	cIn - point to query
// 	roadId - identifier of the road
// 	cntrlPntIdx - index of the nearest control point
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
// 
// Returns: true if cIn collides with an object, false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IfCollideWithStaticObj(
		double*					pZ,
		CVector3D&				normal,				
		const CPoint3D&			cIn,
		TRoadPoolIdx			roadId,
		TLongCntrlPntPoolIdx	cntrlPntIdx,
		const CVector3D&		cTangent,
		const CVector3D&		cLateral,
		const CVector3D&		cNorVector,
		int*                    pIfTrrnObjUsed,
		int*                    pMaterial)
{
	cvTRoad*		pRoad		= BindRoad(roadId);
	cvTCntrlPnt*	pCntrlPnt	= BindCntrlPnt(cntrlPntIdx);
	cvTObjRef*		pObjRefPool	= BindObjRef(0);
	cvTObj*			pObjPool	= BindObj(0);
	bool			find		= false;

	CBoundingBox box(cIn,cIn);
	vector<int> results;
	m_staticObjQTree.SearchRectangle(box,results);

	int objRef = pCntrlPnt->objRefIdx;
	int objId = 0;

	if (objRef != 0) {
		for (vector<int>::iterator itr = results.begin(); itr!= results.end(); itr++){
			objId = *itr;
			if (pObjPool[objId].type != eCV_TERRAIN){
				continue;
			}
			if (IntersectWithTerrainObj( objId, pZ, normal, cIn, pIfTrrnObjUsed,pMaterial)) {
				find = true;
				OrientVectorAlongXY(cNorVector, normal);
			}
		}
	}

#if 0
	objId = m_pHdr->objectCountInitial;
	for(; objId < m_pHdr->objectCount; objId++){
		if (pObjPool[objId].type == eCV_TERRAIN){
			if (IntersectWithTerrainObj(objId, pZ, normal, cIn, cTangent, cLateral, pIfTrrnObjUsed,	pMaterial)){
				find = true;
			}
		}
	}
#endif

	return find;
} // end of IfCollideWithStaticObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: IfCollideWithStaticObj (private)
// 	Determines whether a given point collides with a static object on an
// 	intersection.
//
// Remarks:
//
// Arguments:
// 	pZ - output elevation of point on the static object
// 	normal - output normal of point on the static object
// 	cIn - point to query
// 	cpIntrsctn - pointer to the intersection to search
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
// 
// Returns: true if cIn collides with an object, false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IfCollideWithStaticObj(
		double*					pZ,
		CVector3D&				normal,				
		const CPoint3D&			cIn,
		const TIntrsctn*		cpIntrsctn,
		int*					pIfTrrnObjUsed,
		int*					pMaterial)
{
	CBoundingBox box(cIn,cIn);
	vector<int> results;
	m_staticObjQTree.SearchRectangle(box,results);

	cvTObjRef*		pObjRefPool	= BindObjRef(0);
	cvTObj*			pObjPool	= BindObj(0);
	bool			find		= false;


	int objRef = cpIntrsctn->objRefIdx;
	int objId = 0;
	
	if (objRef != 0) {
		int curr	= objRef;
		int next	= pObjRefPool[curr].next;
		objId	= pObjRefPool[curr].objId;
		for (vector<int>::iterator itr = results.begin(); itr!= results.end(); itr++){
			objId = *itr;
			if (pObjPool[objId].type != eCV_TERRAIN){
				continue;
			}
			if (IntersectWithTerrainObj(objId, pZ, normal, cIn, pIfTrrnObjUsed,	pMaterial))
				find = true;
		}
	}
#if 0
	objId = m_pHdr->objectCountInitial;
	for(; objId < m_pHdr->objectCount; objId++){
		if (IntersectWithTerrainObj(objId, pZ, normal, cIn, pIfTrrnObjUsed,	pMaterial)){
			find = true;
		}
	}
#endif
	return find;
} // end of IfCollideWithStaticObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: Determines whether a given point collides with a 
//  repeated object on a road.
//
// Remarks:
//
// Arguments:
// 	pZ - (output) Elevation of point on the static object.
// 	normal - (output) Normal of point on the static object.
// 	cIn - point to query
// 	roadId - identifier of the road
// 	cntrlPntIdx - index of the nearest control point
// 	cPointOnSpline - 3D point on the road spline
// 	tPar - t parameter on the spline
// 	cTanVector - tangent along the spline
// 	cLatVector - lateral along the spline
// 	cNorVector - lateral along the spline
// 	pIfTrrnObjUsed - (optional) contains true if a terrain object was found
// 		at cIn, false otherwise
// 	pMaterial - (optional) contains a pointer to the material found at cIn.
// 
// Returns: true if cIn collides with an object, false otherwise.
// 	
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IfCollideWithRepObj(
		double*					pZ,
		CVector3D&				normal,
		const CPoint3D&			cIn,
		TRoadPoolIdx			roadId,
		TLongCntrlPntPoolIdx	cntrlPntIdx,
		const CPoint3D&			cPointOnSpline,
		double					tPar,
		const CVector3D&		cTanVector,	//tan along the spline
		const CVector3D&		cLatVector,	//lat along the spline on
		const CVector3D&		cNorVector,
		int*					pIfTrrnObjUsed,
		int*					pMaterial
		)
{
	cvTRoad*		pRoad		= BindRoad(roadId);
	cvTCntrlPnt*	pCurrPnt	= BindCntrlPnt(cntrlPntIdx);
	cvTCntrlPnt* 	pNextPnt	= BindCntrlPnt(cntrlPntIdx + 1);
	cvTRepObj*		pRepObj		= NULL;
	cvTObj*			pObj		= NULL;
	int				firstRepObj = pRoad->repObjIdx;
	bool			find		= false;
	cvTObjState		state;

	int i;
	for( i = 0; i < pRoad->numRepObj; i++ )
	{
		pRepObj						= BindRepObj(firstRepObj + i);
		pObj						= BindObj(pRepObj->objId);
		if( pObj->type != eCV_TERRAIN )
		{
			continue;
		}

		double objLen = pObj->attr.xSize * 0.5;
		double objWid = pObj->attr.ySize * 0.5;

		GetObjState(pObj->myId, state);
		CVector3D tanObj( state.anyState.tangent );
		CVector3D latObj( state.anyState.lateral );	
		CVector3D tanScale( tanObj );
		CVector3D latScale( latObj );
		//////////////////////////////////////////////////////////////
		// lenAlong is the half length of the object along the      //
		// the direction of the road.                               //
		// Since cTanVector(tan on the spline) is a unit vector, the //
		// dot product with cTanVector(absolute value) would be the  //
		// the length along the tan direction                       //
		// widAlong is donw in the same way                         //
		//////////////////////////////////////////////////////////////
		tanScale.Scale( objLen );
		latScale.Scale( objWid );
		CVector3D lengthVec1 = tanScale + latScale;
		CVector3D lengthVec2 = tanScale - latScale;

		CVector3D horizontalVec( 1, 0, 0 );
		CVector3D verticalVec( 0, 1, 0 );
		double lenAlong1 = fabs( lengthVec1.DotP(horizontalVec) );
		double lenAlong2 = fabs( lengthVec2.DotP(horizontalVec) );
		double widAlong1 = fabs( lengthVec1.DotP(verticalVec) );	
		double widAlong2 = fabs( lengthVec2.DotP(verticalVec) );	
		double lenAlong  = (lenAlong1 > lenAlong2) ? lenAlong1 : lenAlong2;
		double widAlong  = (widAlong1 > widAlong2) ? widAlong1 : widAlong2;
		
		double latLenOfIn;
		if( fabs( cLatVector.m_i ) > cCV_ZERO )
		{
			latLenOfIn = cIn.m_x - cPointOnSpline.m_x;
			latLenOfIn = latLenOfIn / cLatVector.m_i;
		}
		else
		{
			latLenOfIn = cIn.m_y - cPointOnSpline.m_y;
			latLenOfIn = latLenOfIn / cLatVector.m_j;
		}

		if( (latLenOfIn > (pRepObj->latdist + widAlong)) ||
			(latLenOfIn < (pRepObj->latdist - widAlong))
			)
		{
			continue;
		}
		double lonLenOfIn = tPar * pCurrPnt->distToNextCubic;
		double ofsOfIn    = pCurrPnt->cummulativeCubicDist + lonLenOfIn;

		int	begin = (int)ceil((pCurrPnt->cummulativeCubicDist - 
									pRepObj->start - lenAlong) / 
									pRepObj->period);
		int	end  = (int)floor((pCurrPnt->cummulativeCubicDist +
									pCurrPnt->distToNextCubic  -
									pRepObj->start + lenAlong)  /
									pRepObj->period);
		int last = (int)((pRepObj->end - pRepObj->start) / pRepObj->period );
		if( end > last )  end = last;
		if( begin < 0 )   begin = 0;
		if( end < 0 )     end = -1;

		int nth;
		for( nth = begin; nth <= end; nth++ )
		{
			double ofsOfRepObj = pRepObj->start + nth * pRepObj->period;
			double head = ofsOfRepObj - lenAlong;
			double back = ofsOfRepObj + lenAlong;
			if( (ofsOfIn > back) || (ofsOfIn < head) )
			{
				continue;
			}

			// do serious detection here
			double t = ((ofsOfRepObj - pCurrPnt->cummulativeCubicDist) /
						pCurrPnt->distToNextCubic);
			double t2 = t * t;
			double t3 = t2 * t;

			CVector3D latNth( 
				pCurrPnt->rightVecCubic.i + t *
				(pNextPnt->rightVecCubic.i - pCurrPnt->rightVecCubic.i), 
				pCurrPnt->rightVecCubic.j + t *
				(pNextPnt->rightVecCubic.j - pCurrPnt->rightVecCubic.j), 
				pCurrPnt->rightVecCubic.k + t *
				(pNextPnt->rightVecCubic.k - pCurrPnt->rightVecCubic.k)); 
			latNth.Normalize();
			CVector3D tanNth(
				pCurrPnt->hermite[0].A * t2 * 3 + 
				pCurrPnt->hermite[0].B * t  * 2 + pCurrPnt->hermite[0].C,
				pCurrPnt->hermite[1].A * t2 * 3 + 
				pCurrPnt->hermite[1].B * t  * 2 + pCurrPnt->hermite[1].C,
				pCurrPnt->hermite[2].A * t2 * 3 + 
				pCurrPnt->hermite[2].B * t  * 2 + pCurrPnt->hermite[2].C);
			tanNth.Normalize();
			CPoint3D  posNth(
				pCurrPnt->hermite[0].A * t3 + pCurrPnt->hermite[0].B * t2 +
				pCurrPnt->hermite[0].C *t   + pCurrPnt->hermite[0].D,
				pCurrPnt->hermite[1].A * t3 + pCurrPnt->hermite[1].B * t2 +
				pCurrPnt->hermite[1].C *t   + pCurrPnt->hermite[1].D,
				pCurrPnt->hermite[2].A * t3 + pCurrPnt->hermite[2].B * t2 +
				pCurrPnt->hermite[2].C *t   + pCurrPnt->hermite[2].D);
			posNth = posNth + pRepObj->latdist * latNth;

			CVector3D	lat(latNth);
			CVector3D	forw(tanNth);
			lat.Scale(objWid);
			forw.Scale(objLen);
			CPoint3D	opt(0.0, 0.0, 0.0);
			CPoint3D	fourCorner[4];
			fourCorner[0] = opt + ((-1) * forw - lat);
			fourCorner[1] = opt + ((-1) * forw + lat);
			fourCorner[2] = opt + (       forw + lat);
			fourCorner[3] = opt + (       forw - lat);

			CVector3D	tang(state.anyState.tangent);
			double c = tang.m_i;		// cos of yaw of the object
			double s = tang.m_j;		// sin of yaw of the object
			// rotate the cordinate by yaw
			/*static*/ CPolygon2D sPoly;
			sPoly.Clear();
		
			int j;
			for( j = 0; j < 4; j++ )
			{
				CPoint2D temp;
				temp.m_x = fourCorner[j].m_x * c - fourCorner[j].m_y * s;
				temp.m_y = fourCorner[j].m_x * s + fourCorner[j].m_y * c;
				fourCorner[j].m_x = temp.m_x;
				fourCorner[j].m_y = temp.m_y;
				fourCorner[j] = fourCorner[j] + posNth;
				sPoly.AddPoint(fourCorner[j].m_x, fourCorner[j].m_y);
			}

			if( sPoly.Contains( cIn ) )
			{
				if( pIfTrrnObjUsed )  *pIfTrrnObjUsed = pObj->myId;
				//
				//
				// do something about z
				//
				//
				// if we wanna find out the local coordinate
				//
				CVector3D xVec;
				CVector3D yVec;
				forw = tanNth;
				lat  = latNth;
				xVec.m_i = forw.m_i * c - forw.m_j * s;
				xVec.m_j = forw.m_i * s + forw.m_j * c;
				yVec.m_i = lat.m_i * c - lat.m_j * s;
				yVec.m_j = lat.m_i * s + lat.m_j * c;
				
				CPoint2D localPnt;
				localPnt = GetLocalCoordinate( xVec, yVec, cIn, posNth );

				int solId = GetObjSolId( pObj->myId );
				const CSolObj* cpSolObj = m_sSol.GetObj( solId );		
				const CSolObjTerrain* cpSolTerrainObj = 
								dynamic_cast<const CSolObjTerrain*>( cpSolObj );
				const CTerrainGrid<Post>& grid = cpSolTerrainObj->GetMap();
				Post gridOut;       // the result of the query
				CPoint3D qryPnt( localPnt.m_x, localPnt.m_y, cIn.m_z );	
				if( grid.Query( qryPnt, gridOut, normal ) ) 
				{
					normal.RotZ( c, s );
					OrientVectorAlongXYZ( cTanVector, cNorVector, normal );

					if( cpSolTerrainObj->GetElevIsDelta() )
					{
						(*pZ) += gridOut.z;
					}
					else
					{
						*pZ = gridOut.z;
					}
					if( pMaterial )  *pMaterial = gridOut.type;
				}
				else 
				{
					cvCInternalError err(
										"Query outside of terrain grid map", 
										__FILE__, 
										__LINE__
										);
					throw err;
				}

				find = true;
			}
		}
	}

	return find;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTan (private)
//
// Remarks:  Used by the lriverification tool
//
// Arguments:
// 	cIn - point to query
//
// Returns: the tangent at that point
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CCved::QryTan(const CPoint3D& cIn)
{

	CVector3D tan;
	CVector3D retTan;
	cvTRoad*  pRoad;
	/*static*/ vector<int>  roadPieces;
	vector<int>::const_iterator  riter;
	/*static*/ vector<Result>  possResults;
	/*static bool sInit = false;
	if (!sInit) {*/
	//roadPieces.reserve( 1000 );
	//possResults.reserve( 1000 );
	////	sInit = true;
	////}
	//roadPieces.clear();
	//possResults.clear();

	 m_rdPcQTree.SearchRectangle(cIn.m_x-1, cIn.m_y-1, cIn.m_x+1, cIn.m_y+1,
		roadPieces);
	
	if ( roadPieces.size() != 0 ) {
		possResults.reserve( roadPieces.size() );
		for (riter = roadPieces.begin(); riter != roadPieces.end(); riter++) {
			pRoad = (cvTRoad *) ( ((char *)m_pHdr) + m_pHdr->roadOfs);
			TLongCntrlPntPoolIdx firstCp, lastCp;
			TRoadPiece *pRoadPiece = BindRoadPiece(*riter);

			firstCp = pRoad[pRoadPiece->roadId].cntrlPntIdx+pRoadPiece->first;
			lastCp  = pRoad[pRoadPiece->roadId].cntrlPntIdx + pRoadPiece->last;
			
			double			zout;
			CVector3D		norm;
			CTerQueryHint	hint;
			CTerQueryHint*  pHint=&hint;
	
			if ( QryTerrainRoadPiece(
						pRoadPiece->roadId,
						firstCp,
						lastCp,
						cIn,
						zout,
						norm,
						tan,
						pHint,
						NULL,
						NULL)) {
				Result ans;
				ans.z          = zout;
                ans.norm       = tan;
                ans.mat        = 0;     // needs fixin...
                ans.code       = CCved::eCV_ON_ROAD;
                possResults.push_back(ans);
            }
        }
		int  iter, best= -1;
		for (iter=0; iter != possResults.size(); iter++) {
        if ( fabs(cIn.m_z - possResults[iter].z) <= cQRY_TERRAIN_SNAP ) {
            if ( best == -1 ) {     // first candidate
                best = iter;
            }
            else {                  // already have a candidate
                if ( possResults[best].z > cIn.m_z ) {
                    if ( possResults[iter].z > cIn.m_z &&
                                possResults[iter].z < possResults[best].z ) {
                        best = iter;
                    }
                }
                else {
                    if ( possResults[iter].z > possResults[best].z ) {
                        best = iter;
                    }
                }
            }
        }
    }

    if ( best == -1 ) {
    }
    else {
        retTan = possResults[best].norm;
    }
		return retTan;
    }
	else
		return	retTan;
} // end of QryTan

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrainOffRoad (private)
//	Find the elevation of the querried point. only called when it is off
//  road.	
//
// Remarks:
//
// Arguments:
//  pZ - output elevation of point on the static object
//  normal - output normal of point on the static object
//  cIn - point to query
//  pIfTrrnObjUsed - (optional) contains true if a terrain object was found
//      at cIn, false otherwise
//  pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if cIn falls on a terrain object
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::QryTerrainOffRoad(
		double*			pZ,
		CVector3D&		normal,
		const CPoint3D&	cIn,
		int*			pIfTrrnObjUsed,
		int*			pMaterial)
{
	bool found = false;
	double maxElev = -100000.0;
	double elev = -100000.0;
	int	maxZobjId = 0;
	int maxZmaterial = 0;
	int material;
	int curObjId;
	CVector3D maxZnormal(0,0,1);
	//static bool sInit = false;
	/*static*/ vector<int> trrnObjs;
	//if (!sInit) {
	//	sInit = true;
		trrnObjs.reserve(10);
	//}
	trrnObjs.clear();

	// seasrch for the static objects specified in LRI
	m_trrnObjQTree.SearchRectangle(
		cIn.m_x-1, cIn.m_y-1, cIn.m_x+1, cIn.m_y+1, trrnObjs);
	if (trrnObjs.size() != 0){
		vector<int>::const_iterator idItr = trrnObjs.begin();
		for (; idItr != trrnObjs.end(); idItr++){
			if(IntersectWithTerrainObjOffRoad( 
										*idItr, 
										pZ, 
										normal, 
										cIn, 
										&curObjId, 
										&material,
										&elev)){
				if (	maxElev < elev){
					maxElev = elev;
					maxZobjId = curObjId;
					maxZmaterial = material;			
				}
				found = true;
			}
		} // for
		if (found){
			*pZ = maxElev;
		}
	}

	// search for the static objects created at runtime
	TU32b objId = m_pHdr->objectCountInitial;
	cvTObj* pObjPool = BindObj(0);
	for(; objId < m_pHdr->objectCount; objId++){
		if (pObjPool[objId].type == eCV_TERRAIN){
			if(IntersectWithTerrainObjOffRoad(
									objId, 
									pZ, 
									normal, 
									cIn, 
									&curObjId, 
									&material,
									&elev ) ){
				if (maxElev < elev){
					maxElev = elev;
					maxZobjId = curObjId;
					maxZmaterial = material;			
				}
				found = true;
			}
		}
	}
	if (found){
		*pZ = maxElev;
		if (pIfTrrnObjUsed)
			*pIfTrrnObjUsed = maxZobjId;
		if (pMaterial)
			*pMaterial = maxZmaterial;
	}
	return found;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: IntersectWithTerrainObjOffRoad (private)
//	Find the elevation of the querried point. called when it is off
//  road by QryTerrainOffRoad.	
//
// Remarks:
//
// Arguments:
//  objId - id of the object to be checked if the point falls within
//  pZ - output elevation of point on the static object
//  normal - output normal of point on the static object
//  cIn - point to query
//  pIfTrrnObjUsed - (optional) contains true if a terrain object was found
//      at cIn, false otherwise
//  pMaterial - (optional) contains a pointer to the material found at cIn.
//  pMaxElev - the max elevation so far
//
// Returns: true if cIn falls on a terrain object
//
//////////////////////////////////////////////////////////////////////////////
bool 
CCved::IntersectWithTerrainObjOffRoad(
					int             objId,
					double*         pZ,
					CVector3D&      normal,
					const CPoint3D& cIn,
					int*            pIfTrrnObjUsed,
					int*            pMaterial,
					double*			pMaxElev
					)
{
	bool found = false;
	cvTObjState  state;
	GetObjState( objId, state );
	CPoint2D    fourCorners[4];
	CPoint3D    objPos( state.anyState.position );
	CVector3D   tanVec( state.anyState.tangent );
	CVector3D   latVec( state.anyState.lateral );
	GetFourCornerOfObj( objId, fourCorners, &objPos, &tanVec, &latVec );
	/*static*/ CPolygon2D sPoly;
	sPoly.Clear();
	int i;
	for( i = 0; i < 4; i++ )
	{
		sPoly.AddPoint( fourCorners[i] );
	}

	if( sPoly.Contains( cIn ) )
	{
		// do something with pZ
		if( pIfTrrnObjUsed )  *pIfTrrnObjUsed = objId;
		CPoint2D localPnt;
		localPnt = GetLocalCoordinate( tanVec, latVec, cIn, objPos );

		int solId = GetObjSolId( objId );
		const CSolObj* cpSolObj = m_sSol.GetObj( solId );		
		const CSolObjTerrain* cpSolTerrainObj = 
						dynamic_cast<const CSolObjTerrain*>( cpSolObj );
		if (!cpSolTerrainObj){
				cerr<<"Error @"<<__FILE__<<":"<<__LINE__<<" unkown SolID : "<<solId<<endl;
				return false;
		}
		if( cpSolTerrainObj->GetElevIsDelta() )
		{
			
		} 
		else 
		{
			const CTerrainGrid<Post>& grid = cpSolTerrainObj->GetMap();
			Post gridOut;       // the result of the query
			CVector3D norm;
			CPoint3D qryPnt(localPnt.m_x, localPnt.m_y, cIn.m_z);
			if( grid.Query(qryPnt, gridOut, norm) ) 
			{
				CVector3D iVec( 1, 0, 0 );
				CVector3D jVec( 0, 1, 0 );
				double cz = tanVec.DotP( iVec );
				double sz = tanVec.DotP( jVec );
				norm.RotZ( cz, sz );

				if( gridOut.z > *pMaxElev )
				{
					*pMaxElev = gridOut.z;
					normal = norm;
					if( pMaterial )
					{
						*pMaterial = gridOut.type;
					}
					found = true;
				}
			} 
			else 
			{
				/*
				cvCInternalError err(
								"Query outside of terrain grid map",
								__FILE__,
								__LINE__
								);
				throw err;
				*/
			}
		}
	}

	return found;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: orient the vector along x y z axises according to the normal 
//  and tangent of the location of the querried point
//
// Remarks:
//
// Arguments:
//  tanVec	tangent of the queried point without considering the object on top
//   should be normalized before getting passed as parameter
//  latVec	lateral of the queried point without considering the object on top
//   should be normalized before getting passed as parameter
//  reVec	vector that needs to be oriented
//
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::OrientVectorAlongXYZ(
		const CVector3D& tanVec, 
		const CVector3D& norVec, 
		CVector3D&       retVec) const
{
#if 0
	CVector3D iVec(1, 0, 0);
	CVector3D jVec(0, 1, 0);
	CVector3D kVec(0, 0, 1);

	double cx = 0 - latVec.DotP(jVec);
	double sx = 0 - latVec.DotP(kVec);
	retVec.RotX(cx, sx);

	double cy = tanVec.DotP(iVec);
	double sy = 0 - tanVec.DotP(kVec);
	retVec.RotY(cy, sy);

	double cz = tanVec.DotP(iVec);
	double sz = tanVec.DotP(jVec);
#endif
#if 1
	double cx = norVec.m_k;
	double sx = norVec.m_j;
	retVec.RotX(cx, sx);

	double cy = norVec.m_k;
	double sy = 0 - norVec.m_i;
	retVec.RotY(cy, sy);

	double cz = tanVec.m_i;
	double sz = tanVec.m_j;
	retVec.RotZ(cz, sz);
#endif
}                    

//////////////////////////////////////////////////////////////////////////////
//
// Description: orient the vector along x and y axises according to the normal 
//  and tangent of the location of the querried point
//
// Remarks:
//
// Arguments:
//  tanVec	tangent of the queried point without considering the object on top
//   should be normalized before getting passed as parameter
//  latVec	lateral of the queried point without considering the object on top
//   should be normalized before getting passed as parameter
//  reVec	vector that needs to be oriented
//
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::OrientVectorAlongXY(
		const CVector3D& norVec, 
		CVector3D&       retVec) const
{
#if 0
	CVector3D iVec(1, 0, 0);
	CVector3D jVec(0, 1, 0);
	CVector3D kVec(0, 0, 1);

	double cx = 0 - latVec.DotP(jVec);
	double sx = 0 - latVec.DotP(kVec);
	retVec.RotX(cx, sx);

	double cy = tanVec.DotP(iVec);
	double sy = 0 - tanVec.DotP(kVec);
	retVec.RotY(cy, sy);
#endif
	double cx = norVec.m_k;
	double sx = norVec.m_j;
	retVec.RotX(cx, sx);

	double cy = norVec.m_k;
	double sy = 0 - norVec.m_i;
	retVec.RotY(cy, sy);
}                    

} // namespace CVED
