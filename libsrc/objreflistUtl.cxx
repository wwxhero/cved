/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Author(s):		
 * Date:		
 *
 * $Id: objreflistUtl.cxx,v 1.6 2004/04/09 15:55:59 yiannis Exp $
 *
 * Description: This file is included by both cved.cxx and objreflist.cxx, 
 *	because it contains code used to generate the object reference list 
 *	and update its contents when a static object is added to the environment 
 *	during runtime.
 *
 ****************************************************************************/
#include "cvedpub.h"
#include "cvedstrc.h"

#include <objlayout.h>

void UpdateRefOfCntrlPnt(
		cvTObjRef*			pObjRef, 
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPIndex, 
		int					lastRefObj)
{
	int firstObjRef = pCntrlPnts[cntrlPIndex].objRefIdx;

	if (firstObjRef != 0) {
		int nextRef = pObjRef[firstObjRef].next;
		int prevRef = firstObjRef;
		
		while(nextRef != 0) {
			prevRef = nextRef;
			nextRef = pObjRef[nextRef].next;
		}
		pObjRef[prevRef].next = lastRefObj;
	}
	else {
		pCntrlPnts[cntrlPIndex].objRefIdx = lastRefObj;
	}
}

void UpdateRefOfIntrsctn(
		cvTObjRef*			pObjRef,
		cvTIntrsctn*		pIntrsctnPool,
		int					intrsctnIdx,
		int					lastRefObj)
{
	int firstObjRef = pIntrsctnPool[intrsctnIdx].objRefIdx;
	
	if (firstObjRef != 0) {
		int nextRef = pObjRef[firstObjRef].next;
		int prevRef = firstObjRef;
		
		while(nextRef != 0) {
			prevRef = nextRef;
			nextRef = pObjRef[nextRef].next;
		}
		pObjRef[prevRef].next = lastRefObj;
	}
	else {
		pIntrsctnPool[intrsctnIdx].objRefIdx = lastRefObj;
	}
}


/************************************************************************/
void GetFourCornersForNonRepObj(
		cvTObj*     pObjects, 
		int         objIndex, 
//		cvTObjAttr* pObjAttr,
		CPoint2D*   pFourCorners
		)
{
	double objLen = pObjects[objIndex].attr.xSize * 0.5f;
	double objWidth = pObjects[objIndex].attr.ySize * 0.5f;

	CPoint3D  objPos( pObjects[objIndex].stateBufA.state.anyState.position );
	CVector3D tanVector( pObjects[objIndex].stateBufA.state.anyState.tangent );
	CVector3D latVector( pObjects[objIndex].stateBufA.state.anyState.lateral );

	CVector3D forw( tanVector );
	CVector3D lat( latVector );

	forw.Scale( objLen );
	lat.Scale( objWidth );

	pFourCorners[0] = objPos - forw -  lat;
	pFourCorners[1] = objPos - forw +  lat;
	pFourCorners[2] = objPos + forw +  lat;
	pFourCorners[3] = objPos + forw -  lat;
}
	

/************************************************************************
 *
 * This function detects if two rectangles intersect in the 2D plane.
 * If the rectangles intersect, the function returns true, else it
 * returnes false.
 * Version 1.4 - before morrison's "optimizations" that broke it
 */ 
bool RectangleIntersection(CPoint2D rect1[4], CPoint2D rect2[4])
{
	CPoint2D  p, p1, p2;        /* rectangle vertices       */
	CPoint2D  p1p2, p1p;        /* edge vectors             */
	CPoint2D  temp[4];
	bool  OneOutside;       /* flats at least one point outside rectangle  */
	bool  Outside[4];       /* true if respective pt is outside rectange   */
	int     v, n, i;          /* indeces for loops iterating across vertices */
	double  cross_k_component;/* k component of cross product                */
	double   r1min_x, r1min_y; /* min bounds of r1                            */
	double   r1max_x, r1max_y; /* max bounds of r1                            */
	double   r2min_x, r2min_y; /* min bounds of r2                            */
	double   r2max_x, r2max_y; /* max bounds of r2                            */
	double   xlk, ylk, xnm,    /* coordinates of edges, for edge intersect    */
			ynm, xmk, ymk;

/*
 * Do bounds checking to eliminate obvious cases
 */
	r1min_x = r1max_x = rect1[0].m_x; 
	r1min_y = r1max_y = rect1[0].m_y;
	r2min_x = r2max_x = rect2[0].m_x; 
	r2min_y = r2max_y = rect2[0].m_y;
	for (i=1; i<4; i++) {
		if ( rect1[i].m_x < r1min_x )
			r1min_x = rect1[i].m_x;
		if ( rect1[i].m_y < r1min_y )
			r1min_y = rect1[i].m_y;
		if ( rect1[i].m_x > r1max_x )
			r1max_x = rect1[i].m_x;
		if ( rect1[i].m_y > r1max_y )
			r1max_y = rect1[i].m_y;

		if ( rect2[i].m_x < r2min_x )
			r2min_x = rect2[i].m_x;
		if ( rect2[i].m_y < r2min_y )
			r2min_y = rect2[i].m_y;
		if ( rect2[i].m_x > r2max_x )
			r2max_x = rect2[i].m_x;
		if ( rect2[i].m_y > r2max_y )
			r2max_y = rect2[i].m_y;
	}

	if ( r1max_x < r2min_x || r2max_x < r1min_x )
		return false;
	if ( r1max_y < r2min_y || r2max_y < r1min_y )
		return false;

/*
 * Make sure both rectangles are specified counterclockwise.  Use
 * cross product between p0p1, p0p2 vectors, pXpY is the vector specified
 * by vertices X and Y.
 */
	cross_k_component = (rect1[1].m_x - rect1[0].m_x) * 
						(rect1[2].m_y - rect1[0].m_y) 
						- 
						(rect1[1].m_y - rect1[0].m_y) * 
						(rect1[2].m_x - rect1[0].m_x);
	if ( cross_k_component < 0 ) 
	{
		int i;
		for (i = 0; i < 4; i++)
		{
			temp[i] = rect1[i];
		}
		rect1[0] = temp[3];
		rect1[1] = temp[2];
		rect1[2] = temp[1];
		rect1[3] = temp[0];
	}

	cross_k_component = (rect2[1].m_x - rect2[0].m_x) * 
						(rect2[2].m_y - rect2[0].m_y) 
						- 
						(rect2[1].m_y - rect2[0].m_y) * 
						(rect2[2].m_x - rect2[0].m_x);
	if ( cross_k_component < 0 ) 
	{
		int i;
		for (i = 0; i < 4; i++)
		{
			temp[i] = rect2[i];
		}
		//memcpy(temp, &rect2[0], 4*sizeof(veSVec));
		rect2[0] = temp[3];
		rect2[1] = temp[2];
		rect2[2] = temp[1];
		rect2[3] = temp[0];
	}

/*
 * If any of the vertices of r1 is inside r2 then we have overlap.  We
 * detect a vertex of r1 being inside r2 by looking 
 */
	for (v=0; v<4; v++) {
		p = rect1[v];

		OneOutside = false;
		for (n=0; n<4; n++) {
			p1 = rect2[n]; 
			p2 = rect2[ (n+1)%4 ];

			p1p2.m_x = p2.m_x - p1.m_x;
			p1p2.m_y = p2.m_y - p1.m_y;

			p1p.m_x  = p.m_x - p1.m_x;
			p1p.m_y  = p.m_y - p1.m_y;

			cross_k_component = p1p.m_x * p1p2.m_y - p1p.m_y * p1p2.m_x;
			if ( cross_k_component > 0.0 ) { /* point to the right half plane*/
				OneOutside = true;
				break;
			}
		}
		Outside[v] = OneOutside;
		if ( !Outside[v] )
			return true;
	}

/*
 * If any of the vertices of r2 is inside r1 then we have overlap
 */
	for (v=0; v<4; v++) {
		p = rect2[v];

		OneOutside = false;
		for (n=0; n<4; n++) {
			p1 = rect1[n]; 
			p2 = rect1[ (n+1)%4 ];

			p1p2.m_x = p2.m_x - p1.m_x;
			p1p2.m_y = p2.m_y - p1.m_y;

			p1p.m_x  = p.m_x - p1.m_x;
			p1p.m_y  = p.m_y - p1.m_y;

			cross_k_component = p1p.m_x * p1p2.m_y - p1p.m_y * p1p2.m_x;
			if ( cross_k_component > 0.0 ) { /* point to the right half plane*/
				OneOutside = true;
				break;
			}
		}
		Outside[v] = OneOutside;
		if ( !Outside[v] )
			return true;
	}

/*
 * The only way we can now have an overlap is when one rect is over the
 * other but none of the points are within and there is no inclusion.
 * We have to do a line segment to line segment intersection test between 
 * each edge of r1 and all edges of r2.
 */
	for (v=0; v<4; v++) {   /* vertices or r1 */
		double  s, t;
		double  determ, one_ov_determ;

		xlk = rect1[(v+1)%4].m_x - rect1[v].m_x;
		ylk = rect1[(v+1)%4].m_y - rect1[v].m_y;

		for ( n=0; n<4; n++) {   /* vertices of r2 */
			xnm = rect2[(n+1)%4].m_x - rect2[n].m_x;
			ynm = rect2[(n+1)%4].m_y - rect2[n].m_y;

			xmk = rect2[n].m_x - rect1[v].m_x;
			ymk = rect2[n].m_y - rect1[v].m_y;

			determ = xnm * ylk - ynm * xlk;
			if ( fabs(determ) < 1e-7 ) {   /* segments are parallel */
				continue;
			}
			else {
				one_ov_determ = 1.0 / determ;
			}

			s = ( xnm * ymk - ynm * xmk) * one_ov_determ;
			if ( s < 0.0 || s > 1.0 )
				continue;

			t = ( xlk * ymk - ylk * xmk) * one_ov_determ;
			if ( t >= 0 && t <= 1.0 )
				return true;
		}
	}

/*
 * Nothing found in terms of intersections.
 */
	return false;
}

