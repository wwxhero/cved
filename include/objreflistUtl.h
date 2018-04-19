/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Author(s):		
 * Date:		
 *
 * $Id: objreflistUtl.h,v 1.2 2004/04/09 15:55:54 yiannis Exp $
 *
 * Description: This file is included by both cved.cxx and objreflist.cxx, 
 *	because it contains code used to generate the object reference list 
 *	and update its contents when a static object is added to the environment 
 *	during runtime.
 *
 ****************************************************************************/

#ifndef __OBJ_REF_LIST_UTL_H
#define __OBJ_REF_LIST_UTL_H
#include <cvedstrc.h>
#include <objlayout.h>

void UpdateRefOfCntrlPnt(
		cvTObjRef*			pObjRef, 
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPIndex, 
		int					lastRefObj);


void UpdateRefOfIntrsctn(
		cvTObjRef*			pObjRef,
		cvTIntrsctn*		pIntrsctnPool,
		int					intrsctnIdx,
		int					lastRefObj);



/************************************************************************/
void GetFourCornersForNonRepObj(
		cvTObj*				pObjects, 
		int					objIndex, 
//		cvTObjAttr*			pObjAttr,
		CPoint2D* 			fourCorners);

	

/************************************************************************
 *
 * This function detects if two rectangles intersect in the 2D plane.
 * If the rectangles intersect, the function returns true, else it
 * returnes false.
 * Version 1.4 - before morrison's "optimizations" that broke it
 */ 
bool RectangleIntersection(CPoint2D rect1[4], CPoint2D rect2[4]);

#endif	/* __OBJ_REF_LIST_H */

