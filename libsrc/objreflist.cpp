/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Author(s):	Paolo Ammann
 * Date:		Maj, 199j
 *
 * $Id: objreflist.cpp,v 1.4 2012/12/18 16:04:39 iowa\dheitbri Exp $
 *
 * Description: Contains the code to generate the object reference list 
 * (for each control point whe have a list with all object on that control
 * point)
 * It is possible to use the code during the "lri" copilation or during
 * the simulation. In the first case whe generate a new pool in the second case
 * whe add the new references to the existin pool
 *
 ****************************************************************************/

#include "cvedpub.h"
#include "cvedstrc.h"
#include "objlayout.h"


/* Define the reserved space for the object reference
 * pool. This space will be used when whe add new 
 * references to the pool at run time.
 */

# define cOBJ_REF_POOL_MEM_RES 300


NAMESPACE_USE;

/* To deal with the repeated objects at the begin i copy all
 * the objects in this struct and i compute then the values for
 * the repeated objects and i also store them in the same vector.
 */

typedef struct TObjInf{
	TObjectPoolIdx realObjRef;
	// The reference to the object in the object pool
	CBoundingBox ObjBoundingBox;
	CPoint3D position;
	// center of the object
	bool repeated;
	// true if it is a repeated object
}TObjInf;



bool rectangleIntersection(CPoint2D rect1[4], CPoint2D rect2[4]);

void getSegment( int curr, int next, cvTCntrlPnt * pCntrlPnts,
		    int cntrlPntsSize,
			cvTLatCntrlPnt * pLatCntrlPnts,
			int latCntrlPntsSize,
			CPoint2D segment[4] );

void getSegment( int next, cvTCntrlPnt * pCntrlPnts,
		    int cntrlPntsSize,
			cvTLatCntrlPnt * pLatCntrlPnts,
			int latCntrlPntsSize,
			CPoint2D segment[4] );

bool rectObjInSegment(CPoint2D cntrlPntSegment[4], int objIndex,
					  cvTObjAttr * pObjAttr, cvTObj * pObjects);

void updateRef(cvTObjRef * pObjRef, cvTCntrlPnt * pCntrlPnts,
			   int cntrlPIndex, int lastRefObj);

void createObjVector(cvTObj * pObjects, int objSize, vector<TObjInf> & objInfVector);

void addRepObjToObjVector(cvTRoad * pRoad, int roadSize, cvTCntrlPnt * pCntrlPnts,
						  int cntrlPntsSize, cvTObj * pObjects, 
						  int objSize, cvTObjAttr * pObjAttr, int objAttrSize,
						  cvTRepObj * pRepObj, vector<TObjInf> & objInfVector);

/*
 * This is the main procedure that create the reference pool
 * and add the references to the pool
 */

void CreateObjectReferencePool ( const CQuadTree & roadPicesQuadTree,
								   cvTRoad * pRoad,
								   int roadSize,
								   cvTRoadPiece * pRoadPieces,
								   int roadPiecesSize,
								   cvTCntrlPnt * pCntrlPnts,
								   int cntrlPntsSize,
								   cvTLatCntrlPnt * pLatCntrlPnts,
								   int latCntrlPntsSize,
								   int newObjIndex,
								   // The index where the new static
								   // objects start in the pool
								   cvTObj * pObjects,
								   int objSize,
								   cvTObjAttr * pObjAttr,
								   int objAttrSize,
								   cvTRepObj * pRepObj,
								   cvTObjRef * & pObjRef,
								   // The pool with the references
								   int & objRefSize,
								   // The size off the reference pool
								   bool newObjRefPool)

{	
	// I set the pointer to the object pool to the beginn of
	// the first new static object.
	// If used to compile the LRI file the first new static object
	// will be equal to the first static object
	pObjects = pObjects + newObjIndex;
	// I have adapt the size to the new pointer position
	objSize = objSize - newObjIndex;

	// The last referenced object 
	// Counter for how many object are referenced
	int lastRefObj = 0;

	// If the procedure is used to compile the LRI file
	// the pool don't exist jet. I have to create the pool
	// and set the first value to 0
	if (newObjRefPool)
	{
		pObjRef = (cvTObjRef * ) malloc (sizeof(cvTObjRef));
		pObjRef[lastRefObj].next = 0;
		pObjRef[lastRefObj].objId = 0;
		lastRefObj = 1;
	}
	
	// I will store in this vector all the objects and
	// all the istances of all the repeated objects.
	// It is easier to deal with the repeated objects.
	// I don't have to make so many differences between the to 
	// objects "type".
	vector<TObjInf> objInfVector;

	// This procedure create the vector and with all 
	// the "new" objects. (only the non repeated)
	createObjVector(pObjects, objSize, objInfVector);

	// If we are in "LRI compiling time" we add to the 
	// "objInfVector" the repeated objects.
	// It is not possible to have repeated objects elswere
	// then in the LRI file.
	if (newObjRefPool)
	{
		addRepObjToObjVector(pRoad, roadSize, pCntrlPnts,
			cntrlPntsSize, pObjects, objSize, pObjAttr, 
			objAttrSize, pRepObj, objInfVector);
	}
	else
	{		
		// If the procedure is used at run time 
		// (To add objects, not during the LRI compilation)
		// Whe have to find the last referenced object, so that
		// whe cann add references to the object reference pool
		// whitout overwrite the alredi existing one
		while ((pObjRef[lastRefObj].objId != 0) & (lastRefObj < objRefSize))
		{
			lastRefObj++;
		}
	}
	
	// For each object in the objInfVector
	// (repeated and non repeated objects)
	int objIndex = 0;
	for (objIndex = 0; objIndex < objInfVector.size(); objIndex++)
	{
		// I use the road piece quad tree to find on wich road piece 
		// the objects belongs
		vector<int>  roadPiecesSet;
		vector<int>::const_iterator  riter;
				
		//Find the road piece vhere the object is
		roadPicesQuadTree.SearchRectangle(objInfVector[objIndex].ObjBoundingBox,
			roadPiecesSet);
			
		// if there are some road pieces where the object belongs
		if ( roadPiecesSet.size() != 0 ) 
		{
			// This is the index of where the object is placed in the
			// object pool.
			int realObjIndex = objInfVector[objIndex].realObjRef;

			// For each road piece in wich to object belongs i search from the first 
			// control point to the last to find in wich control point
			// the object exactly belongs
			for (riter = roadPiecesSet.begin(); riter != roadPiecesSet.end(); riter++) 
			{				
				// first control point of the road piece
				int firstCp = pRoad[pRoadPieces[*riter].roadId].cntrlPntIdx + 
					pRoadPieces[*riter].first;

				//last control point of the road piece
				int lastCp  = pRoad[pRoadPieces[*riter].roadId].cntrlPntIdx + 
					pRoadPieces[*riter].last;

				CPoint2D segment[4];
				int cntrlPIndex;
				for ( cntrlPIndex = firstCp; cntrlPIndex < lastCp; cntrlPIndex++)
				{
					
					if ( cntrlPIndex == firstCp ) 
					{
						getSegment( cntrlPIndex, cntrlPIndex + 1, 
							pCntrlPnts, cntrlPntsSize, pLatCntrlPnts, 
							latCntrlPntsSize, segment);
					}	 
					else 
					{
						segment[0] = segment[3];
						segment[1] = segment[2];	
						getSegment( cntrlPIndex + 1, pCntrlPnts, cntrlPntsSize, 
							pLatCntrlPnts, latCntrlPntsSize, segment);
					}

					bool overlaps = rectObjInSegment(segment, 
						objIndex, pObjAttr, pObjects);

					if (overlaps)
					{
						if (objInfVector[objIndex].repeated)
						{
							pCntrlPnts[cntrlPIndex].cntrlPntFlag |= eREP_OBJ_FLAG; //aggiungere and
						}

						if (pObjects[objIndex].type == eTerrain)
						{
							pCntrlPnts[cntrlPIndex].cntrlPntFlag |= eTERRN_OBJ; //aggiungere and
						}

						if (newObjRefPool)
						{
							pObjRef  = (cvTObjRef * ) realloc ((void * ) pObjRef, 
								(lastRefObj + 1) * sizeof(cvTObjRef));
						}

						pObjRef[lastRefObj].next = 0;
						pObjRef[lastRefObj].objId = objInfVector[objIndex].realObjRef;

						updateRef(pObjRef, pCntrlPnts, cntrlPIndex, lastRefObj);
						
						lastRefObj++;
						
					}// if (overlaps)
				} //each control point
			}// for each road
		}// end if road.size() = 0 
	}// while (last object)

	if (newObjRefPool)
	{
		objRefSize = lastRefObj + cOBJ_REF_POOL_MEM_RES;
		pObjRef  = (cvTObjRef * ) realloc ((void * ) pObjRef, 
			(objRefSize) * sizeof(cvTObjRef));

		int count = 0;
		for (count = lastRefObj; count < objRefSize; count++)
		{
			pObjRef[count].objId = 0;
			pObjRef[count].next = 0;
		}
	}
}

/************************************************************************/

void updateRef(cvTObjRef * pObjRef, cvTCntrlPnt * pCntrlPnts,
			   int cntrlPIndex, int lastRefObj)
{
	int firstObjRef = pCntrlPnts[cntrlPIndex].objRefIdx;
	if (firstObjRef != 0)
	{
		int nextRef = pObjRef[firstObjRef].next;
		int prevRef = firstObjRef;
		
		while(nextRef != 0)
		{
			prevRef = nextRef;
			nextRef = pObjRef[nextRef].next;
		}
		pObjRef[prevRef].next = lastRefObj;
	}
	else
	{
		pCntrlPnts[cntrlPIndex].objRefIdx = lastRefObj;
	}
}

/************************************************************************/

void createObjVector(cvTObj * pObjects, int objSize, vector<TObjInf> & objInfVector)
{
	int objIndex;
	for (objIndex = 0; objIndex < objSize; objIndex++)
	{
		TObjInf objInfo;
		objInfo.realObjRef = objIndex + cNUM_DYN_OBJS;
		objInfo.ObjBoundingBox.SetMinX (pObjects[objIndex].boundBox[0].x);
		objInfo.ObjBoundingBox.SetMinY (pObjects[objIndex].boundBox[0].y); 
		objInfo.ObjBoundingBox.SetMaxX (pObjects[objIndex].boundBox[1].x);
		objInfo.ObjBoundingBox.SetMaxY (pObjects[objIndex].boundBox[1].y);
		objInfo.position.m_x = pObjects[objIndex].stateBufA.state.anyState.position.x;
		objInfo.position.m_y = pObjects[objIndex].stateBufA.state.anyState.position.y;
		objInfo.position.m_z = pObjects[objIndex].stateBufA.state.anyState.position.z;
		objInfo.repeated = false;
		objInfVector.push_back(objInfo);
	}
}

/************************************************************************/

bool rectObjInSegment(CPoint2D cntrlPntSegment[4], int objIndex,
					  cvTObjAttr * pObjAttr, cvTObj * pObjects)
{
	CPoint2D objRectangle[4];
	
	int objAttrIndex = pObjects[objIndex].firstOption;
	float objLen = (pObjAttr[objAttrIndex].xSize) / 2.0;
	float objWidth = (pObjAttr[objAttrIndex].ySize) / 2.0;
	CPoint3D objPos;
	CVector3D tanVector; 
	CVector3D latVector; 

	objPos.m_x = pObjects[objIndex].stateBufA.state.anyState.position.x;
	objPos.m_y = pObjects[objIndex].stateBufA.state.anyState.position.y;
	objPos.m_z = pObjects[objIndex].stateBufA.state.anyState.position.z;

	tanVector.m_i = pObjects[objIndex].stateBufA.state.anyState.tangent.i;
	tanVector.m_j = pObjects[objIndex].stateBufA.state.anyState.tangent.j;
	tanVector.m_k = pObjects[objIndex].stateBufA.state.anyState.tangent.k;
	latVector.m_i = pObjects[objIndex].stateBufA.state.anyState.lateral.i;	
	latVector.m_j = pObjects[objIndex].stateBufA.state.anyState.lateral.j;	
	latVector.m_k = pObjects[objIndex].stateBufA.state.anyState.lateral.k;	

	objRectangle[0] = objPos - objLen * tanVector - objWidth * latVector;
	objRectangle[1] = objPos + objLen * tanVector - objWidth * latVector;
	objRectangle[2] = objPos + objLen * tanVector + objWidth * latVector;
	objRectangle[3] = objPos - objLen * tanVector + objWidth * latVector;

	return rectangleIntersection(cntrlPntSegment, objRectangle);
}
	
/************************************************************************/

void addRepObjToObjVector(cvTRoad * pRoad, int roadSize, cvTCntrlPnt * pCntrlPnts,
						  int cntrlPntsSize, cvTObj * pObjects, 
						  int objSize, cvTObjAttr * pObjAttr, int objAttrSize,
						  cvTRepObj * pRepObj, vector<TObjInf> & objInfVector)
{
	int roadIndex;
	for (roadIndex = 0; roadIndex < roadSize; roadIndex++)
	{
		int numRepObj = pRoad[roadIndex].numRepObj;
		int repObjCount;
		TRepObjPoolIdx repObjIdx = pRoad[roadIndex].repObjIdx;
		
		for (repObjCount = 0; repObjCount < numRepObj; repObjCount++)
		{
			float start = pRepObj[repObjIdx].start;
			float latDist = pRepObj[repObjIdx].latdist;
			float period = pRepObj[repObjIdx].period;
			float end = pRepObj[repObjIdx].end;
			int numOfIstaces = (end - start) / period;

			int cntrlPntsIndex = pRoad[roadIndex].cntrlPntIdx;
			int	numCntrlPnt = pRoad[roadIndex].numCntrlPnt;

			CPoint2D fourCorner[4];
			int objAttrIndex = pObjects[pRepObj[repObjIdx].objId].firstOption;
				
			float xSize  = pObjAttr[objAttrIndex].xSize;
			float ySize  = pObjAttr[objAttrIndex].ySize;

			int istanceCount;
			for (istanceCount = 0; istanceCount < numOfIstaces; istanceCount++ )
			{
				float distance = start + period * istanceCount;

				bool betweenCntrlPnts;
				betweenCntrlPnts = (distance >= pCntrlPnts[cntrlPntsIndex].cummulativeLinDist)
					& (distance < pCntrlPnts[cntrlPntsIndex + 1].cummulativeLinDist);
				
				while ((!betweenCntrlPnts) & (cntrlPntsIndex < numCntrlPnt))
				{
					cntrlPntsIndex++;
				}

				if (cntrlPntsIndex = numCntrlPnt)
				{
					continue;
				}

				distance -= pCntrlPnts[cntrlPntsIndex].cummulativeLinDist;

				TObjInf objInfo;
				objInfo.position.m_x = pCntrlPnts[cntrlPntsIndex].location.x;
				objInfo.position.m_y = pCntrlPnts[cntrlPntsIndex].location.y;
				objInfo.position.m_z = pCntrlPnts[cntrlPntsIndex].location.z;

				objInfo.position = objInfo.position + 
					distance * pCntrlPnts[cntrlPntsIndex].tangVecLinear + 
					latDist * pCntrlPnts[cntrlPntsIndex].rightVecLinear;

				fourCorner[0] = objInfo.position - 
					xSize * pCntrlPnts[cntrlPntsIndex].tangVecLinear - 
					ySize * pCntrlPnts[cntrlPntsIndex].rightVecLinear;
				fourCorner[1] = objInfo.position + 
					xSize * pCntrlPnts[cntrlPntsIndex].tangVecLinear - 
					ySize * pCntrlPnts[cntrlPntsIndex].rightVecLinear;
				fourCorner[2] = objInfo.position + 
					xSize * pCntrlPnts[cntrlPntsIndex].tangVecLinear + 
					ySize * pCntrlPnts[cntrlPntsIndex].rightVecLinear;
				fourCorner[3] = objInfo.position - 
					xSize * pCntrlPnts[cntrlPntsIndex].tangVecLinear + 
					ySize * pCntrlPnts[cntrlPntsIndex].rightVecLinear;
				
				int index;
				for (index = 0; index < 4; index++)
				{
					objInfo.ObjBoundingBox += fourCorner[index];
				}

				objInfo.realObjRef = pRepObj[repObjIdx].objId;
								
				objInfo.repeated = true;
				objInfVector.push_back(objInfo);

			}
		}
	}
}

/************************************************************************/

void getSegment( int curr, int next, cvTCntrlPnt * pCntrlPnts,
		    int cntrlPntsSize, cvTLatCntrlPnt * pLatCntrlPnts,
			int latCntrlPntsSize, CPoint2D segment[4] )
{
	TLatCntrlPntPoolIdx firstLatCntrlPnt = pCntrlPnts[curr].latCntrlPntIdx;

	if (firstLatCntrlPnt != 0)
	{
		TLatCntrlPntPoolIdx lastLatCntrlPnt = firstLatCntrlPnt + 
			pCntrlPnts[curr].numLatCntrlPnt - 1;
	 	
		segment[0].m_x = pCntrlPnts[curr].location.x + pCntrlPnts[curr].rightVecCubic.i * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[0].m_y = pCntrlPnts[curr].location.y + pCntrlPnts[curr].rightVecCubic.j * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[1].m_x = pCntrlPnts[curr].location.x + pCntrlPnts[curr].rightVecCubic.i * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[1].m_y = pCntrlPnts[curr].location.y + pCntrlPnts[curr].rightVecCubic.j * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 

		segment[2].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[2].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[3].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[3].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
	}
	else
	{
		segment[0].m_x = pCntrlPnts[curr].location.x + pCntrlPnts[curr].rightVecCubic.i *
								(-pCntrlPnts[curr].logicalWidth/2.0); 
		segment[0].m_y = pCntrlPnts[curr].location.y + pCntrlPnts[curr].rightVecCubic.j *
								(-pCntrlPnts[curr].logicalWidth/2.0); 
		segment[1].m_x = pCntrlPnts[curr].location.x + pCntrlPnts[curr].rightVecCubic.i *
								(pCntrlPnts[curr].logicalWidth/2.0); 
		segment[1].m_y = pCntrlPnts[curr].location.y + pCntrlPnts[curr].rightVecCubic.j *
								(pCntrlPnts[curr].logicalWidth/2.0); 

		segment[2].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i *
								(pCntrlPnts[next].logicalWidth/2.0); 
		segment[2].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j *
								(pCntrlPnts[next].logicalWidth/2.0); 
		segment[3].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i *
								(-pCntrlPnts[next].logicalWidth/2.0); 
		segment[3].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j *
								(-pCntrlPnts[next].logicalWidth/2.0); 
	}
}

/************************************************************************/

void getSegment( int next, cvTCntrlPnt * pCntrlPnts,
		    int cntrlPntsSize,
			cvTLatCntrlPnt * pLatCntrlPnts,
			int latCntrlPntsSize,
			CPoint2D segment[4] )
{

	int curr = next - 1;

	TLatCntrlPntPoolIdx firstLatCntrlPnt = pCntrlPnts[curr].latCntrlPntIdx;

	if (firstLatCntrlPnt != 0)
	{
		TLatCntrlPntPoolIdx lastLatCntrlPnt = firstLatCntrlPnt + 
			pCntrlPnts[curr].numLatCntrlPnt - 1;
	 	
		segment[0].m_x = pCntrlPnts[curr].location.x + pCntrlPnts[curr].rightVecCubic.i * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[0].m_y = pCntrlPnts[curr].location.y + pCntrlPnts[curr].rightVecCubic.j * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[1].m_x = pCntrlPnts[curr].location.x + pCntrlPnts[curr].rightVecCubic.i * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[1].m_y = pCntrlPnts[curr].location.y + pCntrlPnts[curr].rightVecCubic.j * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 

		segment[2].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[2].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j * 
			pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[3].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[3].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j * 
			pLatCntrlPnts[firstLatCntrlPnt].offset; 
	}
	else
	{
		segment[2].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i *
								(pCntrlPnts[next].logicalWidth/2.0); 
		segment[2].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j *
								(pCntrlPnts[next].logicalWidth/2.0); 
		segment[3].m_x = pCntrlPnts[next].location.x + pCntrlPnts[next].rightVecCubic.i *
								(-pCntrlPnts[next].logicalWidth/2.0); 
		segment[3].m_y = pCntrlPnts[next].location.y + pCntrlPnts[next].rightVecCubic.j *
								(-pCntrlPnts[next].logicalWidth/2.0); 
	}
}


/************************************************************************
 *
 * This function detects if two rectangles intersect in the 2D plane.
 * If the rectangles intersect, the function returns true, else it
 * returnes false.
 * Version 1.4 - before morrison's "optimizations" that broke it
 */ 
bool rectangleIntersection(CPoint2D rect1[4], CPoint2D rect2[4])
{
	CPoint2D  p, p1, p2;        /* rectangle vertices                          */
	CPoint2D  p1p2, p1p;        /* edge vectors                                */
	CPoint2D  temp[4];
	bool  OneOutside;       /* flats at least one point outside rectangle  */
	bool  Outside[4];       /* true if respective pt is outside rectange   */
	int     v, n, i;          /* indeces for loops iterating across vertices */
	double  cross_k_component;/* k component of cross product                */
	float   r1min_x, r1min_y; /* min bounds of r1                            */
	float   r1max_x, r1max_y; /* max bounds of r1                            */
	float   r2min_x, r2min_y; /* min bounds of r2                            */
	float   r2max_x, r2max_y; /* max bounds of r2                            */
	float   xlk, ylk, xnm,    /* coordinates of edges, for edge intersect    */
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
	cross_k_component = (rect1[1].m_x - rect1[0].m_x) * (rect1[2].m_y - rect1[0].m_y) - 
						(rect1[1].m_y - rect1[0].m_y) * (rect1[2].m_x - rect1[0].m_x);
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

	cross_k_component = (rect2[1].m_x - rect2[0].m_x) * (rect2[2].m_y - rect2[0].m_y) - 
						(rect2[1].m_y - rect2[0].m_y) * (rect2[2].m_x - rect2[0].m_x);
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



