/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Author(s):		
 * Date:		
 *
 * $Id: objreflist.cxx,v 1.1 2005/06/10 17:43:36 yhe Exp $
 *
 * Description: Contains the code to generate the object reference list 
 * In order to accelerate the time to detect if the querry point overlaps
 * with any objects, instead of searching for all of the global objects,
 * we only do the checking with all the objects on the section where the
 * query point falls in. In order to achieve this, a refence list is
 * created for every control point and intersection where entries refer
 * to objects in that area formed by that and the next control point or
 * the interesection. Also, repeated objects are considered.
 *
 ****************************************************************************/
#include "objreflistUtl.h"
#include "parser.h"
#include "err.h"

using namespace std;

/* Define the reserved space for the object reference
 * pool. This space will be used when whe add new 
 * references to the pool at run time.
 */

# define cOBJ_REF_POOL_MEM_RES 1000


/*
 * temporary structure to store the object information 
 */
typedef struct TObjInf{
	TObjectPoolIdx realObjRef;		// The reference to the real object
	CBoundingBox ObjBoundingBox;
	CPoint3D position;				// center of the object
	bool repeated;					// true if it is a repeated object
	CPoint2D fourCorner[4];			// actuall corners of the object, 
									// not the corners of the bounding box
}TObjInf;

/*
 *  Prototypes for functions in this file
 */
void CreateObjectReferencePool( 
		const CQuadTree&	roadPicesQuadTree,
		const CQuadTree&	intrsctnQuadTree,
		cvTRoad*			pRoad,
		int					roadSize,
		cvTRoadPiece*		pRoadPieces,
		int					roadPiecesSize,
		cvTIntrsctn*		pIntrsctnPool,
		int					sizeOfIntrsctnPool,
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize,
		cvTLatCntrlPnt*		pLatCntrlPnts,
		int					latCntrlPntsSize,
		cvTObj*				pObjects,
		int					objSize,
		cvTObjAttr*			pObjAttr,
		int					objAttrSize,
		cvTRepObj*			pRepObj,
		cvTObjRef*&			pObjRef,
		int*				pObjRefSize,
		bool				newObjRefPool);

void CreateObjVector(
		cvTObj*				pObjects, 
		int					objSize, 
//		cvTObjAttr*			pObjAttr,
		vector<TObjInf>&	objInfVector);

void AddRepObjToObjVector(
		cvTRoad*			pRoad, 
		int					roadSize, 
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize, 
		cvTIntrsctn*		pIntrsctnPool,
		int					sizeOfIntrsctnPool,
		cvTObj*				pObjects, 
		int					objSize, 
//		cvTObjAttr*			pObjAttr, 
//		int					objAttrSize,
		cvTRepObj*			pRepObj, 
		vector<TObjInf>&	objInfVector);
void RotateFourCorners( CPoint3D& pt, float c, float s);
void GetSegment( 
		int					curr, 
		int					next, 
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize,
		cvTLatCntrlPnt*		pLatCntrlPnts,
		int					latCntrlPntsSize,
		CPoint2D			segment[4]);

void GetSegment(
		int					next,
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize,
		cvTLatCntrlPnt*		pLatCntrlPnts,
		int					latCntrlPntsSize,
		CPoint2D			segment[4]);
bool IsBetweenPnts(float distance, cvTCntrlPnt* pCntrlPnts, int cntrlPntIdx);


/*
 * This is the main procedure that create the reference pool
 * and add the references to the pool
 */
void CreateObjectReferencePool( 
		const CQuadTree&	roadPicesQuadTree,
		const CQuadTree&	intrsctnQuadTree,
		cvTRoad*			pRoad,
		int					roadSize,
		cvTRoadPiece*		pRoadPieces,
		int					roadPiecesSize,
		cvTIntrsctn*		pIntrsctnPool,
		int					sizeOfIntrsctnPool,
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize,
		cvTLatCntrlPnt*		pLatCntrlPnts,
		int					latCntrlPntsSize,
		cvTObj*				pObjects,
		int					objSize,
//		cvTObjAttr*			pObjAttr,
//		int					objAttrSize,
		cvTRepObj*			pRepObj,
		cvTObjRef*&			pObjRef,		// The pool with the references
		int*				pObjRefSize,	// The size off the reference pool
		bool				newObjRefPool)
{
	// The last referenced object 
	// Counter for how many object are referenced
	int		lastRefObj = 0;

	// If the procedure is used to compile the LRI file
	// the pool don't exist jet. I have to create the pool
	// and set the first value to 0
	if (newObjRefPool) {
		pObjRef = (cvTObjRef * ) malloc (sizeof(cvTObjRef));
		pObjRef[lastRefObj].next = 0;
		pObjRef[lastRefObj].objId = 0;
		lastRefObj = 1;
	}


	vector<TObjInf> objInfVector;

	// This procedure create the vector and with all 
	// the "new" objects. (only the non repeated)
	CreateObjVector(pObjects, objSize, /* pObjAttr, */ objInfVector);

	//////////////////////////////////////////////////////////
	// this part is to check if objects are within intrsctn //
	// if it is, update the obj reference pool which is     //
	// indexed by "objRefIdx" field of the cvTIntrsctn      //
	//////////////////////////////////////////////////////////

	unsigned int objIndex = 0;
	double objTop = 0.0, objBot = 0.0;

	for (objIndex = 0; objIndex < objInfVector.size(); objIndex++) {

		int realObjIndex = objInfVector[objIndex].realObjRef;

		objTop = objInfVector[objIndex].position.m_z + 
				 0.5*pObjects[realObjIndex].attr.zSize;
		objBot = objInfVector[objIndex].position.m_z - 
				 0.5*pObjects[realObjIndex].attr.zSize;

		vector<int> intrsctnSet;
		vector<int>::const_iterator iter;
		intrsctnQuadTree.SearchRectangle(
							objInfVector[objIndex].ObjBoundingBox,
							intrsctnSet);

		if ( intrsctnSet.size() != 0 ){

			for (
					iter = intrsctnSet.begin();
					iter != intrsctnSet.end();
					iter++) {

					// If the object is below the intersection, or
					//	if it is above the intersection more than
					//	cQRY_TERRAIN_SNAP units, then the object is not
					//	on the intersection.
					if ( (objTop < pIntrsctnPool[*iter].elevation ) ||
						 (objBot > pIntrsctnPool[*iter].elevation + 
						  cQRY_TERRAIN_SNAP) ) {
						//cout << "object[" << realObjIndex 
						//	 << "].z = [" << objTop << ", " << objBot
						//	 << "] is above or below intrsctn["
						//	 << *iter << "].z = " 
						//	 << pIntrsctnPool[*iter].elevation
						//	 << endl;
						continue;
					}

				if (pObjects[realObjIndex].type == eCV_TERRAIN) {
					pIntrsctnPool[*iter].intrsctnFlag |= eTERRN_OBJ;
				}

				pObjRef  = (cvTObjRef * )realloc((void*) pObjRef,
										(lastRefObj + 1) * sizeof(cvTObjRef));
				pObjRef[lastRefObj].next = 0;
				pObjRef[lastRefObj].objId = realObjIndex;

				UpdateRefOfIntrsctn(pObjRef, pIntrsctnPool, *iter, lastRefObj);

				lastRefObj++;
				
			}
		}
	}
				
			
	// If we are in "LRI compiling time" we add to the 
	// "objInfVector" the repeated objects.
	// It is not possible to have repeated objects elswere
	// then in the LRI file.
	if (newObjRefPool) {
		AddRepObjToObjVector(
						pRoad, 
						roadSize, 
						pCntrlPnts, 
						cntrlPntsSize, 
						pIntrsctnPool,
						sizeOfIntrsctnPool,
						pObjects,
						objSize, 
//						pObjAttr, 
//						objAttrSize, 
						pRepObj, 
						objInfVector);
	} else {		
		// If the procedure is used at run time 
		// (To add objects, not during the LRI compilation)
		// Whe have to find the last referenced object, so that
		// whe cann add references to the object reference pool
		// whitout overwrite the alredi existing one
		while ((pObjRef[lastRefObj].objId != 0) & (lastRefObj < *pObjRefSize)){
			lastRefObj++;
		}
	}
	

	//////////////////////////////////////////////////////
	// this part is to check if objects are within road //
	// if it is, update the obj reference pool which is //
	// indexed by "objRefIdx" field of the cvTCntrlPnt  //
	//////////////////////////////////////////////////////
	for (objIndex = 0; objIndex < objInfVector.size(); objIndex++) {

		int realObjIndex = objInfVector[objIndex].realObjRef;

		objTop = objInfVector[objIndex].position.m_z + 
				 0.5*pObjects[realObjIndex].attr.zSize;
		objBot = objInfVector[objIndex].position.m_z - 
				 0.5*pObjects[realObjIndex].attr.zSize;

		// use the object's bbox and the search funcion provided
		// by the quadtree to find out the bboxes of roadpiece that  
		// contain or overlap with the object's bbox
		vector<int>  roadPiecesSet;
		vector<int>::const_iterator  riter;
		roadPicesQuadTree.SearchRectangle(
								objInfVector[objIndex].ObjBoundingBox,
								roadPiecesSet);
			
		// if there are some road pieces where the object belongs
		if ( roadPiecesSet.size() != 0 ) {
			for (
					riter = roadPiecesSet.begin(); 
					riter != roadPiecesSet.end(); 
					riter++) {				
				// first control point of the road piece
				int firstCp = pRoad[pRoadPieces[*riter].roadId].cntrlPntIdx + 
								pRoadPieces[*riter].first;

				//last control point of the road piece
				int lastCp  = pRoad[pRoadPieces[*riter].roadId].cntrlPntIdx + 
								pRoadPieces[*riter].last;

				CPoint2D segment[4];
				int cntrlPIndex;
				for (
						cntrlPIndex = firstCp; 
						cntrlPIndex < lastCp; 
						cntrlPIndex++) {

					// If the object is below the control point, or
					//	if it is above the control point more than
					//	cQRY_TERRAIN_SNAP units, then the object is not
					//	on the road.
					if ( (objTop < pCntrlPnts[cntrlPIndex].location.z) ||
						 (objBot > pCntrlPnts[cntrlPIndex].location.z + 
						  cQRY_TERRAIN_SNAP) ) {
						//cout << "object[" << realObjIndex 
						//	 << "].z = [" << objTop << ", " << objBot
						//	 << "] is above or below cntrlpnt["
						//	 << cntrlPIndex << "].z = " 
						//	 << pCntrlPnts[cntrlPIndex].location.z
						//	 << endl;
						continue;
					}
					
					GetSegment( 
						cntrlPIndex, 
						cntrlPIndex + 1, 
						pCntrlPnts, 
						cntrlPntsSize, 
						pLatCntrlPnts, 
						latCntrlPntsSize, 
						segment);

					bool overlaps = RectangleIntersection(
										objInfVector[objIndex].fourCorner,
										segment);	

					if (overlaps) {
						if (pObjects[realObjIndex].type == eCV_TERRAIN) {
							pCntrlPnts[cntrlPIndex].cntrlPntFlag |= 
											eTERRN_OBJ; //aggiungere and
						}

						if (objInfVector[objIndex].repeated) {
							pCntrlPnts[cntrlPIndex].cntrlPntFlag |= 
											eREP_OBJ_FLAG; //aggiungere and
							continue;
						}

						if (newObjRefPool) {
							pObjRef  = (cvTObjRef * )realloc((void*) pObjRef, 
										(lastRefObj + 1) * sizeof(cvTObjRef));
						}

						pObjRef[lastRefObj].next = 0;
						pObjRef[lastRefObj].objId = realObjIndex;

						UpdateRefOfCntrlPnt(
										pObjRef, 
										pCntrlPnts, 
										cntrlPIndex, 
										lastRefObj);
						
						lastRefObj++;
					}// if (overlaps)
				} //each control point
			}// for each road
		}// end if road.size() = 0 
	}// while (last object)

	if (newObjRefPool)
	{
		*pObjRefSize = lastRefObj + cOBJ_REF_POOL_MEM_RES;
		pObjRef  = (cvTObjRef * ) realloc ((void * ) pObjRef, 
			(*pObjRefSize) * sizeof(cvTObjRef));

		int count = 0;
		for (count = lastRefObj; count < *pObjRefSize; count++)
		{
			pObjRef[count].objId = 0;
			pObjRef[count].next = 0;
		}
	}
}

/************************************************************************/

void CreateObjVector(
		cvTObj*				pObjects,	 
		int					objSize,	
//		cvTObjAttr*			pObjAttr,
		vector<TObjInf>&	objInfVector)
{
	int objIndex;
	objInfVector.clear();
	objInfVector.reserve(objSize);

	for (objIndex = cNUM_DYN_OBJS; objIndex < objSize; objIndex++){
		TObjInf objInfo;
		objInfo.realObjRef = objIndex;
		objInfo.ObjBoundingBox.SetMinX (
			pObjects[objIndex].stateBufA.state.anyState.boundBox[0].x);
		objInfo.ObjBoundingBox.SetMinY (
			pObjects[objIndex].stateBufA.state.anyState.boundBox[0].y); 
		objInfo.ObjBoundingBox.SetMaxX (
			pObjects[objIndex].stateBufA.state.anyState.boundBox[1].x);
		objInfo.ObjBoundingBox.SetMaxY (
			pObjects[objIndex].stateBufA.state.anyState.boundBox[1].y);

		objInfo.position = pObjects[objIndex].stateBufA.state.anyState.position;

		GetFourCornersForNonRepObj(
								pObjects,
								objIndex,
//								pObjAttr,
								objInfo.fourCorner);
		objInfo.repeated = false;
		objInfVector.push_back(objInfo);
	}
}


/************************************************************************/

void AddRepObjToObjVector(
		cvTRoad*			pRoad, 
		int					roadSize, 
		cvTCntrlPnt*		pPnts,
		int					cntrlPntsSize, 
		cvTIntrsctn*		pIntrsctnPool,
		int					sizeOfIntrsctnPool,
		cvTObj*				pObjects, 
		int					objSize, 
//		cvTObjAttr*			pObjAttr, 
//		int					objAttrSize,
		cvTRepObj*			pRepObj, 
		vector<TObjInf>&	objInfVector)
{
	int roadIndex;
	for (roadIndex = 1; roadIndex < roadSize; roadIndex++)
	{    
		int				numRepObj = pRoad[roadIndex].numRepObj;
		TRepObjPoolIdx	repObjIdx = pRoad[roadIndex].repObjIdx;
		int				repObjCount;
		
		for (repObjCount = 0; repObjCount < numRepObj; repObjCount++)
		{
			int		curRepIdx		= repObjIdx + repObjCount;
			float	start			= pRepObj[curRepIdx].start;
			float	latDist			= pRepObj[curRepIdx].latdist;
			float	period			= pRepObj[curRepIdx].period;
			float	end				= pRepObj[curRepIdx].end;
			int		numOfIstaces	= int((end - start) / period) + 1;

			int		cur				= pRoad[roadIndex].cntrlPntIdx;
			int		numCntrlPnt		= pRoad[roadIndex].numCntrlPnt;
				
			float	xSize			= pObjects[pRepObj[curRepIdx].objId].attr.xSize;
			float	ySize			= pObjects[pRepObj[curRepIdx].objId].attr.ySize;

			int istanceCount;
			
			for (istanceCount=0; istanceCount<numOfIstaces; istanceCount++) {
				float distance = start + period * istanceCount;

				bool betweenCntrlPnts;
				betweenCntrlPnts = IsBetweenPnts(distance, pPnts, cur);
				
				while ((!betweenCntrlPnts) && (cur < (numCntrlPnt-1))) {
					cur++;
					betweenCntrlPnts = IsBetweenPnts( distance, pPnts, cur);
				}
				//////////////////////////////////////////
				// which means the object might be      //
				// placed in intersection               //
				//////////////////////////////////////////
				if (cur == (numCntrlPnt - 1)) {
					continue;
				}

				distance -= pPnts[cur].cummulativeLinDist;

				TObjInf		objInfo;
				float		tPar = distance / pPnts[cur].distToNextCubic; 
				float		tPar2 = tPar * tPar;
				float		tPar3 = tPar2 * tPar;
				
				CPoint3D pntOfSpline(
							pPnts[cur].hermite[0].A * tPar3 +
							pPnts[cur].hermite[0].B * tPar2 +
							pPnts[cur].hermite[0].C * tPar +
							pPnts[cur].hermite[0].D,
							pPnts[cur].hermite[1].A * tPar3 +
							pPnts[cur].hermite[1].B * tPar2 +
							pPnts[cur].hermite[1].C * tPar +
							pPnts[cur].hermite[1].D,
							pPnts[cur].hermite[2].A * tPar3 +
							pPnts[cur].hermite[2].B * tPar2 +
							pPnts[cur].hermite[2].C * tPar +
							pPnts[cur].hermite[2].D);
				CVector3D normalOfPnt(
							pPnts[cur].normal.i + tPar *
							(pPnts[cur+1].normal.i - pPnts[cur].normal.i),
							pPnts[cur].normal.j + tPar *
							(pPnts[cur+1].normal.j - pPnts[cur].normal.j),
							pPnts[cur].normal.k + tPar *
							(pPnts[cur+1].normal.k - pPnts[cur].normal.k) );
				normalOfPnt.Normalize();
				CVector3D tangentOfPnt(	
							pPnts[cur].hermite[0].A * tPar2 * 3 +
							pPnts[cur].hermite[0].B * tPar * 2 +
							pPnts[cur].hermite[0].C,
							pPnts[cur].hermite[1].A * tPar2 * 3 +
							pPnts[cur].hermite[1].B * tPar * 2 +
							pPnts[cur].hermite[1].C,
							pPnts[cur].hermite[2].A * tPar2 * 3 +
							pPnts[cur].hermite[2].B * tPar * 2 +
							pPnts[cur].hermite[2].C );
				tangentOfPnt.Normalize();
				CVector3D rightOfPnt = tangentOfPnt.CrossP(normalOfPnt); 
				rightOfPnt.Normalize();

				objInfo.position = pntOfSpline + latDist * rightOfPnt;

				CVector3D	lat(rightOfPnt);
				CVector3D	forw(tangentOfPnt);

				lat.Scale(0.5 * ySize);
				forw.Scale(0.5 * xSize);
				int objRef = pRepObj[curRepIdx].objId;

				CPoint3D fourCorner[4];
				CPoint3D opt(0.0, 0.0, 0.0);
				fourCorner[0] = opt + ((-1) * forw - lat); 
				fourCorner[1] = opt + ((-1) * forw + lat);
				fourCorner[2] = opt + (       forw + lat);
				fourCorner[3] = opt + (       forw - lat);

				CVector3D tang(
					pObjects[objRef].stateBufA.state.anyState.tangent);
				float c = (float)tang.m_i;
				float s = (float)tang.m_j;
				int index;
				for (index = 0; index < 4; index++) {
					RotateFourCorners(fourCorner[index], c, s);
					objInfo.fourCorner[index] = fourCorner[index] + 
												objInfo.position;
					CPoint2D pt(
								objInfo.fourCorner[index].m_x, 
								objInfo.fourCorner[index].m_y);
					objInfo.ObjBoundingBox += pt;
				}

				objInfo.realObjRef = objRef;
								
				objInfo.repeated = true;
				objInfVector.push_back(objInfo);

			}
		}
	}
}


/////////////////////////////////////////////////////////////////////////
// parameter: c is cos, s is sin, pt is the coordinate to be rotated   //
/////////////////////////////////////////////////////////////////////////
void RotateFourCorners( CPoint3D& pt, float c, float s)
{
	CPoint2D result;
	result.m_x = pt.m_x * c - pt.m_y * s;
	result.m_y = pt.m_x * s + pt.m_y * c;
	pt.m_x = result.m_x;
	pt.m_y = result.m_y;
}

/************************************************************************/
void GetSegment(
		int					curr, 
		int					next, 
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize, 
		cvTLatCntrlPnt*		pLatCntrlPnts,
		int					latCntrlPntsSize, 
		CPoint2D			segment[4])
{
	TLatCntrlPntPoolIdx firstLatCntrlPnt = pCntrlPnts[curr].latCntrlPntIdx;
    TLatCntrlPntPoolIdx lastLatCntrlPnt;

	if (firstLatCntrlPnt != 0) {
		lastLatCntrlPnt = firstLatCntrlPnt +
					pCntrlPnts[curr].numLatCntrlPnt - 1;

		segment[0].m_x = pCntrlPnts[curr].location.x +
					pCntrlPnts[curr].rightVecCubic.i *
					pLatCntrlPnts[firstLatCntrlPnt].offset;
		segment[0].m_y = pCntrlPnts[curr].location.y +
					pCntrlPnts[curr].rightVecCubic.j *
					pLatCntrlPnts[firstLatCntrlPnt].offset;
		segment[1].m_x = pCntrlPnts[curr].location.x +
					pCntrlPnts[curr].rightVecCubic.i *
					pLatCntrlPnts[lastLatCntrlPnt].offset;
		segment[1].m_y = pCntrlPnts[curr].location.y +
					pCntrlPnts[curr].rightVecCubic.j *
					pLatCntrlPnts[lastLatCntrlPnt].offset;
	} else {
		segment[0].m_x = pCntrlPnts[curr].location.x +
					pCntrlPnts[curr].rightVecCubic.i *
					(-pCntrlPnts[curr].logicalWidth/2.0);
		segment[0].m_y = pCntrlPnts[curr].location.y +
					pCntrlPnts[curr].rightVecCubic.j *
					(-pCntrlPnts[curr].logicalWidth/2.0);
		segment[1].m_x = pCntrlPnts[curr].location.x +
					pCntrlPnts[curr].rightVecCubic.i *
					(pCntrlPnts[curr].logicalWidth/2.0);
		segment[1].m_y = pCntrlPnts[curr].location.y +
					pCntrlPnts[curr].rightVecCubic.j *
					(pCntrlPnts[curr].logicalWidth/2.0);
    }

	firstLatCntrlPnt = pCntrlPnts[next].latCntrlPntIdx;
	if (firstLatCntrlPnt != 0) {
		lastLatCntrlPnt = firstLatCntrlPnt +
					pCntrlPnts[next].numLatCntrlPnt - 1;

		segment[2].m_x = pCntrlPnts[next].location.x +
					pCntrlPnts[next].rightVecCubic.i *
					pLatCntrlPnts[lastLatCntrlPnt].offset;
		segment[2].m_y = pCntrlPnts[next].location.y +
					pCntrlPnts[next].rightVecCubic.j *
					pLatCntrlPnts[lastLatCntrlPnt].offset;
		segment[3].m_x = pCntrlPnts[next].location.x +
					pCntrlPnts[next].rightVecCubic.i *
					pLatCntrlPnts[firstLatCntrlPnt].offset;
		segment[3].m_y = pCntrlPnts[next].location.y +
					pCntrlPnts[next].rightVecCubic.j *
					pLatCntrlPnts[firstLatCntrlPnt].offset;
	} else {
		segment[2].m_x = pCntrlPnts[next].location.x +
					pCntrlPnts[next].rightVecCubic.i *
					(pCntrlPnts[next].logicalWidth/2.0);
		segment[2].m_y = pCntrlPnts[next].location.y +
					pCntrlPnts[next].rightVecCubic.j *
					(pCntrlPnts[next].logicalWidth/2.0);
		segment[3].m_x = pCntrlPnts[next].location.x +
					pCntrlPnts[next].rightVecCubic.i *
					(-pCntrlPnts[next].logicalWidth/2.0);
		segment[3].m_y = pCntrlPnts[next].location.y +
					pCntrlPnts[next].rightVecCubic.j *
					(-pCntrlPnts[next].logicalWidth/2.0);
	}
}

/************************************************************************/
void GetSegment(
		int					next, 
		cvTCntrlPnt*		pCntrlPnts,
		int					cntrlPntsSize,
		cvTLatCntrlPnt*		pLatCntrlPnts,
		int					latCntrlPntsSize,
		CPoint2D			segment[4])
{
	TLatCntrlPntPoolIdx firstLatCntrlPnt = pCntrlPnts[next].latCntrlPntIdx;
	TLatCntrlPntPoolIdx lastLatCntrlPnt; 

	if (firstLatCntrlPnt != 0) {
		lastLatCntrlPnt = firstLatCntrlPnt + 
				pCntrlPnts[next].numLatCntrlPnt - 1;

		segment[2].m_x = pCntrlPnts[next].location.x + 
					pCntrlPnts[next].rightVecCubic.i * 
					pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[2].m_y = pCntrlPnts[next].location.y + 
					pCntrlPnts[next].rightVecCubic.j * 
					pLatCntrlPnts[lastLatCntrlPnt].offset; 
		segment[3].m_x = pCntrlPnts[next].location.x - 
					pCntrlPnts[next].rightVecCubic.i * 
					pLatCntrlPnts[firstLatCntrlPnt].offset; 
		segment[3].m_y = pCntrlPnts[next].location.y - 
					pCntrlPnts[next].rightVecCubic.j * 
					pLatCntrlPnts[firstLatCntrlPnt].offset; 
	} else {
		segment[2].m_x = pCntrlPnts[next].location.x + 
					pCntrlPnts[next].rightVecCubic.i *
					(pCntrlPnts[next].logicalWidth/2.0); 
		segment[2].m_y = pCntrlPnts[next].location.y + 
					pCntrlPnts[next].rightVecCubic.j *
					(pCntrlPnts[next].logicalWidth/2.0); 
		segment[3].m_x = pCntrlPnts[next].location.x + 
					pCntrlPnts[next].rightVecCubic.i *
					(-pCntrlPnts[next].logicalWidth/2.0); 
		segment[3].m_y = pCntrlPnts[next].location.y + 
					pCntrlPnts[next].rightVecCubic.j *
					(-pCntrlPnts[next].logicalWidth/2.0); 
	}
}

bool IsBetweenPnts(float distance, cvTCntrlPnt* pCntrlPnts, int cntrlPntsIndex)
{
	bool bigger  = ( (distance) >= 
						pCntrlPnts[cntrlPntsIndex].cummulativeCubicDist);
	bool smaller = ( (distance)< 
						pCntrlPnts[cntrlPntsIndex+1].cummulativeCubicDist);
	return (bigger && smaller);
}

////////////////////////////////////////////////////////////////////////
