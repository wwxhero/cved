
/***************************************************************************
 * (C) Copyright 1998 by NADS & Simulation Center The University of Iowa
 * and The University of Iowa.  All rights reserved.
 *
 * $Id: codegen.cxx,v 1.7 2015/10/06 19:40:06 IOWA\dheitbri Exp $
 *
 * Author(s) :
 * Date:
 *
 * Description:
 * This file contains the code that generates the memory block used
 * by CVED.
 *
 **************************************************************************/
#ifdef _WIN32
#include <ostream>
#include <iostream>
#elif __sgi
#include <typeinfo>
#include <iostream.h>
#include <unistd.h>
#endif

#include <vector>
#include <algorithm>

using namespace std;

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cvedversionnum.h>
#include "parser.h"
#include "dbgdump.h"
#include "err.h"
#include <cvedversionnum.h>
#include <splineHermite.h>
#include "splineHermiteNonNorm.h"
#define DEBUG_DUMP


// the LRI file header information as produced by the parser
extern "C" { extern THeader gHeader; extern int storageOfCntrlPntPool; }

/*
 *	used for matching longitudinal to lateral
 *	only used in this file temparily
 */
typedef struct TLat{
	char				curveName[cMAX_WORDLENGTH];
	float				physicalWidth;
	int					numOfLatPnt;
	TLatCntrlPntPoolIdx	idx;
}TLat;


/*
 * used for making a road border if one doesn't exist
 * only used in this file temporarily
 */

class CEndRoadPoints {
public:
	CPoint3D point1;
	CPoint3D point2;
	CVector3D tangent;
	bool operator<( const CEndRoadPoints& road2 ) const;
	bool operator>( const CEndRoadPoints& road2 ) const;
	bool operator==( const CEndRoadPoints& road2 ) const;
	bool operator!=( const CEndRoadPoints& road2 ) const;
};

inline
bool CEndRoadPoints::operator<( const CEndRoadPoints& road2 ) const {
	double tan1, tan2;
	tan1 = atan2( tangent.m_j, tangent.m_i );
	tan2 = atan2( road2.tangent.m_j, road2.tangent.m_i );
	
	if (tan1 < tan2) return true;
	return false;
}

inline
bool CEndRoadPoints::operator>( const CEndRoadPoints& road2 ) const {
	return ! operator<( road2 );
}

inline
bool CEndRoadPoints::operator==( const CEndRoadPoints& road2 ) const {
	double tan1, tan2;
	tan1 = atan2( tangent.m_j, tangent.m_i );
	tan2 = atan2( road2.tangent.m_j, road2.tangent.m_i );
	
	if (tan1 == tan2) return true;
	return false;
}

inline
bool CEndRoadPoints::operator!=( const CEndRoadPoints& road2 ) const {
	return ! operator!=( road2 );
}

static TLat			*pLatPool = NULL;
static int			sizeOfLatPool = 1;

CQuadTree			rdPcQTree;
CQuadTree			intrsctnQTree;
CQuadTree			staticObjQTree;
CQuadTree			trrnObjQTree;

static void BuildObjQTree(void);
void SetCharPool(const char*); 
static void SetAttrPool(const TAttribute*, int*, int);
static void SetRepObjPool(const TRepObj*, int*);
static void SetCntrlPntPool(const TControlPoint*, int, const TLaneDef*, int);
static void SetAttrName(void);
static void BuildLatCntrlPntPool( const TCurveDef*, float, int*);
static void BuildLatPool(void);
static int GetIdxAndNumOfLatCntrl(
							const char*, 
							TLatCntrlPntPoolIdx*, 
							int*, 
							float*);
static void SetIntrsctnPool(TIntersection*, const TElevMap*, CQuadTree&);
static int SetRoadIdOfIntrsctn(TRoadName*);
static void SetBorderSegPool(TBorder*);
static void SetCorridorPool( TCrdr*, int, int*);
static TRoadPoolIdx GetSrcRdIdx(TCrdr*);
static TRoadPoolIdx GetDstRdIdx(TCrdr*);
static TLanePoolIdx GetSrcLnIdx(TCrdr*);
static TLanePoolIdx GetDstLnIdx(TCrdr*);
static void SetIntrsctnIdxOfRoad(TCrdrPoolIdx, TIntrsctnPoolIdx);
static void SetIntrsctnOfRoad(TRoads*);

static void SetHldOfsPool(THoldOfs*, int*);
static float GetCrdrCntrlPntDistance(int, int);
static void SetCrdrCntrlPntPool(const char *, TCrdrCurveList*);
static void SetCrdrMrgDstPool(cvTIntrsctn*, int&, cvTCrdrMrgDst**);
static float GetCntrlPntDistance(int);
static void SetRoadPiecePool(TRoadPoolIdx, float, float, CQuadTree&);
static void AllocRoadPiece(void);
static void InitItem(
					CQuadTree::CItem&, 
					int, 
					float,
					float,
					float,
					float);
static void InitRoadPiece(
					cvTRoadPiece*,
					TRoadPoolIdx,
					TRoadPiecePoolIdx,
					int,
					int,
					float,
					float,
					float,
					float);
static bool HitTest(cvTRoadPiece, float, float);
static void FindBoundaryOfRoadPiece(
								float*, 
								float*, 
								float*, 
								float*, 
								TLongCntrlPntPoolIdx);
static void FindBoundaryOfIntrsctn(
			                    cvTIntrsctn*    pEntry,
            			        CPoint2D&       llPt,
                   				CPoint2D&       urPt);

static bool NormalizeVector(TVector3D *);
static void CrossProductVector( TVector3D *, TVector3D *, TVector3D * );
static float DotProductVector( TVector3D *, TVector3D * );
static bool ComputeLinTangent(TPoint3D*, TPoint3D*, TVector3D*);
static float ComputeLinDist(const TPoint3D*, const TPoint3D*);
static float ComputeCubicDist( cvTSplCoef * );
static float EvalSqrt( cvTSplCoef *, float );
static void Compute2ndDerivatives(float*, TPoint3D*, int, 
										TVector3D, TVector3D, TVector3D*);
#if 0
static float ComputeRadiusOfCurvature(cvTSplCoef*);
#endif

static void InitMemPool(void);
static void	AllocateDynObjRefPool(void);
static void	AllocateRoadRefPool(void);
static void	AllocateIntrsctnRefPool(void);

static void ProcessCntrlPntAttr(TAttribute*, int, int, float*, bool, bool*, bool*);
static void SetLanePoolFromRoad(const TLaneDef*, int, float*);
static void SetLanePoolFromCntrlPnt(int, int, float*, float*);
static void SetLanePoolFromCntrlPntAttr(
									TAttribute*, 
									int, 
									int, 
									float*, 
									bool);
static float Min(vector<float> vec);
static float Max(vector<float> vec);


#ifdef DEBUG_DUMP
static void DumpRoad(void);
static void DumpLat(void);
static void DumpIntrsctn(void);
#endif


static void DumpLat(void)
{
	int i; 
	for (i=1; i<sizeOfLatPool; i++) 
		printf("\n %d th name:%s Width:%.1f num:%d	idx %d", 
						i, 
						pLatPool[i].curveName,
						pLatPool[i].physicalWidth, 
						pLatPool[i].numOfLatPnt,
						pLatPool[i].idx);
	printf("\n");
	for (i=1; i<sizeOfLatCntrlPntPool; i++)
		printf("\n %d th offset %.1f	height %.1f	material %d",
				i,
				pLatCntrlPntPool[i].offset,
				pLatCntrlPntPool[i].height,
				pLatCntrlPntPool[i].material);
	printf("\n");
	printf("\n haha  %s and \"%c\"\n", &pCharPool[1], pCharPool[0]);
}

static void ComputeHermiteSplineC2Coef( float p0, float p1, float dp2p0, 
	float dp2p1, cvTSplCoef *pCoef )
{

	pCoef->A = (-dp2p0+dp2p1)/6.0;	
	pCoef->B = dp2p0/2.0;
	pCoef->C = -p0+p1-dp2p0/3.0-dp2p1/6.0;
	pCoef->D = p0;

}
		

static void ComputeHermiteSplineC2Coef3D(
								TPoint3D*	p0, 
								TPoint3D*	p1,
								TVector3D*	dp2p0, 
								TVector3D*	dp2p1,
								cvTSplCoef	coef[3] )
{
	ComputeHermiteSplineC2Coef( p0->x, p1->x, dp2p0->i, dp2p1->i, 
											&coef[0] );
	ComputeHermiteSplineC2Coef( p0->y, p1->y, dp2p0->j, dp2p1->j, 
											&coef[1] );
	ComputeHermiteSplineC2Coef( p0->z, p1->z, dp2p0->k, dp2p1->k, 
											&coef[2] );
}

/*-----------------------------------------------------------------------*
 *
 *
 * Description: build the elevation post terrain elevation map pools
 *
 * This function receives a pointer to the linked list of 
 * elevation maps and builds two pools.  One holds all elevation
 * points used in the various elevation maps, and the second
 * one holds the elevation maps themselves.
 *
 */
static void 
MakeElevMapPools(
	const TElevMap*			pElevMaps,
	vector<cvTElevPost>&	posts,
	vector<cvTElevMap>&		maps
	)
{
	cvTElevPost zeroPost = { 0 };
	cvTElevMap  zeroMap  = { 0 };

	// initialize the arrays that represent the pools
	posts.clear();
	maps.clear();

	// add element 0; it's not used since 0 is used as a terminator
	// or to catch unitialized indeces.
	posts.push_back(zeroPost);
	maps.push_back(zeroMap);

	const TElevMap *pMap;
	for (  pMap = pElevMaps; pMap; pMap = pMap->pNextElevMap ) {
		cvTElevMap   oneMap = { 0 };

		oneMap.x1        = 0.0;
		oneMap.y1        = 0.0;
		oneMap.numRows   = pMap->n_rows;
		oneMap.numCols   = pMap->n_cols;
		oneMap.res       = pMap->res;
		oneMap.postIdx   = posts.size();
		oneMap.numPosts  = 0;		// initial value
		
		// go through the linked list of elevation posts.  Note that
		// the names used for these structures are not consistent with
		// their usage, but given time contraints we leave the legacy
		// code as is.
		TElevMap2 *pPost;
		int        count = 0;	// count posts for error checking
		for (pPost = pMap->pElevMap2; pPost; pPost = pPost->pNextElevMap2) {
			cvTElevPost onePost = { 0 };

			onePost.z      = pPost->z;
			onePost.flags  = pPost->materialRef;

			posts.push_back(onePost);
			count++;
		}

		if ( count != pMap->n_rows * pMap->n_cols ) {
			lrierr(eINCONSISTENT_POST_COUNT, "\nMap name is '%s', post count %d row col count %d x %d", 
					pMap->elevMapName, count, pMap->n_rows, pMap->n_cols);
			exit(-1);
		}

		oneMap.numPosts = count;

		maps.push_back(oneMap);
	}
}


/*-----------------------------------------------------------------------*
 *
 *	Name: GenerateCode
 *
 *	REENTRANT
 *
 *  The function creates seperate memory pools for different road-way
 *  network elements.  For instance, it creates a character pool that
 *  is a block of memory containing all the strings road names, intersection
 *  names, control point names, object names, etc.  It also creates control
 *  point pools seperately for both longitudinal and lateral control points.
 *  See cvedstrc.h in the include directory for a detailed view of these
 *  pools.  For the control point pool, some computations are made to
 *  generate Hermite spline coefficients, radius of curvatures at control
 *  points, determinng tangents and lateral vectors at these control points
 *  arc lengths, etc.  All this information is placed inside the pool.  All
 *  This function assumes that all information in the linked lists that
 *  are provided as parameters to this function contain correct and non-redu-
 *  ndant information (as per request).  No error checking for the data is
 *  made here.
 * 
 *  Inputs:
 *		pRoads:  This is a pointer to the begining of a linked list
 *				 containing all relevant information about roads.
 *		pInter:  This is a pointer to the begining of a linked list
 *				 containing all relevant information about intersections.
 *		pElevMaps:  This is a pointer to the begining of a linked list
 *				 containing information about elevation for intersections.
 *
 *  Outputs:
 *		No explicit output is made.  The function merely creates global
 *		memory pools seperately.
 *	Return Value:
 *		The function is of type void and as such does not return a value.
 *		However, in case of a memory allocation problem that might occur
 *		within this function, it prints an error message indicating the
 *		type of error.
 *-------------------------------------------------------------------------*/

void GenerateCode(
	TRoads*					pRoads, 
	TIntersection*			pInter,
	TElevMap*				pElevMaps,
	vector<cvTElevPost>&	posts,
	vector<cvTElevMap>&		maps)
{
	TRoads*					pCurrentRoad = pRoads;
	
	InitMemPool();
	BuildLatPool();

	while( pCurrentRoad ){
		TLaneDef*		pLane;				/* poitner to the lane list */
		TAttribute*		pAttr;				/* pointer to the attr list */
		TRepObj*		pRepObj;			/* pointer to the rep objs list*/
		TControlPoint*	pCntrlPnt;			/* pointer to the lngCntrlPnt list*/

		sizeOfRoadPool++;

		pRoadPool = (struct cvTRoad*)realloc(pRoadPool, 
										sizeof(cvTRoad)*sizeOfRoadPool);
		if (!pRoadPool){
			lrierr(eROAD_POOL_ALLOC_FAIL, "");
			exit(1);
		} else
			memset(pRoadPool+sizeOfRoadPool-1, 0, sizeof(cvTRoad));


		pRoadPool[sizeOfRoadPool-1].myId = sizeOfRoadPool - 1;
		pRoadPool[sizeOfRoadPool-1].nameIdx = sizeOfCharPool;
		SetCharPool(pCurrentRoad->roadName); 	

		pRoadPool[sizeOfRoadPool-1].attrIdx = sizeOfAttrPool;
		pAttr = pCurrentRoad->pAttribute;
		int	numAttr = 0;	/* number of attribute */
		SetAttrPool( pAttr, &numAttr, cCV_LRI_ROAD_ATTR_EXTRA);
		pRoadPool[sizeOfRoadPool-1].numAttr = numAttr;

		pRoadPool[sizeOfRoadPool-1].repObjIdx = sizeOfRepObjPool;
		pRepObj = pCurrentRoad->pRepObjs;
		int	numRepObj = 0;		/* number of repeated objects */
		SetRepObjPool(pRepObj, &numRepObj);
		pRoadPool[sizeOfRoadPool-1].numRepObj = numRepObj;

		pRoadPool[sizeOfRoadPool-1].cntrlPntIdx = sizeOfCntrlPntPool;
		int	numCntrlPnt = pCurrentRoad->longitCurve.n_cpoints;
		pRoadPool[sizeOfRoadPool-1].numCntrlPnt = numCntrlPnt;
		pLane = pCurrentRoad->lanelist.lanes;
		pCntrlPnt = pCurrentRoad->longitCurve.pCpointList;
		int numOfLanes = pCurrentRoad->lanelist.n_lanes;
		pRoadPool[sizeOfRoadPool-1].numOfLanes = numOfLanes;
		pRoadPool[sizeOfRoadPool-1].laneIdx = sizeOfLanePool;
		SetCntrlPntPool(pCntrlPnt, numCntrlPnt, pLane, numOfLanes);

		pRoadPool[sizeOfRoadPool-1].roadLengthLinear = 
					pCntrlPntPool[sizeOfCntrlPntPool-1].distance;

		pRoadPool[sizeOfRoadPool-1].roadLengthSpline = 
					pCntrlPntPool[sizeOfCntrlPntPool-1].cummulativeCubicDist;

		// move this line down to outside of the road loop
		//SetAttrName();
		SetRoadPiecePool(
					sizeOfRoadPool-1, 
					widthOfBoundingBox, 
					heightOfBoundingBox, 
					rdPcQTree);

		pCurrentRoad = pCurrentRoad->pNext;
	}
   
	SetAttrName();


	/* produce elevation map pool */
	if ( pElevMaps ) {
		MakeElevMapPools(pElevMaps, posts, maps);
	}

	SetIntrsctnPool(pInter, pElevMaps, intrsctnQTree);
	SetIntrsctnOfRoad(pRoads);
	
	BuildObjQTree(); 

	rdPcQTree.Optimize();
	intrsctnQTree.Optimize();
	staticObjQTree.Optimize();
	trrnObjQTree.Optimize();

	AllocateDynObjRefPool();
	AllocateRoadRefPool();
	AllocateIntrsctnRefPool();

#if 0
	DumpHldOfsPool(pHldOfsPool, sizeOfHldOfsPool);
	DumpObj(pObjPool, 385);
	DumpObj(pObjPool, 386);
	DumpObj(pObjPool, 387);
	int in = 1;
	for (; in < sizeOfIntrsctnPool; in++){
		int nameIdx = pIntrsctnPool[in].nameIdx; 
		if (!strcmp(&pCharPool[nameIdx], "dummy3_tn1")){
			printf("\n============== intrsctn if of dummy3_tn1 is %d", in);
			break;
		}
	}
	int cr = pIntrsctnPool[in].crdrIdx;
	int numOfCrdrs = pIntrsctnPool[in].numOfCrdrs;
	
	for(; cr < pIntrsctnPool[in].crdrIdx + numOfCrdrs; cr++){
		int hl = pCrdrPool[cr].hldOfsIdx;
		int numHldOfs = pCrdrPool[cr].numHldOfs;
		
		for (; hl < pCrdrPool[cr].hldOfsIdx + numHldOfs; hl++){
				printf("\n in %d th hldPool", hl);
				printf("\tobjId         = %.2d\n", pHldOfsPool[hl].objIdx);
    		    printf("\tdistance  = %.2f\n", pHldOfsPool[hl].distance);
    			 printf("\torientation   = %.2f\n", pHldOfsPool[hl].orientation);
       			 printf("\tthickness = %.2f\n", pHldOfsPool[hl].thickness);
       			 printf("\treason");
       			 if (pHldOfsPool[hl].reason & eHOLD_SIGN)
       			     printf("\t%s", "SIGN");
       			 else if (pHldOfsPool[hl].reason & eHOLD_TLIGHT)
       			     printf("\t%s", "TIGHT");
       			 else if (pHldOfsPool[hl].reason & eHOLD_OVLAP)
       			     printf("\t%s", "OVLAP");
       			 else
       		     printf("\t\t%s", "No reason");
       			 printf("\t %d", pHldOfsPool[hl].reason);
       			 printf("\n");
		}
	}

	
	printf("\n the sizeOfHldOfsPool is %d \n", sizeOfHldOfsPool);
#endif

	if ( gDebugDump ) {
		DumpRoad();	
		DumpLat();
		DumpIntrsctn();
	}
	free(pLatPool);
}


/*---------------------------------------------------------------------------*
 *
 *	Name: InitMemPool
 *	
 *	NON-REENTRANT
 *
 *	This function allocates and initializes all memory pools. The sizes of 
 *	pools are also changed to reflect the actual sizes of the pools. The space 
 *	allocated here will never be used to store information. All indexes to any 
 *	memory pool is refered	as either Null or invalid if the index is 0.
 *
 *--------------------------------------------------------------------------*/

static void InitMemPool(void){

	sizeOfRoadPool++;
	pRoadPool = (struct cvTRoad*)realloc(pRoadPool, sizeof(cvTRoad));
	if (!pRoadPool){
		lrierr(eROAD_POOL_ALLOC_FAIL, "");
		exit(1); 
	}
	memset(pRoadPool, 0, sizeof(cvTRoad));

	sizeOfCharPool++;
	pCharPool = (char*)realloc(pCharPool, sizeof(char));
	if (!pCharPool){
		lrierr(eCHAR_POOL_ALLOC_FAIL, "");
		exit(1); 
	}
	memset(pCharPool, 0, sizeof(char));

	sizeOfAttrPool++;
	pAttrPool = (cvTAttr*)realloc(pAttrPool, sizeof(cvTAttr));
	if (!pAttrPool){
		lrierr(eATTR_POOL_ALLOC_FAIL, "");
		exit(1); 
	}
	memset(pAttrPool, 0, sizeof(cvTAttr));

	sizeOfRepObjPool++;
	pRepObjPool = (cvTRepObj*)realloc(pRepObjPool, sizeof(cvTRepObj));
	if (!pRepObjPool){
		lrierr(eREP_OBJ_POOL_ALLOC_FAIL, "");
		exit(1); 
	}
	memset(pRepObjPool, 0, sizeof(cvTRepObj));

	sizeOfLanePool++;
	pLanePool = (cvTLane*)realloc(pLanePool, sizeof(cvTLane));
	if (!pLanePool){
		lrierr(eLANE_POOL_ALLOC_FAIL, "");
		exit(1); 
	}
	memset(pLanePool, 0, sizeof(cvTLane));

	sizeOfCntrlPntPool++;
	storageOfCntrlPntPool = 10;
	pCntrlPntPool = (cvTCntrlPnt*)calloc(storageOfCntrlPntPool, sizeof(cvTCntrlPnt));
	if (!pCntrlPntPool){
		lrierr(eCNTRL_PNT_POOL_ALLOC_FAIL, "");
		exit(1); 
	}

	sizeOfLatCntrlPntPool++;
	pLatCntrlPntPool = (cvTLatCntrlPnt*)realloc(
											pLatCntrlPntPool, 
											sizeof(cvTLatCntrlPnt));
	if (!pLatCntrlPntPool){
		lrierr(eLAT_CNTRL_PNT_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	memset(pLatCntrlPntPool, 0, sizeof(cvTLatCntrlPnt));

	sizeOfIntrsctnPool++;
	pIntrsctnPool = (cvTIntrsctn*)realloc(pIntrsctnPool, sizeof(cvTIntrsctn));
	if (!pIntrsctnPool){
		lrierr(eINTRSCTN_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	memset(pIntrsctnPool, 0, sizeof(cvTIntrsctn));

	sizeOfBorderSegPool++;
	pBorderSegPool = (cvTBorderSeg*)realloc(
										pBorderSegPool, 
										sizeof(cvTBorderSeg));
	if (!pBorderSegPool){
		lrierr(eBORDER_SEG_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	memset(pBorderSegPool, 0, sizeof(cvTBorderSeg));

	sizeOfCrdrPool++;
	pCrdrPool = (cvTCrdr*)realloc(pCrdrPool, sizeof(cvTCrdr));
	if (!pCrdrPool){
		lrierr(eCRDR_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	memset(pCrdrPool, 0, sizeof(cvTCrdr));

	sizeOfCrdrCntrlPntPool++;
	pCrdrCntrlPntPool = (cvTCrdrCntrlPnt*)realloc(
											pCrdrCntrlPntPool, 
											sizeof(cvTCrdrCntrlPnt));
	if (!pCrdrCntrlPntPool){
		lrierr(eCRDR_CNTRL_PNT_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	memset(pCrdrCntrlPntPool, 0, sizeof(cvTCrdrCntrlPnt));

	sizeOfHldOfsPool++;
	pHldOfsPool = (cvTHldOfs*)realloc(pHldOfsPool, sizeof(cvTHldOfs));
	if (!pHldOfsPool){
		lrierr(eHLD_OFS_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	memset(pHldOfsPool, 0, sizeof(cvTHldOfs));

	sizeOfRoadPiecePool++;
	pRoadPiecePool = (cvTRoadPiece*)realloc(
										pRoadPiecePool, 
										sizeof(cvTRoadPiece));
	if (!pRoadPiecePool){
		lrierr(eROAD_PIECE_ALLOC_FAIL, "");
		exit(1);
	}

	sizeOfCrdrMrgDstPool++;
	pCrdrMrgDstPool = (cvTCrdrMrgDst*)realloc(
										pCrdrMrgDstPool,
										sizeof(cvTCrdrMrgDst));
	if (!pCrdrMrgDstPool){
		lrierr(eCRDR_MRG_DST_ALLOC_FAIL, "");
		exit(1);
	}

}

/*---------------------------------------------------------------------------*
 *
 *	Name: SetCharPool
 *
 *	NON-REENTRANT
 *
 *	This function take an argument name of type const char*, and store it
 *	in the char memory pool. New memory space for the "name" is allocated
 *	and initialized, and sizeOfCharPool is also increased correctly.
 *
 *	Input:
 *		name: a pointer pointing to the first char of a string of chars.
 *
 *---------------------------------------------------------------------------*/
void SetCharPool(const char* name)
{
	int lengthOfName; 				/* length of name */
	int oldSize = sizeOfCharPool;	/* size before this function gets called */

	lengthOfName = strlen(name);
	sizeOfCharPool += (lengthOfName+1);
	pCharPool = (char*)realloc(pCharPool, sizeof(char)*sizeOfCharPool);
	if(!pCharPool){
		lrierr(eROAD_POOL_ALLOC_FAIL, "");
		exit(1); 
	}
	else
		memset(pCharPool+oldSize, 0, (lengthOfName+1)*sizeof(char));
	strcpy(pCharPool+oldSize, name);
}

/*---------------------------------------------------------------------------*
 *
 *	Name: SetAttrPool
 *
 *	NON-REENTRANT
 *	
 *	This function reads the attribute information from the pointer pAttr 
 *	which points to TAttribute, and store the information in the attribute 
 *	pool. New memory space of the attribute pool is allocated and initilized 
 *	and the size of the pool is increased correctly to reflect the added 
 *	space. For "nameIdx" field, it will be done in the function "SetAttrName",
 *	and for the "next" field, we leave it out at this time. It might be added
 *	later on.
 *	
 *	Inputs:
 *		pAttr:		a pointer pointing to TAttribute containing the attribute 
 *					information
 *	OutPut:	
 *		pNumAttr:	a pointer to a integer which will contain the number of
 *					attributes the pAttr contains when this function returns
 *
 *---------------------------------------------------------------------------*/
static void SetAttrPool(const TAttribute* pAttr, int* pNumAttr, int extra)
{
	int i = 0;
	for (; i < extra; i++){
		(*pNumAttr)++;
		sizeOfAttrPool++;
		pAttrPool = (cvTAttr*)realloc(
								pAttrPool,
								sizeOfAttrPool* sizeof(cvTAttr));
		if (!pAttrPool){
			lrierr(eATTR_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		else
			memset(pAttrPool+sizeOfAttrPool-1, 0, sizeof(cvTAttr));
	}


	while ( pAttr ){
		(*pNumAttr)++;
		sizeOfAttrPool++;
		pAttrPool = (cvTAttr*)realloc(
									pAttrPool, 
									sizeOfAttrPool* sizeof(cvTAttr));
		if (!pAttrPool){
			lrierr(eATTR_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		else
			memset(pAttrPool+sizeOfAttrPool-1, 0, sizeof(cvTAttr));

		pAttrPool[sizeOfAttrPool-1].myId = pAttr->inum;
		pAttrPool[sizeOfAttrPool-1].value1 = pAttr->attrNum1;
		pAttrPool[sizeOfAttrPool-1].value2 = pAttr->attrNum2;
		pAttrPool[sizeOfAttrPool-1].from = pAttr->from;
		pAttrPool[sizeOfAttrPool-1].to = pAttr->to;
		pAttrPool[sizeOfAttrPool-1].laneMask = pAttr->laneMask;

		pAttr = pAttr->pNext;
	}
}

/*--------------------------------------------------------------------------*
 *
 *	Name: SetRepObjPool
 *
 *	NON-REENTRANT
 *
 *	This function reads information of repeated objects from pRepObj which
 *	is a pointer pointing to TRepObj containing the data from lri file, and
 *	store it in the repeated objects pool. New space for the rep objs is 
 *	allocated and initilized here, and the global variable sizaOfRepObjPool
 *	is also created correctly. The number of the repeated object pRepObj 
 *	contains is also computed pointed to by pNumRepObj
 *
 *	Input:
 *		pRepObj:	a poitner pointing to TRepObj which contains repeated 
 *					objects of the lri file
 *	Outpu:
 *		pNumRepObj:	a poitner pointing to an integer containing the number
 *					of attributes TRepObj contains
 *
 *--------------------------------------------------------------------------*/

static void SetRepObjPool(const TRepObj* pRepObj, int* pNumRepObj)
{
	*pNumRepObj = 0;

	while ( pRepObj ){
		pRepObjPool = (cvTRepObj *)realloc(
										pRepObjPool, 
										sizeof(cvTRepObj)*(sizeOfRepObjPool+1));
		if (!pRepObjPool){
			lrierr(eREP_OBJ_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		else
			memset(pRepObjPool+sizeOfRepObjPool, 0, sizeof(cvTRepObj));
		int i;
		for (i=0; i<sizeOfObjPool; i++){
			if (!strcmp(pRepObj->name, pObjPool[i].name)){
				pRepObjPool[sizeOfRepObjPool].objId = i;
				break;
			}
		}
		
		if (i == sizeOfObjPool){ //can't find the obj in obj pool
			char msg[200];

			sprintf(msg, " RepObj %s is not found in ObjPool", pRepObj->name);
			fprintf(stderr, "\n%s\n", msg);
#if 1
			lrierr(eBAD_NAME_REP_OBJ, "%s", msg);
			exit(1);
#endif 
		}
		// obj found in obj pool
		else { 
			(*pNumRepObj)++;
			sizeOfRepObjPool++;
			pRepObjPool[sizeOfRepObjPool-1].latdist	= pRepObj->latdist;
			pRepObjPool[sizeOfRepObjPool-1].start	= pRepObj->start;
			pRepObjPool[sizeOfRepObjPool-1].period	= pRepObj->period;
			pRepObjPool[sizeOfRepObjPool-1].end		= pRepObj->end;
			pRepObjPool[sizeOfRepObjPool-1].alligned= pRepObj->alligned;

		}

		pRepObj = pRepObj->pNext;
	}
}

/*---------------------------------------------------------------------------*
 *
 *	Name: SetLanePoolFromRoad
 *
 *	NON-REENTRANT
 *
 *	This function reads lane information from pLane which is a pointer pointing
 *	to "lane" field in the "lanelist" field in the cvTRoad, and write the 
 *	information to lane pool. This function is called in the SetCntrlPntPool
 *	when processing the first contrl pnt of the road, so the information 
 *	read here will be hanging with the first logitidunal control point. New
 *	space for the pool is allocated and initialized, and sizeOfLanePool is
 *	increased correctly. Finally, the sum of the width of every lane is summed
 *	up and is pointed to by pSumOfLaneWidth.
 *
 *	Input:
 *		pLane:	a pointer pointing to "lane" field in the "lanelist" field 
 *				in the cvTRoad.	
 *		numOfLane:	number of lanes that pLane contains
 *	Output:
 *		pSumOfLaneWidth:	a pointer pointing to an integer which contains
 *							the sum of the widths of all the lanes
 *
 *---------------------------------------------------------------------------*/
static void SetLanePoolFromRoad(
				const	TLaneDef*	pLane, 
				int				 	numOfLanes, 
				float*				pSumOfLaneWidth) 
{
	int		i;				/* counter */
	float	center = 0;		/* center of lanes */ 
	float	temp = 0;		
	int 	oldSize;		/* pool size before this funcion gets called */

	for (i=0; i<numOfLanes; i++)
		center += pLane[i].width;
	*pSumOfLaneWidth = center;
	center /= 2.0;
	oldSize = sizeOfLanePool;
	sizeOfLanePool+=numOfLanes;
	pLanePool = (cvTLane*)realloc(pLanePool, sizeOfLanePool*sizeof(cvTLane));
	if (!pLanePool){
		lrierr(eLANE_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	else
		memset(pLanePool+oldSize, 0, numOfLanes*sizeof(cvTLane));
	for (i=0; i<numOfLanes; i++){
		pLanePool[oldSize+i].roadId = sizeOfRoadPool-1;
		pLanePool[oldSize+i].myId = oldSize+i;
		pLanePool[oldSize+i].width = pLane[i].width;
		pLanePool[oldSize+i].laneNo = i;
		pLanePool[oldSize+i].direction = 
			( (pLane[i].direction=='p')||(pLane[i].direction=='P') )? ePOS: 
			( (pLane[i].direction=='n')||(pLane[i].direction=='N') )? eNEG: 
			eNONE;
		pLanePool[oldSize+i].offset = center - (temp+pLane[i].width/2);
		temp += pLane[i].width;
	}
}

/*---------------------------------------------------------------------------*
 *
 *	Name: SetLanePoolFromCntrlPnt
 *
 *	NON-REENTRANT
 *	
 *	This function is called from SetCntrlPntPool when field "cpointWidth" in
 *	the field "pCpointInfo" in the field "pCpointList" in the field of
 *	"longitCurve" of "TRoads" is valid. It allocates the same  number of lanes
 *	as the number of the lanes the road contains, and copy all the information
 *	from the lanes hanging with the first control points except the width     
 *	width field. It copys width from "cpointWidth", which is widths of lanes
 *	where the control point is
 *
 *	Input:
 *		numOfLanes:	the number of lanes the road contains
 *		first:		the index to the first lane hanging with the first control
 *					point.
 *		pWidth:		an array of widths of the lane where the control point is
 *
 *---------------------------------------------------------------------------*/
static void SetLanePoolFromCntrlPnt(
				int		numOfLanes, 
				int		first, 
				float*	pWidth,
				float*	pSumOfLaneWidth) 
{
	int		i;						/* counter */
	int		oldSize;				/* original lane pool size */
	float	center = 0;				/* center of lanes */
	float	temp = 0;				/* sum of the lanes */

	for (i=0; i<numOfLanes; i++)
		center += pWidth[i];
	*pSumOfLaneWidth = center;
	center /=2;

	oldSize = sizeOfLanePool;
	sizeOfLanePool+=numOfLanes;
	pLanePool = (cvTLane *)realloc(pLanePool, sizeOfLanePool*sizeof(cvTLane));
	if (!pLanePool){
		lrierr(eLANE_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	else
		memset(pLanePool+oldSize, 0, numOfLanes*sizeof(cvTLane));
	for (i=0; i<numOfLanes; i++){
		pLanePool[oldSize+i].roadId = sizeOfRoadPool-1;
		pLanePool[oldSize+i].myId = oldSize+i;
		pLanePool[oldSize+i].direction = pLanePool[first+i].direction;
		//pLanePool[oldSize+i].offset = pLanePool[first+i].offset;
		pLanePool[oldSize+i].width = pWidth[i];
		pLanePool[oldSize+i].laneNo = i;
		pLanePool[oldSize+i].offset = center - (temp+pWidth[i]/2);
		temp += pWidth[i];
	}
}

///////////////////////////////////////////////////////////////////////////
//
// Description: compute the distance of the current control point from the
// first control point of the road
//
// Remarks:
// This function compute the distance of the current control point from the
// first control point of the road and return the value of the distance
//
// Argument:
// cur - an index to the control point to be concerned regarding to the   
//       distance from the first point of the road
//
// Return value:
// An float indicating the distance from the current control point to the
// first control point of the road
//
static float GetCntrlPntDistance(int cur)
{
	CVector3D v1(pCntrlPntPool[cur].location);
	CVector3D v2(pCntrlPntPool[cur-1].location);
	float distance;
	distance = (v1-v2).Length();
	distance += pCntrlPntPool[cur-1].distance;
	return (distance);
}

	
/*----------------------------------------------------------------------------*
 *
 *	Name: SetCntrlPntPool
 * 
 *	This function reads the information of longitudinal control points of the
 *	roads and store it in the longitudinal control point pool. 
 *
 *	Input:
 *		pCntrlPnt:	pointer pointing to TControlPoint that contains informa-
 *					tion of longitudinal control points of the road
 *		numCntrlPnt:	number of the longitudinal control points of the road
 *		pLane:		pointer pointing to TLaneDef which contains the information
 *					of the road
 *		numOfLanes:	number of the lanes on the road
 *
 *---------------------------------------------------------------------------*/
static void SetCntrlPntPool(
			const TControlPoint*	pCntrlPnt,
			int						numCntrlPnt,
			const TLaneDef*			pLane,
			int						numOfLanes)
{
	int			cur;							/* counter */
	int			oldSize = sizeOfCntrlPntPool;	/* original pool size */
	TPoint3D	currPt,prevPt,nextPt;
	TVector3D	normal;
	TVector3D	linTang;
	TVector3D	linRight;
	TVector3D	cubicRight;
	float		linDist = 0,cumulativelinDist=0;
	float		cubicDist = 0,cumulativecubicDist=0;
	TVector3D	dPt0, dPtn;
	TPoint3D*	ptArray = NULL;
	TVector3D*	secondDeriv = NULL, firstDeriv;
	float*		tPars = NULL;
	float		tPar1, tPar2;

	// compute necessary size to add new elements
	sizeOfCntrlPntPool += numCntrlPnt;

	// if we don't have enough storage, allocate some and copy
	// old data to new space
	if ( storageOfCntrlPntPool <= sizeOfCntrlPntPool ) {
		cvTCntrlPnt*   pNew;
		int            oldStorage = storageOfCntrlPntPool;

		storageOfCntrlPntPool = 2* storageOfCntrlPntPool;
		if ( storageOfCntrlPntPool <= sizeOfCntrlPntPool ) {
			storageOfCntrlPntPool = 2 * sizeOfCntrlPntPool;
		}

		pNew = (cvTCntrlPnt *)calloc(storageOfCntrlPntPool,sizeof(cvTCntrlPnt));
		if ( pNew == NULL ) {
			lrierr(eCNTRL_PNT_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		memcpy(pNew, pCntrlPntPool, oldStorage*sizeof(cvTCntrlPnt));

		free(pCntrlPntPool);
		pCntrlPntPool = pNew;
	}

	for (cur=0; cur<numCntrlPnt; cur++){
		TCpointInfo	*pCpointInfo = pCntrlPnt[cur].pCpointInfo;
		TLaneWidth	*pLaneWidth;
		float		sumOfLaneWidth;

		/*
		 * Setting the location and normal in the memory pool:
		 *
		 * This is simply read in from the linked lists.  All vectors must
		 * be normalized before placed in memory.
		 */
		currPt.x = pCntrlPnt[cur].x;
		currPt.y = pCntrlPnt[cur].y;
		currPt.z = pCntrlPnt[cur].z;

		normal.i = pCntrlPnt[cur].i;
		normal.j = pCntrlPnt[cur].j;
		normal.k = pCntrlPnt[cur].k;

		if (!NormalizeVector( &normal )) {
			fprintf(stderr,"Line %d point %g %g %g\n",__LINE__,currPt.x,
					currPt.y,currPt.z);
		}

		pCntrlPntPool[oldSize+cur].location = currPt;
		pCntrlPntPool[oldSize+cur].normal = normal;

		/* 
		 * Copy lane information into control points and lane pool 
		 */
		if (cur==0){
			pCntrlPntPool[oldSize].laneIdx = sizeOfLanePool;
			pCntrlPntPool[oldSize].numLanes = numOfLanes;
			SetLanePoolFromRoad(pLane, numOfLanes, &sumOfLaneWidth);
			pCntrlPntPool[oldSize+cur].logicalWidth = sumOfLaneWidth;
			pCntrlPntPool[oldSize+cur].physicalWidth = sumOfLaneWidth;
			pCntrlPntPool[oldSize+cur].latCntrlPntIdx = 0;
			pCntrlPntPool[oldSize+cur].numLatCntrlPnt = 0;
		} else {
			pCntrlPntPool[oldSize+cur].laneIdx = 
								pCntrlPntPool[oldSize+cur-1].laneIdx;
			pCntrlPntPool[oldSize+cur].numLanes =
								pCntrlPntPool[oldSize+cur-1].numLanes;
			pCntrlPntPool[oldSize+cur].logicalWidth =
								pCntrlPntPool[oldSize+cur-1].logicalWidth;
			pCntrlPntPool[oldSize+cur].physicalWidth =
								pCntrlPntPool[oldSize+cur-1].physicalWidth;

			pCntrlPntPool[oldSize+cur].latCntrlPntIdx =
								pCntrlPntPool[oldSize+cur-1].latCntrlPntIdx; 
			pCntrlPntPool[oldSize+cur].numLatCntrlPnt =
								pCntrlPntPool[oldSize+cur-1].numLatCntrlPnt; 
		}

		for ( ; pCpointInfo; pCpointInfo = pCpointInfo->pNext){
			int previous = (cur==0)?(oldSize):(oldSize+cur-1);
			pLaneWidth = &(pCpointInfo->cpointWidth);
			bool setLanePoolFromCntrlPnt = false;
			if ((pLaneWidth != NULL) && (pLaneWidth->n_lanes > 0)){
				setLanePoolFromCntrlPnt = true;
				pCntrlPntPool[oldSize+cur].laneIdx = sizeOfLanePool;
				pCntrlPntPool[oldSize+cur].numLanes = numOfLanes;
				SetLanePoolFromCntrlPnt(
							numOfLanes, 
							pCntrlPntPool[previous].laneIdx,
							pLaneWidth->lanewidth,
							&sumOfLaneWidth);
							
				pCntrlPntPool[oldSize+cur].logicalWidth = sumOfLaneWidth;
				pCntrlPntPool[oldSize+cur].physicalWidth = sumOfLaneWidth;

			}
			if (pCpointInfo->pCpointAttribute != NULL){
				bool laneAlreadyCreated = false;
				if (setLanePoolFromCntrlPnt){
					laneAlreadyCreated = true;
				} else {
					//pCntrlPntPool[oldSize+cur].laneIdx = sizeOfLanePool;
					//pCntrlPntPool[oldSize+cur].numLanes = numOfLanes;
				}
				bool createLanePool = false;
				bool laneWidthChange = false;
				ProcessCntrlPntAttr(
							pCpointInfo->pCpointAttribute,
							numOfLanes,
							pCntrlPntPool[previous].laneIdx,
							&sumOfLaneWidth,
							laneAlreadyCreated,
							&laneWidthChange,
							&createLanePool);
				if (createLanePool){
					pCntrlPntPool[oldSize+cur].laneIdx = sizeOfLanePool - numOfLanes ;
					pCntrlPntPool[oldSize+cur].numLanes = numOfLanes;
				}
				if (laneWidthChange){					
					pCntrlPntPool[oldSize+cur].logicalWidth = sumOfLaneWidth;
					pCntrlPntPool[oldSize+cur].physicalWidth = sumOfLaneWidth;
				}

							

			}
			if ((pCpointInfo != NULL) && (pCpointInfo->curveName[0]!='\0')){
				int numOfPnt;
				TLatCntrlPntPoolIdx idx;
				float physicalWidth;
				GetIdxAndNumOfLatCntrl(	pCpointInfo->curveName, 
										&idx, 
										&numOfPnt, 
										&physicalWidth);
				pCntrlPntPool[oldSize+cur].latCntrlPntIdx = idx;
				pCntrlPntPool[oldSize+cur].numLatCntrlPnt = numOfPnt;
				pCntrlPntPool[oldSize+cur].physicalWidth = physicalWidth;
			}
		}
		pCntrlPntPool[oldSize+cur].distance = 
						(cur==0)?0: GetCntrlPntDistance(oldSize+cur);

		if (pCntrlPnt[cur].cpointName[0] != '\0'){
			pCntrlPntPool[oldSize+cur].nameIdx = sizeOfCharPool;
			SetCharPool(pCntrlPnt[cur].cpointName);
		}else
			pCntrlPntPool[oldSize+cur].nameIdx = 0;
	}

	/*
	 * Linear Tangent, Right and Linear Acc. Dist. and Linear Dist. to Next
	 * Control Point:
	 *
	 * Just simple Normalized(to_point - from_point) for linear tangent.
	 * and simple point to point distances.
	 */
	for (cur=0; cur<numCntrlPnt; cur++){
		currPt.x = pCntrlPnt[cur].x;
		currPt.y = pCntrlPnt[cur].y;
		currPt.z = pCntrlPnt[cur].z;

		if ( cur == 0 ) {
			/* for the first point */
			nextPt.x = pCntrlPnt[cur+1].x;
			nextPt.y = pCntrlPnt[cur+1].y;
			nextPt.z = pCntrlPnt[cur+1].z;
			cumulativelinDist = 0;
			linDist = ComputeLinDist( &currPt, &nextPt );
			if ( !ComputeLinTangent( &currPt, &nextPt, &linTang) ) {
				fprintf(stderr, "ComputeLinTang failed at %d\n", __LINE__);
			}
		}
		else if ( cur > 0 && cur < numCntrlPnt-1 ) { 
			/* there is a point before and after */
			prevPt.x = pCntrlPnt[cur-1].x;
			prevPt.y = pCntrlPnt[cur-1].y;
			prevPt.z = pCntrlPnt[cur-1].z;
			nextPt.x = pCntrlPnt[cur+1].x;
			nextPt.y = pCntrlPnt[cur+1].y;
			nextPt.z = pCntrlPnt[cur+1].z;
			cumulativelinDist = cumulativelinDist + linDist;
			linDist = ComputeLinDist( &currPt, &nextPt );
			if ( !ComputeLinTangent( &currPt, &nextPt, &linTang) ) {
				fprintf(stderr, "ComputeLinTang failed at %d\n", __LINE__);
			}
		}
		else if ( cur == numCntrlPnt-1 ) {
			/* for the last point */
			prevPt.x = pCntrlPnt[cur-1].x;
			prevPt.y = pCntrlPnt[cur-1].y;
			prevPt.z = pCntrlPnt[cur-1].z;
			cumulativelinDist = cumulativelinDist + linDist;
			linDist = 0;
			if ( !ComputeLinTangent( &prevPt, &currPt, &linTang) ) {
				fprintf(stderr, "ComputeLinTang failed at %d\n", __LINE__);
			}
		}

		normal.i = pCntrlPnt[cur].i;
		normal.j = pCntrlPnt[cur].j;
		normal.k = pCntrlPnt[cur].k;

		if (!NormalizeVector( &normal )) {
			fprintf(stderr,"Line %d point %g %g %g\n",__LINE__,currPt.x,
					currPt.y,currPt.z);
		}
		if (!NormalizeVector( &linTang )) {
			fprintf(stderr,"Line %d point %g %g %g\n",__LINE__,currPt.x,
					currPt.y,currPt.z);
		}

		CrossProductVector( &linTang, &normal, &linRight );

		pCntrlPntPool[oldSize+cur].tangVecLinear = linTang;
		pCntrlPntPool[oldSize+cur].rightVecLinear = linRight;

    	pCntrlPntPool[oldSize+cur].cummulativeLinDist = cumulativelinDist;
    	pCntrlPntPool[oldSize+cur].distToNextLinear = linDist;
	}

	/*
	 * Determine Sn for all normal vectors at Cps
	 */
	for (cur=0; cur<numCntrlPnt-1; cur++){
		float theta_2 = 0;
		TVector3D v1,v2,v3;
		v1 = pCntrlPntPool[oldSize+cur].normal;
		v2 = pCntrlPntPool[oldSize+cur+1].normal;	
		v3.i = v1.i + v2.i;
		v3.j = v1.j + v2.j;
		v3.k = v1.k + v2.k;
		if ( !NormalizeVector( &v3 ) ) {
			fprintf(stderr,"Sum of two vectors is 0 at %d\n",__LINE__);
		}
		/*
		 * now, a dot product of v1 and v3 would give cos(theta/2)
		 */
		theta_2 = DotProductVector( &v1, &v3 );
		pCntrlPntPool[oldSize+cur].sn = 4.0 - 4.0 * theta_2;
	}

	/*
	 * Allocate memory for arrays to be used later.
	 */

	ptArray = (TPoint3D *)calloc(numCntrlPnt, sizeof(TPoint3D));
	if ( ptArray == NULL ) {
		fprintf(stderr,"Cannot allocate memory to copy CPT\n");
		return;
	}
	secondDeriv = (TVector3D *)calloc(numCntrlPnt, sizeof(TVector3D));
	if ( secondDeriv == NULL ) {
		fprintf(stderr,"Cannot allocate memory to copy second deriv\n");
		free( ptArray );
		return;
	}
	tPars = (float *)calloc(numCntrlPnt, sizeof(float));
	if ( tPars == NULL ) {
		fprintf(stderr,"Cannot allocate memory to copy distances\n");
		free( ptArray );
		free( secondDeriv );	
		return;
	}

	for ( cur = 0; cur < numCntrlPnt; cur++ ) {
		ptArray[cur] = pCntrlPntPool[oldSize+cur].location;
		tPars[cur] = pCntrlPntPool[oldSize+cur].cummulativeLinDist;
	}

	/*
	 * Second Derivates at all control points:
	 *
	 * For C2 continuity of splines.  The code contained in the function
	 * call comes from "Numerical Recipes in C" pp. 95-110.
	 */

	tPar1 = tPars[0];
	tPar2 = tPars[1];

	dPt0.i = (ptArray[1].x - ptArray[0].x)/(tPar2-tPar1);
	dPt0.j = (ptArray[1].y - ptArray[0].y)/(tPar2-tPar1);
	dPt0.k = (ptArray[1].z - ptArray[0].z)/(tPar2-tPar1);

	tPar1 = tPars[numCntrlPnt-2];
	tPar2 = tPars[numCntrlPnt-1];

	dPtn.i = (ptArray[numCntrlPnt-1].x - ptArray[numCntrlPnt-2].x)/
					(tPar2-tPar1);
	dPtn.j = (ptArray[numCntrlPnt-1].y - ptArray[numCntrlPnt-2].y)/
					(tPar2-tPar1);
	dPtn.k = (ptArray[numCntrlPnt-1].z - ptArray[numCntrlPnt-2].z)/
					(tPar2-tPar1);

	Compute2ndDerivatives( tPars, ptArray, numCntrlPnt, dPt0, dPtn,
									secondDeriv );
   
	for ( cur = 0; cur < numCntrlPnt; cur++ ) {
		TPoint3D Pt0, Pt1;
		TVector3D dp20,dp21;
		cvTSplCoef coef[3];

		if ( cur == numCntrlPnt-1 ) {
			Pt0  = ptArray[cur-1];
			Pt1  = ptArray[cur];
			dp20 = secondDeriv[cur-1];
			dp21 = secondDeriv[cur];
		}
		else {
			Pt0  = ptArray[cur];
			Pt1  = ptArray[cur+1];
			dp20 = secondDeriv[cur];
			dp21 = secondDeriv[cur+1];
		}	
        
		/*
		 * C2 Hermite Spline Coefficient Computations.
		 */

		ComputeHermiteSplineC2Coef3D( &Pt0, &Pt1, &dp20, &dp21, 
				coef ); 

		pCntrlPntPool[oldSize+cur].hermite[0] = coef[0];	
		pCntrlPntPool[oldSize+cur].hermite[1] = coef[1];	
		pCntrlPntPool[oldSize+cur].hermite[2] = coef[2];	

		/*
		 * Compute curve length.
		 */

		if ( cur == 0 ) {
			cumulativecubicDist = 0;
			cubicDist = ComputeCubicDist( coef ); 
		} else if ( cur > 0 && cur < numCntrlPnt - 1 ) {
			cumulativecubicDist = cumulativecubicDist + cubicDist;
			cubicDist = ComputeCubicDist( coef ); 
		} else if ( cur == numCntrlPnt-1 ) {
			cumulativecubicDist = cumulativecubicDist + cubicDist;
			cubicDist = 0;
		}

		pCntrlPntPool[oldSize+cur].distToNextCubic = cubicDist; 
		pCntrlPntPool[oldSize+cur].cummulativeCubicDist = cumulativecubicDist; 

		/*
		 * First Derivative at each control point:
		 * 
		 * if H(t) = At^3 + Bt^2 + Ct + D, then
		 * H'(t) = 3At^2 + 2Bt + C, and and each control point,
		 * H'(0) = C
		 * where H(t) is the Hermite spline polynomial.
		 */

		firstDeriv.i = coef[0].C;
		firstDeriv.j = coef[1].C;
		firstDeriv.k = coef[2].C;
		if (!NormalizeVector( &firstDeriv )) {
			fprintf(stderr,"Line %d point %g %g %g\n",__LINE__,ptArray[cur].x,
					ptArray[cur].y,ptArray[cur].z);
		}
		pCntrlPntPool[oldSize+cur].tangVecCubic = firstDeriv;
		CrossProductVector( &firstDeriv, 
					&pCntrlPntPool[oldSize+cur].normal, &cubicRight );
      pCntrlPntPool[oldSize+cur].rightVecCubic = cubicRight;

#if 0
		/* 
		 * Radius of Curvature:
		 *
		 * we drop the vertical component of the vector for this
		 * computation.  The derivates are analytical and based upon the
		 * spline coefficients calculated earlier.
		 */

		pCntrlPntPool[oldSize+cur].radius = ComputeRadiusOfCurvature( coef );
#endif
	}

	/*
	 * Now calculate the radius of curvature.  We build a
	 * spline and then use the spline's radius of curvature calculation
	 * on the control points.
	 */
	CSplineHermite spl;
    CSplineHermiteNonNorm sp2;
	for (cur=0; cur<numCntrlPnt; cur++){
		cvTCntrlPnt* pC = &pCntrlPntPool[oldSize+cur];
		spl.addPoint(pC->location.x, pC->location.y, 0.0);
        sp2.addPoint(pC->location.x, pC->location.y,pC->location.z);
	}

	spl.calc();
    sp2.calc();

	CCubicSplinePos  pos;
	for (cur=0; cur<numCntrlPnt; cur++){
		cvTCntrlPnt *pC = &pCntrlPntPool[oldSize+cur];

		if (1 ) {
			double r1;
			pos.setPos(cur, 0);
			if (cur<numCntrlPnt-1)
				sp2.AdvanceMidway(pos,0.5);
			if (sp2.evalCurveRadiusAvgOverRegion(pos,15,r1)){
				pC->radius = r1;
			}else{
				pC->radius = (pC-1)->radius;
			}
		}
        if (numCntrlPnt>1){
            sp2.getCoeff(cur,'x', pC->hermite[0].A, pC->hermite[0].B, pC->hermite[0].C,pC->hermite[0].D);
            sp2.getCoeff(cur,'y', pC->hermite[1].A, pC->hermite[1].B, pC->hermite[1].C,pC->hermite[1].D);
            sp2.getCoeff(cur,'z', pC->hermite[2].A, pC->hermite[2].B, pC->hermite[2].C,pC->hermite[2].D);
        }
	}

	/* 
	 * Deallocate memory.
	 */
	free( secondDeriv );
	free( ptArray );
	free( tPars );
}

/*--------------------------------------------------------------------*
 *
 *	Name: SetAttrName
 *	
 *	NON-REENTRANT
 *  
 *	This function gets the attribute name from the global pointer to
 *	gAttrDict, which contains the names and ids of the attribute, and
 *	put them in the char pool and correct index to the pool.
 *
 *----------------------------------------------------------------*/   
static void SetAttrName(void)
{
	int i, j;

	for (i=1; i < sizeOfAttrPool; i++) {

		if ( (pAttrPool[i].myId > 0) &&
			 (pAttrPool[i].myId < cCV_NUM_RESERVED_ATTR) ) {

			pAttrPool[i].nameIdx = sizeOfCharPool;
			SetCharPool(reservedAttrNames[pAttrPool[i].myId-1]);

			return;
		}
		
		else {
			for (j=0; j<gAttrDict.nEntries; j++) {
				if (pAttrPool[i].myId == gAttrDict.pList[j].id){
					pAttrPool[i].nameIdx = sizeOfCharPool;
					SetCharPool(gAttrDict.pList[j].pName);

					return;
				}
			}
		}
	}
}

static void ProcessCntrlPntAttr(
					TAttribute* pAttr, 
					int         numOfLanes,
					int         prevLaneIdx,
					float*      pLaneWidth,
					bool		laneAlreadyCreated,
					bool*		pLaneChanged,
					bool*		pCreateLanePool)
{
	int  numOfAttrWidth = 0;
    while ( pAttr ){
		int i = 0;
		for(; i<gAttrDict.nEntries; i++){
			if (pAttr->inum == gAttrDict.pList[i].id){
				if (!strcmp(gAttrDict.pList[i].pName, "Width")){
					*pLaneChanged = true;
					if (numOfAttrWidth == 0) {
						if (!laneAlreadyCreated){
							*pCreateLanePool = true;
						} else {
							*pCreateLanePool = false;	
						}
					}
					SetLanePoolFromCntrlPntAttr(
										pAttr,
										numOfLanes,
										prevLaneIdx,
										pLaneWidth,
										*pCreateLanePool);
					numOfAttrWidth++;		
				}
				// if (strcmp(gAttrDict.pList[i].pName, "Other Attr"))
			}
		}
        pAttr = pAttr->pNext;
    }
}

static void SetLanePoolFromCntrlPntAttr(
				TAttribute* pAttr,
                int     numOfLanes,
                int     first,
                float*  pSumOfLaneWidth,
				bool	createLanePool)
{
    int     i;                      /* counter */
    int     oldSize;                /* original lane pool size */

	if (createLanePool){	
   		oldSize = sizeOfLanePool;
    	sizeOfLanePool+=numOfLanes;
    	pLanePool = (cvTLane *)realloc(pLanePool, 
					sizeOfLanePool*sizeof(cvTLane));
    	if (!pLanePool){
       		lrierr(eLANE_POOL_ALLOC_FAIL, "");
       		exit(1);
    	} else
       		memset(pLanePool+oldSize, 0, numOfLanes*sizeof(cvTLane));
 		for (i=0; i<numOfLanes; i++){
   	   		pLanePool[oldSize+i].roadId = sizeOfRoadPool-1;
   			pLanePool[oldSize+i].myId = oldSize+i;
			pLanePool[oldSize+i].direction = pLanePool[first+i].direction;
   	   		pLanePool[oldSize+i].width = pLanePool[first+i].width;
   	   		pLanePool[oldSize+i].laneNo = i;
   		}
	} else
		oldSize = sizeOfLanePool - numOfLanes;

	int laneNo = 1;
    float center = 0.0;
	for(i = 0; i<numOfLanes; i++){
		if ((pAttr->laneMask && laneNo)|| pAttr->laneMask == -1){
			pLanePool[oldSize+i].width = pAttr->attrNum1;
		}
		laneNo *=2;
        center += pLanePool[oldSize+i].width;
	}
    *pSumOfLaneWidth = center;
    center /= 2 ;

    float temp = 0;               /* sum of the lanes */
    for (i=0; i<numOfLanes; i++){
		pLanePool[oldSize+i].offset = center - 
							(temp+pLanePool[oldSize+i].width/2);
		temp += pLanePool[oldSize+i].width;
	}
}


#if 0
//
// This function was returning incorrect radius of curavatures.  The
// results were ^2 of what they should have been--i.e if a curve should
// have a radius of curvature of 20...this code was returning something
// more like 400.
//			
/********************************************************************
 *
 * Straight from a Calculus book:
 * "Calculus with Analytic Geometry" by Joe Repka pp. 897-898.
 * Here, we drop the vertical component of the vectors to do the
 * computations only in the xy plane.  We do that by simply setting
 * the k component of the vector to be 0.  **ALERT***ALERT**
 *
 * The radius of curvature at a point is defined by:
 *
 *               || H'(0) ||^3
 *           ----------------------
 *            || H'(0) X H''(0) ||
 *
 * where H(t) is the Hermite spline polynomial.
 * H'(0) will be the tangent at the control point and H''(0) will be
 * the second derivative (rate of change of the tangent) at the control
 * point.
 *
 *******************************************************************/
static float ComputeRadiusOfCurvature( cvTSplCoef* pCoef )
{
	TVector3D d1,d2;   /* The first and second derivatives. */
	TVector3D CrossP;
	double		magCrossP, magD1;
	float		RadiusCurvature;
	
	/* derivative at t = 0 */
	d1.i = pCoef[0].C;
	d1.j = pCoef[1].C;
	d1.k = pCoef[2].C;

	/* 2nd derivate at t = 0 */
	d2.i = 2 * pCoef[0].B;
	d2.j = 2 * pCoef[1].B;
	d2.k = 2 * pCoef[2].B;

	CrossProductVector( &d1, &d2, &CrossP );

#if 0
	magCrossP = sqrt( CrossP.i * CrossP.i + CrossP.j * CrossP.j + CrossP.k * CrossP.k ); 
#endif
	//magCrossP = fabs( CrossP.i * CrossP.i + CrossP.j * CrossP.j + CrossP.k * CrossP.k ); 

	if( magCrossP < cCV_ZERO ) 
	{
		/* Straight line...curvature is infinity */
		return cCV_INFINITY;
	}

	magD1 = d1.i * d1.i + d1.j * d1.j + d1.k * d1.k;
	magD1 = sqrt( magD1 * magD1 * magD1 );

	RadiusCurvature = magD1 / magCrossP;
	
	if ( RadiusCurvature > cCV_INFINITY )
		RadiusCurvature = cCV_INFINITY;

	return RadiusCurvature;
}
#endif

static void Compute2ndDerivatives( 
								float*		t, 
								TPoint3D*	pi,
								int			n, 
								TVector3D	dy_p0, 
								TVector3D	dy_pn,
								TVector3D*	dy2_pi )
{
	TVector3D*	dy = NULL;
	float  dy2_pn,         /* secon dderivative at last point */
          un, p;
	float  sig;
	int    i;

	dy = (TVector3D *)calloc(n, sizeof(TVector3D));
	if ( dy == NULL ) {
		fprintf(stderr,"Cannot allocate memory to copy first deriv\n");
		return;
	}
 
	/*
 	 * Compute X
 	 */
	if(dy_p0.i > 0.99e30){
		dy2_pi[0].i = 0.0;
		dy[0].i       = 0.0;
	}
	else {
		dy2_pi[0].i = -0.5;
		dy[0].i       = (3./(t[1]-t[0]))*
								((pi[1].x-pi[0].x)/(t[1]-t[0]) - dy_p0.i);
	}
	for(i=1;i<n-1;i++) {
		sig   = (t[i]-t[i-1])/(t[i+1]-t[i-1]);
		p     = sig*dy2_pi[i-1].i+2.;
		dy2_pi[i].i = (sig-1.0)/p;
		dy[i].i    = (6.*((pi[i+1].x-pi[i].x)/(t[i+1]-t[i]) -
                    (pi[i].x-pi[i-1].x)/(t[i]-t[i-1]))/
                    (t[i+1]-t[i-1]) - sig*dy[i-1].i)/p;
	}

	if(dy_pn.i > 0.99e30) {
		dy2_pn = 0.;
		un  = 0.;
	}
	else{
		dy2_pn = 0.5;
		un  = (3./(t[n-1]-t[n-2]))*
					(dy_pn.i - (pi[n-1].x-pi[n-2].x)/(t[n-1]-t[n-2]));
	}

	dy2_pi[n-1].i = (un-dy2_pn*dy[n-2].i)/(dy2_pn*dy2_pi[n-2].i+1.);
	for(i=n-2;i>=0;i--) {
		dy2_pi[i].i=dy2_pi[i].i*dy2_pi[i+1].i+dy[i].i;
	}

	/*
	 * Compute Y
	 */
	if(dy_p0.j > 0.99e30){
		dy2_pi[0].j = 0.0;
		dy[0].j       = 0.0;
	}
	else {
		dy2_pi[0].j = -0.5;
		dy[0].j       = (3./(t[1]-t[0]))*
							((pi[1].y-pi[0].y)/(t[1]-t[0]) - dy_p0.j);
	}
	for(i=1;i<n-1;i++) {
		sig   = (t[i]-t[i-1])/(t[i+1]-t[i-1]);
		p     = sig*dy2_pi[i-1].j+2.;
		dy2_pi[i].j = (sig-1.0)/p;
		dy[i].j    = (6.*((pi[i+1].y-pi[i].y) / (t[i+1]-t[i]) -
                    (pi[i].y-pi[i-1].y) / (t[i]-t[i-1]))/
                    (t[i+1]-t[i-1]) - sig*dy[i-1].j) / p;
	}

	if(dy_pn.j > 0.99e30) {
		dy2_pn = 0.;
		un  = 0.;
	}
	else{
		dy2_pn = 0.5;
		un  = (3./(t[n-1]-t[n-2]))*
					(dy_pn.j - (pi[n-1].y-pi[n-2].y)/(t[n-1]-t[n-2]));
	}

	dy2_pi[n-1].j = (un-dy2_pn*dy[n-2].j)/(dy2_pn*dy2_pi[n-2].j+1.);
	for(i=n-2;i>=0;i--) {
		dy2_pi[i].j=dy2_pi[i].j*dy2_pi[i+1].j+dy[i].j;
	}

	/*
	 * Compute Z
	 */
	if(dy_p0.k > 0.99e30){
		dy2_pi[0].k = 0.0;
		dy[0].k       = 0.0;
	}
	else {
		dy2_pi[0].k = -0.5;
		dy[0].k      = (3./(t[1]-t[0]))*
							((pi[1].z-pi[0].z)/(t[1]-t[0]) - dy_p0.k);
	}
	for(i=1;i<n-1;i++) {
		sig   = (t[i]-t[i-1])/(t[i+1]-t[i-1]);
		p     = sig*dy2_pi[i-1].k+2.;
		dy2_pi[i].k = (sig-1.0)/p;
		dy[i].k = (6.*((pi[i+1].z-pi[i].z) / (t[i+1]-t[i]) -
                    (pi[i].z-pi[i-1].z) / (t[i]-t[i-1]))/
                    (t[i+1]-t[i-1]) - sig*dy[i-1].k) / p;
	}

	if(dy_pn.k > 0.99e30) {
		dy2_pn = 0.;
		un  = 0.;
	}
	else{
		dy2_pn = 0.5;
		un  = (3./(t[n-1]-t[n-2]))*
					(dy_pn.k - (pi[n-1].z-pi[n-2].z)/(t[n-1]-t[n-2]));
	}

	dy2_pi[n-1].k = (un-dy2_pn*dy[n-2].k)/(dy2_pn*dy2_pi[n-2].k+1.);

	for(i=n-2;i>=0;i--) {
		dy2_pi[i].k=dy2_pi[i].k*dy2_pi[i+1].k+dy[i].k;
	}

	free( dy );
}


#ifdef DEBUG_DUMP
static void DumpRoad( void )	
{
	int cur, temp, temp2, cur1, cur2;
	int i, j;
	printf("\n num of roads %d \n", sizeOfRoadPool);
	for (cur=1; cur<sizeOfRoadPool; cur++){
		temp = pRoadPool[cur].nameIdx;
		printf("\n---------------------------------------------------------");
		printf("\nRoad namdIdx	%d ", temp);
		printf("	length is %d ", strlen(pCharPool+temp) );
		printf("	roadName \"%s\" ", &pCharPool[temp]);
		printf("\n number of cntrl pnts	%d 	", pRoadPool[cur].numCntrlPnt);
		printf("Idx of cntrl pnts %d \n", pRoadPool[cur].cntrlPntIdx);
		for (cur1=0; cur1<pRoadPool[cur].numCntrlPnt; cur1++){
			printf("\nnameIdx of Cntrl	%d", 
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].nameIdx);
			temp = pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].nameIdx;
			if(pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].nameIdx!=0)
				printf("	\"%s\"", &pCharPool[temp]);
			
			temp=pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].latCntrlPntIdx; 
			temp2=pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].numLatCntrlPnt;
			printf("\n	lateral idx : %d		numOfLateral: %d",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].latCntrlPntIdx,
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].numLatCntrlPnt);
			for (j=0; j<temp2; j++){
				if ((temp!=0)&&(temp2!=0)) {
					printf("\n	%dth pts", j);
					printf("	offset %.1f		height %.1f		material %d ",
						pLatCntrlPntPool[temp+j].offset,
						pLatCntrlPntPool[temp+j].height,
						pLatCntrlPntPool[temp+j].material);
				}
			}
			printf("\n	location		%.2f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].location.x);
			printf("	%.2f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].location.y);
			printf("	%.2f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].location.z);
			printf("	%.2f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].normal.i);
			printf("	%.2f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].normal.j);
			printf("	%.2f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].normal.k);
/*
			printf("\n	tangent			%f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].tangVecCubic.i);
			printf("	%f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].tangVecCubic.j);
			printf("	%f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].tangVecCubic.k);
			printf("\n	right			%f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].rightVecCubic.i);
			printf("	%f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].rightVecCubic.j);
			printf("	%f",
				pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].rightVecCubic.k);

			printf("\n A = %g B = %g C = %g D = %g",
			pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[0].A,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[0].B,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[0].C,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[0].D);

			printf("\n A = %g B = %g C = %g D = %g",
			pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[1].A,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[1].B,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[1].C,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[1].D);

			printf("\n A = %g B = %g C = %g D = %g",
			pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[2].A,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[2].B,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[2].C,
            pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].hermite[2].D);
*/

			temp = pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].laneIdx;
			if (pCntrlPntPool[ pRoadPool[cur].cntrlPntIdx+cur1 ].laneIdx!=0){
				printf("\n how many lanes %d  ",
					pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].numLanes);
				for (	cur2=0; 
						cur2<
						pCntrlPntPool[pRoadPool[cur].cntrlPntIdx+cur1].numLanes;
							cur2++	){
					printf("\n   lanes (w/o/d) %.1f %.1f %c", 
						pLanePool[temp+cur2].width,
						pLanePool[temp+cur2].offset,
						pLanePool[temp+cur2].direction==ePOS? 'p':'n');
				}
			}
		}


		printf("\n    number of attr	%d", pRoadPool[cur].numAttr);
		for (cur1=0; cur1<pRoadPool[cur].numAttr; cur1++){
			printf("\n      nameIdx of Atr		%d",
				pAttrPool[pRoadPool[cur].attrIdx+cur1].nameIdx);
		}
		printf("\n");
	}
	printf("\n try to print char pool-- ");
	for (i=1; i<sizeOfCharPool; i++){
		printf("%c",pCharPool[i]);
	}
	printf("\n");
	
}
#endif

static bool NormalizeVector( TVector3D* v )
{
	double length; 
	double oneOverLength;

	if ( v == NULL ) {
		fprintf(stderr,"Given a null vector\n");
		return 0;
	}

	length = sqrt( v->i * v->i +
						v->j * v->j +
						v->k * v->k );

	if ( length < cCV_ZERO ) {
		fprintf(stderr,"Length of vector %g %g %g is 0\n",v->i,v->j,v->k);
		return false;
	}
	oneOverLength = 1/length;

	v->i *= oneOverLength;
	v->j *= oneOverLength;
	v->k *= oneOverLength;
	return true;
}

static void CrossProductVector( TVector3D*	pv1, 
								TVector3D*	pv2, 
								TVector3D*	result )
{
	result->i = (float)(pv1->j*pv2->k - pv1->k*pv2->j);
	result->j = (float)(pv1->k*pv2->i - pv1->i*pv2->k);
	result->k = (float)(pv1->i*pv2->j - pv1->j*pv2->i);
}

static float DotProductVector( TVector3D*	pv1, TVector3D*	pv2 )
{
	return (float)(pv1->i * pv2->i + pv1->j * pv2->j + pv1->k * pv2->k);
}


/************************************************************************
 * Compute the tangent at point p1, given p1 and the next point p2.
 * Linear interpolation is used.
 ***/
static bool 
ComputeLinTangent(TPoint3D* p1, TPoint3D* p2, TVector3D* tangent)
{
   tangent->i = p2->x - p1->x;
   tangent->j = p2->y - p1->y;
   tangent->k = p2->z - p1->z;

   return NormalizeVector(tangent);
}

/************************************************************************
 * Computes the distance between two points
 * This function might be moved out to a more appropriate place
 * later. Uses linear approximation
 ***/
static float ComputeLinDist(const TPoint3D* p1, const TPoint3D* p2)
{
   float dist;

   dist = (float)sqrt((p2->x-p1->x)*(p2->x-p1->x) +
              (p2->y-p1->y)*(p2->y-p1->y) +
                  (p2->z-p1->z)*(p2->z-p1->z));

   return dist;
}

static float EvalSqrt( cvTSplCoef H[3], float t )
{
	float val = 0.0;
	float Hx = 0.0;	
	float Hy = 0.0;	
	float Hz = 0.0;	
	float t_2 = t*t; 

	Hx = 3*H[0].A*t_2 + 2*H[0].B*t + H[0].C;
	Hx = Hx * Hx; 

	Hy = 3*H[1].A*t_2 + 2*H[1].B*t + H[1].C;
	Hy = Hy * Hy; 

	Hz = 3*H[2].A*t_2 + 2*H[2].B*t + H[2].C;
	Hz = Hz * Hz; 

	val = (float)sqrt( Hx + Hy + Hz );

	return val;
}

/*-----------------------------------------------------------------------*
 *
 * Name: ComputeCubicDist
 *
 * This function computes the arc length of a cubic curve defined
 * between control points.  This function uses three terms to use
 * Simpson's rule to approximate the classic integral that defines the
 * length of a curve in space.
 *
 *-----------------------------------------------------------------------*/
static float ComputeCubicDist( cvTSplCoef H[3] )
{
	float dist = 0;

	dist = 1/6.0 * ( EvalSqrt( H, 0.0 ) + 
					4 * EvalSqrt( H, 1/2.0 ) +
					EvalSqrt( H, 1.0 ));

	return dist;
}

/*--------------------------------------------------------------------------*
 *
 *	Name: BuildLatPool
 *
 *	This function creats a memory pool for TLat, which is basically an      
 *	interium containing the indeces to lateral control point pool. It will
 *	not be used in other files. In 	function SetCntrlPntPool, it calls 
 *	GetIdxAndNumOfCntrl to use this	memory pool(pLatPool) to match the 
 *  pCpointInfo->curveName which is the name of lateral control point to
 *	"curveName" in pLatPool. By doing this, it can find the idx in the 
 *	pLatPool to lateral control point pool and assigned it to "latCntrlPntIdx" 
 *	in the memory pool for cvTCntrlPnt
 *
 *--------------------------------------------------------------------------*/
static void BuildLatPool(void)
{
	TLatCurve	*pCurrentLatCurve = gpLatCurves;
	int 		numOfPnt;
	int			oldSize;
	while(pCurrentLatCurve){
		sizeOfLatPool++;
		oldSize = sizeOfLatCntrlPntPool;
		pLatPool = (TLat *)realloc(pLatPool, sizeof(TLat)*sizeOfLatPool);
		if (!pLatPool){
			lrierr(eLAT_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		BuildLatCntrlPntPool(	pCurrentLatCurve->pCurveDef, 
								pCurrentLatCurve->curveWidth*0.5,
								&numOfPnt);
		strcpy(	pLatPool[sizeOfLatPool-1].curveName, 
				pCurrentLatCurve->curveName);
		pLatPool[sizeOfLatPool-1].idx = oldSize;  
		pLatPool[sizeOfLatPool-1].numOfLatPnt = numOfPnt;
		pLatPool[sizeOfLatPool-1].physicalWidth = 
			pCurrentLatCurve->curveWidth;

		pCurrentLatCurve = pCurrentLatCurve->pNext;
	}
}

/*--------------------------------------------------------------------------*
 *
 *	Name: BuildLatCntrlPntPool
 *	
 *	NON-REENTRANT
 *
 *	This function reads the field "pCurveDef" of the global pointer gpLatCurves
 *	pointing to lateral control point of the lri file, and store it in the
 *	lateral control point pool which is also allocated in this function.
 *
 *	Input:
 *		pCurveDef:	pointer pointing to the field "pCurveDef" of the global
 *					pointer pointing to gpLatCurves which contains the   
 *					information of the lateral control point
 *	Outpu:	
 *		pNumOfPnt:	pointer pointing to an integer containing the number of
 *					the lateral control points pCurveDef contains
 *		pSumOfOffset:	pointer pointing to an integer containing the sum
 *					of the offsets of lateral control points 
 *						
 *--------------------------------------------------------------------------*/
static void BuildLatCntrlPntPool(
				const TCurveDef*	pCurveDef, 
				float				halfCurveWidth,
				int*				pNumOfPnt)
{
	int oldSize = sizeOfLatCntrlPntPool;
	*pNumOfPnt = 0;
	while (pCurveDef){
		(*pNumOfPnt) += 2;
		sizeOfLatCntrlPntPool += 2;
		pLatCntrlPntPool = (cvTLatCntrlPnt *)realloc(pLatCntrlPntPool, 
								sizeof(cvTLatCntrlPnt)*sizeOfLatCntrlPntPool);
		if (!pLatCntrlPntPool){
			lrierr(eLAT_CNTRL_PNT_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		memset(	pLatCntrlPntPool+oldSize, 
				0, 
				2*sizeof(cvTLatCntrlPnt));

		if (fabs(pCurveDef->latCpoint1.offset) > halfCurveWidth)
			printf("Warning:: LatCntrlPnt[%d].offset = %f > 0.5*curve width = %f\n",
				oldSize, pCurveDef->latCpoint1.offset, halfCurveWidth);

		pLatCntrlPntPool[oldSize].offset = pCurveDef->latCpoint1.offset;
		pLatCntrlPntPool[oldSize].height = pCurveDef->latCpoint1.height;
		pLatCntrlPntPool[oldSize].material = pCurveDef->latCpoint1.material;
		pLatCntrlPntPool[oldSize+1].offset = pCurveDef->latCpoint2.offset;
		pLatCntrlPntPool[oldSize+1].height = pCurveDef->latCpoint2.height;
		pLatCntrlPntPool[oldSize+1].material = pCurveDef->latCpoint2.material;

		oldSize += 2;
		pCurveDef = pCurveDef->pNext;
	}
}
	
/*--------------------------------------------------------------------------*
 *
 *	Name: GetIdxAndNumOfLatCntrl
 *
 *	NON-REENTRANT
 *	
 *	This function which is called in SetCntrlPntPool takes as parameter
 *	"pCpointInfo->curveName", which is the name of lateral control point, and
 *	try to match it with the "curveName" field of TLat in the pLatPool. Once
 *	it finds it, it copy the fields "idx", "numOfLatPnt", and "physicalWidth"
 *	and return it for use in SetCntrlPntPool to assign them to fields 
 *	"latCntrlPntIdx", "numLatCntrlPnt" and "physicalWidth"
 *
 *	Input:
 *		curveName:	poitner to the first char of a string of chars which are
 *					name of lateral control point(pCpointInfo->curveName)
 *	Output:
 *		pIdx:		pointer pointing a TlatCntrlPntPoolIdx containing a index
 *					to lateral control point pool
 *		pNumOfPnt:	pointer pointing an integer containing the number of
 *					lateral control points whose name is "curveName"
 *		pPhysicalWidth: pointer pointing a float containing the physical with
 *					of the lateral control points	
 *
 *-------------------------------------------------------------------------*/
static int GetIdxAndNumOfLatCntrl(
				const char*				curveName, 
				TLatCntrlPntPoolIdx*	pIdx, 
				int*					pNumOfPnt, 
				float*					pPhysicalWidth)
{ 
	int i; 
	for (i=1; i<sizeOfLatPool; i++)
		if (!strcmp(curveName, pLatPool[i].curveName)){
			*pIdx = pLatPool[i].idx;
			*pNumOfPnt = pLatPool[i].numOfLatPnt;
			*pPhysicalWidth = pLatPool[i].physicalWidth;
			return(1);
		}
	*pIdx = 0;
	*pNumOfPnt = 0;
	printf("\n the name from cntrl info is \"%s\" which "
			"could not be found in lateral pnt\n", curveName);
	exit(1);
	return 0;
}

static void DumpIntrsctn(void)
{
	int i;
	printf("\n =========== intersection =================\n");
	printf(" 	There are %d intersections in the pool ", sizeOfIntrsctnPool);
	for (i=1; i<sizeOfIntrsctnPool; i++){
		int j;
		printf("\n	%dth intersection	id: %d	name: %s\n		", 
							i,
							pIntrsctnPool[i].myId,
							&pCharPool[pIntrsctnPool[i].nameIdx] );
		if (pIntrsctnPool[i].numOfRoads!=0)
			for (j=0; j<pIntrsctnPool[i].numOfRoads; j++){
				int temp;
				printf(" %dth RdId: %d", j, pIntrsctnPool[i].adjacentRoadId[j]);
				temp = pIntrsctnPool[i].adjacentRoadId[j];
				temp = pRoadPool[temp].nameIdx;
				printf(" %s ", &pCharPool[temp]);
			}
	}
	printf("\n road id \n");
	for (i=1; i<sizeOfRoadPool; i++)
		printf("	%dth	id: %d\n", i, pRoadPool[i].myId);
	printf("\n");
}

/*---------------------------------------------------------------------------*
 *
 *	Name: SetBorderSegPool
 *
 *	NON-REENTRANT
 *	
 *	This function takes as parameter border structure from intersection and
 *	store the information it reads  in the border segment pool. New space
 *	is allocated inside the function. "haveline" and "line style" are done
 *	yet.
 *
 *	Input:
 *		pBorder:	pointer pointing to &(TIntersection->Border) which 
 *					contains information of borders
 *
 *---------------------------------------------------------------------------*/
static void SetBorderSegPool(TBorder*	pBorder)
{
	int i;
	TBorderPt *pBorderPt = pBorder->list;
	int oldSize = sizeOfBorderSegPool;
	sizeOfBorderSegPool += pBorder->nPts;
	pBorderSegPool = (cvTBorderSeg *)realloc(	pBorderSegPool, 
								sizeof(cvTBorderSeg)*sizeOfBorderSegPool);
	if (!pBorderSegPool){
		lrierr(eBORDER_SEG_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	else
		memset(pBorderSegPool+oldSize, 0, pBorder->nPts*sizeof(cvTBorderSeg));
	for (i=0; i<pBorder->nPts; i++){
		pBorderSegPool[oldSize+i].x = pBorderPt[i].borderX;
		pBorderSegPool[oldSize+i].y = pBorderPt[i].borderY;
		pBorderSegPool[oldSize+i].lineFlag = pBorderPt[i].lflag;
	}
}

/*---------------------------------------------------------------------------*
 *
 *	Name: GetCrdrCntrlPntDistance
 *
 *	NON-REENTRANT
 *
 *	This function computes the distance of the corridor control points from
 *	the first corridor control point.
 *
 *	Input:
 *		cur:	integer containing an index to the control points in the     
 *				corridor control point pool
 *		nth:	which control point in the current corridor
 *	Return Value:
 *		a float containing the value of the distance from the first 
 *		corridor control point
 *
 *--------------------------------------------------------------------------*/
static float GetCrdrCntrlPntDistance(int cur, int nth)
{
	float x2, y2;
	float distance;
	if (nth == 0)
		distance = 0;
	else{
		float x = ( pCrdrCntrlPntPool[cur].location.x -
					pCrdrCntrlPntPool[cur-1].location.x);
		float y = ( pCrdrCntrlPntPool[cur].location.y -
					pCrdrCntrlPntPool[cur-1].location.y);
		x2 = x*x;
		y2 = y*y;
		distance = (float)sqrt(x2+y2);
		distance += pCrdrCntrlPntPool[cur-1].distance;
	}
	return (distance);
}
		
/*---------------------------------------------------------------------------*
 *
 *	Name: SetCrdrCntrlPntPool
 *
 *	NON-REENTRANT
 *
 *	This function reads information of the corridor control points from
 *	&(TCrdr->CrdrCurveList) and write the information into corridor control
 *	point pool.
 *
 *	Input:
 *      pName: a string describing the corridor using the src/dst road/lanes.
 *		pCurveList: pointer pointing to an array containing corridor control
 *		point information
 *
 *---------------------------------------------------------------------------*/
static void SetCrdrCntrlPntPool(const char* pName, TCrdrCurveList* pCurveList)
{
	TCrdrCurve *pCrdrCurve = pCurveList->pCrdrCurve;
	int numCntrl = pCurveList->n_crdrpts;
	int i;
	int oldSize = sizeOfCrdrCntrlPntPool;
	TVector3D	normal;
	TVector3D	linTang;
	TVector3D	linRight;
	TPoint3D	currPt,prevPt,nextPt;


	sizeOfCrdrCntrlPntPool += numCntrl;
	pCrdrCntrlPntPool = (cvTCrdrCntrlPnt *)realloc(
								pCrdrCntrlPntPool, 
								sizeof(cvTCrdrCntrlPnt)* sizeOfCrdrCntrlPntPool
													);
	if (!pCrdrCntrlPntPool){
		lrierr(eCRDR_CNTRL_PNT_POOL_ALLOC_FAIL, "");
		exit(1);
	}
	else
		memset(pCrdrCntrlPntPool+oldSize, 0, numCntrl*sizeof(cvTCrdrCntrlPnt));

	normal.i = 0;
	normal.j = 0;
	normal.k = 1;
	for (i=0; i<numCntrl; i++){
		TCrdrLineInfo *pCrdrInfo = pCrdrCurve[i].pCrdrLineInfo;
		pCrdrCntrlPntPool[oldSize+i].location.x = pCrdrCurve[i].x;
		pCrdrCntrlPntPool[oldSize+i].location.y = pCrdrCurve[i].y;
		pCrdrCntrlPntPool[oldSize+i].width = pCrdrCurve[i].width;
		pCrdrCntrlPntPool[oldSize+i].distance = 
										GetCrdrCntrlPntDistance(oldSize+i, i);
/************ compute the right vector **********************************/
		currPt.x = pCrdrCurve[i].x;
		currPt.y = pCrdrCurve[i].y;
		currPt.z = 0;

		if ( i == 0 ) {
            /* for the first point */
            nextPt.x = pCrdrCurve[i+1].x;
            nextPt.y = pCrdrCurve[i+1].y;
            nextPt.z = 0;
            if ( !ComputeLinTangent( &currPt, &nextPt, &linTang ) ) {
				fprintf(stderr, 
"Found coincident successive control points in corridor %s\n", pName);
				exit(-1);
			}
        }
        else if ( i > 0 && i < numCntrl-1 ) {
            /* there is a point before and after */
            prevPt.x = pCrdrCurve[i-1].x;
            prevPt.y = pCrdrCurve[i-1].y;
            prevPt.z = 0;
            nextPt.x = pCrdrCurve[i+1].x;
            nextPt.y = pCrdrCurve[i+1].y;
            nextPt.z = 0;
            if ( !ComputeLinTangent( &currPt, &nextPt, &linTang) ) {
				fprintf(stderr, 
"Found coincident non successive control points (%d,%d)\nin corridor %s\n", 
						i-1, i+1, pName);
				fprintf(stderr, "%f, %f  ->  %f, %f\n",
					prevPt.x, prevPt.y, nextPt.x, nextPt.y);
				exit(-1);
			}
        }
        else if ( i == numCntrl-1 ) {
            /* for the last point */
            prevPt.x = pCrdrCurve[i-1].x;
            prevPt.y = pCrdrCurve[i-1].y;
            prevPt.z = 0;
            if ( !ComputeLinTangent( &prevPt, &currPt, &linTang) ) {
				fprintf(stderr, 
"Found coincident successive control points in corridor %s\n", pName);
				exit(-1);
			}
        }

		CrossProductVector( &linTang, &normal, &linRight );
		if ( !NormalizeVector( &linRight ) ) {
			fprintf(stderr, "Zero linear right vec at %d.\n", __LINE__);
		}
		pCrdrCntrlPntPool[oldSize+i].rightVecLinear.i = linRight.i;
		pCrdrCntrlPntPool[oldSize+i].rightVecLinear.j = linRight.j; 
			
/***********************************************************************/

		if (!(pCrdrInfo == NULL)){
			if (pCrdrInfo->lflag1)
				pCrdrCntrlPntPool[oldSize+i].flagStyle |= eLEFT_FLAG;
			if (!strcmp(pCrdrInfo->lstyle1, "DOTTED"))
				pCrdrCntrlPntPool[oldSize+i].flagStyle |= eLEFT_DOTTED;
			if (pCrdrInfo->lflag2)
				pCrdrCntrlPntPool[oldSize+i].flagStyle |= eRIGHT_FLAG;
			if (!strcmp(pCrdrInfo->lstyle2, "DOTTED"))
				pCrdrCntrlPntPool[oldSize+i].flagStyle |= eRIGHT_DOTTED;
		}
	}

	/*
	 * Now calculate the radius of curvature.  We build a
	 * spline and then use the spline's radius of curvature calculation
	 * on the control points.
	 */

	CSplineHermiteNonNorm   spl;
	for (i=0; i<numCntrl; i++){
		cvTCrdrCntrlPnt *pC = &pCrdrCntrlPntPool[oldSize+i];
		spl.addPoint(pC->location.x, pC->location.y, 0.0);
	}

	spl.calc();

	CCubicSplinePos  pos;
	for (i=0; i<numCntrl; i++){
		cvTCrdrCntrlPnt *pC = &pCrdrCntrlPntPool[oldSize+i];
		double r;
		pos.setPos(i, 0.0);
		spl.AdvanceMidway(pos,0.5); //this type of spline T is distance not percent
		if (spl.evalCurveRadiusAvgOverRegion(pos,10,r) ){
			pC->radius = (float)r;
		}
		else if (i > 0) {
			pC->radius = (pC-1)->radius;
		}else{
					fprintf(stderr, 
	"Can Not calculate curve for %s\n", pName);
					exit(-1);			
			}
	}
}

/*--------------------------------------------------------------------------*
 *
 *	Name: GetSrcRdIdx 
 *
 *	NON-REENTRANT
 *
 *	This function find source road of the corridor and return the index to
 *	the source road. It assumes road names could be found. 
 *
 *	Input:
 *		pCrdr:	pointer pointing to a linked list containing the information
 *				about corridors.
 *	Return Value:
 *		the index to the source road
 *
 *--------------------------------------------------------------------------*/
int debugNumber1 = 0;
int debugNumber2 = 0;

static TRoadPoolIdx GetSrcRdIdx(TCrdr* pCrdr)
{
	int i;
	int found = 0;
	for (i=1; i<sizeOfRoadPool; i++ ){
		int nameIdx;
		nameIdx = pRoadPool[i].nameIdx;
		found = (!strcmp(pCrdr->roadName1, &pCharPool[nameIdx]));
		if (found)
			return (i);
	} 
#if 0	
	printf("\n in GetSrcRdIdx %s is not found\t%d\tcrdrIdx %d", 
						pCrdr->roadName1, debugNumber1++, sizeOfCrdrPool-1);
	printf("\n pass print src\n");
#endif
	return (0);
}

/*--------------------------------------------------------------------------*
 *
 *	Name: GetDstRdIdx 
 *
 *	NON-REENTRANT
 *
 *	This function find destination road of the corridor and return the index to
 *	the destination road. It assumes road names could be found. 
 *
 *	Input:
 *		pCrdr:	pointer pointing to a linked list containing the information
 *				about corridors.
 *	Return Value:
 *		the index to the destination road
 *--------------------------------------------------------------------------*/
static TRoadPoolIdx GetDstRdIdx(TCrdr* pCrdr)
{
	int i;
	int found = 0;

	for (i=1; i<sizeOfRoadPool; i++ ){
		int nameIdx;
		nameIdx = pRoadPool[i].nameIdx;
		found = (!strcmp(pCrdr->roadName2, &pCharPool[nameIdx]));
		if (found)
			return (i);
	}

#if 0	
	printf("\n in GetDstRdIdx %s is not found\t%d\tcrdrIdx %d", 
						pCrdr->roadName2, debugNumber2++, sizeOfCrdrPool-1);
	printf("\n pass print dst\n");
#endif
	return (0);
}


/*--------------------------------------------------------------------------*
 *
 *	Name: GetDstRdIdx 
 *
 *	NON-REENTRANT
 *
 *	This function find source lane of the corridor and return the index to
 *	the source lane 
 *
 *	Input:
 *		pCrdr:	pointer pointing to a linked list containing the information
 *				about corridors.
 *	Return Value:
 *		the index to the source lane
 *--------------------------------------------------------------------------*/
static TLanePoolIdx GetSrcLnIdx(TCrdr* pCrdr)
{
	int srcRdIdx = pCrdrPool[sizeOfCrdrPool-1].srcRdIdx;
	int firstCntrlPntIdx = pRoadPool[srcRdIdx].cntrlPntIdx;
	int laneIdx = pCntrlPntPool[firstCntrlPntIdx].laneIdx;
	return (laneIdx+pCrdr->crdrLane1);
}

/*--------------------------------------------------------------------------*
 *
 *	Name: GetDstLnIdx 
 *
 *	NON-REENTRANT
 *
 *	This function find destination lane of the corridor and return the index to
 *	the destination road 
 *
 *	Input:
 *		pCrdr:	pointer pointing to a linked list containing the information
 *				about corridors.
 *	Return Value:
 *		the index to the destination lane
 *--------------------------------------------------------------------------*/
static TLanePoolIdx GetDstLnIdx(TCrdr* pCrdr)
{
	int dstRdIdx = pCrdrPool[sizeOfCrdrPool-1].dstRdIdx;
	int firstCntrlPntIdx = pRoadPool[dstRdIdx].cntrlPntIdx;
	int laneIdx = pCntrlPntPool[firstCntrlPntIdx].laneIdx;
	return (laneIdx+pCrdr->crdrLane2);
}

/*--------------------------------------------------------------------------*
 *
 *	Name: SetIntrsctnOfRoad
 *
 *	NON-REENTRANT
 *
 *  This function is called in the GenerateCode after the road while loop.
 *  It is to set the correct indexes of the srcIntrsctn and dstIntrsctn
 *  fields in cvTRoad so that source interections and destination 
 *  intersection can be accessed to through cvTRoad.
 *
 *	Input:
 *  pRoads	pointer to the road structure parsed from lri file
 *--------------------------------------------------------------------------*/
static void SetIntrsctnOfRoad(TRoads* pRoads) 
{
	int i;
	int loop = 0;
	TRoads* pCurrentRoad = pRoads;


	while( pCurrentRoad ){
		loop++;

		int nameIdx;
#if 0 
		if (!strcmp(pCurrentRoad->roadName, "R3_660_17820")){
			printf("\n\t\tsrcIntrsctn is %s\n", pCurrentRoad->intersection1);
			printf("\n\t\tdstIntrsctn is %s\n", pCurrentRoad->intersection2);
		}
#endif
		for(i=1; i<sizeOfIntrsctnPool; i++){
			nameIdx = pIntrsctnPool[i].nameIdx;
			if (!strcmp(pCurrentRoad->intersection1, &pCharPool[nameIdx])){
				pRoadPool[loop].srcIntrsctnIdx = i;
				break;
			}
		}

		if (i == sizeOfIntrsctnPool){
			printf("\nin road %s srcIntrsctn %s can't be found",
						pCurrentRoad->roadName, pCurrentRoad->intersection1);
			pRoadPool[loop].srcIntrsctnIdx = 0;
		}
	
		for(i=1; i<sizeOfIntrsctnPool; i++){
			nameIdx = pIntrsctnPool[i].nameIdx;
			if (!strcmp(pCurrentRoad->intersection2, &pCharPool[nameIdx])){
				pRoadPool[loop].dstIntrsctnIdx = i;
				break;
			}
		}

		if (i == sizeOfIntrsctnPool){
			printf("\nin road %s dstIntrsctn %s can't be found",
						pCurrentRoad->roadName, pCurrentRoad->intersection2);	
			pRoadPool[loop].dstIntrsctnIdx = 0;
		}

		pCurrentRoad = pCurrentRoad->pNext;
	}
}
				
				
/*--------------------------------------------------------------------------*
 *
 *	Name: SetIntrsctnIdxOfRoad
 *
 *	NON-REENTRANT
 *
 *	This function is called in the SetCorridorPool, and use the "srcLnIdx"
 *	"dstLnIdx", "srcRdIdx", "dstRdIdx" fields in the corridor to look for
 *	"dstIntrsctnIdx" and "srcIntrsctnIdx" fields in the cvTRoad. There is
 *	no guaranttee that every road will be filled with "srcIntrsctnIdx"
 *	and "dstIntrsctnIdx" unless a lri file does provide all of them
 *
 *	Input:
 *		CrdrIdx:	index of current corridor	
 *		TIntrsctnPoolIdx:	index of the intersection the corridor is on
 *--------------------------------------------------------------------------*/
static void SetIntrsctnIdxOfRoad(
				TCrdrPoolIdx		CrdrIdx, 
				TIntrsctnPoolIdx	intrsctnIdx) 
{
	int srcLnIdx = pCrdrPool[CrdrIdx].srcLnIdx;
	int dstLnIdx = pCrdrPool[CrdrIdx].dstLnIdx;
	int srcRdIdx = pCrdrPool[CrdrIdx].srcRdIdx;
	int dstRdIdx = pCrdrPool[CrdrIdx].dstRdIdx;

	if (pLanePool[srcLnIdx].direction == ePOS)
		pRoadPool[srcRdIdx].dstIntrsctnIdx = intrsctnIdx;
	else if(pLanePool[srcLnIdx].direction == eNEG)
		pRoadPool[srcRdIdx].srcIntrsctnIdx = intrsctnIdx;

	if (pLanePool[dstLnIdx].direction == ePOS)
		pRoadPool[dstRdIdx].srcIntrsctnIdx = intrsctnIdx;
	else if(pLanePool[dstLnIdx].direction == eNEG)
		pRoadPool[dstRdIdx].dstIntrsctnIdx = intrsctnIdx;
}

/*---------------------------------------------------------------------------*
 *
 *	Name: SetHoldOfsPool
 *
 *	NON-REENTRANT
 *
 *	This function reads information about hold offset and write them into
 *	hold offset pool. "isSign" has not been dealed with.
 * 
 * note: there is a comment like this at the end
 * of this function. some modification might
 * be needed in the future
 * ////////////////////////////////////////////////
 * // in fact, only "else{" here should be enough
 * // due to the buggy lri file that add signs or
 * // traffic lights without the existence of those
 * // obj being in the OBJECT field of the lri file,
 * // this "else if" is tempariraly added
 * /////////////////////////////////////////////////// 
 *
 *	Input:
 *		pHoldOfs:	poitner pointing to TCrdr->pHoldOfs which contains 
 *					information about hold offset
 *	Output:
 *		pNumHldOfs:	pointer pointing to an integer containing the number of
 *					hold offsets pHoldOfs contains		
 *---------------------------------------------------------------------------*/
static void SetHldOfsPool(THoldOfs* pHldOfs, int* pNumHldOfs)
{
	while(pHldOfs){
		int i;
		sizeOfHldOfsPool++;
		(*pNumHldOfs)++;
		pHldOfsPool = (cvTHldOfs *)realloc(
										pHldOfsPool, 
										sizeof(cvTHldOfs)*sizeOfHldOfsPool);
		if (!pHldOfsPool){
			lrierr(eHLD_OFS_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		else
			memset(pHldOfsPool+sizeOfHldOfsPool-1, 0, sizeof(cvTHldOfs));

		pHldOfsPool[sizeOfHldOfsPool-1].distance = pHldOfs->distance;

		if (!strcmp(pHldOfs->crdrReason, "SIGN"))
			pHldOfsPool[sizeOfHldOfsPool-1].reason |= eHOLD_SIGN;
		else if (!strcmp(pHldOfs->crdrReason, "TLIGHT"))
			pHldOfsPool[sizeOfHldOfsPool-1].reason |= eHOLD_TLIGHT;
		else if (!strcmp(pHldOfs->crdrReason, "OVLAP"))
			pHldOfsPool[sizeOfHldOfsPool-1].reason |= eHOLD_OVLAP;

		int isLine = 0;
		int isSign = 0;
		for (i=0; i<2; i++){
			if (pHldOfs->info[i].isLine == 1){
				if (isLine == 1){
					lrierr(	
						eSEMANTICS_ERROR, 
						"%s", 
						"having more than one line information in hold"
						"offset");
					exit(1);
				} else {
					isLine = 1;
					pHldOfsPool[sizeOfHldOfsPool-1].thickness = 
													pHldOfs->info[i].thick;
					pHldOfsPool[sizeOfHldOfsPool-1].orientation = 
													pHldOfs->info[i].angle;
				}
			}
			if (pHldOfs->info[i].isSign == 1){
				if (isSign == 1){
					lrierr(
						eSEMANTICS_ERROR,
						"%s",
						"having more than one sign information in hold"
						"offset");
					exit(1);
////////////////////////////////////////////////
// in fact, only "else{" here should be enough
// due to the buggy lri file that add signs or
// traffic lights without the existence of those
// obj being in the OBJECT field of the lri file,
// this "else if" is tempariraly added
/////////////////////////////////////////////////// 
#if 0
				} else if(pHldOfsPool[sizeOfHldOfsPool-1].reason&eHOLD_TLIGHT){
#endif
				} else {
					isSign = 1;

					int objIdx = cNUM_DYN_OBJS;
					for (; objIdx < sizeOfObjPool; objIdx++){
						if (!strcmp(pObjPool[objIdx].name, 
									pHldOfs->info[i].holdSign)){
							pHldOfsPool[sizeOfHldOfsPool-1].objIdx = objIdx;
							break;
						}
					}
#if 1
					if (objIdx == sizeOfObjPool){
						//lrierr(eBAD_SIGN_OR_LIGHT_NAME,"");
						fprintf(stderr, "\n \"%s\" can not be found in"
								" object pool", pHldOfs->info[i].holdSign);
					}
#endif
				}
			}
		}
		

		pHldOfs = pHldOfs->pNextHoldOfs;
	}
}


/****************************************************************************
 *
 * This function makes a border for intersections that don't have
 * one.
 *
 */
static void
MakeBorder(TBorder &border, cvTIntrsctn *pInt)
{
	cvTCntrlPnt *pCP;

	if (pInt->numOfRoads <= 2) {
#if 0
		printf("\nIntersection %d(%s) has %d roads: ", 
				pInt->myId, 
				&pCharPool[pInt->nameIdx],
				pInt->numOfRoads);

		for (int k=0; k<pInt->numOfRoads; k++) {
			printf("%d (%s)", pInt->adjacentRoadId[k], 
					&pCharPool[pRoadPool[pInt->adjacentRoadId[k]].nameIdx]);
		}
		printf("\n");
#endif

		// First deal with the single road case.  The border will be 
		// a triangle with two vertices been the road end points and
		// the third point been away from the road.

		if ( pInt->numOfRoads == 1 ) {
			int  rid = pInt->adjacentRoadId[0];


			if ( pRoadPool[rid].srcIntrsctnIdx == pInt->myId ) {
#if 0
				printf("This is the source intersection for the sole road\n");
#endif
				pCP = &pCntrlPntPool[pRoadPool[rid].cntrlPntIdx];

				CPoint3D   p(pCP->location);
				CVector3D  tang(pCP->tangVecLinear);
				CVector3D  right(pCP->tangVecLinear);
				CPoint3D   p1; 
				CPoint3D   p2; 
				CPoint3D   p3;

				p1 = p + 0.5 * pCP->physicalWidth * right;
				p2 = p - pCP->physicalWidth * tang;
				p3 = p - 0.5 * pCP->physicalWidth * right;

				border.nPts = 3;
				border.list[0].borderX = p1.m_x;
				border.list[0].borderY = p1.m_y;
				border.list[0].lflag   = 0;
				border.list[1].borderX = p2.m_x;
				border.list[1].borderY = p2.m_y;
				border.list[1].lflag   = 0;
				border.list[2].borderX = p3.m_x;
				border.list[2].borderY = p3.m_y;
				border.list[2].lflag   = 0;
			}
			//else if ( pRoadPool[rid].dstIntrsctnIdx == pInt->myId ){
			else{
#if 0
				printf("This is the destination intersection for the sole road\n");
#endif
				pCP = &pCntrlPntPool[pRoadPool[rid].cntrlPntIdx
					+ pRoadPool[rid].numCntrlPnt-1];

				CPoint3D   p(pCP->location);
				CVector3D  tang(pCP->tangVecLinear);
				CVector3D  right(pCP->tangVecLinear);
				CPoint3D   p1; 
				CPoint3D   p2; 
				CPoint3D   p3;

				p1 = p + 0.5 * pCP->physicalWidth * right;
				p2 = p + pCP->physicalWidth * tang;
				p3 = p - 0.5 * pCP->physicalWidth * right;

				border.nPts = 3;
				border.list[0].borderX = p1.m_x;
				border.list[0].borderY = p1.m_y;
				border.list[0].lflag   = 0;
				border.list[1].borderX = p2.m_x;
				border.list[1].borderY = p2.m_y;
				border.list[1].lflag   = 0;
				border.list[2].borderX = p3.m_x;
				border.list[2].borderY = p3.m_y;
				border.list[2].lflag   = 0;
			}
#if 0
			else{
				printf("\n in MakeBorder() when num of roads == 1");
				printf("\n intersection %d does not match ", pInt->myId);
				printf(" either src or dstIntrsctnIdx of ");
				printf(" adjacentRoadId of itsef; \n");
				exit(1);
			}
#endif
		}
		else {
#if 1
			border.nPts = 3;
			border.list[0].borderX = 0;
			border.list[0].borderY = 0;
			border.list[1].borderX = 0;
			border.list[1].borderY = 0;
			border.list[2].borderX = 0;
			border.list[2].borderY = 0;
#endif
			int  rid = pInt->adjacentRoadId[0];

			if ( pRoadPool[rid].srcIntrsctnIdx == pInt->myId ) {
#if 0
				printf("This is the source intersection for the first road\n");
#endif
			}
			//else if ( pRoadPool[rid].dstIntrsctnIdx == pInt->myId ){
			else{
#if 0
				printf("This is the destination intersection for the first road\n");
#endif
			}
#if 0
			else {
				printf("\n in MakeBorder() when num of roads == 2");
				printf("\n intersection %d does not match ", pInt->myId);
				printf(" either src or dstIntrsctnIdx of ");
				printf(" adjacentRoadId of itsef; \n");
				exit(1);
			}
#endif

			rid = pInt->adjacentRoadId[1];
			if ( pRoadPool[rid].srcIntrsctnIdx == pInt->myId ) {
#if 0
				printf("This is the source intersection for the second road\n");
#endif
			}
			else {
#if 0
				printf("This is the destination intersection for the second road\n");
#endif
			}

		}
	}
	else { // Number of roads >= 3
		CEndRoadPoints roadPoints;
		cvTCntrlPnt *pCP;
		vector<CEndRoadPoints> borderPoints;
		for (int roadNum = 0; roadNum < pInt->numOfRoads; roadNum++) {
			int rid = pInt->adjacentRoadId[roadNum];
			if (pRoadPool[rid].srcIntrsctnIdx == pInt->myId ) {
				pCP = &pCntrlPntPool[pRoadPool[rid].cntrlPntIdx];
			}
#if 0
			else if (pRoadPool[rid].dstIntrsctnIdx == pInt->myId ) {
#endif
			else{
				pCP = &pCntrlPntPool[pRoadPool[rid].cntrlPntIdx
					+ pRoadPool[rid].numCntrlPnt-1];
			}
#if 0
			else {
				printf("\n in MakeBorder() when number of roads > 2");
				printf("\n intersection %d does not match ", pInt->myId);
				printf(" either src or dstIntrsctnIdx of ");
				printf(" adjacentRoadId of itsef; \n");
				exit(1);
			}
#endif

			CPoint3D   orig(pCP->location);
			CVector3D  tang(pCP->tangVecLinear);
			CVector3D  right(pCP->rightVecLinear);
			if (pRoadPool[rid].srcIntrsctnIdx != pInt->myId ) {
				tang = CVector3D(0,0,0) - tang;
				right = CVector3D(0,0,0) - right;
			}
			roadPoints.point1 = orig + 0.5 * pCP->physicalWidth * right; 
			roadPoints.point2 = orig - 0.5 * pCP->physicalWidth * right; 
			roadPoints.tangent = tang;
			borderPoints.push_back( roadPoints );
		}
		sort( borderPoints.begin(), borderPoints.end() );
		border.nPts = borderPoints.size() * 2;
		int index = 0;
		for (vector<CEndRoadPoints>::iterator i = borderPoints.begin();
				i != borderPoints.end();
				i++ ) {
			border.list[index].borderX = i->point1.m_x;
			border.list[index].borderY = i->point1.m_y;
			border.list[index].lflag   = 0;
			index++;
			border.list[index].borderX = i->point2.m_x;
			border.list[index].borderY = i->point2.m_y;
			border.list[index].lflag   = 0;
			index++;
		}
	}
}

/*--------------------------------------------------------------------------*
 *
 *	Name: SetCorridorPool
 *
 *	NON-REENTRANT
 *
 *	This function reads corridor information from the intersection and write
 *	into corridor pool.
 *
 *	Input:
 *		pCrdr:	pointer pointing to "TIntersection->pCrdr" which contains
 *				information about corridor
 *		intrsctnId: id of intersection where the corridor is	
 *
 *--------------------------------------------------------------------------*/
static void SetCorridorPool(TCrdr* pCrdr, int intrsctnId, int* pNumCrdrs) 
{
	*pNumCrdrs = 0;
	while(pCrdr){
		int numHldOfs = 0;
		char  crdrName[400];

		(*pNumCrdrs)++;
		sizeOfCrdrPool++;
		pCrdrPool = (cvTCrdr *)realloc(
										pCrdrPool, 
										sizeof(cvTCrdr)*sizeOfCrdrPool
									);	
		if (!pCrdrPool){
			lrierr(eCRDR_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		else
			memset(pCrdrPool+sizeOfCrdrPool-1, 0, sizeof(cvTCrdr));
		sprintf(crdrName, "%s:%d->%s:%d", pCrdr->roadName1,
					pCrdr->crdrLane1, pCrdr->roadName2, pCrdr->crdrLane2);

		pCrdrPool[sizeOfCrdrPool-1].myId = (sizeOfCrdrPool-1);
		pCrdrPool[sizeOfCrdrPool-1].intrsctnId = intrsctnId;
		pCrdrPool[sizeOfCrdrPool-1].cntrlPntIdx = sizeOfCrdrCntrlPntPool;
		pCrdrPool[sizeOfCrdrPool-1].numCntrlPnt = 
											pCrdr->CrdrCurveList.n_crdrpts;
		SetCrdrCntrlPntPool(crdrName, &(pCrdr->CrdrCurveList));
		pCrdrPool[sizeOfCrdrPool-1].srcRdIdx = GetSrcRdIdx(pCrdr);
		pCrdrPool[sizeOfCrdrPool-1].srcLnIdx = GetSrcLnIdx(pCrdr);
		pCrdrPool[sizeOfCrdrPool-1].dstRdIdx = GetDstRdIdx(pCrdr);
		pCrdrPool[sizeOfCrdrPool-1].dstLnIdx = GetDstLnIdx(pCrdr);
		pCrdrPool[sizeOfCrdrPool-1].hldOfsIdx = sizeOfHldOfsPool;
		SetHldOfsPool(pCrdr->pHoldOfs, &numHldOfs);
		pCrdrPool[sizeOfCrdrPool-1].numHldOfs = numHldOfs;
 		pCrdrPool[sizeOfCrdrPool-1].attrIdx = sizeOfAttrPool;
		int numAttr = 0;
		SetAttrPool(pCrdr->pAttrs, &numAttr, cCV_LRI_CRDR_ATTR_EXTRA);	
		pCrdrPool[sizeOfCrdrPool-1].numAttr = numAttr;

		/*
		 *	set src and dst intsctnIdx in RoadPool
		 */
#if 0
// this function is replaced by SetIntrsctnOfRoad called in the    
// GeneraateCode right outside of the road loop
		SetIntrsctnIdxOfRoad(sizeOfCrdrPool-1, sizeOfIntrsctnPool-1);
#endif

		pCrdr = pCrdr->pNext;
	}
}	

/*---------------------------------------------------------------------------*
 *
 *	Name: SetIntrsctnPool
 *
 *	NON-REENTRANT
 *
 *	This function take as parameter "pInter" which contains all relevant
 *	information about intersection, and write them into intersection 
 *	pool
 *
 *	Input:
 *		pInter:	This is a pointer to the begining of a linked list
 *				containing information about elevation for intersections.	
 *
 *---------------------------------------------------------------------------*/
static void SetIntrsctnPool(
				TIntersection*	pInter,
				const TElevMap*	pElevMaps,
				CQuadTree&		intrsctnQTree)
{
	while (pInter) { 
		int numCrdrs;
		TBorder *pBorder = &pInter->Border;

		sizeOfIntrsctnPool++;
		pIntrsctnPool = (cvTIntrsctn *)realloc(pIntrsctnPool, 
								sizeof(cvTIntrsctn)* sizeOfIntrsctnPool);
		if (!pIntrsctnPool){
			lrierr(eINTRSCTN_POOL_ALLOC_FAIL, "");
			exit(1);
		}
		else
			memset(pIntrsctnPool+sizeOfIntrsctnPool-1, 0, sizeof(cvTIntrsctn));

		// make a variable for the entry in the pool that we are using
		cvTIntrsctn *pEntry = &pIntrsctnPool[sizeOfIntrsctnPool-1];

		pEntry->myId = sizeOfIntrsctnPool - 1;
		pEntry->nameIdx = sizeOfCharPool;
		SetCharPool(pInter->name);

		pEntry->numOfRoads = SetRoadIdOfIntrsctn(pInter->pRoadNames);

		pEntry->crdrIdx = sizeOfCrdrPool;
		SetCorridorPool(pInter->pCrdr, pEntry->myId, &numCrdrs);
		pEntry->numOfCrdrs = numCrdrs;


		//
		// If an intersection has only one or two roads and has no
		// border, we make one.  Otherwise, we use the border, or
		// signal an error if a "normal" intersection doesn't have
		// a border polygon.
		//
		if ( pBorder->nPts < 3 ) {
			MakeBorder(*pBorder, pEntry);

		}
#if 0
		else
		if ( pBorder->nPts < 3 ) {
			lrierr(eBAD_INTRSCTN_BORDER, "\nIntersection '%s'.", pInter->name);
			exit(-1);
		}
#endif

		pEntry->numBorderSeg = pBorder->nPts;
		pEntry->borderSegIdx = sizeOfBorderSegPool;
		SetBorderSegPool(pBorder);

		//store intrsctn into quadtree
		CPoint2D llPt, urPt;				//lower left and upper right points
		CQuadTree::CItem	item;
		FindBoundaryOfIntrsctn(pEntry, llPt, urPt);
		InitItem(item, pEntry->myId, llPt.m_x, llPt.m_y, urPt.m_x, urPt.m_y);
		intrsctnQTree.Insert(item);

		pEntry->attrIdx = sizeOfAttrPool;
		int numAttr = 0;	/* number of attributes hanging with intrsctn */
		SetAttrPool(pInter->pAttrs, &numAttr, cCV_LRI_INTRSCTN_ATTR_EXTRA);
		pEntry->numAttr = numAttr;

		if ( pInter->elevInfo.elevMapName[0] == '\0' ) { /* no elev map */
			// zero unused entries
			pEntry->elevMap   = 0;
			pEntry->elevMapX  = 0.0;
			pEntry->elevMapY  = 0.0;

			pEntry->elevation = pInter->elevInfo.height;
		}
		else {
			/* look for the elevation map in the pool */
			const TElevMap   *pMap;
			TElevMapPoolIdx  mapIdx;
			bool found     = false;

			mapIdx = 1;
			for (pMap = pElevMaps; pMap; pMap = pMap->pNextElevMap) {
				if ( !strcmp(pMap->elevMapName, pInter->elevInfo.elevMapName)) {
					found = true;
					break;
				}
				mapIdx++;
			}

			if ( !found ) {
				lrierr(eBAD_ELEV_MAP, 
				"\nIntersection '%s' references unknown elevation map '%s'.",
				pInter->name, pInter->elevInfo.elevMapName);
				exit(-1);
			}

			pEntry->elevMap   = mapIdx;
			pEntry->elevMapX  = pInter->elevInfo.xorig;
			pEntry->elevMapY  = pInter->elevInfo.yorig;

			// zero unused entry
			pEntry->elevation = 0.0;
		}
		SetCrdrMrgDstPool(pEntry, sizeOfCrdrMrgDstPool, &pCrdrMrgDstPool);
		pInter = pInter->pNext;
	}
}

/*--------------------------------------------------------------------------*
 *
 *	Name: SetRoadIdOfIntrsctn
 *
 *	NON-REENTRANT
 *
 *	This function takes "pRoadNames" field which contains names of roads
 *	adjacent to the intersection, find them in the char pool, and assign
 *	the correct index to the char pool in the "adjacentRoadId" field in
 *	the intersection pool
 * 
 *	Input:
 *		pRoadNames: pointer pointing to TIntersection->pRoadNames which
 *					contains the names of the roads adjacent to the inter-
 *					section.
 *
 *	Return Value:
 *		return an integer which contains the number of adjacent roads
 *
 *--------------------------------------------------------------------------*/
static int SetRoadIdOfIntrsctn(TRoadName *pRoadNames)
{
	int nth = 0;				/* number of adjacent road */
	while(pRoadNames){
		int i;					/* counter */
		int nameFound = 0;		/* if the road name is found */
		for (i=1; i<sizeOfRoadPool; i++){
			TCharPoolIdx nameIdx;	
			nameFound = 0;
			nameIdx = pRoadPool[i].nameIdx;
			nameFound = (!strcmp(pRoadNames->roadName, &pCharPool[nameIdx]));
			if (nameFound){
				pIntrsctnPool[sizeOfIntrsctnPool-1].adjacentRoadId[nth++] = i;
				break;
			}
		}
		if (!nameFound){
			lrierr(	eBAD_ROAD, 
					" Road name %s couldn't be found",
					pRoadNames->roadName
					);
			exit(1);
		}
		pRoadNames = pRoadNames->pNextRoadName;
	}
	return (nth);
}
	
void padMultiple(int mult, FILE *pOut, long *pOfs)
{
	long ofs = 0;
	if ( (*pOfs) % mult != 0 ) {
		char c = 0;
		int  i;

		for (i=0; i<(mult - ( (*pOfs) % mult)); i++ ) {
			fwrite(&c, sizeof(char), 1, pOut);
			ofs ++;
		}
		(*pOfs) += ofs;
	}
}

void WriteZeros(FILE *pOut, int n)
{
	char c = 0;
	int  i;

	for (i=0; i<n; i++ ) {
		fwrite(&c, 1, 1, pOut);
	}
}


void DumpPools(const cvTHeader *pH)
{
	DumpCharPool(pCharPool, pH->charCount);
	DumpRoadPool(pRoadPool, pH->roadCount);
	DumpIntrsctnPool(pIntrsctnPool, pH->intrsctnCount);
	DumpRepObjPool(pRepObjPool, pH->repObjCount);
	DumpLanePool(pLanePool, pH->laneCount);
	DumpAttrPool(pAttrPool, pH->attrCount);
	DumpLatCntrlPntPool(pLatCntrlPntPool, pH->latCntrlPntCount);
	DumpCntrlPnt(pCntrlPntPool, pH->longitCntrlCount);
	DumpCrdrPool(pCrdrPool, pH->crdrCount);
	DumpCrdrCntrlPntPool(pCrdrCntrlPntPool, pH->crdrCntrlPntCount);
	DumpBorderSeg(pBorderSegPool, pH->borderCount);
	DumpHldOfsPool(pHldOfsPool, pH->hldCount);
}

void BuildObjQTree()
{
	int i;
	for (i=cNUM_DYN_OBJS; i<sizeOfObjPool; i++){
		CQuadTree::CItem    item;
		// for all static objects
		InitItem(
				item,
				pObjPool[i].myId,
				pObjPool[i].stateBufA.state.anyState.boundBox[0].x,
				pObjPool[i].stateBufA.state.anyState.boundBox[0].y,
				pObjPool[i].stateBufA.state.anyState.boundBox[1].x,
				pObjPool[i].stateBufA.state.anyState.boundBox[1].y);
		staticObjQTree.Insert(item);
		// for objects of type terrain
		if (pObjPool[i].type == eCV_TERRAIN)
			trrnObjQTree.Insert(item);
	}
	//staticObjQTree.Optimize();
	//trrnObjQTree.Optimize();
}


/*--------------------------------------------------------------------------*
 *
 *	Name: WriteOutputFile
 *
 *	This function writes the output lri file.  It first writes
 *  the header and then uses a bunch of global variables holding information
 *  about the various memory pools to write them to disk.  As it
 *  writes each memory pool, it keeps track of their offset and once
 *  done, it  updates the header and re-writes it so it reflects
 *  the actual offsets of the pools.
 * 
 *	Input:
 *		The name of the file to write to.  All global variables
 *      holding pool information.
 *
 *	Return Value:
 *		None
 *
 *--------------------------------------------------------------------------*/
void WriteOutputFile(
		const char *pFname,
		const vector<cvTElevPost> &elevPosts,
		const vector<cvTElevMap> &elevMaps)
{
	
	FILE *pOut = fopen(pFname, "wb");
	cvTHeader header = { 0 };
	long      ofs;
	int       n;
	int       dummyCount;			/* counter for filler objects */
	cvTObj    dummy = { 0 };		/* the filler object */

	if ( pOut == NULL ) {
		perror("Can't open output file");
		exit(-1);
	}

	/* write the header to allocate space in the file.  We
	 * have to come back later and re-write it once it has all
	 * the correct values.
	 */
	ofs = 0;

	n = fwrite(&header, sizeof(header), 1, pOut);
	/* check that n is 1, if not error & exist */
	if (n != 1){
		lrierr(eMEM_WRITE_FAIL, "%s .", "when writting header to binary file");
		fprintf(stderr, "\n didn't get to write header \n");
		exit(1);
	}
	ofs += sizeof(header);
	padMultiple(8, pOut, &ofs);

	/* 
	 *	set fields of shortComment and description
	 */	
	header.shortComment = sizeOfCharPool;
	SetCharPool("shortcomment");
	header.description = sizeOfCharPool;
	SetCharPool("description");

	header.charOfs   = ofs;
	header.charCount = sizeOfCharPool;
	header.charStrgCount = sizeOfCharPool + 2000;
	n = fwrite(pCharPool, sizeof(char), sizeOfCharPool, pOut);
	if (n != sizeOfCharPool){
		lrierr(	eMEM_WRITE_FAIL, 
				"%s .", 
				"when writting char pool to binary file"
				);
		exit(1);
	}
	WriteZeros(pOut, 2000 * sizeof(char));
	ofs += header.charStrgCount;
	padMultiple(8, pOut, &ofs);

	///////////////////////////////////////////
	//
	// write the environment area pool
	//
	//
	header.envAreaOfs   = ofs;
	header.envAreaCount = 1;
	header.envAreaSize  = cNUM_ENV_AREA;
	cvTEnviroArea* pEnvAreaPool	= new cvTEnviroArea[cNUM_ENV_AREA];
	if (!pEnvAreaPool){
		lrierr(eENV_AREA_POOL_ALLOC_FAIL, "");
		exit(1);
	} else
		memset(pEnvAreaPool, 0, sizeof(cvTEnviroArea) * cNUM_ENV_AREA);
	pEnvAreaPool[0].maxNumOfInfo = (cEXTRA_ENV_INFO + cNUM_ENVIRO_TYPE);
	n = fwrite(pEnvAreaPool, sizeof(cvTEnviroArea), (cNUM_ENV_AREA), pOut);
	if (n != (cNUM_ENV_AREA)){
        lrierr( eMEM_WRITE_FAIL,
                "%s .",
                "when writting enviro area pool to binary file"
                );
    }
	delete [] pEnvAreaPool;
	ofs += cNUM_ENV_AREA * sizeof(cvTEnviroArea);
	padMultiple(8, pOut, &ofs);

	///////////////////////////////////////////
	//
	// write the environment information pool
	//
	//
	header.envInfoOfs   = ofs;
	header.envInfoCount = (cEXTRA_ENV_INFO + cNUM_ENVIRO_TYPE);
	int sizeOfInfo = cNUM_ENV_AREA * (cEXTRA_ENV_INFO + cNUM_ENVIRO_TYPE);	
	header.envInfoSize  = sizeOfInfo;
	cvTEnviroInfo* pEnvInfoPool	= new cvTEnviroInfo[sizeOfInfo];
	if (!pEnvInfoPool){
		lrierr(eENV_INFO_POOL_ALLOC_FAIL, "");
		exit(1);
	} else
		memset(pEnvInfoPool, 0, sizeof(cvTEnviroInfo) * sizeOfInfo);
	n = fwrite(pEnvInfoPool, sizeof(cvTEnviroInfo), sizeOfInfo, pOut);
	if (n != sizeOfInfo){
        lrierr( eMEM_WRITE_FAIL,
                "%s .",
                "when writting enviro info pool to binary file"
                );
    }
	delete [] pEnvInfoPool;
	ofs += sizeOfInfo * sizeof(cvTEnviroInfo);
	padMultiple(8, pOut, &ofs);

	header.crdrMrgDstCount = sizeOfCrdrMrgDstPool;
	header.crdrMrgDstOfs = ofs;
	n = fwrite(
			pCrdrMrgDstPool, 
			sizeof(cvTCrdrMrgDst), 
			sizeOfCrdrMrgDstPool,
			pOut);
	if (n != sizeOfCrdrMrgDstPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .",
				"when writing crdr merge dst pool to binary file");
		exit(1);
	}
//	delete [] pCrdrMrgDstPool;
	ofs += sizeOfCrdrMrgDstPool * sizeof(cvTCrdrMrgDst);
	padMultiple(8, pOut, &ofs);
			

	header.roadOfs   = ofs;
	header.roadCount = sizeOfRoadPool;
	n = fwrite(pRoadPool, sizeof(cvTRoad), sizeOfRoadPool, pOut);
	if (n != sizeOfRoadPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting road pool to binary file"
                );

		exit(1);
	}
	ofs += sizeOfRoadPool * sizeof(cvTRoad);
	padMultiple(8, pOut, &ofs);

	header.intrsctnOfs	= ofs;
	header.intrsctnCount = sizeOfIntrsctnPool;
	n = fwrite(pIntrsctnPool, sizeof(cvTIntrsctn), sizeOfIntrsctnPool, pOut);
	if (n != sizeOfIntrsctnPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting intersection pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfIntrsctnPool * sizeof(cvTIntrsctn);
	padMultiple(8, pOut, &ofs);


	///////////////////////////////////////////
	//
	// write the elevation map pool
	//
	//
	header.elevMapOfs = ofs;
	header.elevMapCount = elevMaps.size();
	vector<cvTElevMap>::const_iterator  map;

	for (map = elevMaps.begin(); map != elevMaps.end(); map++) {
		cvTElevMap  oneMap = *map;
		if ( fwrite(&oneMap, sizeof(oneMap), 1, pOut) != 1 ) {
			lrierr( eMEM_WRITE_FAIL, "%s .", "when writing elevation maps.");
			exit(1);
		}
	}
	ofs += elevMaps.size() * sizeof(cvTElevMap);
	padMultiple(8, pOut, &ofs);


	///////////////////////////////////////////
	//
	// write the elevation post pool
	//
	//
	header.elevPostOfs = ofs;
	header.elevPostCount = elevPosts.size();
	vector<cvTElevPost>::const_iterator  post;

	for (post = elevPosts.begin(); post != elevPosts.end(); post++) {
		cvTElevPost  onePost = *post;
		if ( fwrite(&onePost, sizeof(onePost), 1, pOut) != 1 ) {
			lrierr( eMEM_WRITE_FAIL, "%s .", "when writing elevation posts.");
			exit(1);
		}
	}
	ofs += elevPosts.size() * sizeof(cvTElevPost);
	padMultiple(8, pOut, &ofs);


	/////////////////////////
	header.longitCntrlOfs	= ofs;
	header.longitCntrlCount = sizeOfCntrlPntPool;
	n = fwrite(pCntrlPntPool, sizeof(cvTCntrlPnt), sizeOfCntrlPntPool, pOut);
	if (n != sizeOfCntrlPntPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting control point pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfCntrlPntPool * sizeof(cvTCntrlPnt);
	padMultiple(8, pOut, &ofs);

	header.crdrOfs		= ofs;
	header.crdrCount	= sizeOfCrdrPool;
	n = fwrite(pCrdrPool, sizeof(cvTCrdr), sizeOfCrdrPool, pOut);
	if (n != sizeOfCrdrPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting corridor pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfCrdrPool * sizeof(cvTCrdr);
	padMultiple(8, pOut, &ofs);

	header.laneOfs		= ofs;
	header.laneCount	= sizeOfLanePool;
	n = fwrite(pLanePool, sizeof(cvTLane), sizeOfLanePool, pOut);
	if (n != sizeOfLanePool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting lane pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfLanePool * sizeof(cvTLane);
	padMultiple(8, pOut, &ofs);

	header.crdrCntrlPntOfs	= ofs;
	header.crdrCntrlPntCount	= sizeOfCrdrCntrlPntPool;
	n = fwrite(	pCrdrCntrlPntPool, sizeof(cvTCrdrCntrlPnt), 
				sizeOfCrdrCntrlPntPool, pOut);
	if (n != sizeOfCrdrCntrlPntPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting corridor control point pool to binary file" 
				);
		exit(1);
	}
	ofs += sizeOfCrdrCntrlPntPool * sizeof(cvTCrdrCntrlPnt);
	padMultiple(8, pOut, &ofs);

	header.latCntrlPntOfs	= ofs;
	header.latCntrlPntCount = sizeOfLatCntrlPntPool;
	n = fwrite(	pLatCntrlPntPool, sizeof(cvTLatCntrlPnt), 
				sizeOfLatCntrlPntPool, pOut);
	if (n != sizeOfLatCntrlPntPool){
		lrierr(	eMEM_WRITE_FAIL,
				"%s .", 
				"when writting lateral control point pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfLatCntrlPntPool * sizeof(cvTLatCntrlPnt);
	padMultiple(8, pOut, &ofs);

	header.borderOfs  = ofs;
	header.borderCount = sizeOfBorderSegPool;
	n = fwrite( pBorderSegPool, sizeof(cvTBorderSeg),
							 sizeOfBorderSegPool, pOut);
	if (n != sizeOfBorderSegPool){
		lrierr( eMEM_WRITE_FAIL,  
				"%s .", 
				"when writting border segment pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfBorderSegPool * sizeof(cvTBorderSeg);
    padMultiple(8, pOut, &ofs);

				 
	header.hldOfs   = ofs;
	header.hldCount = sizeOfHldOfsPool;
	n = fwrite( pHldOfsPool, sizeof(cvTHldOfs), sizeOfHldOfsPool, pOut);
	if (n != sizeOfHldOfsPool){
		lrierr( eMEM_WRITE_FAIL,  
				"%s .", 
				"when writting hold offset pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfHldOfsPool * sizeof(cvTHldOfs);
    padMultiple(8, pOut, &ofs);

	header.attrOfs   = ofs;
	header.attrCount = sizeOfAttrPool;
	n = fwrite( pAttrPool, sizeof(cvTAttr), sizeOfAttrPool, pOut);
	if (n != sizeOfAttrPool){
		lrierr( eMEM_WRITE_FAIL,  
				"%s .", 
				"when writting attribute pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfAttrPool * sizeof(cvTAttr);
    padMultiple(8, pOut, &ofs);

	header.repObjOfs   = ofs;
	header.repObjCount = sizeOfRepObjPool;
	n = fwrite( pRepObjPool, sizeof(cvTRepObj), sizeOfRepObjPool, pOut);
	if (n != sizeOfRepObjPool){
		lrierr( eMEM_WRITE_FAIL,  
				"%s .", 
				"when writting repeated object pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfRepObjPool * sizeof(cvTRepObj);
    padMultiple(8, pOut, &ofs);

	header.objRefOfs   = ofs;
	header.objRefCount = sizeOfObjRefPool;
	n = fwrite( pObjRefPool, sizeof(cvTObjRef), sizeOfObjRefPool, pOut);
	if (n != sizeOfObjRefPool){
		lrierr( eMEM_WRITE_FAIL,  
				"%s .", 
				"when writting  object reference pool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfObjRefPool * sizeof(cvTObjRef);
    padMultiple(8, pOut, &ofs);


	/* write object pool; after the actual objects, write
	 * a few additional slots which can later be used for
	 * creating static objects at runtime
	 */
	header.objectOfs          = ofs;
	header.objectCountInitial = sizeOfObjPool;
	header.objectCount        = sizeOfObjPool;
	header.objectStrgCount    = sizeOfObjPool + cEXTRA_OBJ_SLOTS;
	n = fwrite(pObjPool, sizeof(cvTObj), sizeOfObjPool, pOut);
	if (n != sizeOfObjPool){
		lrierr( eMEM_WRITE_FAIL, "%s .",
				"when writting object pool to binary file");
		exit(1);
	}
	/* extras */
	for (dummyCount=0; dummyCount < cEXTRA_OBJ_SLOTS; dummyCount++) {
		if ( fwrite(&dummy, sizeof(cvTObj), 1, pOut) != 1 ) {
			lrierr( eMEM_WRITE_FAIL, "%s .",
					"when writting object pool to binary file");
			exit(1);
		}
	}
	ofs += (sizeOfObjPool + cEXTRA_OBJ_SLOTS) * sizeof(cvTObj);
    padMultiple(8, pOut, &ofs);

#if 0
	/* write object attribute pool */
//	header.objectAttrOfsJunk   = ofs;
//	header.objectAttrCount     = sizeOfObjAttrPool;
//	header.objectAttrStrgCount = sizeOfObjAttrPool + cEXTRA_ATTR_SLOTS;
//	n = fwrite(pObjAttrPool, sizeof(cvTObjAttr), sizeOfObjAttrPool, pOut);
//	printf("$$ writing %d bytes\n", sizeof(cvTObjAttr) * (sizeOfObjAttrPool) );
//	if (n != sizeOfObjAttrPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .",
				"when writting object attribute pool to binary file"
				);
		exit(1);
	}
	cvTObjAttr attr = {0};
	int i;
	for(i=0; i<cEXTRA_ATTR_SLOTS; i++)
	{
		n = fwrite(&attr, sizeof(cvTObjAttr), 1, pOut);
	}
	if (n != 1){
		lrierr( eMEM_WRITE_FAIL,
				"%s .",
				"when writting extra object attributes to binary file"
				);
		exit(1);
	}
	ofs += (sizeOfObjAttrPool + cEXTRA_ATTR_SLOTS) * sizeof(cvTObjAttr);
    padMultiple(8, pOut, &ofs);
#endif

	/* write pDynObjRefPool pool */
	header.dynObjRefOfs       = ofs;
	header.dynObjRefCount     = sizeOfDynObjRefPool;
	n = fwrite(pDynObjRefPool, sizeof(cvTDynObjRef), sizeOfDynObjRefPool, pOut);
	if (n != sizeOfDynObjRefPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .",
				"when writting pDynObjRefPool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfDynObjRefPool * sizeof(cvTDynObjRef);
    padMultiple(8, pOut, &ofs);

	/* write pRoadRefPool pool */
	header.roadRefOfs       = ofs;
	header.roadRefCount     = sizeOfRoadRefPool;
	n = fwrite(pRoadRefPool, sizeof(TRoadPoolIdx), sizeOfRoadRefPool, pOut);
	if (n != sizeOfRoadRefPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .",
				"when writting pRoadRefPool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfRoadRefPool * sizeof(TRoadPoolIdx);
    padMultiple(8, pOut, &ofs);

	/* write pIntrsctnRefPool pool */
	header.intrsctnRefOfs   = ofs;
	header.intrsctnRefCount = sizeOfIntrsctnRefPool;
	n = fwrite(pIntrsctnRefPool, sizeof(TIntrsctnPoolIdx), sizeOfIntrsctnRefPool, pOut);
	if (n != sizeOfIntrsctnRefPool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .",
				"when writting pIntrsctnRefPool to binary file"
				);
		exit(1);
	}
	ofs += sizeOfIntrsctnRefPool * sizeof(TIntrsctnPoolIdx);
    padMultiple(8, pOut, &ofs);

	/* write road piece pool */
	header.roadPieceOfs	= ofs;
	header.roadPieceCount	= sizeOfRoadPiecePool;
	n = fwrite(	pRoadPiecePool, sizeof(cvTRoadPiece), 
				sizeOfRoadPiecePool, pOut);
	if (n != sizeOfRoadPiecePool){
		lrierr( eMEM_WRITE_FAIL,
				"%s .", 
				"when writting road piece pool to binary file" 
				);
		exit(1);
	}
	ofs += sizeOfRoadPiecePool * sizeof(cvTRoadPiece);
	padMultiple(8, pOut, &ofs);

	/* write roadpiece quadtree to memory block*/
	unsigned int sizeOfRdPcQTree = rdPcQTree.GetBinarySize();
	char *pRdPcQTreeBlock = new char[sizeOfRdPcQTree];
	rdPcQTree.Save(pRdPcQTreeBlock);
	header.rdPcQTreeOfs = ofs;
	header.rdPcQTreeSize = sizeOfRdPcQTree;
	n = fwrite( pRdPcQTreeBlock, sizeof(char), sizeOfRdPcQTree, pOut);
	if (n != sizeOfRdPcQTree){
		lrierr( 
			eMEM_WRITE_FAIL,
			"%s .",
			"when writting road piece quadtree to binary file");
		exit(1);
	}
	ofs += sizeOfRdPcQTree * sizeof(char);
	padMultiple(8, pOut, &ofs);

	/* write intersection quadtree to memory block*/
	unsigned int sizeOfIntrsctnQTree = intrsctnQTree.GetBinarySize();
	char *pIntrsctnQTreeBlock = new char[sizeOfIntrsctnQTree];
	intrsctnQTree.Save(pIntrsctnQTreeBlock);
	header.intrsctnQTreeOfs = ofs;
	header.intrsctnQTreeSize = sizeOfIntrsctnQTree;
	n = fwrite( pIntrsctnQTreeBlock, sizeof(char), sizeOfIntrsctnQTree, pOut);
	if (n != sizeOfIntrsctnQTree){
		lrierr( 
			eMEM_WRITE_FAIL,
			"%s .",
			"when writting interection quadtree to binary file"
				);
		exit(1);
	}
	ofs += sizeOfIntrsctnQTree * sizeof(char);
	padMultiple(8, pOut, &ofs);

	/* write static object quadtree to memory block */
	unsigned int sizeOfStaticObjQTree = staticObjQTree.GetBinarySize();
	char *pStaticObjQTreeBlock = new char[sizeOfStaticObjQTree];
	staticObjQTree.Save(pStaticObjQTreeBlock);
	header.staticObjQTreeOfs = ofs;
	header.staticObjQTreeSize = sizeOfStaticObjQTree;
	n = fwrite( 
			pStaticObjQTreeBlock, 
			sizeof(char), 
			sizeOfStaticObjQTree, 
			pOut);
	if (n != sizeOfStaticObjQTree){
		lrierr( 
			eMEM_WRITE_FAIL,
			"%s .",
			"when writting static object quadtree to binary file");
		exit(1);
	}
	ofs += sizeOfStaticObjQTree*sizeof(char);
	padMultiple(8, pOut, &ofs);


	/* write terrain object quadtree to memory block */
	unsigned int sizeOfTrrnObjQTree = trrnObjQTree.GetBinarySize();
	char* pTrrnObjObjQTreeBlock = new char[sizeOfTrrnObjQTree];
	trrnObjQTree.Save(pTrrnObjObjQTreeBlock);
	header.trrnObjQTreeOfs = ofs;
	header.trrnObjQTreeSize = sizeOfTrrnObjQTree;
	n = fwrite( 
			pTrrnObjObjQTreeBlock,
			sizeof(char), 
			sizeOfTrrnObjQTree,
			pOut);
	if (n != sizeOfTrrnObjQTree){
		lrierr( 
			eMEM_WRITE_FAIL,
			"%s .",
			"when writting terrain object quadtree to binary file");
		exit(1);
	}
	ofs += sizeOfTrrnObjQTree*sizeof(char);
	padMultiple(8, pOut, &ofs);


	header.zdown = gHeader.zdown;

	header.dataSize = ofs;
	header.magic[0] = 'L';
	header.magic[1] = 'R';
	header.magic[2] = 'I';
	header.magic[3] = ' ';
	header.magic[4] = '1';
	header.magic[5] = '.';
	header.magic[6] = '0';
	header.magic[7] = '\0';

	header.majorVersionNum      = gGetMajorCvedVersionNum();
    header.minorVersionNum      = gGetMinorCvedVersionNum();
    header.minorExt1VersionNum  = gGetMinorExt1CvedVersionNum();
    header.minorExt2VersionNum  = gGetMinorExt2CvedVersionNum();
    header.minorExt3VersionNum  = gGetMinorExt3CvedVersionNum();

	
	rewind(pOut);
	n = fwrite(&header, sizeof(header), 1, pOut);
	fclose(pOut);

	if ( gDebugDump ) 
		DumpPools(&header);
}


////////////////////////////////////////////////////////////////////////////
//
// Description: break up a road in pieces of a bounded size and build up
// the road piece pool
//
// Remarks:
// This function breaks up a road into road pieces so that the
// bounding box of each road piece is a bit larger than the specified
// limit.
//
// The bounding box of a road piece includes the edges of a road (not
// only the centerline).  The edge of a road is determined based
// on the physical width of the road so that if a point is not
// within the bounding box of a road piece, then it cannot overlap
// the road segment represented by the corresponding road piece.
//
// Breaking up a road in road pieces is useful for minimizing search
// times involving roads.
//
// The function will attempt to separate the road into as many
// road pieces as necessary to ensure that each of the pieces
// will extend beyond one control point bigger than the specified limits.
// However, it is possible to end up with smaller piece for the last
// one. it as well build the roadpiece pool at the same time in order to 
// be stored in the memory block
//
//
// Arguments:
// rdId - roadId of the road on which the roadpiece is
// width - the maximum width (x axis) of each road piece
// height - the maximum height (y axis) of each road piece
// qtree - a reference to the quadtree used for road pieces
//
static void 
SetRoadPiecePool(
				TRoadPoolIdx	rdId,
				float			width,
				float			height,
				CQuadTree&		qtree)
{
	TLongCntrlPntPoolIdx		cntrlPntIdx	= pRoadPool[rdId].cntrlPntIdx;
	int							numCntrlPnt	= pRoadPool[rdId].numCntrlPnt;
	cvTCntrlPnt*				pCntrlPnt	= &pCntrlPntPool[cntrlPntIdx];

	cvTRoadPiece				boundBox;
	InitRoadPiece(&boundBox, rdId, 0, 0, 0, 0.0, 0.0, width, height);

	float maxX	= pCntrlPnt->location.x;
	float maxY	= pCntrlPnt->location.y;
	float minX	= pCntrlPnt->location.x;
	float minY	= pCntrlPnt->location.y;

   
	bool				insideBoundingBox = false;
	bool				pushLastRoadPiece = true;
	CQuadTree::CItem	item;
	int					i;
	int					first = 0;
	for (i=0; i<numCntrlPnt; i++)  {
		FindBoundaryOfRoadPiece(&maxX, &maxY, &minX, &minY, cntrlPntIdx + i);
		insideBoundingBox = HitTest(boundBox, maxX-minX, maxY-minY);
		if ( !insideBoundingBox){
			AllocRoadPiece();
			InitRoadPiece(
				&pRoadPiecePool[sizeOfRoadPiecePool-1],
				rdId,
				sizeOfRoadPiecePool-1,
				first,
				i,
				minX,
				minY,
				maxX,
				maxY);
			cvTRoadPiece *pRdPc = pRoadPiecePool+(sizeOfRoadPiecePool-1);
			InitItem(
				item, 
				pRdPc->myId, 
				pRdPc->x1, 
				pRdPc->y1, 
				pRdPc->x2, 
				pRdPc->y2);

			qtree.Insert(item);

			maxX = ( pCntrlPnt+i )->location.x;
			minX = ( pCntrlPnt+i )->location.x;
			maxY = ( pCntrlPnt+i )->location.y;
			minY = ( pCntrlPnt+i )->location.y;
			FindBoundaryOfRoadPiece(&maxX, &maxY, &minX, &minY, cntrlPntIdx+i);
			first = i;
			if (i == (numCntrlPnt-1))
				pushLastRoadPiece = false;
		}
	}
	if (pushLastRoadPiece){
		AllocRoadPiece();
		InitRoadPiece(
				&pRoadPiecePool[sizeOfRoadPiecePool-1],
				rdId,
				sizeOfRoadPiecePool-1,
				first,
				numCntrlPnt-1,
				minX,
				minY,
				maxX,
				maxY);
		cvTRoadPiece *pRdPc = pRoadPiecePool+(sizeOfRoadPiecePool-1);
		InitItem(
				item, 
				pRdPc->myId, 
				pRdPc->x1, 
				pRdPc->y1, 
				pRdPc->x2, 
				pRdPc->y2);
		qtree.Insert(item);
	}
}

////////////////////////////////////////////////////////////////////////////
//
// Description: allocate memory for road piece pool
//  
// Remarks: 
// The function allocate memory for road piece pool                
//
//
static void AllocRoadPiece(void)
{
	sizeOfRoadPiecePool++;
	pRoadPiecePool = (cvTRoadPiece*)realloc(
								pRoadPiecePool,
								sizeOfRoadPiecePool*sizeof(cvTRoadPiece));
	if (!pRoadPiecePool) {
		lrierr(eROAD_PIECE_ALLOC_FAIL, "");
		exit(1);
	}
	else
		memset(pRoadPiecePool+sizeOfRoadPiecePool-1, 0, sizeof(cvTRoadPiece));
}

////////////////////////////////////////////////////////////////////////////
//
// Description: assign a quadtree item corrosponding information  
//
// Remarks: 
// The function assign the fields in the CQuadTree::CItem with
// the values of the values corrosponding objects which could be roadpice or
// intersection, etc.
//
// Arguments:
// item - a reference to an object of type CQuadTree::CItem &item, which will
//        be provided with the values from road piece
// it	- id of the object contained in the item
// x1 y1 x2 y2  - cordinates of lower left corner and upper right corner
//
static void InitItem(
					CQuadTree::CItem&	item, 
					int					id,
					float				x1,
					float				y1,
					float				x2,
					float				y2)
{
	item.id = id;
	item.x1 = x1;
	item.y1 = y1;
	item.x2 = x2;
	item.y2 = y2;
}

////////////////////////////////////////////////////////////////////////////
//
// Description: assign an instance of roadpiece structure
//
// Remarks: 
// The function initialize an instance of roadpiece structure
//
// Arguments:
// pRP - a pointer to a roadpiece to be initialized 
// rdid - id of the road on which the road piece is
// myId - id of the roadpiece
// first - the index to the first control point of the road piece, which is
//         with respect to the road
// last - the index to the last control point of the road piece, which is
//        with respect to the road
// x1, y1 - the coordinate of the lower left 
// x2, y2 - the coordinate of the upper right
//
static void InitRoadPiece(
						cvTRoadPiece*		pRP,
						TRoadPoolIdx		rdIdx,
						TRoadPiecePoolIdx	myId,
						int					first,
						int					last,
						float				x1,
						float				y1,
						float				x2,
						float				y2)
{
	pRP->roadId	= rdIdx;
	pRP->myId	= myId;
	pRP->first	= first;
	pRP->last	= last;
	pRP->x1		= x1;
	pRP->y1		= y1;
	pRP->x2		= x2;
	pRP->y2		= y2;
}

////////////////////////////////////////////////////////////////////////////
//
// Description: to test if the roadpiece is still contained or beyond the
// bounding box of size x by y
//
// Remarks:
// This function tests if the passed roadpiece is contained or beyond the 
// bounding box of size x by y
// 
// Arguments:
// rp - a roadpiece which is tested if it is smaller or larger than the size
//      of x by y
// x - size of the bounding box in x direction 
// y - size of the bounding box in y direction
//
// Return value:
// The function returns true if the roadpice is still smaller than the size
// of x by y
//
static bool HitTest(cvTRoadPiece rp, float x, float y)
{
	if ( x >= rp.x1 && x <= rp.x2 && y >= rp.y1 && y <= rp.y2 )
		return true;
	else
		return false;
}


/*--------------------------------------------------------------------------* 
 *                                                                          
 * Description : find the boundary of the part of the road                  
 *                                                                          
 * Remarks:                                                                 
 * This function find the boundry of the the current set of control points. 
 * The way it works is to find a bounding box which exactly cover the       
 * roads. Given the set of control points, first acqure the width and       
 * the right vector of each control point, and then put together these      
 * information with the location of the control point to compute the        
 * boundary of the road piece.                                              
 *                                                                          
 * Arguments:                                                               
 * pMaxX                                                                    
 * pMaxY    pointers to floats of upper right of boundary                   
 * pMinX                                                                    
 * pMinY    pointers to floats of lower left of boundary                    
 * cntrlIdx index to control points in cntrlPntPool                         
 *                                                                          
 ****************************************************************************/

static void FindBoundaryOfRoadPiece(
					float					*pMaxX,
					float					*pMaxY,
					float					*pMinX,
					float					*pMinY,
					TLongCntrlPntPoolIdx	cntrlIdx)
{
	cvTCntrlPnt *pPnt = &pCntrlPntPool[cntrlIdx];
	float x1 = pPnt->location.x + pPnt->physicalWidth/2*pPnt->rightVecCubic.i;
	float x2 = pPnt->location.x - pPnt->physicalWidth/2*pPnt->rightVecCubic.i;
	float y1 = pPnt->location.y + pPnt->physicalWidth/2*pPnt->rightVecCubic.j;
	float y2 = pPnt->location.y - pPnt->physicalWidth/2*pPnt->rightVecCubic.j;

	if (x1 > *pMaxX)
		*pMaxX = x1;
	else if (x1 < *pMinX)
		*pMinX = x1;

	if (x2 > *pMaxX)
		*pMaxX = x2;
	else if (x2 < *pMinX)
		*pMinX = x2;

	if (y1 > *pMaxY)
		*pMaxY = y1;
	else if (y1 < *pMinY)
		*pMinY = y1;

	if (y2 > *pMaxY)
		*pMaxY = y2;
	else if (y2 < *pMinY)
		*pMinY = y2;
}


/*--------------------------------------------------------------------------* 
 *                                                                          
 * Description : find the boundary of the intersection                  
 *                                                                          
 * Remarks:                                                                 
 * This function find the boundry of the the current set of control points. 
 * The way it works is to find a bounding box which exactly cover the       
 * roads. Given the set of control points, first acqure the width and       
 * the right vector of each control point, and then put together these      
 * information with the location of the control point to compute the        
 * boundary of the road piece.                                              
 *                                                                          
 * Arguments:                                                               
 *	pEntry: pointer pointing to the specified interesction
 *	llPt:	a reference to a objefect of type CPoint2D which will be set to 
 *			the point of lower left of the intersection
 *	urPt:	a reference to a objefect of type CPoint2D which will be set to 
 *			the point of upper right of the intersection
 *                                                                          
 *--------------------------------------------------------------------------*/
static 
void FindBoundaryOfIntrsctn(
							cvTIntrsctn*	pEntry,
							CPoint2D&		llPt,
							CPoint2D&		urPt)
{
	int start = pEntry->borderSegIdx;
	llPt.m_x = pBorderSegPool[start].x;
	llPt.m_y = pBorderSegPool[start].y;
	urPt.m_x = pBorderSegPool[start].x;
	urPt.m_y = pBorderSegPool[start].y;

	int i;
	for (i=1; i<pEntry->numBorderSeg; i++){
		if (pBorderSegPool[start+i].x < llPt.m_x)
				llPt.m_x = pBorderSegPool[start+i].x;
		if (pBorderSegPool[start+i].x > urPt.m_x)
				urPt.m_x = pBorderSegPool[start+i].x;
		if (pBorderSegPool[start+i].y < llPt.m_y)
				llPt.m_y = pBorderSegPool[start+i].y;
		if (pBorderSegPool[start+i].y > urPt.m_y)
				urPt.m_y = pBorderSegPool[start+i].y;
	}
}

void AllocateDynObjRefPool(void)
{
	sizeOfDynObjRefPool = cCV_NUM_DYN_OBJ_REFS;
	pDynObjRefPool = (cvTDynObjRef*)realloc(
									pDynObjRefPool,
									sizeof(cvTDynObjRef) * sizeOfDynObjRefPool);
	if (!pDynObjRefPool){
		lrierr(eDYN_OBJ_REF_POOL_ALLOC_FAIL, "");
		exit(1);
	}else{
		memset(pDynObjRefPool, 0, sizeof(cvTDynObjRef) * sizeOfDynObjRefPool);
	}
}

void AllocateRoadRefPool(void)
{

	sizeOfRoadRefPool = sizeOfRoadPool;
	pRoadRefPool = (cvTRoadRef*)realloc(
								pRoadRefPool,
								sizeof(cvTRoadRef) * sizeOfRoadRefPool);
	if (!pRoadRefPool){
		lrierr(eROAD_REF_POOL_ALLOC_FAIL, "");
		exit(1);
	}else{
		memset(pRoadRefPool, 0, sizeof(cvTRoadRef) * sizeOfRoadRefPool);
	}
}

void AllocateIntrsctnRefPool(void)
{

	sizeOfIntrsctnRefPool = sizeOfIntrsctnPool;
	pIntrsctnRefPool = (cvTIntrsctnRef*)realloc(
										pIntrsctnRefPool,
										sizeof(cvTIntrsctnRef)*sizeOfIntrsctnRefPool);
	if (!pIntrsctnRefPool){
		lrierr(eINTRSCTN_REF_POOL_ALLOC_FAIL, "");
		exit(1);
	}else{
		memset(pIntrsctnRefPool, 0, sizeof(cvTIntrsctnRef)*sizeOfIntrsctnRefPool);
	}
}
			
/*--------------------------------------------------------------------------* 
 *                                                                          
 * Description : set up the CrdrMrgDstPool                  
 *                                                                          
 * Remarks:                                                                 
 *  This function find all the merge point and exit point of every corridor
 *  with other corridors on the same intersection, and store the distances
 *                                                                          
 * Arguments:                                                               
 *	pI:     pointer pointing to the specified interesction
 *	size:	size of the pool 
 *	ppPool:	pointer to the pool 
 *                                                                          
 *--------------------------------------------------------------------------*/
static void SetCrdrMrgDstPool(
			cvTIntrsctn* pI, 
			int& size, 
			cvTCrdrMrgDst** ppPool) 
{
	float CRDR_EPS = 1e-3;
	float CRDR_TOLERANCE = 1;

	int idx0 = 0;
	if (!pI) 
		return;
	// for every corridor
	for (; idx0 < pI->numOfCrdrs; idx0++){

		cvTCrdr* pC0 = &pCrdrPool[pI->crdrIdx + idx0];
		pC0->mrgDstIdx = size;
		size += pI->numOfCrdrs;
		*ppPool = (cvTCrdrMrgDst*)realloc(*ppPool, sizeof(cvTCrdrMrgDst)*size);
		if (!*ppPool){
			lrierr(eCRDR_MRG_DST_ALLOC_FAIL, "");
			exit(1);
		}

		// with every other corridor coming from different lane
		int idx1 = 0;
		for (; idx1 < pI->numOfCrdrs; idx1++){
			
			cvTCrdrMrgDst* pEntry = *ppPool + size - pI->numOfCrdrs + idx1;
			pEntry->firstDist = -1;		
			pEntry->lastDist = -1;		
			const cvTCrdr* pC1 = &pCrdrPool[pI->crdrIdx + idx1];
			if (pC0->srcLnIdx == pC1->srcLnIdx){
				continue;
			}

			// a corridor will have two side : left and right
			// for every side, it will intersect with every other
			// corridor up to two points
			int right = 0;
			int left = 0;
			float lDists[2];
			float rDists[2];
	
			int idx2 = 0;
			int numPnts0 = pC0->numCntrlPnt;
			for(; idx2 < (numPnts0-1); idx2++){
				if ((right == 2) && (left == 2)){
					break;
				}
				
				cvTCrdrCntrlPnt* pCpt =
					&pCrdrCntrlPntPool[pC0->cntrlPntIdx+idx2];
				cvTCrdrCntrlPnt* pNpt=pCpt + 1;
				CPoint2D cpt0(pCpt->location);
				CPoint2D npt0(pNpt->location);
				npt0.m_x = npt0.m_x+(npt0.m_x-cpt0.m_x)*(CRDR_EPS); 
				npt0.m_y = npt0.m_y+(npt0.m_y-cpt0.m_y)*(CRDR_EPS); 
				CVector2D crv0(pCpt->rightVecLinear);
				CVector2D nrv0(pNpt->rightVecLinear);
				crv0.m_i *= (0.5*(pCpt->width-CRDR_TOLERANCE));
				crv0.m_j *= (0.5*(pCpt->width-CRDR_TOLERANCE));
				nrv0.m_i *= (0.5*(pNpt->width-CRDR_TOLERANCE));
				nrv0.m_j *= (0.5*(pNpt->width-CRDR_TOLERANCE));
// right
// contruct right side of the line segment from corridor control point
// and right vector, and intersect it with the other line segments from
// other corridor
				int idx3 = 0;
				int numPnts1 = pC1->numCntrlPnt;
				for(; idx3 < (numPnts1-1); idx3++){

					if (right ==2){
						break;
					} 
					CPoint2D rcpt(cpt0.m_x + crv0.m_i, cpt0.m_y + crv0.m_j);
					CPoint2D rnpt(npt0.m_x + nrv0.m_i, npt0.m_y + nrv0.m_j);
					CLineSeg2D rLine0(rcpt, rnpt);

					cvTCrdrCntrlPnt* pCpt1 = 
						&pCrdrCntrlPntPool[pC1->cntrlPntIdx+idx3]; 
					cvTCrdrCntrlPnt* pNpt1 = pCpt1 + 1; 
					CPoint2D cpt1(pCpt1->location); 
					CPoint2D npt1(pNpt1->location); 
					npt1.m_x = npt1.m_x+(npt1.m_x-cpt1.m_x)*(CRDR_EPS); 
					npt1.m_y = npt1.m_y+(npt1.m_y-cpt1.m_y)*(CRDR_EPS); 
					CVector2D crv1(pCpt1->rightVecLinear); 
					CVector2D nrv1(pNpt1->rightVecLinear); 
					crv1.m_i *= (0.5*(pCpt1->width-CRDR_TOLERANCE));
					crv1.m_j *= (0.5*(pCpt1->width-CRDR_TOLERANCE));
					nrv1.m_i *= (0.5*(pNpt1->width-CRDR_TOLERANCE));
					nrv1.m_j *= (0.5*(pNpt1->width-CRDR_TOLERANCE));
					CPoint2D lpt0(cpt1.m_x - crv1.m_i, cpt1.m_y - crv1.m_j); 
					CPoint2D lpt1(npt1.m_x - nrv1.m_i, npt1.m_y - nrv1.m_j); 
					CLineSeg2D lLine1(lpt0, lpt1); 
					CPoint2D rpt0(cpt1.m_x + crv1.m_i, cpt1.m_y + crv1.m_j); 
					CPoint2D rpt1(npt1.m_x + nrv1.m_i, npt1.m_y + nrv1.m_j); 
					CLineSeg2D rLine1(rpt0, rpt1);

					CPoint2D inPt;
					if(rLine0.GetIntersection(lLine1, inPt)){
						float dx = inPt.m_x - rcpt.m_x;
						float dy = inPt.m_y - rcpt.m_y;
						float dist = sqrt(dx*dx + dy*dy);
						rDists[right++] = pCpt->distance + dist;
					}
					if (right != 2){
						if(rLine0.GetIntersection(rLine1, inPt)){
							float dx = inPt.m_x - rcpt.m_x;
							float dy = inPt.m_y - rcpt.m_y;
							float dist = sqrt(dx*dx + dy*dy);
							rDists[right++] = pCpt->distance + dist;
						}
					}
				}
// left
// contruct right side of the line segment from corridor control point
// and right vector, and intersect it with the other line segments from
// other corridor
				for(idx3 = 0; idx3 < (numPnts1-1); idx3++){
					if (left == 2){
						break;
					} 
					CPoint2D lcpt(cpt0.m_x - crv0.m_i, cpt0.m_y - crv0.m_j);
					CPoint2D lnpt(npt0.m_x - nrv0.m_i, npt0.m_y - nrv0.m_j);
					CLineSeg2D lLine0(lcpt, lnpt);

					cvTCrdrCntrlPnt* pCpt1 = 
						&pCrdrCntrlPntPool[pC1->cntrlPntIdx+idx3]; 
					cvTCrdrCntrlPnt* pNpt1 = pCpt1 + 1; 
					CPoint2D cpt1(pCpt1->location); 
					CPoint2D npt1(pNpt1->location); 
					npt1.m_x = npt1.m_x+(npt1.m_x-cpt1.m_x)*(CRDR_EPS); 
					npt1.m_y = npt1.m_y+(npt1.m_y-cpt1.m_y)*(CRDR_EPS); 
					CVector2D crv1(pCpt1->rightVecLinear); 
					CVector2D nrv1(pNpt1->rightVecLinear); 
					crv1.m_i *= (0.5*(pCpt1->width-CRDR_TOLERANCE));
					crv1.m_j *= (0.5*(pCpt1->width-CRDR_TOLERANCE));
					nrv1.m_i *= (0.5*(pNpt1->width-CRDR_TOLERANCE));
					nrv1.m_j *= (0.5*(pNpt1->width-CRDR_TOLERANCE));
					CPoint2D lpt0(cpt1.m_x - crv1.m_i, cpt1.m_y - crv1.m_j); 
					CPoint2D lpt1(npt1.m_x - nrv1.m_i, npt1.m_y - nrv1.m_j); 
					CLineSeg2D lLine1(lpt0, lpt1); 
					CPoint2D rpt0(cpt1.m_x + crv1.m_i, cpt1.m_y + crv1.m_j); 
					CPoint2D rpt1(npt1.m_x + nrv1.m_i, npt1.m_y + nrv1.m_j); 
					CLineSeg2D rLine1(rpt0, rpt1);

					CPoint2D inPt;
					if(lLine0.GetIntersection(lLine1, inPt)){
						float dx = inPt.m_x - lcpt.m_x;
						float dy = inPt.m_y - lcpt.m_y;
						float dist = sqrt(dx*dx + dy*dy);
						lDists[left++] = pCpt->distance + dist;
					}
					if (left != 2){
						if(lLine0.GetIntersection(rLine1, inPt)){
							float dx = inPt.m_x - lcpt.m_x;
							float dy = inPt.m_y - lcpt.m_y;
							float dist = sqrt(dx*dx + dy*dy);
							lDists[left++] = pCpt->distance + dist;
						}
					}
				}
			}
//
// when left side and right side both intersect twice with
// a corridor, two corridors intersect
//
			if((left == 2) && (right == 2)){
				vector<float> floatVec;
				floatVec.push_back(lDists[0]);							
				floatVec.push_back(lDists[1]);							
				floatVec.push_back(rDists[0]);							
				floatVec.push_back(rDists[1]);
				pEntry->firstDist = Min(floatVec);							
				pEntry->lastDist = Max(floatVec);
//							
// when left side intersects twice and right once
// two corridors merge
//
			} else if ((left == 2) && (right == 1)){
				pEntry->firstDist = 
					(lDists[0] > lDists[1])? lDists[1] : lDists[0];
//							
// when right side intersects twice and left once
// two corridors merge
//
			} else if ((left == 1) && (right == 2)){
				pEntry->firstDist = 
					(rDists[0] > rDists[1])? rDists[1] : rDists[0];
			}
		}
	}
}

float Min(vector<float> vec)
{
	vector<float>::const_iterator itr = vec.begin();
	float min = *itr;
	for(; itr != vec.end(); itr++){
		if (*itr < min){
			min = *itr;
		}
	}
	return min;
}

float Max(vector<float> vec)
{
	vector<float>::const_iterator itr = vec.begin();
	float max = *itr;
	for(; itr != vec.end(); itr++){
		if (*itr > max){
			max = *itr;
		}
	}
	return max;
}
