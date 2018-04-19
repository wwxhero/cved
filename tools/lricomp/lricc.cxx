
/***************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center The University of Iowa
 * and The University of Iowa.  All rights reserved.
 *
 * $Id: lricc.cxx,v 1.6 2014/09/15 18:33:10 IOWA\vhorosewski Exp $
 *
 *
 *
 */
#ifdef _WIN32
#include <ostream>
#include <iostream>
#include <sstream>
#include <iomanip>

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <DbgHelp.h>

// based on dbghelp.h
typedef BOOL (WINAPI *MINIDUMPWRITEDUMP)(HANDLE hProcess, DWORD dwPid, HANDLE hFile, MINIDUMP_TYPE DumpType,
									CONST PMINIDUMP_EXCEPTION_INFORMATION ExceptionParam,
									CONST PMINIDUMP_USER_STREAM_INFORMATION UserStreamParam,
									CONST PMINIDUMP_CALLBACK_INFORMATION CallbackParam);
#elif __sgi
#include <typeinfo>
#include <iostream.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cvedversionnum.h>
#include <assert.h>
#include "parser.h"

#define LR_FLT_MAX         3.402823466e+38F        /* max value */
#define LR_DBL_MAX         1.7976931348623158e+308 /* max value */


typedef struct {
	TPoint3D	location;
	TVector3D	rightVec;
	float		width;
	float		distance;
} TLrCntrlPnt;


void WriteOutputFile(
		const char*,
		const vector<cvTElevPost>&,
		const vector<cvTElevMap>&);

void GenerateCode(
		TRoads*, 
		TIntersection*, 
		TElevMap*,
		vector<cvTElevPost>&, 
		vector<cvTElevMap>&);

void SemanticCheck(TRoads*, TIntersection*, TElevMap*);


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
		cvTObjRef*&			pObjRef,        // The pool with the references
		int*				pObjRefSize,    // The size off the reference pool
		bool				newObjRefPool);



void DisplayObjDataOfRoad(
        char*				roadName,
        char*				pCharPool,
        cvTRoad*			pRoadPool,
		int					sizeOfRoadPool,
        cvTCntrlPnt*		pCntrlPntPool,
        cvTRepObj*			pRepObjPool,
        cvTObjRef*			pObjRef);

void DisplayIntrsctn(
		char*				pCharPool,
		cvTIntrsctn*		pIntrsctnPool,
		int					sizeOfIntrsctnPool,
		cvTObjRef*			pObjRefPool);


void DumpObjPool(cvTObj* pObjPool, int sizeOfObjPool);
void DumpRepObjPool(cvTRepObj* pRepObjPool, int sizeOfRepObjPool);
void DumpObjRefPool(cvTObjRef* pObjRefPool, int sizeOfObjRefPool);


/* parser data structures */
extern "C" {

extern THeader				gHeader;
extern TRoads*				gpRoadList;
extern TIntersection*		gpInterList;
extern TElevMap*			gpElevMaps;
extern TObject*				gpObjects;
extern int					gObjectCount;
extern int					gNoMinusInNames;

//extern cvTObjRef*			pObjRefPool;
//extern int					sizeOfObjRefPool;
extern char					yytext[];


int  yyparse(void);

}

extern CQuadTree		rdPcQTree;
extern CQuadTree		intrsctnQTree;


/****************************************************************************
 *
 * Description: dumps parser data structures
 *
 * Remarks:
 * This function prints all the data structures constructed by the
 * parser but before any conversions have been done for writing to
 * the memory block.  The function is used primarily for debugging
 * the parser.
 *
 */
void dumpInfo(void)
{
	TRoads     *pRd;
	TIntersection *pInt;
	int         i;
	TAttribute *pAtr;
	TRepObj    *pRepObj;
	TLatCurve  *pLat;

	printf("============== Header Information ====================\n");
	printf("Z up is (0, 0, %d)\n", gHeader.zdown ? -1 : 1 );
	printf("Sol check sum is %u\n", gHeader.solCheckSum);
	printf("Header comment follows in next line(s)\n%s\n--- End comment ---\n",
				gHeader.pComment);

	printf("\n");

	for ( pRd = gpRoadList; pRd; pRd = pRd->pNext ) {
		printf(" Road: '%s' from '%s' to '%s'\n",
				pRd->roadName, pRd->intersection1, 
				pRd->intersection2);

		for ( pAtr = pRd->pAttribute; pAtr; pAtr = pAtr->pNext) {
			printf("\tAttr %d: %d %.1f %.1f %.1f %.1f\n", pAtr->inum, 
					pAtr->laneMask, pAtr->attrNum1, pAtr->attrNum2,
					pAtr->from, pAtr->to);
		}

		for ( pRepObj = pRd->pRepObjs; pRepObj; pRepObj=pRepObj->pNext) {
			printf("\tRep obj: %s, %.1f, %.1f, %.1f, %.1f %s\n",
				pRepObj->name, pRepObj->latdist, pRepObj->start,
				pRepObj->period, pRepObj->end,
				pRepObj->alligned ? "alligned" : "vertical");
		}

		printf("\t%d lanes (w/d): ", pRd->lanelist.n_lanes);
		for (i=0; i<pRd->lanelist.n_lanes; i++) {
			printf("%.1f/%c ", pRd->lanelist.lanes[i].width,
						pRd->lanelist.lanes[i].direction);
		}
		printf("\n\tControl Points:\n");
		for (i=0; i<pRd->longitCurve.n_cpoints; i++) {
			TControlPoint *pConP = &pRd->longitCurve.pCpointList[i];
			TCpointInfo   *pI;
			printf("\t  %.2f %.2f %.2f   %.2f %.2f %.2f ",
				pConP->x, pConP->y, pConP->z, pConP->i, pConP->j, pConP->k);

			if ( pConP->cpointName[0] != '\0' )
				printf("Name='%s' ", pConP->cpointName);

			for (pI = pConP->pCpointInfo; pI; pI = pI->pNext ) {

				if ( pI->curveName[0] )
					printf("Crv='%s' ", pI->curveName);
				
				if ( pI->cpointWidth.n_lanes > 0 ) {
					printf("LWid ");
					for (i=0; i<pI->cpointWidth.n_lanes; i++) {
						printf("%.2f ", pI->cpointWidth.lanewidth[i]);
					}
				}

				if ( pI->pCpointAttribute ) {
					TAttribute *pAt = pI->pCpointAttribute;

					printf("Attrs: ");
					for ( ; pAt; pAt = pAt->pNext) {
						printf("(%d %d %.1f %.1f) ", pAt->inum, pAt->laneMask,
								pAt->attrNum1, pAt->attrNum2);
					}
				}
			}
			printf("\n");
		}
	}

	printf("\n--> Intersections: \n");
	for (pInt=gpInterList; pInt; pInt = pInt->pNext) {
		TRoadName *pR;
		TCrdr     *pCr;

		printf(" Intersection '%s', ", pInt->name);
		for (pR=pInt->pRoadNames; pR; pR=pR->pNextRoadName) {
			printf("%s ", pR->roadName);
		}
		printf("\n");

		if ( pInt->elevInfo.elevMapName[0] ) {
			printf("\tElevation: provided by map '%s' at origin %.1f, %.1f\n",
				pInt->elevInfo.elevMapName,
				pInt->elevInfo.xorig,
				pInt->elevInfo.yorig);
		}
		else {
			printf("\tElevation: fixed @ %.1f\n", pInt->elevInfo.height);
		}

		if ( pInt->Border.nPts > 0 ) {
			int bp;

			printf("\tBorder: ");
			for (bp = 0; bp<pInt->Border.nPts; bp++) {
				printf("(%.1f, %.1f, %c) ", 
						pInt->Border.list[bp].borderX,
						pInt->Border.list[bp].borderY,
						pInt->Border.list[bp].lflag);
			}
			printf("\n");
		}
		else {
			printf("\tNo border.\n");
		}

		for (pAtr = pInt->pAttrs; pAtr; pAtr = pAtr->pNext) {
			printf("\tAttribute: %d  %f  %f\n",
					pAtr->inum, pAtr->attrNum1, pAtr->attrNum2);
		}

		for (pCr=pInt->pCrdr; pCr; pCr = pCr->pNext) {
			THoldOfs  *pHo;

			printf("\tCorridor %s:%d->%s:%d and style \n",
				pCr->roadName1, pCr->crdrLane1,
				pCr->roadName2, pCr->crdrLane2);
/*
			if(pCr->CrdrCurveList.n_crdrpts>1){
				printf(" get here \n");
				if(pCr->CrdrCurveList.pCrdrCurve[0].pCrdrLineInfo->lstyle1[0]!='\0'){
				printf(" tell me ");	
				printf(" %s\n", 
					pCr->CrdrCurveList.pCrdrCurve[0].pCrdrLineInfo->lstyle1);
	}
				else
				printf("  why \n");
			}
*/

			for (pHo=pCr->pHoldOfs; pHo; pHo = pHo->pNextHoldOfs) {
				printf("\t  HoldOfs: %.1f %.1f %s",
					pHo->distance, pHo->clrnc, pHo->crdrReason);

				if ( pHo->info[0].isLine ) {
					printf("  line thich = %.2f, line ang = %.2f",
						pHo->info[0].thick, pHo->info[0].angle);
				}
				if ( pHo->info[0].isSign ) {
					printf("  sign name= %s",
						pHo->info[0].holdSign);
				}

				if ( pHo->info[1].isLine ) {
					printf("  line thich = %.2f, line ang = %.2f",
						pHo->info[1].thick, pHo->info[0].angle);
				}

				if ( pHo->info[1].isSign ) {
					printf("  sign name= %s",
						pHo->info[1].holdSign);
				}
				printf("\n");
			}
			for (pAtr = pCr->pAttrs; pAtr; pAtr = pAtr->pNext ) {
				printf("\t  Attribute: %d  %f %f %f %f\n",
					pAtr->inum, pAtr->attrNum1, pAtr->attrNum2,
					pAtr->from, pAtr->to);
				
			}
		}

		printf("\n");
	}


	printf("\n--> Objects: \n");
	for (i = 0; i < gObjectCount; i++) {
		printf("Obj: %s, sol=%s, type=%s, %.1f %.1f %.1f  %.2f %s\n",
				gpObjects[i].name, gpObjects[i].sol,
				gpObjects[i].type,
				gpObjects[i].x, gpObjects[i].y, gpObjects[i].z,
				gpObjects[i].yaw, 
				gpObjects[i].plant ? "planted" : "");
	}

	printf("\n--> Attribute dictionary: \n");
	for (i=0; i < gAttrDict.nEntries; i++) {
		printf("%-25s %d\n", gAttrDict.pList[i].pName,
						gAttrDict.pList[i].id);
	}

	printf("\n--> Lateral curves: \n");
	for (pLat = gpLatCurves; pLat; pLat = pLat->pNext) {
		TCurveDef *pCp;

		printf("%-25s (%.1f wide) : ", pLat->curveName, pLat->curveWidth);
		for (pCp=pLat->pCurveDef; pCp; pCp = pCp->pNext) {
			printf("(%.1f,%.1f,%d) (%.1f,%.1f,%d) ",
				pCp->latCpoint1.height,
				pCp->latCpoint1.offset,
				pCp->latCpoint1.material,
				pCp->latCpoint2.height,
				pCp->latCpoint2.offset,
				pCp->latCpoint2.material);
		}
		printf("\n");
	}
}


/****************************************************************************
 *
 * Description: usage
 *
 * Remarks:
 * Provides usage information to the user
 *
 */
void usage(const char *pPgmName)
{
	fprintf(stderr, "%s (V%d.%d): incorrect usage.\n", pPgmName,
		gGetMajorCvedVersionNum(), gGetMinorCvedVersionNum());
	fprintf(stderr, "Usage: %s [-ct num] [-gap num] [-hs num] [-nc] [-nodash c] [-v1 Ver] [-v2 Ver] [-debug] [-tlex] in out \n", pPgmName);
	fprintf(stderr, 
"\t-nc     perform no corrections on spline data\n"
"\t-ct     corridor tolerance.  Corridors can be [num] units away from\n\t        their source or destination lanes without generating an error\n"
"\t-hs     hermite spline scale factor\n"
"\t-nodash c   replace '-' chars in road/intersection names with c\n"
"\t-v1     override major version number written to output file\n"
"\t-v2     override minor version number written to output file\n"
"\t-v3,v4,v5 override externsion version numbers written to output file\n"
"\t-gap    how much gap is allowed between a corridor and the lanes it connects\n"
"\t-debug  dump internal data structures; for debugging only\n"
"\t-tlex   test lexical analyzer; for debugging only\n"
"\tin      name of input file (if extension is missing .lri is assumed)\n"
"\tout     name of output file (if extension is missing .bli is added)\n");
	exit(0);
}

extern "C" {
	extern FILE *yyin;
	extern FILE *yyout;
	extern cvTObj     *pObjPool;
//	extern cvTObjAttr *pObjAttrPool;
	extern int        sizeOfObjPool;
//	extern int        sizeOfObjAttrPool;
}


/////////////////////////////////////////////////////////////////////////////
//
// Small utility function to replace the '-' character in a name with 'm'
//
//
static void ReplaceMinus(char *p)
{
	if ( p ) {
		while ( *p ) {
			if ( *p == '-' ) *p = gNoMinusInNames;
			p++;
		}
	}
}


static void 
FixNames(TRoads *pRoads,
	TIntersection *pInter)
{
	TRoads*          pRd; 
	TIntersection*   pI;

	for ( pRd = pRoads; pRd; pRd = pRd->pNext ) {
		ReplaceMinus(pRd->roadName);
		ReplaceMinus(pRd->intersection1);
		ReplaceMinus(pRd->intersection2);
	}

	for ( pI = pInter; pI; pI = pI->pNext ) {
		TRoadName* pN;

		for (pN=pI->pRoadNames; pN; pN = pN->pNextRoadName) {
			ReplaceMinus(pN->roadName);
		}
		ReplaceMinus(pI->name);

		TCrdr     *pCr;
		for (pCr=pI->pCrdr; pCr; pCr = pCr->pNext) {
			ReplaceMinus(pCr->roadName1);
			ReplaceMinus(pCr->roadName2);
		}
	}
}


static void
ComputeCorridorDirections(
	cvTIntrsctn*      pIntrsctnPool,
	int			      sizeOfIntrsctnPool,
	cvTCrdrCntrlPnt*  pCrdrCntrlPntPool,
	int				  sizeOfCrdrCntrlPntPool)
{
	int   intr;

	// for each intersection
	//   for each corridor c1
	//		compute its direction
	//		for each corridor c2
	//			compute relative direction between c1 & c2
	for (intr = 1; intr < sizeOfIntrsctnPool; intr++) {
		cvTIntrsctn*   pInt = pIntrsctnPool+intr;
		cvTCrdr*       pC1;
		cvTCrdr*       pC2;

		unsigned int  c1, c2;		// zero based ordinals
		int  cor1, cor2;	// indeces within pool

//		printf("\n----->INT: %s\n", &pCharPool[pInt->nameIdx]);

		for (c1=0; c1 < pInt->numOfCrdrs; c1++ ) {
			TPoint2D   cor1Tan, v2;
			double     len;			/* scratch for vector length */
			double     k;			/* scratch for z component of cross prod */

			cor1 = c1 + pInt->crdrIdx;
			pC1  = &pCrdrPool[cor1];

			// Compute direction of corridor 1 (pC1)
			cor1Tan.x = pCrdrCntrlPntPool[pC1->cntrlPntIdx + 1].location.x -
				pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.x;
			cor1Tan.y = pCrdrCntrlPntPool[pC1->cntrlPntIdx + 1].location.y -
				pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.y;

			len = sqrt(cor1Tan.x * cor1Tan.x + cor1Tan.y * cor1Tan.y);
			if ( fabs(len) < cCV_ZERO )	assert(0);

			cor1Tan.x /= len;
			cor1Tan.y /= len;

			for (c2=0; c2 < pInt->numOfCrdrs; c2++ ) {
				TPoint2D   v3;

				cor2 = c2 + pInt->crdrIdx;
				pC2  = &pCrdrPool[cor2];

#if 0
				printf("C1 = %s:%d->%s:%d.  C2 = %s:%d->%s:%d.  ",
					&pCharPool[pRoadPool[pC1->srcRdIdx].nameIdx], pC1->srcLnIdx,
					&pCharPool[pRoadPool[pC1->dstRdIdx].nameIdx], pC1->dstLnIdx,
					&pCharPool[pRoadPool[pC2->srcRdIdx].nameIdx], pC2->srcLnIdx,
					&pCharPool[pRoadPool[pC2->dstRdIdx].nameIdx], pC2->dstLnIdx);
#endif
				// compute v3: vector from start of cor1 to start of cor2
				v3.x = pCrdrCntrlPntPool[pC2->cntrlPntIdx].location.x -
							pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.x;
				v3.y = pCrdrCntrlPntPool[pC2->cntrlPntIdx].location.y -
							pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.y;

				len = sqrt(v3.x * v3.x + v3.y * v3.y);
				if ( fabs(len) < cCV_ZERO ) {
					pC1->dirRel[c2] = cCV_CRDR_PARALLEL_TO;
					continue;
				}
				v3.x /= len;
				v3.y /= len;

				k = cor1Tan.x * v3.y - cor1Tan.y * v3.x;
				if ( k > 0.49 ) {
					pC1->dirRel[c2] = cCV_CRDR_RIGHT_TO;
			//		printf("C1 to the right of C2\n");
				}
				else if ( k > -0.3 && k < 0.49 ) {
					pC1->dirRel[c2] = cCV_CRDR_PARALLEL_TO;
			//		printf("C1 parallel to C2\n");
				}
				else {
					pC1->dirRel[c2] = cCV_CRDR_LEFT_TO;
			//		printf("C1 to the left of C2\n");
				}

				int ind = c2 / 4;
				unsigned int mask = 0x03 << (c2 % 4);
			}

			// Define two vectors, one is tangent at the start of corridor
			// the second is start to end of corridor.

			v2.x = pCrdrCntrlPntPool[pC1->cntrlPntIdx + pC1->numCntrlPnt - 1].location.x -
				pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.x;
			v2.y = pCrdrCntrlPntPool[pC1->cntrlPntIdx + pC1->numCntrlPnt - 1].location.y -
				pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.y;
			len = sqrt(v2.x * v2.x + v2.y * v2.y);
			if ( fabs(len) < cCV_ZERO ) assert(0);
			v2.x /= len;
			v2.y /= len;

			k = cor1Tan.y * v2.x - cor1Tan.x * v2.y;

#if 0
			printf("P0    : %.2f %.2f\n", pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.x,
				pCrdrCntrlPntPool[pC1->cntrlPntIdx].location.y);
			printf("P1    : %.2f %.2f\n",pCrdrCntrlPntPool[pC1->cntrlPntIdx + 1].location.x,
				pCrdrCntrlPntPool[pC1->cntrlPntIdx + 1].location.y);
			printf("Plast : %.2f %.2f\n", pCrdrCntrlPntPool[pC1->cntrlPntIdx + pC1->numCntrlPnt - 1].location.x,
				pCrdrCntrlPntPool[pC1->cntrlPntIdx + pC1->numCntrlPnt - 1].location.y);
			printf("cor1Tan: (%.2f, %.2f),  v2: (%.2f, %.2f)\n", 
				cor1Tan.x, cor1Tan.y, v2.x, v2.y);
			printf("Vert component = %f\n",k);
#endif

			pC1->directionK = k;
			if ( k > 0.49 ) {
				pC1->direction = cCV_CRDR_RIGHT_DIR;
			//	printf("   Dir=right\n");
			}
			else if ( k > -0.3 && k < 0.49 ) {
				pC1->direction = cCV_CRDR_STRAIGHT_DIR;
			//	printf("   Dir=straight\n");
			}
			else {
				pC1->direction = cCV_CRDR_LEFT_DIR;
			//	printf("   Dir=left\n");
			}
		}
	}
}


/****************************************************************************
 *
 * Description: This function gets the intersection boundary.
 *
 * Remarks:
 * 
 *
 ****************************************************************************/

static 
void GetBoundaryOfIntrsctn(
		cvTIntrsctn*	pEntry,
		CPoint2D&		llPt,
		CPoint2D&		urPt)
{
	int start = pEntry->borderSegIdx;
	llPt.m_x = pBorderSegPool[start].x;
	llPt.m_y = pBorderSegPool[start].y;
	urPt.m_x = pBorderSegPool[start].x;
	urPt.m_y = pBorderSegPool[start].y;

	unsigned int i;
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
}	// end of GetBoundaryOfIntrsctn


/****************************************************************************
 *
 * Description: This function checks if line of quad is inside the grid.
 *
 * Remarks:
 * 
 *
 ****************************************************************************/

static bool
IsLineInsideGrid(const double& v1X, 
				 const double& v1Y, 
				 const double& v2X, 
				 const double& v2Y, 
				 const CBoundingBox& gridBbox
				 )
{
	double minX, minY, maxX, maxY;
	minX = gridBbox.GetMinX();
	minY = gridBbox.GetMinY();
	maxX = gridBbox.GetMaxX();
	maxY = gridBbox.GetMaxY();
	
	if( v1X <= minX && v2X <= minX )	return false;

	if( v1X >= maxX && v2X >= maxX )	return false;

	float y1, y2;

	y1 = v2Y + ( ( v1Y - v2Y ) / ( v1X - v2X )  * ( minX - v2X ) );
	y2 = v2Y + ( ( v1Y - v2Y ) / ( v1X - v2X )  * ( maxX - v2X ) );

	if( v1X <= minX && minX < v2X && v2X <= maxX )
	{
		if( minY < y1 && y1 < maxY )	
		{
			return true;
		}

		if( y1 <= minY && v2Y > minY )	
		{
			return true;
		}	
			
		if( y1 <= minY && v2Y <= minY )	return false;


		if( y1 >= maxY && v2Y < maxY )	
		{
			return true;
		}

		if( y1 >= maxY && v2Y >= maxY )	return false;
	}

	if( v1X <= minX && v2X > maxX )
	{	
		if( minY < y1 && y1 < maxY )	
		{
			return true;
		}

		if( minY < y2 && y2 < maxY )	
		{
			return true;
		}
	}

	bool isV1xBetween = minX < v1X && v1X < maxX;
	bool isV2xBetween = minX < v2X && v2X < maxX;
	if( isV1xBetween && isV2xBetween )
	{
		if( minY < v1Y && v1Y < maxY )	
		{
			return true;
		}

		if( minY < v2Y && v2Y < maxY )	
		{
			return true;
		}
		if( v1Y >= maxY && v2Y <= minY )	
		{
			return true;
		}
	}

	if( isV1xBetween && v2X >= maxX )
	{
		if( minY < y2 && y2 < maxY )	
		{
			return true;
		}

		if( y2 <= minY && (v1Y > minY ))	
		{
			return true;
		}

		if( y2 <= minY && v1Y <= minY )		return false;

		if( y2 >= maxY && v1Y < maxY )	
		{
			return true;
		}

		if( y2 >= maxY && v1Y >= maxY )		return false;
	}

	return false;

}	// end of IsLineInsideGrid


/****************************************************************************
 *
 * Description: This function check if the quad is inside the grid.
 *
 * Remarks:
 * 
 *
 ***************************************************************************/
static bool
IsQuadInsideGrid( 
		const TLrCntrlPnt cp1,
		const TLrCntrlPnt cp2,
		const CBoundingBox& gridBbox
		)
{
	// Build and check if quad intersects the grid.
	float v1X, v1Y, v2X, v2Y, v3X, v3Y, v4X, v4Y;
	v1X = cp1.location.x + cp1.rightVec.i * ( cp1.width / 2 );
	v1Y = cp1.location.y + cp1.rightVec.j * ( cp1.width / 2 );
	v2X = cp1.location.x + cp1.rightVec.i * (- cp1.width / 2 );
	v2Y = cp1.location.y + cp1.rightVec.j * (- cp1.width / 2 );
	v3X = cp2.location.x + cp2.rightVec.i * ( cp2.width / 2 );
	v3Y = cp2.location.y + cp2.rightVec.j * ( cp2.width / 2 );
	v4X = cp2.location.x + cp2.rightVec.i * (- cp2.width / 2 );
	v4Y = cp2.location.y + cp2.rightVec.j * (- cp2.width / 2 );
	
	
	float minX = gridBbox.GetMinX();
	float maxX = gridBbox.GetMaxX();
	float minY = gridBbox.GetMinY();
	float maxY = gridBbox.GetMaxY();


	// First check if quad inside grid or grid inside quad
	bool isV1V3OneSide = false;
	bool isV1V4OneSide = false;

	if( ( v3X == v1X || v3Y == v1Y ) && ( v4X == v2X || v4Y == v2Y ) )
	{ 
		isV1V3OneSide = true;
	}
	if( ( v4X == v1X || v4Y == v1Y ) && ( v2X == v3X || v2Y == v3Y ) )
	{
		isV1V4OneSide = true;
	}


	bool southNorthDir = false;
	bool northSouthDir = false;
	bool westEastDir = false;
	bool eastWestDir = false;

	if( isV1V3OneSide )
	{
		if( v1X > v2X && v1Y == v2Y )	southNorthDir = true;
		
		if( v1X == v2X && v2Y > v1Y )	westEastDir = true;

		if( v2X > v1X && v1Y == v2Y )	northSouthDir = true;

		if( v1X == v2X && v1Y > v2Y )	eastWestDir = true;
	}

	if( southNorthDir )
	{
		if( ( v1X > v2X ) && 
			( v3Y > v1Y ) &&
			( maxX > minX ) &&
			( maxY > minY )
		)
		{
			if( ( maxY <= v3Y ) &&
				( minY >= v1Y ) &&
				( minX >= v2X ) &&
				( maxX <= v1X )
			)
			{
				return true; // grid inside quad
			}
			if( ( maxY >= v3Y ) &&
				( minY <= v1Y ) &&
				( minX <= v2X ) &&
				( maxX >= v1X )
			)
			{
				return true; // quad inside grid
			}
		}
	}
	if( westEastDir )
	{
		if( ( v3X > v1X ) &&
			( v4Y > v3Y ) &&
			( maxX > minX ) &&
			( maxY > minY )
		)
		{
			if( ( maxY <= v4Y ) &&
				( minY >= v3Y ) &&
				( minX >= v1X ) &&
				( maxX <= v3X )
			)
			{
				return true; // grid insdie quad
			}

			if( ( maxY >= v4Y ) &&
				( minY <= v3Y ) &&
				( minX <= v1X ) &&
				( maxX >= v3X )
			)
			{
				return true; // quad inside grid
			}
		}
	}
	
	if( northSouthDir )
	{
		if( ( v4X > v3X ) && 
			( v2Y > v4Y ) &&
			( maxX > minX ) &&
			( maxY > minY )
		)
		{
			if( ( maxY <= v2Y ) &&
				( minY >= v4Y ) &&
				( minX >= v3X ) &&
				( maxX <= v4X )
			)
			{
				return true; // grid inside quad
			}
			if( ( maxY >= v2Y ) &&
				( minY <= v4Y ) &&
				( minX <= v3X ) &&
				( maxX >= v4X )
			)
			{
				return true; // quad inside grid
			}
		}
	}

	if( eastWestDir )
	{
		if( ( v2X > v4X ) &&
			( v1Y > v2Y ) &&
			( maxX > minX ) &&
			( maxY > minY )
		)
		{
			if( ( maxY <= v1Y ) &&
				( minY >= v2Y ) &&
				( minX >= v4X ) &&
				( maxX <= v2X )
			)
			{
				return true; // grid insdie quad
			}

			if( ( maxY >= v1Y ) &&
				( minY <= v2Y ) &&
				( minX <= v4X ) &&
				( maxX >= v2X )
			)
			{
				return true; // quad inside grid
			}
		}
	}


	// When execution reaches here, have to check side by side

	bool isSide1InsideGrid_fc, isSide1InsideGrid_sc;
	
	isSide1InsideGrid_fc = IsLineInsideGrid(v1X, v1Y, v2X, v2Y, gridBbox);

	if( isSide1InsideGrid_fc )
	{
		return true;
	}
	else
	{
		isSide1InsideGrid_sc = IsLineInsideGrid(v2X, v2Y, v1X, v1Y, gridBbox);

		if( isSide1InsideGrid_sc )	return true;
	}


	bool isSide2InsideGrid_fc, isSide2InsideGrid_sc;

	isSide2InsideGrid_fc = IsLineInsideGrid(v3X, v3Y, v4X, v4Y, gridBbox);

	if( isSide2InsideGrid_fc )
	{
		return true;
	}
	else
	{
		isSide2InsideGrid_sc = IsLineInsideGrid(v4X, v4Y, v3X, v3Y, gridBbox);

		if( isSide2InsideGrid_sc )	return true;
			
	}
	
	// Check more sides.
	double dist1_3, dist1_4,tempVal1, tempVal2;
	float x1, y1, x3, y3, x4, y4;
	x1 = fabs(v1X);
	y1 = fabs(v1Y);
	x3 = fabs(v3X);
	y3 = fabs(v3Y);
	x4 = fabs(v4X);
	y4 = fabs(v4Y);

	tempVal1 = (x3 - x1) * (x3 - x1) + ((y3 - y1) * ( y3 - y1));
	tempVal2 = (x4 - x1) * (x4 - x1) + ((y4 - y1) * ( y4 - y1));

	dist1_3 = sqrt(tempVal1);
	dist1_4 = sqrt(tempVal2);

	if( dist1_3 < dist1_4)	
	{
		bool isSide3InsideGrid_fc, isSide3InsideGrid_sc;
		isSide3InsideGrid_fc = IsLineInsideGrid(v1X, v1Y, v3X, v3Y, gridBbox);

		if( isSide3InsideGrid_fc )
		{
			return true;
		}
		else
		{
			isSide3InsideGrid_sc = IsLineInsideGrid(v3X, v3Y, v1X, v1Y, gridBbox);

			if( isSide3InsideGrid_sc )	return true;
			else
			{
				bool isSide4InsideGrid_fc, isSide4InsideGrid_sc;
				isSide4InsideGrid_fc = IsLineInsideGrid(v2X, v2Y, v4X, v4Y, gridBbox);

				if( isSide4InsideGrid_fc )
				{
					return true;
				}
				else
				{
					isSide4InsideGrid_sc = IsLineInsideGrid(v4X, v4Y, v2X, v2Y, gridBbox);
	
					if( isSide4InsideGrid_sc )	return true;
					else	return false;	
				}		
			}
		}
	}
	else		
	{
		bool isSide3InsideGrid_fc, isSide3InsideGrid_sc;
		isSide3InsideGrid_fc = IsLineInsideGrid(v1X, v1Y, v4X, v4Y, gridBbox);

		if( isSide3InsideGrid_fc )
		{
			return true;
		}
		else
		{
			isSide3InsideGrid_sc = IsLineInsideGrid(v4X, v4Y, v1X, v1Y, gridBbox);

			if( isSide3InsideGrid_sc )	return true;
			else
			{
				bool isSide4InsideGrid_fc, isSide4InsideGrid_sc;
				isSide4InsideGrid_fc = IsLineInsideGrid(v2X, v2Y, v3X, v3Y, gridBbox);

				if( isSide4InsideGrid_fc )
				{
					return true;
				}
				else
				{
					isSide4InsideGrid_sc = IsLineInsideGrid(v3X, v3Y, v2X, v2Y, gridBbox);

					if( isSide4InsideGrid_sc )	return true;
					else	return false;	
				}		
			}
		}
	}
	
}	// end of IsQuadInsideGrid


/****************************************************************************
 *
 * Description: This function gets the grid info.
 *
 * Remarks:
 * 
 *
 ****************************************************************************/
static void
GetGridInfo(cvTIntrsctn* pInt, const CBoundingBox& gridBbox, int gridId, cvTGrid& grid)
{
	cvTCrdr* pCrdr;
	unsigned int crdrId, crdrIdx, cntrlPnt;
	
	vector<cvTCrdrInfo> crdrs;

	//	For each corridor	
	for ( crdrId = 0; crdrId < pInt->numOfCrdrs; crdrId++ )
	{
		int startCntrlPntIdx = -1;
		int endCntrlPntIdx = -1;

		crdrIdx = crdrId + pInt->crdrIdx;
		pCrdr  = &pCrdrPool[crdrIdx];
					
		cvTCrdrCntrlPnt *pCur, *pNext;
		pCur = &pCrdrCntrlPntPool[pCrdr->cntrlPntIdx];
		
		// for every two control points
		for( cntrlPnt = 1; cntrlPnt < pCrdr->numCntrlPnt; cntrlPnt++,++pCur  )
		{

			pNext = pCur + 1;
			
			TLrCntrlPnt  cp1, cp2;

			cp1.location.x = pCur->location.x;
			cp1.location.y = pCur->location.y;
			cp1.location.z = 0.0;
			cp1.rightVec.i = pCur->rightVecLinear.i;
			cp1.rightVec.j = pCur->rightVecLinear.j;
			cp1.rightVec.k = 0.0;
			cp1.width = pCur->width;
			cp1.distance = pCur->distance;

			cp2.location.x  = pNext->location.x;
			cp2.location.y  = pNext->location.y;
			cp2.location.z = 0.0;
			cp2.rightVec.i = pNext->rightVecLinear.i;
			cp2.rightVec.j = pNext->rightVecLinear.j;
			cp2.rightVec.k = 0.0;
			cp2.width = pNext->width;
			cp2.distance = pNext->distance;

			// Check if corridor seg between two control points are inside the grid.
			bool isQuadInsideGrid = IsQuadInsideGrid( cp1, cp2, gridBbox);

			if( isQuadInsideGrid )
			{
				if( startCntrlPntIdx == -1 )
				{
					startCntrlPntIdx = cntrlPnt;

					if( endCntrlPntIdx == -1 )
					{
						endCntrlPntIdx = cntrlPnt + 1;
					}
				}
				else
				{
					if( cntrlPnt + 1 == pCrdr->numCntrlPnt )
					{
						endCntrlPntIdx = cntrlPnt + 1;
						break;
					}
				}
			}
			else
			{
				if( startCntrlPntIdx != -1 )
				{
					endCntrlPntIdx = cntrlPnt;	
					break;	// no need to check more control points
				}
			}
		}	//for every two control points

		// After checking current corridor
		if( startCntrlPntIdx != -1 && endCntrlPntIdx != -1 )
		{
			// store the crdrId, startIdx and endIdx
			cvTCrdrInfo crdrInfo;
			crdrInfo.crdrId = pCrdr->myId;
			crdrInfo.startCntrlPtIdx = startCntrlPntIdx;
			crdrInfo.endCntrlPtIdx = endCntrlPntIdx;
			crdrs.push_back( crdrInfo );			
		}
		if( startCntrlPntIdx == -1 && endCntrlPntIdx == -1 )
		{
			//  store the crdrId, startIdx and endIdx
			cvTCrdrInfo crdrInfo;
			crdrInfo.crdrId = -1;
			crdrInfo.startCntrlPtIdx = -1;
			crdrInfo.endCntrlPtIdx = -1;
			crdrs.push_back( crdrInfo);						
		}
		
	pCrdr++;

	}	// for each corridor

	// Store grid info
	grid.gridId = gridId;
	grid.minX = gridBbox.GetMinX();
	grid.minY = gridBbox.GetMinY();
	grid.maxX = gridBbox.GetMaxX();
	grid.maxY = gridBbox.GetMaxY();
	
	vector<cvTCrdrInfo>::iterator itr;
	unsigned int j = 0;

	for( itr = crdrs.begin(); j < cCV_MAX_CRDRS; j++)
	{
		
		if( j < pInt->numOfCrdrs && itr != crdrs.end() )
		{
			int crdrId = (*itr).crdrId;
			int startIdx = (*itr).startCntrlPtIdx;
			int endIdx = (*itr).endCntrlPtIdx;

			grid.intrsctingCrdrs[j].crdrId = crdrId;
			grid.intrsctingCrdrs[j].startCntrlPtIdx = startIdx;
			grid.intrsctingCrdrs[j].endCntrlPtIdx = endIdx;
		}
		else 
		{
			grid.intrsctingCrdrs[j].crdrId = -1;
			grid.intrsctingCrdrs[j].startCntrlPtIdx = -1;
			grid.intrsctingCrdrs[j].endCntrlPtIdx = -1;
		}
		if(itr != crdrs.end())
			itr++;
	}


}	// end  of GetGridInfo


/****************************************************************************
 *
 * Description: Build intersection grids.
 *
 * Remarks:
 * 
 *
 ****************************************************************************/

static void
BuildIntersectionGrids(cvTIntrsctn* pIntrsctnPool, int sizeOfIntrsctnPool)
{
	const int cNUM_COLS = 10;
	const int cNUM_ROWS = 10;

	int i, j, k;
	double length, width, gridLength, gridWidth;
	double x0, y0, x, y, x1, y1, x2, y2;

	// For each intersection
	for (i = 1; i < sizeOfIntrsctnPool; i++) 
	{
		vector<cvTGrid> gridsVec;
		cvTIntrsctn*   pInt = pIntrsctnPool + i;

		CPoint2D llPt, urPt;
		GetBoundaryOfIntrsctn(pInt, llPt, urPt);
		x0 = llPt.m_x;
		y0 = llPt.m_y;
		x = urPt.m_x;
		y = urPt.m_y;

		length = fabs (x0 - x);
		width = fabs (y0 - y);//fabs (y0) - fabs (y);


		gridLength = length / cNUM_ROWS;
		gridWidth = width / cNUM_COLS;

		int gridCount = 0;
		for( j = 0; j < cNUM_ROWS; j++ )
		{
			for ( k = 0; k < cNUM_COLS; k++ )
			{
				x1 = x0 + (j * gridLength);
				y1 = y0 + (k * gridWidth);
				x2 = x1 + gridLength;
				y2 = y1 + gridWidth;

				// Build the grid box.
				CBoundingBox gridBBox(x1,y1,x2,y2);		
				gridCount++;

				// Get info for the grid.
				cvTGrid grid;
				GetGridInfo(pInt, gridBBox, gridCount, grid);

				gridsVec.push_back(grid);

			}
		}

		//  Insert after all grids built for current intersection.
		int gdCount = 0;
		vector<cvTGrid>::iterator itr;
		for( itr = gridsVec.begin(); 
			itr != gridsVec.end() && gdCount < cCV_MAX_GRIDS; 
			gdCount++, itr++ 
			)
		{
			pInt->grids[gdCount] = *itr;
			
		}
	
	}	// for each intersection
	
}	// end of BuildIntersectionGrids

#ifdef _WIN32
////////////////////////////////////////////////////////////////////////////////////////
/// \brief
///		This function handler produces a mini core dump, which is critical for debugging
///		real-time apps when they crash.
/////////////////////////////////////////////////////////////////////////////////////////
LONG WINAPI FatalExceptionHandler(struct _EXCEPTION_POINTERS *pExceptionInfo) {
	LONG retval = EXCEPTION_CONTINUE_SEARCH;

	HMODULE hDll = NULL;
	// load any version we can
	hDll = ::LoadLibrary("DBGHELP.DLL");

	if (hDll) {
		// Get a function pointer to MiniDumpWriteDump from windows
		MINIDUMPWRITEDUMP pDump = (MINIDUMPWRITEDUMP) ::GetProcAddress(hDll, "MiniDumpWriteDump");

		// MiniDumpWriteDump
		if (pDump) {
			string path = "dmp";
			if (!::CreateDirectoryA("dmp", NULL)) {
				if (::GetLastError() != ERROR_ALREADY_EXISTS) {
					path = ".";
				}
			}
			::SetCurrentDirectoryA(path.c_str());

			SYSTEMTIME time;
			memset(&time, 0, sizeof(SYSTEMTIME));

			::GetLocalTime(&time);

			ostringstream conv;
			conv << ".\\lriccCrash";
			conv << '_' << std::setw(2) << std::setfill('0') << time.wMonth;
			conv << '_' << std::setw(2) << std::setfill('0') << time.wDay;
			conv << '_' << std::setw(4) << std::setfill('0') << time.wYear;
			conv << '_' << std::setw(2) << std::setfill('0') << time.wHour;
			conv << '_' << std::setw(2) << std::setfill('0') << time.wMinute;
			conv << '_' << std::setw(2) << std::setfill('0') << time.wSecond;
			conv << ".dmp";

			printf("FATAL ERROR: producing mini-core dump, save the %s file\n", conv.str().c_str());

			// create the file
			HANDLE hFile = ::CreateFile(conv.str().c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, NULL, CREATE_ALWAYS,
										FILE_ATTRIBUTE_NORMAL, NULL);

			if (hFile != INVALID_HANDLE_VALUE) {
				_MINIDUMP_EXCEPTION_INFORMATION ExInfo;

				ExInfo.ThreadId = ::GetCurrentThreadId();
				ExInfo.ExceptionPointers = pExceptionInfo;
				ExInfo.ClientPointers = NULL;

				// write the dump
				BOOL bOK = pDump(::GetCurrentProcess(), ::GetCurrentProcessId(), hFile, MiniDumpNormal, &ExInfo, NULL, NULL);
				if (bOK) {
					retval = EXCEPTION_EXECUTE_HANDLER;
				}

				::CloseHandle(hFile);
			}
		}
	}

	return retval;
}
#endif

/****************************************************************************
 *
 * Description: program entry point
 *
 * Remarks:
 * This is the main entry point for the lri compiler.
 *
 */
void main (int argc, char *argv[]) 
{
	char        filename[256] = { 0 };
	int         token;
	int         arg;
	char		binLRI[256] = { 0 };

#ifdef _WIN32
	::SetUnhandledExceptionFilter(FatalExceptionHandler);
#endif

	if ( argc < 3 )
		usage(argv[0]);

	for (arg=1; arg<argc; arg++) {
		if ( !strcmp(argv[arg], "-nc") ) {
			gNoSplineCorrect = 1;
		}
		else
		if ( !strcmp(argv[arg], "-hs") ) {
			arg++;
			gHermiteSplineScale = atof(argv[arg]);
		}
		else
		if ( !strcmp(argv[arg], "-v1") ) {
			arg++;
			gOvrdVersionMaj = atoi(argv[arg]);
		}
		else
		if ( !strcmp(argv[arg], "-v2") ) {
			arg++;
			gOvrdVersionMin = atoi(argv[arg]);
		}
		else
		if ( !strcmp(argv[arg], "-v3") ) {
			arg++;
			gOvrdVersion1 = atoi(argv[arg]);
		}
		else
		if ( !strcmp(argv[arg], "-v4") ) {
			arg++;
			gOvrdVersion2 = atoi(argv[arg]);
		}
		else
		if ( !strcmp(argv[arg], "-v5") ) {
			arg++;
			gOvrdVersion3 = atoi(argv[arg]);
		}
		else
		if ( !strcmp(argv[arg], "-debug") ) {
			gDebugDump = 1;
		}
		else
		if ( !strcmp(argv[arg], "-tlex") ) {
			gTestLex = 1;
		}
		else
		if ( !strcmp(argv[arg], "-nodash") ) {
			arg++;
			gNoMinusInNames = argv[arg][0];
		}
		else
		if ( !strcmp(argv[arg], "-ct") ) {
			arg++;
			gCrdrTolerance = atof( argv[arg] );
		}
		else
		if ( !strcmp(argv[arg], "-gap") ) {
			arg++;
			gGapTolerance = atof( argv[arg] );
		}
		else
		if ( argv[arg][0] == '-' ) {
			usage(argv[0]);
		}
		else
		if ( filename[0] == '\0' ) {	/* have no input */
			strncpy(filename, argv[arg], sizeof(filename)-1);
		}
		else
		if ( binLRI[0] == '\0' ) {	/* have no output */
			strncpy(binLRI, argv[arg], sizeof(filename)-1);
		}
		else
			usage(argv[0]);
	}

	if ( strrchr(filename, '.') == NULL ) strcat(filename, ".lri");
	if ( strrchr(binLRI, '.') == NULL ) strcat(binLRI, ".bli");

	printf("LRI Compiler (C) NADS & SC, UI and the UI. Version %d.%d.%d.%d.%d\n",
		gGetMajorCvedVersionNum(), gGetMinorCvedVersionNum(),
		gGetMinorExt1CvedVersionNum(), gGetMinorExt2CvedVersionNum(),
		gGetMinorExt3CvedVersionNum());

	printf("Compiling %s into %s.\n", filename, binLRI);

	yyin = fopen (filename, "r");
	if (yyin == NULL) {
		fprintf(stderr, "Cannot open file '%s'.  ", filename);
		perror("");
		exit(-1);
	}
	yyout = stdout;


	if ( gTestLex ) {
		int         count;
		printf("Testing lexical analyzer\n");

		count = 0;
		while ( token = yylex() ) {

			count++;
			printf("Tok = %d, '%s' \n", token, yytext);
		}
	
		printf("Count of tokens is %d\n", count);
		exit(0);
	}

	if ( yyparse() != 0 ) {  /* yyparse returns 0 in case of syntax error */
		fprintf(stdout, " ** Errors found, no output generated.\n");
		exit(-1);
	}

	fclose(yyin);

	if ( gDebugDump ) 
		dumpInfo();


	vector<cvTElevPost> postPool;
	vector<cvTElevMap>  elevMapPool;

	if ( gNoMinusInNames ) 
		FixNames(gpRoadList, gpInterList);

	SemanticCheck(gpRoadList, gpInterList, gpElevMaps);

	GenerateObjects(
			gpObjects, 
			gObjectCount, 
			&pObjPool,
			&sizeOfObjPool);

	GenerateCode(gpRoadList, gpInterList, gpElevMaps, postPool, elevMapPool);

	// Free memory for the data structures maintained by the parser.
	// Set pointers to zero to catch any access that takes place 
	// after memory is freed.

	{

		while ( gpInterList ) {
			TIntersection *pInt = gpInterList;

			gpInterList = gpInterList->pNext;
			free(pInt);
		}
		gpInterList = 0;

		while ( gpRoadList ) {
			TRoads *pRd = gpRoadList;

			gpRoadList = gpRoadList->pNext;
			free(pRd);
		}
		gpRoadList = 0;

		while ( gpElevMaps ) {
			TElevMap *pMap = gpElevMaps;

			gpElevMaps = gpElevMaps->pNextElevMap;
			free(pMap);
		}
		gpElevMaps = 0;

		free(gpObjects);
		gpObjects = 0;
	}

	CreateObjectReferencePool(   
			rdPcQTree,
			intrsctnQTree,
		    pRoadPool,
		    sizeOfRoadPool,
		    pRoadPiecePool,
		    sizeOfRoadPiecePool,
			pIntrsctnPool,
			sizeOfIntrsctnPool,
		    pCntrlPntPool,
		    sizeOfCntrlPntPool,
		    pLatCntrlPntPool,
		    sizeOfLatCntrlPntPool,
		    pObjPool,
		    sizeOfObjPool,
//		    pObjAttrPool,
//		    sizeOfObjAttrPool,
		    pRepObjPool,
		    pObjRefPool,
		    &sizeOfObjRefPool,
		    true);

	ComputeCorridorDirections(
			pIntrsctnPool,
			sizeOfIntrsctnPool,
			pCrdrCntrlPntPool,
			sizeOfCrdrCntrlPntPool
			);

	BuildIntersectionGrids(
		pIntrsctnPool,
		sizeOfIntrsctnPool
		);
#if 0
	// these are the test codes only for highway.lri
	DisplayIntrsctn(pCharPool, pIntrsctnPool, sizeOfIntrsctnPool, pObjRefPool);
	DisplayObjDataOfRoad(
			"R3_660_19800", 
			pCharPool, 
			pRoadPool, 
			sizeOfRoadPool,
			pCntrlPntPool,
			pRepObjPool,
			pObjRefPool);
	DisplayObjDataOfRoad(
			"R3_660_40260", 
			pCharPool, 
			pRoadPool, 
			sizeOfRoadPool,
			pCntrlPntPool,
			pRepObjPool,
			pObjRefPool);
	DumpObjPool(pObjPool, sizeOfObjPool);
	DumpRepObjPool(pRepObjPool, sizeOfRepObjPool);
	DumpObjRefPool(pObjRefPool, sizeOfObjRefPool);
#endif

	WriteOutputFile(binLRI, postPool, elevMapPool);
	printf(" Binary LRI generation completed successfully.\n");
	exit(0);
}
