/***************************************************************************
 * (C) Copyright 1998 by NADS & Simulation Center The University of Iowa
 * and The University of Iowa.  All rights reserved.
 *
 * $Id: dbgdump.c,v 1.2 2009/08/26 16:43:45 iowa\oahmad Exp $
 *
 * Author(s) :
 * Date: September 1998
 *
 * Description:
 * Code to dump internal data structures, primarily for debugging.
 *
 **************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include "parser.h"

void 
DumpBorderSeg( const cvTBorderSeg* pBorderSegPool, int size )
{
	int i;

	printf(" ---- Contents of border seg pool ----\n");
	for (i=1; i<size; i++) {
		printf(" BorderSeg	%d\n", i);
		printf("\tx		= %.2f\n", pBorderSegPool[i].x);
		printf("\ty		= %.2f\n", pBorderSegPool[i].y);
		printf("\tlineFlag	= %c\n", pBorderSegPool[i].lineFlag);
	}
	printf("\n");
}

void 
DumpHldOfsPool( const cvTHldOfs* pHldOfsPool, int size )
{
	int i;

	printf(" ---- Contents of holdoffset pool -----\n");
	for (i=1; i<size; i++) {
		printf(" Hold offset %d\n", i);
		printf("\tobjId 		= %.2d\n", pHldOfsPool[i].objIdx);
		printf("\tdistance	= %.2f\n", pHldOfsPool[i].distance);
		printf("\torientation	= %.2f\n", pHldOfsPool[i].orientation);
		printf("\tthickness	= %.2f\n", pHldOfsPool[i].thickness);
		printf("\treason");
		if (pHldOfsPool[i].reason & eHOLD_SIGN)
			printf("\t%s", "SIGN");
		else if (pHldOfsPool[i].reason & eHOLD_TLIGHT)
			printf("\t%s", "TIGHT");
		else if (pHldOfsPool[i].reason & eHOLD_OVLAP)
			printf("\t%s", "OVLAP");
		else
			printf("\t\t%s", "No reason");
		printf("\t %d", pHldOfsPool[i].reason);
		printf("\n");
	}
	printf("\n");
}

		
void 
DumpCrdrPool( const cvTCrdr* pCrdrPool, int size )
{
	int i;

	printf(" ---- Contents of corridor pool -----\n");
	for (i=1; i<size; i++) {
		printf("Crdr %d\n",i);
		printf("\tmyId		= %d\n", pCrdrPool[i].myId);
		printf("\tsrcRdIdx	= %d\n", pCrdrPool[i].srcRdIdx);
		printf("\tdstRdIdx	= %d\n", pCrdrPool[i].dstRdIdx);
		printf("\tsrcLnIdx	= %d\n", pCrdrPool[i].srcLnIdx);
		printf("\tdstLnIdx	= %d\n", pCrdrPool[i].dstLnIdx);
		printf("\tcntrlPntIdx	= %d\n", pCrdrPool[i].cntrlPntIdx);
		printf("\tnumCntrlPnt	= %d\n", pCrdrPool[i].numCntrlPnt);
		printf("\tintrsctnId	= %d\n", pCrdrPool[i].intrsctnId);
		printf("\thldOfsIdx	= %d\n", pCrdrPool[i].hldOfsIdx);
		printf("\tnumHldOfs	= %d\n", pCrdrPool[i].numHldOfs);
		printf("\tattrIdx	= %d\n", pCrdrPool[i].attrIdx);
		printf("\tnumAttr	= %d\n", pCrdrPool[i].numAttr);
	}
	printf("\n");
}


void 
DumpCrdrCntrlPntPool( const cvTCrdrCntrlPnt* pCntrlPntPool, int size )
{
	int i;

	printf(" ---- Contents of corridor control point pool -----\n");	 
	for (i=1; i<size; i++) {
		printf("CrdrCntrlPnt	%d\n", i);
		printf("\tx		= %.2f\n", pCntrlPntPool[i].location.x);
		printf("\ty		= %.2f\n", pCntrlPntPool[i].location.y);
		printf("\twidth		= %.2f\n", pCntrlPntPool[i].distance);
		printf("\trightvec  = %.2f %.2f\n", 
			pCntrlPntPool[i].rightVecLinear.i, pCntrlPntPool[i].rightVecLinear.j);

		printf("\tflagStyle");
		if (pCntrlPntPool[i].flagStyle & eLEFT_FLAG)
			printf("\t%s", "LEFT_LINE");
		if (pCntrlPntPool[i].flagStyle & eLEFT_DOTTED)
			printf("\t%s", "LEFT_DOTTED");
		if (pCntrlPntPool[i].flagStyle & eRIGHT_FLAG)
			printf("\t%s", "RIGHT_LINE");
		if (pCntrlPntPool[i].flagStyle & eRIGHT_DOTTED)
			printf("\t%s", "RIGHT_DOTTED");
		printf("\n");
	}
	printf("\n");
}


void 
DumpCntrlPnt( const cvTCntrlPnt* pCntrlPnt, int size )
{
	int i;

	printf(" ----- Contents of control point -----\n");
	for (i=1; i<size; i++) {
		int j;

		printf("CntrlPnt %d\n", i);
		printf("\tphysicalWidth		= %.2f\n", 
									(float)pCntrlPnt[i].physicalWidth);
		printf("\tlogicalWidth		= %.2f\n", 
									(float)pCntrlPnt[i].logicalWidth);
		printf("\tlatCntrlPntIdx\t	= %d\n", (int)pCntrlPnt[i].latCntrlPntIdx);
		printf("\tnumLatCntrlPnt\t	= %d\n", (int)pCntrlPnt[i].numLatCntrlPnt);
		printf("\tlaneIdx			= %d\n", (int)pCntrlPnt[i].laneIdx);
		printf("\tnumLanes		= %d\n", (int)pCntrlPnt[i].numLanes);
		printf("\tlocation		= %.2f	%.2f	%.2f\n",
									(float)pCntrlPnt[i].location.x,
									(float)pCntrlPnt[i].location.y,
									(float)pCntrlPnt[i].location.z);
		printf("\tnormal			= %.2f  %.2f    %.2f\n",
									(float)pCntrlPnt[i].normal.i,
									(float)pCntrlPnt[i].normal.j,
									(float)pCntrlPnt[i].normal.k);
		printf("\ttangVecCubic		= %.2f  %.2f    %.2f\n",
									(float)pCntrlPnt[i].tangVecCubic.i,
									(float)pCntrlPnt[i].tangVecCubic.j,
									(float)pCntrlPnt[i].tangVecCubic.k);
		printf("\trightVecCubic		= %.2f  %.2f    %.2f\n",
									(float)pCntrlPnt[i].rightVecCubic.i,
									(float)pCntrlPnt[i].rightVecCubic.j,
									(float)pCntrlPnt[i].rightVecCubic.k);
		printf("\ttangVecCubic		= %.2f  %.2f    %.2f\n",
									(float)pCntrlPnt[i].tangVecCubic.i,
									(float)pCntrlPnt[i].tangVecCubic.j,
									(float)pCntrlPnt[i].tangVecCubic.k);
		printf("\trightVecCubic		= %.2f  %.2f    %.2f\n",
									(float)pCntrlPnt[i].rightVecCubic.i,
									(float)pCntrlPnt[i].rightVecCubic.j,
									(float)pCntrlPnt[i].rightVecCubic.k);
		for (j=0; j<3; j++){
			printf("\thermite %d", j);
			printf("\t\t= %.2f	%.2f	%.2f	%.2f\n",
									(float)pCntrlPnt[i].hermite[j].A,
									(float)pCntrlPnt[i].hermite[j].B,
									(float)pCntrlPnt[i].hermite[j].C,
									(float)pCntrlPnt[i].hermite[j].D);
		}
		printf("\tcummulativeLinDist	= %.2f\n",
									(float)pCntrlPnt[i].cummulativeLinDist);
		printf("\tcummulativeCubicDist	= %.2f\n",
									(float)pCntrlPnt[i].cummulativeCubicDist);
		printf("\tdistToNextLinear		= %.2f\n",
									(float)pCntrlPnt[i].distToNextLinear);
		printf("\tdistToNextCubic		= %.2f\n",
									(float)pCntrlPnt[i].distToNextCubic);
		printf("\tsurfacePropIdx	= %d\n", (int)pCntrlPnt[i].surfacePropIdx);
		printf("\tsn			= %.2f\n", (float)pCntrlPnt[i].sn);
		printf("\tst			= %.2f\n", (float)pCntrlPnt[i].st);
		printf("\tradius			= %.2f\n", (float)pCntrlPnt[i].radius);
		printf("\tnameIdx			= %d\n", (int)pCntrlPnt[i].nameIdx);
	}
	printf("\n");
}


void 
DumpRoadPool( const cvTRoad* pRoadPool, int size )
{
	int  i;

	printf(" ----- Contents of road pool -----\n");
	for (i=1; i<size; i++) {
		printf("Road %d\n", i);
        printf( "\tmyId             = %d\n", (int)pRoadPool[i].myId );
        printf( "\tcntrlPntIdx      = %d\n", (int)pRoadPool[i].cntrlPntIdx );
        printf( "\tnumCntrlPnt      = %d\n", (int)pRoadPool[i].numCntrlPnt );
        printf( "\tnameIdx          = %d\n", (int)pRoadPool[i].nameIdx );
        printf( "\tsrcIntrsctn      = %d\n", (int)pRoadPool[i].srcIntrsctnIdx);
        printf( "\tdstIntrsctn      = %d\n", (int)pRoadPool[i].dstIntrsctnIdx);
        printf( "\troadLengthLinear = %f\n", pRoadPool[i].roadLengthLinear );
        printf( "\troadLengthSpline = %f\n", pRoadPool[i].roadLengthSpline );
        printf( "\tattrIdx          = %d\n", (int)pRoadPool[i].attrIdx );
        printf( "\tnumAttr          = %d\n", (int)pRoadPool[i].numAttr );
        printf( "\trepObjIdx        = %d\n", (int)pRoadPool[i].repObjIdx );
        printf( "\tnumRepObj        = %d\n", (int)pRoadPool[i].numRepObj );
        printf( "\tnumOfLanes       = %d\n", (int)pRoadPool[i].numOfLanes);

	}
}

	
void 
DumpCharPool( const char* pCharPool, int size )
{
	int  i;

	printf(" ----- Contents of the char pool -----\n");
	for (i=0; i<size; i++) {
		if ( i == 0 || (i%8) == 0 ) {
			printf("\nOfs%d\t", i);
		}
		printf("'%c'%4d", 
			pCharPool[i] == 0 ? ' ' : pCharPool[i], 
			(int)pCharPool[i]);
	}
	printf("\n");
}

void 
DumpIntrsctnPool( const cvTIntrsctn* pIntrsctnPool, int size )
{
	int i;

	printf("\n ---- Contents of intersection pool -----\n");
	for (i=1; i<size; i++) {
		int  j, k;

		printf("Intersection %d\n", i);
		printf("\tmyId          = %d\n", (int)pIntrsctnPool[i].myId);
		printf("\tnameIdx       = %d\n", (int)pIntrsctnPool[i].nameIdx);
		printf("\tname          = %s\n", &pCharPool[(int)pIntrsctnPool[i].nameIdx]);
		printf("\tnumOfRoads    = %d\n", (int)pIntrsctnPool[i].numOfRoads);
		printf("\tAdjacent roads:\t");
		for (j=0; j<pIntrsctnPool[i].numOfRoads; j++) {
			printf("%d ", (int)pIntrsctnPool[i].adjacentRoadId[j]);
		}
		printf("\n");
		printf("\tCorridor idx   = %d\n", (int)pIntrsctnPool[i].crdrIdx);
		printf("\tnumOfCrdrs     = %d\n", (int)pIntrsctnPool[i].numOfCrdrs);

		printf("\tElevmap idx    = %d\n", (int)pIntrsctnPool[i].elevMap);
		printf("\tElevMap X, Y   = %lf, %lf\n", pIntrsctnPool[i].elevMapX,
			pIntrsctnPool[i].elevMapY);
		printf("\tElevation      = %lf\n", pIntrsctnPool[i].elevation);

		printf("\tnumOfBorderSeg\t= %d\n", (int)pIntrsctnPool[i].numBorderSeg);
		printf("\tborderSegIdx	= %d\n", (int)pIntrsctnPool[i].borderSegIdx);

		printf("\tobjRefIdx     = %d\n", (int)pIntrsctnPool[i].objRefIdx);
		printf("\tintrsctnFlag  = %d\n", (int)pIntrsctnPool[i].intrsctnFlag);

		printf("\tattrIdx		= %d\n", (int)pIntrsctnPool[i].attrIdx);
		printf("\tnumAttr		= %d\n", (int)pIntrsctnPool[i].numAttr);

		printf("\tGrids\n");
		for (j=0; j<cCV_MAX_GRIDS; j++) {
			printf("\t\tgridId         = %d\n", pIntrsctnPool[i].grids[j].gridId);
			printf("\t\tgridbound      = %lf %lf %lf %lf\n", 
				pIntrsctnPool[i].grids[j].minX,
				pIntrsctnPool[i].grids[j].minY,
				pIntrsctnPool[i].grids[j].maxX,
				pIntrsctnPool[i].grids[j].maxY);
			printf("\t\tIntersecting corridors:\n");
			for (k=0; k<cCV_MAX_CRDRS; k++) {
				printf("\t\t\tcrdrId   = %d,  start=%d, end=%d\n", 
						pIntrsctnPool[i].grids[j].intrsctingCrdrs[k].crdrId,
						pIntrsctnPool[i].grids[j].intrsctingCrdrs[k].startCntrlPtIdx,
						pIntrsctnPool[i].grids[j].intrsctingCrdrs[k].endCntrlPtIdx);
			}
		}

		printf("\n");
	}
	printf("\n");
}

void 
DumpLatCntrlPntPool( const cvTLatCntrlPnt* pCntrlPnt, int size )
{
	int i;

	printf(" ---- Contents of lateral control point pool -----\n");
	for (i=1; i<size; i++) {
		printf("LatCntrlPnt %d\n", i);
		printf("\toffset		= %.2f\n", (float)pCntrlPnt[i].offset);
		printf("\theight		= %.2f\n", (float)pCntrlPnt[i].height);
		printf("\tmaterial	= %d\n", (int)pCntrlPnt[i].material);
		printf("\tnormal	:	%.2f	%.2f	%.2f\n",
		  pCntrlPnt[i].normal.i, pCntrlPnt[i].normal.j, pCntrlPnt[i].normal.k);
		printf("\tnormIntrepScale	= %.2f\n", 
							(float)pCntrlPnt[i].normIntrepScale);
	}
	printf("\n");
}

void 
DumpLanePool( const cvTLane* pLanePool, int size )
{
	int i;
	
	printf(" ---- Contents of lane pool -----\n");
	for (i=1; i<size; i++) {
		printf("Lane %d\n", i);
		printf("\tmyId		= %d\n", (int)pLanePool[i].myId);
		printf("\troadId		= %d\n", (int)pLanePool[i].roadId);	
		printf("\twidth		= %.2f\n", (float)pLanePool[i].width);
		printf("\toffset		= %.2f\n", (float)pLanePool[i].offset);
		printf("\tdirection	= %s\n", (pLanePool[i].direction == eNONE)?
								"eNONE": (pLanePool[i].direction == ePOS)?
								"ePOS": "eNEG");
		printf("\tattrIdx		= %d\n", (int)pLanePool[i].attrIdx);
		printf("\tnumAttr		= %d\n", (int)pLanePool[i].numAttr);
		printf("\tpassInfo	= '%c'\n", (char)pLanePool[i].passInfo);
	} 
	printf("\n");
}

void 
DumpAttrPool( const cvTAttr* pAttrPool, int size )
{
	int i;
	printf(" ---- Contents of attr pool ---%d--\n", size);
	for (i=1; i<size; i++){
		printf("Attr %d\n", i);
		printf("\tnameIdx	= %d\n", (int)pAttrPool[i].nameIdx);  
		printf("\tmyId		= %d\n", (int)pAttrPool[i].myId);  
		printf("\tvalue1	= %.2f\n", (float)pAttrPool[i].value1);
		printf("\tvalue2	= %.2f\n", (float)pAttrPool[i].value2);
		printf("\tfrom		= %.2f\n", (float)pAttrPool[i].from);
		printf("\tto		= %.2f\n", (float)pAttrPool[i].to); 
		printf("\tnext		= %d\n", (int)pAttrPool[i].next);
	}
	printf("\n");
}
	

void 
DumpRepObjPool( const cvTRepObj* pRepObjPool, int size )
{
	int i;

	printf(" ---- Contents of repeated object pool -----\n");
	for( i = 1; i < size; i++ ) 
	{
		printf( "Repeated Object %d\n", i );
		/*printf("\tnameIdx       = %d\n", (int)pRepObjPool[i].nameIdx);*/
	}
} 

void 
DumpObj( const cvTObj* pObjPool, int id )
{
	const cvTObj* pO;
	pO = pObjPool + id;
	printf( "\nObject \n");
    printf( "\tid            = %d\n", pO->myId );
    printf( "\tnameIdx       = %s\n", pO->name );
}


void 
DumpObjPool( const cvTObj* pObjPool, int size )
{
	int i;
	const cvTObj* pO;

	printf(" ---- Contents of object pool (size=%d)----\n", size);
	
	for (i=cNUM_DYN_OBJS; i<size; i++) {
		pO = pObjPool + i;

		printf("Object %d\n", i);
		printf("\tid            = %d\n", pO->myId);
		printf("\ttype          = %d\n", pO->type);
		printf("\tphase         = %d\n", pO->phase);
		printf("\tnameIdx       = %s\n", pO->name);
		printf("\tactiveOption  = %d\n", pO->activeOption);
		printf("\tstateBufA :\n");
		printf("\t\tposition    = %.1f %.1f %.1f\n",
				pO->stateBufA.state.anyState.position.x,
				pO->stateBufA.state.anyState.position.y,
				pO->stateBufA.state.anyState.position.z);
		printf("\t\ttangent = %.1f %.1f %.1f\n",
				pO->stateBufA.state.anyState.tangent.i,
				pO->stateBufA.state.anyState.tangent.j,
				pO->stateBufA.state.anyState.tangent.k);
		printf("\txSize    = %.2f\n", pO->attr.xSize);
		printf("\tySize    = %.2f\n", pO->attr.ySize);
	}
}


void 
DumpObjAttrPool( const cvTObjAttr* pPool, int size )
{
	int      i;
	const cvTObjAttr  *pA;
	printf(" ---- Contents of object attribute pool (size=%d)----\n", size);
	
	for (i=cNUM_DYN_OBJS; i<size; i++) {
		pA = pPool + i;

		printf("Attribute %d\n", i);
		printf("\tsolId    = %d\n", pA->solId);
		printf("\thcsmId   = %d\n", pA->hcsmId);
		printf("\txSize    = %.2f\n", pA->xSize);
		printf("\tySize    = %.2f\n", pA->ySize);
	}
}
