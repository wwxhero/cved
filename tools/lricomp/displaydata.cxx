/***************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of Iowa,
 * and The University of Iowa.  All rights reserved.
 *
 * Version:         $Id: displaydata.cxx,v 1.1 2005/06/10 17:43:32 yhe Exp $
 *
 * Author(s):
 * Date:
 *
 * Contains the error checking utilities used by the LRI
 * compiler.
 *
 */



#include<string.h>
#include<cvedstrc.h>


bool FindRoadIdx(
		char*		roadName,
		char*		pCharPool,
		cvTRoad*	pRoadPool,
		int			sizeOfRoadPool,
		int*		pRoadIdx)
{
	int i;
	for (i=1; i<sizeOfRoadPool; i++){
		int nameIdx = pRoadPool[i].nameIdx;
		if (!strcmp(roadName, &pCharPool[nameIdx])){
			*pRoadIdx = i;
			return true;
		}
	}
	return false;
}

#if 0
bool FindIntrsctnIdx(
		char*			intrsctnName,
		char*			pCharPool,
		cvTIntrsctn*	pIntrsctnPool,
		int				sizeOfIntrsctnPool,
		int*			pIntrsctnIdx)
{
	int i;
	for (i=1; i<sizeOfIntrsctnPool; i++){
		int nameIdx = pIntrsctnPool[i].nameIdx;
		if (!strcmp(intrsctnName, &pCharPool[nameIdx])){
			*pIntrsctnIdx = i;
			return true;
		}
	}
	return false;
}
#endif

void DisplayIntrsctn(
		char*			pCharPool,
		cvTIntrsctn*	pIntrsctnPool,
		int				sizeOfIntrsctnPool,
		cvTObjRef*		pObjRefPool)
{
	int i;
	printf("\n************** DisplayIntrsctn ***************\n");
	for (i=1; i<sizeOfIntrsctnPool; i++){
		int firstObjRef = pIntrsctnPool[i].objRefIdx;
		if (firstObjRef == 0)
			continue;
		int nameIdx = pIntrsctnPool[i].nameIdx;
		printf("\n Intrsctn: %s ", &pCharPool[nameIdx]);
		//printf("\n\t\t objId = %d", pObjRefPool[firstObjRef].objId);
		printf("\n\t\t objId = %d \tintrsctnId %d", pObjRefPool[firstObjRef].objId, i);
		int nextRef = pObjRefPool[firstObjRef].next;
		while(nextRef != 0) {
			printf("\n\t\t objId = %d \tintrsctnId %d", pObjRefPool[nextRef].objId, i);
			nextRef = pObjRefPool[nextRef].next;
		}
	}
	printf("\n*************** End of DisplayIntrsctn *****************\n");
}


void PrintLinearDist(
		int				firstPnt, 
		int				numOfPnt, 
		cvTCntrlPnt*	pCntrlPntPool,
		cvTObjRef*		pObjRefPool)
{
	int i;
	printf("\n-------- linear distance of each control point ----------\n");
	printf("\n the first is : %d ", firstPnt);
	for (i=firstPnt; i<(firstPnt+numOfPnt); i++){
		printf("\n pt[%d]\t=\t%f\t%f", 
								i - firstPnt,
								pCntrlPntPool[i].cummulativeLinDist,
								pCntrlPntPool[i].location.x);
		printf("\t if repeated : %s", 
		(pCntrlPntPool[i].cntrlPntFlag & eREP_OBJ_FLAG) ? "true" : "false");

		int firstObjRef = pCntrlPntPool[i].objRefIdx;

		if (firstObjRef!=0){
			printf("\n\t\t objId = %d", pObjRefPool[firstObjRef].objId);
			int nextRef = pObjRefPool[firstObjRef].next;
			while(nextRef != 0) {
				printf("\n\t\t objId = %d", pObjRefPool[nextRef].objId);
				nextRef = pObjRefPool[nextRef].next;
			}
		}
	}
	printf("\n-------------- end of printing of linear dist -----------\n");
}


void PrintRepData(
		int			roadIdx, 
		cvTRoad*	pRoadPool, 
		cvTRepObj*	pRepObjPool,
		char*		pCharPool)
{
	int i = pRoadPool[roadIdx].numRepObj;
	printf("\n Road Name %s", &pCharPool[pRoadPool[roadIdx].nameIdx]);
	printf("\n There're %d kinds of repobj \n", i);
	int j;
	for (j = 0; j<i; j++){
		int repObjIdx = pRoadPool[roadIdx].repObjIdx + j;
		printf("\n %dth \t repobjIdx %d \t objIdx %d", 
						j,
						repObjIdx,
						pRepObjPool[repObjIdx].objId);
		printf("\n start %f latDist %f period %f end %f",
						pRepObjPool[repObjIdx].start,
						pRepObjPool[repObjIdx].latdist,
						pRepObjPool[repObjIdx].period,
						pRepObjPool[repObjIdx].end);
						
	}
}
	

void DisplayObjDataOfRoad(
		char*			roadName,
		char*			pCharPool,
		cvTRoad*		pRoadPool,
		int				sizeOfRoadPool,
		cvTCntrlPnt*	pCntrlPntPool,
		cvTRepObj*		pRepObjPool,
		cvTObjRef*		pObjRefPool)
{
	int roadIdx;
	if (!FindRoadIdx(roadName, pCharPool, pRoadPool, sizeOfRoadPool, &roadIdx)){
		printf("\n can't find the road whose name is %s \n", roadName);
		exit(1);
	}
	
	int firstPnt = pRoadPool[roadIdx].cntrlPntIdx;
	int numOfPnt = pRoadPool[roadIdx].numCntrlPnt;
	PrintRepData(roadIdx, pRoadPool, pRepObjPool, pCharPool);
	PrintLinearDist(firstPnt, numOfPnt, pCntrlPntPool, pObjRefPool);
}

#if 0
void DisplayObjDataOfIntrsctn(
		char*			intrsctnName,
		char*			pCharPool,
		cvTIntrsctn*	pIntrsctnPool,
		int				sizeOfIntrsctnPool,
		cvTRepObj*		pRepObjPool,
		cvTObjRef*		pObjRefPool)
{
	int intrsctnIdx;
	if (!FindIntrsctnIdx(
				intrsctnName, 
				pCharPool, 
				pIntrsctnPool, 
				sizeOfIntrsctnPool, intrsctnIdx)){
		printf("\n can't find the intrsctn whose name is %s \n", intrsctnName);
		exit(1);
	}
}
#endif

void DumpObjPool(cvTObj* pObjPool, int sizeOfObjPool)
{
	int i;
	printf("\n========== Dump Obj Pool ===========\n");
	for (i=cNUM_DYN_OBJS; i<sizeOfObjPool; i++){
		printf("\n myId = %d", pObjPool[i].myId);
		printf("\t name = %s", pObjPool[i].name);
	}
	printf("\n");
}

void DumpObjRefPool(cvTObjRef* pObjRefPool, int sizeOfObjRefPool)
{
	int i;
	printf("\n========== Dump ObjRef Pool ===========\n");
	int max = sizeOfObjRefPool - 300; // which is cOBJ_REF_POOL_MEM_RES;
	for ( i = 0; i< max; i++){
		printf("\n objid %d\t next %d", pObjRefPool[i].objId, pObjRefPool[i].next);
	}
	printf("\n========== end of dump objref pool =========\n");
}

void DumpRepObjPool(cvTRepObj* pRepObjPool, int sizeOfRepObjPool)
{
	int i;
	printf("\n========== Dump RepObj Pool ===========\n");
	for (i=0; i<sizeOfRepObjPool; i++){
		printf("\n %d th", i);
		printf("\t myId = %d", pRepObjPool[i].objId);
		printf("\t lat = %.1f", pRepObjPool[i].latdist);
		printf(" start = %.1f", pRepObjPool[i].start);
		printf("\t period = %.1f", pRepObjPool[i].period);
		printf("\t end = %.1f", pRepObjPool[i].end);
	}
	printf("\n");
}








