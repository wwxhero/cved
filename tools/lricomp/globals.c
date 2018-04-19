/*****************************************************************************
 * (C) Copyright 1999 by National Advanced Driving Simulator, the University
 *     of Iowa.  All rights reserved.
 * 
 * Version: 		$Id: globals.c,v 1.1 2005/06/10 17:43:33 yhe Exp $
 * 
 * Author:			
 * 
 * Date:			
 * 
 * Description:		  
 *	
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "parser.h"
#include "dbgdump.h"


cvTRoad			*pRoadPool = NULL;
int				sizeOfRoadPool = 0;

cvTLane 		*pLanePool = NULL;
int      		sizeOfLanePool = 0;

cvTCntrlPnt		*pCntrlPntPool = NULL;
int				sizeOfCntrlPntPool = 0;
int             storageOfCntrlPntPool = 0;

char			*pCharPool = NULL;
int				sizeOfCharPool = 0;

cvTAttr			*pAttrPool = NULL;
int				sizeOfAttrPool = 0;

cvTRepObj		*pRepObjPool = NULL;
int				sizeOfRepObjPool = 0;

cvTLatCntrlPnt	*pLatCntrlPntPool = NULL;
int				sizeOfLatCntrlPntPool = 0;

cvTIntrsctn		*pIntrsctnPool = NULL;
int				sizeOfIntrsctnPool = 0;

cvTBorderSeg	*pBorderSegPool = NULL;
int				sizeOfBorderSegPool = 0;

cvTCrdr			*pCrdrPool = NULL;
int				sizeOfCrdrPool = 0;

cvTCrdrCntrlPnt	*pCrdrCntrlPntPool = NULL;
int				sizeOfCrdrCntrlPntPool = 0; 

cvTHldOfs		*pHldOfsPool = NULL;
int				sizeOfHldOfsPool = 0;

cvTObj          *pObjPool = NULL;
int             sizeOfObjPool = 0;

/* cvTObjAttr      *pObjAttrPool = NULL; */
/* int             sizeOfObjAttrPool = 0; */

cvTRoadPiece	*pRoadPiecePool = NULL;
int				sizeOfRoadPiecePool = 0;

cvTObjRef*		pObjRefPool = NULL;
int				sizeOfObjRefPool = 0;

cvTDynObjRef	*pDynObjRefPool = NULL;
int				sizeOfDynObjRefPool = 0;

cvTRoadRef		*pRoadRefPool = NULL;
int				sizeOfRoadRefPool = 0;

cvTIntrsctnRef	*pIntrsctnRefPool = NULL;
int				sizeOfIntrsctnRefPool = 0;

cvTCrdrMrgDst	*pCrdrMrgDstPool = NULL;
int				sizeOfCrdrMrgDstPool = 0;

char*		reservedAttrNames[cCV_NUM_RESERVED_ATTR] = {
	"SpeedLimit", "LaneChangeRules", "PassingRules", 
	"Merge", "DrivingLane", "BicycleLane",
	"TurnLane", "ExitOnlyLane", 
	"VehicleRestrictionLane", "EmergencyLane",
	"HOVLane", "Highway", "Interstate", "CityRoad", 
	"RuralRoad", "InterstateOnRamp", 
	"InterstateOffRamp"
};

int         gTestLex         = 0;
int         gDebugDump       = 0;
int         gNoSplineCorrect = 0;
float		gCrdrTolerance   = 0.5;
float		gGapTolerance    = 0.5;
float       gHermiteSplineScale = 0.25;
int         gOvrdVersionMaj = 0;
int         gOvrdVersionMin = 0;
int         gOvrdVersion1   = 0;
int         gOvrdVersion2   = 0;
int         gOvrdVersion3   = 0;
int			gNoMinusInNames = 0;