/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: parser.y,v 1.2 2012/04/20 21:11:15 iowa\yefeihe Exp $
 *
 * Author(s):    Jennifer Galvez, Yiannis Papelis
 * Date:         July, 1998
 *
 * Description:  Parser for lri compiler.
 *
 ****************************************************************************/

%{
/*****************************************************************************
 *		THIS FILE IS AUTOMATICALLY GENERATED FROM parser.y
 *		 Version:      $Id: parser.y,v 1.2 2012/04/20 21:11:15 iowa\yefeihe Exp $		 
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <malloc.h>
#include <math.h>

#include "parser.h"

extern int  yylex(void);
extern int yyerror(const char *);
extern char *gettxt(const char *, const char *);


/*************************************************************************
 *
 * Global storage for the parser
 *
 ****************************************************************************/
THeader         gHeader;
TRoads         *gpRoadList;
TIntersection  *gpInterList;
TElevMap       *gpElevMaps;
TAttrDict      gAttrDict = { 0 };
TLatCurve      *gpLatCurves;

/* objects are not managed by a linked list because we
 * want them to be stored in the order they appear.  So we use
 * an "expanding" array.
 */
TObject        *gpObjects    = NULL;
int             gObjectCount = 0;
int             gObjectSpace = 0;

%}

%union
{
	int      			ival;
	double   			dval;
	char     			*strval;
	unsigned 			uval;
	TLatCurve			latcurvedef;
	TLatCurve*			latcurveptr;
	TLatCpoint			latcpointdef;
	TCurveDef*			curvedefptr;
	TAttribute			attributedef;
	TAttribute*			attributeptr;
	TLaneDef 			lanedef;
	TLaneList			lanelistdef;
	TControlPoint		controlpointdef;
	TControlPoint*  	controlpointptr;
	TCpointList			cpointlistdef;
	TCpointInfo			cpointinfodef;
	TCpointInfo*		cpointinfoptr;
	TLaneWidth			lanewidthdef;
	TLaneWidth*			lanewidthptr;
	TIntersection*     intersectionsptr;
	TElevMap*           elevmapptr;
	TElevMap            elevmap;
	TElevMap2*          elevmap2ptr;
	TElevInfo           elevinfodef;
	TRoadName*          roadptr;
	TBorderPt           borderptdef;
	TBorder             borderdef;
	TCrdr               crdrdef;
	TCrdr               *crdrdefptr;
	THoldOfs*           holdofsptr;
	THoldOfs            holdofs;
	TCrdrCurveList      crdrcurvelist;
	TCrdrCurve          crdrcurvedef;
	TCrdrLineInfo*      crdrlineinfoptr;
	THoldOfsInfo*       holdofsinfoptr;
	THoldOfsInfo        holdofsinfo;
	TRepObj*            repobjptr;
	TObject             objdef;
}

%token 			LRI_ROADS			500 
%token 			LRI_LANES			501
%token 			LRI_POSITIVE		503  
%token 			LRI_NEGATIVE		504  
%token 			LRI_CURLY_OPEN		505
%token 			LRI_CURLY_CLOSE		506
%token <ival> 	LRI_DIRECTION    	507
%token <strval> LRI_IDENTIFIER 		513
%token 			LRI_LAT_CURVES	  	514
%token 			LRI_LATCURVE	  	515
%token 			LRI_LANEWIDTH	  	516
%token 			LRI_ATTR			517
%token 			LRI_LONGCURVE	  	518
%token 			LRI_SEMI			519
%token 			LRI_INTERSECTIONS 	520
%token 			LRI_ELEVMAP			521
%token 			LRI_BORDER			522
%token 			LRI_CRDR			523
%token 			LRI_HOLDOFS			524
%token 			LRI_CRDR_CURVE		525
%token 			LRI_HOLDLINE		526
%token 			LRI_HOLDSIGN		527
%token 			LRI_EQUALTO			528
%token 			LRI_LINES			529
%token <ival> 	LRI_LFLAG			530
%token <strval> LRI_CRDR_REASON		531
%token <strval> LRI_LSTYLE			532
%token <dval>   LRI_REAL            533
%token			LRI_REPOBJS         534
%token 			LRI_ALIGNED         535
%token			LRI_OBJECTS			536
%token			LRI_PLANT			537
%token          LRI_ZDOWN           539
%token          LRI_SOLCHECKSUM     540
%token          LRI_HEADER          541
%token <strval> LRI_QUOTED_STRING   542
%token          LRI_COMMENT         543
%token          LRI_ATTR_DICT       544

/*
 * define types for nonterminals
 */
%type <latcurvedef> curve_def 
%type <latcurveptr> lateral_curve_def
%type <latcpointdef> lat_cpoint
%type <curvedefptr> lat_cpoints_def
%type <attributedef> attribute, inter_attr
%type <attributeptr> road_attributes attribute_def inter_attr_list
%type <attributeptr> inter_attr_spec
%type <lanedef> road_lane
%type <lanelistdef> road_lanes
%type <controlpointdef> control_point 
%type <cpointlistdef> control_points
%type <cpointinfodef> cpoint_info
%type <cpointinfoptr> cpoint_infolist 
%type <lanewidthdef> width_def
%type <strval> control_point_name
%type <elevmapptr> elev_map
%type <elevmapptr> elev_map_list
%type <elevmap2ptr> elev_map_part
%type <intersectionsptr> intersec_list intersec
%type <elevinfodef> elev_info
%type <roadptr> road_names
%type <borderptdef> border_pt
%type <borderdef> border_def border_pt_list
%type <crdrdef> crdr
%type <crdrdefptr> crdr_list
%type <holdofsptr> hold_ofs_list
%type <holdofs> hold_ofs
%type <crdrcurvelist> crdr_curve
%type <crdrcurvedef> crdr_curve_point
%type <crdrlineinfoptr> crdr_line_info
%type <holdofsinfo> hold_ofs_info
%type <repobjptr> repeat_objs rep_obj_list rep_obj
%type <objdef> obj
%type <attributedef> crdr_attr
%type <attributeptr> crdr_attr_list
%type <attributeptr> crdr_attr_spec 

%%

/************************************************************************/
lri : 
		lri_header roads_list intersec_defs obj_defs attr_dict

	| 	lri_header roads_list error obj_defs
	{
		fprintf(stderr, "in intersection list definition.\n");
		exit(-1);
	}

	| 	lri_header error intersec_defs obj_defs attr_dict
	{
		fprintf(stderr, "in road list definition.\n");
		exit(-1);
	}

	| 	lri_header roads_list intersec_defs error attr_dict
	{
		fprintf(stderr, "in object list definition.\n");
		exit(-1);
	}

	| 	error roads_list intersec_defs obj_defs attr_dict
	{
		fprintf(stderr, "in LRI header.\n");
		exit(-1);
	}
	;

/************************************************************************/
lri_header :
		LRI_HEADER LRI_CURLY_OPEN header_items LRI_CURLY_CLOSE

		|
		;

/************************************************************************/
header_items :
		header_items header_item
		{
		}

		| header_item
		{
		}

		|
		;

/************************************************************************/
header_item :
		LRI_ZDOWN  LRI_EQUALTO LRI_REAL
		{
			if ( fabs($3) < 0.01 )
				gHeader.zdown = 0;
			else
				gHeader.zdown = 1;
		}

		| LRI_SOLCHECKSUM LRI_EQUALTO LRI_REAL
		{
			gHeader.solCheckSum = (unsigned int)$3;
		}

		| LRI_COMMENT LRI_EQUALTO LRI_QUOTED_STRING
		{
			gHeader.pComment = $3;
		}
		;

/************************************************************************/
roads_list	: 	
		LRI_ROADS LRI_CURLY_OPEN   lateral_curve_list 
				road_def_list  LRI_CURLY_CLOSE  

	|	LRI_ROADS LRI_CURLY_OPEN road_def_list LRI_CURLY_CLOSE
	;


/************************************************************************/
lateral_curve_list	: 
		LRI_LAT_CURVES LRI_CURLY_OPEN lateral_curve_def LRI_CURLY_CLOSE
		{
		gpLatCurves = $3;
		}
		;


/************************************************************************/
lateral_curve_def	: 
		lateral_curve_def LRI_IDENTIFIER LRI_CURLY_OPEN curve_def 
				LRI_CURLY_CLOSE
		{
			TLatCurve* pTemp = calloc(1, sizeof(TLatCurve));

			*pTemp = $4;
			strncpy (pTemp->curveName, $2, cMAX_WORDLENGTH);
			pTemp->pNext = $$;
			$$ = pTemp;
		}

	| 	LRI_IDENTIFIER LRI_CURLY_OPEN curve_def LRI_CURLY_CLOSE
		{
			TLatCurve* pTemp = calloc(1, sizeof(TLatCurve));

			*pTemp = $3;
			strncpy (pTemp->curveName, $1, cMAX_WORDLENGTH);
			$$ = pTemp;
		}
		;


/************************************************************************/
curve_def	: 
		LRI_REAL lat_cpoints_def 
		{
			TCurveDef *pHead=NULL, *pT;

			$$.curveWidth = $1;

			/* the list of points is in $2 but we need */
			/* to flip its direction */

			for ( pT = $2; pT; pT = pT->pNext ) {
				TCurveDef *pNew = calloc(1, sizeof(TLatCurve));

				*pNew = *pT;
				pNew->pNext = pHead;
				pHead = pNew;
			}
			$$.pCurveDef = pHead;
		}
		;


/************************************************************************/
lat_cpoints_def	: 
		lat_cpoints_def lat_cpoint lat_cpoint
		{
            TCurveDef* pTemp 	  = calloc(1, sizeof(TCurveDef));
            pTemp->latCpoint1 	  = $2;
            pTemp->latCpoint2 	  = $3;
            pTemp->pNext          = $$;
            $$ = pTemp;
        }

    	| lat_cpoint lat_cpoint
        {
            TCurveDef* pTemp= calloc(1, sizeof(TCurveDef));
            pTemp->latCpoint1 = $1;
            pTemp->latCpoint2 = $2;
            $$ = pTemp;
        }
		;


/************************************************************************/
lat_cpoint	: 
		LRI_REAL LRI_REAL LRI_REAL
		{
			$$.height   = $1;
			$$.offset   = $2;
			$$.material = (int)$3;
		}
		;


/************************************************************************/
road_def_list	: 
		road_def_list LRI_IDENTIFIER LRI_IDENTIFIER LRI_IDENTIFIER 
				LRI_CURLY_OPEN road_attributes repeat_objs
                LRI_LANES LRI_CURLY_OPEN  road_lanes LRI_CURLY_CLOSE
                LRI_LONGCURVE LRI_CURLY_OPEN control_points LRI_CURLY_CLOSE 
				LRI_CURLY_CLOSE
		{
			TRoads* p = calloc(1, sizeof(TRoads));

			strncpy(p->roadName, $2, cMAX_WORDLENGTH);
			strncpy(p->intersection1, $3, cMAX_WORDLENGTH);
			strncpy(p->intersection2, $4, cMAX_WORDLENGTH);
			p->pAttribute   = $6;
			p->pRepObjs     = $7;
			p->lanelist     = $10;
			p->longitCurve  = $14;
			p->pNext        = gpRoadList;
			gpRoadList 	    = p;
		}

		| 	LRI_IDENTIFIER LRI_IDENTIFIER LRI_IDENTIFIER 
	  		LRI_CURLY_OPEN road_attributes repeat_objs
      		LRI_LANES LRI_CURLY_OPEN road_lanes LRI_CURLY_CLOSE
      		LRI_LONGCURVE LRI_CURLY_OPEN control_points  LRI_CURLY_CLOSE 
			LRI_CURLY_CLOSE
		{
			TRoads* p = calloc(1, sizeof(TRoads));

			strncpy(p->roadName, $1, cMAX_WORDLENGTH);
			strncpy(p->intersection1, $2, cMAX_WORDLENGTH);
			strncpy(p->intersection2, $3, cMAX_WORDLENGTH);
			p->pAttribute   = $5;
			p->pRepObjs     = $6;
			p->lanelist     = $9;
			p->longitCurve  = $13;
			p->pNext        = gpRoadList;
			gpRoadList 	    = p;
        }
      	;


/************************************************************************/
repeat_objs :
		LRI_REPOBJS LRI_CURLY_OPEN rep_obj_list LRI_CURLY_CLOSE
		{
			$$ = $3;
		}

		|
		{
			$$ = NULL;
		}
		;

/************************************************************************/
rep_obj_list :
		rep_obj_list rep_obj
		{
			/*$$->pNext = $2;*/
			$2->pNext = $$;
			$$ = $2;
		}

		| rep_obj
		{
			$$ = $1;
		}
		;

/************************************************************************/
rep_obj:
		LRI_IDENTIFIER LRI_REAL LRI_REAL LRI_REAL LRI_REAL
		{
			TRepObj *pTem = calloc(1, sizeof(TRepObj));

			strcpy(pTem->name, $1);
			pTem->latdist  = $2;
			pTem->start    = $3;
			pTem->period   = $4;
			pTem->end      = $5;
			pTem->alligned = 0;
			$$ = pTem;
		}

		|
		LRI_IDENTIFIER LRI_REAL LRI_REAL LRI_REAL LRI_REAL LRI_ALIGNED
		{
			TRepObj *pTem = calloc(1, sizeof(TRepObj));

			strcpy(pTem->name, $1);
			pTem->latdist  = $2;
			pTem->start    = $3;
			pTem->period   = $4;
			pTem->end      = $5;
			pTem->alligned = 1;
			$$ = pTem;
		}
		;
		

/************************************************************************/
road_attributes	: 
		LRI_ATTR attribute_def 
		{
			$$ = $2;
		}

		|
		{
			$$ = NULL;
		}
		;


/************************************************************************/
attribute_def	: 
		attribute_def attribute
		{
			TAttribute* pAttr = calloc(1, sizeof(TAttribute));
			*pAttr = $2;
			pAttr->pNext = $$;
			$$ = pAttr;
		}

		| attribute
		{
			TAttribute* pAttr = calloc(1, sizeof(TAttribute));
            *pAttr = $1;
			$$ = pAttr;
		}
		;


/************************************************************************/
attribute	: 
		LRI_REAL LRI_REAL LRI_REAL LRI_REAL LRI_REAL LRI_REAL
		{
			$$.inum 	= (int)$1;
			$$.laneMask = (int)$2;
			$$.attrNum1	= $3;
			$$.attrNum2	= $4;
			$$.from	    = $5;
			$$.to      	= $6;
		}
		;


/************************************************************************/
road_lanes	: 
		road_lanes road_lane
		{
			$$.lanes[$$.n_lanes] = $2;
			$$.n_lanes++;
		}

		| road_lane
		{
			$$.n_lanes  = 1;
			$$.lanes[0] = $1;
		}
		;


/************************************************************************/
road_lane	:  
		LRI_REAL  LRI_DIRECTION
		{
			$$.width = $1;
			if ( $2 == LRI_POSITIVE )
				$$.direction = 'p';
			else
				$$.direction = 'n';
		}
		;


/************************************************************************/
control_points	: 
		control_points control_point
		{
			if ($$.n_cpoints == $$.total_cpoints)
			{
				int newSize;
				TControlPoint* temp;
				newSize = $$.total_cpoints + cINITIAL_CPOINTS;
				temp 	= calloc(newSize, sizeof(TControlPoint));
				memcpy(temp, $$.pCpointList, 
									$$.total_cpoints*sizeof(TControlPoint));
				$$.total_cpoints 			 = newSize;
				free ($$.pCpointList);
				$$.pCpointList 				 = temp;
				$$.pCpointList[$$.n_cpoints] = $2;
				$$.n_cpoints++;
			}
			else
			{
				$$.pCpointList[$$.n_cpoints] = $2;
				$$.n_cpoints++;
			}
		}

		| control_point
		{
			$$.pCpointList 	  = (TControlPoint*)calloc(cINITIAL_CPOINTS,
										sizeof(TControlPoint));
			$$.total_cpoints  = cINITIAL_CPOINTS;
			$$.n_cpoints	  = 1;
			$$.pCpointList[0] = $1;
		}

		| error
		{ 
			puts("Error in definition of control point.\n"); 
			return 1;
		}
		;


/************************************************************************/
control_point	: 
				control_point_name LRI_REAL LRI_REAL LRI_REAL LRI_REAL 
				LRI_REAL LRI_REAL cpoint_infolist LRI_SEMI
		{
			if ( $1 )
				strncpy($$.cpointName, $1, cMAX_WORDLENGTH);
            $$.x           = $2;
            $$.y           = $3;
            $$.z           = $4;
            $$.i           = $5;
            $$.j           = $6;
            $$.k           = $7;
            $$.pCpointInfo = $8;
		}

		|  control_point_name LRI_REAL LRI_REAL LRI_REAL LRI_REAL 
				LRI_REAL LRI_REAL 
		{
			if ( $1 )
				strncpy($$.cpointName, $1, cMAX_WORDLENGTH);
            $$.x          	 = $2;
            $$.y          	 = $3;
            $$.z          	 = $4;
            $$.i          	 = $5;
            $$.j          	 = $6;
            $$.k          	 = $7;
            $$.pCpointInfo 	 = NULL;
		}
		;


/************************************************************************/
cpoint_infolist	: 
		cpoint_infolist cpoint_info
		{
			TCpointInfo* pCP = calloc(1, sizeof(TCpointInfo));

			*pCP = $2;
			pCP->pNext = $$;
			$$ = pCP;
		}

		| cpoint_info
		{
			TCpointInfo* pCP = calloc(1, sizeof(TCpointInfo));

			*pCP = $1;
			$$ = pCP;
		}
		;

/************************************************************************/
cpoint_info	: 
		LRI_LATCURVE LRI_IDENTIFIER 
		{
			memset(&$$, 0, sizeof($$));
			strncpy($$.curveName, $2, cMAX_WORDLENGTH);
		}

		| LRI_ATTR attribute_def 
		{
			memset(&$$, 0, sizeof($$));
			$$.pCpointAttribute = $2;
		}

		| LRI_LANEWIDTH width_def 
		{
			memset(&$$, 0, sizeof($$));
			$$.cpointWidth = $2;
		}
		;


/************************************************************************/
control_point_name :
		LRI_IDENTIFIER 
		{
			$$ = $1;
		}

		|
		{
			$$ = NULL;
		}
		;

/************************************************************************/
width_def	: 
		width_def LRI_REAL
		{
			$$.lanewidth[$$.n_lanes] = $2;
			$$.n_lanes++;
		}

		| LRI_REAL
		{
			$$.n_lanes 		= 1;
			$$.lanewidth[0] = $1; 
		}
		;

/************************************************************************/
/* INTERSECTIONS */
/************************************************************************/

intersec_defs	: 
		LRI_INTERSECTIONS LRI_CURLY_OPEN elev_map_list  intersec_list  
			LRI_CURLY_CLOSE 
		{
			gpElevMaps            = $3;
			gpInterList 		  = $4;
		}

		| LRI_INTERSECTIONS LRI_CURLY_OPEN  intersec_list LRI_CURLY_CLOSE
		{
			gpElevMaps	 		  = NULL;
			gpInterList 		  = $3;
		}

		| LRI_INTERSECTIONS LRI_CURLY_OPEN  error intersec_list LRI_CURLY_CLOSE
		{
			fprintf(stderr, "in elevation map list.\n");
			exit(-1); 
		}

		| LRI_INTERSECTIONS LRI_CURLY_OPEN  elev_map_list error LRI_CURLY_CLOSE
		{
			fprintf(stderr, "in intersection list.\n");
			exit(-1); 
		}
		;


/************************************************************************/
elev_map_list :
		elev_map_list elev_map
		{
			$2->pNextElevMap = $$;
			$$ = $2;
		}

		| elev_map
		{
			$$ = $1;
		}
		;

/************************************************************************/
elev_map	: 
		LRI_ELEVMAP LRI_IDENTIFIER LRI_REAL LRI_REAL LRI_REAL LRI_CURLY_OPEN 
				elev_map_part LRI_CURLY_CLOSE
		{
			$$ = calloc(1, sizeof(TElevMap));
			strncpy ($$->elevMapName, $2, cMAX_WORDLENGTH);
			$$->n_rows 		= (int)$3;
			$$->n_cols 		= (int)$4;
			$$->res          = $5;
			$$->pElevMap2 	= $7->pElevMap2Head;
			$$->pNextElevMap  = NULL;
			printf("Processed elevation map %s\n",$$->elevMapName);
		}
		;


/************************************************************************/
elev_map_part	: 
		elev_map_part LRI_REAL LRI_REAL
		{
			TElevMap2* pTemp	 = malloc(sizeof(TElevMap2));

			/* add to end of list; list contains at least one item */
			/* we add to the tail of the list because order
			 * is important
			 */
			TElevMap2* pTrav;
			pTrav = $$->pElevMap2Tail;

			pTemp->z 			 = $2;
			pTemp->materialRef	 = (int)$3;
			pTemp->pNextElevMap2 = NULL;

			pTrav->pNextElevMap2 = pTemp;
			$$->pElevMap2Tail = pTemp;
		}
	
		| LRI_REAL LRI_REAL
		{
			TElevMap2ListInfo* pElevMapListInfo = malloc(sizeof(TElevMap2ListInfo));
			TElevMap2* pTemp	 = malloc(sizeof(TElevMap2));
			pTemp->z 			 = $1;
			pTemp->materialRef	 = (int)$2;
			pTemp->pNextElevMap2 = NULL;
			pElevMapListInfo->pElevMap2Head = pTemp;
			pElevMapListInfo->pElevMap2Tail = pTemp;
			$$ 					 = pElevMapListInfo;
		}
		;


/************************************************************************/
intersec_list	: 
		intersec_list intersec
		{
			$2->pNext = $$;
			$$ = $2;
		}

		| intersec  
		{
			$$ = $1;
		}
		;

/************************************************************************/
/* add inter_attr_spec before LRI_CURLY_CLOSE */
intersec :
		LRI_IDENTIFIER  elev_info LRI_CURLY_OPEN LRI_ROADS road_names  
				border_def crdr_list inter_attr_spec LRI_CURLY_CLOSE
		{
			$$ = calloc(1, sizeof(TIntersection));

			strncpy($$->name, $1, cMAX_WORDLENGTH);
			$$->elevInfo 		  = $2;
			$$->pRoadNames 		  = $5;
			$$->Border		 	  = $6;
			$$->pCrdr 			  = $7;
			$$->pAttrs            = $8;
		}

		| LRI_IDENTIFIER  elev_info LRI_CURLY_OPEN 
				border_def crdr_list inter_attr_spec LRI_CURLY_CLOSE
		{
			$$ = calloc(1, sizeof(TIntersection));

			strncpy($$->name, $1, cMAX_WORDLENGTH);
			$$->elevInfo 		  = $2;
			$$->Border		 	  = $4;
			$$->pCrdr 			  = $5;
			$$->pAttrs            = $6;
		}

		| LRI_IDENTIFIER  error LRI_CURLY_OPEN LRI_ROADS road_names  
				border_def crdr_list inter_attr_spec LRI_CURLY_CLOSE
		{
			fprintf(stderr, "in elevation specification.\n");
			exit(-1);
		}

		| LRI_IDENTIFIER  elev_info LRI_CURLY_OPEN LRI_ROADS error  
				border_def crdr_list inter_attr_spec LRI_CURLY_CLOSE
		{
			fprintf(stderr, "in definition of '%s'.\n", $1);
			exit(-1);
		}
		;


/************************************************************************/
road_names	: 
		road_names LRI_IDENTIFIER
		{
			TRoadName* pTemp = malloc(sizeof(TRoadName));

			strncpy (pTemp->roadName, $2, cMAX_WORDLENGTH);
			pTemp->pNextRoadName = $$;
			$$ = pTemp;
		}

		| LRI_IDENTIFIER
		{
			TRoadName* pTemp = calloc(1, sizeof(TRoadName));

			strncpy (pTemp->roadName, $1, cMAX_WORDLENGTH);
			$$ = pTemp;
		}
		;


/************************************************************************/
elev_info	: 
		LRI_REAL
		{
			$$.elevMapName[0] = '\0';
			$$.height = $1;
		}

		| LRI_IDENTIFIER LRI_REAL LRI_REAL
		{
			strncpy ($$.elevMapName, $1, cMAX_WORDLENGTH);
			$$.xorig = $2;
			$$.yorig = $3;
		}
		;

/************************************************************************/
inter_attr_spec	: 
		LRI_ATTR inter_attr_list
		{
			$$ = $2;
		}

		|
		{
			$$ = NULL;
		}
		;

/************************************************************************/
inter_attr_list	: 
		inter_attr_list inter_attr
		{
			TAttribute *pNew = malloc(sizeof(TAttribute));

			*pNew = $2;
			pNew->pNext = $$;
			$$ = pNew;
		}

		|
		inter_attr
		{
			$$ = malloc(sizeof(TAttribute));
			*$$ = $1;
		}
		;
		
/************************************************************************/
inter_attr	: 
			LRI_REAL LRI_REAL LRI_REAL 
		{
			$$.inum     = (int)$1;
			$$.laneMask = 0;
			$$.attrNum1 = $2;
			$$.attrNum2 = $3;
			$$.from     = -1;
			$$.to       = -2;
			$$.pNext    = NULL;
		}
		;
		
/************************************************************************/
border_def	: 
		LRI_BORDER border_pt_list
		{
			$$ = $2;
		}

		|
		{
			$$.nPts = 0;
		}
		;

/************************************************************************/
border_pt_list:
		border_pt_list border_pt
		{
			if ( $$.nPts == cMAX_BORDER_PTS ) {
				char  err[256];

				fprintf(stderr, "Number of border points exceeds internal "
					"maximum.  Recompile parser \nwith higher value "
					"for cMAX_BORDER_PTS (currently %d).\n", cMAX_BORDER_PTS);
				exit(-1);
			}
			$$.list[$$.nPts++] = $2;
		}

		| border_pt
		{
			$$.nPts = 1;
			$$.list[0] = $1;
		}
		;


/************************************************************************/
border_pt : 
		LRI_REAL LRI_REAL LRI_LFLAG
		{
			$$.borderX = $1;
			$$.borderY = $2;
			$$.lflag   = $3;
		}
		;


/************************************************************************/
crdr_list :
			crdr_list crdr
		{
			TCrdr *pC = malloc(sizeof(*pC));

			*pC = $2;
			pC->pNext = $$;
			$$ = pC;
		}

		|	crdr
		{
			TCrdr *pC = calloc(1, sizeof(*pC));

			*pC = $1;
			$$ =  pC;
		}

		|
		{
			$$ = NULL;
		}
		;


/************************************************************************/
crdr : 
		 LRI_CRDR LRI_IDENTIFIER LRI_REAL LRI_IDENTIFIER LRI_REAL 
				LRI_CURLY_OPEN  hold_ofs_list LRI_CRDR_CURVE LRI_CURLY_OPEN 
				crdr_curve LRI_CURLY_CLOSE crdr_attr_spec LRI_CURLY_CLOSE
		{
			strncpy ($$.roadName1, $2, cMAX_WORDLENGTH);
			$$.crdrLane1 	  = (int)$3;
			strncpy ($$.roadName2, $4, cMAX_WORDLENGTH);
			$$.crdrLane2 	  = (int)$5;
			$$.pHoldOfs	 	  = $7;
			$$.CrdrCurveList  = $10;
			$$.pAttrs         = $12;
		}

		|
		 LRI_CRDR LRI_IDENTIFIER LRI_REAL LRI_IDENTIFIER LRI_REAL 
				LRI_CURLY_OPEN  LRI_CRDR_CURVE LRI_CURLY_OPEN 
				crdr_curve LRI_CURLY_CLOSE crdr_attr_spec LRI_CURLY_CLOSE
		{
			strncpy ($$.roadName1, $2, cMAX_WORDLENGTH);
			$$.crdrLane1 	  = (int)$3;
			strncpy ($$.roadName2, $4, cMAX_WORDLENGTH);
			$$.crdrLane2 	  = (int)$5;
			$$.pHoldOfs	 	  = NULL;
			$$.CrdrCurveList  = $9;
			$$.pAttrs         = $11;
		}
		;


/************************************************************************/
hold_ofs_list : 
		hold_ofs_list hold_ofs
		{
			THoldOfs *pH = malloc(sizeof(*pH));

			*pH = $2;
			pH->pNextHoldOfs = $$;
			$$ = pH;
		}

		|  hold_ofs
		{
			THoldOfs *pH = calloc(1, sizeof(*pH));

			*pH = $1;
			$$ =  pH;
		}
		;
		
/************************************************************************/
hold_ofs : 
		LRI_HOLDOFS LRI_REAL LRI_REAL LRI_CRDR_REASON 
				hold_ofs_info hold_ofs_info
		{
			$$.distance = $2;
			$$.clrnc    = $3;
			strncpy($$.crdrReason, $4, cMAX_WORDLENGTH);
			if ( $5.isLine || $5.isSign ) 
				$$.info[0] = $5;

			if ( $6.isLine || $6.isSign ) 
				$$.info[1] = $6;
		}
		;

 
/************************************************************************/
hold_ofs_info	: 
		LRI_HOLDLINE LRI_EQUALTO LRI_REAL LRI_REAL
		{
			$$.isLine           = 1;
			$$.isSign           = 0;
			$$.thick 			= $4;
			$$.angle 			= $4;
		}

		| LRI_HOLDSIGN LRI_EQUALTO LRI_IDENTIFIER
		{
			$$.isLine           = 0;
			$$.isSign           = 1;
			strncpy($$.holdSign, $3, cMAX_WORDLENGTH);
		}

		|
		{
			$$.isLine           = 0;
			$$.isSign           = 0;
		}
		;

/************************************************************************/
crdr_curve	: 
		crdr_curve crdr_curve_point
		{
			if ($$.n_crdrpts == $$.total_crdrpts)
            {
				int newSize;
				TCrdrCurve* pTemp;
				newSize = $$.total_crdrpts + cCRDRPOINTS;
				pTemp   = malloc (newSize*sizeof(TCrdrCurve));
				memcpy (pTemp, $$.pCrdrCurve, $$.total_crdrpts*sizeof(TCrdrCurve));
				$$.total_crdrpts 			  = newSize;
				free ($$.pCrdrCurve);
				$$.pCrdrCurve 	  			  = pTemp;
				$$.pCrdrCurve[$$.n_crdrpts] = $2;
				$$.n_crdrpts++;
            }
            else
            {
                $$.pCrdrCurve[$$.n_crdrpts] = $2;
                $$.n_crdrpts++;
            }
		}

		| crdr_curve_point
		{
/* type of crdr_curve_point is TCrdrCurve           */
			$$.pCrdrCurve	  = malloc (cCRDRPOINTS*sizeof(TCrdrCurve));
			$$.total_crdrpts = cCRDRPOINTS;
			$$.n_crdrpts	  = 1;
			$$.pCrdrCurve[0] = $1;
		}
		;


/************************************************************************/
crdr_curve_point	: 
		LRI_REAL LRI_REAL LRI_REAL crdr_line_info
		{
			$$.x 			 = $1;
			$$.y 			 = $2;
			$$.width 		 = $3;
			$$.pCrdrLineInfo = $4;
		}
		;

/************************************************************************/
crdr_line_info	: 
		LRI_LINES LRI_LFLAG LRI_LSTYLE LRI_LFLAG LRI_LSTYLE
		{
			$$ = calloc(1, sizeof(TCrdrLineInfo));
			$$->lflag1 = $2;
			strncpy ($$->lstyle1, $3, cMAX_WORDLENGTH);
			$$->lflag2 = $4;
			strncpy ($$->lstyle2, $5, cMAX_WORDLENGTH);
		}

		|
		{
			$$ = NULL;
		}
		;

/************************************************************************/
crdr_attr_spec	: 
		LRI_ATTR crdr_attr_list
		{
			$$ = $2;
		}

		|
		{
			$$ = NULL;
		}
		;

/************************************************************************/
crdr_attr_list:
		crdr_attr_list crdr_attr
		{
			TAttribute *pNew = malloc(sizeof(TAttribute));

			*pNew = $2;
			pNew->pNext = $$;
			$$ = pNew;
		}

		|
		crdr_attr
		{
			$$ = malloc(sizeof(TAttribute));

			*$$ = $1;
		}

/************************************************************************/
crdr_attr:
		LRI_REAL LRI_REAL LRI_REAL LRI_REAL LRI_REAL
		{
			$$.inum     = (int)$1;
			$$.laneMask = 0;
			$$.attrNum1 = $2;
			$$.attrNum2 = $3;
			$$.from     = $4;
			$$.to       = $5;
			$$.pNext    = NULL;
		}
		;


/************************************************************************/
/* OBJECTS */
/************************************************************************/

obj_defs :
	LRI_OBJECTS LRI_CURLY_OPEN obj_list LRI_CURLY_CLOSE
	{
	}
	;

/************************************************************************/
obj_list :
		obj_list obj
		{
			if ( gObjectCount == gObjectSpace ) {
				gObjectSpace += 200;
				gpObjects = realloc(gpObjects, gObjectSpace * sizeof(TObject));
			}

			gpObjects[gObjectCount++] = $2;
		}

		| obj
		{
			if ( gObjectCount == gObjectSpace ) {
				gObjectSpace += 200;
				gpObjects = realloc(gpObjects, gObjectSpace * sizeof(TObject));
			}

			gpObjects[gObjectCount++] = $1;
		}

		|
		{
		}
		;

/************************************************************************/
/*
 *   <obj_type> <obj_name> <sol_name> <x> <y> <zval> <yaw> <cigi>
 *   zval can be a number or 'plant'
 *
 */
obj : 
		LRI_IDENTIFIER LRI_IDENTIFIER LRI_IDENTIFIER LRI_REAL LRI_REAL LRI_REAL
					LRI_REAL 
		{
			strcpy($$.type, $1);
			strcpy($$.name, $2);
			strcpy($$.sol, $3);
			$$.x     = $4;
			$$.y     = $5;
			$$.z     = $6;
			$$.yaw   = $7;
			$$.plant = 0;
			$$.cigi  = -1;
		}

		| LRI_IDENTIFIER LRI_IDENTIFIER LRI_IDENTIFIER LRI_REAL LRI_REAL 
					LRI_PLANT LRI_REAL 
		{
			strcpy($$.type, $1);
			strcpy($$.name, $2);
			strcpy($$.sol, $3);
			$$.x     = $4;
			$$.y     = $5;
			$$.z     = 0.0;
			$$.yaw   = $7;
			$$.plant = 1;
			$$.cigi  = -1;
		}
		
		|		LRI_IDENTIFIER LRI_IDENTIFIER LRI_IDENTIFIER LRI_REAL LRI_REAL LRI_REAL
					LRI_REAL LRI_REAL
		{
			strcpy($$.type, $1);
			strcpy($$.name, $2);
			strcpy($$.sol, $3);
			$$.x     = $4;
			$$.y     = $5;
			$$.z     = $6;
			$$.yaw   = $7;
			$$.plant = 0;
			$$.cigi  = (int) $8;
		}

		| LRI_IDENTIFIER LRI_IDENTIFIER LRI_IDENTIFIER LRI_REAL LRI_REAL 
					LRI_PLANT LRI_REAL LRI_REAL
		{
			strcpy($$.type, $1);
			strcpy($$.name, $2);
			strcpy($$.sol, $3);
			$$.x     = $4;
			$$.y     = $5;
			$$.z     = 0.0;
			$$.yaw   = $7;
			$$.plant = 1;
			$$.cigi  = (int) $8;
		}
		;

/************************************************************************/
attr_dict : 
		LRI_ATTR_DICT LRI_CURLY_OPEN attr_dict_entry_list LRI_CURLY_CLOSE

		|
		;

attr_dict_entry_list:
		attr_dict_entry_list attr_dict_entry

		|
		attr_dict_entry
		;

attr_dict_entry : LRI_IDENTIFIER LRI_REAL
		{
			AttrDictEntry *pEntry;
			int i, found=0;

			/* Disallow use of dict entries that are already */
			/*  reserved by cved                             */
			if ((int)$2 < cCV_NUM_RESERVED_ATTR) {

				fprintf(stderr, "Cannot use reserved attribute id: %d\n", 
					(int)$2);
				exit(-1);
			}

			/* allocate space if we run out of the available */
			if ( gAttrDict.nSpace == gAttrDict.nEntries ) {
				gAttrDict.nSpace += 20;
				gAttrDict.pList = (AttrDictEntry *)realloc(
							gAttrDict.pList, gAttrDict.nSpace *
									sizeof(AttrDictEntry));
				if ( gAttrDict.pList == NULL ) {
					fprintf(stderr, "Out of memory\n");
					exit(-1);
				}
			}

			/* if the same entry exists, let the latest overwrite */
			for (i=0; i<gAttrDict.nEntries; i++) {
				if ( !strcmp($1, gAttrDict.pList[i].pName) ) {
					found = 1;
					pEntry = &gAttrDict.pList[i];
					break;
				}
			}

			if ( !found )  {
				pEntry = &gAttrDict.pList[gAttrDict.nEntries++];
			}
			else {
				free(pEntry->pName);
			}
			pEntry->id = (int)$2;
			pEntry->pName = malloc(strlen($1) + 1);
			strcpy(pEntry->pName, $1);
		}
		;

%%
