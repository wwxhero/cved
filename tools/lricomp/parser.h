/***************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of Iowa,
 * and The University of Iowa.  All rights reserved.
 *
 * Version: 		$Id: parser.h,v 1.4 2013/05/08 15:17:51 IOWA\vhorosewski Exp $
 *
 * Author(s):   Jennifer Galvez, Yiannis Papelis,
 * Date:        July, 1998
 * 
 * This header files contains the data structures used by the LRI 
 * parser to hold the data in the LRI file.  This header file is not
 * visible in the CVED library software, only to the lri compiler.
 *
 */

#ifndef __PARSER_H
#define __PARSER_H

#ifdef __cplusplus
using namespace std;
#endif /* ifdef __cplusplus */

#include "cveddecl.h"
#include "objtypes.h"
#include "envirotype.h"
#include "cvedstrc.h"

#ifdef __cplusplus
extern "C" {
using namespace std;
#endif /* ifdef __cplusplus */

#define  cMAX_WORDLENGTH     64
#define  cINITIAL_ATTRIBUTES 5
#define  cINITIAL_CPOINTS    5
#define  cCRDRPOINTS         10
#define  cYes                'y'
#define  cNo                 'n'
#define  cMAX_BORDER_PTS     30
#define  cOK                 1
#define  cERROR              0

int yylex(void);

void DumpData(const cvTHeader *);
void DumpRoadPool(const cvTRoad *, int size);
void DumpIntrsctPool(const cvTIntrsctn *, int size);
void DumpCharPool(const char *, int size);
void DumpObjPool(const cvTObj *, int);
void DumpObjAttrPool(const cvTObjAttr *, int);

/***************************************************************************
 *
 * Information about the header of the LRI file
 *
 */
typedef struct Header
{
	char          *pComment;		/* comment, if NULL no comment        */
	int            zdown;			/* if non-zero, negative z is higher  */
    TU32b           solCheckSum;		/* checksum of sol used by compiler   */
} THeader;


/***************************************************************************
 *
 * Lateral control point
 *
 */
typedef struct 
{
	float			  height;		/* vert distance per to lateral plane */
	float             offset;		/* offset of point from centerline */
	int               material;		/* material code */
} TLatCpoint;


/***************************************************************************
 *
 * A node in th definition of a lateral curve.  A lateral curve
 * consists of a series of lateral points as defined by this
 * structure.  Note that the structure contains two control
 * points.  This is a side effect of the definition of the grammar
 * that forces an even numer of control poits on a lateral profile.
 *
 */
typedef struct CurveDef
{
	TLatCpoint		 latCpoint1;	/* the first control point */
	TLatCpoint		 latCpoint2;	/* the second control point */
	struct CurveDef* pNext;			/* pointer to next node */
} TCurveDef;


/***************************************************************************
 *
 * Definition of a lateral curve.
 *
 */
typedef struct LatCurve
{
	char			 curveName[cMAX_WORDLENGTH];	/* curve's name */
	float			 curveWidth;			/* width of curve */
	TCurveDef*		 pCurveDef;				/* linked list of points */
	struct LatCurve* pNext;					/* ptr to next lateral curve */
} TLatCurve;


/***************************************************************************
 *
 * Definition of an attribute.
 *
 */
typedef struct Attribute
{
	int				  inum;			/* attribute id */
	int				  laneMask;		/* which lane it applis */
	float			  attrNum1;		/* first value */
	float			  attrNum2;		/* second value */
	float             from;			/* start point, if -1, start of item */
	float             to;			/* end point, if -1, end of item */
	struct Attribute* pNext;		/* ptr to next, used for linked lists */
} TAttribute;


/***************************************************************************
 *
 * Definition of a lane.
 *
 */
typedef struct 
{
	float		width;			/* width of the lane */
	char		direction; 		/* p if positive, n if negative */
} TLaneDef;


/***************************************************************************
 *
 * All lanes associated witha road.
 *
 */
typedef struct 
{
	int 	 n_lanes;				/* how many lanes valid in array */
	TLaneDef lanes[cCV_MAX_LANES];	/* array of lanes */
} TLaneList;


/***************************************************************************
 *
 * Provides information about the width of lanes.  The structure
 * is used to update the width of lanes once an initial defintion
 * has been given (by TLaneList).
 */
typedef struct
{
	int 	n_lanes;					/* how many items vlid in array */
	float	lanewidth[cCV_MAX_LANES];	/* width of lanes */
} TLaneWidth;

/***************************************************************************
 *
 * This structure provides various information about a control point.
 * The structure holds one of three possible pieces of information,
 * each held by one of the 'curveName', pCpointAttribute', or
 * 'cpointWidth' fields.
 *
 * The cPpointAttribute field, if not NULL, associates a new
 * attribute with starting at the specified control point.
 *
 * The cpointWidth field contains information about the width
 * of the lanes at the control point.  This field is valid
 * if the n_lanes field is > 0.
 *
 * The curveName string contains the name of a new lateral
 * profile to be applied to the road starting at the
 * control point.  Valid if curveName[0] is non 0.
 * 
 */
typedef struct CpointInfo
{
	char				curveName[cMAX_WORDLENGTH];	/* name of lateral curve */
	TAttribute*			pCpointAttribute;			/* attribute information */
	TLaneWidth			cpointWidth;				/* lane width info */
	struct CpointInfo*	pNext; 		/* ptr to next node in linked lists */
} TCpointInfo;


/***************************************************************************
 *
 * This structure represents a control point on the longitudinal
 * curve defining the road's centerline.
 *
 */
typedef struct 
{
	char			cpointName[cMAX_WORDLENGTH];	/* control pnt name */
	float			x;
	float			y;
	float			z;
	float			i;
	float			j;
	float			k;
	TCpointInfo*	pCpointInfo;	/* head of linked list of assoc. info */
} TControlPoint;


/***************************************************************************
 *
 * A structure repesenting repeated objects.  Repeated objects
 * are objects that are instanced once but are supposed to 
 * appear on a regular interval along the road's centerline
 */
typedef struct RepObj 
{
	char           name[cMAX_WORDLENGTH];	/* template obj name */
	float          latdist;					/* lateral distance of obj */
	float          start;		/* longit. dist when obj starts appearing */
	float          period;		/* period of appearance along longitud. axis*/
	float          end;			/* logit. dist when obj ends appearing */
	int            alligned;	/* if set, object is alligned with road */
	struct RepObj *pNext;
} TRepObj;


/***************************************************************************
 *
 * A control point container.
 *
 */
typedef struct
{
	TControlPoint* 	pCpointList;	/* ptr to array of objects */
	int 			total_cpoints;	/* size of array (not all occupied) */
	int				n_cpoints;		/* size of array actually populated */
} TCpointList;


/***************************************************************************
 *
 * Definition for a road.
 *
 */
typedef struct Roads 
{
	char 		  	roadName[cMAX_WORDLENGTH];	/* name of road */
	char 		  	intersection1[cMAX_WORDLENGTH]; /* name of source intrs */
	char 		  	intersection2[cMAX_WORDLENGTH]; /* name of dest. intrs. */
	TAttribute*		pAttribute;		/* attributes applying to whole road */
	TRepObj*        pRepObjs;		/* repeated objects */
	TLaneList	  	lanelist;		/* lanes associated with road */
	TCpointList		longitCurve;	/* longitudinal road curve */
	struct Roads* 	pNext;			/* next road */
} TRoads;

/***********************************************************************
 *
 * This structure represents an elevation post within
 * an elevation map on a square grid
 *
 */
typedef struct ElevMap2
{
	float				z;				/* elevation at the post */
	int 				materialRef;	/* material at the post */
	struct ElevMap2*	pNextElevMap2;	/* next elevation post */
} TElevMap2;

/***********************************************************************
 *
 * This structure stores the head and tail of a linked list of elev posts.
 *
 */
typedef struct ElevMap2ListInfo
{
	TElevMap2*		pElevMap2Head;	/* head of the linked list of elev posts */
	TElevMap2*		pElevMap2Tail;	/* tail of the linked list of elev posts */
} TElevMap2ListInfo;

/***********************************************************************
 *
 * This structure represents an elevation grid.
 *
 */
typedef struct ElevMap
{
	char			elevMapName[cMAX_WORDLENGTH];	/* its name */
	int				n_rows;		/* number of rows (posts on y axis) */
	int				n_cols;		/* number of columsn (posts on x axis) */
	float           res;        /* the map's resolution */
	TElevMap2*		pElevMap2;	/* linked lists of n_rows*n_cols elev posts */
	struct ElevMap* pNextElevMap;	/* ptr to next map */
} TElevMap;


/***********************************************************************
 *
 * This structure represents an intersection's elevation information
 * which is either a fixed elevation or an elevation grid.
 *
 */
typedef struct
{
	float	height;				/* flat elevation, if elevMapName[0]==0 */
	char	elevMapName[cMAX_WORDLENGTH];	/* name of elev map */
	/* the following two are valid if elevMapName[0] != 0 */
	float	xorig;	/* lower left corner of elevation grid */
	float	yorig;
} TElevInfo;


/***********************************************************************
 *
 * This structure represents a point that is part of the definition
 * of  a border.
 */
typedef struct BorderPt
{
	float			borderX;	/* x coord */
	float			borderY;	/* y coord */
	char			lflag;		/* line flag */
} TBorderPt;


/***********************************************************************
 *
 * This structure represents an intersection's border.
 *
 */
typedef struct Border
{
	TBorderPt       list[cMAX_BORDER_PTS]; /* list of points */
	int             nPts; /* how many points are actually in list */
} TBorder;


/***********************************************************************
 *
 * This structure represents information about a hold offset.
 *
 */
typedef struct HoldOfsInfo
{
	/* the following two flags (only one should be set at any time)
	 * indicate the reason for the hold offset
	 */
	int                 isLine;		/* if set, there is line on the crdr */
	int                 isSign;		/* if set, there is a sign */
	float				thick;		/* line thickness, if a line is there */
	float				angle;		/* angle of line in global coord sys */
	char				holdSign[cMAX_WORDLENGTH];	/* name of sign */
} THoldOfsInfo;


/***********************************************************************
 *
 * This structure represents a hold offset.
 *
 */
typedef struct HoldOfs
{
	float			distance; /* where the hold ofs is, along crdr curve */
	float			clrnc;	/* clearance for other crdrs */
	char			crdrReason[cMAX_WORDLENGTH];  /* why crdr exists */
	THoldOfsInfo	info[2]; /* information about crdr */
	struct HoldOfs *pNextHoldOfs; /* ptr to next hold offs */
} THoldOfs;


/***********************************************************************
 *
 * This structure holds information about a line along th
 * side of an intersection's corridor.
 *
 */
typedef struct
{
	char		lflag1;		/* line on left */
	char		lstyle1[cMAX_WORDLENGTH];	/* style of line on left */
	char		lflag2;		/* line on right */
	char		lstyle2[cMAX_WORDLENGTH];	/* style of line on right */
} TCrdrLineInfo;


/***********************************************************************
 *
 * This structure holds a control point that is part of the
 * longitudinal curve of a corridor
 *
 */
typedef struct CrdrCurve
{
	float				x;		/* x coord in global coord. sys */
	float				y;		/* y coord */
	float				width;	/* width of corridor */
	TCrdrLineInfo*		pCrdrLineInfo;	/* edge line info */
} TCrdrCurve;


/***********************************************************************
 *
 * This structure holds the longitudinal curve of a corridor
 *
 */
typedef struct
{
	TCrdrCurve*	pCrdrCurve;		/* array of points */
	int			n_crdrpts;		/* number of points that are valid */
	int			total_crdrpts;	/* allocated size of the array */
} TCrdrCurveList;


/***********************************************************************
 *
 * This structure represents a corridor 
 *
 */
typedef struct Crdr
{
	char			roadName1[cMAX_WORDLENGTH];	/* name of source road */
	int				crdrLane1;			/* source lane */
	char			roadName2[cMAX_WORDLENGTH]; /* name of destination road */
	int				crdrLane2;		/* destination lane */
	THoldOfs*		pHoldOfs;		/* linked list of hold offsets */
	TCrdrCurveList 	CrdrCurveList;	/* longitudinal control points */
	TAttribute*     pAttrs;			/* attributes associated with crdr */
	struct Crdr     *pNext;			/* next corridor */
} TCrdr;


/***********************************************************************
 *
 * This structure is used as a node in the linked list of 
 * road names that holds the roads adjacent to an intersection.
 *
 */
typedef struct RoadName
{
	char				roadName[cMAX_WORDLENGTH];
	struct RoadName*	pNextRoadName;
} TRoadName;


/***********************************************************************
 *
 * This structure represents an intersection
 *
 */
typedef struct Intersection
{
	char						name[cMAX_WORDLENGTH]; /* its name */
	TElevInfo					elevInfo;		/* information about elev */
	TRoadName*					pRoadNames;		/* adjacent roads */
	TBorder						Border;			/* outline of intersection */
	TCrdr					   *pCrdr;		/* linked list of corridors */
	struct Intersection        *pNext;		/* next intersection in list */
	TAttribute                 *pAttrs;  	/* attrs associated with isec */
} TIntersection;


/***********************************************************************
 *
 * This structure represents a static object
 *
 */
typedef struct Object {
	char            name[cMAX_WORDLENGTH];	/* object's name */
	char            sol[cMAX_WORDLENGTH];	/* sol identifier */
	char            type[cMAX_WORDLENGTH];	/* object type */
	float           x, y, z, yaw;           /* position/alignment */
	int             plant;
	int	            cigi;                  /* cigi number */
						/* if set, z is adjusted on underlying terrain */
} TObject;


/***********************************************************************
 *
 * This structure represents an entry in the attribute name
 * dictionary.
 *
 */
typedef struct AttrDictEntry {
	int   id;		/* the id */
	char  *pName;	/* the name associated with the id */
} AttrDictEntry;


/***********************************************************************
 *
 * This structure represents a dictionary that provides names
 * for attributes that are only identified by a number.
 */
typedef struct TAttrDict {
	int             nEntries;	/* how many entries */
	int             nSpace;		/* how much space allocated */
	AttrDictEntry  *pList;		/* ptr to array of entries */
} TAttrDict;



extern cvTRoad			*pRoadPool;
extern int				sizeOfRoadPool;
extern cvTLane 			*pLanePool;
extern int      		sizeOfLanePool;
extern cvTCntrlPnt		*pCntrlPntPool ;
extern int				sizeOfCntrlPntPool;
extern int              storageOfCntrlPntPool;
extern char				*pCharPool;
extern int				sizeOfCharPool;
extern cvTAttr			*pAttrPool;
extern int				sizeOfAttrPool;
extern cvTRepObj		*pRepObjPool;
extern int				sizeOfRepObjPool;
extern cvTLatCntrlPnt	*pLatCntrlPntPool;
extern int				sizeOfLatCntrlPntPool;
extern cvTIntrsctn		*pIntrsctnPool;
extern int				sizeOfIntrsctnPool;
extern cvTBorderSeg		*pBorderSegPool ;
extern int				sizeOfBorderSegPool;
extern cvTCrdr			*pCrdrPool;
extern int				sizeOfCrdrPool;
extern cvTCrdrCntrlPnt	*pCrdrCntrlPntPool;
extern int				sizeOfCrdrCntrlPntPool; 
extern cvTHldOfs		*pHldOfsPool;
extern int				sizeOfHldOfsPool;
extern cvTObj           *pObjPool;
extern int              sizeOfObjPool;
// extern cvTObjAttr       *pObjAttrPool;
// extern int              sizeOfObjAttrPool;
extern cvTRoadPiece     *pRoadPiecePool;
extern int              sizeOfRoadPiecePool;
extern cvTObjRef*       pObjRefPool;
extern int              sizeOfObjRefPool;
extern cvTDynObjRef		*pDynObjRefPool;
extern int				sizeOfDynObjRefPool; 
extern cvTRoadRef		*pRoadRefPool;
extern int				sizeOfRoadRefPool;
extern cvTIntrsctnRef	*pIntrsctnRefPool;
extern int				sizeOfIntrsctnRefPool;
extern cvTCrdrMrgDst	*pCrdrMrgDstPool;
extern int				sizeOfCrdrMrgDstPool;



extern int         gTestLex;
extern int         gDebugDump;
extern int         gNoSplineCorrect;
extern float       gCrdrTolerance;
extern float       gGapTolerance;
extern float       gHermiteSplineScale;
extern int         gOvrdVersionMaj;
extern int         gOvrdVersionMin;
extern int         gOvrdVersion1;
extern int         gOvrdVersion2;
extern int         gOvrdVersion3;


extern TLatCurve	*gpLatCurves;
extern TAttrDict	gAttrDict;

extern char* reservedAttrNames[cCV_NUM_RESERVED_ATTR];

#ifdef __cplusplus
}

//  C++ function declarations
void WriteOutputFile(
		const char *,
		const vector<cvTElevPost> &,
		const vector<cvTElevMap> &);
void GenerateCode(
		TRoads *, 
		TIntersection *, 
		TElevMap *,
		vector<cvTElevPost> &, 
		vector<cvTElevMap> &);
void SemanticCheck(TRoads *, TIntersection *, TElevMap *);
void GenerateObjects(
			const TObject*, 
			int, 
			cvTObj**, 
			int *);



#endif /* __cpluspus */


#endif /* _PARSER_H */
