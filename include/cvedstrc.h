/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Author(s):	Yiannis Papelis
 * Date:		August, 1998
 *
 * $Id: cvedstrc.h,v 1.68 2007/07/23 17:17:05 dheitbri Exp $
 *
 * Description:	Contains all C/C++ structures used in the memory
 * block.  This file MUST maintain C & C++ compilation because
 * it is used both by the main cved code and the lri parser.
 *
 ****************************************************************************/
#ifndef __CVED_STRC_H
#define __CVED_STRC_H

#include "cveddecl.h"
#include "objtypes.h"
#include "time.h"

#ifdef __cplusplus
extern "C" {
#endif /* ifdef __cplusplus */

#define widthOfBoundingBox  25.0       	/* used for dividing the road into */
#define heightOfBoundingBox  25.0	/* road pieces */

/*
 * This structure reprsents the header information.  The header
 * is the first structure stored in the compiled LRI file and
 * in the memory block.  It contains general information about the
 * LRI file and enough data to allow decoding of the information
 * in the remaining of the compiled LRI file (and memory block).
 *
 * Because the binary lri file maps directly to the internal data
 * structures of the memory block, there are no pointers stored
 * in any fields that are shared by multiple clients which are
 * running in different processes.  In addition, the binary lri
 * file uses pools of structures for storage.  A pool of "items"
 * is nothing more than a block of memory starting at a certain
 * offset of the lri file (and memory) block that contains 
 * an array of items (generally structures).  The header contains
 * the offset of every pool in the LRI file along with the
 * number of items stored in that pool.  
 *
 * The size of most pools
 * is equal to the number of elements stored in it, but for a few
 * pools that hold items that can be created at runtime, the pool
 * is larter to accomodate creation of new items.  For these
 * pools, the header contains the actual size, and the number of
 * items stored at LRI compile time.
 *
 * The type of the various fields used as pool indeces indicates which
 * pool they are used for.  There exists a unique type for each pool.
 * 
 */
typedef struct cvTHeader {
/* the following fields have initial values in the compiled LRI file */

	int				majorVersionNum;   /* used for version control */
	int				minorVersionNum;
	int				minorExt1VersionNum;
	int				minorExt2VersionNum;
	int				minorExt3VersionNum;

	char            magic[8];          /* allow recognition of LRI files */
	TU32b           checkSum;          /* checksum of compiled LRI file */
	TU32b           solCheckSum;       /* checksum of referenced SOL */
	int             zdown;             /* if set, z up is (0, 0, -1) */
	//time_t          date;              /* compilation date */
	TU32b			date;
	TCharPoolIdx    shortComment;      /* a short comment about the contents*/
	TCharPoolIdx    description;       /* description of LRI file */
	TU32b           dataSize;          /* size of memory block inc header*/

	TU32b           charCount;         /* number of chars in message pool */
	TU32b           charStrgCount;     /* size of char pool is > charCount */
	TU32b           charOfs;           /* offset of pool holding characters */

	TU32b           envAreaCount;      /* num of envArea in the pool */
	TU32b           envAreaOfs;        /* offset of pool */
	TU32b           envAreaSize;       /* size of pool */

	TU32b            envInfoCount;      /* num of envInfo in the pool */
	TU32b           envInfoOfs;        /* offset of pool */
	TU32b           envInfoSize;       /* size of pool */
            
	TU32b           roadCount;         /* number of roads in VE */
	TU32b           roadOfs;           /* offset of pool holding roads */

	TU32b			rdPcQTreeSize;     /* size and offseg of the */
	TU32b			rdPcQTreeOfs;      /* quadtree block of roadpieces*/

	TU32b			intrsctnQTreeSize; /* size and offseg of the */
	TU32b			intrsctnQTreeOfs;  /* quadtree block of intrsctns*/
	
	TU32b			staticObjQTreeSize;/* size and offseg of the */
	TU32b			staticObjQTreeOfs; /* quadtree block of static objects */

	TU32b			trrnObjQTreeSize;  /* size and offseg of the */
	TU32b			trrnObjQTreeOfs;   /* quadtree block of terrain objects*/

	TU32b			roadPieceCount;    /* number of roadpieces in VE */
	TU32b			roadPieceOfs;      /* offset of pool holding roadpieces */	

	TU32b           intrsctnCount;     /* number of intersections in VE */
	TU32b           intrsctnOfs;       /* offset of pool holding intersecs */

	TU32b           longitCntrlCount;  /* number of longitud control points */
	TU32b           longitCntrlOfs;    /* offset of pool holding long. pnts*/

	TU32b           crdrCount;         /* number of corridors */
	TU32b			crdrOfs;		   /* offset of pool holding corridors */

	TU32b			laneCount;		   /* number of lanes */
	TU32b			laneOfs;		   /* offset of pool holding lanes */	

	TU32b			crdrCntrlPntCount; /* number of cntrl pnts of crdrs */
	int				crdrCntrlPntOfs;   /* offset of pool holding cntrl pnts */
									   /* of the crdr */
		
	TU32b	        latCntrlPntCount;  /* number of lateral control points */
	TU32b           latCntrlPntOfs;    /* offset of pool holding lat. pnts */

	TU32b           borderCount;	   /* number of border segments */
	TU32b           borderOfs;		   /* offset of pool holding border seg*/

	TU32b			hldCount;		   /* number of hold offset	*/
	TU32b			hldOfs;			   /* offset of pool holding hldOfs */

	TU32b			attrCount;		   /* number of attributes */
	TU32b			attrOfs;		   /* offset of pool holding attr	*/

	TU32b			repObjCount;	   /* number of repeated objs */
	TU32b			repObjOfs;		   /* offset of pool holding rep objs */

	TU32b			objRefCount;	   /* number of object refs */
	TU32b			objRefOfs;		   /* offset of pool holding obj refs */	

	TU32b			dynObjRefCount;    /* number of dynObjRef slots */
	TU32b			dynObjRefOfs;      /* offset of the pool */ 

	TU32b			roadRefCount;      /* number of road references */
	TU32b			roadRefOfs;        /* offset of the pool */	

	TU32b			intrsctnRefCount;  /* number of intersection references */
	TU32b			intrsctnRefOfs;    /* offset of the pool */	

	TU32b           elevMapCount;      /* number of elevation maps in VE */
	TU32b           elevMapOfs;        /* offset of pool holding elev maps */

	TU32b           elevPostCount;     /* number of elevation posts in VE */
	TU32b           elevPostOfs;       /* offset of pool holding elev posts */

	TU32b			crdrMrgDstCount;   /* number of sets of merge dist in VE*/
	TU32b			crdrMrgDstOfs;     /* offset */

   /* Note: the difference between the three object "counts" is as
    * follows:  objectCountInitial is the # of slots for the
    * dynamic objects plus the number of static objects in the LRI
    * file.  Upon initialization, objectCount is the same as
    * objectCountInitial, but as static objects are created, objectCount
    * is incremented.  Finally, objectStrgCount is the total number of 
    * slots in the header.
	*/
	TU32b           objectCountInitial;/* inital count of objects */
	TU32b           objectCount;       /* number of objects in LRI */
	TU32b           objectStrgCount;   /* how many objects fit in obj pool */
	TU32b           objectOfs;         /* offset of pool holding objects */

#if 0
	TU32b           objectAttrCount;   /* number of obj attrs in LRI */
	TU32b           objectAttrStrgCount;/* how many attrs fit in pool */
	TU32b           objectAttrOfsJunk;     /* offset of pool holding attrs */
#endif

	int             dynObjectCount;    /* number of dynamic objects */

/* Various state variables used by the cved code */
	TU32b           frame;             /* counts execution of maintainer */
	TU32b			dynaMult;			/* Dynamics interleave frequency */
	double          elapsedTime;       /* frame * deltaT */
	double          deltaT;			   /* time increment per frame */
	TObjectPoolIdx  lastObjAlloc;      /* last object id allocated */
	
/* the following fields are unitialized in the compiled lri file */
	int             initialized;	   /* until set, clients can't use */
	TU32b           numClients;
} cvTHeader;


/* 
 * This structure represents the road structure of lri file 
 */ 
typedef struct cvTRoad{
	TRoadPoolIdx			myId;			/* id of the road */
	TLongCntrlPntPoolIdx	cntrlPntIdx;	/* index of starting CntrlPnt */
											/* in the plngCtrlPntPool */
	int						numCntrlPnt;	/* number of lngCntrlPnt */
	TCharPoolIdx			nameIdx;		/* index of the road name in*/
											/* the pCharPool */
	TIntrsctnPoolIdx		srcIntrsctnIdx;	/* index of source intrsctn */
											/* in the Intrsctn Pool */
	TIntrsctnPoolIdx		dstIntrsctnIdx;	/* index of dstntn intrsctn */
											/* in the Intrsctn Pool */
	double					roadLengthLinear; /* linear length of the road */
	double					roadLengthSpline; /* spline length of the road */
	TAttrPoolIdx			attrIdx;		  /* index of starting attrbt */
											  /* in attribute pool */
	int						numAttr;		/* number of attributes */
	TRepObjPoolIdx			repObjIdx;		/* index of starting rep objs */
	int						numRepObj;		/* number of repeated objs*/
	TLanePoolIdx			laneIdx;		
	int						numOfLanes;		/* number of lanes */
}cvTRoad;

typedef struct cvTRoadPiece{
	TRoadPoolIdx			roadId;			/* id of the road the     */ 
											/* roadpiece belongs to   */
	TRoadPiecePoolIdx		myId;
	int						first;			/* index to the cntrlPnts */
	int						last;			/* with respect to the road */
											/* not globally */
	double					x1, y1;			/* lower left	*/
	double					x2, y2;			/* upper right	*/
}cvTRoadPiece;

typedef struct cvTObjRef{
	TObjectPoolIdx			objId;			/* object id it refer to */
	TObjRefPoolIdx			next;			/* next object id */
}cvTObjRef;

typedef struct cvTRoadRef{
	TObjectPoolIdx	objIdx;					/* Index into the dynamic object*/
											/*  reference pool of the first */
											/*	object in the list.         */ 
}cvTRoadRef;

typedef struct cvTIntrsctnRef{
	TObjectPoolIdx	objIdx;					/* Index into the dynamic object*/
											/*  reference pool of the first */
											/*	object in the list.         */ 
}cvTIntrsctnRef;

typedef enum {eTERR_NONE = 0, eTERR_ROAD, eTERR_ISEC} ETerrainCode;
typedef struct cvTDynObjRef{
	/* Both Roads and Intersections */
	TObjectPoolIdx			objId;			/* Index of the object in the   */
											/*	object pool                 */
	ETerrainCode			terrain;		/* Indicates whether object is  */
											/*	on a road, an intersection, */
											/*	or neither.  Default is     */
											/*	eTERR_NONE = 0.              */	
	TObjectPoolIdx			next;			/* Index into the dynamic object*/
											/*	reference pool of next      */
											/*	object in the list.         */
	/* Road information */
	TRoadPoolIdx			roadId;			/* Index of the road in the road*/
											/*	or road reference pool      */
	unsigned long			lanes;			/* Lanes the object overlaps.   */
											/*	This value may be given to a*/
											/*	bitset constructor.         */
	double					distance;		/* Linear distance from the     */
											/*  beginning of the road to the*/
											/*	center of the object.       */
	cvELnDir				direction;		/* Apparent direction of object,*/
											/*	based on the predominate    */
											/*	direction of the lanes      */
	TLongCntrlPntPoolIdx	roadCntrlPnt;	/* Index into the long cntrl pnt*/
											/*  pool of the first control   */
											/*	point of the segment the    */
											/*	object overlaps.            */
	/* Intersection information */
	TIntrsctnPoolIdx		intrsctnId;		/* Index of the intersection in */
											/*	the intrsctn or intrsctn ref*/
											/*	pool.                       */
	unsigned long			corridors;		/* Corridors the object overlaps*/
											/*	This value may be given to  */
											/*  the bitset constructor.     */
	double					crdrDistances[cCV_MAX_CRDRS];
											/* Linear distances from the    */
											/*	beginning of the corridor to*/
											/*	the center of object        */
}cvTDynObjRef;

typedef struct cvTRepObj{
	TObjectPoolIdx			objId;			/* object id it refer to */
	double					latdist;		/* lateral distance of obj */
	double					start;			/* longit. dist when obj starts */ 
											/* appearing */
	double					period;			/* period of appearance along */
											/* longitud. axis */
	double					end;			/* logit. dist when obj ends */ 
											/* appearing */
	int						alligned;		/* if set, object is allighed  */
											/* with road */
}cvTRepObj;


/*
 * This structure represents lane structure of the lri file 
 */
typedef struct cvTLane{
	TRoadPoolIdx			roadId;		/* id of the road where the lane is */
	TLanePoolIdx			myId;		/* id of the lane */
	TU8b					laneNo;		/* local id on the road */
	double					width;		/* width of the lane */
	double					offset;		/*offset of the lane from the center*/
	cvELnDir				direction;  /* direction of the lane, could be */
										/* eNONE, ePOS or eNEG */
	TAttrPoolIdx			attrIdx;	/* index of attribute */
	int						numAttr;	/* number of attributes */
	char					passInfo;	/* ?????? */
}cvTLane;

/*
 * This structure represnets attribute structure of the lri file
 */
typedef struct cvTAttr{
	TCharPoolIdx			nameIdx;	/* index of the name in char pool */
	TAttrPoolIdx			myId;		/* id of the attribute */
	double					value1;		
	double					value2;
	double					from;
	double					to;
	int						laneMask;
	TAttrPoolIdx			next;		/* index to the next attribute */
}cvTAttr;

typedef struct cvTSurfaceProp{
	int						id;
}cvTSurfaceProp;

/*
 * This structure represents the lateral control pont of the lri file
 */
typedef struct cvTLatCntrlPnt{
	double					offset;			/* offset of the point */
	double					height;
	int						material;
	TVector3D				normal;			/* normal of the point */
	double					normIntrepScale;
}cvTLatCntrlPnt;

/* cubic spline coefficient */
typedef struct cvTSplCoef{
	double					A;
	double					B;
	double					C;
	double					D;
}cvTSplCoef;

/* 
 * this structure represents longitudinal control point of the lri file
 */
typedef struct cvTCntrlPnt{
	double					physicalWidth;		/* logicalWidth+width of shoulder*/
	double					logicalWidth;		/* Sum of the width of the lanes */


	TLatCntrlPntPoolIdx		latCntrlPntIdx;		/* index of latCntrlPnt */ 
	int						numLatCntrlPnt;		/* number of latCntrlPnt */
	TLanePoolIdx			laneIdx;			/* index of starting lane */
	int						numLanes;			/* number of lanes */
	TPoint3D				location;			/* location of the point */
	TVector3D				normal;				/* normal of the point */
	TU16b					cntrlPntFlag;		/* use "or" operation(|)   */   
												/* with eTERRN_OBJ and/or  */ 
												/* eREP_OBJ_FLAG, both of  */
												/* which could be found in */
												/* cveddecl.h              */
	TObjRefPoolIdx			objRefIdx;			/* index to obj ref pool   */
	TVector3D				tangVecCubic;
	TVector3D				rightVecCubic;
	TVector3D				tangVecLinear;
	TVector3D				rightVecLinear;
	cvTSplCoef				hermite[3];
	double					cummulativeLinDist;	
	double					cummulativeCubicDist;	
	double					distToNextLinear;
	double					distToNextCubic;
	TSurfacePropPoolIdx		surfacePropIdx;
	double					sn;
	double					st;
	double					radius;
	double					distance;			/* this is redundant coz */ 
												/* there is */
												/* cummulativeLinDist */
	TCharPoolIdx			nameIdx;			/* index of the point name */
} cvTCntrlPnt;


/*
 * this structure represents an elevation post.  Elevation posts
 * are used by intersection elevation maps and elevation maps
 * in various objects
 */
typedef struct cvTElevPost {
	double          z;						/* the height of the terrain */
	TU32b          flags;					/* additional info */
} cvTElevPost;


/*
 * this structure represents an elevation map.  An elevation map
 * represents a rectangular area within which the terrain can
 * be interrogated.  This structure is used primarily to store
 * an elevation map.  Additional class(s) may be used to query it.
 */
typedef struct cvTElevMap {
	double             x1, y1;     /* lower left corner of rectangle */
	int               numRows;    /* number of rows (y axis) */
	int               numCols;    /* number of columns (x axis) */
	double             res;        /* grid resolution */
	TElevPostPoolIdx  postIdx;    /* index of first elevation post in pool */
	int               numPosts;   /* total number of elevation posts */
} cvTElevMap;

#if 1
/* 
 * These two structs are for intersection grids
 */
typedef struct cvTCrdrInfo {
	int						crdrId;	
	int						startCntrlPtIdx;
	int						endCntrlPtIdx;
} cvTCrdrInfo;

typedef struct cvTGrid {
	int						gridId;
	double					minX;			
	double					minY;
	double					maxX;
	double					maxY;
	cvTCrdrInfo				intrsctingCrdrs[cCV_MAX_CRDRS];
} cvTGrid;

#endif

/*
 * this structure represents the intersection of the lri file
 */
typedef struct cvTIntrsctn{
	TIntrsctnPoolIdx		myId;			/* id of the intersection */
	TCharPoolIdx			nameIdx;		/* index of the name */
	int						numOfRoads;		/* num of adfacent road */
	TRoadPoolIdx			adjacentRoadId[cCV_MAX_ROADS];	/* ids of */
															/* adjacent road*/
	TCrdrPoolIdx			crdrIdx;		/* index of corridor */
	TU32b					numOfCrdrs;		/* number of corridors */
	TU32b					numBorderSeg;	/* number of border segment */
	TBorderSegPoolIdx		borderSegIdx;	/* index of border segment */

	TElevMapPoolIdx         elevMap;		/* if == 0, then flat */
	double                   elevMapX;		/* x origin of elevation map */
	double                   elevMapY;		/* y origin of elevation map */
	double                   elevation;		/* terrain elevation */
											/* valid only if elevMap == 0 */

	TAttrPoolIdx			attrIdx;		/* index of starting attribute */
	int						numAttr;		/* number of attributes */

	TObjRefPoolIdx			objRefIdx;		/* index to obj ref pool   */
	TU16b					intrsctnFlag;	/* use "or" operation(|)   */   
											/* with eTERRN_OBJ and/or  */ 
											/* eREP_OBJ_FLAG, both of  */
											/* which could be found in */
											/* cveddecl.h              */
	cvTGrid					grids[cCV_MAX_GRIDS];

}cvTIntrsctn;

/*
 * this structure represents the corridor of the lri file 
 */
typedef struct cvTCrdr{
	TCrdrPoolIdx			myId;			/* id of the corridor */	
	TRoadPoolIdx			srcRdIdx;		/* index of the source road */
	TRoadPoolIdx			dstRdIdx;		/* index of the dst road */
	TLanePoolIdx			srcLnIdx;		/* index of the source lane */
	TLanePoolIdx			dstLnIdx;		/* index of the destination lane*/
	TCrdrCntrlPntPoolIdx	cntrlPntIdx;	/* index of starting crrdr point*/
	unsigned int			numCntrlPnt;	/* number of corridor points */		
	TIntrsctnPoolIdx		intrsctnId;		/* id of the intrsctn where the */
											/* corridor is */
	TCrdrMrgDstPoolIdx		mrgDstIdx;		/* index of the merge matrix */
	
	THldOfsPoolIdx			hldOfsIdx;		/* index of the hold offset */
	int						numHldOfs;		/* number of hold offsets */
	TAttrPoolIdx			attrIdx;		/* index of starting attrb */
	int						numAttr;		/* number of attributes */

	int                     direction;		/* -1=left, 0=straight, 1=right */
	double                  directionK;

	unsigned char           dirRel[cCV_MAX_CRDRS]; 
											/* this tells us the direction of each
											 * corridor relative to this one.
											 * For two corridors, c1 & c2, 
											 * dirRel[cor2] stored in c1 tells us
											 * the direction of c1 relative to c2.
											 * The cor? numbers are corridor ordinals
											 * within each intersection
											 */
}cvTCrdr;

/*
 * this structure represents the corridor control points of the lri  
 */
typedef struct cvTCrdrCntrlPnt{
	TPoint2D				location;
	TVector2D				rightVecLinear;
	double					width;		
	double					distance;
	double                   radius;
	unsigned char			flagStyle;	/* or operation with eLEFT_FLAG, */
										/* eLEFT_DOTTED, eRIGHT_FLAG, or */
										/* eRIGHT_DOTTED, eNO_FLAG */	
}cvTCrdrCntrlPnt;

/*
 * this structure represents the distances of the merge point and
 * the exit point with other corridors in the same intersection.
 * if there're n corroders in a intersection, there will be n*n entries
 * of instances of this this structure for this intersection
 */
typedef struct cvTCrdrMrgDst{
	double	firstDist;
	double	lastDist;
} cvTCrdrMrgDst;

/*
 * this structures represent the border segment of the lri file
 */
typedef struct cvTBorderSeg{
	double					x;
	double					y;
	char					lineFlag;
}cvTBorderSeg;

/*
 * this strcture represents the hold offset of the lri file
 */
typedef struct cvTHldOfs{
	TObjectPoolIdx			objIdx;        /* index of the hldofs name*/
	char					reason;			/* or operation with eSIGN, */
											/* eTLIGHT, eOVLAP */
	double					distance;
	double					orientation;
	double					thickness;
}cvTHldOfs;
	
typedef enum cvEObjPhase { 
	eDEAD,
	eBORN,
	eALIVE,
	eDYING
} cvEObjPhase;

typedef struct cvTEnviroArea {
	TEnviroAreaPoolIdx      id;
	TPoint2D                origin;
    TPoint2D                polyPt[cMAX_ENV_POLY_PTS];
	int                  	numOfPolyPts;  
	TEnviroInfoPoolIdx      infoIdx;
	int                     numOfInfo;
	int                     maxNumOfInfo;
} cvTEnviroArea;

#include "objlayout.h"

/*
 * A convenience structure holding both the derived state and control
 * inputs of an object
 */
typedef struct cvTObjStateBuf {
	cvTObjState		state;
	cvTObjContInp	contInp;
} cvTObjStateBuf;

/*
 * The object structure.  Used to represent objects in the virtual 
 * environment.
 */
typedef struct cvTObj {
	TObjectPoolIdx			myId;			/* object identifier */
	cvEObjType              type;			/* object type */
	int						hcsmType;		/* hcsm object type */
	cvEObjPhase				phase;			/* object phase */
	char                    name[cOBJ_NAME_LEN];

	cvTObjAttr              attr;           /* The attributes */
	TU16b                   activeOption;   /* which option is active*/

	/* The following fields represent the state (and control inputs)
	 * of objects.  Each ObjStateBuf structure contains both state
	 *  and control inputs.  Two buffers are provided, buffer A a~nd
	 *  buffer B.  The two buffers behave differently during odd
	 *  and even frames of the maintainer.
	 * On even frames:
	 * 	When clients read state or write control inputs buffer A is used
	 * 	When dynaservers read state or write contr inp, buffer B is used
	 * 	The maintainer read state from buffer B
	 * On odd frames the usage of buffers A and B is reversed
	 */	
	cvTObjStateBuf			stateBufA;
	cvTObjStateBuf			stateBufB;

	/* The changedFlag field is set whenever a static object changes.
	 * This is so that it is possible to query for objects that have
	 * changed since the last time they were queried.  Since we have
	 * 8 bits, 8 different threads can query for changed objects, each
	 * giving the query it's own unique identifier.
	 */
	TU8b					changedFlag;
	
} cvTObj;








#ifdef __cplusplus
}		// extern "C"

/// define stream operators for the various structures; they come in very
// handy

#ifdef _WIN32
#include <ostream>
#endif

#endif /* ifdef __cplusplus */

#endif /* ifdef __CVED_STRC_H */
