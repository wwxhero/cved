/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Author(s):	Yiannis Papelis
 * Date:		August, 1998
 *
 * $Id: cveddecl.h,v 1.74 2014/10/15 14:46:39 IOWA\dheitbri Exp $
 *
 * Description:	Various constant and other global declarations for CVED.
 * The file will be compiled by C & C++ so it should maintain compatibility
 *
 * It is assumed that this file will be included by most other header
 * files.
 *
 ****************************************************************************/
#ifndef __CVED_DECL_H
#define __CVED_DECL_H

#include <stdio.h>

#include <point2d.h>
#include <point3d.h>
#include <vector2d.h>
#include <vector3d.h>
#include <vectors.h>

#ifdef __cplusplus
#include <lineseg2d.h>
#include <lineseg3d.h>
#include <polygon2d.h>
#include <polygon3d.h>
#include <quadtree.h>
#include <terraingrid.h>

extern "C" {
#endif /* ifdef __cplusplus */

/*
 * The zero value used for doubleing point zero checks.
 */
#define cCV_ZERO 1e-5

/*
 * A large number ????
 */
#define cCV_INFINITY 1e5

/*
 * The maximum number of lanes allowed on a road.  This constant has
 * to be a multiple of 8.
 */
#define cCV_MAX_LANES       16

/*
 * The maximum number of characters comprising an object's name
 */
#define cOBJ_NAME_LEN       64

/*
 * The maximum number of roads allowed on an intersection
 */
#define cCV_MAX_ROADS		16

/*
 * The maximum number of parts a Composite sign can have
 */
#define cMAX_PIECES_IN_COMPOSITE_SIGN 10

/*
 * The maximum number of corridors an object can overlap.  This is
 *	an arbitrary limit, and can be changed if needed.
 */
#define cCV_MAX_CRDRS		25
#define	cCV_MAX_GRIDS		100

#define cMAX_ENV_POLY_PTS   20
#ifdef SMALL_BLI
#define cNUM_ENV_AREA       20
#else
#define cNUM_ENV_AREA       200
#endif
#define cEXTRA_ENV_INFO     10

/*
 * The maximum number of dynamic objects
 */
#ifdef SMALL_BLI
#define cNUM_DYN_OBJS       1200
#else
#define cNUM_DYN_OBJS      12000
#endif
/*
 * The following 2 declarations are used in the ByRoadDynObj data structures.
 *	They determine the size of the dynamic object reference array.
 */
/*
 * This constant indicates how many dynamic object references exist for each
 *	potential dynamic object in the simulation.
 */
#define cCV_NUM_DOR_REPS	3
/*
 * This constant determines the total number of dynamic object references in
 *	the memory pool.  The extra DOR is used as a dummy or NULL item.
 */
#define	cCV_NUM_DYN_OBJ_REFS	1+cNUM_DYN_OBJS*cCV_NUM_DOR_REPS

/*
 * The number of dynamic objects slots that can be used
 * by externally specified  and controlled objects (generally
 * represent human drivers).  The specified number
 * reduces the number of slots available for dynamic objects
 * as specified in cNUM_DYN_OBJS
 */
#define cMAX_EXT_CNTRL_OBJS 10


/*
 * How close to a terrain surface the input estimate has to
 * be in order for the surface to be considered as a possible
 * answer.  Only applies when multiple terrain surfaces
 * overlap eachother.
 *
 * Also used by objreflist and dynobjreflist to determine whether
 *	an object is "on" a road or intersection.  The object must be
 *	within this distance above the road or intersection to be
 *	considered "on" the road or intersection.
 */
#define cQRY_TERRAIN_SNAP  4.0

/*
 * Extra space in the array of static objects.  Controls the
 * maximum number of static objects that can be created at
 * run time.
 */
#ifdef SMALL_BLI
#define cEXTRA_OBJ_SLOTS  500
#else
#define cEXTRA_OBJ_SLOTS  50000
#endif
#define cEXTRA_ATTR_SLOTS (cEXTRA_OBJ_SLOTS * 8)

/*
 * Maximum number of vertices used to represent the polygon associated
 * with a given environment controller type.
 */
#define cCV_MAX_ENVIRO_VERTS 10

/*
 * Definitions for integral types based on their size.  These
 * should be used only when the actual size of an integral
 * quantity matters.  Any other time, simply use C/C++ built in
 * types.
 */

typedef unsigned char  TU8b;	    /* Unsigned 8 bit quantity. */
typedef char           TS8b;	    /* Signed 8 bit quantity. */
typedef unsigned short TU16b;    /* Unsigned 16 bit quantity. */
typedef short          TS16b;    /* Signed 16 bit quantity. */
typedef unsigned int   TU32b;    /* Unsigned 32 bit quantity. */
typedef int            TS32b;    /* Signed 32 bit quantity. */

static_assert(sizeof(TU8b) == 1, "TU8b must be 8 bit in size");

/* the key to identify the shared memory used in multi user mode */
#define cCVED_SHARED_MEM_KEY  191919

/*
 * Definitions of integral types that index the various pools. We
 * use different types to allow strict type checking.
 */

/* Index into the enviro area pool */
typedef TU8b    TEnviroAreaPoolIdx;

/* Index into the enviro info pool */
typedef TU16b   TEnviroInfoPoolIdx;


/* Index into the road pool */
typedef TU16b	TRoadPoolIdx;

/* Index into the character pool. */
typedef TU32b	TCharPoolIdx;

/* Index into longitudinal control point pool.  */
typedef TU32b	TLongCntrlPntPoolIdx;

/* Index into lateral control point pool.  */
typedef TU16b	TLatCntrlPntPoolIdx;

/* Index into object pool.  */
typedef TU16b	TObjectPoolIdx;

/* Index into object reference pool */
typedef TU16b	TObjRefPoolIdx;

/* Index into intersection pool */
typedef TU16b	TIntrsctnPoolIdx;

/* Index into lane pool */
typedef TU32b	TLanePoolIdx;

/* Index into attribute pool */
typedef TU32b	TAttrPoolIdx;

/* Index into elevation map pool */
typedef TU16b	TElevMapPoolIdx;

/* Index into elevation post pool */
typedef TU32b	TElevPostPoolIdx;

/* Index into repeat object pool*/
typedef TU32b	TRepObjPoolIdx;

/* Index into surface prop pool */
typedef TU32b	TSurfacePropPoolIdx;

/* Index into border pool */
typedef TU32b	TBorderSegPoolIdx;

/* Index into CrdrCntrlPnt Pool */
typedef TU32b	TCrdrCntrlPntPoolIdx;

/* Index into CrdrMrgDst Pool */
typedef TU32b	TCrdrMrgDstPoolIdx;

/* Index into CorridorPool Idx */
typedef TU32b	TCrdrPoolIdx;

/* Index into HoldOffsetPool */
typedef TU32b	THldOfsPoolIdx;

/* Index into attribute pool */
typedef TU16b	TObjAttrPoolIdx;

/* Index into object state variable pool */
typedef TU16b	TObjStatePoolIdx;

/* Index into object control inputs pool for  */
typedef TU16b	TObjContInpPoolIdx;

/* Index into roadpiece pool */
typedef TU32b	TRoadPiecePoolIdx;

/* Enumeration for lane direction */
typedef enum { eNONE, ePOS, eNEG } cvELnDir;

/* Enumeration for holdoffset */
enum {eHOLD_SIGN=1, eHOLD_TLIGHT=2, eHOLD_OVLAP=4};

/* Enumeration for flag and style in CrdrCntrlPnt */
enum {eLEFT_FLAG=1, eRIGHT_FLAG=2, eLEFT_DOTTED=4, eRIGHT_DOTTED=8, eNO_FLAG};

/* Enumeration for properties which could change along the road */
enum {eNON_VALID, eWIDTH, eSHOULDER, ePROFILE};

/* Enumeration for flags in contro point used in terrain querry */
enum ECntrlPntFlg { eTERRN_OBJ = 1, eREP_OBJ_FLAG = 2};

/* Number of the properties which might change along the road */
#define cCV_NUM_ATTR_CHANGE		4

/* constants for relative corridor placement */
#define cCV_CRDR_LEFT_TO      1
#define cCV_CRDR_PARALLEL_TO  2
#define cCV_CRDR_RIGHT_TO     3

/* constants for general corridor direction */
#define cCV_CRDR_LEFT_DIR     -1
#define cCV_CRDR_STRAIGHT_DIR  0
#define cCV_CRDR_RIGHT_DIR     1

/* Number of reserved attribute ids */
#define	cCV_NUM_RESERVED_ATTR	17

#define cCV_LRI_ROAD_ATTR_EXTRA				3
#define cCV_LRI_INTRSCTN_ATTR_EXTRA			2
#define cCV_LRI_CRDR_ATTR_EXTRA				1

/* Reserved attribute declarations */
#define	cCV_BICYCLE_LANE_ATTR				31
#define	cCV_CITY_ROAD_ATTR					32
#define	cCV_DRIVING_LANE_ATTR				33
#define	cCV_EMERGENCY_LANE_ATTR				34
#define	cCV_EXIT_ONLY_LANE_ATTR				35
#define	cCV_RURAL_ROAD_ATTR					36
#define	cCV_HIGHWAY_ATTR					37
#define	cCV_HOV_LANE_ATTR					38
#define	cCV_INTERSTATE_ATTR					39

#define	cCV_INTERSTATE_OFF_RAMP				40
#define	cCV_INTERSTATE_ON_RAMP				41
#define	cCV_LANE_CHANGE_RULES_ATTR			42
#define	cCV_MERGE_ATTR						45
#define	cCV_PASSING_RULES_ATTR				46
#define	cCV_SPEED_LIMIT_ATTR				47
#define	cCV_TURN_LANE_ATTR					48
#define	cCV_VEHICLE_RESTRICTION_LANE_ATTR	49
#define cCV_ATTR_RSTYLE					    51
#define cCV_ATTR_LSTYLE					    52

#define cCV_ROAD_LSTYLE_NO_MARKING			0
#define cCV_ROAD_LSTYLE_SINGLE_SOLID_W		1
#define cCV_ROAD_LSTYLE_SINGLE_DASHED_W		2
#define cCV_ROAD_LSTYLE_DOUBLE_SOLID_W		3
#define cCV_ROAD_LSTYLE_DOUBLE_DASHED_W		4
#define cCV_ROAD_LSTYLE_SINGLE_SOLID_Y		5
#define cCV_ROAD_LSTYLE_SINGLE_DASHED_Y		6
#define cCV_ROAD_LSTYLE_DOUBLE_SOLID_Y		7
#define cCV_ROAD_LSTYLE_DOUBLE_DASHED_Y		8



#ifdef __cplusplus
}
#endif /* ifdef __cplusplus */

#endif	/* __CVED_DECL_H */

