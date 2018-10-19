/***********************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Version: 		$Id: objtypes.h,v 1.18 2011/07/08 22:35:02 iowa\dheitbri Exp $
 *
 * Author(s):  Yiannis Papelis
 * Date:       November, 1998
 *
 * This header file contains type specific information about objects.
 * THIS FILE SHOULD COMPILE IN BOTH C & C++
 *
 * The following steps should be taken when a new object type is to be
 * added in CVED.
 *
 *  1) add an enumeration constant to the cvEObjType enumeration listed
 *     below in this header file
 *  2) In the file libsrc/objtype.c add code to two conversion functions
 *     that convert between strings and enumeration types
 *  3) In the file objlayout.h add appropriately named members to
 *     the state and control input unions.  Make sure to keep the
 *     standard fiels to maintain allignment with the remaining structures
 *  4) Add forward delcarations for the CDynObj-derivative class that
 *     represents the new type.  This goes to the file dynobj.h
 *  5) Add appropriate code for the runDynamicModel() and other
 *     type specific access functions in the file dynobj.cxx
 *
 */

#ifndef __OBJ_TYPES_H
#define __OBJ_TYPES_H		/* {secret} */

typedef enum cvEObjType {
	eCV_INVALID = 0,
	eCV_TRAJ_FOLLOWER,
	eCV_VEHICLE,
	eCV_TRAILER,
	eCV_RAIL_VEH,
	eCV_TERRAIN,
	eCV_TRAFFIC_LIGHT,
	eCV_TRAFFIC_SIGN,
	eCV_COMPOSITE_SIGN,
	eCV_OBSTACLE,
	eCV_POI,
	eCV_COORDINATOR,
	eCV_EXTERNAL_DRIVER,
	eCV_WALKER,
	eCV_EXTERNAL_TRAILER,
	eCV_VIRTUAL_OBJECT,
	eCV_EXTERNAL_AVATAR,
	eCV_OBJ_TYPE_END
} cvEObjType;


/* The number of object types.  Object types start with the
 * value of 1, as 0 is reserved as an invalid type
 */
#define cNUM_OBJECT_TYPES   eCV_OBJ_TYPE_END


/* Declaration for functions that convert between type names
 * and type identifiers.
 */
#ifdef __cplusplus
extern "C" {
#endif

cvEObjType cvString2ObjType(const char *);
const char *cvObjType2String(cvEObjType);

#ifdef __cplusplus
}
#endif

#endif	/* #ifndef __OBJ_TYPES_H */

