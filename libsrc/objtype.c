/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *      Iowa.  All rights reserved.
 *
 * Version:		$Id: objtype.c,v 1.24 2018/04/03 15:59:23 IOWA\dheitbri Exp $
 *
 * Author(s):	Yiannis Papelis
 * Date:		September, 1998
 *
 * Description: conversion routines for object types and strings.
 * This is C code, even when included in a C++ project
 *
 ****************************************************************************/
#include <string.h>
#include "objtypes.h"

/*****************************************************************************
 *
 * Description: cvString2ObjType
 * 	Convert a string to a cved object type
 *
 * Remarks:
 *
 * Arguments:
 *  cpStr : pointer to string holding name to convert
 *
 * Returns: The object type or TINVALID (value 0) if no match
 *
 ****************************************************************************/
cvEObjType
cvString2ObjType( const char* cpStr )
{
	if( !strcmp( cpStr,  "TrajFollower"))     return eCV_TRAJ_FOLLOWER;
	if( !strcmp( cpStr,  "Vehicle"))          return eCV_VEHICLE;
	if( !strcmp( cpStr,  "Trailer"))          return eCV_TRAILER;
	if( !strcmp( cpStr,  "RailVeh"))          return eCV_RAIL_VEH;
	if( !strcmp( cpStr,  "Terrain"))          return eCV_TERRAIN;
	if( !strcmp( cpStr,  "TrafLight"))        return eCV_TRAFFIC_LIGHT;
	if( !strcmp( cpStr,  "TrafSign"))         return eCV_TRAFFIC_SIGN;
	if( !strcmp( cpStr,  "Composite" ))       return eCV_COMPOSITE_SIGN;
	if( !strcmp( cpStr,  "Obstacle"))         return eCV_OBSTACLE;
	if( !strcmp( cpStr,  "Poi"))              return eCV_POI;
	if( !strcmp( cpStr,  "SpecialEffect"))    return eCV_POI;
	if( !strcmp( cpStr,  "Coordinator"))      return eCV_COORDINATOR;
	if( !strcmp( cpStr,  "ExternalDriver"))   return eCV_EXTERNAL_DRIVER;
	if( !strcmp( cpStr,  "ExternalTrailer"))  return eCV_EXTERNAL_TRAILER;
	if( !strcmp( cpStr,  "Walker"))           return eCV_WALKER;
	if( !strcmp( cpStr,  "VirtualObject"))    return eCV_VIRTUAL_OBJECT;
	if( !strcmp( cpStr, "Avatar") ) return eCV_EXTERNAL_AVATAR;
	if( !strcmp( cpStr,  "ExternalVehicle"))  return eCV_EXTERNAL_VEH_OBJECT;

	return eCV_INVALID;
} /* end of cvString2ObjType */

/****************************************************************************
 *
 * Description: cvObjType2String
 * 	Convert a cved object type to a string
 *
 * Remarks:
 *
 * Arguments:
 *  type : the cved type type to convert
 *
 * Returns: A pointer to a constant string that holds the corresponding
 *  name.  If the type is invalid, the function returns a pointer to the
 *  string "Invalid".
 *
 ****************************************************************************/
const char*
cvObjType2String(cvEObjType type)
{
	switch ( type ) {    
		case eCV_TRAJ_FOLLOWER	     : return "TrajFollower";
		case eCV_VEHICLE		     : return "Vehicle";
		case eCV_TRAILER		     : return "Trailer";
		case eCV_RAIL_VEH		     : return "RailVeh";
		case eCV_TERRAIN		     : return "Terrain";
		case eCV_TRAFFIC_LIGHT	     : return "TrafLight";
		case eCV_TRAFFIC_SIGN	     : return "TrafSign";
		case eCV_COMPOSITE_SIGN      : return "Composite";
		case eCV_OBSTACLE		     : return "Obstacle";
		case eCV_POI                 : return "Poi";
		case eCV_COORDINATOR	     : return "Coordinator";
		case eCV_EXTERNAL_DRIVER     : return "ExternalDriver";
		case eCV_EXTERNAL_TRAILER    : return "ExternalTrailer";
		case eCV_WALKER              : return "Walker";
		case eCV_VIRTUAL_OBJECT      : return "VirtualObject";
		case eCV_EXTERNAL_AVATAR			: return "Avatar";
		case eCV_EXTERNAL_VEH_OBJECT : return "ExternalVehicle";
		default					     : return "Invalid";
	}
} /* end of cvObjType2String */

