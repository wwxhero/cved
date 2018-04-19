/*****************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Version: 		$Id: enumtostring.c,v 1.8 2012/01/18 17:34:25 iowa\yefeihe Exp $
 *
 * Author(s):  Jillian Vogel
 * Date:       December, 1999
 *
 * Description: conversion routines for various CCved enumerated types 
 *	and strings.  This is C code, even when included in a C++ project.
 *
 ****************************************************************************/
#include <stdlib.h>
#include <string.h>
#include "enumtostring.h"
#include <ctype.h>

/*****************************************************************************
 *
 * Description: cvStringToEnviroType
 *
 * Remarks: Convert a string to a cved environment type.
 *	The function expects a string in all caps.
 *
 * Arguments:
 *  cpStr : pointer to string holding name to convert
 *
 * Returns:
 *  The environment type or eZERO (value 0) if no match
 *
 ****************************************************************************/
eCVEnviroType 
cvStringToEnviroType(const char *cpStr)
{
	eCVEnviroType type = eZERO;

	if (!strcmp(cpStr, "LIGHTNING")) type = eLIGHTNING;
	else if (!strcmp(cpStr, "VISIBILITY")) type = eVISIBILITY;
	else if (!strcmp(cpStr, "HAZE")) type = eHAZE;
	else if (!strcmp(cpStr, "FOG")) type = eFOG;
	else if (!strcmp(cpStr, "SMOKE")) type = eSMOKE;
	else if (!strcmp(cpStr, "CLOUDS")) type = eCLOUDS;
	else if (!strcmp(cpStr, "GLARE")) type = eGLARE;
	else if (!strcmp(cpStr, "SNOW")) type = eSNOW;
	else if (!strcmp(cpStr, "RAIN")) type = eRAIN;
	else if (!strcmp(cpStr, "WIND")) type = eWIND;

	return type;
}

/*****************************************************************************
 *
 * Description: cvStringToLMHScale
 *
 * Remarks: Convert a string to a heavy, median or low scale.
 *	The function expects a string in all caps.
 *
 * Arguments:
 *  cpStr : pointer to string holding name to convert
 *
 * Returns:
 *  The environment type or eNO_SCALE (value 0) if no match
 *
 ****************************************************************************/
eCVLMHScale 
cvStringToLMHScale(const char *cpStr)
{
	eCVLMHScale type = eNO_SCALE;

	if (!strcmp(cpStr, "LOW")) type = eLOW;
	else if (!strcmp(cpStr, "MEDIAN")) type = eMEDIAN;
	else if (!strcmp(cpStr, "HEAVY")) type = eHEAVY;

	return type;
}

/*****************************************************************************
 *
 * Description: cvLMHScaleToString
 *
 * Remarks: Convert a eCVLMHScale to a string.
 *
 * Arguments:
 * 	type - the type to convert
 *
 * Returns:
 * 	A string, in all caps, containing the name of the low, median or heavy
 * 	scale according to the type in the parameter, or an empty string if 
 *  no match.
 *
 ****************************************************************************/
const char* 
cvLMHScaleToString(eCVLMHScale type)
{
	switch(type) {

		case eLOW:
			return "LOW";
		case eMEDIAN:
			return "MEDIAN";
		case eHEAVY:
			return "HEAVY";
		default:
			return "NONE";
	};
}

/*****************************************************************************
 *
 * Description: cvEnviroTypeToString
 *
 * Remarks: Convert a a cved environment type to a string.
 *
 * Arguments:
 * 	type - the type to convert
 *
 * Returns:
 * 	A string, in all caps, containing the name of the environment
 * 	type in the parameter, or an empty string if no match.
 *
 ****************************************************************************/
const char * 
cvEnviroTypeToString(eCVEnviroType type)
{
	switch(type) {

		case eLIGHTNING:
			return "LIGHTNING";
		case eVISIBILITY:
			return "VISIBILITY";
		case eHAZE:
			return "HAZE";
		case eFOG:
			return "FOG";
		case eSMOKE:
			return "SMOKE";
		case eCLOUDS:
			return "CLOUDS";
		case eGLARE:
			return "GLARE";
		case eSNOW:
			return "SNOW";
		case eRAIN:
			return "RAIN";
		case eWIND:
			return "WIND";
		default:
			return "NONE";
	};
}

/*****************************************************************************
 *
 * Description: cvStringToTrafficLightState
 *
 * Remarks: Convert a string to a cved traffic light state.
 *	This function expects an abbreviation in all caps.  For example, 
 *	a TrafficLight state is represented by an abbreviation.
 *	For example, Red-> R, Flashing yellow-> FY
 *
 * Arguments:
 *  cpStr : pointer to string holding name to convert
 *
 * Returns:
 *  The environment type or eOFF (value 0) if no match
 *
 ****************************************************************************/
eCVTrafficLightState
cvStringToTrafficLightState(const char *cpStr)
{
	eCVTrafficLightState type = eOFF;

	if (!strcmp(cpStr, "O"))        type = eOFF;
	else if (!strcmp(cpStr, "OFF")) type = eOFF;
	else if (!strcmp(cpStr, "R"))   type = eRED;
	else if (!strcmp(cpStr, "Y"))   type = eYELLOW;
	else if (!strcmp(cpStr, "G"))   type = eGREEN;
	else if (!strcmp(cpStr, "FR"))  type = eFLASH_RED;
	else if (!strcmp(cpStr, "FY"))  type = eFLASH_YELLOW;
	else if (!strcmp(cpStr, "FG"))  type = eFLASH_GREEN;
	else if (!strcmp(cpStr, "LTR")) type = eRED_TURN_LEFT;
	else if (!strcmp(cpStr, "LTY")) type = eYELLOW_TURN_LEFT;
	else if (!strcmp(cpStr, "LTG")) type = eGREEN_TURN_LEFT;
	else if (!strcmp(cpStr, "RTR")) type = eRED_TURN_RIGHT;
	else if (!strcmp(cpStr, "RTY")) type = eYELLOW_TURN_RIGHT;
	else if (!strcmp(cpStr, "RTG")) type = eGREEN_TURN_RIGHT;
	else if (!strcmp(cpStr, "FLTR")) type = eFLASH_RED_TURN_LEFT;
	else if (!strcmp(cpStr, "FLTY")) type = eFLASH_YELLOW_TURN_LEFT;
	else if (!strcmp(cpStr, "FLTG")) type = eFLASH_GREEN_TURN_LEFT;
	else if (!strcmp(cpStr, "FRTR")) type = eFLASH_RED_TURN_RIGHT;
	else if (!strcmp(cpStr, "FRTY")) type = eFLASH_YELLOW_TURN_RIGHT;
	else if (!strcmp(cpStr, "FRTG")) type = eFLASH_GREEN_TURN_RIGHT;
	else if (!strcmp(cpStr, "SR"))   type = eRED_STRAIGHT;
	else if (!strcmp(cpStr, "SY"))   type = eYELLOW_STRAIGHT;
	else if (!strcmp(cpStr, "SG"))   type = eGREEN_STRAIGHT;
	else if (!strcmp(cpStr, "FSR"))  type = eFLASH_RED_STRAIGHT;
	else if (!strcmp(cpStr, "FSY"))  type = eFLASH_YELLOW_STRAIGHT;
	else if (!strcmp(cpStr, "FSG"))  type = eFLASH_GREEN_STRAIGHT;

	return type;
}

/*****************************************************************************
 *
 * Description: cvTrafficLightStateToString
 *
 * Remarks: Convert a a cved traffic light state to a string.
 *
 * Arguments:
 * 	type - the type to convert
 *
 * Returns:
 * 	A string, in all caps, containing the abbreviation for the
 * 	type in the parameter, or an empty string if no match.
 *
 ****************************************************************************/
const char * 
cvTrafficLightStateToString(eCVTrafficLightState type)
{
	switch(type) {

		case eOFF:
			return "OFF";
		case eRED:
			return "R";
		case eYELLOW:
			return "Y";
		case eGREEN:
			return "G";
		case eFLASH_RED:
			return "FR";
		case eFLASH_YELLOW:
			return "FY";
		case eFLASH_GREEN:
			return "FG";
		case eRED_TURN_LEFT:
			return "LTR";
		case eYELLOW_TURN_LEFT:
			return "LTY";
		case eGREEN_TURN_LEFT:
			return "LTG";
		case eRED_TURN_RIGHT:
			return "RTR";
		case eYELLOW_TURN_RIGHT:
			return "RTY";
		case eGREEN_TURN_RIGHT:
			return "RTG";
		case eFLASH_RED_TURN_LEFT:
			return "FLTR";
		case eFLASH_YELLOW_TURN_LEFT:
			return "FLTY";
		case eFLASH_GREEN_TURN_LEFT:
			return "FLTG";
		case eFLASH_RED_TURN_RIGHT:
			return "FRTR";
		case eFLASH_YELLOW_TURN_RIGHT:
			return "FRTY";
		case eFLASH_GREEN_TURN_RIGHT:
			return "FRTG";
		case eRED_STRAIGHT:
			return "SR";
		case eYELLOW_STRAIGHT:
			return "SY";
		case eGREEN_STRAIGHT:
			return "SG";
		case eFLASH_RED_STRAIGHT:
			return "FSR";
		case eFLASH_YELLOW_STRAIGHT:
			return "FSY";
		case eFLASH_GREEN_STRAIGHT:
			return "FSG";
		default:
			return "";
	};
}


