/**********************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Version: 		$Id: enumtostring.h,v 1.10 2012/01/17 20:19:00 iowa\yefeihe Exp $
 *
 * Author(s):  Jillian Vogel
 * Date:       December, 1999
 *
 * Description: conversion routines for various CCved enumerated types 
 *	and strings.  This is C code, even when included in a C++ project.
 *
 */

#ifndef __ENUM_TO_STRING_H
#define __ENUM_TO_STRING_H
/*
 * This is the enumeration of the different environmental conditions.
 *  the enum is used in the EnviroState structure.
 */
#define cNUM_ENVIRO_TYPE 10
typedef enum { eZERO = 0, eLIGHTNING, eVISIBILITY, eHAZE, eFOG, eSMOKE, 
			   eCLOUDS, eGLARE, eSNOW, eRAIN, eWIND } eCVEnviroType;

/* 
 * This is the enumeration of the different traffic light states.
 * 	The enum is used in the TrafficLightState structure.
 */
typedef enum { 
			eOFF = 0, 
			eRED, 
			eGREEN,
            eFLASH_GREEN,
			eYELLOW, 
			eFLASH_YELLOW, 
			eFLASH_RED,
			eRED_TURN_LEFT,
			eYELLOW_TURN_LEFT,
			eGREEN_TURN_LEFT,
			eRED_TURN_RIGHT,
			eYELLOW_TURN_RIGHT,
			eGREEN_TURN_RIGHT,
			eFLASH_RED_TURN_LEFT,
			eFLASH_YELLOW_TURN_LEFT,
			eFLASH_GREEN_TURN_LEFT,
			eFLASH_RED_TURN_RIGHT,
			eFLASH_YELLOW_TURN_RIGHT,
			eFLASH_GREEN_TURN_RIGHT,
			eRED_STRAIGHT,
			eGREEN_STRAIGHT,
			eYELLOW_STRAIGHT,
			eFLASH_RED_STRAIGHT,
			eFLASH_YELLOW_STRAIGHT,
			eFLASH_GREEN_STRAIGHT
			} eCVTrafficLightState;

/* 
 * This is the enumeration of the different degree of environmental 
 * condition of type Lightning, snow and rain
 * 	The enum is used in the TrafficLightState structure.
 */
typedef enum { eNO_SCALE = 0, eLOW, eMEDIAN, eHEAVY } eCVLMHScale;

/*
 *	Conversion routines
 */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

eCVLMHScale cvStringToLMHScale(const char *);
const char* cvLMHScaleToString(eCVLMHScale);

eCVEnviroType cvStringToEnviroType(const char *);
const char * cvEnviroTypeToString(eCVEnviroType);

eCVTrafficLightState cvStringToTrafficLightState(const char *);
const char * cvTrafficLightStateToString(eCVTrafficLightState);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ENUM_TO_STRING_H */
