/***************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of Iowa,
 * and The University of Iowa.  All rights reserved.
 *
 * Version: 		$Id: err.h,v 1.1 2005/06/10 17:43:33 yhe Exp $
 *
 * Author(s):   
 * Date:        
 * 
 * This header file contains the error checking interfaces used by the LRI 
 * compiler.  To use lrierr(), include err.h in the source file.  Add entries
 * in both TErrorType in err.h and errMsgs[] to expand the capacity of 
 * lrierr().
 *
 */
#ifndef _LRI_ERROR_H
#define _LRI_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif /* ifdef __cplusplus */

#include <stdio.h>

/* 
 * Definitions for errors.  After adding a new error code here
 * make sure to add a corresponing message to the error message
 * table located in err.c
 */
typedef enum {
	eBAD_ROAD						= 0,
	eBAD_LANE						= 1,
	eBAD_INTRSCTN					= 2,
	eBAD_OBJECT_TYPE				= 3,
	eBAD_SOL_NAME					= 4,
	eROAD_POOL_ALLOC_FAIL			= 5,
	eCHAR_POOL_ALLOC_FAIL			= 6,
	eATTR_POOL_ALLOC_FAIL			= 7,
	eREP_OBJ_POOL_ALLOC_FAIL		= 8,
	eLANE_POOL_ALLOC_FAIL			= 9,
	eCNTRL_PNT_POOL_ALLOC_FAIL		= 10,
	eLAT_CNTRL_PNT_POOL_ALLOC_FAIL	= 11,			
	eINTRSCTN_POOL_ALLOC_FAIL		= 12,
	eBORDER_SEG_POOL_ALLOC_FAIL		= 13,
	eCRDR_POOL_ALLOC_FAIL			= 14,
	eCRDR_CNTRL_PNT_POOL_ALLOC_FAIL	= 15,
	eHLD_OFS_POOL_ALLOC_FAIL		= 16,
	eLAT_POOL_ALLOC_FAIL			= 17,
	eMEM_WRITE_FAIL					= 18,
	eSEMANTICS_ERROR				= 19,
	eROAD_PIECE_ALLOC_FAIL			= 20,
	eINCONSISTENT_POST_COUNT        = 21,
	eBAD_ELEV_MAP                   = 22,
	eBAD_INTRSCTN_BORDER            = 23,
	eBAD_NAME_REP_OBJ				= 24,
	eDYN_OBJ_REF_POOL_ALLOC_FAIL	= 25,
	eROAD_REF_POOL_ALLOC_FAIL		= 26,
	eINTRSCTN_REF_POOL_ALLOC_FAIL	= 27,
	eBAD_SIGN_OR_LIGHT_NAME         = 28,
	eENV_AREA_POOL_ALLOC_FAIL		= 29,
	eENV_INFO_POOL_ALLOC_FAIL		= 30,
	eCRDR_MRG_DST_ALLOC_FAIL		= 31
}TErrorType;

/*************************************************************************
 *
 * lrierr() interface 
 *
 */
extern void lrierr(TErrorType err_id, char* fmt, ...);

#ifdef __cplusplus
}
#endif /* __cpluspus */

#endif /* _LRI_ERROR_H */
