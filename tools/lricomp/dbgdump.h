
/***************************************************************************
 * (C) Copyright 1998 by NADS & Simulation Center The University of Iowa
 * and The University of Iowa.  All rights reserved.
 *
 * 
 * Version: 		$Id: dbgdump.h,v 1.1 2005/06/10 17:43:31 yhe Exp $
 *
 *
 * Author(s) :
 * Date:       Octobor 1998
 *
 * Description:
 * header dump internal data structures, primarily for debugging.
 *
 **************************************************************************/

#ifndef _DBGDUMP_H
#define _DBGDUMP_H

#include "cvedstrc.h"

#ifdef __cplusplus
extern "C"{  
#endif /* ifdef __cplusplus */

void DumpRepObjPool(const cvTRepObj*, int);
void DumpAttrPool(const cvTAttr*, int);
void DumpLanePool(const cvTLane*, int);
void DumpLatCntrlPntPool(const cvTLatCntrlPnt*, int);
void DumpIntrsctnPool(const cvTIntrsctn*, int);
void DumpCharPool(const char*, int);
void DumpRoadPool(const cvTRoad*, int);
void DumpCntrlPnt(const cvTCntrlPnt*, int);
void DumpCrdrPool(const cvTCrdr*, int);
void DumpCrdrCntrlPntPool(const cvTCrdrCntrlPnt*, int);
void DumpBorderSeg(const cvTBorderSeg*, int);
void DumpHldOfsPool(const cvTHldOfs*, int);
void DumpObj(const cvTObj*, int);



#ifdef __cplusplus
}
#endif /* __cpluspus */

#endif /* _DBGDUMP_H */

