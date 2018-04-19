/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: cvedpub.h,v 1.33 2016/10/28 20:38:07 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		October, 1998
//
// Description:	A header file that includes all possible header
// files needed by implementation source code of CVED.  Including
// this header file ensures that everything defined privately and
// publicly by CVED is included in the compilation.
//
// Not all source files need to include this header, as it probably
// includes more items than necessary, but if used in conjunction
// with precompiled headers, it can provide a lot of time savings.
//
//////////////////////////////////////////////////////////////////////
#ifndef __CVED_PUB_H
#define __CVED_PUB_H		// {secret}
#include <stdio.h>
#include <math.h>
#include <assert.h>

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS 1
#endif

#ifdef _WIN32
#pragma warning(disable:4786)
#endif

#include <string>
#include <vector>
#include <list>
#include <set>
#include <bitset>
#include <map>

#ifdef _WIN32
#ifndef _WINSOCKAPI
#define _WINSOCKAPI
#endif
#include <ostream>
#include <fstream>
#include <iostream>
#elif __sgi				// {secret}
#include <iostream.h>
#include <fstream.h>
#elif _PowerMAXOS
#include <iostream>
#include <fstream>
#endif
#if __cplusplus < 199711L
#include <boost/shared_ptr.hpp>
#endif
using namespace std;
/*version control*/
#include "cvedversionnum.h"
#include "sol2.h"

using namespace std;
#include "sharedmem.h"

#include "cveddecl.h"
#include "cvederr.h"
#include "cveditem.h"
#include "lanemask.h"
#include "attr.h"
#include "hldofs.h"
#include "road.h"
#include "lane.h"
#include "crdr.h"
#include "intrsctn.h"
#include "roadpos.h"
#include "objtypes.h"
#include "enumtostring.h"
#include "envirotype.h"
#include "enviro.h"
#include "objlayout.h"
#include "objmask.h"
#include "objattr.h"
#include "obj.h"
#include "dynobj.h"
#include "reconfobj.h"
#include "odePublic.h"
#include "cved.h"
#include "cntrlpnt.h"
#include "path.h"
#include "pathnetwork.h"
#include "ExternalControlInterface.h"
#include <DebugStream.h>


#endif	// #ifndef __CVED_PUB_H

