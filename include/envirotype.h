
/**********************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Version:         $Id: envirotype.h,v 1.3 2003/12/04 16:32:34 schikore Exp $ *
 * Author(s):  Lijen Tsao
 * Date:       Januaray, 2000
 *
 * Description: The declaration of the cvTEnviroInfo
 *
 **************************************************************************/
#ifndef __ENVIRO_TYPE_H
#define __ENVIRO_TYPE_H

#include "enumtostring.h"

struct cvTEnviroInfo{
	eCVEnviroType type; 

	union info{
		struct Lightning {
			eCVLMHScale degree;
		} Lightning;

		struct Visibility {
			double       dist;
		} Visibility;

		struct Haze {
			double       dist;
		} Haze;

		struct Fog {
			double       dist;
		} Fog;

		struct Smoke {
			double       degree;
		} Smoke;

		struct Clouds {
			double       altitude;
			int         type;
		} Clouds;

		struct Glare {
			double       degree;
		} Glare;

		struct Snow {
			eCVLMHScale degree;
		} Snow;

		struct Rain {
			eCVLMHScale degree;
		} Rain;

		struct Wind {
			double       dir_i;
			double       dir_j;
			double       vel;
			double       gust;
		} Wind;
	} info;
};


#endif 
