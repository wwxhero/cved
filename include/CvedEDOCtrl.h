#pragma once
#ifndef _CVEDEDOCTRL_H
#define _CVEDEDOCTRL_H
#include "CvedDistri.h"
namespace CVED {

class CCvedEDOCtrl :
	public CCvedDistri
{
public:
	CCvedEDOCtrl(IExternalObjectControl* pCtrl);
	virtual ~CCvedEDOCtrl(void);
};

};
#endif
