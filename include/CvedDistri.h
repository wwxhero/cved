#pragma once
#ifndef _CCVEDDISTRI_H
#define _CCVEDDISTRI_H
#include "cved.h"
namespace CVED {
class IExternalObjectControl;
class CCvedDistri :
	public CCved
{
public:
	CCvedDistri(IExternalObjectControl* pCtrl);
	virtual ~CCvedDistri(void);
protected:
	IExternalObjectControl* m_pCtrl;
};


};
#endif
