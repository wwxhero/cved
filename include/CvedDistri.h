#pragma once
#ifndef _CCVEDDISTRI_H
#define _CCVEDDISTRI_H
#include "cved.h"
#include "HeaderDistriParseBlock.h"
namespace CVED {
class IExternalObjectControl;
class CCvedDistri :
	public CCved
{
public:
	CCvedDistri(IExternalObjectControl* pCtrl);
	virtual ~CCvedDistri(void);
	virtual CDynObj* CreatePeerDriver(CHeaderDistriParseBlock& blk) = 0;
protected:
	CDynObj* CreatePeerDriver(CHeaderDistriParseBlock& blk, cvEObjType type);
	IExternalObjectControl* m_pCtrl;
};


};
#endif
