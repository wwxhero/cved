#pragma once
#ifndef _CVEDADOCTRL_H
#define _CVEDADOCTRL_H
#include "cveddistri.h"
namespace CVED
{

class CCvedADOCtrl :
	public CCvedDistri
{
public:
	CCvedADOCtrl(IExternalObjectControl* pCtrl);
	virtual ~CCvedADOCtrl(void);
	virtual void ExecuteDynamicModels(void);
	virtual CDynObj*	DistriCreateDynObj(const string&		cName,
								const cvTObjAttr&	cAttr,
								const CPoint3D*		cpInitPos=0,
								const CVector3D*	cpInitTan=0,
								const CVector3D*	cpInitLat=0);
	virtual void		DistriDeleteDynObj( CDynObj* );
	virtual CDynObj* LocalCreatePedObj(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
};

};
#endif
