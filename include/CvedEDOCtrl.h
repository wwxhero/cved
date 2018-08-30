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
	virtual void ExecuteDynamicModels(void);
	virtual CDynObj*	DistriCreateADO(const string&		cName,
								const cvTObjAttr&	cAttr,
								const CPoint3D*		cpInitPos=0,
								const CVector3D*	cpInitTan=0,
								const CVector3D*	cpInitLat=0);
	virtual void		DistriDeleteADO( CDynObj* );
private:
	virtual CDynObj* LocalCreateEDO(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
	virtual CDynObj* LocalCreatePDO(
					bool				own,
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
};

};
#endif
