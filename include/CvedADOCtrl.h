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
	virtual void ExecuteDynamicModels(void) override;
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
