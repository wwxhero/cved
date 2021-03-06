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
	virtual void		DistriTeleportPDO(CAvatarObj* obj, const CPoint3D* pos, const CVector3D* tan);
private:
	virtual void Maintainer(void);
	virtual CDynObj* LocalCreateEDO(
					bool				own,
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
