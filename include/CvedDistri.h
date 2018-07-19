#pragma once
#ifndef _CCVEDDISTRI_H
#define _CCVEDDISTRI_H
#include "cved.h"
#include "HeaderDistriParseBlock.h"
namespace CVED {
class IExternalObjectControl;
class ICvedDistri
{
public:
	virtual CDynObj* LocalCreateEDO(CHeaderDistriParseBlock& blk) = 0;
	virtual CDynObj* LocalCreateADO(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0) = 0;
	virtual void LocalDeleteDynObj( CDynObj* ) = 0;

	virtual CDynObj*	DistriCreateADO(const string&		cName,
								const cvTObjAttr&	cAttr,
								const CPoint3D*		cpInitPos=0,
								const CVector3D*	cpInitTan=0,
								const CVector3D*	cpInitLat=0) = 0;
	virtual void		DistriDeleteADO( CDynObj* ) = 0;

	virtual CDynObj* LocalCreatePDO(CHeaderDistriParseBlock& blk) = 0;
	virtual void LocalDeletePDO(CDynObj* ) = 0;
};

class CCvedDistri :	public ICvedDistri
				  , public CCved
{
public:
	CCvedDistri(IExternalObjectControl* pCtrl);
	virtual ~CCvedDistri(void);
	virtual void LocalDeleteDynObj( CDynObj* );
	virtual CDynObj* LocalCreateADO(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
	virtual CDynObj* LocalCreatePDO(CHeaderDistriParseBlock& blk);
protected:
	virtual CDynObj* LocalCreatePDO(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0) = 0;
public:
	virtual void LocalDeletePDO(CDynObj* );
protected:
	IExternalObjectControl* m_pCtrl;
};


};
#endif
