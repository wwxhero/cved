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
	virtual CDynObj* LocalCreateDynObj(CHeaderDistriParseBlock& blk) = 0;
	virtual void LocalDeleteDynObj( CDynObj* ) = 0;
	virtual CDynObj* LocalCreateDynObj(
					const string&		cName,
					cvEObjType			type,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0) = 0;

	virtual CDynObj*	DistriCreateDynObj(const string&		cName,
								const cvTObjAttr&	cAttr,
								const CPoint3D*		cpInitPos=0,
								const CVector3D*	cpInitTan=0,
								const CVector3D*	cpInitLat=0) = 0;
	virtual void		DistriDeleteDynObj( CDynObj* ) = 0;
};
class CCvedDistri :	public ICvedDistri
				  , public CCved
{
public:
	CCvedDistri(IExternalObjectControl* pCtrl);
	virtual ~CCvedDistri(void);
	virtual void LocalDeleteDynObj( CDynObj* );
	virtual CDynObj* LocalCreateDynObj(
					const string&		cName,
					cvEObjType			type,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
protected:
	CDynObj* LocalCreateDynObj(CHeaderDistriParseBlock& blk, cvEObjType type);
	IExternalObjectControl* m_pCtrl;
};


};
#endif
