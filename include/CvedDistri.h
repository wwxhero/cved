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
	virtual CDynObj* LocalCreateEDO(CHeaderDistriParseBlock& blk, bool own) = 0;
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

	virtual CDynObj* LocalCreatePDO(CHeaderDistriParseBlock& blk, bool own) = 0;
	virtual void LocalDeletePDO(CDynObj* ) = 0;
	virtual void DistriTeleportPDO(CAvatarObj* obj, const CPoint3D* pos, const CVector3D* tan) = 0;
	virtual void LocalTeleportPDO(CAvatarObj* obj, const CPoint3D* pos, const CVector3D* tan, const CVector3D* lat) = 0;
	virtual void PeggingPair(const string& cParent, const string& cChild) = 0;
	virtual void PegPDOs() = 0;
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
	virtual CDynObj* LocalCreateEDO(CHeaderDistriParseBlock& blk, bool own);
	virtual CDynObj* LocalCreatePDO(CHeaderDistriParseBlock& blk, bool own);
	virtual void Maintainer(void);
	virtual void PeggingPair(const string& cParent, const string& cChild);
	virtual void PegPDOs();
	virtual void DistriTeleportPDO(CAvatarObj* obj, const CPoint3D* pos, const CVector3D* tan);
	virtual void LocalTeleportPDO(CAvatarObj* obj, const CPoint3D* pos, const CVector3D* tan, const CVector3D* lat);
protected:
	virtual CDynObj* LocalCreateEDO(
					bool				own,
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0) = 0;
	virtual CDynObj* LocalCreatePDO(
					bool				own,
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0) = 0;
	CDynObj * CreateDynObj(
					const string&     cName,
					cvEObjType        type,
					const cvTObjAttr& cAttr,
					const CPoint3D*   cpInitPos,
					const CVector3D*  cpInitTan,
					const CVector3D*  cpInitLat);
	void DeleteDynObj( CDynObj* dynObj );

public:
	virtual void LocalDeletePDO(CDynObj* );
protected:
	IExternalObjectControl* m_pCtrl;
	typedef std::pair<std::string, std::string> NamePair;
	typedef std::list<NamePair> NamePairs;
	NamePairs m_lstPeggings;
};


};
#endif
