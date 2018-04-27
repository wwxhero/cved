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
	virtual void GetAllDynamicObjs(TIntVec &out, CObjTypeMask mask) const;
	virtual void GetObjsNear(
			const CPoint3D& cLoc,
			double          radius,
			TIntVec&        out,
			CObjTypeMask    mask
			) const;
	virtual void GetAllDynObjsOnRoad(
			int roadId,
			const bitset<cCV_MAX_LANES> &lanes,
			TIntVec& result,
			const CObjTypeMask cMask
			) const;
	virtual void GetAllDynObjsOnLane (CLane lane,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								const CObjTypeMask cMask) const;
	virtual void GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								bitset<cCV_MAX_LANES>& usedLanes,
								const CObjTypeMask cMask) const;
	virtual void GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								bitset<cCV_MAX_LANES>& usedLanes,
								const CObjTypeMask cMask) const;
	virtual void GetAllDynObjsOnIntrsctn (int intrsctnId,
								const bitset<cCV_MAX_CRDRS> &crdrs,
								TIntVec& result,
								const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnIntrsctn (int intrsctnId,
								const bitset<cCV_MAX_CRDRS> &crdrs,
								TIntVec& result,
								const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnIntrsctn (int intrsctnId,
								const bitset<cCV_MAX_CRDRS> &crdrs,
								TIntVec& result,
								const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnIntrsctnRange (
			int intrsctnId,
			const bitset<cCV_MAX_CRDRS> &crdrs,
			const double startDist[cCV_MAX_CRDRS],
			const double endDist[cCV_MAX_CRDRS],
			TIntVec& result,
			const CObjTypeMask m
			) const;
	virtual void GetAllDynObjsOnIntrsctnRange (
			int intrsctnId,
			const bitset<cCV_MAX_CRDRS> &crdrs,
			const double startDist[cCV_MAX_CRDRS],
			const double endDist[cCV_MAX_CRDRS],
			TIntVec& result,
			bitset<cCV_MAX_CRDRS>& usedCrdrs,
			const CObjTypeMask m
			) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const;
	virtual void GetAllDynObjsOnCrdrRange (int intrsctnId,
								 int crdrId,
								 double startDist,
								 double endDist,
								 TIntVec& result,
								 const CObjTypeMask m) const;
	virtual void BuildFwdObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& fwdObjs
			);
	virtual void BuildBackObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& backObjs
			);
	virtual void BuildOncomingObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& oncomObjs
			);
	virtual void BuildApprchObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& apprchObjs
			);
	virtual void BuildBackObjList2(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CRoad& prevRoad,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& backObjs2
			);
private:
	IExternalObjectControl* m_pCtrl;
};


};
#endif
