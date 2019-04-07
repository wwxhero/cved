//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2018 by National Advanced Driving Simulator and
// Simulation Center, The University of Iowa and the University of Iowa.
// All rights reserved.
//
// Version:		$Id: ExternalControlMap.cxx,v 1.2 2018/04/12 14:41:33 IOWA\dheitbri Exp $
// Author(s):   David Heitbrink
// Date:        March, 2018
//
// Description: Header file for the CScenarioControl class.
//
/////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"
namespace CVED{
///\todo add concerency control
class CControlBuffer{
public:
    
    typedef std::shared_ptr<CControlBuffer> TRef;
    typedef std::pair<TObjectPoolIdx,cvTObjState> TUpdate;
    typedef vector<CControlBuffer::TUpdate> TUpdateSet;
    typedef vector<IUpdateControl::TCreate> TCreateSet;

    static TRef Make();
    /////////////////////////////////////////////////////////////////////////////////
    /// internal interfaces
    void GetCreateSet(TCreateSet&);
    void GetDeleteSet(vector<int>&);
    void GeUpdateSet(TUpdateSet&);
    /////////////////////////////////////////////////////////////////////////////////
    /// external interfaces
    void CreateObject(const IUpdateControl::TCreate&);
    void DeleteObject(const int&);  
	bool UpdateObject(TObjectPoolIdx id,  cvTObjState* curState);
private:
    CControlBuffer(){};
    vector<int>  m_deleteSet;
    TCreateSet   m_createSet;
    TUpdateSet   m_updateSet;
};


class CUpdateControl: public IUpdateControl{
public:
    typedef TSelfRef<CExternalObjectMap>::TRef TParentRef;
    CUpdateControl(CUpdateControl::TParentRef parent,
                   CExternalObjectMap::TControlBufferRef buffer)
                   :_parent(parent),_buffer(buffer){};
    ///////////////////////////////////////////////////////////////////////
    ///external interfaces
    virtual void CreateObject(
        	const string&		cName, 
			const cvTObjAttr&	cAttr,
			const CPoint3D*		cpInitPos,
			const CVector3D*	cpInitTan,
			const CVector3D*	cpInitLat) override;
    virtual void CreateObject(const IUpdateControl::TCreate&) override;
    virtual void DeleteObject(const int&) override;  
	virtual bool UpdateObject(TObjectPoolIdx id,  cvTObjState* curState) override;
private:
    CExternalObjectMap::TControlBufferRef _buffer;
    TParentRef _parent;
};

void CUpdateControl::CreateObject(
const string&		cName, 
const cvTObjAttr&	cAttr,
const CPoint3D*		cpInitPos,
const CVector3D*	cpInitTan,
const CVector3D*	cpInitLat) {

}
void CUpdateControl::CreateObject(const IUpdateControl::TCreate& obj){
    _buffer->CreateObject(obj);
}
void CUpdateControl::DeleteObject(const int& id){
    _buffer->DeleteObject(id);
}
bool CUpdateControl::UpdateObject(TObjectPoolIdx id,  cvTObjState* curState){
   return _buffer->UpdateObject(id,curState);
}
CControlBuffer::TRef 
CControlBuffer::Make(){
    return std::shared_ptr<CControlBuffer>( new CControlBuffer());
}
void CControlBuffer::CreateObject(const IUpdateControl::TCreate& obj){
    m_createSet.push_back(obj);
}
void CControlBuffer::DeleteObject(const int& id){
    m_deleteSet.push_back(id);
}
bool CControlBuffer::UpdateObject(TObjectPoolIdx id,  cvTObjState* curState){
    m_updateSet.push_back(TUpdate(id,*curState));
    return true;
}

void CControlBuffer::GetCreateSet(TCreateSet& cset){
    cset = std::move(m_createSet);
}
void CControlBuffer::GetDeleteSet(vector<int>& dset){
    dset = std::move(m_deleteSet);
}
void CControlBuffer::GeUpdateSet(TUpdateSet& uset){
    uset = std::move(m_updateSet);
}
CExternalObjectMap::CExternalObjectMap():m_self(1000){
    m_self.Init(this);
}
void 
CExternalObjectMap::AddController(TExternalObjectControlRef ref, int id){
    TRefPair temp(id,ref);
    m_pRefs[id] = temp;
    auto buff = CControlBuffer::Make();
    auto obj = ref.lock();
    if (obj){
        CUpdateControl* cntrlOjb = new CUpdateControl(m_self.Get(),buff);
        obj->Initialize(cntrlOjb);
        m_buffers[id] = buff;
    }
}

TExternalObjectControlRef& 
CExternalObjectMap::operator[](int id){
    static TExternalObjectControlRef gEmptyRef;
    if (id < 0 || id > m_pRefs.size())
        return gEmptyRef;
    return m_pRefs[id].second;//(*itr).second;
}
bool CExternalObjectMap::has(int id){
    if (id < 0 || id > m_pRefs.size())
        return false;
    auto ref = m_pRefs[id];
    if (!ref.second.expired())
        return true;
    else
        return false;
}
////////////////////////////////////////////////////////////////////
///\brief
///   Maps internal HCSM Id to ID external system is using
///\return id (greater than 1) on sucsess, -1 on fail
int  
CExternalObjectMap::LocalToSystemId(int id){
    auto itr = m_localIdToSystemIdMap.find(id);
    if (itr != m_localIdToSystemIdMap.end())
        return itr->second;
    else
        return -1;

}
//////////////////////////////////////////////////////////////////
///\brief let the object know we are about to do a frame update;
void CExternalObjectMap::PreUpdateDynamicModels(){
    std::map<int,TExternalObjectControlRef>::iterator itr;
    for (auto itr = m_pRefs.begin(); itr != m_pRefs.end(); ++itr){
        auto &ref = itr->second.lock();
        auto &id = itr->first;
        if (ref.get()){
            ref->PreUpdateDynamicModels();
        }
    }
}
cvTObjAttr MakeAttr(CVED::CCved& cved, int solID, int option){ 
    cvTObjAttr objAttr = { 0 };
    auto cpSolObj = cved.GetSol().GetObj(solID);
	objAttr.solId = cpSolObj->GetId();
	objAttr.xSize = cpSolObj->GetLength();
	objAttr.ySize = cpSolObj->GetWidth();
	objAttr.zSize = cpSolObj->GetHeight();
	objAttr.colorIndex = option;
	objAttr.hcsmId = -1;
    return objAttr;
}
void CExternalObjectMap::HandleExternalCreateAndDeletes(CVED::CCved& cved){
for (auto itr = m_buffers.begin(); itr != m_buffers.end(); ++itr){
    auto &ref = itr->second;
    auto &sysId = itr->first;
    auto &idMap = m_SystemIdToLocalIdMap[sysId];
    if (ref.get()){
        CControlBuffer::TCreateSet tSet;
        ref->GetCreateSet(tSet);
        for (auto itr = tSet.begin(); itr != tSet.end(); ++itr){
            auto& item  =*itr;
            int &id     =std::get<IUpdateControl::eID>(item);
            int &solId  =std::get<IUpdateControl::eSOLID>(item);
            string &name=std::get<IUpdateControl::eName>(item);
            int &option =std::get<IUpdateControl::eOption>(item);
            
            auto &ori   =std::get<IUpdateControl::eOrientation>(item);
            auto pos    =std::get<IUpdateControl::ePosition>(item);
            auto attr   = MakeAttr(cved,solId,option);
            CVector3D forward,right;
            ori.GetUnitVectors(forward,right);
            auto obj = cved.CreateDynObj(
                name,cvEObjType(eCV_EXTERNAL_VEH_OBJECT),
                -1,attr,
                &pos,&forward,&right);
            if (obj){
                int internalId = obj->GetId();
                m_localIdToSystemIdMap[internalId] = id;
                idMap[id]=internalId;
                m_DynObjs[internalId]= TObjRef(obj);
            }
           
        }
        vector<int> dSet;
        ref->GetDeleteSet(dSet);
        for (auto itr = dSet.begin(); itr != dSet.end(); ++itr){
            int& sid = *itr;
            auto litr = idMap.find(sid);
            if (litr != idMap.end()){
                int lid = litr->second;
                auto ditr = m_DynObjs.find(lid);
                if (ditr != m_DynObjs.end()){
                    cved.DeleteDynObj(ditr->second.get());
                    m_DynObjs.erase(ditr);
                }
            }
        }
        CControlBuffer::TUpdateSet updateSet;
        ref->GeUpdateSet(updateSet);
        for (auto itr = updateSet.begin(); itr!= updateSet.end(); ++itr){
            auto &id = itr->first;
            auto state = itr->second;
            auto idItr = idMap.find(id);
            if (idItr != idMap.end()){
                int localId = idItr->second;
                m_extBufferUpdates[localId] = state;
            }
            
        }
    }
}
}
//////////////////////////////////////////////////////////////////
///\brief frame done
void 
CExternalObjectMap::GetUpdate(int id,cvTObjState *pFutState){
    auto itr  = m_extBufferUpdates.find(id);
    if (itr == m_extBufferUpdates.end())
        return;
    memcpy(pFutState,&itr->second,sizeof(cvTObjState));
    
}
//////////////////////////////////////////////////////////////////
///\brief frame done
void CExternalObjectMap::PostUpdateDynamicModels(){
    for (auto itr = m_pRefs.begin(); itr != m_pRefs.end(); ++itr){
        auto &ref = itr->second.lock();
        auto &id = itr->first;
        if (ref.get()){
            ref->PostUpdateDynamicModels();
        }
    }
}

///\brief frame done
void CExternalObjectMap::OnPushUpdate(TObjectPoolIdx id, const cvTObjState* nextState){
    for (auto itr = m_pRefs.begin(); itr != m_pRefs.end(); ++itr){
        auto &ref = itr->second.lock();
        auto &sid = itr->first;
        if (ref.get()){
            if (!Owns(id,sid)) ref->OnPushUpdate(id,nextState);
        }
    }
}
///\brief frame done
void CExternalObjectMap::OnCreateObj(TObjectPoolIdx id, const cvTObjAttr* obj, const cvTObjState* nextState){
    for (auto itr = m_pRefs.begin(); itr != m_pRefs.end(); ++itr){
        auto &ref = itr->second.lock();
        auto &sid = itr->first;
        if (ref.get()){
            if (!Owns(id,sid))
                ref->OnCreateObj(id,obj,nextState);
        }
    }
}
void CExternalObjectMap::OnPushDeleteObject(int id){
    for (auto itr = m_pRefs.begin(); itr != m_pRefs.end(); ++itr){
        auto &ref = itr->second.lock();
        auto &sid = itr->first;
        if (ref.get()){
            if (!Owns(id,sid))
                ref->OnDeleteObj(id);
        }
    }
}
bool CExternalObjectMap::Owns(int id, int sysId){
    auto itr = m_ownershipMap.find(id);
    if (itr == m_ownershipMap.end())
        return false;
    if (itr->second == sysId)
        return true;
    else
        return false;
}
size_t  
CExternalObjectMap::size(){
    return m_pRefs.size();
}
void CExternalObjectMap::Clear(CCved* pCved){
    m_pRefs.clear();
    m_ownershipMap.clear();
    m_localIdToSystemIdMap.clear();
    m_SystemIdToLocalIdMap.clear();
    m_DynObjs.clear();
}
}