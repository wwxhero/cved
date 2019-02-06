//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2018 by National Advanced Driving Simulator and
// Simulation Center, The University of Iowa and the University of Iowa.
// All rights reserved.
//
// Version:		$Id: ExternalControlMap.h,v 1.2 2018/04/12 14:40:02 IOWA\dheitbri Exp $
// Author(s):   David Heitbrink
// Date:        April, 2018
//
// Description: Header file for the CScenarioControl class.
//
/////////////////////////////////////////////////////////////////////////////
#ifndef _EXTERNALCONTROL_OBJECT_MAP_H
#define _EXTERNALCONTROL_OBJECT_MAP_H
#pragma once
namespace CVED {
    ////////////////////////////////////////
    ///\brief
    ///  object that holds a pointer
    ///This template is so we can do a smart
    ///to pointer we cannot delete (like ourself)
    template <class T> class   TSelf{
    public:
        TSelf(T* ptr):_ptr(ptr){};
        ~TSelf(){};
        T* operator -> (){return _ptr};
    private:
        T* _ptr; 
    };
    ////////////////////////////////////////
    ///\brief
    ///  Smart Pointer meant to hold This*
    ///This template is designed to provide 
    ///weak pointers to "this", when this ref
    ///gets deleted it deletes the "self" reference
    ///it holds. The underlying pointer holds is
    ///only accesible as a weak pointer. Childeren
    ///who are passed this object are expected
    ///to lock and release there reference as used.
    ///This class blocks on destruct 
    template <class T> class TSelfRef{
    public:
        typedef weak_ptr<TSelf<T>> TRef;
        TSelfRef(int waitToDeleteCnt=-1):_waitCnt(waitToDeleteCnt),_ptr(nullptr){};
        ~TSelfRef(){
            if (!_ptr.unique()){
                int cnt = 0;
                while(_waitCnt > 0 && !_ptr.unique()){
                    Sleep(0);
                    cnt++;
                    if (cnt > _waitCnt)
                        break;
                }
            }
        };
        void Init(T* ptr){
            _ptr = std::make_shared<TSelf<T>>(ptr);
        }
        std::weak_ptr<TSelf<T>> Get(){
            std::weak_ptr<TSelf<T>> p;
            p = _ptr;
            return p;
        }
    private:
        int _waitCnt;
        std::shared_ptr<TSelf<T>> _ptr;
    };
    
    class CDynObj;
    class CControlBuffer;
    /////////////////////////////////////////////////////////////////
    ///\brief
    ///    Class for holding a map of external controllers
    ///This class is mostly for convience, so we can dispatch the
    ///same command out to X number of controllers
    /////////////////////////////////////////////////////////////////
	class CExternalObjectMap {
	public:
        typedef std::shared_ptr<CControlBuffer> TControlBufferRef;
        CExternalObjectMap();
        void AddController(TExternalObjectControlRef, int id);
		TExternalObjectControlRef& operator[](int);
        
        bool has(int id);
        int LocalToSystemId(int id);
        virtual void PreUpdateDynamicModels();
        virtual void HandleExternalCreateAndDeletes(CVED::CCved& cved);
        //////////////////////////////////////////////////////////////////
        ///\brief frame done
		virtual void PostUpdateDynamicModels();
        //////////////////////////////////////////////////////////////////
		virtual void OnPushUpdate(TObjectPoolIdx id, const cvTObjState* nextState);
        virtual void OnCreateObj(TObjectPoolIdx id, const cvTObjAttr*, const cvTObjState* nextState);
		virtual void Clear(CCved* pCved);
        virtual void OnPushDeleteObject(int id);
        virtual void GetUpdate(int id,cvTObjState *pFutState);
        size_t size();
        
    private:
        bool Owns(int id, int sysId);
        typedef std::pair<int,TExternalObjectControlRef> TRefPair;
        typedef std::vector<TRefPair>  TRefMap;
        typedef std::unique_ptr<CVED::CDynObj> TObjRef;
        
        typedef std::map<int,TControlBufferRef> TBufferMap;
        typedef std::map<int,int> TIDmap;
        TRefMap m_pRefs;
        std::map<int,int> m_ownershipMap;
        std::map<int,int> m_localIdToSystemIdMap;
        std::map<int,TIDmap> m_SystemIdToLocalIdMap;
        std::map<int,TObjRef > m_DynObjs;
        std::map<int,cvTObjState> m_extBufferUpdates;
        typedef TSelfRef<CExternalObjectMap> TOwnRef;
        TBufferMap m_buffers;
        TOwnRef m_self; //< pointer to self, used to provide non owning pointers to childeren 
	};
    
};
#endif