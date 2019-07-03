//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2017 by National Advanced Driving Simulator and
// Simulation Center, The University of Iowa and the University of Iowa.
// All rights reserved.
//
// Version:		$Id: ScenarioControl.h,v 1.43 2015/08/28 21:21:47 IOWA\dheitbri Exp $
// Author(s):   Yiannis Papelis
// Date:        Jan, 2017
//
// Description: Header file for the CScenarioControl class.
//
/////////////////////////////////////////////////////////////////////////////
#ifndef _EXTERNALCONTROLINTERFACE_H
#define _EXTERNALCONTROLINTERFACE_H
#pragma once
#include "cvedpub.h"
#include "SnoParserDistri.h"
#include "HeaderDistriParseBlock.h"
namespace CVED {
	class ICvedDistri;
	class IExternalObjectControl {
	public:
		virtual void PreUpdateDynamicModels() = 0;
		virtual void PostUpdateDynamicModels() = 0;
		virtual bool OnGetUpdate(TObjectPoolIdx id_local, cvTObjContInp* curInput, cvTObjState* curState) = 0;
		virtual bool OnGetUpdateArt(TObjectPoolIdx id_local, cvTObjState* curState) = 0;
		virtual void OnPushUpdate(TObjectPoolIdx id_local, const cvTObjContInp* nextInput, const cvTObjState* nextState) = 0;
		virtual void OnPushUpdateArt(TObjectPoolIdx id_local, const cvTObjState* curState) = 0;
		virtual void OnCreateADO(TObjectPoolIdx id_local
							, const char* szName
							, const cvTObjAttr& cAttr
							, const CPoint3D& pos
							, const CVector3D& t
							, const CVector3D& l) = 0;
		virtual void OnDeleteADO(TObjectPoolIdx id_local) = 0;
		virtual void OnTelePDO(TObjContInpPoolIdx id_local, const CPoint3D& pos, const CVector3D& tan, const CVector3D& lat) = 0;
		virtual bool Initialize(CHeaderDistriParseBlock& blk, CVED::ICvedDistri* pCvedDistri) = 0;
		virtual void UnInitialize() = 0;
	};
};
#endif