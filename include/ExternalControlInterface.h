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
	class IExternalObjectControl {
	public:
		enum Type {edo_controller = 0, ado_controller};
		virtual void PreUpdateDynamicModels() = 0;
		virtual void PostUpdateDynamicModels() = 0;
		virtual bool OnGetUpdate(TObjectPoolIdx id, cvTObjContInp* curInput, cvTObjState* curState) = 0;
		virtual void OnPushUpdate(const cvTObjContInp* nextInput, const cvTObjState* nextState) = 0;
		virtual bool Initialize(CHeaderDistriParseBlock& blk, CVED::CCved* pCved, Type runAs) = 0;
		virtual void UnInitialize(CCved* pCved) = 0;
		// virtual bool CreateAndDeleteObjects(CVED::CCved&) = 0;
		// virtual bool Shutdown() = 0;
	};
};
#endif