/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2004 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: sol2.cxx,v 1.58 2016/11/01 15:11:58 IOWA\dheitbri Exp $
//
// Author(s):	Matt Schikore
// Date:		Feb 2004
//
// Description:	The new SOL 2.0 classes.  Now embedded in CVED, this
//              replaces the solgen module.
//
//////////////////////////////////////////////////////////////////////

#include "sol2.h"
#include <iostream>
#include <fstream>
#include <assert.h>
#include <algorithm>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <WinSock2.h>
#include <windows.h>
#include <sstream>
#include "EnvVar.h"
static CRandNumGen         gRandNumGen(24, 42);

const double cGRAVITY = 9.80665;      // meters/sec^2

bool CSol::Init(bool ForceAfterFail)
{
	string data;
	stringstream errors_stream;
	if (!ForceAfterFail && m_failed)
		return false;
	string envVar;
    NADS::GetEnvVar(envVar,"NADSSDC_SCN");
	if (envVar == "") {
		m_lastError = "Environment variable NADSSDC_SCN is not set." ;
		m_hasParseError = true;
		m_parseLog = m_lastError;
		m_failed = true;
		return false;
	}
	string fileName = envVar;
	fileName += "sol2.txt";

	ifstream soltxt(fileName.c_str());

	if (!soltxt) {
	 m_lastError = "Missing file: \"" + fileName + "\"";
	 m_failed = true;
	 m_hasParseError = true;
	 m_parseLog = m_lastError;
	 return false;
	}
	if (!ParseFile(soltxt)){
		m_failed = true;
		m_hasParseError = true;
		m_parseLog = m_lastError;
		return false;
	}
	soltxt.close();
	soltxt.clear();
    WIN32_FIND_DATA FindFileData;
    HANDLE hFind = INVALID_HANDLE_VALUE;
    //char DirSpec[MAX_PATH + 1];  // directory specification
    DWORD dwError;
    string dir = envVar;
    dir +="*.txt";
    hFind = FindFirstFile(dir.c_str(), &FindFileData);
    
    while (FindNextFile(hFind, &FindFileData) != 0) 
    {
        string tempFileName = FindFileData.cFileName;
	    if (tempFileName.size() > 4 && 
	 	   tempFileName.substr(tempFileName.size()-4) == ".txt" && 
	 	   tempFileName.substr(0,8) == "sol2_aux" &&
	 	   tempFileName != "sol2.txt"){
	 		   tempFileName.insert(0,envVar);
	 		   soltxt.open(tempFileName.c_str());
	 		   if (soltxt.is_open()){
	 			 if (!ParseFile(soltxt)){
	 				 errors_stream<<"Error in File" <<tempFileName<<":"<<endl
	 					 <<this->GetLastError()<<endl;
					 m_hasParseError = true;
	 			 }
	 		   }
	 		   soltxt.close();
	 		   soltxt.clear();
	    }
     }
    if (errors_stream.str().size() > 0){
	    ofstream error_file("sol_aux_errors.txt");
	    error_file<<errors_stream.str();
	    error_file.close();
    }
    dwError = ::GetLastError();
    FindClose(hFind);
    if (dwError != ERROR_NO_MORE_FILES) 
    {
    
    }
    m_parseLog += errors_stream.str();
    
    
	m_init = true;
	return true;
}

bool  CSol::ParseFile(ifstream& soltxt){
	// Parse
	while (!soltxt.eof()) {
		string objType;

		soltxt >> objType;
		if (soltxt.eof()) break;
		if (objType == "SOL2") {
			// parse the headers
			continue;
		}
		CSolObj* pObj;
		if (objType == "Vehicle") {
			pObj = new CSolObjVehicle((int) -1);
		}
		else if (objType == "TrafLight") {
			pObj = new CSolObjTrafLight((int) -1);
		}
		else if (objType == "Obstacle") {
 			pObj = new CSolObjObstacle((int) -1);
		}
		else if (objType == "TrafSign") {
			pObj = new CSolObjTrafSign((int) -1);
		}
		else if (objType == "Terrain") {
			pObj = new CSolObjTerrain((int) -1);
		}
		else if (objType == "Poi") {
			pObj = new CSolObjPoi((int) -1);
		}
		else if (objType == "Walker") {
			pObj = new CSolObjWalker((int) -1);
		}
    	else if (objType == "DiGuy") {
			pObj = new CSolObjDiGuy((int) -1);
		}
		else if (objType == "SpecialEffect") {
			pObj = new CSolObjSpecialEffect((int) -1);
		}
		else if (objType == "Composite") {
			pObj = new CSolObjComposite((int) -1);
		}
		else {
			m_lastError = "Invalid object type (";
			m_lastError += objType;
			m_lastError += ")";
			return false;
		}
		if (!pObj->Parse(soltxt)) {
			string error;
			error = "Could not parse ";
			error += objType;
			error += "(";
			error += pObj->GetName();
			error += ")";
			m_lastError = error + m_lastError;
			delete pObj;

			return false;
		}
		else {
			if (m_nameToId.find(pObj->GetName())!=m_nameToId.end()){
				string error;
				error = "Object Must Have Unique NAME  type";
				error += objType;
				error += "(";
				error += pObj->GetName();
				error += ")";
				m_lastError = error + m_lastError;
				delete pObj;
				return false;
			}

			if (pObj->GetId() < 0 || m_objs.find(pObj->GetId())!=m_objs.end()){
				string error;
				error = "Object Must Have Unique ID > -1 type ";
				error += objType;
				error += "(";
				error += pObj->GetName();
				error += ")";
				m_lastError = error + m_lastError;
				delete pObj;
				return false;
			}
			m_nameToId[pObj->GetName()] = (int) pObj->GetId(); // this is before inserting into the m_objs vector
			m_objs[pObj->GetId()] = (pObj);
			m_typeToElements[objType].push_back(pObj->GetId());
			m_objCnt++;
		}

	}
	return true;
}


bool CSol::ParseHeader(ifstream& in)
{

	return true;
}

int CSol::GetAllVehicles(vector<int>& ids, int canBeMask, int typeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;
	map<string,int> sortMap;
	for (map<int, CSolObj*>::const_iterator i = m_objs.begin(); i != m_objs.end(); i++) {
		const CSolObjVehicle* pObj = dynamic_cast<CSolObjVehicle*>(i->second);
		if (pObj != NULL && pObj->IsType(typeMask)) {
			ids.push_back(pObj->GetId());
            if(sort){
                string name = pObj->GetFriendlyName();
                if (name == ""){
                    name = pObj->GetName();
                }
                std::transform(name.begin(), name.end(),name.begin(), ::tolower);
                sortMap[name] = pObj->GetId();
            }
		}
	}

	if (sort){
        ids.clear();

        for (auto itr = sortMap.begin(); itr!= sortMap.end(); ++itr){
            ids.push_back(itr->second);
        }
    }



	return (int) ids.size();
}


int CSol::GetAllSpecialEffects(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin(); i != m_objs.end(); i++) {
		const CSolObjSpecialEffect* pObj = dynamic_cast<CSolObjSpecialEffect*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	//sorts list on user interface
	if(sort)
	{
		for(j = 1; j < (int)ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjSpecialEffect* theObj = dynamic_cast<CSolObjSpecialEffect*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				
				for(int z = 0; z < 4; z++)
				{
					str1[z] = toupper(str1[z]);
					str2[z] = toupper(str2[z]);
				}
				
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}

	return (int) ids.size();
}


int CSol::GetAllRailVehs(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			const CSolObjRailVeh* pObj = dynamic_cast<CSolObjRailVeh*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	
	//sorts list on user interface
	if(sort)
	{
		for(j = 1; j <(int) ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjRailVeh* theObj = dynamic_cast<CSolObjRailVeh*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				
				for(int z = 0; z < 4; z++)
				{
					str1[z] = toupper(str1[z]);
					str2[z] = toupper(str2[z]);
				}
				
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}

	return (int) ids.size();
}


int CSol::GetAllTrailers(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();i++) {
		const CSolObjTrailer* pObj = dynamic_cast<CSolObjTrailer*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	//sorts list on user interface
	if(sort)
	{
		for(j = 1; j < (int) ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjTrailer* theObj = dynamic_cast<CSolObjTrailer*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				
				for(int z = 0; z < 4; z++)
				{
					str1[z] = toupper(str1[z]);
					str2[z] = toupper(str2[z]);
				}
				
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}

	return (int) ids.size();
}


int CSol::GetAllWalkers(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjWalker* pObj = dynamic_cast<CSolObjWalker*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	//sorts list on user interface
	if(sort)
	{
		for(j = 1; j < (int) ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjWalker* theObj = dynamic_cast<CSolObjWalker*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				
				for(int z = 0; z < 4; z++)
				{
					str1[z] = toupper(str1[z]);
					str2[z] = toupper(str2[z]);
				}
				
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}

	return (int) ids.size();
}

int CSol::GetAllDiGuys(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjDiGuy* pObj = dynamic_cast<CSolObjDiGuy*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	//sorts list on user interface
	if(sort)
	{
		for(j = 1; j < (int) ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjDiGuy* theObj = dynamic_cast<CSolObjDiGuy*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				
				for(int z = 0; z < 4; z++)
				{
					str1[z] = toupper(str1[z]);
					str2[z] = toupper(str2[z]);
				}
				
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}

	return (int) ids.size();
}


int CSol::GetAllObstacles(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjObstacle* pObj = dynamic_cast<CSolObjObstacle*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	//sorts list on user interface
	if(sort)
	{
		for(j = 1; j < (int) ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjObstacle* theObj = dynamic_cast<CSolObjObstacle*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				
				for(int z = 0; z < 4; z++)
				{
					str1[z] = toupper(str1[z]);
					str2[z] = toupper(str2[z]);
				}
				
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}

	return (int) ids.size();
}


int CSol::GetAllPois(vector<int>& ids, int canBeMask) const
{
	ids.clear();
	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjPoi* pObj = dynamic_cast<CSolObjPoi*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	return (int) ids.size();
}


int CSol::GetAllTerrains(vector<int>& ids, int canBeMask) const
{
	ids.clear();
	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjTerrain* pObj = dynamic_cast<CSolObjTerrain*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	return (int) ids.size();
}


int CSol::GetAllTrafSigns(vector<int>& ids, int canBeMask, bool sort) const
{
	ids.clear();

	int j = 0;
	int k = 0;
	int temp;
	string str1;
	string str2;
	int tempID = -1;

	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjTrafSign* pObj = dynamic_cast<CSolObjTrafSign*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
		CSolObjComposite* pComp = dynamic_cast<CSolObjComposite*>(i->second);
        if (pComp != NULL && pComp->CanBe(canBeMask)) {
        ids.push_back(pComp->GetId());
        }
	}
	
	if(sort)
	{
		for(j = 1; j < (int) ids.size(); j++)
			for(k = j; k > 0; k--)
			{
				for (map<int, CSolObj*>::const_iterator s = m_objs.begin(); s != m_objs.end(); s++)
				{
					const CSolObjTrafSign* theObj = dynamic_cast<CSolObjTrafSign*>(s->second);
					if(theObj != NULL && theObj->CanBe(canBeMask))
						tempID = theObj->GetId();
					if(tempID == ids.at(k))
						str1 = theObj->GetName();
					if(tempID == ids.at(k-1)) 
						str2 = theObj->GetName();
					tempID =-1;
				}
				if( (str1.length() > 3) && (str2.length() > 3) )
				{
					for(int z = 0; z < 4; z++)
					{
						str1[z] = toupper(str1[z]);
						str2[z] = toupper(str2[z]);
					}
				}
				if(str1 < str2){
					temp = ids.at(k);
					ids.at(k) = ids.at(k-1);
					ids.at(k-1) = temp;
				}
			}
	}
	return (int) ids.size();
}


int CSol::GetAllCoordinators(vector<int>& ids, int canBeMask) const
{
	ids.clear();
	for (map<int, CSolObj*>::const_iterator i = m_objs.begin();
		i != m_objs.end();
		i++) {
			CSolObjCoordinator* pObj = dynamic_cast<CSolObjCoordinator*>(i->second);
		if (pObj != NULL && pObj->CanBe(canBeMask)) {
			ids.push_back(pObj->GetId());
		}
	}
	return (int) ids.size();
}


const string 
CSol::PickRandomSolObjectFromCategory(const string& cTypeName, const vector<int>& excludeIds) const
{
	map<string, vector<int> >::const_iterator found = m_typeToElements.find(cTypeName);
	if (found == m_typeToElements.end()) return cTypeName;

	vector<int> pickFrom = found->second;
	for (vector<int>::const_iterator i = excludeIds.begin(); i != excludeIds.end(); i++) {
		vector<int>::iterator inSet = find(pickFrom.begin(), pickFrom.end(), *i);
		if (inSet != pickFrom.end()) pickFrom.erase(inSet);
	}

	if (pickFrom.size() == 0) {
		return cTypeName;
	}

	long index;
	int count = 0;
	index = gRandNumGen.RandomLongRange(0, (int) pickFrom.size(), 0);

	const CSolObj* pObj = GetObj(pickFrom[index]);

	return pObj->GetName();
}

CSolObj::CSolObj(int id) :
	  m_canBe(0)
	, m_length(0.0)
	, m_width(0.0)
	, m_height(0.0)
	, m_id(id)
	, m_visModelCigiId(-1)
	, m_isatBbox(10.0)
	, m_bounceEnergyLoss(0.05)
	, m_frictionCoeff(0.7)
	, m_DefaultSoundID(3)
	, m_DefaultSoundIntesity(100)
	, m_CollisionSoundID(17)
	, m_CollisionSoundIntensity(100)
	, m_SirenSoundID(0)
{
}

bool CSolObj::ParseCommonField(const string& field, ifstream& in)
{
	string dummy;
	string str;
	int i = 0;

	if (field == "Length") {
		in >> m_length >> dummy;
		return true;
	}
	else if (field == "Width") {
		in >> m_width >> dummy;
		return true;
	}
	else if (field == "Height") {
		in >> m_height >> dummy;
		return true;
	}
	else if (field == "FriendlyName"){
		in >> m_friendlyName >> dummy;
		return true;
	}
	else if (field == "VisModelName") {
		if (ParseString(in, str)) {
			in >> dummy;
			m_visModelName = str;
			return true;
		}
		else return false;
	}
	else if (field == "VisModelCigiId") {
		in >> m_visModelCigiId >> dummy;
		return true;
	}
	else if (field == "CanBeAdo") {
		in >> i >> dummy;
		if (i) m_canBe |= cSOL_ADO;
		return true;
	}
	else if (field == "CanBeDdo") {
		in >> i >> dummy;
		if (i) m_canBe |= cSOL_DDO;
		return true;
	}
	else if (field == "CanBeStatic") {
		in >> i >> dummy;
		if (i) m_canBe |= cSOL_STATIC;
		return true;
	}
	else if (field == "IsatModelBitmap") {
		if (ParseString(in, str)) {
			in >> dummy;
			m_isatBitmap = str;
			return true;
		}
		else return false;
	}
	else if (field == "IsatModelHighLod") {
		if (ParseString(in, str)) {
			in >> dummy;
			m_isatModelHigh = str;
			return true;
		}
		else return false;
	}
	else if (field == "IsatModelLowLod") {
		if (ParseString(in, str)) {
			in >> dummy;
			m_isatModelLow = str;
			return true;
		}
		else return false;
	}
	else if (field == "IsatBboxSize") {
		in >> m_isatBbox >> dummy;
		return true;
	}
	else if (field == "FrictionCoeff") {
		in >> m_frictionCoeff >> dummy;
		return true;
	}
	else if (field == "BounceEnergyLoss") {
		in >> m_bounceEnergyLoss >> dummy;
		return true;
	}
	else if (field == "DefaultSoundID") {
		in >> m_DefaultSoundID >> dummy;
		return true;
	}
	else if (field == "DefaultSoundIntesity") {
		in >> m_DefaultSoundIntesity >> dummy;
		return true;
	}

	else if (field == "CollisionSoundID") {
		in >> m_CollisionSoundID >> dummy;
		return true;
	}
	else if (field == "CollisionSoundIntensity"){
		in>>m_CollisionSoundIntensity>>dummy;
		return true;
	}
	else if (field == "ObjID"){
		in>>m_id>>dummy;
		return true;
	}
	else if (field == "SirenSoundID") {
		in >> m_SirenSoundID >> dummy;
		return true;
	}
	else if (field == "SirenSoundIntensity") {
		in >> m_SirenSoundIntensity >> dummy;
		return true;
	}
	else if (field == "Option") {
		CSolOption option;
		in >> option.switchChild;
		if (ParseString(in, option.name)) {
			if (ParseString(in, option.bitmap)) {
				if (ParseString(in, option.modelLow)) {
					if (ParseString(in, option.modelHigh)) {
						in >> dummy;
						m_options.push_back(option);
						return true;
					}
					else return false;
				}
				else return false;
			}
			else return false;
		}
		else return false;
	}
	else return false;
}


bool CSolObj::ParseString(ifstream& in, string& value)
{
	value = "";
	char c;
	in.get(c);
	while (c != '"') {
		in.get(c);
		if (in.fail()) return false;
	}

	do {
		in.get(c);
		if (c != '"') value += c;
		if (in.fail()) return false;
	} while (c != '"');

	return true;
}

bool CSolObj::ParseUntilEnd(ifstream& in, string& value)
{
	value = "";
	char c;

	do {
		in.get(c);
		value += c;
		if (in.fail()) return false;
	} while (value.find("&end&") == value.npos);

	return true;
}


//////////////////////////////////////////////////////////////////////////
bool CSolObj::Parse(ifstream& in)
{
	string dummy, str;

	string token;
	in >> m_name >> token;
	while (token != "&end&") {
		if (!ParseCommonField(token, in))
			return false;
		in >> token;
	}
	return true;
}


const string& CSol::GetChecksum() const
{
	return m_checksum;
}


const CSolObj* CSol::GetObj(const string& name) const
{
	map<string, int>::const_iterator i = m_nameToId.find(name);
	if (i == m_nameToId.end()) return NULL;
	else{
		int index = i->second;
		map<int,CSolObj*>::const_iterator iPobj =  m_objs.find(index);
		if (iPobj == m_objs.end()) return NULL;
		return iPobj->second;
	}
}


const CSolObj* CSol::GetObj(unsigned int id) const
{

	map<int,CSolObj*>::const_iterator iPobj =  m_objs.find(id);
	if (iPobj == m_objs.end()) return NULL;
	return iPobj->second;
	
}


bool CSolObj::CanBe(int mask) const
{
	assert(mask >= 1 &&  mask <= 7);
	if ((mask & cSOL_ADO) && !(m_canBe & cSOL_ADO)) return false;
	if ((mask & cSOL_DDO) && !(m_canBe & cSOL_DDO)) return false;
	if ((mask & cSOL_STATIC) && !(m_canBe & cSOL_STATIC)) return false;
	return true;
}


bool CSolObjVehicle::Parse(ifstream& in)
{
	string dummy, str;

	string token;
	in >> m_name >> token;
	while (token != "&end&") {
		if (!ParseCommonField(token, in)) {
			if (token == "HornSoundId") {
				in >> m_hornSoundId >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "HornSoundIntensity") {
				in >> m_hornSoundIntensity >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "CollisionVehSoundIntensity") {
				in >> m_collisionVehSoundIntensity >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "PassingVehSoundId") {
				in >> m_passingVehSoundId >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "PassingVehSoundIntensity") {
				in >> m_passingVehSoundIntensity >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "Colors") {
				m_colors.clear();
				CSolColor color;
				while (!in.fail()) {
					string temp;
					int g, b;
					in >> temp;
					if (temp == "&end&") break;

					in >> g >> b >> color.cigi;
					color.r = atoi(temp.c_str());
					color.g = g;
					color.b = b;
					m_colors.push_back(color);
				}
			}
			else if (token == "Axles") {
				m_axles.clear();
				pair<int,float> tOffset;
				while (!in.fail()) {
					string temp;
					float x,y;
					x =0; y = 0;

					in >> temp;
					if (temp == "&end&") break;

					in >>y;
					tOffset.first = atoi(temp.c_str());
					tOffset.second = y;
					m_axles.push_back(tOffset);
				}
			}

			else if (token == "DynaParams") {
				if (!ParseDynaParams(in)) return false;
			}
			else if (token == "SpecialOptions") {
				m_colors.clear();
				CSolColor color;
				CSpecialOption currOption;
				while (!in.fail()) {
					string temp;
					//int  b;
					in >> token;
					if (token == "Switch") {
						currOption.m_switchOptions.clear();
						m_colors.clear();
						CSolColor color;
						while (!in.fail()) {
							string temp;
							int num;
							in >> temp;
							if (temp == "&end&"){ 
								m_specialOptions.push_back(currOption);
								break;
							}
							else if (temp == "Name"){
								in >> temp;
								currOption.name = temp;
							}
							else if (temp == "Description"){
								ParseString(in, currOption.desc);
							}
							else if (temp == "Option"){
								in >> num;
								ParseString(in, temp);
								currOption.m_switchOptions[num] = temp;
							}
						}
					}
					else if (token == "&end&"){
						break;
					}
				}
			}
			else if (token == "LateralAccelLimitGs"){
				in >> m_lateralAccelLimitInMs2>> dummy;
				if (in.fail()) return false;
				m_lateralAccelLimitInMs2 *=cGRAVITY;
			}

			else if (token == "VehBrakesSoundID") {
				in >> m_VehBrakesSoundID >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "VehBrakesSoundIntensity") {
				in >> m_VehBrakesSoundIntensity >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "VehMufflerSoundID") {
				in >> m_VehMufflerSoundID >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "VehMufflerSoundIntensity") {
				in >> m_VehMufflerSoundIntensity >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "VehWheelSoundIntensity"){
				in >>m_WheelVolIntensity>>dummy;
				if (in.fail()) return false;
			}
			else if (token == "DefaultSoundID") {
				in >> m_DefaultSoundID >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "DefaultSoundIntesity") {
				in >> m_DefaultSoundIntesity >> dummy;
				if (in.fail()) return false;
			}
			else if (token == "Category") {
				string cat;
				in >> cat >> dummy;
				if (in.fail()) return false;
				if (cat == "TRUCK") m_type |= cSOL_TRUCK;
				else if (cat == "CAR") m_type |= cSOL_CAR;
				else if (cat == "SUV") m_type |= cSOL_SUV;
				else if (cat == "VAN") m_type |= cSOL_VAN;
				else if (cat == "UTILITY") m_type |= cSOL_UTILITY;
				else if (cat == "BUS") m_type |= cSOL_BUS;
				else if (cat == "POLICE") m_type |= cSOL_POLICE;
				else if (cat == "AMBULANCE") m_type |= cSOL_AMBULANCE;
				else if (cat == "MOTORCYCLE") m_type |= cSOL_MOTORCYCLE;
				else if (cat == "COMMERCIAL") m_type |= cSOL_COMMERCIAL;
				else if (cat == "SPECIAL") m_type |= cSOL_SPECIAL;
				else if (cat == "OWNSHIP") m_type |= cSOL_OWNSHIP;
				else return false;
			}
			else if (token == "EyepointOffset") {
				in >> m_eyepointOffset.m_x >> m_eyepointOffset.m_y >> m_eyepointOffset.m_z >> dummy;
				if (in.fail()) return false;
			}
			else {
				// ERROR
			}
		}
		in >> token;
	}

	return true;
}



bool CSolObjVehicle::ParseDynaParams(ifstream& in) {
	string str;
	if (ParseUntilEnd(in, str)) {
		m_dynaParams = CDynaParams(str);
	} 
	else {
		return false;
	}
	return true;
}

bool CSolObjComposite::Parse(ifstream& in)
{
	string dummy, str;
	
	string token;
	in >> m_name >> token;
	while (token != "&end&") {
		if (!ParseCommonField(token, in)) {
			if (token == "Reference") {
				string refName;
				if (ParseString(in, refName)) {
					CCompositeComponent comp;
					comp.solRef = refName;
					in >> comp.childNum;
					in >> comp.x;
					in >> comp.y;
					in >> comp.width;
					in >> comp.height;
					in >> comp.group;
					in >> dummy;
					m_references.push_back(comp);
				}
			}
			else {
				// Unknown token?
				return false;
			}
		}
		in >> token;
	}

	return true;
}



bool CSolObjTerrain::Parse(ifstream& in)
{
	string dummy, str;

	string token;
	in >> m_name >> token;
#if 0
	cout<<"Parsing terrain obj "<<m_name<<endl;
#endif
	while (token != "&end&") {
		if (!ParseCommonField(token, in)) {
			if (token == "Map") {
				if (!ParseMap(in)) return false;
			}
            else if (token == "MapFile"){
                if (!ParseMapFile(in)) return false;
            }
			else if (token == "ElevIsDelta") {
				int elevIsDelta;
				in >> elevIsDelta >> dummy;
				m_elevIsDelta = (elevIsDelta != 0);
				if (in.fail()) return false;
			}
			else if (token == "SurfaceObjectSoundID") {
				in >> m_surfaceObjectSoundId >> dummy;
				if (in.fail()) return false;
			}
		}
		in >> token;
	}

	return true;
}

bool CSolObjWalker::Parse(ifstream& in) {

	double	m_animationSpeed; //< Speed the base speed for the key-frame animation
	string dummy, str;

	string token;
	in >> m_name >> token;
	while (token != "&end&") {
		if (!ParseCommonField(token, in)) {
			if (token == "AnimationSpeed") {
				in >> m_animationSpeed >> dummy;
				if (in.fail()) return false;
			}
		}
		in >> token;
	}

	return true;
	// TBD
};
bool CSolObjDiGuy::Parse(ifstream& in) {

	string dummy, str;

	string token;
	in >> m_name >> token;
	while (token != "&end&") {
		if (!ParseCommonField(token, in)) {

		}
		in >> token;
		if (in.bad())
			return false;
	}

	return true;
	// TBD
};


bool CSolObjTerrain::ParseMap(ifstream& in) {
	string str;
	if (ParseUntilEnd(in, str)) {
		str.erase(str.find("&end&"));
		m_map = CTerrainGrid<Post>(str);
	} 
	else {
		return false;
	}
	return true;
}

bool CSolObjTerrain::ParseMapFile(ifstream& in) {
	string token;
    stringstream lineparser;
    int numrows;
    int numcols;
    int xres;
    int yres;
    int currPoint =0;
    CBoundingBox box;
    vector<Post> data;

    string fileName;
	in>>fileName;
    string fullFileName;
    int lineCnt = 0;
    string EnvVar;
    NADS::GetEnvVar(EnvVar,"NADSSDC_SCN");
    if (EnvVar != "")
        fullFileName = EnvVar;
    else 
        return false;
    fullFileName+=fileName;
    if (fileName.find(".txt") != string::npos){
		ifstream mapFile(fullFileName.c_str());
        if (!mapFile.is_open()){
            return false;
        }
        while(!mapFile.eof()&& mapFile.good()){
            
            char buffer[65536];
            memset(buffer,0,sizeof(buffer));
            mapFile.getline(buffer,sizeof(buffer)/sizeof(char));
            lineparser.clear();
            lineparser.str(buffer);
            char op;
            
            lineparser>>token>>op;
            if (!lineparser.fail() && lineparser.good()){
               lineCnt++;
               if (token == "bbox"){
                  if (op == '='){
                    float tempMinX,tempMinY, tempMaxX,tempMaxY;
                    lineparser>>tempMinX>>tempMinY>>tempMaxX>>tempMaxY;
                    box.SetMinX(tempMinX);box.SetMaxX(tempMaxX);
                    box.SetMinY(tempMinY);box.SetMaxY(tempMaxY);
                  }
               }else if (token == "xres"){
                  if (op == '='){
                      lineparser>>xres;
                  }
               }else if (token == "yres"){
                  if (op == '='){
                      lineparser>>yres;
                  }
               }else if (token == "numrows"){
                  if (op == '='){
                      lineparser>>numrows;
                  }
               }else if (token == "numcols"){
                  if (op == '='){
                      lineparser>>numcols;
                  }
               }else if (token == "data"){
                  if (data.size() == 0){
                      data.resize(numrows*numcols);
                  }
                  if (op == '='){
                      if ((int) data.size() > currPoint){
                          int itemCnt = 0;
                          while(!lineparser.fail() && (int) data.size() > currPoint){
                              lineparser>>data[currPoint].z>>data[currPoint].type;
                              if (!lineparser.fail()){
                                  currPoint++;
                                  itemCnt++;
                                  if (data.size() == currPoint){
                                      //check to see if we have too much data, next read should fail. 
                                      double tempf1 = 0,tempf2 = 0;
                                      lineparser>>tempf1>>tempf2;
                                      if (!lineparser.fail()){
                                        return false; //too much data
                                      }
                                      break;
                                      streampos pos = lineparser.tellg();
                                  }
                              }else{
                                break;
                              }
                          }
                      }else{
                        return false;
                      }
                  }
               }
            }//parse partialy tokenized line
        }//parse line
    }
    m_map = CTerrainGrid<Post>(box,numcols,numrows,data,true,NULL);

	return true;
}