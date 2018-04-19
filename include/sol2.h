/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2004 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: sol2.h,v 1.48 2016/08/18 17:10:48 IOWA\dheitbri Exp $
//
// Author(s):	Matt Schikore
// Date:		Feb 2004
//
// Description:	The new SOL 2.0 classes.  Now embedded in CVED, this
//              replaces the solgen module.
//
//////////////////////////////////////////////////////////////////////


#ifndef __SOL2_H_
#define __SOL2_H_

#pragma warning(disable:4786)

#include <string>
#include <vector>
#include <map>
#include <set>

#include <TerrainGrid.h>
#include <DynaParams.h>
#include <RandNumGen.h>

using namespace std;

/// VEHICLE TYPES
#define cSOL_TRUCK        1
#define cSOL_CAR          2
#define cSOL_SUV          4
#define cSOL_VAN          8
#define cSOL_UTILITY      16
#define cSOL_BUS          32
#define cSOL_POLICE       64
#define cSOL_AMBULANCE    128
#define cSOL_MOTORCYCLE   256
#define cSOL_COMMERCIAL   512
#define cSOL_SPECIAL      1024
#define cSOL_OWNSHIP      2048


/// OBJECT TYPE SPECIFICATIONS
#define cSOL_ADO		1
#define cSOL_DDO		2
#define cSOL_STATIC		4


class CSolColor {
public:
	CSolColor(){
		cigi = 0;
		r = 0;
		g = 0;
		b = 0; 
	}
	int                 cigi;
	unsigned char       r, g, b;
};


class CSolOption {
public:
	int					switchChild;
	string				name;
	string				bitmap;
	string				modelLow;
	string				modelHigh;
};


class CSolObj {
	friend class CSol;
public:
	CSolObj(int id);
	virtual ~CSolObj() {};
	string       GetName() const { return m_name; }
	string 		 GetFriendlyName() const { 
		if (m_friendlyName.empty())
			return m_name;
		return m_friendlyName; 
	}

	int                 GetId() const { return m_id; }

	double              GetLength() const { return m_length; }
	double              GetWidth() const { return m_width; }
	double              GetHeight() const { return m_height; }

	const string&       GetVisModelName() const { return m_visModelName; }
	int					GetVisModelCigiId() const { return m_visModelCigiId; }

	bool                CanBe(int mask) const;

	const string&       GetIsatModelBitmap() const { return m_isatBitmap; }
	const string&       GetIsatModelLowLod() const { return m_isatModelLow; }
	const string&       GetIsatModelHighLod() const { return m_isatModelHigh; }
	double              GetIsatBboxSize() const { return m_isatBbox; }

	const string&       GetCategoryName() const { return m_categoryName; }

	const vector<CSolOption>&	GetOptions() const { return m_options; }

	int					GetNumOptions() const { return (int)m_options.size(); }
	const CSolOption&	GetOption(int num) const { return m_options[num]; }

	double				GetFrictionCoeff() const { return m_frictionCoeff; }
	double				GetBounceEnergyLoss() const { return m_bounceEnergyLoss; }
	int					GetSirenSoundIntensity()		{return m_SirenSoundIntensity;}

	int					GetDefaultSoundId() const {return m_DefaultSoundID;}
	int					GetDefaultSoundVol() const {return m_DefaultSoundIntesity;}
	int					GetCollisionSoundID() const {return m_CollisionSoundID;}
	int					GetSirenSoundID()	const			{return m_SirenSoundID;}

	virtual bool        Parse(ifstream& in);
private:
	int                 m_id;
	double              m_length;
	double				m_width;
	double              m_height;
	double              m_isatBbox;
	string              m_visModelName;
	int					m_visModelCigiId;
	int                 m_canBe;
	string              m_isatBitmap;
	string              m_isatModelLow;
	string              m_isatModelHigh;
	double				m_frictionCoeff;
	double				m_bounceEnergyLoss;
	int					m_SirenSoundID;
	int					m_SirenSoundIntensity;

	vector<CSolOption>	m_options;

protected:
	string				m_categoryName;
	string              m_name;
	string              m_friendlyName; //<name to use in user interface
	//ID's used with AIX sound engine
	int					m_DefaultSoundID;
	int					m_DefaultSoundIntesity;
	int					m_CollisionSoundID;
	int					m_CollisionSoundIntensity;

	bool				ParseCommonField(const string& field, ifstream& in);
	bool				ParseString(ifstream& in, string& value);
	bool				ParseUntilEnd(ifstream& in, string& value);
};

class CSpecialOption{
public:
	string name;
	string desc;
	map<int,string> m_switchOptions; 
};

class CSolObjVehicle : public CSolObj {
public:
	CSolObjVehicle(int id) : CSolObj(id), m_type(0),m_WheelVolIntensity(100),m_CollisionSoundID(0),m_lateralAccelLimitInMs2(5) 
	                                               { m_categoryName = "Vehicle"; 
													 CSolColor tColor; tColor.r = 0; tColor.g=0;tColor.b =0; tColor.cigi =0;
													 m_colors.push_back(tColor);}
	int                 GetType() const { return m_type; }
	bool                IsType(int type) const { return (m_type & type) != 0; }

	int                 GetColorCigiId(int colorIndex) const { return m_colors[colorIndex].cigi + GetVisModelCigiId(); }
	const CSolColor&    GetColor(int index) const { if (index > 0 && index < (int)m_colors.size() )
														return m_colors[index];
													else
														return m_BlankColor;

													}
	const vector<CSolColor>& GetColors() const { return m_colors; }
	const vector<pair<int,float>>& GetAxles() const {return m_axles;}
	const vector<CSpecialOption>& GetSpecialOptions() const {return m_specialOptions;};
	const CDynaParams&  GetDynaParams() const { return m_dynaParams; }

	int                 GetHornSoundId() const { return m_hornSoundId; }
	int                 GetHornSoundIntensity() const 
	                         { return m_hornSoundIntensity; }
	int                 GetCollisionVehSoundIntensity() const 
	                         { return m_collisionVehSoundIntensity; }
	int                 GetPassingVehSoundId() const 
	                         { return m_passingVehSoundId; }
	int                 GetPassingVehSoundIntensity() const 
	                         { return m_passingVehSoundIntensity; }
	CPoint3D            GetEyepointOffset() const
	                         { return m_eyepointOffset; }
	virtual bool        Parse(ifstream& in);

	
	
	
	int					GetVehBrakesSoundID()			{return m_VehBrakesSoundID;}
	int					GetVehBrakesSoundIntensity()	{return m_VehBrakesSoundIntensity;}
	int					GetVehMufflerSoundID()			{return m_VehMufflerSoundID;}
	int					GetVehMufflerSoundIntensity()	{return m_VehMufflerSoundIntensity;}
	int					GetNumColors()	const			{return (int)m_colors.size();}
	int					GetWheelVol() const				{return m_WheelVolIntensity;}
	int                 GetIsCabType() const            {return m_isCabType;}
	double				GetLatAccelLimit() const	    {return m_lateralAccelLimitInMs2;}

private:
	int                 m_type;
	vector<CSolColor>   m_colors;
	vector<pair<int,float>>	m_axles;
	vector<CSpecialOption> m_specialOptions;
	CDynaParams         m_dynaParams;
	int                 m_hornSoundId;
	int                 m_hornSoundIntensity;
	int                 m_collisionVehSoundIntensity;
	int                 m_passingVehSoundId;
	int                 m_passingVehSoundIntensity;
	CPoint3D			m_eyepointOffset;
	//ID's used with AIX sound engine
	int					m_CollisionSoundID;
	int					m_VehBrakesSoundID;
	int					m_VehBrakesSoundIntensity;
	int					m_VehMufflerSoundID;
	int					m_VehMufflerSoundIntensity;
	int					m_WheelVolIntensity;
	bool                m_isCabType;
	double				m_lateralAccelLimitInMs2; //<limit on lateral accleration in meters per seconds^2
protected:
	CSolColor			m_BlankColor;
	bool				ParseDynaParams(ifstream& in);
};


class CSolObjRailVeh : public CSolObj {
public:
	CSolObjRailVeh(int id) : CSolObj(id) { m_categoryName = "RailVeh"; }
// TBD
};

class CSolObjTrailer : public CSolObj {
public:
	CSolObjTrailer(int id) : CSolObj(id) { m_categoryName = "Trailer"; }
	// TBD
};

class CSolObjWalker : public CSolObj {
public:
	CSolObjWalker(int id) : CSolObj(id) { m_categoryName = "Walker"; m_animationSpeed = -1.0f; }
	double GetAnimationSpeed() {return m_animationSpeed;}
	virtual bool              Parse(ifstream& in);
protected:
	double	m_animationSpeed; //< Speed the base speed for the key-frame animation
	// TBD
};

class CSolObjDiGuy : public CSolObj {
public:
	CSolObjDiGuy(int id) : CSolObj(id) { m_categoryName = "DiGuy"; }
	virtual bool              Parse(ifstream& in);
protected:
	//string m_digGuyName
};


class CSolObjObstacle : public CSolObj {
public:
	CSolObjObstacle(int id) : CSolObj(id) { m_categoryName = "Obstacle"; }
	// TBD
};


class CSolObjPoi : public CSolObj {
public:
	CSolObjPoi(int id) : CSolObj(id) { m_categoryName = "Poi"; }
	// TBD
};

class CSolObjSpecialEffect : public CSolObj {
public:
	CSolObjSpecialEffect(int id) : CSolObj(id) { m_categoryName = "SpecialEffect"; }
	// TBD
};

class CSolObjTrafLight : public CSolObj {
public:
	CSolObjTrafLight(int id) : CSolObj(id) { m_categoryName = "TrafLight"; }
};


class CSolObjTerrain : public CSolObj {
public:
	CSolObjTerrain(int id) : CSolObj(id) { m_categoryName = "Terrain"; m_surfaceObjectSoundId = 0; }
	const CTerrainGrid<Post>& GetMap() const { return m_map; }
	bool                      GetElevIsDelta() const { return m_elevIsDelta; }
	int                       GetSurfaceObjectSoundId() const { return m_surfaceObjectSoundId; }

	virtual bool              Parse(ifstream& in);
private:
	bool                      m_elevIsDelta;
	int                       m_surfaceObjectSoundId;
	CTerrainGrid<Post>        m_map;
protected:
	bool                      ParseMap(ifstream& in);
    bool                      ParseMapFile(ifstream& in);
};


class CSolObjTrafSign : public CSolObj {
public:
	CSolObjTrafSign(int id) : CSolObj(id) { m_categoryName = "TrafSign"; }
	// TBD
};

class CSolObjCoordinator : public CSolObj {
public:
	CSolObjCoordinator(int id) : CSolObj(id) { m_categoryName = "Coordinator"; }
	// TBD
};

struct CCompositeComponent
{
   int group;
   int childNum;
   string solRef;
   int x, y;
   int width, height;
};

class CSolObjComposite : public CSolObj {
public:
	CSolObjComposite(int id) : CSolObj(id) { m_categoryName = "Composite"; }
	
	const vector<CCompositeComponent>&     GetReferences() const { return m_references; }
	virtual bool              Parse(ifstream& in);
private:
	vector<CCompositeComponent> m_references;
};

/////////////////////////////////////////////////////////////////////////////
///\brief
///    Class for parsing/providing access to sol2 file
///\remark
///    This class parses and provides access to the sol2.txt file, and all
/// the files matching the pattern *sol2.txt. The sol2.txt file is considered
/// critical. If this file has a parse error this class will fail the parse.
/// Any other errors secondary sol2 files will not. This is done as these are
/// used for external sites to NADS, were the chances of parse errors are high,
/// and a parse error made could render an installed site inoperable. 
///
///\remark
///	   The sol2 files are to be located in the dir spec'ed by the env var
/// <b>NADSSDC_SCN</b>. A log of all error critical or not are return by
///  GetParseLog.
/////////////////////////////////////////////////////////////////////////////
class CSol {
public:
	CSol() : 
		  m_init(false),m_failed(false),m_objCnt(0),m_hasParseError(false)
	  {}
	bool                Init(bool ForceAfterFail = false);
	bool				IsInitialized() { return m_init; }
	const string&       GetChecksum() const;

	const CSolObj*      GetObj(const string& name) const;
	const CSolObj*      GetObj(unsigned int id) const;

	int                 GetAllVehicles(vector<int>&,			
	                        int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC,
							int typeMask = cSOL_TRUCK | cSOL_CAR | cSOL_SUV |
							cSOL_VAN | cSOL_UTILITY | cSOL_BUS | cSOL_POLICE |
							cSOL_AMBULANCE | cSOL_MOTORCYCLE | cSOL_COMMERCIAL |
							cSOL_SPECIAL| cSOL_OWNSHIP, bool sort = false) const;
	int                 GetAllRailVehs(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;		
	int                 GetAllTrailers(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;   
	int                 GetAllWalkers(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;   
    int                 GetAllDiGuys(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;       
	int                 GetAllObstacles(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;    
	int                 GetAllPois(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC) const;			
	int                 GetAllTerrains(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC) const;		
	int                 GetAllTrafSigns(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;		
	int                 GetAllCoordinators(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC) const;	
	int                 GetAllSpecialEffects(vector<int>&, int canBeMask = cSOL_ADO | cSOL_DDO | cSOL_STATIC, bool sort = false) const;	


	const string		PickRandomSolObjectFromCategory(const string& name, const vector<int>& excludeIds) const;
	const string&		GetLastError() const { return m_lastError; }
	bool                GetHasParseErrors() const { return m_hasParseError;}
	const string&       GetParseLog() const{return m_parseLog;}
private:
	//vector<CSolObj*>    m_objs;
	map<int,CSolObj*>	m_objs;
	string              m_checksum;
	map<string, int>    m_nameToId;
	int					m_objCnt;

	bool				ParseHeader(ifstream& in);
	string				m_lastError;
	string              m_parseLog;
	bool                m_hasParseError; //<AUX sol2 files are considered "non-critical"
	bool				m_init;
	map<string, vector<int> > m_typeToElements;
	bool				ParseFile(ifstream& in);
	bool				m_failed;
};




#endif // __SOL2_H_