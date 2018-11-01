//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: dynobj.h,v 1.73 2016/07/15 14:39:53 IOWA\dheitbri Exp $
//
// Author(s):   Yiannis Papelis
// Date:        August, 1998
//
// Description: The declaration for the CDynObj class.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __DYN_OBJ_H
#define __DYN_OBJ_H	// {secret}

#include "cvedpub.h"

struct cvTObjAttr;
union  cvTObjState;
union  cvTObjContInp;

namespace CVED {

//////////////////////////////////////////////////////////////////////////////
//
// This class represents a dynamic object in the virtual environment.
// The dynamic object class derives from the CObj class and thus
// possesses all features of static objects.
//
// This class is an abstract class that cannot be instanced directly.
// However, there will be one class derived per each object type
// defined in the current version of CVED.
//
//////////////////////////////////////////////////////////////////////////////
class CDynObj : public CObj
{
public:
	CDynObj();
	virtual ~CDynObj();
	CDynObj( const CDynObj& );
	CDynObj& operator=( const CDynObj& );

	void MakeReadOnly( void );

	virtual void SetPos( const CPoint3D& cPos, bool useDoubleBuffer = true );
	virtual void SetTan( const CVector3D& cTan, bool useDoubleBuffer = true );
	virtual void SetLat( const CVector3D& cLat, bool useDoubleBuffer = true );
	virtual void SetVel( double vel, bool useDoubleBuffer = true );

protected:
	CDynObj( const CCved&, TObj* );

	friend class CCved;
	bool m_readOnly;
};

//////////////////////////////////////////////////////////////////////////////
//
// This class represents objects that operate as trajectory followers.
// Trajectory followers follow a predefined path without concern
// over roads or other obstructions, and have very limited reactive
// behavior.
//
//////////////////////////////////////////////////////////////////////////////
class CTrajFollowerObj : public CDynObj
{
public:
	CTrajFollowerObj();
	CTrajFollowerObj( const CCved&, TObj* );
	CTrajFollowerObj( const CTrajFollowerObj& );
	CTrajFollowerObj& operator=( const CTrajFollowerObj& );
	virtual ~CTrajFollowerObj();


	void SetTargDir( const CVector2D& );
	void SetObjOri( const CVector2D& );
	void SetTargVel( double );

	TU16b GetVisualState( void ) const;
	void SetVisualState( TU16b );
	TU16b GetAudioState( void ) const;
	void SetAudioState( TU16b );
	void SetAnimationState(bool);
	bool GetAnimationState(void) const;
	// mode transition related functions
	void InitializeMode( unsigned char );
	unsigned char GetCurrentMode( void ) const;
	void SetCurrentMode( unsigned char );
//	unsigned char GetPreviousMode ( void ) const;
//	void SetPreviousMode( unsigned char );
    int GetParentId( void ) const;
	void SetParentId( int );
	void GetOffset( double offset[6] );
	void SetOffset( const double offset[6] );
	void GetInitPosRotVel( double initPosRot[6], double initVel[6] );
	void SetInitPosRotVel( const double initPosRot[6], const double initVel[6] );
	void InitModeValues( void );
};


//////////////////////////////////////////////////////////////////////////////
//
// This class represents general vehicles.  Examples of vehicles include
// cars, small trucks and SUVs.
//
//////////////////////////////////////////////////////////////////////////////
class CVehicleObj : public CDynObj
{
public:
	CVehicleObj();
	CVehicleObj( const CCved&, TObj* );
	CVehicleObj( const CVehicleObj& );
	CVehicleObj& operator=( const CVehicleObj& );
	virtual ~CVehicleObj();

	void SetTargDist( const double& );
	void SetTargPos( const CPoint3D& );
    void StoreOldTargPos();
	void SetTargVel( const double& );
	void SetLeadObj( const int );
	void SetLeadTargDist( const double& );
	void SetLeadActualDist( const double& );
	void SetHaveSteer( bool );
	void SetTargSteer( const double& );
	void SetHaveAccel( bool );
	void SetTargAccel( const double& );
    void SetSteerMax( const double& cSteerMax );

	double GetSuspStif( void ) const;
	void   SetSuspStif( const double& );
	double GetSuspStifImm( void ) const;
	void   SetSuspStifImm( const double& );
	double GetSuspDamp( void ) const;
	void   SetSuspDamp( const double& );
	double GetSuspDampImm( void ) const;
	void   SetSuspDampImm( const double& );
	double GetTireStif( void ) const;
	void   SetTireStif( const double& );
	double GetTireStifImm( void ) const;
	void   SetTireStifImm( const double& );
	double GetTireDamp( void ) const;
	void   SetTireDamp( const double& );
	double GetTireDampImm( void ) const;
	void   SetTireDampImm( const double& );
	double GetVelBrake( void ) const;
	void   SetVelBrake( const double& );
	cvTFourWheelVeh GetStateVector( void ) const;
	void   SetStateVector( const cvTFourWheelVeh& );
	cvTVehLin GetVehLin( void ) const;
	void   SetVehLin( const cvTVehLin& );
	int    GetDynaInitComplete( void ) const;
	void   SetDynaInitComplete( int );
	int    GetDynaInitCompleteImm( void ) const;
	void   SetDynaInitCompleteImm( int );
	EDynaFidelity GetDynaFidelity( void ) const;
	void   SetDynaFidelity( const EDynaFidelity );
	void   SetDynaFidelityImm( const EDynaFidelity );
	int    GetQryTerrainErrCount( void ) const;
	void   SetQryTerrainErrCount( const int );
	void   SetQryTerrainErrCountImm( const int );

	TU16b  GetVisualState( void ) const;
	TU16b  GetVisualStateImm( void ) const;
	void   SetVisualState( TU16b );
	TU16b  GetAudioState( void ) const;
	void   SetAudioState( TU16b );

	double GetLatAccel( void ) const;
	double   GetSteeringAngle(void) const;
    virtual double GetAccel( void ) const;

	float  GetTireRotation( int ) const;
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents trailers.  Trailers are towed behind
// other entities.
//
class CTrailerObj : public CDynObj
{
public:
	CTrailerObj();
	virtual ~CTrailerObj();
	CTrailerObj( const CTrailerObj& );
	CTrailerObj& operator=( const CTrailerObj& );

	CTrailerObj( const CCved&, TObj* );
};

/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents trailers towed by external objects outside
// the virtual environment, while CTrailerObj are trailers towed
// by other entities inside the virtual environment.
//
class CExternalTrailerObj : public CDynObj
{
public:
	CExternalTrailerObj();
	virtual ~CExternalTrailerObj();
	CExternalTrailerObj( const CExternalTrailerObj& );
	CExternalTrailerObj& operator=( const CExternalTrailerObj& );

	CExternalTrailerObj( const CCved&, TObj* );
};

/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents rail vehicles such as trains cars, engines
// etc.  Rail vehicles can be chained to allow creation of long
// strings.  Rail vehicles don't have much behavior other than
// going down the tracks.
//
class CRailVehObj : public CDynObj
{
public:
	CRailVehObj();
	virtual ~CRailVehObj();
	CRailVehObj( const CRailVehObj& );
	CRailVehObj& operator=( const CRailVehObj& );

	CRailVehObj( const CCved& , TObj* );
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents terrain objects.  Terrain objects
// change the underlying elevation height and surface properties.
//
class CTerrainObj : public CDynObj
{
public:
	CTerrainObj();
	virtual ~CTerrainObj();
	CTerrainObj( const CTerrainObj& );
	CTerrainObj& operator=( const CTerrainObj& );

	CTerrainObj( const CCved&, TObj* );
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents traffic lights.
//
class CTrafficLightObj : public CDynObj
{
public:
	CTrafficLightObj();
	virtual ~CTrafficLightObj();
	CTrafficLightObj( const CTrafficLightObj& );
	CTrafficLightObj& operator=( const CTrafficLightObj& );

	CTrafficLightObj( const CCved&, TObj* );

	eCVTrafficLightState GetState( void ) const;
	void SetState( const eCVTrafficLightState& );

};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents traffic signs.  Traffic signs can
// affect the way autonomous behaviors operate since they modify
// the rules of the road.
//
class CTrafficSignObj : public CDynObj
{
public:
	CTrafficSignObj();
	virtual ~CTrafficSignObj();
	CTrafficSignObj( const CTrafficSignObj& );
	CTrafficSignObj& operator=( const CTrafficSignObj& );

	CTrafficSignObj( const CCved&, TObj* );
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents composite signs.
//
class CCompositeSignObj : public CDynObj
{
public:
	CCompositeSignObj();
	virtual ~CCompositeSignObj();
	CCompositeSignObj( const CCompositeSignObj& );
	CCompositeSignObj& operator=( const CCompositeSignObj& );

	CCompositeSignObj( const CCved&, TObj* );
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents various obstacles.  Examples include
// cones, trees, cultural objects etc.
//
class CObstacleObj : public CDynObj
{
public:
	CObstacleObj();
	virtual ~CObstacleObj();
	CObstacleObj( const CObstacleObj& );
	CObstacleObj& operator=( const CObstacleObj& );

	CObstacleObj( const CCved&, TObj* );
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents POIs (point-of-interests).
//
class CPoiObj : public CDynObj
{
public:
	CPoiObj();
	virtual ~CPoiObj();
	CPoiObj( const CPoiObj& );
	CPoiObj& operator=( const CPoiObj& );

	CPoiObj( const CCved&, TObj* );
};


/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents scenario coordinators.  These are invisible
// entities used to coordinate scenarios during execution.
//
class CCoordinatorObjectObj : public CDynObj
{
public:
	CCoordinatorObjectObj();
	virtual ~CCoordinatorObjectObj();
	CCoordinatorObjectObj( const CCoordinatorObjectObj& );
	CCoordinatorObjectObj& operator=( const CCoordinatorObjectObj& );

	CCoordinatorObjectObj( const CCved&, TObj* );
};

/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents visual object.  These are invisible
// entities used to coordinate scenarios during execution.
//
class CVisualObjectObj : public CDynObj
{
public:
	CVisualObjectObj();
	virtual ~CVisualObjectObj();
	CVisualObjectObj( const CVisualObjectObj& );
	CVisualObjectObj& operator=( const CVisualObjectObj& );
	CVisualObjectObj( const CCved&, TObj* );
	void SetColor(float R,float G, float B, float A, bool doublebuffer = true);
	void SetRotation(float Rotation, bool doublebuffer = true);
	void SetBoarderColor(float R,float G, float B, float A, bool doublebuffer = true);
	void SetDrawPosition(const CPoint3D &pos, bool doublebuffer = true);
	void SetDrawType(TU8b objType, bool doublebuffer = true);
	void SetDrawScreen(TU8b objType, bool doublebuffer = true);
	void SetDrawSize(float x, float y, bool doublebuffer = true);
	void SetTargetId(int id, bool doublebuffer = true);


	void GetColor(float &R,float &G, float &B, float &A) const;
	void GetDrawSize(float &x, float &y) const;
	float GetRotation() const;
	void GetBoarderColor(float &R,float &G, float &B, float &A) const;
	int GetTargetId() const;
	CPoint3D GetDrawPosition() const;
	TU8b GetDrawType() const;
	TU8b GetDrawScreen() const;
	TU16b GetStateIndex() const;
	void SetStateIndex(TU16b, bool doublebuffer = true);
};
/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents objects whose behavior is generated
// outside the simulator's virtual environment.  It primarily
// includes the object driver by the simulator driver, and in the
// future may hold additional such drivers when multiple simulators
// are linked.
//
class CExternalDriverObj : public CVehicleObj
{
public:
	CExternalDriverObj();
	virtual ~CExternalDriverObj();
	CExternalDriverObj( const CExternalDriverObj& );
	CExternalDriverObj& operator=( const CExternalDriverObj& );

	CExternalDriverObj( const CCved&, TObj*  );

	void SetAccel( double acc, bool useDoubleBuffer = true );
	void SetAngularVel( const CVector3D& angularVel, bool useDoubleBuffer = true );

	double GetAccel( void ) const;

	//bool IsOwnVehicle() const;
	//const cvTObjStateBuf& GetState( void ) const;
	//void Update(const cvTObjStateBuf&);

	// Note: these are redundant and should be removed.
	CPoint3D      GetPosHighFreq( void ) const;
	CVector3D     GetTanHighFreq( void ) const;
	CVector3D     GetLatHighFreq( void ) const;
	CBoundingBox  GetBoundBoxHighFreq( void ) const;
	double        GetAccelHighFreq( void ) const;
	CVector3D     GetAngularVelHighFreq( void ) const;
};




/////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents walkers.
//
class CWalkerObj : public CDynObj
{
public:
	CWalkerObj();
	virtual ~CWalkerObj();
	CWalkerObj( const CWalkerObj& );
	CWalkerObj& operator=( const CWalkerObj& );

	CWalkerObj( const CCved&, TObj* );
	void SetAnimationState(bool isOn);
};

class CAvatarBase
{
public:
	CAvatarBase(bool init);
	~CAvatarBase();
	unsigned int GetNumParts() const;
	void BFTAlloc(const char* rootName, const char*** szNames, unsigned int* num) const;
	void BFTFree(const char** szNames, unsigned int num) const;

	//calling this function with cautious, it reads A buffer for even frame and B buffer for odd frame
	void BFTGetJoints(const char** names, TVector3D* angles, unsigned int num, bool evenFm) const;
	static void BFTSetJoints(cvTObjState* s, const TVector3D* angles, unsigned int num);
	static void BFTGetJoints(const cvTObjState* s, TVector3D* angles, unsigned int num);
private:
	typedef struct JointTemplate_tag
	{
		const char*		name;
		int				type;			//joint vrlink articulated type
		TVector3D		angle; 			//taitbryan euler
		int				child_first;
		int				sibling_next;
	} JointTemplate;
	void InitJoints();
	void UnInitJoints();
	TAvatarJoint* InitJoint();
	void UnInitJoint(TAvatarJoint*);

	typedef struct NameBlock_tag
	{
		const char** entries; 		//block starts from address entries[0]
		unsigned int cap;
		unsigned int num;
	} NameBlock;
#define NAME_BLOCK_M 1024
	inline void Init(NameBlock* blk) const
	{
		blk->cap = 32;
		blk->entries = (const char**)malloc(blk->cap * sizeof(const char*));
		blk->entries[0] = (const char*)malloc(blk->cap * NAME_BLOCK_M * sizeof(char));
		for (int i_entry = 1; i_entry < blk->cap; i_entry ++)
			blk->entries[i_entry] = blk->entries[i_entry-1] + NAME_BLOCK_M;
		blk->num = 0;
	}
	inline void Grow(NameBlock* blk) const
	{
		unsigned int cap_m = blk->cap;
		blk->cap = (blk->cap << 1);
		blk->entries = (const char**)realloc(blk->entries, blk->cap * sizeof(const char*));
		blk->entries[0] =(const char*)realloc((void *)blk->entries[0], blk->cap * NAME_BLOCK_M * sizeof(char));
		for (int i_entry = cap_m; i_entry < blk->cap; i_entry ++)
			blk->entries[i_entry] = blk->entries[i_entry-1] + NAME_BLOCK_M;
	}
	inline void UnInit(NameBlock* blk) const
	{
		free((void*)blk->entries[0]);
		free(blk->entries);
		blk->cap = 0;
		blk->num = 0;
		blk->entries = NULL;
	}
#undef NAME_BLOCK_M
protected:
	TAvatarJoint *m_jointsA, *m_jointsB; //extension to default state, it takes heap memory
	static JointTemplate s_jointTemplate[];
};

class CExternalAvatarObj : public CExternalDriverObj
						 , public CAvatarBase
{
public:
	CExternalAvatarObj();
	CExternalAvatarObj(const CExternalAvatarObj& );
	CExternalAvatarObj ( const CCved&, TObj* );
	virtual ~CExternalAvatarObj();
	CExternalAvatarObj& operator=( const CExternalAvatarObj& );
	void BFTGetJoints(const char** names, TVector3D* angles, unsigned int num) const;
};

/////////////////////////////////////////////////////////////////////////////
//
// Inline functions.
//
/////////////////////////////////////////////////////////////////////////////
#include "dynobj.inl"

#endif // __DYN_OBJ_H

