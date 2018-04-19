//////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:    $Id: cved.h,v 1.163 2016/08/16 22:20:37 IOWA\dheitbri Exp $
//
// Author(s):
// Date:       September, 1998
//
// Description:	The definition of the CCved class.
//
//////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#pragma warning(disable:4786)
#endif

#ifndef __CVED_H
#define __CVED_H

#include "cvedpub.h"

struct cvTHeader;
struct cvTObj;
struct cvTObjRef;
struct cvTRepObj;
struct cvTDynObjRef;
struct cvTRoadRef;
struct cvTIntrsctnRef;
struct cvTLatCntrlPnt;
class CODE;
class CODEObject;
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#if defined(_DEBUG)
#ifndef TRACE
#define TRACE CVED::CCved::Logoutf
#endif
#else
#undef TRACE
#define TRACE __noop
#endif
class CVED::CCved;

namespace CVED {
	class IExternalObjectControl;
// the CVED shared memory key
#define cCVED_SHARED_MEM_KEY_ENV  "CvedMem"
	class CTrafLightData;
//////////////////////////////////////////////////////////////////////////////
///
/// Description:
/// <B>Correlated Virtual Environment Database</B>
/// This class represents a virtual environment.  It internally stores all
/// information about the static road network, static objects, and dynamic
/// objects within the virtual environment.  It provides a variety of
/// access methods that return that information.
///
/// The virtual environment modeled by this class is stored in a file
/// which is a compiled version of the LRI format.  Once loaded, it
/// defines all aspects of the static environment and starts with no
/// dynamic objects.  Through the API, the class allows dynamic creation
/// and deletion of objects, along with a rich set of functions that return
/// information about the location of objects on the road network.
///
/// Given the complexity and scope of this class, instancing and copying
/// of the class is severely limited.  In fact, the class prohibits
/// copying of its instances through assignment operators or copy
/// constructors.
///
///
class CCved
{
public:

	CCved();
	virtual ~CCved();

	// Useful public types
	typedef vector<CRoad>		TRoadVec;
	typedef vector<CLane>		TLaneVec;
	typedef vector<int>			TIntVec;
	typedef vector<CIntrsctn>	TIntrsctnVec;

	typedef struct {
		int objId;
		double dist;
	} TObjWithDist;



	typedef struct{
		double laneDeviation;
		double distToLeftEdge;
		double distToRightEdge;
		double distToRightmostEdge;
	} TOwnVehiclePositionInfo;


	typedef struct{
		int vehicleId;
		CPoint3D XYZpoint;
		CRoadPos roadpos;
		double velocity;
		eCVTrafficLightState lightState;
	} TLeadObjInfo;




	typedef struct{
		double followDist;
		double bumperToBumperDist;
		double followTime;
		double ttc;
		TLeadObjInfo leadObjInfo;
	} TOwnVehicleFollowInfo;


	// Cved initalization mode
	enum ECvedMode { eCV_SINGLE_USER, eCV_MULTI_USER };

	// System related
	bool		Configure(ECvedMode, double deltaT, int dynaMult);
	bool		Attach(void);
	bool		Init(const string& cLriName, string& errMsg);
	void		ReInit(void);
	void		Maintainer(void);
	void		ExecuteDynamicModels(IExternalObjectControl* ctrl = NULL);
	void		SetDebug(int level);

	// SOL related
	static const CSol& GetSol();

	//  Terrain related
	typedef enum {eCV_OFF_ROAD = 0, eCV_ON_ROAD, eCV_ON_INTRSCTN} EQueryCode;

	typedef struct {
		int objId;
		double objDist;
		bool isOnRoad;
		double distFromOwner;
		bool sameDirAsLane;
	} TObjListInfo;


	/////////////////////////////////////////////////////////////////////
	///
	/// This is a utility class that stores information useful
	/// to the terrain interrogation function of the CCved class.  The
	/// data stored in this class allows the terrain interrogation
	/// function to start the search from where it found the most
	/// recent point, thus significantly reducing the time it takes
	/// to compute the elevation.
	///
	class CTerQueryHint : public CCvedItem {
		public:
			CTerQueryHint();
			virtual ~CTerQueryHint();
			CTerQueryHint(const CTerQueryHint&);
			CTerQueryHint &operator=(const CTerQueryHint&);

			void CopyFromStruct (const cvTerQueryHint&);
			void CopyToStruct (cvTerQueryHint&) const;

			CTerQueryHint(const CCved&);

			// Hint state typedef
			typedef EQueryCode EHintState;

			friend class CCved;

		protected:
			EHintState				m_hintState;
			TRoadPoolIdx 			m_roadId;
			TLongCntrlPntPoolIdx	m_roadPiece;
			TIntrsctnPoolIdx        m_intersection;
	};	// end of CTerQueryHint class

	CVector3D	QryTan(const CPoint3D& in);
	void		QryTrfLghtOnIntrsctn(const CIntrsctn&, vector<int>&);
	void		QryTrfLghtOnIntrsctn(const string&, vector<int>&);
	EQueryCode	QryTerrain(
			double			x,
			double			y,
			double			z,
			double&			zout,
			CVector3D&		norm,
			CTerQueryHint*	pHint,
			int*			pIfTrrnObjUsed = NULL,
			int*			pMaterial = NULL
			);
	EQueryCode	QryTerrain(
			const CPoint3D&	in,
			double&			zout,
			CVector3D&		norm,
			CTerQueryHint*	pHint,
			int*			pIfTrrnObjUsed = NULL,
			int*			pMaterial = NULL
			);
	EQueryCode	QryTerrain(
			const CRoadPos&	cRoadPos,
			double&			zout,
			CVector3D&		norm,
			CTerQueryHint*	pHint,
			int*			pIfTrrnObjUsed = NULL,
			int*			pMaterial = NULL
			);

	void SetNullTerrainQuery(
		const double&     zout,
		const double&  i,
		const double&  j,
		const double&  k,
		int*          pMaterial = NULL);

	void UnsetNullTerrainQuery( void );

	bool QryTerrainIntersection(
					const TIntrsctn*, // which intersection to search
					const CPoint3D&,  // query point
					double&,          // output z
					CVector3D&,       // normal
					int*,
					int*              //material
					);

	// Road related
	void		GetAllRoads(TRoadVec&) const;
	void		GetAllRoadPieces(vector<CRoadPiece>& out) const;
	CRoad		GetRoad(int) const;
	CRoad		GetRoad(const string&) const;

	// Intersection related
	void		GetAllIntersections(TIntrsctnVec&) const;
	CIntrsctn   GetIntersection(int) const;
	CIntrsctn   GetIntersection(const string&) const;
	int         GetNumIntersections() const;
    // corridor related
    bool        GetCrdrsCntrlPointsNear(const CPoint3D&,int crdrId,TIntVec&) const;

	// Object related

	int			GetNumObjects(CObjTypeMask m=CObjTypeMask::m_all) const;
	int			GetNumStaticObjs(CObjTypeMask m=CObjTypeMask::m_all) const;
	int			GetNumDynamicObjs(CObjTypeMask m=CObjTypeMask::m_all) const;

	void		GetAllObjs(TIntVec&, CObjTypeMask m=CObjTypeMask::m_all) const;
	void		GetAllStaticObjs(
					TIntVec&,
					CObjTypeMask m=CObjTypeMask::m_all
					) const;
	int         GetTrafLightsNear(
					const CPoint3D& loc,
					int* pCvedId,
					int maxSize
					);
	void		GetAllDynamicObjs(
					TIntVec&,
					CObjTypeMask m=CObjTypeMask::m_all
					) const;
	void		GetObjsNear(
					const CPoint3D&	loc,
					double 			radius,
					TIntVec&,
					CObjTypeMask	m=CObjTypeMask::m_all
					) const;
	void		GetChangedStaticObjsNear(
					const CPoint3D&	loc,
					double 			boxHalfSide,
					TU8b			changedId,
					TIntVec&,
					CObjTypeMask	m=CObjTypeMask::m_all
					) const;

	void 		GetAllDynObjsOnRoad(
					int roadId,
					const bitset<cCV_MAX_LANES>& cLanes,
					TIntVec& result,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;
	void		GetAllDynObjsOnLane (
					CLane lane,
					vector<TObjWithDist>& result,
					const CObjTypeMask m=CObjTypeMask::m_all
					) const;
	void 		GetAllDynObjsOnRoadRange(
					int roadId,
					const bitset<cCV_MAX_LANES>& cLanes,
					double startDist,
					double endDist,
					TIntVec& result,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;
	void 		GetAllDynObjsOnRoadRange(
					int roadId,
					const bitset<cCV_MAX_LANES>& cLanes,
					double startDist,
					double endDist,
					TIntVec& result,
					bitset<cCV_MAX_LANES>& usedLanes,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;

	void 		GetAllDynObjsOnIntrsctn (
					int intrsctnId,
					const bitset<cCV_MAX_CRDRS>& cCrdrs,
					TIntVec& result,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;
	void 		GetAllDynObjsOnIntrsctnRange (
					int intrsctnId,
					const bitset<cCV_MAX_CRDRS>& cCrdrs,
					const double startDist[cCV_MAX_CRDRS],
					const double endDist[cCV_MAX_CRDRS],
					TIntVec& result,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;
	void 		GetAllDynObjsOnIntrsctnRange (
					int intrsctnId,
					const bitset<cCV_MAX_CRDRS>& cCrdrs,
					const double startDist[cCV_MAX_CRDRS],
					const double endDist[cCV_MAX_CRDRS],
					TIntVec& result,
					bitset<cCV_MAX_CRDRS>& usedCrdrs,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;
	void 		GetAllDynObjsOnCrdr (
					int intrsctnId,
					int crdrId,
					vector<TObjWithDist>& result,
					const CObjTypeMask m=CObjTypeMask::m_all
					) const;
	void 		GetAllDynObjsOnCrdrRange (
					int intrsctnId,
					int crdrId,
					double startDist,
					double endDist,
					TIntVec& result,
					const CObjTypeMask cM=CObjTypeMask::m_all
					) const;
	void        BuildFwdObjList(
					int cvedId,
					const CRoadPos& roadPos,
					const class CPath& path,
					int maxObjs,
					const CObjTypeMask& objMask,
					vector<TObjListInfo>& objs
					);
	void        BuildBackObjList(
					int ownerObjId,
					const CRoadPos& roadPos,
					int maxObjs,
					const CObjTypeMask& objMask,
					vector<TObjListInfo>& backObjs
					);
	void		BuildBackObjList2(
					int ownerObjId,
					const CRoadPos& roadPos,
					const CRoad& prevRoad,
					int maxObjs,
					const CObjTypeMask& objMask,
					vector<TObjListInfo>& backObjs2
					);
	void		BuildOncomingObjList(
					int ownerObjId,
					const CRoadPos& roadPos,
					const CPath& path,
					int maxObjs,
					const CObjTypeMask& objMask,
					vector<TObjListInfo>& oncomObjs
					);
	void		BuildApprchObjList(
					int ownerObjId,
					const CRoadPos& roadPos,
					const CPath& path,
					int maxObjs,
					const CObjTypeMask& objMask,
					vector<TObjListInfo>& apprchObjs
					);
	int			GetFirstObjOnIntrsctingCrdr(
					int crdrId,
					const CCrdr& interCrdr,
					const CObjTypeMask& objMask
					);
	int			GetObjOnCrdrBehindObj(
					int obj,
					int crdrId,
					const CCrdr& interCrdr,
					const CObjTypeMask& objMask
					);

	void		TestIM(
					int ownerObjId,
					const CRoadPos& roadPos,
					const CPath& path,
					const CObjTypeMask& objMask
					);
	bool		GetOwnVehicleInfo(
					const CObjTypeMask& objMask,
					TOwnVehiclePositionInfo& positionInfo,
					TOwnVehicleFollowInfo& followInfo
					);

	bool		GetObj(const string&, int& objId) const;
	bool        GetObj(const string&, vector<int>& objId) const;
	bool		GetLeadObj(int objId, int& leadObjId, double& followDist, const CPath* path = NULL, bool withRoadDir = true, int prefCrdr = -1) const;
	bool	    IsExplicitObj( int objId ) const;
	bool	    IsDynObj(int objId) const;
	bool        IsStaticObj(int objId) const { return !IsDynObj(objId); }
	bool        IsLriObj(int objId) const { return IsStaticObj(objId) && !IsExplicitObj(objId); }

	// Type-specific accessors and mutators
	eCVTrafficLightState GetTrafficLightState(int objId) const;
	void        SetTrafficLightState(int objId, eCVTrafficLightState st);
	int         GetVehicleAudioState(int objId); // CHANGED
	int         GetVehicleVisualState(int objId);
	void        SetVehicleAudioState(int objId, int state);
	void        SetVehicleVisualState(int objId, int state);


	bool		SetRoadAttr(
					const string& rd,
					int           attrId,
					double         value1 = -1.0,
					double         value2 = -1.0,
					double         from = -1.0,
					double         to = -1.0,
					int           laneMask = -1);
	bool		SetRoadAttr(
					const CRoad& rd,
					int          attrId,
					double        value1 = -1.0,
					double        value2 = -1.0,
					double        from = -1.0,
					double        to = -1.0,
					int          laneMask = -1);
	bool		SetIntrsctnAttr(
					const CIntrsctn& intrsctn,
					int              attrId = -1,
					double            value1 = -1,
					double            value2 = -1,
					double            from = -1,
					double            to = -1);
	bool		SetIntrsctnAttr(
					const string& intrsctnName,
					int              attrId = -1,
					double            value1 = -1,
					double            value2 = -1,
					double            from = -1,
					double            to = -1);
	bool		SetCrdrAttr(
					const CCrdr&     crdr,
					int              attrId = -1,
					double            value1 = -1,
					double            value2 = -1,
					double            from = -1,
					double            to = -1);
	bool		SetCrdrAttr(
					const CIntrsctn& intrsctn,
					int              crdrId,
					int              attrId = -1,
					double            value1 = -1,
					double            value2 = -1,
					double            from = -1,
					double            to = -1);
	bool		SetCrdrAttr(
					const string&    intrsctnName,
					int              crdrId,
					int              attrId = -1,
					double            value1 = -1,
					double            value2 = -1,
					double            from = -1,
					double            to = -1);


	int         CollisionDetection(
							int,
							vector<int>&,
							CObjTypeMask m=CObjTypeMask::m_all
							) const;

    void GetStaticObjectsInBoundingBox(const CBoundingBox &bbox,vector<int>& objIdVec) const;
    void GetTerrianObjectsInBoundingBox(const CBoundingBox &bbox,vector<int>& objIdVec) const;

	bool        IfOverlap(int, int) const;

	void        DumpEnvArea();
	void        GetAllEnvArea(vector<CEnvArea>&) const;
	CEnvArea    GetGlobalEnvArea() const;
	bool        GetEnvArea(int id, CEnvArea&) const;
	void        GetEnvArea(const CPoint2D&, vector<CEnvArea>&);
	void        GetEnvArea(const CPoint3D&, vector<CEnvArea>&);
	void        GetEnvArea(double, double, vector<CEnvArea>&);
	bool        CreateEnvArea(
						vector<cvTEnviroInfo>& info,
						vector<CPoint2D>& polyPts,
						CPoint2D& originPt
						);
	void        SetEnviron(CEnvArea&, vector<cvTEnviroInfo>& info);
	void        SetGlobalEnviron(vector<cvTEnviroInfo>& info);
	CExternalDriverObj*	CreatePeerExternalObj(
					const string&     cName,
					const cvTObjAttr& cAttr,
					const CPoint3D*   cpInitPos,
					const CVector3D*  cpInitTan,
					const CVector3D*  cpInitLat);
	CDynObj*	CreateDynObj(
					const string&		cName,
					cvEObjType			type,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
	CDynObj*	CreateDynObj(
					const string&		cName,
					cvEObjType			type,
					int					hcsmType,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos=0,
					const CVector3D*	cpInitTan=0,
					const CVector3D*	cpInitLat=0);
	int   		CreateStaticObj(
					const string&		cName,
					cvEObjType 			type,
					const cvTObjAttr&	cAttr,
					const cvTObjState&	cState);
	void		UpdateObjRefList(TObjectPoolIdx objId);

	void		DeleteDynObj( CDynObj* );

	// Object access functions
	bool		IsObjValid( int objId ) const;
	//fix me: shall we do it in derived class?
	cvEObjType	GetObjType( int objId, bool singleExt ) const;
	cvEObjType	GetObjType( int objId ) const;
	int			GetObjHcsmTypeId( int objId ) const;
	TS32b		GetObjSolId( int objId ) const;
	TS32b		GetObjHcsmId( int objId ) const;
	int         GetObjCigiId( int objId ) const;
	const char*	GetObjName( int objId ) const;
	CPoint3D	GetObjPos( int objId ) const;
    bool        GetVehicleFourCorners(int objId, vector<CPoint3D>&) const;
	CVector3D	GetObjTan( int objId ) const;
	CVector3D	GetObjLat( int objId ) const;
    CVector3D	GetObjTargetPoint( int objId ) const;
	double      GetObjVel( int objId ) const;
	void		GetObjState( int objId, cvTObjState& state ) const;
	void		GetObjState(
					int,
					CPoint3D&,
					CVector3D&,
					CVector3D&
					) const;
	CBoundingBox GetObjBoundBox(int) const;
	const CDynObj* BindObjIdToClass(int);
	CDynObj* BindObjIdToClass2(int);

	double      GetObjLength( int objId ) const;
	double      GetObjWidth( int objId ) const;
	bool		SetObjOption( int objId, int option );
	bool		GetObjOption( int objId, int& option ) const;
	bool		SetObjVisualState( int objId, short  state, bool instant = false );
	bool		GetObjVisualState( int objId, short& state ) const;
	bool		SetObjAudioState( int objId, short   state, bool instant = false );
	bool		GetObjAudioState( int objId, short&  state ) const;

	bool		SetObjCompositeOptions( int objId, const int* options);
	bool		GetObjCompositeOptions( int objId, int* options) const;
	int         GetObjColorIndex( int objId ) const;
	CPoint3D	GetObjPosInstant( int objId ) const;
	CVector3D	GetObjTanInstant( int objId ) const;
	CVector3D	GetObjLatInstant( int objId ) const;
	double      GetObjVelInstant( int objId ) const;
	void		GetObjStateInstant( int objId, cvTObjState& state ) const;
	void		GetObjStateInstant(
					int,
					CPoint3D&,
					CVector3D&,
					CVector3D&
					) const;
	CBoundingBox GetObjBoundBoxInstant(int) const;

	bool        GetOwnVehiclePos( CPoint3D& pos ) const;
	bool        GetOwnVehicleTan( CVector3D& tang ) const;
	bool        GetOwnVehicleLat( CVector3D& lat ) const;
	bool        GetOwnVehicleAngularVel( CVector3D& angularVel ) const;
	bool        GetOwnVehicleVel( double& vel ) const;
	void        SetFakeExternalDriver( bool );
	bool        HaveFakeExternalDriver() const;
	void        SetExternalDriverHcsmId( int hcsmId );

	// General
	static void	UpdateBBox(
					const cvTObjAttr*,
					const CVector3D&	cTang,
					const CVector3D&	cLat,
					const CPoint3D&		cPos,
					CPoint3D&			lowerLeft,
					CPoint3D&			upperRight
					);

	// debugging function
	void		Verify( void );
	bool        GetSol2ParseLog(string&) const;

	// friend
	friend class CCvedItem;

//for testing, will be removed
	CQuadTree	GetRdPcQuadTree() {return m_rdPcQTree;}
	CQuadTree	GetIntrsctnQuadTree() {return m_intrsctnQTree;}

	// Used by CRoadPos to search for (x,y,z) in road network
	int 		SearchRdPcQuadTree(double, double, double, double,
						   vector<int>&) const;
	int			SearchIntrsctnQuadTree(double, double, double, double,
							   vector<int>&) const;

	// query terrain performance evaluation
	void QryTerrainPerfCheck(string &, bool reset=false);

	// functions to transition the mode of a traj follower
	// if some of the arguments are not specified, use the values stored in the traj follower state
	bool CoupledObjectMotion( int child, int parent=-1, double offset[6]=NULL );
	bool ObjRelTrajMotion( int child, int parent=-1, double offset[6]=NULL );
	bool FreeObjectMotion( int child, double initPosRot[6]=NULL, double initVel[6]=NULL );
	void CreateODEObject(odePublic::EODEObjType type,
						CODEObject* pOdeObj,
						const double initPosRot[6],
						const double initVel[6],
						double mass,
						double cgoffset[3],
						double dimension[3],
						double visOrigOffset[3],
						odePublic::SBodyProps bodyProps
						);
	void DeleteODEObj(CODEObject* obj);
	void SetupODEObject( const cvTObjAttr*,
						 int,
						 CCved&,
						 const cvTObjState::TrajFollowerState*,
						 cvTObjState::TrajFollowerState*,
						 bool);
	void SetupVehODEObject( const cvTObjAttr*,
						 int,
						 CCved&,
						 TVehicleState*,
						 bool);

protected:
	// these components are declared private to disallow their use
	CCved(const CCved&);
	CCved &operator=(const CCved&);

	void MemBlockInit(void);
	void ClassInit(void);
	void GetSegment(cvTCntrlPnt*, cvTCntrlPnt*, CPoint2D*);
	void GetSegment(cvTCntrlPnt*, CPoint2D*);
	bool GetObjLinear(const string&, int& objId) const;

	enum EState {eUNCONFIGURED, eCONFIGURED, eACTIVE};
	typedef map<string, int>  TStr2IntMap;
	typedef CTerrainGrid<Post> *CTerrainGridPtr;
    typedef std::unique_ptr<CQuadTree> TQtreeRef;
    typedef std::pair<int, TQtreeRef> TQtreeIdRef;
    typedef map<int, TQtreeRef> TQtreeMap;

	EState      m_state;		// class state
	ECvedMode	m_mode;			// current mode (single/multi user)
	cvTHeader*	m_pHdr;			// ptr to memory block
	double		m_delta;		// timestep between execution of maintainer
	int			m_dynaMult;		// Dynamics interleave frequency
	CSharedMem  m_shm;			// class keeping track of shared memory
	bool        m_haveFakeExternalDriver;  // do we have a fake driver??
	int         m_debug;		// debug level, 0-none, 1-min, 2-more, 3-max
	CQuadTree	m_rdPcQTree;	// quadtree for road pieces
	CQuadTree	m_intrsctnQTree;	// quadtree for intersection
	CQuadTree	m_staticObjQTree;	// quadtree for staic objects
	CQuadTree	m_trrnObjQTree;		// quadtree for the objects of type terrain
    TQtreeMap   m_intersectionMap; //<

	static CSol m_sSol;         // Sol library that is the same for all
								//	CCved instances
	TStr2IntMap	m_roadNameMap;		// maps road names to road identifiers
	TStr2IntMap	m_intrsctnNameMap;	// maps intersection names to identifiers
	CDynObj*	m_dynObjCache[cNUM_DYN_OBJS];

	vector<CPolygon2D>  m_intrsctnBndrs;	// intersection boundary polys
	vector<CTerrainGridPtr> m_intrsctnGrids;	// intersection elev maps

	// functions that help access internal pools
public:
	cvTObj*			BindObj(TObjectPoolIdx) const;
	cvTObjRef*		BindObjRef(TObjRefPoolIdx) const;
	cvTRepObj*		BindRepObj(TRepObjPoolIdx) const;
	cvTObjAttr*		BindObjAttr(TObjAttrPoolIdx) const;
	cvTIntrsctn*	BindIntrsctn(TIntrsctnPoolIdx) const; // {secret}
	cvTElevMap*		BindElevMap(TElevMapPoolIdx) const; // {secret}
	cvTElevPost*	BindElevPost(TElevPostPoolIdx) const; // {secret}
	cvTBorderSeg*	BindBorderSeg(TBorderSegPoolIdx) const; // {secret}
	cvTRoadPiece*   BindRoadPiece(TRoadPiecePoolIdx) const; // {secret}
	cvTRoad*		BindRoad(TRoadPoolIdx) const;
	cvTLane*        BindLane(TLanePoolIdx) const;
	cvTCntrlPnt*	BindCntrlPnt(TLongCntrlPntPoolIdx) const;
	cvTLatCntrlPnt*	BindLatCntrlPnt(TLatCntrlPntPoolIdx) const;
	cvTDynObjRef*	BindDynObjRef(TObjectPoolIdx) const;
	cvTRoadRef*		BindRoadRef(TRoadPoolIdx) const;
	cvTIntrsctnRef*	BindIntrsctnRef(TIntrsctnPoolIdx) const;
	static void __cdecl Logoutf(const TCHAR* format, ...);
protected:

	int	GetObjWithClosestDistOnCrdr(
							 double intersectingCrdrLength,
							 double intersectingPointDist,
							 vector<TObjWithDist>& objs
							 );

	int	GetObjWithClosestDistOnLane(
							 CLane& srcLane,
							 vector<TObjWithDist>& objs );
	int GetClosestObjBehindOnCrdr(
							 int obj,
							 double intersectingCrdrLength,
							 double intersectingPointDist,
							 vector<TObjWithDist>& objs
							 );
	int	GetClosestObjBehindOnLane(
							 int obj,
							 CLane& srcLane,
							 vector<TObjWithDist>& objs
							 );

	// functions that are used by terrain queries
#include "terrain.h"

	// functions and variabled that are used by the dynamic
	// 	object reference list
#include "dynobjreflist.h"

	void LockObjectPool(void);		// mutex access through a blocking lock
	void UnlockObjectPool(void);	// unlock object pool

	CDynObj *CreateTypedObject(cvEObjType, int);

	// dispatcher for dynamic servers
    void DynamicModel(
		    int,
			cvEObjType,
			const cvTObjAttr*,
			const cvTObjState*,
			const cvTObjContInp*,
			cvTObjState*);

	// help with object types
	const type_info &GetRunTimeDynObjType(cvEObjType type);

	// these variables help with performance evaluation of the
	// terrain query function
	long        m_terQryCalls;		// number of calls to terrain query
	long        m_terQryRoadHits;	// number of calls that used the road hint
	long        m_terQryInterHits;	// number of calls that used the intrs hint

	// this variable, when set, short-circuits the terrain query so
	// that it returns a known value, no matter what the state of the class is
	bool        m_NullTerrQuery;
	double       m_NullQueryZ;
	CVector3D   m_NullQueryNorm;
	int         m_NullQueryMaterial;

	multimap<string, int> m_objNameToId;

	// environment areas
	vector<CEnvArea> m_envAreas;

	// ode world
	std::unique_ptr<CODE>		m_pOde;
	//traffic light data
	bool m_FirstTimeLightsNear; //<First
	vector<CTrafLightData> m_tlData;
#ifdef _WIN32
	HANDLE	m_MUTEX_LightsNear; //only one thread at time can access GetTrafLightsNear
#endif
};


//////////////////////////////////////////////////////////////////////////////
//	CTerQueryHint Inline functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: CTerQueryHint
// 	Default constructor creates an unbound and invalid CTerQueryHint instance.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CCved::CTerQueryHint::CTerQueryHint( void )
{
	m_hintState = eCV_OFF_ROAD;
	m_roadId = 0;
	m_roadPiece = 0;
	m_intersection = 0;
} // end of CTerQueryHint

//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CTerQueryHint
// 	Default destructor does nothing.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CCved::CTerQueryHint::~CTerQueryHint(void)
{}

//////////////////////////////////////////////////////////////////////////////
//
// Description: CTerQueryHint
// 	Creates a bound and valid CTerQueryHint instance.
//
// Remarks:
//
// Arguments:
// 	cCved - reference to a CCved instance
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CCved::CTerQueryHint::CTerQueryHint(const CCved &cCved)
	: CCvedItem(&cCved)
{
	m_hintState = eCV_OFF_ROAD;
	m_roadId = 0;
	m_roadPiece = 0;
	m_intersection = 0;
} // end of CTerQueryHint

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
//	Performs a deep copy of the parameter to the current CTerQueryHint
//	instance.
//
// Remarks:
//
// Arguments:
// 	cRhs - CTerQueryHint that is on the right-hand side of the operator.
//
// Returns: a reference to the current CTerQueryHint instance, so that the
// 	assignments can be nested.
//
//////////////////////////////////////////////////////////////////////////////
inline CCved::CTerQueryHint&
CCved::CTerQueryHint::operator=(const CTerQueryHint& cRhs)
{
	if ( this != &cRhs ) {
		m_hintState    = cRhs.m_hintState;
		m_roadId       = cRhs.m_roadId;
		m_roadPiece    = cRhs.m_roadPiece;
		m_intersection = cRhs.m_intersection;
	}

	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: CTerQueryHint
// 	Copy constructor constructs a CTerQueryHint instance from the parameter.
//
// Remarks: Calls the assignment operator
//
// Arguments:
// 	cCopy - CTerQueryHint that is the instance to copy
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CCved::CTerQueryHint::CTerQueryHint(const CTerQueryHint& cCopy)
{
	*this = cCopy;
} // end of CTerQueryHint

//////////////////////////////////////////////////////////////////////////////
//
// Description: CopyFromStruct
// 	Copies the contents of the parameter struct to the current instance.
//
// Remarks:
//
// Arguments:
// 	structHint - struct to copy from
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void
CCved::CTerQueryHint::CopyFromStruct(const cvTerQueryHint& structHint)
{
	m_hintState = (EHintState)structHint.hintState;
	m_roadId = structHint.roadId;
	m_roadPiece = structHint.roadPiece;
	m_intersection = structHint.intersection;
} // end of CopyFromStruct

//////////////////////////////////////////////////////////////////////////////
//
// Description: CopyToStruct
// 	Copies the contents of the current instance to the struct parameter.
//
// Remarks:
//
// Arguments:
// 	structHint - struct to copy to
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void
CCved::CTerQueryHint::CopyToStruct(cvTerQueryHint& structHint) const
{
	structHint.hintState = m_hintState;
	structHint.roadId = m_roadId;
	structHint.roadPiece = m_roadPiece;
	structHint.intersection = m_intersection;
} // end of CopyToStruct


//////////////////////////////////////////////////////////////////////////////
//	CCved Inline functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetDebug
// 	Sets the debugging level.
//
// Remarks:
//
// Arguments:
//	level - level to set debugging to
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void
CCved::SetDebug(int level)
{
	if (level < 0) level = 3;
	if (level > 3) level = 3;
	m_debug = level;
} // end of SetDebug

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetSol
// 	Returns the static ScenObjLib associated with CCved
//
// Remarks:
//
// Arguments:
//
// Returns: a const reference to the ScenObjLib
//
//////////////////////////////////////////////////////////////////////////////
inline const CSol&
CCved::GetSol()
{
	if (!m_sSol.IsInitialized())
		m_sSol.Init();
	return m_sSol;
} // end of GetSol


} // namespace CVED

#endif	// __CVED_H

