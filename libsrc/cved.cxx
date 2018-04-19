///////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: cved.cxx,v 1.284 2016/10/28 15:58:21 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		September, 1998
//
// The implementation of the CCvedclass
//
//////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"
#include "objreflistUtl.h"
#include "EnvVar.h"
#include <algorithm>
#include "hcsmobject.h"
#include "ExternalControlInterface.h"
#include "polygon2d.h"

#include <filename.h>
#include "hcsmobject.h"
#ifdef _PowerMAXOS
template int* std::copy<int*,int*>(int*,int*,int*);
template int* std::copy_backward<int*,int*>(int*,int*,int*);
#endif

#include "odeDynamics.h"
#include <winhrt.h>
#include <TCHAR.H>

//
// Debugging macros.
//
#undef DEBUG_GET_ALL_DYNOBJS
#undef DEBUG_GET_ALL_DYNOBJS_ON_ROAD
#undef DEBUG_GET_ALL_DYNOBJS_ON_CRDR
#undef DEBUG_GET_ALL_DYNOBJS_ON_CRDR_RANGE
#undef DEBUG_GET_ALL_DYNOBJS_ON_LANE
#undef DEBUG_BUILD_FWD_OBJ_LIST
#undef DEBUG_BUILD_BACK_OBJ_LIST
#undef DEBUG_BUILD_ONCOM_OBJ_LIST
#undef DEBUG_BUILD_APPRCH_OBJ_LIST
#undef DEBUG_BUILD_BACK_OBJ_LIST2
#undef DEBUG_FIRST_OBJ
#undef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
#undef DEBUG_GET_CLOSEST_OBJ_ON_LANE
#undef DEBUG_GET_OBJ_BEHIND_OBJ
#undef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
#undef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_LANE
#undef DEBUG_TEST_IM
#undef DEBUG_GET_OWN_VEHICLE_INFO
#undef DEBUG_MAINTAINER
#undef DEBUG_GET_LEAD_OBJ


// Constants
const double cNEAR_ZERO = 0.001;
const double cFEET_TO_METER = 0.3048;


// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{
CSol CCved::m_sSol;

//////////////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
///
/// Description: Default constructor. Initializes local variables to
///   null values. Parses sol2.txt.
///
/// Remarks:
///
/// Arguments:
///
/// Returns: void
///
///////////////////////////////////////////////////////////////////////////////
CCved::CCved()
	: m_pHdr( 0 ),
	  m_debug( 0 ),
	  m_state( eUNCONFIGURED ),
	  m_haveFakeExternalDriver( true ),
	  m_FirstTimeLightsNear(true)
{

	m_terQryCalls = m_terQryRoadHits = m_terQryInterHits = 0;
	int i;
	m_pOde = NULL;
#ifdef _WIN32
	m_MUTEX_LightsNear = CreateMutex(NULL,0,"m_MUTEX_LightsNear");
	ReleaseMutex(m_MUTEX_LightsNear);
#endif
	for( i = 0; i < cNUM_DYN_OBJS; i++ )
	{
		m_dynObjCache[i] = 0;
	}
	m_pSavedObjLoc = 0;
	m_NullTerrQuery = false;

	if (m_sSol.IsInitialized()) return;

	bool solSuccess = m_sSol.Init();
	if ( solSuccess == false ) {
		fprintf(stderr, "SolInit failed: %s\n", m_sSol.GetLastError().c_str());
		assert(0);
	}
} // end of CCved()

//////////////////////////////////////////////////////////////////////////////
//
// Description: Destructor. Deletes all the dynamically allocated memory
//   used.
//
// Remarks: By the current CCved instance.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CCved::~CCved()
{
	int i;
	for( i = 0; i < cNUM_DYN_OBJS; i++ )
	{
		delete m_dynObjCache[i];
	}

	if( m_pSavedObjLoc != 0 )
	{
		for( i = 0; i < cNUM_DYN_OBJS; i++ )
		{
		   delete[] m_pSavedObjLoc[i].pDynObjRefs;
		}
		delete[] m_pSavedObjLoc;
	}

	vector<CTerrainGridPtr>::const_iterator itr;
	for(
		itr = m_intrsctnGrids.begin();
		itr != m_intrsctnGrids.end();
		itr++
		)
	{
		delete( *itr );
	}

	if( m_mode == eCV_SINGLE_USER )
	{
		if( m_pHdr )  delete[] m_pHdr;
	}
	else
	{
		if( m_pHdr )
		{
			m_pHdr->numClients--;
			if( m_pHdr->numClients == 0 )
			{
				m_shm.Delete();
			}
			else
			{
				m_shm.Disconnect();
			}
		}
	}

	if ( m_pOde )
		m_pOde.reset();

} // end of ~CCved

//////////////////////////////////////////////////////////////////////////////
// System related
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function configures the ininitialized CCved
//   instance by providing configuration information.
//
// Remarks: The function can only be used on a CCved instance that has not
//   been initialized (by calling the Init function).  For an example, see
//   the ReInit function.
//
// Arguments:
//   mode 	- single user (eCV_SINGLE_USER) or multi user (eCV_MULTI_USER)
//   delta 	- how much time is simulated between invokations of the maintainer
//   dynaMult - how many times the dynamic models will execute
//            between invokations of the maintainer
//
// Returns: The function returns true to indicate that the parameters are
//   consistent and have been set, or false otherwise.  Invalid parameters
//   include a negative time step or dynamics multiplier.  If the function
//   returns false, then it is considered unconfigured and cannot be
//   initilialized.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::Configure( CCved::ECvedMode mode, double delta, int dynaMult )
{
	// standard error checking
	if( delta <= 0.0 )  return false;
	if( dynaMult < 1 )  return false;
	if( mode == eCV_MULTI_USER )
	{
		fprintf(
			stderr,
			"CVED: Configure: unable to support eCV_MULTI_USER mode\n"
			);
		fflush( stderr );
		return false;
	}

	// copy parameters
	m_mode		= mode;
	m_delta		= delta;
	m_dynaMult	= dynaMult;

	// class state transition
	m_state		= eCONFIGURED;


	// success
	return true;
} // end of Configure

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function attaches the current cved instance to another
// 	 existing instance that is currently operating in multi user mode.  Upon
// 	 attachment, the current instance can be used to interrogate or modify
// 	 the same virtual environment as the other instance.
//
// Remarks: The attach function can only be called immediately after the
//   default constructor.  It also implies that the class is used in multi
//   user mode.  For an example, see the ReInit function.
//
// Arguments:
//
// Returns: The function returns true to indicate that the attachment was
//   successful, or false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::Attach( void )
{
	// only allowed to attach when unconfigured
	if( m_state != eUNCONFIGURED )  return false;


	// force the mode to multi user
	m_mode = eCV_MULTI_USER;
    string pKey;
    NADS::GetEnvVar(pKey,cCVED_SHARED_MEM_KEY_ENV);
	if( pKey == "" )
	{
		char tempBuf[64];
		sprintf_s( tempBuf, "%d", cCVED_SHARED_MEM_KEY );
		pKey = tempBuf;
	}

	// if we are attaching, we should not create the memory
	// segment, simply connect to it
	m_pHdr = static_cast<cvTHeader *> (m_shm.Connect( pKey.c_str() ));
	if( m_pHdr == NULL )  return false;

	// we can only use the block if the initialized flag is set,
	// if it isn't set, wait for a bit.  To avoid infinite
	// wait, bail out after a while.

	int count = 0;
	while( m_pHdr->initialized == 0 )
	{
#ifdef _WIN32
		Sleep(500);
#endif
		if( count++ > 5 )  return false;		// bailout
	}

	ClassInit();				// do any class specific initializations
	m_pHdr->numClients++;		// make our presense known
	m_state = eACTIVE;			// ready to go
	m_delta = m_pHdr->deltaT;
	m_dynaMult = m_pHdr->dynaMult;

	return true;
} // end of Attach

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function is responsible for initializing the contents
//   of the memory block so they reflect the virtual environment at its
//   initial state.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::MemBlockInit( void )
{
	m_pHdr->frame        = 0;
	m_pHdr->elapsedTime  = 0;
	m_pHdr->deltaT       = m_delta;
	m_pHdr->dynaMult	 = m_dynaMult;

	// mark all dynamic objects as 'dead'
	TU32b   objId;
	TObj* pO;
	for( objId=0, pO=BindObj( objId ); objId<cNUM_DYN_OBJS; objId++, pO++ )
	{
		pO->phase = eDEAD;
	}

	// initialize the few number of slots allocated for
	// creation of static objects.  These are the slots following
	// the slots allocated for the dynamic objects.
	m_pHdr->objectCount = m_pHdr->objectCountInitial;
	for(
		objId = m_pHdr->objectCount, pO=BindObj( objId );
		objId < m_pHdr->objectStrgCount;
		objId++, pO++
		)
	{
		pO->phase = eDEAD;
		pO->myId  = objId;
	}
} // end of MemBlockInit

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function is responsible for initializing various
//   fields in the class that can only be in itialized after the memory
//   block is loaded.
//
// Remarks: This function should be kept short in execution time as any
//   really complicated initializations should take place during the
//   compilation and be saved in the binary lri file.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::ClassInit(void)
{
	// insert all road names in the map
	TU32b  rid;
	for (rid=1; rid<m_pHdr->roadCount; rid++) {
		TRoad *pRdPool = (TRoad *) (((char *)m_pHdr) + m_pHdr->roadOfs);
		TRoad *pR      = &pRdPool[rid];
		char *pChPool  = ((char *)m_pHdr) + m_pHdr->charOfs;

		m_roadNameMap[pChPool + pR->nameIdx] = rid;
	}

	// insert all intersection names in the map
	TU32b  iid;
	for (iid=1; iid<m_pHdr->intrsctnCount; iid++) {
		TIntrsctn *pI  = BindIntrsctn(iid);
		char *pChPool  = ((char *)m_pHdr) + m_pHdr->charOfs;

		m_intrsctnNameMap[pChPool + pI->nameIdx] = iid;
	}

	// delete any remaining pointers in the dynamic object pointer cache
	int objId;
	for ( objId=0; objId < cNUM_DYN_OBJS; objId++ ) {
		delete m_dynObjCache[objId];
		m_dynObjCache[objId] = 0;
	}

	// create all the intersection boundary polygons.  We have to
	// a "dummy" poly for slot 0, even though slot 0 is invalid.  If
	// we don't do that, the indeces won't match between the intersection
	// slots in the pool and the border slots in the m_intrsctnBndrs.
	//	m_intrsctnBndrs.reserve(1);
	CPolygon2D dummyPoly;
	m_intrsctnBndrs.push_back(dummyPoly);
	for (iid=1; iid<m_pHdr->intrsctnCount; iid++) {
		TIntrsctn* pInter = BindIntrsctn(iid);
		CPolygon2D poly;

		TBorderSegPoolIdx vertex;
		for (vertex=0; vertex < pInter->numBorderSeg; vertex++) {
			TBorderSeg *pVert = BindBorderSeg(pInter->borderSegIdx+vertex);

			poly.AddPoint(pVert->x, pVert->y);
		}
		m_intrsctnBndrs.push_back(poly);
	}

	// create the terrain grid objects for all intersections
	m_intrsctnGrids.push_back(0);
	for (iid=1; iid<m_pHdr->intrsctnCount; iid++) {
		TIntrsctn* pInter = BindIntrsctn(iid);

		// if the intersection has no elevation map (meaning it
		// is flat, then we simply put a blank pointer in the
		// vector so the elevation map indeces are the same
		// as the intersection ids.
		if ( pInter->elevMap == 0 )	{
			m_intrsctnGrids.push_back(0);
			continue;
		}

		// if the intersection has an elevation map, then
		// build the CTerrainGrid class and store it in memory
		// so when we need the elevation within the intersection's
		// border we can consult the map.
		// Note:
		// The ReCalc() function of the CTerrainGrid class is not
		// called which means that the z vertical proximity
		// will not be taken into account when querying the map
		cvTElevMap*	pMap = BindElevMap(pInter->elevMap);
		double bx2        = pInter->elevMapX + (pMap->numCols) * pMap->res;
		double by2        = pInter->elevMapY + (pMap->numRows) * pMap->res;

		CBoundingBox bbox(pInter->elevMapX, pInter->elevMapY, bx2, by2);
		CTerrainGrid<Post>  *pGrid = new CTerrainGrid<Post>(
				bbox, pMap->numRows, pMap->numCols, true);

		Post post;
		int  r, c;
		for (r=0; r<pMap->numRows; r++) {
			for (c=0; c<pMap->numCols; c++) {

				cvTElevPost* pPost;
				int          index = r * pMap->numCols + c;

				pPost      = BindElevPost(pMap->postIdx + index);
				post.z     = pPost->z;
				post.type  = pPost->flags;

				pGrid->SetPost(r, c, post);
			}
		}

		m_intrsctnGrids.push_back(pGrid);
	}

	// Initialize dynamic object efficiency structures
	m_pRoadRefPool = BindRoadRef(0);
	m_pIntrsctnRefPool = BindIntrsctnRef(0);
	m_pDynObjRefPool = BindDynObjRef(0);

	// Load quadtrees
	char *pQTreeBlock =	static_cast<char *>(static_cast<void *>(m_pHdr)) +
						m_pHdr->rdPcQTreeOfs;
	m_rdPcQTree.Load(pQTreeBlock);

    pQTreeBlock =	static_cast<char *>(static_cast<void *>(m_pHdr)) +
					m_pHdr->intrsctnQTreeOfs;
	m_intrsctnQTree.Load(pQTreeBlock);

	pQTreeBlock =	static_cast<char *>(static_cast<void *>(m_pHdr)) +
					m_pHdr->staticObjQTreeOfs;
	m_staticObjQTree.Load(pQTreeBlock);

	pQTreeBlock =   static_cast<char *>(static_cast<void *>(m_pHdr)) +
					m_pHdr->trrnObjQTreeOfs;
	m_trrnObjQTree.Load(pQTreeBlock);


	TU32b  oid;
	for (oid=cNUM_DYN_OBJS; oid<m_pHdr->objectCount; oid++) {
		TObj* pObj = BindObj( oid );
		// Add the object to the list of
		pair<string, int> pairToInsert;
		pairToInsert.first = pObj->name;
		pairToInsert.second = oid;
		m_objNameToId.insert( pairToInsert );
	}
    CCved::TIntrsctnVec inters;
    GetAllIntersections(inters);
    for (auto itr = inters.begin(); itr != inters.end(); ++itr){
        CVED::TCrdrVec crds;
        itr->GetAllCrdrs(crds);
        for (auto citr = crds.begin(); citr != crds.end(); ++citr){
            CQuadTree* pQtree = new CQuadTree();
            auto &crdr = (*citr);
            char*  pOfs = reinterpret_cast<char*> (m_pHdr) + m_pHdr->crdrOfs;
            char* pCtrlPntOfs = reinterpret_cast<char*> (m_pHdr) + m_pHdr->crdrCntrlPntOfs;
            int crdrId =  crdr.GetId();
            cvTCrdr* pcdr = (reinterpret_cast<cvTCrdr*>(pOfs)) + crdrId;
            int cnt = pcdr->numCntrlPnt;
            TCrdrCntrlPntPoolIdx idx = pcdr->cntrlPntIdx;
            for (int i = 0; i<cnt-1; i++){
                int currId = idx + i;
                int nextId = idx + i + 1;
                cvTCrdrCntrlPnt* pCurrCp  = reinterpret_cast<TCrdrCntrlPnt*>(pCtrlPntOfs) + currId;
                cvTCrdrCntrlPnt* pNextCp  = reinterpret_cast<TCrdrCntrlPnt*>(pCtrlPntOfs) + nextId;
                CPolygon2D roadSegment;
                CPoint2D tempPnt;

                ///first right bound
                double halfWidth = pCurrCp->width *0.5;
                tempPnt.m_x = pCurrCp->location.x + pCurrCp->rightVecLinear.i * halfWidth;
                tempPnt.m_y = pCurrCp->location.y + pCurrCp->rightVecLinear.j * halfWidth;
                roadSegment.AddPoint(tempPnt);
                ///first left bound
                tempPnt.m_x = pCurrCp->location.x - pCurrCp->rightVecLinear.i * halfWidth;
                tempPnt.m_y = pCurrCp->location.y - pCurrCp->rightVecLinear.j * halfWidth;
                roadSegment.AddPoint(tempPnt);


                ///second right bound
                halfWidth = pNextCp->width *0.5;
                tempPnt.m_x = pNextCp->location.x + pNextCp->rightVecLinear.i * halfWidth;
                tempPnt.m_y = pNextCp->location.y + pNextCp->rightVecLinear.j * halfWidth;
                roadSegment.AddPoint(tempPnt);
                ///second left bound
                tempPnt.m_x = pNextCp->location.x - pNextCp->rightVecLinear.i * halfWidth;
                tempPnt.m_y = pNextCp->location.y - pNextCp->rightVecLinear.j * halfWidth;
                roadSegment.AddPoint(tempPnt);

                auto bounds = roadSegment.GetBounds();
                pQtree->Add(currId,
                    bounds.first.m_x, bounds.first.m_y,
                    bounds.second.m_x,bounds.second.m_y);
            }
            pQtree->Optimize();
            m_intersectionMap.insert(std::make_pair(crdrId,TQtreeRef(pQtree)));
        }
    }
} // end of ClassInit

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function initilializes the internal data structures
//   to reflect a virtual environment contained in an lri file.
//
// Remarks: After successful completion of this function, the environment
//   contains all the static objects that were instanced in the lri file (but
//   no dynamic objects) and the full set of interrogations functions can be
//   used to query the virtual environment.
//
//   The following are examples of using this function.
//
//		// Master process
//		CCved   ve;
//		string	err;
//
//		ve.Configure(eTMultipleUser, 0.1, 2);
//		if ( !ve.Init("heaven.lri", err) ) {
//			gout << "Cannot initialize file: " << err << endl;
//			exit(-1);
//		}
//
//		// Here use the ve
//
//		ve.Detach();
//		exit(0);
//		//////////////// End of example segment 1
//
//
//		// Client process (any number of processes can do this)
//		CCved  ve;
//
//		if ( !ve.Attach() ) {
//			gout << "Cannot attach to ve." << endl;
//			exit(-1);
//		}
//
//		// Here use the ve
//
//		ve.Detach();
//		exit(0);
//		//////////////// End of example segment 2
//
//
// Arguments:
//   cLriName - the name of the lri file to load
//   message - if the function fails, this argument describes the reason
//
// Returns: The function returns true to indicate successful initialization,
//   or false to indicate failure.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::Init(const string& cLriName, string& message)
{
	char buf[356];
	CFileName lriFile(cLriName);

	// this function can only be called from TConfigured state
	if ( m_state != eCONFIGURED ) {
		message = "Init called before Configure.\n";
		return false;
	}
#if (_MSC_VER > 1500)
    #pragma warning( push )
    #pragma warning(disable:4996)
#endif
	// Translate the LriName, if necessary, with the
	// NADSSDC_LRI environment variable
	lriFile.TranslatePath("NADSSDC_LRI");

	// now open the file and read the header
	FILE *pF = fopen(lriFile.GetFullPathFileName().c_str(), "rb");
	if ( pF == NULL ) {
        pF = fopen(lriFile.TranslateAltPath("..\\data\\").c_str(), "rb");
        if (pF == nullptr){
		    sprintf_s(buf, "Cannot open file '%s': %s", cLriName.c_str(),
                strerror(errno));
            message = buf;
            return false;
        }
	}

	cvTHeader  head;
	if ( fread(&head, sizeof(head), 1, pF) != 1 ) {
		sprintf_s(buf, "Cannot read header in file '%s': %s",
				cLriName.c_str(), strerror(errno));
		message = buf;

		fclose(pF);
		return false;
	}
#if (_MSC_VER > 1500)
    #pragma warning(pop)
#endif
	// verify that this is a binary LRI file; check the
	// 'magic' field
	if ( head.magic[0] != 'L' ||
			 head.magic[1] != 'R' ||
			 head.magic[2] != 'I' ||
			 head.magic[3] != ' ' ||
			 head.magic[4] != '1' ||
			 head.magic[5] != '.' ||
			 head.magic[6] != '0' ) {
		sprintf_s(buf, "Not a compiled LRI file.\n");
		message = buf;
		fclose(pF);
		return false;
	}

	// verify the version
	// unlike earlier, this is now just a warning
    if ( head.majorVersionNum != gGetMajorCvedVersionNum() ||
			head.minorVersionNum != gGetMinorCvedVersionNum() ||
			head.minorExt1VersionNum != gGetMinorExt1CvedVersionNum() ||
			head.minorExt2VersionNum != gGetMinorExt2CvedVersionNum() ||
			head.minorExt3VersionNum != gGetMinorExt3CvedVersionNum() ){
        printf("CVED Warning: the binary lri version (%d.%d.%d.%d%d) does not match \nthe "
			" internal CVED version number (%d.%d.%d.%d.%d)\n"
			" Execution continues but use matching lricc to re-generate the binary LRI\n",
			head.majorVersionNum, head.minorVersionNum,
			head.minorExt1VersionNum, head.minorExt2VersionNum,
			head.minorExt3VersionNum,
			gGetMajorCvedVersionNum(), gGetMinorCvedVersionNum(),
			gGetMinorExt1CvedVersionNum(), gGetMinorExt2CvedVersionNum(),
			gGetMinorExt3CvedVersionNum());
    }

	// if we are in multi user mode, we create a shared segment to
	// hold the newly read data.  If we are in single user mode,
	// we allocated memory for the header.
	if ( m_mode == eCV_SINGLE_USER ) {
		char   *pAlias = new char[head.dataSize];
		m_pHdr = (cvTHeader *)pAlias;
	}
	else {
		// if an environment variable is defined, we use it as
		// the key otherwise we use the string representation
		// of a numeric value defined in the header file.
		string pKey;
        NADS::GetEnvVar(pKey, cCVED_SHARED_MEM_KEY_ENV);
		char tempBuf[64];
		if ( pKey == "" ) {
			sprintf_s(tempBuf, "%d", cCVED_SHARED_MEM_KEY);
			pKey = tempBuf;
		}

		// make sure we are not trying to re-create an
		// existing segment.  That could happen if two processes
		// are both calling Init(), as opposed to one calling
		// init and the other calling Attach()
		cvTHeader *pTest = static_cast<cvTHeader *>
					(m_shm.Connect(pKey.c_str()));
		if ( pTest && pTest->initialized ) {	// bad news, we're second
			m_shm.Disconnect();
			sprintf_s(buf, "Another process has already initialized.\n");
			message = buf;
			fclose(pF);
			return false;
		}

		m_pHdr = static_cast<cvTHeader *>
						(m_shm.Connect(pKey.c_str(), true, head.dataSize));
		if ( m_pHdr == NULL ) {
			sprintf_s(buf, "Cannot create shared segment : %s.\n",
						m_shm.GetErrorMsg());
			message = buf;
			fclose(pF);
			return false;
		}
	}

	assert(m_pHdr);

	// now read the whole file
	rewind(pF);
	if ( fread(m_pHdr, 1, head.dataSize, pF) != head.dataSize ) {
		message = "Short file.\n";
		if ( m_mode == eCV_SINGLE_USER ) {
			delete[] m_pHdr;
			m_pHdr = 0;
		}
		fclose(pF);
		return false;
	}

	fclose(pF);

	m_objNameToId.clear();

	MemBlockInit();				// do any necessary initializations
	ClassInit();				// do any class specific initializations
	m_pHdr->numClients++;
	m_pHdr->initialized = 1;	// allow clients to connect
	m_state = eACTIVE;

	double center[3] = {0, 0, 0}, extents[3] = { 80000, 80000, 400 }, elevation = -100.0;
	m_pOde = std::unique_ptr<CODE>(new CODE(this));
	m_pOde->Initialize( center, extents, elevation );
	m_FirstTimeLightsNear = true; //
	return true;
} // end of Init

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function re-initilializes the internal data structures
//   to	reflect the virtual environment at the same state as after been
//   loaded by the Init function.
//
// Remarks: After this function has been run, no dynamic objects exist, all
//   reconfigurable objects revert back to their default attributes, and any
//   other user configurable aspects of the environment are reset to their
//   default state.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::ReInit(void)
{
	MemBlockInit();

	int objId;
	for ( objId=0; objId < cNUM_DYN_OBJS; objId++ ) {
		delete m_dynObjCache[objId];
		m_dynObjCache[objId] = 0;
	}
} // end of ReInit

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function rebuilds all internal data structures to
//   reflect the current state of all dynamic objects.
//
// Remarks: Any changes to the states of dynamic objects are not reflected
//   in object searches until after this function is called.  At the same
//   time, clients of the virtual environment that issue queries to CVED
//   will get consistent answers for the period between two successive
//   invokations of the maintainer.
//
//   As an example, consider an object located at the end of a road near
//   edge of an intersection, immediately after the maintainer executes.
//   A query that returns all objects on the intersection will correctly
//   not include that object in the list.  Now consider what happens with
//   the execution of the dynamic model.  The position of the vehicle is
//   updated, and assuming it travels towards the intersection, at least
//   part of it will be on the intersection.  If at that point, the
//   previous query is re-issued, it will return the same answer as before
//   (i.e., the object is not in the intersection).  After the maintainer
//   executes, the query will correctly find the object in the intersection.
//
//   The execution time of this function is linear to the number of dynamic
//   objects that have moved since the last invokation of this function.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::Maintainer(void)
{
	int    i;
	TObj   *pO;

	if ( m_debug > 0 ) {
		gout << "->Enter CCved::Maintainer, frm=" << m_pHdr->frame << endl;
	}

	// implement the object phase transition diagram
	LockObjectPool();
	for (i=0, pO = BindObj(0); i<cNUM_DYN_OBJS; i++, pO++) {
		if ( pO->phase == eBORN ) {
			if ( m_debug > 2 ) {
				gout << "Object " << i << " from Born->Alive " << endl;
			}
			pO->phase = eALIVE;
		}
		else
		if ( pO->phase == eDYING ) {
			if ( m_debug > 2 ) {
				gout << "Object " << i << " from Dying->Dead " << endl;
			}
			pO->phase = eDEAD;
			m_pHdr->dynObjectCount--;
		}
	}

	//
	// Copy dynamic object state between buffers.
	//
	for( i = 0, pO = BindObj( 0 ); i < cNUM_DYN_OBJS; i++, pO++ )
	{
		if( pO->phase == eALIVE )
		{
			if( (m_pHdr->frame & 1) == 0 )
			{		// even frame
				pO->stateBufA.state = pO->stateBufB.state;
			}
			else {
				pO->stateBufB.state = pO->stateBufA.state;
			}

		}
	}
	m_pHdr->frame++;
	UnlockObjectPool();

	FillByRoadDynObjList();

#ifdef DEBUG_MAINTAINER
if( m_pHdr->frame > 0 ) {

	gout << "<<<<< *************** Maintainer [frame=" <<  m_pHdr->frame;
	gout << "] *************** >>>>>" << endl;

	int s;
	for( s = 0; s < m_pHdr->roadRefCount; s++ )
	{
		if( m_pRoadRefPool[s].objIdx != 0 )
		{
			gout << "RoadRefPool[" << s << "].objIdx = ";
			gout << m_pRoadRefPool[s].objIdx << endl;

		}
	}
	for( s = 0; s < m_pHdr->intrsctnRefCount; s++ )
	{
		if( m_pIntrsctnRefPool[s].objIdx != 0 )
		{
			gout << "IntrsctnRefPool[" << s << "].objIdx = ";
			gout << m_pIntrsctnRefPool[s].objIdx << endl;
		}
	}

	for( s = 0; s < m_pHdr->dynObjRefCount; s++ )
	{
		if( m_pDynObjRefPool[s].objId == 0 ) continue;

		gout << "DynObjRefPool[" << s << "]:" << endl;

		cvTDynObjRef* pDorNode = &( m_pDynObjRefPool[s] );
		if( pDorNode->objId != 0 )
			gout << "  objId = "<< pDorNode->objId << endl;
		if( (int)pDorNode->terrain != 0 )
			gout << "  terrain = "<< (int)pDorNode->terrain << endl;
		if( pDorNode->next != 0 )
			gout<< "  next = " << pDorNode->next << endl;
		if( pDorNode->roadId != 0 )
			gout<< "  roadId = " << pDorNode->roadId << endl;
		if( pDorNode->lanes != 0 )
			gout << "  lanes = " << pDorNode->lanes << endl;
		if( pDorNode->distance != 0.0f )
			gout << "  distance = " << pDorNode->distance << endl;
		if( (int)pDorNode->direction != 0 )
			gout << "  direction = " << (int)pDorNode->direction << endl;
		if( pDorNode->roadCntrlPnt != 0 )
			gout << "  roadCntrlPnt = " << pDorNode->roadCntrlPnt << endl;
		if( pDorNode->intrsctnId != 0 )
			gout << "  intrsctnId = " << pDorNode->intrsctnId << endl;
		if( pDorNode->corridors != 0 )
			gout << "  corridors = " << pDorNode->corridors << endl;

		int t;
		for( t = 0; t < cCV_MAX_CRDRS; t++ )
		{
			if( pDorNode->crdrDistances[t] != 0.0f )
				gout <<  "  crdrDistances[t] = " << pDorNode->crdrDistances[t] << endl;
		}
	}

	// print out objs on road 2
	int curRoadId = 14;
	int curRoadIdx = m_pRoadRefPool[curRoadId].objIdx;
	gout << "objs on road " << curRoadId << " are:  " ;
	while( curRoadIdx != 0 )
	{
		gout << m_pDynObjRefPool[curRoadIdx].objId << ",";
		curRoadIdx = m_pDynObjRefPool[curRoadIdx].next;
	}
	gout << endl;


	// print out objs on intersection 2
	int curIntrsctnId = 37;
	int curIntrsctnIdx = m_pIntrsctnRefPool[curIntrsctnId].objIdx;
	gout << "objs on intersection " << curIntrsctnId << " are:  ";
	while( curIntrsctnIdx != 0 )
	{
		gout << m_pDynObjRefPool[curIntrsctnIdx].objId << ",";
		curIntrsctnIdx = m_pDynObjRefPool[curIntrsctnIdx].next;
	}
	gout << endl;
}


#endif


	if ( m_debug > 0 ) {
		gout << "<-Exit CCved::Maintainer." << endl;
	}
} // end of Maintainer


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function informs CVED if the user plans to have
//   a fake external driver during execution.  By default, CVED assumes
//   that there is a fake external driver.
//
// Remarks: If true, the dynamic models for objects 0 and 1 are also executed
//  when ExecuteDymamicModels is called.
//
// Arguments: A boolean indicating if there is an external driver.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetFakeExternalDriver(bool haveFakeExternalDriver)
{

	m_haveFakeExternalDriver = haveFakeExternalDriver;

}  // end of SetFakeExternalDriver


//////////////////////////////////////////////////////////////////////////////
//
// Description: Is CVED running with a fake external driver?
//
// Remarks:
//
// Arguments:
//
// Returns: A boolean indicating if there is an external driver.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::HaveFakeExternalDriver() const
{

	return m_haveFakeExternalDriver;

}  // end of SetFakeExternalDriver


//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the external driver's hcsm id.
//
// Remarks: This function is necessary since the HCSM id is not available
//   to the entity that creates the external driver object on the simulator.
//   This function will be called by the DriverMirror with its HCSM id
//   after the external driver object has been created. If the external
//   trailer is present, it will also share the HCSM id of DriverMirror.
//
// Arguments:
//   hcsmId - The external driver object's HCSM id.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetExternalDriverHcsmId( int hcsmId )
{
	//
	// Bind object 0 as the external driver and make sure it's a valid
	// object before setting its id.
	//
	TObj* pO = BindObj( 0 );
	if( pO->phase == eALIVE || pO->phase == eDYING )
	{
		pO->attr.hcsmId = hcsmId;
	}

	//
	// Bind object 1 as the external trailer and make sure it's a valid
	// object before setting its id.
	//
	pO = BindObj( 1 );
	if( pO->phase == eALIVE || pO->phase == eDYING )
	{
		pO->attr.hcsmId = hcsmId;
	}
}  // end of SetExternalDriverHcsmId


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function executes one iteration of the dynamic model
//   for all dynamic objects in the virtual environment.
//
// Remarks: The time step used for the dynamic model calculations is equal
//   to the time step specified for the maintainer divided by the dynamic
//   model multiplier.  For example, in the Configure function, if the
//   timestep was specified as 0.1 and the multiplier was 2, then the time
//   step will be 0.05 or the equivalent of 20Hz.
//
//   This function is designed to execute concurrently with the maintainer
//   or any other interrogations, even from different CVED instances.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::ExecuteDynamicModels( IExternalObjectControl* ctrl )
{
	TObjectPoolIdx id;
	TObj*          pO;
	int            count = 0; // optimization
	TObjectPoolIdx attachedObjIds[cNUM_DYN_OBJS]; // list of objects that are attached to others
	                                              // they need to be processed after their parents
	                                              // are processed.
	TObjectPoolIdx freeMotionObjIds[cNUM_DYN_OBJS]; // list of objects in free motion. The main
	                                                // loop will create the ode objects for them
	                                                // if they are newly transitioned to free motion
	                                                // mode, while the second loop processes them
	                                                // after the ode simstep so that updated pos,
	                                                // rot, vel can be obtained. This way newly
	                                                // transitioned free motion objects won't lose
	                                                // one frame of dynamics.
	int            attachedObjCount = 0, freeMotionObjCount = 0, i;
//	static short timer = 0;
//	double currentSecs;

	/*
	if ( !timer )
	{
		timer   = hrt_timer_alloc(HRT_MICROSECOND, "");
		hrt_timer_start( timer );
	}
	printf("ExecDyna: current sec %f simstep size %f\n",
		hrt_timer_currentsecs( timer ), m_pHdr->deltaT / m_pHdr->dynaMult * 2);
	*/
	if (NULL != ctrl)
		ctrl->PreUpdateDynamicModels();
	if( m_haveFakeExternalDriver
	|| NULL != ctrl )
	{
		id = 0;
	}
	else
	{
		id = cMAX_EXT_CNTRL_OBJS;
	}
	pO = BindObj( id );

	while( id < cNUM_DYN_OBJS )
	{
		bool executeDynaModel = (
					(
						pO->type == eCV_TRAJ_FOLLOWER ||
						pO->type == eCV_VEHICLE ||
						pO->type == eCV_EXTERNAL_DRIVER
						)
					&&
					( pO->phase == eALIVE || pO->phase == eDYING )
					);
		bool localSim = (0 == id);
		bool peerSim = (0 != id
					&& (pO->type == eCV_EXTERNAL_DRIVER || pO->type == eCV_VEHICLE)); //fixme: pO->type == eCV_VEHICLE

//		if ( pO->type == eCV_TRAJ_FOLLOWER )
//			fprintf(stdout, " traj follower %d phase %d\n", id, pO->phase);

		if( executeDynaModel )
		{
			cvTObjState currState;
			const cvTObjState* pCurrState = &currState;
			cvTObjState* pFutState;
			const cvTObjContInp* pCurrContInp;

			// on even frames, dynamic servers use buffer A for
			// control inputs and buffer B for state.  Vice versa
			// for odd frames.  Same attributes are used as
			// attributes don't change based on the frame.

			if( (m_pHdr->frame & 1) == 0 )
			{
				// even frame
				pCurrContInp = &pO->stateBufA.contInp;
				currState    = pO->stateBufB.state;
				pFutState    = &pO->stateBufB.state;
			}
			else
			{
				// odd frame
				pCurrContInp = &pO->stateBufB.contInp;
				currState    = pO->stateBufA.state;
				pFutState    = &pO->stateBufA.state;
			}

			if ( pO->type == eCV_TRAJ_FOLLOWER )
			{
				if ( currState.trajFollowerState.curMode == eCV_FREE_MOTION )
				// free motion object, its pos, rot, vel information has to
				// be obtained in a separate loop after the ode sim step.
				{
					freeMotionObjIds[freeMotionObjCount] = id;
					++freeMotionObjCount;
				}

				if ( currState.trajFollowerState.curMode != eCV_GROUND_TRAJ )
				// traj follower that was or is attached to a parent object, delay the dynamics computation
				{
					attachedObjIds[attachedObjCount] = id;
					++attachedObjCount;
				}
				else{
					CVED::CCved &me = *this;
					CVED::CObj* obj = BindObjIdToClass2(id);
					if ( !peerSim
					 ||	(NULL == ctrl
					 ||	!ctrl->OnPeerSimUpdating(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState)))
					{
						if (!localSim
						 || m_haveFakeExternalDriver) //local simulator has its own dynamic
						DynamicModel(
							id,
							pO->type,
							&pO->attr,
							pCurrState,
							pCurrContInp,
							pFutState
							);
					}
					if (localSim
						&& NULL != ctrl)
						ctrl->OnOwnSimUpdated(pCurrContInp, pFutState);

				}
			}
			else
			{
				CVED::CCved &me = *this;
				CVED::CObj* obj = BindObjIdToClass2(id);
				if ( !peerSim
					 ||	(NULL == ctrl
					 ||	!ctrl->OnPeerSimUpdating(id, const_cast<cvTObjContInp*>(pCurrContInp), pFutState)))
				{
					if (!localSim
					 || m_haveFakeExternalDriver) //local simulator has its own dynamic
					DynamicModel(
						id,
						pO->type,
						&pO->attr,
						pCurrState,
						pCurrContInp,
						pFutState
						);
				}
				if (localSim
					&& NULL != ctrl)
						ctrl->OnOwnSimUpdated(pCurrContInp, pFutState);
			}

			// This is an optimization.
			// We know the total number of dynamic objects because
			// is stored in the header so once we have processed
			// as many objects, there is no reason to keep going through
			// the object pool since the rest of the objects will
			// certainly be in the TDead state
			count++;
			if( count == m_pHdr->dynObjectCount )  break;
		}

		id++;
		pO++;
	}

//	fprintf(stdout, "processing %d attached objs now\n", attachedObjCount);
	// now process the child objects
	for ( i=0; i<attachedObjCount; ++i )
	{
//		fprintf(stdout, "attached obj %d\n", attachedObjIds[i]);
		pO = BindObj( attachedObjIds[i] );

//		fprintf(stdout, "bind ok\n");
		cvTObjState currState;
		const cvTObjState* pCurrState = &currState;
		cvTObjState* pFutState;
		const cvTObjContInp* pCurrContInp;

		// on even frames, dynamic servers use buffer A for
		// control inputs and buffer B for state.  Vice versa
		// for odd frames.  Same attributes are used as
		// attributes don't change based on the frame.

		if( (m_pHdr->frame & 1) == 0 )
		{
			// even frame
			pCurrContInp = &pO->stateBufA.contInp;
			currState    = pO->stateBufB.state;
			pFutState    = &pO->stateBufB.state;
		}
		else
		{
			// odd frame
			pCurrContInp = &pO->stateBufB.contInp;
			currState    = pO->stateBufA.state;
			pFutState    = &pO->stateBufA.state;
		}

		CVED::CCved &me = *this;
		bool localSim = (0 == attachedObjIds[i]);
		bool peerSim = (0 != attachedObjIds[i]
					&& pO->type == eCV_EXTERNAL_DRIVER);
		if ( !peerSim
		||	(NULL == ctrl
		||	!ctrl->OnPeerSimUpdating(attachedObjIds[i], const_cast<cvTObjContInp*>(pCurrContInp), pFutState)))
		{
			if (!localSim
			 || m_haveFakeExternalDriver) //local simulator has its own dynamic
			DynamicModel(
				attachedObjIds[i],
				pO->type,
				&pO->attr,
				pCurrState,
				pCurrContInp,
				pFutState
				);
		}
		if (localSim
		&& NULL != ctrl)
			ctrl->OnOwnSimUpdated(pCurrContInp, pFutState);
	}

	// execute ode dynamics from objects in free motion mode
	m_pOde->SimStep( (float)m_pHdr->deltaT / m_pHdr->dynaMult );

	// get updated information of free motion objects
	for ( i=0; i<freeMotionObjCount; ++i )
	{
//		fprintf(stdout, "free motion obj %d\n", freeMotionObjIds[i]);
		pO = BindObj( freeMotionObjIds[i] );

//		fprintf(stdout, "bind ok\n");
		cvTObjState* pFutState;
		double position[3], tangent[3], lateral[3];
		CODEObject* pOdeObj;

		// on even frames, dynamic servers use buffer A for
		// control inputs and buffer B for state.  Vice versa
		// for odd frames.  Same attributes are used as
		// attributes don't change based on the frame.

		if( (m_pHdr->frame & 1) == 0 )
		{
			// even frame
			pOdeObj = (CODEObject*)(pO->stateBufB.state.trajFollowerState.pODEObj);
			pFutState = &pO->stateBufB.state;
		}
		else
		{
			// odd frame
			pOdeObj = (CODEObject*)(pO->stateBufA.state.trajFollowerState.pODEObj);
			pFutState = &pO->stateBufA.state;
		}

		pOdeObj->GetPosRot( position, tangent, lateral );
		pFutState->anyState.position.x = position[0];
		pFutState->anyState.position.y = position[1];
		pFutState->anyState.position.z = position[2];
		pFutState->anyState.tangent.i = tangent[0];
		pFutState->anyState.tangent.j = tangent[1];
		pFutState->anyState.tangent.k = tangent[2];
		pFutState->anyState.lateral.i = lateral[0];
		pFutState->anyState.lateral.j = lateral[1];
		pFutState->anyState.lateral.k = lateral[2];

		bool DebugModeFlag = false;

		if ( DebugModeFlag )
			printf("FM object #%d (%.4f %.4f %.4f) tan (%.4f %.4f %.4f) lat (%.4f %.4f %.4f)\n",
				freeMotionObjIds[i],
				position[0], position[1], position[2],
				tangent[0], tangent[1], tangent[2],
				lateral[0], lateral[1], lateral[2]);


	}
	if (NULL != ctrl)
		ctrl->PostUpdateDynamicModels();

} // end of ExecuteDynamicModels

//////////////////////////////////////////////////////////////////////////////
//	Road related
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all roads in the virtual environment.
//
// Remarks: The roads are inserted in the provided STL vector in no specific
//   order.
//
// Arguments:
//   out - the container to hold the roads.  Previous contents will be erased.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllRoads(TRoadVec &out) const
{
	out.clear();
	out.reserve(m_pHdr->roadCount);

	TU32b road;
	for (road = 1; road < m_pHdr->roadCount; road++) {
		CRoad  r(*this, road);
		out.push_back(r);
	}
} // end of GetAllRoads

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all road pieces in the virtual
//   environment.
//
// Remarks: The road pieces are inserted in the provided STL vector in no
//   specific order.
//
// Arguments:
//   out - The container to hold the road pieces.  Previous contents will be
// 		   erased.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllRoadPieces(vector<CRoadPiece> &out) const
{
	out.clear();
	out.reserve(m_pHdr->roadPieceCount);

	TU32b piece;
	for(piece = 1; piece < m_pHdr->roadPieceCount; piece++){
		CRoadPiece r(*this, piece);
		out.push_back(r);
	}
} // end of GetAllRoadPieces

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the road segment associated with the two control
//   points parameters.
//
// Remarks: The segment consists of 4 CPoint2D instances, returned in
//   counter-clockwise order.
//
// Arguments:
//   pCurr - pointer to the current control point instance
//   pNext - pointer to the next control point instance
//   segment - location for the resulting 4 CPoint2D instances
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetSegment(
			cvTCntrlPnt*	pCurr,
			cvTCntrlPnt*	pNext,
			CPoint2D		segment[4]
			)
{
	TLatCntrlPntPoolIdx firstLatPnt = pCurr->latCntrlPntIdx;
	TLatCntrlPntPoolIdx lastLatPnt;

	if (firstLatPnt != 0) {
		lastLatPnt = firstLatPnt + pCurr->numLatCntrlPnt-1;

		cvTLatCntrlPnt* pLatPnts;
		pLatPnts = (cvTLatCntrlPnt *)
			(((char *)m_pHdr) + m_pHdr->latCntrlPntOfs);

		segment[0].m_x = pCurr->location.x + pCurr->rightVecLinear.i *
			pLatPnts[firstLatPnt].offset;
		segment[0].m_y = pCurr->location.y + pCurr->rightVecLinear.j *
			pLatPnts[firstLatPnt].offset;
		segment[1].m_x = pCurr->location.x + pCurr->rightVecLinear.i *
			pLatPnts[lastLatPnt].offset;
		segment[1].m_y = pCurr->location.y + pCurr->rightVecLinear.j *
			pLatPnts[lastLatPnt].offset;
	} else {
		segment[0].m_x = pCurr->location.x + pCurr->rightVecLinear.i *
								(-pCurr->logicalWidth/2.0);
		segment[0].m_y = pCurr->location.y + pCurr->rightVecLinear.j *
								(-pCurr->logicalWidth/2.0);
		segment[1].m_x = pCurr->location.x + pCurr->rightVecLinear.i *
								(pCurr->logicalWidth/2.0);
		segment[1].m_y = pCurr->location.y + pCurr->rightVecLinear.j *
								(pCurr->logicalWidth/2.0);
	}

	firstLatPnt = pNext->latCntrlPntIdx;
	if (firstLatPnt != 0) {
		lastLatPnt = firstLatPnt + pNext->numLatCntrlPnt - 1;

		cvTLatCntrlPnt* pLatPnts;
		pLatPnts = (cvTLatCntrlPnt *)
			(((char *)m_pHdr) + m_pHdr->latCntrlPntOfs);

		segment[2].m_x = pNext->location.x + pNext->rightVecLinear.i *
			pLatPnts[lastLatPnt].offset;
		segment[2].m_y = pNext->location.y + pNext->rightVecLinear.j *
			pLatPnts[lastLatPnt].offset;
		segment[3].m_x = pNext->location.x + pNext->rightVecLinear.i *
			pLatPnts[firstLatPnt].offset;
		segment[3].m_y = pNext->location.y + pNext->rightVecLinear.j *
			pLatPnts[firstLatPnt].offset;
	}
	else {
		segment[2].m_x = pNext->location.x + pNext->rightVecLinear.i *
								(pNext->logicalWidth/2.0);
		segment[2].m_y = pNext->location.y + pNext->rightVecLinear.j *
								(pNext->logicalWidth/2.0);
		segment[3].m_x = pNext->location.x + pNext->rightVecLinear.i *
								(-pNext->logicalWidth/2.0);
		segment[3].m_y = pNext->location.y + pNext->rightVecLinear.j *
								(-pNext->logicalWidth/2.0);
	}
} // end of GetSegment

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function is used to retrieve multiple segments along
//   a road.
//
// Remarks: It sets the seconds and third CPoint2D instances in the segment
//   array using the control point parameter.
//
// Arguments:
//   pNext - pointer to the next control point
//   segment - location for the segment points
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetSegment(
			cvTCntrlPnt*	pNext,
			CPoint2D		segment[4]
			)
{

	TLatCntrlPntPoolIdx firstLatPnt = pNext->latCntrlPntIdx;

	if (firstLatPnt != 0) {
		TLatCntrlPntPoolIdx lastLatPnt = firstLatPnt + pNext->numLatCntrlPnt -1;

		cvTLatCntrlPnt* pLatPnts;
		pLatPnts = (cvTLatCntrlPnt *)
			(((char *)m_pHdr) + m_pHdr->latCntrlPntOfs);

		segment[2].m_x = pNext->location.x + pNext->rightVecLinear.i *
			pLatPnts[lastLatPnt].offset;
		segment[2].m_y = pNext->location.y + pNext->rightVecLinear.j *
			pLatPnts[lastLatPnt].offset;
		segment[3].m_x = pNext->location.x + pNext->rightVecLinear.i *
			pLatPnts[firstLatPnt].offset;
		segment[3].m_y = pNext->location.y + pNext->rightVecLinear.j *
			pLatPnts[firstLatPnt].offset;
	}
	else {
		segment[2].m_x = pNext->location.x + pNext->rightVecLinear.i *
								(pNext->logicalWidth/2.0);
		segment[2].m_y = pNext->location.y + pNext->rightVecLinear.j *
								(pNext->logicalWidth/2.0);
		segment[3].m_x = pNext->location.x + pNext->rightVecLinear.i *
								(-pNext->logicalWidth/2.0);
		segment[3].m_y = pNext->location.y + pNext->rightVecLinear.j *
								(-pNext->logicalWidth/2.0);
	}
} // end of GetSegment

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns a class representing the road whose
//   identifier is given as an argument.
//
// Remarks: Using integer identifiers to represent roads is not recommeded
//   since the road identifiers could potentially change if a given LRI file
//   is recompiled.  The use of the road's name is the preferred way to
//   reference roads.
//
// Arguments:
//   id - the identifier of the road
//
// Returns: The function returns a class representing the road. If the id is
//   invalid the function returns an invalid road.
//
//////////////////////////////////////////////////////////////////////////////
CRoad
CCved::GetRoad(int id) const
{
	CRoad  r(*this, id);

	return r;
} // end of GetRoad


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns a class representing the road
//   whose name is given as an argument.
//
// Remarks: Using integer identifiers to represent roads is not recommeded
//   since the road identifiers could potentially change if a given LRI file
//   is recompiled.  The use of the road's name is the preferred way to
//   reference roads.
//
// Arguments:
//   cName - the name of the road
//
// Returns: The function returns a class representing the road. If the name
//   does not correspond to an existing road's name, the function returns an
//   invalid road.
//
//////////////////////////////////////////////////////////////////////////////
CRoad
CCved::GetRoad(const string &cName) const
{
	CRoad RetVal;

	TStr2IntMap::const_iterator  item;

	item = m_roadNameMap.find(cName);
	if ( item != m_roadNameMap.end() && (*item).second > 0 ) {
		CRoad r(*this, (*item).second);
		return r;
	}
	if ( item == m_roadNameMap.end() ) {
		string msg;

		msg = "invalid road:";
		msg += cName;
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	return RetVal;
} // end of GetRoad


//////////////////////////////////////////////////////////////////////////////
//	Intersection related
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns all intersections in the virtual environment.
//
// Remarks: The intersections are inserted in the provided STL vector in no
// 	specific order.
//
// Arguments:
// 	out - the container to hold the intersections.  Previous contents will
// 		be erased
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllIntersections(CCved::TIntrsctnVec &out) const
{
	out.clear();
	out.reserve(m_pHdr->intrsctnCount);

	TU32b inter;
	for (inter = 1; inter < m_pHdr->intrsctnCount; inter++) {
		CIntrsctn  i(*this, inter);
		out.push_back(i);
	}
} // end of GetAllIntersections

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns a class representing the intersection whose
// 	identifier is given as an argument.
//
// Remarks: Using integer identifiers to represent intersections is not
// 	recommeded since the identifiers could potentially change if a given LRI
// 	file is recompiled.  The use of the name is the preferred way to
// 	reference intersections.
//
// Arguments:
//	id  - the identifier of the intersection
//
// Return value: The function returns a class representing the intersection.
// 	If the id is invalid the function returns an invalid intersection.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CCved::GetIntersection(int id) const
{
	CIntrsctn  i(*this, id);

	return i;
} // end of GetIntersection

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns a class representing the intersection whose name is
// 	given as an argument.
//
// Remarks: Using integer identifiers to represent intersections is not
// 	recommeded since the identifiers could potentially change if a given LRI
// 	file is recompiled.  The use of the name is the preferred way to
// 	reference intersections.
//
// Arguments:
// 	cName - the name of the intersection
//
// Return value: The function returns a class representing the intersection.
// 	If the name does not correspond to an existing intersection's name, the
// 	function returns an invalid intersection.
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CCved::GetIntersection(const string &cName) const
{
	CIntrsctn rval;		// an invalid one, in case we can't find it

	TStr2IntMap::const_iterator  item;

	item = m_intrsctnNameMap.find(cName);
	if ( item != m_intrsctnNameMap.end() && (*item).second > 0 ) {
		CIntrsctn i(*this, (*item).second);
		return i;
	}
	if ( item == m_intrsctnNameMap.end() ) {
		string msg;

		msg = "invalid intersection: ";
		msg += cName;

		cvCInternalError e( msg, __FILE__, __LINE__ );

		throw e;
	}

	return rval;
} // end of GetIntersection

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the total number of intersections.
//
// Remarks:
//
// Arguments:
//
// Returns: An integer specifying the number of intersections.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetNumIntersections() const
{
	return m_pHdr->intrsctnCount;
} // end of GetNumIntersections
//////////////////////////////////////////////////////////////////////////////
///\brief
///     Gets all the control points near a pnt
///\remark
///     This does only a 2D projections, the Z is ignored. The points are near
///     if they lay within the  axis-aligned min bounding box of the road
///     segment, current point + next point
///\return true if crdr exist, else returns false
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetCrdrsCntrlPointsNear(const CPoint3D& pnt,int crdrId,TIntVec& result) const{

    auto itr = m_intersectionMap.find(crdrId);
    if (itr == m_intersectionMap.end()){
        return false;
    }
    (*itr->second).SearchRectangle(CBoundingBox(pnt,pnt),result);
    return true;
}

//////////////////////////////////////////////////////////////////////////////
//	Object related
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns the number of objects whose type is included in the
// 	mask.
//
// Remarks: Static, relocatable and dynamic objects are included in the count.
//
// Arguments:
//  mask - (optional) object type mask indicating which object types to count.
//  	Default value is all object types.
//
// Returns: The number of objects.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetNumObjects(CObjTypeMask mask) const
{
	return GetNumStaticObjs(mask) + GetNumDynamicObjs(mask);
} // end of GetNumObjects

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns the number of static objects whose type is included
// 	in the mask.
//
// Remarks: Reconfigurable objects are included in the count.
//
// Arguments:
//  mask - (optional) object type mask indicating which object types to count.
//  	Default value is all object types.
//
// Returns: The number of static objects.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetNumStaticObjs(CObjTypeMask mask) const
{
	int    count;
	TU32b    i;
	TObj  *pO;

	// static objects are stored at the slots immediately after
	// the dynamic objects.  The objectCount field of the header
	// contains the total number of objects that are active in
	// the current cved instance so we simply have to go through
	// all the slots and verify that they are not reconfigurable
	// and their type is in the mask.
	count = 0;
	i     = cNUM_DYN_OBJS;
	while ( i < m_pHdr->objectCount ) {
		pO = BindObj(i);
		assert(pO->phase == eALIVE);
		if ( mask.Has(pO->type) ) count++;
		i++;
	}

	return count;
} // end of GetNumStaticObjs

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns the number of dynamic objects whose type is included
// 	in the mask.
//
// Remarks: The ownship is included in the count.
//
// Arguments:
//  mask - (optional) object type mask indicating which object types to count.
//  	Default value is all object types.
//
// Returns: The number of dynamic objects.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetNumDynamicObjs(CObjTypeMask mask) const
{
	int    count = 0;
	int    i;
	TObj  *pO;

	for (i=0, pO = BindObj(0); i<cNUM_DYN_OBJS; i++, pO++) {
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type, i) ) count++;
		}
	}

	return count;
} // end of GetNumDynamicObjs

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns all objects whose type is included in the specified
// 	mask.
//
// Remarks: The output is given in a set of integers that are the identifiers
// 	of all applicable objects.  The appropriate constructor or conversion
// 	operators can be used to obtain the appropriately typed object class.
//
// Arguments:
//	out - the container to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllObjs(TIntVec &out, CObjTypeMask mask) const
{
	out.clear();
	GetAllStaticObjs(out, mask);

	vector<int> dynlist;
	GetAllDynamicObjs(dynlist, mask);
	out.insert(out.end(), dynlist.begin(), dynlist.end());
} // end of GetAllObjs

//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function returns all static objects whose type is included in the
//	specified mask.
//
// Remarks: The output is given in a set of integers that are the identifiers
// 	of all applicable objects.  The appropriate constructor or conversion
// 	operators can be used to obtain the appropriately typed object class.
//
// Arguments:
//	out - the container to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllStaticObjs(TIntVec &out, CObjTypeMask mask) const
{

	// static objects are stored at the slots immediately after
	// the dynamic objects.  The objectCount field of the header
	// contains the total number of objects that are active in
	// the current cved instance so we simply have to go through
	// all the slots and verify that they are not reconfigurable
	// and their type is in the mask.

	TU32b    i  = cNUM_DYN_OBJS;

	TObj  *pO = BindObj(i);
	out.clear();

	while ( i < m_pHdr->objectCount ) {
		assert(pO->phase == eALIVE);	// static objects have to be Alive
		if ( mask.Has(pO->type) ) {
			out.push_back(i);
		}
		i++;
		pO++;
	}
} // end of GetAllStaticObjs

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns all dynamic objects whose type is included in the
// 	specified mask.
//
// Remarks: The output is given in a set of integers that are the identifiers
// 	of all applicable objects.  The appropriate constructor or conversion
// 	operators can be used to obtain the appropriately typed object class.
//
// Arguments:
//	out - the container to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynamicObjs(TIntVec &out, CObjTypeMask mask) const
{
	int    i;
	TObj  *pO;
	out.clear();
	for (i=0, pO = BindObj(0); i<cNUM_DYN_OBJS; i++, pO++) {
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type, i) ) out.push_back(i);
		}
	}
} // end of GetAllDynamicObjs

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the OwnVehicle's current position.
//
// Remarks:
//
// Arguments:
//	pos - (output) The OwnVehicle's position.
//
// Returns: A boolean indicating if it is returning a valid position.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetOwnVehiclePos( CPoint3D& pos ) const
{
	TObj* pO = BindObj( 0 );

	if( pO->phase == eALIVE )
	{
		if( (m_pHdr->frame & 1) == 0 )
		{
			pos = pO->stateBufA.state.anyState.position;
		}
		else
		{
			pos = pO->stateBufB.state.anyState.position;
		}

		return true;
	}
	else
	{
		// ownvehicle not alive
		return false;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the OwnVehicle's current tangent vector.
//
// Remarks:
//
// Arguments:
//	tang - (output) The OwnVehicle's tangent vector.
//
// Returns: A boolean indicating if it is returning a valid tangent vector.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetOwnVehicleTan( CVector3D& tang ) const
{
	TObj* pO = BindObj( 0 );

	if( pO->phase == eALIVE )
	{
		if( (m_pHdr->frame & 1) == 0 )
		{
			tang = pO->stateBufA.state.anyState.tangent;
		}
		else
		{
			tang = pO->stateBufB.state.anyState.tangent;
		}

		return true;
	}
	else
	{
		// ownvehicle not alive
		return false;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the OwnVehicle's current lateral.
//
// Remarks:
//
// Arguments:
//	lat - (output) The OwnVehicle's lateral.
//
// Returns: A boolean indicating if it is returning a valid lateral.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetOwnVehicleLat( CVector3D& lat ) const
{
	TObj* pO = BindObj( 0 );

	if( pO->phase == eALIVE )
	{
		if( (m_pHdr->frame & 1) == 0 )
		{
			lat = pO->stateBufA.state.anyState.lateral;
		}
		else
		{
			lat = pO->stateBufB.state.anyState.lateral;
		}

		return true;
	}
	else
	{
		// ownvehicle not alive
		return false;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the OwnVehicle's current angular velocity.
//
// Remarks:
//
// Arguments:
//	angularVel - (output) The OwnVehicle's angular velocity.
//
// Returns: A boolean indicating if it is returning a valid lateral.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetOwnVehicleAngularVel( CVector3D& angularVel ) const
{
	TObj* pO = BindObj( 0 );

	if( pO->phase == eALIVE )
	{
		if( (m_pHdr->frame & 1) == 0 )
		{
			angularVel.m_i = pO->stateBufA.state.externalDriverState.angularVel.i;
			angularVel.m_j = pO->stateBufA.state.externalDriverState.angularVel.j;
			angularVel.m_k = pO->stateBufA.state.externalDriverState.angularVel.k;
		}
		else
		{
			angularVel.m_i = pO->stateBufB.state.externalDriverState.angularVel.i;
			angularVel.m_j = pO->stateBufB.state.externalDriverState.angularVel.j;
			angularVel.m_k = pO->stateBufB.state.externalDriverState.angularVel.k;
		}

		return true;
	}
	else
	{
		// ownvehicle not alive
		return false;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the OwnVehicle's current velocity.
//
// Remarks:
//
// Arguments:
//	vel - (output) The OwnVehicle's velocity.
//
// Returns: A boolean indicating if it is returning a valid velocity.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetOwnVehicleVel( double& vel ) const
{
	TObj* pO = BindObj( 0 );

	if( pO->phase == eALIVE )
	{
		if( (m_pHdr->frame & 1) == 0 )
		{
			vel = pO->stateBufA.state.anyState.vel;
		}
		else
		{
			vel = pO->stateBufB.state.anyState.vel;
		}

		return true;
	}
	else
	{
		//
		// OwnVehicle not alive.
		//
		return false;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all the static objects whose type is
//  included in the specified mask and are located near the specified point
//  and have changed since the last time this function was called with the
//  same changedId.
//
// Remarks: The output is given in a set of integers that are the identifiers
// 	of all applicable objects.  The appropriate constructor or conversion
// 	operators can be used to obtain the appropriately typed object class.
//
//	To be included in the output, the bounding box of an object should overlap
//	a circle whose center is at loc and has the given radius.
//
// Arguments:
//  cLoc - the location to search for objects
//  radius - how far around loc to search for objects
//  changedId - the id of the client, used so more than one client can query
//      for changed objects.
//	out - the STL vector to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetChangedStaticObjsNear(
			const CPoint3D& cLoc,
			double          boxHalfSide,
			TU8b            changedId,
			TIntVec&        out,
			CObjTypeMask    mask
			) const
{
	CPoint3D objPos;
	TU8b changedMask = 1 << changedId;
	out.clear();

	TObj* pO = BindObj( 0 );
	CBoundingBox bbox(
				cLoc.m_x - boxHalfSide,
				cLoc.m_y - boxHalfSide,
				cLoc.m_x + boxHalfSide,
				cLoc.m_y + boxHalfSide
				);

	// for static objects in the LRI
	vector<int> soVec;
	m_staticObjQTree.SearchRectangle(bbox, soVec);
	vector<int>::const_iterator soIter;
	for( soIter = soVec.begin(); soIter != soVec.end(); soIter++ )
	{
		if (mask.Has(GetObjType(*soIter))) {
			objPos = GetObjPos(*soIter);
			pO = BindObj( *soIter );
			if (pO->changedFlag & changedMask) {
				out.push_back( *soIter );
				pO->changedFlag &= ~changedMask;
			}
		}
	}

	// static objects created at runtime
	TU32b soIdx = m_pHdr->objectCountInitial;
	for	(; soIdx < m_pHdr->objectCount; soIdx++){
		if (mask.Has(GetObjType(soIdx))){
			objPos = GetObjPos(soIdx);
			if (bbox.Encloses(objPos)) {
				pO = BindObj( soIdx );
				if (pO->changedFlag & changedMask) {
					out.push_back( soIdx );
					pO->changedFlag &= ~changedMask;
				}
			}
		}
	}
} // GetChangedStaticObjsNear


//////////////////////////////////////////////////////////////////////////////
//
// Description: Gets up to maxSize closest traffic lights to the given
//  location.
//
// Remarks:
//
// Arguments:
//  loc - The location to search for objects.
//  pCvedId - (output) An array of cved id of traffic lights.
//	maxSize - The maximum size of the output array.
//
// Returns: The actual number of traffic lights written to the output array.
//
//////////////////////////////////////////////////////////////////////////////
class CTrafLightData {
public:
	int cvedId;
	double sqrdDistFromLoc;
	bool operator<( const CTrafLightData& cTl ) const { return sqrdDistFromLoc < cTl.sqrdDistFromLoc; };
	bool operator>( const CTrafLightData& cTl ) const { return sqrdDistFromLoc > cTl.sqrdDistFromLoc; };
};

struct TTrafLightData
{
	int cvedId;
	double sqrdDistFromLoc;
};

int
CCved::GetTrafLightsNear(
			const CPoint3D& loc,
			int* pCvedId,
			int maxSize
			)
{
#ifdef _WIN32
	WaitForSingleObject(m_MUTEX_LightsNear,1);
#endif
/*
	static bool sFirstTime = true;
*/
	//static vector<CTrafLightData> m_tlData;

	if( m_FirstTimeLightsNear )
	{
		//
		// Get all the traffic lights and insert them into the vector.
		//
		CObjTypeMask objMask;
		objMask.Set( eCV_TRAFFIC_LIGHT );

		TU32b i  = cNUM_DYN_OBJS;
		TObj* pObj = BindObj( i );

		while( i < m_pHdr->objectCount )
		{
			assert( pObj->phase == eALIVE );  // static objects have to be Alive
			if ( objMask.Has( pObj->type ) )
			{
				CTrafLightData elem;
				elem.cvedId          = i;
				elem.sqrdDistFromLoc = -1.0;

				m_tlData.push_back( elem );
			}

			i++;
			pObj++;
		}

		m_FirstTimeLightsNear = false;
	}

	//
	// For each trafficlight, calculate its squared distance to the given
	// location.
	//
	CPoint3D tlPos;
	double sqrdDist;
	vector<CTrafLightData>::iterator itr;
	for( itr = m_tlData.begin(); itr != m_tlData.end(); itr++ )
	{
		tlPos = GetObjPos( itr->cvedId );

		// calculate the distance between given loc and tl loc
		sqrdDist = (
			( loc.m_x - tlPos.m_x ) * ( loc.m_x - tlPos.m_x ) +
			( loc.m_y - tlPos.m_y ) * ( loc.m_y - tlPos.m_y )
			);

		itr->sqrdDistFromLoc = sqrdDist;
	}

	//
	// Sort the traffic lights according to the their distance
	// from the given location.
	//
	sort( m_tlData.begin(), m_tlData.end() );

	//
	// Fill the output array with the maxSize closest traffic lights.
	//
	int outSize = 0;
	for( itr = m_tlData.begin(); itr != m_tlData.end(); itr++ )
	{
		if( outSize < maxSize )
		{
			pCvedId[ outSize ] = itr->cvedId;
			outSize++;
		}
		else
		{
			break;
		}
	}
#ifdef _WIN32
	ReleaseMutex(m_MUTEX_LightsNear);
#endif
	return outSize;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all objects whose type is included
//  in the specified mask and are located near the specified point.
//
// Remarks: The output is given in a set of integers that are the
//  identifiers of all applicable objects.  The appropriate constructor
//  or conversion operators can be used to obtain the appropriately typed
//  object class.
//
//	To be included in the output, the bounding box of an object should overlap
//	a circle whose center is at loc and has the given radius.
//
// Arguments:
//  cLoc - the location to search for objects
//  radius - how far around loc to search for objects
//	out - the STL vector to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetObjsNear(
			const CPoint3D& cLoc,
			double          radius,
			TIntVec&        out,
			CObjTypeMask    mask
			) const
{
	int       i;
	TObj*     pO;
	CPoint3D  objPos;
	double    radiusSquare = radius * radius;  // avoid square roots

	out.clear();
	pO = BindObj(0);
	CBoundingBox bbox(
					cLoc.m_x - radius,
					cLoc.m_y - radius,
					cLoc.m_x + radius,
					cLoc.m_y + radius);
	// for dynamic object
	i  = 0;
	for(; i < cNUM_DYN_OBJS; i++){
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type, i) ) {
				objPos = GetObjPos(i);
				if ( bbox.Encloses(objPos) ) {
					out.push_back(i);
				}
			}
		}
		pO++;
	}

	// for static objects in the LRI
	vector<int> soVec;
	m_staticObjQTree.SearchRectangle(bbox, soVec);
	if (soVec.size() != 0){
		vector<int>::const_iterator soIter = soVec.begin();
		for (; soIter != soVec.end(); soIter++){
			if (mask.Has(GetObjType(*soIter))){
				objPos = GetObjPos(*soIter);
				if (bbox.Encloses(objPos)){
					out.push_back(*soIter);
				}
			}
		}
	}

	// static objects created at runtime
	TU32b soIdx = m_pHdr->objectCountInitial;
	for	(; soIdx < m_pHdr->objectCount; soIdx++){
		if (mask.Has(GetObjType(soIdx))){
			objPos = GetObjPos(soIdx);
			if (bbox.Encloses(objPos)){
				out.push_back(soIdx);
			}
		}
	}


#if 0
	while ( i < m_pHdr->objectCount ) {
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type) ) {
				double distSquare;

				objPos = GetObjPos(i);
#endif
#if 0
				double x = fabs(objPos.m_x - cLoc.m_x);
				double y = fabs(objPos.m_y - cLoc.m_y);
				if ((x > radius) || (y > radius))
					continue;
#endif
#if 0
				distSquare = (objPos.m_x - cLoc.m_x) * (objPos.m_x - cLoc.m_x)
						+ (objPos.m_y - cLoc.m_y) * (objPos.m_y - cLoc.m_y);

				if ( distSquare <= radiusSquare ) {
					out.push_back(i);
				}
#endif
#if 0
				double diff = radiusSquare - (x*x);
				if (diff < 0){
					continue;
				}
				diff -= (y*y);
				if (diff >= 0){
					out.push_back(i);
				}
#endif
#if 0
			}
		}
		i++;
		pO++;
	}
#endif
} // GetObjsNear


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function looks through all available objects until it finds one whose
// 	name matches the parameter.
//
// Remarks:
//
// Arguments:
//  cObjName - the name to look for
//	objId - the resulting object identifier
//
// Returns:  The function returns true if it finds an object or false
// 	otherwise.  When the function returns true, ObjId will contain the
// 	identifier of the object.  If the function returns false, objId will be
// 	set to -1.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetObj(const string &cObjName, int &objId) const
{
	TObj  *pO;

	// linear search was too slow, so now we search through the multimap and
	// return the first object with the given name.  In the future, this may
	// be expanded (or another method added) to return all objects with the
	// given name.
	objId = -1;
	multimap<string,int>::const_iterator lb = m_objNameToId.lower_bound( cObjName );
	multimap<string,int>::const_iterator ub = m_objNameToId.upper_bound( cObjName );
	for ( ; lb != ub; lb++ ) {
		pO = BindObj( lb->second );
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			objId = pO->myId;
			return true;
		}
	}

	// Haven't found it yet.  Try a linear search on dynamic objects, because
	// attached cved's won't have the multimap initialized.

	return GetObjLinear( cObjName, objId );
} // end of GetObj


bool
CCved::GetObj(const string &cObjName, vector<int> &objId) const
{
	TObj  *pO;
#ifdef _DEBUG
	TObj tObj;
#endif
	// Search through the multimap and return all objects with the given name.
	// Will not work when the shared memory interface is used because
	// the multimap does not get updated (???)
	objId.clear();
	multimap<string,int>::const_iterator lb = m_objNameToId.lower_bound( cObjName );
	multimap<string,int>::const_iterator ub = m_objNameToId.upper_bound( cObjName );
	for ( ; lb != ub; lb++ ) {
		pO = BindObj( lb->second );
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			objId.push_back(pO->myId);
		}
	}

	return objId.size() > 0;
} // end of GetObj


//////////////////////////////////////////////////////////////////////////////
//
// Description:
//  This function looks through all available objects until it finds one whose
//  name matches the parameter.
//
// Remarks:
//
// Arguments:
//  cObjName - the name to look for
//  objId - the resulting object identifier
//
// Returns:  The function returns true if it finds an object or false
//  otherwise.  When the function returns true, ObjId will contain the
//  identifier of the object.  If the function returns false, objId will be
//  set to -1.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetObjLinear(const string &cObjName, int &objId) const
{
    int   i;
    TObj  *pO;


    // For now, we perform a simple linear search.
    // If this function becomes a bottleneck, we will have to
    // manually create a hash table that can be stored in the
    // memory block.  Keep in mind that a regular STL container
    // won't work since it won't be shared by all CVED instances.
    objId = -1;
    i     = 0;
    pO    = BindObj( i );
    while ( i < cNUM_DYN_OBJS ) {
        if ( pO->phase == eALIVE || pO->phase == eDYING ) {
            if ( cObjName == pO->name ) {
                objId = i;
                return true;
            }
        }
        i++;
        pO++;
    }

    return false;
} // end of GetObj



//////////////////////////////////////////////////////////////////////////////
//
// Description:  Figures out if the given object id belongs to an explicit
//  static object.
//
// Remarks:
//
// Arguments:
//  objId - the object identifier
//
// Returns: a boolean true if the object was created during runtime, or false
// 	if the object is a static object declared in the LRI file.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsExplicitObj( int objId ) const
{
	bool isExplicitObj = (
				objId >= (int)m_pHdr->objectCountInitial &&
				objId < (int)m_pHdr->objectStrgCount
				);
	return isExplicitObj;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function returns true if the object is a dynamic object.
//
// Remarks:
//
// Arguments:
//  objId - the object identifier
//
// Returns: a boolean true if the object is in the dynamic object pool, or
//	false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsDynObj( int objId ) const
{
	bool isDynObj = ( objId >= 0 ) && ( objId < cNUM_DYN_OBJS );
	return isDynObj;
} // end of IsDynObj

//////////////////////////////////////////////////////////////////////////////
/// Description:
/// 	Places the IDs of all the dynamic objects on the given road and lanes and
/// 	of the given types into the integer vector.
///
/// Remarks: The results are sorted by distance.
///  This function accesses the data stored in the dynamic object referece
///	lists associated with each road reference.  This data is refreshed
///	during the Maintainer() function, and so the results of this query
///	are only accurate up to the last Maintainer() call.
///
/// Arguments:
///	roadId - index of the road which contains the objects of interest.
///	lanes - bitset of lane IDs which the objects may occupy
///	result - integer vector result of object IDs
///	cMask - (optional) object types allowed in result.  Default is all types.
///
/// Returns: void
///
///\todo This function does not clear out the result set first it needs to
///
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnRoad(
			int roadId,
			const bitset<cCV_MAX_LANES> &lanes,
			TIntVec& result,
			const CObjTypeMask cMask
			) const
{
	if (m_pRoadRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pRoadRefPool[roadId].objIdx;
	cvEObjType type;


#ifdef DEBUG_GET_ALL_DYNOBJS_ON_ROAD
	if( curIdx == 0 )
	{
		gout << " roadId = " << roadId << endl;
		gout << " no obj on this road " << endl;
	}
#endif

	while (curIdx != 0) {

		int objId = m_pDynObjRefPool[curIdx].objId;
		bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);

		int laneId;
		for( laneId = 0; laneId < cCV_MAX_LANES; laneId++ )
		{
			if( objLanes[laneId] )
			{

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_ROAD
				gout << " current obj " << objId  <<  " is on lane " << laneId << endl;
#endif

			}
		}

		objLanes &= lanes;

		// If the input lanes overlap with the lanes the object is on,
		//	Or if both the input and object lanes are 0, then
		//	the object is on the road and lane.
		if ((objLanes.any()) ||
		    ((objLanes.to_ulong() == 0) &&
			 (lanes.to_ulong() == 0))) {

			type = GetObjType(m_pDynObjRefPool[curIdx].objId);
			if (cMask.Has(type, m_pDynObjRefPool[curIdx].objId))
				result.push_back(m_pDynObjRefPool[curIdx].objId);
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}



} // end of GetAllDynObjsOnRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on a given lane.
//
// Remarks:This function access the data stored in the dynamic object
//	referece lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	lane - A lane of a road.
//	result - vector of object ids with their distances.
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnLane (CLane lane,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const {

	result.clear();
	CRoad road = lane.GetRoad();
	int roadId = road.GetId();
	int curIdx = m_pRoadRefPool[roadId].objIdx;

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
	gout << " =========GetAllDynObjsOnLane==============" << endl;
	gout << " road name = " << road.GetName();
	gout << " lane id = " << lane.GetId() << endl;
#endif

	while (curIdx != 0)
	{
		int objId = m_pDynObjRefPool[curIdx].objId;
		cvEObjType objType = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = m.Has( objType, m_pDynObjRefPool[curIdx].objId );
		double objDist = m_pDynObjRefPool[curIdx].distance;

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
		gout << " current obj in the pool = " << objId << endl;
		gout << " value of inObjMask = " << inObjMask << endl;
		gout << " objDist = " << objDist << endl;
#endif


#if 1
		bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);

		int laneId;
		for( laneId = 0; laneId < cCV_MAX_LANES; laneId++ )
		{
			if( objLanes[laneId] )
			{

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
				gout << " current obj is on lane " << laneId << endl;
#endif

			}
		}


#endif

		if( inObjMask )
		{

			bitset<cCV_MAX_LANES> laneMask;
			laneMask.set( lane.GetRelativeId() );

			// Make sure the object is on the lane.
			objLanes &= laneMask;
			bool objOnNeededLane = (
									objLanes.any() ||
									objLanes.to_ulong() == 0 &&
									laneMask.to_ulong() == 0
									   );

			if( objOnNeededLane )
			{
				TObjWithDist node;
				node.objId = objId;
				node.dist = objDist;
				result.push_back( node );

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
				gout << " ******The current obj inserted is: " << objId << endl;
#endif

			}

		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}

}	// end of GetAllDynObjsOnLane
//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given road and
// 	lanes, between the given distances, and of the given types into the
// 	integer vector.
//
// Remarks: The results are sorted by distance.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	roadId - index of the road which contains the objects of interest.
//	lanes - bitset of lane IDs which the objects may occupy
//	startDist - starting distance along the road to search
//	endDist - end distance along the road to search
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								const CObjTypeMask cMask) const {

#ifdef _DEBUG
	cvTDynObjRef temp; //for the debugger........
    cvTRoadRef tempf;
#endif
    if (m_pRoadRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}
	int curIdx = m_pRoadRefPool[roadId].objIdx;
	cvEObjType type;
	while (curIdx != 0) {
		if ((m_pDynObjRefPool[curIdx].distance >= startDist) &&
			(m_pDynObjRefPool[curIdx].distance <= endDist)) {
			bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
			objLanes &= lanes;

			// If the input lanes overlap with the lanes the object is on,
			//	Or if both the input and object lanes are 0, then
			//	the object is on the road and lane.
			if ((objLanes.any()) ||
				((objLanes.to_ulong() == 0) &&
				 (lanes.to_ulong() == 0))) {

				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (cMask.Has(type, m_pDynObjRefPool[curIdx].objId))

				//	gout << " *** checking inserted obj id = ";
				//	gout << m_pDynObjRefPool[curIdx].objId << endl;
				//	gout << " *** checking inserted obj dist = ";
				//	gout << m_pDynObjRefPool[curIdx].distance << endl;

					result.push_back(m_pDynObjRefPool[curIdx].objId);
			}
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnRoadRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given road and
// 	lanes, between the given distances, and of the given types into the
// 	integer vector.  Also returns a bitset set with the ids of the corridors
// 	that overlap objects.
//
// Remarks: The results are sorted by distance.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	roadId - index of the road which contains the objects of interest.
//	lanes - bitset of lane IDs which the objects may occupy
//	startDist - starting distance along the road to search
//	endDist - end distance along the road to search
//	result - integer vector result of object IDs
//	usedLanes - Will contain the ids of the lanes that overlap objects
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								bitset<cCV_MAX_LANES>& usedLanes,
								const CObjTypeMask cMask) const
{
	if (m_pRoadRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pRoadRefPool[roadId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		if ((m_pDynObjRefPool[curIdx].distance >= startDist) &&
			(m_pDynObjRefPool[curIdx].distance <= endDist)) {

			usedLanes = (int) m_pDynObjRefPool[curIdx].lanes;
			usedLanes &= lanes;

			// If the input lanes overlap with the lanes the object is on,
			//	Or if both the input and object lanes are 0, then
			//	the object is on the road and lane.
			if ((usedLanes.any()) ||
				( (usedLanes.to_ulong() == 0) &&
				  (lanes.to_ulong() == 0) ) ) {

				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (cMask.Has(type, m_pDynObjRefPool[curIdx].objId))
					result.push_back(m_pDynObjRefPool[curIdx].objId);
			}
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnRoadRange


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridors and of the given types into the integer vector.
//
// Remarks: The results are sorted by distance.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrs - bitset of corridor IDs which the objects may occupy
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnIntrsctn (int intrsctnId,
								const bitset<cCV_MAX_CRDRS> &crdrs,
								TIntVec& result,
								const CObjTypeMask m) const
{

#ifdef DEBUG_GET_ALL_DYNOBJS
	gout << "=========GetAllDynObjsOnIntrsctn=============" << endl;
#endif

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {
		int objId = m_pDynObjRefPool[curIdx].objId;

#ifdef DEBUG_GET_ALL_DYNOBJS
		gout << "current objid = " << objId << endl;
#endif

		bitset<cCV_MAX_CRDRS> objCrdrs((int)(int)m_pDynObjRefPool[curIdx].corridors);
		int crdrId;
		for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
		{
			if( objCrdrs[crdrId] )
			{

#ifdef DEBUG_GET_ALL_DYNOBJS
				gout << " current obj is on crdr " << crdrId << endl;
#endif

			}
		}



		objCrdrs &= crdrs;

		// If the input crdrs overlap with the crdrs the object is on,
		//	Or if both the input and object crdrs are 0, then
		//	the object is on the intrsctn and crdrs.
		if ((objCrdrs.any()) ||
		    ((objCrdrs.to_ulong() == 0) &&
			 (crdrs.to_ulong() == 0))) {

				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (m.Has(type, m_pDynObjRefPool[curIdx].objId))
				{
					// If an object appears more than once in the internal list,
					// only insert it once.
			//		TIntVec::iterator i= find(
			//								result.begin(),
			//								result.end(),
			//								m_pDynObjRefPool[curIdx].objId
			//								);
			//		if( i == result.end() )
			//		{
						result.push_back(m_pDynObjRefPool[curIdx].objId);
			//		}
				}
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnIntrsctn


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridors and of the given types into the integer vector.
//
// Remarks: The results are sorted by distance along each corridor.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrs - bitset of corridor IDs which the objects may occupy
//	startDist - array of starting distances along the crdr to search
//	endDist - array of end distances along the crdr to search
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnIntrsctnRange (
			int intrsctnId,
			const bitset<cCV_MAX_CRDRS> &crdrs,
			const double startDist[cCV_MAX_CRDRS],
			const double endDist[cCV_MAX_CRDRS],
			TIntVec& result,
			const CObjTypeMask m
			) const
{

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	vector<TObjWithDist> tempVec;
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
		objCrdrs &= crdrs;

		// If the input crdrs overlap with the crdrs the object is on,
		//	Or if both the input and object crdrs are 0, then
		//	the object is on the intrsctn and crdrs.
		if ((objCrdrs.any()) ||
		    ((objCrdrs.to_ulong() == 0) &&
			 (crdrs.to_ulong() == 0))) {

			// For each corridor in the mask
			for (int crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++) {

				// If the object lies on the current corridor
				if (objCrdrs[crdrId]) {

					// If object lies within the distance range
					if ((m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
							>= startDist[crdrId]) &&
						(m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
						 	<= endDist[crdrId])) {

						// If the object's type is in the mask
						type = GetObjType(m_pDynObjRefPool[curIdx].objId);
						if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {
							// Add the object id to the list

						//	gout << " *** checking inserted obj id = ";
						//	gout << m_pDynObjRefPool[curIdx].objId << endl;
						//	gout << " *** checking inserted obj dist = ";
						//	gout << m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
						//	gout << endl;

							TObjWithDist node;
							node.objId = m_pDynObjRefPool[curIdx].objId;
							node.dist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
							tempVec.push_back(node);

						//	result.push_back(m_pDynObjRefPool[curIdx].objId);

							// Don't want to insert the same object twice, so
							// 	break out of this for-loop
							crdrId = cCV_MAX_CRDRS;
						}
					} // If object lies within the distance range
				} // If the object lies on the current corridor
			} // For each corridor in the mask
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}

	// sort the objs
	TObjWithDist hold;
	TU32b k, loc;
	for( k =1; k < tempVec.size(); k++ )
	{
		hold = tempVec[k];
		loc = k;
		while( 0 < loc && ( hold.dist < tempVec[loc-1].dist ) )
		{
			tempVec[loc] = tempVec[loc-1];
			loc--;
		}
		tempVec[loc] = hold;
	}

	vector<TObjWithDist>::iterator i;
	for( i = tempVec.begin(); i != tempVec.end(); i++ )
	{
		result.push_back( (*i).objId );

	}

} // end of GetAllDynObjsOnIntrsctnRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridors and of the given types into the integer vector.  Also returns
// 	a bitset containing the ids of the corridors with objects on them.
//
// Remarks: The results are sorted by distance along each corridor.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrs - bitset of corridor IDs which the objects may occupy
//	startDist - array of starting distances for the corridors in crdr
//	endDist - array of ending distances for the corridors in crdr
//	result - integer vector result of object IDs
//	crdrsUsed - bitset containing the corridors with objects on them
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnIntrsctnRange (
			int intrsctnId,
			const bitset<cCV_MAX_CRDRS> &crdrs,
			const double startDist[cCV_MAX_CRDRS],
			const double endDist[cCV_MAX_CRDRS],
			TIntVec& result,
			bitset<cCV_MAX_CRDRS>& usedCrdrs,
			const CObjTypeMask m
			) const
{

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
		objCrdrs &= crdrs;

		// If the input crdrs overlap with the crdrs the object is on,
		//	Or if both the input and object crdrs are 0, then
		//	the object is on the intrsctn and crdrs.
		if ((objCrdrs.any()) ||
		    ((objCrdrs.to_ulong() == 0) &&
			 (crdrs.to_ulong() == 0))) {

			// For each corridor in the mask
			for (int crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++) {

				// If the object lies on the current corridor
				if (objCrdrs[crdrId]) {

					// If object lies within the distance range
					if ((m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
							>= startDist[crdrId]) &&
						(m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
						 	<= endDist[crdrId])) {

						// If the object's type is in the mask
						type = GetObjType(m_pDynObjRefPool[curIdx].objId);
						if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {
							// Add the object id to the list
							result.push_back(m_pDynObjRefPool[curIdx].objId);

							// Don't want to insert the same object twice, so
							// 	break out of this for-loop
							crdrId = cCV_MAX_CRDRS;
						}
					} // If object lies within the distance range
				} // If the object lies on the current corridor
			} // For each corridor in the mask
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnIntrsctnRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridor and of the given types into the output vector.
//
// Remarks:This function access the data stored in the dynamic object
//	referece lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrId - index of the corridor, with respect to the intersection.
//	result - vector of object ids with their distances.
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const
{

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	if( curIdx == 0 )
	{
#ifdef DEBUG_GET_ALL_DYNOBJS_ON_CRDR
		gout << " no obj in pool " << endl;
#endif
	}

	while (curIdx != 0) {

		int objId = m_pDynObjRefPool[curIdx].objId;
		double objDist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_CRDR
		gout << " current obj in pool = " << objId << endl;
		gout << " obj dist = " << objDist << endl;
#endif

		// If the object lies on the current corridor
		if (objCrdrs[crdrId]) {

			// If the object's type is in the mask
			type = GetObjType(m_pDynObjRefPool[curIdx].objId);
			if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {

			// Add the object id to the list
			TObjWithDist node;
			node.objId = objId;
			node.dist = objDist;
			result.push_back( node );
			}

		} // If the object lies on the current corridor

		curIdx = m_pDynObjRefPool[curIdx].next;
	}	// while

} // end of GetAllDynObjsOnCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridor and of the given types into the integer vector.
//
// Remarks: The results are sorted by distance along the corridor.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrId - relative id of the corridor, with respect to the intersection.
//	startDist - starting distance along the crdr to search
//	endDist - end distance along the crdr to search
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllDynObjsOnCrdrRange (int intrsctnId,
								 int crdrId,
								 double startDist,
								 double endDist,
								 TIntVec& result,
								 const CObjTypeMask m) const {
	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	vector<TObjWithDist> tempVec;
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);

		// If the object lies on the current corridor
		if (objCrdrs[crdrId]) {

			// If object lies within the distance range
			if ((m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
					>= startDist) &&
				(m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
				 	<= endDist)) {

				// If the object's type is in the mask
				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {
					// Add the object id to the list

				//	gout << " *** checking inserted obj id = ";
				//	gout << m_pDynObjRefPool[curIdx].objId << endl;
				//	gout << " *** checking inserted obj dist = ";
				//	gout << m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
				//	gout << endl;

				//	result.push_back(m_pDynObjRefPool[curIdx].objId);

					TObjWithDist node;
					node.objId = m_pDynObjRefPool[curIdx].objId;
					node.dist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];

					tempVec.push_back(node);

					// Don't want to insert the same object twice, so
					// 	break out of this for-loop
				//	 crdrId = cCV_MAX_CRDRS;	// comment in (1-15-04)
				}
			} // If the object lies within the distance range
		} // If the object lies on the current corridor

		curIdx = m_pDynObjRefPool[curIdx].next;
	}

	// sort the objs
	TObjWithDist hold;
	TU32b k, loc;
	for( k =1; k < tempVec.size(); k++ )
	{
		hold = tempVec[k];
		loc = k;
		while( 0 < loc && ( hold.dist < tempVec[loc-1].dist ) )
		{
			tempVec[loc] = tempVec[loc-1];
			loc--;
		}
		tempVec[loc] = hold;
	}

	vector<TObjWithDist>::iterator i;
	for( i = tempVec.begin(); i != tempVec.end(); i++ )
	{
		result.push_back( (*i).objId );

	}

} // end of GetAllDynObjsOnIntrsctnRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function creates a dynamic object and returns a pointer to a newly
// 	allocated CDynObj instance.  The attributes of the object will be set
// 	according to the argument attr.
//
// Remarks: The new object will not appear in the various interrogation
// 	functions until after the next execution of the database maintainer,
// 	however, it is possible to set the state of the object using object
// 	functions.
//
//	The actual type of the class instance pointed by the return pointer will
//	match the derived class of CDynObj that corresponds to type.  In systems
// 	that support run time type identification, the pointer can be safely
// 	casted to the appropriate derived type but keep in mind that unpredictable
// 	results will occur if the pointer is down casted to the wrong derived
// 	class.
//
//  For example:
//            CDynObj  *pObj;
//            pObj = cv.CreateDynObj("Bike", TBicycle, attr);
//            CBicycleObj *pBike;
//            pBike = dynamic_cast<CBicycleObj *>(pObj);
//            // now can access Bicycle specific functions using 'pBike'
//
//	If no initial conditions are provided in the argument list, the object is
//	initialized at the origin, facing along the x-axis, otherwise it is
//	initialized as per the arguments.  Note that if a tangent and/or lateral
//	vectors are specified, they will be normalized by this function before
//	they are used.
//
// 	If the specified type is TExternalDriver, the function will create an
// 	object with the special property of representing the driver of the
// 	ownship. There is a limited number of objects that can be created using
// 	this type, and they have the special property that CVED does not manage
// 	their movement.  Specifically, it is up to the user of CVED to specify
// 	the state of this object using the standard access functions.  The solId
// 	field of the attributes can be used to differentiate between external
// 	driver and trailers that may be towed by the driver.
//
//	The function returns a pointer to a dynamically allocated class instance.
//	The allocated memory should be freed when not in use but keep in mind that
//	deleting the class instance does not remove the object from the
//	environment.  Use the DeleteDynObj function for that purpose, followed
// 	by a call to the C++ delete operator for freeing the memory allocated for
// 	the new class.
//
// Arguments:
//  cName - the name to associate with the object
//	type - the type of the object
//  cAttr - the attributes to be used for the new object
//  cpInitPos - ptr to a point providing the initial position
//  cpInitTan - ptr to a vector indicating the initial tangent of
//    the object's orientation
//  cpInitLat - ptr to a vector indicating the initial lateral direction of
//    the object's orientation
//
// Returns: The function returns a pointer to a dynamically allocated class.
// 	It will return 0 in case of an error.  Potential errors include overflow
// 	of the internal dynamic object list, a duplicate name (i.e., the provided
// 	name already is in use), or a type that is invalid.
//
//////////////////////////////////////////////////////////////////////////////
CDynObj*
CCved::CreateDynObj(
				const string&		cName,
				cvEObjType			type,
				int					hcsmType,
				const cvTObjAttr&	cAttr,
				const CPoint3D*		cpInitPos,
				const CVector3D*	cpInitTan,
				const CVector3D*	cpInitLat)
{
	CDynObj* pObj = CreateDynObj( cName, type, cAttr, cpInitPos, cpInitTan, cpInitLat );
	if (pObj==0){
		MessageBox(NULL,"Error Creating Object","Error",MB_ICONERROR);
		exit(-1);
	}
	pObj->m_pObj->hcsmType = hcsmType; //check to see if we got a good pointer
	return pObj;
}

CExternalDriverObj *
CCved::CreatePeerExternalObj(
	const string&     cName,
	const cvTObjAttr& cAttr,
	const CPoint3D*   cpInitPos,
	const CVector3D*  cpInitTan,
	const CVector3D*  cpInitLat
	)
{
	// Search for an empty slot; We use the lastObjAlloc field of
	// the header to remember which was the last allocated id.  That
	// way, we keep advancing identifiers so we don't return the
	// same id as soon as they are freed.
	// In searching for an empty slot, we ignore identifiers less
	// than cMAX_EXT_CNTRL_OBJS, which are reserved for the own
	// driver (and any other externallyd driven objects).
	// Also, we have to lock the whole header to ensure that we don't
	// end up competing for the same slot from a concurrently
	// running process that also is trying to create an object
	//
	// When creating an "ownship" driver, we simply allocate a slot
	// between 0 and cMAX_EXT_CNTRL_OBJS.  We don't expect frequent
	// allocation/deallocation so we don't keep track of which id
	// was allocated
	int    objId;
	TObj  *pO;

	if ( m_debug > 0 ) {
		gout << "->Enter CCved::CreatePeerExternalObj("
			<< cName << ", )" << endl;
		gout << "header addr is " << (int) m_pHdr << endl << flush;
	}

	LockObjectPool();

	objId = 1;
	pO    = BindObj(objId);
	while ( pO->phase != eDEAD && objId != cMAX_EXT_CNTRL_OBJS )  {
		objId++;
		pO = BindObj(objId);
	}
	if ( pO->phase != eDEAD ) {
		if ( m_debug ) {
			gout << "  no slot for externally driven obj" << endl;
		}
		return 0;
	}
	pO->phase = eBORN;

	UnlockObjectPool();

	if( m_debug > 1 )
	{
		gout << "  Object id is " << objId << endl;
	}

	//
	// insert information into the slot
	//
	pO->myId = objId;
	pO->type = eCV_EXTERNAL_DRIVER;
	strncpy_s(pO->name, cName.c_str(), cOBJ_NAME_LEN-1);

	pair<string, int> pairToInsert;
	pairToInsert.first = cName;
	pairToInsert.second = objId;
	m_objNameToId.insert( pairToInsert );

	// set up the cAttributes; we simply copy whatever
	// the user provided into the allocated slot.
	// Dynamic objects can't have multiple cAttributes
	// so the cAttribute slot is the same the object id.
	pO->attr = cAttr;

	cvTObjStateBuf initVals = { 0 }; // make everything 0 to start with

	if( cpInitPos )
	{
		initVals.state.anyState.position.x = cpInitPos->m_x;
		initVals.state.anyState.position.y = cpInitPos->m_y;
		initVals.state.anyState.position.z = cpInitPos->m_z;
	}

	// If the user specified tangent or lateral = (0,0,0),
	//	then the tangent/lateral must be set to the proper
	//	default value.  This is to insure that the object's
	//	bounding box never has a length/width = 0.  Also,
	//	it's important to insure that the tangent/lateral
	//	vectors are normalized.
	if( cpInitTan )
	{
		CVector3D tang( *cpInitTan );

		if( tang.Normalize() != 0.0 )
		{
			initVals.state.anyState.tangent.i = tang.m_i;
			initVals.state.anyState.tangent.j = tang.m_j;
			initVals.state.anyState.tangent.k = tang.m_k;
		}
		else
		{
			initVals.state.anyState.tangent.i = 1.0;
			initVals.state.anyState.tangent.j = 0.0f;
			initVals.state.anyState.tangent.k = 0.0f;
		}
	}
	else
	{
		initVals.state.anyState.tangent.i = 1.0f;
		initVals.state.anyState.tangent.j = 0.0f;
		initVals.state.anyState.tangent.k = 0.0f;
	}

	if( cpInitLat )
	{
		CVector3D lat( *cpInitLat );

		if( lat.Normalize() != 0.0 )
		{
			initVals.state.anyState.lateral.i = lat.m_i;
			initVals.state.anyState.lateral.j = lat.m_j;
			initVals.state.anyState.lateral.k = lat.m_k;
		}
		else
		{
			initVals.state.anyState.lateral.i = 0.0f;
			initVals.state.anyState.lateral.j = -1.0f;
			initVals.state.anyState.lateral.k = 0.0f;
		}
	}
	else
	{
		initVals.state.anyState.lateral.i = 0.0f;
		initVals.state.anyState.lateral.j = -1.0f;
		initVals.state.anyState.lateral.k = 0.0f;
	}

	// Initialize the bounding box, based on the
	//	tangent, lateral, and position calculated
	//	above.
	CPoint3D ll, ur;
	CCved::UpdateBBox(
				&pO->attr,
				CVector3D( initVals.state.anyState.tangent ),
				CVector3D( initVals.state.anyState.lateral ),
				CPoint3D( initVals.state.anyState.position ),
				ll,
				ur
				);
	initVals.state.anyState.boundBox[0].x = ll.m_x;
	initVals.state.anyState.boundBox[0].y = ll.m_y;
	initVals.state.anyState.boundBox[1].x = ur.m_x;
	initVals.state.anyState.boundBox[1].y = ur.m_y;

	pO->stateBufA = initVals;
	pO->stateBufB = initVals;

	// select the appropriate derived class (based on the type)
	// and create the object in the specified slot
	CDynObj* pTheObj = CreateTypedObject( eCV_EXTERNAL_DRIVER, objId );

	m_pHdr->dynObjectCount++;

	if( m_debug > 0 )
	{
		gout << "->Exit CCved::CreateDynObj, ptr is"  << (int)pTheObj << endl;
	}

	return static_cast<CExternalDriverObj*>(pTheObj);
}
CDynObj *
CCved::CreateDynObj(
			const string&     cName,
			cvEObjType        type,
			const cvTObjAttr& cAttr,
			const CPoint3D*   cpInitPos,
			const CVector3D*  cpInitTan,
			const CVector3D*  cpInitLat
			)
{
	// Search for an empty slot; We use the lastObjAlloc field of
	// the header to remember which was the last allocated id.  That
	// way, we keep advancing identifiers so we don't return the
	// same id as soon as they are freed.
	// In searching for an empty slot, we ignore identifiers less
	// than cMAX_EXT_CNTRL_OBJS, which are reserved for the own
 	// driver (and any other externallyd driven objects).
	// Also, we have to lock the whole header to ensure that we don't
	// end up competing for the same slot from a concurrently
	// running process that also is trying to create an object
	//
	// When creating an "ownship" driver, we simply allocate a slot
	// between 0 and cMAX_EXT_CNTRL_OBJS.  We don't expect frequent
	// allocation/deallocation so we don't keep track of which id
	// was allocated
	int    objId;
	TObj  *pO;

	if ( m_debug > 0 ) {
		gout << "->Enter CCved::CreateDynObj("
			<< cName << ", " << type << ")" << endl;
		gout << "header addr is " << (int) m_pHdr << endl << flush;
	}

	LockObjectPool();
	if ( type == eCV_EXTERNAL_DRIVER ) {
		objId = 0;
		pO    = BindObj(objId);

		while ( pO->phase != eDEAD && objId != cMAX_EXT_CNTRL_OBJS )  {
			objId++;
			pO = BindObj(objId);
		}

		if ( pO->phase != eDEAD ) {
			if ( m_debug ) {
				gout << "  no slot for externally driven obj" << endl;
			}
			return 0;
		}
		pO->phase = eBORN;
	}
	else if ( type == eCV_EXTERNAL_TRAILER ) {
		objId = 1;
		pO    = BindObj(objId);

		while ( pO->phase != eDEAD && objId != cMAX_EXT_CNTRL_OBJS )  {
			objId++;
			pO = BindObj(objId);
		}

		if ( pO->phase != eDEAD ) {
			if ( m_debug ) {
				gout << "  no slot for external trailer obj" << endl;
			}
			return 0;
		}
		pO->phase = eBORN;
	}
	else {
		objId = m_pHdr->lastObjAlloc;
		do {
			objId++;
			if ( objId < cMAX_EXT_CNTRL_OBJS )  {
				objId = cMAX_EXT_CNTRL_OBJS;
			}
			else
			if ( objId == cNUM_DYN_OBJS )  {
				objId = cMAX_EXT_CNTRL_OBJS;
			}

			pO = BindObj(objId);
		} while ( pO->phase != eDEAD && objId != m_pHdr->lastObjAlloc );

		if ( m_debug > 2 ) {
			gout << "found one : " << objId << endl << flush;
		}
		if ( pO->phase != eDEAD ) {
			// !! dynamic object overflow !!
			return 0;
		}
		m_pHdr->lastObjAlloc = objId;
		pO->phase            = eBORN;
	}

	UnlockObjectPool();

	if( m_debug > 1 )
	{
		gout << "  Object id is " << objId << endl;
	}

	//
	// insert information into the slot
	//
	pO->myId = objId;
	pO->type = type;
	strncpy_s(pO->name, cName.c_str(), cOBJ_NAME_LEN-1);

	pair<string, int> pairToInsert;
	pairToInsert.first = cName;
	pairToInsert.second = objId;
	m_objNameToId.insert( pairToInsert );

	// set up the cAttributes; we simply copy whatever
	// the user provided into the allocated slot.
	// Dynamic objects can't have multiple cAttributes
	// so the cAttribute slot is the same the object id.
	pO->attr = cAttr;

	cvTObjStateBuf initVals = { 0 }; // make everything 0 to start with

	if( cpInitPos )
	{
		initVals.state.anyState.position.x = cpInitPos->m_x;
		initVals.state.anyState.position.y = cpInitPos->m_y;
		initVals.state.anyState.position.z = cpInitPos->m_z;
	}

	// If the user specified tangent or lateral = (0,0,0),
	//	then the tangent/lateral must be set to the proper
	//	default value.  This is to insure that the object's
	//	bounding box never has a length/width = 0.  Also,
	//	it's important to insure that the tangent/lateral
	//	vectors are normalized.
	if( cpInitTan )
	{
		CVector3D tang( *cpInitTan );

		if( tang.Normalize() != 0.0 )
		{
			initVals.state.anyState.tangent.i = tang.m_i;
			initVals.state.anyState.tangent.j = tang.m_j;
			initVals.state.anyState.tangent.k = tang.m_k;
		}
		else
		{
			initVals.state.anyState.tangent.i = 1.0;
			initVals.state.anyState.tangent.j = 0.0f;
			initVals.state.anyState.tangent.k = 0.0f;
		}
	}
	else
	{
		initVals.state.anyState.tangent.i = 1.0f;
		initVals.state.anyState.tangent.j = 0.0f;
		initVals.state.anyState.tangent.k = 0.0f;
	}

	if( cpInitLat )
	{
		CVector3D lat( *cpInitLat );

		if( lat.Normalize() != 0.0 )
		{
			initVals.state.anyState.lateral.i = lat.m_i;
			initVals.state.anyState.lateral.j = lat.m_j;
			initVals.state.anyState.lateral.k = lat.m_k;
		}
		else
		{
			initVals.state.anyState.lateral.i = 0.0f;
			initVals.state.anyState.lateral.j = -1.0f;
			initVals.state.anyState.lateral.k = 0.0f;
		}
	}
	else
	{
		initVals.state.anyState.lateral.i = 0.0f;
		initVals.state.anyState.lateral.j = -1.0f;
		initVals.state.anyState.lateral.k = 0.0f;
	}

	// Initialize the bounding box, based on the
	//	tangent, lateral, and position calculated
	//	above.
	CPoint3D ll, ur;
	CCved::UpdateBBox(
				&pO->attr,
				CVector3D( initVals.state.anyState.tangent ),
				CVector3D( initVals.state.anyState.lateral ),
				CPoint3D( initVals.state.anyState.position ),
				ll,
				ur
				);
	initVals.state.anyState.boundBox[0].x = ll.m_x;
	initVals.state.anyState.boundBox[0].y = ll.m_y;
	initVals.state.anyState.boundBox[1].x = ur.m_x;
	initVals.state.anyState.boundBox[1].y = ur.m_y;

	pO->stateBufA = initVals;
	pO->stateBufB = initVals;

	// select the appropriate derived class (based on the type)
	// and create the object in the specified slot
	CDynObj* pTheObj = CreateTypedObject( type, objId );

	m_pHdr->dynObjectCount++;

	if( m_debug > 0 )
	{
		gout << "->Exit CCved::CreateDynObj, ptr is"  << (int)pTheObj << endl;
	}

	return pTheObj;
} // end of CreateDynObj

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function creates a static object and returns a pointer to a newly
// 	allocated CObj instance.
//
// Remarks: The attributes of the object will be set according to the argument
// 	attr.  Similarly, the state of the object is specified in the argument
// 	list, and cannot be changed once the object is created.
//
// 	The new object will not appear in the various interrogations functions
// 	until after the next execution of the database maintainer.
//
// 	The function returns a pointer to a dynamically allocated class instance.
// 	The allocated memory should be freed when not in use but keep in mind that
// 	deleting the class instance does not remove the object from the
// 	environment.  Static objects cannot be deleted once created.
//
// Arguments:
//  cName - the name to associate with the object
//	type - the type of the object
//  cAttr - the attributes to be used for the new object
//  cState - the state of the object
//
// Returns: The function returns the identifier of the newly created object
// 	or -1 to indicate an error.  Potential errors include overflow of the
// 	internal object list, a duplicate name (i.e., the provided name already
// 	is in use), or a type that is invalid.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::CreateStaticObj(
			const string&       cName,
			cvEObjType          type,
			const cvTObjAttr&   cAttr,
			const cvTObjState&  cState
			)
{
	// Search for an empty slot in the free space allocated
	// in the header.  If no slots are available, we return 0.
	int objId;

	if( m_pHdr->objectCount == m_pHdr->objectStrgCount )
	{
		return -1;
	}
	LockObjectPool();
	objId = m_pHdr->objectCount++;
	UnlockObjectPool();

	TObj* pO = BindObj( objId );
	pO->phase = eALIVE;
	pO->myId = objId;
	pO->type = type;
	strncpy_s( pO->name, cName.c_str(), cOBJ_NAME_LEN-1 );

	// set up the cAttributes; we simply copy whatever
	// the user provided into the allocated slot.
	// Dynamic objects can't have multiple cAttributes
	// so the cAttribute slot is the same the object id.
	pO->attr = cAttr;
//	m_pHdr->objectAttrCount++;
//	m_pHdr->attrCount++;
	pO->changedFlag  = 0xFF;

	cvTObjStateBuf defaultVals = { 0 };
	defaultVals.state.anyState.tangent.i = 1.0;
	defaultVals.state.anyState.lateral.j = -1.0;

	cvTObjStateBuf initVals = defaultVals;
	initVals.state = cState;

	// If the user specified tangent or lateral = (0,0,0),
	//	then the tangent/lateral must be set to the proper
	//	default value.  This is to insure that the object's
	//	bounding box never has a length/width = 0.  Also,
	//	it's important to insure that the tangent/lateral
	//	vectors are normalized.
	CVector3D cTan(initVals.state.anyState.tangent);
	if (cTan.Normalize() == 0.0) {
		initVals.state.anyState.tangent =
			defaultVals.state.anyState.tangent;
	}
	else {
		initVals.state.anyState.tangent.i = cTan.m_i;
		initVals.state.anyState.tangent.j = cTan.m_j;
		initVals.state.anyState.tangent.k = cTan.m_k;
	}

	CVector3D cLat(initVals.state.anyState.lateral);
	if (cLat.Normalize() == 0.0) {
		initVals.state.anyState.lateral =
			defaultVals.state.anyState.lateral;
	}
	else {
		initVals.state.anyState.lateral.i = cLat.m_i;
		initVals.state.anyState.lateral.j = cLat.m_j;
		initVals.state.anyState.lateral.k = cLat.m_k;
	}

	// Initialize the bounding box, based on the
	//	tangent, lateral, and position calculated
	//	above.
	CPoint3D ll, ur;
	CCved::UpdateBBox(
				&pO->attr,
				cTan,
				cLat,
				CPoint3D( initVals.state.anyState.position ),
				ll,
				ur
				);
	initVals.state.anyState.boundBox[0].x = ll.m_x;
	initVals.state.anyState.boundBox[0].y = ll.m_y;
	initVals.state.anyState.boundBox[1].x = ur.m_x;
	initVals.state.anyState.boundBox[1].y = ur.m_y;

	pO->stateBufA = initVals;
	pO->stateBufB = initVals;

	UpdateObjRefList( objId );

	pair<string, int> pairToInsert;
	pairToInsert.first = cName;
	pairToInsert.second = objId;
	m_objNameToId.insert( pairToInsert );

	return objId;
} // end of CreateStaticObj

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Update the (static) object reference list and include the object in the
// 	parameter.
//
// Remarks: This function searches the intersection and road piece quadtrees
// 	for the intersections and road control points that the object overlaps.
// 	If the object overlaps one of these, its object reference ID is added to
// 	the end of the existing list of object references.
//
//  The code for this function was taken almost entirely from the function
//  CreateObjectReferencePool(..) in tools/lricomp/objreflist.cxx.  Some parts
//  of the original code dealt with creating the obj ref pool, so they were
//  ommitted here.  Because the CreateObjectReferencePool function uses
//	several utility functions, these functions were moved to a file called
//	objreflistutils.cxx, which is included in both the cvedlib and lricc
//	compilations.  There is also a header file in cved/include/objreflist.h.
//
// Arguments:
// 	objId - identifier for new static object to add to list
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::UpdateObjRefList( TObjectPoolIdx objId )
{

	cvTObjRef*		pObjRefs = BindObjRef( 0 );
	cvTObj*			pObjects = BindObj( 0 );
	cvTObjAttr*		pObjAttrs = BindObjAttr( 0 );
	cvTIntrsctn*	pIntrsctns = BindIntrsctn( 0 );
	cvTRoad*		pRoads = BindRoad( 0 );
	cvTRoadPiece*	pRoadPieces = BindRoadPiece( 0 );
	cvTCntrlPnt*	pCntrlPnts = BindCntrlPnt( 0 );
	cvTLatCntrlPnt*	pLatCntrlPnts = BindLatCntrlPnt( 0 );

	CBoundingBox bbox;
	CPoint3D position;
	CPoint2D fourCorner[4];			// actual corners of the object,
									// not the corners of the bounding box
	TU32b lastRefObj = 1;

	int firstCp;
	int lastCp;
	CPoint2D segment[4];
	int cntrlPIndex;
	bool overlaps;

	// Iterate through obj ref list to find an unused slot
	while( ( pObjRefs[lastRefObj].objId != 0 ) && ( lastRefObj < m_pHdr->objRefCount ) )
	{
		lastRefObj++;
	}
	if( lastRefObj == m_pHdr->objRefCount )
	{
		cerr << "Error updating the obj ref list for obj " << objId << endl;
		return;
	}

	// Get the information about the object that's needed
	//	to determine which roads and intersections it is on.
	position = GetObjPos( objId );
	bbox = GetObjBoundBox( objId );
	GetFourCornerOfObj( objId, fourCorner, &position );

//	int objAttrIndx = pObjects[objId].attrs;
	double objTop = position.m_z + 0.5 * pObjects[objId].attr.zSize;
	double objBot = position.m_z - 0.5 * pObjects[objId].attr.zSize;

	// Search the intersection quadtree for the object
	vector<int> intSet;
	intSet.clear();
	m_intrsctnQTree.SearchRectangle(bbox, intSet);
	vector<int>::const_iterator itr;
	for( itr = intSet.begin(); itr != intSet.end(); ++itr )
	{

		// If the object is below the intersection, or
		//	if it is above the intersection more than
		//	cQRY_TERRAIN_SNAP units, then the object is not
		//	on the intersection.
		if ( (objTop < pIntrsctns[*itr].elevation ) ||
			 (objBot > pIntrsctns[*itr].elevation +
			  cQRY_TERRAIN_SNAP) ) {
			//gout << "object[" << objId
			//	 << "].z = [" << objTop << ", " << objBot
			//	 << "] is above or below intrsctn["
			//	 << *itr << "].z = "
			//	 << pIntrsctns[*itr].elevation
			//	 << endl;
			continue;
		}

		if( pObjects[objId].type == eCV_TERRAIN )
		{
			pIntrsctns[*itr].intrsctnFlag |= eTERRN_OBJ;
		}

		pObjRefs[lastRefObj].next = 0;
		pObjRefs[lastRefObj].objId = objId;

		UpdateRefOfIntrsctn(pObjRefs, pIntrsctns, *itr, lastRefObj);

		lastRefObj++;
	}

	// Search the roadpiece quadtree for the object
	intSet.clear();
	m_rdPcQTree.SearchRectangle( bbox, intSet );
	for( itr = intSet.begin(); itr != intSet.end(); ++itr )
	{
		// first control point of the road piece
		firstCp = pRoads[pRoadPieces[*itr].roadId].cntrlPntIdx +
					pRoadPieces[*itr].first;

		//last control point of the road piece
		lastCp  = pRoads[pRoadPieces[*itr].roadId].cntrlPntIdx +
						pRoadPieces[*itr].last;

		for( cntrlPIndex = firstCp; cntrlPIndex < lastCp; cntrlPIndex++ )
		{
			// If the object is below the control point, or
			//	if it is above the control point more than
			//	cQRY_TERRAIN_SNAP units, then the object is not
			//	on the road.
			if ( (objTop < pCntrlPnts[cntrlPIndex].location.z) ||
				 (objBot > pCntrlPnts[cntrlPIndex].location.z +
				  cQRY_TERRAIN_SNAP) ) {
				//gout << "object[" << objId
				//	 << "].z = [" << objTop << ", " << objBot
				//	 << "] is above or below cntrlpnt["
				//	 << cntrlPIndex << "].z = "
				//	 << pCntrlPnts[cntrlPIndex].location.z
				//	 << endl;
				continue;
			}

			GetSegment(
				&pCntrlPnts[cntrlPIndex],
				&pCntrlPnts[cntrlPIndex+1],
				segment
				);

			overlaps = RectangleIntersection( fourCorner, segment );

			if( overlaps )
			{
				if( pObjects[objId].type == eCV_TERRAIN )
				{
					pCntrlPnts[cntrlPIndex].cntrlPntFlag |= eTERRN_OBJ; //aggiungere and
				}

				pObjRefs[lastRefObj].next = 0;
				pObjRefs[lastRefObj].objId = objId;

				UpdateRefOfCntrlPnt( pObjRefs, pCntrlPnts, cntrlPIndex, lastRefObj );
				lastRefObj++;

			}// if (overlaps)
		} //each control point
	} // for each road piece

#if 0	// Debugging
	int i;
	printf("\n========== Dump ObjRef Pool ===========\n");
	int max = m_pHdr->objRefCount;
	for ( i = 0; i< max; i++){
		if (pObjRefs[i].objId != 0)
			printf("\n objref %d\t objId %d\t next %d",
				i, pObjRefs[i].objId, pObjRefs[i].next);
	}
	printf("\n========== end of dump objref pool =========\n");

	printf("\n========== Dump Intrsctn objref Pool ===========\n");
	int objRefIdx = 0;
	max = m_pHdr->intrsctnCount;
	for ( i = 0; i< max; i++){

		objRefIdx = pIntrsctns[i].objRefIdx;
		while (objRefIdx != 0) {
			printf("\n intrsctnId %d\t objref %d\t objId %d",
				i, objRefIdx, pObjRefs[objRefIdx].objId);
			objRefIdx = pObjRefs[objRefIdx].next;
		}
	}
	printf("\n========== end of dump Intrsctn objref pool =========\n");
	printf("\n========== Dump CntrlPnt objref Pool ===========\n");
	max = m_pHdr->longitCntrlCount;
	for ( i = 0; i< max; i++){

		objRefIdx = pCntrlPnts[i].objRefIdx;
		while (objRefIdx != 0) {
			printf("\n cntrlPnt %d\t objref %d\t objId %d",
				i, objRefIdx, pObjRefs[objRefIdx].objId);
			objRefIdx = pObjRefs[objRefIdx].next;
		}
	}
	printf("\n========== end of dump Road objref pool =========\n");
#endif
} // end of UpdateObjRefList

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns a typed CDynObj subclass instance that is of the
// 	specified type, and is bound to the specified object.
//
// Remarks: Unpredictable results may occur if the object id does not belong
//	to an object of that type.
//
// Arguments:
// 	type - Type of the object
// 	id - Identifier of the object
//
// Returns: A pointer to a valid CDynObj subclass instance, if the specified
// 	type is valid and id binds to a valid object.  Otherwise, a null pointer
// 	is returned
//
//////////////////////////////////////////////////////////////////////////////
CDynObj *
CCved::CreateTypedObject(cvEObjType type, int id)
{
	const char* typeStr[] = {
		"eCV_INVALID",
		"eCV_TRAJ_FOLLOWER",
		"eCV_VEHICLE",
		"eCV_TRAILER",
		"eCV_RAIL_VEH",
		"eCV_TERRAIN",
		"eCV_TRAFFIC_LIGHT",
		"eCV_TRAFFIC_SIGN",
		"eCV_COMPOSITE_SIGN",
		"eCV_OBSTACLE",
		"eCV_POI",
		"eCV_COORDINATOR",
		"eCV_EXTERNAL_DRIVER",
		"eCV_WALKER",
		"eCV_EXTERNAL_TRAILER",
		"eCV_VIRTUAL_OBJECT",
		"eCV_OBJ_TYPE_EN"
	};

	CCved::Logoutf("CCved::CreateTypedObject(%s, %d)\n", typeStr[type], id);

	switch ( type ) {
		case eCV_TRAJ_FOLLOWER   : return new CTrajFollowerObj(*this, BindObj(id));
 		case eCV_VEHICLE         : return new CVehicleObj(*this, BindObj(id));
		case eCV_TRAILER         : return new CTrailerObj(*this, BindObj(id));
		case eCV_RAIL_VEH        : return new CRailVehObj(*this, BindObj(id));
		case eCV_TERRAIN         : return new CTerrainObj(*this, BindObj(id));
		case eCV_TRAFFIC_LIGHT   : return new CTrafficLightObj(*this, BindObj(id));
		case eCV_TRAFFIC_SIGN    : return new CTrafficSignObj(*this, BindObj(id));
		case eCV_COMPOSITE_SIGN  : return new CCompositeSignObj(*this, BindObj(id));
		case eCV_OBSTACLE        : return new CObstacleObj(*this, BindObj(id));
		case eCV_POI             : return new CPoiObj(*this, BindObj(id));
		case eCV_COORDINATOR     : return new CCoordinatorObjectObj(*this, BindObj(id));
		case eCV_EXTERNAL_DRIVER : return new CExternalDriverObj(*this, BindObj(id));
		case eCV_EXTERNAL_TRAILER: return new CExternalTrailerObj(*this, BindObj(id));
		case eCV_WALKER          : return new CWalkerObj(*this, BindObj(id));
		case eCV_VIRTUAL_OBJECT  : return new CVisualObjectObj(*this, BindObj(id));
		default                  : return 0;
	}
} // end of CreateTypedObject

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns the runtime type_info structure that corresponds to
// 	the derived class whose type corresponds to the specified cvObjType
// 	variable.
//
// Remarks:
//
// Arguments:
// 	type - object type to check
//
// Returns: If type is valid then a a const type_info reference to that type
// 	is returned.  Otherwise, the function causes a failed assertion.
//
//////////////////////////////////////////////////////////////////////////////
const type_info &
CCved::GetRunTimeDynObjType(cvEObjType type)
{
	switch ( type ) {
		case eCV_TRAJ_FOLLOWER   : return typeid( CTrajFollowerObj );
		case eCV_VEHICLE         : return typeid( CVehicleObj );
		case eCV_TRAILER         : return typeid( CTrailerObj );
		case eCV_RAIL_VEH        : return typeid( CRailVehObj );
		case eCV_TERRAIN         : return typeid( CTerrainObj );
		case eCV_TRAFFIC_LIGHT   : return typeid( CTrafficLightObj );
		case eCV_TRAFFIC_SIGN    : return typeid( CTrafficSignObj );
		case eCV_COMPOSITE_SIGN  : return typeid( CCompositeSignObj );
		case eCV_OBSTACLE        : return typeid( CObstacleObj );
		case eCV_POI             : return typeid( CPoiObj );
		case eCV_COORDINATOR	 : return typeid( CCoordinatorObjectObj );
		case eCV_EXTERNAL_DRIVER : return typeid( CExternalDriverObj );
		case eCV_EXTERNAL_TRAILER: return typeid( CExternalTrailerObj );
		case eCV_WALKER          : return typeid( CWalkerObj );
		case eCV_VIRTUAL_OBJECT  : return typeid(CVisualObjectObj);
		default                  : assert( 0 ); return typeid( int ); // dummy
	}
} // end of GetRunTimeDynObjType

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function receives a dynamic object identifier and returns a pointer
// 	to a dynamic object that reflects the type of the object.
//
// Remarks: The returned pointer can be used similar to the pointer returned
// 	by the CreateDynObj() function, but whereas the CreateDynObj() function
// 	actually creates a new object, this function simply returns a pointer to
// 	a class for an existing object.
//
// 	The pointer can be down casted to the appropriate type to allow the user
// 	to obtain type specific state variables.
//
// 	Do not free or delete the pointer.
//
// Arguments:
// 	objId - the identifiers of an existing object
//
// Returns: A pointer to a derived class of CDynObj.  The actual class depends
// 	on the object's type.  The function returns 0 when there is a problem.
// 	Potential problems include:
// 	* an identifier for an object that does not exist
// 	* an identifier for an object that is not a dynamic object
// 	* memory allocation problems (highly unlikely)
//
//////////////////////////////////////////////////////////////////////////////
const CDynObj *
CCved::BindObjIdToClass(int objId)
{
	if ( objId < 0 || objId > cNUM_DYN_OBJS )
		return 0;

	TObj  *pO = BindObj(objId);

	if ( (pO->phase == eDEAD ) || (pO->phase == eBORN) )
		return 0;

	// if it exists, use it.  We check that the existing object match
	// the type of the object to cover the case of a client CVED
	// instance that is caching an object class, but the master
	// has deleted the object in the slot, then created a new one
	// with a different type; in that case we reallocate a new
	// object, appropriately typed.
	if ( m_dynObjCache[objId] &&
			typeid(*m_dynObjCache[objId]) == GetRunTimeDynObjType(pO->type)) {

		m_dynObjCache[objId]->MakeReadOnly();
		return m_dynObjCache[objId];
	}

	// if it doesn't exist, delete old, create a new one
	// note that if it is 0, delete still works
	delete m_dynObjCache[objId];
	m_dynObjCache[objId] = CreateTypedObject(pO->type, objId);
	m_dynObjCache[objId]->MakeReadOnly();
	return m_dynObjCache[objId];
} // end of BinObjIdToClass

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function receives a dynamic object identifier and returns a pointer
// 	to a dynamic object that reflects the type of the object.
//
// Remarks: The returned pointer can be used similar to the pointer returned
// 	by the CreateDynObj() function, but whereas the CreateDynObj() function
// 	actually creates a new object, this function simply returns a pointer to
// 	a class for an existing object.
//
// 	The pointer can be down casted to the appropriate type to allow the user
// 	to obtain type specific state variables.
//
// 	Do not free or delete the pointer.
//
// Arguments:
// 	objId - the identifiers of an existing object
//
// Returns: A pointer to a derived class of CDynObj.  The actual class depends
// 	on the object's type.  The function returns 0 when there is a problem.
// 	Potential problems include:
// 	* an identifier for an object that does not exist
// 	* an identifier for an object that is not a dynamic object
// 	* memory allocation problems (highly unlikely)
//
//////////////////////////////////////////////////////////////////////////////
CDynObj *
CCved::BindObjIdToClass2(int objId)
{
	if ( objId < 0 || objId > cNUM_DYN_OBJS )
		return 0;

	TObj  *pO = BindObj(objId);

	if ( (pO->phase == eDEAD ) || (pO->phase == eBORN) )
		return 0;

	// if it exists, use it.  We check that the existing object match
	// the type of the object to cover the case of a client CVED
	// instance that is caching an object class, but the master
	// has deleted the object in the slot, then created a new one
	// with a different type; in that case we reallocate a new
	// object, appropriately typed.
	if ( m_dynObjCache[objId] &&
			typeid(*m_dynObjCache[objId]) == GetRunTimeDynObjType(pO->type)) {

		m_dynObjCache[objId]->MakeReadOnly(); //make read only prevents the object from deleting
		return m_dynObjCache[objId];
	}

	// if it doesn't exist, delete old, create a new one
	// note that if it is 0, delete still works
	delete m_dynObjCache[objId];
	m_dynObjCache[objId] = CreateTypedObject(pO->type, objId);
	m_dynObjCache[objId]->MakeReadOnly(); //make read only prevents the object from deleting
	return m_dynObjCache[objId];
} // end of BinObjIdToClass

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function deletes a dynamic object that has been created by calling
// 	the CreateDynObj function.
//
// Remarks: Upon deletion, the object will still appear in the various
// 	interrogation functions until after the next execution of the database
// 	maintainer.
//
// 	The function will de-allocate the memory occupied by the object.  Do not
// 	call the delete operator on the object pointer after this function is
// 	called as this will cause heap corruption.
//
// Arguments:
//  pObj - pointer to the object to delete
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::DeleteDynObj(CDynObj *pObj)
{
	assert(pObj);
	TObj *pO  = BindObj(pObj->GetId());

	LockObjectPool();
	pO->phase = eDYING;
	UnlockObjectPool();
	int id = pObj->GetId();
	multimap<string,int>::iterator lb = m_objNameToId.lower_bound( pObj->GetName() );
	multimap<string,int>::iterator ub = m_objNameToId.upper_bound( pObj->GetName() );
	for ( ; lb != ub; lb++ ) {
		if ( id == lb->second ) {
			m_objNameToId.erase( lb );
			break;
		}
	}
	delete pObj;
} // end of DeleteDynObj


//////////////////////////////////////////////////////////////////////////////
//	Object access functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function can be used to verify if a particular integer object
// 	identifier refers to a valid object.
//
// Remarks: A valid object is an object whose internal state is Alive or
// 	Dying.  Note an object that has just been created using the CreateDynObj
// 	function is not considered valid until after the next execution of the
// 	CCved::Maintain function.
//
//	Object id 0 is used exlusively for the driver so a very simple method to
//	verify if the driver is specified is calling this function with an
//	identifier of 0.
//
// Arguments:
// 	objId - the identifier to verify
//
// Returns: The function returns true of the id refers to a valid object or
// 	false if the object is not valid.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsObjValid( int objId ) const
{
	if ( objId < 0 || objId >= (int)m_pHdr->objectStrgCount ) return false;

	TObj *pO = BindObj( objId );
	if ( pO->phase == eALIVE || pO->phase == eDYING )
		return true;
	else
		return false;
} // end of IsObjValid


//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the type of an object whose identifer
//  is provided as an argument.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The given object's type.
//
//////////////////////////////////////////////////////////////////////////////
cvEObjType
CCved::GetObjType( int objId ) const
{
	TObj* pO = BindObj( objId );
	cvEObjType t = pO->type;
	return t;
} // end of GetObjType


cvEObjType
CCved::GetObjType( int objId, bool singleExt ) const
{
	TObj* pO = BindObj( objId );
	cvEObjType t = pO->type;
	if (singleExt
		&& eCV_EXTERNAL_DRIVER == t
		&& 0 != objId)
		t = eCV_VEHICLE;
	return t;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the SOL identifier of the specified
//  object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The given object's sol id.
//
//////////////////////////////////////////////////////////////////////////////
TS32b
CCved::GetObjSolId( int objId ) const
{
	cvTObj* pObj = BindObj( objId );
	return pObj->attr.solId;
} // end of GetObjSolId

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the HCSM identifier of the specified
//  object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The given object's hcsm id.
//
//////////////////////////////////////////////////////////////////////////////
TS32b
CCved::GetObjHcsmId( int objId ) const
{
	cvTObjAttr* pAttr = BindObjAttr( objId );
	return pAttr->hcsmId;
} // end of getObjHcsmId

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the CIGI identifier of the specified
//  object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
//  This function will return -1 if the given object is NOT a LRI static
//  object.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The given object's cigi id; -1 for non-LRI static objects.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetObjCigiId( int objId ) const
{
	cvTObj* pObj = BindObj( objId );
	return pObj->attr.cigi;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the HCSM Type identifier of the
//  specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The given object's hcsm type id.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetObjHcsmTypeId( int objId ) const
{
	cvTObj* pObj = BindObj( objId );
	return pObj->hcsmType;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the name of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The given object's name.
//
//////////////////////////////////////////////////////////////////////////////
const char*
CCved::GetObjName( int objId ) const
{
	TObj* pO = BindObj( objId );
	return pO->name;
} // End of GetObjName

//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function returns the environment area requested.
//
// Remarks:
//
// Arguments:
//  id - The id indicating which area to return.
//  area - A reference to hold the environment area requested.
//
// Return: A bool to indicate if returning a valid area.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetEnvArea( int id, CEnvArea& area ) const
{
	bool invalidAreaId = id < 0 || id >= (int)m_pHdr->envAreaCount;
	if( invalidAreaId )
	{
		return false;
	}

	// return the area associated with the given id
	bool needGlobalArea = id == 0;
	if( needGlobalArea )
	{
		CEnvArea a( *this, id );
		area = a;
	}
	else
	{
		area = m_envAreas[id - 1];
	}

	return true;
}

void
CCved::GetEnvArea( const CPoint2D& pt, vector<CEnvArea>& areas )
{
	GetEnvArea( pt.m_x, pt.m_y, areas );
}


void
CCved::GetEnvArea( const CPoint3D& pt, vector<CEnvArea>& areas )
{
	GetEnvArea( pt.m_x, pt.m_y, areas );
}


void
CCved::GetEnvArea( double x, double y, vector<CEnvArea>& areas )
{
	areas.clear();
	vector<CEnvArea>::iterator i;
	for( i = m_envAreas.begin(); i != m_envAreas.end(); i++ )
	{
		bool pointInsideArea = i->Enclose( x, y );
		if( pointInsideArea )
		{
			areas.push_back( *i );
		}
	}
}

CEnvArea
CCved::GetGlobalEnvArea( void ) const
{
   return( CEnvArea( *this, 0 ) );
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all environment areas in the
//  virtual environment.
//
// Remarks: This function inserts all the environment areas in the
//  provided STL vector.  It erases all previous contents of this vector.
//
// Arguments:
//  areas - A container to hold the environment areas.
//
// Return:
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetAllEnvArea( vector<CEnvArea>& areas ) const
{
	areas.clear();
	areas = m_envAreas;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function creates an environment area.
//
// Remarks: This function creates an environment area by going to the
//  memory pool and putting down the information in appropriate location.
//  It only writes the information without creating any object.
//
// Arguments:
//  info - A reference to cvTEnviroInfo as the environmental info of the area.
//  polyPts - A reference to a vector of 2d ponints representing the area.
//  originPt - A 2d point as the origin of the area.
//  id - A reference as the id of the area.
//
// Return: A bool indicating if creation is successful.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::CreateEnvArea(
			vector<cvTEnviroInfo>& info,
			vector<CPoint2D>& polyPts,
			CPoint2D& originPt
			)
{
	bool failedErrorCheck = (
				m_pHdr->envAreaCount == m_pHdr->envAreaSize ||
				polyPts.size() >= cMAX_ENV_POLY_PTS
				);
	if( failedErrorCheck )  return false;

	TU32b ofs;
    char* pEnvAreaPool;
	cvTEnviroArea* pEnvArea;
    ofs          = m_pHdr->envAreaOfs;
    pEnvAreaPool = reinterpret_cast<char *>(m_pHdr) + ofs;
    pEnvArea     = reinterpret_cast<cvTEnviroArea *>(pEnvAreaPool) +
					m_pHdr->envAreaCount;

	char   *pEnvInfoPool;
    cvTEnviroInfo* pEnvInfo;
	ofs          = m_pHdr->envInfoOfs;
    pEnvInfoPool = reinterpret_cast<char *>(m_pHdr) + ofs;
    pEnvInfo     = reinterpret_cast<cvTEnviroInfo *>(pEnvInfoPool) +
					m_pHdr->envInfoCount;

	// check to see if there is enough space in the memory pools to
	// add the new environment area
	bool outOfSpace = (
				m_pHdr->envInfoCount + info.size() >= m_pHdr->envInfoSize
				);
	if( outOfSpace )  return false;

	m_pHdr->envAreaCount++;

	pEnvArea->origin.x = originPt.m_x;
	pEnvArea->origin.y = originPt.m_y;
	pEnvArea->numOfPolyPts = (int) polyPts.size();

	int i;
	for( i = 0; i<pEnvArea->numOfPolyPts; i++ )
	{
		pEnvArea->polyPt[i].x = polyPts[i].m_x;
		pEnvArea->polyPt[i].y = polyPts[i].m_y;
	}

	pEnvArea->infoIdx = m_pHdr->envInfoCount;
	pEnvArea->numOfInfo = (int) info.size();
	pEnvArea->maxNumOfInfo = pEnvArea->numOfInfo + cEXTRA_ENV_INFO;

	m_pHdr->envInfoCount += pEnvArea->maxNumOfInfo;

	for( i = 0; i < pEnvArea->numOfInfo; i++ )
	{
		pEnvInfo[i] = info[i];
	}

	// build the class that represents this environment and add it to
	// the vector that holds all non-global areas
	CEnvArea envArea( *this, m_pHdr->envAreaCount - 1 );
	m_envAreas.push_back( envArea );

	return true;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  This function add environmental conditions to an area.
//
// Remarks: This function takes as paraters an reference to
//  environmental area and a reference to the environmental contiditons
//  to be added, and simply call the member function SetEnvCndtn() of
//  CEnvArea which goes into the memory pool to put down this informaion.
//
// Arguments:
//  area - A reference to CEnvArea as the area for added environmental info.
//  info - A reference to cvTEnviroInfo as the environmental info to be added.
//
// Return:
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetEnviron( CEnvArea& area, vector<cvTEnviroInfo>& info )
{
	area.SetEnvCndtn( info );
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function adds Global environmental conditions
//  to an area.
//
// Remarks: This function takes as paraters a reference to the
//  environmental contiditons to be added, and  simply call the
//  member function SetEnvCndtn() of the global environment area
//  which goes into the memory pool to put down this informaion .
//
// Arguments:
//  info - A reference to cvTEnviroInfo as the environmental info to be added.
//
// Return:
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetGlobalEnviron( vector<cvTEnviroInfo>& info )
{
	CEnvArea area = GetGlobalEnvArea();
	area.SetEnvCndtn( info );
}

void
CCved::DumpEnvArea()
{
	vector<CEnvArea>::const_iterator i;
	for( i = m_envAreas.begin(); i != m_envAreas.end(); i++ )
	{
		i->DumpEnvArea();
	}

	GetGlobalEnvArea().DumpEnvArea();
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function returns the state of the specified traffic light.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - the identifier of the queried light
//
// Return: state of traffic light
//
//////////////////////////////////////////////////////////////////////////////
eCVTrafficLightState
CCved::GetTrafficLightState(int objId) const
{
	TObj* pO = BindObj(objId);
	return pO->stateBufA.state.trafficLightState.state;
} // end of GetTrafficLightState

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the audio state of the specified
//  object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Return: Audio state of the given object.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetVehicleAudioState( int objId )
{
	TObj* pO = BindObj( objId );

	if( (m_pHdr->frame & 1) == 0 )
	{
		return pO->stateBufA.state.anyState.audioState;
	}
	else
	{
		return pO->stateBufB.state.anyState.audioState;
	}
} // end of GetAudioState

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the visual state of the specified
//  object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Return: The given object's visual state.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetVehicleVisualState( int objId )
{
	TObj* pO = BindObj( objId );

	if( (m_pHdr->frame & 1) == 0 )
	{
		return pO->stateBufA.state.anyState.visualState;
	}
	else
	{
		return pO->stateBufB.state.anyState.visualState;
	}
} // end of GetVisualState

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function sets the state of the specified traffic light.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the modified object.
//	state - The desired state to set the traffic light.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetTrafficLightState( int objId, eCVTrafficLightState state )
{
	TObj* pO = BindObj( objId );
	pO->changedFlag = 0xFF;

	// Set both buffers with the understanding that traffic lights
	// state is not propagated by the maintainer.
	pO->stateBufB.state.trafficLightState.state = state;
	pO->stateBufA.state.trafficLightState.state = state;
} // end of SetTrafficLightState

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function sets the audio state of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the modified object.
//	state - The desired audio state to set the object.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetVehicleAudioState( int objId, int state )
{
	TObj* pO = BindObj( objId );
	pO->changedFlag = 0xFF;
	if( (m_pHdr->frame & 1) == 0 )
	{
		pO->stateBufB.state.anyState.audioState = state;
	}
	else
	{
		pO->stateBufA.state.anyState.audioState = state;
	}
} // end of SetAudioState

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function sets the visual state of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the modified object.
//	state - The desired visual state to set the object.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::SetVehicleVisualState( int objId, int state )
{
	TObj* pO = BindObj( objId );
	pO->changedFlag = 0xFF;
	if( (m_pHdr->frame & 1) == 0 )
	{
		pO->stateBufB.state.anyState.visualState = state;
	}
	else
	{
		pO->stateBufA.state.anyState.visualState = state;
	}
} // end of SetVisualState



//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the position of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	Keep in mind that the position returned is the position of the object as
// 	per the time before the most recent execution of the CCved::Maintain()
// 	function.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The object position.
//
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CCved::GetObjPos( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		CPoint3D pos( pO->stateBufA.state.anyState.position );
		return pos;
	}
	else
	{
		CPoint3D pos( pO->stateBufB.state.anyState.position );
		return pos;
	}
} // end of GetObjPos
bool
CCved::GetVehicleFourCorners(int objId, vector<CPoint3D>& pnts) const{
    BindObj(objId);
    pnts.clear();
    pnts.resize(4);
    GetFourCornerOfObj(objId,&(pnts[0]));
    return true;
}
//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the immediate position of the
//  specified object.
//
// Remarks: The difference between this and the GetObjPos function is that
// 	this function returns the most up-to-date position of the object. This
// 	function can potentially return different positions for the same object
// 	during the same frame and its use is discouraged unless other means of
// 	coordination between the dynamic servers and the caller are in place.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Return Value: The immediate position of object.
//
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CCved::GetObjPosInstant( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		CPoint3D pos( pO->stateBufB.state.anyState.position );
		return pos;
	}
	else
	{
		CPoint3D pos( pO->stateBufA.state.anyState.position );
		return pos;
	}
} // end of GetObjPosInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns a vector indicating the tangent
//  orientation of the specified object.
//
// 	The vector is normalized, i.e., its length is 1.  In addition, the angle
// 	between this vector and the tangent vector is 90 degrees.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The object tangent.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CCved::GetObjTan( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		CVector3D tang( pO->stateBufA.state.anyState.tangent );
		return tang;
	}
	else
	{
		CVector3D tang( pO->stateBufB.state.anyState.tangent );
		return tang;
	}
} // End of GetObjTan

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns a vector indicating the instant
//  tangent orientation of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	The difference between this and the GetObjTan function is that this
// 	function returns the most up-to-date tangent of the object. This function
// 	can potentially return different tangents for the same object during the
// 	same frame and its use is discouraged unless other means of coordination
// 	between the dynamic servers and the caller are in place.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The immediate tangent of the object.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CCved::GetObjTanInstant( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		CVector3D tang( pO->stateBufB.state.anyState.tangent );
		return tang;
	}
	else
	{
		CVector3D tang( pO->stateBufA.state.anyState.tangent );
		return tang;
	}
} // end of GetObjTanInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns a vector indicating the instant
//  lateral orientation of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	The vector is normalized, i.e., its length is 1.  In addition, the angle
// 	between this vector and the tangent vector is 90 degrees.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The object lateral.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CCved::GetObjLat( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		CVector3D lat( pO->stateBufA.state.anyState.lateral );
		return lat;
	}
	else
	{
		CVector3D lat( pO->stateBufB.state.anyState.lateral );
		return lat;
	}
} // end of GetObjLat
//////////////////////////////////////////////////////////////////////////////
///
///\brief This function gets the target point for a ADO
///
///\remark Make sure to verify that the object identifier is valid before
/// 	using it, as this function will throw an exception if the object is
/// 	invalid. This only works with ADOs, empty vector is returned on non
///     ADOSs
///
///\param[in]  objId - The identifier of the queried object.
///
/// Returns: The object lateral.
///
//////////////////////////////////////////////////////////////////////////////
CVector3D
CCved::GetObjTargetPoint( int objId ) const
{
	TObj* pO = BindObj( objId );
    if (pO->hcsmType != eCV_VEHICLE){
        return CVector3D();
    }
	if( (m_pHdr->frame & 1) == 0 )
	{
        CVector3D tp( pO->stateBufA.contInp.vehicleContInp.contInp.targPos );
		return tp;
	}
	else
	{
		CVector3D tp( pO->stateBufA.contInp.vehicleContInp.contInp.targPos );
		return tp;
	}
} // end of GetObjLat
//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns a vector indicating the instant
//  lateral orientation of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	The difference between this and the GetObjLat function is that this
// 	function returns the most up-to-date lateral of the object. This function
// 	can potentially return different laterals for the same object during the
// 	same frame and its use is discouraged unless other means of coordination
// 	between the dynamic servers and the caller are in place.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Return Value: The immediate object lateral.
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CCved::GetObjLatInstant( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		CVector3D lat( pO->stateBufB.state.anyState.lateral );
		return lat;
	}
	else
	{
		CVector3D lat( pO->stateBufA.state.anyState.lateral );
		return lat;
	}
} // end of GetObjLatInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the velocity of the specified object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	Keep in mind that the velocity returned is the velcoity of the object as
// 	per the time before the most recent execution of the CCved::Maintain()
// 	function.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Returns: The object velocity.
//
//////////////////////////////////////////////////////////////////////////////
double
CCved::GetObjVel( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		return pO->stateBufA.state.anyState.vel;
	}
	else
	{
		return pO->stateBufB.state.anyState.vel;
	}
} // end of GetObjVel

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function returns the immediate velocity of the
//  specified object.
//
// Remarks: The difference between this and the GetObjVel function is that
// 	this function returns the most up-to-date velocity of the object. This
// 	function can potentially return different velocities for the same object
// 	during the same frame and its use is discouraged unless other means of
// 	coordination between the dynamic servers and the caller are in place.
//
// Arguments:
//  objId - The identifier of the queried object.
//
// Return Value: The immediate velocity of object.
//
//////////////////////////////////////////////////////////////////////////////
double
CCved::GetObjVelInstant( int objId ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		return pO->stateBufB.state.anyState.vel;
	}
	else
	{
		return pO->stateBufA.state.anyState.vel;
	}
} // end of GetObjVelInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all type independent information
//  about the state of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	It is faster to call this function once than calling all of the individual
// 	versions separately (i.e., GetObjPos) especially when more than a couple
// 	of the data is needed.
//
// Arguments:
//  objId - The identifier of the queried object.
//  state - Output variable that contains the state of the object.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetObjState( int objId, cvTObjState& state ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		state = pO->stateBufA.state;
#if 0
		state.anyState.position = pO->stateBufA.state.anyState.position;
		state.anyState.tangent = pO->stateBufA.state.anyState.tangent;
		state.anyState.lateral = pO->stateBufA.state.anyState.lateral;
		state.anyState.boundBox[0] = pO->stateBufA.state.anyState.boundBox[0];
		state.anyState.boundBox[1] = pO->stateBufA.state.anyState.boundBox[1];
		state.anyState.vel = pO->stateBufA.state.anyState.vel;
#endif
	}
	else
	{
		state = pO->stateBufB.state;
#if 0
		state.anyState.position = pO->stateBufB.state.anyState.position;
		state.anyState.tangent = pO->stateBufB.state.anyState.tangent;
		state.anyState.lateral = pO->stateBufB.state.anyState.lateral;
		state.anyState.boundBox[0] = pO->stateBufB.state.anyState.boundBox[0];
		state.anyState.boundBox[1] = pO->stateBufB.state.anyState.boundBox[1];
		state.anyState.vel = pO->stateBufB.state.anyState.vel;
#endif
	}
} // end of GetObjState

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all the type independent information
//  about the state of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	It is faster to call this function once than calling all of the individual
// 	versions separately (i.e., GetObjPos) especially when more than a couple
// 	of the data is needed.
//
// Arguments:
//  objId - The identifier of the queried object.
//  pos - The output variable containing the position of the object.
//  tan - The output variable containing the tangent of the object.
//  lat - The output variable containing the lateral of the object.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetObjState(
			int objId,
			CPoint3D& pos,
			CVector3D& tan,
			CVector3D& lat
			) const
{
	cvTObjState state;
	GetObjState( objId, state );
	pos = state.anyState.position;
	tan = state.anyState.tangent;
	lat = state.anyState.lateral;
} // end of GetObjState

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the immediate type independent
//  information about the state of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	The difference between this and the GetObjLat function is that this
// 	function returns the most up-to-date lateral of the object. This function
// 	can potentially return different laterals for the same object during the
// 	same frame and its use is discouraged unless other means of coordination
// 	between the dynamic servers and the caller are in place.
//
// 	It is faster to call this function once than calling all of the individual
// 	versions separately (i.e., GetObjPos) especially when more than a couple
// 	of the data is needed.
//
// Arguments:
//  objId - The identifier of the queried object.
//  state - Output variable that contains the immediate object state.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetObjStateInstant( int objId, cvTObjState& state ) const
{
	TObj* pO = BindObj( objId );
	if( (m_pHdr->frame & 1) == 0 )
	{
		state = pO->stateBufB.state;

#if 0
		state.anyState.position = pO->stateBufB.state.anyState.position;
		state.anyState.tangent = pO->stateBufB.state.anyState.tangent;
		state.anyState.lateral = pO->stateBufB.state.anyState.lateral;
		state.anyState.boundBox[0] = pO->stateBufB.state.anyState.boundBox[0];
		state.anyState.boundBox[1] = pO->stateBufB.state.anyState.boundBox[1];
		state.anyState.vel = pO->stateBufB.state.anyState.vel;
#endif
	}
	else
	{
		state = pO->stateBufA.state;

#if 0
		state.anyState.position = pO->stateBufA.state.anyState.position;
		state.anyState.tangent = pO->stateBufA.state.anyState.tangent;
		state.anyState.lateral = pO->stateBufA.state.anyState.lateral;
		state.anyState.boundBox[0] = pO->stateBufA.state.anyState.boundBox[0];
		state.anyState.boundBox[1] = pO->stateBufA.state.anyState.boundBox[1];
		state.anyState.vel = pO->stateBufA.state.anyState.vel;
#endif
	}
} // end of GetObjStateInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the immediate type independent
//  information about the state of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	The difference between this and the GetObjLat function is that this
// 	function returns the most up-to-date lateral of the object. This function
// 	can potentially return different laterals for the same object during the
// 	same frame and its use is discouraged unless other means of coordination
// 	between the dynamic servers and the caller are in place.
//
// 	It is faster to call this function once than calling all of the individual
// 	versions separately (i.e., GetObjPos) especially when more than a couple
// 	of the data is needed.
//
// Arguments:
//  objId - The identifier of the queried object.
//  pos - Output variable that contains the immediate object position.
//  tan - Output variable that contains the immediate object tangent.
//  lat - Output variable that contains the immediate object lateral.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::GetObjStateInstant(
			int objId,
			CPoint3D& pos,
			CVector3D& tan,
			CVector3D& lat
			) const
{
	cvTObjState state;
	GetObjStateInstant( objId, state );
	pos = state.anyState.position;
	tan = state.anyState.tangent;
	lat = state.anyState.lateral;
} // end of GetObjStateInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the length of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
// 	objId - The identifier of the object to query.
//
// Returns: The length of the object.
//
//////////////////////////////////////////////////////////////////////////////
double
CCved::GetObjLength(int objId) const
{
	cvTObjAttr *pA = BindObjAttr( objId );
	return pA->xSize;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the width of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
// 	objId - The identifier of the object to query.
//
// Returns: The width of the object.
//
//////////////////////////////////////////////////////////////////////////////
double
CCved::GetObjWidth(int objId) const
{
	cvTObjAttr *pA = BindObjAttr( objId );
	return pA->ySize;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the color index of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
// 	objId - The identifier of the object to query.
//
// Returns: The color index of the object.
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::GetObjColorIndex(int objId) const
{
	cvTObjAttr *pA = BindObjAttr( objId );
	return pA->colorIndex;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function returns the bounding box of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// Arguments:
// 	objId - the identifier of the object to query
//
// Returns: The bounding box of the object.
//
//////////////////////////////////////////////////////////////////////////////
CBoundingBox
CCved::GetObjBoundBox(int objId) const
{
	TObj *pO = BindObj(objId);
	cvTObjState::AnyObjState *pS;

	if ( (m_pHdr->frame & 1) == 0 ) {
		pS = &pO->stateBufB.state.anyState;
	}
	else {
		pS = &pO->stateBufA.state.anyState;
	}
	CBoundingBox bb(pS->boundBox[0].x, pS->boundBox[0].y,
			pS->boundBox[1].x, pS->boundBox[1].y);

	return bb;
} // end of GetObjBoundBox

//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function returns the instant bounding box of an object.
//
// Remarks: Make sure to verify that the object identifier is valid before
// 	using it, as this function will throw an exception if the object is
// 	invalid.
//
// 	The difference between this and the GetObjLat function is that this
// 	function returns the most up-to-date lateral of the object. This function
// 	can potentially return different laterals for the same object during the
// 	same frame and its use is discouraged unless other means of coordination
// 	between the dynamic servers and the caller are in place.
//
// Arguments:
// 	objId - the identifier of the object to query
//
// Returns: The instant bounding box of the object.
//
//////////////////////////////////////////////////////////////////////////////
CBoundingBox
CCved::GetObjBoundBoxInstant(int objId) const
{
	TObj *pO = BindObj(objId);
	cvTObjState::AnyObjState *pS;

	if ( (m_pHdr->frame & 1) == 0 ) {
		pS = &pO->stateBufA.state.anyState;
	}
	else {
		pS = &pO->stateBufB.state.anyState;
	}
	CBoundingBox bb(pS->boundBox[0].x, pS->boundBox[0].y,
			pS->boundBox[1].x, pS->boundBox[1].y);

	return bb;
} // end of GetObjBoundBoxInstant

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function updates the bounding box of an object based on the object's
// 	location and orientation.
//
// Remarks: This is a static member function to allow the CCved class to call
// 	it without having access to the class pointer.
//
// Arguments:
// 	cpAttr - pointer to the attribute structure to use
// 	cTangent - tangent vector of object
// 	cLateral - lateral vector of object
// 	cPosition - position of object
// 	lowerLeft - output variable containing the lower left corner of the
// 		resulting bounding box.
// 	upperRight - output variable containing the upper right corner of the
// 		resulting bounding box.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::UpdateBBox(const cvTObjAttr *cpAttr,
		const CVector3D &cTangent,
		const CVector3D &cLateral,
		const CPoint3D &cPosition,
		CPoint3D   &lowerLeft,
		CPoint3D   &upperRight)
{
	CVector3D  lat(cLateral);
	CVector3D  forw(cTangent);

	forw.Scale(0.5 * cpAttr->xSize);
	lat.Scale(0.5 * cpAttr->ySize);

	CPoint3D  corners[4];

	corners[0] = cPosition + forw + lat;
	corners[1] = cPosition + forw - lat;
	corners[2] = cPosition - forw - lat;
	corners[3] = cPosition - forw + lat;

	CPoint3D mn(corners[0]), mx(corners[0]);

	for (int i=1; i<4; i++) {
		if ( corners[i].m_x < mn.m_x ) mn.m_x = corners[i].m_x;
		if ( corners[i].m_y < mn.m_y ) mn.m_y = corners[i].m_y;
		if ( corners[i].m_z < mn.m_z ) mn.m_z = corners[i].m_z;

		if ( corners[i].m_x > mx.m_x ) mx.m_x = corners[i].m_x;
		if ( corners[i].m_y > mx.m_y ) mx.m_y = corners[i].m_y;
		if ( corners[i].m_z > mx.m_z ) mx.m_z = corners[i].m_z;
	}

	lowerLeft  = mn;
	upperRight = mx;
} // end of UpdateBBox

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function generates a string that provides various statistics on how
// 	the query terrain function performs, especially in how often the hint is
// 	used.
//
// Remarks:
//
// Arguments:
// 	desc - a string contain a description of the stats.  Just print it.
//	reset - (optional) parameter indicating whether the previous calcualtions
//		should be reset or not.  Default value is false.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::QryTerrainPerfCheck(string &desc, bool reset)
{
	char buf[600];
	long    misses       = m_terQryCalls-(m_terQryRoadHits+m_terQryInterHits);
	double   roadHitPerc  = 100.0 * m_terQryRoadHits / m_terQryCalls;
	double   interHitPerc = 100.0 * m_terQryInterHits / m_terQryCalls;
	double   missPerc     = 100.0 * misses / m_terQryCalls;

	sprintf_s(buf,
		"==== QryTerrain performance information ====\n"
		"The function was called      : %d times.\n"
		"Road hint succesful          : %d times (%4.1f%%).\n"
		"Intersection hint successful : %d times (%4.1f%%).\n"
		"Cold searches                : %d times (%4.1f%%).\n",
		m_terQryCalls,
		m_terQryRoadHits, roadHitPerc,
		m_terQryInterHits, interHitPerc,
		misses, missPerc);

	desc = buf;

	if ( reset ) {
		m_terQryCalls = m_terQryRoadHits = m_terQryInterHits = 0;
	}
} // end of QryTerrainPerfCheck

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Returns the results of a SearchRectangle on the road piece quadtree.
//
// Remarks: This function is used by CRoadPos, so that a road position may be
// 	calculated from an (x,y,z) position.
//
// Arguments:
// 	Input:
// 		x1, y1 - lower left corner of the rectangle for which to search
// 		x2, y2 - upper right corner of the rectangle for which to search
// 	Output:
// 		result - set of IDs of the roadpieces which overlap the rectangle
//
// 	Returns: The number of roadpieces found to overlap with the rectangle
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::SearchRdPcQuadTree(double x1, double y1,
						  double x2, double y2,
						  vector<int>& result) const
{
	return m_rdPcQTree.SearchRectangle(x1, y1, x2, y2, result);
} // end of SearchRdPcQuadTree

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Returns the results of a SearchRectangle on the intersection quadtree.
//
// Remarks: This function is used by CRoadPos, so that an intersection
// 	position may be calculated from an (x,y,z) position.
//
// Arguments:
// 	Input:
// 		x1, y1 - lower left corner of the rectangle for which to search
// 		x2, y2 - upper right corner of the rectangle for which to search
// 	Output:
// 		result - set of IDs of the intersections which overlap the rectangle
//
// 	Returns: The number of intersections found to overlap with the rectangle
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::SearchIntrsctnQuadTree(double x1, double y1,
							  double x2, double y2,
							  vector<int>& result) const
{
	return m_intrsctnQTree.SearchRectangle(x1, y1, x2, y2, result);
} // end of SearchIntrsctnQuadTree

//////////////////////////////////////////////////////////////////////////////
// functions to transition the mode of a traj follower
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Transition the traj follower object to coupled obj mode.
//
// Remarks:
//
// Arguments:
// 	Input:
// 		child  - the cved id of the object to be transitioned
// 		parent - the cved id of the object that the child is to couple to.
//               -1 means use the parent id already stored in the object
//               state, which probably has been set during initialization
//               in HCSM
//      offset - the offset in position and rotation for the coupling.
//               NULL pointer means use the offset already stored in the
//               object state.
// 	Output:
//
// 	Returns: true if the transition is successful, false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::CoupledObjectMotion( int child, int parent, double offset[6] )
{
	int i;

	TObj* pO = BindObj( child );
	if ( pO->type != eCV_TRAJ_FOLLOWER ) // wrong type, not a traj follower!
		return false;

	if( (m_pHdr->frame & 1) == 0 )
	// even frame
	{
		if ( parent > 0 )
			pO->stateBufB.state.trajFollowerState.parentId = parent;
		if ( offset )
			for ( i=0; i<6; ++i )
				pO->stateBufB.state.trajFollowerState.offset[i] = offset[i];
		pO->stateBufB.state.trajFollowerState.prevMode = pO->stateBufB.state.trajFollowerState.curMode;
		pO->stateBufB.state.trajFollowerState.curMode = eCV_COUPLED_OBJ;
		pO->stateBufB.state.trajFollowerState.useAsAbsoluteValue = false;
		// so that by default at the next transition to free mode, initial pos, rot and
		// velocities will be calcuated from the condition at the moment of transition
	}
	else
	{
		if ( parent > 0 )
			pO->stateBufA.state.trajFollowerState.parentId = parent;
		if ( offset )
			for ( i=0; i<6; ++i )
				pO->stateBufA.state.trajFollowerState.offset[i] = offset[i];
		pO->stateBufA.state.trajFollowerState.prevMode = pO->stateBufB.state.trajFollowerState.curMode;
		pO->stateBufA.state.trajFollowerState.curMode = eCV_COUPLED_OBJ;
		pO->stateBufA.state.trajFollowerState.useAsAbsoluteValue = false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Transition the traj follower object to obj relative traj follower mode.
//
// Remarks:
//
// Arguments:
// 	Input:
// 		child  - the cved id of the object to be transitioned
// 		parent - the cved id of the object that the trajectory is relative to.
//               -1 means use the parent id already stored in the object
//               state, which probably has been set during initialization
//               in HCSM
//      offset - the offset in position and rotation for the relative
//               trajectory. NULL pointer means use the offset already stored
//               in the object state.
// 	Output:
//
// 	Returns: true if the transition is successful, false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::ObjRelTrajMotion( int child, int parent, double offset[6] )
{
	int i;

	TObj* pO = BindObj( child );
	if ( pO->type != eCV_TRAJ_FOLLOWER ) // wrong type, not a traj follower!
		return false;

	if( (m_pHdr->frame & 1) == 0 )
	// even frame
	{
		if ( parent > 0 )
			pO->stateBufB.state.trajFollowerState.parentId = parent;
		if ( offset )
			for ( i=0; i<6; ++i )
				pO->stateBufB.state.trajFollowerState.offset[i] = offset[i];
		pO->stateBufB.state.trajFollowerState.prevMode = pO->stateBufB.state.trajFollowerState.curMode;
		pO->stateBufB.state.trajFollowerState.curMode = eCV_OBJ_REL_TRAJ;
		pO->stateBufB.state.trajFollowerState.useAsAbsoluteValue = false;
	}
	else
	{
		if ( parent > 0 )
			pO->stateBufA.state.trajFollowerState.parentId = parent;
		if ( offset )
			for ( i=0; i<6; ++i )
				pO->stateBufA.state.trajFollowerState.offset[i] = offset[i];
		pO->stateBufA.state.trajFollowerState.prevMode = pO->stateBufB.state.trajFollowerState.curMode;
		pO->stateBufA.state.trajFollowerState.curMode = eCV_OBJ_REL_TRAJ;
		pO->stateBufA.state.trajFollowerState.useAsAbsoluteValue = false;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Transition the traj follower object to free motion mode.
//
// Remarks:
//
// Arguments:
// 	Input:
// 		child      - the cved id of the object to be transitioned
// 		initPosRot - initial position and rotation of the free motion object.
//                   NULL pointer means use the values already stored in the
//                   object class or calculate at real time.
//      initVel    - initial velocities of the free motion object.
//                   NULL pointer means use the values already stored in the
//                   object state or calculate at real time.
// 	Output:
//
// 	Returns: true if the transition is successful, false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::FreeObjectMotion( int child, double initPosRot[6], double initVel[6] )
{
	int i;

	TObj* pO = BindObj( child );
	if ( pO->type != eCV_TRAJ_FOLLOWER ) // wrong type, not a traj follower!
		return false;

	if( (m_pHdr->frame & 1) == 0 )
	// even frame
	{
		if ( initPosRot )
		{
			for ( i=0; i<6; ++i )
				pO->stateBufB.state.trajFollowerState.offset[i] = initPosRot[i];
				pO->stateBufB.state.trajFollowerState.useAsAbsoluteValue = true;
				// initial pos, rot and velocities are explicitly assigned, use
				// them as absolute values, instead of calculating them
		}
		if ( initVel )
			for ( i=0; i<6; ++i )
				pO->stateBufB.state.trajFollowerState.initVel[i] = initVel[i];
		pO->stateBufB.state.trajFollowerState.prevMode = pO->stateBufB.state.trajFollowerState.curMode;
		pO->stateBufB.state.trajFollowerState.curMode = eCV_FREE_MOTION;
	}
	else
	{
		if ( initPosRot )
		{
			for ( i=0; i<6; ++i )
				pO->stateBufA.state.trajFollowerState.offset[i] = initPosRot[i];
				pO->stateBufA.state.trajFollowerState.useAsAbsoluteValue = true;
		}
		if ( initVel )
			for ( i=0; i<6; ++i )
				pO->stateBufA.state.trajFollowerState.initVel[i] = initVel[i];
		pO->stateBufA.state.trajFollowerState.prevMode = pO->stateBufB.state.trajFollowerState.curMode;
		pO->stateBufA.state.trajFollowerState.curMode = eCV_FREE_MOTION;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////////
//	Private binding functions
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an object id, return pointer to object slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	object slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the object to bind
//
// Returns: a pointer to the cvTObj structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTObj *
CCved::BindObj(TObjectPoolIdx id) const
{
	if (id >= m_pHdr->objectStrgCount) {
		string msg;

		msg = "invalid object id in BindObj()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b  ofs;
	char   *pObjPool;
	cvTObj *pObj;

	ofs      = m_pHdr->objectOfs;
	pObjPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pObj     = reinterpret_cast<cvTObj *>(pObjPool);

	return pObj + id;
} // end of BindObj

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an object ref id, return pointer to object slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	object reference slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the object reference to bind
//
// Returns: a pointer to the cvTObjRef structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTObjRef*
CCved::BindObjRef(TObjRefPoolIdx id) const
{
	if ( id >= m_pHdr->objRefCount){
		string msg;
		msg = "invalid object reference in BindObjRef()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b		ofs;
	char*		pObjRefPool;
	cvTObjRef*	pObjRef;

	ofs            = m_pHdr->objRefOfs;
	pObjRefPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pObjRef        = reinterpret_cast<cvTObjRef*>(pObjRefPool);

	return pObjRef + id;
} // end of BindObjRef

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a repeated object id, return pointer to object slot in the memory
// 	block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	repeated object slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the repeated object to bind
//
// Returns: a pointer to the cvTRepObj structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTRepObj*
CCved::BindRepObj(TRepObjPoolIdx id) const
{
	if ( id >= m_pHdr->repObjCount){
		string msg;
		msg = "invalid repeated object index in BindRepObj()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b       ofs;
	char*       pRepObjPool;
	cvTRepObj*	pRepObj;

	ofs				= m_pHdr->repObjOfs;
	pRepObjPool		= reinterpret_cast<char *>(m_pHdr) + ofs;
	pRepObj			= reinterpret_cast<cvTRepObj*>(pRepObjPool);

	return pRepObj + id;
} // end of BindRepObj

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a object attribute id, return pointer to its slot in the memory
// 	block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	object attribute slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the object attribute to bind
//
// Returns: a pointer to the cvTObjAttr structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTObjAttr *
CCved::BindObjAttr(TObjAttrPoolIdx id) const
{
	cvTObj*		pObj    = BindObj( id );
	return &pObj->attr;
} // end of BindObjAttr

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an intersection id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	intersection slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the intersection to bind
//
// Returns: a pointer to the cvTIntrsctn structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTIntrsctn*
CCved::BindIntrsctn(TIntrsctnPoolIdx id) const
{
	if ( id >= m_pHdr->intrsctnCount ) {
		string msg;

		msg = "invalid intersection id in BindIntrsctn()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b			ofs;
	char*			pIntrsctnPool;
	cvTIntrsctn*	pIntrsctn;

	ofs           = m_pHdr->intrsctnOfs;
	pIntrsctnPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pIntrsctn     = reinterpret_cast<cvTIntrsctn *>(pIntrsctnPool);

	return pIntrsctn + id;
} // end of BindIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an elevation map id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	elevation map slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the elevation map to bind
//
// Returns: a pointer to the cvTElevMap structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTElevMap *
CCved::BindElevMap(TElevMapPoolIdx id) const
{
	if ( id >= m_pHdr->elevMapCount ) {
		string msg;

		msg = "invalid elevatio nmap id in BindElevMap()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b         ofs;
	char        *pElevMapPool;
	cvTElevMap  *pElevMap;

	ofs           = m_pHdr->elevMapOfs;
	pElevMapPool  = reinterpret_cast<char *>(m_pHdr) + ofs;
	pElevMap      = reinterpret_cast<cvTElevMap *>(pElevMapPool);

	return pElevMap + id;
} // end of BindElevMap

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an road piece id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	road piece slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the road piece to bind
//
// Returns: a pointer to the cvTRoadPiece structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTRoadPiece *
CCved::BindRoadPiece(TRoadPiecePoolIdx id) const
{
	if ( id >= m_pHdr->roadPieceCount ) {
		string msg;

		msg = "invalid road piece id in BindRoadPiece()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b         ofs;
	char*         pPool;
	cvTRoadPiece* pRoadPiece;

	ofs           = m_pHdr->roadPieceOfs;
	pPool         = reinterpret_cast<char *>(m_pHdr) + ofs;
	pRoadPiece    = reinterpret_cast<cvTRoadPiece *>(pPool);

	return pRoadPiece + id;
} // end of BindRoadPiece

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an road id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	road slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the road to bind
//
// Returns: a pointer to the cvTRoad structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTRoad*
CCved::BindRoad(TRoadPoolIdx id) const
{
	if ( id >= m_pHdr->roadCount ) {
		string msg;
		msg = "invalid road id in BindRoad()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b		ofs;
	char*		pPool;
	cvTRoad*	pRoad;

	ofs		= m_pHdr->roadOfs;
	pPool	= reinterpret_cast<char *>(m_pHdr) + ofs;
	pRoad	= reinterpret_cast<cvTRoad*>(pPool);

	return pRoad + id;
} // end of BindRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description: Given a lane id, return pointer to its slot in the
//   memory block.
//
// Remarks: This function is used internally by CVED to facilitate
//   accessing lane slots given their id.  The function performs
//   debug checks.
//
// Arguments:
//   id - Identifier of the lane to bind.
//
// Returns: A pointer to the cvTLane structure with the given id.
//
//////////////////////////////////////////////////////////////////////////////
cvTLane*
CCved::BindLane(TLanePoolIdx id) const
{
	if ( id >= m_pHdr->laneCount ) {
		string msg;
		msg = "invalid lane id in BindLane()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b		ofs;
	char*		pPool;
	cvTLane*	pLane;

	ofs		= m_pHdr->laneOfs;
	pPool	= reinterpret_cast<char *>(m_pHdr) + ofs;
	pLane	= reinterpret_cast<cvTLane*>(pPool);

	return pLane + id;
} // end of BindLane

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a control point id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	control point slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the control point to bind
//
// Returns: a pointer to the cvTCntrlPnt structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTCntrlPnt*
CCved::BindCntrlPnt(TLongCntrlPntPoolIdx id) const
{
	if ( id >= m_pHdr->longitCntrlCount ) {
		string msg;
		msg = "invalid cntrlPnt id in BindCntrlPnt()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b			ofs;
	char*			pPool;
	cvTCntrlPnt*	pCntrlPnt;

	ofs			= m_pHdr->longitCntrlOfs;
	pPool		= reinterpret_cast<char *>(m_pHdr) + ofs;
	pCntrlPnt	= reinterpret_cast<cvTCntrlPnt*>(pPool);

	return pCntrlPnt + id;
} // end of BindCntrlPnt

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a lateral control point id, return pointer to its slot in the memory
// 	block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	lateral control point slots given their id.  The function performs debug
// 	checks.
//
// Arguments:
// 	id - identifier of the lateral control point to bind
//
// Returns: a pointer to the cvTLatCntrlPnt structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTLatCntrlPnt*
CCved::BindLatCntrlPnt(TLatCntrlPntPoolIdx id) const
{
	if ( id >= m_pHdr->latCntrlPntCount ) {
		string msg;
		msg = "invalid cntrlPnt id in BindLatCntrlPnt()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b			ofs;
	char*			pPool;
	cvTLatCntrlPnt*	pCntrlPnt;

	ofs			= m_pHdr->latCntrlPntOfs;
	pPool		= reinterpret_cast<char *>(m_pHdr) + ofs;
	pCntrlPnt	= reinterpret_cast<cvTLatCntrlPnt*>(pPool);

	return pCntrlPnt + id;
} // end of BindLatCntrlPnt

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a elevation post id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	elevation post slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the elevation post to bind
//
// Returns: a pointer to the cvTElevPost structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTElevPost *
CCved::BindElevPost(TElevPostPoolIdx id) const
{
	if ( id >= m_pHdr->elevPostCount ) {
		string msg;

		msg = "invalid elevation post id in BindElevPost()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b         ofs;
	char        *pElevPostPool;
	cvTElevPost *pElevPost;

	ofs           = m_pHdr->elevPostOfs;
	pElevPostPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pElevPost     = reinterpret_cast<cvTElevPost *>(pElevPostPool);

	return pElevPost + id;
} // end of BindElevPost

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a border segment id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	border segment slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the border segment to bind
//
// Returns: a pointer to the cvTBorderSeg structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTBorderSeg *
CCved::BindBorderSeg(TBorderSegPoolIdx id) const
{
	if ( id >= m_pHdr->borderCount ) {
		string msg;

		msg = "invalid border segment id in BindBorderSeg()";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b          ofs;
	char         *pBorderSegPool;
	cvTBorderSeg *pBorderSeg;

	ofs            = m_pHdr->borderOfs;
	pBorderSegPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pBorderSeg     = reinterpret_cast<cvTBorderSeg *>(pBorderSegPool);

	return pBorderSeg + id;
} // end of BindBorderSeg

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a dynamic object ref id, return pointer to its slot in the memory
// 	block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	dynamic object ref slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the dynamic object ref to bind
//
// Returns: a pointer to the cvTDynObjRef structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTDynObjRef*
CCved::BindDynObjRef(TObjectPoolIdx id) const
{
	if ( id >= m_pHdr->dynObjRefCount ) {
		string msg("invalid dynamic object reference id in BindDynObjRef()");
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b         ofs;
	char         *pDynObjRefPool;
	cvTDynObjRef *pDynObjRef;

	ofs            = m_pHdr->dynObjRefOfs;
	pDynObjRefPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pDynObjRef     = reinterpret_cast<cvTDynObjRef *>(pDynObjRefPool);

	return pDynObjRef + id;
} // end of BindDynObjRef

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given a road reference id, return pointer to its slot in the memory block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	road reference slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the road reference to bind
//
// Returns: a pointer to the cvTRoadRef structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTRoadRef*
CCved::BindRoadRef(TRoadPoolIdx id) const
{
	if ( id >= m_pHdr->roadRefCount ) {
		string msg("invalid road reference id in BindRoadRef()");
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b        ofs;
	char        *pRoadRefPool;
	cvTRoadRef	*pRoadRef;

	ofs          = m_pHdr->roadRefOfs;
	pRoadRefPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pRoadRef     = reinterpret_cast<cvTRoadRef *>(pRoadRefPool);

	return pRoadRef + id;
} // end of BindRoadRef

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Given an intersection ref id, return pointer to its slot in the memory
// 	block.
//
// Remarks: This function is used internally by CVED to facilitate accessing
// 	intersection ref slots given their id.  The function performs debug checks.
//
// Arguments:
// 	id - identifier of the intersection ref to bind
//
// Returns: a pointer to the cvTIntrsctnRef structure with the given id
//
//////////////////////////////////////////////////////////////////////////////
cvTIntrsctnRef*
CCved::BindIntrsctnRef(TIntrsctnPoolIdx id) const
{
	if ( id >= m_pHdr->intrsctnRefCount ) {
		string msg("invalid intrsctn reference id in BindIntrsctnRef()");
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}

	TU32b			ofs;
	char			*pIntrsctnRefPool;
	cvTIntrsctnRef	*pIntrsctnRef;

	ofs				 = m_pHdr->intrsctnRefOfs;
	pIntrsctnRefPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	pIntrsctnRef     = reinterpret_cast<cvTIntrsctnRef *>(pIntrsctnRefPool);

	return pIntrsctnRef + id;
} // end of BindIntrsctnRef

/////////////////////////////////////////////////////////////////////////////
//
// Description: The function find object ids of the traffic lights on the
//  given intersection
//
// Remark: This function goes into every corridor of the intersection, and
//  again goes into every holdof in the corridor to find all the traffic
//  lights on the given intersection, instead of finding them geometrically
//
// Arguments:
//  intrsctn - const reference of type CIntrsctn as the intersection querried
//  intrsctnId - id of the intersection qerried
//  objid - a vector of int storing the obj ids of trafficlights on the
//          intersection querried
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::QryTrfLghtOnIntrsctn(const CIntrsctn& intrx, vector<int>& tId)
{
	TU32b ofs;
	char* pool;

	tId.clear();
	cvTIntrsctn* pIntrx = BindIntrsctn(intrx.GetId());
	int      cr     = pIntrx->crdrIdx;
	int      lastCr = cr + pIntrx->numOfCrdrs;

	ofs = m_pHdr->crdrOfs;
	pool = reinterpret_cast<char *>(m_pHdr) + ofs;
	cvTCrdr* pCrdr = reinterpret_cast<cvTCrdr *>(pool);
	pCrdr = pCrdr + cr;

	for (; cr < lastCr; cr++){
		int         hl    = pCrdr->hldOfsIdx;
		int        lastHl = hl + pCrdr->numHldOfs;

		ofs = m_pHdr->hldOfs;
		pool = reinterpret_cast<char *>(m_pHdr) + ofs;
		cvTHldOfs* pHld = reinterpret_cast<cvTHldOfs *>(pool);
		pHld += hl;
		for (; hl < lastHl; hl++){
			if (pHld->reason & eHOLD_TLIGHT){
				tId.push_back(pHld->objIdx);
			}
			pHld++;
		}
		pCrdr++;
	}
}

void
CCved::QryTrfLghtOnIntrsctn(const string& name, vector<int>& tId)
{
	CIntrsctn intrx(*this, name);
	QryTrfLghtOnIntrsctn(intrx, tId);
}

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	LockObjectPool and UnlockObjectPool implement a blocking lock to allow
// 	mutual exclusion when accessing the object array within the memory block.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::LockObjectPool(void)
{} // end of LockObjectPool

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	LockObjectPool and UnlockObjectPool implement a blocking lock to allow
// 	mutual exclusion when accessing the object array within the memory block.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::UnlockObjectPool(void)
{} // end of UnlockObjectPool
///////////////////////////////////////////////////////////////////////////////
///\brief
///   Gets a log of sol2 parse errors, some of these errors maybe non-critical
///\param[in] string to hold the error log
///\return false if a parse error was found, true if log is empty
///
///////////////////////////////////////////////////////////////////////////////
bool
CCved::GetSol2ParseLog(string& str) const{
	str = m_sSol.GetParseLog();
	return m_sSol.GetHasParseErrors();
}
///////////////////////////////////////////////////////////////////////////////
///
/// Description:
/// 	A debugging function that dumps internal data structures
///
/// Remarks:
///
/// Arguments:
///
/// Returns: void
///
///////////////////////////////////////////////////////////////////////////////
void
CCved::Verify(void)
{
	if ( m_state != eACTIVE ) {
		gout << "Verify was called but state wasn't Active" << endl;
	}

	TU32b i;

	gout << "In Verify " << endl;

	/////////// dump character pool
	gout << " ---- Contents of the char pool -----" << endl;
	gout << "At offset " << m_pHdr->charOfs << ", size=" << m_pHdr->charStrgCount
				<< ", use=" << m_pHdr->charCount;
	char *pPool = ((char *)m_pHdr) + m_pHdr->charOfs;

	for (i=0; i<m_pHdr->charCount; i++) {
		if ( i==0 || (i % 8)==0 )
			gout << endl << "Ofs" << i << "\t";

		gout << "'" << pPool[i] << "'";
		gout.width(4);
		gout << (int)pPool[i] << " ";
	}
	gout << endl << endl;

	////////////// dump road pool
	gout << " ---- Contents of the road pool -----" << endl;
	gout << "At offset " << m_pHdr->roadOfs << ", size="
				<< m_pHdr->roadCount << endl;

	TRoad *pRoads;
	pRoads = (TRoad *) ( ((char *)m_pHdr) + m_pHdr->roadOfs);
	for (i=1; i<m_pHdr->roadCount; i++) {
		gout << "Road " << i << endl;
		gout << "\tmyId             = " << (int)pRoads[i].myId << endl;
		gout << "\tcntrlPntIdx      = " << pRoads[i].cntrlPntIdx << endl;
		gout << "\tnumCntrlPnt      = " << pRoads[i].numCntrlPnt << endl;
		gout << "\tnameIdx          = " << pRoads[i].nameIdx << endl;
		gout << "\tsrcIntrsctn      = " << pRoads[i].srcIntrsctnIdx << endl;
		gout << "\tdstIntrsctn      = " << pRoads[i].dstIntrsctnIdx << endl;
		gout << "\troadLengthLinear = " << pRoads[i].roadLengthLinear << endl;
		gout << "\troadLengthSpline = " << pRoads[i].roadLengthSpline << endl;
		gout << "\tattrIdx          = " << pRoads[i].attrIdx << endl;
		gout << "\tnumAttr          = " << pRoads[i].numAttr << endl;
		gout << "\trepObjIdx        = " << pRoads[i].repObjIdx << endl;
		gout << "\tnumRepObj        = " << pRoads[i].numRepObj << endl;
		gout << "\tnumOfLanes       = " << pRoads[i].numOfLanes << endl;
	}
	gout << endl;

	//////////// dump intersection pool
	gout << " ----- Contents of intersection pool -----" << endl;
	gout << "At offset " << m_pHdr->intrsctnOfs << ", size="
					<< m_pHdr->intrsctnCount << endl;

	TIntrsctn *pIntr;
	pIntr = (TIntrsctn *) ( ((char *)m_pHdr) + m_pHdr->intrsctnOfs);

	for (i=1; i<m_pHdr->intrsctnCount; i++) {
		gout << "Intersection " << i << endl;
		gout << "\tmyId           = " << (int) pIntr[i].myId << endl;
		gout << "\tnameIdx        = " << (int) pIntr[i].nameIdx << endl;
		gout << "\tnumOfRoads     = " << (int) pIntr[i].numOfRoads << endl;
		gout << "\tadjacentRoadId = ";
		for (int j=0; j<pIntr[i].numOfRoads; j++)
			gout << (int) pIntr[i].adjacentRoadId[j] << " ";
		gout << endl;
		gout << "\tnumBorderSeg   = " << (int) pIntr[i].numBorderSeg << endl;
		gout << "\tborderSegIdx   = " << (int) pIntr[i].borderSegIdx << endl;
	}

	////////////// dump control point pool
	cvTCntrlPnt *pCntr;
	pCntr = (cvTCntrlPnt *) ( ((char *)m_pHdr) + m_pHdr->longitCntrlOfs);

	gout << " ----- Contents of longitudinal control point pool -----" << endl;
	gout << "At offset " << m_pHdr->longitCntrlOfs << ", size ="
					<< m_pHdr->longitCntrlCount << endl;
	for (i=1; i<m_pHdr->longitCntrlCount; i++ ) {
		gout << "Control point " << i << endl;
		gout << "\tphysWid, logWid      = " << pCntr[i].physicalWidth
				<< ", " << pCntr[i].logicalWidth << endl;
		gout << "\tlatCntrlPntIdx       = " << pCntr[i].latCntrlPntIdx << endl;
		gout << "\tnumLatCntrlPnt       = " << pCntr[i].numLatCntrlPnt << endl;
		gout << "\tlaneIdx              = " << pCntr[i].laneIdx << endl;
		gout << "\tnumLanes             = " << pCntr[i].numLanes << endl;
		gout << "\tlocation             = " << pCntr[i].location << endl;
		gout << "\tnormal               = " << pCntr[i].normal << endl;
		gout << "\ttangVecCubic         = " << pCntr[i].tangVecCubic << endl;
		gout << "\trightVecCubic        = " << pCntr[i].rightVecCubic << endl;
		gout << "\ttangVecLinear        = " << pCntr[i].tangVecLinear << endl;
		gout << "\trightVecLinear       = " << pCntr[i].rightVecLinear << endl;
		gout << "\thermite              = " << pCntr[i].hermite << endl;
		gout << "\tcummulativeLinDist   = " << pCntr[i].cummulativeLinDist << endl;
		gout << "\tcummulativeCubicDist = " << pCntr[i].cummulativeCubicDist<<endl;
		gout << "\tdistToNextLinear     = " << pCntr[i].distToNextLinear << endl;
		gout << "\tdistToNExtCubic      = " << pCntr[i].distToNextCubic << endl;
		gout << "\tsurfacePropIdx       = " << pCntr[i].surfacePropIdx << endl;
		gout << "\tsn                   = " << pCntr[i].sn << endl;
		gout << "\tst                   = " << pCntr[i].st << endl;
		gout << "\tradius               = " << pCntr[i].radius << endl;
		gout << "\tnameIdx              = " << pCntr[i].nameIdx << endl;
	}

	/////////////// dump corridords

	////////////// dump elevation maps
	gout << "Elevation map pool at offset " << m_pHdr->elevMapOfs << endl;
	for (i=1; i<m_pHdr->elevMapCount; i++) {
		TElevMap  *pMap = BindElevMap(i);

		gout << "Elevation map " << i << endl;
		gout << "   X1, Y1 = " << pMap->x1 << "," << pMap->y1
				<< " res = " << pMap->res
				<< " rows/cols = " << pMap->numRows << ", "
				<< pMap->numCols << "  numPosts = " << pMap->numPosts
				<< endl;
		gout << "  Elevation posts: " << endl;

		int j;
		for (j=0; j < pMap->numPosts; j++) {
			TElevPost *pPost = BindElevPost( pMap->postIdx + j);

			gout << "     z = " << pPost->z << ", material="
				<< pPost->flags << endl;
		}
	}

	////// dump object pool
	gout << "Object pool at offset " << m_pHdr->objectOfs << endl;
	for (i=0; i<m_pHdr->objectStrgCount; i++) {
		cvTObj *pO = BindObj(i);

		gout << "Object " << i << ", at addr " << (int)pO << endl;
		gout << "  myId      = " << (int)pO->myId << endl;
		gout << "  type      = " << (int)pO->type << endl;
		gout << "  phase     = " << (int)pO->phase << endl;
		gout << "  name      = " << pO->name << endl;

		gout << endl;
	}
} // end of Verify


//////////////////////////////////////////////////////////////////////////////
///
/// Description: Given a composite object's id, this funtion returns its current
///  option settings.
///
/// Remarks: Returns false if called for a dynamic object or if the object
///  is not alive or is not a composite object.
///
/// Arguments:
///   objId - The object's CVED id.
///   option - (output) The objects current option.
///
/// Returns: A boolean indicating if the operation was successful.
///
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetObjCompositeOptions( int objId, int* pOptions ) const
{
	//
	// Make sure it's a valid static object.
	//
	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || pO->phase != eALIVE ) return false;

	//
	// Make sure the object is composite
	//
	if (pO->type != eCV_COMPOSITE_SIGN) return false;

	if( (m_pHdr->frame & 1) == 0 ) {
		memcpy(pOptions, pO->stateBufA.state.compositeSignState.childrenOpts,
			cMAX_PIECES_IN_COMPOSITE_SIGN * sizeof(int));
	}
	else {
		memcpy(pOptions, pO->stateBufB.state.compositeSignState.childrenOpts,
			cMAX_PIECES_IN_COMPOSITE_SIGN * sizeof(int));
	}

	return true;
}


///////////////////////////////////////////////////////////////////////////////
///
/// Description: This function sets the option on a static
///  object.
///
/// Remarks: Returns false if called for a dynamic object or if the object
///  is not alive or the option is invalid.
///
/// Arguments:
///   objId - The object's CVED id.
///   option - The option to set.
///
/// Returns: A boolean indicating if the operation was successful.
///
///////////////////////////////////////////////////////////////////////////////
bool
CCved::SetObjCompositeOptions( int objId, const int* pOptions )
{
	//
	// Make sure it's a valid static object.
	//
	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || pO->phase != eALIVE ) return false;

	//
	// Make sure the object is composite
	//
	if (pO->type != eCV_COMPOSITE_SIGN) return false;

	int n;
	if( (m_pHdr->frame & 1) == 0 ) {
		for (n = 0; n < cMAX_PIECES_IN_COMPOSITE_SIGN; n++) {
			if (pOptions[n] >= 0) {
				pO->stateBufB.state.compositeSignState.childrenOpts[n] = pOptions[n];
			}
		}
	}
	else {
		for (n = 0; n < cMAX_PIECES_IN_COMPOSITE_SIGN; n++) {
			if (pOptions[n] >= 0) {
				pO->stateBufA.state.compositeSignState.childrenOpts[n] = pOptions[n];
			}
		}
	}

	pO->changedFlag = 0xFF;
	return true;
}


///////////////////////////////////////////////////////////////////////////////
///
/// Description: Given an object's id, this funtion returns its current
///  option setting.
///
/// Remarks: Returns false if called for a dynamic object or if the object
///  is not alive.
///
/// Arguments:
///   objId - The object's CVED id.
///   option - (output) The objects current option.
///
/// Returns: A boolean indicating if the operation was successful.
///
///////////////////////////////////////////////////////////////////////////////
bool
CCved::GetObjOption( int objId, int& option ) const
{
	//
	// Make sure it's a valid static object.
	//
//	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || pO->phase != eALIVE ) return false;

	option = pO->activeOption;
	return true;
}


///////////////////////////////////////////////////////////////////////////////
///
/// Description: This function sets the option on a static
///  object.
///
/// Remarks: Returns false if called for a dynamic object or if the object
///  is not alive or the option is invalid.
///
/// Arguments:
///   objId - The object's CVED id.
///   option - The option to set.
///
/// Returns: A boolean indicating if the operation was successful.
///
///////////////////////////////////////////////////////////////////////////////
bool
CCved::SetObjOption( int objId, int option )
{
	//
	// Make sure it's a valid static object.
	//
//	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || ((pO->phase != eALIVE) && (pO->phase != eBORN)) ) return false;
	const CSolObj* cObj = m_sSol.GetObj( pO->attr.solId );
	bool invalidOption = (
			option < 0 || cObj == NULL ||
			option >= cObj->GetNumOptions()
			);
	if( invalidOption ) return false;

	// else, everything must be valid.
	pO->activeOption = option;
	pO->changedFlag = 0xFF;

//	if ( objId < cNUM_DYN_OBJS )
//		printf("%d set option step4, option = %d\n", objId, option);
	return true;
}


//////////////////////////////////////////////////////////////////////////////
///
///\brief This function sets the visual state for an object
///
///\par Remarks:
///	 Returns false if called if the object is not alive.
///
///\par Arguments:
///   objId - The object's CVED id.
///   option - The option to set.
///	  instant -Change both A and B state at once.(sould only be done for static objs)
///\return A boolean indicating if the operation was successful.
///
//////////////////////////////////////////////////////////////////////////////
bool
CCved::SetObjAudioState( int objId, short state, bool instant )
{
	//
	// Make sure it's a valid static object.
	//
//	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || ((pO->phase != eALIVE) && (pO->phase != eBORN)) ) return false;


	if (!instant){
		if( ( m_pHdr->frame & 1 ) == 0 )
		{
			// even frame
			pO->stateBufB.state.anyState.audioState = state;
		}
		else
		{
			// odd frame
			pO->stateBufA.state.anyState.audioState = state;
		}
		pO->changedFlag = 0xFF;
	}else{
			pO->stateBufB.state.anyState.audioState = state;
			pO->stateBufA.state.anyState.audioState = state;
	}

//	if ( objId < cNUM_DYN_OBJS )
//		printf("%d set option step4, option = %d\n", objId, option);
	return true;
}


//////////////////////////////////////////////////////////////////////////////
///
///\brief This function sets the visual state for an object
///
///\par Remarks:
///	 Returns false if called if the object is not alive.
///
///\par Arguments:
///   objId - The object's CVED id.
///   option - The option to set.
///
///\return A boolean indicating if the operation was successful.
///
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetObjAudioState( int objId, short &state) const
{
	//
	// Make sure it's a valid static object.
	//
//	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || ((pO->phase != eALIVE) && (pO->phase != eBORN)) )
		return false;


	if( ( m_pHdr->frame & 1 ) == 0 )
	{
		// even frame
		state = pO->stateBufB.state.anyState.audioState;
	}
	else
	{
		// odd frame
		state = pO->stateBufA.state.anyState.audioState;
	}

	return true;
}
//////////////////////////////////////////////////////////////////////////////
///
///\brief This function sets the visual state for an object
///
///\par Remarks:
///	 Returns false if called if the object is not alive.
///
///\par Arguments:
///   objId - The object's CVED id.
///   option - The option to set.
///   instant -Change both A and B state at once.(sould only be done for static objs)
///\return A boolean indicating if the operation was successful.
///
//////////////////////////////////////////////////////////////////////////////
bool
CCved::SetObjVisualState( int objId, short state, bool instant )
{
	//
	// Make sure it's a valid static object.
	//
//	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || ((pO->phase != eALIVE) && (pO->phase != eBORN)) ) return false;


	if (!instant){
		if( ( m_pHdr->frame & 1 ) == 0 )
		{
			// even frame
			pO->stateBufB.state.anyState.visualState = state;
		}
		else
		{
			// odd frame
			pO->stateBufA.state.anyState.visualState = state;
		}
	}else{
		pO->stateBufB.state.anyState.visualState = state;
		pO->stateBufA.state.anyState.visualState = state;
	}
	pO->changedFlag = 0xFF;

//	if ( objId < cNUM_DYN_OBJS )
//		printf("%d set option step4, option = %d\n", objId, option);
	return true;
}


//////////////////////////////////////////////////////////////////////////////
///
///\brief This function sets the visual state for an object
///
///\par Remarks:
///	 Returns false if called if the object is not alive.
///
///\par Arguments:
///   objId - The object's CVED id.
///   option - The option to set.
///
///\return A boolean indicating if the operation was successful.
///
//////////////////////////////////////////////////////////////////////////////
bool
CCved::GetObjVisualState( int objId, short &state) const
{
	//
	// Make sure it's a valid static object.
	//
//	if( objId < cNUM_DYN_OBJS ) return false;

	//
	// Make sure that the static object is alive.
	//
	TObj* pO = BindObj( objId );
	if( !pO || ((pO->phase != eALIVE) && (pO->phase != eBORN)) )
		return false;


	if( ( m_pHdr->frame & 1 ) == 0 )
	{
		// even frame
		state = pO->stateBufB.state.anyState.visualState;
	}
	else
	{
		// odd frame
		state = pO->stateBufA.state.anyState.visualState;
	}

	return true;
}
///////////////////////////////////////////////////////////////////////////////
///
/// Description:
///  This function set an attribute to a road.
///
/// Remarks:
///  This function will set an attribute to a road. Only
///  cCV_LRI_ROAD_ATTR_EXTRA number of attributes are allowed to
///  be set to a road after lri compilation. if the function is
///  to be called for more than cCV_LRI_ROAD_ATTR_EXTRA different
///  attributes, it will return false
///
/// Arguments:
///  rd - road to be set attribute to
///  attrId - attribute id defined in cveddecl.h
///  value1 - (optional) value1 of the attribute
///  value2 - (optional) value2 of the attribute
///  from - (optional) starting distance along the road of the attribute
///  to - (optional) ending distance along the road of the attribute
///  landMask - (optional) lane mask of the attribute
///
/// Return:
///  true if setup is successful
///  false otherwise
///
///////////////////////////////////////////////////////////////////////////////

bool
CCved::SetRoadAttr(
			const CRoad& rd,
			int          attrId,
			double        value1,
			double        value2,
			double        from,
			double        to,
			int          laneMask)
{
	if (!rd.IsValid())
		return false;

	cvTRoad*	pRoad = BindRoad(rd.GetId());

	TU32b       ofs = m_pHdr->attrOfs;
	char*       pPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	cvTAttr*    pAttr = (reinterpret_cast<cvTAttr*>(pPool)) + pRoad->attrIdx;

	int i = 0;
	for (; i < cCV_LRI_ROAD_ATTR_EXTRA; i++, pAttr++) {
		if ((pAttr->myId == attrId) || (pAttr->myId == 0)){
			pAttr->myId = attrId;
			pAttr->value1 = value1;
			pAttr->value2 = value2;
			pAttr->from = from;
			pAttr->to = to;
			pAttr->laneMask = laneMask;
			return true;
		}
	}


	return false;
}


bool
CCved::SetRoadAttr(
			const string& rdName,
			int           attrId,
			double         value1,
			double         value2,
			double         from,
			double         to,
			int           laneMask)
{
	CRoad rd = GetRoad(rdName);
	return SetRoadAttr(rd, attrId, value1, value2, from, to, laneMask);
}

///////////////////////////////////////////////////////////////////////////////
///
/// Description:
///  This function set an attribute to an intersection.
///
/// Remarks:
///  This function will set an attribute to an intersection. Only
///  cCV_LRI_INTRSCTN_ATTR_EXTRA number of attributes are allowed to
///  be set to an intersection after lri compilation. if the function is
///  to be called for more than cCV_LRI_INTRSCTN_ATTR_EXTRA different
///  attributes, it will return false
///
/// Arguments:
///  intrsctn - intersection to be set attribute to
///  intrsctnName - name of the intersection to be set attribute to
///  attrId - attribute id defined in cveddecl.h
///  value1 - (optional) value1 of the attribute
///  value2 - (optional) value2 of the attribute
///  from - (optional) starting distance along the road of the attribute
///  to - (optional) ending distance along the road of the attribute
///
/// Return:
///  true if setup is successful
///  false otherwise
///
///////////////////////////////////////////////////////////////////////////////
bool
CCved::SetIntrsctnAttr(
			const CIntrsctn& intrsctn,
			int          attrId,
			double        value1,
			double        value2,
			double        from,
			double        to)
{
	if (!intrsctn.IsValid())
		return false;

	cvTIntrsctn*	pI = BindIntrsctn(intrsctn.GetId());

	TU32b       ofs = m_pHdr->attrOfs;
	char*       pPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	cvTAttr*    pAttr = (reinterpret_cast<cvTAttr*>(pPool)) + pI->attrIdx;

	int i = 0;
	for (; i < cCV_LRI_INTRSCTN_ATTR_EXTRA; i++, pAttr++) {
		if ((pAttr->myId == attrId) || (pAttr->myId == 0)){
			pAttr->myId = attrId;
			pAttr->value1 = value1;
			pAttr->value2 = value2;
			pAttr->from = from;
			pAttr->to = to;
			pAttr->laneMask = -1;
			return true;
		}
	}

	return false;
}


bool
CCved::SetIntrsctnAttr(
			const string& intrsctnName,
			int           attrId,
			double         value1,
			double         value2,
			double         from,
			double         to)
{
	CIntrsctn intrsctn = GetIntersection(intrsctnName);
	return SetIntrsctnAttr(intrsctn, attrId, value1, value2, from, to);
}


///////////////////////////////////////////////////////////////////////////////
///
/// Description:
///  This function set an attribute to a crdr.
///
/// Remarks:
///  This function will set an attribute to a crdr. Only
///  cCV_LRI_INTRSCTN_ATTR_EXTRA number of attributes are allowed to
///  be set to a corridor after lri compilation. if the function is
///  to be called for more than cCV_LRI_CRDR_ATTR_EXTRA different
///  attributes, it will return false
///
/// Arguments:
///  crdr - corridor to be set attribute to
///  crdrId - relative id of corridor to be set attribute to
///  intrsctn - intrsctn where the crdr to be set attribute to
///  intrsctnName - name of the intrsctn where the crdr to be set attribute to
///  attrId - attribute id defined in cveddecl.h
///  value1 - (optional) value1 of the attribute
///  value2 - (optional) value2 of the attribute
///  from - (optional) starting distance along the road of the attribute
///  to - (optional) ending distance along the road of the attribute
///
/// Return:
///  true if setup is successful
///  false otherwise
///
///////////////////////////////////////////////////////////////////////////////
bool
CCved::SetCrdrAttr(
			const CCrdr& crdr,
			int          attrId,
			double        value1,
			double        value2,
			double        from,
			double        to)
{
	if (!crdr.IsValid())
		return false;

    char*       pOfs = reinterpret_cast<char*> (m_pHdr) + m_pHdr->crdrOfs;
    cvTCrdr* pC = (reinterpret_cast<cvTCrdr*>(pOfs)) + crdr.GetId();

	TU32b       ofs = m_pHdr->attrOfs;
	char*       pPool = reinterpret_cast<char *>(m_pHdr) + ofs;
	cvTAttr*    pAttr = (reinterpret_cast<cvTAttr*>(pPool)) + pC->attrIdx;

	int i = 0;
	for (; i < cCV_LRI_CRDR_ATTR_EXTRA; i++, pAttr++) {
		if ((pAttr->myId == attrId) || (pAttr->myId == 0)){
			pAttr->myId = attrId;
			pAttr->value1 = value1;
			pAttr->value2 = value2;
			pAttr->from = from;
			pAttr->to = to;
			return true;
		}
	}

	return false;
}

bool
CCved::SetCrdrAttr(
			const CIntrsctn& intrsctn,
			int          crdrId,
			int          attrId,
			double        value1,
			double        value2,
			double        from,
			double        to)
{
	if (!intrsctn.IsValid())
		return false;

	cvTIntrsctn*	pI = BindIntrsctn(intrsctn.GetId());

	if ((crdrId < 0) || ((TU32b)crdrId >= pI->numOfCrdrs)){
		return false;
	}
	vector<CCrdr> crdrVec;
	intrsctn.GetAllCrdrs(crdrVec);
	return SetCrdrAttr(crdrVec[crdrId], attrId, value1, value2, from, to);
}

bool
CCved::SetCrdrAttr(
			const string& intrsctnName,
			int          crdrId,
			int          attrId,
			double        value1,
			double        value2,
			double        from,
			double        to)
{
	CIntrsctn intrsctn = GetIntersection(intrsctnName);
	return SetCrdrAttr(intrsctn, crdrId, attrId, value1, value2, from, to);
}

///////////////////////////////////////////////////////////////////////////////
///
/// Description:	This function creates a list of objects in front of the
///   specified object in the parameter.
///
/// Remarks:
///
/// Arguments:
///   ownerObjId - Cved id of the owner object.
///   aPath   - The current path of the specified object in the parameter.
///   roadPos - The current roadPos of the specified object in the parameter.
///   maxObjs - The maximum number of objects the user is interested in
///             looking at.
///   fwdObjs - (output) Contains the objects sorted by distance.
///
/// Returns: void
///
///////////////////////////////////////////////////////////////////////////////
void
CCved::BuildFwdObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& fwdObjs
			)
{

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
	gout << "[" << ownerObjId << "] == BuildFwdObjList ======================";
	gout << endl;
	gout << "The maximum number of objs is: " << maxObjs << endl;
#endif

	//
	// Error checking.
	//
	bool invalidPath = !path.IsValid() || path.Size() <= 0;
	if( invalidPath )
	{

		cerr << "CCved::BuildFwdObjList: invalid path....exit" << endl;
		return;

	}

#if 0
	gout << "  input path [size = " << path.Size() << "]" << endl;
	gout << path << endl;
#endif

	//
	// Initialize variables.
	//
	double ownerDist = -101.0;
	double distTraveledSoFar = 0.0;
	bool reachedFirstValidPathPoint = false;

	//
	// Iterate through all of the path points to build the forward
	// object list.
	//
	CPath::cTPathIterator pathItr;
	for( pathItr = path.Begin(); pathItr != path.End(); pathItr++ )
	{

		//
		// Skip all path points until reaching the one that is
		// the same as the given road position.
		//
		bool samePathPointAsOwner = false;
		if ( !reachedFirstValidPathPoint )
		{
			if ( path.GetPoint(pathItr).IsRoad() )
			{
				reachedFirstValidPathPoint = (
							roadPos.IsRoad() &&
							roadPos.GetRoad() == path.GetPoint(pathItr).GetRoad()
							);
			}
			else
			{
				reachedFirstValidPathPoint = (
							!roadPos.IsRoad() &&
							roadPos.GetIntrsctn().GetId() ==
							path.GetPoint(pathItr).GetIntrsctn().GetId()
							);
			}
			samePathPointAsOwner = reachedFirstValidPathPoint;

			//
			// Get the owner's distance from the list.  We cannot use
			// the dist from the given roadPos because that's newer
			// than dist for all the other objects stored in the lists.
			//
			if( samePathPointAsOwner )
			{
				int curIdx;
				if( roadPos.IsRoad() )
				{
					int currRoadId = roadPos.GetRoad().GetId();
					curIdx = m_pRoadRefPool[currRoadId].objIdx;

					while( curIdx != 0 )
					{
						if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
						{
							ownerDist = m_pDynObjRefPool[curIdx].distance;
							break;
						}

						curIdx = m_pDynObjRefPool[curIdx].next;
					}

				}
				else
				{
					int currIntrsctnId = roadPos.GetIntrsctn().GetId();
					curIdx = m_pIntrsctnRefPool[currIntrsctnId].objIdx;

					while( curIdx != 0 )
					{
						if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
						{
							int crdrId = roadPos.GetCorridor().GetRelativeId();
							ownerDist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
							break;
						}

						curIdx = m_pDynObjRefPool[curIdx].next;
					}

				}

				if( ownerDist < -100.0 )
				{
					//cerr << "CCved::BuildFwdObjList: unable to determine ";
					//cerr << "owner object " << ownerObjId << " distance.  Using ";
					//cerr << "dist from given roadPos" << endl;
					ownerDist = roadPos.GetDistance();
				}
			}
		}

		if ( !reachedFirstValidPathPoint )  continue;

		//
		// If the pathpoint in on road, it means the roadPos is on road.
		// Then get the road lanemask.
		//
		if( path.GetPoint(pathItr).IsRoad() )
		{

			//
			// Looking for objects on road.
			//
			bitset<cCV_MAX_CRDRS> tmpLaneMask = path.GetPoint(pathItr).GetLaneCrdrMask();
			bitset<cCV_MAX_LANES> laneMask =(int) tmpLaneMask.to_ulong();
			double startDist = path.GetPoint(pathItr).GetStartDist();
			double endDist = path.GetPoint(pathItr).GetEndDist();
			CRoad currRoad = path.GetPoint(pathItr).GetRoad();
			if ( fabs( endDist - currRoad.GetLinearLength() ) < 1.0 )
			{
				// this is for cases where the object might have a value
				// slightly larger than the end of road when it's about to
				// step onto an intersection
				endDist += 20.0;
			}

			//
			// Get the direction of one of the lanes in the mask.  The
			// assumption is that all of the lanes must have the same
			// direction.
			//
			cvELnDir laneDir = ePOS;
			int i;
			for( i = 0; i < cCV_MAX_LANES; i++ )
			{
				if( laneMask[i] )
				{
					CLane aLane( currRoad, i );
					laneDir = aLane.GetDirection();
					break;
				}
			}


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  looking at road = " << currRoad << endl;
            gout << "    laneMask = " << laneMask;
			gout << "   start = "<< startDist;
			gout << "   end = " << endDist << endl;

#endif

			//
			// Insert objects into the fwd list until there are no
			// more objects on this road or we reach the maximum
			// number of objects needed.
			//
			int currRoadId = currRoad.GetId();
			int curIdx = m_pRoadRefPool[currRoadId].objIdx;

			while( curIdx != 0 && fwdObjs.size() < (TU32b)maxObjs )
			{
				//
				// Perform quick checks to make sure that this object is not
				// the owner object, part of the object mask and is located
				// within the needed distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
//				bool withinDist = (
//							m_pDynObjRefPool[curIdx].distance >= startDist &&
//							m_pDynObjRefPool[curIdx].distance <= endDist
//							);
				cvTObjAttr* pObjAttr = BindObjAttr( objId );
				double objLength = pObjAttr->xSize;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
				gout << "    looking at obj " << objId << "   dist = ";
				gout << m_pDynObjRefPool[curIdx].distance << endl;
#endif

				//
				// An object should be placed on the owner's forward list as long
				// as any part of the object is ahead of the owner's center.
				//
				bool withinDist = true;
				if ( samePathPointAsOwner )
				{
					if( laneDir == ePOS )
					{
						double objAdjustedDist =
								m_pDynObjRefPool[curIdx].distance + (objLength * 0.5);
						withinDist = (
								objAdjustedDist >= ownerDist &&
								m_pDynObjRefPool[curIdx].distance <= endDist
								);
#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "      pos lane   objAdjustedDist = " << objAdjustedDist;
						gout << "   ownerDist = " << ownerDist << endl;
#endif
					}
					else
					{
						double objAdjustedDist =
								m_pDynObjRefPool[curIdx].distance - (objLength * 0.5);
						withinDist = (
								objAdjustedDist <= ownerDist &&
								m_pDynObjRefPool[curIdx].distance >= endDist
								);
#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "      neg lane   objAdjustedDist = " << objAdjustedDist;
						gout << "   ownerDist = " << ownerDist << endl;
#endif
					}

				}
				bool passedFirstTest = notOwnerObj && inObjMask && withinDist;


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
				gout << "*********ON Road*********" << endl;
				gout << " notOwnerObj = " << notOwnerObj << endl;
				gout << " inObjMask = " << inObjMask << endl;
				gout << " withinDist = " << withinDist << endl;
				gout << " passedFirstTest = " << passedFirstTest << endl;
#endif

				if( passedFirstTest )
				{

					//
					// If the input lanes overlap with the lanes the object is on,
					// or if both the input and object lanes are 0, then the object
					// is on the road and lane.
					//
					bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
					objLanes &= laneMask;
					bool objOnNeededLanes = (
								objLanes.any() ||
								( objLanes.to_ulong() == 0 && laneMask.to_ulong() == 0 )
								);
					if( objOnNeededLanes )
					{
						//
						// Compute the distance from the owner object to this object.
						//
						double distFromOwner;
						if( samePathPointAsOwner )
						{
							distFromOwner = fabs(
												ownerDist -
												m_pDynObjRefPool[curIdx].distance
												);
						}
						else {
							if ( laneDir == ePOS )
							{
								distFromOwner = (
											distTraveledSoFar +
											m_pDynObjRefPool[curIdx].distance
											);
							}
							else
							{
								distFromOwner = (
											distTraveledSoFar +
											currRoad.GetLinearLength() -
											m_pDynObjRefPool[curIdx].distance
											);
							}
						}

						//
						// Insert object into main list.
						//
						TObjListInfo node;
						node.objId = objId;
						node.distFromOwner = distFromOwner;
						node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "    **inserting " << node.objId;
						gout << " with dist = " << node.distFromOwner;
						gout << endl;
#endif

						fwdObjs.push_back( node );

					}
				}

				curIdx = m_pDynObjRefPool[curIdx].next;
			}

			if ( samePathPointAsOwner )
			{
				if ( roadPos.GetLane().GetDirection() == ePOS )
				{
					distTraveledSoFar += (
								currRoad.GetLinearLength() -
								ownerDist
								);
				}
				else
				{
					distTraveledSoFar += ownerDist;
				}
			}
			else
			{
				distTraveledSoFar += currRoad.GetLinearLength();
			}

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  distTraveledSoFar = " << distTraveledSoFar << endl;
#endif

		}
		else
		{

			//
			// Looking for objects on intersection.
			//

			//
			// On intersection, get the owner distance from the roadPos.
			//
		//	ownerDist = roadPos.GetDistance();

			bitset<cCV_MAX_CRDRS> crdrMask = path.GetPoint(pathItr).GetLaneCrdrMask();
			CIntrsctn currIntrsctn = path.GetPoint(pathItr).GetIntrsctn();

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  looking at intrsctn = " << currIntrsctn << endl;
            gout << "    crdrMask = " << crdrMask << endl;
#endif

			int intrsctnId = currIntrsctn.GetId();
			const double* cpStartDists = path.GetPoint(pathItr).GetStartDists();
			const double* cpEndDists = path.GetPoint(pathItr).GetEndDists();
			TIntVec dynObjs;

			//
			// Insert objects into the fwd list until there are no
			// more objects on this road or we reach the maximum
			// number of objects needed.
			//
			int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
			double maxCrdrDist = 0.0;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << " fwdObjs current size = " << fwdObjs.size() << endl;
			if( curIdx == 0 )
			{
				gout << " curIdx = 0 ...obj not reported in internal list. " << endl;
				gout << " size of fwdObjs = " << fwdObjs.size() << endl;
				gout << " maxObjs = " << maxObjs << endl;
			}
#endif

			while( curIdx != 0 && fwdObjs.size() < (TU32b)maxObjs )
			{

				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
				bool passedFirstTest = notOwnerObj && inObjMask;


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
				gout << "*********ON Intersection*********" << endl;
				gout << " looking at obj " << objId << endl;
				gout << " notOwnerObj = " << notOwnerObj << endl;
				gout << " inObjMask = " << inObjMask << endl;
				gout << " passedFirstTest = " << passedFirstTest << endl;
#endif


				if( passedFirstTest )
				{
					bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
					objCrdrs &= crdrMask;

					//
					// If the input crdrs overlap with the crdrs the object is on,
					// or if both the input and object crdrs are 0, then the object
					// is on the intrsctn and crdrs.
					//
					bool objOnNeededCrdrs = (
									objCrdrs.any() ||
									( objCrdrs.to_ulong() == 0 && crdrMask.to_ulong() == 0 )
									);
#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << " ********The value of objOnNeededCrdr is: " << objOnNeededCrdrs << endl;
#endif

					if( objOnNeededCrdrs )
					{

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << " The obj is on needed corridors. " << endl;
#endif

						int crdrId;
						for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
						{
							// if the object lies on the current corridor
							if ( objCrdrs[crdrId] )
							{

								// if object lies within the distance range
								CCrdr currCrdr( currIntrsctn, crdrId );
								double startDist = cpStartDists[crdrId];
								double endDist = cpEndDists[crdrId];
								if ( fabs( endDist - currCrdr.GetLength() ) < 1.0 )
								{
									// this is for cases where the object might have a value
									// slightly larger than the end of road when it's about to
									// step onto an intersection
									endDist += 15.0;
								}


								// Use obj dist from obj pool.
								double objDistAlongCrdr =
											m_pDynObjRefPool[curIdx].crdrDistances[crdrId];



#ifdef DEBUG_BUILD_FWD_OBJ_LIST
								gout << " $$$$$$$$ checking obj dist on intersection$$$$$$$$$$$$ ";
								gout << endl;
								gout << " $$$$$$$$ obj " << objId << " : dist from pool = " << objDistAlongCrdr;
								gout << " $$$$$$$$$$$$$ " << endl;
#endif

								bool withinDist = (
										objDistAlongCrdr >= startDist &&
										objDistAlongCrdr <= endDist
										);

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
								gout << "    start = " << startDist;
								gout << "   end = " << endDist << endl;
#endif

								//
								// If the owner and the object are on the same interesction,
								// then make sure that the object is really ahead of the
								// owner.
								//
								cvTObjAttr* pObjAttr = BindObjAttr( objId );
								double objLength = pObjAttr->xSize;
								bool objFwd = true;
								if ( samePathPointAsOwner )
								{
									objFwd = (
												objDistAlongCrdr  + (objLength * 0.5) >=
										ownerDist
										);
								}


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << " ******* withinDist =  " << withinDist << endl;
						gout << " ******* objFwd = " << objFwd << endl;
#endif


								if( withinDist && objFwd )
								{

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
									gout << " The obj is within distance. " << endl;
#endif
									//
									// Compute the distance from the owner object
									// to this object.
									//
									double distFromOwner;
									if( samePathPointAsOwner )
									{
										distFromOwner = fabs(
													ownerDist -
													objDistAlongCrdr
													);
									}
									else {
										distFromOwner = (
											distTraveledSoFar +
											objDistAlongCrdr
											);
									}
									if(  objDistAlongCrdr > maxCrdrDist )
									{
										maxCrdrDist = objDistAlongCrdr ;
									}

									TObjListInfo node;
									node.objId = objId;
									node.distFromOwner = distFromOwner;
									node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
									gout << "    **inserting " << node.objId;
									gout << " with dist = " << node.distFromOwner;
									gout << endl;
#endif


									fwdObjs.push_back( node );
									break;
								} // If object lies within the distance range
							} // If the object lies on the current corridor
						} // For each corridor in the mask
					}
				}

				curIdx = m_pDynObjRefPool[curIdx].next;
			}

			if( samePathPointAsOwner )
			{
				distTraveledSoFar += (
					roadPos.GetCorridor().GetLength() -
					ownerDist
					);
			}
			else
			{
				//
				// Get the length of one of the corridors in the intersection.
				int i;
				for( i = 0; i < cCV_MAX_CRDRS; i++ )
				{
					if( crdrMask[i] )
					{
						CCrdr aCrdr( currIntrsctn, i );
						distTraveledSoFar += aCrdr.GetLength();

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "  crdr " << i << " has a length of ";
						gout << aCrdr.GetLength() << endl;
#endif

						break;
					}
				}
			}


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  distTraveledSoFar = " << distTraveledSoFar << endl;
#endif
		}

		if ( fwdObjs.size() >= (TU32b)maxObjs )  break;
	}

	//
	// Sort the list by distance to the owner.
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < fwdObjs.size(); k++ )
	{
		hold = fwdObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < fwdObjs[loc-1].distFromOwner ) )
		{
			fwdObjs[loc] = fwdObjs[loc-1];
			loc--;
		}
		fwdObjs[loc] = hold;
	}

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
	gout << "  fwdObjs [size = " << fwdObjs.size() << "]: ";
	vector<TObjListInfo>::iterator i;
	for( i = fwdObjs.begin(); i != fwdObjs.end(); i++ )
	{
		gout << " " << i->objId << " [" << i->distFromOwner << "]";
	}
	gout << endl;
#endif

}	// end of BuildFwdObjList



//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects behind the
//   specified object in the parameter.
//
// Remarks: For now, this function only looks at objects on the same
//   road as the owner object.
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos    - The current roadPos of the specified object in the
//                parameter.
//   maxObjs    - The maximum number of objects the user is interested in
//                looking at.
//   backObjs   - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::BuildBackObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& backObjs
			)
{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
	gout << "[" << ownerObjId << "] == BuildBackObjList =====================";
	gout << endl;
#endif

	//
	// No back object object list inside intersections for now.
	//
	if( !roadPos.IsRoad() )  return;

	double ownerDist = -101.0;
	CRoad currRoad = roadPos.GetRoad();
	CLane currLane = roadPos.GetLane();
	cvELnDir currLaneDir = currLane.GetDirection();

	{
		int curIdx = m_pRoadRefPool[currRoad.GetId()].objIdx;

		while( curIdx != 0 )
		{
			if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
			{
				ownerDist = m_pDynObjRefPool[curIdx].distance;
				break;
			}

			curIdx = m_pDynObjRefPool[curIdx].next;
		}

		if( ownerDist < -100.0 )
		{
			//cerr << "CCved::BuildBackObjList: unable to determine ";
			//cerr << "owner object " << ownerObjId << " distance.  Using ";
			//cerr << "dist from given roadPos" << endl;
			ownerDist = roadPos.GetDistance();
		}
	}


	//
	// Build a lane mask with all lanes traveling in the same direction
	// as the lane in the specified roadPos.
	//
	int laneId;
	int numLanes = currRoad.GetNumLanes();
	int firstLaneIdx = currRoad.GetLaneIdx();
	bitset<cCV_MAX_LANES> laneMask;
	for( laneId = 0; laneId < numLanes; laneId++ )
	{
		cvTLane* pLane = BindLane( firstLaneIdx + laneId );
		if( pLane->direction == currLaneDir )
		{
			laneMask.set( laneId );
		}
	}

	//
	// Compute the distances at which to start and end looking for
	// objects on this road.
	//
	double startDist;
	double endDist;
	if( currLaneDir == ePOS )
	{
		startDist = 0.0;
		endDist = ownerDist;
	}
	else
	{
		startDist = ownerDist;
		endDist = currRoad.GetLinearLength();
	}

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
	gout << "  looking at road = " << currRoad << endl;
    gout << "    laneMask = " << laneMask;
	gout << "   start = "<< startDist;
	gout << "   end = " << endDist << endl;

#endif

	//
	// Insert objects into the fwd list until there are no
	// more objects on this road or we reach the maximum
	// number of objects needed.
	//
	int currRoadId = currRoad.GetId();
	int curIdx = m_pRoadRefPool[currRoadId].objIdx;

	//
	// Build a temporary array that holds the indexes of
	// all of the object that lie on this road.
	//
	int tempObjArr[cNUM_DYN_OBJS];
	int tempObjArrSize = 0;
	while( curIdx != 0 )
	{
		tempObjArr[tempObjArrSize] = curIdx;
		tempObjArrSize++;
		curIdx = m_pDynObjRefPool[curIdx].next;
	}

	int i;
	for( i = tempObjArrSize - 1; i >= 0; i-- )
	{
		if ( (int)backObjs.size() >= maxObjs )  break;
		int curIdx = tempObjArr[i];

		//
		// Perform quick checks to make sure that this object is not
		// the owner object, part of the object mask and is located
		// within the needed distances.
		//
		int objId = m_pDynObjRefPool[curIdx].objId;
		bool notOwnerObj = ownerObjId != objId;
		cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
		bool withinDist = (
					m_pDynObjRefPool[curIdx].distance >= startDist &&
					m_pDynObjRefPool[curIdx].distance <= endDist
					);
		cvTObjAttr* pObjAttr = BindObjAttr( objId );
		double objLength = pObjAttr->xSize;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
		gout << "    looking at obj " << objId << "   dist = ";
		gout << m_pDynObjRefPool[curIdx].distance << endl;
#endif

		//
		// An object should be placed on the owner's back list as long
		// as any part of the object is behind the owner's center.
		//
		bool objBack = true;
		if( currLaneDir == ePOS )
		{
			objBack = (
				m_pDynObjRefPool[curIdx].distance - (objLength * 0.5) <=
				ownerDist
				);
		}
		else
		{
			objBack = (
				m_pDynObjRefPool[curIdx].distance + (objLength * 0.5) >=
				ownerDist
				);
		}

		bool passedFirstTest = notOwnerObj && inObjMask && withinDist && objBack;
		if( passedFirstTest )
		{

			//
			// If the input lanes overlap with the lanes the object is on,
			// or if both the input and object lanes are 0, then the object
			// is on the road and lane.
			//
			bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
			objLanes &= laneMask;
			bool objOnNeededLanes = (
						objLanes.any() ||
						( objLanes.to_ulong() == 0 && laneMask.to_ulong() == 0 )
						);
			if( objOnNeededLanes )
			{
				//
				// Compute the distance from the owner object to this object.
				//
				double distFromOwner = fabs(
										ownerDist -
										m_pDynObjRefPool[curIdx].distance
										);
				//
				// Insert object into main list.
				//
				TObjListInfo node;
				node.objId = objId;
				node.distFromOwner = distFromOwner;
				node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
				gout << "    **inserting " << node.objId;
				gout << " with dist = " << node.distFromOwner;
				gout << endl;
#endif

				backObjs.push_back( node );
			}
		}

		//curIdx = m_pDynObjRefPool[curIdx].next;
	}

	//
	// Sort the list by distance to the owner.
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < backObjs.size(); k++ )
	{
		hold = backObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < backObjs[loc-1].distFromOwner ) )
		{
			backObjs[loc] = backObjs[loc-1];
			loc--;
		}
		backObjs[loc] = hold;
	}


}  // end of BuildBackObjList

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects that are on the oncoming
//	lane of the specified object in the parameter. Note that the oncoming lane
//	is the closest opposite direction lane with repect to the lane the owner object
//	is located on.
//
// Remarks:
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos - The current roadPos of the specified object in the parameter.
//   path   - The current path of the specified object in the parameter.
//   maxObjs - The maximum number of objects the user is interested in
//             looking at.
//   oncomObjs - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::BuildOncomingObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& oncomObjs
			)
{


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << endl;
		gout << " == BuildOncomingObjList====";
		gout << endl;
#endif


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
	gout << " The input path size is: " << path.Size() << endl;
	gout << " The path is: " << path;
#endif


	//
	// Error checking.
	//
	if ( !path.IsValid() || path.Size() <= 0)
	{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << "CCved::BuildOncomingObjList: invalid path....exit" << endl;
#endif

		return;

	}
	if( ! roadPos.IsValid() )
	{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << "CCved::BuildOncomingObjList: invalid road position....exit" << endl;
#endif

		return;

	}

	//
	// Initialize variables.
	//
	double ownerDist = - 101.0;
	double distTraveledSoFar = 0.0;
	bool reachedFirstValidPathPoint = false;

	//
	// Iterate through all of the path points to build the oncoming
	// object list.
	//

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
	gout << " This is just before the path point iteration. " << endl;
#endif

	CPath::cTPathIterator pathItr;
	for( pathItr = path.Begin(); pathItr != path.End(); pathItr++ )
	{
		double startDist = path.GetPoint(pathItr).GetStartDist();
		double endDist = path.GetPoint(pathItr).GetEndDist();

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << " This is the current path point: " << pathItr << endl;
		gout << " The start of the path point is: " << startDist << endl;
		gout << " The end of the path point is: " << endDist << endl;
#endif

		//
		// Skip all path points until reaching the one that is
		// the same as the given road position.
		//
		bool samePathPointAsOwner = false;
		if ( !reachedFirstValidPathPoint )
		{
			if ( path.GetPoint(pathItr).IsRoad() )
			{
				reachedFirstValidPathPoint = (
							roadPos.IsRoad() &&
							roadPos.GetRoad() == path.GetPoint(pathItr).GetRoad()
							);
			}
			else
			{
				reachedFirstValidPathPoint = (
							!roadPos.IsRoad() &&
							roadPos.GetIntrsctn().GetId() ==
							path.GetPoint(pathItr).GetIntrsctn().GetId()
							);
			}
			samePathPointAsOwner = reachedFirstValidPathPoint;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The value of samePathPointAsOwner is: " << samePathPointAsOwner << endl;
#endif

		}

		if ( !reachedFirstValidPathPoint )  continue;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << " Have found the first valid path point. " << endl;

#endif

		//
		// If the pathpoint is on road, need to find the closest lane that
		// moves in the opposite direction with respect to the owner object.
		// So keep moving left until the lane in the opposite direction is reached.
		//
		CLane closestOppositeDirLn;
		CLane leftLane;
		if( path.GetPoint(pathItr).IsRoad() )
		{
			CRoad currRoad = path.GetPoint(pathItr).GetRoad();

			if ( fabs( endDist - currRoad.GetLinearLength() ) < 1.0 )
			{
				// This is for cases where the object might have a value
				// slightly larger than the end of road when it's about to
				// step onto an intersection
				endDist += 20.0;
			}

			//
			// This returns the first lane of the road that
			// the path point is on.
			CLane currLane = path.GetPoint(pathItr).GetLane();

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The current lane from the path point is: "<< currLane.GetId() << endl;
			gout << " The current road name is: " << currRoad.GetName() << endl;
			gout << " The current road id is: " << currRoad.GetId() << endl;
#endif

			//
			// If the current lane is already the leftmost lane,
			// it means it is a one-way road, then no oncoming objects, return.
			//
			if ( currLane.IsLeftMost() )
			{
				return;

			}
			leftLane = currLane.GetLeft();

			if( !leftLane.IsValid() )
			{
				return;
			}
			else
			{
				cvELnDir leftLaneDir = leftLane.GetDirection();
				cvELnDir currLaneDir = currLane.GetDirection();

				bool sameDirLanes = leftLaneDir == currLaneDir;

				if ( sameDirLanes && leftLane.IsLeftMost() )
				{
					return;
				}

				while( sameDirLanes && !leftLane.IsLeftMost() )
				{

					leftLane = leftLane.GetLeft();
					if( !leftLane.IsValid() )
					{
						return;
					}
					leftLaneDir = leftLane.GetDirection();
					sameDirLanes = leftLaneDir == currLaneDir;
				}
				if ( !sameDirLanes )
				{
					closestOppositeDirLn = leftLane;
				}
			}

			if( ! closestOppositeDirLn.IsValid() )
			{
				return;
			}

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " Have found this closestOppositeDirLn: "<< closestOppositeDirLn.GetId() << endl;
#endif


			int currRoadId = currRoad.GetId();
			int curIdx = m_pRoadRefPool[currRoadId].objIdx;

			//
			// Get the owner distance from the list. The distance from the roadPos
			// is inaccurate. This step is only necessary when the obj is on the same
			// path point as the owner.
			//
			if( samePathPointAsOwner )
			{
				while( curIdx != 0 )
				{
					if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
					{
						ownerDist = m_pDynObjRefPool[curIdx].distance;
						break;
					}
					curIdx = m_pDynObjRefPool[curIdx].next;
				}
			}

			if( ownerDist < -100.0 )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
					gout << "CCved::BuildFwdObjList: unable to determine ";
					gout << "owner object # " << ownerObjId << " distance. " << endl;
					gout << " So using dist from given roadPos" << endl;
#endif
					ownerDist = roadPos.GetDistance();
			}

			curIdx = m_pRoadRefPool[currRoadId].objIdx;

			//
			// Insert objects into the oncoming list until there are no
			// more objects on this road or we reach the maximum
			// number of objects needed.
			//
			while( curIdx != 0 && (int)oncomObjs.size() < maxObjs )
			{
				//
				// Make sure that this object is not the owner object, part of the object mask,
				// on the closest opposite direction lane and is located within the needed
				// distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
				bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " The current object in the pool is: "<< objId << endl;
				gout << " The value of the correct obj type or not is: " << inObjMask << endl;
				gout << " The road id of the current obj is: " << m_pDynObjRefPool[curIdx].roadId << endl;
#endif


				if( passedFirstTest )
				{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
					gout << " This obj has passed the first test. " << endl;
#endif

					//
					// Make sure the objects to be returned is on the closest
					// opposite direction lane.
					bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
					bitset<cCV_MAX_LANES> closestOppositeDirLnMask;
					if( ! closestOppositeDirLn.IsValid() )
					{
						return;
					}
					closestOppositeDirLnMask.set( closestOppositeDirLn.GetRelativeId() );
					bool objOnNeededLanes = (
								( objLanes.to_ulong() == closestOppositeDirLnMask.to_ulong() ) );


//
//
//								|| ( objLanes.to_ulong() & closestOppositeDirLnMask.to_ulong() )
//								);

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
					gout << " The value of objOnNeededLanes is: "<< objOnNeededLanes << endl;
					gout << " The objLanes.to_ulong is: " << objLanes.to_ulong() << endl;
					gout << " The closestOppositeDirLnMask.to_ulong is: ";
					gout << closestOppositeDirLnMask.to_ulong() << endl;
#endif

					cvTObjAttr* pObjAttr = BindObjAttr ( objId );
					double objLength = pObjAttr->xSize;
					double distFromOwner;
					if( objOnNeededLanes )
					{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
						gout << " The obj is on the closestOppositeDirLn. " << endl;
#endif

						bool foundDupsOnRoad = false;
						if( samePathPointAsOwner )
						{
							//
							// Make sure the obj is within distances if it is on
							// the same path point as owner.
							//
							bool withinDist;
							if( currLane.GetDirection() == ePOS )
							{

								double objAdjustedDist =
										m_pDynObjRefPool[curIdx].distance + (objLength * 0.5);

								//
								// Make sure the obj on the boundary between intersection and
								// road get picked up.
								//
								withinDist = (
										objAdjustedDist >= ownerDist &&
										m_pDynObjRefPool[curIdx].distance <= endDist
										);
							}
							else
							{
								double objAdjustedDist =
										m_pDynObjRefPool[curIdx].distance - (objLength * 0.5);
								withinDist = (
										objAdjustedDist <= ownerDist &&
										m_pDynObjRefPool[curIdx].distance >= endDist
										);

							}


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST

							gout << " The object is within distance or not: " << withinDist << endl;
							gout << " The obj distance is: " << m_pDynObjRefPool[curIdx].distance ;
							gout << endl;
							gout << " The owner dist is: " << ownerDist << endl;
							gout << " The endDist is: " << endDist << endl;

#endif

							if( withinDist )
							{
								//
								// Compute the distance from the owner object to this object.
								//
								distFromOwner = fabs(
												ownerDist -
												m_pDynObjRefPool[curIdx].distance
												);

								//
								// Insert object into main list without duplicates.
								//
								vector<TObjListInfo>::iterator itr;
								for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
								{
									if( (*itr).objId == objId )
									{
										foundDupsOnRoad = true;
										break;
									}
								}
								if( ! foundDupsOnRoad )
								{
									TObjListInfo node;
									node.objId = objId;
									node.distFromOwner = distFromOwner;
									node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
									gout << "  **inserting " << node.objId << " to the oncoming list.";
									gout << endl;
									gout << " The dist from the owner is:" << node.distFromOwner;
									gout << endl;;
#endif

									oncomObjs.push_back( node );
								}
							}
						}
						else
						{
							cvELnDir laneDir;
							if( roadPos.IsRoad() )
							{
								laneDir = roadPos.GetLane().GetDirection();
							}
							else
							{
								laneDir = roadPos.GetCorridor().GetDstntnLn().GetDirection();
							}
							if ( laneDir == ePOS )
							{
								distFromOwner = (
											distTraveledSoFar +
											m_pDynObjRefPool[curIdx].distance
											);


							}
							else
							{
								distFromOwner = (
											distTraveledSoFar +
											currRoad.GetLinearLength() -
											m_pDynObjRefPool[curIdx].distance
											);

							}

							//
							// Insert object into main list without duplicates.
							//
							vector<TObjListInfo>::iterator itr;
							for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
							{
								if( (*itr).objId == objId )
								{
									foundDupsOnRoad = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
									gout << " The obj # " << objId << " is already in the list. ";
									gout << endl;
#endif

									break;
								}
							}
							if( ! foundDupsOnRoad )
							{
								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
								gout << "  **inserting " << node.objId << " to the oncoming list.";
								gout << endl;
								gout << " The dist from the owner is:" << node.distFromOwner;
								gout << endl;;
#endif

								oncomObjs.push_back( node );
							}
						}
					}	// On needed lane.
				}	// Passed first test.

				curIdx = m_pDynObjRefPool[curIdx].next;

			}	// while loop

			//
			// Update distance traveled so far.
			//
			if ( samePathPointAsOwner )
			{
				if ( roadPos.GetLane().GetDirection() == ePOS )
				{
					distTraveledSoFar += (
								currRoad.GetLinearLength() -
								ownerDist
								);
				}
				else
				{
					distTraveledSoFar += ownerDist;
				}
			}
			else
			{
				distTraveledSoFar += currRoad.GetLinearLength();
			}

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The distTraveledSoFar is:" << distTraveledSoFar << endl;
#endif


		}	// if the path point is on road.
		else
		{
			//
			// If the path point is on intersection.
			//

			//
			// In order to find the oncoming corridor, need to find the
			// closest opposite direction lane of the destination road of the
			// the corridors on the current path point ( refered as lane A ) and
			// the closest opposite direction lane of the source road of these
			// corridors ( referred as lane B ).Then get all the corridors from
			// the current intersection and get the corridor whose source lane is
			// lane A and whose destination lane is lane B. This corridor is
			// the oncoming corridor.
			//

			//
			// Find the closest opposite direction lane of the destination road and
			// source road of the corridors on the current path point.
			//
			bool foundA = false;
			bool foundB = false;
			CCrdr currCrdr;
			CLane leftMostLnAlongDirOfDstntnRoad;
			CLane leftMostLnAlongDirOfSrcRoad;
			CLane closestOppositeDirLnOfDstntnRd;
			CLane closestOppositeDirLnOfSrcRd;
			bitset<cCV_MAX_CRDRS> crdrMask = path.GetPoint( pathItr ).GetLaneCrdrMask();
			CIntrsctn currIntrsctn = path.GetPoint( pathItr ).GetIntrsctn();

			CLane tempLane;
			CLane tempSrcLane;
			int crdrId;
			for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
			{
				if( crdrMask[crdrId] )
				{

					CCrdr temCrdr( currIntrsctn, crdrId );
					CLane temLane = temCrdr.GetDstntnLn();
					tempLane = temLane;
					CLane temSrcLane = temCrdr.GetSrcLn();
					tempSrcLane = temSrcLane;
					if( ! foundA  && temLane.IsLeftMostAlongDir() )
					{

						leftMostLnAlongDirOfDstntnRoad = temLane;
						if( ! leftMostLnAlongDirOfDstntnRoad.IsLeftMost() )
						{
							closestOppositeDirLnOfDstntnRd =
								leftMostLnAlongDirOfDstntnRoad.GetLeft();
						}

						//
						// If it is a one-way road, return.
						//
						if( !closestOppositeDirLnOfDstntnRd.IsValid() )
						{
							return;
						}
						foundA = true;
						CCrdr currentCrdr = temCrdr;
						currCrdr = currentCrdr;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
						gout << " The current corridor is: ";
						gout << currCrdr.GetRelativeId() << endl;
#endif

					}
					if( ! foundB  && temSrcLane.IsLeftMostAlongDir() )
					{
						leftMostLnAlongDirOfSrcRoad = temSrcLane;
						if( ! leftMostLnAlongDirOfSrcRoad.IsLeftMost() )
						{

							closestOppositeDirLnOfSrcRd =
								leftMostLnAlongDirOfSrcRoad.GetLeft();
						}

						//
						// If it is a one-way road, return.
						//
						if( !closestOppositeDirLnOfSrcRd.IsValid() )
						{
							return;
						}
						foundB = true;
					}
					if( foundA && foundB )
					{
						break;
					}

				}
			}	// for loop

			//
			// If none of the corridor connects to the leftmostlane of the
			// destination road or source road, have to look for it.
			if( ! foundA )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " Have to move left to get the leftmost lane. " << endl;
				gout << " The lane that is used to move left is: " << tempLane << endl;
#endif

				CLane leftLaneOfDstntnRd;
				if( ! tempLane.IsLeftMost() )
				{
					leftLaneOfDstntnRd = tempLane.GetLeft();
				}

				if( !leftLaneOfDstntnRd.IsValid() )
				{
					return;
				}
				else
				{
					cvELnDir leftLaneOfDstntnRdDir = leftLaneOfDstntnRd.GetDirection();
					cvELnDir tempLaneDir = tempLane.GetDirection();

					bool sameDirLanesOfDstntnRd = leftLaneOfDstntnRdDir == tempLaneDir;

					if( sameDirLanesOfDstntnRd && leftLaneOfDstntnRd.IsLeftMost() )
					{
						return;
					}

					while( sameDirLanesOfDstntnRd && !leftLaneOfDstntnRd.IsLeftMost() )
					{

						leftLaneOfDstntnRd = leftLaneOfDstntnRd.GetLeft();
						if( !leftLaneOfDstntnRd.IsValid() )
						{
							return;
						}
						leftLaneOfDstntnRdDir = leftLaneOfDstntnRd.GetDirection();
						sameDirLanesOfDstntnRd = leftLaneOfDstntnRdDir == tempLaneDir;
					}
					if ( !sameDirLanesOfDstntnRd )
					{
						closestOppositeDirLnOfDstntnRd = leftLaneOfDstntnRd;
						leftMostLnAlongDirOfDstntnRoad = closestOppositeDirLnOfDstntnRd .GetLeft();
					}
				}
			}
			if( ! foundB )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " Have to move left to get the leftmost lane. " << endl;
				gout << " The lane that is used to move left on source road is: ";
				gout << tempSrcLane << endl;
#endif

				CLane leftLaneOfSrcRd;
				if( ! tempSrcLane.IsLeftMost() )
				{

					leftLaneOfSrcRd = tempSrcLane.GetLeft();
				}

				if( !leftLaneOfSrcRd.IsValid() )
				{
					return;
				}
				else
				{
					cvELnDir leftLaneOfSrcRdDir = leftLaneOfSrcRd.GetDirection();
					cvELnDir tempSrcLaneDir = tempSrcLane.GetDirection();

					bool sameDirLanesOfSrcRd = leftLaneOfSrcRdDir == tempSrcLaneDir;

					if ( sameDirLanesOfSrcRd && leftLaneOfSrcRd.IsLeftMost() )
					{
						return;
					}

					while( sameDirLanesOfSrcRd && !leftLaneOfSrcRd.IsLeftMost() )
					{

						leftLaneOfSrcRd = leftLaneOfSrcRd.GetLeft();
						if( !leftLaneOfSrcRd.IsValid() )
						{
							return;
						}
						leftLaneOfSrcRdDir = leftLaneOfSrcRd.GetDirection();
						sameDirLanesOfSrcRd = leftLaneOfSrcRdDir == tempSrcLaneDir;
					}
					if ( !sameDirLanesOfSrcRd )
					{
						closestOppositeDirLnOfSrcRd = leftLaneOfSrcRd;
						leftMostLnAlongDirOfSrcRoad = closestOppositeDirLnOfSrcRd .GetLeft();
					}
				}
			}

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The current intersection is: " << currIntrsctn.GetName() << endl;
			gout << " The current destination road is: " << tempLane.GetRoad() << endl;
			gout << " The current source road is: " << tempSrcLane.GetRoad() << endl;
			gout << " The leftMostLnAlongDir on the destination road is: ";
			gout << leftMostLnAlongDirOfDstntnRoad << endl;
			gout << " The leftMostLnAlongDir on the source road is: ";
			gout << leftMostLnAlongDirOfSrcRoad << endl;
			gout << " The closest opposite direction lane of the destination road is: "<< endl;
			gout << " " << closestOppositeDirLnOfDstntnRd << endl;
			gout << " The closest opposite direction lane of the source road is: "<< endl;
			gout << " " << closestOppositeDirLnOfSrcRd << endl;
#endif

			//
			// Get the oncoming corridor.
			//

			if (!closestOppositeDirLnOfSrcRd.IsValid())
				continue; //if we have found no oncomming corridor
			if (!closestOppositeDirLnOfDstntnRd.IsValid())
				continue;//we have some kind logical fault, some case is not being handled.

			CCrdr oncomingCrdr;
			/*static*/ vector<CCrdr> allCrdrs;
			allCrdrs.clear();
			allCrdrs.reserve(15);
			currIntrsctn.GetAllCrdrs( allCrdrs );
			vector<CCrdr>::iterator itr;
			bool foundCrdr = false;
			for( itr = allCrdrs.begin(); itr != allCrdrs.end(); itr++ )
			{
				if( (*itr).GetDstntnLn() == closestOppositeDirLnOfSrcRd )
				{
					if((*itr).GetSrcLn() == closestOppositeDirLnOfDstntnRd )
					{
						oncomingCrdr = *itr;
						foundCrdr = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The oncoming corridor is: " << oncomingCrdr.GetRelativeId() << endl;
#endif

						break;
					}
				}
			}

			//
			// For the some intersections, the oncomingCrdr may not be found because
			// closest opposite lanes of the destination road and source road may not
			// be connected. Then don't look at objs on this intersection and move on
			// to the next path point.
			//
			if( ! foundCrdr )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " oncomingCrdr is not found. " << endl;
#endif

				continue;
			}


			int intrsctnId = currIntrsctn.GetId();
			int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
			double distFromOwner;

			// Insert objects into the oncoming list until there are no
			// more objects on this intersection or we reach the maximum
			// number of objects needed.
			//
			while( curIdx != 0 && (int)oncomObjs.size() < maxObjs )
			{

				//
				// Make sure that this object is not the owner object, part of
				// the object mask, on the oncoming corridor and is located
				// within the needed distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
				bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " The current obj in the pool is:  #" << objId << endl;

#endif


				if( passedFirstTest )
				{
					bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
					bitset<cCV_MAX_CRDRS> oncomingCrdrMask;
					oncomingCrdrMask.set( oncomingCrdr.GetRelativeId() );


					objCrdrs &= oncomingCrdrMask;
					bool objOnNeededCrdrs= (
						objCrdrs.any() ||
						( objCrdrs.to_ulong() == 0 && oncomingCrdrMask.to_ulong() == 0 )
						);


					int oncomingCrdrId = oncomingCrdr.GetRelativeId();
					double objDistAlongOncomCrdr;
					objDistAlongOncomCrdr = m_pDynObjRefPool[curIdx].crdrDistances
											[oncomingCrdrId] ;
					if( objOnNeededCrdrs )
					{
						bool foundDupsOnIntrsctn = false;

						//
						// Make sure the object is within distance if the obj is on the
						// same path point as the owner.
						//
						if( samePathPointAsOwner )
						{
							//
							// Only need to get obj ahead of the owner.
							//
							double dist =  oncomingCrdr.GetLength() - roadPos.GetDistance();
							bool withinDist =  objDistAlongOncomCrdr >= 0  &&
											   objDistAlongOncomCrdr  <= dist;

							if( withinDist )
							{

									distFromOwner = fabs( oncomingCrdr.GetLength() -
									objDistAlongOncomCrdr - ownerDist ) ;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
								gout << " The oncomCrdr length is: " << oncomingCrdr.GetLength();
								gout << endl;
								gout << " The oncom obj dist from the pool is: ";
								gout << objDistAlongOncomCrdr << endl;
								gout << " The owner dist is: " << ownerDist << endl;
#endif

								//
								// Insert object into main list without duplicates.
								//
								vector<TObjListInfo>::iterator itr;
								for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
								{
									if( (*itr).objId == objId )
									{
										foundDupsOnIntrsctn = true;
										break;
									}
								}
								if( ! foundDupsOnIntrsctn )
								{
									TObjListInfo node;
									node.objId = objId;
									node.distFromOwner = distFromOwner;
									node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
									gout << "  **inserting " << node.objId << " to the oncoming list.";
									gout << endl;
									gout << " The dist from the owner is:" << node.distFromOwner;
									gout << endl;;
#endif

									oncomObjs.push_back( node );
								}

							}

						}
						else
						{
							distFromOwner = distTraveledSoFar + ( oncomingCrdr.GetLength()
												- objDistAlongOncomCrdr );

							//
							// Insert object into main list without duplicates.
							//
							vector<TObjListInfo>::iterator itr;
							for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
							{
								if( (*itr).objId == objId )
								{
									foundDupsOnIntrsctn = true;
									break;
								}
							}
							if( ! foundDupsOnIntrsctn)
							{
								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
								gout << "  **inserting " << node.objId << " to the oncoming list.";
								gout << endl;
								gout << " The dist from the owner is:" << node.distFromOwner;
								gout << endl;;
#endif

								oncomObjs.push_back( node );
							}
						}

					}	// On desired corridor.
				}	// Passed first test.

				curIdx = m_pDynObjRefPool[curIdx].next;


			}	// while loop

			//
			// Update distance traveled.
			//
			if( samePathPointAsOwner )
			{
				distTraveledSoFar += ( roadPos.GetCorridor().GetLength() - ownerDist );
			}
			else
			{
				distTraveledSoFar += oncomingCrdr.GetLength();
			}

		}	// If the path point is on intersection.
		if ( (int)oncomObjs.size() >= maxObjs )
		{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " Already reached maximum #of objects before end of path. " << endl;
#endif
			break;

		}

	}	// for loop


	//
	// Sort the list according to distance from the owner. (Although the
	// objs on road are already sorted, those on intersection are not.So need sorting.)
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < oncomObjs.size(); k++ )
	{
		hold = oncomObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < oncomObjs[loc-1].distFromOwner ) )
		{
			oncomObjs[loc] = oncomObjs[loc-1];
			loc--;
		}
		oncomObjs[loc] = hold;
	}


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
	gout << " The size of the oncoming list is:" << oncomObjs.size() << endl;
	vector<TObjListInfo>::iterator i;
	gout << " The following is the final list for obj #" << ownerObjId<< ":  ";
	for( i = oncomObjs.begin(); i != oncomObjs.end(); i++ )
	{
		gout << " " << i->objId << " [" << i->distFromOwner << "]";
	}
	gout << endl;

#endif

}	// end of BuildOncomingList



///////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects that approach the
//   specified object in the parameter from the side in a merging situation.
//
// Remarks: The owner object is on the current corridor and it looks at the
//	corridor that's approaching it from the side (refered as the side corridor).
//	The function first gets objects on this side corridor and then gets objects
//	on the side corridor's source lane.
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos - The current roadPos of the specified object in the
//                parameter.
//	 path - The current path of the specified object in the parameter.
//   maxObjs - The maximum number of objects the user is interested in
//                looking at.
//   apprchObjs - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::BuildApprchObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& apprchObjs
			)
{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
	gout << endl;
	gout << " == BuildApprchObjList==== for obj # " << ownerObjId;
	gout << endl;
#endif

	//
	// Error checking.
	//
	if( ! roadPos.IsValid() )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << "CCved::BuildApprchObjList: invalid road position....exit" << endl;
#endif

		return;
	}
	if ( !path.IsValid() || path.Size() <= 0 )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << "CCved::BuildOnApprchObjList: invalid path....exit" << endl;
#endif

		return;

	}

	//
	// Get the proper intersection.
	//
	CIntrsctn currIntrsctn;
	CCrdr currCrdr;
	if( roadPos.IsRoad() )
	{
		currIntrsctn = path.GetNextIntrsctn( roadPos );
		currCrdr = path.GetNextCrdr( roadPos );
		if( ! currIntrsctn.IsValid() )
		{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
			gout << " No upcoming intersection...exit. " << endl;
#endif

			return;
		}
	}
	else
	{
		currIntrsctn = roadPos.GetIntrsctn();
		currCrdr = roadPos.GetCorridor();
	}

	if( ! currCrdr.IsValid() )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " Current corridor is invalid...exit. " << endl;
#endif

		return;
	}


	//
	// Get the onramp corridor and the corridor next to it, since this function
	// is only interested in objects on these two corridors.
	//
	CCrdr crdr1, crdr2;
	/*static*/ vector<CCrdr> allCrdrs;
	allCrdrs.clear();
	allCrdrs.reserve(15);
	currIntrsctn.GetAllCrdrs( allCrdrs );
	TU32b left, right;
	bool foundMrgCrdrs = false;
	for( left = 0; left < allCrdrs.size(); left++ )
	{
		for( right = left + 1; right < allCrdrs.size(); right++ )
		{
			if( allCrdrs[left].GetDstntnLn() == allCrdrs[right].GetDstntnLn() )
			{
				crdr1 = allCrdrs[left];
				crdr2 = allCrdrs[right];
				foundMrgCrdrs = true;
				break;
			}
		}
	}

	if( ! foundMrgCrdrs )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " This is not a situation where the ADO approaches each other...exit. ";
		gout << endl;
#endif

		return;
	}
	//
	// Make sure the current corridor is one of the two corridors.
	//
	if(
	  currCrdr.GetRelativeId() != crdr1.GetRelativeId() &&
	  currCrdr.GetRelativeId() != crdr2.GetRelativeId()
	)
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << ownerObjId << " is not on a merging lane or corridor...exit " << endl;
#endif

		return;
	}

	CCrdr sideCorridor;
	if( currCrdr.GetRelativeId() == crdr1.GetRelativeId() )
	{
		sideCorridor = crdr2;
	}
	if( currCrdr.GetRelativeId() == crdr2.GetRelativeId() )
	{
		sideCorridor = crdr1;
	}

	int sideCorridorId = sideCorridor.GetRelativeId();
	int intrsctnId = currIntrsctn.GetId();
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	double mergDist = 0;
	double distFromOwner;

	//Calculate the distance from the end of the intersection to merge point between side corridor
	//and the current corr.
	vector<double> dists;
	sideCorridor.GetMrgDstFirst( dists );
	TU32b i;
	for( i = 0; i < dists.size(); i++ )
	{
		//
		// The index of the vector matches the corridor Id.
		//
		if( i == currCrdr.GetRelativeId() )
		{
			mergDist = dists[i];
			break;
		}
	}

	//
	// Get objects on the side corridor.
	//
	while( curIdx != 0 )
	{
		//
		// Make sure that this object is not the owner object, part of
		// the object mask, and on the side corridor.
		//
		int objId = m_pDynObjRefPool[curIdx].objId;
		bool notOwnerObj = ownerObjId != objId;
		cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
		bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " The current obj in the pool is:  #" << objId << endl;

#endif

		if( passedFirstTest )
		{
			bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
			bitset<cCV_MAX_CRDRS> sideCorridorMask;
			sideCorridorMask.set( sideCorridorId );



			bool objOnNeededCrdrs = (
						( objCrdrs.to_ulong() == sideCorridorMask.to_ulong() )
						|| ( objCrdrs.to_ulong() & sideCorridorMask.to_ulong() )
						);


			if( objOnNeededCrdrs )
			{

				TObjListInfo node;
				node.objId = objId;

				//
				// Compute the obj's distance to the merging point.
				//

				double objDist;
				objDist = m_pDynObjRefPool[curIdx].crdrDistances[sideCorridorId];

				node.objDist = objDist;
				node.isOnRoad = false;

				double distToMrgPoint = mergDist - objDist;
				distFromOwner = distToMrgPoint;

				node.distFromOwner = distFromOwner;
				// this field needs to be changed later to the distToMrgPoint

				node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
				gout << " The current obj # " << objId <<" is on side corridor " << endl;
				gout << "  **inserting " << node.objId << " to the approaching list.";
				gout << endl;
				gout << " The obj dist is: " << objDist <<  endl;
				gout << " The mergDist is: " << mergDist << endl;
				gout << " The distToMrgPoint is: " << distToMrgPoint << endl;
				gout << " The dist from the owner is:" << node.distFromOwner;
				gout << endl;;
#endif

				apprchObjs.push_back( node );
			}
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}	// while loop


	//
	// Get objects on the sideCorridor's source lane.
	//
	CLane currLane = sideCorridor.GetSrcLn();
	CRoad currRoad = currLane.GetRoad();
	int currRoadId = currRoad.GetId();
	curIdx = m_pRoadRefPool[currRoadId].objIdx;

	//
	// Insert objects into the approaching list until there are no
	// more objects on this road or we reach the maximum
	// number of objects needed.
	//
	while( curIdx != 0 && (int)apprchObjs.size() < maxObjs )
	{

		//
		// Make sure that this object is not the owner object, part of the object mask,
		// and is on the source lane.
		//
		int objId = m_pDynObjRefPool[curIdx].objId;
		bool notOwnerObj = ownerObjId != objId;
		cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
		bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " The current object in the pool is: "<< objId << endl;
		gout << " The road id of the current obj is: " << m_pDynObjRefPool[curIdx].roadId << endl;
#endif

		if( passedFirstTest )
		{
			bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
			bitset<cCV_MAX_LANES> sideCrdrSrcLnMask;
			sideCrdrSrcLnMask.set( currLane.GetRelativeId() );
			bool objOnNeededLanes = (
						( objLanes.to_ulong() == sideCrdrSrcLnMask.to_ulong() )
						|| ( objLanes.to_ulong() & sideCrdrSrcLnMask.to_ulong() )
						);

			double distFromOwner;
			if( objOnNeededLanes )
			{
				//
				// Inserting obj into the list.
				//
				TObjListInfo node;
				node.objId = objId;

				//
				// Compute distance to the merging point.
				//
				double objDist;
				objDist = m_pDynObjRefPool[curIdx].distance;

				node.objDist = objDist;
				node.isOnRoad = true;

				double distToMrgPoint;

				if( currLane.GetDirection() == ePOS )
				{

					distToMrgPoint = mergDist + currRoad.GetLinearLength()
							- objDist;
				}
				else
				{
					distToMrgPoint = mergDist + objDist;
				}

				distFromOwner = distToMrgPoint;

				node.distFromOwner = distFromOwner;
				// this field need to change to a different name later to distToMrgPoint

				node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
				gout << " The current obj # " << objId <<" is on side corridor's source road " << endl;
				gout << "  **inserting " << node.objId << " to the approaching list.";
				gout << endl;
				gout << " The obj dist is: " << objDist <<  endl;
				gout << " The mergDist is: " << mergDist << endl;
				gout << " The distToMrgPoint is: " << distToMrgPoint << endl;
				gout << " The dist from the owner is:" << node.distFromOwner;
				gout << endl;;
#endif

				apprchObjs.push_back( node );
			}
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}	// second while loop.


	//
	// Sort the list in an ascending order according to the distance to merging point.
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < apprchObjs.size(); k++ )
	{
		hold = apprchObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < apprchObjs[loc-1].distFromOwner ) )
		{
			apprchObjs[loc] = apprchObjs[loc-1];
			loc--;
		}
		apprchObjs[loc] = hold;
	}


#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
	gout << " The size of the approching list is:" << apprchObjs.size() << endl;
	vector<TObjListInfo>::iterator itr;
	gout << " The following is the final list for obj #" << ownerObjId<< ":  ";
	for( itr = apprchObjs.begin(); itr != apprchObjs.end(); itr++ )
	{
		gout << " " << itr->objId << " [" << itr->distFromOwner << "]";
	}
	gout << endl;

#endif


}	// end of BuildApprchObjList







//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects behind the
//   specified object in the parameter.
//
// Remarks:	If the roadPos is on road, this function looks at objs on
// the current road, the previous intersection and on one more road further back
// from the intersection. If the roadPos is on intersection, it looks at objs
// on the current intersection and on the road that is the roadPos's source road.
// But it can be extended to search even further back.
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos - The current roadPos of the specified object in the
//                parameter.
//	 prevRoad - The previous road of the current roadPos of the specified
//				  object in the parameter.
//   maxObjs - The maximum number of objects the user is interested in
//                looking at.
//	 objMask - The type of object.
//   backObjs2   - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::BuildBackObjList2(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CRoad& prevRoad,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& backObjs2
			)
{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
	gout << "[" << ownerObjId << "] == BuildBackObjList2 ==================";
	gout << endl;
#endif

	//
	// Error checking.
	//
	if( !roadPos.IsValid() )
	{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
		gout << "invalid roadPos...exit. " << endl;
#endif

		return;
	}
	if( !prevRoad.IsValid() )
	{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
		gout << "invalid prevRoad...exit. " << endl;
#endif

		return;
	}

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
	gout << "roadPos = " << roadPos << endl;
	gout << "prevRoad = " << prevRoad.GetName() << endl;
#endif

	//
	// Initialize variables.
	//
	const int cMAXROADS = 3;
	// This variable is for the maximum number of roads and intersections to
	// look at. When the roadPos is on road, the program searches the
	// road-intersection-road(3 roads); when the roadPos is on
	// intersection, it searches intersection-road(2 roads).
	// But the number of roads in this later case is incremented
	// in the proper place to make it the value of cMAXROADS.

	int numRoads = 0;
	int roadCount = 0;
	int intrsctnCount =  0;
	double distLookedSoFar = 0.0;
	double ownerDist = -101.0;


	// Get owner distance from obj ref list.
	int curIdx;
	if( roadPos.IsRoad() )
	{
		int currRoadId = roadPos.GetRoad().GetId();
		curIdx = m_pRoadRefPool[currRoadId].objIdx;

		while( curIdx != 0 )
		{
			if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
			{
				ownerDist = m_pDynObjRefPool[curIdx].distance;
				break;
			}

			curIdx = m_pDynObjRefPool[curIdx].next;
		}

	}
	else
	{
		int currIntrsctnId = roadPos.GetIntrsctn().GetId();
		curIdx = m_pIntrsctnRefPool[currIntrsctnId].objIdx;

		while( curIdx != 0 )
		{
			if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
			{
				int crdrId = roadPos.GetCorridor().GetRelativeId();
				ownerDist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
				break;
			}

			curIdx = m_pDynObjRefPool[curIdx].next;
		}

	}

	if( ownerDist < -100.0 )
	{
		//cerr << "CCved::BuildBackObjList2: unable to determine ";
		//cerr << "owner object " << ownerObjId << " distance.  Using ";
		//cerr << "dist from given roadPos" << endl;
		ownerDist = roadPos.GetDistance();
	}


	// Check the maximum number of roads
	while ( numRoads < cMAXROADS )
	{
		bool lookOnRoad = true;
		bool lookOnIntrsctn = true;

		//
		// If the roadPos is on intersection, skip looking on road.
		//
		if( ! roadPos.IsRoad() && intrsctnCount == 0 )
		{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " The owner obj is on intersection. " << endl;
#endif

			//
			// Increment the number of roads here so that when the
			// roadPos is on intersection, the program will search
			// only the current intersection and the previous road
			// specified in the parameter.
			//
			numRoads++;

			lookOnRoad = false;
		}

		//
		// Looking on road.
		//
		if( lookOnRoad )
		{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " Looking on road. " << endl;
#endif

			CRoad currRoad;
			bool onSameRoadAsRoadPos = false;

			//
			// Get the road.
			//
			CLane currLane;
			if( roadPos.IsRoad() && roadCount == 0 )
			{
				currLane = roadPos.GetLane();
				currRoad = currLane.GetRoad();
				roadCount++;
				onSameRoadAsRoadPos = true;
			}
			else
			{
				currRoad = prevRoad;
			}


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " The current road is: " << currRoad.GetName() << endl;
#endif

			//
			// If the road is where the roadPos is on, build
			// a lane mask with all lanes traveling in the
			// same direction as the lane in the specified roadPos.
			//
			bitset<cCV_MAX_LANES> laneMask;
			if( onSameRoadAsRoadPos )
			{
				int laneId;
				int numLanes = currRoad.GetNumLanes();
				int firstLaneIdx = currRoad.GetLaneIdx();
				for( laneId = 0; laneId < numLanes; laneId++ )
				{
					cvTLane* pLane = BindLane( firstLaneIdx + laneId );
					if( pLane->direction == currLane.GetDirection() )
					{
						laneMask.set( laneId );
					}
				}

			}

			int currRoadId = currRoad.GetId();
			int curIdx = m_pRoadRefPool[currRoadId].objIdx;


			//
			// Examing the objs on the road.
			//
			while( curIdx != 0 )
			{
				//
				// Make sure that this object is not the owner object, part of the
				// object mask, on the desired lanes and is located within the needed
				// distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );

				//
				// An object should be placed on the owner's back list as long
				// as any part of the object is behind the owner's center.
				//
				cvTObjAttr* pObjAttr = BindObjAttr( objId );
				double objLength = pObjAttr->xSize;

				bool objBack = true;
				if( onSameRoadAsRoadPos )
				{
					if( roadPos.GetLane().GetDirection() == ePOS )
					{
						objBack = (
						m_pDynObjRefPool[curIdx].distance - (objLength * 0.5) <=
						ownerDist
						);
					}
					else
					{
						objBack = (
							m_pDynObjRefPool[curIdx].distance + (objLength * 0.5) >=
							ownerDist
							);
					}
				}

				bool passedFirstTest = notOwnerObj && inObjMask && objBack;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
				gout << " The current object in the pool is: "<< objId << endl;
#endif

				if(  passedFirstTest )
				{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
					gout << " The obj passed first test. " << endl;
#endif

					bool objOnNeededLanes = false;

					if( onSameRoadAsRoadPos )
					{
						//
						// Make sure the object is on at least one of the
						// desired lanes.
						//
						bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
						objLanes &= laneMask;
						objOnNeededLanes = (
									objLanes.any() ||
									( objLanes.to_ulong() == 0 && laneMask.to_ulong() == 0 )
									);
					}
					else
					{
						//
						// If this is not the road where the roadPos is on, then
						// get the objects that have the same direction as the
						// lane the roadPos is located on. ( can't use lane mask to
						// to get the objs on desired lanes here. )
						//
						if( roadPos.IsRoad() )
						{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
							gout << " The obj dir is: " << m_pDynObjRefPool[curIdx].direction;
							gout << endl;
							gout << " The currLane dir is: " << roadPos.GetLane().GetDirection();
							gout << endl;
#endif
							if( m_pDynObjRefPool[curIdx].direction
										== roadPos.GetLane().GetDirection() )
							{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " The two dir is the same. " << endl;
#endif

								objOnNeededLanes = true;

							}
						}
						else
						{
							//
							// If the roadPos is on intersection, use the current corridor's
							// source lane's direction to determine which objects to get
							// on this previous road.
							//
							CLane lane = roadPos.GetCorridor().GetSrcLn();
							if( m_pDynObjRefPool[curIdx].direction == lane.GetDirection() )
							{
								objOnNeededLanes = true;
							}
						}
					}

					bool withinDist = true;

					if( objOnNeededLanes )
					{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
						gout << " The obj is on needed lanes. " << endl;
#endif

						if( onSameRoadAsRoadPos )
						{
							double startDist;
							double endDist;
							if( currLane.GetDirection() == ePOS )
							{
								startDist = 0.0;
								endDist = ownerDist;
							}
							else
							{
								startDist = ownerDist;
								endDist = currRoad.GetLinearLength();
							}
							withinDist = (
									( m_pDynObjRefPool[curIdx].distance >= startDist ) &&
									( m_pDynObjRefPool[curIdx].distance <= endDist )
									);
						}

						if( withinDist )
						{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
							gout << " The obj is with in distance. " << endl;
#endif

							//
							// Compute distance from the owner object to this object.
							//
							double distFromOwner;
							if( onSameRoadAsRoadPos )
							{
								distFromOwner = fabs( ownerDist -
											m_pDynObjRefPool[curIdx].distance );

							}
							else
							{

								if( m_pDynObjRefPool[curIdx].direction == ePOS )
								{
									distFromOwner = distLookedSoFar + prevRoad.GetLinearLength()
												- m_pDynObjRefPool[curIdx].distance;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
									gout << " On road...distLookedSoFar is: " << distLookedSoFar;
									gout << endl;
									gout << " On road...prevRoad length is: ";
									gout << prevRoad.GetLinearLength() << endl;
									gout << " On road...obj dist from pool is: ";
									gout << m_pDynObjRefPool[curIdx].distance << endl;
#endif

								}
								else
								{
									distFromOwner = distLookedSoFar +

											m_pDynObjRefPool[curIdx].distance;
								}


							}

							//
							// Insert the obj into the list without duplicates.
							//
							bool foundDupsOnRoad = false;
							vector<TObjListInfo>::iterator itr;
							for( itr = backObjs2.begin(); itr != backObjs2.end(); itr++ )
							{
									if( (*itr).objId == objId )
									{
										foundDupsOnRoad = true;
										break;
									}
							}
							if( ! foundDupsOnRoad )
							{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " *** inserting obj # " << objId << " to the list. ";
								gout << endl;
#endif

								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;
								backObjs2.push_back( node );
							}

						}	// Within distances

					}	// On needed lanes

				}	// Passed first test

				curIdx = m_pDynObjRefPool[curIdx].next;

			}	// the inside while loop


			//
			// Update distance looked so far.
			//
			if ( onSameRoadAsRoadPos )
			{
				if ( roadPos.GetLane().GetDirection() == ePOS )
				{
					distLookedSoFar = ownerDist;
				}
				else
				{
					distLookedSoFar = currRoad.GetLinearLength() - ownerDist;
				}
			}
			else
			{
				distLookedSoFar += currRoad.GetLinearLength();

			}

			//
			// Increment the number of roads after each road.
			//
			numRoads++;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
				gout << " The list size is: " << backObjs2.size() << endl;
				gout << " The value of maxObjs is: " << maxObjs << endl;
				gout << " The value of numRoads is: " << numRoads << endl;
#endif
			//
			// Quit looking if reached the maximum number of objs or if
			// reached the maximum number of roads.
			//
			if( numRoads >= cMAXROADS || (int)backObjs2.size() >= maxObjs )
			{
				break;
			}

			lookOnRoad = false;
		}	// look on road


		//
		// Look on intersection.
		//
		if( lookOnIntrsctn )
		{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " Looking on intersection. " << endl;
#endif



#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " On intersection...The ownerDist is: " << ownerDist << endl;
#endif

			//
			// Get the intersection.
			//
			CIntrsctn currIntrsctn;
			bool onSameIntrsctnAsRoadPos = false;
			if( ! roadPos.IsRoad() && intrsctnCount == 0 )
			{
				currIntrsctn = roadPos.GetIntrsctn();
				intrsctnCount++;
				onSameIntrsctnAsRoadPos = true;
			}
			else if( roadPos.IsRoad() )
			{
				currIntrsctn = roadPos.GetLane().GetPrevIntrsctn();
				intrsctnCount++;
			}

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " The current intersection is: " << currIntrsctn.GetName() << endl;
#endif

			//
			// Get the desired corridors of the current intersection.
			//
			/*static*/ vector<CCrdr> allCrdrs;
			allCrdrs.clear();
			allCrdrs.reserve(16);
			/*static*/ vector<CCrdr> desiredCrdrs;
			desiredCrdrs.clear();
			desiredCrdrs.reserve(16);
			/*static*/ vector<int> desiredCrdrIds;
			desiredCrdrIds.clear();
			desiredCrdrIds.reserve(16);

			vector<CCrdr>::iterator i;
			currIntrsctn.GetAllCrdrs( allCrdrs );
			for( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
			{
				if( onSameIntrsctnAsRoadPos )
				{
					if( (*i).GetSrcRd() == prevRoad )
					{
						desiredCrdrs.push_back( *i );
						desiredCrdrIds.push_back( (*i).GetRelativeId() );
					}
				}
				else
				{
					if(  (*i).GetDstntnRd() == roadPos.GetLane().GetRoad()  )
//						&& (*i).GetSrcRd() == prevRoad

					{
						desiredCrdrs.push_back( *i );
						desiredCrdrIds.push_back( (*i).GetRelativeId() );
					}
				}
			}


			int intrsctnId = currIntrsctn.GetId();
			int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;

			double currCrdrLength;

			//
			// Examing objs on the intersection.
			//
			while( curIdx != 0 )
			{
				//
				// Make sure that this object is not the owner object, part of
				// the object mask, on the desired corridors and is located
				// within the needed distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );

				bool passedFirstTest = notOwnerObj && inObjMask;

				cvTObjAttr* pObjAttr = BindObjAttr( objId );
				double objLength = pObjAttr->xSize;


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
				gout << " The current obj in the pool is:  #" << objId << endl;

#endif

				if( passedFirstTest )
				{


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
					gout << " The obj passed first test. " << endl;
#endif

					bool objOnNeededCrdrs = false;

					//
					// Build mask for the desired corridors.
					//
					bitset<cCV_MAX_CRDRS> desiredCrdrMask;
					int crdrId;
					for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
					{
						vector<int>::iterator j = find( desiredCrdrIds.begin(),
												desiredCrdrIds.end(), crdrId );
						if( j != desiredCrdrIds.end() )
						{
							desiredCrdrMask.set( crdrId );
						}
					}

					//
					// Make sure the object's corridors overlap the
					// desired corridors.
					//
					bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
					objCrdrs &= desiredCrdrMask;
					objOnNeededCrdrs = (
										objCrdrs.any() ||
										objCrdrs.to_ulong() == 0 &&
										desiredCrdrMask.to_ulong() == 0
										);


					double objDist;
					bool withinDist = true;
					if( objOnNeededCrdrs )
					{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
						gout << " The obj is on needed corridors. " << endl;
#endif

						//
						// Gat a corridor that the object is on.
						//
						int k;
						for( k = 0; k < cCV_MAX_CRDRS; k++ )
						{

							if ( objCrdrs[k] )
							{

								CCrdr crdr( currIntrsctn, k );
								currCrdrLength = crdr.GetLength();

								// Assuming obj has same dist when it is
								// on more than one crdr since the crdrs
								// have same source and detination roads.
								objDist = m_pDynObjRefPool[curIdx].crdrDistances[k];


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " $$$$$$$$$$$$$ obj dist from pool = " << objDist;
								gout << " $$$$$$$$$$$$$ " << endl;
								gout << " The corridor id the obj is on is: ";
								gout << crdr.GetRelativeId();
								gout << endl;
								gout << " The obj's destination road is: ";
								gout << crdr.GetDstntnRd().GetName() << endl;
								gout << " The obj's source road is: ";
								gout << crdr.GetSrcRd().GetName() << endl;
								gout << endl;

#endif

								break;
							}
						}

						if( onSameIntrsctnAsRoadPos )
						{

							withinDist = ownerDist >= ( objDist - ( objLength * 0.5 ) );

						}

						if( withinDist )
						{
#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
							gout << " The obj is within distance. " << endl;
#endif

							//
							// Compute distance from owner.
							//
							double distFromOwner;
							if( onSameIntrsctnAsRoadPos )
							{
								distFromOwner = ownerDist - objDist;

							}
							else
							{

								distFromOwner = fabs( distLookedSoFar + currCrdrLength -
																objDist );

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " distFromOwner is: " << distFromOwner;
								gout << endl;
								gout << " distLookedSoFar is: " << distLookedSoFar << endl;
								gout << " current corridor length is: " << currCrdrLength << endl;
								gout << " current obj dist is: " << objDist << endl;
#endif

							}

							//
							// Insert the obj into the list without duplicates.
							//
							bool foundDupsOnIntrsctn = false;
							vector<TObjListInfo>::iterator itr;
							for( itr = backObjs2.begin(); itr != backObjs2.end(); itr++ )
							{
									if( (*itr).objId == objId )
									{
										foundDupsOnIntrsctn = true;
										break;
									}
							}

							if( ! foundDupsOnIntrsctn )
							{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " *** inserting obj # " << objId << " to the list. " << endl;
#endif
								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;
								backObjs2.push_back( node );
							}

						}	// Within distance.

					}	// On needed corridors

				}	// Passed first test

				curIdx = m_pDynObjRefPool[curIdx].next;
			}	// while loop

			//
			// Update distance looked so far by picking
			// a proper corridor. This is for cases when there is no
			// obj on the intersection or when the obj on the intersection
			// is the owner himself.
			CCrdr crdrPicked = desiredCrdrs[0];
			currCrdrLength = crdrPicked.GetLength();
			distLookedSoFar += currCrdrLength;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " picked crdr length = " << currCrdrLength << endl;
#endif


			if( onSameIntrsctnAsRoadPos )
			{
				distLookedSoFar = ownerDist;
			}
			else
			{
				distLookedSoFar += currCrdrLength;
			}

			lookOnIntrsctn = false;

			if( (int)backObjs2.size() >= maxObjs )
			{
				break;
			}

			//
			// Increment the number of roads after looking at
			// each intersection.
			numRoads++;

		}	// Look on intersection


	}	// end of while loop

	//
	// Sort the list by distance to the owner.
	//
	TObjListInfo hold;
	unsigned int k, loc;
	for( k =1; k < backObjs2.size(); k++ )
	{
		hold = backObjs2[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < backObjs2[loc-1].distFromOwner ) )
		{
			backObjs2[loc] = backObjs2[loc-1];
			loc--;
		}
		backObjs2[loc] = hold;
	}


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
	gout << " The size of the back list2 is:" << backObjs2.size() << endl;
	vector<TObjListInfo>::iterator i;
	gout << " The following is the final list for obj #" << ownerObjId<< ":  ";
	for( i = backObjs2.begin(); i != backObjs2.end(); i++ )
	{
		gout << " " << i->objId << " [" << i->distFromOwner << "]";
	}
	gout << endl;

#endif


}	// end of BuildBackObjList2



//////////////////////////////////////////////////////////////////////////////
//
// Description:	GetObjWithClosestDistOnCrdr(private)
//	This function obtains the vehicle that is before and closest to the intersecting
//	point of a corridor.
//
// Remarks:
//
// Arguments:
//	intersectingCrdrLength - The length of the corridor to be examined.
//	intersectingPointDist - The distance where the related corridors intersect with.
//	objs - A vector of all the objs on the corridor to be examined.
//
// Returns:
//	It returns the id of the closest vehicle desired. When there is no vehicle found,
//  it returns -1.
//


////////////////////////////////////////////////////////////////////////////////////////

int
CCved::GetObjWithClosestDistOnCrdr(
							 double intersectingCrdrLength,
							 double intersectingPointDist,
							 vector<TObjWithDist>& objs )
{
	int closestObjId;
	bool hasObjBeforeIntersectingPoint = false;
	double smallest;
	double diff;
	int count = 0;

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
	gout << " ============GetObjWithClosestDistOnCrdr========" << endl;
	gout << " count initialized val = " << count << endl;
#endif

	// Iterate through the objects on the corridor.
	vector<TObjWithDist>::iterator i;
	for( i = objs.begin(); i != objs.end(); i++ )
	{
		int objId = (*i).objId;
		double objDist = (*i).dist;

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
		gout << " The current obj on the intersecting crdr is: " << objId << " ";
		gout << " obj dist =  " << objDist << endl;
		gout << " intersectingPointDist = " << intersectingPointDist << endl;
		gout << " intersecting crdr length = " << intersectingCrdrLength << endl;
#endif

		// When the input vector contains objects from a corridor,
		// include the obj on boundary of the intersecting corridor and
		// its source road.
#if 0
		if( objDist * 1.5f >= intersectingCrdrLength )
		{
			if (BindObj(objId)){
				gout << "Error Invalid Length along corridor "<<objDist<< " For Object :" <<BindObj(objId)->name<<endl;
				gout << "Corridor Length :"<<intersectingCrdrLength<<endl;
			}else{
				gout << "Error Invalid Length along corridor "<<objDist<< " For Object with invalid id of:" <<objId<<endl;
				gout << "Corridor Length :"<<intersectingCrdrLength<<endl;
			}
			//continue;
			//objDist = 0.0;
		}
#endif

		// Make sure a vehicle is returned as the closest vehicle when it is
		// half way passed the given intersecting distance.
		cvTObjAttr* pObjAttr = BindObjAttr( objId );
		double objLength = pObjAttr->xSize;
		objDist = objDist - ( objLength * 0.5 );

		// Elimilate those vehicles passed the given intersectingPointDist.
		if ( objDist <= intersectingPointDist )
		{
			hasObjBeforeIntersectingPoint = true;
			diff = fabs( intersectingPointDist - objDist );

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
			gout << " objDist <= intersecting point dist. " << endl;
			gout << " count should be 0, count  = " << count << endl;
			gout << " diff = " << diff << endl;
#endif

			if( count == 0 )
			{
				smallest = diff;
				closestObjId = objId;

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
				gout << " The current obj when count is 0 is: " << objId << endl;
				gout << " smallest = diff when count is 0: " << endl;
				gout << " smallest when count is 0 is: " << smallest << endl;
				gout << " diff when count is 0 is " << diff << endl;
				gout << " closestObjId when counter is 0 is: " << closestObjId;
				gout << endl;
#endif


			}
			else
			{
#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
				gout << " count > 0 " << endl;
#endif

				if( diff < smallest )
				{
					smallest = diff;

					closestObjId = objId;

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
				gout << " The current obj when count > 0 is: " << objId << endl;
				gout << " smallest = diff when count > 0 : " << endl;
				gout << " smallest when count > 0 is: " << smallest << endl; ;
				gout << " diff when count > 0 is: " << diff << endl;
				gout << " closestObjId when counter > 0 is: " << closestObjId;
				gout << endl;
#endif


				}
			}
			count++;

		}
	}	// end of for
	if( ! hasObjBeforeIntersectingPoint )
	{
		// All objs have passed the given intersectingPointDist
		return -1;
	}
	else
	{

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_CRDR
		gout << " returned value = " << closestObjId << endl;
#endif

		return closestObjId;
	}


}	// end of GetObjWithClosestDistOnCrdr


//////////////////////////////////////////////////////////////////////////////
//
// Description:	GetObjWithClosestDistOnLane(private)
//	This function obtains the first vehicle on a corridor's source lane.
//
//
// Remarks:
//
// Arguments:
//	srcLane - The source lane.
//	objs - A vector of object ids with their distances.
//
// Returns:
//	It returns the id of the closest vehicle desired. When there is no vehicle found,
//  it returns -1.
//
////////////////////////////////////////////////////////////////////////////////////////
int
CCved::GetObjWithClosestDistOnLane(
							 CLane& srcLane,
							 vector<TObjWithDist>& objs )
{
	int closestObjId;
	double smallest;
	double diff;
	int count = 0;

	// Iterate through the objects on the lane.
	vector<TObjWithDist>::iterator i;
	for( i = objs.begin(); i != objs.end(); i++ )
	{
		int objId = (*i).objId;
		double objDist = (*i).dist;


#ifdef DEBUG_GET_CLOSEST_OBJ_ON_LANE
		gout << " The current obj on the lane = " << objId << " ";
		gout << " obj dist =  " << objDist << endl;
#endif

		// Compute how close is the vehicle to the target corridor.
		if( srcLane.GetDirection() == ePOS )
		{
			double roadLength = srcLane.GetRoad().GetLinearLength();
			diff = roadLength - objDist;
		}
		else
		{
			diff = objDist;
		}

		// Find the shortest distance.
		if( count == 0 )
		{
			smallest = diff;
			closestObjId = objId;

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_LANE
			gout << " The current obj when count is 0 is: " << objId << endl;
			gout << " smallest = diff when count is 0: " << endl;
			gout << " smallest when count is 0 is: " << smallest << endl;
			gout << " diff when count is 0 is " << diff << endl;
			gout << " closestObjId when counter is 0 is: " << closestObjId;
			gout << endl;
#endif


		}
		else
		{
			if( diff < smallest )
			{
				smallest = diff;

				closestObjId = objId;

#ifdef DEBUG_GET_CLOSEST_OBJ_ON_LANE
				gout << " The current obj when count > 0 is: " << objId << endl;
				gout << " smallest = diff when count > 0 : " << endl;
				gout << " smallest when count > 0 is: " << smallest << endl; ;
				gout << " diff when count > 0 is: " << diff << endl;
				gout << " closestObjId when counter > 0 is: " << closestObjId;
				gout << endl;
#endif

			}
		}

		count++;
	}	// end of for
	return closestObjId;

}	// end of GetObjWithClosestDistOnLane




//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function obtains the first vehicle on an intersecting corridor, or
//   the first vehicle on its source lane when there is no vehicle on this
//	 intersecting corridor.
//
// Remarks:
//
// Arguments:
//	crdrId - The id of the corridor that intersects with the intersecting corridor.
//	interCrdr - An intersecting corridor.
//	objMask - The object mask indicating the type of object.
//
// Returns:
//	It returns the id of the first vehicle on the intersecting corridor or on its source
//	lane. When there is no vehicle found, it returns -1.
////////////////////////////////////////////////////////////////////////////////////////
int
CCved::GetFirstObjOnIntrsctingCrdr(
								int crdrId,
								const CCrdr& interCrdr,
							    const CObjTypeMask& objMask
								)
{
	int firstObjId;
	/*static*/ vector<TObjWithDist> objs;
	objs.clear();
	objs.reserve(10);
	int interCrdrId = interCrdr.GetRelativeId();
	int intrsctnId = interCrdr.GetIntrsctn().GetId();
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;

#ifdef DEBUG_FIRST_OBJ
	gout << "========GetFistObjOnIntrsctingCrdr=======" << endl;
	gout << " intersecting crdr id = " << interCrdr.GetRelativeId() << endl;
#endif

	// Error checking.
	if (m_pIntrsctnRefPool == 0)
	{

#ifdef DEBUG_FIRST_OBJ
		gout << "Maintainer has not been run yet. " << endl;
#endif

		return -1;
	}

	// Get all the vehicles on the intersecting corridor.
	GetAllDynObjsOnCrdr(intrsctnId, interCrdrId, objs, objMask);

#ifdef DEBUG_FIRST_OBJ
	gout << " The number of objs on the intersecting corridor is: ";
	gout << objs.size() << endl;
#endif

	bool found = false;
	bool lookOnSrcLane = false;

	// If there are vehicles before the intersecting point on the
	// intersecting corridor, get the vehicle closest to the intersecting point.
	if( objs.size() > 0 )
	{
		// Get the last intersecting distance of this intersecting corridor
		// with the other corridor.
		double intrsctingDist;
		/*static*/ vector<double> dists;

		//double intrsctingDist;
		/*static*/

		interCrdr.GetMrgDstLast( dists );
		unsigned int i;

		for( i = 0; i < dists.size(); i++ )
		{
			// The index of the vector matches the corridor Id.
			if( i ==crdrId )
			{
				found = true;
				intrsctingDist = dists[i];

#ifdef DEBUG_FIRST_OBJ
				gout << " intrsctingDist of crdr: " << interCrdrId;
			    gout << " with " << crdrId << " = " << intrsctingDist << endl;
#endif

				break;
			}
		}
		if( !found )
		{

#ifdef DEBUG_FIRST_OBJ
			gout << " No intersecting dist found...Maybe the two corridors in the ";
			gout << " parameter don't intersect with each other????" << endl;
#endif
			return -1;
		}
        if (intrsctingDist < 0){
            //if the last point of itersection is -1, chances are this intersection
            //merges in with our target.
            vector<double> firstdist;
            int id = i;
            interCrdr.GetMrgDstFirst(firstdist);
            if (firstdist[i] > 0){
                intrsctingDist = interCrdr.GetLength();
            }
        }
		// Get obj closest to the intersecting point.
		double interCrdrLength = interCrdr.GetLength();
		firstObjId = GetObjWithClosestDistOnCrdr( interCrdrLength, intrsctingDist, objs );
		if( firstObjId == -1 )
		{
			lookOnSrcLane = true;
		}
		else
		{
			return firstObjId;
		}
	}	// look at the intersecting corridor.
	if( objs.size() == 0 || lookOnSrcLane )
	{
		// If there is no vehicle on the intersecting corridor or the vehicles
		// on the intersecting corridor have passed the intersecting point,
		// then look at its source lane.

		CLane srcLane = interCrdr.GetSrcLn();

#ifdef DEBUG_FIRST_OBJ
		gout << " Looking at the source lane since there is no vehicle on corridor ";
		gout << endl;
		gout << " srcRoad = " << srcLane.GetRoad().GetName();
		gout << " srcLane = " << srcLane.GetId() << endl;

#endif

		// Get all vehicles on the source lane.
		GetAllDynObjsOnLane( srcLane, objs, objMask );

		// If there is no obj on the source lane either
		if( objs.size() == 0 )
		{

#ifdef	DEBUG_FIRST_OBJ
			gout << " There are no obj on source lane either. " << endl;
#endif

			return -1;
		}
		else
		{

#ifdef DEBUG_FIRST_OBJ
			gout << " The number of objs on the source lane is: ";
			gout << objs.size() << endl;
#endif

			firstObjId = GetObjWithClosestDistOnLane( srcLane, objs );
			return firstObjId;
		}

	}	// look at the source lane
	else  return -1;

}	// end of GetFirstObjOnIntrsctingCrdr


//////////////////////////////////////////////////////////////////////////////
//
// Description:	GetClosestObjBehindOnCrdr(private)
//	This function obtains the vehicle that is before the intersecting
//	point of a corridor and is right behind the object specified in the
//  parameter obj.
//
// Remarks:
//
// Arguments:
//  obj - An object id and need to return the object right behind it.
//	intersectingCrdrLength - The length of the corridor to be examined.
//	intersectingPointDist - The distance where the related corridors intersect with.
//	objs - A vector of object ids with their distances.
//
// Returns:
//	It returns the id of the desired object. When there is no vehicle found,
//  it returns -1. When the specified object in the parameter is on a corridor
//  but there is no object behind it on the corridor, return -2 which means
//  it is necessary to get the first object on the source lane if any.
//
////////////////////////////////////////////////////////////////////////////////////////
int
CCved::GetClosestObjBehindOnCrdr(
							 int obj,
							 double intersectingCrdrLength,
							 double intersectingPointDist,
							 vector<TObjWithDist>& objs )
{
	bool hasObjBeforeIntersectingPoint = false;
	bool found = false;
	double diff;
	vector<TObjWithDist> temp;


#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
	gout << " ============GetClosestObjBehindOnCrdr========" << endl;
#endif

	// Get the distance of the 'obj'
	double objDistance = -1;
	vector<TObjWithDist>::iterator itr;
	for( itr = objs.begin(); itr != objs.end(); itr++ )
	{
		int objId = (*itr).objId;

		if( objId == obj )
		{
			objDistance = (*itr).dist;
			break;
		}
	}

	// Iterate through the objects on the corridor.
	vector<TObjWithDist>::iterator i;
	for( i = objs.begin(); i != objs.end(); i++ )
	{
		int objId = (*i).objId;
		double objDist = (*i).dist;

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
		gout << " The current obj on the intersecting crdr is: " << objId << " ";
		gout << " obj dist =  " << objDist << endl;
		gout << " intersectingPointDist = " << intersectingPointDist << endl;
		gout << " intersecting crdr length = " << intersectingCrdrLength << endl;
#endif

		// When the input vector contains objects from a corridor,
		// include the obj on boundary of the intersecting corridor and
		// its source road.
#if 0
		if( objDist * 1.5 >= intersectingCrdrLength )
		{
			if (BindObj(objId)){
				gout << "Error Invalid Length along corridor "<<objDist<< " For Object :" <<BindObj(objId)->name<<endl;
				gout << "Corridor Length :"<<intersectingCrdrLength<<endl;
			}else{
				gout << "Error Invalid Length along corridor "<<objDist<< " For Object with invalid id of:" <<objId<<endl;
				gout << "Corridor Length :"<<intersectingCrdrLength<<endl;
			}
			//continue;
			//objDist = 0.0;
		}
#endif
		// Elimilate those vehicles ahead of the 'obj'.
		if ( objDist <= objDistance )
		{
			hasObjBeforeIntersectingPoint = true;
			diff = objDistance - objDist;

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
			gout << " objDist <= owner object distance " << endl;
			gout << " diff = " << diff << endl;
#endif

			TObjWithDist node;
			node.objId = objId;
			node.dist = diff;
			temp.push_back(node);
		}
	}	// end of for

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
	gout << " Size of temp = " << temp.size() << endl;
#endif

	// Sort by distances to the intersecting point.
	TObjWithDist hold;
	unsigned int k, loc;
	for( k =1; k < temp.size(); k++ )
	{
		hold = temp[k];
		loc = k;
		while( 0 < loc && ( hold.dist < temp[loc-1].dist ) )
		{
			temp[loc] = temp[loc-1];
			loc--;
		}
		temp[loc] = hold;
	}

	// Retrieve the object desired.
	int objDesired;
	TU32b count = 0;
	for( TU32b j = 0; j < temp.size(); j++ )
	{
		if( found )
		{
			objDesired = temp[j].objId;

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
			gout << " obj desired = " << objDesired << endl;
#endif
			break;
		}

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
			gout << " temp[j].objId = " << temp[j].objId << endl;
#endif

		if( temp[j].objId == obj )
		{

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
			gout << " temp[j].objId = " << temp[j].objId << endl;
			gout << " obj in parameter = " << obj << endl;
#endif

			found = true;
		}
		count++;
		if( count == temp.size() )
		{
			return -2;
		}

	}


	// If all the objects have passed the intersecting point or
	// if there is no more object behind the specified object in
	// the parameter.
	if( ! hasObjBeforeIntersectingPoint || !found )
	{
		return -1;
	}

	else
	{

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_CRDR
		gout << " returned value = " << objDesired << endl;
#endif

		return objDesired;
	}


}	// end of GetClosestObjBehindOnCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description:	GetClosestObjBehindOnLane(private)
//	This function obtains the object behind the object specified in the parameter
//  on a corridor's source lane.
//
//
// Remarks:
//
// Arguments:
//  obj - An object and need to return the object right behind it.
//	srcLane - The source lane.
//	objs - A vector of object ids with their distances.
//
// Returns:
//	It returns the id of the desired object. When there is no vehicle found,
//  it returns -1.
//
////////////////////////////////////////////////////////////////////////////////////////
int
CCved::GetClosestObjBehindOnLane(
							 int obj,
							 CLane& srcLane,
							 vector<TObjWithDist>& objs )
{
	int objDesired;
	double diff;
	vector<TObjWithDist> temp;
	bool found = false;

	// Iterate through the objects on the lane.
	vector<TObjWithDist>::iterator i;
	for( i = objs.begin(); i != objs.end(); i++ )
	{
		int objId = (*i).objId;
		double objDist = (*i).dist;


#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_LANE
		gout << " The current obj on the lane = " << objId << " ";
		gout << " obj dist =  " << objDist << endl;
#endif

		// Compute how close is the vehicle to the target corridor.
		if( srcLane.GetDirection() == ePOS )
		{
			double roadLength = srcLane.GetRoad().GetLinearLength();
			diff = roadLength - objDist;
		}
		else
		{
			diff = objDist;
		}

		TObjWithDist node;
		node.objId = objId;
		node.dist = diff;
		temp.push_back( node );
	}

	// Sort by distances to the intersecting point.
	TObjWithDist hold;
	unsigned int k, loc;
	for( k =1; k < temp.size(); k++ )
	{
		hold = temp[k];
		loc = k;
		while( 0 < loc && ( hold.dist < temp[loc-1].dist ) )
		{
			temp[loc] = temp[loc-1];
			loc--;
		}
		temp[loc] = hold;
	}

	unsigned int count = 0;
	for( unsigned int j = 0; j < temp.size(); j++ )
	{
		if( found )
		{
			objDesired = temp[j].objId;

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_LANE
			gout << " obj desired = " << objDesired << endl;
#endif
			break;
		}

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_LANE
			gout << " temp[j].objId = " << temp[j].objId << endl;
#endif

		if( temp[j].objId == obj )
		{

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_LANE
			gout << " temp[j].objId = " << temp[j].objId << endl;
			gout << " obj in parameter = " << obj << endl;
#endif

			found = true;
		}
		count++;
		if( count == temp.size() )
		{
			return -1;
		}

	}

	if( !found )
	{
		return -1;
	}

	else
	{

#ifdef DEBUG_GET_CLOSEST_OBJ_BEHIND_ON_LANE
		gout << " returned value = " << objDesired << endl;
#endif

		return objDesired;
	}

}	// end of GetClosestObjBehindOnLane



//////////////////////////////////////////////////////////////////////////////
//
// Description:
//	This function obtains the object behind the object specified in the parameter obj.
//
// Remarks:
//
// Arguments:
//	obj - An object and need to return the object behind it.
//	crdrId - The id of the corridor that intersects with the intersecting corridor.
//	interCrdr - An intersecting corridor.
//	objMask - The object mask indicating the type of object.
//
// Returns:
//	It returns the id of the object behind the object specified in the parameter.
//  When there is no object found, it returns -1.
////////////////////////////////////////////////////////////////////////////////////////
int
CCved::GetObjOnCrdrBehindObj(
							int obj,
							int crdrId,
							const CCrdr& interCrdr,
							const CObjTypeMask& objMask
							)
{
	int objId;
	/*static*/ vector<TObjWithDist> objs;
	objs.clear();
	objs.reserve(30);
	int interCrdrId = interCrdr.GetRelativeId();
	int intrsctnId = interCrdr.GetIntrsctn().GetId();
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
	gout << "========GetObjOnCrdrBehindObj=======" << endl;
	gout << " intersecting crdr id = " << interCrdr.GetRelativeId() << endl;
#endif

	// Error checking.
	if (m_pIntrsctnRefPool == 0)
	{

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
		gout << "Maintainer has not been run yet. " << endl;
#endif

		return -1;
	}

	// Get all the vehicles on the intersecting corridor.
	GetAllDynObjsOnCrdr(intrsctnId, interCrdrId, objs, objMask);

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
	gout << " The number of objs on the intersecting corridor is: ";
	gout << objs.size() << endl;
#endif

	bool found = false;
	bool lookOnSrcLane = false;
	bool getFirstOnLane = false;

	// If there are vehicles before the intersecting point on the
	// intersecting corridor, get the vehicle closest to the intersecting point.
	if( objs.size() > 0 )
	{
		// Get the intersecting distance of this intersecting corridor
		// with the other corridor.
		double intrsctingDist;
		/*static*/ vector<double> dists;
		dists.clear();
		dists.reserve(30);
		interCrdr.GetMrgDstFirst( dists );
		unsigned int i;

		for( i = 0; i < dists.size(); i++ )
		{
			// The index of the vector matches the corridor Id.
			if( i ==crdrId )
			{
				found = true;
				intrsctingDist = dists[i];

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
				gout << " intrsctingDist of crdr: " << interCrdrId;
			    gout << " with " << crdrId << " = " << intrsctingDist << endl;
#endif

				break;
			}
		}
		if( !found )
		{

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
			gout << " No intersecting dist found...Maybe the two corridors in the ";
			gout << " parameter don't intersect with each other????" << endl;
#endif
			return -1;
		}

		// Get obj closest to the intersecting point.
		double interCrdrLength = interCrdr.GetLength();
		objId = GetClosestObjBehindOnCrdr( obj, interCrdrLength, intrsctingDist, objs );
		if( objId == -1 )
		{
			lookOnSrcLane = true;
		}
		else if( objId == -2 )
		{
			lookOnSrcLane = true;
			getFirstOnLane = true;
		}
		else
		{
			return objId;
		}
	}	// look at the intersecting corridor.
	if( objs.size() == 0 || lookOnSrcLane )
	{
		// If there is no vehicle on the intersecting corridor or the vehicles
		// on the intersecting corridor have passed the intersecting point,
		// then look at its source lane.

		CLane srcLane = interCrdr.GetSrcLn();

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
		gout << " Looking at the source lane since there is no vehicle(behind) on corridor ";
		gout << endl;
		gout << " srcRoad = " << srcLane.GetRoad().GetName();
		gout << " srcLane = " << srcLane.GetId() << endl;

#endif

		// Get all vehicles on the source lane.
		GetAllDynObjsOnLane( srcLane, objs, objMask );
        if (getFirstOnLane){
            for (unsigned int i = 0; i < objs.size(); i++){
                if (objs[i].objId == obj){
                    getFirstOnLane = false;
                }
            }
        }

		// If there is no obj on the source lane either
		if( objs.size() == 0 )
		{

#ifdef	DEBUG_FIRST_OBJ
			gout << " There are no obj on source lane either. " << endl;
#endif

			return -1;
		}
		else
		{

#ifdef DEBUG_GET_OBJ_BEHIND_OBJ
			gout << " The number of objs on the source lane is: ";
			gout << objs.size() << endl;
#endif

			if( getFirstOnLane )
			{
				objId = GetObjWithClosestDistOnLane( srcLane, objs );
				if( objId != obj )
				{
					return objId;
				}
				else
					return -1;
			}
			else
			{
				objId = GetClosestObjBehindOnLane( obj, srcLane, objs );
				return objId;
			}
		}

	}	// look at the source lane
	else  return -1;
}	// end of GetObjOnCrdrBehindObj
//////////////////////////////////////////////////////////////////////////////
///
///\brief
///	This function provides driver information.
///
///\remark
///  If own vehicle is between lanes or corridors, a -1 is
///  returned for these distances: the width of lane, the dist from
///  center of own vehicle to left edge of lane or corridor, the dist
///  from center of own vehicle to right edge of lane or corridor, and
///  the dist from center of own vehicle to right edge of the rightmost
///  lane or corridor. Also note that the function returns false when
///  the own vehicle has no lead vehicle.
///
/// Arguments:
///
///\par	objMask      - The type of the object.
///\par	positionInfo - (output) A structure that has such fields: laneDeviation,
///					    distToLeftEdge, distToRightEdge, distToRightmostEdge.
///
///\par	 followInfo  - (output) A structure that has such fields: followDist,
///				       bumperDist, followTime, ttc, leadObjInfo.
///				       (Note: Please refer to the design document of the funciton
///				       for details on the output parameters)
///
/// Returns:
///\return	True if all the desired information is successfully obtained; false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool
CCved::GetOwnVehicleInfo(
				 const CObjTypeMask& objMask,
				 TOwnVehiclePositionInfo& positionInfo,
				 TOwnVehicleFollowInfo& followInfo
				)
{

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " ===== CCVed::GetOwnVehicleInfo ======" << endl;
#endif


	// If there is no driver, return false
	if ( ! m_haveFakeExternalDriver )	return false;


	// Obtain the driver's position from CVED.
	CPoint3D ownVehPos;
	bool gotPos = GetOwnVehiclePos( ownVehPos );
	if( !gotPos )	return false;
	CRoadPos ownVehRoadPos( *this, ownVehPos );
	if( !ownVehRoadPos.IsValid() )	return false;

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " ownVehRoadPos = " << ownVehRoadPos << endl;
#endif


	// Compute lane deviation or get offset from center of own
	// vehicle to the center of lane or corridor.
	double laneDeviation;
	CCrdr ownVehCrdr;
	int ownVehCrdrId;

	if( ownVehRoadPos.IsRoad() )
	{
		// On road.
		laneDeviation = ownVehRoadPos.GetOffset();
	}
	else
	{
		// On intersection.
		ownVehCrdr = ownVehRoadPos.GetCorridor();
		if ( ! ownVehCrdr.IsValid() )	return false;
		ownVehCrdrId = ownVehCrdr.GetRelativeId();
		laneDeviation = ownVehRoadPos.GetOffset( ownVehCrdrId );
	}

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " laneDeviation = " << laneDeviation << endl;
#endif


	// Compute distance from own vehicle's center to the left
	// edge of the vehicle's lane or corridor. Return -1
	// when own vehicle is between lanes or corridors.
	double distToLeftEdge;
	double laneWidth;
	double ownVehDist;
	CLane ownVehLane;
	int ownVehLaneId;
	CRoad ownVehRoad;
	int ownVehRoadId;
	bool computeLeftEdge = false;

	cvTObjAttr* pOwnVehAttr = BindObjAttr( 0 );
	double ownVehLength = pOwnVehAttr->xSize;
	double ownVehWidth = pOwnVehAttr->ySize;

	double distFromOwnVehEdgeToLaneEdge;

	if( ownVehRoadPos.IsRoad() )
	{
		// On road.
		ownVehLane = ownVehRoadPos.GetLane();
		if( !ownVehLane.IsValid() )		return false;
		ownVehLaneId = ownVehLane.GetRelativeId();
		ownVehRoad = ownVehLane.GetRoad();
		if( !ownVehRoad.IsValid() )		return false;
		ownVehRoadId = ownVehRoad.GetId();


		ownVehDist = ownVehRoadPos.GetDistance();
		laneWidth = ownVehLane.GetWidth( ownVehDist );


#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
			gout << " lane id = " << ownVehLaneId << endl;
			gout << " obj dist along road = " << ownVehDist << endl;
#endif

		distFromOwnVehEdgeToLaneEdge = ( laneWidth / 2 ) - ( ownVehWidth / 2 )- fabs( laneDeviation );
		if( distFromOwnVehEdgeToLaneEdge < 0 )
		{
			laneWidth = -1;
			distToLeftEdge = -1;

		}
		else
		{
			computeLeftEdge = true;

		}

	}
	else
	{
		// On intersection.
		ownVehDist = ownVehRoadPos.GetDistance();
		laneWidth = ownVehCrdr.GetWidth( ownVehDist );
		distFromOwnVehEdgeToLaneEdge = ( laneWidth / 2 ) - ( ownVehWidth / 2 )- fabs( laneDeviation );
		if( distFromOwnVehEdgeToLaneEdge < 0 )
		{
			laneWidth = -1;
			distToLeftEdge = -1;

		}
		else
		{

			computeLeftEdge = true;

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
			gout << " corridor id = " << ownVehCrdrId << endl;
			gout << " obj dist along corridor = " << ownVehDist << endl;
#endif
		}

	}

	if( computeLeftEdge )
	{
		if( laneDeviation < 0 )
		{
			distToLeftEdge = ( laneWidth * 0.5 ) - fabs( laneDeviation );
		}
		else
		{
			distToLeftEdge = ( laneWidth * 0.5 ) + fabs( laneDeviation );
		}
	}



#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " lane or corridor width = " << laneWidth << endl;
	gout << " dist to left edge of lane or crdr = " << distToLeftEdge << endl;
#endif



	// Compute distance from own vehicle's center to the right edge of
	// the vehicle's lane or corridor. Return -1 when own vehicle is
	// between lanes or corridors.
	double distToRightEdge;
	bool computeRightEdge = false;

	if( ownVehRoadPos.IsRoad() )
	{
		// On road.
		if( distFromOwnVehEdgeToLaneEdge < 0 )	distToRightEdge = -1;
		else	computeRightEdge = true;

	}
	else
	{
		// On intersection
		if( distFromOwnVehEdgeToLaneEdge < 0 )	distToRightEdge = -1;
		else	computeRightEdge = true;

	}

	if( computeRightEdge )
	{
		if( laneDeviation < 0 )
		{
			distToRightEdge = ( laneWidth * 0.5 ) + fabs( laneDeviation );
		}
		else
		{
			distToRightEdge = ( laneWidth * 0.5 ) - fabs( laneDeviation );
		}
	}

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " dist to right edge of lane or crdr = " << distToRightEdge << endl;
#endif


	// Compute distance from own vehicle's center to the right edge of
	// the rightmost lane or corridor of the vehicle's road.
	double distToRightmostEdge;
	CLane rightLane;
	CLane rightSrcLane;
	double totalLaneWidths = 0.0;
	CLane ownVehSrcLane;
	int ownVehSrcLaneId;

	if( laneWidth == -1 )
	{
		distToRightmostEdge = -1;
	}
	else
	{
		if( ownVehRoadPos.IsRoad() )
		{
			// On road.
			if ( ownVehLane.IsRightMost() )
			{
				distToRightmostEdge = distToRightEdge;
			}
			else
			{
				rightLane = ownVehLane.GetRight();
				if( !rightLane.IsValid() )
				{
					return false;
				}
				totalLaneWidths += rightLane.GetWidth();
				while( !rightLane.IsRightMost() && rightLane.IsValid() )
				{
					rightLane = rightLane.GetRight();
					totalLaneWidths += rightLane.GetWidth();
				}
				distToRightmostEdge = distToRightEdge + totalLaneWidths;
			}
		}	// on road
		else
		{
			// On intersection.
			// (Use the corridor's source lane to check if
			// the own vehicle is on the rightmost lane, etc. The
			// merging lane(s) is ignored.)
			ownVehSrcLane = ownVehRoadPos.GetCorridor().GetSrcLn();
			if( !ownVehSrcLane.IsValid() )		return false;
			ownVehSrcLaneId = ownVehSrcLane.GetRelativeId();

			if ( ownVehSrcLane.IsRightMost() )
			{
				distToRightmostEdge = distToRightEdge;
			}
			else
			{
				rightSrcLane = ownVehSrcLane.GetRight();
				if( !rightSrcLane.IsValid() )
				{
					return false;
				}
				totalLaneWidths += rightSrcLane.GetWidth();
				while( !rightSrcLane.IsRightMost() && rightSrcLane.IsValid() )
				{
					rightSrcLane = rightSrcLane.GetRight();
					totalLaneWidths += rightSrcLane.GetWidth();
				}
				distToRightmostEdge = distToRightEdge + totalLaneWidths;
			}

		}// on intersection
	}	// when lane or corridor width is not -1.

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " dist to right edge of rightmost lane or crdr = " << distToRightmostEdge;
	gout << endl;
#endif


	// Output part of driver info to a structure.
	positionInfo.laneDeviation = laneDeviation;
	positionInfo.distToLeftEdge = distToLeftEdge;
	positionInfo.distToRightEdge = distToRightEdge;
	positionInfo.distToRightmostEdge = distToRightmostEdge;


	// Get the lead vehicle of the own vehicle and the
	// follow distance.
	int leadVehId;
	double followDist;
	bool hasLeadObj = GetLeadObj( 0, leadVehId, followDist );

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " value of hasLeadObj = " << hasLeadObj << endl;
#endif

	if( hasLeadObj )
	{


#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
		gout << " lead obj id = " << leadVehId << endl;
		gout << " follow dist = " << followDist << endl;
#endif

	}
	else
	{

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
		gout << " no lead vehicle ... return false ." << endl;
#endif

		return false;
	}


	// Compute bumper to bumper distance from own vehicle to lead vehicle.
	double bumperDist;
	cvTObjAttr* pLeadVehAttr = BindObjAttr( leadVehId );
	double leadVehLength = pLeadVehAttr->xSize;
	bumperDist = followDist - ( ownVehLength * 0.5 ) - (leadVehLength * 0.5 );

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " own vehicle length = " << ownVehLength << endl;
	gout << " lead vehicle length = " << leadVehLength << endl;
	gout << " bumper to bumper dist = " << bumperDist << endl;
#endif


	// Compute follow time and ttc.
	double followTime, ttc;

	const CDynObj* pOwnVehObj = BindObjIdToClass( 0 );
	if( !pOwnVehObj->IsValid() )	return false;

	double ownVehVel = pOwnVehObj->GetVelImm();	// in meters
	if ( ownVehVel > cNEAR_ZERO )
	{

		followTime = followDist *cFEET_TO_METER / ownVehVel;
		ttc = bumperDist *cFEET_TO_METER / ownVehVel;

	}
	else
	{
		followTime = 1000.0;  // a large value
		ttc = 1000.0;
	}

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " own vehicle vel = " << ownVehVel << " ms" << endl;
	gout << " follow time = " << followTime << " seconds " <<endl;
	gout << " ttc = " << ttc << " seconds " << endl;
#endif


	// Compute lead obj info.
	CPoint3D leadVehPos = GetObjPos( leadVehId );
	CRoadPos leadVehRoadPos ( *this, leadVehPos );
	const CDynObj* pLeadObj = BindObjIdToClass( leadVehId );
	if( !pLeadObj->IsValid() )	return false;
	double leadVehVel = pLeadObj->GetVelImm();	// in ms
	eCVTrafficLightState leadVehLightState = GetTrafficLightState ( leadVehId );

#ifdef	DEBUG_GET_OWN_VEHICLE_INFO
	gout << " lead vehicle id = " << leadVehId << endl;
	gout << " lead vehicle pos = " << leadVehPos << endl;
	gout << " lead vehicle roadpos = " << leadVehRoadPos << endl;
	gout << " lead vehicle vel = " << leadVehVel << " ms " << endl;
	gout << " lead vehicle light state = " << leadVehLightState << endl;
#endif


	// Output the rest of the driver info to another structure.
	TLeadObjInfo node;
	node.vehicleId = leadVehId;
	node.XYZpoint = leadVehPos;
	node.roadpos = leadVehRoadPos;
	node.velocity = leadVehVel;
	node.lightState = leadVehLightState;

	followInfo.followDist = followDist;
	followInfo.bumperToBumperDist = bumperDist;
	followInfo.followTime = followTime;
	followInfo.ttc = ttc;
	followInfo.leadObjInfo = node;

	return true;

}	// end of GetOwnVehicleInfo




//////////////////////////////////////////////////////////////////////////////////
///\brief
///	 This function gets the lead obj of an obj and the follow distance.
///\remark
///  Remarks: If the input obj is on road, this function checks if it has lead obj on
///  the same road; if not, it will check the next intersection depending on whether
///  the obj dist to road end is greater than 2500ft. If no, it will check related
///  corridors on the next intersection; if yes, it returns false without checking
///  the next intersection. If the input obj is on intersection, this function checks
///  if it has lead obj on the same corridor. If yes, it gets the lead obj; if no, it
///  will check the destination lane of the corridor for a possible lead obj. This function
///  ignores signs, traffic lights or coordniators as potential lead objects
///  Arguments:
///
///\par 	objId      - the input obj id.
///\par 	leadObjId  - (output) the lead obj id of the input obj.
///\par 	followDist - (output) dist from input obj center to lead obj center
///\par 	path       - Currently not used, inteded to be used to specify which
///						 coordidor the object will favor at intersections
///
///\return true if the lead obj id and follow dist are obtained; false otherwise.
////////////////////////////////////////////////////////////////////////////////
bool
CCved::GetLeadObj(int objId, int& leadObjId, double& followDist, const CPath* path, bool withRoadDir, int prefCrdr) const
{

#ifdef	DEBUG_GET_LEAD_OBJ
	gout << " =========== CCved::GetLeadObj ========== " << endl;
#endif
	bool onPath = false;
	const float overLapPad = 12.0f ;

	CObjTypeMask objMask;
	objMask.SetAll();
	objMask.Clear( eCV_TRAFFIC_SIGN );
	objMask.Clear( eCV_TRAFFIC_LIGHT );
	objMask.Clear( eCV_COORDINATOR );
	objMask.Clear( eCV_COMPOSITE_SIGN);
	// Get the road position of the input obj
	CPoint3D objPos =  GetObjPos( objId );
	CRoadPos roadPos(*this, objPos );

	//see if we are on the path
	if (path  && roadPos.IsValid() && path->IsValid() && path->Contains(roadPos)){
		onPath = true;
	}

	if( !roadPos.IsValid() )	return false;

	// When the input obj is on road.
	if( roadPos.IsRoad() )
	{

#ifdef	DEBUG_GET_LEAD_OBJ
		gout << " input obj " << objId << " on road " << endl;
#endif

		CLane lane = roadPos.GetLane();

		if( !lane.IsValid() )	return false;

		// Get all objs on the input obj lane.
		vector<TObjWithDist> objsOnLane;
		GetAllDynObjsOnLane( lane, objsOnLane,objMask );

		// Find the input obj. If not found, return false. If found, get
		// the obj immediately in front. If there is no obj in front, and
		// if the input obj distance to the end of road is less than or
		// equal to the max distance specified, check the next intersection.
		// If it is over that distance, then return false.

#ifdef	DEBUG_GET_LEAD_OBJ
		gout << " number of objs on input obj lane = " << objsOnLane.size();
		gout << endl;
#endif


		bool foundInputObj = false;
		double objDist = roadPos.GetDistance();

		for( unsigned int i = 0; i < objsOnLane.size(); i++ )
		{
			if ( !(IsObjValid(objsOnLane[i].objId) && IsDynObj(objsOnLane[i].objId )))
				continue;

			TObj* obj = BindObj(objsOnLane[i].objId);

			if (!obj) //we do not have a object
				continue;

			//if the object is not Active, don't get it
			if (!(obj->phase == eALIVE || obj->phase == eDYING)){
				continue;
			}


			if( objsOnLane[i].objId == objId )
			{
				foundInputObj = true;

				if( i < objsOnLane.size() - 1 && ((lane.GetDirection() == ePOS  && withRoadDir) ||
                                                  (lane.GetDirection() == eNEG  && !withRoadDir) ) )
				{


					leadObjId = objsOnLane[i+1].objId;
					followDist = fabs( objsOnLane[i+1].dist - objDist );

#ifdef	DEBUG_GET_LEAD_OBJ
					gout << " input obj = " << objId;
					gout << " lead obj = " << leadObjId;
					gout << " follow dist = " << followDist;
					gout << endl << endl;
					gout << " followDist = fabs( objsOnLane[i+1].dist - objDist ) " << endl;
					gout << " objsOnLane[i+1].dist = " << objsOnLane[i+1].dist;
					gout << " objDist = " << objDist << endl << endl << endl;;
#endif
					return true;

				}
				if( i > 0 && ( (lane.GetDirection() == eNEG && withRoadDir) ||
                               (lane.GetDirection() == ePOS && !withRoadDir) ))
				{
					leadObjId = objsOnLane[i-1].objId;
					followDist = fabs( objsOnLane[i-1].dist - objDist );

#ifdef	DEBUG_GET_LEAD_OBJ
					gout << " input obj = " << objId;
					gout << " lead obj = " << leadObjId;
					gout << " follow dist = " << followDist;
					gout << endl << endl;
					gout << " followDist = fabs( objsOnLane[i-1].dist - objDist ) " << endl;
					gout << " objsOnLane[i-1].dist = " << objsOnLane[i-1].dist;
					gout << " objDist = " << objDist << endl << endl << endl;
#endif
					return true;

				}
			}
		}	// for

		if( !foundInputObj ){ //	return false;
			if ((objDist < overLapPad || objDist > lane.GetRoad().GetCubicLength() - overLapPad )&& objsOnLane.size()>0){
				//let just trust we have overlap issues
				if (objDist < overLapPad && lane.GetDirection() == ePOS){
					leadObjId = objsOnLane[0].objId;
					followDist = fabs( objsOnLane[0].dist - objDist );
					return true;
				}else if (objDist + overLapPad > lane.GetRoad().GetCubicLength()){
					leadObjId = objsOnLane[objsOnLane.size() -1].objId;
					followDist = fabs( objsOnLane[objsOnLane.size() -1].dist - objDist );
					return true;
				}
			}
			return false;
			//lets just trust that we have
		}


		// When execution reaches here, it means there is no obj
		// in front of the input obj on same road, need to check
		// the input obj distance to end of road to determine whether
		// it is necessary to check the next intersection. If so,
		// get the next intersection. Get all the corridors that
		// have the input obj lane as its source lane. Check all
		// these corridors. If only one corridor has objs, use this
		// corridor to get the lead obj; if there are more than
		// one corridors having objs, get all the first objs on these
		// corridors and the lead obj is the one closest to the
		// input obj or with the smallest distance.
		double objDistToRoadEnd;
		CRoad road = lane.GetRoad();

		if( !road.IsValid() )	return false;

		if( lane.GetDirection() == ePOS && withRoadDir ||  lane.GetDirection() == eNEG && !withRoadDir)
		{
			objDistToRoadEnd = road.GetLinearLength() - objDist;
		}
		else
		{
			objDistToRoadEnd = objDist;
		}

#ifdef DEBUG_GET_LEAD_OBJ
		gout << " no lead obj on same road, check distance " << endl;
		gout << " obj dist = " << objDist << endl;
		gout << " obj dist to road end = " << objDistToRoadEnd << endl;
#endif

		const double MAX_DIST = 2500;	// ft
		if( objDistToRoadEnd > MAX_DIST )
		{

#ifdef DEBUG_GET_LEAD_OBJ
			gout << " obj dist to road end is greater than dist specified ";
			gout << " .... return false " << endl;
			gout << " input obj = " << objId;
			gout << " lead obj = -1 " << endl;
#endif
			return false;
		}

		CIntrsctn intrsctn;
        if (withRoadDir)
            intrsctn = lane.GetNextIntrsctn();
        else
            intrsctn = lane.GetPrevIntrsctn();

		if( !intrsctn.IsValid() )	return false;
		int intrsctnId = intrsctn.GetId();

#ifdef	DEBUG_GET_LEAD_OBJ
		gout << " next intersection is valid " << endl;
#endif

		vector<CCrdr> allCrdrs;
		vector<CCrdr> desiredCrdrs;

		intrsctn.GetAllCrdrs( allCrdrs );

		vector<CCrdr>::iterator itr;
		//if we have a path, and we are on the path, use crds on the path
		int prefcrdr = -1;
		if (onPath){
            if (withRoadDir){
			    CCrdr crd = path->GetNextCrdr(roadPos,true);
			    if (crd.IsValid()){
				    prefcrdr = crd.GetRelativeId();
				    desiredCrdrs.push_back(crd);
			    }
            }else{
                //no ideo what we should do here right now.............
/*			    CCrdr crd = path->GetNextCrdr(roadPos,true);
			    if (crd.IsValid()){
				    prefcrdr = crd.GetRelativeId();
				    desiredCrdrs.push_back(crd);
			    }  */
            }
		}
		if (prefcrdr < 0){
			for ( itr = allCrdrs.begin(); itr != allCrdrs.end(); itr++ )
			{
				CCrdr aCrdr = *itr;
                if (withRoadDir){
				    if( aCrdr.GetSrcLn() == lane )
				    {
    #ifdef DEBUG_GET_LEAD_OBJ
					    gout << " desired crdr = " << aCrdr.GetRelativeId() << endl;
    #endif
					    desiredCrdrs.push_back( aCrdr );
				    }
                }else{
                    if( aCrdr.GetDstntnLn() == lane )
				    {
    #ifdef DEBUG_GET_LEAD_OBJ
					    gout << " desired crdr = " << aCrdr.GetRelativeId() << endl;
    #endif
					    desiredCrdrs.push_back( aCrdr );
				    }
                }
			}	// for
		}

		// If there is no corridor that has the input obj lane as its source
		// lane( that is, the input obj lane doesn't lead to any corridor and
		// the obj needs to make a lane change before entering the next
		// intersection ), return false; Else iterate through all the corridors
		// that have the input obj lane as their source lane; for each corridor
		// get all the objs on the corridor, sort the distance and get the obj
		// with the smallest dist.
		if( desiredCrdrs.size() == 0 )
		{

#ifdef DEBUG_GET_LEAD_OBJ
			gout << " input obj lane leads to no corridor ... return false ";
			gout << endl;
			gout << " input obj = " << objId;
			gout << " lead obj = -1 " << endl;
#endif
			return false;
		}

		vector<TObjWithDist> firstObjOnCrdrs;

		vector<CCrdr>::iterator crdrItr;

	    for( crdrItr = desiredCrdrs.begin(); crdrItr != desiredCrdrs.end(); crdrItr++ )
		{
			CCrdr crdr = *crdrItr;
			int crdrId = crdr.GetRelativeId();
            float crdrLenght = (float)crdr.GetLength();
			vector<TObjWithDist> objsOnCrdr;

			GetAllDynObjsOnCrdr(intrsctnId, crdrId, objsOnCrdr,objMask );

			if( objsOnCrdr.size() == 0 )
			{

#ifdef DEBUG_GET_LEAD_OBJ
				gout << " no obj on crdr " << crdrId << " ... continue " << endl;
#endif
				continue;
			}
			else
			{

#ifdef DEBUG_GET_LEAD_OBJ
				gout << " number of objs on crdr " << crdrId << " is ";
				gout << objsOnCrdr.size() << endl;
#endif

				// sort the objs.
				TObjWithDist hold;
				unsigned int j, loc;
				for( j = 1; j < objsOnCrdr.size(); j++ )
				{
					hold = objsOnCrdr[j];
					loc = j;
					while( 0 < loc && ( hold.dist < objsOnCrdr[loc-1].dist ) )
					{
						objsOnCrdr[loc] = objsOnCrdr[loc-1];
						loc--;
					}
					objsOnCrdr[loc] = hold;
				}
                if (withRoadDir){
				// When obj is on boundary, it is reported both on road
				// and on intersection. Don't get the input obj as its
				// own lead obj.
				    if( objId == objsOnCrdr[0].objId )
				    {
					    if( objsOnCrdr.size() == 1 )
					    {

    #ifdef DEBUG_GET_LEAD_OBJ
						    gout << " obj reported more than once ... skip this ";
						    gout << " desired corridor ... continue " << endl;
    #endif

						    continue;
					    }
					    else
					    {

						    firstObjOnCrdrs.push_back( objsOnCrdr[1] );
					    }
				    }
				    else
				    {
					    firstObjOnCrdrs.push_back( objsOnCrdr[0] );
				    }
                }else{
				// When obj is on boundary, it is reported both on road
				// and on intersection. Don't get the input obj as its
				// own lead obj.
				    if( objId == objsOnCrdr[objsOnCrdr.size()-1].objId )
				    {
					    if( objsOnCrdr.size() == 1 )
					    {

    #ifdef DEBUG_GET_LEAD_OBJ
						    gout << " obj reported more than once ... skip this ";
						    gout << " desired corridor ... continue " << endl;
    #endif

						    continue;
					    }
					    else
					    {

                            objsOnCrdr[objsOnCrdr.size()-2].dist = crdrLenght -  objsOnCrdr[objsOnCrdr.size()-2].dist;
                            firstObjOnCrdrs.push_back( objsOnCrdr[objsOnCrdr.size()-2]  );
					    }
				    }
				    else
				    {
                        objsOnCrdr[objsOnCrdr.size()-1].dist = crdrLenght -  objsOnCrdr[objsOnCrdr.size()-1].dist;
                        firstObjOnCrdrs.push_back( objsOnCrdr[objsOnCrdr.size()-1]  );
				    }
                }
            } //else
        }// for

		// When no corridors have obj.
		if( firstObjOnCrdrs.size() == 0 )
		{

#ifdef DEBUG_GET_LEAD_OBJ
			gout << " no obj found on any connected corridors ... return false ";
			gout << endl;
			gout << " input obj = " << objId;
			gout << " lead obj = -1 " << endl;
#endif
			if (onPath){
				//if we have path...lets project ahead to the next road
				if (desiredCrdrs.size() > 0){
					const CRoad road = desiredCrdrs[0].GetDstntnRd();
					const CLane lane = desiredCrdrs[0].GetDstntnLn();
					objDistToRoadEnd += desiredCrdrs[0].GetLength();
					if (objDistToRoadEnd > MAX_DIST ){
						//we are beyond our road dist;
						return false;
					}
					if (!road.IsValid() || !lane.IsValid()){
						return false;
					}
					CRoadPos projectedRoadPos(road,lane.GetRelativeId());
					if (lane.GetDirection() == eNEG){
						//we are starting at the negitive end of the road
						projectedRoadPos.SetDistance(road.GetCubicLength() - 0.001);
					}
					vector<TObjWithDist> objsOnLane;
					GetAllDynObjsOnLane( lane, objsOnLane,objMask );

					// Find the input obj. If not found, return false. If found, get
					// the obj immediately in front. If there is no obj in front, and
					// if the input obj distance to the end of road is less than or
					// equal to the max distance specified, check the next intersection.
					// If it is over that distance, then return false.
					if (objsOnLane.size() == 0)
						return false;

					#ifdef	DEBUG_GET_LEAD_OBJ
						gout << " number of objs on input obj lane = " << objsOnLane.size();
						gout << endl;
					#endif


					bool foundInputObj = false;
					double objDist = projectedRoadPos.GetDistance();

					if( lane.GetDirection() == ePOS && withRoadDir  || lane.GetDirection() == eNEG && !withRoadDir)
					{

						leadObjId = objsOnLane[0].objId;
						followDist = fabs( objsOnLane[0].dist ) + objDistToRoadEnd;

						#ifdef	DEBUG_GET_LEAD_OBJ
								gout << " input obj = " << objId;
								gout << " lead obj = " << leadObjId;
								gout << " follow dist = " << followDist;
								gout << endl << endl;
								gout << " followDist = fabs( objsOnLane[i+1].dist - objDist ) " << endl;
								gout << " objsOnLane[i+1].dist = " << objsOnLane[i+1].dist;
								gout << " objDist = " << objDist << endl << endl << endl;;
						#endif
						return true;

					}
					if( lane.GetDirection() == eNEG || lane.GetDirection() == ePOS && !withRoadDir)
					{
						leadObjId = objsOnLane[objsOnLane.size()-1].objId;
						followDist = fabs( objsOnLane[objsOnLane.size()-1].dist - projectedRoadPos.GetDistance() ) + objDistToRoadEnd;

						#ifdef	DEBUG_GET_LEAD_OBJ
							gout << " input obj = " << objId;
							gout << " lead obj = " << leadObjId;
							gout << " follow dist = " << followDist;
							gout << endl << endl;
							gout << " followDist = fabs( objsOnLane[i-1].dist - objDist ) " << endl;
							gout << " objsOnLane[i-1].dist = " << objsOnLane[i-1].dist;
							gout << " objDist = " << objDist << endl << endl << endl;
						#endif
						return true;
					}
					return false;
				}
				return false;
			}
			return false;
		}
		else
		{
			// sort the first objs on all those corridors.
			TObjWithDist hold;
			unsigned int k, loc;
			for( k = 1; k < firstObjOnCrdrs.size(); k++ )
			{
				hold = firstObjOnCrdrs[k];
				loc = k;
				while( 0 < loc && ( hold.dist < firstObjOnCrdrs[loc-1].dist ) )
				{
					firstObjOnCrdrs[loc] = firstObjOnCrdrs[loc-1];
					loc--;
				}
				firstObjOnCrdrs[loc] = hold;
			}

			// Among the first objs on all those desired corridors, get the one
			// closest to the input obj as the lead obj.
			//if (!onPath || prefcrdr == -1){
            if (withRoadDir){
			    leadObjId = firstObjOnCrdrs[0].objId;
			    followDist = objDistToRoadEnd + firstObjOnCrdrs[0].dist;
            }else{
                leadObjId = firstObjOnCrdrs[firstObjOnCrdrs.size()-1].objId;
			    followDist = objDistToRoadEnd + firstObjOnCrdrs[firstObjOnCrdrs.size()-1].dist;
            }


#ifdef	DEBUG_GET_LEAD_OBJ
			gout << " input obj = " << objId;
			gout << " lead obj = " << leadObjId;
			gout << " follow dist = " << followDist;
			gout << endl << endl;
			gout << " followDist = objDistToRoadEnd + firstObjOnCrdrs[0].dist " << endl;
			gout << " objDistToRoadEnd = " << objDistToRoadEnd;
			gout << " firstObjOnCrdrs[0].dist = " << firstObjOnCrdrs[0].dist;
			gout << endl << endl << endl;
#endif
			return true;
		}


	}	// on road
	else{
		CCrdr crdr;
		if (onPath){
			int intrId = roadPos.GetIntrsctn().GetId();
			int crdId;
			if (path->GetCrdrFromIntrscn(intrId,crdId,&roadPos)){
				crdr = roadPos.GetCorridor(crdId);
				if (!crdr.IsValid()){
					//a path can have multiple crdrs, we need to machup our crdr with
					//our path point.
					crdr = roadPos.GetCorridor();
					if (!crdr.IsValid()){
						return false;
					}
					const CLane lane = crdr.GetSrcLn();
					if (lane.IsValid()){
						//find the corridor with the same src id
						if (path->GetCrdrFromIntrscn(intrId,crdId,&roadPos,lane.GetId())){
							crdr = roadPos.GetCorridor(crdId);
							if (!crdr.IsValid()){
								crdr = roadPos.GetCorridor();
							}
						}
					}
				}
			}
		}else{
            if (prefCrdr > 0 && roadPos.HasCorridorWithAbsoluteId(prefCrdr)){
                crdr = roadPos.GetCorridorWithAbsoluteId(prefCrdr);
            }else{
			    crdr = roadPos.GetCorridor();
            }
		}

		if( !crdr.IsValid() )	return false;
		int crdrId = crdr.GetRelativeId();
        roadPos.SetCrdPriority(&crdr);
        double crdrLength = crdr.GetLength();
		double objDist = roadPos.GetDistance();

		CIntrsctn intrsctn = crdr.GetIntrsctn();

		if( !intrsctn.IsValid() )	return false;
		int intrsctnId = intrsctn.GetId();

		vector<TObjWithDist> objsOnCrdr;

		GetAllDynObjsOnCrdr( intrsctnId, crdrId, objsOnCrdr,objMask );

		// sort the objs
		TObjWithDist hold;
		unsigned int i, loc;
		for( i = 1; i < objsOnCrdr.size(); i++ )
		{
			hold = objsOnCrdr[i];
			loc = i;
			while( 0 < loc && ( hold.dist < objsOnCrdr[loc-1].dist ) )
			{
				objsOnCrdr[loc] = objsOnCrdr[loc-1];
				loc--;
			}
			objsOnCrdr[loc] = hold;
		}

		// Find the input obj. If not found, return false. If found, get
		// the obj immediately in front. If there is no obj in front,
		// then check the obj corridor's destination lane.

#ifdef	DEBUG_GET_LEAD_OBJ
		gout << " crdr id from input obj roadpos = " << crdrId << endl;
		gout << " number of objs on input obj corridor = " << objsOnCrdr.size();
		gout << endl;
#endif

		bool foundInputObj = false;

		for( unsigned int j = 0; j < objsOnCrdr.size(); j++ )
		{
			if( objsOnCrdr[j].objId == objId  )
			{
				foundInputObj = true;

				if (withRoadDir){
                    if( j < objsOnCrdr.size() - 1 )
				    {
					    leadObjId = objsOnCrdr[j+1].objId;
					    followDist = objsOnCrdr[j+1].dist - objDist;

    #ifdef	DEBUG_GET_LEAD_OBJ
					    gout << " input obj = " << objId;
					    gout << " lead obj = " << leadObjId;
					    gout << " follow dist = " << followDist;
					    gout << endl << endl;
					    gout << " followDist = objsOnCrdr[j+1].dist - objDist " << endl;
					    gout << " objsOnCrdr[j+1].dist = " << objsOnCrdr[j+1].dist;
					    gout << " objDist = " << objDist << endl << endl << endl;
    #endif
					    return true;

				    }
                }else{
                    if( j > 0 )
				    {
					    leadObjId = objsOnCrdr[j-1].objId;
					    followDist = objDist - objsOnCrdr[j-1].dist;
                        return true;
                    }

                }

			}
		}	// for

		if( !foundInputObj ){
			if (objDist < overLapPad && objsOnCrdr.size()>0){
				//let just trust we have overlap issues
				leadObjId = objsOnCrdr[0].objId;
				followDist = fabs( objsOnCrdr[0].dist - objDist );
				return true;
			}
			return false;
		}

		// When execution reaches here, it means there is no lead obj in
		// front of the input obj on the same corridor, need to check
		// the destination lane. Get all the objs on the destination lane.
		// If no obj found in that lane, return false; otherwise get the
		// first obj on that lane.

#ifdef	DEBUG_GET_LEAD_OBJ
		gout << " no lead obj on same corridor ... check dstntn lane " << endl;
#endif

		CLane dstntnLn = crdr.GetDstntnLn();

		if( !dstntnLn.IsValid() )	return false;

		CRoad dstntnRd = dstntnLn.GetRoad();

		if ( !dstntnRd.IsValid() )	return false;
		double dstntnRdLength = dstntnRd.GetLinearLength();

		vector<TObjWithDist> objsOnLane;

		GetAllDynObjsOnLane( dstntnLn, objsOnLane,objMask );

#ifdef	DEBUG_GET_LEAD_OBJ
		gout << " number of objs on dstntn lane = " << objsOnLane.size();
		gout << endl;
#endif

		if( objsOnLane.size() == 0 || //no objects
			( (objsOnLane.size() == 1)  &&  objsOnLane[0].objId == objId ) ) //we are on the next road, and we are the only obj on said road
		{

#ifdef	DEBUG_GET_LEAD_OBJ
			gout << " no obj on dstntn lane, so no lead obj ... return false ";
			gout << endl;
			gout << " input obj = " << objId;
			gout << " lead obj = -1 " << endl;
#endif
			//ok now lets check out the next crdr, I am lazy, its the day before this needs to be
			//used, its easier to do this if we have a path, so well for now this will only
			//work if we have a path

			double nextRoadDist = dstntnLn.GetRoad().GetLinearLength();
			if (path){
				vector<TObjWithDist> results;
				CVED::CIntrsctn intr = dstntnLn.GetNextIntrsctn();
				if (path->GetCrdrFromIntrscn(intr.GetId(),crdrId,NULL)){
					GetAllDynObjsOnCrdr(intr.GetId(),crdrId,results);
					if (results.size() == 0){
						if (withRoadDir){
							double nextDist = 0;
							CVED::CCrdr nextCrd(intr,crdrId);
							CVED::CLane nextLane = nextCrd.GetDstntnLn();
							GetAllDynObjsOnLane(nextLane,results);
							if (results.size() == 0){
								return false;
							}
							if (nextLane.GetDirection() == ePOS){
								nextDist = results[0].dist;
								leadObjId = results[0].objId;
							}else{
								nextDist = nextLane.GetRoad().GetLinearLength() - results.back().dist;
								leadObjId = results.back().objId;
							}
							followDist = (crdrLength - objDist) + nextRoadDist + nextCrd.GetLength() + nextDist;
							return true;
						}else{//too much work.......
							return false;
						}

						return false;
					}
					if (withRoadDir){
						double distToNextObj = results.begin()->dist;
						followDist = crdrLength - objDist + distToNextObj + nextRoadDist;
						leadObjId  = results.begin()->objId;
						return true;
					}else{
						double distToNextObj = results.back().dist;
						CVED::CCrdr nextCrd(intr,crdrId);
						followDist = objDist + nextRoadDist +( distToNextObj  - nextCrd.GetLength());
						leadObjId  = results.begin()->objId;
						return true;
					}

				}
			}
			return false;
		}
		else
		{
			// When obj is reported both on intersection and on road,
			// don't get the obj itself as its lead obj.
			if( objId == objsOnLane[0].objId )
			{
				if( objsOnLane.size() == 1 )
				{

#ifdef	DEBUG_GET_LEAD_OBJ
					gout << " obj reporting on boundary case: obj is its own lead obj ";
					gout << " ... return false " << endl;
					gout << " input obj = " << objId;
					gout << " lead obj = -1 " << endl;
#endif
					return false;
				}
				else
				{
					leadObjId = objsOnLane[1].objId;

					if( dstntnLn.GetDirection() == ePOS )
					{
						followDist = crdrLength - objDist  + ( objsOnLane[1].dist );

#ifdef	DEBUG_GET_LEAD_OBJ
						gout << " input obj = " << objId;
						gout << " lead obj = " << leadObjId;
						gout << " follow dist = " << followDist;
						gout << endl << endl;
						gout << " followDist = crdrLength - objDist  + ( objsOnLane[1].dist ) ";
						gout << endl;
						gout << " crdrLength = " << crdrLength;
						gout << " objDist = " << objDist;
						gout << " objsOnLane[1].dist = " << objsOnLane[1].dist << endl << endl << endl;
#endif

					}
					else
					{
						followDist = crdrLength - objDist + dstntnRdLength - ( objsOnLane[1].dist );

#ifdef	DEBUG_GET_LEAD_OBJ
						gout << " input obj = " << objId;
						gout << " lead obj = " << leadObjId;
						gout << " follow dist = " << followDist;
						gout << endl << endl;
						gout << " followDist = crdrLength - objDist + dstntnRdLength - ( objsOnLane[1].dist ) ";
						gout << endl;
						gout << " crdrLength = " << crdrLength;
						gout << " objDist = " << objDist;
						gout << " dstntnRdLength = " << dstntnRdLength;
						gout << " objsOnLane[1].dist = " << objsOnLane[1].dist << endl << endl << endl;
#endif

					}

					return true;
				}
			}
			else
			{
				leadObjId = objsOnLane[0].objId;

				if( dstntnLn.GetDirection() == ePOS )
				{
                    //since we can have a small amount of overlap between the road and the
                    //intersection, we may be on both, so make sure we are not the first object on the list;
                    size_t index = 0;
                    bool found = false;
                    while (index < objsOnLane.size()){
                        if (objsOnLane[index].objId != objId){
                            found = true;
                            break;
                        }
                        index++;
                    }

                    if (!found)
                        return false;

                    followDist = crdrLength - objDist  + ( objsOnLane[index].dist );

#ifdef	DEBUG_GET_LEAD_OBJ
					gout << " input obj = " << objId;
					gout << " lead obj = " << leadObjId;
					gout << " follow dist = " << followDist;
					gout << endl;
					gout << " followDist = crdrLength - objDist  + ( objsOnLane[0].dist ) ";
					gout << endl << endl;
					gout << " crdrLength = " << crdrLength;
					gout << " objDist = " << objDist;
					gout << " objsOnLane[0].dist = " << objsOnLane[0].dist << endl << endl << endl;
#endif

				}
				else
				{
                    //since we can have a small amount of overlap between the road and the
                    //intersection, we may be on both, so make sure we are not the last object on the list;
                    int index = (int)objsOnLane.size()-1;
                    bool found = false;
                    while (index >= 0){
                        if (objsOnLane[index].objId != objId){
                            found = true;
                            break;
                        }
                        index--;
                    }
                    if (!found)
                        return false;
                    followDist = crdrLength - objDist + dstntnRdLength - ( objsOnLane[index].dist );

#ifdef	DEBUG_GET_LEAD_OBJ
					gout << " input obj = " << objId;
					gout << " lead obj = " << leadObjId;
					gout << " follow dist = " << followDist;
					gout << endl << endl;
					gout << " followDist = crdrLength - objDist + dstntnRdLength - ( objsOnLane[0].dist ) ";
					gout << endl;
					gout << " crdrLength = " << crdrLength;
					gout << " objDist = " << objDist;
					gout << " dstntnRdLength = " << dstntnRdLength;
					gout << " objsOnLane[0].dist = " << objsOnLane[index].dist << endl << endl << endl;
#endif

				}

				return true;
			}
		}


	}	// on intersection

}	// end of GetLeadObj

//////////////////////////////////////////////////////////////////////////////
//
// Description:
//
// Remarks:
//
// Arguments:
//	type -
//	*pOdeObj -
//	initPosRot -
//	initVel -
//	mass -
//	cgoffset -
//	dimension -
//	visOrigOffset -
//	bodyProps -
//
// Returns:
//
////////////////////////////////////////////////////////////////////////////////////////
void CCved::CreateODEObject( odePublic::EODEObjType type,
						CODEObject* pOdeObj,
						const double initPosRot[6],
						const double initVel[6],
						double mass,
						double cgoffset[3],
						double dimension[3],
						double visOrigOffset[3],
						odePublic::SBodyProps bodyProps){
	//m_pOde = std::unique_ptr<CODE>(new CODE(this));
	m_pOde->CreateODEObj(type, *pOdeObj, initPosRot, initVel, mass, cgoffset, dimension, visOrigOffset, bodyProps);
}
void
CCved::DeleteODEObj(CODEObject* obj){
	delete obj;
}

void __cdecl CCved::Logoutf(const TCHAR* format, ...)
{
#ifdef _DEBUG
	va_list arglist;
	va_start(arglist, format);
	int cap = 256;
	TCHAR* buffer = (TCHAR *)malloc(sizeof(TCHAR) * cap);
	bool gen = false;
	do
	{
		int l = _vsntprintf(buffer, cap, format, arglist);
		gen = (l > 0 && l < cap);
		if (!gen)
		{
			cap += cap;
			buffer = (TCHAR *)realloc(buffer, sizeof(TCHAR) * cap);
		}
	} while(!gen);

	OutputDebugString(buffer);
	free(buffer);
#endif
}

} // namespace CVED




















