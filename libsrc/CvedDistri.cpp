#include "CvedDistri.h"
#include "ExternalControlInterface.h"
#include "cvedstrc.h"
namespace CVED {

CCvedDistri::CCvedDistri(IExternalObjectControl* pCtrl) : m_pCtrl(pCtrl)
{
}


CCvedDistri::~CCvedDistri(void)
{
}

//it creates a mock edo
CDynObj* CCvedDistri::LocalCreateEDO(CHeaderDistriParseBlock& blk)
{
	const double cMETER_TO_FEET = 3.2808; // feet
	//
	// Get the SOL object.
	//
	const string cSolName = blk.GetSolName();

	const CSolObj* cpSolObj = this->GetSol().GetObj( cSolName );
	if( !cpSolObj )
	{
		assert(0);
		return NULL;
	}

	cvEObjType type = eCV_VEHICLE;
	//
	// Initialize the attributes.
	//
	cvTObjAttr attr = { 0 };
	attr.solId = cpSolObj->GetId();
	attr.xSize = cpSolObj->GetLength();
	attr.ySize = cpSolObj->GetWidth();
	attr.zSize = cpSolObj->GetHeight();
	attr.colorIndex = blk.GetColorIndex();
	attr.hcsmId = 0;



	double initVel = 0;
	this->GetOwnVehicleVel(initVel);
	//
	// Get the starting location and add the vehicle's CG in the z-axis.
	//
	CPoint3D cartPos = blk.GetOwnVehPos();
	CRoadPos roadPos(*this, cartPos);
	CDynObj* pObj = NULL;
	if( !roadPos.IsValid() )
	{
		//
		// The vehicle is currently off-road...use the offroad point.
		//

		CPoint3D targPos = roadPos.GetXYZ();
		CVector3D tan = targPos - cartPos;
		CVector3D lat( 0.0, 0.0, 0.0 );

		//
		// Create the CVED object.
		//
		pObj = LocalCreateEDO(
							blk.GetSimName()
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end if vehicle offroad
	else
	{
		//
		// The vehicle is on the road.
		//
		cartPos = roadPos.GetXYZ();
        CRoadPos roadPosN = roadPos;
        if (initVel > 0 ){
            float distLookAhead = initVel * cMETER_TO_FEET;
            if (roadPosN.Travel(distLookAhead) == CRoadPos::eERROR){
                roadPosN = roadPos;
            }
        }
        CVector3D rotVec;
        if (initVel > 0){
            CVector3D vec1 = roadPos.GetTangentInterpolated();
            CVector3D vec2 = roadPosN.GetTangentInterpolated();
            CVector3D vec3 = vec1-vec2;

            auto targPosRp = roadPosN.GetBestXYZ();
            auto currPosRp = roadPos.GetBestXYZ();
            CVector3D targPos(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            CVector3D currPos(currPosRp.m_x,currPosRp.m_y,currPosRp.m_z);
	        CVector3D targetSeg(targPos - currPos);
            float laneWidth = 0;

            float currentMaxOffset = 6.0f;

	        //Scaling this by 1/3 the segment length
	        //was found through experimentations
	        //to work well, this should be parmaiterized
	        vec3.Scale(targetSeg.Length()/3);


            if (vec3.Length() > currentMaxOffset){
                vec3.Scale(currentMaxOffset/vec3.Length());
            }
            CPoint3D projectedXYZ  = roadPos.GetBestXYZ();
	        CRoadPos tempRoadPos(roadPos);
	        tempRoadPos.SetXYZ(projectedXYZ + vec3);
            targPosRp = tempRoadPos.GetBestXYZ();
            CVector3D vec4(targPosRp.m_x,targPosRp.m_y,targPosRp.m_z);
            rotVec = vec4 - currPos;
            rotVec.Normalize();
        }
		//
		// Get the staring tangent and lateral vectors.
		//
		CVector3D tan = roadPos.GetTangent();
		CVector3D lat = roadPos.GetRightVec();
        if (initVel > 0){
            double yaw = acos(rotVec.DotP(tan) );
            if (/*diff.Length()*/ yaw > 0.01){
                auto axis = tan.CrossP(rotVec);
                if (axis.m_k<0) yaw*=-1;
                tan.RotZ(yaw);
                lat.RotZ(yaw);
            }
        }
		//
		// Create the CVED object.
		//
		pObj = LocalCreateEDO(
							blk.GetSimName()
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end else vehicle on the road

	//
	// Get a pointer to the vehicle object.
	//
	CVehicleObj* pVehicleObj = dynamic_cast<CVehicleObj *>( pObj );

	//
	// Set the initial velocity.
	//


	bool haveInitVel = initVel >= 0.0;
	if( haveInitVel )
	{
		// set velocity to both buffers
		pObj->SetVel( initVel, false );
	}

	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	const CSolObjVehicle* cpSolVeh =
				dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
	if( !cpSolVeh )
	{
		assert(0);
		this->DeleteDynObj(pObj);
		return NULL;
	}

	const CDynaParams& dynaParams = cpSolVeh->GetDynaParams();

	double suspStif =
			( dynaParams.m_SuspStifMin + dynaParams.m_SuspStifMax ) / 2;
	pVehicleObj->SetSuspStif( suspStif );
	pVehicleObj->SetSuspStifImm( suspStif );
	double suspDamp =
			( dynaParams.m_SuspDampMin + dynaParams.m_SuspDampMax ) / 2;
	pVehicleObj->SetSuspDamp( suspDamp );
	pVehicleObj->SetSuspDampImm( suspDamp );

	double tireStif =
			( dynaParams.m_TireStifMin + dynaParams.m_TireStifMax ) / 2;
	pVehicleObj->SetTireStif( tireStif );
	pVehicleObj->SetTireStifImm( tireStif );
	double tireDamp =
			( dynaParams.m_TireDampMin + dynaParams.m_TireDampMax ) / 2;
	pVehicleObj->SetTireDamp( tireDamp );
	pVehicleObj->SetTireDampImm( tireDamp );
	//pVehicleObj->SetDynaFidelity( m_pI->m_dynModel );
	//pVehicleObj->SetDynaFidelityImm( m_pI->m_dynModel );
	pVehicleObj->SetQryTerrainErrCount( 0 );
	pVehicleObj->SetQryTerrainErrCountImm( 0 );
	pVehicleObj->SetDynaInitComplete( 0 );
	pVehicleObj->SetDynaInitCompleteImm( 0 );

	//
	// Set the initial audio and visual state.
	//
	pVehicleObj->SetAudioState( blk.GetAudioState() );
	pVehicleObj->SetVisualState( blk.GetVisualState() );

	// //
	// // If the audio or visual has been set then assign these values
	// // to the dials.  This way the ADO will play those sounds and
	// // display those lights until they are explicity turned off.
	// //
	// if( blk.GetVisualState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetVisualState() );
	// 	string str( buf );
	// 	m_dialVisualState.SetValue( str );
	// }

	// if( blk.GetAudioState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetAudioState() );
	// 	string str( buf );
	// 	m_dialAudioState.SetValue( str );
	// }

    //init control
    pVehicleObj->SetTargPos(cartPos);
    pVehicleObj->StoreOldTargPos();

	return pObj;
}

CDynObj* CCvedDistri::LocalCreatePDO(CHeaderDistriParseBlock& blk, bool own)
{
	const double cMETER_TO_FEET = 3.2808; // feet
	//
	// Get the SOL object.
	//
	const string cSolName = blk.GetSolName();

	const CSolObj* cpSolObj = this->GetSol().GetObj( cSolName );
	if( !cpSolObj )
	{
		assert(0);
		return NULL;
	}

	//
	// Get the CVED object type and check to make sure it's valid.
	//

	//
	// Initialize the attributes.
	//
	cvTObjAttr attr = { 0 };
	attr.solId = cpSolObj->GetId();
	attr.xSize = cpSolObj->GetLength();
	attr.ySize = cpSolObj->GetWidth();
	attr.zSize = cpSolObj->GetHeight();
	attr.colorIndex = blk.GetColorIndex();
	attr.hcsmId = 0;



	double initVel = 0;
	this->GetOwnVehicleVel(initVel);
	//
	// Get the starting location and add the vehicle's CG in the z-axis.
	//
	CPoint3D cartPos = blk.GetOwnVehPos();
	CRoadPos roadPos(*this, cartPos);
	CDynObj* pObj = NULL;

	//
	// The vehicle is currently off-road...use the offroad point.
	//

	CVector3D tan = blk.GetOwnVehOri();
	const CVector3D up(0, 0, 1);
	CVector3D lat = tan.CrossP(up);

	//
	// Create the CVED object.
	//
	pObj = LocalCreatePDO(
						own
						, blk.GetSimName()
						, attr
						, &cartPos
						, &tan
						, &lat
		);



	//
	// Get a pointer to the vehicle object.
	//
	CVehicleObj* pVehicleObj = dynamic_cast<CVehicleObj *>( pObj );

	//
	// Set the initial velocity.
	//


	bool haveInitVel = initVel >= 0.0;
	if( haveInitVel )
	{
		// set velocity to both buffers
		pObj->SetVel( initVel, false );
	}

	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	const CSolObjVehicle* cpSolVeh =
				dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
	if( !cpSolVeh )
	{
		assert(0);
		this->DeleteDynObj(pObj);
		return NULL;
	}

	const CDynaParams& dynaParams = cpSolVeh->GetDynaParams();

	double suspStif =
			( dynaParams.m_SuspStifMin + dynaParams.m_SuspStifMax ) / 2;
	pVehicleObj->SetSuspStif( suspStif );
	pVehicleObj->SetSuspStifImm( suspStif );
	double suspDamp =
			( dynaParams.m_SuspDampMin + dynaParams.m_SuspDampMax ) / 2;
	pVehicleObj->SetSuspDamp( suspDamp );
	pVehicleObj->SetSuspDampImm( suspDamp );

	double tireStif =
			( dynaParams.m_TireStifMin + dynaParams.m_TireStifMax ) / 2;
	pVehicleObj->SetTireStif( tireStif );
	pVehicleObj->SetTireStifImm( tireStif );
	double tireDamp =
			( dynaParams.m_TireDampMin + dynaParams.m_TireDampMax ) / 2;
	pVehicleObj->SetTireDamp( tireDamp );
	pVehicleObj->SetTireDampImm( tireDamp );
	//pVehicleObj->SetDynaFidelity( m_pI->m_dynModel );
	//pVehicleObj->SetDynaFidelityImm( m_pI->m_dynModel );
	pVehicleObj->SetQryTerrainErrCount( 0 );
	pVehicleObj->SetQryTerrainErrCountImm( 0 );
	pVehicleObj->SetDynaInitComplete( 0 );
	pVehicleObj->SetDynaInitCompleteImm( 0 );

	//
	// Set the initial audio and visual state.
	//
	pVehicleObj->SetAudioState( blk.GetAudioState() );
	pVehicleObj->SetVisualState( blk.GetVisualState() );

	// //
	// // If the audio or visual has been set then assign these values
	// // to the dials.  This way the ADO will play those sounds and
	// // display those lights until they are explicity turned off.
	// //
	// if( blk.GetVisualState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetVisualState() );
	// 	string str( buf );
	// 	m_dialVisualState.SetValue( str );
	// }

	// if( blk.GetAudioState() > 0 )
	// {
	// 	char buf[128];
	// 	sprintf( buf, "%d", blk.GetAudioState() );
	// 	string str( buf );
	// 	m_dialAudioState.SetValue( str );
	// }

    //init control
    pVehicleObj->SetTargPos(cartPos);
    pVehicleObj->StoreOldTargPos();

	return pObj;
}
//fixme: this function will be removed to confine external(vechicle|avatar) convension
CDynObj *
CCvedDistri::CreateDynObj(
			bool own,
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
	if ( type == eCV_EXTERNAL_DRIVER
		|| (type == eCV_EXTERNAL_AVATAR && own) ) {
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

void CCvedDistri::LocalDeletePDO( CDynObj* dynObj )
{
	CCved::DeleteDynObj(dynObj);
}

void CCvedDistri::LocalDeleteDynObj( CDynObj* dynObj )
{
	CCved::DeleteDynObj(dynObj);
}

CDynObj* CCvedDistri::LocalCreateADO(
					const string&		cName,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos,
					const CVector3D*	cpInitTan,
					const CVector3D*	cpInitLat)
{
	return CCved::CreateDynObj(cName, eCV_VEHICLE, cAttr, cpInitPos, cpInitTan, cpInitLat);
}


};