#include "CvedDistri.h"
#include "ExternalControlInterface.h"
namespace CVED {

CCvedDistri::CCvedDistri(IExternalObjectControl* pCtrl):m_pCtrl(pCtrl)
{
}


CCvedDistri::~CCvedDistri(void)
{
}

CVED::CDynObj* CCvedDistri::LocalCreatePeerDriver(CHeaderDistriParseBlock& blk, cvEObjType type)
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
	CVED::CRoadPos roadPos(*this, cartPos);
	CVED::CDynObj* pObj = NULL;
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
		pObj = this->CreateDynObj(
							blk.GetSimName()
							, type
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
        CVED::CRoadPos roadPosN = roadPos;
        if (initVel > 0 ){
            float distLookAhead = initVel * cMETER_TO_FEET;
            if (roadPosN.Travel(distLookAhead) == CVED::CRoadPos::eERROR){
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
	        CVED::CRoadPos tempRoadPos(roadPos);
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
		pObj = this->CreateDynObj(
							blk.GetSimName()
							, type
							, attr
							, &cartPos
							, &tan
							, &lat
			);
	}  // end else vehicle on the road

	//
	// Get a pointer to the vehicle object.
	//
	CVED::CVehicleObj* pVehicleObj = dynamic_cast<CVED::CVehicleObj *>( pObj );

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

void CCvedDistri::LocalDeleteDynObj( CDynObj* dynObj )
{
	CCved::DeleteDynObj(dynObj);
}

CDynObj* CCvedDistri::LocalCreateDynObj(
					const string&		cName,
					cvEObjType			type,
					const cvTObjAttr&	cAttr,
					const CPoint3D*		cpInitPos,
					const CVector3D*	cpInitTan,
					const CVector3D*	cpInitLat)
{
	return CCved::CreateDynObj(cName, type, cAttr, cpInitPos, cpInitTan, cpInitLat);
}

};