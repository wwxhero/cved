/*****************************************************************************
 *
 * (C) Copyright 2005 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:
 *
 * Author: Yefei He, Joshua Svenson
 *
 * Date: March 2005
 *
 * Description: CODEObject and CODE classes that run dynamics for
 *              free motion objects using the ODE library
 *
 *
 ****************************************************************************/
#include "cvedpub.h"
#include "cvedstrc.h"

#include "odeDynamics.h"

#define CUBE_CONTACTS 4
#define MAX_CONTACTS 64
const double FEET_TO_METERS = 0.3048;
const double METERS_TO_FEET = 3.28083;

#define _R(i,j) R[(i)*4+(j)]   // taken from rotation.cpp in ode

static void ODE_CollisionCallback(void* data, dGeomID o1, dGeomID o2) 
{ 
	int i;	

	CODE* pOde = (CODE*)data;

	if(dGeomGetClass( o1 ) == dTriMeshClass && dGeomGetClass( o2 ) == dTriMeshClass){
		return;
	}

	// exit without doing anything if the two bodies are connected by a joint
	dBodyID b1 = dGeomGetBody(o1);
	dBodyID b2 = dGeomGetBody(o2);	
	if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;

	dReal frictionCoeff = 8, // hack for friction coeff to let object slide less
		 bounceEnergyLoss = 0, contactSoftCFM = 0.1; 
	odePublic::SBodyProps *pBodyProps1, *pBodyProps2;

	if ( b1 )
	{
		pBodyProps1 = (odePublic::SBodyProps*)(dBodyGetData(b1));
		if ( pBodyProps1 )
		{
			frictionCoeff *= pBodyProps1->frictionCoeff;
			bounceEnergyLoss += pBodyProps1->bounceEnergyLoss;
			if ( pBodyProps1->contactSoftCFM < contactSoftCFM )
				contactSoftCFM = pBodyProps1->contactSoftCFM;
//			printf("body 1 friction %.4f bounce %.4f\n", pBodyProps1->frictionCoeff, pBodyProps1->bounceEnergyLoss);
		}
	}

	if ( b2 )
	{
		pBodyProps2 = (odePublic::SBodyProps*)(dBodyGetData(b2));
		if ( pBodyProps2 )
		{
			frictionCoeff *= pBodyProps2->frictionCoeff;
			bounceEnergyLoss += pBodyProps2->bounceEnergyLoss;
			if ( pBodyProps2->contactSoftCFM < contactSoftCFM )
				contactSoftCFM = pBodyProps2->contactSoftCFM;
//			printf("body 2 friction %.4f bounce %.4f\n", pBodyProps2->frictionCoeff, pBodyProps2->bounceEnergyLoss);
		}
	}


	if ( bounceEnergyLoss > 0.98 )
		bounceEnergyLoss = 0.98;
//	if ( b1 && b2 )
//		printf("b1 #%d and b2 #%d collide\n", pBodyProps1->cvedId, pBodyProps2->cvedId);

	dContact contact[MAX_CONTACTS];   // up to MAX_CONTACTS contacts per box-box
	for (i=0; i<MAX_CONTACTS; i++) 
	{
		contact[i].surface.mode = dContactBounce; // | dContactSoftCFM;
		contact[i].surface.mu = frictionCoeff;  // 0.70 rubber on dry concrete  // 50
		contact[i].surface.mu2 = 0;
		contact[i].surface.bounce = bounceEnergyLoss;
		contact[i].surface.bounce_vel = 0.1;
		contact[i].surface.soft_cfm = 0.01;
		
		if ( (dGeomGetClass( o1 ) != dPlaneClass) && (dGeomGetClass( o1 ) != dTriMeshClass) && 
			(dGeomGetClass( o2 ) != dPlaneClass) && (dGeomGetClass( o2 ) != dTriMeshClass)) 
		// neither body is not a ground surface, make it either harder or softer
		{
			contact[i].surface.mode |= dContactSoftCFM;
			contact[i].surface.soft_cfm = contactSoftCFM;
		}
		
	}
	
	//////////////////////////////
	/*if(dGeomGetClass( o1 ) != dPlaneClass && dGeomGetClass( o2 ) != dPlaneClass ){
		if( dGeomGetClass( o1 ) == dTriMeshClass ){
			const dReal* o2_Pos = dGeomGetPosition(o2);
			CVector3D o2_normal;

			double vertexHeight1, vertexHeight2, vertexHeight3, vertexHeight4 = 0.0;
			double objHeight = 0.0;
			(*pOde->m_pCved).QryTerrain(o2_Pos[0]* METERS_TO_FEET, o2_Pos[1]* METERS_TO_FEET, o2_Pos[2]* METERS_TO_FEET, objHeight, o2_normal, NULL);
			objHeight = objHeight*FEET_TO_METERS;
			double xQuery = 0.0;
			double yQuery = 0.0;
			double zQuery = 0.0;

			//Get the size of the object to size the triMesh
			pOde->m_vertices[0][0] = -2;
			pOde->m_vertices[0][1] = 2;
			xQuery = (o2_Pos[0] + pOde->m_vertices[0][0])* METERS_TO_FEET;
			yQuery = (o2_Pos[1] + pOde->m_vertices[0][1])* METERS_TO_FEET;
			zQuery = o2_Pos[2]* METERS_TO_FEET;
			(*pOde->m_pCved).QryTerrain(xQuery, yQuery, zQuery, vertexHeight1, o2_normal, NULL);
			vertexHeight1 = vertexHeight1*FEET_TO_METERS;
			pOde->m_vertices[0][2] = vertexHeight1 - objHeight;

			pOde->m_vertices[1][0] = -2;
			pOde->m_vertices[1][1] = -2;
			xQuery = (o2_Pos[0] + pOde->m_vertices[1][0])* METERS_TO_FEET;
			yQuery = (o2_Pos[1] + pOde->m_vertices[1][1])* METERS_TO_FEET;
			zQuery = o2_Pos[2]* METERS_TO_FEET;
			(*pOde->m_pCved).QryTerrain(xQuery, yQuery, zQuery, vertexHeight2, o2_normal, NULL);
			vertexHeight2 = vertexHeight2*FEET_TO_METERS;
			pOde->m_vertices[1][2] = vertexHeight2 - objHeight;

			pOde->m_vertices[2][0] = 2;
			pOde->m_vertices[2][1] = -2;
			xQuery = (o2_Pos[0] + pOde->m_vertices[2][0])* METERS_TO_FEET;
			yQuery = (o2_Pos[1] + pOde->m_vertices[2][1])* METERS_TO_FEET;
			zQuery = o2_Pos[2]* METERS_TO_FEET;
			(*pOde->m_pCved).QryTerrain(xQuery, yQuery, zQuery, vertexHeight3, o2_normal, NULL);
			vertexHeight3 = vertexHeight3*FEET_TO_METERS;
			pOde->m_vertices[2][2] = vertexHeight3 - objHeight;

			pOde->m_vertices[3][0] = 2;
			pOde->m_vertices[3][1] = 2;
			xQuery = (o2_Pos[0] + pOde->m_vertices[3][0])* METERS_TO_FEET;
			yQuery = (o2_Pos[1] + pOde->m_vertices[3][1])* METERS_TO_FEET;
			zQuery = o2_Pos[2]* METERS_TO_FEET;
			(*pOde->m_pCved).QryTerrain(xQuery, yQuery, zQuery, vertexHeight4, o2_normal, NULL);
			vertexHeight4 = vertexHeight4*FEET_TO_METERS;
			pOde->m_vertices[3][2] = vertexHeight4 - objHeight;
		}

		for(int i = 0; i < MAX_TERRAIN_TRIMESH; i++){
			if(pOde->m_terrainTriMeshGeom[i] == o1 || pOde->m_terrainTriMeshGeom[i] == o2){
				pOde->m_terrainTriMeshData[i] = dGeomTriMeshDataCreate();
				dGeomTriMeshDataBuildSingle(pOde->m_terrainTriMeshData[i], pOde->m_vertices[0], 3 * sizeof(float), pOde->m_vertexCount, &(pOde->m_indices[0]), pOde->m_indexCount, 3 * sizeof(dTriIndex));
				//pOde->m_terrainTriMeshGeom[i] = dCreateTriMesh(pOde->m_space, pOde->m_terrainTriMeshData[i], 0, 0, 0);
				dGeomTriMeshSetData(pOde->m_terrainTriMeshGeom[i], pOde->m_terrainTriMeshData[i]);

				if(b2){
					const dReal* pos = dBodyGetPosition(b2);

					double terrainHeight;
					CVector3D normal;
					(*pOde->m_pCved).QryTerrain(pos[0]*METERS_TO_FEET, pos[1]*METERS_TO_FEET, pos[2]*METERS_TO_FEET, terrainHeight, normal, NULL);
					terrainHeight = terrainHeight*FEET_TO_METERS;

					pOde->m_triMeshObjPos[i][0] = pos[0];
					pOde->m_triMeshObjPos[i][1] = pos[1];
					pOde->m_triMeshObjPos[i][2] = terrainHeight;
				}

				break;
			}
		}	
	}
	*/
	///////////////////////////////////////

	if((dGeomGetClass( o1 ) == dSphereClass && dGeomGetClass( o2 ) == dBoxClass) || (dGeomGetClass( o2 ) == dSphereClass && dGeomGetClass( o1 ) == dBoxClass)){
		return;
	}

	// As far as I've seen whenever there is a collision with a triMesh and another object, the other object is always o2
	// temporarily checking for box 
	if(b2 && dGeomGetClass( o2 ) == dSphereClass || dGeomGetClass( o2 ) == dBoxClass){

		// Get the position of the object that will collide with the triMesh
		const dReal* pos = dBodyGetPosition(b2);

		// Query the height of the terrain at the object's location
		double terrainHeight;
		CVector3D terrainNormal;
		if((*pOde->m_pCved).QryTerrain(pos[0]*METERS_TO_FEET, pos[1]*METERS_TO_FEET, pos[2]*METERS_TO_FEET, terrainHeight, terrainNormal, NULL) == (*pOde->m_pCved).eCV_OFF_ROAD){
			terrainHeight = pos[2]*METERS_TO_FEET;
			terrainNormal.m_k = 1;
		}

		terrainHeight = terrainHeight*FEET_TO_METERS;

		// To rotate the plane to match the terrain, we need the normal vectors of both the triMesh and the terrain.
		// We have the normal to the terrain, below we get the unit vector normal to the triMesh
		CVector3D meshNormal;

		double a1 = pOde->m_vertices[0][0]; 
		double a2 = pOde->m_vertices[0][1]; 
		double a3 = pOde->m_vertices[0][2]; 

		double b1 = pOde->m_vertices[1][0]; 
		double b2 = pOde->m_vertices[1][1]; 
		double b3 = pOde->m_vertices[1][2]; 

		double c1 = pOde->m_vertices[2][0]; 
		double c2 = pOde->m_vertices[2][1]; 
		double c3 = pOde->m_vertices[2][2]; 

		meshNormal.m_i = (a2*b3) - (a3*b2);
		meshNormal.m_j = -((a1*b3)-(a3*b1));
		meshNormal.m_k = ((a1*b2)-(a2*b1));
		meshNormal.Normalize();
		
		// Compute the rotation matrix
		double costheta = meshNormal.DotP(terrainNormal)/(meshNormal.Length()*terrainNormal.Length());
		CVector3D axis = terrainNormal.CrossP(meshNormal);
		axis.Normalize();
		double c = costheta;
		double s = sqrt(1-c*c);
		double C = 1-c;
		double x = axis.m_i;
		double y = axis.m_j;
		double z = axis.m_k;

		//The rotation matrix
		dMatrix3 rmat = {
			x*x*C+c,	x*y*C-z*s,	x*z*C+y*s,
			y*x*C+z*s,	y*y*C+c,	y*z*C-x*s,
			z*x*C-y*s,	z*y*C+x*s,	z*z*C+c
		};
			
		// Perform matriix multiplication between the old points on the triMesh and the rotation matrix
		pOde->m_vertices[0][0] = float((pOde->m_vertices[0][0]*rmat[0]) + (pOde->m_vertices[0][1]*rmat[3]) + (pOde->m_vertices[0][2]*rmat[6]));
		pOde->m_vertices[0][1] = float((pOde->m_vertices[0][0]*rmat[1]) + (pOde->m_vertices[0][1]*rmat[4]) + (pOde->m_vertices[0][2]*rmat[7]));
		pOde->m_vertices[0][2] = float((pOde->m_vertices[0][0]*rmat[2]) + (pOde->m_vertices[0][1]*rmat[5]) + (pOde->m_vertices[0][2]*rmat[8]));

		pOde->m_vertices[1][0] = float((pOde->m_vertices[1][0]*rmat[0]) + (pOde->m_vertices[1][1]*rmat[3]) + (pOde->m_vertices[1][2]*rmat[6]));
		pOde->m_vertices[1][1] = float((pOde->m_vertices[1][0]*rmat[1]) + (pOde->m_vertices[1][1]*rmat[4]) + (pOde->m_vertices[1][2]*rmat[7]));
		pOde->m_vertices[1][2] = float((pOde->m_vertices[1][0]*rmat[2]) + (pOde->m_vertices[1][1]*rmat[5]) + (pOde->m_vertices[1][2]*rmat[8]));

		pOde->m_vertices[2][0] = float((pOde->m_vertices[2][0]*rmat[0]) + (pOde->m_vertices[2][1]*rmat[3]) + (pOde->m_vertices[2][2]*rmat[6]));
		pOde->m_vertices[2][1] = float((pOde->m_vertices[2][0]*rmat[1]) + (pOde->m_vertices[2][1]*rmat[4]) + (pOde->m_vertices[2][2]*rmat[7]));
		pOde->m_vertices[2][2] = float((pOde->m_vertices[2][0]*rmat[2]) + (pOde->m_vertices[2][1]*rmat[5]) + (pOde->m_vertices[2][2]*rmat[8]));

		pOde->m_vertices[3][0] = float((pOde->m_vertices[3][0]*rmat[0]) + (pOde->m_vertices[3][1]*rmat[3]) + (pOde->m_vertices[3][2]*rmat[6]));
		pOde->m_vertices[3][1] = float((pOde->m_vertices[3][0]*rmat[1]) + (pOde->m_vertices[3][1]*rmat[4]) + (pOde->m_vertices[3][2]*rmat[7]));
		pOde->m_vertices[3][2] = float((pOde->m_vertices[3][0]*rmat[2]) + (pOde->m_vertices[3][1]*rmat[5]) + (pOde->m_vertices[3][2]*rmat[8]));
		
		// Loop through the triMesh array to find the current object and update the triMesh with the new points
		for(int i = 0; i < MAX_TERRAIN_TRIMESH; i++){
			if(pOde->m_terrainTriMeshGeom[i] == o1 || pOde->m_terrainTriMeshGeom[i] == o2){
				pOde->m_terrainTriMeshData[i] = dGeomTriMeshDataCreate();
				dGeomTriMeshDataBuildSingle(pOde->m_terrainTriMeshData[i], pOde->m_vertices[0], 3 * sizeof(float), pOde->m_vertexCount, &(pOde->m_indices[0]), pOde->m_indexCount, 3 * sizeof(dTriIndex));
				dGeomTriMeshSetData(pOde->m_terrainTriMeshGeom[i], pOde->m_terrainTriMeshData[i]);

				//Set the position of the triMesh
				pOde->m_triMeshObjPos[i][0] = pos[0];
				pOde->m_triMeshObjPos[i][1] = pos[1];
				pOde->m_triMeshObjPos[i][2] = terrainHeight;

				break;
			}
		}	
		
	}
	///////////////////////////////////////////////////////////////

	if (int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact))) 
	{
		dMatrix3 RI;
		dRSetIdentity (RI);			
		for (i=0; i<numc; i++) 
		{
			dJointID c = dJointCreateContact(pOde->m_world,pOde->m_contactGroup,contact+i);
			dJointAttach (c,b1,b2);						
		}
	}
}

CODEObject::CODEObject()
{
	m_body = NULL;
//	m_pBody = NULL;
	m_geomCount = 0;
	for ( int i=0; i<MAX_GEOM_PER_BODY; ++i )
		m_geom[i] = NULL;
//	m_pGeom = NULL;
	m_pMass = NULL;
	m_visOrigOffset[0] = 0;
	m_visOrigOffset[1] = 0;
	m_visOrigOffset[2] = 0;
}

CODEObject::~CODEObject()
{
	if ( m_body )
		dBodyDestroy( m_body );
//	if ( m_pBody )
//		delete m_pBody;

	for ( int i=0; i<MAX_GEOM_PER_BODY; ++i )
		if ( m_geom[i] )
			dGeomDestroy( m_geom[i] );
//	if ( m_pGeom )
//		delete m_pGeom;

	if ( m_pMass )
		delete m_pMass;
}

void
CODEObject::GetPosRot( double position[3], double tangent[3], double lateral[3] )
{
	dVector3 pos;
	dReal* R;

	dBodyGetRelPointPos( m_body, m_visOrigOffset[0], m_visOrigOffset[1], m_visOrigOffset[2], pos );
//	m_pBody->getRelPointPos( m_visOrigOffset[0], m_visOrigOffset[1], m_visOrigOffset[2], pos );
	R = const_cast<dReal*>(dBodyGetRotation( m_body ));
//	R = const_cast<dReal*>(m_pBody->getRotation());
	position[0] = pos[0] * METERS_TO_FEET;
	position[1] = pos[1] * METERS_TO_FEET;
	position[2] = pos[2] * METERS_TO_FEET;

	tangent[0] = _R(0,0);
	tangent[1] = _R(1,0);
	tangent[2] = _R(2,0);
	lateral[0] = -_R(0,1);
	lateral[1] = -_R(1,1);
	lateral[2] = -_R(2,1);
}

void 
CODEObject::GetVels( double vels[6] )
{
	dVector3 linearVel;
	dReal* angularVel;

	dBodyGetRelPointVel( m_body, m_visOrigOffset[0], m_visOrigOffset[1], m_visOrigOffset[2], linearVel );
//	m_pBody->getRelPointVel( m_visOrigOffset[0], m_visOrigOffset[1], m_visOrigOffset[2], linearVel );
	angularVel = const_cast<dReal*>(dBodyGetAngularVel( m_body ));
//	angularVel = const_cast<dReal*>(m_pBody->getAngularVel());
	vels[0] = linearVel[0];
	vels[1] = linearVel[1];
	vels[2] = linearVel[2];
	vels[3] = angularVel[3];
	vels[4] = -angularVel[4];
	vels[5] = angularVel[5];
}


CODE::CODE(CVED::CCved * ptr)
{
	m_bInitialized = false;
	m_world = NULL;
	m_space = NULL;
	m_contactGroup = NULL;
//	m_pJointGroup = NULL;
	for ( int i=0; i<MAX_TERRAIN_TRIMESH; ++i )
	{
		m_terrainTriMeshData[i] = NULL;
		m_terrainTriMeshGeom[i] = NULL;
		/*m_triMeshObjPos[i][0] = NULL;
		m_triMeshObjPos[i][1] = NULL;
		m_triMeshObjPos[i][2] = NULL;*/
	}
	m_triMeshCount = 0;
	m_pCved = ptr;

	// two triangles for one rectangle centered around (x,y,z), vertices are:
	//                 0-----------3
	//                 |        -  |
	//                 |     *     |
	//                 |  -        |
	//                 1-----------2

	m_indices[0] = 0;
	m_indices[1] = 1;
	m_indices[2] = 3;

	m_indices[3] = 1;
	m_indices[4] = 2;
	m_indices[5] = 3;
}

CODE::~CODE()
{
	if ( m_contactGroup )
		dJointGroupDestroy( m_contactGroup );
//	if ( m_pJointGroup )
//		delete m_pJointGroup;

	for ( int i=0; i<m_triMeshCount; ++i )
	{
		if ( m_terrainTriMeshGeom[i] )
			dGeomDestroy( m_terrainTriMeshGeom[i] );
		if ( m_terrainTriMeshData[i] )
			dGeomTriMeshDataDestroy( m_terrainTriMeshData[i] );
	}

	if ( m_world )
		dWorldDestroy( m_world );
//	if ( m_pWorld )
//		delete m_pWorld;

	if ( m_space )
		dSpaceDestroy( m_space );
//	if ( m_pSpace )
//		delete m_pSpace;

	dCloseODE();
}

void 
CODE::Initialize( const double center[3], const double extents[3], double elevation )
{
	dVector3 spaceCenter, spaceExtents;

	if ( m_bInitialized )
		return;

	for ( int i=0; i<3; ++i )
	{
		spaceCenter[i] = center[i];
		spaceExtents[i] = extents[i];
	}

	dInitODE();
	m_world = dWorldCreate();
//	m_pWorld = new dWorld;
	dWorldSetGravity( m_world, 0, 0, -9.81 );  // earth gravity
//	m_pWorld->setGravity( 0, 0, -9.81 );  // earth gravity
	//m_pWorld->setCFM( 0 );
	dWorldSetCFM( m_world, 1e-5 );
//	m_pWorld->setCFM( 1e-5 );
	m_space = dQuadTreeSpaceCreate( 0, spaceCenter, spaceExtents, 6 );
//	m_pSpace = new dQuadTreeSpace( 0, spaceCenter, spaceExtents, 6 );
	//m_pSpace = new dHashSpace( 0 );
	dCreatePlane( m_space, 0, 0, 1, elevation * FEET_TO_METERS ); // level ground
//	dCreatePlane( m_pSpace->id(), 0, 0, 1, elevation ); // level ground
	m_contactGroup = dJointGroupCreate( 0 );
//	m_pJointGroup = new dJointGroup;

	m_bInitialized = true;
}

void
CODE::SimStep( float stepsize )
{
	if ( !m_bInitialized )
		return;

	dSpaceCollide( m_space, this, &ODE_CollisionCallback );
	dWorldQuickStep( m_world, stepsize);
	dJointGroupEmpty( m_contactGroup );

	// Update the position of all triMeshes
	for(int index = 0; index < m_triMeshCount; index++){
		dGeomSetPosition( m_terrainTriMeshGeom[index], m_triMeshObjPos[index][0], m_triMeshObjPos[index][1], m_triMeshObjPos[index][2] );
	}

}

/////////////////////////////////////////////////////////////////////////
//\brief 
//		This functions creates the ODE (Open Dynamics Engine) objects in the scenario
//
//\params 
//		CODEObject::EODEObjType objType - object type -- sphere, capped cylinder, cylinder or box
//		CODEObject& ODEObj
//		const dReal posRot[6] - initial position and rotation of the visual model origin 
//		const dReal vels[6] - initial linear and angular velocities
//		dReal mass - The mass of the object
//		const dReal cgOffset[3] - center of gravity in the object frame
//		const dReal dimension[3] - demension of the geometry
//		const dReal visOrigOffset[3] - the offset of the visual model origin relative to the center of the body
//		const SBodyProps& bodyProps
//\returns 
//		bool
/////////////////////////////////////////////////////////////////////////
bool
CODE::CreateODEObj(
	odePublic::EODEObjType objType,	// object type -- sphere, capped cylinder, cylinder or box
	CODEObject& ODEObj,
	const dReal posRot[6],				// initial position and rotation of the visual model origin 
	const dReal vels[6],				// initial linear and angular velocities 
	dReal mass,							// mass of the object 
	const dReal cgOffset[3],			// center of gravity in the object frame
	const dReal dimension[3],			// demension of the geometry
	const dReal visOrigOffset[3],		// the offset of the visual model origin relative to the center of the body
	const odePublic::SBodyProps& bodyProps
	)
{
	int i;
	dMatrix3 R;

	/*
	dRFromAxisAndAngle( R, 1, 0, 0, M_PI/3 );
	printf("rotx PI/3\n %.4f %.4f %.4f\n(%.4f %.4f %.4f)\n(%.4f %.4f %.4f)\n", 
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	dRFromAxisAndAngle( R, 0, 1, 0, M_PI/3 );
	printf("roty PI/3\n %.4f %.4f %.4f\n(%.4f %.4f %.4f)\n(%.4f %.4f %.4f)\n", 
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	dRFromAxisAndAngle( R, 0, 0, 1, M_PI/3 );
	printf("rotz PI/3\n %.4f %.4f %.4f\n(%.4f %.4f %.4f)\n(%.4f %.4f %.4f)\n", 
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	dRFromEulerAngles( R, M_PI/3, 0, 0 );
	printf("Euler phi PI/3\n %.4f %.4f %.4f\n(%.4f %.4f %.4f)\n(%.4f %.4f %.4f)\n", 
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	dRFromEulerAngles( R, 0, M_PI/3, 0 );
	printf("Euler theta PI/3\n %.4f %.4f %.4f\n(%.4f %.4f %.4f)\n(%.4f %.4f %.4f)\n", 
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	dRFromEulerAngles( R, 0, 0, M_PI/3 );
	printf("Euler psi PI/3\n %.4f %.4f %.4f\n(%.4f %.4f %.4f)\n(%.4f %.4f %.4f)\n", 
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	*/

	// Looks like the rotation matrix generated by dRFromEulerAngles() is transposed (a mistake?)
	// Have to reverse orders of rotx, roty, rotz, and negate them as well, which means pitchRate 
	// ( posRot[4] ) doesn't need to be negated.
	dReal phi, theta, psi;

	cgOffset[0];
	cgOffset[1];
	cgOffset[2];

	phi = posRot[3];
	theta = -posRot[4];
	psi = posRot[5];
	dRFromEulerAngles( R, phi, theta, psi );
	/*
	printf("Euler phi %.4f theta %.4f psi %.4f\n %.4f %.4f %.4f\n%.4f %.4f %.4f\n%.4f %.4f %.4f\n", 
		phi, theta, psi,
		_R(0,0), _R(0,1), _R(0,2),
		_R(1,0), _R(1,1), _R(1,2),
		_R(2,0), _R(2,1), _R(2,2));
	*/

	ODEObj.m_body = dBodyCreate( m_world );
	dBodySetPosition( ODEObj.m_body, posRot[0] * FEET_TO_METERS, posRot[1] * FEET_TO_METERS, posRot[2] * FEET_TO_METERS ); // use the position of visual model origin 
	dBodySetRotation( ODEObj.m_body, R );
	dBodySetLinearVel( ODEObj.m_body, vels[0], vels[1], vels[2] );
	dBodySetAngularVel( ODEObj.m_body, vels[3], -vels[4], vels[5] );

	ODEObj.m_pMass = new dMass;

	if ( objType == odePublic::e_COMPOSITE )
	{
		dReal compsize[MAX_GEOM_PER_BODY][3];
		dReal compmass[MAX_GEOM_PER_BODY];
		dReal dpos[MAX_GEOM_PER_BODY][3];	// position offsets for encapsulated geometries
		dMass m2;

		switch ( bodyProps.cigiId ) {
		case 505:  // drum
		{
			ODEObj.m_geomCount = 4;

			// start accumulating masses for the encapsulated geometries
			dMassSetZero(ODEObj.m_pMass);

			// set component sizes
			compsize[0][0] = 1.63;  // bottom plate (cylinder)
			compsize[0][2] = 0.1;
			compsize[1][0] = 0.9775;  // lower half of drum (cylinder)
			compsize[1][2] = 1.45;
			compsize[2][0] = 0.745;  // upper half of drum (cylinder)
			compsize[2][2] = 1.45;
			compsize[3][0] = 1.125;  // top handle (box)
			compsize[3][1] = 0.125;
			compsize[3][2] = 0.29;

			// set component masses
			compmass[0] = 3.0;
			compmass[1] = 3.8;
			compmass[2] = 2.8;
			compmass[3] = 0.4;
			    
			// set component position offsets
			dpos[0][0] = 0;
			dpos[0][1] = 0;
			dpos[0][2] = 0.05;
			dpos[1][0] = 0;
			dpos[1][1] = 0;
			dpos[1][2] = 0.825;
			dpos[2][0] = 0;
			dpos[2][1] = 0;
			dpos[2][2] = 2.275;
			dpos[3][0] = 0;
			dpos[3][1] = 0;
			dpos[3][2] = 3.145;
			
			// bottom plate
			dMassSetCylinder( &m2, 1, 3, compsize[0][0] * FEET_TO_METERS, 
				compsize[0][2] * FEET_TO_METERS );
			dMassAdjust( &m2, compmass[0] );
			dMassTranslate( &m2, dpos[0][0], dpos[0][1], dpos[0][2] );
			dMassAdd( ODEObj.m_pMass, &m2 );
			ODEObj.m_geom[0] = dCreateCylinder( m_space, 
				compsize[0][0] * FEET_TO_METERS, compsize[0][2] * FEET_TO_METERS );
			
			// lower half of drum
			dMassSetCylinder( &m2, 1, 3, compsize[1][0] * FEET_TO_METERS, 
				compsize[1][2] * FEET_TO_METERS );
			dMassAdjust( &m2, compmass[1] );
			dMassTranslate( &m2, dpos[1][0], dpos[1][1], dpos[1][2] );
			dMassAdd( ODEObj.m_pMass, &m2 );
			ODEObj.m_geom[1] = dCreateCylinder( m_space, 
				compsize[1][0] * FEET_TO_METERS, compsize[1][2] * FEET_TO_METERS );

			// upper half of drum
			dMassSetCylinder( &m2, 1, 3, compsize[2][0] * FEET_TO_METERS, 
				compsize[2][2] * FEET_TO_METERS );
			dMassAdjust( &m2, compmass[2] );
			dMassTranslate( &m2, dpos[2][0], dpos[2][1], dpos[2][2] );
			dMassAdd( ODEObj.m_pMass, &m2 );
			ODEObj.m_geom[2] = dCreateCylinder( m_space, 
				compsize[2][0] * FEET_TO_METERS, compsize[2][2] * FEET_TO_METERS );

			// top handle (box)
			dMassSetBox( &m2, 1, compsize[3][0] * FEET_TO_METERS, 
				compsize[3][1] * FEET_TO_METERS, compsize[3][2] * FEET_TO_METERS );
			dMassAdjust( &m2, compmass[3] );
			dMassTranslate( &m2, dpos[3][0], dpos[3][1], dpos[3][2] );
			dMassAdd( ODEObj.m_pMass, &m2 );
			ODEObj.m_geom[3] = dCreateBox( m_space, 
				compsize[3][0] * FEET_TO_METERS, 
				compsize[3][1] * FEET_TO_METERS,
				compsize[3][2] * FEET_TO_METERS );

			break;
		}

		case 515:  // traffic cone 36"
			ODEObj.m_geomCount = 3;
			break;

		default:
			break;
		}

		ODEObj.m_visOrigOffset[0] = -ODEObj.m_pMass->c[0];
		ODEObj.m_visOrigOffset[1] = -ODEObj.m_pMass->c[1];
		ODEObj.m_visOrigOffset[2] = -ODEObj.m_pMass->c[2];

		for ( i=0; i<ODEObj.m_geomCount; ++i ) 
		{
			dGeomSetBody( ODEObj.m_geom[i], ODEObj.m_body );
			dGeomSetOffsetPosition( ODEObj.m_geom[i],
				dpos[i][0] - ODEObj.m_pMass->c[0],
				dpos[i][1] - ODEObj.m_pMass->c[1],
				dpos[i][2] - ODEObj.m_pMass->c[2] );
		}
		dMassTranslate( ODEObj.m_pMass, 
			-ODEObj.m_pMass->c[0],
			-ODEObj.m_pMass->c[1],
			-ODEObj.m_pMass->c[2] );

	}
	else
	{
		ODEObj.m_geomCount = 1;  // single geometry for the body 

		switch ( objType ) {
			case odePublic::e_SPHERE:
				dMassSetSphere( ODEObj.m_pMass, 1, dimension[0]/2.0 * FEET_TO_METERS );
				dMassAdjust( ODEObj.m_pMass, mass );
				ODEObj.m_geom[0] = dCreateSphere( m_space, dimension[0]/2.0 * FEET_TO_METERS );
				break;

			case odePublic::e_CYLINDER:
				dMassSetCylinder( ODEObj.m_pMass, 1, 3, dimension[0] * FEET_TO_METERS, dimension[2]/2.0 * FEET_TO_METERS );
				dMassAdjust( ODEObj.m_pMass, mass );
				ODEObj.m_geom[0] = dCreateCylinder( m_space, dimension[0] * FEET_TO_METERS, dimension[2]/2.0 * FEET_TO_METERS );
				break;

			case odePublic::e_CAPSULE:
				dMassSetCapsule( ODEObj.m_pMass, 1, 3, dimension[0] * FEET_TO_METERS, dimension[2]/2.0 * FEET_TO_METERS );
				dMassAdjust( ODEObj.m_pMass, mass );
				ODEObj.m_geom[0] = dCreateCapsule( m_space, dimension[0] * FEET_TO_METERS, dimension[2]/2.0 * FEET_TO_METERS );
				break;

			case odePublic::e_BOX:
				dMassSetBox( ODEObj.m_pMass, 5, dimension[0] * FEET_TO_METERS, dimension[1] * FEET_TO_METERS, dimension[2] * FEET_TO_METERS );
				dMassAdjust( ODEObj.m_pMass, mass );
				ODEObj.m_geom[0] = dCreateBox( m_space, dimension[0] * FEET_TO_METERS, dimension[1] * FEET_TO_METERS, dimension[2] * FEET_TO_METERS );
				break;

			default:
				break;
		}

		dGeomSetBody( ODEObj.m_geom[0], ODEObj.m_body );
		ODEObj.m_visOrigOffset[0] = visOrigOffset[0] * FEET_TO_METERS;
		ODEObj.m_visOrigOffset[1] = visOrigOffset[1] * FEET_TO_METERS;
		ODEObj.m_visOrigOffset[2] = visOrigOffset[2] * FEET_TO_METERS;
	}
	
	dMassTranslate( ODEObj.m_pMass, cgOffset[0] * FEET_TO_METERS, cgOffset[1] * FEET_TO_METERS, cgOffset[2] * FEET_TO_METERS );
	dBodySetMass( ODEObj.m_body, ODEObj.m_pMass ); 

	dVector3 relPos;

	dBodyGetRelPointPos( ODEObj.m_body,
						-ODEObj.m_visOrigOffset[0], 
						-ODEObj.m_visOrigOffset[1],
						-ODEObj.m_visOrigOffset[2], 
						relPos ); // relPos is where the origin of the body should be
	dBodySetPosition( ODEObj.m_body, relPos[0], relPos[1], relPos[2] ); // Now set the correct body position
	
	odePublic::SBodyProps* pBodyProps = new odePublic::SBodyProps;

	pBodyProps->bounceEnergyLoss = bodyProps.bounceEnergyLoss;
	pBodyProps->frictionCoeff = bodyProps.frictionCoeff;
	pBodyProps->solId = bodyProps.solId;
	pBodyProps->cvedId = bodyProps.cvedId;
	pBodyProps->contactSoftCFM = bodyProps.contactSoftCFM;
	pBodyProps->contactSoftERP = bodyProps.contactSoftERP;
	dBodySetData( ODEObj.m_body, pBodyProps );

	// objectSize represents the largest of the object's x, y, and z coords which
	// is used to size the triMesh
	int objectSize = (int)dimension[0];
	for(int i = 1; i < 3; i++){
		if(dimension[i] > objectSize){
			objectSize = (int)dimension[i];
		}
	}

	//The vertices are set so that the triMesh is a square, indices are set in the CODE constructor
	m_vertices[0][0] = (-2.0f*objectSize);
	m_vertices[0][1] = ( 2.0f*objectSize);
	m_vertices[0][2] = 0.0;

	m_vertices[1][0] = (-2.0f*objectSize);
	m_vertices[1][1] = (-2.0f*objectSize);
	m_vertices[1][2] = 0.0;

	m_vertices[2][0] = ( 2.0f*objectSize);
	m_vertices[2][1] = (-2.0f*objectSize);
	m_vertices[2][2] = 0.0;

	m_vertices[3][0] = (2.0f*objectSize);
	m_vertices[3][1] = (2.0f*objectSize);
	m_vertices[3][2] = 0.0;

	CVector3D normal;

	// Create the triMesh using vertices and indices indicated
	m_terrainTriMeshData[m_triMeshCount] = dGeomTriMeshDataCreate();
	dGeomTriMeshDataBuildSingle(m_terrainTriMeshData[m_triMeshCount], m_vertices[0], 3 * sizeof(float), m_vertexCount, &m_indices[0], m_indexCount, 3 * sizeof(dTriIndex));
	m_terrainTriMeshGeom[m_triMeshCount] = dCreateTriMesh(m_space, m_terrainTriMeshData[m_triMeshCount], 0, 0, 0);
	dGeomTriMeshSetData(m_terrainTriMeshGeom[m_triMeshCount], m_terrainTriMeshData[m_triMeshCount]);

	// Query the terrain for the height which will be used to place the triMesh
	double odeTerrainHeight;

	m_pCved->QryTerrain(relPos[0]* METERS_TO_FEET, relPos[1]* METERS_TO_FEET, relPos[2]* METERS_TO_FEET, odeTerrainHeight, normal, NULL);
	odeTerrainHeight = odeTerrainHeight*FEET_TO_METERS;

	// Set x an y positions for the triMesh
	m_triMeshObjPos[m_triMeshCount][0] = relPos[0];
	m_triMeshObjPos[m_triMeshCount][1] = relPos[1];

	// I beleive the relPos is in the center of the object. If the bottom of the object is below the terrain then move the object above the terrain
	// then set the height of the triMesh
	if(relPos[2] - 0.5*dimension[2] < odeTerrainHeight){
		dBodySetPosition( ODEObj.m_body, relPos[0], relPos[1], odeTerrainHeight + dimension[2]);
		m_triMeshObjPos[m_triMeshCount][2] = odeTerrainHeight;
	}
	else{
		m_triMeshObjPos[m_triMeshCount][2] = odeTerrainHeight;
	}

	m_triMeshCount++;
	
	return true;
}

bool
CODE::DeleteODEObj( CODEObject* pODEObj )
{
	odePublic::SBodyProps* pBodyProps = (odePublic::SBodyProps*)(dBodyGetData( pODEObj->m_body ));
	if ( pBodyProps )
		delete pBodyProps;
	delete pODEObj;

	return true;
}
