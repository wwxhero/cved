/*****************************************************************************
 *
 * (C) Copyright 2005 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:
 *
 * Author: Yefei He
 *
 * Date: March 2005
 *
 * Description: header file for the CODEObject and CODE classes that run
 *              dynamics for free motion objects using the ODE library
 *
 *
 ****************************************************************************/

#ifndef __ODE_DYNAMICS_H
#define __ODE_DYNAMICS_H

#ifndef dDOUBLE
#define dDOUBLE
#endif 

#include <ode/ode.h>
#include <map>
#include "odePublic.h"

#define MAX_GEOM_PER_BODY 10
#define MAX_TERRAIN_TRIMESH 100
#define MAX_ODE_OBJ 100

class CODEObject
{
public:
	CODEObject();
	~CODEObject();

	void GetPosRot( double position[3], double tangent[3], double lateral[3] ); // in feet
	void GetVels( double vels[6] ); // linear velocity in m/s and angular velocity in radians/s
	friend class CODE;

protected:
	int m_geomCount;
//	dBody* m_pBody;
//	dGeom* m_pGeom;
	dBodyID m_body;
	dGeomID m_geom[MAX_GEOM_PER_BODY];
	dMass* m_pMass;
	dReal m_visOrigOffset[3];
};
class CVED::CCved;
class CODE
{
public:
	CODE(CVED::CCved *);
	~CODE();
	void Initialize( const double center[3], const double extents[3], double elevation );
	void SimStep( float stepsize );

	bool CreateODEObj(
		odePublic::EODEObjType objType,  // object type -- sphere, capped cylinder, cylinder or box
		CODEObject& ODEObj,
		const double posRot[6],       // initial position and rotation in feet
		const double vels[6],         // initial linear and angular velocities 
										// linear velocity in m/s, and angular velocity in radians/s, 
										// in the order of roll rate, pitch rate and yaw rate
		double mass,            // mass of the object 
		const double cgOffset[3],     // center of gravity in the object frame
		const double dimension[3],    // demension of the geometry
		const double visOrigOffset[3], // the offset of the visual model origin relative to the center of the body
		const odePublic::SBodyProps& bodyProps   // body properties
		);
	bool DeleteODEObj( CODEObject* pODEObj );

	static friend void ODE_CollisionCallback(void* data, dGeomID o1, dGeomID o2);
	

protected:
	bool m_bInitialized;
	dWorldID m_world;
	dSpaceID m_space;
	dJointGroupID m_contactGroup;
	dTriMeshDataID m_terrainTriMeshData[MAX_TERRAIN_TRIMESH];
	dGeomID m_terrainTriMeshGeom[MAX_TERRAIN_TRIMESH];
	double m_triMeshObjPos[MAX_TERRAIN_TRIMESH][3];

	//CODEObject m_odeObjects[MAX_ODE_OBJ];

	static const int m_vertexCount = 4;
	static const int m_indexCount = 2 * 3;
	float m_vertices[m_vertexCount][3];
	int m_indices[m_indexCount]; 
	int m_triMeshCount;
	CVED::CCved* m_pCved;
};

#endif /* __ODE_DYNAMICS_H */
