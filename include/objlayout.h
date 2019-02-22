/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: objlayout.h,v 1.112 2018/09/12 20:06:55 IOWA\dheitbri Exp $
 *
 * Author:
 *
 * Date:
 *
 * Description:
 *
 * This file contains the various type specific definitions for the
 * data structures used by objects.
 *
 * The file should be updated when new SOL categories are included
 * in the sol file.  In general, each SOL category should have
 * a corresponding CVED type, although additional CVED types
 * can be defined that have no corresponding SOL categories.
 *
 * The file should compile under C & C++ as it is included by
 * the implementation files of CVED and the LRI parser.
 *
 * The file contains:
 *	struct cvObjAttr - shared attributes for all object types
 *	union  cvTObjState - union with type specific object state vars
 *  union  cvTObjContInp - union with type specific object control inputs
 *
 ****************************************************************************/

#ifndef __OBJ_LAYOUT_H
#define __OBJ_LAYOUT_H

#include <point3d.h>
#include "enumtostring.h"

#define cMAX_PIECES_IN_COMPOSITE_SIGN 10
//in the future we might want to manualy set the packing,
//the compilier will pick a packing that is optimal for
//itself, but changes can cause different sizes of bli's.
#pragma pack()
/*
 * This structure contains object attributes.  All
 * object types share the same attributes.
 */
typedef struct cvTObjAttr {
	TU32b        solId;     /* SOL identifier */
	TU32b        hcsmId;    /* HCSM identifier */
	double       xSize;     /* bounding box size along x axis */
	double       ySize;     /* bounding box size along y axis */
	double       zSize;     /* Height of object along z axis  */
	int	         cigi;		/* cigi identifier, should not be here but for convenience ...*/
	TU16b        colorIndex; /* Index into the sol object's color array, if any */

	float        frictionCoeff; /* surface friction coefficient for sliding motion */
	float        bounceEnergyLoss; /* fraction of the energy that will be lost when the object bounces */
} cvTObjAttr;

typedef struct cvTerQueryHint {
	int                   hintState;
	TRoadPoolIdx          roadId;
	TLongCntrlPntPoolIdx  roadPiece;
	TIntrsctnPoolIdx      intersection;
} cvTerQueryHint;

#define NUM_STATES_4_WHEEL_VEH 25
typedef struct cvTFourWheelVeh {
	double state[NUM_STATES_4_WHEEL_VEH];
} cvTFourWheelVeh;

#define NUM_INPUTS_4_WHEEL_VEH 10
typedef struct cvTVehLin {
	double A[NUM_STATES_4_WHEEL_VEH][NUM_STATES_4_WHEEL_VEH];
	double B[NUM_STATES_4_WHEEL_VEH][NUM_INPUTS_4_WHEEL_VEH];
} cvTVehLin;


/*
 * This enumeration represents the varying fidelity of dynamics available
 * for 4-wheel vehicles.
 */
typedef enum {
			eNON_LINEAR = 0,
			eFULL_LINEAR = 1,
			eREDUCED_LINEAR = 2
			} EDynaFidelity;

#define cCV_LEFT_TURN_SIGNAL	0x00000001
#define cCV_RIGHT_TURN_SIGNAL	0x00000002
#define cCV_HAZARD_LIGHTS		0x00000004
#define cCV_HIGHBEAM_LIGHTS		0x00000008
#define cCV_BRAKE_LIGHTS		0x00000010
#define cCV_OPERATING_LIGHTS	0x00000020
#define cCV_BACKUP_LIGHTS		0x00000040
#define cCV_EMERGENCY			0x00000080
#define cCV_DOOR_FRONTLEFT      0x00000100
#define cCV_DOOR_FRONTRIGHT     0x00000200
#define cCV_DOOR_BACKLEFT       0x00000400
#define cCV_DOOR_BACKRIGHT      0x00000800
#define cCV_DOOR_REARLEFT       0x00001000
#define cCV_DOOR_REARRIGHT      0x00002000
#define cCV_DOOR_SPEED(x) ((x)>>14)

#define cCV_AUD_SIREN			0x00000001
#define cCV_AUD_WARBLE			0x00000002
#define cCV_AUD_WORNBRAKES		0x00000010
#define cCV_AUD_WORNMUFFLER		0x00000020
#define cCV_AUD_SCRAPINGOBJ		0x00000040
#define cCV_AUD_COLLISION		0x00000080
#define cCV_AUD_HORN			0x00000100

#define cCV_SIGN_ARM_DOWN		0x00000001
#define cCV_SIGN_FLASHING_LIGHT	0x00000080
#define cCV_SIGN_GATE_BLINKER	0x00000004

#define cVO_CIRCLE				0x00000001
#define cVO_OCTAGON				0x00000002
#define cVO_TRIANGLE			0x00000004
#define cVO_RECTANGLE			0x00000008
#define cVO_HEXAGON				0x00000010
#define cVO_STAR				0x00000020
#define cVO_DIAMOND				0x00000040
#define cVO_CUSTOM_ICON			0x00000080

typedef struct TAvatarJoint {
	const char*		name;
	int				type;			//joint vrlink articulated type
	TVector3D		angle; 			//taitbryan euler
	TVector3D       angleRate;
	TVector3D		offset;
	TVector3D		offsetRate;
	TAvatarJoint*	child_first;
	TAvatarJoint*	sibling_next;
} TAvatarJoint;

#define VIRTUAL_ROOT(childjoint)\
	{"pos", 4064, {0}, {0}, {0}, {0},childjoint, NULL};

/*
 * The following 2 structures contains the states and control inputs
 * for vehicle objects that use the standard vehicle dynamics.
 */
typedef struct TVehicleState {
	TPoint3D          position;     /* object's position */
	TVector3D         tangent;      /* tangent vector (normalized) */
	TVector3D         lateral;      /* lateral vector (normalized) */
	TPoint3D          boundBox[2];  /* object's bounding box */
	double            vel;          /* velocity along tangent */
	TU16b             visualState;
	TU16b             audioState;
	double            acc;          /* acceleration along tangent */
	double            suspStif;     /* suspension stiffness */
	double            suspDamp;     /* suspension dampness */
	double            tireStif;     /* tire stiffness */
	double            tireDamp;     /* tire dampness */
	double            velBrake;
	cvTerQueryHint    posHint[4];
	double            latAccel;     /* vehicle's current lat accel  */
	EDynaFidelity     dynaFidelity; /* vehicle's dynamics fidelity  */
	//External Driver State Matches with DynObjState above this point
	//All new cells must be added beneath this point,
	//or be added to External Driver State as well
	int               dynaInitComplete; /* has dynamics been initialized? */
	int               qryTerrainErrCount; /* # of time QryTerrain offroad */
	int               braking;		/* when set, dec below threshold */
	int               brakeLightOnCount; /* how long in 'braking' mode */
	int               brakeLightOffCount;/* how long not in 'braking' mode */
	double			  steeringWheelAngle; /* stores the current steering wheel angle */

	/*double          horsepower;*/
	double            velLat;       /* lateral velocity       */
	double            velNorm;      /* velocity along terrain normal  */
	double            rollRate;     /* vehicle roll rate      */
	double            pitchRate;    /* vehicle pitch rate     */
	double            yawRate;      /* vehicle yaw rate       */
	float             tireRot[3];   /*front left, front right, rear*/
    int               extrnControlId; /* System ID if we are running a diff dyna */
#ifndef SMALL_BLI
	TS8b              reserved[44]; /*reserved for future use */
#endif
} TVehicleState;

typedef struct TVehicleContInp {
	double      targVel;       /* target velocity                       */
	double      targDist;      /* distance for achieving velocity       */
	TPoint3D    targPos;       /* target position                       */
	int         leadObj;       /* lead object's CVED id                 */
	double      leadTargDist;  /* target dist to maintain from lead obj */
	double      leadActualDist;/* actual dist to lead object            */

	/* new control inputs */
	int         haveAccel;     /* does the targAccel have a valid value?*/
	double      targAccel;     /* target acceleration                   */
	int         haveSteer;     /* does the targSteer have a valid value?*/
	float       targSteer;     /* target steering  (change float to fit
                                  new field */
    float       maxSteerRateRadpS;  /* Max turning rate Radians Per S   */
    TPoint3D    oldtargPos;       /* old target position                    */
#ifndef SMALL_BLI
	TS8b        reserved[64];     /*reserved for future use */
#endif
} TVehicleContInp;

typedef enum cvEObjMode {
	eCV_GROUND_TRAJ = 0,
	eCV_FREE_MOTION,
	eCV_COUPLED_OBJ,
	eCV_OBJ_REL_TRAJ,
	eCV_DIGUY, //<Pre programed path
	eCV_DIGUY_GUIDE_CONTROL, //<Guide based -
	eCV_DIGUY_DIR_CONTROL //< 100% control from Scenario control
} cvEObjMode;

/*
 * A union containing one structure per object type, each structure
 * containing type specific state variables.  The AnyObjState field
 * contains type independent state variables and is shared
 * by all types.
 */
typedef union cvTObjState {
	/* the following structure holds fields representing state  */
	/* variables applicable to all types of objects. */
	struct AnyObjState {
		TPoint3D          position;     /* where object is located */
		/* the following two vectors use right hand rule */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* objects' bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;  /*   1: left turn signal        */
                                        /*   2: right turn signal       */
                                        /*   4: hazard lights           */
                                        /*   8: high beam lights        */
                                        /*  16: brake lights            */
                                        /*  32: operating lights        */
                                        /*  64: backup lights           */

		TU16b             audioState;   /* 1-15: not used               */
                                        /*   16: worn brakes            */
                                        /*   32: worn muffler           */
                                        /*   64: scraping pillar or sign*/
                                        /*  128: collision              */
                                        /*  256: car horn               */

	} anyState;

	struct TrajFollowerState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* objects' bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
		//TU8b			  animationOn;  /*< is the animation on? */
        TU8b              classType;   /*0, vehicle, walker, diGuy, free-body*/
		cvTerQueryHint    posHint[3];

		/* traj follower mode related data */
		unsigned char curMode;
		unsigned char prevMode;
		int parentId;
		int useAsAbsoluteValue;   /* use initial position, rotation and velocities
								    * as absolute values for the free motion mode */
		double offset[6];
		double initVel[6];
		double rotMat[3][3];
		float tireRot[3];
		/* ode related variables */
		void* pODEObj;

	} trajFollowerState;

	struct VehicleState {
		TVehicleState     vehState;     /* contains state variables for
										 * vehicle objects that use the
										 * vehicle dynamics */
	} vehicleState;

	struct TrailerState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} trailerState;

	struct RailVehState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} railVehState;

	struct TerrainState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} terrainState;

	struct TrafficLightState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;

		eCVTrafficLightState	state;	/* Current state of the        */
										/* traffic light               */

	} trafficLightState;

	struct TrafficSignState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} trafficSignState;

	struct CompositeSignState {
		TPoint3D          position;     /* where object is located */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* objects' bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
		int               numChildren;  /* how many of the below array entries are valid */
		int               childrenOpts[cMAX_PIECES_IN_COMPOSITE_SIGN]; /* one option per child */
	} compositeSignState;

	struct ObstacleState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} obstacleState;
#if 0
	struct EnviroState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
		TPoint2D          vertex[cCV_MAX_ENVIRO_VERTS];
										/* array of vertices associated */
										/*  with the visual rep. of the */
										/*  environmental condition     */
		eCVEnviroType     enviroType;   /* type of enviroment condition */
	} enviroState;
#endif
	struct PoiState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];	/* object's bounding box */
		double            vel;		    /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} poiState;

	struct CoordinatorObjectState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];	/* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
	} coordinatorObjectState;

	struct VirtualObjectState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];	/* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
		///new stuff.........
		TPoint3D		  overlayPosition; /*Overlay Draw Position */
		double            rotation;
		TU8b              objType;         /*predef shape ID */
		float             color[4];		   /*RGBA*/
		float             colorBorder[4];
		float             scale[2];       /*scale X, scale y*/
		int	              parentId;
		TU16b             StateIndex;
		TU8b              DrawScreen;         /*predef shape ID */
		TU16b             ParseBlockID;       /*predef shape ID */
        TS8b              lightID;
	} virtualObjectState;

	struct ExternalDriverState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];	/* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;

		/* the following are needed for fake driver vehicle dynamics */
		double            acc;          /* acceleration along tangent */
		double            suspStif;     /* suspension stiffness */
		double            suspDamp;     /* suspension dampness */
		double            tireStif;     /* tire stiffness */
		double            tireDamp;     /* tire dampness */
		double            velBrake;
		cvTerQueryHint    posHint[4];
		double            latAccel;     /* vehicle's current lat accel  */
		EDynaFidelity     dynaFidelity; /* vehicle's dynamics fidelity  */
		//External Driver State Matches with DynObjState above this point
		//....new elements should be added after here.

		TVector3D         angularVel;   /* object's angular velocity */

	} externalDriverState;

	struct WalkerState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];  /* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;
		TU8b			  animationOn;
		//bool              animationOn;
	} walkerState;

	struct AvatarState {
		TPoint3D          position;     /* object's position */
		TVector3D         tangent;      /* tangent vector (normalized) */
		TVector3D         lateral;      /* lateral vector (normalized) */
		TPoint3D          boundBox[2];	/* object's bounding box */
		double            vel;          /* velocity along tangent */
		TU16b             visualState;
		TU16b             audioState;

		/* the following are needed for fake driver vehicle dynamics */
		double            acc;          /* acceleration along tangent */
		double            latAccel;     /* vehicle's current lat accel  */
		//....new elements should be added after here.

		TVector3D         angularVel;   /* object's angular velocity */
		TAvatarJoint*     child_first;		/* the memory it points to will be located in heap */
	} avatarState;

} cvTObjState;



/*
 * The control inputs are state variables that in conjunction
 * with the current state predict the future state.
 * A union containing one structure per object type.
 */
typedef union cvTObjContInp {
	struct AnyObjContInp {
		int dummy;              /* C requires at least one field */
	} anyContInp;

	struct TrajFollowerContInp {
		int         dummy;         /* C requires at least one field         */
		TVector2D   targDir;       /* normalized 2d directional vector      */
		double      targVel;       /* target velocity                       */
		double      brakeDist;     /* distance for braking                  */
		double      accelDist;     /* distance for reaching targVel         */
		TVector2D   objOri;        /* orientation independent of targDir    */
	} trajFollowerContInp;

	struct VehicleContInp {
		int         dummy;         /* C requires at least one field         */
		TVehicleContInp  contInp;  /* control inputs for vehicle objects    *
								    * that use the standard vehicle         *
									* dynamics                              */
	} vehicleContInp;

	struct TrailerContInp {
		int dummy;
	} trailerContInp;

	struct RailVehContInp {
		int dummy;
	} railVehContInp;

	struct TerrainContInp {
		int dummy;
	} terrainContInp;

	struct TrafficLightContInp {
		int dummy;
	} trafficLightContInp;

	struct TrafficSignContInp {
		int dummy;
	} trafficSignContInp;

	struct CompositeSignContInp {
		int dummy;
	} compositeSignContInp;

	struct ObstacleContInp {
		int dummy;
	} obstacleContInp;

	struct PoiContInp {
		int dummy;
	} poiContInp;

	struct CoordinatorContInp {
		int dummy;
	} coordinatorContInp;

	struct ExternalDriverContInp {
		int dummy;
	} externalDriverContInp;

	struct WalkerContInp {
		int dummy;
	} walkerContInp;

	struct VisualObjectContInp{
		int dummy;
	} visualObjectContInp;

} cvTObjContInp;
#pragma pack()
#endif  /* __OBJ_LAYOUT_H */
