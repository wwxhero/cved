/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: vehicledynamics.cxx,v 1.115 2016/10/18 14:21:35 IOWA\dheitbri Exp $
 *
 * Author:  Gustavo Ordaz Hernandez, Chris Schwarz
 *
 * Date:    August, 1999
 *
 * Description:  Vehicle dynamics.
 *
 ****************************************************************************/

#include "cvedpub.h"
#include "cvedstrc.h"

using namespace CVED;

#undef DEBUG_DYN
#undef DEBUG_DYN_FOLLOW
#undef MANUAL_TERRAIN
#undef DEBUG_VEH_STATE

//
// Macros.
//
#define NUM_TIRES 4
#define NUM_INPUTS 7

cvTFourWheelVeh	g_stateVector[cNUM_DYN_OBJS];
cvTVehLin		g_vehLin[cNUM_DYN_OBJS];

//
// Define a structure to hold vehicle information.
//
typedef struct TVehInfo {
	CPoint3D  position;
	CVector3D velocity;
	CVector3D acceleration;
	CPoint3D  orient;
	CVector3D orientVel;
	double    steerInput;
} TVehInfo;

typedef struct TVehSolAttr {
	double axleHeight;
	double mass;
	double tireInert;
	double tireMass;
	double tireRadius;
	double wheelBaseForw;
	double wheelBaseRear;
	double wheelTrack;
	double brakeLimitMassTorqueRatio;
	double horsePower;
	double maximumSpeed;
	CDblArr vInertia;
	CTireConfig tireLocPos;
} TVehSolAttr;

typedef struct TQryTerrainEfficientInfo {
	int            refresh;
	cvTerQueryHint hint;
} TQryTerrainEfficientInfo;

typedef struct TQryTerrainInfo {
	double         terrainHeight[NUM_TIRES];
	CVector3D      terrainNormal[NUM_TIRES];
} TQryTerrainInfo;

//
// Constants.
//
const int cNUM_LIN_STATES = 17;
const double cGRAVITY = 9.80665;
const double cPI = 3.14159;
const double cMU = 1.0; //0.85;
const double cTargetDistLimitRel = 0.1;
const double cTargetDistLimitAbs = 0.001;
const double cBrakeAccelThresh = -0.5;
//Horatiu
const double cDragCoeff = 0.1;

const double cSteerLimit = 0.41;
const double cAccelLimitMassTorqueRatio = 2.3;
const double cVelocityThresh = 0.1;
const double cLinearVelocityThresh = 0.1;
const double cFeetToMeters = 0.3048;
const double cMetersToFeet = 3.28083;
const double cNEAR_ZERO = 0.001;
const double cAlfaMult  = 100.0;
const double cSteerRateLimit = 0.0376; // approx. 65deg/sec rate limit


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates an acceleration given the initial velocity,
//   final velocity and the distance.
//
// Remarks:  Returns a large acceleration for distances near zero.
//
// Arguments:
//   initVel - The initial velocity in meters/second.
//   finalVel - The final velocity in meters/second.
//   dist - The distance in meters.
//
// Returns:  A double containing the acceleration in meters/second^2.
//
//////////////////////////////////////////////////////////////////////////////
static double
CalcAccel( 
			const double initVel, 
			const double finalVel,
			const double dist
			)
{

	double velDiff = ( finalVel * finalVel ) - ( initVel * initVel );

	//
	// Protect from divide by zero.
	//
	double accel;
	if( dist > cNEAR_ZERO )
	{
		accel = velDiff / ( 2.0 * dist );
	}
	else 
	{
		if( finalVel <= initVel )
		{
			accel = -1000.0;
		}
		else
		{
			accel = 1000.0;  // a large value
		}
	}

	return accel;

}  // end of CalcAccel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Returns the sign of a double
//
// Remarks:  
//
// Arguments:
//   x - The argument
//   y - The sign of the argument
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static double 
SignOf( double x )
{

	double y = 1.0;
	if ( x < 0.0 ) y = -1.0;
	return y;

}  // end of SignOf


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Limits the rate of change of a signal
//
// Remarks:  
//
// Arguments:
//   input - the signal input
//   *prevIn - the previous value of the input
//   limit - the rate limit value
//   output - the rate-limited output
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static double 
RateLimit( double input, double prevIn, double limit )
{
	double output;

	if ( input > prevIn )
	{
		if (input-prevIn > limit)
		{
			output = prevIn + limit;
		}
		else
		{
			output = input;
		}
	}
	else
	{
		if (prevIn-input > limit)
		{
			output = prevIn - limit;
		}
		else
		{
			output = input;
		}
	}

	return output;

}  // end of RateLimit


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes a coordinate transformation matrix.
//
// Remarks:  Use the roll, pitch and yaw to initialize a coordinate
//   transofrmation matrix.
//
// Arguments:
//   roll   - The roll.
//   pitch  - The pitch.
//   yaw    - The yaw.
//   Matrix - The output matrix.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void 
OrientMatrix(
			double roll,
			double pitch,
			double yaw,
			double (*pMatrix)[3][3] 
			)
{
   (*pMatrix)[0][0] = cos(yaw) * cos(pitch);
   (*pMatrix)[0][1] = -sin(yaw) * cos(roll) - cos(yaw) * sin(pitch) * sin(roll);
   (*pMatrix)[0][2] = sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll);
   (*pMatrix)[1][0] = sin(yaw) * cos(pitch);
   (*pMatrix)[1][1] = cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll);
   (*pMatrix)[1][2] = -cos(yaw) * sin(roll) - sin(yaw) * sin(pitch) * cos(roll);
   (*pMatrix)[2][0] = sin(pitch);
   (*pMatrix)[2][1] = cos(pitch) * sin(roll);
   (*pMatrix)[2][2] = cos(pitch) * cos(roll);
}  // end of OrientMatrix


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the product of a matrix and a vector.
//
// Remarks:  
//
// Arguments:
//   pMatrix - Input matrix.
//   vecIn   - Input vector (CVector3D).
//   trans   - Is it a transpose matrix.
//
// Returns: A vector with the result.
//
//////////////////////////////////////////////////////////////////////////////
static CVector3D 
ProdMatrixVector(
			double (*pMatrix)[3][3],
			CVector3D vecIn,
			bool trans = false
			)
{
	CVector3D temp1, temp2, temp3, vecOut;

	if( trans ) 
	{	
		// transpose matrix
		temp1.m_i = (*pMatrix)[0][0];
		temp1.m_j = (*pMatrix)[1][0];
		temp1.m_k = (*pMatrix)[2][0];
		temp2.m_i = (*pMatrix)[0][1];
		temp2.m_j = (*pMatrix)[1][1];
		temp2.m_k = (*pMatrix)[2][1];
		temp3.m_i = (*pMatrix)[0][2];
		temp3.m_j = (*pMatrix)[1][2];
		temp3.m_k = (*pMatrix)[2][2];
	} 
	else 
	{
		// direct matrix
		temp1.m_i = (*pMatrix)[0][0];
		temp1.m_j = (*pMatrix)[0][1];
		temp1.m_k = (*pMatrix)[0][2];
		temp2.m_i = (*pMatrix)[1][0];
		temp2.m_j = (*pMatrix)[1][1];
		temp2.m_k = (*pMatrix)[1][2];
		temp3.m_i = (*pMatrix)[2][0];
		temp3.m_j = (*pMatrix)[2][1];
		temp3.m_k = (*pMatrix)[2][2];
	}

	vecOut.m_i = temp1.DotP( vecIn );
	vecOut.m_j = temp2.DotP( vecIn );
	vecOut.m_k = temp3.DotP( vecIn );

	return vecOut;
}  // end of ProdMatrixVector


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Calculates the product of a matrix and a vector.
//
// Remarks:  
//
// Arguments:
//   pMatrix - Input matrix.
//   vecIn   - Input vector (CVector3D).
//   trans   - Is it a transpose matrix.
//
// Returns: A vector with the result.
//
//////////////////////////////////////////////////////////////////////////////
static CPoint3D 
ProdMatrixVector(
			double (*pMatrix)[3][3],
			CPoint3D vecIn,
			bool trans = false
			)
{

	CPoint3D vecOut;
	if( trans ) 
	{
		// transpose matrix
		vecOut.m_x = ( (*pMatrix)[0][0] * vecIn.m_x + 
					   (*pMatrix)[1][0] * vecIn.m_y + 
					   (*pMatrix)[2][0] * vecIn.m_z );
		vecOut.m_y = ( (*pMatrix)[0][1] * vecIn.m_x + 
					   (*pMatrix)[1][1] * vecIn.m_y + 
					   (*pMatrix)[2][1] * vecIn.m_z );
		vecOut.m_z = ( (*pMatrix)[0][2] * vecIn.m_x + 
					   (*pMatrix)[1][2] * vecIn.m_y + 
					   (*pMatrix)[2][2] * vecIn.m_z );
	} 
	else 
	{
		// direct matrix
		vecOut.m_x = ( (*pMatrix)[0][0] * vecIn.m_x + 
					   (*pMatrix)[0][1] * vecIn.m_y + 
					   (*pMatrix)[0][2] * vecIn.m_z );
		vecOut.m_y = ( (*pMatrix)[1][0] * vecIn.m_x + 
					   (*pMatrix)[1][1] * vecIn.m_y + 
					   (*pMatrix)[1][2] * vecIn.m_z );
		vecOut.m_z = ( (*pMatrix)[2][0] * vecIn.m_x + 
					   (*pMatrix)[2][1] * vecIn.m_y + 
					   (*pMatrix)[2][2] * vecIn.m_z );
	}

	return vecOut;
}  // end of ProdMatrixVector (for CPoint3D)

//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the linear matrices for the full linear 
//   vehicle model.
//
// Remarks:  
//
// Arguments:
//   cpSolObj  - Pointer to SOL base object.
//   cpCurState - Pointer to the vehicle's current state.
//
// Returns:
//   vehLin   - Linear vehicle structure.
//
//////////////////////////////////////////////////////////////////////////////
static cvTVehLin
InitFullLinearMatrix(
			const CSolObj* cpSolObj, 
			const TVehicleState* cpCurState,
			const TVehSolAttr& cVehAttr
			)
{
	int i;
	int j;
	cvTVehLin vehLin;
	for( i=0; i<NUM_STATES_4_WHEEL_VEH; i++ ) 
	{
	    for( j=0; j<NUM_STATES_4_WHEEL_VEH; j++ ) 
		{
	        vehLin.A[i][j] = 0.0;
	    }
	    for( j=0; j<NUM_INPUTS_4_WHEEL_VEH; j++ ) 
		{
	        vehLin.B[i][j] = 0.0;
	    }
	}
	double Fn = 2000;  //  normal force hack
	double ks = cpCurState->suspStif;
	double us = cpCurState->suspDamp;
	double kt = cpCurState->tireStif;
	double ut = cpCurState->tireDamp;

	//
	//  Initialize matrices A and B for the formula
	//  yd = A*y + B*u
	//  where y is the state vector and u is the input vector
	//  The states in y are organized as follows:
	//  [yd wz tz zd z wx tx wy ty zdt1 zt1 zdt2 zt2 zdt3 zt3 zdt4 zt4]
	//
	vehLin.A[0][0] = -4.0*cMU*Fn/cVehAttr.mass;
	vehLin.A[0][1] = -2.0*cMU*Fn*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)/cVehAttr.mass;
	vehLin.A[1][0] = -2.0*cMU*Fn*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)/cVehAttr.vInertia.m_items[2];
	vehLin.A[1][1] = -2.0*cMU*Fn*(cVehAttr.wheelBaseForw*cVehAttr.wheelBaseForw+cVehAttr.wheelBaseRear*cVehAttr.wheelBaseRear)/cVehAttr.vInertia.m_items[2];
	vehLin.A[2][1] = 1.0;
	vehLin.A[3][3] = -4.0*us/cVehAttr.mass;
	vehLin.A[3][4] = -4.0*ks/cVehAttr.mass;
	vehLin.A[3][9] = 1.0*us/cVehAttr.mass;
	vehLin.A[3][10] = 1.0*ks/cVehAttr.mass;
	vehLin.A[3][11] = 1.0*us/cVehAttr.mass;
	vehLin.A[3][12] = 1.0*ks/cVehAttr.mass;
	vehLin.A[3][13] = 1.0*us/cVehAttr.mass;
	vehLin.A[3][14] = 1.0*ks/cVehAttr.mass;
	vehLin.A[3][15] = 1.0*us/cVehAttr.mass;
	vehLin.A[3][16] = 1.0*ks/cVehAttr.mass;
	vehLin.A[4][3] = 1.0;
	vehLin.A[5][5] = -cVehAttr.wheelTrack*cVehAttr.wheelTrack*us/cVehAttr.vInertia.m_items[0];
	vehLin.A[5][6] = -cVehAttr.wheelTrack*cVehAttr.wheelTrack*ks/cVehAttr.vInertia.m_items[0];
	vehLin.A[5][9] = cVehAttr.wheelTrack*us/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][10] = cVehAttr.wheelTrack*ks/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][11] = -cVehAttr.wheelTrack*us/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][12] = -cVehAttr.wheelTrack*ks/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][13] = cVehAttr.wheelTrack*us/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][14] = cVehAttr.wheelTrack*ks/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][15] = -cVehAttr.wheelTrack*us/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[5][16] = -cVehAttr.wheelTrack*ks/(2.0*cVehAttr.vInertia.m_items[0]);
	vehLin.A[6][5] = 1.0;
	vehLin.A[7][3] = -2.0*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][4] = -2.0*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][7] = -2.0*(cVehAttr.wheelBaseForw*cVehAttr.wheelBaseForw+cVehAttr.wheelBaseRear*cVehAttr.wheelBaseRear)*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][8] = -2.0*(cVehAttr.wheelBaseForw*cVehAttr.wheelBaseForw+cVehAttr.wheelBaseRear*cVehAttr.wheelBaseRear)*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][9] = cVehAttr.wheelBaseForw*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][10] = cVehAttr.wheelBaseForw*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][11] = cVehAttr.wheelBaseForw*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][12] = cVehAttr.wheelBaseForw*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][13] = -cVehAttr.wheelBaseRear*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][14] = -cVehAttr.wheelBaseRear*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][15] = -cVehAttr.wheelBaseRear*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][16] = -cVehAttr.wheelBaseRear*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[8][7] = 1.0;
	vehLin.A[9][3] = 1.0*us/cVehAttr.tireMass;
	vehLin.A[9][4] = 1.0*ks/cVehAttr.tireMass;
	vehLin.A[9][5] = cVehAttr.wheelTrack*us/(2.0*cVehAttr.tireMass);
	vehLin.A[9][6] = cVehAttr.wheelTrack*ks/(2.0*cVehAttr.tireMass);
	vehLin.A[9][7] = -cVehAttr.wheelBaseForw*us/cVehAttr.tireMass;
	vehLin.A[9][8] = -cVehAttr.wheelBaseForw*ks/cVehAttr.tireMass;
	vehLin.A[9][9] = -1.0*(us+ut)/cVehAttr.tireMass;
	vehLin.A[9][10] = -1.0*(ks+kt)/cVehAttr.tireMass;
	vehLin.A[10][9] = 1.0;
	vehLin.A[11][3] = 1.0*us/cVehAttr.tireMass;
	vehLin.A[11][4] = 1.0*ks/cVehAttr.tireMass;
	vehLin.A[11][5] = -cVehAttr.wheelTrack*us/(2.0*cVehAttr.tireMass);
	vehLin.A[11][6] = -cVehAttr.wheelTrack*ks/(2.0*cVehAttr.tireMass);
	vehLin.A[11][7] = -cVehAttr.wheelBaseForw*us/cVehAttr.tireMass;
	vehLin.A[11][8] = -cVehAttr.wheelBaseForw*ks/cVehAttr.tireMass;
	vehLin.A[11][11] = -1.0*(us+ut)/cVehAttr.tireMass;
	vehLin.A[11][12] = -1.0*(ks+kt)/cVehAttr.tireMass;
	vehLin.A[12][11] = 1.0;
	vehLin.A[13][3] = 1.0*us/cVehAttr.tireMass;
	vehLin.A[13][4] = 1.0*ks/cVehAttr.tireMass;
	vehLin.A[13][5] = cVehAttr.wheelTrack*us/(2.0*cVehAttr.tireMass);
	vehLin.A[13][6] = cVehAttr.wheelTrack*ks/(2.0*cVehAttr.tireMass);
	vehLin.A[13][7] = cVehAttr.wheelBaseRear*us/cVehAttr.tireMass;
	vehLin.A[13][8] = cVehAttr.wheelBaseRear*ks/cVehAttr.tireMass;
	vehLin.A[13][13] = -1.0*(us+ut)/cVehAttr.tireMass;
	vehLin.A[13][14] = -1.0*(ks+kt)/cVehAttr.tireMass;
	vehLin.A[14][13] = 1.0;
	vehLin.A[15][3] = 1.0*us/cVehAttr.tireMass;
	vehLin.A[15][4] = 1.0*ks/cVehAttr.tireMass;
	vehLin.A[15][5] = -cVehAttr.wheelTrack*us/(2.0*cVehAttr.tireMass);
	vehLin.A[15][6] = -cVehAttr.wheelTrack*ks/(2.0*cVehAttr.tireMass);
	vehLin.A[15][7] = cVehAttr.wheelBaseRear*us/cVehAttr.tireMass;
	vehLin.A[15][8] = cVehAttr.wheelBaseRear*ks/cVehAttr.tireMass;
	vehLin.A[15][15] = -1.0*(us+ut)/cVehAttr.tireMass;
	vehLin.A[15][16] = -1.0*(ks+kt)/cVehAttr.tireMass;
	vehLin.A[16][15] = 1.0;

	vehLin.B[0][0] = 2.0*cMU*Fn/cVehAttr.mass;
	vehLin.B[1][0] = 2.0*cMU*Fn*cVehAttr.wheelBaseForw/cVehAttr.vInertia.m_items[2];
	vehLin.B[9][1] = vehLin.B[11][2] = vehLin.B[13][3] = vehLin.B[15][4] = 1.0*kt/cVehAttr.tireMass;
	vehLin.B[3][5] = vehLin.B[9][5] = vehLin.B[11][5] = vehLin.B[13][5] = vehLin.B[15][5] = -1.0;
	vehLin.B[3][6] = -4.0*ks/cVehAttr.mass;
	vehLin.B[9][6] = vehLin.B[11][6] = vehLin.B[13][6] = vehLin.B[15][6] = 1.0*ks/cVehAttr.tireMass;

	return vehLin;
}  // End of InitFullLinearMatrix


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the linear matrices for the reduced linear 
//   vehicle model.
//
// Remarks:  
//
// Arguments:
//   cpSolObj  - Pointer to SOL base object.
//   cpCurState - Pointer to the vehicle's current state.
//
// Returns:
//   vehLin   - Linear vehicle structure.
//
//////////////////////////////////////////////////////////////////////////////
static cvTVehLin
InitReducedLinearMatrix( 
			const CSolObj* cpSolObj,
			const TVehicleState* cpCurState,
			const TVehSolAttr& cVehAttr
			)
{
	int i,j;
	cvTVehLin vehLin;
	for( i=0; i<NUM_STATES_4_WHEEL_VEH; i++ ) 
	{
	    for( j=0; j<NUM_STATES_4_WHEEL_VEH; j++ ) 
		{
	        vehLin.A[i][j] = 0.0;
	    }
	    for( j=0; j<NUM_INPUTS_4_WHEEL_VEH; j++ ) 
		{
	        vehLin.B[i][j] = 0.0;
	    }
	}
	double Fn = 2000;  //  normal force hack
	double ks = cpCurState->suspStif;
	double us = cpCurState->suspDamp;
	double kt = cpCurState->tireStif;
	double ut = cpCurState->tireDamp;

	//
	//  Initialize matrices A and B for the formula
	//  yd = A*y + B*u
	//  where y is the state vector and u is the input vector
	//  The states in y are organized as follows:
	//  [yd wz tz zd z wx tx wy ty zdt1 zt1 zdt2 zt2 zdt3 zt3 zdt4 zt4]
	//
	vehLin.A[0][0] = -4.0*cMU*Fn/cVehAttr.mass;
	vehLin.A[0][1] = -2.0*cMU*Fn*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)/cVehAttr.mass;
	vehLin.A[1][0] = -2.0*cMU*Fn*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)/cVehAttr.vInertia.m_items[2];
	vehLin.A[1][1] = -2.0*cMU*Fn*(cVehAttr.wheelBaseForw*cVehAttr.wheelBaseForw+cVehAttr.wheelBaseRear*cVehAttr.wheelBaseRear)/cVehAttr.vInertia.m_items[2];
	vehLin.A[2][1] = 1.0;
	vehLin.A[3][3] = -4.0*us/cVehAttr.mass;
	vehLin.A[3][4] = -4.0*ks/cVehAttr.mass;
	vehLin.A[4][3] = 1.0;
	vehLin.A[5][5] = -cVehAttr.wheelTrack*cVehAttr.wheelTrack*us/cVehAttr.vInertia.m_items[0];
	vehLin.A[5][6] = -cVehAttr.wheelTrack*cVehAttr.wheelTrack*ks/cVehAttr.vInertia.m_items[0];
	vehLin.A[6][5] = 1.0;
	vehLin.A[7][3] = -2.0*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][4] = -2.0*(cVehAttr.wheelBaseForw-cVehAttr.wheelBaseRear)*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][7] = -2.0*(cVehAttr.wheelBaseForw*cVehAttr.wheelBaseForw+cVehAttr.wheelBaseRear*cVehAttr.wheelBaseRear)*us/cVehAttr.vInertia.m_items[1];
	vehLin.A[7][8] = -2.0*(cVehAttr.wheelBaseForw*cVehAttr.wheelBaseForw+cVehAttr.wheelBaseRear*cVehAttr.wheelBaseRear)*ks/cVehAttr.vInertia.m_items[1];
	vehLin.A[8][7] = 1.0;

	vehLin.B[0][0] = 2.0*cMU*Fn/cVehAttr.mass;
	vehLin.B[1][0] = 2.0*cMU*Fn*cVehAttr.wheelBaseForw/cVehAttr.vInertia.m_items[2];
	vehLin.B[3][1] = vehLin.B[3][2] = vehLin.B[3][3] = vehLin.B[3][4] = 1.0*ks/cVehAttr.mass;
	vehLin.B[5][1] = vehLin.B[5][3] = cVehAttr.wheelTrack*ks/2.0/cVehAttr.vInertia.m_items[0];
	vehLin.B[5][2] = vehLin.B[5][4] = -cVehAttr.wheelTrack*ks/2.0/cVehAttr.vInertia.m_items[0];
	vehLin.B[7][1] = vehLin.B[7][2] = cVehAttr.wheelBaseForw*ks/cVehAttr.vInertia.m_items[1];
	vehLin.B[7][3] = vehLin.B[7][4] = -cVehAttr.wheelBaseRear*ks/cVehAttr.vInertia.m_items[1];
	vehLin.B[3][5] = -1.0;
	vehLin.B[3][6] = -4.0*ks/cVehAttr.mass;

	return vehLin;
}  // End of InitReducedLinearMatrix


/////////////////////////////////////////////////////////////////////////////
//
// This function provides the minimum torque necessary to keep the
// vehicle running at constant speed, when the target acceleration  is 0.0
//
//
static double
BiasInterpol( double vel )
{
	static double lut[] = {
		19.3,		// 5 mph,   2.21 
		20.1,		// 10 mph   4.443
		21.8,		// 15 mph
		23.2,		// 20 mph   8.91
		26.1,		// 25 mph
		28.6,		// 30 mph   13.329
		32.6,
		36.6,		// 40 mph   17.772
		41.3,
		46.2,		// 50 mph   22.215
		52.1,
		58.3,		// 60 mph   26.658
		65.2,
		72.5,		// 70 mph   31.101
		80.4,
		89.0,		// 80 mph   35.544
		97.9,
		107.0		// 90 mph   39.987
	};

	const double spacing = 5.0;		// mph diff between entries
	const double base    = 5.0;		// first entry mph
	const double top     = 90.0;	// last entry mph

	double velmph = vel / 0.44703;	// lut was written using mph values

	if ( velmph <= base ) 
	{
		return 19.2;
	}
	else if ( velmph >= top || vel != vel )  //if we have a NaN, 
	{
		return 107.0;
	}
	else 
	{
		int    b0 = (int)((velmph - base) / spacing);
		double v0 = lut[b0];
		double v1 = lut[b0+1];

		return v0 + (velmph - base - b0*spacing) * (v1-v0)/spacing;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// This function provides the gain coefficients necessary to give
// a realistic looking acceleration & braking profile that also 
// matches the desired acceleration provided by behaviors.  Values were 
// initially determined for specific points experimentally and
// a simple quadratic function is used to match the shape of the
// resultant curve.
//
static void
GainInterpol(
			double vel,
			const TVehSolAttr&     cVehAttr,
			double& ffg,    // feedforward gain
			double& fbg     // feedback gain
			)
{
	double offset;
	double gain;
	double forwardGainScaler = 0.0995;
	double feedBackGain = 50.0;
	// for the feedforward gain, we use the approximating equation
	// y = 0.027 * (x-91)*(x-91), with min/max : 10, 200
	// Note: 
	//   to reduce acceleration, reduce the overall gain
	//   to make for a sharper dropoff in acceleration reduce the offset 
	
	//gain   = 0.0255;
	gain = forwardGainScaler*cVehAttr.mass+0.0226; // is .0255 for default mass
	offset   = 90.3;

	ffg = gain ;//* (vel - offset) * (vel - offset);

	// feedback is not as critical, magic number, this works fine
	fbg = feedBackGain;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Solves the vehicle's equations of motion.
//
// Remarks:  The integrator calls this routine with the current
//   vector of states, and this routine calcuates the state derivates
//   and returns them to the integrator.
//
// Arguments:
//   cved - A reference to the CVED instance.
//   cpCurState - The current value of the vehicle states
//   cpContInp - Continuous vehicle input signals
//   cpAttr - Vehicle attribues
//   numStates - Number of states in vehicle model
//   time - The current simulation time
//   state - Vehicle state vector
//   derivs - Vehicle state derivative vector
//   vehAccelLoc - Local vehicle acceleration vector
//   makeQryTerrainEfficient - Limit calls to QryTerrain
//   qryTerrainEfficientInfo - QryTerrain info to keep from one call to another.
//   qryTerrainErrCount - # of time QryTerrain failes consecutively.
//   percentBetweenControlInputUpdates - the % we have progressed between control updates
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void
FourWheelVehDyna(
			int                    cvedId,
			CCved&                 cved, 
			const TVehicleState*   cpCurState,
			TVehicleState*         pFutState,
			const TVehicleContInp* cpContInp,
			const cvTObjAttr*      cpAttr,
			const TVehSolAttr&     cVehAttr,
			int                    numStates,
			double                 delta,
			double                 time, 
			cvTFourWheelVeh&       state, 
			cvTFourWheelVeh&       derivs,
			CVector3D&             vehAccelLoc,
			CPoint3D&              vehPosVisual,
			bool                   firstIteration,
			TQryTerrainInfo&       qryTerrainInfo,
			bool                   makeQryTerrainEfficient,
			bool&                  refreshQryTerrain,
			int&                   qryTerrainErrCount,
			double&				   steerWheelPos, //<current steering wheel output
            float                  percentBetweenControlInputUpdates
			)
{
	double orientMtrx[3][3];

	//
	// Declare variables for vehicle states.
	//
	CPoint3D vehPosition;
	CPoint3D vehOrient;
	CVector3D vehVelocity;
	CVector3D vehVelLoc;
	CVector3D vehAngVel;
	CVector3D force;
	CVector3D forceLoc;
	CVector3D grade;
	double tireZPos[NUM_TIRES];
	double tireZVel[NUM_TIRES];
	double tireOmeg[NUM_TIRES];
	double yawMoment;
	double suspRoll;
	double suspPitch;
	double terrainAvgHt;
	bool   stopped = false;

	//
	// Mapping states.
	//
	vehPosition.m_x = state.state[1 ]*cFeetToMeters;
	vehPosition.m_y = state.state[2 ]*cFeetToMeters;
	vehPosition.m_z = state.state[3 ]*cFeetToMeters;
	vehVelocity.m_i = state.state[4 ];
	vehVelocity.m_j = state.state[5 ];
	vehVelocity.m_k = state.state[6 ];
	vehOrient.m_x   = state.state[7 ];
	vehOrient.m_y   = state.state[8 ];
	vehOrient.m_z   = state.state[9 ];
	vehAngVel.m_i   = state.state[10];
	vehAngVel.m_j   = state.state[11];
	vehAngVel.m_k   = state.state[12];

	int i;
	for ( i=0; i<NUM_TIRES; i++ ) {
		tireZPos[i] = state.state[12+3*i+1]*cFeetToMeters;
		tireZVel[i] = state.state[12+3*i+2];
		tireOmeg[i] = state.state[12+3*i+3];
	}

	//
	// Create orientation transformation matrix.
	//
	OrientMatrix( vehOrient.m_x, vehOrient.m_y, vehOrient.m_z, &orientMtrx );

	//
	// Assign vehicle variables with initial values
	//
	vehVelLoc  = ProdMatrixVector( &orientMtrx, vehVelocity, true );

	yawMoment    = 0.0;
	suspRoll     = 0.0;
	suspPitch    = 0.0;
	terrainAvgHt = 0.0;

	//
	// Read the control inputs.
	//
	double targetVelocity = (double) cpContInp->targVel;             // m/s
	double targetDistance = (double) cpContInp->targDist;            // m
    CPoint3D targetPoint;//  = cpContInp->targPos;                      // ft
   
    targetPoint.m_x = cpContInp->targPos.x * cFeetToMeters * percentBetweenControlInputUpdates;				 // m
	targetPoint.m_y = cpContInp->targPos.y * cFeetToMeters * percentBetweenControlInputUpdates;				 // m
	targetPoint.m_z = cpContInp->targPos.z * cFeetToMeters * percentBetweenControlInputUpdates;				 // m

    targetPoint.m_x += cpContInp->oldtargPos.x * cFeetToMeters * (1-percentBetweenControlInputUpdates);		 // m
	targetPoint.m_y += cpContInp->oldtargPos.y * cFeetToMeters * (1-percentBetweenControlInputUpdates);		 // m
	targetPoint.m_z += cpContInp->oldtargPos.z * cFeetToMeters * (1-percentBetweenControlInputUpdates);		 // m

	//
	// Limit minimum target distance so that there is no singularity.
	//
	if ( targetDistance < targetVelocity * cTargetDistLimitRel )
		targetDistance = targetVelocity * cTargetDistLimitRel;
	if ( targetDistance < cTargetDistLimitAbs ) 
		targetDistance = cTargetDistLimitAbs;

	// e
	double accelDesired = cpContInp->targAccel;
#if 0
    fprintf(stdout, "AccelDesired : %.2f \n ", accelDesired );
#endif

	double accelFeedback;
	double driveTorque;
	double AccelFeedforwardGain;
	double AccelFeedbackGain;


	// Gains are computed based on a lookup table that provides
	// fairly high, but realistic acceleration profiles
	GainInterpol(vehVelLoc.m_i, cVehAttr, AccelFeedforwardGain, AccelFeedbackGain);

	accelFeedback = AccelFeedbackGain*(accelDesired - cpCurState->acc);
	driveTorque   = AccelFeedforwardGain * accelDesired + accelFeedback;

	if ( vehVelLoc.m_i <= 0.04 && driveTorque <= 0.0 ) {
		driveTorque = 0.0;
		stopped     = true;
	}
	else {
		//
		// The bias is a torque value that ensures the vehicle
		// does not decelerate when given 0 accel as the input.
		// The BiasInterpol function performs linear interpolation
		// on a table containing experimentally determined values
		// for appropriate values of torque to maintain the vehicle
		// running at a constant speed when the acceleration is 0.
		double bias = BiasInterpol(vehVelLoc.m_i);
		driveTorque += bias;
	}

#if 0
	fprintf(stdout, "accelFeedback : %.2f driveTorque : %.2f\n ", accelFeedback, driveTorque );
#endif

	if ( 0 ) 
	{
		printf("\n ========================== DYNAMICS ==========================\n");
		printf("  %d==> feedback=%.1f,  feedforw=%.1f, drag=%.1f, %s\n",
			cvedId,	accelFeedback, AccelFeedforwardGain * accelDesired,
			cDragCoeff * vehVelLoc.m_i * vehVelLoc.m_i,
			stopped ? "STOPPED" : "MOVING");
		printf("  Torque=%8.2f accDsrd=%6.2f  CrAcc=%.5f)\n", 
			driveTorque, accelDesired, cpCurState->acc);

		printf("vehPosition: %.5E, %.5E, %.5E\n", vehPosition.m_x, vehPosition.m_y, vehPosition.m_z);
		printf("vehVelocity: %.5E, %.5E, %.5E\n", vehVelocity.m_i, vehVelocity.m_j, vehVelocity.m_k);
		printf("vehOrient: %.5E, %.5E, %.5E\n", vehOrient.m_x, vehOrient.m_y, vehOrient.m_z);
		printf("vehAngVel: %.5E, %.5E, %.5E\n", vehAngVel.m_i, vehAngVel.m_j, vehAngVel.m_k);

	}

	//
	// Limit drive torque for more realistic vehicle performance
	//
	double brakeTorqueLimiter = cVehAttr.mass / cVehAttr.brakeLimitMassTorqueRatio;
	bool limitTorque = driveTorque < -brakeTorqueLimiter;
	if ( limitTorque ) 
	{
		//printf("Limiter: was %f, now is %f\n", driveTorque, -brakeTorqueLimiter);
		driveTorque = -brakeTorqueLimiter;
	}

	//
	// Steering controller.
	//
	//double steerWheelPos = 0.0; 
	double yawDesired,yaw;
	yawDesired = atan2((targetPoint.m_y-vehPosition.m_y),
							(targetPoint.m_x-vehPosition.m_x));
	yaw = fmod(vehOrient.m_z,2*cPI);
	if( yaw > cPI + 0.1 ) yaw -= 2.0*cPI;
	if( yaw < -cPI - 0.1 ) yaw += 2.0*cPI;
	if( targetPoint.m_x < vehPosition.m_x && fabs(yaw) > cPI/2.0 ) 
	{
		if( yawDesired < 0.0 && yaw > 0.0 ) 
		{
			yawDesired += 2.0*cPI;
		}
		if( yawDesired > 0.0 && yaw < 0.0 ) 
		{
			yawDesired -= 2.0*cPI;
		}
	}

	double steerGain = 10;
	steerWheelPos = steerGain*( yawDesired - yaw -
		0.01*(1.0-exp(-vehVelLoc.m_i/5.0))*vehAngVel.m_k -
		0.5*(1.0-exp(-vehVelLoc.m_i/5.0))*atan2(vehVelLoc.m_j,vehVelLoc.m_i) 
		);
	if( vehVelLoc.m_i <= 0.5 ) steerWheelPos = 0.0;
	if( steerWheelPos >=  cSteerLimit ) 
        steerWheelPos =  cSteerLimit;
	if( steerWheelPos <= -cSteerLimit ) 
        steerWheelPos = -cSteerLimit;
    //cpContInp
    float steerRateLimit = cpContInp->maxSteerRateRadpS * (float)delta;
	steerWheelPos = RateLimit(steerWheelPos,cpCurState->steeringWheelAngle,steerRateLimit/*cSteerRateLimit*/);

#if 0
	gout << "targetPoint.m_x=" << targetPoint.m_x << "vehPosition.m_x=" << vehPosition.m_x << endl;
	gout << "targetPoint.m_y=" << targetPoint.m_y << "vehPosition.m_y=" << vehPosition.m_y << endl;
	gout << "yawerr=" << yawDesired-yaw << "yawrateerr=" << -0.01*(1.0-exp(-vehVelLoc.m_i/5.0))*vehAngVel.m_k << "slipangle=" << -0.5*(1.0-exp(-vehVelLoc.m_i/5.0))*atan2(vehVelLoc.m_j,vehVelLoc.m_i) << endl;
	gout << "vehVelLoc Long=" << vehVelLoc.m_i << "yawDesired=" << yawDesired << "yaw=" << yaw << "steerWheelPos=" << steerWheelPos << endl;
#endif
	//
	//  Map steerWheelPos to steer of each wheel using ackerman formula
	//
	double a0,a1,steerTirePos[4], ackermanAngle[2];
	a0 = ( cVehAttr.wheelTrack / 
		   ( cVehAttr.wheelBaseForw + 
			 cVehAttr.wheelBaseRear ) );
	a1 = ( cVehAttr.wheelBaseRear / 
		   ( cVehAttr.wheelBaseForw + 
			 cVehAttr.wheelBaseRear ) );
	ackermanAngle[0] = atan2( tan(steerWheelPos), 
		( a1 - 0.5 * a0 * tan(steerWheelPos) ) );
	ackermanAngle[1] = atan2( tan(steerWheelPos), 
		( a1 + 0.5 * a0 * tan(steerWheelPos) ) );
	if( steerWheelPos >= 0.0 ) 
	{
		steerTirePos[0] = ackermanAngle[0];
		steerTirePos[1] = ackermanAngle[1];
	}
	else 
	{
		steerTirePos[0] = ackermanAngle[1];
		steerTirePos[1] = ackermanAngle[0];
	}
	steerTirePos[2] = 0.0;
	steerTirePos[3] = 0.0;

	//
	// Declare variables for tires and for vehicle forces
	//
	CPoint3D tirePos;
	CVector3D suspVel;
	CVector3D terrainNormal;
	double suspDefl, suspDeflVel, suspFrc[NUM_TIRES];
	double terrainHeight, tireAvgPos;
	double tireDefl, tireDeflVel;
	double tireEffectRad, tireAvgRad, tireTorque;
	double tireNrmFrc[NUM_TIRES],tireLongFrc[NUM_TIRES],tireLatFrc[NUM_TIRES];
	double a2, a3, a4, b1, b2, b3, b4;
	double longForce[NUM_TIRES], latForce[NUM_TIRES];
	double tireZAcc[NUM_TIRES], tireOmeD[NUM_TIRES];
	double tpx[NUM_TIRES], tpy[NUM_TIRES];

	tireAvgRad = 0.0;
	tireAvgPos = 0.0;
	
	for( i=0; i<NUM_TIRES; i++ ) 
	{
		//
		// Determine global tire positions
		//
		const CPoint3D tireLocalPos = cVehAttr.tireLocPos.m_items[i];

		tirePos = ProdMatrixVector( &orientMtrx, tireLocalPos, false );
		tirePos.m_x += vehPosition.m_x;
		tirePos.m_y += vehPosition.m_y;
		tirePos.m_z += vehPosition.m_z;
		tireAvgPos = tireAvgPos + tireZPos[i]/4.0;

		tpx[i] = tirePos.m_x;
		tpy[i] = tirePos.m_y;

		//
		// Determine local suspension velocity
		//
		suspVel.m_i = -vehAngVel.m_k * tireLocalPos.m_x + 
			vehAngVel.m_j * tireLocalPos.m_z;
		suspVel.m_j = -vehAngVel.m_i * tireLocalPos.m_z + 
			vehAngVel.m_k * tireLocalPos.m_x;
		suspVel.m_k = -vehAngVel.m_i * tireLocalPos.m_y - 
			vehAngVel.m_j * tireLocalPos.m_x;
		//
		// Calculate Suspension force
		//
		suspDefl = tireZPos[i] - tirePos.m_z;
		suspDeflVel = -vehVelLoc.m_k + suspVel.m_k;
		suspFrc[i] = ( cpCurState->suspStif * suspDefl + 
					   cpCurState->suspDamp * suspDeflVel );
		//
		// Obtain terrain height and use to calculate tire normal force
		// Assumption: Tire normal force is always in the direction of
		// terrain normal
		//
		terrainHeight     = 0.0;
		terrainNormal.m_i = 0.0;
		terrainNormal.m_j = 0.0;
		terrainNormal.m_k = 1.0;

		//
		// QryTerrain can be executed in 1 of 2 modes.  In the default
		// mode, QryTerrain is executed once for every tire in the
		// first iteration of the Runge-Kutta algorithm for a total
		// of 4 executions in every invocation of the vehicle dynamics.
		// In the second (efficient) mode, QryTerrain is executed once 
		// overall.
		//
		// Mode 1 --> makeQryTerrainEfficient == false
		// Mode 2 --> makeQryTerrainEfficient == true
		//
		bool executeQryTerrain = ( firstIteration || 
								   makeQryTerrainEfficient ||
								   refreshQryTerrain
								   );
		if ( executeQryTerrain ) 
		{
			CCved::EQueryCode code;
			double tx = tirePos.m_x * cMetersToFeet;
			double ty = tirePos.m_y * cMetersToFeet;
			double tz = tirePos.m_z * cMetersToFeet;
			double tzout;

			//
			// Terrain query.
			//
			CCved::CTerQueryHint lastRLDPosition;
			lastRLDPosition.CopyFromStruct( cpCurState->posHint[i] );
			code = cved.QryTerrain( 
								tx, 
								ty, 
								tz, 
								tzout, 
								terrainNormal, 
								&lastRLDPosition 
								);
			lastRLDPosition.CopyToStruct( pFutState->posHint[i] );

			if ( code == CCved::eCV_OFF_ROAD ) 
			{
				gout << "**QryTerrain (FourWheelDyna1) returns ";
				gout << "eTOffRoad for point (";
				gout << tx << ", " << ty << ", " << tz << ")  zout = ";
				gout << tzout << "  cvedId = " << cvedId;
				gout << "  name = " << cved.GetObjName( cvedId );
				gout << endl;
				gout << "*** qryTerrainErrCount = " << qryTerrainErrCount << endl;

				// set new z to be same as old and try to continue
				tzout = tz;

				// increment the number of times that qryTerrain is off-road
				qryTerrainErrCount++;
			}
			else
			{
				qryTerrainErrCount = 0;
			}
			terrainHeight = tzout * cFeetToMeters;

			if( makeQryTerrainEfficient && refreshQryTerrain ) 
			{
				int j;
				for ( j = 0; j < NUM_TIRES; j++ ) 
				{
					qryTerrainInfo.terrainHeight[j] = terrainHeight;
					qryTerrainInfo.terrainNormal[j] = terrainNormal;
				}
				refreshQryTerrain = false;
			}
			else if( firstIteration ) 
			{
				qryTerrainInfo.terrainHeight[i] = terrainHeight;
				qryTerrainInfo.terrainNormal[i] = terrainNormal;
			}
		}
		else 
		{
			terrainHeight = qryTerrainInfo.terrainHeight[i];
			terrainNormal = qryTerrainInfo.terrainNormal[i];
		}

		terrainAvgHt += terrainHeight / 4.0;
		grade.m_i    += terrainNormal.m_i / 4.0;
		grade.m_j    += terrainNormal.m_j / 4.0;
		grade.m_k    += terrainNormal.m_k / 4.0;

		tireDefl = terrainHeight - ( tireZPos[i] - cVehAttr.tireRadius );
		tireDeflVel = -tireZVel[i];
#if 0
		fprintf(stdout,"i=%d.  tireDefl=%.2f.  terrainHeight=%.2f.  tireZPos[i]=%.2f.\n",i,tireDefl,terrainHeight,tireZPos[i]);
#endif
		if( tireDefl <= 0.0 ) 
		{
			tireDefl = 0.0;
			tireDeflVel = 0.0;
		}
		tireNrmFrc[i] = ( cpCurState->tireStif * tireDefl + 
						  cpCurState->tireDamp * tireDeflVel
						  );
		//
		// Modify tire radius. Calculate tire longitudnal and lateral force
		//
		double tireRadius = tireZPos[i] - terrainHeight;
		tireEffectRad = ( tireRadius * (1.0 + 4.0 * cVehAttr.tireInert / 
						  ( cVehAttr.mass * tireRadius * tireRadius ) )
						  );

		double tireCircumference = 2*cPI*(tireRadius*cMetersToFeet);
		double wheelRPM = ((cVehAttr.maximumSpeed*5280)/60)*(1/tireCircumference);
		double speedTorqueLimiter = (cVehAttr.horsePower*5252)/wheelRPM; 
		
		limitTorque = driveTorque > speedTorqueLimiter;
		if( limitTorque )
		{
			driveTorque = speedTorqueLimiter;
		}

		tireTorque     = driveTorque;
		tireLongFrc[i] = tireTorque / tireEffectRad;
        tireAvgRad     = tireAvgRad + tireRadius/4.0;
		
#if 0
		if(i<3)
			fprintf(stdout,"tireLongFrc[i] = %.2f ",tireLongFrc[i]);
		else
			fprintf(stdout,"tireLongFrc[i] = %.2f \n ",tireLongFrc[i]);

#endif
  		double alfa;
		if( fabs( vehVelLoc.m_i ) > cVelocityThresh ) 
		{
			alfa = ( ( vehVelLoc.m_j + suspVel.m_j ) / vehVelLoc.m_i - 
					 steerTirePos[i]
					 );
		}
		else 
		{
			alfa = ( ( vehVelLoc.m_j + suspVel.m_j ) / 
					 ( SignOf( vehVelLoc.m_i ) * cVelocityThresh * 10.0 )
					 - steerTirePos[i]
					 );
		}

		tireLatFrc[i] = ( -SignOf( alfa ) * cMU * tireNrmFrc[i] * 
						  ( 1.0 - exp( -cAlfaMult * alfa * alfa ) )
						  );
		tireZAcc[i] = ( ( tireNrmFrc[i] - suspFrc[i] ) /
						cVehAttr.tireMass - terrainNormal.m_k * cGRAVITY 
						);
		tireOmeD[i] = ( ( tireTorque - tireRadius * tireLongFrc[i] ) / 
						cVehAttr.tireInert 
						);

		//
		// Assemble the tire forces to find the forces and moments 
		//   on the chassis
		//
		a1 = tireLongFrc[i] * cos( steerTirePos[i] );
		a2 = tireLatFrc[ i] * sin( steerTirePos[i] );
		a3 = tireLongFrc[i] * sin( steerTirePos[i] );
		a4 = tireLatFrc[i]  * cos( steerTirePos[i] );
		b1 = tireLongFrc[i] * sin( steerTirePos[i] ) * tireLocalPos.m_x;
		b2 = tireLatFrc[i]  * cos( steerTirePos[i] ) * tireLocalPos.m_x;
		b3 = tireLongFrc[i] * cos( steerTirePos[i] ) * tireLocalPos.m_y;
		b4 = tireLatFrc[i]  * sin( steerTirePos[i] ) * tireLocalPos.m_y;

		longForce[i]  = a1 - a2;
		latForce[i]   = a3 + a4;
		yawMoment    += b1 + b2 - b3 + b4;
		suspPitch    += tireLocalPos.m_x * suspFrc[i];
		suspRoll     += tireLocalPos.m_y * suspFrc[i];
		forceLoc.m_i += longForce[i];
		forceLoc.m_j += latForce[i];
		forceLoc.m_k += tireNrmFrc[i];
#if 0
		fprintf(stdout,"i=%d.  alfa=%.2f.  tireLongFrc[i]=%.2f  tireLatFrc[i]=%.2f tireNrmFrc[i]=%.2f\n",i,alfa,tireLongFrc[i],tireLatFrc[i],tireNrmFrc[i]);
#endif
	}
#if 0
		fprintf(stdout,"forceLoc Long = %.2f \n ",forceLoc.m_i);
#endif

#ifdef DEBUG_VEH_STATE
	if ( cvedId == 5 && firstIteration )
	{
		printf("%.4f %.4f %.4f %.4f ", 
			terrainAvgHt, grade.m_i, grade.m_j, grade.m_k);
	}
#endif

	//
	// Modify longitudinal force by rolling resistance and
	//   aerodynamic drag.  These formulas use magic numbers.
	//
	double RollingResFrc,AeroFrc;
	if( fabs( vehVelLoc.m_i ) > 0.01 ) 
	{
		 RollingResFrc = ( ( 0.013 + 0.0000065 * vehVelLoc.m_i *
						   vehVelLoc.m_i ) * forceLoc.m_k);
	} 
	else 
	{
		 RollingResFrc = 0.0;
	}
	AeroFrc = 0.5 * 1.22570 * 2.06 * 0.360 * vehVelLoc.m_i * vehVelLoc.m_i;
	forceLoc.m_i -= ( AeroFrc + RollingResFrc );

	//
	// Calculate roll and pitch moments
	//
	double CGHeight = vehPosition.m_z - terrainAvgHt;
	double rollMoment = suspRoll;
	for( i=0; i<NUM_TIRES; i++ ) rollMoment += latForce[i] * CGHeight;

	double pitchMoment = suspPitch;
	for( i=0; i<NUM_TIRES; i++ ) pitchMoment += longForce[i] * CGHeight;

	//
	// Transform local forces into global reference frame
	//   on a possible grade
	//
	force.m_i = ( forceLoc.m_i * cos( vehOrient.m_z ) - 
				  forceLoc.m_j * sin( vehOrient.m_z )
				  );
	force.m_j = ( forceLoc.m_i * sin( vehOrient.m_z ) + 
				  forceLoc.m_j * cos( vehOrient.m_z )
				  );
	force.m_k = forceLoc.m_k ;

	CVector3D GlbForce;
	if( fabs( vehVelLoc.m_i ) > cVelocityThresh ) 
	{
		GlbForce.m_i = grade.m_k * force.m_i - grade.m_i * force.m_k;
		GlbForce.m_j = grade.m_k * force.m_j - grade.m_j * force.m_k;
	}
	else
	{
		if ( fabs(grade.m_i) < cMU ) 
		{
			GlbForce.m_i = grade.m_k * force.m_i;
		}
		else 
		{
			GlbForce.m_i = ( grade.m_k * force.m_i - 
							 ( grade.m_i - cMU ) * force.m_k
							 );
		}
		if ( fabs(grade.m_j) < cMU ) 
		{
			GlbForce.m_j = grade.m_k * force.m_j;
		}
		else 
		{
			GlbForce.m_j = ( grade.m_k * force.m_j - 
							 ( grade.m_j - cMU ) * force.m_k
							 );
		}
	}
	GlbForce.m_k = grade.m_k * force.m_k;


	/************************************************************
	*  Accelerations
	*/
	CVector3D vehAccel;
	vehAccel.m_i = GlbForce.m_i / cVehAttr.mass;
	vehAccel.m_j = GlbForce.m_j / cVehAttr.mass;
	vehAccel.m_k = GlbForce.m_k / cVehAttr.mass - cGRAVITY;

	CVector3D vehAccelLoc2;
	vehAccelLoc2.m_i = vehAccel.m_i + vehVelocity.m_j * vehAngVel.m_k;
	vehAccelLoc2.m_j = vehAccel.m_j - vehVelocity.m_i * vehAngVel.m_k;
	vehAccelLoc2.m_k = vehAccel.m_k;
	vehAccelLoc = ProdMatrixVector( &orientMtrx, vehAccelLoc2, true );

	CVector3D CrossTerm;
	CrossTerm.m_i = vehAngVel.m_j * vehAngVel.m_k * 
		( cVehAttr.vInertia.m_items[1] - 
		cVehAttr.vInertia.m_items[2] );
	CrossTerm.m_j = vehAngVel.m_i * vehAngVel.m_k * 
		( cVehAttr.vInertia.m_items[2] - 
		cVehAttr.vInertia.m_items[0] );
	CrossTerm.m_k = vehAngVel.m_i * vehAngVel.m_j * 
		( cVehAttr.vInertia.m_items[0] - 
		cVehAttr.vInertia.m_items[1] );

	CVector3D vehAngAcc;
	vehAngAcc.m_i = (rollMoment  + CrossTerm.m_i) /
		cVehAttr.vInertia.m_items[0];
	vehAngAcc.m_j = (pitchMoment + CrossTerm.m_j) /
		cVehAttr.vInertia.m_items[1];
	vehAngAcc.m_k = (yawMoment   + CrossTerm.m_k) /
		cVehAttr.vInertia.m_items[2];

	/************************************************************
	* Euler Velocity Mapping
	*/
	CVector3D vehEulerVel;
	vehEulerVel.m_k =  (vehAngVel.m_j * sin(vehOrient.m_x) 
		+ vehAngVel.m_k * cos(vehOrient.m_x))/cos(vehOrient.m_y);
	vehEulerVel.m_j =  vehAngVel.m_j * cos(vehOrient.m_x)
		- vehAngVel.m_k * sin(vehOrient.m_x);
	vehEulerVel.m_i = vehAngVel.m_i + vehEulerVel.m_k * sin(vehOrient.m_y);

	/************************************************************
	* Mapping Derivatives
	*/
	if( stopped ) 
	{
		for( i = 0; i < 13;i++ ) derivs.state[i] = 0.0;
	}
	else 
	{
		derivs.state[0 ] = 0.0;
		derivs.state[1 ] = vehVelocity.m_i*cMetersToFeet;
		derivs.state[2 ] = vehVelocity.m_j*cMetersToFeet;
		derivs.state[3 ] = vehVelocity.m_k*cMetersToFeet;
		derivs.state[4 ] = vehAccel.m_i;
		derivs.state[5 ] = vehAccel.m_j;
		derivs.state[6 ] = vehAccel.m_k;
		derivs.state[7 ] = vehEulerVel.m_i;
		derivs.state[8 ] = vehEulerVel.m_j;
		derivs.state[9 ] = vehEulerVel.m_k;
		derivs.state[10] = vehAngAcc.m_i;
		derivs.state[11] = vehAngAcc.m_j;
		derivs.state[12] = vehAngAcc.m_k;
	}

#if 0
	fprintf(stdout,"vehAccel.m_i : %.2f vehAccel.m_j : %.2f fvehAccel.m_k : %.2f\n ", vehAccel.m_i, vehAccel.m_j, vehAccel.m_k );
#endif

	for( i = 0; i < NUM_TIRES; i++ ) 
	{
		derivs.state[12+3*i+1] = tireZVel[i];
		derivs.state[12+3*i+2] = tireZAcc[i];
		derivs.state[12+3*i+3] = tireOmeD[i];
	}

	vehPosVisual.m_x = vehPosition.m_x;
	vehPosVisual.m_y = vehPosition.m_y;
	vehPosVisual.m_z = tireAvgPos - tireAvgRad;
}  // FourWheelVehDyna


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Reads dynamics-related information from the SOL.
//
// Remarks:  
//
// Arguments:
//   cved - A reference to the CVED instance.
//   cSolId - The SOL id of the vehicle object.
//   vehSolAttr - Structure that info from SOL should be written to.
//
// Returns:  Sets vehSolAttr.
//
//////////////////////////////////////////////////////////////////////////////
static void
ReadInfoFromSol( 
			CCved& cved, 
			const int cSolId, 
			TVehSolAttr& vehSolAttr 
			)
{
	//
	// Get dynamics parameters from the SOL.
	//
	const CSolObj* cpSolObj = cved.GetSol().GetObj( cSolId );
	const CSolObjVehicle* cpSolObjVeh = 
				dynamic_cast<const CSolObjVehicle*> ( cpSolObj );
	const CDynaParams& dynaParams = cpSolObjVeh->GetDynaParams();

	vehSolAttr.axleHeight = dynaParams.m_AxleHeight * cFeetToMeters;
	vehSolAttr.mass = dynaParams.m_Mass;
	vehSolAttr.tireInert = dynaParams.m_TireInert;
	vehSolAttr.tireMass = dynaParams.m_TireMass;
	vehSolAttr.tireRadius = dynaParams.m_TireRadius * cFeetToMeters;
	vehSolAttr.wheelBaseForw = dynaParams.m_WheelBaseForw * cFeetToMeters;
	vehSolAttr.wheelBaseRear = dynaParams.m_WheelBaseRear * cFeetToMeters;
	vehSolAttr.wheelTrack = dynaParams.m_WheelTrack * cFeetToMeters;
	vehSolAttr.vInertia = dynaParams.m_VInertia;
	vehSolAttr.tireLocPos = dynaParams.m_TireLocPos;
	vehSolAttr.brakeLimitMassTorqueRatio = dynaParams.m_brakeLimitMassTorqueRatio;
	vehSolAttr.horsePower = dynaParams.m_HorsePower;
	vehSolAttr.maximumSpeed = dynaParams.m_MaximumSpeed;

	// 
	// Convert elements to feet to meters.
	//
	Vec::iterator i;
	for( 
		i = vehSolAttr.tireLocPos.m_items.begin(); 
		i != vehSolAttr.tireLocPos.m_items.end(); 
		i++ 
		) 
	{
		CPoint3D pt = *i;
		(*i).m_x = pt.m_x * cFeetToMeters;
		(*i).m_y = pt.m_y * cFeetToMeters;
		(*i).m_z = pt.m_z * cFeetToMeters;
	}
}  // ReadInfoFromSol


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Initializes the vehicle dynamics.
//
// Remarks:  Use the attributes and position, orientation, velocity 
//   fields of cvTObjState::VehicleState to initialize the "state" 
//   field of cvTObjState::VehicleState.
//
// Arguments:
//   cved - A reference to the CVED instance.
//   solId - The SOL id of the vehicle object.
//   cvedId - The cved id of the current vehicle object.
//   cpCurState - The current state.
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
static void
InitVehicleDynamicModel( 
			CCved&               cved,
			const int            solId,
			int		             cvedId,
			const TVehicleState* cpCurState,
			const TVehSolAttr&   cVehAttr
			)
{

	EDynaFidelity dynaFidelity = cpCurState->dynaFidelity;

	//
	// Initialize the chassis position.  Zero out the orientation 
	// and orient velocity at first.
	//
	CPoint3D pos = cpCurState->position;
	TVehInfo chassis;
	chassis.position.m_x  = pos.m_x;
	chassis.position.m_y  = pos.m_y;
	chassis.position.m_z  = pos.m_z;
	chassis.velocity.m_i  = 0.0;
	chassis.velocity.m_j  = 0.0;
	chassis.velocity.m_k  = 0.0;
	chassis.orient.m_x    = 0.0;
	chassis.orient.m_y    = 0.0;
	chassis.orient.m_z    = 0.0;
	chassis.orientVel.m_i = 0.0;
	chassis.orientVel.m_j = 0.0;
	chassis.orientVel.m_k = 0.0;

	//
	// Converting CVED lateral and tangent vectors into roll, pitch
	// and yaw.  The yaw will be used to locate the tires with
	// respect to the chassis CG.
	//
	CVector3D lat = cpCurState->lateral;
	lat.m_i = -lat.m_i;
	lat.m_j = -lat.m_j;
	lat.m_k = -lat.m_k;

	CVector3D tng = cpCurState->tangent;
	CVector3D nrm;
	nrm.m_i = ( tng.m_j * lat.m_k ) - ( tng.m_k * lat.m_j );
	nrm.m_j = ( tng.m_k * lat.m_i ) - ( tng.m_i * lat.m_k );
	nrm.m_k = ( tng.m_i * lat.m_j ) - ( tng.m_j * lat.m_i );

	chassis.orient.m_x = atan2( lat.m_k, nrm.m_k );
	chassis.orient.m_y = asin( tng.m_k );
	chassis.orient.m_z = atan2( tng.m_j, tng.m_i );
	double roll  = chassis.orient.m_x;
	double pitch = chassis.orient.m_y;
	double yaw   = chassis.orient.m_z;

	//
	// Create orientation transformation matrix.
	//
	double orientMtrx[3][3];
	OrientMatrix( roll, pitch, yaw, &orientMtrx );

#ifdef DEBUG_DYN
	gout << "* tang = " << tng.m_i << " " << tng.m_j << " " << tng.m_k << endl;
	gout << "* latr = " << lat.m_i << " " << lat.m_j << " " << lat.m_k << endl;
	gout << "* norm = " << nrm.m_i << " " << nrm.m_j << " " << nrm.m_k << endl;
#endif
	
	//
	// Initialize the linearized vehicle model.
	//
	//
	// Use the SOL id to get access to vehicle dynamics parameters.
	//
	// **MISSING**  Test the SOL id to make sure it's valid.
	//
	const CSolObj* cpSolObj = cved.GetSol().GetObj( solId );
	g_vehLin[cvedId] = InitReducedLinearMatrix( 
								cpSolObj, 			
								cpCurState,	
								cVehAttr
								);

	//
	// Initialize the tire elevations from the terrain height.
	//
	cvTFourWheelVeh stateVector;
	stateVector.state[0] = 0.0;  // time
	CPoint3D tireLocalPos;
	CPoint3D tirePos;
	CPoint3D tmpPnt;
	CVector3D terrainNormal;
	double terrainHeight;
	double TVertPos[NUM_TIRES];

	int i;
	for( i = 0; i < NUM_TIRES; i++ ) 
	{
		tireLocalPos = cVehAttr.tireLocPos.m_items[i];
		tmpPnt = ProdMatrixVector( &orientMtrx, tireLocalPos, false );
		tirePos.m_x = chassis.position.m_x + tmpPnt.m_x;
		tirePos.m_y = chassis.position.m_y + tmpPnt.m_y;
		tirePos.m_z = chassis.position.m_z + tmpPnt.m_z;

		double tx = tirePos.m_x;
		double ty = tirePos.m_y;
		double tz = tirePos.m_z;
		double tzout;

		//
		// Terrain query.
		//
		CCved::EQueryCode code;
		code = cved.QryTerrain( tx, ty, tz, tzout, terrainNormal, NULL );

		if( code == CCved::eCV_OFF_ROAD ) 
		{
			gout << "**QryTerrain (InitDyn) returns eTOffRoad for point (";
			gout << tx << ", " << ty << ", " << tz << ")   zout = " << tzout;
			gout << endl;

			tzout = 0.0;
		}

		terrainHeight = tzout*cFeetToMeters;

		TVertPos[i]   = terrainHeight + cVehAttr.tireRadius;

		// the tires' states
		stateVector.state[12+3*i+1] = TVertPos[i]*cMetersToFeet;

#ifdef DEBUG_DYN
//		gout << "* tnorm = " << terrainNormal.m_i << " " << terrainNormal.m_j;
//		gout << " " << terrainNormal.m_k << endl;
		gout << "terrainHeight = " << terrainHeight << endl;
		gout << "TVertPos = " << TVertPos[i] << endl;
		gout << "tireLocalPos = " << tireLocalPos.m_x << " ";
		gout << tireLocalPos.m_y << " " << tireLocalPos.m_z << endl;
		//gout << "vel = " << vehVelLoc.m_j << endl;
#endif

	}

	//
	// Now that the tire elevations are known, reinitialize the vehicle 
	// position from tire positions and suspension characteristics.
	//
	double ks = cpCurState->suspStif;

	chassis.position.m_z = ( ( TVertPos[0] + TVertPos[1] + TVertPos[2] + 
							   TVertPos[3] - cVehAttr.mass * cGRAVITY / ks ) / 
							 4.0 - tireLocalPos.m_z
							 )*cMetersToFeet;
	chassis.orient.m_x = ( -0.5 * 
						   ( TVertPos[0] - TVertPos[1] + 
						     TVertPos[2] - TVertPos[3] 
							 ) / 
						   cVehAttr.wheelTrack
						   );
	chassis.orient.m_y = ( ( TVertPos[0] + TVertPos[1] - 
							 TVertPos[2] - TVertPos[3] 
							 ) * 
						   ( cVehAttr.wheelBaseForw + cVehAttr.wheelBaseRear ) + 
						   ( cVehAttr.mass * cGRAVITY / ks + 4.0 * tireLocalPos.m_z ) *
						   ( cVehAttr.wheelBaseForw - cVehAttr.wheelBaseRear ) 
						   ) / 
						 ( 4.0 * 
						   ( cVehAttr.wheelBaseForw * cVehAttr.wheelBaseForw + 
						     cVehAttr.wheelBaseRear * cVehAttr.wheelBaseRear 
							 ) 
						   );

	//
	// update the roll and pitch and recalculate the orientation matrix
	//
	roll  = chassis.orient.m_x;
	pitch = chassis.orient.m_y;
	OrientMatrix( roll, pitch, yaw, &orientMtrx );

	//
	// Initialize the initial velocity of the vehichle and transform
	// to the global reference frame
	//
	CVector3D vehVelLoc;
	vehVelLoc.m_i = cpCurState->vel;
	vehVelLoc.m_j = 0.0;
	vehVelLoc.m_k = 0.0;
	chassis.velocity = ProdMatrixVector( &orientMtrx, vehVelLoc, false );

	//
	// Assign chassis states to the vehicle state vector.  The
	// chassis has 12 states.
	//
	stateVector.state[1] = chassis.position.m_x;
	stateVector.state[2] = chassis.position.m_y;
	stateVector.state[3] = chassis.position.m_z;

	stateVector.state[4] = chassis.velocity.m_i;
	stateVector.state[5] = chassis.velocity.m_j;
	stateVector.state[6] = chassis.velocity.m_k;
	stateVector.state[7]  = chassis.orient.m_x;
	stateVector.state[8]  = chassis.orient.m_y;
	stateVector.state[9]  = chassis.orient.m_z;
	stateVector.state[10] = chassis.orientVel.m_i;
	stateVector.state[11] = chassis.orientVel.m_j;
	stateVector.state[12] = chassis.orientVel.m_k;

#ifdef DEBUG_DYN
	gout << "[" << cvedId << "] initial starting pos = (" << stateVector.state[1];
	gout << ", " << stateVector.state[2] << ", " << stateVector.state[3];
	gout << ")" << endl;
#endif

	//
	// Initializing tire states: velocity and rotational velocity.
	//
	for ( i = 0; i < NUM_TIRES; i++ ) {
		// get the current tire's position
		tireLocalPos = cVehAttr.tireLocPos.m_items[i];
		CVector3D tireLocalPosVec;
		tireLocalPosVec.m_i = tireLocalPos.m_x;
		tireLocalPosVec.m_j = tireLocalPos.m_y;
		tireLocalPosVec.m_k = tireLocalPos.m_z;

		CVector3D lTVel, tmpVec;
		lTVel = chassis.orientVel.CrossP( tireLocalPosVec );
		tmpVec = ProdMatrixVector( &orientMtrx, lTVel, false );
		double TVertVel = chassis.velocity.m_k + tmpVec.m_k;
		double Omega = vehVelLoc.m_i / cVehAttr.tireRadius;

		// the tires' states
		stateVector.state[12+3*i+2] = TVertVel;
		stateVector.state[12+3*i+3] = Omega;
	}

	//
	// Update information to CVED.
	//
	g_stateVector[cvedId] = stateVector;

}  // InitVehicleDynamicModel


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Executes the main loop for vehicle simulation
//
// Remarks:  This routine calls the vehicle equations of motion and
//   numerically integrates the state derivatives to obtain the states for
//   the next time step.  A Runga Kutta integration method is employed.
//
// Arguments:
//   cved - A reference to the CVED instance.
//   delta - Step size
//   cpAttr - Vehicle attributes
//   cpCurState - Current values of VehicleStates (not for integration)
//   cpContInp - Continuous inputs to vehicle
//   pFutState - values to modify VehicleStates
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
void
DoVehicleDynamics(
			int                    cvedId,
			CCved&                 cved,	
			double                 delta,
			const cvTObjAttr*      cpAttr,
			const TVehicleState*   cpCurState,
			const TVehicleContInp* cpContInp,
			TVehicleState*         pFutState,
            bool                   updateSteering
			)
{

#ifdef DEBUG_DYN
	gout << "===== Dynamics ===( delta=" << delta << " )===" << endl;
#endif

	//
	// Read information for this object type from the SOL.
	//
	TVehSolAttr vehSolAttr;
	ReadInfoFromSol( cved, cpAttr->solId, vehSolAttr );

	//
	// Does the dynamics need to initialize itself?
	//
//	if( cpContInp->initDynamics > 0 ) 
    bool firstRun = false;

	if( !cpCurState->dynaInitComplete ) 
	{
		InitVehicleDynamicModel( 
					cved, 
					cpAttr->solId, 
					cvedId, 
					cpCurState,
					vehSolAttr
					);
        firstRun= true;
	}

	//
	// Using the 4-step Runge-Kutta method to solve the equations of motion.
	// 
	cvTFourWheelVeh k1;   // names taken from the original equation
	cvTFourWheelVeh k2;
	cvTFourWheelVeh k3;
	cvTFourWheelVeh k4;

	cvTFourWheelVeh initState = g_stateVector[cvedId];

    updateSteering = true;
	//
	// Calculate distance to the OwnVehicle.  If I'm too far from the
	// OwnVehicle then call QryTerrain only 1 each execution of dynamics.
	//
	bool makeQryTerrainEfficient = false;
	bool refreshQryTerrain = false;
	if( cved.IsObjValid( 0 ) ) 
	{
		CPoint3D driverPos = cved.GetObjPosInstant( 0 );
		CPoint3D myPos;
		myPos.m_x = initState.state[1];
		myPos.m_y = initState.state[2];
		myPos.m_z = initState.state[3];

		const double cMAX_DRIVER_VIEW_DIST = 600.0;    // ft
		double distToDriverSq = myPos.DistSq( driverPos );

		if( distToDriverSq > cMAX_DRIVER_VIEW_DIST * cMAX_DRIVER_VIEW_DIST ) 
		{
			refreshQryTerrain = true;
			makeQryTerrainEfficient = true;
		}
	}

	TQryTerrainInfo qryTerrainInfo;

    CVector3D vehAccelLoc;
    CPoint3D vehPosVisual;
	int qryTerrainErrCount = cpCurState->qryTerrainErrCount;

	// current steering
	double steeringWheelPos = cpCurState->steeringWheelAngle;
	
	// 
	// First iteration.
	//
	
#ifdef DEBUG_VEH_STATE

	if ( cvedId == 5 )
		printf("%.4f %.4f %.4f 999 ", 
			cpCurState->vel, 
			pFutState->vel,
			cpContInp->targAccel
//			cpContInp->targVel,
//			cpContInp->targSteer,
//			cpContInp->targDist
			);

#endif
    float evenFrameBase = updateSteering ? 0.5f:0;
	FourWheelVehDyna(
				cvedId,
				cved, 
				cpCurState, 
				pFutState,
				cpContInp, 
				cpAttr, 
				vehSolAttr,
				NUM_STATES_4_WHEEL_VEH,
				delta,
				initState.state[0], 
				initState,
				k1, 
				vehAccelLoc,
				vehPosVisual,
				true,
				qryTerrainInfo,
				makeQryTerrainEfficient,
				refreshQryTerrain,
				qryTerrainErrCount,
				steeringWheelPos,
                evenFrameBase + 0.125f
				);
	
	// 
	// Second iteration.
	//
	initState.state[0] += 0.5 * delta;
	
	int j;
	cvTFourWheelVeh tempState;  // keeps track of intermediate states
	for( j = 0; j < NUM_STATES_4_WHEEL_VEH; j++ ) 
	{
		 tempState.state[j] = initState.state[j] + 0.5 * delta * k1.state[j];
	}
	FourWheelVehDyna(
				cvedId,
				cved, 
				cpCurState, 
				pFutState,
				cpContInp, 
				cpAttr, 
				vehSolAttr,
				NUM_STATES_4_WHEEL_VEH,
				delta,
				initState.state[0], 
				tempState, 
				k2, 
				vehAccelLoc,
				vehPosVisual,
				false,
				qryTerrainInfo,
				makeQryTerrainEfficient,
				refreshQryTerrain,
				qryTerrainErrCount,
				steeringWheelPos,
                evenFrameBase + 0.25f
				);

	// 
	// Third iteration.
	//
	for( j = 0; j < NUM_STATES_4_WHEEL_VEH; j++ ) 
	{
		tempState.state[j] = initState.state[j] + 0.5 * delta * k2.state[j];
	}
	FourWheelVehDyna(
				cvedId,
				cved, 
				cpCurState, 
				pFutState,
				cpContInp, 
				cpAttr, 
				vehSolAttr,
				NUM_STATES_4_WHEEL_VEH,
				delta,
				initState.state[0], 
				tempState, 
				k3, 
				vehAccelLoc,
				vehPosVisual,
				false,
				qryTerrainInfo,
				makeQryTerrainEfficient,
				refreshQryTerrain,
				qryTerrainErrCount,
				steeringWheelPos,
                evenFrameBase + 0.375f
				);

	// 
	// Fourth iteration.
	//
	initState.state[0] += 0.5 * delta;
	for( j = 0; j < NUM_STATES_4_WHEEL_VEH; j++ ) 
	{
		 tempState.state[j] = initState.state[j] + delta * k3.state[j];
	}
	FourWheelVehDyna(
				cvedId,
				cved, 
				cpCurState, 
				pFutState,
				cpContInp, 
				cpAttr, 
				vehSolAttr,
				NUM_STATES_4_WHEEL_VEH,
				delta,
				initState.state[0], 
				tempState, 
				k4, 
				vehAccelLoc,
				vehPosVisual,
				false,
				qryTerrainInfo,
				makeQryTerrainEfficient,
				refreshQryTerrain,
				qryTerrainErrCount,
				steeringWheelPos,
                evenFrameBase + 0.5f
				);

	//
	// Assemble intermediate results to calculate the next state.
	//
	for( j = 0; j < NUM_STATES_4_WHEEL_VEH; j++ ) 
	{
		initState.state[j] = ( initState.state[j] + delta * 
							 ( k1.state[j] + 2.0 * 
							   ( k2.state[j] + k3.state[j] ) + 
							   k4.state[j] ) / 6.0 );
	}

	//
	// Copy init state to future state.
	//
	g_stateVector[cvedId] = initState;

	CVector3D vehVelocity;
	CVector3D vehEulerVel;
	double orientMtrx[3][3];

		//
		// Create orientation transformation matrix.
		//
		OrientMatrix( 
					initState.state[7], 
					initState.state[8], 
					initState.state[9],
					&orientMtrx
					);


	pFutState->position.x = initState.state[1];
	pFutState->position.y = initState.state[2];
//	pFutState->position.z = initState.state[3];
	pFutState->position.z = vehPosVisual.m_z * cMetersToFeet;
	double roll  = initState.state[7];
	double pitch = initState.state[8];
	double yaw   = initState.state[9];

	pFutState->tangent.i = cos(yaw) * cos(pitch);
	pFutState->tangent.j = sin(yaw) * cos(pitch);
	pFutState->tangent.k = sin(pitch);
	pFutState->lateral.i = -( -sin(yaw) * cos(roll) - 
							  cos(yaw) * sin(pitch) * sin(roll)
							  );
	pFutState->lateral.j = -( cos(yaw) * cos(roll) - 
							  sin(yaw) * sin(pitch) * sin(roll)
							  );
	pFutState->lateral.k = -( cos(pitch) * sin(roll) );
	vehVelocity.m_i = initState.state[4];
	vehVelocity.m_j = initState.state[5];
	vehVelocity.m_k = initState.state[6];
    vehEulerVel.m_i = initState.state[10];
    vehEulerVel.m_j = initState.state[11];
	vehEulerVel.m_k = initState.state[12];
	CVector3D vehVelLoc = ProdMatrixVector( &orientMtrx, vehVelocity, true );
	pFutState->vel = vehVelLoc.m_i;
	pFutState->velLat = vehVelLoc.m_j;
	pFutState->velNorm = vehVelLoc.m_k;
	pFutState->acc = vehAccelLoc.m_i;
	pFutState->latAccel = vehAccelLoc.m_j;
	pFutState->qryTerrainErrCount = qryTerrainErrCount;
	pFutState->rollRate = vehEulerVel.m_i;
	pFutState->pitchRate = vehEulerVel.m_j;
	pFutState->yawRate = vehEulerVel.m_k;
	pFutState->steeringWheelAngle = steeringWheelPos;
	pFutState->dynaInitComplete = 1;

	float tireCircumference;
	tireCircumference = float(2*cPI*vehSolAttr.tireRadius);
	  
	float tireDist;
	tireDist =(float)( sqrt(pow((cpCurState->position.x - pFutState->position.x),2) + 
		              pow((cpCurState->position.y - pFutState->position.y),2)));

	float frontLeftTireRot = cpCurState->tireRot[0];
	float frontRightTireRot = cpCurState->tireRot[1];
	float rearTireRot = cpCurState->tireRot[2];

	frontLeftTireRot = fmod((((tireDist/tireCircumference)*360) + pFutState->tireRot[0]),360);
	frontRightTireRot = fmod((((tireDist/tireCircumference)*360) + pFutState->tireRot[1]),360);
	rearTireRot = fmod((((tireDist/tireCircumference)*360) + pFutState->tireRot[2]),360);

	pFutState->tireRot[0] = frontLeftTireRot;
	pFutState->tireRot[1] = frontRightTireRot;
	pFutState->tireRot[2] = rearTireRot;

}  // DoVehicleDynamics
