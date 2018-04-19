/////////////////////////////////////////////////////////////////////////////
//
// Program to test and help build the automatic vehicle capability tables
//
//
#include "pch.h"
using namespace CVED;

CCved m_Cved;

// options
string    m_LriName;
double    m_DeltaT;
string    m_SolName;

const double cNEAR_ZERO = 0.0001;
const double cMPH_TO_MS = 0.44703;    // meters/sec
const double cMS_TO_MPH = 2.23700;    // mph
const double cGRAVITY = 9.80665;      // meters/sec^2
const double cMETER_TO_FEET = 3.2808; // feet 
const double cFEET_TO_METER = 0.3048; // meters
const double cRAD_TO_DEG = 57.2957795;
const double cDEG_TO_RAD = 0.0174533;


/////////////////////////////////////////////////////////////////////////////
//
// Class that represents a target speed goal
//
//
class CSpeedGoal {
public:
	CSpeedGoal() : targSpeed(0), duration(0), urgency(0.0f) {};
	CSpeedGoal(float ts, int fr, float u) 
		: targSpeed(ts), duration(fr), urgency(u) {};
	~CSpeedGoal() {};

	float targSpeed;
	int   duration;
	float urgency;
};


static void
Usage(const string &name)
{
	fprintf(stderr, "Usage: %s lri_file [options] \n", name.c_str());
	fprintf(stderr, "Options can be\n"
"-SpeedMaint   vel dur [vel dur ...]\n"
"    vel dur urg - target velocity, how long to keep it fixed, urgency\n"
"    NOTE: speeds are in miles/hour, duration in seconds\n"
"-ZeroAccel    vel  zeroDur\n"
"    Reach specified speed, then feed 0 acceleration for zeroDur seconds\n"
"    vel = speed to reach\n"
);

	exit(-1);
}

void
InitializeDynamicsVars( 
		const cvEObjType objType,
		const slObj* pSolObj, 
		CVehicleObj* pVehicleObj 
		)
{
	//
	// Get a reference to a SOL object.
	//
	switch ( objType ) {

	case eCV_BUS :		{
			const slBusesObj* pSolVeh = dynamic_cast<const slBusesObj*> ( pSolObj );
			if ( !pSolVeh ) {

				cerr << "InitializeDynamicVars: unable to get reference ";
				cerr << "to SOL object of category = ";
				cerr << pSolObj->GetCategoryName() << "...[SUICIDE]" << endl;
				return;
			}

			double suspStif = ( pSolVeh->GetDynaSuspStifMin() + 
								pSolVeh->GetDynaSuspStifMax() ) / 2;
			pVehicleObj->SetSuspStif( suspStif );
			pVehicleObj->SetSuspStifImm( suspStif );
			double suspDamp = ( pSolVeh->GetDynaSuspDampMin() + 
								pSolVeh->GetDynaSuspDampMax() ) / 2;
			pVehicleObj->SetSuspDamp( suspDamp );
			pVehicleObj->SetSuspDampImm( suspDamp );

			double tireStif = ( pSolVeh->GetDynaTireStifMin() + 
								pSolVeh->GetDynaTireStifMax() ) / 2;
			pVehicleObj->SetTireStif( tireStif );
			pVehicleObj->SetTireStifImm( tireStif );
			double tireDamp = ( pSolVeh->GetDynaTireDampMin() + 
								pSolVeh->GetDynaTireDampMax() ) / 2;
			pVehicleObj->SetTireDamp( tireDamp );
			pVehicleObj->SetTireDampImm( tireDamp );
			pVehicleObj->SetDynaFidelity( eNON_LINEAR );
			pVehicleObj->SetDynaFidelityImm( eNON_LINEAR );
			pVehicleObj->SetQryTerrainErrCount( 0 );
			pVehicleObj->SetQryTerrainErrCountImm( 0 );

		}

		break;


	case eCV_EMERGENCY :
		{
			const slEmergVehObj* pSolVeh = dynamic_cast<const slEmergVehObj*> ( pSolObj );
			if ( !pSolVeh ) {

				cerr << "InitializeDynamicVars: unable to get reference ";
				cerr << "to SOL object of category = ";
				return;

			}

			double suspStif = ( pSolVeh->GetDynaSuspStifMin() + 
								pSolVeh->GetDynaSuspStifMax() ) / 2;
			pVehicleObj->SetSuspStif( suspStif );
			pVehicleObj->SetSuspStifImm( suspStif );
			double suspDamp = ( pSolVeh->GetDynaSuspDampMin() + 
								pSolVeh->GetDynaSuspDampMax() ) / 2;
			pVehicleObj->SetSuspDamp( suspDamp );
			pVehicleObj->SetSuspDampImm( suspDamp );

			double tireStif = ( pSolVeh->GetDynaTireStifMin() + 
								pSolVeh->GetDynaTireStifMax() ) / 2;
			pVehicleObj->SetTireStif( tireStif );
			pVehicleObj->SetTireStifImm( tireStif );
			double tireDamp = ( pSolVeh->GetDynaTireDampMin() + 
								pSolVeh->GetDynaTireDampMax() ) / 2;
			pVehicleObj->SetTireDamp( tireDamp );
			pVehicleObj->SetTireDampImm( tireDamp );
			pVehicleObj->SetDynaFidelity( eNON_LINEAR );
			pVehicleObj->SetDynaFidelityImm( eNON_LINEAR );
			pVehicleObj->SetQryTerrainErrCount( 0 );
			pVehicleObj->SetQryTerrainErrCountImm( 0 );

		}

		break;

	case eCV_TRUCK :
		{
			const slTruckObj* pSolVeh = dynamic_cast<const slTruckObj*> ( pSolObj );
			if ( !pSolVeh ) {

				cerr << "InitializeDynamicVars: unable to get reference ";
				cerr << "to SOL object of category = ";
				return;

			}

			double suspStif = ( pSolVeh->GetDynaSuspStifMin() + 
								pSolVeh->GetDynaSuspStifMax() ) / 2;
			pVehicleObj->SetSuspStif( suspStif );
			pVehicleObj->SetSuspStifImm( suspStif );
			double suspDamp = ( pSolVeh->GetDynaSuspDampMin() + 
								pSolVeh->GetDynaSuspDampMax() ) / 2;
			pVehicleObj->SetSuspDamp( suspDamp );
			pVehicleObj->SetSuspDampImm( suspDamp );

			double tireStif = ( pSolVeh->GetDynaTireStifMin() + 
								pSolVeh->GetDynaTireStifMax() ) / 2;
			pVehicleObj->SetTireStif( tireStif );
			pVehicleObj->SetTireStifImm( tireStif );
			double tireDamp = ( pSolVeh->GetDynaTireDampMin() + 
								pSolVeh->GetDynaTireDampMax() ) / 2;
			pVehicleObj->SetTireDamp( tireDamp );
			pVehicleObj->SetTireDampImm( tireDamp );
			pVehicleObj->SetDynaFidelity( eNON_LINEAR );
			pVehicleObj->SetDynaFidelityImm( eNON_LINEAR );
			pVehicleObj->SetQryTerrainErrCount( 0 );
			pVehicleObj->SetQryTerrainErrCountImm( 0 );

		}

		break;

	case eCV_EXTERNAL_DRIVER:
	case eCV_VEHICLE :
		{
			const slVehicleObj* pSolVeh = dynamic_cast<const slVehicleObj*> ( pSolObj );
			if ( !pSolVeh ) {

				cerr << "InitializeDynamicVars: unable to get reference ";
				cerr << "to SOL object of category = ";
				return;

			}

			double suspStif = ( pSolVeh->GetDynaSuspStifMin() + 
								pSolVeh->GetDynaSuspStifMax() ) / 2;
			pVehicleObj->SetSuspStif( suspStif );
			pVehicleObj->SetSuspStifImm( suspStif );
			double suspDamp = ( pSolVeh->GetDynaSuspDampMin() + 
								pSolVeh->GetDynaSuspDampMax() ) / 2;
			pVehicleObj->SetSuspDamp( suspDamp );
			pVehicleObj->SetSuspDampImm( suspDamp );

			double tireStif = ( pSolVeh->GetDynaTireStifMin() + 
								pSolVeh->GetDynaTireStifMax() ) / 2;
			pVehicleObj->SetTireStif( tireStif );
			pVehicleObj->SetTireStifImm( tireStif );
			double tireDamp = ( pSolVeh->GetDynaTireDampMin() + 
								pSolVeh->GetDynaTireDampMax() ) / 2;
			pVehicleObj->SetTireDamp( tireDamp );
			pVehicleObj->SetTireDampImm( tireDamp );
			pVehicleObj->SetDynaFidelity( eNON_LINEAR );
			pVehicleObj->SetDynaFidelityImm( eNON_LINEAR );
			pVehicleObj->SetQryTerrainErrCount( 0 );
			pVehicleObj->SetQryTerrainErrCountImm( 0 );

		}

		break;

	default :

		cerr << "InitializeDynamicVars: objects of type ";
		cerr << cvObjType2String( objType ) << " not supported yet..[SUICIDE]";
		cerr << endl;

		return;
	}

}  // end of InitializeDynamicsVars


double
SpeedMaintenanceController(
			const double currVel,
			const double prevVel,
			const double targVel,
			const double urgency
			)
{
	// PID parameters; kP is modulated by aggressiveness
	const float cKP0 = 2.4f;		// Kp for aggressiveness 0.0
	const float cKP1 = 5.5f;		// Kp for aggressiveness 1.0

	float velMph = targVel / cMPH_TO_MS;	// convert to mi/hr

	float kp = cKP0 + urgency * (cKP1-cKP0);
	float ki = -500.0f;
	float kd = 0.0f;

#if 0
	// the bias is a control  input added to the output of the controller
	// to compensate for non-linearities, in this case, aero drag
	float bias;

	// this table provides the appropriate bias control at each
	// target speed;
	static float biasTable[] = {
		0.05f,	// 20 mph
		0.1f,	// 30 mph
		0.35f,	// 40 mph
		0.55f,	// 50 mph
		1.5f,	// 60 mph
		2.1f,	// 70 mph
		2.7f	// 80 mph
	};

	//
	// compute appropriate bias using linear interpolation within the table
	// Note: the table was created manually through trial & error, thus 
	// the mi/hr units
	//
	float biasVel = targVel / cMPH_TO_MS;	// convert to mi/hr
	int   index;	// which table entry
	float ofset;	// linear interpolation factor

	if ( biasVel <= 20.0f ) {	// 20 is the lookup table low limit 
		index = 0;
		ofset = 0.0f;
	}
	else if ( biasVel >= 80.0 ) { // 80 mph is the lookup table upper limit
		index = 6;
		ofset = 0.0f;
	}
	else {
		index = (biasVel - 20.0f)/10.0f;
		ofset = (biasVel - 20.0f - index * 10.0f) / 10.0;
	}
	bias = biasTable[index] + ofset * (biasTable[index+1]-biasTable[index]);
#endif

	// speed dependent acceleration clipping; helps stabilize
	// the controller and clips too high accelerations when
	// the controller first takes over.
	// Modulated by urgency
	const  float cABS_MAX_ACCEL_10MPH_0 = 1.5f;	// aggr = 0.0
	const  float cABS_MAX_ACCEL_80MPH_0 = 2.1f;	// aggr = 0.0
	const  float cABS_MAX_ACCEL_10MPH_1 = 2.5f;	// aggr = 1.0
	const  float cABS_MAX_ACCEL_80MPH_1 = 3.3f;	// aggr = 1.0

	float maxAccel;
	float maxAt10, maxAt80;
	
	maxAt10 = cABS_MAX_ACCEL_10MPH_0 + 
		urgency * (cABS_MAX_ACCEL_10MPH_1-cABS_MAX_ACCEL_10MPH_0);
	maxAt80 = cABS_MAX_ACCEL_80MPH_0 + 
		urgency * (cABS_MAX_ACCEL_80MPH_1-cABS_MAX_ACCEL_80MPH_0);

	if ( velMph <= 10.0f ) {
		maxAccel = maxAt10;
	}
	else if ( velMph >= 80.0f ) {
		maxAccel = maxAt80;
	}
	else {
		maxAccel = maxAt10 + ((velMph-10.0f)/70.0f) * 
			(maxAt80 - maxAt10);
	}

	double velDiff = targVel - currVel;
	double time    = 1.0 / 30.0;
	double errDist = (currVel - prevVel) * time;
	double errAcc  = (currVel - prevVel) / time;
	double accel   = kp * velDiff + ki * errDist + kd * errAcc;

	// clip acceleration; when slowing down, do it slower
	if ( accel > maxAccel ) accel = maxAccel;
	if ( accel < -maxAccel ) accel = -maxAccel * 0.5;
	
#if 0
	printf("SpdMntCtrl: crVel=%6.3f, prevVel=%6.3f, trgVel=%6.3f, aggr=%3.1f, out=%7.3f\n",
	   currVel, prevVel, targVel, urgency, accel);
	printf("err(kp)=%7.4f, errInt(ki)=%7.4f, errDer(kd)=%7.4f\n",
		velDiff, errDist, errAcc);
	printf("kpTerm=%7.3f   KiTerm=%7.3f   KdTerm=%7.3f\n",
		kp* velDiff, ki*errDist, kd * errAcc);
	printf("biasVel = %.2f, index = %d, ofset = %.2f\n\n", biasVel, index, ofset);

	FILE* pOut;
	if ( firstime ) {
		pOut = fopen("output.txt", "w");
	}
	else 
		pOut = fopen("output.txt", "a+");
	fprintf(pOut, "%.3f %.3f \n", currVel, targVel);
	fclose(pOut);

	if ( firstime ) firstime = false;
#endif

	return accel;
}  // end of SpeedMaintenanceController



void
TestSpeedMaintenance(CVehicleObj* pObj, vector<CSpeedGoal>& goal)
{
	CPoint3D   initPos, curPos, targPos;
	bool       goalReached;
	initPos     = curPos = pObj->GetPosImm();
	goalReached = false;
	double      oldVel = 0.0f;
	double      accel;

	bool done    = false;
	int  frames  = 0;
	int  goalInd = 0;

	while ( !done ) {
		frames++;

// establish goals
		targPos = curPos;
		targPos.m_x += 30.0;
		pObj->SetTargPos(targPos);

		accel = SpeedMaintenanceController(pObj->GetVelImm(),
						oldVel,
						goal[goalInd].targSpeed,
						goal[goalInd].urgency);

		pObj->SetTargAccel(accel);

		oldVel = pObj->GetVelImm();
		pObj->SetInitDynamics(frames == 1);

// execute simulation
		m_Cved.Maintainer();
		m_Cved.ExecuteDynamicModels();
		m_Cved.ExecuteDynamicModels();

// log data
		curPos = pObj->GetPosImm();

		printf("%7.2f %7.2f %7.2f\n",
				accel, 
				cMS_TO_MPH*pObj->GetVelImm(), 
				cMS_TO_MPH*goal[goalInd].targSpeed); 

		goal[goalInd].duration--;
		if ( goal[goalInd].duration == 0 ) {
			goalInd++;
			if ( goalInd == goal.size() )
				done = true;
		}
	}
}


void
TestZeroAccelSpeed(CVehicleObj* pObj, double speed, int zeroAccelDur)
{
	CPoint3D   curPos, targPos;
	double     oldVel = 0.0f;
	double     accel;

	bool done         = false;
	int  withinCount  = 0;
	bool reachedVel   = false;
	int  frames       = 0;

	curPos = pObj->GetPosImm();
	while ( zeroAccelDur ) {
		frames++;

		reachedVel = withinCount > 60;

		targPos = curPos;
		targPos.m_x += 30.0;
		pObj->SetTargPos(targPos);

		if ( !reachedVel ) {
			accel = SpeedMaintenanceController(pObj->GetVelImm(),
						oldVel,
						speed,
						0.5);
		}
		else {
			accel = 0.0;
			zeroAccelDur--;
		}

		pObj->SetTargAccel(accel);

		oldVel = pObj->GetVelImm();
		pObj->SetInitDynamics(frames == 1);

// execute simulation
		m_Cved.Maintainer();
		m_Cved.ExecuteDynamicModels();
		m_Cved.ExecuteDynamicModels();

// log data
		curPos = pObj->GetPosImm();

		printf("%7.2f %7.2f %7.2f\n",
				accel, 
				cMS_TO_MPH*pObj->GetVelImm(), 
				cMS_TO_MPH*speed); 

		if ( fabs(pObj->GetVelImm()-speed) < 0.2 && !reachedVel ) {
			withinCount++;
		}
		else 
		if ( !reachedVel ) {
			withinCount = 0;
		}
		if ( reachedVel ) 
			zeroAccelDur--;

//		printf("withinCount=%d, reached=%d, zeroAccDur=%d\n",
//			withinCount, reachedVel, zeroAccelDur);
	}

}

/////////////////////////////////////////////////////////////////////////////
//
// A function that forces a model to accelerate using maximum
// possible acceleration
//
void
MaxAcceleration(CVehicleObj* pObj)
{
	CPoint3D   initPos, curPos, targPos;
	bool       goalReached;
	int        frames;
	double     targSpeed = 30.0;
	initPos     = curPos = pObj->GetPos();
	goalReached = false;
	frames      = 0;
	FILE*       pF = stdout;


	while ( !goalReached ) {
		frames++;

// establish goals
		targPos = curPos;
		targPos.m_x += 30.0;

		pObj->SetTargPos(targPos);
		pObj->SetTargAccel(2.902968);
		pObj->SetInitDynamics(frames == 1);

// execute simulation
		m_Cved.Maintainer();
		m_Cved.ExecuteDynamicModels();
		m_Cved.ExecuteDynamicModels();

// log data
		if ( pF ) {
			fprintf(pF, "%.2f  %.2f  %.1f\n",
				frames * m_DeltaT, pObj->GetVel(), curPos.m_x - initPos.m_x); 
		}

// evaluate current situation
		curPos = pObj->GetPos();

		goalReached = pObj->GetVel() > targSpeed;
	}

	printf("Acceleration from zero report\n");
	printf("0 to %.2f m/sec in %.1f sec, travel distance %.1f\n\n", 
		targSpeed, frames * m_DeltaT, curPos.m_x - initPos.m_x);
}


/////////////////////////////////////////////////////////////////////////////
//
// A function that forces a model to accelerate  
// 
//
void
Acceleration(CVehicleObj* pObj, double targSpeed, double acc, FILE *pF = 0)
{
	CPoint3D   initPos, curPos, targPos;
	bool       goalReached;
	int        frames;

	initPos     = curPos = pObj->GetPos();
	goalReached = false;
	frames      = 0;

	while ( !goalReached ) {
		frames++;

// establish goals
		targPos = curPos;
		targPos.m_x += 10.0;
		pObj->SetTargPos(targPos);
		pObj->SetTargDist(5.0);

		double v0 = pObj->GetVel();
		double tvel = 1.0 + v0 * (1.0 + 2.0 * acc);

		pObj->SetTargVel(tvel);
		pObj->SetInitDynamics(frames == 1);

		//// PROBLEM: Here, after two executions of the
		// dynamics, the velocity appears to be getting smaller
		// as opposed to larger
// execute simulation
		m_Cved.Maintainer();
		m_Cved.ExecuteDynamicModels();

		v0 = pObj->GetVel();
		v0 = pObj->GetVelImm();

		m_Cved.ExecuteDynamicModels();
		v0 = pObj->GetVel();
		v0 = pObj->GetVelImm();

// log data
		if ( pF ) {
			fprintf(pF, "%.2f  %.2f  %.1f\n",
				frames * m_DeltaT, pObj->GetVel(), curPos.m_x - initPos.m_x); 
		}

// evaluate current situation
		curPos = pObj->GetPos();

		goalReached = pObj->GetVel() > targSpeed;
	}

	printf("Acceleration from zero report\n");
	printf("0 to %.2f m/sec in %.1f sec, travel distance %.1f\n\n", 
		targSpeed, frames * m_DeltaT, curPos.m_x - initPos.m_x);
}

	




//////////////////////////////////////////////////////////////////////////
CVehicleObj *
CreateObject(const string &solName, 
			 const CPoint3D& pos,
			 const CVector3D& tang, 
			 const CVector3D& lat)
{
	CCved &cved = m_Cved;

	const slObj* pSolObj = cved.GetSol().QryByName( solName );
	if( !pSolObj ) {
		printf("Could not get sol object\n");
		return 0;
	}

	cvEObjType objType = eCV_VEHICLE;

	//
	// Initialize the attributes.
	//
	cvTObjAttr attr = { 0 };
	attr.solId = pSolObj->GetId();
	attr.xSize = pSolObj->GetXSize();
	attr.ySize = pSolObj->GetYSize();
	attr.zSize = pSolObj->GetZSize();
	attr.hcsmId = 0;

	//
	// Get the starting location and add the vehicle's CG in the z-axis.
	//

	CDynObj  *pObj;
	pObj = m_Cved.CreateDynObj("testDriver",
		objType, attr, &pos, &tang, &lat);

	//
	// Make sure that the CVED object got created properly.
	//
	bool invalidObj = !pObj || !pObj->IsValid();
	if( invalidObj ) {
		printf("Could not create object\n");
		return 0;
	}

	//
	// Get a pointer to the vehicle object.
	//
	CVehicleObj* pVehicleObj = dynamic_cast<CVehicleObj *>( pObj );

	//
	// Initialize the vehicle state and dynamics.  The SOL contains vehicle
	// dynamics settings particular to each vehicle.
	//
	InitializeDynamicsVars( objType, pSolObj, pVehicleObj );
	return pVehicleObj;
}



/////////////////////////////////////////////////////////////////////////////
//
// Implements a lane change controller
//
// Remarks:
// use a PD controller, x is desired lateral distance, xDot is 
// lateral velocity.  The coefficients vary by speed with special
// case for low speed maneuvers
//
// Returns:
// True if the lane change is done, or false if not done
//
bool LaneChangeControl(
	double  latdist,		// how far to travel laterally
	double  longVel,		// current longitudinal velocity
	double  latVel,			// current lateral velocity
	double  urg,			// urgency
	double& latGoal,		// how far offset the target point should be
	double& longGoalDist	// how far ahead the target point should be
)
{
	// check if we are done;  to be done, we have to be close to
	// the goal and have no lateral velocity
	if ( fabs(latdist) < 0.05 && fabs(latVel) < 0.5 ) {
		return true;
	}

	// special low speed section; if going less than a certain mph, 
	// we can do a very sharp turn.  The actual speed is somewhat
	// modified by the urgency
	if ( longVel < (1.0 + urg) * 10.0 ) {
		latGoal      = latdist;
		longGoalDist = longVel;
		if ( longGoalDist < 2.0 ) longGoalDist = 2.0;

		return false;
	}
return false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Lane change   
// 
//
void
LaneChange(
	CVehicleObj*    pObj,		// the object to try out
	double          targV,		// velocity to do lane change
	double          urg,		// urgency to use, 0 < urg <= 1.0
	double          latdist,	// how much lateral travel for lance change
	FILE*           pF = 0		// where to log data
)
{
	CPoint3D   initPos, lastPos, curPos, targPos;
	bool       goalReached;
	int        frames;

	// accelerate to baseline speed first
	Acceleration(pObj, targV, 0.4);

	initPos     = curPos = pObj->GetPos();
	goalReached = false;
	frames      = 0;

	while ( !goalReached ) {
		double distRem;				// remaining lateral distance to cover
		double latGoal;				// how far laterally the goal is
		double longGoal;			// how far ahead goal point is
		double latVel;

		frames++;
		targPos      = curPos;

		// compute remaining distance to travel
		distRem = latdist - (curPos.m_y - initPos.m_y);
		latVel  = (lastPos.m_y - curPos.m_y) / m_DeltaT;

		// we quit only upon completing the lane change
		if ( LaneChangeControl(distRem, pObj->GetVel(),
									latVel, urg, latGoal, longGoal) )
			break;

		targPos.m_x += longGoal;
		targPos.m_y += latGoal;
		pObj->SetTargPos(targPos);

		pObj->SetTargVel(targV);
		pObj->SetTargDist(2.0 * targV);
		pObj->SetInitDynamics(frames == 1);

		// remember last position
		lastPos = curPos;

// execute simulation
		m_Cved.Maintainer();
		m_Cved.ExecuteDynamicModels();
		m_Cved.ExecuteDynamicModels();
	}
}

/////////////////////////////////////////////////////////////////////////////
//
//
main(int argc, char **argv)
{
	void TimeTest(vector<CVehicleObj *>& );
	string err;

	if ( argc < 3 ) {
		Usage(argv[0]);
	}

	m_LriName = argv[1];
	m_SolName = "ChevyBlazerRed";

	m_DeltaT  = 1.0 / 30.0;


	m_Cved.Configure(CCved::eCV_SINGLE_USER, m_DeltaT, 2);
	if ( !m_Cved.Init(m_LriName, err) ) {

		fprintf(stderr, "CCved::Init(%s, ...) failed: %s\n",
			argv[1], err.c_str());
		exit(-1);
	}

	// setup query terrain so it returns flat no matter where we are
	m_Cved.SetNullTerrainQuery(-10.0f, 0.0, 0.0, 1.0);

	// create the object somewhere; this may need to become
	// a command line argument
	CPoint3D  pos(26.701, -4343.0, -10.0);
	CVector3D tang(1.0, 0.0, 0.0), lat(0.0, -1.0, 0.0);
	CVehicleObj *pObj = CreateObject(m_SolName, pos, tang, lat);

	int arg = 2;
	while ( arg < argc ) {
		if ( !strcmp(argv[arg], "-SpeedMaint") ) {
			arg++;

			vector<CSpeedGoal>  goalList;
			while ( arg < argc ) {
				CSpeedGoal goal;
				goal.targSpeed = cMPH_TO_MS*atof(argv[arg]);
				arg++;
				goal.duration  = 30.0*atof(argv[arg]);
				arg++;
				goal.urgency   = atof(argv[arg]);
				arg++;

				goalList.push_back(goal);
			}

			// call the function
			TestSpeedMaintenance(pObj, goalList);

		}
		else if ( !strcmp(argv[arg], "-ZeroAccel") ) {
			arg++;
			double targ = atof(argv[arg]);
			arg++;
			TestZeroAccelSpeed(pObj, cMPH_TO_MS*targ, 30*atoi(argv[arg]));
			arg++;
		}
		else {
			Usage(argv[0]);
		}
	}

#if 0
	CRoad road(m_Cved, "R1_13200_3960");
	CRoadPos startPos(road, 0, 10);

	CPoint3D pos;
	CVector3D tang, lat;

	pos = startPos.GetXYZ();
	tang = startPos.GetTangent();
	lat = startPos.GetRightVec();
#endif
	return 0;
}


/////////////////////////////////////////////////////////////////////////////
//
// A function that forces a model to accelerate  
// 
//
void
TimeTest(vector<CVehicleObj*>& objs)
{
	CPoint3D   initPos, curPos, targPos;
	int        i;
	vector<CVehicleObj*>::iterator  pI;

	for (i=0; i<300; i++) {
		for (pI = objs.begin(); pI != objs.end(); pI++) {
			CVehicleObj  *pObj;
			pObj = *pI;

			initPos     = curPos = pObj->GetPos();
			targPos     = curPos;
			targPos.m_x += 10.0;
			pObj->SetTargPos(targPos);
			pObj->SetTargDist(5.0);

			double v0 = pObj->GetVel();

			pObj->SetTargVel(20.0);
			pObj->SetInitDynamics(i==0);
		}

		m_Cved.Maintainer();
		m_Cved.ExecuteDynamicModels();
		m_Cved.ExecuteDynamicModels();
	}
}
