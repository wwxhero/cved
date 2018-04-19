//////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	 $Id: dyntest.cxx,v 1.8 1999/10/19 17:25:35 oahmad Exp $
//
// Author(s):	 Gustavo Ordaz Hernandez, Omar Ahmad
// Date:		   August, 1999
//
// Description:	Vehicle dynamics test application.
//
//////////////////////////////////////////////////////////////////////

#ifdef _WIN32
#pragma warning(disable:4786)
#endif

#include <string>

#include "cvedpub.h"
#include "cvedstrc.h"
#include "objlayout.h"

using namespace CVED;

#include "vehicledynamics.h"

//////////////////////////////////////////////////////////////////////////////
//
// Global Variables 
//
//////////////////////////////////////////////////////////////////////////////
int g_framesToRun = 4000;


//////////////////////////////////////////////////////////////////////////////
//
// Local Prototypes
//
//////////////////////////////////////////////////////////////////////////////
static void InitializeCved( string lriFileName, CCved& cved );
static void ParseCommandLineOptions( int argc, char** argv );
static void Usage( void );




//////////////////////////////////////////////////////////////////////////////
//
// Initializes CVED.
//
//////////////////////////////////////////////////////////////////////////////
static void InitializeCved( string lriFileName, CCved& cved )
{

	string cvedErr;

	cved.Configure( CCved::eTMultiUser, 0.1f, 3 );
	if ( cved.Init( lriFileName, cvedErr ) == false ) {

		cerr << "Cved initialization failed with lri file '"; 
		cerr << lriFileName << "'." << endl << "Error message: " ;
		cerr << cvedErr << endl;
		cerr << "Quiting ...." << endl;

		exit( -1 );

	}

	cved.SetDebug( 0 );

}


//////////////////////////////////////////////////////////////////////////////
//
// Parses command-line arguments.  WARNING:  this function uses global
// variables.
//
//////////////////////////////////////////////////////////////////////////////
static void ParseCommandLineOptions( int argc, char** argv )
{

	int arg;
	for ( arg = 1; arg < argc; arg++ ) {

		if ( argv[arg][0] == '-' ) {

			if ( !strcmp(argv[arg], "-frames")) {

				if ( arg + 1 >= argc )  Usage();

				arg++;
				g_framesToRun = atoi( argv[arg] );

			}
			else if ( !strcmp(argv[arg], "-h")) {

				Usage();

			}
			else {

				// user has given an invalid command-line argument
				Usage();

			}

		}  // end if argv[arg][0]

	}  // end for

}  // ParseCommandLineArguments


//////////////////////////////////////////////////////////////////////////////
//
// How to use this application.
//
//////////////////////////////////////////////////////////////////////////////
static void Usage( void )
{

	cerr << "Usage: dyntest [ -frames ### ]" << endl;
	exit(0);

}


//////////////////////////////////////////////////////////////////////////////
//
// Main entry point.
//
//////////////////////////////////////////////////////////////////////////////
main( int argc, char* argv[] ) 
{

	ParseCommandLineOptions( argc, argv );

        int i;
	CCved  cved;
	InitializeCved( "../../data/ipte.bli", cved );

	cvTObjAttr attr;
	cvTObjState::VehicleState state;

	cvTObjAttr attr = { 0 };

#if 0
	// Use the SOL id to get access to vehicle dynamics parameters.
	string solObjName = cved.GetSol().QryNameByID( attr.solId );
	const slObj* pSolObj = cved.GetSol().QryByName( solObjName );
	const slVehicleObj* pSolVeh = dynamic_cast<const slVehicleObj*> ( pSolObj );


	attr.solId = pSolObj->getId();
	attr.xSize = pSolObj->getXSize();
	attr.ySize = pSolObj->getYSize();
	attr.zSize = pSolObj->getZSize();
	attr.hcsmId = m_pRootCollection->GetHcsmId( this );

	attr.Mass = 1150.0;
	attr.VInertia[0] = 850.0;
	attr.VInertia[1] = 1600.0;
	attr.VInertia[2] = 1850.0;
	attr.VInertia[3] = 0.0;
	attr.VInertia[4] = 0.0;
	attr.VInertia[5] = 150.0;
	attr.WheelBase_F = 1.064;
	attr.WheelBase_R = 1.596;
	attr.WheelTrack  = 1.5;
	attr.AxleHeight  = -0.3;
	attr.SuspStif    = 350300.0;
	attr.SuspDamp    = 17500.0;
	attr.TireMass    = 50.0;
	attr.TireInert   = 6.25;
	attr.TireRadius  = 0.35;
	attr.TireStif    = 630450.0;
	attr.TireDamp    = 17510.0;
#endif

	InitFourWheelVehState( cved, attr, &state );

	cvTObjState::VehicleState     curState = state;
	cvTObjState::VehicleState     futState;
	cvTObjContInp::VehicleContInp contInp;

	float dstep = 0.01;
	for ( i = 0; i < g_framesToRun; i++ ) {

		DoVehicleDynamics( cved, dstep, &attr, &curState, &contInp, &futState );
		curState = futState;
		
	}

	exit( 0 );

}
