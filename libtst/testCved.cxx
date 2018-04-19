/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: testCved.cxx,v 1.7 1999/05/11 22:56:42 lmoineau Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	Basic tests for the CCved class
//
/////////////////////////////////////////////////////////////////////////////
#ifdef _WIN32
#include <iostream>
#elif __sgi
#include <iostream.h>
#endif

#include <cved.h>
#include <cvedpub.h>

using namespace CVED;
using namespace std;

// this dummy definition to bring in the necessary template instances
vector<CIntrsctn> a;
TRoadPieceVec dummy1;
TCrdrVec dummy2;
TRoadVec dummy3;

void testBasic(bool print = false);

/////////////////////////////////////////////////////////////////////////////
//
// Test system related calls, verify state diagram, proper error
// checking and exceptions behavior
//
void
testSystem(void)
{
	CCved   cved;

	// calling Init without configure should faile
	string  lri, msg;

	lri = "../tools/lricomp/smallb.lri";

	if ( cved.Init(lri, msg) == true ) {
		cout << "Should be error:" << __LINE__ << endl;
		exit(1);
	}
}



void
testBasic(bool print)
{
	// this array contains the correct answers for the various roads.
	static struct TCorrectAnswers {
		char  *pName;
		float linLen;
		float cubLen;
	} ca[] = {  
		{ "George", 1.0 },
		{ "Cheetah", 2.0 },
		{ "Chester", 2.0 },
		{ "Main_1", 2.0 }
	};

	CCved   cved;
	string  msg;

	// proper behavior
	if (!cved.Configure(CCved::TSingleUser, 0.1f, 2))
	{
		cout << "cved::configure failed: " << __LINE__ << endl;
		exit(0);
	}

	if ( cved.Init("smallb.lri", msg) == false ) {
		cout << "cved::Init failed: " << msg << endl;
		exit(1);
	}
	
	if (cved.Attach())
	{
		cout << "Should not Attach as is the only instance!" << endl;
		exit(1);
	}

	cved.verify();
	return;

#if 0
	// get all roads and print them
	CCved::TRoadVec  roads;
	int    i=0;

	cved.GetAllRoads(roads);
	cout << "  ----- All Roads -----" << endl;
	for (i=0; i<roads.size(); i++) {
		if ( print ) cout << "Road" << i << ":" << roads[i].GetName()<< ", "
			<< " Id = "<< roads[i].GetId() << ", ";

		// test copy constructor and assignment op
		CRoad r = roads[i];
		CRoad r2(roads[i]);
		
		assert(r.GetName() == ca[i].pName);
		assert(roads[i].GetName() == r2.GetName());
		assert(r.GetName() == r2.GetName());
		assert(r.GetId() == roads[i].GetId());
		assert(r.GetId() == r2.GetId());

		// test overloaded << for Road Objects, better used when road fully initialized
		cout << "Road r info: " << r << " & ";
		cout << "Road r2 info: " << r2 << endl;

		if ( print ) cout << roads[i].GetLinearLength() << ", ";
		// assert(r.GetLinearLength() == ca[i].linLen);
		assert(roads[i].GetLinearLength() == r2.GetLinearLength());
		assert(r.GetLinearLength() == r2.GetLinearLength());

		if ( print ) cout << roads[i].GetCubicLength() << ", ";
		// assert(r.GetCubicLength() == ca[i].cubLen);
		assert(roads[i].GetCubicLength() == r2.GetCubicLength());
		assert(r.GetCubicLength() == r2.GetCubicLength());

		if ( print ) cout << roads[i].GetNumLanes() << ", ";
		// assert(r.GetNumLanes() == ca[i].cubLen);
		assert(roads[i].GetNumLanes() == r2.GetNumLanes());
		assert(r.GetNumLanes() == r2.GetNumLanes());

		if ( print ) cout << roads[i].GetCntrlPntCount() << ", ";
		// assert(r.GetCntrlPntCount() == ca[i].cubLen);
		assert(roads[i].GetCntrlPntCount() == r2.GetCntrlPntCount());
		assert(r.GetCntrlPntCount() == r2.GetCntrlPntCount());

		CIntrsctn SrcInt(&cved, r.GetSource());
		CIntrsctn DstInt(&cved, r.GetDest());

		if ( r.GetSource() && r.GetDest()){
				if ( print ) cout << "From " << SrcInt.GetName() <<
					" -> " << DstInt.GetName() << endl;
		}
		cout << endl;

		// test ostream << and GetRoad()
		cout << r << endl;
		cout << cved.GetRoad(r.GetId()) << endl;
		//cout << cved.GetRoad(r2.GetName()).GetId() << endl;
		
		assert( cved.GetRoad(r.GetId()).GetId() == r2.GetId());
		cout <<r.GetName() << endl;
		cout << cved.GetRoad(r.GetName()) << endl;
		//assert(r.GetId() == (cved.GetRoad(r2.GetName())).GetId());
	}
	
	CCved cvedDup;
	if (!cvedDup.Attach())
	{
		cout << "Can't Attach: " << __LINE__ << endl;
		exit(1);
	}
#endif

}


main(int argc, char **argv)
{
	testSystem();
	testBasic(true);
	return 0;
}
