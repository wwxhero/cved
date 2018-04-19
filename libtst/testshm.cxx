/////////////////////////////////////////////////////////////////////////////
//
// Version: 		$Id: testshm.cxx,v 1.2 1999/05/11 22:56:42 lmoineau Exp $
//
// A program to test the shared memory class.  Depending on the
// arguments, it creates or attaches to a segment and then
// either produces or consumes values while printing them
// for verification.
//
#include <time.h>
#include <iostream>
#include "sharedmem.h"

#include "cved.h"

using namespace std;

// timing routine
//
void timing(void)
{
	CSharedMem  s;
	char       *p1, *p2;
	int         size = 16 * 1024 * 1024;

	p1 = (char *)s.Connect("time", true, size);
	if ( p1 == NULL ) 
		return;
	p2 = new char[size];


	int t;
	time_t start,end;
	
	start = time(NULL);
	for (t=0; t<50; t++)
		memset(p1, t, size);
	end = time(NULL);
	cout << "Elapsed seconds: " << difftime(end, start) << endl;

	start = time(NULL);
	for (t=0; t<50; t++)
		memset(p2, t, size);
	end = time(NULL);
	cout << "Elapsed seconds: " << difftime(end, start) << endl;


}


// 
// tstshm  create | attach  name  size 
// when create, start writing an int value
// when attaching, print the int value
int
main(int argc, char **argv)
{
	struct Layout {
		int  id;
	} *pData;

	CVED::CRoad r(0, 1);

	timing();

	if ( argc != 4 ) {
		cout << "Expected 3 args: create | attach name size " << endl;
		return -1;
	}

	CSharedMem  shm;
	bool        create;

	create = !strcmp(argv[1], "create");
	if ( create ) {
		cout << "Creating a new shared segment ..." << endl;
		pData = static_cast<Layout *>
			(shm.Connect(argv[2], true, atoi(argv[3])));
	}
	else {
		pData = static_cast<Layout *>
			(shm.Connect(argv[2]));
	}

	if ( pData == 0 ) {
		cout << "Connect failed: " << shm.GetErrorMsg() << endl;
		return -1;
	}

	if ( create ) pData->id = 0;

	for (int i=0; i<15; i++) {
		if ( create ) {
			pData->id++;
		}
		else {
			cout << pData->id << endl;
		}
		Sleep(1000);
	}

	return 0;
}
