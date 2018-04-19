// printroad.cpp : Defines the entry point for the console application.
//
#include <cvedpub.h>
#include <cved.h>

#include <iostream>

using namespace std;
using namespace CVED;

CCved g_Cved;
string g_RoadName;
float  g_StartDist;
float  g_EndDist;
int    g_Lane;
float  g_Offset;
float  g_Freq;
string g_Fname;

void Usage()
{
	cerr << 
		" -- This utility allows interrogation of a CVED terrain database\n"
		" -- using various modes, including along a road or based on queries\n"
		" -- read from a file\n\n";

	cerr << 
		"Usage: " << endl
		<< "printroad lri_name road_name start_dist end_dist lane offs intrvl outfile_root" << endl
		<< "printroad lri_name -line x1 y1 z1 x2 y2 nsteps" << endl
		<< "printroad lri_name -file fname [-zofs num] [-out fname]" << endl << endl
		<< "    -file:  read each line in file as 'x y ' inpus to a query \n"
		   "            -zofs: add num to z before printing out\n"
		   "            -out: specify file to write data as opposed to standard out\n"
		   "            Note: x,y are flipped before query, zout negated after query;\n"
		   "                  output file contains: x y z i j k  trnObjFlag  material\n"
		<< endl
		<< endl;
	exit(-1);
}



/////////////////////////////////////////////////////////////////////////////
//
// Opens a file and treats each line in the file as the basis for a 
// terrain query with hint.
// 
void
FileQuery(FILE *pF, float ofs, FILE *pOut)
{
	char                   line[2048];
	float                  xin, yin, zin, zout;
	CCved::CTerQueryHint   hint;
	CVector3D              norm;

	if ( pOut == 0 ) pOut = stdout;

	while ( !feof(pF) ) {
		if ( !fgets(line, 2047, pF) ) break;

		zin = 0.0f;
		int n = sscanf(line, "%f%f%f", &xin, &yin, &zin);
		if ( n < 2 ) {
			fprintf(stderr, "ERROR:  Could not parse line "
								"of input file that reads: %s", line);
			return;
		}
		int ter, mat;

		g_Cved.QryTerrain(yin, xin, zin, zout, norm, &hint, &ter, &mat);

		fprintf(pOut, "%.4f  %.4f  %.6f   %.6f %.6f %.6f  %d %d\n",
			xin, yin, -zout + ofs, norm.m_i, norm.m_j, norm.m_k, ter, mat);
	}
}



void
DumpInventor(
	FILE *pF, 
	vector<CRoad::CtrlInfo>&   rawData,
	float                      startDist,
	float                      endDist)
{
	int i;

	fprintf(pF, "#Inventor V2.1 ascii\n\n");
	fprintf(pF, "Separator {\n");
	
	for (i=0; i<rawData.size()-1; i++) {
		float high = rawData[i].cummLinDist;
		if ( rawData[i].cummLinDist < startDist ) continue;
		if ( rawData[i].cummLinDist > endDist ) continue;

		CPoint3D  pCur(rawData[i].pos.x, rawData[i].pos.y,
			rawData[i].pos.z);
		CPoint3D  pNext(rawData[i+1].pos.x, rawData[i+1].pos.y,
			rawData[i+1].pos.z);
		CVector3D  norm(rawData[i].norm.i, rawData[i].norm.j,
			rawData[i].norm.k);

		CVector3D  tang, rght;
		tang = pCur - pNext;
		rght = tang.CrossP(norm);

		tang.Normalize();
		rght.Normalize();

		CPoint3D  p1, p2, p3, p4, pTop;

		p1 = pCur + 20 * tang - 20 * rght;
		p2 = pCur + 20 * tang + 20 * rght;
		p3 = pCur - 20 * tang + 20 * rght;
		p4 = pCur - 20 * tang - 20 * rght;
		pTop = pCur + 120 * norm;

		fprintf(pF, "\tSeparator {\n\t\tCoordinate3 {\n\t\t\tpoint [ ");
	
		fprintf(pF, "%f %f %f, \n\t\t\t", p1.m_x, p1.m_y, p1.m_z);
		fprintf(pF, "%f %f %f, \n\t\t\t", p2.m_x, p2.m_y, p2.m_z);
		fprintf(pF, "%f %f %f, \n\t\t\t", p3.m_x, p3.m_y, p3.m_z);
		fprintf(pF, "%f %f %f, \n\t\t\t", p4.m_x, p4.m_y, p4.m_z);
		fprintf(pF, "%f %f %f, \n\t\t\t", pCur.m_x, pCur.m_y, pCur.m_z);
		fprintf(pF, "%f %f %f", pTop.m_x, pTop.m_y, pTop.m_z);

		fprintf(pF, "]\n\t\t}\n\t\tIndexedFaceSet { coordIndex[ 3, 2, 1, 0 ] }\n");

		fprintf(pF, "\t\tIndexedLineSet { coordIndex[ 4, 5, -1 ] } \n\t}\n");
	}

//////////////// Now put terrain queries in the mix
	float curDist = startDist;
	CRoadPos pos( g_Cved, g_RoadName, g_Lane, g_StartDist, g_Offset );
	CCved::CTerQueryHint hint;

	while ( pos.GetDistance() <= endDist ) {
		float     zout;
		CVector3D norm;
		CPoint3D  pnt;
		CPoint3D  pTop;

		pnt = pos.GetBestXYZ();

		g_Cved.QryTerrain( pnt, zout, norm, &hint);
		pnt.m_z = zout;
		
		pTop = pnt + 80 * norm;

		fprintf(pF, "\tSeparator {\n\t\tCoordinate3 {\n\t\t\tpoint [ ");
		fprintf(pF, "%f %f %f, \n\t\t\t", pnt.m_x, pnt.m_y, pnt.m_z);
		fprintf(pF, "%f %f %f", pTop.m_x, pTop.m_y, pTop.m_z);
		fprintf(pF, "]\n\t\t}\n\t\tIndexedLineSet { coordIndex[ 0, 1, -1 ] }\n\t}\n");

		curDist += g_Freq;
		pos.Travel( g_Freq );
	}

	fprintf(pF, "}\n");
}


int main(int argc, char* argv[])
{

	if ( argc < 3 ) Usage();

	string lriName = argv[1];
	string err;

	g_Cved.Configure( CCved::eCV_SINGLE_USER, 0.3f, 3 );
	if ( !g_Cved.Init( lriName, err ) ) {
		cerr << "Cannot load lri file " << lriName << endl;
		cerr << "Error: " << err << endl;
		exit(-1);
	}

	if ( !strcmp(argv[2], "-file") ) {
		if ( argc < 4 ) Usage();

		int   arg;
		float ofs          = 0;
		char* pOutFileName = 0;

		if ( argc >= 6 ) {
			for (arg=4; arg<argc; arg++) {
				if ( !strcmp(argv[arg], "-zofs") ) {
					arg++;
					ofs = atof(argv[arg]);
				}
				else if ( !strcmp(argv[arg], "-out") ) {
					arg++;
					pOutFileName = argv[arg];
				}
				else {
					Usage();
				}
			}
		}

		FILE *pOut = 0;
		FILE *pF   = fopen(argv[3], "r");

		if ( pF == NULL ) {
			cerr << "Cannot open specified data file: " << argv[3] << endl;
			exit(-1);
		}

		if ( pOutFileName ) {
			pOut = fopen(pOutFileName, "w");
			if ( pOut == 0 ) {
				perror("Can't open output file\n");
				exit(-1);
			}
		}

		FileQuery(pF, ofs, pOut);
		fclose(pF);
	}
	else
	if ( !strcmp(argv[2], "-line") ) {
		if ( argc != 9 ) Usage();

		float  x1      = atof(argv[3]);
		float  y1      = atof(argv[4]);
		float  z1      = atof(argv[5]);
		float  x2      = atof(argv[6]);
		float  y2      = atof(argv[7]);
		int    nsteps  = atoi(argv[8]);

		if ( nsteps < 0 ) {
			cerr << "Need positive number of steps" << endl;
			exit(-1);
		}

		double deltaX = (x2-x1)/nsteps;
		double deltaY = (y2-y1)/nsteps;

		double x = x1;
		double y = y1;
		double z = z1;

		CCved::CTerQueryHint hint;
		for (int i=0; i<nsteps; i++) {
			float zout;
			int   trnUsed;	// terrain object used flag
			int   mat;		// material

			CVector3D norm;
			g_Cved.QryTerrain(x, y, z, zout, norm, &hint, &trnUsed, &mat);

			printf("%10.2f %10.2f %8.2f   %7.4f %7.4f %7.4f %d %d\n",
				x, y, zout, norm.m_i, norm.m_j, norm.m_k, trnUsed, mat);

			x += deltaX;
			y += deltaY;
			z = zout;
		}
		return 0;
	}
	else {
		vector<CRoad::CtrlInfo>  rawData;
		int i;

		// usage: printroad lri_name road_name 
		//            start_dist end_dist lane offs intrvl outfile_root

		if ( argc != 9 ) Usage();

		g_RoadName   = argv[2];
		g_StartDist  = atof( argv[3] );
		g_EndDist    = atof( argv[4] );
		g_Lane       = atoi( argv[5] );
		g_Offset     = atof( argv[6] );
		g_Freq       = atof( argv[7] );
		g_Fname      = argv[8];

		if ( g_StartDist < 0 ) g_StartDist = 0.0;

		CRoad  road(g_Cved, g_RoadName);
		if ( !road.IsValid() ) {
			cerr << "Can't find road " << g_RoadName << endl;
			exit(-1);
		}
		road.GetCntrlPoint(-1, -1, rawData);

		if ( g_EndDist < 0 ) {
			g_EndDist =  road.GetLinearLength() - 1.0;
		}

		FILE *pF;

		//
		// first dump the raw road data to a file
		//
		pF = fopen((g_Fname + "_raw.txt").c_str(), "w");
		if ( pF == NULL ) {
			perror("Cannot create _raw output file");
			return 0;
		}
	
		for (i=0; i<rawData.size(); i++) {
			if ( rawData[i].cummLinDist < g_StartDist ) continue;
			if ( rawData[i].cummLinDist > g_EndDist ) continue;
			fprintf(pF, "%f %f %f %f %f %f\n", 
				rawData[i].pos.x, rawData[i].pos.y, rawData[i].pos.z,
				rawData[i].norm.i, rawData[i].norm.j, rawData[i].norm.k);
		}
		fclose(pF);


		//
		//  dump the raw road data to an inventor file
		//
		pF = fopen((g_Fname + "_raw.iv").c_str(), "w");
		if ( pF == NULL ) {
			perror("Cannot create _raw output file");
			return 0;
		}
		DumpInventor(pF, rawData, g_StartDist, g_EndDist);
		fclose(pF);

		//
		// Now dump data using the GetBestXYZ interpolation that is supposed
		// to use the splines
		//

		pF = fopen((g_Fname + "_cub.txt").c_str(), "w");
		if ( pF == NULL ) {
			perror("Cannot create _cub output file");
			return 0;
		}

		CRoadPos pos( g_Cved, g_RoadName, g_Lane, g_StartDist, g_Offset );
		float    curDist = g_StartDist;

		while ( pos.GetDistance() <= g_EndDist ) {
			CPoint3D pnt = pos.GetBestXYZ();

			fprintf(pF, "%f %f %f\n", pnt.m_x, pnt.m_y, pnt.m_z);
			curDist += g_Freq;
			pos.Travel( g_Freq );
		}
		fclose(pF);

#if 0
		float curDist = startDist;
		CCved::CTerQueryHint hint;

		while ( pos.GetDistance() <= endDist ) {
			float zout;

			CVector3D norm;
			cved.QryTerrain( pos, zout, norm, &hint );
			CPoint3D pnt = pos.GetBestXYZ();
			cout << pnt.m_x << " " << pnt.m_y << " " << pnt.m_z << " " << 
				norm.m_i << " " << norm.m_j << " " << norm.m_k << endl;
			curDist += freq;
			pos.Travel( freq );
		}
#endif
		return 0;
	}

	return 0;
}

