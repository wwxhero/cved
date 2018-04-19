/***************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center The University of Iowa
 * and The University of Iowa.  All rights reserved.
 *
 * $Id: semcheck.cxx,v 1.3 2014/10/09 23:57:44 IOWA\dheitbri Exp $
 *
 * Author(s) :  Imran A. Pirwani
 * Date:        Aug. 17, 1998
 *
 * Description:
 * This file contains the code that verifies correct semantics for
 * parser data structures.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#include <ostream>
#include <iostream>
#elif __sgi
#include <typeinfo>
#include <iostream.h>
#endif

#include "parser.h"

#define MAX_TOL 1e-3
#define MAX_Z_TOL 1e-2
#define EPSILON 1e-5

#ifndef M_PI
#define M_PI 3.1415926
#endif

#undef DEBUG_SEMCHECK

typedef struct SplineData {
   float x, y, z;
   float i, j, k;
   int index;
} TSplineData, *SplineDataPtr;

typedef struct Vec {
	float i,j,k;
} TVec, *pVec; 

typedef struct Pt {
	float x,y,z;
} TPt, *pPt; 


static bool
NormalizeVec( pVec v )
{
	double mag, one_ov_mag;

	mag = sqrt( v->i*v->i + v->j*v->j + v->k*v->k );
	if ( mag <= EPSILON ) {
		return false;
	}		

	one_ov_mag = 1.0 / mag;
	v->i      *= static_cast<float>(one_ov_mag);
	v->j      *= static_cast<float>(one_ov_mag);
	v->k      *= static_cast<float>(one_ov_mag);	
	return true;
}

static double
DotP(const pVec v1, const pVec v2)
{
   return v1->i*v2->i + v1->j*v2->j + v1->k*v2->k;
}

void
GetAllSplinePoints( TSplineData *spline_in, TControlPoint *TCp, 
								int orig_pts )
{
	int i;
	for ( i = 0; i < orig_pts; i++ ) {
		spline_in[i].x = TCp[i].x;
		spline_in[i].y = TCp[i].y;
		spline_in[i].z = TCp[i].z;
		spline_in[i].i = TCp[i].i;
		spline_in[i].j = TCp[i].j;
		spline_in[i].k = TCp[i].k;
		spline_in[i].index = i;
	}
}

void
PutSplinePoints( int old_num_pts, int num_pts, SplineDataPtr &spline_in,
									TControlPoint * &TCp )
{
	int i;

#if 0
	TCp = (TControlPoint *)realloc( TCp, num_pts * sizeof( TControlPoint ) ); 
	if ( TCp == NULL ) {
		fprintf(stderr,"Can't realloc memory\n"); 
		exit(-1);
	}

	for ( i = 0; i < num_pts; i++ ) {
		TCp[i].x = spline_in[i].x;
		TCp[i].y = spline_in[i].y;
		TCp[i].z = spline_in[i].z;
		TCp[i].i = spline_in[i].i;
		TCp[i].j = spline_in[i].j;
		TCp[i].k = spline_in[i].k;
	}
#else

	int j;
	TControlPoint* newTCp = (TControlPoint *)malloc( num_pts*sizeof( TControlPoint ) ); 
	if ( newTCp == NULL ) {
		fprintf(stderr,"Can't alloc memory\n"); 
		exit(-1);
	}

	for ( i = 0; i < num_pts; i++ ) {
		newTCp[i].x = spline_in[i].x;
		newTCp[i].y = spline_in[i].y;
		newTCp[i].z = spline_in[i].z;
		newTCp[i].i = spline_in[i].i;
		newTCp[i].j = spline_in[i].j;
		newTCp[i].k = spline_in[i].k;
		memcpy( newTCp[i].cpointName, TCp[spline_in[i].index].cpointName, sizeof(newTCp[i].cpointName) );
		newTCp[i].pCpointInfo = TCp[spline_in[i].index].pCpointInfo;

		TCpointInfo* lastPointInfo = newTCp[i].pCpointInfo;
		if ( lastPointInfo != NULL )
			while ( lastPointInfo->pNext != NULL )
				lastPointInfo = lastPointInfo->pNext;
		
		int upper_limit;
		if ( i < num_pts-1 )
			upper_limit = spline_in[i+1].index;
		else
			upper_limit = old_num_pts;

		for ( j=spline_in[i].index+1; j<upper_limit; ++j ) {
			// the last non-empty name among the merged points is used
			if ( strlen(TCp[j].cpointName) > 0 )
				memcpy( newTCp[i].cpointName, TCp[j].cpointName, sizeof(newTCp[i].cpointName) );
			// the control point information of all the merged points are
			// attached to the new point. There may be multiple appearances
			// of information of the same category that conflict each other, 
			// such as lane width. It seems the last appearance within the 
			// same category will be used, which is desired.
			if ( TCp[j].pCpointInfo != NULL ) {
				if ( lastPointInfo != NULL )
					lastPointInfo->pNext = TCp[j].pCpointInfo;
				else {
					newTCp[i].pCpointInfo = TCp[j].pCpointInfo;
					lastPointInfo = newTCp[i].pCpointInfo;
				}
				while ( lastPointInfo->pNext != NULL )
					lastPointInfo = lastPointInfo->pNext;
			}
		}
	}

	free( TCp );
	TCp = newTCp;
#endif

}

bool
AverageNormal( pVec v1, pVec v2, pVec res )
{
	res->i = (v1->i+v2->i)/2.0f;
	res->j = (v1->j+v2->j)/2.0f;
	res->k = (v1->k+v2->k)/2.0f;

	return NormalizeVec( res );
}

void
AveragePt( pPt p1, pPt p2, pPt res )
{
	res->x = (p1->x + p2->x)/2;
	res->y = (p1->y + p2->y)/2;
	res->z = (p1->z + p2->z)/2;

}

#if 1//def DEBUG_SEMCHECK
void
dumpSpline( TSplineData *s, int size )
{
	int i;
	for (i=0; i < size; i++ ) {
		printf("\t%f %f %f %f %f %f\n",s[i].x,s[i].y,s[i].z,s[i].i,s[i].j,s[i].k);
	}
	printf("\n");
}

void
ptPrint( pPt pt )
{
	printf("(%g, %g, %g)\n",pt->x,pt->y,pt->z);
}
#endif

int
CorrectSplineData( SplineDataPtr &spline, int nsp_in, char *pRdName )
{
	int i = 1, nsp_out = nsp_in;
	TVec n1,n2;
	TVec ave_n;
	TPt p1,p2;
	TPt ave_p;
	TSplineData *spl = spline;

	while( i < nsp_out ) {	
		float theta;
		/* First, normalize the vectors */
		n1.i = spl[i-1].i; n1.j = spl[i-1].j; n1.k = spl[i-1].k;
		n2.i = spl[i].i; n2.j = spl[i].j; n2.k = spl[i].k;

		p1.x = spl[i-1].x; p1.y = spl[i-1].y; p1.z = spl[i-1].z;
		p2.x = spl[i].x; p2.y = spl[i].y; p2.z = spl[i].z;

		if ( !NormalizeVec( &n1 ) ) {
			fprintf(stderr, "Normal vector for control point %d is 0.\n", i-1);
			return -1;
		}

		if ( !NormalizeVec( &n2 ) ) {
			fprintf(stderr, "Normal vector for control point %d is 0.\n", i);
			return -1;
		}

		/* Check relative super-elevation */
		theta = static_cast<float>(acos( DotP( &n1, &n2 ) ));
		if ( theta > M_PI/4 ) {
			/* Superelevation too large */
			fprintf(stderr, 
			"Angular change between normals in points %d & %d on"
			" road %s too big\n",
				i-1, i, pRdName);
			fprintf(stderr,
				"Normal1: %f,%f,%f\n", spl[i-1].i, spl[i-1].j,spl[i-1].k);
			fprintf(stderr, 
				"Normal2: %f,%f,%f\n", spl[i].i, spl[i].j,spl[i].k);
			return -1;
		}

		/* Copy them back into the array */
		spl[i-1].i = n1.i; spl[i-1].j = n1.j; spl[i-1].k = n1.k;
		spl[i].i = n2.i; spl[i].j = n2.j; spl[i].k = n2.k;
		if( fabs( p2.x - p1.x ) < MAX_TOL &&
			 fabs( p2.y - p1.y ) < MAX_TOL ) { 
			/* if close on the projection on the xy plane */
			if( fabs( p2.z - p1.z ) < MAX_Z_TOL ) {
				/*
				 * almost duplicate points.  Take average point and normal and
				 * move all points up in the array.
				 */ 
				AverageNormal( &n1, &n2, &ave_n );
				AveragePt( &p1, &p2, &ave_p );
#if 1// def DEBUG_SEMCHECK
				printf("Moving a point\n");
				ptPrint( &p1 );
				ptPrint( &p2 );
				ptPrint( &ave_p );
#endif
				spl[i-1].x = ave_p.x; spl[i-1].y = ave_p.y; spl[i-1].z = ave_p.z;
				spl[i-1].i = ave_n.i; spl[i-1].j = ave_n.j; spl[i-1].k = ave_n.k;

				memmove( &spl[i], &spl[i+1], (nsp_out-i-1)*sizeof(TSplineData ) );
				nsp_out--;
				continue;
			} else {
				/* The point falls pretty sharply, we can't model this */
				printf("Error!! Can't model this...z is too steep\n");
#ifdef DEBUG_SEMCHECK
				ptPrint(&p1);
				ptPrint(&p2);
#endif
				exit(-4);
			}
		}

		i++;
	}

	if ( nsp_out < 2 ) {
		printf("Error...fewer than 2 points for road\n");
		exit( -6 );
	}

	if (nsp_out != nsp_in) {
		if(!( spl = (TSplineData *)realloc( spl, nsp_out * sizeof( TSplineData ) ) ))
			printf("Error!! Can't allocate memory for spline\n");
		spline = spl; 
	}

	if ( nsp_out > 2 ) {
		/* Must check to see if points are in some order, or
		 * if the curvature of the spline is too severe*/
		for ( i = 2; i < nsp_out; i++ ) { 
			double dp;
			TVec v1,v2;

			v1.i = spl[i-1].x - spl[i-2].x;
			v1.j = spl[i-1].y - spl[i-2].y;
			v1.k = spl[i-1].z - spl[i-2].z;
			v2.i = spl[i].x - spl[i-1].x; 
			v2.j = spl[i].y - spl[i-1].y; 
			v2.k = spl[i].z - spl[i-1].z;

			NormalizeVec( &v1 );
			NormalizeVec( &v2 );

			dp = DotP( &v1, &v2 );


			if ( (dp > (1/sqrt(2.0))) && ( dp-1.5 <= cCV_ZERO ) ) {
				/* Points are good...do nothing */
			} else {
				fprintf(stderr, 
					"Points %d, %d and %d in road %s are not in order.\n",
					i-2, i-1, i, pRdName);
				fprintf(stderr, "Point %d: %f, %f, %f\n", i-2, 
					spl[i-2].x, spl[i-2].y, spl[i-2].z);
				fprintf(stderr, "Point %d: %f, %f, %f\n", i-1, 
					spl[i-1].x, spl[i-1].y, spl[i-1].z);
				fprintf(stderr, "Point %d: %f, %f, %f\n", i, 
					spl[i].x, spl[i].y, spl[i].z);
				return -1;
			}
		}
	}
#ifdef DEBUG_SEMCHECK
	dumpSpline( spl, nsp_out );
#endif

	fprintf(stderr, "Removed %d duplicate point(s).\n", nsp_in-nsp_out);
	return nsp_out;
}


//////////////////////////////////////////////////////////////////////////
//
// Adds an item in a list if the item is not already there
//
//
void
InsertInArray( vector<string> &list, const string &item)
{
	int found = 0;
	vector<string>::const_iterator  pItem;

	for (pItem=list.begin(); pItem != list.end(); pItem++) {
		if ( *pItem == item ) {
			found = 1;
			break;
		}
	}

	if ( !found ) {
		list.push_back(item);
	}
}


int
FoundRoadInIntrsxn( TRoadName * rdnames, char * rdname )
{
	TRoadName * ptr = rdnames;
	int found = 0;
	for ( ptr = rdnames; ptr != NULL; ptr=ptr->pNextRoadName ) {
		if ( !strcmp(ptr->roadName, rdname) ) {
			found = 1;
			break;
		}
	} 

	return found;
}

int
IntExists( char * intrsxn, TIntersection *pInter, char * rdname )
{
	TIntersection *pI = pInter;
	int found = 0;
	if ( pInter == NULL )
		return 0;
	for ( pI = pInter; pI != NULL; pI=pI->pNext ) {
		if ( !strcmp(intrsxn, pI->name) ) {
			if ( FoundRoadInIntrsxn( pI->pRoadNames, rdname ) ) 
				found = 1;
			else
				found = 0;
			break;
		}
	}

	return found;
}

int
RdExists(char * rdname, TRoads * pRoads, char * intname)
{
	TRoads *ptr = pRoads;
	int found = 0;
	for ( ; ptr != NULL; ptr=ptr->pNext ) {
		if ( !strcmp( rdname, ptr->roadName ) ) {
			if ( !strcmp(ptr->intersection1, intname) ||
				  !strcmp(ptr->intersection2, intname) ) {
				found = 1;
				break;
			}
		}
	} 

	return found;
}


///////////////////////////////////////////////////////////////////
//
// This function checks the corridor data for duplicate points.
// Any duplicate points are eliminated.  Warnings are printed
// when the eliminated points had line attributes since we are
// not copying the attributes.
//
void 
CheckCorridorData(const char *pIntName, TCrdr* pCrdr)
{
	vector<TCrdrCurve>  newData;
	vector<TCrdrCurve>::const_iterator i;
	int pt;
	bool warningPrinted = false;	// don't print too many warnings

	for (pt=0; pt<pCrdr->CrdrCurveList.n_crdrpts; pt++) {
		TCrdrCurve *pC1 = &pCrdr->CrdrCurveList.pCrdrCurve[pt];

		if ( pt == pCrdr->CrdrCurveList.n_crdrpts-1 ) {
			newData.push_back(*pC1);
		}
		else {
			TCrdrCurve *pC2 = &pCrdr->CrdrCurveList.pCrdrCurve[pt+1];

			CPoint2D p1(pC1->x, pC1->y);
			CPoint2D p2(pC2->x, pC2->y);

			if ( (p2-p1).Length() > 1e-3 ) {
				newData.push_back(*pC1);
			}
			else 
			if ( !warningPrinted && pC1->pCrdrLineInfo ) {
				fprintf(stderr, "Warning: "
"Corridor %s:%d->%s:%d at intersection %s \nhas duplicate points (at %d)"
"which were removed but line attributes may be lost\n",
				pCrdr->roadName1, pCrdr->crdrLane1, 
				pCrdr->roadName2, pCrdr->crdrLane2, pIntName, pt);
				warningPrinted = true;
			}
		}
	}

	// Now the newData vector contains the unique points.  Note
	// that the control points contain references to memory
	// which we don't free.  We are also copying the structures
	// without doing a "deep" copy but since these attributes
	// are read only and we don't particularly care for memory
	// leaks on the compiler, that should be ok.
	pt = 0;
	for (i=newData.begin(); i!= newData.end(); i++) {
		pCrdr->CrdrCurveList.pCrdrCurve[pt++] = *i;
	}
	pCrdr->CrdrCurveList.n_crdrpts = pt;
}



string GetAttrName( int id )
{
	for(int i = 0; i<gAttrDict.nEntries; i++){
		if (id == gAttrDict.pList[i].id){
			return gAttrDict.pList[i].pName;
		}
	}
	return "";
}



float GetRoadWidth( const TRoads& road )
{
	float width = 0;
	for (int n=0; n < road.lanelist.n_lanes; n++) {
		width += road.lanelist.lanes[n].width;
	}
//	printf("Width is %f\n", width );
	return width;
}

float GetRoadWidth( const string& name, TRoads* pRoads ) {
	TRoads * curr_rd_top; 
	for ( curr_rd_top = pRoads; curr_rd_top != NULL;
				curr_rd_top = curr_rd_top->pNext ) {
		if (!strcmp( curr_rd_top->roadName, name.c_str() )) {
			return GetRoadWidth( *curr_rd_top );
		}
	}
	return -1;
}


float GetEndOfRoadWidth( const TRoads& road, float* pWidths = NULL )
{
	float widths[cCV_MAX_LANES];
	float* pStoreWidths;
	if (pWidths == NULL) pStoreWidths = widths;
	else pStoreWidths = pWidths;

	int numLanes = road.lanelist.n_lanes;

	int n = 0;
	for (; n < numLanes; n++) {
		pStoreWidths[n] = road.lanelist.lanes[n].width;
	}

	int cp = 0;
	for (; cp < road.longitCurve.n_cpoints; cp++) {
		TControlPoint& pt = road.longitCurve.pCpointList[cp];
		TCpointInfo* pInfo = pt.pCpointInfo;
		while (pInfo != NULL) {
			int lane = 0;
			for (; lane < pInfo->cpointWidth.n_lanes; lane++) {
				pStoreWidths[n] = pInfo->cpointWidth.lanewidth[lane];
			}
			for (TAttribute* pAttr = pInfo->pCpointAttribute; pAttr != NULL; pAttr = pAttr->pNext) {
				// if it's a lane width change
				if (!strcmp( GetAttrName( pAttr->inum ).c_str(), "Width") ) {
					int laneNo = 1;
					for(lane  = 0; lane < numLanes; lane++){
						if ((pAttr->laneMask && laneNo)|| pAttr->laneMask == -1){
							pStoreWidths[lane] = pAttr->attrNum1;
						}
						laneNo *=2;
					}
				}
			}
			pInfo = pInfo->pNext;
		}
	}

	float width = 0;
	for (n = 0; n < numLanes; n++) {
		width += pStoreWidths[n];
	}

	return width;
}

float GetEndOfRoadWidth( const string& name, TRoads* pRoads, float* pWidths = NULL ) {
	TRoads * curr_rd_top; 
	for ( curr_rd_top = pRoads; curr_rd_top != NULL;
				curr_rd_top = curr_rd_top->pNext ) {
		if (!strcmp( curr_rd_top->roadName, name.c_str() )) {
			return GetEndOfRoadWidth( *curr_rd_top, pWidths );
		}
	}
	return -1;
}

TPt
GetEndOfRoadLane( char* pRoad, int lane, TRoads* pRoads )
{
	TPt retPt;
	retPt.x = retPt.y = retPt.z = 0.0;
	
	TRoads * curr_rd_top; 
	for ( curr_rd_top = pRoads; curr_rd_top != NULL;
				curr_rd_top = curr_rd_top->pNext ) {
		if (!strcmp( curr_rd_top->roadName, pRoad )) {
			TCpointList& pointList = curr_rd_top->longitCurve;
			int numPts = pointList.n_cpoints;
			for (int x=0 ; x < numPts; x++) {
				TControlPoint& cp = pointList.pCpointList[x];
//				printf("Cpt %d: (%f, %f, %f)\n", x, cp.x, cp.y, cp.z );
				for (CpointInfo* pInfo = cp.pCpointInfo; pInfo != NULL && pInfo->pNext != NULL; pInfo = pInfo->pNext) {
//					printf("  Width %f\n", pInfo->cpointWidth.lanewidth[lane] );
				}
			}
			if (numPts > 0) {
				TControlPoint* pPoint;
				TControlPoint* pOther;
				float x, y;
				int n;
				float offset;
				float lanes[cCV_MAX_LANES];
				switch (curr_rd_top->lanelist.lanes[lane].direction) {
				case 'p':
				case 'P':
					pPoint = &curr_rd_top->longitCurve.pCpointList[curr_rd_top->longitCurve.n_cpoints-1];
					pOther = &curr_rd_top->longitCurve.pCpointList[curr_rd_top->longitCurve.n_cpoints-2];
					x = pPoint->x - pOther->x;
					y = pPoint->y - pOther->y;
					offset = GetEndOfRoadWidth( *curr_rd_top, lanes ) / 2;
					for (n = 0; n < lane; n++) {
						offset -= lanes[n];
					}
					offset -= lanes[lane] / 2;
					break;
				case 'n':
				case 'N':
					pPoint = &curr_rd_top->longitCurve.pCpointList[0];
					pOther = &curr_rd_top->longitCurve.pCpointList[1];
					x = -(pPoint->x - pOther->x);
					y = -(pPoint->y - pOther->y);
					offset = GetRoadWidth( *curr_rd_top ) / 2;
					for (n=0; n < lane; n++) {
						offset -= curr_rd_top->lanelist.lanes[n].width;
					}
					offset -= curr_rd_top->lanelist.lanes[lane].width / 2;
					break;
				default:
					printf("Unknown direction for road %s, lane %d!\n", pRoad, lane );
					continue;
				}
				retPt.x = pPoint->x;
				retPt.y = pPoint->y;

				float dist = sqrt(x*x + y*y);
				x /= dist;
				y /= dist;

//				printf("Offset is %f\n", offset );
				retPt.x += offset * y;
				retPt.y += offset * -x;
//				printf("Road %s, lane %d: End at (%f, %f)\n", pRoad, lane, retPt.x, retPt.y );
				return retPt;
			}
			else {
				printf("Road %s, lane %d: No Points\n", pRoad, lane );
			}
		}
	}
	printf("Couldn't find road %s\n", pRoad );
	return retPt;
}


TPt
GetStartOfRoadLane( char* pRoad, int lane, TRoads* pRoads )
{
	TPt retPt;
	retPt.x = retPt.y = retPt.z = 0.0;
	
	TRoads * curr_rd_top; 
	for ( curr_rd_top = pRoads; curr_rd_top != NULL;
				curr_rd_top = curr_rd_top->pNext ) {
		if (!strcmp( curr_rd_top->roadName, pRoad )) {
			int numPts = curr_rd_top->longitCurve.n_cpoints;
			if (numPts > 0) {
				TControlPoint* pPoint;
				TControlPoint* pOther;
				float x, y;
				int n;
				float offset;
				float lanes[cCV_MAX_LANES];
				switch (curr_rd_top->lanelist.lanes[lane].direction) {
				case 'p':
					pPoint = &curr_rd_top->longitCurve.pCpointList[0];
					pOther = &curr_rd_top->longitCurve.pCpointList[1];
					x = -(pPoint->x - pOther->x);
					y = -(pPoint->y - pOther->y);
					offset = GetRoadWidth( *curr_rd_top ) / 2;
					for (n=0; n < lane; n++) {
						offset -= curr_rd_top->lanelist.lanes[n].width;
					}
					offset -= curr_rd_top->lanelist.lanes[lane].width / 2;
					break;
				case 'n':
					pPoint = &curr_rd_top->longitCurve.pCpointList[curr_rd_top->longitCurve.n_cpoints-1];
					pOther = &curr_rd_top->longitCurve.pCpointList[curr_rd_top->longitCurve.n_cpoints-2];
					x = pPoint->x - pOther->x;
					y = pPoint->y - pOther->y;
					offset = GetEndOfRoadWidth( *curr_rd_top, lanes ) / 2;
					for (n = 0; n < lane; n++) {
						offset -= lanes[n];
					}
					offset -= lanes[lane] / 2;
					break;
				default:
					printf("Unknown direction for road %s, lane %d!\n", pRoad, lane );
					continue;
				}
				retPt.x = pPoint->x;
				retPt.y = pPoint->y;

				float dist = sqrt(x*x + y*y);
				x /= dist;
				y /= dist;

//				printf("Offset is %f\n", offset );
				retPt.x += offset * y;
				retPt.y += offset * -x;
//				printf("Road %s, lane %d: Start at (%f, %f)\n", pRoad, lane, retPt.x, retPt.y );
				return retPt;
			}
			else {
				printf("Road %s, lane %d: No Points\n", pRoad, lane );
			}
		}
	}
	printf("Couldn't find road %s\n", pRoad );
	return retPt;
}

float Sqr( float x ) { return x*x; }
float Min( float x, float y ) { return x < y ? x : y ; }

float PointDist( const TPt& p1, const TPt& p2 )
{
	return sqrt(Sqr(p1.x-p2.x) + Sqr(p1.y-p2.y) + Sqr(p1.z-p2.z));
}


////////////////////////////////////////////////////////////////////
//
//
void SemanticCheck(
	TRoads *pRoads, 
	TIntersection *pInter,
	TElevMap *pElevMaps
)
{
	TRoads *        curr_rd_top; 
	TRoads *        curr_rd_other; 
	TIntersection * curr_int_top;
	TIntersection * curr_int_other;
	TCrdr *         curr_crdr_top;
	int num_roads=0,num_intrsxns=0;

	bool incorrectData = false;
	/* Check all roads for duplicate names and duplicate/invalid points */
	for ( curr_rd_top = pRoads; curr_rd_top != NULL; 
							curr_rd_top = curr_rd_top->pNext ) {
		TSplineData *pSpline;
		int num_sp_pts, new_num_sp_pts;

		num_roads++;
		if ( curr_rd_top->longitCurve.n_cpoints < 2 ) {
			fprintf(stderr, "Road '%s' needs at least two points\n", 
				pRoads->roadName);
			exit(-1);
		}
	
		/* Get all spline points */
		num_sp_pts = curr_rd_top->longitCurve.n_cpoints;
		pSpline = (TSplineData *)calloc( num_sp_pts, sizeof(TSplineData) );
		GetAllSplinePoints( pSpline, curr_rd_top->longitCurve.pCpointList,
							num_sp_pts );

		/* Remove all duplicate points */ 
		new_num_sp_pts = CorrectSplineData( pSpline, num_sp_pts, 
			curr_rd_top->roadName );
		if ( new_num_sp_pts < 0 ) {
			fprintf(stderr, "Road %s contains incorrect data.\n", 
				curr_rd_top->roadName);
			incorrectData = true;
		}

		/*
		 * Some points were removed so, copy the new ones back in 
     	 * to the linked list. 
     	 */
		if ( num_sp_pts > new_num_sp_pts && !incorrectData) {
			PutSplinePoints( num_sp_pts, new_num_sp_pts, pSpline, 
												curr_rd_top->longitCurve.pCpointList );	
			curr_rd_top->longitCurve.n_cpoints = new_num_sp_pts;
		}
#ifdef DEBUG_SEMCHECK
		printf("%s: %s --> %s\n",curr_rd_top->roadName,
				curr_rd_top->intersection1,curr_rd_top->intersection2);
#endif
		for ( curr_rd_other = curr_rd_top->pNext; curr_rd_other != NULL;
			curr_rd_other = curr_rd_other->pNext ) {
			if ( !strcmp( curr_rd_top->roadName, curr_rd_other->roadName ) ) {
				fprintf(stderr,"Error!!!! Exiting\n");
				fprintf(stderr,"%s is duplicate rdname\n",curr_rd_top->roadName);
				exit(-2);
			}
		}

		free( pSpline );	
	}
	if (incorrectData) {
		fprintf(stderr, "Qutting due to spline errors.\n" );
		exit(-1);
	}

	/* Check all intrsxns for duplicate names */
	for ( curr_int_top = pInter; curr_int_top != NULL; 
			curr_int_top = curr_int_top->pNext ) {
#ifdef DEBUG_SEMCHECK
		printf("%s\n",curr_int_top->name);
#endif
		num_intrsxns++;
		for ( curr_int_other = curr_int_top->pNext; curr_int_other != NULL;
			curr_int_other = curr_int_other->pNext ) {
			if ( !strcmp( curr_int_top->name, curr_int_other->name ) ) {
				fprintf(stderr,"Error!!!! Exiting\n");
				fprintf(stderr,"%s is duplicate int_name\n",curr_int_top->name);
				exit(-3);
			}
		}
	}

	vector<string> InterList1, InterList2;
	for (curr_int_top=pInter; curr_int_top; curr_int_top=curr_int_top->pNext){
		InterList1.push_back(curr_int_top->name);
	}

	/* Make sure all roads src and dest intersections exist */
	for (curr_rd_top=pRoads; curr_rd_top; curr_rd_top=curr_rd_top->pNext ) {
		string intrsxn1(curr_rd_top->intersection1);
		string intrsxn2(curr_rd_top->intersection2);

		vector<string>::const_iterator pI;

		bool found = false;
		for (pI=InterList1.begin(); pI != InterList1.end(); pI++) {
			if ( *pI == intrsxn1 ) {
				found = true;
				break;
			}
		}
		if ( !found ) {
			printf("**Cross reference check for road '%s'.\n",
				curr_rd_top->roadName);
			printf("  Cannot find intersectsion '%s'.\n", intrsxn1.c_str());
			exit(-1);
		}


		found = false;
		for (pI=InterList1.begin(); pI != InterList1.end(); pI++) {
			if ( *pI == intrsxn2 ) {
				found = true;
				break;
			}
		}
		if ( !found ) {
			printf("**Cross reference check for road '%s'.\n",
				curr_rd_top->roadName);
			printf("  Cannot find intersectsion '%s'.\n", intrsxn2.c_str());
			exit(-1);
		}

	}


	for ( curr_rd_top = pRoads; curr_rd_top != NULL;
				curr_rd_top = curr_rd_top->pNext ) {
		char i1[cMAX_WORDLENGTH], i2[cMAX_WORDLENGTH];
		strcpy(i1,curr_rd_top->intersection1);
		strcpy(i2,curr_rd_top->intersection2);
		if ( !(IntExists(i1,pInter,curr_rd_top->roadName) && 
				 IntExists(i2,pInter,curr_rd_top->roadName)) ) {
			printf("**Cross reference check for road '%s' failed.\n",
				curr_rd_top->roadName);
			printf("  Road not included in intersection '%s' or in '%s'.\n",
				i1, i2);
			exit( -11 );
		}
	}

	for ( curr_int_top = pInter; curr_int_top != NULL;
				curr_int_top = curr_int_top->pNext ) {
		TRoadName * rdname = curr_int_top->pRoadNames;
		for ( ; rdname != NULL; rdname = rdname->pNextRoadName ) {
			if ( !(RdExists(rdname->roadName,pRoads,curr_int_top->name)) ) {
				printf("**Cross reference check for intrxn '%s' failed.\n",
					curr_int_top->name);
				printf("  Cannot find road '%s' in road list.\n", 
					rdname->roadName);
				exit( -12 );
			} 
		} 
	}

	TIntersection *pIntrsctn;
	for ( pIntrsctn = pInter; pIntrsctn; pIntrsctn = pIntrsctn->pNext) {
		TCrdr  *pCrdr;
		for (pCrdr = pIntrsctn->pCrdr; pCrdr; pCrdr = pCrdr->pNext ) {
#if 1
			CheckCorridorData(pIntrsctn->name, pCrdr);
#endif
		}
	}


	/*  For each road, check to make sure that the source and destination
     *  intersections are in the correct order */
	for ( curr_rd_top = pRoads; curr_rd_top != NULL;
				curr_rd_top = curr_rd_top->pNext ) {
	}


	bool foundCrdrError = false;
	/*  For each corridor, make sure that the source and destination
     *  lanes are in the correct direction */
	for ( curr_int_top = pInter; curr_int_top != NULL;
				curr_int_top = curr_int_top->pNext ) {
		for (curr_crdr_top = curr_int_top->pCrdr; curr_crdr_top != NULL;
				curr_crdr_top = curr_crdr_top->pNext ) {
//printf("Crdr from %s:%d to %s:%d\n", curr_crdr_top->roadName1, 
//					curr_crdr_top->crdrLane1,
//					curr_crdr_top->roadName2, 
//					curr_crdr_top->crdrLane2);
			TPt srcPt = GetEndOfRoadLane( curr_crdr_top->roadName1,
					curr_crdr_top->crdrLane1, pRoads );
			TPt srcPtBack = GetStartOfRoadLane( curr_crdr_top->roadName1,
					curr_crdr_top->crdrLane1, pRoads );
			TPt destPt = GetStartOfRoadLane( curr_crdr_top->roadName2,
					curr_crdr_top->crdrLane2, pRoads );
			TPt destPtBack = GetEndOfRoadLane( curr_crdr_top->roadName2,
					curr_crdr_top->crdrLane2, pRoads );
			/* For now (?), ignore z values */
			srcPt.z = destPt.z = 0.0;
			if (curr_crdr_top->CrdrCurveList.n_crdrpts > 0) {
				TPt startCrdr;
				startCrdr.x = curr_crdr_top->CrdrCurveList.pCrdrCurve[0].x;
				startCrdr.y = curr_crdr_top->CrdrCurveList.pCrdrCurve[0].y;
				startCrdr.z = 0.0;

				TPt endCrdr;
				int numPts = curr_crdr_top->CrdrCurveList.n_crdrpts;
				endCrdr.x = curr_crdr_top->CrdrCurveList.pCrdrCurve[numPts-1].x;
				endCrdr.y = curr_crdr_top->CrdrCurveList.pCrdrCurve[numPts-1].y;
				endCrdr.z = 0.0;

				float s2s, s2e, e2s, e2e;
				double t;
				CPoint2D pt2;
				pt2.m_x = curr_crdr_top->CrdrCurveList.pCrdrCurve[1].x;
				pt2.m_y = curr_crdr_top->CrdrCurveList.pCrdrCurve[1].y;
				CLineSeg2D seg( startCrdr.x, startCrdr.y, pt2.m_x, pt2.m_y );
				s2s = (float)sqrt((float)seg.PerpDistSquare( srcPt.x, srcPt.y, &t ));
				t *= sqrt((float)(Sqr((float)(startCrdr.x - pt2.m_x)) + Sqr((float)(startCrdr.y - pt2.m_y))));
//				t = fabs(t);
				if (t < -gGapTolerance) {
					fprintf(stderr, "Intersection %s: Corridor from %s:%d to %s:%d has gap at source road (%f)\n",
									curr_int_top->name,
									curr_crdr_top->roadName1, 
									curr_crdr_top->crdrLane1,
									curr_crdr_top->roadName2, 
									curr_crdr_top->crdrLane2,
									t);
				} else if ((t < 0) && !gNoSplineCorrect) {
					// the gap is better than the tolerance but still negative,
					// so fix the gap by snapping the crdr point to the road point

					curr_crdr_top->CrdrCurveList.pCrdrCurve[0].x = srcPt.x;
					curr_crdr_top->CrdrCurveList.pCrdrCurve[0].y = srcPt.y;

				}
				s2e = PointDist( startCrdr, srcPtBack );
				e2s = PointDist( endCrdr,   destPtBack );
				pt2.m_x = curr_crdr_top->CrdrCurveList.pCrdrCurve[numPts-2].x;
				pt2.m_y = curr_crdr_top->CrdrCurveList.pCrdrCurve[numPts-2].y;
				seg = CLineSeg2D( endCrdr.x, endCrdr.y, pt2.m_x, pt2.m_y );
				e2e = (float)sqrt(seg.PerpDistSquare( destPt.x, destPt.y, &t ));
				t *= sqrt(Sqr(float(endCrdr.x - pt2.m_x)) + Sqr(float(endCrdr.y - pt2.m_y)));
//				t = fabs(t);
				if (t < -gGapTolerance) {
					fprintf(stderr, "Intersection %s: Corridor from %s:%d to %s:%d has gap at dest road (%f)\n",
									curr_int_top->name,
									curr_crdr_top->roadName1, 
									curr_crdr_top->crdrLane1,
									curr_crdr_top->roadName2, 
									curr_crdr_top->crdrLane2,
									t);
				} else if ((t < 0) && !gNoSplineCorrect) {
					// the gap is better than the tolerance but still negative,
					// so fix the gap by snapping the crdr point to the road point

					curr_crdr_top->CrdrCurveList.pCrdrCurve[numPts-1].x = destPt.x;
					curr_crdr_top->CrdrCurveList.pCrdrCurve[numPts-1].y = destPt.y;

				}
//				e2e = PointDist( endCrdr,   destPt );
//				printf("s2s: %f\ns2e: %f\ne2s: %f\ne2e: %f\n", s2s, s2e, e2s, e2e );
				if (gCrdrTolerance >= 0) {
					if (s2s > gCrdrTolerance) {
						if (s2e <= gCrdrTolerance) {
							foundCrdrError = true;
							fprintf(stderr, "Road %s appears to have it's source and dest intersections swapped\n",
									curr_crdr_top->roadName1);
						}
						else {
							foundCrdrError = true;
							fprintf(stderr, "Intersection %s: Corridor from %s:%d to %s:%d doesn't connect properly at source road\n",
									curr_int_top->name,
									curr_crdr_top->roadName1, 
									curr_crdr_top->crdrLane1,
									curr_crdr_top->roadName2, 
									curr_crdr_top->crdrLane2);
							fprintf(stderr, "(off by %f, tolerance is %f\n", s2s, gCrdrTolerance);
						}
					}
					if (e2e > gCrdrTolerance) {
						if (e2s <= gCrdrTolerance) {
							foundCrdrError = true;
							fprintf(stderr, "Road %s appears to have it's source and dest intersections swapped\n",
									curr_crdr_top->roadName2);
						}
						else {
							foundCrdrError = true;
							fprintf(stderr, "Intersection %s: Corridor from %s:%d to %s:%d doesn't connect properly at destination road\n",
									curr_int_top->name,
									curr_crdr_top->roadName1, 
									curr_crdr_top->crdrLane1,
									curr_crdr_top->roadName2, 
									curr_crdr_top->crdrLane2);
							fprintf(stderr, "(off by %f, tolerance is %f\n", e2e, gCrdrTolerance);
						}
					}
				}
			}
		}
	}
	if (foundCrdrError && (gCrdrTolerance >=0 )) {
		fprintf( stderr, "\n\n" );
		exit(-1);
	}

	if ( pElevMaps ) {
	}
}
