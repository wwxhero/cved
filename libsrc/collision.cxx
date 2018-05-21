//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: collision.cxx,v 1.15 2015/05/21 19:35:24 IOWA\dheitbri Exp $
//
// Author(s):   Lijen Tsao
// Date:        August, 2000
//
// Description: The implementation of CollisionDetection 
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"
#include "objreflistUtl.h"

#define DEBUG_COLLISION 0

#undef USE_NEAR_OBJS

namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: CollisionDetection
//  Provide functionality of collision detection
//
// Remarks: this function has to be provided with an id of a dynamic
//  object, and ids of objects thare are within the rectangle of the specified
//  object will be inserted in the provided STL vector in no specific order 
//
// Arguments:
//  objid - id of the queried dynamic object id
//  objIdVec - the container to hold the ids of the objects that are within
//   the rectangle the the specified obj
//  mask - object type mask indicating which object types to count. Defualt
//   value is all object types
//
// Returns: number of objects that are within the rectangle of the specified
//  dynamic object, or -1 if the specified object is not dynamic object
//
//////////////////////////////////////////////////////////////////////////////
int
CCved::CollisionDetection(
				int objId, 
				vector<int>& objIdVec, 
				CObjTypeMask mask) const
{
	if (objId >= cNUM_DYN_OBJS)
		return -1;

	objIdVec.clear();	
	///////////// for dynamic objects
	cvTObj* pObj = BindObj(objId);

#ifdef USE_NEAR_OBJS
	TPoint3D* pPos;
	if ((m_pHdr->frame & 1) == 0){
		pPos = &(pObj->stateBufA.state.anyState.position);
    } else {
		pPos = &(pObj->stateBufB.state.anyState.position);
    }
	CPoint3D objLoc( pPos->x, pPos->y, pPos->z );
	vector<int> objVec;
	GetObjsNear( objLoc, 50.0, objVec, mask );

	if (objVec.size() != 0)
	{
		vector<int>::const_iterator oIter;
		for( oIter = objVec.begin(); oIter != objVec.end(); oIter++ )
		{
#if DEBUG_COLLISION >= 3
			printf("Checking obj %d, ", *oIter);
#endif
			if( *oIter == objId ){
#if DEBUG_COLLISION >= 3
				printf("is itself\n");
#endif
				continue;
			}
			if( IfOverlap( objId, *oIter ) )
			{
#if DEBUG_COLLISION >= 3
				printf("overlapping, added to the list\n");
#endif
				objIdVec.push_back( *oIter );
			}
#if DEBUG_COLLISION >= 3
			else
				printf("not overlapping.\n");
#endif
		}
	}
#else
	TPoint3D* pBox;
	if ((m_pHdr->frame & 1) == 0){
        pBox = (pObj->stateBufA.state.anyState.boundBox);
    } else {
        pBox = (pObj->stateBufB.state.anyState.boundBox);
    }
	CBoundingBox bbox(pBox[0].x, pBox[0].y, pBox[1].x, pBox[1].y);

	set<int> objIdSet;	// the reason of using set is because
						// an dynamic obj might be on a road
						// and intersection at the time
	vector<int>::const_iterator oIter;
	vector<int> intrsctnVec;
	m_intrsctnQTree.SearchRectangle(bbox, intrsctnVec);
	if (intrsctnVec.size() != 0){
		vector<int>::const_iterator iIter = intrsctnVec.begin();
		for(; iIter != intrsctnVec.end(); iIter++){
			vector<int> objVec;
			bitset<cCV_MAX_CRDRS> crdrs;
			crdrs.set();
			objVec.clear();
#if DEBUG_COLLISION >= 3
			printf("Looking for objects on intersection %d\n", *iIter);
#endif
			GetAllDynObjsOnIntrsctn(*iIter, crdrs, objVec, mask);
			if (objVec.size() != 0){
				for (oIter = objVec.begin(); oIter != objVec.end(); oIter++){
#if DEBUG_COLLISION >= 3
					printf("Checking obj %d, ", *oIter);
#endif
					if (*oIter == objId){
#if DEBUG_COLLISION >= 3
						printf("is itself\n");
#endif
						continue;
					}
					if(IfOverlap(objId, *oIter)){
#if DEBUG_COLLISION >= 3
						printf("overlapping, added to the list\n");
#endif
						objIdSet.insert(*oIter);
					}
#if DEBUG_COLLISION >= 3
					else
						printf("not overlapping.\n");
#endif
				}
			}
		}
	}
	
	vector<int> rdpcsVec;
	set<int>    roadsUsed;

	m_rdPcQTree.SearchRectangle(bbox, rdpcsVec);
	if (rdpcsVec.size() != 0){
		vector<int>::const_iterator rIter = rdpcsVec.begin();
		for (; rIter != rdpcsVec.end(); rIter++){
			int roadId = BindRoadPiece(*rIter)->roadId;

			if ( roadsUsed.find( roadId ) == roadsUsed.end() ) {
				roadsUsed.insert( roadId );
			}
			else {
				continue;
			}

#if DEBUG_COLLISION >= 4
			printf("Looking for objects on road piece %d\n", roadId);
#endif

			vector<int> objVec;
			bitset<cCV_MAX_LANES> lanes;
			lanes.set();
			objVec.clear();
			GetAllDynObjsOnRoad(roadId, lanes, objVec, mask);
			if (objVec.size() != 0)
			{
				for( oIter = objVec.begin(); oIter != objVec.end(); oIter++ )
				{
#if DEBUG_COLLISION >= 4
					printf("Checking obj %d, ", *oIter);
#endif
					if( *oIter == objId ){
#if DEBUG_COLLISION >= 4
						printf("is itself\n");
#endif
						continue;
					}
					if( IfOverlap( objId, *oIter ) )
					{
#if DEBUG_COLLISION >= 4
						printf("overlapping, added to the list\n");
#endif
						objIdSet.insert( *oIter );
					}
#if DEBUG_COLLISION >= 4
					else
						printf("not overlapping.\n");
#endif
				}
			}
		}
	}

	set<int>::const_iterator osItr = objIdSet.begin();
	for(; osItr != objIdSet.end(); osItr++){
		objIdVec.push_back(*osItr);
	}

#if DEBUG_COLLISION >= 5
//	printf("Looking for LRI objects\n");
#endif
	///////////// for LRI static objects
	vector<int> soVec;
	m_staticObjQTree.SearchRectangle(bbox, soVec);
	if (soVec.size() != 0){
		vector<int>::const_iterator soIter = soVec.begin();
		for (; soIter != soVec.end(); soIter++){
#if DEBUG_COLLISION >= 5
			printf("Checking LRI obj %d type %d, ", *soIter, GetObjType(*soIter, *soIter));
#endif
			if (mask.Has(GetObjType(*soIter))){
				if (IfOverlap(objId, *soIter)){
#if DEBUG_COLLISION >= 5
					printf("overlapping, added to the list\n");
#endif
					objIdVec.push_back(*soIter);
				}
#if DEBUG_COLLISION >= 5
				else
					printf("not overlapping.\n");
#endif
			}
#if DEBUG_COLLISION >= 5
			else
				printf(" wrong type\n");
#endif
		}
	}

#if DEBUG_COLLISION >= 6
//	printf("Looking for non-LRI static objects\n");
#endif
	TU32b soIdx = m_pHdr->objectCountInitial;
	for	(; soIdx < m_pHdr->objectCount; soIdx++){
#if DEBUG_COLLISION >= 6
		printf("Checking non-LRI static obj %d type %d, ", soIdx, GetObjType(soIdx, *soIter));
#endif
		if (mask.Has(GetObjType(soIdx))){
			if (IfOverlap(objId, soIdx)){
				objIdVec.push_back(soIdx);
#if DEBUG_COLLISION >= 6
				printf("overlapping, added to the list\n");
#endif
			}
#if DEBUG_COLLISION >= 6
			else
				printf("not overlapping.\n");
#endif
		}
#if DEBUG_COLLISION >= 6
		else
			printf(" wrong type\n");
#endif
	}
#endif  // USE_NEAR_OBJS

	return (int) objIdVec.size();
}

////////////////////////////////////////////////////////////////////
//
// Description: IfOverlap
//  This function returns if the two objects specified by the two
//  parameters as their ids overlap
// Remarks:
//
// Arguments:
//  objId1: id of the first object
//  objId2: id of the second object 
//
// Returns: if the two objects overlap
//
/////////////////////////////////////////////////////////////////////

static void         
myUpdateBBox(const cvTObjAttr *cpAttr,
		const CVector3D &cTangent,
		const CVector3D &cLateral,
		const CPoint3D &cPosition,
		CPoint3D   &lowerLeft,
		CPoint3D   &upperRight)
{
	CVector3D  lat(cLateral);
	CVector3D  forw(cTangent);

	forw.Scale(0.5 * cpAttr->xSize);
	lat.Scale(0.5 * cpAttr->ySize);

	CPoint3D  corners[4];

	corners[0] = cPosition + forw + lat;
	corners[1] = cPosition + forw - lat;
	corners[2] = cPosition - forw - lat;
	corners[3] = cPosition - forw + lat;

	CPoint3D mn(corners[0]), mx(corners[0]);

	for (int i=1; i<4; i++) {
		if ( corners[i].m_x < mn.m_x ) mn.m_x = corners[i].m_x;
		if ( corners[i].m_y < mn.m_y ) mn.m_y = corners[i].m_y;
		if ( corners[i].m_z < mn.m_z ) mn.m_z = corners[i].m_z;

		if ( corners[i].m_x > mx.m_x ) mx.m_x = corners[i].m_x;
		if ( corners[i].m_y > mx.m_y ) mx.m_y = corners[i].m_y;
		if ( corners[i].m_z > mx.m_z ) mx.m_z = corners[i].m_z;
	}

	lowerLeft  = mn;
	upperRight = mx;
} // end of UpdateBBox


bool 
CCved::IfOverlap(int objId1, int objId2) const
{
	cvTObj* pObjPool = BindObj(0);	
	cvTObjState* pState1;
	if ((m_pHdr->frame & 1) == 0)
	{
        pState1 = &(pObjPool[objId1].stateBufA.state);
    } else {
        pState1 = &(pObjPool[objId1].stateBufB.state);
    }

	cvTObjState* pState2;
	if ((m_pHdr->frame & 1) == 0){
        pState2 = &(pObjPool[objId2].stateBufA.state);
    } else {
        pState2 = &(pObjPool[objId2].stateBufB.state);
    }

	CBoundingBox bbox1;
	double zmin1, zmax1;
	
	if ( objId1 == 0 || objId1 == 1 ) {	// if driver or trailer, use our own bbox calculation
		CPoint3D ll, ur;
		myUpdateBBox(&pObjPool[objId1].attr, pState1->anyState.tangent, pState1->anyState.lateral, pState1->anyState.position,
				ll, ur);
		bbox1.SetMinX(ll.m_x);
		bbox1.SetMinY(ll.m_y);
		bbox1.SetMaxX(ur.m_x);
		bbox1.SetMaxY(ur.m_y);
	}
	else {
		TPoint3D* pBox1 = (pState1->anyState.boundBox);
		bbox1.SetMinX(pBox1[0].x);
		bbox1.SetMinY(pBox1[0].y);
		bbox1.SetMaxX(pBox1[1].x);
		bbox1.SetMaxY(pBox1[1].y);
	}
	zmin1 = pState1->anyState.position.z;
	zmax1 = pState1->anyState.position.z+pObjPool[objId1].attr.zSize;

#if DEBUG_COLLISION >= 1
	if ( objId2 >= 350 && objId2 <= 351 )
	{
		printf( "Own obj #%d PS: %.2f, %.2f, %.2f\n", objId1, pState1->anyState.position.x, pState1->anyState.position.y, pState1->anyState.position.z);
		printf( "BB: %.2f, %.2f, %.2f -> %.2f, %.2f %.2f\n", bbox1.GetMinX(), bbox1.GetMinY(), zmin1, bbox1.GetMaxX(), bbox1.GetMaxY(), zmax1 );
	}
#endif

	TPoint3D* pBox2 = (pState2->anyState.boundBox);
	CBoundingBox bbox2(pBox2[0].x, pBox2[0].y, pBox2[1].x, pBox2[1].y);
	double zmin2 = pState2->anyState.position.z, 
		zmax2 = pState2->anyState.position.z+pObjPool[objId2].attr.zSize;

#if DEBUG_COLLISION >= 1
	if ( objId2 >= 350 && objId2 <= 351 )
	{
		printf( "Obj #%d PS: %.1f, %.1f, %.1f height %.1f\n", objId2, pState2->anyState.position.x, pState2->anyState.position.y, pState2->anyState.position.z, pObjPool[objId2].attr.zSize);
		printf( "BB: %.1f, %.1f, %.1f -> %.1f, %.1f %.1f\n", bbox2.GetMinX(), bbox2.GetMinY(), zmin2, bbox2.GetMaxX(), bbox2.GetMaxY(), zmax2 );
	}
#endif

	// collision check on z axis
	if ( zmin2 > zmax1 || zmin1 > zmax2 )
	{
#if DEBUG_COLLISION >= 1
		if ( objId2 < 600 )
			printf("Obj1 %d z(%f %f), obj2 %d z(%f %f), non overlapping\n", 
				objId1, zmin1, zmax1, objId2, zmin2, zmax2);
#endif
		return false;
	}

	// if two bounding boxes do not overlap, these two objects do not overlap
	if( !bbox1.Encloses( bbox2 ) )
	{
#if DEBUG_COLLISION >= 1
		if ( objId2 >= 350 && objId2 <= 351 )
		{
			printf("box1 does not enclose box2\n");
		}
#endif
		return false;
	}

	CPoint2D cornersObj1[4];
	CPoint2D cornersObj2[4];
	GetFourCornerOfObj(objId1, cornersObj1);
	GetFourCornerOfObj(objId2, cornersObj2);

#if DEBUG_COLLISION >= 1
	if ( objId2 >= 350 && objId2 <= 351 )
	{
		printf("box1 id %d corners (%.4f %.4f) (%.4f %.4f) (%.4f %.4f) (%.4f %.4f)\n",
			objId1,
			cornersObj1[0].m_x, cornersObj1[0].m_y,
			cornersObj1[1].m_x, cornersObj1[1].m_y,
			cornersObj1[2].m_x, cornersObj1[2].m_y,
			cornersObj1[3].m_x, cornersObj1[3].m_y);
		printf("box2 id %d corners (%.4f %.4f) (%.4f %.4f) (%.4f %.4f) (%.4f %.4f)\n",
			objId2,
			cornersObj2[0].m_x, cornersObj2[0].m_y,
			cornersObj2[1].m_x, cornersObj2[1].m_y,
			cornersObj2[2].m_x, cornersObj2[2].m_y,
			cornersObj2[3].m_x, cornersObj2[3].m_y);
	}
#endif

	bool haveCollision = RectangleIntersection( cornersObj1, cornersObj2 );

	if( haveCollision )
	{
		return true;
	}

#if DEBUG_COLLISION >= 1
	if ( objId2 >= 350 && objId2 <= 351 )
	{
		printf("boxes don't intersect\n");
	}
#endif
	return false;
}
/////////////////////////////////////////////////////////////////////////////
///\brief
///     Gets all the static objects enclosed within the bounding box
///\param[in] bbox the bounding box
///\param[out] objectIds
/////////////////////////////////////////////////////////////////////////////
void 
CCved::GetStaticObjectsInBoundingBox(const CBoundingBox &bbox,vector<int>& objIdVec) const{
	vector<int> soVec;
	CObjTypeMask mask;
    m_staticObjQTree.SearchRectangle(bbox, objIdVec);
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: QryTerrainOffRoad (private)
//	Find the elevation of the querried point. only called when it is off
//  road.	
//
// Remarks:
//
// Arguments:
//  pZ - output elevation of point on the static object
//  normal - output normal of point on the static object
//  cIn - point to query
//  pIfTrrnObjUsed - (optional) contains true if a terrain object was found
//      at cIn, false otherwise
//  pMaterial - (optional) contains a pointer to the material found at cIn.
//
// Returns: true if cIn falls on a terrain object
//
//////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
///\brief
///     Gets all the static objects enclosed within the bounding box
///\param[in] bbox the bounding box
///\param[out] objectIds
/////////////////////////////////////////////////////////////////////////////
void 
CCved::GetTerrianObjectsInBoundingBox(const CBoundingBox &bbox,vector<int>& objIdVec) const
{
	bool found = false;
	double maxElev = -100000.0;
	double elev = -100000.0;
	int	maxZobjId = 0;
	int maxZmaterial = 0;
	//int curObjId;
	CVector3D maxZnormal(0,0,1);
	//static bool sInit = false;
	/*static*/ vector<int> trrnObjs;
	//if (!sInit) {
	//	sInit = true;
		trrnObjs.reserve(10);
	//}
	trrnObjs.clear();

	// seasrch for the static objects specified in LRI
	m_trrnObjQTree.SearchRectangle(
		bbox, trrnObjs);
	if (trrnObjs.size() != 0){
		vector<int>::const_iterator idItr = trrnObjs.begin();
		for (; idItr != trrnObjs.end(); idItr++){
             objIdVec.push_back(*idItr);
		}
	} // for
    vector<int> statics;
	CObjTypeMask mask;
	mask.Set(eCV_TERRAIN);
    m_staticObjQTree.SearchRectangle(bbox, statics);
    cvTObj* pObjPool = BindObj(0);
    for (auto itr = statics.begin(); itr != statics.end(); itr++){
        if (pObjPool[*itr].type == eCV_TERRAIN){
            objIdVec.push_back(*itr);
        }
    }
}
} // namespace CVED
