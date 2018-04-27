#include "CvedDistri.h"
namespace CVED {

CCvedDistri::CCvedDistri(void)
{
}


CCvedDistri::~CCvedDistri(void)
{
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	This function returns all dynamic objects whose type is included in the
// 	specified mask.
//
// Remarks: The output is given in a set of integers that are the identifiers
// 	of all applicable objects.  The appropriate constructor or conversion
// 	operators can be used to obtain the appropriately typed object class.
//
// Arguments:
//	out - the container to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynamicObjs(TIntVec &out, CObjTypeMask mask) const
{
	int    i;
	TObj  *pO;
	out.clear();
	for (i=0, pO = BindObj(0); i<cNUM_DYN_OBJS; i++, pO++) {
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type, i) ) out.push_back(i);
		}
	}
} // end of GetAllDynamicObjs

//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns all objects whose type is included
//  in the specified mask and are located near the specified point.
//
// Remarks: The output is given in a set of integers that are the
//  identifiers of all applicable objects.  The appropriate constructor
//  or conversion operators can be used to obtain the appropriately typed
//  object class.
//
//	To be included in the output, the bounding box of an object should overlap
//	a circle whose center is at loc and has the given radius.
//
// Arguments:
//  cLoc - the location to search for objects
//  radius - how far around loc to search for objects
//	out - the STL vector to hold the object identifiers
//  mask - (optional) object type mask indicating which object types to get.
//  	Default value is all object types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetObjsNear(
			const CPoint3D& cLoc,
			double          radius,
			TIntVec&        out,
			CObjTypeMask    mask
			) const
{
	int       i;
	TObj*     pO;
	CPoint3D  objPos;
	double    radiusSquare = radius * radius;  // avoid square roots

	out.clear();
	pO = BindObj(0);
	CBoundingBox bbox(
					cLoc.m_x - radius,
					cLoc.m_y - radius,
					cLoc.m_x + radius,
					cLoc.m_y + radius);
	// for dynamic object
	i  = 0;
	for(; i < cNUM_DYN_OBJS; i++){
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type, i) ) {
				objPos = GetObjPos(i);
				if ( bbox.Encloses(objPos) ) {
					out.push_back(i);
				}
			}
		}
		pO++;
	}

	// for static objects in the LRI
	vector<int> soVec;
	m_staticObjQTree.SearchRectangle(bbox, soVec);
	if (soVec.size() != 0){
		vector<int>::const_iterator soIter = soVec.begin();
		for (; soIter != soVec.end(); soIter++){
			if (mask.Has(GetObjType(*soIter))){
				objPos = GetObjPos(*soIter);
				if (bbox.Encloses(objPos)){
					out.push_back(*soIter);
				}
			}
		}
	}

	// static objects created at runtime
	TU32b soIdx = m_pHdr->objectCountInitial;
	for	(; soIdx < m_pHdr->objectCount; soIdx++){
		if (mask.Has(GetObjType(soIdx))){
			objPos = GetObjPos(soIdx);
			if (bbox.Encloses(objPos)){
				out.push_back(soIdx);
			}
		}
	}


#if 0
	while ( i < m_pHdr->objectCount ) {
		if ( pO->phase == eALIVE || pO->phase == eDYING ) {
			if ( mask.Has(pO->type) ) {
				double distSquare;

				objPos = GetObjPos(i);
#endif
#if 0
				double x = fabs(objPos.m_x - cLoc.m_x);
				double y = fabs(objPos.m_y - cLoc.m_y);
				if ((x > radius) || (y > radius))
					continue;
#endif
#if 0
				distSquare = (objPos.m_x - cLoc.m_x) * (objPos.m_x - cLoc.m_x)
						+ (objPos.m_y - cLoc.m_y) * (objPos.m_y - cLoc.m_y);

				if ( distSquare <= radiusSquare ) {
					out.push_back(i);
				}
#endif
#if 0
				double diff = radiusSquare - (x*x);
				if (diff < 0){
					continue;
				}
				diff -= (y*y);
				if (diff >= 0){
					out.push_back(i);
				}
#endif
#if 0
			}
		}
		i++;
		pO++;
	}
#endif
} // GetObjsNear

//////////////////////////////////////////////////////////////////////////////
/// Description:
/// 	Places the IDs of all the dynamic objects on the given road and lanes and
/// 	of the given types into the integer vector.
///
/// Remarks: The results are sorted by distance.
///  This function accesses the data stored in the dynamic object referece
///	lists associated with each road reference.  This data is refreshed
///	during the Maintainer() function, and so the results of this query
///	are only accurate up to the last Maintainer() call.
///
/// Arguments:
///	roadId - index of the road which contains the objects of interest.
///	lanes - bitset of lane IDs which the objects may occupy
///	result - integer vector result of object IDs
///	cMask - (optional) object types allowed in result.  Default is all types.
///
/// Returns: void
///
///\todo This function does not clear out the result set first it needs to
///
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnRoad(
			int roadId,
			const bitset<cCV_MAX_LANES> &lanes,
			TIntVec& result,
			const CObjTypeMask cMask
			) const
{
	if (m_pRoadRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pRoadRefPool[roadId].objIdx;
	cvEObjType type;


#ifdef DEBUG_GET_ALL_DYNOBJS_ON_ROAD
	if( curIdx == 0 )
	{
		gout << " roadId = " << roadId << endl;
		gout << " no obj on this road " << endl;
	}
#endif

	while (curIdx != 0) {

		int objId = m_pDynObjRefPool[curIdx].objId;
		bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);

		int laneId;
		for( laneId = 0; laneId < cCV_MAX_LANES; laneId++ )
		{
			if( objLanes[laneId] )
			{

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_ROAD
				gout << " current obj " << objId  <<  " is on lane " << laneId << endl;
#endif

			}
		}

		objLanes &= lanes;

		// If the input lanes overlap with the lanes the object is on,
		//	Or if both the input and object lanes are 0, then
		//	the object is on the road and lane.
		if ((objLanes.any()) ||
		    ((objLanes.to_ulong() == 0) &&
			 (lanes.to_ulong() == 0))) {

			type = GetObjType(m_pDynObjRefPool[curIdx].objId);
			if (cMask.Has(type, m_pDynObjRefPool[curIdx].objId))
				result.push_back(m_pDynObjRefPool[curIdx].objId);
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}



} // end of GetAllDynObjsOnRoad

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on a given lane.
//
// Remarks:This function access the data stored in the dynamic object
//	referece lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	lane - A lane of a road.
//	result - vector of object ids with their distances.
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnLane (CLane lane,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const {

	result.clear();
	CRoad road = lane.GetRoad();
	int roadId = road.GetId();
	int curIdx = m_pRoadRefPool[roadId].objIdx;

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
	gout << " =========GetAllDynObjsOnLane==============" << endl;
	gout << " road name = " << road.GetName();
	gout << " lane id = " << lane.GetId() << endl;
#endif

	while (curIdx != 0)
	{
		int objId = m_pDynObjRefPool[curIdx].objId;
		cvEObjType objType = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = m.Has( objType, m_pDynObjRefPool[curIdx].objId );
		double objDist = m_pDynObjRefPool[curIdx].distance;

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
		gout << " current obj in the pool = " << objId << endl;
		gout << " value of inObjMask = " << inObjMask << endl;
		gout << " objDist = " << objDist << endl;
#endif


#if 1
		bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);

		int laneId;
		for( laneId = 0; laneId < cCV_MAX_LANES; laneId++ )
		{
			if( objLanes[laneId] )
			{

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
				gout << " current obj is on lane " << laneId << endl;
#endif

			}
		}


#endif

		if( inObjMask )
		{

			bitset<cCV_MAX_LANES> laneMask;
			laneMask.set( lane.GetRelativeId() );

			// Make sure the object is on the lane.
			objLanes &= laneMask;
			bool objOnNeededLane = (
									objLanes.any() ||
									objLanes.to_ulong() == 0 &&
									laneMask.to_ulong() == 0
									   );

			if( objOnNeededLane )
			{
				TObjWithDist node;
				node.objId = objId;
				node.dist = objDist;
				result.push_back( node );

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_LANE
				gout << " ******The current obj inserted is: " << objId << endl;
#endif

			}

		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}

}	// end of GetAllDynObjsOnLane
//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given road and
// 	lanes, between the given distances, and of the given types into the
// 	integer vector.
//
// Remarks: The results are sorted by distance.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	roadId - index of the road which contains the objects of interest.
//	lanes - bitset of lane IDs which the objects may occupy
//	startDist - starting distance along the road to search
//	endDist - end distance along the road to search
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								const CObjTypeMask cMask) const {

#ifdef _DEBUG
	cvTDynObjRef temp; //for the debugger........
    cvTRoadRef tempf;
#endif
    if (m_pRoadRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}
	int curIdx = m_pRoadRefPool[roadId].objIdx;
	cvEObjType type;
	while (curIdx != 0) {
		if ((m_pDynObjRefPool[curIdx].distance >= startDist) &&
			(m_pDynObjRefPool[curIdx].distance <= endDist)) {
			bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
			objLanes &= lanes;

			// If the input lanes overlap with the lanes the object is on,
			//	Or if both the input and object lanes are 0, then
			//	the object is on the road and lane.
			if ((objLanes.any()) ||
				((objLanes.to_ulong() == 0) &&
				 (lanes.to_ulong() == 0))) {

				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (cMask.Has(type, m_pDynObjRefPool[curIdx].objId))

				//	gout << " *** checking inserted obj id = ";
				//	gout << m_pDynObjRefPool[curIdx].objId << endl;
				//	gout << " *** checking inserted obj dist = ";
				//	gout << m_pDynObjRefPool[curIdx].distance << endl;

					result.push_back(m_pDynObjRefPool[curIdx].objId);
			}
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnRoadRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given road and
// 	lanes, between the given distances, and of the given types into the
// 	integer vector.  Also returns a bitset set with the ids of the corridors
// 	that overlap objects.
//
// Remarks: The results are sorted by distance.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	roadId - index of the road which contains the objects of interest.
//	lanes - bitset of lane IDs which the objects may occupy
//	startDist - starting distance along the road to search
//	endDist - end distance along the road to search
//	result - integer vector result of object IDs
//	usedLanes - Will contain the ids of the lanes that overlap objects
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnRoadRange(int roadId,
								const bitset<cCV_MAX_LANES> &lanes,
								double startDist,
								double endDist,
								TIntVec& result,
								bitset<cCV_MAX_LANES>& usedLanes,
								const CObjTypeMask cMask) const
{
	if (m_pRoadRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pRoadRefPool[roadId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		if ((m_pDynObjRefPool[curIdx].distance >= startDist) &&
			(m_pDynObjRefPool[curIdx].distance <= endDist)) {

			usedLanes = (int) m_pDynObjRefPool[curIdx].lanes;
			usedLanes &= lanes;

			// If the input lanes overlap with the lanes the object is on,
			//	Or if both the input and object lanes are 0, then
			//	the object is on the road and lane.
			if ((usedLanes.any()) ||
				( (usedLanes.to_ulong() == 0) &&
				  (lanes.to_ulong() == 0) ) ) {

				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (cMask.Has(type, m_pDynObjRefPool[curIdx].objId))
					result.push_back(m_pDynObjRefPool[curIdx].objId);
			}
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnRoadRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridors and of the given types into the integer vector.
//
// Remarks: The results are sorted by distance.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrs - bitset of corridor IDs which the objects may occupy
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnIntrsctn (int intrsctnId,
								const bitset<cCV_MAX_CRDRS> &crdrs,
								TIntVec& result,
								const CObjTypeMask m) const
{

#ifdef DEBUG_GET_ALL_DYNOBJS
	gout << "=========GetAllDynObjsOnIntrsctn=============" << endl;
#endif

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {
		int objId = m_pDynObjRefPool[curIdx].objId;

#ifdef DEBUG_GET_ALL_DYNOBJS
		gout << "current objid = " << objId << endl;
#endif

		bitset<cCV_MAX_CRDRS> objCrdrs((int)(int)m_pDynObjRefPool[curIdx].corridors);
		int crdrId;
		for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
		{
			if( objCrdrs[crdrId] )
			{

#ifdef DEBUG_GET_ALL_DYNOBJS
				gout << " current obj is on crdr " << crdrId << endl;
#endif

			}
		}



		objCrdrs &= crdrs;

		// If the input crdrs overlap with the crdrs the object is on,
		//	Or if both the input and object crdrs are 0, then
		//	the object is on the intrsctn and crdrs.
		if ((objCrdrs.any()) ||
		    ((objCrdrs.to_ulong() == 0) &&
			 (crdrs.to_ulong() == 0))) {

				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (m.Has(type, m_pDynObjRefPool[curIdx].objId))
				{
					// If an object appears more than once in the internal list,
					// only insert it once.
			//		TIntVec::iterator i= find(
			//								result.begin(),
			//								result.end(),
			//								m_pDynObjRefPool[curIdx].objId
			//								);
			//		if( i == result.end() )
			//		{
						result.push_back(m_pDynObjRefPool[curIdx].objId);
			//		}
				}
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridors and of the given types into the integer vector.
//
// Remarks: The results are sorted by distance along each corridor.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrs - bitset of corridor IDs which the objects may occupy
//	startDist - array of starting distances along the crdr to search
//	endDist - array of end distances along the crdr to search
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnIntrsctnRange (
			int intrsctnId,
			const bitset<cCV_MAX_CRDRS> &crdrs,
			const double startDist[cCV_MAX_CRDRS],
			const double endDist[cCV_MAX_CRDRS],
			TIntVec& result,
			const CObjTypeMask m
			) const
{

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	vector<TObjWithDist> tempVec;
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
		objCrdrs &= crdrs;

		// If the input crdrs overlap with the crdrs the object is on,
		//	Or if both the input and object crdrs are 0, then
		//	the object is on the intrsctn and crdrs.
		if ((objCrdrs.any()) ||
		    ((objCrdrs.to_ulong() == 0) &&
			 (crdrs.to_ulong() == 0))) {

			// For each corridor in the mask
			for (int crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++) {

				// If the object lies on the current corridor
				if (objCrdrs[crdrId]) {

					// If object lies within the distance range
					if ((m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
							>= startDist[crdrId]) &&
						(m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
						 	<= endDist[crdrId])) {

						// If the object's type is in the mask
						type = GetObjType(m_pDynObjRefPool[curIdx].objId);
						if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {
							// Add the object id to the list

						//	gout << " *** checking inserted obj id = ";
						//	gout << m_pDynObjRefPool[curIdx].objId << endl;
						//	gout << " *** checking inserted obj dist = ";
						//	gout << m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
						//	gout << endl;

							TObjWithDist node;
							node.objId = m_pDynObjRefPool[curIdx].objId;
							node.dist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
							tempVec.push_back(node);

						//	result.push_back(m_pDynObjRefPool[curIdx].objId);

							// Don't want to insert the same object twice, so
							// 	break out of this for-loop
							crdrId = cCV_MAX_CRDRS;
						}
					} // If object lies within the distance range
				} // If the object lies on the current corridor
			} // For each corridor in the mask
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}

	// sort the objs
	TObjWithDist hold;
	TU32b k, loc;
	for( k =1; k < tempVec.size(); k++ )
	{
		hold = tempVec[k];
		loc = k;
		while( 0 < loc && ( hold.dist < tempVec[loc-1].dist ) )
		{
			tempVec[loc] = tempVec[loc-1];
			loc--;
		}
		tempVec[loc] = hold;
	}

	vector<TObjWithDist>::iterator i;
	for( i = tempVec.begin(); i != tempVec.end(); i++ )
	{
		result.push_back( (*i).objId );

	}

} // end of GetAllDynObjsOnIntrsctnRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridors and of the given types into the integer vector.  Also returns
// 	a bitset containing the ids of the corridors with objects on them.
//
// Remarks: The results are sorted by distance along each corridor.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrs - bitset of corridor IDs which the objects may occupy
//	startDist - array of starting distances for the corridors in crdr
//	endDist - array of ending distances for the corridors in crdr
//	result - integer vector result of object IDs
//	crdrsUsed - bitset containing the corridors with objects on them
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnIntrsctnRange (
			int intrsctnId,
			const bitset<cCV_MAX_CRDRS> &crdrs,
			const double startDist[cCV_MAX_CRDRS],
			const double endDist[cCV_MAX_CRDRS],
			TIntVec& result,
			bitset<cCV_MAX_CRDRS>& usedCrdrs,
			const CObjTypeMask m
			) const
{

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
		objCrdrs &= crdrs;

		// If the input crdrs overlap with the crdrs the object is on,
		//	Or if both the input and object crdrs are 0, then
		//	the object is on the intrsctn and crdrs.
		if ((objCrdrs.any()) ||
		    ((objCrdrs.to_ulong() == 0) &&
			 (crdrs.to_ulong() == 0))) {

			// For each corridor in the mask
			for (int crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++) {

				// If the object lies on the current corridor
				if (objCrdrs[crdrId]) {

					// If object lies within the distance range
					if ((m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
							>= startDist[crdrId]) &&
						(m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
						 	<= endDist[crdrId])) {

						// If the object's type is in the mask
						type = GetObjType(m_pDynObjRefPool[curIdx].objId);
						if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {
							// Add the object id to the list
							result.push_back(m_pDynObjRefPool[curIdx].objId);

							// Don't want to insert the same object twice, so
							// 	break out of this for-loop
							crdrId = cCV_MAX_CRDRS;
						}
					} // If object lies within the distance range
				} // If the object lies on the current corridor
			} // For each corridor in the mask
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}
} // end of GetAllDynObjsOnIntrsctnRange

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridor and of the given types into the output vector.
//
// Remarks:This function access the data stored in the dynamic object
//	referece lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrId - index of the corridor, with respect to the intersection.
//	result - vector of object ids with their distances.
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnCrdr (int intrsctnId,
							int crdrId,
							vector<TObjWithDist>& result,
							const CObjTypeMask m) const
{

	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	if( curIdx == 0 )
	{
#ifdef DEBUG_GET_ALL_DYNOBJS_ON_CRDR
		gout << " no obj in pool " << endl;
#endif
	}

	while (curIdx != 0) {

		int objId = m_pDynObjRefPool[curIdx].objId;
		double objDist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);

#ifdef DEBUG_GET_ALL_DYNOBJS_ON_CRDR
		gout << " current obj in pool = " << objId << endl;
		gout << " obj dist = " << objDist << endl;
#endif

		// If the object lies on the current corridor
		if (objCrdrs[crdrId]) {

			// If the object's type is in the mask
			type = GetObjType(m_pDynObjRefPool[curIdx].objId);
			if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {

			// Add the object id to the list
			TObjWithDist node;
			node.objId = objId;
			node.dist = objDist;
			result.push_back( node );
			}

		} // If the object lies on the current corridor

		curIdx = m_pDynObjRefPool[curIdx].next;
	}	// while

} // end of GetAllDynObjsOnCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description:
// 	Places the IDs of all the dynamic objects on the given intersection and
// 	corridor and of the given types into the integer vector.
//
// Remarks: The results are sorted by distance along the corridor.
//  This function access the data stored in the dynamic object referece
//	lists associated with each road reference.  This data is refreshed
//	during the Maintainer() function, and so the results of this query
//	are only accurate up to the last Maintainer() call.
//
// Arguments:
//	intrsctnId - index of the intersection to search.
//	crdrId - relative id of the corridor, with respect to the intersection.
//	startDist - starting distance along the crdr to search
//	endDist - end distance along the crdr to search
//	result - integer vector result of object IDs
//	cMask - (optional) object types allowed in result.  Default is all types.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::GetAllDynObjsOnCrdrRange (int intrsctnId,
								 int crdrId,
								 double startDist,
								 double endDist,
								 TIntVec& result,
								 const CObjTypeMask m) const {
	if (m_pIntrsctnRefPool == 0) {
		// EXIT: Maintainer has not been run yet.
		return;
	}

	vector<TObjWithDist> tempVec;
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	cvEObjType type;

	while (curIdx != 0) {

		bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);

		// If the object lies on the current corridor
		if (objCrdrs[crdrId]) {

			// If object lies within the distance range
			if ((m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
					>= startDist) &&
				(m_pDynObjRefPool[curIdx].crdrDistances[crdrId]
				 	<= endDist)) {

				// If the object's type is in the mask
				type = GetObjType(m_pDynObjRefPool[curIdx].objId);
				if (m.Has(type, m_pDynObjRefPool[curIdx].objId)) {
					// Add the object id to the list

				//	gout << " *** checking inserted obj id = ";
				//	gout << m_pDynObjRefPool[curIdx].objId << endl;
				//	gout << " *** checking inserted obj dist = ";
				//	gout << m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
				//	gout << endl;

				//	result.push_back(m_pDynObjRefPool[curIdx].objId);

					TObjWithDist node;
					node.objId = m_pDynObjRefPool[curIdx].objId;
					node.dist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];

					tempVec.push_back(node);

					// Don't want to insert the same object twice, so
					// 	break out of this for-loop
				//	 crdrId = cCV_MAX_CRDRS;	// comment in (1-15-04)
				}
			} // If the object lies within the distance range
		} // If the object lies on the current corridor

		curIdx = m_pDynObjRefPool[curIdx].next;
	}

	// sort the objs
	TObjWithDist hold;
	TU32b k, loc;
	for( k =1; k < tempVec.size(); k++ )
	{
		hold = tempVec[k];
		loc = k;
		while( 0 < loc && ( hold.dist < tempVec[loc-1].dist ) )
		{
			tempVec[loc] = tempVec[loc-1];
			loc--;
		}
		tempVec[loc] = hold;
	}

	vector<TObjWithDist>::iterator i;
	for( i = tempVec.begin(); i != tempVec.end(); i++ )
	{
		result.push_back( (*i).objId );

	}

} // end of GetAllDynObjsOnIntrsctnRange

///////////////////////////////////////////////////////////////////////////////
///
/// Description:	This function creates a list of objects in front of the
///   specified object in the parameter.
///
/// Remarks:
///
/// Arguments:
///   ownerObjId - Cved id of the owner object.
///   aPath   - The current path of the specified object in the parameter.
///   roadPos - The current roadPos of the specified object in the parameter.
///   maxObjs - The maximum number of objects the user is interested in
///             looking at.
///   fwdObjs - (output) Contains the objects sorted by distance.
///
/// Returns: void
///
///////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::BuildFwdObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& fwdObjs
			)
{

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
	gout << "[" << ownerObjId << "] == BuildFwdObjList ======================";
	gout << endl;
	gout << "The maximum number of objs is: " << maxObjs << endl;
#endif

	//
	// Error checking.
	//
	bool invalidPath = !path.IsValid() || path.Size() <= 0;
	if( invalidPath )
	{

		cerr << "CCvedDistri::BuildFwdObjList: invalid path....exit" << endl;
		return;

	}

#if 0
	gout << "  input path [size = " << path.Size() << "]" << endl;
	gout << path << endl;
#endif

	//
	// Initialize variables.
	//
	double ownerDist = -101.0;
	double distTraveledSoFar = 0.0;
	bool reachedFirstValidPathPoint = false;

	//
	// Iterate through all of the path points to build the forward
	// object list.
	//
	CPath::cTPathIterator pathItr;
	for( pathItr = path.Begin(); pathItr != path.End(); pathItr++ )
	{

		//
		// Skip all path points until reaching the one that is
		// the same as the given road position.
		//
		bool samePathPointAsOwner = false;
		if ( !reachedFirstValidPathPoint )
		{
			if ( path.GetPoint(pathItr).IsRoad() )
			{
				reachedFirstValidPathPoint = (
							roadPos.IsRoad() &&
							roadPos.GetRoad() == path.GetPoint(pathItr).GetRoad()
							);
			}
			else
			{
				reachedFirstValidPathPoint = (
							!roadPos.IsRoad() &&
							roadPos.GetIntrsctn().GetId() ==
							path.GetPoint(pathItr).GetIntrsctn().GetId()
							);
			}
			samePathPointAsOwner = reachedFirstValidPathPoint;

			//
			// Get the owner's distance from the list.  We cannot use
			// the dist from the given roadPos because that's newer
			// than dist for all the other objects stored in the lists.
			//
			if( samePathPointAsOwner )
			{
				int curIdx;
				if( roadPos.IsRoad() )
				{
					int currRoadId = roadPos.GetRoad().GetId();
					curIdx = m_pRoadRefPool[currRoadId].objIdx;

					while( curIdx != 0 )
					{
						if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
						{
							ownerDist = m_pDynObjRefPool[curIdx].distance;
							break;
						}

						curIdx = m_pDynObjRefPool[curIdx].next;
					}

				}
				else
				{
					int currIntrsctnId = roadPos.GetIntrsctn().GetId();
					curIdx = m_pIntrsctnRefPool[currIntrsctnId].objIdx;

					while( curIdx != 0 )
					{
						if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
						{
							int crdrId = roadPos.GetCorridor().GetRelativeId();
							ownerDist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
							break;
						}

						curIdx = m_pDynObjRefPool[curIdx].next;
					}

				}

				if( ownerDist < -100.0 )
				{
					//cerr << "CCvedDistri::BuildFwdObjList: unable to determine ";
					//cerr << "owner object " << ownerObjId << " distance.  Using ";
					//cerr << "dist from given roadPos" << endl;
					ownerDist = roadPos.GetDistance();
				}
			}
		}

		if ( !reachedFirstValidPathPoint )  continue;

		//
		// If the pathpoint in on road, it means the roadPos is on road.
		// Then get the road lanemask.
		//
		if( path.GetPoint(pathItr).IsRoad() )
		{

			//
			// Looking for objects on road.
			//
			bitset<cCV_MAX_CRDRS> tmpLaneMask = path.GetPoint(pathItr).GetLaneCrdrMask();
			bitset<cCV_MAX_LANES> laneMask =(int) tmpLaneMask.to_ulong();
			double startDist = path.GetPoint(pathItr).GetStartDist();
			double endDist = path.GetPoint(pathItr).GetEndDist();
			CRoad currRoad = path.GetPoint(pathItr).GetRoad();
			if ( fabs( endDist - currRoad.GetLinearLength() ) < 1.0 )
			{
				// this is for cases where the object might have a value
				// slightly larger than the end of road when it's about to
				// step onto an intersection
				endDist += 20.0;
			}

			//
			// Get the direction of one of the lanes in the mask.  The
			// assumption is that all of the lanes must have the same
			// direction.
			//
			cvELnDir laneDir = ePOS;
			int i;
			for( i = 0; i < cCV_MAX_LANES; i++ )
			{
				if( laneMask[i] )
				{
					CLane aLane( currRoad, i );
					laneDir = aLane.GetDirection();
					break;
				}
			}


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  looking at road = " << currRoad << endl;
            gout << "    laneMask = " << laneMask;
			gout << "   start = "<< startDist;
			gout << "   end = " << endDist << endl;

#endif

			//
			// Insert objects into the fwd list until there are no
			// more objects on this road or we reach the maximum
			// number of objects needed.
			//
			int currRoadId = currRoad.GetId();
			int curIdx = m_pRoadRefPool[currRoadId].objIdx;

			while( curIdx != 0 && fwdObjs.size() < (TU32b)maxObjs )
			{
				//
				// Perform quick checks to make sure that this object is not
				// the owner object, part of the object mask and is located
				// within the needed distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
//				bool withinDist = (
//							m_pDynObjRefPool[curIdx].distance >= startDist &&
//							m_pDynObjRefPool[curIdx].distance <= endDist
//							);
				cvTObjAttr* pObjAttr = BindObjAttr( objId );
				double objLength = pObjAttr->xSize;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
				gout << "    looking at obj " << objId << "   dist = ";
				gout << m_pDynObjRefPool[curIdx].distance << endl;
#endif

				//
				// An object should be placed on the owner's forward list as long
				// as any part of the object is ahead of the owner's center.
				//
				bool withinDist = true;
				if ( samePathPointAsOwner )
				{
					if( laneDir == ePOS )
					{
						double objAdjustedDist =
								m_pDynObjRefPool[curIdx].distance + (objLength * 0.5);
						withinDist = (
								objAdjustedDist >= ownerDist &&
								m_pDynObjRefPool[curIdx].distance <= endDist
								);
#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "      pos lane   objAdjustedDist = " << objAdjustedDist;
						gout << "   ownerDist = " << ownerDist << endl;
#endif
					}
					else
					{
						double objAdjustedDist =
								m_pDynObjRefPool[curIdx].distance - (objLength * 0.5);
						withinDist = (
								objAdjustedDist <= ownerDist &&
								m_pDynObjRefPool[curIdx].distance >= endDist
								);
#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "      neg lane   objAdjustedDist = " << objAdjustedDist;
						gout << "   ownerDist = " << ownerDist << endl;
#endif
					}

				}
				bool passedFirstTest = notOwnerObj && inObjMask && withinDist;


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
				gout << "*********ON Road*********" << endl;
				gout << " notOwnerObj = " << notOwnerObj << endl;
				gout << " inObjMask = " << inObjMask << endl;
				gout << " withinDist = " << withinDist << endl;
				gout << " passedFirstTest = " << passedFirstTest << endl;
#endif

				if( passedFirstTest )
				{

					//
					// If the input lanes overlap with the lanes the object is on,
					// or if both the input and object lanes are 0, then the object
					// is on the road and lane.
					//
					bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
					objLanes &= laneMask;
					bool objOnNeededLanes = (
								objLanes.any() ||
								( objLanes.to_ulong() == 0 && laneMask.to_ulong() == 0 )
								);
					if( objOnNeededLanes )
					{
						//
						// Compute the distance from the owner object to this object.
						//
						double distFromOwner;
						if( samePathPointAsOwner )
						{
							distFromOwner = fabs(
												ownerDist -
												m_pDynObjRefPool[curIdx].distance
												);
						}
						else {
							if ( laneDir == ePOS )
							{
								distFromOwner = (
											distTraveledSoFar +
											m_pDynObjRefPool[curIdx].distance
											);
							}
							else
							{
								distFromOwner = (
											distTraveledSoFar +
											currRoad.GetLinearLength() -
											m_pDynObjRefPool[curIdx].distance
											);
							}
						}

						//
						// Insert object into main list.
						//
						TObjListInfo node;
						node.objId = objId;
						node.distFromOwner = distFromOwner;
						node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "    **inserting " << node.objId;
						gout << " with dist = " << node.distFromOwner;
						gout << endl;
#endif

						fwdObjs.push_back( node );

					}
				}

				curIdx = m_pDynObjRefPool[curIdx].next;
			}

			if ( samePathPointAsOwner )
			{
				if ( roadPos.GetLane().GetDirection() == ePOS )
				{
					distTraveledSoFar += (
								currRoad.GetLinearLength() -
								ownerDist
								);
				}
				else
				{
					distTraveledSoFar += ownerDist;
				}
			}
			else
			{
				distTraveledSoFar += currRoad.GetLinearLength();
			}

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  distTraveledSoFar = " << distTraveledSoFar << endl;
#endif

		}
		else
		{

			//
			// Looking for objects on intersection.
			//

			//
			// On intersection, get the owner distance from the roadPos.
			//
		//	ownerDist = roadPos.GetDistance();

			bitset<cCV_MAX_CRDRS> crdrMask = path.GetPoint(pathItr).GetLaneCrdrMask();
			CIntrsctn currIntrsctn = path.GetPoint(pathItr).GetIntrsctn();

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  looking at intrsctn = " << currIntrsctn << endl;
            gout << "    crdrMask = " << crdrMask << endl;
#endif

			int intrsctnId = currIntrsctn.GetId();
			const double* cpStartDists = path.GetPoint(pathItr).GetStartDists();
			const double* cpEndDists = path.GetPoint(pathItr).GetEndDists();
			TIntVec dynObjs;

			//
			// Insert objects into the fwd list until there are no
			// more objects on this road or we reach the maximum
			// number of objects needed.
			//
			int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
			double maxCrdrDist = 0.0;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << " fwdObjs current size = " << fwdObjs.size() << endl;
			if( curIdx == 0 )
			{
				gout << " curIdx = 0 ...obj not reported in internal list. " << endl;
				gout << " size of fwdObjs = " << fwdObjs.size() << endl;
				gout << " maxObjs = " << maxObjs << endl;
			}
#endif

			while( curIdx != 0 && fwdObjs.size() < (TU32b)maxObjs )
			{

				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
				bool passedFirstTest = notOwnerObj && inObjMask;


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
				gout << "*********ON Intersection*********" << endl;
				gout << " looking at obj " << objId << endl;
				gout << " notOwnerObj = " << notOwnerObj << endl;
				gout << " inObjMask = " << inObjMask << endl;
				gout << " passedFirstTest = " << passedFirstTest << endl;
#endif


				if( passedFirstTest )
				{
					bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
					objCrdrs &= crdrMask;

					//
					// If the input crdrs overlap with the crdrs the object is on,
					// or if both the input and object crdrs are 0, then the object
					// is on the intrsctn and crdrs.
					//
					bool objOnNeededCrdrs = (
									objCrdrs.any() ||
									( objCrdrs.to_ulong() == 0 && crdrMask.to_ulong() == 0 )
									);
#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << " ********The value of objOnNeededCrdr is: " << objOnNeededCrdrs << endl;
#endif

					if( objOnNeededCrdrs )
					{

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << " The obj is on needed corridors. " << endl;
#endif

						int crdrId;
						for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
						{
							// if the object lies on the current corridor
							if ( objCrdrs[crdrId] )
							{

								// if object lies within the distance range
								CCrdr currCrdr( currIntrsctn, crdrId );
								double startDist = cpStartDists[crdrId];
								double endDist = cpEndDists[crdrId];
								if ( fabs( endDist - currCrdr.GetLength() ) < 1.0 )
								{
									// this is for cases where the object might have a value
									// slightly larger than the end of road when it's about to
									// step onto an intersection
									endDist += 15.0;
								}


								// Use obj dist from obj pool.
								double objDistAlongCrdr =
											m_pDynObjRefPool[curIdx].crdrDistances[crdrId];



#ifdef DEBUG_BUILD_FWD_OBJ_LIST
								gout << " $$$$$$$$ checking obj dist on intersection$$$$$$$$$$$$ ";
								gout << endl;
								gout << " $$$$$$$$ obj " << objId << " : dist from pool = " << objDistAlongCrdr;
								gout << " $$$$$$$$$$$$$ " << endl;
#endif

								bool withinDist = (
										objDistAlongCrdr >= startDist &&
										objDistAlongCrdr <= endDist
										);

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
								gout << "    start = " << startDist;
								gout << "   end = " << endDist << endl;
#endif

								//
								// If the owner and the object are on the same interesction,
								// then make sure that the object is really ahead of the
								// owner.
								//
								cvTObjAttr* pObjAttr = BindObjAttr( objId );
								double objLength = pObjAttr->xSize;
								bool objFwd = true;
								if ( samePathPointAsOwner )
								{
									objFwd = (
												objDistAlongCrdr  + (objLength * 0.5) >=
										ownerDist
										);
								}


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << " ******* withinDist =  " << withinDist << endl;
						gout << " ******* objFwd = " << objFwd << endl;
#endif


								if( withinDist && objFwd )
								{

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
									gout << " The obj is within distance. " << endl;
#endif
									//
									// Compute the distance from the owner object
									// to this object.
									//
									double distFromOwner;
									if( samePathPointAsOwner )
									{
										distFromOwner = fabs(
													ownerDist -
													objDistAlongCrdr
													);
									}
									else {
										distFromOwner = (
											distTraveledSoFar +
											objDistAlongCrdr
											);
									}
									if(  objDistAlongCrdr > maxCrdrDist )
									{
										maxCrdrDist = objDistAlongCrdr ;
									}

									TObjListInfo node;
									node.objId = objId;
									node.distFromOwner = distFromOwner;
									node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
									gout << "    **inserting " << node.objId;
									gout << " with dist = " << node.distFromOwner;
									gout << endl;
#endif


									fwdObjs.push_back( node );
									break;
								} // If object lies within the distance range
							} // If the object lies on the current corridor
						} // For each corridor in the mask
					}
				}

				curIdx = m_pDynObjRefPool[curIdx].next;
			}

			if( samePathPointAsOwner )
			{
				distTraveledSoFar += (
					roadPos.GetCorridor().GetLength() -
					ownerDist
					);
			}
			else
			{
				//
				// Get the length of one of the corridors in the intersection.
				int i;
				for( i = 0; i < cCV_MAX_CRDRS; i++ )
				{
					if( crdrMask[i] )
					{
						CCrdr aCrdr( currIntrsctn, i );
						distTraveledSoFar += aCrdr.GetLength();

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
						gout << "  crdr " << i << " has a length of ";
						gout << aCrdr.GetLength() << endl;
#endif

						break;
					}
				}
			}


#ifdef DEBUG_BUILD_FWD_OBJ_LIST
			gout << "  distTraveledSoFar = " << distTraveledSoFar << endl;
#endif
		}

		if ( fwdObjs.size() >= (TU32b)maxObjs )  break;
	}

	//
	// Sort the list by distance to the owner.
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < fwdObjs.size(); k++ )
	{
		hold = fwdObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < fwdObjs[loc-1].distFromOwner ) )
		{
			fwdObjs[loc] = fwdObjs[loc-1];
			loc--;
		}
		fwdObjs[loc] = hold;
	}

#ifdef DEBUG_BUILD_FWD_OBJ_LIST
	gout << "  fwdObjs [size = " << fwdObjs.size() << "]: ";
	vector<TObjListInfo>::iterator i;
	for( i = fwdObjs.begin(); i != fwdObjs.end(); i++ )
	{
		gout << " " << i->objId << " [" << i->distFromOwner << "]";
	}
	gout << endl;
#endif

}	// end of BuildFwdObjList

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects behind the
//   specified object in the parameter.
//
// Remarks: For now, this function only looks at objects on the same
//   road as the owner object.
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos    - The current roadPos of the specified object in the
//                parameter.
//   maxObjs    - The maximum number of objects the user is interested in
//                looking at.
//   backObjs   - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::BuildBackObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& backObjs
			)
{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
	gout << "[" << ownerObjId << "] == BuildBackObjList =====================";
	gout << endl;
#endif

	//
	// No back object object list inside intersections for now.
	//
	if( !roadPos.IsRoad() )  return;

	double ownerDist = -101.0;
	CRoad currRoad = roadPos.GetRoad();
	CLane currLane = roadPos.GetLane();
	cvELnDir currLaneDir = currLane.GetDirection();

	{
		int curIdx = m_pRoadRefPool[currRoad.GetId()].objIdx;

		while( curIdx != 0 )
		{
			if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
			{
				ownerDist = m_pDynObjRefPool[curIdx].distance;
				break;
			}

			curIdx = m_pDynObjRefPool[curIdx].next;
		}

		if( ownerDist < -100.0 )
		{
			//cerr << "CCvedDistri::BuildBackObjList: unable to determine ";
			//cerr << "owner object " << ownerObjId << " distance.  Using ";
			//cerr << "dist from given roadPos" << endl;
			ownerDist = roadPos.GetDistance();
		}
	}


	//
	// Build a lane mask with all lanes traveling in the same direction
	// as the lane in the specified roadPos.
	//
	int laneId;
	int numLanes = currRoad.GetNumLanes();
	int firstLaneIdx = currRoad.GetLaneIdx();
	bitset<cCV_MAX_LANES> laneMask;
	for( laneId = 0; laneId < numLanes; laneId++ )
	{
		cvTLane* pLane = BindLane( firstLaneIdx + laneId );
		if( pLane->direction == currLaneDir )
		{
			laneMask.set( laneId );
		}
	}

	//
	// Compute the distances at which to start and end looking for
	// objects on this road.
	//
	double startDist;
	double endDist;
	if( currLaneDir == ePOS )
	{
		startDist = 0.0;
		endDist = ownerDist;
	}
	else
	{
		startDist = ownerDist;
		endDist = currRoad.GetLinearLength();
	}

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
	gout << "  looking at road = " << currRoad << endl;
    gout << "    laneMask = " << laneMask;
	gout << "   start = "<< startDist;
	gout << "   end = " << endDist << endl;

#endif

	//
	// Insert objects into the fwd list until there are no
	// more objects on this road or we reach the maximum
	// number of objects needed.
	//
	int currRoadId = currRoad.GetId();
	int curIdx = m_pRoadRefPool[currRoadId].objIdx;

	//
	// Build a temporary array that holds the indexes of
	// all of the object that lie on this road.
	//
	int tempObjArr[cNUM_DYN_OBJS];
	int tempObjArrSize = 0;
	while( curIdx != 0 )
	{
		tempObjArr[tempObjArrSize] = curIdx;
		tempObjArrSize++;
		curIdx = m_pDynObjRefPool[curIdx].next;
	}

	int i;
	for( i = tempObjArrSize - 1; i >= 0; i-- )
	{
		if ( (int)backObjs.size() >= maxObjs )  break;
		int curIdx = tempObjArr[i];

		//
		// Perform quick checks to make sure that this object is not
		// the owner object, part of the object mask and is located
		// within the needed distances.
		//
		int objId = m_pDynObjRefPool[curIdx].objId;
		bool notOwnerObj = ownerObjId != objId;
		cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
		bool withinDist = (
					m_pDynObjRefPool[curIdx].distance >= startDist &&
					m_pDynObjRefPool[curIdx].distance <= endDist
					);
		cvTObjAttr* pObjAttr = BindObjAttr( objId );
		double objLength = pObjAttr->xSize;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
		gout << "    looking at obj " << objId << "   dist = ";
		gout << m_pDynObjRefPool[curIdx].distance << endl;
#endif

		//
		// An object should be placed on the owner's back list as long
		// as any part of the object is behind the owner's center.
		//
		bool objBack = true;
		if( currLaneDir == ePOS )
		{
			objBack = (
				m_pDynObjRefPool[curIdx].distance - (objLength * 0.5) <=
				ownerDist
				);
		}
		else
		{
			objBack = (
				m_pDynObjRefPool[curIdx].distance + (objLength * 0.5) >=
				ownerDist
				);
		}

		bool passedFirstTest = notOwnerObj && inObjMask && withinDist && objBack;
		if( passedFirstTest )
		{

			//
			// If the input lanes overlap with the lanes the object is on,
			// or if both the input and object lanes are 0, then the object
			// is on the road and lane.
			//
			bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
			objLanes &= laneMask;
			bool objOnNeededLanes = (
						objLanes.any() ||
						( objLanes.to_ulong() == 0 && laneMask.to_ulong() == 0 )
						);
			if( objOnNeededLanes )
			{
				//
				// Compute the distance from the owner object to this object.
				//
				double distFromOwner = fabs(
										ownerDist -
										m_pDynObjRefPool[curIdx].distance
										);
				//
				// Insert object into main list.
				//
				TObjListInfo node;
				node.objId = objId;
				node.distFromOwner = distFromOwner;
				node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST
				gout << "    **inserting " << node.objId;
				gout << " with dist = " << node.distFromOwner;
				gout << endl;
#endif

				backObjs.push_back( node );
			}
		}

		//curIdx = m_pDynObjRefPool[curIdx].next;
	}

	//
	// Sort the list by distance to the owner.
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < backObjs.size(); k++ )
	{
		hold = backObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < backObjs[loc-1].distFromOwner ) )
		{
			backObjs[loc] = backObjs[loc-1];
			loc--;
		}
		backObjs[loc] = hold;
	}


}  // end of BuildBackObjList

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects that are on the oncoming
//	lane of the specified object in the parameter. Note that the oncoming lane
//	is the closest opposite direction lane with repect to the lane the owner object
//	is located on.
//
// Remarks:
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos - The current roadPos of the specified object in the parameter.
//   path   - The current path of the specified object in the parameter.
//   maxObjs - The maximum number of objects the user is interested in
//             looking at.
//   oncomObjs - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::BuildOncomingObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& oncomObjs
			)
{


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << endl;
		gout << " == BuildOncomingObjList====";
		gout << endl;
#endif


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
	gout << " The input path size is: " << path.Size() << endl;
	gout << " The path is: " << path;
#endif


	//
	// Error checking.
	//
	if ( !path.IsValid() || path.Size() <= 0)
	{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << "CCvedDistri::BuildOncomingObjList: invalid path....exit" << endl;
#endif

		return;

	}
	if( ! roadPos.IsValid() )
	{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << "CCvedDistri::BuildOncomingObjList: invalid road position....exit" << endl;
#endif

		return;

	}

	//
	// Initialize variables.
	//
	double ownerDist = - 101.0;
	double distTraveledSoFar = 0.0;
	bool reachedFirstValidPathPoint = false;

	//
	// Iterate through all of the path points to build the oncoming
	// object list.
	//

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
	gout << " This is just before the path point iteration. " << endl;
#endif

	CPath::cTPathIterator pathItr;
	for( pathItr = path.Begin(); pathItr != path.End(); pathItr++ )
	{
		double startDist = path.GetPoint(pathItr).GetStartDist();
		double endDist = path.GetPoint(pathItr).GetEndDist();

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << " This is the current path point: " << pathItr << endl;
		gout << " The start of the path point is: " << startDist << endl;
		gout << " The end of the path point is: " << endDist << endl;
#endif

		//
		// Skip all path points until reaching the one that is
		// the same as the given road position.
		//
		bool samePathPointAsOwner = false;
		if ( !reachedFirstValidPathPoint )
		{
			if ( path.GetPoint(pathItr).IsRoad() )
			{
				reachedFirstValidPathPoint = (
							roadPos.IsRoad() &&
							roadPos.GetRoad() == path.GetPoint(pathItr).GetRoad()
							);
			}
			else
			{
				reachedFirstValidPathPoint = (
							!roadPos.IsRoad() &&
							roadPos.GetIntrsctn().GetId() ==
							path.GetPoint(pathItr).GetIntrsctn().GetId()
							);
			}
			samePathPointAsOwner = reachedFirstValidPathPoint;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The value of samePathPointAsOwner is: " << samePathPointAsOwner << endl;
#endif

		}

		if ( !reachedFirstValidPathPoint )  continue;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
		gout << " Have found the first valid path point. " << endl;

#endif

		//
		// If the pathpoint is on road, need to find the closest lane that
		// moves in the opposite direction with respect to the owner object.
		// So keep moving left until the lane in the opposite direction is reached.
		//
		CLane closestOppositeDirLn;
		CLane leftLane;
		if( path.GetPoint(pathItr).IsRoad() )
		{
			CRoad currRoad = path.GetPoint(pathItr).GetRoad();

			if ( fabs( endDist - currRoad.GetLinearLength() ) < 1.0 )
			{
				// This is for cases where the object might have a value
				// slightly larger than the end of road when it's about to
				// step onto an intersection
				endDist += 20.0;
			}

			//
			// This returns the first lane of the road that
			// the path point is on.
			CLane currLane = path.GetPoint(pathItr).GetLane();

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The current lane from the path point is: "<< currLane.GetId() << endl;
			gout << " The current road name is: " << currRoad.GetName() << endl;
			gout << " The current road id is: " << currRoad.GetId() << endl;
#endif

			//
			// If the current lane is already the leftmost lane,
			// it means it is a one-way road, then no oncoming objects, return.
			//
			if ( currLane.IsLeftMost() )
			{
				return;

			}
			leftLane = currLane.GetLeft();

			if( !leftLane.IsValid() )
			{
				return;
			}
			else
			{
				cvELnDir leftLaneDir = leftLane.GetDirection();
				cvELnDir currLaneDir = currLane.GetDirection();

				bool sameDirLanes = leftLaneDir == currLaneDir;

				if ( sameDirLanes && leftLane.IsLeftMost() )
				{
					return;
				}

				while( sameDirLanes && !leftLane.IsLeftMost() )
				{

					leftLane = leftLane.GetLeft();
					if( !leftLane.IsValid() )
					{
						return;
					}
					leftLaneDir = leftLane.GetDirection();
					sameDirLanes = leftLaneDir == currLaneDir;
				}
				if ( !sameDirLanes )
				{
					closestOppositeDirLn = leftLane;
				}
			}

			if( ! closestOppositeDirLn.IsValid() )
			{
				return;
			}

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " Have found this closestOppositeDirLn: "<< closestOppositeDirLn.GetId() << endl;
#endif


			int currRoadId = currRoad.GetId();
			int curIdx = m_pRoadRefPool[currRoadId].objIdx;

			//
			// Get the owner distance from the list. The distance from the roadPos
			// is inaccurate. This step is only necessary when the obj is on the same
			// path point as the owner.
			//
			if( samePathPointAsOwner )
			{
				while( curIdx != 0 )
				{
					if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
					{
						ownerDist = m_pDynObjRefPool[curIdx].distance;
						break;
					}
					curIdx = m_pDynObjRefPool[curIdx].next;
				}
			}

			if( ownerDist < -100.0 )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
					gout << "CCvedDistri::BuildFwdObjList: unable to determine ";
					gout << "owner object # " << ownerObjId << " distance. " << endl;
					gout << " So using dist from given roadPos" << endl;
#endif
					ownerDist = roadPos.GetDistance();
			}

			curIdx = m_pRoadRefPool[currRoadId].objIdx;

			//
			// Insert objects into the oncoming list until there are no
			// more objects on this road or we reach the maximum
			// number of objects needed.
			//
			while( curIdx != 0 && (int)oncomObjs.size() < maxObjs )
			{
				//
				// Make sure that this object is not the owner object, part of the object mask,
				// on the closest opposite direction lane and is located within the needed
				// distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
				bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " The current object in the pool is: "<< objId << endl;
				gout << " The value of the correct obj type or not is: " << inObjMask << endl;
				gout << " The road id of the current obj is: " << m_pDynObjRefPool[curIdx].roadId << endl;
#endif


				if( passedFirstTest )
				{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
					gout << " This obj has passed the first test. " << endl;
#endif

					//
					// Make sure the objects to be returned is on the closest
					// opposite direction lane.
					bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
					bitset<cCV_MAX_LANES> closestOppositeDirLnMask;
					if( ! closestOppositeDirLn.IsValid() )
					{
						return;
					}
					closestOppositeDirLnMask.set( closestOppositeDirLn.GetRelativeId() );
					bool objOnNeededLanes = (
								( objLanes.to_ulong() == closestOppositeDirLnMask.to_ulong() ) );


//
//
//								|| ( objLanes.to_ulong() & closestOppositeDirLnMask.to_ulong() )
//								);

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
					gout << " The value of objOnNeededLanes is: "<< objOnNeededLanes << endl;
					gout << " The objLanes.to_ulong is: " << objLanes.to_ulong() << endl;
					gout << " The closestOppositeDirLnMask.to_ulong is: ";
					gout << closestOppositeDirLnMask.to_ulong() << endl;
#endif

					cvTObjAttr* pObjAttr = BindObjAttr ( objId );
					double objLength = pObjAttr->xSize;
					double distFromOwner;
					if( objOnNeededLanes )
					{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
						gout << " The obj is on the closestOppositeDirLn. " << endl;
#endif

						bool foundDupsOnRoad = false;
						if( samePathPointAsOwner )
						{
							//
							// Make sure the obj is within distances if it is on
							// the same path point as owner.
							//
							bool withinDist;
							if( currLane.GetDirection() == ePOS )
							{

								double objAdjustedDist =
										m_pDynObjRefPool[curIdx].distance + (objLength * 0.5);

								//
								// Make sure the obj on the boundary between intersection and
								// road get picked up.
								//
								withinDist = (
										objAdjustedDist >= ownerDist &&
										m_pDynObjRefPool[curIdx].distance <= endDist
										);
							}
							else
							{
								double objAdjustedDist =
										m_pDynObjRefPool[curIdx].distance - (objLength * 0.5);
								withinDist = (
										objAdjustedDist <= ownerDist &&
										m_pDynObjRefPool[curIdx].distance >= endDist
										);

							}


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST

							gout << " The object is within distance or not: " << withinDist << endl;
							gout << " The obj distance is: " << m_pDynObjRefPool[curIdx].distance ;
							gout << endl;
							gout << " The owner dist is: " << ownerDist << endl;
							gout << " The endDist is: " << endDist << endl;

#endif

							if( withinDist )
							{
								//
								// Compute the distance from the owner object to this object.
								//
								distFromOwner = fabs(
												ownerDist -
												m_pDynObjRefPool[curIdx].distance
												);

								//
								// Insert object into main list without duplicates.
								//
								vector<TObjListInfo>::iterator itr;
								for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
								{
									if( (*itr).objId == objId )
									{
										foundDupsOnRoad = true;
										break;
									}
								}
								if( ! foundDupsOnRoad )
								{
									TObjListInfo node;
									node.objId = objId;
									node.distFromOwner = distFromOwner;
									node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
									gout << "  **inserting " << node.objId << " to the oncoming list.";
									gout << endl;
									gout << " The dist from the owner is:" << node.distFromOwner;
									gout << endl;;
#endif

									oncomObjs.push_back( node );
								}
							}
						}
						else
						{
							cvELnDir laneDir;
							if( roadPos.IsRoad() )
							{
								laneDir = roadPos.GetLane().GetDirection();
							}
							else
							{
								laneDir = roadPos.GetCorridor().GetDstntnLn().GetDirection();
							}
							if ( laneDir == ePOS )
							{
								distFromOwner = (
											distTraveledSoFar +
											m_pDynObjRefPool[curIdx].distance
											);


							}
							else
							{
								distFromOwner = (
											distTraveledSoFar +
											currRoad.GetLinearLength() -
											m_pDynObjRefPool[curIdx].distance
											);

							}

							//
							// Insert object into main list without duplicates.
							//
							vector<TObjListInfo>::iterator itr;
							for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
							{
								if( (*itr).objId == objId )
								{
									foundDupsOnRoad = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
									gout << " The obj # " << objId << " is already in the list. ";
									gout << endl;
#endif

									break;
								}
							}
							if( ! foundDupsOnRoad )
							{
								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
								gout << "  **inserting " << node.objId << " to the oncoming list.";
								gout << endl;
								gout << " The dist from the owner is:" << node.distFromOwner;
								gout << endl;;
#endif

								oncomObjs.push_back( node );
							}
						}
					}	// On needed lane.
				}	// Passed first test.

				curIdx = m_pDynObjRefPool[curIdx].next;

			}	// while loop

			//
			// Update distance traveled so far.
			//
			if ( samePathPointAsOwner )
			{
				if ( roadPos.GetLane().GetDirection() == ePOS )
				{
					distTraveledSoFar += (
								currRoad.GetLinearLength() -
								ownerDist
								);
				}
				else
				{
					distTraveledSoFar += ownerDist;
				}
			}
			else
			{
				distTraveledSoFar += currRoad.GetLinearLength();
			}

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The distTraveledSoFar is:" << distTraveledSoFar << endl;
#endif


		}	// if the path point is on road.
		else
		{
			//
			// If the path point is on intersection.
			//

			//
			// In order to find the oncoming corridor, need to find the
			// closest opposite direction lane of the destination road of the
			// the corridors on the current path point ( refered as lane A ) and
			// the closest opposite direction lane of the source road of these
			// corridors ( referred as lane B ).Then get all the corridors from
			// the current intersection and get the corridor whose source lane is
			// lane A and whose destination lane is lane B. This corridor is
			// the oncoming corridor.
			//

			//
			// Find the closest opposite direction lane of the destination road and
			// source road of the corridors on the current path point.
			//
			bool foundA = false;
			bool foundB = false;
			CCrdr currCrdr;
			CLane leftMostLnAlongDirOfDstntnRoad;
			CLane leftMostLnAlongDirOfSrcRoad;
			CLane closestOppositeDirLnOfDstntnRd;
			CLane closestOppositeDirLnOfSrcRd;
			bitset<cCV_MAX_CRDRS> crdrMask = path.GetPoint( pathItr ).GetLaneCrdrMask();
			CIntrsctn currIntrsctn = path.GetPoint( pathItr ).GetIntrsctn();

			CLane tempLane;
			CLane tempSrcLane;
			int crdrId;
			for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
			{
				if( crdrMask[crdrId] )
				{

					CCrdr temCrdr( currIntrsctn, crdrId );
					CLane temLane = temCrdr.GetDstntnLn();
					tempLane = temLane;
					CLane temSrcLane = temCrdr.GetSrcLn();
					tempSrcLane = temSrcLane;
					if( ! foundA  && temLane.IsLeftMostAlongDir() )
					{

						leftMostLnAlongDirOfDstntnRoad = temLane;
						if( ! leftMostLnAlongDirOfDstntnRoad.IsLeftMost() )
						{
							closestOppositeDirLnOfDstntnRd =
								leftMostLnAlongDirOfDstntnRoad.GetLeft();
						}

						//
						// If it is a one-way road, return.
						//
						if( !closestOppositeDirLnOfDstntnRd.IsValid() )
						{
							return;
						}
						foundA = true;
						CCrdr currentCrdr = temCrdr;
						currCrdr = currentCrdr;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
						gout << " The current corridor is: ";
						gout << currCrdr.GetRelativeId() << endl;
#endif

					}
					if( ! foundB  && temSrcLane.IsLeftMostAlongDir() )
					{
						leftMostLnAlongDirOfSrcRoad = temSrcLane;
						if( ! leftMostLnAlongDirOfSrcRoad.IsLeftMost() )
						{

							closestOppositeDirLnOfSrcRd =
								leftMostLnAlongDirOfSrcRoad.GetLeft();
						}

						//
						// If it is a one-way road, return.
						//
						if( !closestOppositeDirLnOfSrcRd.IsValid() )
						{
							return;
						}
						foundB = true;
					}
					if( foundA && foundB )
					{
						break;
					}

				}
			}	// for loop

			//
			// If none of the corridor connects to the leftmostlane of the
			// destination road or source road, have to look for it.
			if( ! foundA )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " Have to move left to get the leftmost lane. " << endl;
				gout << " The lane that is used to move left is: " << tempLane << endl;
#endif

				CLane leftLaneOfDstntnRd;
				if( ! tempLane.IsLeftMost() )
				{
					leftLaneOfDstntnRd = tempLane.GetLeft();
				}

				if( !leftLaneOfDstntnRd.IsValid() )
				{
					return;
				}
				else
				{
					cvELnDir leftLaneOfDstntnRdDir = leftLaneOfDstntnRd.GetDirection();
					cvELnDir tempLaneDir = tempLane.GetDirection();

					bool sameDirLanesOfDstntnRd = leftLaneOfDstntnRdDir == tempLaneDir;

					if( sameDirLanesOfDstntnRd && leftLaneOfDstntnRd.IsLeftMost() )
					{
						return;
					}

					while( sameDirLanesOfDstntnRd && !leftLaneOfDstntnRd.IsLeftMost() )
					{

						leftLaneOfDstntnRd = leftLaneOfDstntnRd.GetLeft();
						if( !leftLaneOfDstntnRd.IsValid() )
						{
							return;
						}
						leftLaneOfDstntnRdDir = leftLaneOfDstntnRd.GetDirection();
						sameDirLanesOfDstntnRd = leftLaneOfDstntnRdDir == tempLaneDir;
					}
					if ( !sameDirLanesOfDstntnRd )
					{
						closestOppositeDirLnOfDstntnRd = leftLaneOfDstntnRd;
						leftMostLnAlongDirOfDstntnRoad = closestOppositeDirLnOfDstntnRd .GetLeft();
					}
				}
			}
			if( ! foundB )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " Have to move left to get the leftmost lane. " << endl;
				gout << " The lane that is used to move left on source road is: ";
				gout << tempSrcLane << endl;
#endif

				CLane leftLaneOfSrcRd;
				if( ! tempSrcLane.IsLeftMost() )
				{

					leftLaneOfSrcRd = tempSrcLane.GetLeft();
				}

				if( !leftLaneOfSrcRd.IsValid() )
				{
					return;
				}
				else
				{
					cvELnDir leftLaneOfSrcRdDir = leftLaneOfSrcRd.GetDirection();
					cvELnDir tempSrcLaneDir = tempSrcLane.GetDirection();

					bool sameDirLanesOfSrcRd = leftLaneOfSrcRdDir == tempSrcLaneDir;

					if ( sameDirLanesOfSrcRd && leftLaneOfSrcRd.IsLeftMost() )
					{
						return;
					}

					while( sameDirLanesOfSrcRd && !leftLaneOfSrcRd.IsLeftMost() )
					{

						leftLaneOfSrcRd = leftLaneOfSrcRd.GetLeft();
						if( !leftLaneOfSrcRd.IsValid() )
						{
							return;
						}
						leftLaneOfSrcRdDir = leftLaneOfSrcRd.GetDirection();
						sameDirLanesOfSrcRd = leftLaneOfSrcRdDir == tempSrcLaneDir;
					}
					if ( !sameDirLanesOfSrcRd )
					{
						closestOppositeDirLnOfSrcRd = leftLaneOfSrcRd;
						leftMostLnAlongDirOfSrcRoad = closestOppositeDirLnOfSrcRd .GetLeft();
					}
				}
			}

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The current intersection is: " << currIntrsctn.GetName() << endl;
			gout << " The current destination road is: " << tempLane.GetRoad() << endl;
			gout << " The current source road is: " << tempSrcLane.GetRoad() << endl;
			gout << " The leftMostLnAlongDir on the destination road is: ";
			gout << leftMostLnAlongDirOfDstntnRoad << endl;
			gout << " The leftMostLnAlongDir on the source road is: ";
			gout << leftMostLnAlongDirOfSrcRoad << endl;
			gout << " The closest opposite direction lane of the destination road is: "<< endl;
			gout << " " << closestOppositeDirLnOfDstntnRd << endl;
			gout << " The closest opposite direction lane of the source road is: "<< endl;
			gout << " " << closestOppositeDirLnOfSrcRd << endl;
#endif

			//
			// Get the oncoming corridor.
			//

			if (!closestOppositeDirLnOfSrcRd.IsValid())
				continue; //if we have found no oncomming corridor
			if (!closestOppositeDirLnOfDstntnRd.IsValid())
				continue;//we have some kind logical fault, some case is not being handled.

			CCrdr oncomingCrdr;
			/*static*/ vector<CCrdr> allCrdrs;
			allCrdrs.clear();
			allCrdrs.reserve(15);
			currIntrsctn.GetAllCrdrs( allCrdrs );
			vector<CCrdr>::iterator itr;
			bool foundCrdr = false;
			for( itr = allCrdrs.begin(); itr != allCrdrs.end(); itr++ )
			{
				if( (*itr).GetDstntnLn() == closestOppositeDirLnOfSrcRd )
				{
					if((*itr).GetSrcLn() == closestOppositeDirLnOfDstntnRd )
					{
						oncomingCrdr = *itr;
						foundCrdr = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " The oncoming corridor is: " << oncomingCrdr.GetRelativeId() << endl;
#endif

						break;
					}
				}
			}

			//
			// For the some intersections, the oncomingCrdr may not be found because
			// closest opposite lanes of the destination road and source road may not
			// be connected. Then don't look at objs on this intersection and move on
			// to the next path point.
			//
			if( ! foundCrdr )
			{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " oncomingCrdr is not found. " << endl;
#endif

				continue;
			}


			int intrsctnId = currIntrsctn.GetId();
			int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
			double distFromOwner;

			// Insert objects into the oncoming list until there are no
			// more objects on this intersection or we reach the maximum
			// number of objects needed.
			//
			while( curIdx != 0 && (int)oncomObjs.size() < maxObjs )
			{

				//
				// Make sure that this object is not the owner object, part of
				// the object mask, on the oncoming corridor and is located
				// within the needed distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
				bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
				gout << " The current obj in the pool is:  #" << objId << endl;

#endif


				if( passedFirstTest )
				{
					bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
					bitset<cCV_MAX_CRDRS> oncomingCrdrMask;
					oncomingCrdrMask.set( oncomingCrdr.GetRelativeId() );


					objCrdrs &= oncomingCrdrMask;
					bool objOnNeededCrdrs= (
						objCrdrs.any() ||
						( objCrdrs.to_ulong() == 0 && oncomingCrdrMask.to_ulong() == 0 )
						);


					int oncomingCrdrId = oncomingCrdr.GetRelativeId();
					double objDistAlongOncomCrdr;
					objDistAlongOncomCrdr = m_pDynObjRefPool[curIdx].crdrDistances
											[oncomingCrdrId] ;
					if( objOnNeededCrdrs )
					{
						bool foundDupsOnIntrsctn = false;

						//
						// Make sure the object is within distance if the obj is on the
						// same path point as the owner.
						//
						if( samePathPointAsOwner )
						{
							//
							// Only need to get obj ahead of the owner.
							//
							double dist =  oncomingCrdr.GetLength() - roadPos.GetDistance();
							bool withinDist =  objDistAlongOncomCrdr >= 0  &&
											   objDistAlongOncomCrdr  <= dist;

							if( withinDist )
							{

									distFromOwner = fabs( oncomingCrdr.GetLength() -
									objDistAlongOncomCrdr - ownerDist ) ;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
								gout << " The oncomCrdr length is: " << oncomingCrdr.GetLength();
								gout << endl;
								gout << " The oncom obj dist from the pool is: ";
								gout << objDistAlongOncomCrdr << endl;
								gout << " The owner dist is: " << ownerDist << endl;
#endif

								//
								// Insert object into main list without duplicates.
								//
								vector<TObjListInfo>::iterator itr;
								for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
								{
									if( (*itr).objId == objId )
									{
										foundDupsOnIntrsctn = true;
										break;
									}
								}
								if( ! foundDupsOnIntrsctn )
								{
									TObjListInfo node;
									node.objId = objId;
									node.distFromOwner = distFromOwner;
									node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
									gout << "  **inserting " << node.objId << " to the oncoming list.";
									gout << endl;
									gout << " The dist from the owner is:" << node.distFromOwner;
									gout << endl;;
#endif

									oncomObjs.push_back( node );
								}

							}

						}
						else
						{
							distFromOwner = distTraveledSoFar + ( oncomingCrdr.GetLength()
												- objDistAlongOncomCrdr );

							//
							// Insert object into main list without duplicates.
							//
							vector<TObjListInfo>::iterator itr;
							for( itr = oncomObjs.begin(); itr != oncomObjs.end(); itr++ )
							{
								if( (*itr).objId == objId )
								{
									foundDupsOnIntrsctn = true;
									break;
								}
							}
							if( ! foundDupsOnIntrsctn)
							{
								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
								gout << "  **inserting " << node.objId << " to the oncoming list.";
								gout << endl;
								gout << " The dist from the owner is:" << node.distFromOwner;
								gout << endl;;
#endif

								oncomObjs.push_back( node );
							}
						}

					}	// On desired corridor.
				}	// Passed first test.

				curIdx = m_pDynObjRefPool[curIdx].next;


			}	// while loop

			//
			// Update distance traveled.
			//
			if( samePathPointAsOwner )
			{
				distTraveledSoFar += ( roadPos.GetCorridor().GetLength() - ownerDist );
			}
			else
			{
				distTraveledSoFar += oncomingCrdr.GetLength();
			}

		}	// If the path point is on intersection.
		if ( (int)oncomObjs.size() >= maxObjs )
		{

#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
			gout << " Already reached maximum #of objects before end of path. " << endl;
#endif
			break;

		}

	}	// for loop


	//
	// Sort the list according to distance from the owner. (Although the
	// objs on road are already sorted, those on intersection are not.So need sorting.)
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < oncomObjs.size(); k++ )
	{
		hold = oncomObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < oncomObjs[loc-1].distFromOwner ) )
		{
			oncomObjs[loc] = oncomObjs[loc-1];
			loc--;
		}
		oncomObjs[loc] = hold;
	}


#ifdef DEBUG_BUILD_ONCOM_OBJ_LIST
	gout << " The size of the oncoming list is:" << oncomObjs.size() << endl;
	vector<TObjListInfo>::iterator i;
	gout << " The following is the final list for obj #" << ownerObjId<< ":  ";
	for( i = oncomObjs.begin(); i != oncomObjs.end(); i++ )
	{
		gout << " " << i->objId << " [" << i->distFromOwner << "]";
	}
	gout << endl;

#endif

}	// end of BuildOncomingList

///////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects that approach the
//   specified object in the parameter from the side in a merging situation.
//
// Remarks: The owner object is on the current corridor and it looks at the
//	corridor that's approaching it from the side (refered as the side corridor).
//	The function first gets objects on this side corridor and then gets objects
//	on the side corridor's source lane.
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos - The current roadPos of the specified object in the
//                parameter.
//	 path - The current path of the specified object in the parameter.
//   maxObjs - The maximum number of objects the user is interested in
//                looking at.
//   apprchObjs - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::BuildApprchObjList(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CPath& path,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& apprchObjs
			)
{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
	gout << endl;
	gout << " == BuildApprchObjList==== for obj # " << ownerObjId;
	gout << endl;
#endif

	//
	// Error checking.
	//
	if( ! roadPos.IsValid() )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << "CCvedDistri::BuildApprchObjList: invalid road position....exit" << endl;
#endif

		return;
	}
	if ( !path.IsValid() || path.Size() <= 0 )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << "CCvedDistri::BuildOnApprchObjList: invalid path....exit" << endl;
#endif

		return;

	}

	//
	// Get the proper intersection.
	//
	CIntrsctn currIntrsctn;
	CCrdr currCrdr;
	if( roadPos.IsRoad() )
	{
		currIntrsctn = path.GetNextIntrsctn( roadPos );
		currCrdr = path.GetNextCrdr( roadPos );
		if( ! currIntrsctn.IsValid() )
		{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
			gout << " No upcoming intersection...exit. " << endl;
#endif

			return;
		}
	}
	else
	{
		currIntrsctn = roadPos.GetIntrsctn();
		currCrdr = roadPos.GetCorridor();
	}

	if( ! currCrdr.IsValid() )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " Current corridor is invalid...exit. " << endl;
#endif

		return;
	}


	//
	// Get the onramp corridor and the corridor next to it, since this function
	// is only interested in objects on these two corridors.
	//
	CCrdr crdr1, crdr2;
	/*static*/ vector<CCrdr> allCrdrs;
	allCrdrs.clear();
	allCrdrs.reserve(15);
	currIntrsctn.GetAllCrdrs( allCrdrs );
	TU32b left, right;
	bool foundMrgCrdrs = false;
	for( left = 0; left < allCrdrs.size(); left++ )
	{
		for( right = left + 1; right < allCrdrs.size(); right++ )
		{
			if( allCrdrs[left].GetDstntnLn() == allCrdrs[right].GetDstntnLn() )
			{
				crdr1 = allCrdrs[left];
				crdr2 = allCrdrs[right];
				foundMrgCrdrs = true;
				break;
			}
		}
	}

	if( ! foundMrgCrdrs )
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " This is not a situation where the ADO approaches each other...exit. ";
		gout << endl;
#endif

		return;
	}
	//
	// Make sure the current corridor is one of the two corridors.
	//
	if(
	  currCrdr.GetRelativeId() != crdr1.GetRelativeId() &&
	  currCrdr.GetRelativeId() != crdr2.GetRelativeId()
	)
	{

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << ownerObjId << " is not on a merging lane or corridor...exit " << endl;
#endif

		return;
	}

	CCrdr sideCorridor;
	if( currCrdr.GetRelativeId() == crdr1.GetRelativeId() )
	{
		sideCorridor = crdr2;
	}
	if( currCrdr.GetRelativeId() == crdr2.GetRelativeId() )
	{
		sideCorridor = crdr1;
	}

	int sideCorridorId = sideCorridor.GetRelativeId();
	int intrsctnId = currIntrsctn.GetId();
	int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;
	double mergDist = 0;
	double distFromOwner;

	//Calculate the distance from the end of the intersection to merge point between side corridor
	//and the current corr.
	vector<double> dists;
	sideCorridor.GetMrgDstFirst( dists );
	TU32b i;
	for( i = 0; i < dists.size(); i++ )
	{
		//
		// The index of the vector matches the corridor Id.
		//
		if( i == currCrdr.GetRelativeId() )
		{
			mergDist = dists[i];
			break;
		}
	}

	//
	// Get objects on the side corridor.
	//
	while( curIdx != 0 )
	{
		//
		// Make sure that this object is not the owner object, part of
		// the object mask, and on the side corridor.
		//
		int objId = m_pDynObjRefPool[curIdx].objId;
		bool notOwnerObj = ownerObjId != objId;
		cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
		bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " The current obj in the pool is:  #" << objId << endl;

#endif

		if( passedFirstTest )
		{
			bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
			bitset<cCV_MAX_CRDRS> sideCorridorMask;
			sideCorridorMask.set( sideCorridorId );



			bool objOnNeededCrdrs = (
						( objCrdrs.to_ulong() == sideCorridorMask.to_ulong() )
						|| ( objCrdrs.to_ulong() & sideCorridorMask.to_ulong() )
						);


			if( objOnNeededCrdrs )
			{

				TObjListInfo node;
				node.objId = objId;

				//
				// Compute the obj's distance to the merging point.
				//

				double objDist;
				objDist = m_pDynObjRefPool[curIdx].crdrDistances[sideCorridorId];

				node.objDist = objDist;
				node.isOnRoad = false;

				double distToMrgPoint = mergDist - objDist;
				distFromOwner = distToMrgPoint;

				node.distFromOwner = distFromOwner;
				// this field needs to be changed later to the distToMrgPoint

				node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
				gout << " The current obj # " << objId <<" is on side corridor " << endl;
				gout << "  **inserting " << node.objId << " to the approaching list.";
				gout << endl;
				gout << " The obj dist is: " << objDist <<  endl;
				gout << " The mergDist is: " << mergDist << endl;
				gout << " The distToMrgPoint is: " << distToMrgPoint << endl;
				gout << " The dist from the owner is:" << node.distFromOwner;
				gout << endl;;
#endif

				apprchObjs.push_back( node );
			}
		}

		curIdx = m_pDynObjRefPool[curIdx].next;
	}	// while loop


	//
	// Get objects on the sideCorridor's source lane.
	//
	CLane currLane = sideCorridor.GetSrcLn();
	CRoad currRoad = currLane.GetRoad();
	int currRoadId = currRoad.GetId();
	curIdx = m_pRoadRefPool[currRoadId].objIdx;

	//
	// Insert objects into the approaching list until there are no
	// more objects on this road or we reach the maximum
	// number of objects needed.
	//
	while( curIdx != 0 && (int)apprchObjs.size() < maxObjs )
	{

		//
		// Make sure that this object is not the owner object, part of the object mask,
		// and is on the source lane.
		//
		int objId = m_pDynObjRefPool[curIdx].objId;
		bool notOwnerObj = ownerObjId != objId;
		cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
		bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );
		bool passedFirstTest = notOwnerObj && inObjMask;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
		gout << " The current object in the pool is: "<< objId << endl;
		gout << " The road id of the current obj is: " << m_pDynObjRefPool[curIdx].roadId << endl;
#endif

		if( passedFirstTest )
		{
			bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
			bitset<cCV_MAX_LANES> sideCrdrSrcLnMask;
			sideCrdrSrcLnMask.set( currLane.GetRelativeId() );
			bool objOnNeededLanes = (
						( objLanes.to_ulong() == sideCrdrSrcLnMask.to_ulong() )
						|| ( objLanes.to_ulong() & sideCrdrSrcLnMask.to_ulong() )
						);

			double distFromOwner;
			if( objOnNeededLanes )
			{
				//
				// Inserting obj into the list.
				//
				TObjListInfo node;
				node.objId = objId;

				//
				// Compute distance to the merging point.
				//
				double objDist;
				objDist = m_pDynObjRefPool[curIdx].distance;

				node.objDist = objDist;
				node.isOnRoad = true;

				double distToMrgPoint;

				if( currLane.GetDirection() == ePOS )
				{

					distToMrgPoint = mergDist + currRoad.GetLinearLength()
							- objDist;
				}
				else
				{
					distToMrgPoint = mergDist + objDist;
				}

				distFromOwner = distToMrgPoint;

				node.distFromOwner = distFromOwner;
				// this field need to change to a different name later to distToMrgPoint

				node.sameDirAsLane = true;

#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
				gout << " The current obj # " << objId <<" is on side corridor's source road " << endl;
				gout << "  **inserting " << node.objId << " to the approaching list.";
				gout << endl;
				gout << " The obj dist is: " << objDist <<  endl;
				gout << " The mergDist is: " << mergDist << endl;
				gout << " The distToMrgPoint is: " << distToMrgPoint << endl;
				gout << " The dist from the owner is:" << node.distFromOwner;
				gout << endl;;
#endif

				apprchObjs.push_back( node );
			}
		}
		curIdx = m_pDynObjRefPool[curIdx].next;
	}	// second while loop.


	//
	// Sort the list in an ascending order according to the distance to merging point.
	//
	TObjListInfo hold;
	TU32b k, loc;
	for( k =1; k < apprchObjs.size(); k++ )
	{
		hold = apprchObjs[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < apprchObjs[loc-1].distFromOwner ) )
		{
			apprchObjs[loc] = apprchObjs[loc-1];
			loc--;
		}
		apprchObjs[loc] = hold;
	}


#ifdef DEBUG_BUILD_APPRCH_OBJ_LIST
	gout << " The size of the approching list is:" << apprchObjs.size() << endl;
	vector<TObjListInfo>::iterator itr;
	gout << " The following is the final list for obj #" << ownerObjId<< ":  ";
	for( itr = apprchObjs.begin(); itr != apprchObjs.end(); itr++ )
	{
		gout << " " << itr->objId << " [" << itr->distFromOwner << "]";
	}
	gout << endl;

#endif


}	// end of BuildApprchObjList

//////////////////////////////////////////////////////////////////////////////
//
// Description:	This function creates a list of objects behind the
//   specified object in the parameter.
//
// Remarks:	If the roadPos is on road, this function looks at objs on
// the current road, the previous intersection and on one more road further back
// from the intersection. If the roadPos is on intersection, it looks at objs
// on the current intersection and on the road that is the roadPos's source road.
// But it can be extended to search even further back.
//
// Arguments:
//   ownerObjId - Cved id of the owner object.
//   roadPos - The current roadPos of the specified object in the
//                parameter.
//	 prevRoad - The previous road of the current roadPos of the specified
//				  object in the parameter.
//   maxObjs - The maximum number of objects the user is interested in
//                looking at.
//	 objMask - The type of object.
//   backObjs2   - (output) Contains the objects sorted by distance.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedDistri::BuildBackObjList2(
			int ownerObjId,
			const CRoadPos& roadPos,
			const CRoad& prevRoad,
			int maxObjs,
			const CObjTypeMask& objMask,
			vector<TObjListInfo>& backObjs2
			)
{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
	gout << "[" << ownerObjId << "] == BuildBackObjList2 ==================";
	gout << endl;
#endif

	//
	// Error checking.
	//
	if( !roadPos.IsValid() )
	{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
		gout << "invalid roadPos...exit. " << endl;
#endif

		return;
	}
	if( !prevRoad.IsValid() )
	{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
		gout << "invalid prevRoad...exit. " << endl;
#endif

		return;
	}

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
	gout << "roadPos = " << roadPos << endl;
	gout << "prevRoad = " << prevRoad.GetName() << endl;
#endif

	//
	// Initialize variables.
	//
	const int cMAXROADS = 3;
	// This variable is for the maximum number of roads and intersections to
	// look at. When the roadPos is on road, the program searches the
	// road-intersection-road(3 roads); when the roadPos is on
	// intersection, it searches intersection-road(2 roads).
	// But the number of roads in this later case is incremented
	// in the proper place to make it the value of cMAXROADS.

	int numRoads = 0;
	int roadCount = 0;
	int intrsctnCount =  0;
	double distLookedSoFar = 0.0;
	double ownerDist = -101.0;


	// Get owner distance from obj ref list.
	int curIdx;
	if( roadPos.IsRoad() )
	{
		int currRoadId = roadPos.GetRoad().GetId();
		curIdx = m_pRoadRefPool[currRoadId].objIdx;

		while( curIdx != 0 )
		{
			if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
			{
				ownerDist = m_pDynObjRefPool[curIdx].distance;
				break;
			}

			curIdx = m_pDynObjRefPool[curIdx].next;
		}

	}
	else
	{
		int currIntrsctnId = roadPos.GetIntrsctn().GetId();
		curIdx = m_pIntrsctnRefPool[currIntrsctnId].objIdx;

		while( curIdx != 0 )
		{
			if( m_pDynObjRefPool[curIdx].objId == ownerObjId )
			{
				int crdrId = roadPos.GetCorridor().GetRelativeId();
				ownerDist = m_pDynObjRefPool[curIdx].crdrDistances[crdrId];
				break;
			}

			curIdx = m_pDynObjRefPool[curIdx].next;
		}

	}

	if( ownerDist < -100.0 )
	{
		//cerr << "CCvedDistri::BuildBackObjList2: unable to determine ";
		//cerr << "owner object " << ownerObjId << " distance.  Using ";
		//cerr << "dist from given roadPos" << endl;
		ownerDist = roadPos.GetDistance();
	}


	// Check the maximum number of roads
	while ( numRoads < cMAXROADS )
	{
		bool lookOnRoad = true;
		bool lookOnIntrsctn = true;

		//
		// If the roadPos is on intersection, skip looking on road.
		//
		if( ! roadPos.IsRoad() && intrsctnCount == 0 )
		{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " The owner obj is on intersection. " << endl;
#endif

			//
			// Increment the number of roads here so that when the
			// roadPos is on intersection, the program will search
			// only the current intersection and the previous road
			// specified in the parameter.
			//
			numRoads++;

			lookOnRoad = false;
		}

		//
		// Looking on road.
		//
		if( lookOnRoad )
		{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " Looking on road. " << endl;
#endif

			CRoad currRoad;
			bool onSameRoadAsRoadPos = false;

			//
			// Get the road.
			//
			CLane currLane;
			if( roadPos.IsRoad() && roadCount == 0 )
			{
				currLane = roadPos.GetLane();
				currRoad = currLane.GetRoad();
				roadCount++;
				onSameRoadAsRoadPos = true;
			}
			else
			{
				currRoad = prevRoad;
			}


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " The current road is: " << currRoad.GetName() << endl;
#endif

			//
			// If the road is where the roadPos is on, build
			// a lane mask with all lanes traveling in the
			// same direction as the lane in the specified roadPos.
			//
			bitset<cCV_MAX_LANES> laneMask;
			if( onSameRoadAsRoadPos )
			{
				int laneId;
				int numLanes = currRoad.GetNumLanes();
				int firstLaneIdx = currRoad.GetLaneIdx();
				for( laneId = 0; laneId < numLanes; laneId++ )
				{
					cvTLane* pLane = BindLane( firstLaneIdx + laneId );
					if( pLane->direction == currLane.GetDirection() )
					{
						laneMask.set( laneId );
					}
				}

			}

			int currRoadId = currRoad.GetId();
			int curIdx = m_pRoadRefPool[currRoadId].objIdx;


			//
			// Examing the objs on the road.
			//
			while( curIdx != 0 )
			{
				//
				// Make sure that this object is not the owner object, part of the
				// object mask, on the desired lanes and is located within the needed
				// distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );

				//
				// An object should be placed on the owner's back list as long
				// as any part of the object is behind the owner's center.
				//
				cvTObjAttr* pObjAttr = BindObjAttr( objId );
				double objLength = pObjAttr->xSize;

				bool objBack = true;
				if( onSameRoadAsRoadPos )
				{
					if( roadPos.GetLane().GetDirection() == ePOS )
					{
						objBack = (
						m_pDynObjRefPool[curIdx].distance - (objLength * 0.5) <=
						ownerDist
						);
					}
					else
					{
						objBack = (
							m_pDynObjRefPool[curIdx].distance + (objLength * 0.5) >=
							ownerDist
							);
					}
				}

				bool passedFirstTest = notOwnerObj && inObjMask && objBack;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
				gout << " The current object in the pool is: "<< objId << endl;
#endif

				if(  passedFirstTest )
				{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
					gout << " The obj passed first test. " << endl;
#endif

					bool objOnNeededLanes = false;

					if( onSameRoadAsRoadPos )
					{
						//
						// Make sure the object is on at least one of the
						// desired lanes.
						//
						bitset<cCV_MAX_LANES> objLanes((int)m_pDynObjRefPool[curIdx].lanes);
						objLanes &= laneMask;
						objOnNeededLanes = (
									objLanes.any() ||
									( objLanes.to_ulong() == 0 && laneMask.to_ulong() == 0 )
									);
					}
					else
					{
						//
						// If this is not the road where the roadPos is on, then
						// get the objects that have the same direction as the
						// lane the roadPos is located on. ( can't use lane mask to
						// to get the objs on desired lanes here. )
						//
						if( roadPos.IsRoad() )
						{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
							gout << " The obj dir is: " << m_pDynObjRefPool[curIdx].direction;
							gout << endl;
							gout << " The currLane dir is: " << roadPos.GetLane().GetDirection();
							gout << endl;
#endif
							if( m_pDynObjRefPool[curIdx].direction
										== roadPos.GetLane().GetDirection() )
							{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " The two dir is the same. " << endl;
#endif

								objOnNeededLanes = true;

							}
						}
						else
						{
							//
							// If the roadPos is on intersection, use the current corridor's
							// source lane's direction to determine which objects to get
							// on this previous road.
							//
							CLane lane = roadPos.GetCorridor().GetSrcLn();
							if( m_pDynObjRefPool[curIdx].direction == lane.GetDirection() )
							{
								objOnNeededLanes = true;
							}
						}
					}

					bool withinDist = true;

					if( objOnNeededLanes )
					{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
						gout << " The obj is on needed lanes. " << endl;
#endif

						if( onSameRoadAsRoadPos )
						{
							double startDist;
							double endDist;
							if( currLane.GetDirection() == ePOS )
							{
								startDist = 0.0;
								endDist = ownerDist;
							}
							else
							{
								startDist = ownerDist;
								endDist = currRoad.GetLinearLength();
							}
							withinDist = (
									( m_pDynObjRefPool[curIdx].distance >= startDist ) &&
									( m_pDynObjRefPool[curIdx].distance <= endDist )
									);
						}

						if( withinDist )
						{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
							gout << " The obj is with in distance. " << endl;
#endif

							//
							// Compute distance from the owner object to this object.
							//
							double distFromOwner;
							if( onSameRoadAsRoadPos )
							{
								distFromOwner = fabs( ownerDist -
											m_pDynObjRefPool[curIdx].distance );

							}
							else
							{

								if( m_pDynObjRefPool[curIdx].direction == ePOS )
								{
									distFromOwner = distLookedSoFar + prevRoad.GetLinearLength()
												- m_pDynObjRefPool[curIdx].distance;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
									gout << " On road...distLookedSoFar is: " << distLookedSoFar;
									gout << endl;
									gout << " On road...prevRoad length is: ";
									gout << prevRoad.GetLinearLength() << endl;
									gout << " On road...obj dist from pool is: ";
									gout << m_pDynObjRefPool[curIdx].distance << endl;
#endif

								}
								else
								{
									distFromOwner = distLookedSoFar +

											m_pDynObjRefPool[curIdx].distance;
								}


							}

							//
							// Insert the obj into the list without duplicates.
							//
							bool foundDupsOnRoad = false;
							vector<TObjListInfo>::iterator itr;
							for( itr = backObjs2.begin(); itr != backObjs2.end(); itr++ )
							{
									if( (*itr).objId == objId )
									{
										foundDupsOnRoad = true;
										break;
									}
							}
							if( ! foundDupsOnRoad )
							{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " *** inserting obj # " << objId << " to the list. ";
								gout << endl;
#endif

								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;
								backObjs2.push_back( node );
							}

						}	// Within distances

					}	// On needed lanes

				}	// Passed first test

				curIdx = m_pDynObjRefPool[curIdx].next;

			}	// the inside while loop


			//
			// Update distance looked so far.
			//
			if ( onSameRoadAsRoadPos )
			{
				if ( roadPos.GetLane().GetDirection() == ePOS )
				{
					distLookedSoFar = ownerDist;
				}
				else
				{
					distLookedSoFar = currRoad.GetLinearLength() - ownerDist;
				}
			}
			else
			{
				distLookedSoFar += currRoad.GetLinearLength();

			}

			//
			// Increment the number of roads after each road.
			//
			numRoads++;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
				gout << " The list size is: " << backObjs2.size() << endl;
				gout << " The value of maxObjs is: " << maxObjs << endl;
				gout << " The value of numRoads is: " << numRoads << endl;
#endif
			//
			// Quit looking if reached the maximum number of objs or if
			// reached the maximum number of roads.
			//
			if( numRoads >= cMAXROADS || (int)backObjs2.size() >= maxObjs )
			{
				break;
			}

			lookOnRoad = false;
		}	// look on road


		//
		// Look on intersection.
		//
		if( lookOnIntrsctn )
		{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " Looking on intersection. " << endl;
#endif



#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " On intersection...The ownerDist is: " << ownerDist << endl;
#endif

			//
			// Get the intersection.
			//
			CIntrsctn currIntrsctn;
			bool onSameIntrsctnAsRoadPos = false;
			if( ! roadPos.IsRoad() && intrsctnCount == 0 )
			{
				currIntrsctn = roadPos.GetIntrsctn();
				intrsctnCount++;
				onSameIntrsctnAsRoadPos = true;
			}
			else if( roadPos.IsRoad() )
			{
				currIntrsctn = roadPos.GetLane().GetPrevIntrsctn();
				intrsctnCount++;
			}

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " The current intersection is: " << currIntrsctn.GetName() << endl;
#endif

			//
			// Get the desired corridors of the current intersection.
			//
			/*static*/ vector<CCrdr> allCrdrs;
			allCrdrs.clear();
			allCrdrs.reserve(16);
			/*static*/ vector<CCrdr> desiredCrdrs;
			desiredCrdrs.clear();
			desiredCrdrs.reserve(16);
			/*static*/ vector<int> desiredCrdrIds;
			desiredCrdrIds.clear();
			desiredCrdrIds.reserve(16);

			vector<CCrdr>::iterator i;
			currIntrsctn.GetAllCrdrs( allCrdrs );
			for( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
			{
				if( onSameIntrsctnAsRoadPos )
				{
					if( (*i).GetSrcRd() == prevRoad )
					{
						desiredCrdrs.push_back( *i );
						desiredCrdrIds.push_back( (*i).GetRelativeId() );
					}
				}
				else
				{
					if(  (*i).GetDstntnRd() == roadPos.GetLane().GetRoad()  )
//						&& (*i).GetSrcRd() == prevRoad

					{
						desiredCrdrs.push_back( *i );
						desiredCrdrIds.push_back( (*i).GetRelativeId() );
					}
				}
			}


			int intrsctnId = currIntrsctn.GetId();
			int curIdx = m_pIntrsctnRefPool[intrsctnId].objIdx;

			double currCrdrLength;

			//
			// Examing objs on the intersection.
			//
			while( curIdx != 0 )
			{
				//
				// Make sure that this object is not the owner object, part of
				// the object mask, on the desired corridors and is located
				// within the needed distances.
				//
				int objId = m_pDynObjRefPool[curIdx].objId;
				bool notOwnerObj = ownerObjId != objId;
				cvEObjType type = GetObjType( m_pDynObjRefPool[curIdx].objId );
				bool inObjMask = objMask.Has( type, m_pDynObjRefPool[curIdx].objId );

				bool passedFirstTest = notOwnerObj && inObjMask;

				cvTObjAttr* pObjAttr = BindObjAttr( objId );
				double objLength = pObjAttr->xSize;


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
				gout << " The current obj in the pool is:  #" << objId << endl;

#endif

				if( passedFirstTest )
				{


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
					gout << " The obj passed first test. " << endl;
#endif

					bool objOnNeededCrdrs = false;

					//
					// Build mask for the desired corridors.
					//
					bitset<cCV_MAX_CRDRS> desiredCrdrMask;
					int crdrId;
					for( crdrId = 0; crdrId < cCV_MAX_CRDRS; crdrId++ )
					{
						vector<int>::iterator j = find( desiredCrdrIds.begin(),
												desiredCrdrIds.end(), crdrId );
						if( j != desiredCrdrIds.end() )
						{
							desiredCrdrMask.set( crdrId );
						}
					}

					//
					// Make sure the object's corridors overlap the
					// desired corridors.
					//
					bitset<cCV_MAX_CRDRS> objCrdrs((int)m_pDynObjRefPool[curIdx].corridors);
					objCrdrs &= desiredCrdrMask;
					objOnNeededCrdrs = (
										objCrdrs.any() ||
										objCrdrs.to_ulong() == 0 &&
										desiredCrdrMask.to_ulong() == 0
										);


					double objDist;
					bool withinDist = true;
					if( objOnNeededCrdrs )
					{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
						gout << " The obj is on needed corridors. " << endl;
#endif

						//
						// Gat a corridor that the object is on.
						//
						int k;
						for( k = 0; k < cCV_MAX_CRDRS; k++ )
						{

							if ( objCrdrs[k] )
							{

								CCrdr crdr( currIntrsctn, k );
								currCrdrLength = crdr.GetLength();

								// Assuming obj has same dist when it is
								// on more than one crdr since the crdrs
								// have same source and detination roads.
								objDist = m_pDynObjRefPool[curIdx].crdrDistances[k];


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " $$$$$$$$$$$$$ obj dist from pool = " << objDist;
								gout << " $$$$$$$$$$$$$ " << endl;
								gout << " The corridor id the obj is on is: ";
								gout << crdr.GetRelativeId();
								gout << endl;
								gout << " The obj's destination road is: ";
								gout << crdr.GetDstntnRd().GetName() << endl;
								gout << " The obj's source road is: ";
								gout << crdr.GetSrcRd().GetName() << endl;
								gout << endl;

#endif

								break;
							}
						}

						if( onSameIntrsctnAsRoadPos )
						{

							withinDist = ownerDist >= ( objDist - ( objLength * 0.5 ) );

						}

						if( withinDist )
						{
#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
							gout << " The obj is within distance. " << endl;
#endif

							//
							// Compute distance from owner.
							//
							double distFromOwner;
							if( onSameIntrsctnAsRoadPos )
							{
								distFromOwner = ownerDist - objDist;

							}
							else
							{

								distFromOwner = fabs( distLookedSoFar + currCrdrLength -
																objDist );

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " distFromOwner is: " << distFromOwner;
								gout << endl;
								gout << " distLookedSoFar is: " << distLookedSoFar << endl;
								gout << " current corridor length is: " << currCrdrLength << endl;
								gout << " current obj dist is: " << objDist << endl;
#endif

							}

							//
							// Insert the obj into the list without duplicates.
							//
							bool foundDupsOnIntrsctn = false;
							vector<TObjListInfo>::iterator itr;
							for( itr = backObjs2.begin(); itr != backObjs2.end(); itr++ )
							{
									if( (*itr).objId == objId )
									{
										foundDupsOnIntrsctn = true;
										break;
									}
							}

							if( ! foundDupsOnIntrsctn )
							{

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
								gout << " *** inserting obj # " << objId << " to the list. " << endl;
#endif
								TObjListInfo node;
								node.objId = objId;
								node.distFromOwner = distFromOwner;
								node.sameDirAsLane = true;
								backObjs2.push_back( node );
							}

						}	// Within distance.

					}	// On needed corridors

				}	// Passed first test

				curIdx = m_pDynObjRefPool[curIdx].next;
			}	// while loop

			//
			// Update distance looked so far by picking
			// a proper corridor. This is for cases when there is no
			// obj on the intersection or when the obj on the intersection
			// is the owner himself.
			CCrdr crdrPicked = desiredCrdrs[0];
			currCrdrLength = crdrPicked.GetLength();
			distLookedSoFar += currCrdrLength;

#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
			gout << " picked crdr length = " << currCrdrLength << endl;
#endif


			if( onSameIntrsctnAsRoadPos )
			{
				distLookedSoFar = ownerDist;
			}
			else
			{
				distLookedSoFar += currCrdrLength;
			}

			lookOnIntrsctn = false;

			if( (int)backObjs2.size() >= maxObjs )
			{
				break;
			}

			//
			// Increment the number of roads after looking at
			// each intersection.
			numRoads++;

		}	// Look on intersection


	}	// end of while loop

	//
	// Sort the list by distance to the owner.
	//
	TObjListInfo hold;
	unsigned int k, loc;
	for( k =1; k < backObjs2.size(); k++ )
	{
		hold = backObjs2[k];
		loc = k;
		while( 0 < loc && ( hold.distFromOwner < backObjs2[loc-1].distFromOwner ) )
		{
			backObjs2[loc] = backObjs2[loc-1];
			loc--;
		}
		backObjs2[loc] = hold;
	}


#ifdef DEBUG_BUILD_BACK_OBJ_LIST2
	gout << " The size of the back list2 is:" << backObjs2.size() << endl;
	vector<TObjListInfo>::iterator i;
	gout << " The following is the final list for obj #" << ownerObjId<< ":  ";
	for( i = backObjs2.begin(); i != backObjs2.end(); i++ )
	{
		gout << " " << i->objId << " [" << i->distFromOwner << "]";
	}
	gout << endl;

#endif


}	// end of BuildBackObjList2

};