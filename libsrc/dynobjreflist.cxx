
//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:    $Id: dynobjreflist.cxx,v 1.56 2018/09/13 19:30:09 IOWA\dheitbri Exp $
//
// Author(s):  Jillian Vogel, Omar Ahmad
// Date:       September, 1999
//
// Description: The CCved class methods associated with the dynamic 
//   object reference data structure.  
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"
#include <float.h>
#include <algorithm>
#include <string>

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{
#undef DYN_OBJ_REF_DEBUG
#undef DEBUG_DYN_OBJ_LIST //1  // frame number on which to start debugging

const double DIST_TO_CHECK = 10.0;

//////////////////////////////////////////////////////////////////////////////
//
// Description: Fill the dynamic object reference and road reference 
//  lists with the current locations of the live dynamic objects in 
//  the simulation.
//
// Remarks: This function iterates through all the dynamic objects in the 
// 	simulation and calculates their positions with respect to the current 
// 	road database.  This information is then stored in the linked lists 
// 	associated with each road reference, sorted by their distance along the 
// 	road.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CCved::FillByRoadDynObjList( void )
{
	/*static*/ TIntVec dynObjs;
	dynObjs.clear();
	dynObjs.reserve(30);
	TIntVec::const_iterator	objItr;
	cvTObj* pObj;
	CBoundingBox objBBox;

	bool debug = false;

	//
	// Allocate the saved object data structure, if needed.
	//
	if( !m_pSavedObjLoc ) 
	{
		// Allocate and initialize the data structure
		m_pSavedObjLoc = new TSavedObjLoc[cNUM_DYN_OBJS];
		if( !m_pSavedObjLoc )
		{
			cerr << "CCved::FillByRoadDynObjList: Unable to allocate " << endl
				 << "    memory for the m_pSavedObjLoc array.  The " << endl
				 << "    dynamic object data structures cannot be " << endl
				 << "    refreshed efficiently." << endl;
		}
		else
		{
            for (int i = 0; i < cNUM_DYN_OBJS; i++) {
                m_pSavedObjLoc[i].boundBox.SetMin(CPoint2D());
                m_pSavedObjLoc[i].boundBox.SetMax(CPoint2D());
                m_pSavedObjLoc[i].pDynObjRefs = nullptr;
                m_pSavedObjLoc[i].same = false;
                m_pSavedObjLoc[i].valid = false;
            }
			int i;
			for( i = 0; i < cNUM_DYN_OBJS; ++i )
			{
				m_pSavedObjLoc[i].pDynObjRefs =
					new cvTDynObjRef[cCV_NUM_DOR_REPS];
				memset( 
					m_pSavedObjLoc[i].pDynObjRefs, 
					0, 
					cCV_NUM_DOR_REPS * sizeof(cvTDynObjRef)
					);
			}
		}
	} // if m_pSavedObjLoc hasn't been allocated

	//
	// Clear out the road, intersection, and dynamic object reference lists.
	//
	memset( m_pRoadRefPool, 0, m_pHdr->roadRefCount * sizeof(cvTRoadRef) );
	memset( 
		m_pIntrsctnRefPool, 
		0, 
		m_pHdr->intrsctnRefCount * sizeof(cvTIntrsctnRef)
		);
	memset( 
		m_pDynObjRefPool, 
		0, 
		m_pHdr->dynObjRefCount * sizeof(cvTDynObjRef)
		);

#ifdef DYN_OBJ_REF_DEBUG
	// 
	// Iterate through the road, intrsctn, and dynobjref pools and print out.
	//
	FILE* pOut = fopen( "dor.txt", "a" );
	if( !pOut )
	{
		cerr << "Unable to open DOR debug file." << endl;
	}
	else
	{
		fprintf(pOut, "\n\nFrame = %d\n", m_pHdr->frame);

		int i;
		for( i = 0; i < m_pHdr->roadRefCount; i++ )
		{
			if( m_pRoadRefPool[i].objIdx != 0 )
			{
				fprintf(
					pOut, 
					"RoadRefPool[%d].objIdx = %d\n", 
					i, 
					m_pRoadRefPool[i].objIdx
					);
			}
		}

		for( i = 0; i < m_pHdr->intrsctnRefCount; i++ )
		{
			if( m_pIntrsctnRefPool[i].objIdx != 0 )
			{
				fprintf(
					pOut, 
					"IntrsctnRefPool[%d].objIdx = %d\n", 
					i, 
					m_pIntrsctnRefPool[i].objIdx
					);
			}
		}

		for( i = 0; i < m_pHdr->dynObjRefCount; i++ )
		{
			if( m_pDynObjRefPool[i].objId != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].objId = %d\n", 
					i, 
					m_pDynObjRefPool[i].objId
					);
			if( (int)m_pDynObjRefPool[i].terrain != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].terrain = %d\n", 
					i, 
					(int)m_pDynObjRefPool[i].terrain
					);
			if( m_pDynObjRefPool[i].next != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].next = %d\n", 
					i, 
					m_pDynObjRefPool[i].next
					);

			if( m_pDynObjRefPool[i].roadId != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].roadId = %d\n", 
					i, 
					m_pDynObjRefPool[i].roadId
					);
			if( m_pDynObjRefPool[i].lanes != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].lanes = %ld\n", 
					i, 
					m_pDynObjRefPool[i].lanes
					);
			if( m_pDynObjRefPool[i].distance != 0.0f )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].distance = %lf\n", 
					i, 
					m_pDynObjRefPool[i].distance
					);
			if( (int)m_pDynObjRefPool[i].direction != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].direction = %d\n", 
					i, 
					(int)m_pDynObjRefPool[i].direction
					);
			if( m_pDynObjRefPool[i].roadCntrlPnt != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].roadCntrlPnt = %d\n", 
					i, 
					m_pDynObjRefPool[i].roadCntrlPnt
					);

			if( m_pDynObjRefPool[i].intrsctnId != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].intrsctnId = %d\n", 
					i,
					m_pDynObjRefPool[i].intrsctnId
					);
			if( m_pDynObjRefPool[i].corridors != 0 )
				fprintf(
					pOut, 
					"DynObjRefPool[%d].corridors = %ld\n", 
					i, 
					m_pDynObjRefPool[i].corridors);
			int j;
			for( j = 0; j < cCV_MAX_CRDRS; j++) 
			{
				if( m_pDynObjRefPool[i].crdrDistances[j] != 0.0f)
					fprintf(
						pOut, 
						"DynObjRefPool[%d].crdrDistances[%d] = %lf\n", 
						i, 
						j, 
						m_pDynObjRefPool[i].crdrDistances[j]
						);
			}
		}
		fclose( pOut );
	}
#endif


#ifdef DEBUG_DYN_OBJ_LIST
	gout << "--- FillByRoadDynObjList " << m_pHdr->frame;
	gout << " -----------------------------" << endl;
#endif

	//
	// For each object in the saved object location array, check to see
	// if it's position or orientation has changed.
	//
	if( m_pSavedObjLoc )
	{
		TObjectPoolIdx objId;
		for( objId = 0; objId < cNUM_DYN_OBJS; ++objId )
		{
			m_pSavedObjLoc[objId].same = false;
			if( m_pSavedObjLoc[objId].valid )
			{
				//
				// Compare bounding boxes to figure out if the object
				// position or orientation has changed.
				//
				objBBox = GetObjBoundBox( objId );
				bool same = ( 
					( objBBox.GetMin() == m_pSavedObjLoc[objId].boundBox.GetMin() ) &&
					( objBBox.GetMax() == m_pSavedObjLoc[objId].boundBox.GetMax() )
					);
				m_pSavedObjLoc[objId].same = same;
			} // if saved object is valid
		} // for each saved object
	}


	TObjectPoolIdx dorIdx;
	int i;

	//
	// For each live dynamic object in the simulation.
	//
	GetAllDynamicObjs( dynObjs );
	for( objItr = dynObjs.begin(); objItr != dynObjs.end(); ++objItr )
	{
#ifdef DYN_OBJ_REF_DEBUG
		bool debugThisObj = *objItr == 0;
		if ( debugThisObj ) 
		{
			gout << "looking at obj " << *objItr << endl;
			gout << "  same = " << m_pSavedObjLoc[*objItr].same << endl;
		}
#endif

		bool objNotMoved = m_pSavedObjLoc && m_pSavedObjLoc[*objItr].same;
		if( objNotMoved ) 
		{
			//
			// Object hasn't moved.
			//
			// Copy data for each of the instances of the object in the 
			// dynamic object reference pool into DOR pool and re-link 
			// the nodes into list.
			//
			for( i = 0; i < cCV_NUM_DOR_REPS; ++i )
			{
				dorIdx = i * cNUM_DYN_OBJS + (*objItr) + 1;
				m_pDynObjRefPool[dorIdx] = 
					m_pSavedObjLoc[*objItr].pDynObjRefs[i];
				bool haveTerrain =
					m_pSavedObjLoc[*objItr].pDynObjRefs[i].terrain != eTERR_NONE;
				if( haveTerrain ) 
				{
					LinkDorIntoList( dorIdx );
				}
			}
		} // If object has not moved
		else 
		{	
			//
			// Object has moved.
			//
			pObj = BindObj( *objItr );
			objBBox = GetObjBoundBox( pObj->myId );

#ifdef DYN_OBJ_REF_DEBUG
		if ( debugThisObj ) 
		{
			gout << "  obj has moved. " << endl;
		}
#endif

			// Figure out if the object is on a road network.
			bool onRoadNetwork = IsObjOnRoadNetwork( pObj );

#ifdef DYN_OBJ_REF_DEBUG
		if ( debugThisObj ) 
		{
			gout << "  onRoadNetwork = " << onRoadNetwork << endl;
		}
#endif

			if( onRoadNetwork )
			{
				if( m_pSavedObjLoc )
				{
					//
					// Update the data in m_pSavedObjLoc to reflect the
					// object's current position.
					//
					m_pSavedObjLoc[pObj->myId].boundBox = objBBox;
					m_pSavedObjLoc[pObj->myId].valid = true;

					int dynObjRefPoolIdx;
					for( i = 0; i < cCV_NUM_DOR_REPS; i++ )
					{
						dynObjRefPoolIdx = (
									( i * cNUM_DYN_OBJS ) + 
									( pObj->myId + 1 )
									);
						m_pSavedObjLoc[pObj->myId].pDynObjRefs[i] = 
							m_pDynObjRefPool[dynObjRefPoolIdx];
					}
				}
			}
			else if( m_pSavedObjLoc )
			{
				m_pSavedObjLoc[pObj->myId].valid = false;
			}

		} // if object has moved

	} // For each live dynamic object

	if( debug )
	{
		ofstream out("testDOR.txt", ios::app);
		DumpDynObjRefLists(out);
		out.close();
	}

} // end of FillByRoadDynObjList

//////////////////////////////////////////////////////////////////////////////
//
// Description: DumpDynObjRefLists (private)
// 	Dump the contents of the road and intersection reference lists to gout.  
// 	It also prints the frame number for this iteration of the simulation.
//
// Remarks: This function iterates through both the road reference and the 
// 	intersection reference lists and prints the dynamic object references 
// 	found there.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCved::DumpDynObjRefLists(ostream &out) {

	// Iterate through all dyn obj ref lists and print out their contents
	unsigned int itr, crdr;

	cvTRoad *pRoad;
	cvTIntrsctn *pIsec;
	CCrdr corridor;
	CRoad road;

	char *pCharPool	= reinterpret_cast<char*>(m_pHdr) + m_pHdr->charOfs;

	out << "DumpDynObjRefLists: frame" << m_pHdr->frame << endl;
	
	// For each road reference
	pRoad = BindRoad(0);
	for (itr = 0; itr < m_pHdr->roadRefCount; ++itr, ++pRoad) {
		
		// Print out all objects on the road 
		int curObjIdx = m_pRoadRefPool[itr].objIdx;
		if (curObjIdx != 0) {

			out << "    Road[" << pRoad->myId << "]: " 
				<< pCharPool+pRoad->nameIdx << endl;
			while (curObjIdx != 0) {
			
				bitset<cCV_MAX_LANES> laneMask((int)m_pDynObjRefPool[curObjIdx].lanes);
				out << "        obj[" << m_pDynObjRefPool[curObjIdx].objId << "]:"
#ifndef _PowerMAXOS
					//no ostream operator << for bitset on concurrent alpha5
					 << "  lanes: " << laneMask 
#endif
					 << "  distance: " << m_pDynObjRefPool[curObjIdx].distance
					 << "  direction: " << m_pDynObjRefPool[curObjIdx].direction
					 << endl;

				curObjIdx = m_pDynObjRefPool[curObjIdx].next;
			}
		}
	} // For each road reference

	// For each intersection reference
	pIsec = BindIntrsctn(0);
	for (itr = 0; itr < m_pHdr->intrsctnRefCount; ++itr, ++pIsec) {
		
		// Print out all objects on the road 
		int curObjIdx = m_pIntrsctnRefPool[itr].objIdx;
		if (curObjIdx != 0) {
			out << "    Isec[" << pIsec->myId << "]: " 
				<< pCharPool+pIsec->nameIdx << endl;
			while (curObjIdx != 0) {
			
				bitset<cCV_MAX_CRDRS> 
					crdrMask((int)m_pDynObjRefPool[curObjIdx].corridors);
				out << "        obj[" << m_pDynObjRefPool[curObjIdx].objId << "]:"
#ifndef _PowerMAXOS
					//no ostream operator << for bitset on concurrent alpha5
					 << "  corridors: " << crdrMask 
#endif
					<< endl;

				// For each corridor
				for (crdr = 0; crdr < cCV_MAX_CRDRS; ++crdr) {
				
					// If obj is on this corridor
					if (crdrMask[crdr]) {
						corridor = CCrdr(*this, pIsec->crdrIdx+crdr);
						out << "      crdr[" << pIsec->crdrIdx+crdr 
							<< "]: from (";
						road = corridor.GetSrcRd();
						out << road.GetName() << ", " 
							<< corridor.GetSrcLn().GetRelativeId() - road.GetLaneIdx()  
							 << ") to (";
						road = corridor.GetDstntnRd();
						out << road.GetName() << ", " 
							<< corridor.GetDstntnLn().GetRelativeId() - 
								road.GetLaneIdx()
							<< ") at " 
							<< m_pDynObjRefPool[curObjIdx].crdrDistances[crdr] 
							<< endl;
					} // If obj is on this corridor
				} // For each corridor

				// Increment curObjIdx
				curObjIdx = m_pDynObjRefPool[curObjIdx].next;
			}
		}
	} // For each intersection reference

} // end of DumpDynObjRefLists

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the ID of the next available slot in the 
//  dynamic object reference pool for the objId parameter.  
//				
// Remarks: The DOR pool has a fixed number of references per dynamic 
//  object in the simulation.  Therefore the proper slots for the 
//  object with an ID of objId occur at cNUM_DYN_OBJS intervals.  If 
//  there aren't enough slots for the number of times the object is 
//  referenced in the pool, then 0 is returned.  Or, if the object is 
//  already referenced on the given road, then -1 is returned.  
//  Otherwise, the slot ID is returned.
//
// Arguments:
// 	objId - Index of the object in the object pool.
//
// Returns: int - 0 if there are not enough slots in the pool
//	idx if a proper slot was found in the pool.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::FindDorIdx( TObjectPoolIdx objId ) 
{
	TObjectPoolIdx tmpIdx;
	int repItr;

	// For each repetition of a dynamic object's reference
	for( repItr = 0; repItr < cCV_NUM_DOR_REPS; ++repItr )
	{
		tmpIdx = ( repItr * cNUM_DYN_OBJS ) + ( objId + 1 );

		bool foundOpenSlot = (
				m_pDynObjRefPool[tmpIdx].next == 0 && 
				m_pDynObjRefPool[tmpIdx].terrain == eTERR_NONE
				);
		if( foundOpenSlot )
		{
			// EXIT: Found an available slot
			return tmpIdx;
		}
#ifdef DYN_OBJ_REF_DEBUG
		else 
		{
			FILE* pOut = fopen( "dor.txt", "a" );
			if( !pOut )  cerr << "Unable to open DOR debug file." << endl;

			fprintf( pOut, "objId = %d, repItr = %d\n", objId, repItr );
			fprintf( pOut, "DynObjRefPool[%d]:\n", tmpIdx );

			cvTDynObjRef* pDorNode = &( m_pDynObjRefPool[tmpIdx] );

			if( pDorNode->objId != 0 )
			{
				fprintf( pOut, "  objId = %d\n", pDorNode->objId );
			}
			if( (int)pDorNode->terrain != 0 )
			{
				fprintf( pOut, "  terrain = %d\n", (int)pDorNode->terrain );
			}
			if( pDorNode->next != 0 )
			{
				fprintf( pOut, "  next = %d\n", pDorNode->next );
			}

			if( pDorNode->roadId != 0 )
			{
				fprintf( pOut, "  roadId = %d\n", pDorNode->roadId );
			}
			if( pDorNode->lanes != 0 )
			{
				fprintf( pOut, "  lanes = %ld\n", pDorNode->lanes );
			}
			if( pDorNode->distance != 0.0f )
			{
				fprintf( pOut, "  distance = %lf\n", pDorNode->distance );
			}
			if( (int)pDorNode->direction != 0 )
			{
				fprintf( pOut, "  direction = %d\n", (int)pDorNode->direction );
			}
			if( pDorNode->roadCntrlPnt != 0 )
			{
				fprintf( pOut, "  roadCntrlPnt = %d\n", pDorNode->roadCntrlPnt );
			}

			if( pDorNode->intrsctnId != 0 )
			{
				fprintf( pOut, "  intrsctnId = %d\n", pDorNode->intrsctnId );
			}
			if( pDorNode->corridors != 0 )
			{
				fprintf( pOut, "  corridors = %ld\n", pDorNode->corridors );
			}
			int j;
			for( j = 0; j < cCV_MAX_CRDRS; j++ )
			{
				if( pDorNode->crdrDistances[j] != 0.0f )
				{
					fprintf( pOut, "  crdrDistances[%d] = %lf\n", j, pDorNode->crdrDistances[j] );
				}
			}
			fprintf( pOut, "\n" );
			fclose( pOut );
		}
#endif

	} 	// For each repetition of a dynamic object's reference
	
	//
	// EXIT::Not enough slots in the pool, so print error and return 0.
	//
	cerr << "Not enough room in dynamic object reference pool for object ";
	cerr << objId << ".\n  Please increase value of cCV_NUM_DOR_REPS.";
	cerr << endl;

	return 0;

} // end of FindDorIdx

//////////////////////////////////////////////////////////////////////////////
//
// Description: Determines the proper position in the road reference 
//  list for the dynamic object reference at objIdx in the pool.  The 
//  DOR node is then linked into the proper road list.
//
// Remarks: This function treats the road reference items as the 
//  beginning of a linked list of dynamic object reference items.  
//  The nodes in the list are ordered by their distance parameter, 
//  and linked using the index of the next item in the list.  This 
//  function only does the linking, not the initialization of the 
//  DOR node. 
//
// Arguments: 
// 	dorIdx - Index of the object to be linked into the dynamic object 
// 		reference pool.
//
// Returns: void
// 
//////////////////////////////////////////////////////////////////////////////
void
CCved::LinkDorIntoList(TObjectPoolIdx dorIdx) {


#ifdef LINK_DOR_INTO_LIST
	gout << endl;
	gout << "LinkDorIntoList========LDIL " << endl;
#endif
	
	// If the object is on a road

	if (m_pDynObjRefPool[dorIdx].terrain == eTERR_ROAD) {

		TRoadPoolIdx roadIdx = m_pDynObjRefPool[dorIdx].roadId;
		TObjectPoolIdx curIdx, nextIdx;
		bool inserted;

		// Check to see if the object should be 
		//	inserted at the front of the list.
		curIdx = m_pRoadRefPool[roadIdx].objIdx;

		if ((curIdx == 0) ||						// The road's list is empty
			(m_pDynObjRefPool[dorIdx].distance <	// or the Object's distance   
			 m_pDynObjRefPool[curIdx].distance))	// is smallest in the list
		{
			// Object belongs at the front of the list.
			m_pDynObjRefPool[dorIdx].next = curIdx;
			m_pRoadRefPool[roadIdx].objIdx = dorIdx;
		}
		else {	// Object belongs in body of list somewhere.

			// Trace through dyn obj ref list to find 
			//	where the object belongs
			inserted = false;
			while (!inserted) {

				nextIdx = m_pDynObjRefPool[curIdx].next;
				if ((nextIdx == 0) ||						// At end of list or
					(m_pDynObjRefPool[dorIdx].distance <	// Object belongs 
					 m_pDynObjRefPool[nextIdx].distance)) 	// just after 
															// curIdx's object
				{
					m_pDynObjRefPool[dorIdx].next = nextIdx;
					m_pDynObjRefPool[curIdx].next = dorIdx;
					inserted = true;
				}
				else
					curIdx = nextIdx;
			
			} // while (!inserted)

		}// else search through list body

	} // If the object is on a road

	// If the object is on an intersection
	else if (m_pDynObjRefPool[dorIdx].terrain == eTERR_ISEC) {
		TIntrsctnPoolIdx isecIdx = m_pDynObjRefPool[dorIdx].intrsctnId;
		TObjectPoolIdx curIdx, nextIdx;
		bool inserted;

		// Check to see if the object should be 
		//	inserted at the front of the list
		curIdx = m_pIntrsctnRefPool[isecIdx].objIdx;

		if ((curIdx == 0) ||
			(m_pDynObjRefPool[dorIdx].corridors <
			 m_pDynObjRefPool[curIdx].corridors))
		{
			// Object belongs at the front of the list.
			m_pDynObjRefPool[dorIdx].next = curIdx;
			m_pIntrsctnRefPool[isecIdx].objIdx = dorIdx;
		}
		else {	// Object belongs in the body of the list somewhere

			// Trace through dyn obj ref list to find
			//	where the object belongs
			inserted = false;
			while (!inserted) {

				nextIdx = m_pDynObjRefPool[curIdx].next;
				if ((nextIdx == 0) ||						// At end of list or
					(m_pDynObjRefPool[dorIdx].corridors <	// obj belongs just
					 m_pDynObjRefPool[nextIdx].corridors)) 	// after curIdx
				{
					 m_pDynObjRefPool[dorIdx].next = nextIdx;
					 m_pDynObjRefPool[curIdx].next = dorIdx;
					 inserted = true;
				}
				else
					curIdx = nextIdx;
			} // while (!inserted)

		} // else search through list body

	} // If the object is on an intersection

} // end of LinkDorIntoList

//////////////////////////////////////////////////////////////////////////////
//
// Description: Determines whether the given object is on the road network.
//
// Remarks:  Finds the parameter pObj's position with respect to the road 
// 	network.  Once it is found, the object's reference has its local 
// 	variables set to the proper values and it is inserted into its road 
// 	reference and/or intersection reference list.
//
// Arguments: 
// 	cpObj - Pointer to the object data structure of the object to check.
//
// Returns: true if the object overlaps a road or intersection, 
// 	false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsObjOnRoadNetwork( const cvTObj* cpObj )
{
#ifdef _DEBUG
	cvTObj tobj; //for the benifit of the debugger
    tobj.myId =-1;
#endif
    string spc = "  ";
#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside IsObjOnRoadNetwork" << endl;
		spc += "  ";
	}
#endif

	//// Variables associated with the object ////
	cvTObjState curState;
	GetObjState( cpObj->myId, curState );
	CPoint3D objCenter( curState.anyState.position );
	CVector3D tanVector( curState.anyState.tangent );
	CVector3D latVector( curState.anyState.lateral );
	double xSize = cpObj->attr.xSize;
	double ySize = cpObj->attr.ySize;
	double zSize = cpObj->attr.zSize;

#ifdef DEBUG_DYN_OBJ_LIST
	if( debugThisFrame )
	{
		gout << spc << "objCenter = " << objCenter << endl;
	}
#endif


	/*static*/ vector<int> intrsctns;
	intrsctns.clear();
	intrsctns.reserve(30);

	vector<int>::const_iterator isecItr;
	cvTIntrsctn* pIntrsctn;

	/*static*/ vector<int> rdPcs;
	rdPcs.clear();
	rdPcs.reserve(30);

	vector<int>::const_iterator rdPcItr;
	cvTRoadPiece* pRdPc;
	/*static*/ vector<TRoadPoolIdx> foundRoads;
	foundRoads.clear();
	foundRoads.reserve(30);

	// Generate the vertices of the object quadrangle.
	CPoint3D quad[4];
	GetFourCornerOfObj( cpObj->myId, quad, &objCenter );

	cvTDynObjRef tmpDor = {0};
	tmpDor.objId = cpObj->myId;

	int numPlaces = 0;
	if( 0 && m_pSavedObjLoc && m_pSavedObjLoc[cpObj->myId].valid )
	{
		numPlaces += SearchAroundPrevLocation(
									spc,
									cpObj->myId, 
									objCenter, 
									xSize, 
									ySize, 
									zSize, 
									quad
									);

#ifdef DEBUG_DYN_OBJ_LIST
		if( debugThisFrame )
		{
			gout << spc << "the saved obj is valid" << endl;
			gout << spc << "numPlaces = " << numPlaces << endl;
		}
#endif
	} // If saved object is valid

	// 
	// If the object wasn't found yet, search for it in the entire 
	// road network.
	//
	int dorIdx;
	if( numPlaces == 0 )
	{

#ifdef DEBUG_DYN_OBJ_LIST
		if( debugThisFrame )
		{
			gout << spc << "obj not found yet... have to search for it";
			gout << endl;
		}
#endif

		// Find the intersections the object overlaps
		m_intrsctnQTree.SearchRectangle(
								curState.anyState.boundBox[0].x,
								curState.anyState.boundBox[0].y,
								curState.anyState.boundBox[1].x,
								curState.anyState.boundBox[1].y,
								intrsctns
								);
		for( 
			isecItr = intrsctns.begin(); 
			isecItr != intrsctns.end(); 
			++isecItr
			)
		{
			pIntrsctn = CCved::BindIntrsctn( *isecItr );

			// Find a slot for the dynamic object reference in the pool
			dorIdx = FindDorIdx( cpObj->myId );
			// If the returned dorIdx is valid
			if( dorIdx > 0 )
			{
				// If the object overlaps the intersection
				bool objOnIntrsctn = IsObjOnIntrsctn(
												spc,
												pIntrsctn, 
												objCenter,
												xSize, 
												ySize, 
												zSize, 
												quad,
												tmpDor
												);
				if( objOnIntrsctn ) 
				{
					//
					// Check to see if the DOR list already contains
					// this intersection with this object.
					//
					//gout << "### trying to insert obj " << cpObj->myId << " on isec " << pIntrsctn->myId << endl;
					int i;
					bool foundIsec = false;
					for( i = 0; i < cCV_NUM_DOR_REPS; i++ )
					{
						int dynObjRefPoolIdx = (
									( i * cNUM_DYN_OBJS ) + 
									( cpObj->myId + 1 )
									);
						bool isecMatch = (
								m_pDynObjRefPool[dynObjRefPoolIdx].intrsctnId ==
								pIntrsctn->myId
								);
						if( isecMatch )
						{
							foundIsec = true;
							break;
						}
					}
					//gout << "    foundIsec = " << foundIsec << endl;
					if( !foundIsec )
					{
						//
						// The DOR list doesn't already contain this
						// intersection so add it to the list.
						//
						tmpDor.terrain = eTERR_ISEC;
						tmpDor.intrsctnId = pIntrsctn->myId;
						m_pDynObjRefPool[dorIdx] = tmpDor;

						// Link object into list
						LinkDorIntoList( dorIdx );
					}

					numPlaces++;

#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "found on intersection:  intrId = ";
						gout << pIntrsctn->myId << endl;
					}
#endif
				} // If object overlaps the intersection
			} // If the returned dorIdx is valid
		}

		// Find the road pieces the object overlaps
		m_rdPcQTree.SearchRectangle(
							curState.anyState.boundBox[0].x, 
							curState.anyState.boundBox[0].y,
							curState.anyState.boundBox[1].x, 
							curState.anyState.boundBox[1].y,
							rdPcs
							);

		// For each road piece in the result
		for( rdPcItr = rdPcs.begin(); rdPcItr != rdPcs.end(); ++rdPcItr ) 
		{
			pRdPc = BindRoadPiece( *rdPcItr );

			// Find a slot for the dynamic object reference in the pool
			dorIdx = FindDorIdx( cpObj->myId );

			// The returned dorIdx is valid
			if( dorIdx > 0 )
			{
				// If the object has not already been found on this road
				if( find( foundRoads.begin(), foundRoads.end(), pRdPc->roadId ) == foundRoads.end() )
				{
					// If the object is found on this road piece
					bool objOnRoadPiece = IsObjOnRoadPiece(
													spc,
													pRdPc, 
													objCenter, 
													xSize, 
													ySize, 
													zSize, 
													quad,
													tmpDor
													);
					if( objOnRoadPiece )
					{
						// Set object reference values
						tmpDor.terrain = eTERR_ROAD;
						tmpDor.roadId = pRdPc->roadId;
						m_pDynObjRefPool[dorIdx] = tmpDor;

						// Link object into list.
						LinkDorIntoList( dorIdx );

						foundRoads.push_back( pRdPc->roadId );
						numPlaces++;

#ifdef DEBUG_DYN_OBJ_LIST
						if( debugThisFrame )
						{
							gout << spc << "found on roadPc:  roadId = ";
							gout << pRdPc->roadId << endl;
						}
#endif
					} // If the object is found on this road piece

				} // If the object has not already been found on this road

			} // dorIdx is valid 

		} // for each road piece

	} // If the object wasn't found yet.

	return( numPlaces > 0 );

} // end of IsObjOnRoadNetwork


//////////////////////////////////////////////////////////////////////////////
//
// Description: Searches around the previous location of the dynamic 
//  object to find out where the object is now.   
//				
// Remarks: The area around each location stored in the saved object array is 
// 	checked.  First, the road segment or intersection the object previously 
// 	occupied is checked, then the one in the direction the object was headed, 
// 	then the one behind.  
//
// Arguments:
//  objId - Index of the object in the object pool.
//  center - Current center point of the object.
//  xSize, ySize, zSize - Current sizes of the object.
//  quad - Current bounding quadrilateral of the object.
//
// Returns: int - -1 if no space was found in the DOR pool for the object
//  else the number of places where the object was found.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::SearchAroundPrevLocation(
			const string& s,
			TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			double xSize, 
			double ySize,
			double zSize,
			const CPoint3D cQuad[]
			)
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside SearchAroundPrevLocation" << endl;
		spc += "  ";
	}
#endif

	double distChecked = 0.0;
	TRoadPoolIdx roadId = 0;
	TIntrsctnPoolIdx isecId = 0; 
	TLongCntrlPntPoolIdx roadCntrlPntIdx = 0;
	double distToPrevCntrlPnt = 0.0;
	int numPlaces = 0; //our total count of found locations
    int justFound = 0; //our immediate result from each query
	int repItr;
	/*static*/ vector<TRoadPoolIdx> foundRoads;
	foundRoads.clear();
	foundRoads.reserve(16);
	cvTCntrlPnt* pFirstCp;

	//
	// Iterate for each DOR (dynamic object reference) repitition.
	//
	for( repItr = 0; repItr < cCV_NUM_DOR_REPS; ++repItr )
	{
#ifdef DEBUG_DYN_OBJ_LIST
		if( debugThisFrame )
		{
			gout << "  repItr = " << repItr << endl;
		}
#endif

		cvTDynObjRef* pCurrDynObjRef = 
							&( m_pSavedObjLoc[objId].pDynObjRefs[repItr] );
		if( pCurrDynObjRef->terrain == eTERR_ROAD )
		{
			//
			// This dynamic object reference is on a road.
			//
			roadId = pCurrDynObjRef->roadId;
			isecId = 0;
			roadCntrlPntIdx = pCurrDynObjRef->roadCntrlPnt;
			pFirstCp = BindCntrlPnt( roadCntrlPntIdx );
			distToPrevCntrlPnt = -fabs( 
						pCurrDynObjRef->distance - 
						pFirstCp->cummulativeLinDist 
						);

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "##on a road" << endl;
				gout << spc << "roadId = " << roadId << endl;
				gout << spc << "roadCntrlPntIdx = " << roadCntrlPntIdx << endl;
				gout << spc << "distToPrevCntrlPnt = " << distToPrevCntrlPnt;
				gout << "  =  -( " << pCurrDynObjRef->distance << " - ";
				gout << pFirstCp->cummulativeLinDist << " )" << endl;
			}
#endif

			if( pCurrDynObjRef->direction == ePOS )
			{
				//
				// On a positive direction lane.  Search forward first.
				//
#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << "    ePOS lane...SearchForward" << endl;
				}
#endif

				distChecked = distToPrevCntrlPnt;
                justFound = SearchForward(
									spc,
									objId, 
									cCenter, 
									xSize, 
									ySize, 
									zSize, 
									cQuad,
									distChecked, 
									roadId, 
									isecId, 
									roadCntrlPntIdx, 
									foundRoads
									);

				// If the object was not found before, then search backward.
				if( justFound == 0 )
				{
#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "nothing found...SearchBackward";
						gout << endl;
					}
#endif

					distChecked = distToPrevCntrlPnt;
                    roadId = pCurrDynObjRef->roadId; //if we hit a corr in the last search this can change roadId
					justFound = SearchBackward(
										spc,
										objId, 
										cCenter, 
										xSize, 
										ySize, 
										zSize, 
										cQuad, 
										distChecked, 
										roadId, 
										isecId,  
										roadCntrlPntIdx, 
										foundRoads
										);
				} // If the object was not found before, then search backward
                numPlaces += justFound;
			

#ifdef DEBUG_DYN_OBJ_LIST			
				if( debugThisFrame )
				{
					gout << spc << "numPlaces = " << numPlaces << endl;
				}
#endif 
			} // If the object is moving forward, search forward first
			else 
			{
				//
				// On a negative direction lane, search backward first.
				//
#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "eNEG lane....SearchBackward" << endl;
				}
#endif
              
				distChecked = distToPrevCntrlPnt;
				justFound = SearchBackward(
									spc,
									objId, 
									cCenter, 
									xSize, 
									ySize, 
									zSize, 
									cQuad, 
									distChecked, 
									roadId, 
									isecId,  
									roadCntrlPntIdx, 
									foundRoads
									);

				// If the object was not found behind, then search forward
				if( justFound == 0 ) 
				{

#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "nothing found...SearchForward";
						gout << endl;
					}
#endif
                    roadId = pCurrDynObjRef->roadId; //if we hit a corr in the last search this can change roadId
					distChecked = distToPrevCntrlPnt;
					justFound= SearchForward(
										spc,
										objId, 
										cCenter,
										xSize, 
										ySize, 
										zSize, 
										cQuad,
										distChecked, 
										roadId, 
										isecId,
										roadCntrlPntIdx, 
										foundRoads
										);
				} // If the object was not found behind, then search forward
                numPlaces+= justFound;
#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "numPlaces = " << numPlaces << endl;
				}
#endif
			} // If the object is moving backward, search backward first

		} // If this dyn obj ref is on a road

		// If this dyn obj ref is on an intersection
		else if( pCurrDynObjRef->terrain == eTERR_ISEC ) 
		{
			//
			// This dynamic object reference is on an intersection.
			//
			isecId = pCurrDynObjRef->intrsctnId;
			roadId = 0;
			distChecked = 0.0;

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "##on an intersection" << endl;
				gout << spc << "isecId = " << isecId << endl;
			}
#endif

			//
			// We must only search forward for intersections.  The
			// SearchIsec function takes care of looking backward.
			//
			numPlaces += SearchForward(
								spc,
								objId, 
								cCenter, 
								xSize, 
								ySize, 
								zSize, 
								cQuad, 
								distChecked, 
								roadId, 
								isecId, 
								roadCntrlPntIdx, 
								foundRoads
								);

		} // If this dyn obj ref is on an intersection
		else 
		{
			//
			// This dynamic object reference is off the road network.
			//
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "##off the road" << endl;
			}
#endif
			// Continue on to next repetition.
			continue;
		}

	} // For each DOR repetition

	return numPlaces;

} // end of SearchAroundPrevLocation

//////////////////////////////////////////////////////////////////////////////
//
// Description: Searches forward from the previous location into the 
//  roads and intersections DIST_TO_CHECK meters, until the object is 
//  found on one or more roads or intersections.
//
// Remarks: 
//
// Arguments:	
//	Input:
//		objId - Index of the object in the object pool.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize, zSize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Input/Output:
//		distChecked - The distance from the original position that has 
//						already been searched for the object.
//		roadId - The index of the road to check for the object.
//		isecId - The index of the intersection to check for the object.
//		roadCntrlPntIdx - Contains the control point of the road segment 
//                      to search.
//
// Returns: int - The number of places where the object was found.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::SearchForward(
			const string& s,
			const TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			const double xSize,
			const double ySize,
			const double zSize, 
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPntIdx, 
			vector<TRoadPoolIdx>& foundRoads
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside SearchForward" << endl;
		spc += "  ";
	}
#endif			

	int numPlaces = 0;

	//
	// While there's distance left to be searched and the object 
	// has yet to be found.
	//
	while( ( distChecked < DIST_TO_CHECK ) && ( numPlaces == 0 ) )
	{
		if( roadId != 0 )
		{
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "before SRF r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked << endl;
			}
#endif			
			numPlaces += SearchRoadForward(
								spc,
								objId, 
								cCenter, 
								xSize, 
								ySize, 
								zSize, 
								cQuad, 
								distChecked, 
								roadId, 
								isecId, 
								roadCntrlPntIdx, 
								foundRoads
								);
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "after  SRF r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked << endl;
			}
#endif		
		
		}
		else if( isecId != 0 ) 
		{

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "before SI  r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked << endl;
			}	
#endif			
			numPlaces += SearchIsec(
								spc,
								objId, 
								cCenter, 
								xSize, 
								ySize, 
								zSize, 
								cQuad, 
								distChecked, 
								roadId, 
								isecId, 
								roadCntrlPntIdx, 
								foundRoads
								);

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "after  SI  r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked << endl;
			}
#endif		
		}
	}

	return numPlaces;

} // end of SearchForward

//////////////////////////////////////////////////////////////////////////////
//
// Description: Searches backward from the previous location into the 
//  roads and intersections DIST_TO_CHECK meters, until the object is 
//  found on one or more roads or intersections.
//
// Remarks:
//
// Arguments:	
//	Input:
//		objId - Index of the object in the object pool.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize, zSize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Input/Output:
//		distChecked - The distance from the original position that has 
//						already been searched for the object.
//		roadId - The index of the road to check for the object.
//		isecId - The index of the intersection to check for the object.
//		roadCntrlPntIdx - Contains the control point of the road segment 
//                      to search.
//
// Returns: int - The number of places where the object was found.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::SearchBackward(
			const string& s,
			const TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			const double xSize,
			const double ySize,
			const double zSize, 
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPntIdx, 
			vector<TRoadPoolIdx>& foundRoads
			) 
{
	int numPlaces = 0;
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST	
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside SearchBackward" << endl;
		spc += "  ";
		gout << spc << "distChecked is: " << distChecked << endl;
		gout << spc << "distToCheck is: " << DIST_TO_CHECK << endl;
		gout << spc << "numPlaces is: " << numPlaces << endl;
	}
#endif

	//
	// While there's distance left to be searched and the object has 
	// yet to be found.
	//
	while( ( distChecked < DIST_TO_CHECK ) && ( numPlaces == 0 ) ) 
	{
		if( roadId != 0 )
		{
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "before SRB r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked;
				gout << endl;
			}
#endif			
			
			numPlaces += SearchRoadBackward(
								spc,
								objId, 
								cCenter, 
								xSize, 
								ySize, 
								zSize, 
								cQuad, 
								distChecked, 
								roadId, 
								isecId, 
								roadCntrlPntIdx, 
								foundRoads
								);
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "after  SRB r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked;
				gout << endl;
			}
#endif		
		
		}
		else if( isecId != 0 ) 
		{

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "before SI  r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked;
				gout << endl;
			}
			
#endif			
			numPlaces += SearchIsec(
								spc,
								objId, 
								cCenter, 
								xSize, 
								ySize, 
								xSize, 
								cQuad, 
								distChecked, 
								roadId, 
								isecId, 
								roadCntrlPntIdx, 
								foundRoads
								);

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "after  SI  r=" << roadId;
				gout << "  i=" << isecId << "  dc = " << distChecked;
				gout << endl;
			}
#endif		
		
		}
	}

	return numPlaces;

} // end of SearchBackward

//////////////////////////////////////////////////////////////////////////////
//
// Description: Searches he road forward from the previous location up 
//  to DIST_TO_CHECK meters, until the object is found on the road and/or 
//  joining intersection.
//
// Remarks: If the object is found on the road, but it is found to hang over 
//  onto the joining intersection, then that intersection is also searched.
//
// Arguments:	
//	Input:
//		objId - Index of the object in the object pool.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize, zSize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Input/Output:
//		distChecked - The distance from the original position that has 
//						already been searched for the object.
//		roadId - The index of the road to check for the object.
//		isecId - The index of the intersection to check for the object.
//		roadCntrlPntIdx - Contains the control point of the road segment 
//			            to search.
//
// Returns: int - -1 if no space was found in the DOR pool for the object
//				  else the number of places where the object was found.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::SearchRoadForward(
			const string& s,
			const TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			const double xSize,
			const double ySize,
			const double zSize, 
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPntIdx, 
			vector<TRoadPoolIdx>& foundRoads
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside SRF" << endl;
		spc += "  ";
	}
#endif

	int numPlaces = 0;
	cvTCntrlPnt* pFirstCp;
	cvTCntrlPnt* pNextCp;
	cvTRoad* pRoad;
	TIntrsctnPoolIdx tmpIsecId = 0;
	TObjectPoolIdx dorIdx;
	cvTDynObjRef tmpDor = {0};
	tmpDor.objId = objId;
	bool prevSeg = false;
	bool nextSeg = false;

	//
	// If the object hasn't already been found on this road.
	//
	vector<TRoadPoolIdx>::iterator i = find( 
											foundRoads.begin(), 
											foundRoads.end(), 
											roadId
											);
	bool foundOnThisRoad = ( i == foundRoads.end() );
	if( foundOnThisRoad ) 
	{
		pFirstCp = BindCntrlPnt( roadCntrlPntIdx );

		//
		// While there's distance left to be searched and the object 
		// has yet to be found.
		//
		while( ( distChecked < DIST_TO_CHECK ) && ( numPlaces == 0 ) ) 
		{
			pRoad = BindRoad( roadId );

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "SRF checking roadId=" << roadId;
				gout << "  startIdx=" << pRoad->cntrlPntIdx;
				gout << "   currIdx=" << roadCntrlPntIdx << endl;
			}
#endif

			unsigned int maxCntrlPntIdxOnRoad = pRoad->cntrlPntIdx + pRoad->numCntrlPnt - 1;
			bool cntrlPntOnRoad = roadCntrlPntIdx < maxCntrlPntIdxOnRoad;
			if( cntrlPntOnRoad )
			{
				pNextCp = pFirstCp + 1;

				distChecked += pFirstCp->distToNextLinear;

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "checking " << pFirstCp->location;
					gout << " to " << pNextCp->location << endl;
				}
#endif
			
				bool objOnRoadSeg = IsObjOnRoadSegment(
											spc,
											pFirstCp, 
											pNextCp,
											cCenter, 
											xSize, 
											ySize, 
											cQuad, 
											tmpDor, 
											prevSeg, 
											nextSeg
											);
				if( objOnRoadSeg )
				{
					//
					// This appears to be where the obj is found.
					//
#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "##object on road " << roadId;
						gout << " between cp1 and cp2" << endl;
					}
#endif

					numPlaces++;
					tmpDor.terrain = eTERR_ROAD;
					tmpDor.roadId = roadId;
					tmpDor.roadCntrlPnt = roadCntrlPntIdx;
					dorIdx = FindDorIdx( objId );

					// If the dorIdx is valid
					if( dorIdx > 0 )
					{ 
						m_pDynObjRefPool[dorIdx] = tmpDor;
						LinkDorIntoList(dorIdx);

						vector<TRoadPoolIdx>::iterator i = find( 
														foundRoads.begin(), 
														foundRoads.end(), 
														roadId 
														);
						bool roadNotFound = ( i == foundRoads.end() );
						if( roadNotFound )
						{
							foundRoads.push_back( roadId );
						}
					}
					// The dorIdx is invalid
					else
					{
						return numPlaces;
					}

					//
					// If the object overlaps into the next segment and
					// we're checking the last segment, then check the
					// destination intersection too.
					//
					if( nextSeg && ( roadCntrlPntIdx == pRoad->cntrlPntIdx + pRoad->numCntrlPnt - 2 ) ) 
					{
						tmpIsecId = pRoad->dstIntrsctnIdx;
						numPlaces += SearchIsec(
											spc,
											objId, 
											cCenter, 
											xSize, 
											ySize, 
											zSize, 
											cQuad, 
											distChecked, 
											roadId,  
											tmpIsecId, 
											roadCntrlPntIdx, 
											foundRoads
											);
					}

					return numPlaces;
				}
			}
			else 
			{
				roadId = 0;
				isecId = pRoad->dstIntrsctnIdx;
				return numPlaces;
			}

			pFirstCp = pNextCp;
			++roadCntrlPntIdx;
		}	// While there's distance left to be searched 
			//	and the object has yet to be found

	} // If the object hasn't already been found on this road
	else 
	{
		// Change distChecked to prevent infinite loop
		distChecked = DIST_TO_CHECK + 1.0;
	}

	return numPlaces;
	
} // end of SearchRoadForward

//////////////////////////////////////////////////////////////////////////////
//
// Description: Searches the road backward from the previous location up 
//  to DIST_TO_CHECK meters, until the object is found on the road and/or 
//  joining intersection.
//
// Remarks: If the object is found on the road, but it is found to hang over 
//	onto the joining intersection, then that intersection is also searched.
//
// Arguments:	
//	Input:
//		objId - Index of the object in the object pool.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize, zSize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Input/Output:
//		distChecked - The distance from the original position that has 
//			already been searched for the object.
//		roadId - The index of the road to check for the object
//		isecId - The index of the intersection to check for the object.
//		roadCntrlPntIdx - Contains the control point of the road segment .
//			to search.
//
// Returns: int - the number of places where the object was found.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::SearchRoadBackward(
			const string& s,
			const TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			const double xSize,
			const double ySize,
			const double zSize, 
			const CPoint3D cQuad[],
			double& distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPntIdx, 
			vector<TRoadPoolIdx>& foundRoads
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside SRB" << endl;
		spc += "  ";
	}
#endif

	int numPlaces = 0;
	cvTCntrlPnt* pFirstCp;
	cvTCntrlPnt* pNextCp;
	cvTRoad* pRoad;
	TIntrsctnPoolIdx tmpIsecId = 0;
	cvTDynObjRef tmpDor = {0};
	tmpDor.objId = objId;

	TObjectPoolIdx dorIdx;
	bool prevSeg = false;
	bool nextSeg = false;

	//
	// If the object hasn't already been found on this road.
	//
	vector<TRoadPoolIdx>::iterator i = find( 
										foundRoads.begin(), 
										foundRoads.end(), 
										roadId 
										);
	bool objNotFoundOnRoad = ( i == foundRoads.end() );
	if( objNotFoundOnRoad )
	{
		pNextCp = BindCntrlPnt( roadCntrlPntIdx );

		//
		// While there's distance left to be searched and the object 
		// has yet to be found		
		//
		while( ( distChecked < DIST_TO_CHECK ) && ( numPlaces == 0 ) )
		{
			pRoad = BindRoad(roadId);

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "roadId = " << roadId;
				gout << "   startIdx=" << pRoad->cntrlPntIdx;
				gout << "   currIdx=" << roadCntrlPntIdx << endl;
			}
#endif

			//
			// The roadCntrlPntIdx has to be the 2nd or greater 
			// control point on the road.
			//
			bool cntrlPntIdxOnRoad = roadCntrlPntIdx > pRoad->cntrlPntIdx;
			if( cntrlPntIdxOnRoad )
			{
				pFirstCp = pNextCp - 1;

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "distChecked = " << distChecked << endl;
				}
#endif

				distChecked += pFirstCp->distToNextLinear;

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "checking " << pFirstCp->location;
					gout << " to " << pNextCp->location << endl;
				}
#endif

				bool objOnRoadSeg = IsObjOnRoadSegment(
											spc,
											pFirstCp, 
											pNextCp,
											cCenter, 
											xSize, 
											ySize, 
											cQuad, 
											tmpDor, 
											prevSeg, 
											nextSeg
											);
				if( objOnRoadSeg )
				{
					//
					// This appears to be where the obj is found.
					//
#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "**object on roadId = " << roadId;
						gout << "   roadCntrlPntIdx = " << roadCntrlPntIdx;
						gout << "   prevSeg = " << prevSeg;
						gout << "   nextSeg = " << nextSeg;
						gout << endl;
					}
#endif

					numPlaces++;
					tmpDor.terrain = eTERR_ROAD;
					tmpDor.roadId = roadId;
					tmpDor.roadCntrlPnt = roadCntrlPntIdx;
					dorIdx = FindDorIdx( objId );

					// If the dorIdx is valid
					if( dorIdx > 0 )
					{ 
						m_pDynObjRefPool[dorIdx] = tmpDor;
						LinkDorIntoList( dorIdx );

						vector<TRoadPoolIdx>::iterator i = find( 
														foundRoads.begin(), 
														foundRoads.end(), 
														roadId 
														);
						bool roadNotFound = ( i == foundRoads.end() );
						if( roadNotFound )
						{
							foundRoads.push_back( roadId );
						}
					}
					// The dorIdx is invalid
					else
					{
						return numPlaces;
					}

					//
					// If the object overlaps into the prev segment and
					// we're checking the first segment, then check the
					// source intersection too.
					//]
					int localCntrlPntIdx = 
									roadCntrlPntIdx - pRoad->cntrlPntIdx;
					bool checkIntrsctn = prevSeg && localCntrlPntIdx <= 1;
//					if( prevSeg && ( roadCntrlPntIdx == 0 ) )
					if( checkIntrsctn )
					{
						tmpIsecId = pRoad->srcIntrsctnIdx;
						numPlaces += SearchIsec(
											spc,
											objId, 
											cCenter, 
											xSize, 
											ySize, 
											zSize,
											cQuad, 
											distChecked, 
											roadId,  
											tmpIsecId, 
											roadCntrlPntIdx, 
											foundRoads
											);
					}

					return numPlaces;
				}
			}
			else 
			{
				roadId = 0;
				isecId = pRoad->srcIntrsctnIdx;
				return numPlaces;
			}

			pNextCp = pFirstCp;
			--roadCntrlPntIdx;

		}	// While there's distance left to be searched 
			//	and the object has yet to be found		

	} // If the object hasn't already been found on this road
	else 
	{
		// Change distChecked to prevent infinite loop
		distChecked = DIST_TO_CHECK + 1.0;
	}

	return numPlaces; 

} // end of SearchRoadBackward

////////////////////////////////////////////////////////////////////////////////
//
// Description: Searches the intersection designated by the parameter 
//  for the object.  Because of the nature of the intersection searching 
//  method, there is no advantage to searching only a fixed number of 
//  meters ahead.
//
// Remarks: If the object is found on the intersection, but it is found to hang 
// 	over onto one or more joining roads, then those roads are also searched.
//
// Arguments:	
//	Input:
//		objId - Index of the object in the object pool.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize, zSize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Input/Output:
//		distChecked - The distance from the original position that has 
//						already been searched for the object.
//		roadId - The index of the road to check for the object.
//		isecId - The index of the intersection to check for the object.
//		roadCntrlPntIdx - Contains the control point of the road segment .
//                      to search.
//
// Returns: int - -1 if no space was found in the DOR pool for the object
//				  else the number of places where the object was found.
//
//////////////////////////////////////////////////////////////////////////////
int 
CCved::SearchIsec(
			const string& s,
			const TObjectPoolIdx objId,
			const CPoint3D& cCenter,
			const double xSize,
			const double ySize,
			const double zSize, 
			const CPoint3D cQuad[],
			double &distChecked, 
			TRoadPoolIdx& roadId,
			TIntrsctnPoolIdx& isecId, 
			TLongCntrlPntPoolIdx roadCntrlPntIdx, 
			vector<TRoadPoolIdx>& foundRoads
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside SI" << endl;
		spc += "  ";
	}
#endif
	int numPlaces = 0;

	/*static*/ vector<TRoadPoolIdx> overlapRoads;
	overlapRoads.clear();
	overlapRoads.reserve(30);

	/*static*/ vector<TRoadPoolIdx> searchedRoads;
	searchedRoads.clear();
	searchedRoads.reserve(30);

	bool onIntrsctn = false;
	TIntrsctnPoolIdx tmpIsecId = 0;	
	cvTIntrsctn* pIntrsctn;
	cvTRoad* pRoad;
	cvTCrdr* pCrdr;
	unsigned int crdr;
	char* pOfs;
	cvTCrdrCntrlPnt* pCrdrCp;
	double crdrLength = 0.0;
	TObjectPoolIdx dorIdx;
	cvTDynObjRef tmpDor = {0};
	tmpDor.objId = objId;

	pIntrsctn = BindIntrsctn(isecId);

#ifdef DEBUG_DYN_OBJ_LIST
	if( debugThisFrame )
	{
		gout << spc << "distChecked = " << distChecked << endl;
	}
#endif

	// If the object is on the intersection
	bool objOnIntrsctn = IsObjOnIntrsctn(
								spc,
								pIntrsctn, 
								cCenter,
								xSize, 
								ySize, 
								zSize, 
								cQuad, 
								tmpDor, 
								&overlapRoads
								);
	if( objOnIntrsctn ) 
	{
		numPlaces++;

		//
		// Check to see if the DOR list already contains
		// this intersection with this object.
		//
		//gout << "$$$ trying to insert obj " << objId << " on isec " << isecId << endl;
		int i;
		bool foundIsec = false;
		for( i = 0; i < cCV_NUM_DOR_REPS; i++ )
		{
			int dynObjRefPoolIdx = (
						( i * cNUM_DYN_OBJS ) + 
						( objId + 1 )
						);
			bool isecMatch = (
					m_pDynObjRefPool[dynObjRefPoolIdx].intrsctnId == isecId
					);
			if( isecMatch )
			{
				foundIsec = true;
				break;
			}
		}
		//gout << "    foundIsec = " << foundIsec << endl;

		onIntrsctn = true;

		if( !foundIsec )
		{
			tmpDor.objId = objId;
			tmpDor.terrain = eTERR_ISEC;
			tmpDor.intrsctnId = isecId;
			dorIdx = FindDorIdx( objId );

			// If the dorIdx is valid
			if( dorIdx > 0 ) 
			{ 
				m_pDynObjRefPool[dorIdx] = tmpDor;
				LinkDorIntoList( dorIdx );
			}
		}
	} // If the object is on the intersection

	//
	// Check all roads that can be reached through isecId from roadId.
	//
	pOfs = reinterpret_cast<char*>(m_pHdr) + m_pHdr->crdrOfs;
	pCrdr = reinterpret_cast<cvTCrdr *>(pOfs) + pIntrsctn->crdrIdx;

	// For each corridor in the intersection
	for( crdr = 0; crdr < pIntrsctn->numOfCrdrs; ++crdr, ++pCrdr )
	{
		roadId = pCrdr->dstRdIdx;

		//
		// If this road hasn't been searched yet and the object 
		// hasn't already been found on this road.
		//
		if( ( find( searchedRoads.begin(), searchedRoads.end(), roadId ) == searchedRoads.end() ) &&
			( find( foundRoads.begin(), foundRoads.end(), roadId) == foundRoads.end() ) )
		{
			searchedRoads.push_back( roadId );

			//
			// If the destination road of this corridor should be checked
			// If the object was found in the intersection and the road is 
			// in the set of overlapping roads, then search the road.  Or, 
			// search if the object was not found in the intersection and 
			// the roadId is valid.
			//
			if( ( onIntrsctn && ( find( overlapRoads.begin(), overlapRoads.end(), roadId ) != overlapRoads.end() ) ) || ( ( !onIntrsctn ) && ( roadId != 0 ) ) )
			{
				tmpIsecId = 0;

				// Calculate the length of the corridor+distance checked so far
				pOfs = reinterpret_cast<char*>(m_pHdr)+m_pHdr->crdrCntrlPntOfs;
				pCrdrCp = (reinterpret_cast<TCrdrCntrlPnt*>(pOfs)) 
						 + pCrdr->cntrlPntIdx + pCrdr->numCntrlPnt - 2;

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "distChecked updated here" << endl;
					gout << spc << "partOne = " << distChecked << endl;
					gout << spc << " partTwo = " << (pCrdrCp+1)->distance << endl;
					gout << spc << "partThree = " << pCrdrCp->distance << endl;
				}
#endif


				crdrLength = distChecked + 
							 fabs( (pCrdrCp+1)->distance - pCrdrCp->distance );

				//
				// If the road begins at the intersection, then	check 
				// forward along roads and intersections
				//
				pRoad = BindRoad( roadId );

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "roadId before condition check = " << roadId << endl;
					gout << spc << "isecId before condition check = " << isecId << endl;
					gout << spc << "left part = " << pRoad->srcIntrsctnIdx << endl;
				}
#endif

				if( pRoad->srcIntrsctnIdx == isecId )
				{
					roadCntrlPntIdx = pRoad->cntrlPntIdx;

#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "suspected SearchForward" << endl;
						gout << spc << "distChecked, crdrLength = " << crdrLength << endl;
						gout << spc << "crdr id = " << crdr << endl;
					}
#endif
#if 0 // removed since there was infinite loop and this code didn't make any sense
					if( crdrLength  > 0 )
					{
						crdrLength = crdrLength * ( -1.0 );
					}
#endif
					numPlaces += SearchForward(
										spc,
										objId, 
										cCenter, 
										xSize, 
										ySize, 
										zSize, 
										cQuad, 
										crdrLength, 
										roadId, 
										tmpIsecId, 
										roadCntrlPntIdx, 
										foundRoads
										);
				}
				// If the road ends at the intersection, then
				//	check backward along roads and intersections
				else 
				{
					roadCntrlPntIdx = pRoad->cntrlPntIdx + pRoad->numCntrlPnt - 1;

#ifdef DEBUG_DYN_OBJ_LIST
					if( debugThisFrame )
					{
						gout << spc << "suspected SearchBackward" << endl;
						gout << spc << "distChecked, crdrLength = " << crdrLength << endl;
						gout << spc << "crdr id = " << crdr << endl;
					}
#endif
#if 0 // removed since there was infinite loop and this code didn't make any sense
					if( crdrLength  > 0 )
					{
						crdrLength = crdrLength * ( -1.0 );
					}
#endif
					numPlaces += SearchBackward(
										spc,
										objId, 
										cCenter, 
										xSize, 
										ySize, 
										zSize, 
										cQuad, 
										crdrLength, 
										roadId, 
										tmpIsecId,
										roadCntrlPntIdx, 
										foundRoads
										);
				}

			} // If the destination road of this corridor should be checked

		} // If this road hasn't been searched yet

	} // For each corridor in the intersection

	// ????????????
	distChecked = DIST_TO_CHECK + 1.0;
	// Return the number of places where the object was found

	return numPlaces;

} // end of SearchIsec


//////////////////////////////////////////////////////////////////////////////
//
// Description: Determine if the object overlaps a segment on the road 
//  piece parameter.  If it does, fill in the values for the corresponding 
//  dynamic object reference node and return true.
//
// Remarks: This function iterates through each line segment contained in the 
// 	road piece parameter.  To avoid having an object linked to more than one 
// 	line segment, the first one that the object is found to overlap is used.  
// 	The function determines "overlap" by examining the range of offsets from 
// 	the line segment occupied by the object's cQuadrilateral.  If the object's 
// 	offset range overlaps the road's offset range of [-roadWidth, roadWidth], 
// 	then the object is on the road.  A similar method is used to determine 
// 	which lanes the object overlaps.
//
// Paramaters:	
//	input:
//		pRdPc - Pointer to the road piece data structure to check.
//		cCenter - Object's cCenter point.
//		xSize, ySize, zSize - Size of object.
//		cQuad - Array of 4 vertices defining the object.
//	output (calculated by IsObjOnRoadSegment):
//		distance - Linear distance from the beginning of the road to 
//                      the object.
//		lanes - Mask that indicates which lane(s) the object overlaps.
//		direction - Direction on road where object seems to be headed.
//		roadCntrlPntIdx - Index of the first control point on the segment 
//                      the object overlaps.
//
// Returns: bool -	true if the object is on the road piece, 
//					false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsObjOnRoadPiece(
			const string& s,
			const cvTRoadPiece* cpRdPc,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			double zSize,
			const CPoint3D cQuad[],
			cvTDynObjRef& tmpDor
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside IsObjOnRoadPiece" << endl;
		spc += "  ";
	}
#endif

	cvTRoad* pRoad = BindRoad( cpRdPc->roadId );
	cvTCntrlPnt* pCurCp;
	cvTCntrlPnt* pNextCp;
	TLongCntrlPntPoolIdx roadCntrlPntIdx;
	bool prevSeg = false;
	bool nextSeg = false;

	double objTop = cCenter.m_z + ( 0.5 * zSize );
	double objBot = cCenter.m_z - ( 0.5 * zSize );
	double zHeight = 0; //height of current road point;
	

	//
	// Get the index of first and last control points associated with 
	// the road.
	//
	TLongCntrlPntPoolIdx firstCp = pRoad->cntrlPntIdx + cpRdPc->first;
	TLongCntrlPntPoolIdx lastCp  = pRoad->cntrlPntIdx + cpRdPc->last;

	//
	// Iterate for each road segment in the road piece.
	//
	for( 
		roadCntrlPntIdx = firstCp, pCurCp = BindCntrlPnt( roadCntrlPntIdx ); 
		roadCntrlPntIdx < lastCp; 
		++roadCntrlPntIdx, ++pCurCp
		)
	{
		pNextCp = pCurCp + 1;

		//
		// If the object is below the control point, or if it is above 
		// the control point more than cQRY_TERRAIN_SNAP units, then 
		// the object is not on the road.
		//
		double z = pCurCp->location.z;
		int id = pRoad->myId;
		if (pCurCp->location.z == pNextCp->location.z){
			zHeight = pCurCp->location.z;
		}else{
			double dist,zDist,xDist,yDist,interDist; //distance to the control point
			zDist = pCurCp->location.z - cCenter.m_z;
			xDist = pCurCp->location.x - cCenter.m_x;
			yDist = pCurCp->location.y - cCenter.m_y;
			//8.7.09 fix to add square root function addressing miscalculation with large numbers
			//dist = zDist*zDist + yDist*yDist + xDist*xDist; 
			dist = sqrt(zDist*zDist + yDist*yDist + xDist*xDist);
			interDist = pNextCp->distance - pCurCp->distance;
			//8.7.09 fix to use square root function 'dist' value to determine interpolation percentage
			//interDist = abs(dist/(interDist * interDist)); //% to interpolate
			interDist = abs(dist/interDist);
			if (interDist > 1.1) //our current point is to far from the cp to be considered.
				zHeight = pCurCp->location.z;
			else{
				zHeight = pCurCp->location.z + (pNextCp->location.z - pCurCp->location.z) * interDist;
			}
		}
		bool objNotOnRoad = (
					( objTop < zHeight ) || 
					( objBot > zHeight + cQRY_TERRAIN_SNAP )
					);
		if( objNotOnRoad )  continue;

		bool objOnRoadSeg = IsObjOnRoadSegment(
									spc,
									pCurCp, 
									pNextCp, 
									cCenter, 
									xSize, 
									ySize, 
									cQuad,
									tmpDor, 
									prevSeg, 
									nextSeg
									);
		if( objOnRoadSeg )
		{

			// A  road segment was found that the object overlaps, 
			//	so return.  Note that the data structure does not permit
			//	an object to overlap >1 segment, so we take the first one.
			bool negativeDirAndFirstCp = (
					tmpDor.direction == eNEG && roadCntrlPntIdx == firstCp
					);
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "tmpDor.direction = " << tmpDor.direction << endl;
				gout << spc << "roadCntrlPntIdx = " << roadCntrlPntIdx << endl;
				gout << spc << "firstCp = " << firstCp << endl;
				gout << spc << "negativeDirAndFirstCp = ";
				gout << negativeDirAndFirstCp << endl;
			}
#endif

			if( negativeDirAndFirstCp )
			{
				//
				// Objects that are on a negative lane and are found on the
				// first control point should really be reported as being
				// found on the next control point.  This will enable 
				// SearchAroundPrevLocation to properly and quickly
				// locate them.
				//
				pNextCp = pCurCp + 1;
#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "tmpDor.distance = " << tmpDor.distance << endl;
					gout << spc << "nextCp cumLinDist = " << pNextCp->cummulativeLinDist << endl;
				}
#endif
				tmpDor.roadCntrlPnt = roadCntrlPntIdx + 1;


#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "tmpDor.roadCntrlPnt = " << tmpDor.roadCntrlPnt << endl;
					gout << spc << "tmpDor.distance = " << tmpDor.distance << endl;
				}
#endif
			}
			else
			{
				//
				// Found the object on a positive direction lane.
				//
				tmpDor.roadCntrlPnt = roadCntrlPntIdx;
			}

			return true;
		}

	} // For each road segment

	//
	// If execution made it here, then the object does not overlap
	// any road segments within the road piece.  Return false.
	//
	return false;

} // end of IsObjOnRoadPiece


//////////////////////////////////////////////////////////////////////////////
//
// Description: Determine if the object overlaps the linear road segment 
//  define by the control point parameters.  Returns the values needed 
//  to set the dynamic object reference parameters, if the object is found 
//  to be on this road segment.
//
// Remarks: This function examines the offsets between the object and the  
// 	line segment.  The range of offsets spanned by the object's boundaries is 
// 	compared with the width of the road, to determine if the object overlaps 
// 	the road.
//
// Arguments:
//	input:	
//		cpCurCp - Pointer to the first control point defining the segment.
//		cpNextCp - Pointer to the next control point defining the segment.
//		cpRoad - Pointer to the road structure the segment is on.
//		cCenter - Object's cCenter point.
//		xSize, ySize - Sze of object.
//		cQuad - Array of 4 vertices defining the object.
//	output:
//		tmpDor - Fills the dyn obj ref with the distance, lanes, and 
//			direction.
//		intrsctnId - If the object hangs off the road segment, this parameter 
//			is the index of the connecting intersection that the object may 
//			overlap. 
//
// Returns: bool -	true if the object is on the road segment, 
//					false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsObjOnRoadSegment(
			const string& s,
			const cvTCntrlPnt* cpCurCp,
			const cvTCntrlPnt* cpNextCp,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			const CPoint3D cQuad[],
			cvTDynObjRef& tmpDor, 
			bool& prevSeg,
			bool& nextSeg
			) 
{
#define	SIGN(x)	(((x) < 0)? -1 : 1)
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside IsObjOnRoadSegment" << endl;
		spc += "  ";
	}
#endif

	double minOffset2;
	double maxOffset2;
	double segmentDist;

	int laneItr;
	cvTLane* pLane;
	double upper;
	double lower;
	bitset<cCV_MAX_LANES> laneMask;
	int posCnt;
	int negCnt;
	char* pOfs;

	// 
	// Set up the 2 corridor control point structures.
	//
	TDorCntrlPnt cp1;
	cp1.location = cpCurCp->location;
	cp1.rightVec = cpCurCp->rightVecLinear;
	cp1.width = cpCurCp->physicalWidth;
	cp1.distance = cpCurCp->distToNextLinear;

	TDorCntrlPnt cp2;
	cp2.location = cpNextCp->location;
	cp2.rightVec = cpNextCp->rightVecLinear;
	cp2.width = cpNextCp->physicalWidth;
	cp2.distance = cpNextCp->distToNextLinear;

	bool objOverlapsSegOnRoad = ObjectOverlapsSegmentOnRoad(
										spc,
										&cp1, 
										&cp2,
										cCenter, 
										xSize, 
										ySize, 
										cQuad,  
										segmentDist, 
										minOffset2, 
										maxOffset2, 
										prevSeg, 
										nextSeg
										);
	if( objOverlapsSegOnRoad )
	{
		//
		// Find the lanes that overlap [minOffset2, maxOffset2].
		//
		pOfs = reinterpret_cast<char*>(m_pHdr) + m_pHdr->laneOfs;
		pLane = reinterpret_cast<cvTLane *>(pOfs) + cpCurCp->laneIdx;

		laneMask.reset();
		posCnt = negCnt = 0;
		for( laneItr = 0; laneItr < cpCurCp->numLanes; ++laneItr, ++pLane )
		{	
			//
			// Find squares of the upper and lower bounds of the lanes.
			// If the real boundary is negative, then the square is
			// multiplied by -1, to match the [minOffset2, maxOffset2] 
			// ranges.
			//
			upper = pLane->offset + ( pLane->width * 0.5 );
			upper *= ( upper * SIGN(upper) );
			lower = pLane->offset - ( pLane->width * 0.5 );
			lower *= ( lower * SIGN(lower) );

			if( maxOffset2 > lower && minOffset2 < upper )
			{	
				laneMask.set( laneItr );

				if( pLane->direction == ePOS )      ++posCnt;
				else if( pLane->direction == eNEG ) ++negCnt;
			}
		}
		tmpDor.lanes = laneMask.to_ulong();

		if( posCnt > negCnt )       tmpDor.direction = ePOS;
		else if( negCnt > posCnt )  tmpDor.direction = eNEG;
		else                        tmpDor.direction = eNONE;

		// Find the linear distance between the object cCenter  
		//	projection and the beginning of the segment
		tmpDor.distance = cpCurCp->cummulativeLinDist + segmentDist;

#ifdef DEBUG_DYN_OBJ_LIST
		if( debugThisFrame )
		{
			gout << spc << "tmpDor.direction = " << tmpDor.direction << endl;
			gout << spc << "tmpDor.distance = " << tmpDor.distance << endl;
			gout << spc << "partA = " << cpCurCp->cummulativeLinDist << endl;
			gout << spc << "partB = " << segmentDist << endl;
		}
#endif

		// EXIT: The object overlaps this road segment. Return true.
		return true;
	}

	// If control makes it here, then the object is not on the road segment.
	return false;

} // end of IsObjOnRoadSegment

//////////////////////////////////////////////////////////////////////////////
//
// Description: Determine whether the object was found in the intersection.
//				
// Remarks: Each corridor of the intersection is checked to see if the 
//  object's box geometrically overlaps it.  The corridors the object 
//  overlaps are set into the corridor mask in the temporary DOR.
//
// Arguments:	
//	Input:
//		pIntrsctn - Pointer to the intersection structure to check.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize, zSize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Output:
//		tmpDor - Contains the resulting corridors the object overlaps.
//		roadIds - If the object hangs off a corridor, this array contains
//			the indeces of the connecting road that the object may overlap. 
// Returns: bool -	true if the object overlaps more than one corridor, 
//					false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsObjOnIntrsctn(
			const string& s,
			const cvTIntrsctn* cpIntrsctn, 
			const CPoint3D& cCenter, 
			const double xSize, 
			const double ySize, 
			const double zSize,
			const CPoint3D cQuad[4],
			cvTDynObjRef& tmpDor, 
			vector<TRoadPoolIdx>* pRoadSet
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside IsObjOnIntrsctn" << endl;
		spc += "  ";
		gout << spc << "input param: tmpDor.distance = ";
		gout << tmpDor.distance << endl;
	}
#endif

	//
	// If the object is below the intersection, or if it is above the 
	// intersection more than cQRY_TERRAIN_SNAP units, then the object 
	// is not on the intersection.
	//
	double objTop = cCenter.m_z + ( 0.5 * zSize );
	double objBot = cCenter.m_z - ( 0.5 * zSize );
	bool objNotOnIntrsctn = ( 
                cpIntrsctn->elevMap == 0 &&
                (( objTop < cpIntrsctn->elevation ) || 
                 ( objBot > cpIntrsctn->elevation + cQRY_TERRAIN_SNAP ))
				);
	if( objNotOnIntrsctn )  
        return false;
    //if we have an intersection map, we need to check the intersection map
    //for the height cpIntrsctn->elevation, is not going to be valid
    if (cpIntrsctn->elevMap != 0){
        if (m_intrsctnGrids.size() <= cpIntrsctn->myId)
            return false;
		const CTerrainGrid<Post>* pGrid = 
            m_intrsctnGrids[cpIntrsctn->myId];
		if (pGrid) {
			Post gridOut;
			if (pGrid->QueryElev(cCenter, gridOut) ){
                objNotOnIntrsctn = ( 
           				( objTop < gridOut.z ) ||
                        ( objBot > gridOut.z + cQRY_TERRAIN_SNAP )
                        );
                if( objNotOnIntrsctn )  
                    return false;
            }else{
                return false;
            }
		}else{
          return false;  
        }
    }
	//
	// Test to see if object's bounding box overlaps the intersection's
	// bounding box.  If so, the object is on the intersection.
	//
	CBoundingBox intrsctnBBox;
	CBoundingBox objBBox;
	char* pBorders = reinterpret_cast<char*>(m_pHdr) + m_pHdr->borderOfs;
	TBorderSeg* pSegPool = reinterpret_cast<cvTBorderSeg*>( pBorders );
	TBorderSegPoolIdx vertex;
	TBorderSeg* pV;
	for( vertex = 0; vertex < cpIntrsctn->numBorderSeg; vertex++ )
	{
		pV = pSegPool + ( cpIntrsctn->borderSegIdx + vertex );
		CPoint2D point( pV->x, pV->y );
		intrsctnBBox += point;
	}

	for( vertex = 0; vertex < 4; vertex ++ )
	{
		objBBox += cQuad[vertex];
	}

	if( intrsctnBBox.Intersects( objBBox ) )
	{
		//
		// Object lies on this intersection.
		//
		char* pOfs = reinterpret_cast<char*>(m_pHdr) + m_pHdr->crdrOfs;
	
		unsigned int crdrId;
		bitset<cCV_MAX_CRDRS> corridors;
		TRoadPoolIdx tmpRoadId;
		
		vector<cvTCrdrInfo> intrsctingCrdrs;

		//
		// Check if object intersects grids.
		//
		int numIntrsctingGrids = 0;
	
		for ( unsigned int gdCount = 0; gdCount < cCV_MAX_GRIDS; gdCount++ )
		{
			unsigned int gridId;
			double minX, minY, maxX, maxY;

			gridId = cpIntrsctn->grids[gdCount].gridId;
			minX = cpIntrsctn->grids[gdCount].minX;
			minY = cpIntrsctn->grids[gdCount].minY;
			maxX = cpIntrsctn->grids[gdCount].maxX;
			maxY = cpIntrsctn->grids[gdCount].maxY;

			CBoundingBox gridBBox( minX, minY, maxX, maxY );

			cvTCrdr* pCrdr = reinterpret_cast<cvTCrdr *>(pOfs) + cpIntrsctn->crdrIdx;

			if( gridBBox.Intersects( objBBox ) )
			{
				numIntrsctingGrids++;

				//
				// Iterate for each corridor in the intersection.
				//
				for( crdrId = 0; crdrId < cpIntrsctn->numOfCrdrs; crdrId++, pCrdr++	)
				{
					int curCrdrId = pCrdr->myId;

					int intrsctingCrdrId;
					intrsctingCrdrId = cpIntrsctn->grids[gdCount].intrsctingCrdrs[crdrId].crdrId;
						
					if( intrsctingCrdrId == curCrdrId )
					{
						int startIdx = cpIntrsctn->grids[gdCount].intrsctingCrdrs[crdrId].startCntrlPtIdx;
						int endIdx = cpIntrsctn->grids[gdCount].intrsctingCrdrs[crdrId].endCntrlPtIdx;
						
						bool objOnCrdr = IsObjOnCorridor(
														spc,
														pCrdr, 
														startIdx,
														endIdx,
														cCenter, 
														xSize, 
														ySize, 
														cQuad, 
														tmpDor, 
														tmpRoadId
														);
						if( objOnCrdr )
						{
							if( tmpRoadId != 0 && pRoadSet != 0 )
							{
								if( find( pRoadSet->begin(), pRoadSet->end(), tmpRoadId ) != pRoadSet->end() )
											pRoadSet->push_back( tmpRoadId );
							}

#ifdef DEBUG_DYN_OBJ_LIST
							if( debugThisFrame )
							{
								gout << spc << "End: tmpDor.distance = " << tmpDor.distance << endl;
							}
#endif
								
							tmpDor.crdrDistances[crdrId] = tmpDor.distance;


#ifdef DEBUG_DYN_OBJ_LIST
							if( debugThisFrame )
							{
								gout << spc << "End: tmpDor.crdrDistances[crdrId] = ";
								gout << tmpDor.crdrDistances[crdrId] << endl;
							}
#endif

							corridors.set( crdrId );
						}

					}	// crdr id matches

				} // For each corridor

			} // obj on grid
		} // for each grid

		tmpDor.corridors = corridors.to_ulong();
		return (tmpDor.corridors != 0);

	}  // if object lies on intersection

	return false;

} // end of IsObjOnIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: Determine whether the object lies on the corridor.
//				
// Remarks:  Each segment of the corridor is checked to see if the 
//  object's box geometrically overlaps it.  Note that the data 
//  structure does not permit an object to overlap > 1 segment, so 
//  we take the first one.
//
// Arguments:	
//	Input:
//		pCrdr - Pointer to the corridor structure to check.
//		startCntrlPntIdx - The start control point idx the obj may lie on.
//		endCntrlPntIdx - The end control point idx the obj may lie on.
//		cCenter - Current cCenter point of the object.
//		xSize, ySize - Current sizes of the object.
//		cQuad - Current bounding cQuadrilateral of the object.
//	Output:
//		tmpDor - Contains the distance along the segment where the object 
//			cCenter projects to.
//		roadId - If the object hangs off the corridor, this parameter is
//			the index of the connecting road that the object may overlap. 
// Returns: bool -	true if the object overlaps the segment,
//					false otherwise
//
//////////////////////////////////////////////////////////////////////////////
bool
CCved::IsObjOnCorridor(
			const string& s,
			const cvTCrdr* cpCrdr,
			TCrdrCntrlPntPoolIdx   startCntrlPntIdx,
			TCrdrCntrlPntPoolIdx   endCntrlPntIdx,
			const CPoint3D& cCenter, 
			double xSize, 
			double ySize, 
			const CPoint3D cQuad[4],
			cvTDynObjRef& tmpDor, 
			TRoadPoolIdx& roadId
			) 
{
	string spc = s;

#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside IsObjOnCorridor   crdrId = ";
		gout << cpCrdr->myId << endl;
		spc += "  ";
	}
#endif

	char* pOfs;
	cvTCrdrCntrlPnt* pCurCp;
	cvTCrdrCntrlPnt* pNextCp;
	TDorCntrlPnt cp1;
	TDorCntrlPnt cp2;
	TCrdrCntrlPntPoolIdx crdrCntrlPnt;
	double minOffset2;
	double maxOffset2;
	bool prevSeg = false;
	bool nextSeg = false;

	roadId = 0;

	// For each line segment in the corridor curve
	pOfs = reinterpret_cast<char*>(m_pHdr) + m_pHdr->crdrCntrlPntOfs;
	pCurCp = (reinterpret_cast<TCrdrCntrlPnt*>(pOfs)) + cpCrdr->cntrlPntIdx;
	for( 
		crdrCntrlPnt = 1; 
		crdrCntrlPnt < cpCrdr->numCntrlPnt; 
		++crdrCntrlPnt, ++pCurCp
		)
	{
		if ( crdrCntrlPnt < startCntrlPntIdx	)	continue;

		if( crdrCntrlPnt + 1 > endCntrlPntIdx )		break;

		pNextCp = pCurCp + 1;
		cp1.location.x = pCurCp->location.x;
		cp1.location.y = pCurCp->location.y;
		cp1.location.z = 0.0;
		cp1.rightVec.i = pCurCp->rightVecLinear.i;
		cp1.rightVec.j = pCurCp->rightVecLinear.j;
		cp1.rightVec.k = 0.0;
		cp1.width = pCurCp->width;
		//cp1.distance = pNextCp->distance;
		cp1.distance = pCurCp->distance;

		cp2.location.x = pNextCp->location.x;
		cp2.location.y = pNextCp->location.y;
		cp2.location.z = 0.0;
		cp2.rightVec.i = pNextCp->rightVecLinear.i;
		cp2.rightVec.j = pNextCp->rightVecLinear.j;
		cp2.rightVec.k = 0.0;
		cp2.width = pNextCp->width;
//		cp2.distance = 0.0;
		cp2.distance = pNextCp->distance;

		bool objOverlapsSegOnCrdr = ObjectOverlapsSegmentOnCrdr(
											spc,
											&cp1, 
											&cp2, 
											cCenter, 
											xSize, 
											ySize, 
											cQuad, 
											tmpDor.distance, 
											minOffset2, 
											maxOffset2, 
											prevSeg, 
											nextSeg
											);
		if( objOverlapsSegOnCrdr ) 
		{
			//
			// If the object is found to overlap the next or previous segment, 
			// and the segment is the last or first in the corridor, then
			// set roadId to the destination road or the source road, 
			// respectively.
			//
			if( prevSeg && crdrCntrlPnt == 1 )
			{
				roadId = cpCrdr->srcRdIdx;
			}
			else if( nextSeg && crdrCntrlPnt == cpCrdr->numCntrlPnt - 1 ) 
			{
				roadId = cpCrdr->dstRdIdx;
			}

#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "obj overlaps crdr....roadId = ";
				gout << roadId << endl;
			}
#endif
			// EXIT: A corridor segment was found that the object overlaps, 
			//	so return.  Note that the data structure does not permit
			//	an object to overlap >1 segment, so we take the first one.
			return true;
		}

	} // For each road segment

	// If execution made it here, then the object does not overlap
	//	any road segments within the road piece.  Return false.
	return false;

} // end of IsObjOnCorridor


//////////////////////////////////////////////////////////////////////////////
//
// Description: Determine whether the object's box overlaps the segment 
//  box determined by the line between the two control points, the width 
//  of the segment at the first control point, and the right vector of 
//  the first control point.  
//
// Remarks:
//	This function uses the range of offsets from the line segment that the 
//	object occupies to determine if it overlaps the width of the segment. It
//	also uses the range of distances along the line segment that the object
//	occupies to determine if it overlaps the length of the segment.  
//
// Arguments:
//	input:	
//		cpCurCp - Pointer to the first control point defining the segment.
//		cpNextCp - Pointer to the next control point defining the segment.
//		cCenter - Object's cCenter point.
//		xSize, ySize - Size of object.
//		cQuad - Array of 4 vertices defining the object.
//	output:
//		distance - Distance along the corridor.
//		minOffset2 - Minimum offset squared of the object from the line seg.
//		maxOffset2 - Maximum offset squared of the object from the line seg.
//		checkPrevSeg - Indicates whether the object's box hangs off the front 
//			of the segment.  The object may overlap with the previous seg.
//		checkNextSeg - Indicates whether the object's box hangs off the back 
//			of the segment.  The object may overlap with the next segment.
//
// Returns: bool -	true if the object overlaps the segment, 
//					false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CCved::ObjectOverlapsSegmentOnCrdr(
			const string& s,
			const TDorCntrlPnt* cpCurCp,
			const TDorCntrlPnt* cpNextCp,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			const CPoint3D cQuad[],
			double& distance, 
			double& minOffset2, 
			double& maxOffset2, 
			bool& checkPrevSeg, 
			bool& checkNextSeg
			) 
{
	
#ifdef DEBUG_DYN_OBJ_LIST
	bool debugThisCrdr = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	string spc = s;
	if( debugThisCrdr )
	{
		gout << spc << "**inside ObjectOverlapsSegmentOnCrdr" << endl;
		spc += "  ";
		gout << spc << "input param: cpCurCp->distance = ";
		gout << cpCurCp->distance << endl;
	}
#endif


	double curOffset2;
	CPoint2D proj;
	int vertItr;

	double minT, maxT, curT;

	// Initialize check*Seg to false
	checkPrevSeg = false;
	checkNextSeg = false;

#ifdef DEBUG_DYN_OBJ_LIST
	if( debugThisCrdr )
	{
		gout << spc << "control p1:  x = " << cpCurCp->location.x;
		gout << "  y = " << cpCurCp->location.y;
		gout << "  d = " << cpCurCp->distance << endl;
		gout << spc << "control p1:  x = " << cpNextCp->location.x;
		gout << "  y = " << cpNextCp->location.y;
		gout << "  d = " << cpNextCp->distance << endl;
	}
#endif

	// Create a line segment between the 2 control points
	CLineSeg2D lineSeg (
				cpCurCp->location.x, 
				cpCurCp->location.y, 
				cpNextCp->location.x, 
				cpNextCp->location.y
				);

	// Check the distance to the line segment.  If it is greater than 
	//	the width of the road + 1/2*diagonal length of object, then
	//	the object cannot be on the road.
	double tCenter;
	double offsetCenter = lineSeg.PerpDistSquare(
				cCenter.m_x, 
				cCenter.m_y, 
				&tCenter
				);

#ifdef DEBUG_DYN_OBJ_LIST
//	if( debugThisCrdr )
//	{
//		gout << "                offsetCenter = " << offsetCenter << endl;
//		gout << "                cCenter.m_x = " << cCenter.m_x << endl;
//		gout << "                cCenter.m_y = " << cCenter.m_y << endl;
//		gout << "                tCenter = " << tCenter << endl;
//	}
#endif

	double maxObjOffset = (
				( 0.25 * cpCurCp->width * cpCurCp->width ) + 
				( 0.25 * ( xSize*xSize + ySize*ySize ) )
				);

	bool roadOverlaps = offsetCenter < maxObjOffset;
	if( roadOverlaps )
	{
#ifdef DEBUG_DYN_OBJ_LIST
//		if( debugThisCrdr )  gout << "                road overlaps " << endl;
#endif
		// Get the range of offsets and t values from  
		//	the segment that the cQuadrangle covers. 
		minOffset2 = FLT_MAX; 
		maxOffset2 = -FLT_MAX;
		minT = DBL_MAX; 
		maxT = -DBL_MAX;

		// For each vertex in the object box
		for( vertItr = 0; vertItr < 4; ++vertItr )
		{
			// Iterate through each vertex in cQuad to 
			
			// Determine the range of offsets from the line segment
			//	the object spans. 

			curOffset2 = lineSeg.PerpDistSquare(cQuad[vertItr], &curT);

			// Set curOffset2 to -curOffset2 if the vertex is in
			//	the same direction as the rightVector.
			//	This allows for the possibility that cQuad
			//	straddles lineSeg and maintains the concept 
			//	of a minOffset2 and maxOffset2.
			CVector3D rightVector(cpCurCp->rightVec);
			CVector3D vertVector(
							cQuad[vertItr].m_x - cpCurCp->location.x, 
							cQuad[vertItr].m_y - cpCurCp->location.y,
							cQuad[vertItr].m_z - cpCurCp->location.z
							);
			if( rightVector.DotP( vertVector ) < 0 ) curOffset2 = -curOffset2;
			
			if( curOffset2 < minOffset2 )  minOffset2 = curOffset2;
			if( curOffset2 > maxOffset2 )  maxOffset2 = curOffset2;

			// Set minT or maxT if needed
			if( curT < minT )  minT = curT;
			if( curT > maxT )  maxT = curT;

		} // For each vertex

		bool lengthOverlaps = maxT >= 0.0 && minT <= 1.0;
		if( lengthOverlaps ) 
		{
#ifdef DEBUG_DYN_OBJ_LIST
//			if( debugThisCrdr )  gout << "                length overlaps" << endl;
#endif

			// If the object overlaps the road or shoulder
			bool widthOverlaps = abs(minOffset2) < cpCurCp->width * cpCurCp->width;
			if( widthOverlaps )
			{
#ifdef DEBUG_DYN_OBJ_LIST
//				if( debugThisCrdr )  gout << "                width overlaps" << endl;
#endif

				// Set check*Seg if needed
				if( minT < 0.0 )  checkPrevSeg = true;
				if( maxT > 1.0 )  checkNextSeg = true;

				// Find the linear distance between the object cCenter  
				//	projection and the beginning of the segment
				//distance = tCenter * cpCurCp->distance;
				distance = (
					cpCurCp->distance + 
					( ( cpNextCp->distance - cpCurCp->distance ) * tCenter )
					);

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisCrdr )
				{
					gout << spc << "output dist = " << distance << endl;
					gout << spc << "found obj on crdr" << endl;
				}
#endif

				// EXIT: The object overlaps this road segment. Return true.
				return true;

			} // If the object's width overlaps the line segment's width

		} // If object's length overlaps line segment's length

	} // If the object can overlap the road

	// If control makes it here, then the object is not on the road segment.
	return false;

} // end of ObjectOverlapsSegmentOnCrdr

/////////////////////////////////////////////////////////////////////////////
//
// Description: Determine whether the object's box overlaps the segment 
//  box determined by the line between the two control points, the width 
//  of the segment at the first control point, and the right vector of 
//  the first control point.  
//
// Remarks:
//  This function uses the range of offsets from the line segment that the 
//  object occupies to determine if it overlaps the width of the segment.  It
//  also uses the range of distances along the line segment that the object
//  occupies to determine if it overlaps the length of the segment.  
//
// Arguments:
//	input:	
//		cpCurCp - Pointer to the first control point defining the segment.
//		cpNextCp - Pointer to the next control point defining the segment.
//		cCenter - Object's cCenter point.
//		xSize, ySize - Size of object.
//		cQuad - Array of 4 vertices defining the object.
//	output:
//		distance - Distance along the road.
//		minOffset2 - Minimum offset squared of the object from the line seg.
//		maxOffset2 - Maximum offset squared of the object from the line seg.
//		checkPrevSeg - Indicates whether the object's box hangs off the front 
//			of the segment.  The object may overlap with the previous seg.
//		checkNextSeg - Indicates whether the object's box hangs off the back 
//			of the segment.  The object may overlap with the next segment.
//
// Returns: bool -	true if the object overlaps the segment, 
//					false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CCved::ObjectOverlapsSegmentOnRoad(
			const string& s,
			const TDorCntrlPnt* cpCurCp,
			const TDorCntrlPnt* cpNextCp,
			const CPoint3D& cCenter,
			double xSize,
			double ySize,
			const CPoint3D cQuad[],
			double& distance, 
			double& minOffset2, 
			double& maxOffset2, 
			bool& checkPrevSeg, 
			bool& checkNextSeg
			) 
{
	
#ifdef DEBUG_DYN_OBJ_LIST
	string spc = "  ";
	bool debugThisFrame = m_pHdr->frame >= DEBUG_DYN_OBJ_LIST;
	if( debugThisFrame )
	{
		gout << spc << "**inside ObjectOverlapsSegmentOnRoad" << endl;
		spc += "  ";
	}
#endif


	double curOffset2;
	CPoint2D proj;
	int vertItr;

	double minT;
	double maxT;
	double curT;

	// Initialize check*Seg to false
	checkPrevSeg = false;
	checkNextSeg = false;

#ifdef DEBUG_DYN_OBJ_LIST
	if( debugThisFrame )
	{
		gout << spc << "control p1: " << " x = " << cpCurCp->location.x;
		gout << " " << " y = " << cpCurCp->location.y << endl;
		gout << spc << "control p2: " << " x = " << cpNextCp->location.x;
		gout << " " << " y = " << cpNextCp->location.y << endl;
	}
#endif

	// Create a line segment between the 2 control points
	CLineSeg2D lineSeg (
					cpCurCp->location.x, cpCurCp->location.y, 
					cpNextCp->location.x, cpNextCp->location.y
					);

	//
	// Check the distance to the line segment.  If it is greater than 
	// the width of the road + 1/2*diagonal length of object, then
	// the object cannot be on the road.
	//
	double tCenter;
	double offsetCenter = lineSeg.PerpDistSquare(
											cCenter.m_x, 
											cCenter.m_y, 
											&tCenter
											);
	double maxObjOffset = (
				( 0.25 * cpCurCp->width * cpCurCp->width ) + 
				( 0.25 * ( (xSize * xSize) + (ySize * ySize) ) )
				);
	if( offsetCenter < maxObjOffset )
	{
#ifdef DEBUG_DYN_OBJ_LIST
		if( debugThisFrame )
		{
			gout << spc << "obj overlaps the road" << endl;
			gout << spc << "offsetCenter = " << offsetCenter << endl;
			gout << spc << "maxObjOffset = " << maxObjOffset << endl;
			gout << spc << "cpCurCp->width = " << cpCurCp->width << endl;
		}
#endif

		//
		// Get the range of offsets and t values from the segment that 
		// the cQuadrangle covers. 
		//
		minOffset2 = FLT_MAX; 
		maxOffset2 = -FLT_MAX;
		minT = DBL_MAX; 
		maxT = -DBL_MAX;

		//
		// Iterate for each vertex in the object box.
		//
		for( vertItr = 0; vertItr < 4; ++vertItr )
		{
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "vertex " << vertItr << " = ";
				gout << cQuad[vertItr] << endl;
			}
#endif

			// Determine the range of offsets from the line segment
			// the object spans. 

			curOffset2 = lineSeg.PerpDistSquare( cQuad[vertItr], &curT );

			//
			// Set curOffset2 to -curOffset2 if the vertex is in the 
			// same direction as the rightVector.  This allows for 
			// the possibility that cQuad straddles lineSeg and 
			// maintains the concept of a minOffset2 and maxOffset2.
			//
			CVector3D rightVector( cpCurCp->rightVec );
			CVector3D vertVector(
							cQuad[vertItr].m_x - cpCurCp->location.x, 
							cQuad[vertItr].m_y - cpCurCp->location.y,
							cQuad[vertItr].m_z - cpCurCp->location.z
							);
			if( rightVector.DotP(vertVector) < 0 )  curOffset2 = -curOffset2;
			
			if( curOffset2 < minOffset2 )  minOffset2 = curOffset2;
			if( curOffset2 > maxOffset2 )  maxOffset2 = curOffset2;

			// Set minT or maxT if needed
			if( curT < minT )  minT = curT;
			if( curT > maxT )  maxT = curT;

		} // For each vertex

		if( maxT >= 0.0 && minT <= 1.0 ) 
		{
#ifdef DEBUG_DYN_OBJ_LIST
			if( debugThisFrame )
			{
				gout << spc << "length overlaps " << endl;
				gout << spc << "maxT = " << maxT;
				gout << "   minT = " << minT << endl;
			}
#endif

			// If the object overlaps the road or shoulder
			if( abs(minOffset2) < cpCurCp->width * cpCurCp->width ) 
			{
#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "width overlaps " << endl;
					gout << spc << "minOffset2 = " << minOffset2;
					gout << "   cpCurCp->width = " << cpCurCp->width << endl;
				}
#endif

				// Set check*Seg if needed
				if( minT < 0.0 )  checkPrevSeg = true;
				if( maxT > 1.0 )  checkNextSeg = true;

				//
				// Find the linear distance between the object cCenter  
				// projection and the beginning of the segment
				//
				distance = tCenter * cpCurCp->distance;

#ifdef DEBUG_DYN_OBJ_LIST
				if( debugThisFrame )
				{
					gout << spc << "distance = " << distance << endl;
					gout << spc << "tCenter =  " << tCenter << endl;
					gout << spc << "cpCurCp->distance: " << cpCurCp->distance << endl;
					gout << spc << "checkPrevSeg = " << checkPrevSeg;
					gout << "   checkNextSeg = " << checkNextSeg << endl;
					gout << spc << "*******return true******" << endl;
				}
#endif

				// EXIT: The object overlaps this road segment. Return true.
				return true;

			} // If the object's width overlaps the line segment's width

		} // If object's length overlaps line segment's length

	} // If the object can overlap the road

	// If control makes it here, then the object is not on the road segment.
#ifdef DEBUG_DYN_OBJ_LIST
	if( debugThisFrame )
	{
		gout << spc << "*******return false*******" << endl;
	}
#endif

	return false;

} // end of ObjectOverlapsSegmentOnRoad


} // namespace CVED
