//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: intrsctn.cxx,v 1.43 2018/03/27 15:03:04 IOWA\dheitbri Exp $
//
// Author(s):	Li-Jen Tsao
// Date:		September, 1998
//
// Description:	The implementation of the CIntrsctn class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"		// all needed for the class
#include "cvedstrc.h"		// private CVED data structs

#include <algorithm>
using namespace std;

#undef DEBUG_QRY_ATTR
#undef DEBUG_QRY_ATTR2
#undef DEBUG_PRIORITIZE_CORRIDORS
// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Perform a deep copy of the parameter to the current instance.
//
// Remarks:
//
// Arguments:
//	cRhs - a reference to an object intended to be to the right of the =
//
// Returns: a reference to the current instance
// 
/////////////////////////////////////////////////////////////////////////////
CIntrsctn&
CIntrsctn::operator=(const CIntrsctn& cRhs)
{
	if ( this != &cRhs ) {
		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pIntrs = cRhs.m_pIntrs;
	}

	return *this;
} // end of operator=
//////////////////////////////////////////////////////////////////////////////
///\brief
/// 	checks to see if the IDs of the ints match
///\return true if match
/////////////////////////////////////////////////////////////////////////////
bool
CIntrsctn::operator==(const CIntrsctn& cRhs)
{
	return bool(m_pIntrs->myId == cRhs.m_pIntrs->myId);
} // end of operator=
//////////////////////////////////////////////////////////////////////////////
//
// Description: CIntrsctn
// 	Constructor that takes a CCved instance and an id.
//
// Remarks:
//
// Arguments:
//	cCved - const reference to a CCved instance
//	id   - intersection id
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn::CIntrsctn( const CCved& cCved, TU32b id ) 
	: CCvedItem( &cCved )
{
	cvTHeader* pH   = static_cast<cvTHeader*> ( GetInst() );
	char*      pOfs = static_cast<char*>      ( GetInst() ) + pH->intrsctnOfs;

	// 
	// Make sure the id is within bounds.
	//
	if( id >= pH->intrsctnCount )
	{
		stringstream ss;
		ss<<"No intersection with id = :"<<id;
		cvCInternalError e( ss.str(), __FILE__, __LINE__ );
		throw e;
	}

	m_pIntrs = reinterpret_cast<TIntrsctn*>(pOfs) + id;
} // end of CIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: CIntrsctn
// 	Constructor that takes a CCved instance and a name
//
// Remarks:
//
// Arguments:
//	cCved - const reference to a CCved instance
//	cName - intersection name
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CIntrsctn::CIntrsctn( const CCved& cCved, const string& cName ) 
	: CCvedItem( &cCved )
{
	cvTHeader* pH   = static_cast<cvTHeader*> ( GetInst() );
	char*      pOfs = static_cast<char*>      ( GetInst() ) + pH->intrsctnOfs;
	char*      pCharPool = static_cast<char*> ( GetInst() ) + pH->charOfs;

	unsigned int i;
	TIntrsctn* pIntrsctn;
	for( i = 1; i < pH->intrsctnCount; i++ )
	{
		pIntrsctn = ( reinterpret_cast<TIntrsctn *>( pOfs ) ) + i;
		if( cName == ( pCharPool + pIntrsctn->nameIdx ) )
		{
			m_pIntrs = pIntrsctn;
			break;
		}
	}

	//
	// Unable to find the intersection with the same name
	//
	if( i == pH->intrsctnCount )
	{
		string msg( " No intersection with name: " );
		msg += cName;
		cvCInternalError e( msg, __FILE__, __LINE__ );
		throw e;
	}
} // end of CIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetName
// 	Return the name of the current intersection
//
// Remarks: This function causes a failed assertion if it is run on an 
// 	invalid CIntrsctn instance.
//
// Arguments:
//
// Return value: A string holding the name of the intersection.
//
//////////////////////////////////////////////////////////////////////////////
string
CIntrsctn::GetName(void) const
{
	AssertValid();

	char*		pCharPool;

	cvTHeader*	pH	= static_cast<cvTHeader*>(GetInst());
	pCharPool		= static_cast<char*> (GetInst()) + pH->charOfs;
	string s(pCharPool + m_pIntrs->nameIdx);

	return s;
} // end of GetName

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetId
// 	Return the identifier of the current intersection
//
// Remarks: This function causes a failed assertion if it is run on an 
// 	invalid CIntrsctn instance.
//
// Arguments:
//
// Return value: An integer holding the id of the intersection.
//
//////////////////////////////////////////////////////////////////////////////
int
CIntrsctn::GetId(void) const
{
	AssertValid();
	return m_pIntrs->myId;
} // end of GetId

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRoads
// 	Collect all roads adjacent to the current intersection
//
// Remarks: This function causes a failed assertion if it is run on an 
// 	invalid CIntrsctn instance.
//
// Arguments:
// 	out - a vector that upon return holds all roads adjacent to the intrsctn.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CIntrsctn::GetRoads(TRoadVec& out) const
{ 
	AssertValid();

	out.clear();
	out.reserve(m_pIntrs->numOfRoads);

	for (int i = 0; i < m_pIntrs->numOfRoads; i++) {
		CRoad r( GetCved(), m_pIntrs->adjacentRoadId[i] );
		out.push_back(r);
	}
} // end of GetRoads

///////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is a two-road
//  intersection--i.e. it has one source road and one destination road.
// 
// Remarks: 
//
// Returns: true or false  
// 	
///////////////////////////////////////////////////////////////////////////////
bool
CIntrsctn::IsTwoRoad( void ) const
{
	return m_pIntrs->numOfRoads == 2;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: Collect all corridors contained in the current 
//   intersection
//
// Remarks: This function causes a failed assertion if it is run on an 
// 	 invalid CIntrsctn instance.
//
// Arguments:
// 	 out - (output) A vector that will hold all corridors in the intrsctn.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetAllCrdrs( TCrdrVec& out ) const
{
	AssertValid();

	//
	// Clearing the vector and reserving the maximum amount of space
	// needed in the vector for efficiency.
	//
	out.clear();
	out.reserve( m_pIntrs->numOfCrdrs );

	unsigned int i;
	for( i = 0; i < m_pIntrs->numOfCrdrs; i++ )
	{
		CCrdr c( GetCved(), m_pIntrs->crdrIdx + i );
		out.push_back( c );
	}
} // end of GetAllCrdrs

/////////////////////////////////////////////////////////////////////////////
//
// Description: Collect all corridors with a given source lane.
//
// Remarks: Each corridor has a single source and a single destination 
//   lane, and this function collects all corridors who share the 
//   source lane specified in the cLane argument.
//
//   Note that it is allowable for a lane to lead into no corridors
//   in a given intersection.  In that case, the output set will be 
//   emtpy.
//
//   Throws an exception if the corridor is invalid.
//
// Arguments:
// 	 cLane - The source lane.
// 	 crdrs - (output) An STL vector holding the corridors starting 
//       at the given lane.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetCrdrs( const CLane& cLane, TCrdrVec& crdrs ) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs( c );

	crdrs.clear();

	TCrdrVec::iterator i;
	for( i = c.begin(); i != c.end(); i++ )
	{
		if( i->GetSrcLn() == cLane )  crdrs.push_back( *i );
	}
} // end of GetCrdrs

/////////////////////////////////////////////////////////////////////////////
//
// Description: Collect all corridors with a given source lane and 
//   their destination roads.
//
// Remarks: Each corridor has a single source and a single destination 
//   lane and road, and this function collects all corridors who share 
//   the source lane specified in the cLane argument.
//
//   Note that it is allowable for a lane to lead into no corridors
//   in a given intersection.  In that case, the output set will be
//   empty.
//
//   Throws an exception if the corridor is invalid.
//
// Arguments:
//   cLane - The source lane.
//   crdrs - (output) An STL vector holding the corridords starting at Lane.
//   roads - (output) An STL vector holding the destination roads.
//	
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetCrdrs(
			const CLane& cLane, 
			TCrdrVec& crdrs, 
			TRoadVec& roads
			) const
{
	AssertValid();

	TCrdrVec c;
	TRoadVec r;
	GetAllCrdrs( c );
	GetRoads( r );
	
	crdrs.clear();
	roads.clear();

	TCrdrVec::iterator i;
	for( i = c.begin(); i != c.end(); i++ )
	{
		if( i->GetSrcLn() == cLane )
		{
			crdrs.push_back( *i );
			roads.push_back( i->GetDstntnRd() );
		}
	}
} // end of GetCrdrs

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdr
// 	Return a corridor connecting two lanes.
//
// Remarks: If no corridor can be found in the current intersection that 
// 	connects the two lanes, then an invalid CCrdr instance is returned.
//
// Arguments:
// 	cSrc - the source lane
// 	cDst - the destination lane
//
// Return value: The corridor connecting the lanes.  If no such corridor 
// 	exists, the returned CCrdr class will be bound but invalid.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr
CIntrsctn::GetCrdr(const CLane& cSrc, const CLane& cDst) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);
	bool	found = false;

	TCrdrVec::iterator i;
	for (i = c.begin(); i != c.end(); i++)
		if ( (i->GetSrcLn() == cSrc) &&
			 (i->GetDstntnLn() == cDst) ){
			found = true;
			break;
		}
	
	if (found)
		return (*i);
	else {
		CCrdr	bound(GetCved(), 0);
		return	bound;
	}
} // end of GetCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdrIdx
// 	Retrieves the index of the first corridor in the current intersection.
//
// Remarks:
//
// Arguments:
//
// Returns: the integer identifier of the corridor index
//
//////////////////////////////////////////////////////////////////////////////
int
CIntrsctn::GetCrdrIdx(void) const
{
	AssertValid();
	return m_pIntrs->crdrIdx;
} // end of GetCrdrIdx

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNumCrdrs
// 	Retrieves the number of the corridors in the current intersection.
//
// Remarks:
//
// Arguments:
//
// Returns: the integer quantity of corridors
//
//////////////////////////////////////////////////////////////////////////////
int
CIntrsctn::GetNumCrdrs(void) const
{
	AssertValid();
	return m_pIntrs->numOfCrdrs;
} // end of GetNumCrdrs

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdrsLeadingTo
// 	Collect all corridors leading to a target road.
//
// Remarks: This function puts in the specified vector all corridors that
// 	lead to the specified road.
//
// 	Note that it is allowable for a road to be adjacent to an intersection 
// 	but have no corridors coming out or leading into that intersection.
//
// Arguments:
// 	cDst - the destination road
// 	out - an STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetCrdrsLeadingTo(const CRoad& cDst, TCrdrVec& out) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);

	out.clear();
	TCrdrVec::iterator i;
	for (i = c.begin(); i != c.end(); i++)
		if (i->GetDstntnRd() == cDst)
			out.push_back(*i);
} // end of GetCrdrsLeadingTo

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdrsLeadingTo
// 	Collect all corridors leading to a target lane.
//
// Remarks: This function puts in the specified vector all corridors that
// 	lead to the specified lane.
//
// 	Note that it is allowable for a lane to be adjacent to an intersection 
// 	but have no corridors coming out or leading into that intersection.
//
// Arguments:
// 	cDst - the destination lane
// 	out - an STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetCrdrsLeadingTo(const CLane& cDst, TCrdrVec& out) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);

	out.clear();
	TCrdrVec::iterator i;
	for (i = c.begin(); i != c.end(); i++)
		if (i->GetDstntnLn() == cDst)
			out.push_back(*i);
} // end of GetCrdrsLeadingTo
////////////////////////////////////////////////////////////////////////////////
///\brief
///     This function get all the Crdrs controlled by a light
///\remark 
///     This function gets all the crdrs that have a hold offset that is controlled
///     by the specified light. A crdr may have multiple lights 
///\par TrafLightId -cved id of the light
///\par out -vector of the the crdrs controlled by TrafLightId
////////////////////////////////////////////////////////////////////////////////
void CIntrsctn::GetCrdrsControlledByLight( int TrafLightId, TCrdrVec& out) const{
 	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);

	out.clear();
	TCrdrVec::iterator i;
	vector<CVED::CHldOfs> hldOfsVec;

	for (i = c.begin(); i != c.end(); i++){
        i->GetAllHldOfs(hldOfsVec);
#ifdef _DEBUG
        cvTHldOfs temp; //for the debugger
        temp.distance = 0;
#endif
		vector<CVED::CHldOfs>::iterator hIter = hldOfsVec.begin();
        for (; hIter != hldOfsVec.end(); hIter++){
            if (hIter->GetObjId() == TrafLightId)
                out.push_back(*i);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
///\brief
///     This function checks to see if any crdr has a control
///\remark 
///     A "Dummy" intersection may have some sort of traffic control device in
///     it, like a tollbooth, metered interstate, or ped crossing, this checks
///     for a holdoffset and a control with the holdoffset, if any int has this,
///     then ADOs need to pay attention to this intersection.
////////////////////////////////////////////////////////////////////////////////
bool
CIntrsctn::IsControlled() const{
 	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);

	TCrdrVec::iterator i;
	vector<CVED::CHldOfs> hldOfsVec;

	for (i = c.begin(); i != c.end(); i++){
        i->GetAllHldOfs(hldOfsVec);
#ifdef _DEBUG
        cvTHldOfs temp; //for the debugger
        temp.distance =0;
#endif
		vector<CVED::CHldOfs>::iterator hIter = hldOfsVec.begin();
        for (; hIter != hldOfsVec.end(); hIter++){
            if (hIter->GetObjId() > 0 )
                return true;
        }
    }
    return false;
}
/////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdrsStartingFrom
// 	Collect all corridors leading from a source road.
//
// Remarks:  This function puts in the specified vector all corridors 
//   that lead from the specified source road.
//
//   Note that it is allowable for a road to be adjacent to an 
//   intersection but have no corridors coming out or leading into 
//   that intersection.
//
// Arguments:
//   src - The source road.
//   out - A STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetCrdrsStartingFrom(const CRoad& src, TCrdrVec& out) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);

	out.clear();
	TCrdrVec::iterator i;
	for (i=c.begin(); i!=c.end(); ++i)
		if (i->GetSrcRd() == src)
			out.push_back(*i);
} // end of GetCrdrsStartingFrom

/////////////////////////////////////////////////////////////////////////////
//
// Description: GetCrdrsStartingFrom
// 	Collect all corridors leading from a source lane.
//
// Remarks:  This function puts in the specified vector all corridors 
//   that lead from the specified source lane.
//
//   Note that it is allowable for a lane to be adjacent to an 
//   intersection but have no corridors coming out or leading into 
//   that intersection.
//
// Arguments:
//   src - The source lane.
//   out - A STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetCrdrsStartingFrom(const CLane& src, TCrdrVec& out) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);

	out.clear();
	TCrdrVec::iterator i;
	for (i=c.begin(); i!=c.end(); ++i)
		if (i->GetSrcLn() == src)
			out.push_back(*i);
} // end of GetCrdrsStartingFrom

//////////////////////////////////////////////////////////////////////////////
//
// Description: Collect roads that can be reached from another road.
//
// Remarks: This function collects all roads that can be reached when 
// 	traveling through the source and destination intersections and starting 
//	from the specified road.
//
// 	To be listed in the output vector, a road must have one (or more) of its 
// 	lanes be the destination of a corridor whose source is one of the lanes 
// 	of source.
//
//  Clears the output vector.
//
// 	This function guards against returning duplicate roads in the
//  output vector.  It is possible for the output vector to be empty.
//
// Arguments:
// 	cRoad - The road that will be used to find reachable roads.
// 	out - (output) An STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetRoadAccessList( const CRoad& cRoad, TRoadVec& out ) const
{
	AssertValid();

	CIntrsctn srcIntrsctn( GetCved(), cRoad.GetSourceIntrsctnIdx() );
	CIntrsctn dstIntrsctn( GetCved(), cRoad.GetDestIntrsctnIdx() );

	//
	// Clear the output vector and reserve the maxiumum amount of 
	// space necessary in the output vector to make it more efficient.
	//
	out.clear();
	out.reserve( srcIntrsctn.GetNumCrdrs() + dstIntrsctn.GetNumCrdrs() );

	// 
	// Get all corridors from the given road's source intersection.
	// Look at each corridor; if the source road is the same as the
	// given road, then insert the destination road in the output
	// vector.
	//
	TCrdrVec c;
	srcIntrsctn.GetAllCrdrs( c );

	TCrdrVec::iterator i;
	for( i = c.begin(); i != c.end(); i++ )
	{
		bool srcRoadMatch = i->GetSrcRd() == cRoad;
		if( srcRoadMatch )
		{
			//
			// Insert this road only if it hasn't already been inserted.
			//
			CRoad roadToInsert = i->GetDstntnRd();
			TRoadVec::const_iterator j = find( 
											out.begin(), 
											out.end(), 
											roadToInsert 
											);
			bool foundRoad = ( j != out.end() );
			if( !foundRoad )  out.push_back( i->GetDstntnRd() );
		}
	}

	// 
	// Get all corridors from the given road's destination intersection.
	// Look at each corridor; if the source road is the same as the
	// given road, then insert the destination road in the output
	// vector.
	//
	dstIntrsctn.GetAllCrdrs( c );
	for( i = c.begin(); i != c.end(); i++ )
	{
		bool srcRoadMatch = i->GetSrcRd() == cRoad;
		if( srcRoadMatch )
		{
			//
			// Insert this road only if it hasn't already been inserted.
			//
			CRoad roadToInsert = i->GetDstntnRd();
			TRoadVec::const_iterator j = find( 
											out.begin(), 
											out.end(), 
											roadToInsert 
											);
			bool foundRoad = ( j != out.end() );
			if( !foundRoad )  out.push_back( i->GetDstntnRd() );
		}
	}
} // end of GetRoadAccessList

//////////////////////////////////////////////////////////////////////////////
//
// Description: Collect roads that can be reached from the given 
//   road through the current intersection.
//
// Remarks: This function collects all roads that can be reached 
//   when traveling through this intersection and starting from 
//   the specified road.  To be listed in the output vector, a 
//   road must have one (or more) of its lanes be the destination 
//   of a corridor whose source lane is one of the lanes of srcRd.
//
// 	 This function guards against returning duplicate roads in the
//   output vector.  It is possible for the output vector to be empty.
//
// Arguments:
//   cSrcRd - The road that will be used to find reachable roads.
//   out - (output) An STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CIntrsctn::GetRoadAccessListThrghIntrsctn(
			const CRoad& cSrcRd, 
			TRoadVec& out
			) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);
	
	out.clear();
	
	TCrdrVec::iterator i;
	for( i = c.begin(); i != c.end(); i++ )
	{
		if( i->GetSrcRdIdx() == cSrcRd.GetId() )
		{
			//
			// Insert this road only if it hasn't already been inserted.
			//
			CRoad roadToInsert = i->GetDstntnRd();
			TRoadVec::const_iterator j = find( 
											out.begin(), 
											out.end(), 
											roadToInsert 
											);
			bool foundRoad = ( j != out.end() );
			if( !foundRoad )  out.push_back( i->GetDstntnRd() );
		}
	}
} // end of GetRoadAccessListThrghIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: Collect roads that can be reached from the given 
//   lane through the current intersection.
//
// Remarks: This function collects all roads that can be reached 
//   when traveling through this intersection and starting from 
//   the specified lane.  To be listed in the output vector, a 
//   road must have one (or more) of its lanes be the destination 
//   of a corridor whose source lane is the one given.
//
// 	 This function guards against returning duplicate roads in the
//   output vector.  It is possible for the output vector to be empty.
//
// Arguments:
//   cSrcLane - The lane that will be used to find reachable roads.
//   out - (output) An STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CIntrsctn::GetRoadAccessListThrghIntrsctn(
			const CRoad& cSrcRd, 
			const bitset<cCV_MAX_CRDRS>& cSrcLaneMask, 
			TRoadVec& out
			) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs(c);
	
	out.clear();
	
	TCrdrVec::iterator i;
	for( i = c.begin(); i != c.end(); i++ )
	{
		if( i->GetSrcRdIdx() == cSrcRd.GetId() )
		{
			if( cSrcLaneMask.test( i->GetSrcLn().GetRelativeId() ) )
			{
				//
				// Insert this road only if it hasn't already been inserted.
				//
				CRoad roadToInsert = i->GetDstntnRd();
				TRoadVec::const_iterator j = find( 
												out.begin(), 
												out.end(), 
												roadToInsert 
												);
				bool foundRoad = ( j != out.end() );
				if( !foundRoad )  out.push_back( i->GetDstntnRd() );
			}
		}
	}
} // end of GetRoadAccessListThrghIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: Collect roads that can reach the given road through 
//   the current intersection.
//
// Remarks: This function collects all roads that can reach the given 
//   road when traveling through this intersection.  To be listed in 
//   the output vector, a road must have one (or more) of its lanes 
//   be the source of a corridor whose destination lane is one of the 
//   lanes of srcRd.
//
// 	 This function guards against returning duplicate roads in the
//   output vector.  It is possible for the output vector to be empty.
//
// Arguments:
// 	 cDstRd - The road the is the destination of the resulting road 
//       vector.
// 	 out - (output) An STL vector to hold the specified corridors.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CIntrsctn::GetReverseRoadAccessListThrghIntrsctn(
			const CRoad& cDstRd, 
			TRoadVec& out
			) const
{
	AssertValid();

	TCrdrVec c;
	GetAllCrdrs( c );
	
	out.clear();
	
	TCrdrVec::iterator i;
	for( i = c.begin(); i != c.end(); i++ )
	{
		if( i->GetDstntnRdIdx() == cDstRd.GetId() )
		{
			//
			// Insert this road only if it hasn't already been inserted.
			//
			CRoad roadToInsert = i->GetDstntnRd();
			TRoadVec::const_iterator j = find( 
											out.begin(), 
											out.end(), 
											roadToInsert 
											);
			bool foundRoad = ( j != out.end() );
			if( !foundRoad )  out.push_back( i->GetSrcRd() );
		}
	}
} // end of GetReverseRoadAccessListThrghIntrsctn

//////////////////////////////////////////////////////////////////////////////
//
// Description: DoRoadsConnect
// 	Determine if two roads connect through the current intersection
//
// Remarks: This function determines if two roads connect through this
// 	intersection.  The two roads connect, if and only if there is a corridor 
// 	that has as a source one of the lanes of the cSrcRd road and as a 
// 	destination one of the lanes of cDstRd.
//
// Arguments:
// 	cSrcRd - the source road 
// 	cDstRd - the destination road
//
// Return value: True if the roads connect or false otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CIntrsctn::DoRoadsConnect(const CRoad& cSrcRd, const CRoad& cDstRd) const
{
	AssertValid();

	CIntrsctn srcIntrsctnOfSrcRd(GetCved(), cSrcRd.GetSourceIntrsctnIdx());
	CIntrsctn dstIntrsctnOfSrcRd(GetCved(), cSrcRd.GetDestIntrsctnIdx());

	TCrdrVec c;
	TCrdrVec::iterator i;

	srcIntrsctnOfSrcRd.GetAllCrdrs(c);
	for (i = c.begin(); i != c.end(); i++)
		if ( (i->GetSrcRd() == cSrcRd) &&
			 (i->GetDstntnRd() == cDstRd) )
			return true;
	
	dstIntrsctnOfSrcRd.GetAllCrdrs(c);
	for (i = c.begin(); i != c.end(); ++i)
		if ( (i->GetSrcRd() == cSrcRd) &&
			 (i->GetDstntnRd()== cDstRd) )
			return true;
	
	CIntrsctn srcIntrsctnOfDstRd(GetCved(), cDstRd.GetSourceIntrsctnIdx());
	CIntrsctn dstIntrsctnOfDstRd(GetCved(), cDstRd.GetDestIntrsctnIdx());

	srcIntrsctnOfDstRd.GetAllCrdrs(c);
	for (i = c.begin(); i != c.end(); i++)
		if ( (i->GetSrcRd() == cDstRd) &&
			 (i->GetDstntnRd() == cSrcRd) )
			return true;
	
	dstIntrsctnOfDstRd.GetAllCrdrs(c);
	for (i = c.begin(); i != c.end(); i++)
		if ( (i->GetSrcRd() == cDstRd) &&
			 (i->GetDstntnRd() == cSrcRd) )
			return true;

	return false;
} // end of DoRoadsConnect

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetBorder
// 	Obtain the border of the intersection
//
// Remarks: This function constructs a 2D polygon that represents the border 
// 	of the intersection.
//
// Arguments:
// 	poly - where to store the polygon
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::GetBorder(CPolygon2D& poly) const
{
	TBorderSegPoolIdx	vertex;
	TBorderSeg*			pSegPool;
	char*				pStorage;

	poly.Clear();
	
	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	pStorage = reinterpret_cast<char*>(pH) + pH->borderOfs;
	pSegPool = reinterpret_cast<cvTBorderSeg*>(pStorage);

	for (vertex = 0; vertex < m_pIntrs->numBorderSeg; vertex++) {
		TBorderSeg*	pV = pSegPool + (m_pIntrs->borderSegIdx + vertex);
		poly.AddPoint(pV->x, pV->y);
	}
} // end of GetBorder

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryAttr
// 	Obtains the CAttr that has the given id and is on the current 
// 	intersection.
//
// Remarks: This function will cause an assertion failure if it is called on 
// 	an invalid CIntrsctn object.
//
// Arguments:
//	id - identifier of an attribute type
//	attr - output parameter that contains the desired attribute, if the 
//			function returns true.
//
// Returns: If an attribute is found that meets the given criteria, the 
//	attribute is placed in attr and the function returns true.  Otherwise, 
//	the function returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CIntrsctn::QryAttr(int id, 
				   CAttr& attr
				   ) const
{
	AssertValid();

	cvTHeader	*pH   = static_cast<cvTHeader*>  (GetInst());
	char        *pOfs = static_cast<char*>       (GetInst()) + 
						pH->attrOfs;
	cvTAttr		*pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +
						m_pIntrs->attrIdx;
	int i;

	// For each attribute on the intersection
	for (i = 0; i < m_pIntrs->numAttr; i++, pAttr++) {

		// take out those reserved for attributes created at runtime.
		if (pAttr->myId == 0) 
			continue;
 
		// If the attr ids match ...
		if (pAttr->myId == id) {

#ifdef DEBUG_QRY_ATTR2
			gout << " Attribute ids match. " << endl;
#endif


			// Return the current attribute
			attr = CAttr(GetCved(), pAttr);
			return true;
		}

	} // For each attribute on the intersection

	// No matching attribute found
	return false;
} // end of QryAttr

//////////////////////////////////////////////////////////////////////////////
//
// Description: QryAttr
// 	Obtains the CAttrs that are on the current intersection, 
// 
//
// Remarks: This function will cause an assertion failure if it is called on 
// 	an invalid CIntrsctn object.
//
// Arguments:
//	attrVec - output parameter that contains the desired attributes, if the 
//			function returns true.	
//
// Returns: If an attribute is found, it is placed in attr and the function
//	returns true.  Otherwise, the function returns false.
//
//////////////////////////////////////////////////////////////////////////////
bool 
CIntrsctn::QryAttr(
				vector<CAttr>& attrVec
				) const
{
	AssertValid();

	attrVec.clear();

	cvTHeader	*pH   = static_cast<cvTHeader*>  (GetInst());
	char        *pOfs = static_cast<char*>       (GetInst()) + 
						pH->attrOfs;
	cvTAttr		*pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +
						m_pIntrs->attrIdx;
	int i;

	// For each attribute on the intersection
	for (i = 0; i < m_pIntrs->numAttr; i++, pAttr++) {

		// take out those reserved for attributes created at runtime.
		if (pAttr->myId == 0) 
			continue;

		// Return the current attribute
		attrVec.push_back(CAttr(GetCved(), pAttr));
	
	} // For each attribute on the intersection

	if (attrVec.size() == 0){

#ifdef	DEBUG_QRY_ATTR
		gout << " No matching attribute found. " << endl;
#endif
		// No matching attribute found
		return false;
	} else {
		return true;
	}
 
} // end of QryAttr





//////////////////////////////////////////////////////////////////////////////
//
// Description: For each corridor in the current intersection this 
//   function returns a list of its intersecting corridors that have 
//   equal or higher priority. The priority is determined by road 
//   signs first, by corridor geometry next (a corridor on the right
//   has the right of way over the corridor on the left), and by the 
//   direction of the corridor last (the corridor going straight has 
//   the right of way over the corridor making a left turn).	 
//
// Remarks: The output vector could be empty. 
//
// Arguments:
//	 out - The output parameter that contains all the corridors in 
//         the current intersection and their lists of intersecting 
//         corridors that have equal or higher priority. Note that 
//         the vector index matches the corridor ids.
//
// Returns:	void 
//
//////////////////////////////////////////////////////////////////////////////
void 
CIntrsctn::PrioritizeCorridors( 
			vector<TCrdrPriorityList>& out 
			) const
{
#ifdef DEBUG_PRIORITIZE_CORRIDORS
	gout << "=== PrioritizeCorridors ========================" << endl;
	gout << "intrsctn = " << GetName() << endl;
#endif
	
	//
	// Get all corridors at the current intersection.
	//
	/*static*/ vector<CCrdr> allCrdrs;

	GetAllCrdrs( allCrdrs );

	//
	// For each corridor, get the corridors intersecting with it.
	// Then look at each corridor one by one to compare its
	// priority with the current corridor. Then eliminate those
	// intersecting corridors that have lower priority than
	// the current corridor based on the 3 types of corridors with
	// uncontrolled corridor having priority 1, corridor with yield 
	// sign having priority 2 and corridor with stop sign having 
	// priority 3.
	//
	vector<int>::iterator j;
	int currCrdrPriority;
	int interCrdrPriority;

	/*static*/ vector<int> temp;
	temp.clear();
	temp.reserve(allCrdrs.size());
	vector<CCrdr>::iterator i;
	for( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		CCrdr crdr = *i;
		if( !crdr.IsValid() )  continue;

		// Get the current corridor's priority.
		currCrdrPriority = crdr.GetCrdrPriority();

		// Get the current corridor's direction.
		CCrdr::ECrdrDirection currCrdrDir = crdr.GetCrdrDirection();

#ifdef DEBUG_PRIORITIZE_CORRIDORS
		gout << "looking at crdr = " << crdr.GetRelativeId();
		gout << "   " << crdr.GetSrcLn().GetName();
		gout << " --> " << crdr.GetDstntnLn().GetName() << endl;
		gout << "priority = " << currCrdrPriority;
		gout << "  direction " << crdr.GetCrdrDirectionStr() << endl;
#endif

		/*static*/ vector<int> intrsctingCrdrs;
		//intrsctingCrdrs.clear();
		crdr.GetIntersectingCrdrs( intrsctingCrdrs );
		for( j = intrsctingCrdrs.begin(); j != intrsctingCrdrs.end(); j++ )
		{
			// Create a corridor instance from the id.
			int interCrdrId = *j;
			CCrdr interCrdr( *this, interCrdrId );

			// Get the corridor's priority.
			interCrdrPriority = interCrdr.GetCrdrPriority();

			// Get the corridor's direction.
			CCrdr::ECrdrDirection interCrdrDir = interCrdr.GetCrdrDirection();

#ifdef DEBUG_PRIORITIZE_CORRIDORS
			gout << "  other crdr id = " << interCrdr.GetRelativeId();
			gout << "   priority = " << interCrdrPriority;
			gout << "   direction = " << interCrdr.GetCrdrDirectionStr();
			gout << endl;
#endif

			//
			// A lower priority value actually has a higher priority.
			// First use the type of the corridor to collect the corridors with
			// higher priority in the output vector. When their types are
			// equal, elimilate those corridors(vehicles) to the left 
			// geometrically since the corridors(vehicles) to the right have
			// the right of way. Then further elimilate those intersecting
			// corridors that are making a left turn while the current corridor
			// is going straight, which has the right of way.
			//
			if( currCrdrPriority > interCrdrPriority )
			{	
#ifdef DEBUG_PRIORITIZE_CORRIDORS
				gout << "  *has higher priority" << endl;
#endif

				temp.push_back( interCrdrId );	
			}
			else if( currCrdrPriority == interCrdrPriority )
			{
				CCrdr::ECrdrGeometry geo = crdr.DecideCrdrByGeometry( interCrdr );
                bool isOnRamp = false;
                auto lane =  crdr.GetSrcLn();
                if (lane.IsValid()){
                    isOnRamp = lane.IsOnRamp();
                }

#ifdef DEBUG_PRIORITIZE_CORRIDORS
				gout << "  crdr geometry = " << crdr.GeometryToStr( geo );
				gout << endl;
#endif
                if (isOnRamp || currCrdrPriority == CCrdr::eSTOP){
                    temp.push_back( interCrdrId );
                }
				else if( geo == CCrdr::eTOTHELEFT )
				{
#ifdef DEBUG_PRIORITIZE_CORRIDORS
					gout << "  *has higher priority...to my right" << endl;
#endif

					temp.push_back( interCrdrId );
				}
				else if( geo == CCrdr::eNOTLEFTNORRIGHT )
				{
					bool interCrdrStraightOrRight = (
								interCrdrDir == CCrdr::eSTRAIGHT ||
								interCrdrDir == CCrdr::eRIGHT
								);
					bool hasHigherPriority = (
								currCrdrDir == CCrdr::eLEFT && 
								interCrdrStraightOrRight
								);
					if( hasHigherPriority )
					{
#ifdef DEBUG_PRIORITIZE_CORRIDORS
						gout << "  *has higher priority...to my straight" << endl;
#endif

						temp.push_back( interCrdrId );
					}
				}
			}
		} 

		//
		// Get the hold offsets.
		//
		double hldOfsDist = 0.0;
		/*static*/ vector<CHldOfs> hldOfs;
		//hldOfs.clear();
		
		crdr.GetAllHldOfs( hldOfs );
		if( hldOfs.size() > 0 )
		{
			hldOfsDist = hldOfs[0].GetDistance();
		}

		// Insert into the output list
		TCrdrPriorityList node;
		node.intrsctingCrdrs = temp;
		node.hldOfsDist = hldOfsDist;
		out.push_back( node );
		temp.clear();
	}
}  // end of PrioritizeCorridors

} // namespace CVED