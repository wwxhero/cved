//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: crdr.cxx,v 1.53 2015/10/30 16:56:22 iowa\oahmad Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	The implementation of the CCrdr class
//
//////////////////////////////////////////////////////////////////////////////

#include "cvedpub.h"
#include "cvedstrc.h"		// private CVED data structs
#include <algorithm>

#undef DEBUG_GET_CRDR_PRIORITY
#undef DEBUG_GET_CRDR_DIRECTION
#undef DEBUG_DECIDE_CRDR_BY_GEOMETRY
#undef DEBUG_GET_WIDTH
// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: This constructor initializes the current instance 
//   with a CIntrsctn, a source CLane, and a destination CLane.
//
// Remarks: 
//
// Arguments:
//	 intrsctn - A reference to the intrsctn where the crdr is located.
//	 src      - The source lane of the crdr.
//	 dst      - The destination lane of the crdr.
//
// Returns: void 
//
//////////////////////////////////////////////////////////////////////////////
CCrdr::CCrdr(
			const CIntrsctn& intrsctn, 
			const CLane& src, 
			const CLane& dst
			) 
	:
	CCvedItem( intrsctn )
{
	TCrdrVec allCrdrs;
	
	intrsctn.GetCrdrs( src, allCrdrs );
	TCrdrVec::iterator i;
	for( i = allCrdrs.begin(); i != allCrdrs.end(); i++ )
	{
		if( i->GetDstntnLn() == dst )
		{
			m_pCrdr = i->m_pCrdr;
			return;
		}
	}
} // end of CCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: This constructor initializes the current instance 
//   with a CCved instance and an integer corridor id.
//
// Remarks: This constructor will cause a failed assertion if the given
//   corridor id is invalid.
//
// Arguments:  
//	 cved - A reference to a CCved instance.
//   id   - The crdr id.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CCrdr::CCrdr( const CCved& cCved, int id ) 
	:
	CCvedItem( &cCved )
{
	cvTHeader* pH   = static_cast<cvTHeader*> (GetInst());
	char*      pOfs = static_cast<char*>      (GetInst()) + pH->crdrOfs;

	// 
	// Make sure the id is within bounds.
	//
	if( id <= 0 || id >= (int)pH->crdrCount )
	{
		stringstream err;
        err<<"No crdr with id = "<<id;
        string msg = err.str();
		cvCInternalError e( msg, __FILE__, __LINE__ );
		throw e;
	}

	m_pCrdr = ( reinterpret_cast<TCrdr*>( pOfs ) ) + id;
} // end of CCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: This constructor initializes the current instance 
//   with a CIntrsctn and an integer corridor id, relative to the 
//   given CIntrsctn.
//
// Remarks: This constructor will cause a failed assertion if the given
//   corridor id is invalid.
//
// Arguments:  
// 	 cIntrsctn  - A valid CIntrsctn instance.
//   relativeId - The id of the crdr, relative to cIntrsctn.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CCrdr::CCrdr( const CIntrsctn& cIntrsctn, int relativeId )
	:
	CCvedItem( cIntrsctn )
{
	cvTHeader* pH   = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs = static_cast<char*>       ( GetInst() ) + pH->crdrOfs;

	int id = cIntrsctn.GetCrdrIdx() + relativeId;

	// 
	// Make sure the id is within bounds.
	//
	if( id <= 0 || id >= (int)pH->crdrCount )
	{
		stringstream err;
        err<<"No crdr with id = "<<id;
        string msg = err.str();
		cvCInternalError e( msg, __FILE__, __LINE__ );
		throw e;
	}

	m_pCrdr = ( reinterpret_cast<TCrdr*>( pOfs ) ) + id;
} // end of CCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: Performs a deep copy of the parameter to the 
//   current instance.
//
// Remarks: 
//
// Arguments:
//   cRhs - A reference to an object intended to be to the right of the =.
//
// Returns: A reference to the current CCrdr instance.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr&
CCrdr::operator=( const CCrdr& cRhs )
{
	if( this != &cRhs )
	{
		// Assign superclass members
		((CCvedItem*)(this))->operator=(cRhs);

		m_pCrdr = cRhs.m_pCrdr;
	}

	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: Return a string describing the CCrdr.
//
// Remarks: If the CCrdr is invalid, this function will cause a failed
// 	assertion.
//
//	This function returns a string that contains a textual description
// 	of the corridor.  The string simply contains an integer that is the
//  corridor's id.
//
// Arguments:
//
// Returns: A string describing the component.
//
//////////////////////////////////////////////////////////////////////////////
string 
CCrdr::GetString( void ) const
{
	AssertValid();

	char buf[128];
	sprintf_s( buf, "%d", m_pCrdr->myId );
	string str = buf;

	return str;
} // end of GetString

//////////////////////////////////////////////////////////////////////////////
//
// Description: Initializes the current CCrdr with the string parameter.
//
// Remarks: The parameter should contain a string with the same format 
//   as the string returned by GetString().  The string should simply 
//   contain one integer that represents the crdr's unique id in the
//   crdr array.
//
//   This function will cause a failed assertion if string contains an
//   invalid corridor id.
//
// Arguments:
// 	 cName - String containing the formatted CCrdr name.
//
// Returns: false if there are any format errors or if the name is invalid, 
//	 true otherwise.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCrdr::SetString( const string& cName )
{

	CCvedItem::AssertValid();

	string strName;
	int id;
    int size = (int)strlen(cName.c_str())+1;
	char* pBuf = new char[size];
    char* pTokDel = " \t:";
    char* pTok;
	char* pCurPos = NULL;

    strcpy_s( pBuf,size, cName.c_str() );
    pTok = strtok_s( pBuf, pTokDel,&pCurPos );
    id = atoi( pTok );
	delete [] pBuf;

	cvTHeader* pH   = static_cast<cvTHeader*> (GetInst());
	char*      pOfs = static_cast<char*>      (GetInst()) + pH->crdrOfs;

	// 
	// Make sure the id is within bounds.
	//
	if( id <= 0 || id >= (int)pH->crdrCount )
	{
		char buf[1024];
		sprintf_s( buf, "No crdr with id = %d", id );
		string msg = buf;
		cvCInternalError e( msg, __FILE__, __LINE__ );
		throw e;
	}

	m_pCrdr = ( reinterpret_cast<TCrdr*>( pOfs ) ) + id;

	return true;
} // end of SetString

//////////////////////////////////////////////////////////////////////////////////
// Description: GetIntrsctnId
// 	This function returns the identifier of the intersection the current 
// 	corridor belongs to.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: The integer identifier of the current intersection.
//
//////////////////////////////////////////////////////////////////////////////////
int 
CCrdr::GetIntrsctnId(void) const
{
	AssertValid();
	return m_pCrdr->intrsctnId;

} // GetIntrsctnId

//////////////////////////////////////////////////////////////////////////////////
// Description: GetSrcLnIdx
// 	This function returns the source lane index of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: The integer identifier of the source lane of the corridor. 
// 
//////////////////////////////////////////////////////////////////////////////
int
CCrdr::GetSrcLnIdx(void) const
{
	AssertValid();
	return m_pCrdr->srcLnIdx;

} // end of GetSrcLnIdx

//////////////////////////////////////////////////////////////////////////////////
// Description: GetDstntnLnIdx
// 	This function returns the destination lane index of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: The integer identifier of the destination lane of the corridor. 
// 
//////////////////////////////////////////////////////////////////////////////
int
CCrdr::GetDstntnLnIdx(void) const
{
	AssertValid();
	return m_pCrdr->dstLnIdx;

} // end of GetDstntnLnIdx

//////////////////////////////////////////////////////////////////////////////////
// Description: GetSrcRdIdx
// 	This function returns the source road index of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: The integer identifier of the source road of the corridor. 
// 
//////////////////////////////////////////////////////////////////////////////
int
CCrdr::GetSrcRdIdx(void) const
{
	AssertValid();
	return m_pCrdr->srcRdIdx;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////////
// Description: GetDstntnRdIdx
// 	This function returns the destination road index of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: The integer identifier of the destination road of the corridor. 
// 
//////////////////////////////////////////////////////////////////////////////
int
CCrdr::GetDstntnRdIdx(void) const
{
	AssertValid();
	return m_pCrdr->dstRdIdx;

} // end of GetDstntnRdIdx

//////////////////////////////////////////////////////////////////////////////////
// Description: GetIntrsctn
// 	This function returns a CIntrsctn representing the intersection the current 
// 	corridor belongs to.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: A CIntrsctn representing of the current intersection.
//
////////////////////////////////////////////////////////////////////////////////
CIntrsctn
CCrdr::GetIntrsctn(void) const
{
	AssertValid();
	CIntrsctn intrsctn(GetCved(), m_pCrdr->intrsctnId);
	return intrsctn;

} // GetIntrsctn

//////////////////////////////////////////////////////////////////////////
//
// Description: GetSrcLn
//
// 	This function returns the source lane of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: A bound and initialized lane class representing the source lane 
// 	of the corridor.  
// 
//////////////////////////////////////////////////////////////////////////////
CLane 
CCrdr::GetSrcLn(void) const
{
	AssertValid();

	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*>       (GetInst()) + pH->laneOfs;
	TLane*		pLane = (reinterpret_cast<TLane*>(pOfs))+m_pCrdr->srcLnIdx;

	CLane srcLn(GetCved(), pLane);

	return srcLn;
} // end of GetSrcLn

//////////////////////////////////////////////////////////////////////////
//
// Description: GetDstntnLn
// 	This function returns the destination lane of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: A bound and initialized lane class representing the destination 
// 	lane of the corridor.  
// 
//////////////////////////////////////////////////////////////////////////////
CLane 
CCrdr::GetDstntnLn(void) const
{
	AssertValid();

	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*>       (GetInst()) + pH->laneOfs;
	TLane*		pLane = reinterpret_cast<TLane*>(pOfs)+m_pCrdr->dstLnIdx;

    CLane dstLn(GetCved(), pLane);

	return dstLn;
} // end of GetDstntnLn

//////////////////////////////////////////////////////////////////////////
//
// Description: GetSrcRd
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: A bound and initialized road class representing the source road 
// 	of the corridor.  
// 
//////////////////////////////////////////////////////////////////////////////
CRoad 
CCrdr::GetSrcRd(void) const
{
	AssertValid();

	CRoad srcRd(GetCved(), m_pCrdr->srcRdIdx);

	return srcRd;
} // end of GetSrcRd

//////////////////////////////////////////////////////////////////////////
//
// Description: GetDstntnRd
// 	This function returns the destination road of the corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: A bound and initialized road class representing the destination 
// 	road of the corridor.  
// 
//////////////////////////////////////////////////////////////////////////////
CRoad 
CCrdr::GetDstntnRd(void) const
{
	AssertValid();

	CRoad dstRd(GetCved(), m_pCrdr->dstRdIdx);

	return dstRd;
} // end of GetDstntnRd

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetId
// 	This function returns the identifier of the current corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: integer id of the current corridor
// 
//////////////////////////////////////////////////////////////////////////////
int
CCrdr::GetId(void) const
{
	AssertValid();

	return m_pCrdr->myId;
} // end of GetId
	
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRelativeId
// 	This function returns the identifier of the current corridor with respect
//	to the corridor's intersection.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
//
// Returns: integer id of the current corridor with respect to the intersection
// 
//////////////////////////////////////////////////////////////////////////////
int
CCrdr::GetRelativeId(void) const
{
	AssertValid();

	TIntrsctn* pIntrsctn = BindIntrsctn(m_pCrdr->intrsctnId);
	return m_pCrdr->myId - pIntrsctn->crdrIdx;
} // end of GetRelativeId
	
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLength
// 	Return the length of the current corridor.  
//
// Remarks: The length is determined by the distance value of the last corridor 
// 	control point.
// 	If this corridor is not bound or uninitialized the function will throw an 
// 	exception.
//
// Arguments:
//
// Returns: double length
// 
//////////////////////////////////////////////////////////////////////////////
double
CCrdr::GetLength(void) const
{
	AssertValid();

	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*> (GetInst()) + pH->crdrCntrlPntOfs;
	TCrdrCntrlPnt*	pCrdrCntrlPnt = 
					(reinterpret_cast<TCrdrCntrlPnt*>(pOfs)) +
					m_pCrdr->cntrlPntIdx;
	pCrdrCntrlPnt += m_pCrdr->numCntrlPnt - 1;

	return pCrdrCntrlPnt->distance;
} // end of GetLength

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCntrlPnts
// 	This function puts in the specified vector all the control points of the 
// 	current corridor.
//
// Remarks: If this corridor is not bound or uninitialized the function will 
// 	throw an exception.
//
// Arguments:
// 	out - a vector that upon return holds all the control points
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CCrdr::GetCntrlPnts(TCntrlPntVec& out) const
{
	AssertValid();

	out.clear();
	TU32b i;
	CCrdr::TCntrlPnt	pt;

	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*> (GetInst()) + pH->crdrCntrlPntOfs;
	TCrdrCntrlPnt*	m_pCrdrCntrlPnt = 
					(reinterpret_cast<TCrdrCntrlPnt*>(pOfs)) +
					m_pCrdr->cntrlPntIdx;
	
	for (i=0; i<m_pCrdr->numCntrlPnt; i++){
		pt.width			= (m_pCrdrCntrlPnt+i)->width;
		pt.location			= (m_pCrdrCntrlPnt+i)->location;
		pt.rightVecLinear	= (m_pCrdrCntrlPnt+i)->rightVecLinear;
		pt.distance		= (m_pCrdrCntrlPnt+i)->distance;
		pt.flagStyle		= (m_pCrdrCntrlPnt+i)->flagStyle;

		out.push_back(pt);
	}
} // end of GetCntrlPnts

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetBoundaryPnts
//	This function puts in the specified vector the calculated boundary point 
//	structures of all contrl points of the current corridor. 
//
// Remarks: 
//	The boundary points are calculated by adding and subtracting 
//	rightVector * (width of road at point/2) from each control point location. 
// 	If this corridor is not bound or uninitialized the function will throw an 
// 	exception.
//
// Arguments:
//	out - a vector that upon return holds all the boundary points
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void 
CCrdr::GetBoundaryPnts(TCPoint3DVec& out) const
{
	AssertValid();

	out.clear();
	TU32b i;
	double		x1, x2, y1, y2;

	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());    
	char*		pOfs = static_cast<char*> (GetInst()) + pH->crdrCntrlPntOfs;
	TCrdrCntrlPnt*	m_pCrdrCntrlPnt =			
					(reinterpret_cast<TCrdrCntrlPnt*>(pOfs)) +
					m_pCrdr->cntrlPntIdx;

	
	for (i=0; i<m_pCrdr->numCntrlPnt; i++){

		x1 =(m_pCrdrCntrlPnt+i)->location.x + 
			(m_pCrdrCntrlPnt+i)->width/2*(m_pCrdrCntrlPnt+i)->rightVecLinear.i;
		x2 =(m_pCrdrCntrlPnt+i)->location.x - 
			(m_pCrdrCntrlPnt+i)->width/2*(m_pCrdrCntrlPnt+i)->rightVecLinear.i;
		y1 =(m_pCrdrCntrlPnt+i)->location.y + 
			(m_pCrdrCntrlPnt+i)->width/2*(m_pCrdrCntrlPnt+i)->rightVecLinear.j;
		y2 =(m_pCrdrCntrlPnt+i)->location.y - 
			(m_pCrdrCntrlPnt+i)->width/2*(m_pCrdrCntrlPnt+i)->rightVecLinear.j;

		CPoint3D pt1(x1, y1, 0);
		CPoint3D pt2(x2, y2, 0);

		out.push_back(pt1);
		out.push_back(pt2);
	}
} // end of GetBoundaryPnts

//////////////////////////////////////////////////////////////////////////////
//
// Description: Collect all hold offsets inside the current corridor.
//
// Remarks: This function causes a failed assertion if it is run on an
//  invalid CCrdr instance.
//
// Arguments:
//  out - (output) A vector that holds all hold offsets in the crdr.
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCrdr::GetAllHldOfs( vector<CHldOfs>& out ) const
{
	AssertValid();

	out.clear();
	out.reserve( m_pCrdr->numHldOfs );

	int i;
	for( i = 0; i < m_pCrdr->numHldOfs; i++ )
	{
		CHldOfs h( GetCved(), m_pCrdr->hldOfsIdx + i );
		out.push_back( h );
	}
}



//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the distance along the corridor at which the 
//   first hold offest is located.
//
// Remarks: This function causes a failed assertion if it is run on an
//  invalid CCrdr instance.  This function returns 0.0 if the corridor
//  doesn't contain any hold offsets.
//
// Arguments:
//
// Returns: A double indicating the distance along the corridor at which
//   the first hold offset is located.
//
//////////////////////////////////////////////////////////////////////////////
double
CCrdr::GetFirstHldOfsDist() const
{
	AssertValid();

	if( m_pCrdr->numHldOfs <= 0 )
	{
		return 0.0;
	}
	else
	{
		CHldOfs h( GetCved(), m_pCrdr->hldOfsIdx + 0 );
		return h.GetDistance();
	}
}



//////////////////////////////////////////////////////////////////////////////
//
// Description: GetHldOfByDist 
//  This function will return the holdoff specified by the dist parameter. If
//  there is no dist parameter, it will return the first holdoff of the current
//  corridor instance. If the dist parameter passed in is incorrect, there will be
//  no CHldOfs returned.
//
// Remarks: This function causes a failed assertion if it is run on an
//  invalid CCrdr instance.
//
// Arguments:
//  hldofs - The output parameter that holds the holdoff.
//  dist - The optional parameter that specify where the holdoff is located 
//  along the current corridor.
//
// Returns: Return true if the holdoff is successfully obtained; otherwise
//  return false.
//
//////////////////////////////////////////////////////////////////////////////


bool 
CCrdr::GetHldOfByDist( CHldOfs& hldofs, double dist ) const
{
	AssertValid();

	int i = 0;
	for (; i < m_pCrdr->numHldOfs; i++) {
		CHldOfs h( GetCved(), m_pCrdr->hldOfsIdx + i );
		if( dist < 0 || h.GetDistance() == dist )
		{
			hldofs = h;
			return true;
		}		
	}
	return false;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:  Finds a corridor that has a destination lane that is to 
//   the left of the destination lane of this corridor.
//
// Remarks: The function checks to make sure that the given corridor
//   and the resulting corridor have the same source road.
//
// Arguments:
//   intrsctn - Intersection in which to look for the corridor.
//   leftCrdr - (output) The corridor which has as its destination lane
//              the lane which is to the left of the given lane.
//
// Returns:  A boolean indicating if a corridor was found.
//
//////////////////////////////////////////////////////////////////////////////
bool
CCrdr::GetLeftCrdrAlongDir( CCrdr& targetCrdr ) const
{
	AssertValid();

	//
	// Find the lane to the left of the destination lane that's moving
	// in the same direction.
	//
	//cvTLane* pDstLn = BindLane( m_pCrdr->srcLnIdx );
	cvTLane* pDstLn = BindLane( m_pCrdr->dstLnIdx );
	if( !pDstLn )  return false;

	bool moreLanesToLeft;
	int dstLeftLaneNo = -1;

	bool dstLanePosDir = pDstLn->direction == ePOS;
	cvTLane* pDstLeftLane;
	if( dstLanePosDir )
	{
		// add 1 for positive direction lanes
		cvTRoad* pDstRd = BindRoad( m_pCrdr->dstRdIdx );
		if( !pDstRd )  return false;
		int numOfLanes = pDstRd->numOfLanes;
		moreLanesToLeft = !( pDstLn->laneNo == numOfLanes - 1 );
		if( moreLanesToLeft )
		{
			dstLeftLaneNo = pDstLn->laneNo + 1;
			pDstLeftLane = BindLane( m_pCrdr->dstLnIdx + 1 );
		}
	}
	else 
	{
		// subtract 1 for negative direction lanes
		moreLanesToLeft = ( pDstLn->laneNo != 0 );
		if( moreLanesToLeft )
		{
			dstLeftLaneNo = pDstLn->laneNo - 1;
			pDstLeftLane = BindLane( m_pCrdr->dstLnIdx - 1 );
		}
	}

	//
	// Make sure that the lane to the left is moving in the same
	// direction.
	//
//	cvTLane* pDstLeftLane;
	if( moreLanesToLeft )
	{
//		pDstLeftLane = BindLane( dstLeftLaneNo );
		if( !pDstLeftLane )  return false;
		bool leftMostAlongDir = pDstLn->direction != pDstLeftLane->direction;
		if( leftMostAlongDir )  return false;
	}
	else
	{
		return false;
	}



	//
	// Iterate through all corridors inside this intersection to find
	// a corridor that has the same source road and has a destination
	// lane that's the same as the lane to the left just calculated.
	//
	bool foundCrdr = false;
	cvTIntrsctn* pIntrsctn = BindIntrsctn( m_pCrdr->intrsctnId );
	TU32b i;
	for( i = 0; i < pIntrsctn->numOfCrdrs; i++ )
	{
		cvTCrdr* pCrdr = BindCrdr( pIntrsctn->crdrIdx + i );
		
		foundCrdr = (
			pCrdr->srcRdIdx == m_pCrdr->srcRdIdx &&
			pCrdr->dstLnIdx == pDstLeftLane->myId
			);

		if( foundCrdr )
		{
			CCrdr c( GetCved(), pCrdr );
			targetCrdr = c;
			break;
		}
		
	}

	return foundCrdr;
}  // end of GetLeftCrdrAlongDir


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetMrgDstFirst 
//  Find the distance(s) of the first(merge) intersected point(s) with other 
//  corridors
//
// Remarks: 
//
// Arguments:
//  dists - a vector of distances of the all the first intersected points with
//   other corridors 
//  dist - distance of the the first intersected point with the closest 
//   corridor
//  crdrId - relative id of the corridor that intersects with the current
//   corridor 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCrdr::GetMrgDstFirst(vector<double>& dists) const
{
	cvTHeader*  pH   = static_cast<cvTHeader*>  (GetInst());
	char*       pOfs = static_cast<char*> (GetInst()) + pH->crdrMrgDstOfs;
	cvTCrdrMrgDst* pEntry = 
				(reinterpret_cast<cvTCrdrMrgDst*>(pOfs)) + m_pCrdr->mrgDstIdx;

	int numCrdrs = GetIntrsctn().GetNumCrdrs();
	//pEntry += (numCrdrs * GetRelativeId());
	
	dists.clear();
	int i = 0;
	for(; i<numCrdrs; i++){
		dists.push_back(pEntry->firstDist);
		pEntry += 1;
	}
}

void
CCrdr::GetMrgDstFirst(double& dist, int& crdrId) const
{
	vector<double> dists;
	GetMrgDstFirst(dists);
	dist = -1;
	crdrId = -1;
	bool first = false;

	int size = (int) dists.size();
	int i = 0;
	for(; i<size; i++){
		if (dists[i] < 0){
			continue;
		}
		if (!first){
			dist = dists[i];
            crdrId = i;
			first = true;
			continue;
		}
		if (dists[i]<dist){
			dist = dists[i];
			crdrId = i;
		}
	}	
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetMrgDist 
//  This function return the first merging distance of any two corridors.
//
// Remarks: 
//
// Arguments:
//  crdrId - relative id of the corridor that intersects with the current
//   corridor 
//
// Returns: The merging distance. If there is no merging dist found, return -1.0.
//
//////////////////////////////////////////////////////////////////////////////
double 
CCrdr::GetFirstMrgDist( int crdrId ) const
{
	double mrgDist;
	/*static*/ vector<double> dists;
	dists.clear();
	dists.reserve(15);
	GetMrgDstFirst( dists );
	bool found = false;
	for( unsigned int i = 0; i < dists.size(); i++ )
	{
		if( i == crdrId )
		{
			found = true;
			mrgDist = dists[i];
			break;
		}
	}
	if( !found )
		return -1.0;
	else
		return mrgDist;

}	// end of GetMrgDist



//////////////////////////////////////////////////////////////////////////////
//
// Description: GetMrgDist 
//  This function return the last merging distance of any two corridors.
//
// Remarks: 
//
// Arguments:
//  crdrId - relative id of the corridor that intersects with the current
//   corridor 
//
// Returns: The merging distance. If there is no merging dist found, return -1.0.
//
//////////////////////////////////////////////////////////////////////////////
double 
CCrdr::GetLastMrgDist( int crdrId ) const
{
	double mrgDist;
	vector<double> dists;
	GetMrgDstLast( dists );
	bool found = false;
	for( unsigned int i = 0; i < dists.size(); i++ )
	{
		if( i == crdrId )
		{
			found = true;
			mrgDist = dists[i];
			break;
		}
	}
	if( !found )
		return -1.0;
	else
		return mrgDist;

}	// end of GetMrgDist



//////////////////////////////////////////////////////////////////////////////
//
// Description: GetMrgDstLast 
//  Find the distance(s) of the last(exit) intersected point(s) with other 
//  corridors
//
// Remarks: 
//
// Arguments:
//  dists - a vector of distances of the all the last(exit) intersected 
//   points with  other corridors 
//  dist - distance of the the last intersected point with the farthest 
//   corridor
//  crdrId - relative id of the corridor that intersects with the current
//   corridor 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CCrdr::GetMrgDstLast(vector<double>& dists) const
{
	cvTHeader*  pH   = static_cast<cvTHeader*>  (GetInst());
	char*       pOfs = static_cast<char*> (GetInst()) + pH->crdrMrgDstOfs;
	cvTCrdrMrgDst* pEntry = 
				(reinterpret_cast<cvTCrdrMrgDst*>(pOfs)) + m_pCrdr->mrgDstIdx;

	int numCrdrs = GetIntrsctn().GetNumCrdrs();
	//pEntry += (numCrdrs * GetRelativeId());
	
	dists.clear();
	int i = 0;
	for(; i<numCrdrs; i++){
		dists.push_back(pEntry->lastDist);
		pEntry += 1;
	}
}

void
CCrdr::GetMrgDstLast(double& dist, int& crdrId) const
{
	/*static*/ vector<double> dists;
	dists.clear();
	dists.reserve(15);
	GetMrgDstLast(dists);
	dist = -1;
	crdrId = -1;
	bool first = false;

	int size = (int) dists.size();
	int i = 0;
	for(; i<size; i++){
		if (dists[i] < 0){
			continue;
		}
		if (!first){
			dist = dists[i];
            crdrId = i;
			first = true;
			continue;
		}
		if (dists[i]>dist){
			dist = dists[i];
			crdrId = i;
		}
	}	
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: 
//	This function obtains all the corridors that intersect with the current
//	corridor instance.
//
// Remarks: 
//
// Arguments:
//	out - The output parameter that contains the desired intersecting corridors ids.
//		Note that the vector index matches the corridor ids.
// 
// Returns:	void
//////////////////////////////////////////////////////////////////////////////
void
CCrdr::GetIntersectingCrdrs( vector<int>& out ) const
{
	/*static*/ vector<double> dists;
	dists.clear();
	dists.reserve(15);
	this->GetMrgDstFirst( dists );
	for( unsigned int i = 0; i < dists.size(); i++ )
	{
		if( dists[i] != -1 )
		{
			out.push_back( i );
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function obtains an integer value indicating 
//   the priority of the calling corridor instance with 1 representing 
//   the highest priority. etc. 	
// 
// Remarks: 
//
// Arguments:
//	
// Returns: An enumeration indicating the priority of the current corridor.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr::ECrdrPriority 
CCrdr::GetCrdrPriority( void ) const
{
	AssertValid();
	
#ifdef DEBUG_GET_CRDR_PRIORITY
	CIntrsctn intrsctn( GetCved(), m_pCrdr->intrsctnId );
	gout << "=== GetCrdrPriority ================================" << endl;	
	gout << "crdr = " << m_pCrdr->myId;
	gout << "   intrsctn = " << intrsctn.GetName() << endl;
#endif

	//
	// The following numbers associated with the constants
	// indicate the priority of the corridor with 1 representing
	// the highest priority.
	//
//	const int cUNCONTROLLED = 1;
//	const int cLIGHT_GREEN = 2;
//	const int cYIELD = 3;
//	const int cLIGHT_FLASH_YELLOW = 3;
//	const int cLIGHT_YELLOW = 4;
//	const int cSTOP = 5;
//	const int cLIGHT_FLASH_RED = 5;
//	const int cLIGHT_RED = 6;
	
	/*static*/ vector<CHldOfs> hldofs;
	hldofs.clear();
	hldofs.reserve(30);
	GetAllHldOfs( hldofs );
	
#ifdef DEBUG_GET_CRDR_PRIORITY
	gout << "number of holdoffs = " << hldofs.size() << endl;
#endif

	//
	// Note:
	//  
	// This code needs to be modified to work with multiple hold offset.
	// The user will have to pass into the function their distance along
	// the corridor to enable this function to decide which is the next
	// corridor.
	//
	vector<CHldOfs>::iterator i;
	for( i = hldofs.begin(); i != hldofs.end(); i++ )
	{
		CHldOfs hldOfs = *i;
		
		if( !hldOfs.IsValid() )  continue;
		
		int hldOfsId = hldOfs.GetObjId();
		cvEObjType objType = GetCved().GetObjType( hldOfsId );

#ifdef DEBUG_GET_CRDR_PRIORITY
		string name = hldOfs.GetName();
		int solId = hldOfs.GetSolId();
		gout << "  id = " << hldOfsId;
		gout << "   reason = "<< hldOfs.GetReason() << "   solId = ";
		gout << solId << endl;
		gout << "  objtype = " << objType << endl;
#endif

		// Only interested in traffic sign holdoffs.
		if( objType == eCV_TRAFFIC_SIGN )
		{
		
#ifdef DEBUG_GET_CRDR_PRIORITY
//			string signName = GetCved().GetSol().QryNameByID( solId );
			gout << "  it is a stop sign" << endl;
//			gout << " The string is: " << signName << endl;
#endif
			//now lets get the name of the sign;
            string signName = hldOfs.GetName();
            int id;
            auto &cved = GetCved();
            if (cved.GetObj(signName,id)){
                int solId = cved.GetObjSolId(id);
                int solOption;
                if (cved.GetObjOption(id,solOption)){
                    vector<CSolOption> options = cved.GetSol().GetObj(solId)->GetOptions();
                    if ((unsigned int) solOption < options.size() && solOption >= 0 ){
                        string optionName = options[solOption].name;
                        std::transform(optionName.begin(), optionName.end(), optionName.begin(), ::tolower);
                        if (optionName == "off"){
                           return eUNCONTROLLED; 
                        }else{
                            return eSTOP;
                        }
                    }else{
                        return eSTOP;
                    }
                }else{
                    return eSTOP;
                }
            }else{
                return eSTOP;
            }

			//
			// For now assuming all are stop signs since the SOL doesn't
			// contain any yield signs.
			//
			return eSTOP;	
		}
		else if( objType == eCV_TRAFFIC_LIGHT )
		{
			eCVTrafficLightState lightState = 
								GetCved().GetTrafficLightState( hldOfsId );

#ifdef DEBUG_GET_CRDR_PRIORITY
			gout << "  it is a traffic light  state = ";
			gout << lightState << "  ";
#endif

			switch( lightState )
			{
			case eRED : 
				return eLIGHT_RED;
			case eGREEN :
				return eLIGHT_GREEN;
            case eFLASH_GREEN :
				return eLIGHT_FLASH_GREEN;
			case eYELLOW : 
				return eLIGHT_YELLOW;
			case eFLASH_YELLOW : 
				return eLIGHT_FLASH_YELLOW;
			case eFLASH_RED :
				return eLIGHT_FLASH_RED;
			case eRED_TURN_LEFT :
				return eLIGHT_RED_TURN_LEFT;
			case eYELLOW_TURN_LEFT :
				return eLIGHT_YELLOW_TURN_LEFT;
			case eGREEN_TURN_LEFT :
				return eLIGHT_GREEN_TURN_LEFT;
			case eRED_TURN_RIGHT :
				return eLIGHT_RED_TURN_RIGHT;
			case eYELLOW_TURN_RIGHT :
				return eLIGHT_YELLOW_TURN_RIGHT;
			case eGREEN_TURN_RIGHT :
				return eLIGHT_GREEN_TURN_RIGHT;
			case eFLASH_RED_TURN_LEFT :
				return eLIGHT_FLASH_RED_TURN_RIGHT;
			case eFLASH_YELLOW_TURN_LEFT :
				return eLIGHT_FLASH_YELLOW_TURN_LEFT;
			case eFLASH_GREEN_TURN_LEFT :
				return eLIGHT_FLASH_GREEN_TURN_LEFT;
			case eFLASH_RED_TURN_RIGHT :
				return eLIGHT_FLASH_RED_TURN_RIGHT;
			case eFLASH_YELLOW_TURN_RIGHT :
				return eLIGHT_FLASH_YELLOW_TURN_RIGHT;
			case eFLASH_GREEN_TURN_RIGHT :
				return eLIGHT_FLASH_GREEN_TURN_RIGHT;
			case eRED_STRAIGHT :
				return eLIGHT_RED_STRAIGHT;
			case eGREEN_STRAIGHT :
				return eLIGHT_GREEN_STRAIGHT;
			case eYELLOW_STRAIGHT :
				return eLIGHT_YELLOW_STRAIGHT;
			case eFLASH_RED_STRAIGHT :
				return eLIGHT_FLASH_RED_STRAIGHT;
			case eFLASH_YELLOW_STRAIGHT :
				return eLIGHT_FLASH_YELLOW_STRAIGHT;
			case eFLASH_GREEN_STRAIGHT :
				return eLIGHT_FLASH_GREEN_STRAIGHT;
			case eOFF :
				return eUNCONTROLLED;

			default :
//#ifdef DEBUG_GET_CRDR_PRIORITY
				gout << "UNKNOWN" << endl;;
//#endif

				char buf[1024];
				sprintf_s( buf, "unknown light state = %d", lightState );
				string msg = buf;
				cvCInternalError e( msg, __FILE__, __LINE__ );
				throw e;
				
				break;
			}
		}
	}

	//
	// If there is no traffic sign control, then treat it like an 
	// uncontrolled corridor.
	//
#ifdef DEBUG_GET_CRDR_PRIORITY
	gout << "  it is a uncontrolled corridor" << endl;
#endif
	
	return eUNCONTROLLED;
}	// end of GetCrdrPriority


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function determines whether the current corridor 
//   is going straight, turning left or right.	
// 
// Remarks: Throws an exception if the corridor is invalid.
//
// Arguments:
//	
// Returns: eLEFT to indicate the current corridor is turning to the
//   left; eSTRAIGHT to indicate the current corridor is going 
//   straight; and eRIGHT to indicate the current corridor is turning 
//   to the right.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr::ECrdrDirection
CCrdr::GetCrdrDirection( void ) const
{
	AssertValid();

	if ( m_pCrdr->direction == cCV_CRDR_LEFT_DIR ) {
		return eLEFT;
	}
	else if ( m_pCrdr->direction == cCV_CRDR_STRAIGHT_DIR ) {
		return eSTRAIGHT;
	}
	else if ( m_pCrdr->direction == cCV_CRDR_RIGHT_DIR) {
		return eRIGHT;
	}
	else {
		assert( 0 );
		return eSTRAIGHT;	/* to avoid warning about no return value */
	}
}	// end of GetCrdrDirection


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function returns the corridor's direction k.
// 
// Remarks: Throws an exception if the corridor is invalid.
//
// Arguments:
//	
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
double
CCrdr::GetCrdrDirectionK( void ) const
{
	AssertValid();
	return m_pCrdr->directionK;
}	// end of GetCrdrDirectionK


//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns a string that indicates the direction of the
//   corridor.
// 
// Remarks: 
//
// Arguments:
//   cDirection - The corridor's direction.
//	
// Returns: A string that represents the direction of the corridor.
//   The string is either "left", "right", "straight", or "unknown".
//
//////////////////////////////////////////////////////////////////////////////
string
CCrdr::DirectionToStr( const ECrdrDirection cDirection ) const
{
	string dirStr;
	
	if( cDirection == eLEFT )
	{
		dirStr = "left";
	}
	else if( cDirection == eRIGHT )
	{
		dirStr = "right";
	}
	else if( cDirection == eSTRAIGHT )
	{
		dirStr = "straight";
	}
	else
	{
		dirStr = "unknown";
	}

	return dirStr;
}	// end of DirectionToStr



//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns a string that indicates the direction of this
//   corridor.
// 
// Remarks: 
//
// Arguments:
//	
// Returns: A string that represents the direction of the corridor.
//   The string is either "left", "right", "straight", or "unknown".
//
//////////////////////////////////////////////////////////////////////////////
string
CCrdr::GetCrdrDirectionStr()
{
	ECrdrDirection m_direction = GetCrdrDirection();

	string dirStr;
	
	if( m_direction == eLEFT )
	{
		dirStr = "left";
	}
	else if( m_direction == eRIGHT )
	{
		dirStr = "right";
	}
	else if( m_direction == eSTRAIGHT )
	{
		dirStr = "straight";
	}
	else
	{
		dirStr = "unknown";
	}

	return dirStr;
}	// end of GetCrdrDirectionStr



//////////////////////////////////////////////////////////////////////////////
//
// Description: This function determines whether the current corridor 
//   is to the right or to the left of the corridor geometrically. 
//   Sometimes it may not be neither to the left nor to the right.
//
// Remarks: 
//
// Arguments:
//
//	otherCrdr - The corridor to compare this corridor to.
//
// Returns: eNONE to indicate no result is returned; eTOTHELEFT to indicate
//	 the current corridor is on the left of the compared corridor in 
//	 the parameter; eTOTHERIGHT to indicate the current corridor is on 
//	 the right of the compared corridor in the parameter; eNOTLEFTNORRIGHT 
//	 to indicate that the current corridor is neither on the left nor on
//	 the right of the corridor in the parameter.
//
//////////////////////////////////////////////////////////////////////////////
CCrdr::ECrdrGeometry
CCrdr::DecideCrdrByGeometry( const CCrdr& otherCrdr ) const
{
	AssertValid();

	if( otherCrdr.IsValid() ) {
		cvTIntrsctn*  pInt       = BindIntrsctn(m_pCrdr->intrsctnId);
		int           idRelToInt = otherCrdr.m_pCrdr->myId - pInt->crdrIdx;

		assert(idRelToInt >= 0);
		assert(idRelToInt < (int)pInt->numOfCrdrs);

		switch ( m_pCrdr->dirRel[idRelToInt] ) {
			case cCV_CRDR_LEFT_TO     : return eTOTHELEFT;
			case cCV_CRDR_PARALLEL_TO : return eNOTLEFTNORRIGHT;
			case cCV_CRDR_RIGHT_TO    : return eTOTHERIGHT;
			default                   : assert(0);
		}
	}

	return eNONE;
}  // end if DecideCrdrByGeometry


//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns a string that indicates the geometry of the
//   corridor.
// 
// Remarks: 
//
// Arguments:
//   cGeometry - The corridor's geometry.
//	
// Returns: A string that represents the geometry of the corridor.
//   The string is either "to the left", "to the right", 
//   "not left nor right", or "unknown".
//
//////////////////////////////////////////////////////////////////////////////
string
CCrdr::GeometryToStr( const ECrdrGeometry cGeometry ) const
{
	string geomStr;
	
	if( cGeometry == eTOTHELEFT )
	{
		geomStr = "to the left";
	}
	else if( cGeometry == eTOTHERIGHT )
	{
		geomStr = "to the right";
	}
	else if( cGeometry == eNOTLEFTNORRIGHT )
	{
		geomStr = "not left nor right";
	}
	else
	{
		geomStr = "unknown";
	}

	return geomStr;
}	// end of GeometryToStr



//////////////////////////////////////////////////////////////////////////////////
// Description: QryAttr
//  Collect all attributes contained in the current Crdrs
//
// Arguments:
//  attrs - a vector of CAttr 
//
////////////////////////////////////////////////////////////////////////////////
void
CCrdr::QryAttr(vector<CAttr>& attrs) const
{
	AssertValid();
	attrs.clear();

    cvTHeader   *pH   = static_cast<cvTHeader*>  (GetInst());
    char        *pOfs = static_cast<char*>       (GetInst()) +
                        pH->attrOfs;
    cvTAttr     *pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +
                        m_pCrdr->attrIdx;
	int i;
	for (i = 0; i < m_pCrdr->numAttr; i++, pAttr++) {
		// take out those reserved for attributes created at runtime.
		if (pAttr->myId == 0)
			continue;
		attrs.push_back(CAttr(GetCved(), pAttr));
	}

}


//////////////////////////////////////////////////////////////////////////////
//
// Description: This function gets the width of the corridor.
// 
// Remarks: Throws an exception if the input distance is a negative
//  value or is greater than the length of the current corridor instance.
//
// Arguments:
//   distance - The obj distance along the corridor.
//	
// Returns: A double value representing the width of the corridor.
//
//////////////////////////////////////////////////////////////////////////////
double 
CCrdr::GetWidth( double distance ) const
{
	AssertValid();

	double crdrLength = GetLength(); 
	assert ( distance >= 0 && distance <= crdrLength );

	double width;

	// Get all the control points on the corridor.
	vector<TCntrlPnt> cntrlPnts;
	GetCntrlPnts( cntrlPnts );
	int size = (int) cntrlPnts.size();

	for( int i = 0; i < size; i++ )
	{
		if( distance == 0 )	return cntrlPnts[0].width;	

		if ( distance == crdrLength )	return cntrlPnts[size - 1].width;
	
		if( cntrlPnts[i].distance == distance ) return cntrlPnts[i].width;
		
		if( 
			cntrlPnts[i].distance > distance && 
			cntrlPnts[i - 1].distance < distance
			)
		{
			// Interpolate the desired width.
			double d1 = cntrlPnts[i - 1].distance;
			double w1 = cntrlPnts[i - 1].width;

			double d2 = cntrlPnts[i].distance;
			double w2 = cntrlPnts[i].width;

			double r1 = ( d2 - distance ) / ( d2 - d1 );
			double r2 = ( distance - d1 ) / ( d2 - d1 );

#ifdef DEBUG_GET_WIDTH
			gout << " d1 = " << d1;
			gout << " d2 = " << d2 << endl;
			gout << " w1 = " << w1;
			gout << " w2 = " << w2 << endl;
			gout << " r1 = " << r1;
			gout << " r2 = " << r2 << endl;

#endif		
			width = w1 * r1 + w2 * r2;
			return width;
		}

	}	// for
	return -1;
}	// end of Getwidth


} // namespace CVED

