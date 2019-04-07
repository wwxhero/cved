//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: road.cxx,v 1.52 2018/12/07 21:13:32 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	The implementation of the CRoad class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

namespace CVED {

//////////////////////////////////////////////////////////////////////////////
///\brief operator<<
///
///\par Description: operator<<
/// 	Prints the contents of the cRoad parameter to the ostream parameter.
///
///\remark This operator has to appear within the namespace else it won't
/// 	be properly associated.
///
///\param 	out   - a reference to an ostream instance
///\param	cRoad - a const reference to a CRoad instance
///
///\return	The ostream instance for nested operator<< calls.
///
//////////////////////////////////////////////////////////////////////////////
ostream& 
operator<<(ostream &out, const CRoad &cRoad)
{
	out << cRoad.GetName();
	return out;
} // end of operator<<

} // end of namespace CVED

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//	CRoadPiece constructors
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
///\brief The constructor initializes a roadpiece
///
///\par Description: CRoadPiece
/// 	This constructor takes a CCved instance, a roadId, (x1, y1), (x2, y2), 
/// 	the beginning control point and the end control point from which to 
/// 	construct the CRoadPiece.
///
///\remark 
///		The constructor initializes a roadpiece.
///
/// 
///\param	cCved	  - reference to a CCved instance
///\param	roadId	  - road id
///\param	x1	      - lowerleft x
///\param	y1		  - lowerleft y
///\param	x2		  - upperleft x
///\param	y2		  - upperleft y
///\param	beginning - index of the first control point being bound by the roadpiece
///\param	end		  - index of the last control point being bound by the roadpiece
///
///\return	void
///
//////////////////////////////////////////////////////////////////////////////
CRoadPiece::CRoadPiece(	
			const CCved&	cCved,
			TRoadPoolIdx	roadId,
			double			x1,
			double			y1,
			double			x2,
			double			y2,
			int				beginning,
			int				end) 
	: CCvedItem(&cCved)
{
    m_road  = roadId;
    m_x1    = x1;
    m_y1    = y1;
    m_x2    = x2;
    m_y2    = y2;
	m_first	= static_cast<TLongCntrlPntPoolIdx>(beginning);
	m_last	= static_cast<TLongCntrlPntPoolIdx>(end);
} // end of CRoadPiece

//////////////////////////////////////////////////////////////////////////////
///\brief constructor takes a CCved instance, and a roadpiece id
///
///\par Description: CRoadPiece
/// 	This constructor takes a CCved instance, and a roadpiece id.
///
///\remark 
///		The constructor initializes a roadpiece.
///
///\param  cCved	- reference to a CCved instance
///\param  id 		- road piece identifier
///
///\return void
///
//////////////////////////////////////////////////////////////////////////////
CRoadPiece::CRoadPiece(const CCved&	cCved, TRoadPiecePoolIdx id) 
	: CCvedItem(&cCved)
{
	cvTHeader *pH   = static_cast<cvTHeader*>  (GetInst());
	char      *pOfs = static_cast<char*>       (GetInst()) + pH->roadPieceOfs;
	cvTRoadPiece *pRP = (reinterpret_cast<cvTRoadPiece*>(pOfs)) + id;

	m_road	= pRP->roadId;
	m_x1	= pRP->x1;
	m_x2	= pRP->x2;
	m_y1	= pRP->y1;
	m_y2	= pRP->y2;
	m_first	= pRP->first;
	m_last	= pRP->last;
} // end of CRoadPiece


//////////////////////////////////////////////////////////////////////////////
//	CRoadPiece accessors
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
///\brief bounding box of the current CRoadPiece
///
///\par Description: GetBoundingBox 
/// 	Return the bounding box of the current CRoadPiece
///
///\remark
///
///\return	an object of type CBoundBox2D
///
//////////////////////////////////////////////////////////////////////////////
CBoundingBox
CRoadPiece::GetBoundingBox(void) const
{
	CBoundingBox	bound(m_x1, m_y1, m_x2, m_y2);
	return	bound;
} // end of GetBoundingBox


//////////////////////////////////////////////////////////////////////////////
//	CRoad constructors
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
///\brief default constructor
///
///\par Description: CRoad
/// 	The default constructor initializes the object to an unbound and invalid 
/// 	state.
///
///\remark
/// 
///\return void
///
//////////////////////////////////////////////////////////////////////////////
CRoad::CRoad() 
	: CCvedItem(0), m_pRoad(0)
{} // end of CRoad

//////////////////////////////////////////////////////////////////////////////
///\brief Creates a bound and valid CRoad instance
///
///\par Description: 
///		This constructor takes a CCved reference and a road 
///		identifier.
///
///\remark 
///		Creates a bound and valid CRoad instance.
///
///\param 	 cCved - reference to a CCved instance
///\param 	 id	   - the road identifier
///
///\return	 void
///
//////////////////////////////////////////////////////////////////////////////
CRoad::CRoad( const CCved& cCved, TU32b id ) 
	: CCvedItem( &cCved )
{
	cvTHeader *pH   = static_cast<cvTHeader*>  ( GetInst() );
	char      *pOfs = static_cast<char*>       ( GetInst() ) + pH->roadOfs;

	// 
	// Make sure the id is within bounds.
	//
	if( id >= (int)pH->roadCount )
	{
        stringstream ss;
		ss<<"No road with id = "<<id;
		cvCInternalError e( ss.str(), __FILE__, __LINE__ );
		throw e;
	}

	m_pRoad = ( reinterpret_cast<TRoad*>( pOfs ) ) + id;
} // end of CRoad

//////////////////////////////////////////////////////////////////////////////
///\brief Creates a bound and valid CRoad instance
///
///\par Description: 
///		This constructor takes a CCved reference and a road name.
///
///\par Remarks: 
///		Creates a bound and valid CRoad instance.
///
///\param	cCved	  - Reference to a CCved instance.
///\param	cRoadName - The road's name.
///
///\return	void
///
//////////////////////////////////////////////////////////////////////////////
CRoad::CRoad(const CCved& cCved, const string& cRoadName)
	: CCvedItem(&cCved)
{
	THeader* pH        = static_cast<THeader*> ( GetInst() );
	char*    pCharPool = static_cast<char*>    ( GetInst() ) + pH->charOfs;

	unsigned int i;
	for( i = 1; i < pH->roadCount; i++ )
	{
		m_pRoad = BindRoad( i );
		if( cRoadName == ( pCharPool + m_pRoad->nameIdx) )  break;
	}

	//
	// Unable to find the road with the same name.
	//
	if( i == pH->roadCount )
	{
		string msg( " No road with name: " );
		msg += cRoadName;
		cvCInternalError e( msg, __FILE__, __LINE__ );
		throw e;
	}
} // end of CRoad

//////////////////////////////////////////////////////////////////////////////
///\brief default destructor
///
///\par Description: ~CRoad
/// 	The default destructor does nothing.
///
///\remark
///
///\return void
///
//////////////////////////////////////////////////////////////////////////////
CRoad::~CRoad()
{} // end of ~CRoad

//////////////////////////////////////////////////////////////////////////////
///\brief deep copy
///
///\par Description: operator=
/// 	Performs a deep copy from the parameter to the current instance.
///
///\remark
///
///\param  cRhs - a reference to an object intended to be to the right of the =
///
///\return a reference to the current instance so that the assignments can
///			be nested.
///
//////////////////////////////////////////////////////////////////////////////
CRoad&
CRoad::operator=(const CRoad &cRhs)
{
	if (this != &cRhs)  {
		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pRoad = cRhs.m_pRoad;
	}

	return *this;
} // end of operator=


//////////////////////////////////////////////////////////////////////////////
//	CRoad accessors
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
///\brief Allows access to the raw control points
///
///\par Description: 
/// 	Allows access to the raw control points associated with a road
///
///\remark 
///		This function allows the user to obtain information about
///		the raw control points that comprise a road.  In general, there should
///		be no need to get the raw control points, as there is a lot of
///		processing and interpretation necessary to get any usefulness out of
///		these points, however this function is there to provide them just in case
///		they are needed, also for debugging.
/// 
///\param	first - the index of the first point to obtain, relative to the
///					road use -1 to indicate starting with the first point
///\param	last  - the index of the last point to get, use -1 to indicate the
///					last point
///\param	data  - where to store the information about the control points
///
///\return  True if all was ok, otherwise it returns false.
///
//////////////////////////////////////////////////////////////////////////////
bool
CRoad::GetCntrlPoint(int first, int last, vector<CtrlInfo> &data) const
{
	AssertValid();

	cvTCntrlPnt *pPts;
	int          i, j;

	if ( first < 0 ) first = 0;
	if ( last < 0 )  last = m_pRoad->numCntrlPnt-1;

	if ( first >= last )                       return false;
	if ( last-first > m_pRoad->numCntrlPnt-1 ) return false;

	pPts = BindCntrlPnt(m_pRoad->cntrlPntIdx + first);

	data.clear();
	for (i=0; i<last-first+1; i++) {
		CtrlInfo  info;

		info.pos = pPts->location;
		info.norm = pPts->normal;

		for (j=0; j<3; j++) {
			info.spline[0][j] = pPts->hermite[j].A;
			info.spline[1][j] = pPts->hermite[j].B;
			info.spline[2][j] = pPts->hermite[j].C;
			info.spline[3][j] = pPts->hermite[j].D;
		}

		info.cummLinDist = pPts->cummulativeLinDist;

		data.push_back(info);
		pPts++;
	}

	return true;
}


//////////////////////////////////////////////////////////////////////////////
///\brief Return the name of the current road
///
///\par Description: GetName
/// 	Return the name of the current road
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///
///\return An STL string containing the road's name.
///
//////////////////////////////////////////////////////////////////////////////
string 
CRoad::GetName(void) const
{
	AssertValid();

	char *pCharPool;

	cvTHeader *pH = static_cast<cvTHeader *> (GetInst());
	pCharPool     = static_cast<char *> (GetInst()) + pH->charOfs;
	string s(pCharPool + m_pRoad->nameIdx);

	return s;
} // end of GetName
  //////////////////////////////////////////////////////////////////////////////
  ///\brief Return the left most lane relative to the given lane
  ///
  ///\param[in] targLane
  /// 	Lane to start searching from
  ///
  ///\remark 
  ///		This function will start at the target lane, and keep looking
  ///       left until it either runs out of lanes, or the lane to the left
  ///       changes directions. If the lane the supplied lane is the left most,
  ///       this will return a copy of the lane.
  ///
  ///\return lane that is the left most of the current lane.
  ///
  //////////////////////////////////////////////////////////////////////////////
CLane       
CRoad::GetLeftMostLaneInSameDir(const CLane& targLane) const {
    CLane curLane(targLane);
    if (GetNumLanes() == 1)
        return curLane;
    while (true) {
        if (curLane.IsLeftMostAlongDir()) {
            return curLane;
        }
        try {
            curLane = curLane.GetLeft();
        }catch(cvCInternalError e){
            return CLane();
        }
    }
    return CLane();
}
//////////////////////////////////////////////////////////////////////////////
///\brief Return the right most lane relative to the given lane
///
///\param[in] targLane
/// 	Lane to start searching from
///
///\remark 
///		This function will start at the target lane, and keep looking
///       right until it either runs out of lanes, or the lane to the right
///       changes directions. If the lane the supplied lane is the right most,
///       this will return a copy of the lane.
///
///\return lane that is the right most of the current lane.
///
//////////////////////////////////////////////////////////////////////////////
CLane  
CRoad::GetRightMostLaneInSameDir(const CLane& targLane) const {
    CLane curLane(targLane);
    if (GetNumLanes() == 1)
        return curLane;
    while (true) {
        if (curLane.IsRightMostAlongDir()) {
            return curLane;
        }
        try {
            curLane = curLane.GetRight();
        }catch(cvCInternalError e){
            return CLane();
        }
    }
    return CLane();
}
//////////////////////////////////////////////////////////////////////////////
///\brief Return length of the current road
///
///\par Description: GetLinearLength
/// 	Return length of the current road using linear interpolation
///
///\remark  
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the length of a road measured as the sum of
/// 	the linear distances among successive pairs of control points.
/// 
///\return a double containing the linear length of the road.
///
//////////////////////////////////////////////////////////////////////////////
double 
CRoad::GetLinearLength(void) const
{
	AssertValid();		// will throw an exception if object not ready
	return m_pRoad->roadLengthLinear;
} // end of GetLinearLength

//////////////////////////////////////////////////////////////////////////////
///\brief Return length of road using cubic interpolation
///
///\par Description: GetCubicLength
/// 	Return length of road using cubic interpolation
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the length of a road measured as the sum of the 
/// 	cubic distances among successive pairs of control points.  The cubic 
/// 	distance is the distance along the hermite spline defined between 
/// 	control points and provides C2 continuity.
///
///\return a double containing the cubic length of the road.
///
//////////////////////////////////////////////////////////////////////////////
double 
CRoad::GetCubicLength(void) const
{
	AssertValid();
	return m_pRoad->roadLengthSpline;
} // end of GetCubicLength

//////////////////////////////////////////////////////////////////////////////
///\brief Return number of lanes 
///
///\par Description: GetNumLanes
/// 	Return number of lanes on the current road.
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	A road has at least one lane.
///
///\return The number of lanes on the road ( >=1 )  
///
//////////////////////////////////////////////////////////////////////////////
int   
CRoad::GetNumLanes(void) const
{
	AssertValid();
	return m_pRoad->numOfLanes;
} // end of GetNumLanes

//////////////////////////////////////////////////////////////////////////////
///\brief Return the index to the first lane
///
///\par Description: GetLaneIdx
/// 	Return the index to the first lane on this road
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
///		This function returns the index to the first lane so that when accessing
/// 	lane information we do not have to go to "road pool > contrl point pool"
///		to get the lane information
///
///\return The index to the first lane on this road    
///
//////////////////////////////////////////////////////////////////////////////
int
CRoad::GetLaneIdx(void) const
{
	AssertValid();
	return m_pRoad->laneIdx;
} // end of GetLaneIdx

//////////////////////////////////////////////////////////////////////////////
///\brief Return the internal id of a road
///
///\par Description: GetId
/// 	Return the internal id of a road
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
///		This function returns a unique integer identifier associated with
///		the road.  The use of the integer id is discouraged as it may
/// 	change in virtual environments after the addition or deletion
/// 	of roads.  For example, if LRI file A is a superset of LRI file
/// 	B, there is no guarantee that the roads common to A and B will
/// 	have the same identifiers when loaded in CVED.  Use of the road's
/// 	name provides a somewhat less efficient, yet deterministic way
/// 	of identifying roads.
///
///\return The integer identifier of the road.  A value of 0 indicates that
///	the road is not valid.
//
//////////////////////////////////////////////////////////////////////////////
int   
CRoad::GetId(void) const
{
	AssertValid();
	return m_pRoad->myId;
} // end of GetId

//////////////////////////////////////////////////////////////////////////////
///\brief Return the number of control points
///
///\par Description: GetCntrlPntCount
/// 	Return the number of control points in a road curve
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the count of control points that comprise
/// 	the road's centerline.
///
///\return The number of control points that comprise the road centerline.
///
//////////////////////////////////////////////////////////////////////////////
int   
CRoad::GetCntrlPntCount(void) const
{
	AssertValid();
	return m_pRoad->numCntrlPnt;
} // end of GetCntrlPntCount

//////////////////////////////////////////////////////////////////////////////
///\brief Return the index to the logitudinal control point
///
///\par Description: GetCntrlPntIdx
/// 	Return the index to the logitudinal control point
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the index to the first longitudinal control point
/// 	that comprise the road's centreline.
///
///\return The index to the first control points on the road
///
//////////////////////////////////////////////////////////////////////////////
int  
CRoad::GetCntrlPntIdx(void) const
{
	AssertValid();
	return m_pRoad->cntrlPntIdx;
} // end of GetCntrlPntIdx

//////////////////////////////////////////////////////////////////////////////
///\brief Return the source intersection id
///
///\par Description: GetSourceIntrsctnIdx
/// 	Return the source intersection identifier.
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the intersection that originates the road.
/// 	Each road has a single source and a single destination intersection.
///
///\return An integer representing the id of the source intersection.
///
//////////////////////////////////////////////////////////////////////////////
int
CRoad::GetSourceIntrsctnIdx(void) const
{
	AssertValid();
	return static_cast<int>(m_pRoad->srcIntrsctnIdx);
} // end of GetSourceIntrsctnIdx

//////////////////////////////////////////////////////////////////////////////
///\brief Return the destination intersection id
///
///\par Description: GetDestIntrscntIdx
/// 	Return the destination intersection identifier.
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the intersection that the roads leads to.
///		Each road has a single source and a single destination intersection.
///
///\return An integer representing the id of the destination intersection.
///
//////////////////////////////////////////////////////////////////////////////
int
CRoad::GetDestIntrsctnIdx(void) const
{
	AssertValid();
	return static_cast<int>(m_pRoad->dstIntrsctnIdx);
} // end of GetDestIntrsctnIdx

//////////////////////////////////////////////////////////////////////////////
///\brief GetSourceIntrsctn
///
///\par Description: GetSourceIntrsctn
/// 	Return the source intersection.
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the intersection that originates the road.
/// 	Each road has a single source and a single destination intersection.
///
///\return A CIntrsctn representing the source intersection.
///
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CRoad::GetSourceIntrsctn(void) const
{
	AssertValid();
	return CIntrsctn(GetCved(), m_pRoad->srcIntrsctnIdx);
} // end of GetSourceIntrsctn

//////////////////////////////////////////////////////////////////////////////
///\brief Return the destination intersection
///
///\par Description: GetDestIntrscntn
/// 	Return the destination intersection.
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function returns the intersection that the roads leads to.
///		Each road has a single source and a single destination intersection.
///
///\return A CIntrsctn representing the destination intersection.
///
//////////////////////////////////////////////////////////////////////////////
CIntrsctn
CRoad::GetDestIntrsctn(void) const
{
	AssertValid();
	return CIntrsctn(GetCved(), m_pRoad->dstIntrsctnIdx);
} // end of GetDestIntrsctn

//////////////////////////////////////////////////////////////////////////////
///\brief Returns a CRoadPos that lies at the beginning
///
///\par Description: Begin
/// 	Returns a CRoadPos that lies at the beginning of the current road.
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	If there is a CLane passed to this function that does not lie on the
/// 	current road, an invalid CRoadPos is returned.
///
///\param	cLane - optional parameter; a valid CLane instance which lies on the
///					current road.
///
///\return  a CRoadPos instance with its fields set like this:
///			road - current road instance
///			lane - to the optional lane parameter or to the first lane on road
///			distance - 0.0
///			offset - 0.0
///\sa End
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CRoad::Begin(const CLane* cLane) {

	AssertValid();

	// If the user didn't pass a lane parameter, 
	// 	then use the first lane on the road.
	if (!cLane) {

		CLane l(*this, 0);

		CRoadPos roadPos(*this, l, 0.0, 0.0);
		return roadPos;
	}
	// If the user did pass a lane parameter, and it lies on
	// 	the current road, then use it.
	else {

		TU8b laneId = /*cLane->GetRelativeId();*/cLane->GetId(); //GetRID_
		if ( (laneId >= m_pRoad->laneIdx) &&
			 (laneId < m_pRoad->laneIdx + m_pRoad->numOfLanes) ) {

			CRoadPos roadPos(*this, *cLane, 0.0, 0.0);
			return roadPos;
		}
		else {

			CRoadPos roadPos;
			return roadPos;
		}
	}
} // end of Begin

//////////////////////////////////////////////////////////////////////////////
///\brief Returns a CRoadPos that lies at the end
///
///\par Description: End
/// 	Returns a CRoadPos that lies at the end of the current road.
///
///\remarks 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	If there is a CLane passed to this function that does not lie on the
/// 	current road, an invalid CRoadPos is returned.
///
///\param	cLane - optional parameter; a valid CLane instance which lies on the
///			        current road.
///
///\return	a CRoadPos instance with its fields set like this:
///			road - current road instance
///			lane - to the optional lane parameter or to the first lane on road
///			distance - end of road
///			offset - 0.0
///\sa Begin
//////////////////////////////////////////////////////////////////////////////
CRoadPos
CRoad::End(const CLane* cLane) {

	AssertValid();
	double end = m_pRoad->roadLengthLinear;

	// If the user didn't pass a lane parameter, 
	// 	then use the first lane on the road.
	if (!cLane) {

		CLane l(*this, 0);
		CRoadPos roadPos(*this, l, end, 0.0);
		return roadPos;
	}
	// If the user did pass a lane parameter, and it lies on
	// 	the current road, then use it.
	else {

		TU8b laneId = /*cLane->GetRelativeId();*/ cLane->GetId(); //GetRID_
		if ( (laneId >= m_pRoad->laneIdx) &&
			 (laneId < m_pRoad->laneIdx + m_pRoad->numOfLanes) ) {

			CRoadPos roadPos(*this, *cLane, end, 0.0);
			return roadPos;
		}
		else {

			CRoadPos roadPos;
			return roadPos;
		}
	}
} // end of End

//////////////////////////////////////////////////////////////////////////////
///\brief Determine if an attribute applies on the part of a road
///
///\par Description: 
///		Determine if an attribute applies on the part of a road.
///
///\remarks 
///		This function will cause a failed assertion if it is called on 
///		an invalid CRoad instance.
///\par 
/// 	This function determines if the attribute with identifier id applies on 
/// 	the current road at the point of the road located dist units from its 
/// 	start.
///
///\param 	id   - The attribute identifier.
///\param 	dist - The distance along the road to check for the attribute.
///
///\return	true if the attribute applies, false otherwise.
///
//////////////////////////////////////////////////////////////////////////////
bool 
CRoad::ActiveAttr( int id, double dist ) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pRoad->attrIdx;

	// For each attribute on the road
	int i;
	for( i = 0; i < m_pRoad->numAttr; i++, pAttr++ )
	{
		// If the attr ids match ...
		bool match = pAttr->myId == id;
		if( match )
		{
			// and dist is in the range of
			//	the attribute ...
			bool distInRangeOfAttr = (
					( dist < 0 || pAttr->from < 0 || dist > pAttr->from ) &&
					( dist < 0 || pAttr->to < 0 || dist < pAttr->to )
					);
			if( distInRangeOfAttr )  return true;
		}

	} // For each attribute on the road

	// No matching attribute found
	return false;
} // end of ActiveAttr

//////////////////////////////////////////////////////////////////////////////
///\brief Obtains the CAttr that has the given id
///
///\par Description: 
///		Obtains the CAttr that has the given id and is on the 
///		current road, the given lanes, and the given distance.
///
///\remark 
///		This function will cause a failed assertion if it is called 
///		on an invalid CRoad instance.
/// 
/// 
///\param	id    - Identifier of an attribute type.
///\param	attr  - Output parameter that contains the desired attribute, if the 
///					function returns true.
///\param	laneMask- The lane mask.
///\param	dist  - Distance along the current road to search for attributes.
///
///\return  If an attribute is found that meets the given criteria, the 
///			attribute is placed in attr and the function returns true.  Otherwise, 
///			the function returns false.
///
//////////////////////////////////////////////////////////////////////////////
bool 
CRoad::QryAttr( int id, CAttr& attr, int laneMask, double dist) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pRoad->attrIdx;


	// For each attribute on the road
	int i;
	for( i = 0; i < m_pRoad->numAttr; i++, pAttr++ )
	{
		// take out those reserved for attributes created at runtime.
		if( pAttr->myId == 0 )  continue;

		// If the attr ids match ...
		if( pAttr->myId == id )
		{
			// and the lanes overlap ...
			if( pAttr->laneMask & laneMask )
			{
				// and dist is in the range of
				//	the attribute ...
				bool distInRangeOfAttr = (
					( dist < 0 || pAttr->from < 0 || dist > pAttr->from ) &&
					( dist < 0 || pAttr->to < 0 || dist < pAttr->to )
					);

				if( distInRangeOfAttr )
				{
					// Return the current attribute
					attr = CAttr( GetCved(), pAttr );
					return true;
				}
			}
		}
	} // For each attribute on the road

	// No matching attribute found
	return false;
} // end of QryAttr

//////////////////////////////////////////////////////////////////////////////
///\brief Obtain all attributes applicable on the road.
///
///\par Description: 
///		Obtain all attributes applicable on the road.
///
///\remark 
///		This function will cause a failed assertion if it is called 
///		on an invalid CRoad instance.
///\par 
/// 	This function returns all attributes that are applicable either to a 
/// 	specific point, or to a range of the road.
///
///\param 	d1  - A range on the road to collect attributes.
///\param   d2  - A range on the road to collect attributes.
///\param 	out - An STL vector to hold the attributes.
///
///\return true if at least one attribute is found on the specified 
///		   location or range.
///
//////////////////////////////////////////////////////////////////////////////
bool 
CRoad::QryAttr( vector<CAttr>& out, double d1, double d2 ) const
{
	AssertValid();

	cvTHeader* pH    = static_cast<cvTHeader*>  ( GetInst() );
	char*      pOfs  = static_cast<char*>       ( GetInst() ) + pH->attrOfs;
	cvTAttr*   pAttr = (reinterpret_cast<cvTAttr*>(pOfs)) +	m_pRoad->attrIdx;

	// For each attribute on the road
	int i;
	int cntr = 0;
	for( i = 0; i < m_pRoad->numAttr; i++, pAttr++ )
	{
		// take out those reserved for attributes created at runtime.
		if( pAttr->myId == 0 )  continue;

		// If the distance ranges overlap 
		bool distInRangeOfAttr = (
			( d1 < 0 || pAttr->from < 0 || d1 > pAttr->from ) &&
			( d2 < 0 || pAttr->to < 0 || d2 < pAttr->to )
			);

		if( distInRangeOfAttr )
		{
			// Add the current attribute
			CAttr attr( GetCved(), pAttr );
			out.push_back( attr );
			cntr++;
		}

	} // For each attribute on the road

	return cntr > 0;
} // end of QryAttr

//////////////////////////////////////////////////////////////////////////////
///\brief Break up a road in pieces of a bounded size
///
///\par Description: BreakUp
/// 	Break up a road in pieces of a bounded size
///
///\remark 
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function breaks up a road into road pieces so that the bounding box 
/// 	of each road piece is a bit larger than the specified limit. 
///\par
/// 	The bounding box of a road piece includes the edges of a road (not
/// 	only the centerline).  The edge of a road is determined based
/// 	on the physical width of the road so that if a point is not
/// 	within the bounding box of a road piece, then it cannot overlap
/// 	the road segment represented by the corresponding road piece.
///\par
/// 	Breaking up a road in road pieces is useful for minimizing search
/// 	times involving roads.
///\par
/// 	The function will attempt to separate the road into as many 
/// 	road pieces as necessary to ensure that each of the pieces
/// 	will extend beyond one control point bigger than the specified limits.  
/// 	However, it is possible to end up with smaller piece for the last
/// 	one 
///
/// 
///\param 	out    - an STL vector that contains the road pieces in which there're two
///					 indeces to the first and last control points in the roadpiece, and
///					 the last will overlap with the first in the next road piece
///\param 	maxwid - the maximum width (x axis) of each road piece
///\param 	maxhei - the maximum height (y axis) of each road piece
/// 
///\return void
///
//////////////////////////////////////////////////////////////////////////////
void 
CRoad::BreakUp(TRoadPieceVec& out, 
			   double maxwid, 
			   double maxhei) const
{
	AssertValid();

	cvTHeader   *pH   = static_cast<cvTHeader*>  (GetInst());
	char        *pOfs = static_cast<char*>       (GetInst()) + 
						pH->longitCntrlOfs;
	cvTCntrlPnt *pCntrlPnt = (reinterpret_cast<cvTCntrlPnt*>(pOfs)) +
						m_pRoad->cntrlPntIdx;

	double maxX		= pCntrlPnt->location.x;
	double maxY		= pCntrlPnt->location.y;
	double minX		= pCntrlPnt->location.x;
	double minY		= pCntrlPnt->location.y;

	CRoadPiece	boundBox(
					GetCved(), 
					m_pRoad->myId, 
					0.0, 
					0.0,
					maxwid, 
					maxhei,
					0,
					0); 

	out.clear();
	bool insideBoundingBox;
	int first =0;		//first control point of roadpiece  
	bool pushLastRoadPiece = true;
	for (int i=0; i<m_pRoad->numCntrlPnt; i++)	{
		FindBoundary(&maxX, &maxY, &minX, &minY, pCntrlPnt+i);
		insideBoundingBox = boundBox.HitTest(maxX-minX, maxY-minY);
		if ( !insideBoundingBox){
			CRoadPiece piece(
							GetCved(), 
							m_pRoad->myId, 
							minX, 
							minY, 
							maxX, 
							maxY,
							first,
							i);
			out.push_back(piece);
			maxX = ( pCntrlPnt+i )->location.x;
			minX = ( pCntrlPnt+i )->location.x;
			maxY = ( pCntrlPnt+i )->location.y;
			minY = ( pCntrlPnt+i )->location.y;
			FindBoundary(&maxX, &maxY, &minX, &minY, pCntrlPnt+i);
#if 0
			if ( first==(i-1) )
			{
				// which means bounding box used for testing is too small
				string msg("bounding box used in BreakUp() is too small");
				cvInternalError e(msg, __FILE__, __LINE__);
				throw e;
			}
#endif
			first = i;
			if (i == (m_pRoad->numCntrlPnt-1))
				pushLastRoadPiece = false;
		}
	}
	// for the last few points thare are not beyound the bounding box
	if (pushLastRoadPiece){
		CRoadPiece piece(
				GetCved(), 
				m_pRoad->myId, 
				minX, 
				minY, 
				maxX, 
				maxY,
				first,
				m_pRoad->numCntrlPnt-1);	
		out.push_back(piece);
	}
} // end of	BreakUp

//////////////////////////////////////////////////////////////////////////////
///\brief Find the boundary of the part of the road
///
///\par Description: FindBoundary
/// 	Find the boundary of the part of the road
/// 
///\remark
///		This function will cause a failed assertion if it is called on an
/// 	invalid CRoad instance.
///\par 
/// 	This function find the boundry of the the current set of control points.
/// 	The way it works is to find a bounding box which exactly cover the
/// 	roads. Given the set of control points, first acqure the width and 
/// 	the right vector of each control point, and then put together these
/// 	information with the location of the control point to compute the 
/// 	boundary of the road piece.
///
///\param 	pMaxX - pointers to doubles of upper right of boundary
///\param   pMaxY - pointers to doubles of upper right of boundary    
///\param 	pMinX - pointers to doubles of lower left of boundary
///\param   pMinY - pointers to doubles of lower left of boundary
///\param 	pPnt  - pointer to information of control points in memory block
///
///\return void
///
//////////////////////////////////////////////////////////////////////////////
void 
CRoad::FindBoundary(
			double*			pMaxX, 
			double*			pMaxY, 
			double*			pMinX, 
			double*			pMinY, 
			cvTCntrlPnt*	pPnt) const
{ 
	AssertValid();

	double x1 = pPnt->location.x + pPnt->physicalWidth/2*pPnt->rightVecCubic.i;
	double x2 = pPnt->location.x - pPnt->physicalWidth/2*pPnt->rightVecCubic.i;
	double y1 = pPnt->location.y + pPnt->physicalWidth/2*pPnt->rightVecCubic.j;
	double y2 = pPnt->location.y - pPnt->physicalWidth/2*pPnt->rightVecCubic.j;

	if (x1 > *pMaxX)
		*pMaxX = x1;
	else if (x1 < *pMinX)
		*pMinX = x1;

	if (x2 > *pMaxX)    
		*pMaxX = x2;    
	else if (x2 < *pMinX)   
		*pMinX = x2;    

	if (y1 > *pMaxY)    
		*pMaxY = y1;    
	else if (y1 < *pMinY)   
		*pMinY = y1;    

	if (y2 > *pMaxY)        
		*pMaxY = y2;        
	else if (y2 < *pMinY)
		*pMinY = y2;
} // end of FindBoundary	

//////////////////////////////////////////////////////////////////////////////
///\brief This function return if the road is a highway
///
///\par Description: 
///		This function return if the road is a highway
///
///\remarks
///		This function will look through all the attributes of the road and
///		check if there is an attribute of cCV_HIGHWAY_ATTR
///
///\return  true if it's a highway
///			false otherwise
///
//////////////////////////////////////////////////////////////////////////////
bool
CRoad::IsHighway() const
{
	vector<CAttr> attrVec;
	QryAttr(attrVec);

	unsigned int i = 0;
	for(; i<attrVec.size(); i++){
		if (attrVec[i].GetId() == cCV_HIGHWAY_ATTR){
			return true;
		}
	}
	return false;
}

	
	

} // namespace CVED
