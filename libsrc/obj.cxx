//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: obj.cxx,v 1.40 2015/12/17 21:20:40 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	The implementation of the CObj class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//	Construction/Destruction
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//	
// Description: ~CObj
// 	Default destructor does nothing.
//
// Remarks:
//
// Arguments:
//
// Returns:
//
//////////////////////////////////////////////////////////////////////////////
CObj::~CObj()
{} // end of ~CObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: CObj
// 	This constructor takes a CCved instance and an object id.
//
// Remarks: If an uninitialized CCved instance is given, then the function 
// 	will cause a failed assertion.  If an invalid object id is given, then an
// 	invalid CObj will be created.
//
// Arguments:
// 	cCved - CCved instance
// 	id - valid object id
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CObj::CObj(const CCved& cCved, int id) 
	: CCvedItem(&cCved)
{
	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());

	if ( id < cNUM_DYN_OBJS || id >= (int)pH->objectCount ) {
		m_pObj = 0;
	}
	else {
		m_pObj = BindObj(id);
	}
} // end of CObj

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy from the parameter to the current instance.
//
// Remarks:
//
// Arguments:
// 	cRhs - reference to another CObj instance
//
// Returns:
// 	A reference to the current instance to allow nested assignments.
//
//////////////////////////////////////////////////////////////////////////////
CObj&
CObj::operator=(const CObj& cRhs)
{
	// Assign superclass members
	((CCvedItem*)(this))->operator=(cRhs);

	if (this != &cRhs)
		m_pObj = cRhs.m_pObj;
	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: Copy Constructor
//  This copy constructor assigns the contents of the parameter to the current
//  instance.
//
// Remarks:
//
// Arguments:
//  cSrc - a CObj instance
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////

CObj::CObj(const CObj& cSrc) 
{
	*this = cSrc;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetType
//	Retrieves the type of the current object.
//	
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: a cvEObjType enumerated value
//
//////////////////////////////////////////////////////////////////////////////
cvEObjType
CObj::GetType(void) const
{
	AssertValid();
	return m_pObj->type;
} // end of GetType

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetId
//	Retrieves the id of the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: the integer identifier of the current object
//
//////////////////////////////////////////////////////////////////////////////
int
CObj::GetId(void) const
{
	AssertValid();
	return m_pObj->myId;
} // end of GetId

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetName
// 	Retrieves the name of the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: a const char* to the name of the current object.
//
//////////////////////////////////////////////////////////////////////////////
const char*
CObj::GetName(void) const
{
	AssertValid();
	return m_pObj->name;
} // end of GetName

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the HCSM id of the current object.
//
// Remarks: This function will cause a failed assertion if it is called 
//  on an invalid CObj instance.
//
// Arguments:
//
// Returns: The integer HCSM identifier of the current object.
//
//////////////////////////////////////////////////////////////////////////////
int
CObj::GetHcsmId( void ) const
{
	AssertValid();
	return m_pObj->attr.hcsmId;
} // end of GetHcsmId

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the Color index of the current object.
//
// Remarks: This function will cause a failed assertion if it is called 
//  on an invalid CObj instance.
//
// Arguments:
//
// Returns: The Color index of the current object.
//
//////////////////////////////////////////////////////////////////////////////
int
CObj::GetColorIndex( void ) const
{
	AssertValid();
	return m_pObj->attr.colorIndex;
} // end of GetHcsmId

//////////////////////////////////////////////////////////////////////////////
//
// Description: Sets the HCSM id of the current object.
//
// Remarks: This function will cause a failed assertion if it is called 
//  on an invalid CObj instance.
//
// Arguments:
//  hcsmId - The hcsm id.
//
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
void
CObj::SetHcsmId( int hcsmId )
{
	AssertValid();
	m_pObj->attr.hcsmId = hcsmId;
} // end of GetHcsmId

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the CSolObj of the current object.
//
// Remarks: This function will cause a failed assertion if it is called 
//  on an invalid CObj instance.
//
// Arguments:
//
// Returns: A pointer to the CSolObj of the current object.
//
//////////////////////////////////////////////////////////////////////////////
const CSolObj*
CObj::GetSolObj( void ) const
{
	AssertValid();
	return CCved::GetSol().GetObj(m_pObj->attr.solId);
} // end of GetSolId


//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the SOL id of the current object.
//
// Remarks: This function will cause a failed assertion if it is called 
//  on an invalid CObj instance.
//
// Arguments:
//
// Returns: The integer SOL identifier of the current object.
//
//////////////////////////////////////////////////////////////////////////////
int
CObj::GetSolId( void ) const
{
	AssertValid();
	return m_pObj->attr.solId;
} // end of GetSolId

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the x size of the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: The double x size of the current object.
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetXSize( void ) const
{
	AssertValid();
	return m_pObj->attr.xSize;
} // end of GetXSize

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the y size of the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: The double y size of the current object.
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetYSize( void ) const
{
	AssertValid();
	return m_pObj->attr.ySize;
} // end of GetYSize

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the z size of the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: The double z size of the current object.
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetZSize( void ) const
{
	AssertValid();
	return m_pObj->attr.zSize;
} // end of GetZSize

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the number of composite pieces in the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: The number of composite pieces in the current object.
//
//////////////////////////////////////////////////////////////////////////////
int
CObj::GetNumCompositePieces( void ) const
{
	AssertValid();
	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.compositeSignState.numChildren;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.compositeSignState.numChildren;
	}
} // end of GetNumCompositePieces

//////////////////////////////////////////////////////////////////////////////
//
// Description: Retrieves the composite pieces of the current object.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: The composite pieces of the current object.
//
//////////////////////////////////////////////////////////////////////////////
const int*
CObj::GetCompositePieces(void) const
{
	AssertValid();
	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.compositeSignState.childrenOpts;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.compositeSignState.childrenOpts;
	}
} // end of GetZSize


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPos
// 	Retrieves the position of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D position of the current object
//
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CObj::GetPos(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CPoint3D pos(m_pObj->stateBufA.state.anyState.position);
		return pos;
	}
	else {							// odd frame
		CPoint3D pos(m_pObj->stateBufB.state.anyState.position);
		return pos;
	}
} // end of GetPos
//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Get the target orientation in Yaw Pitch Roll
///
///\remark This function will cause a failed assertion if it is called on an
/// 	invalid CObj instance. The Value from this function is only valid of
///     ADOs
///
///\return a current Target Position in feet
///
//////////////////////////////////////////////////////////////////////////////
COrientation CObj::GetEulerAngles(void) const{
    cvTObjState::AnyObjState *cpCurState;
	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		cpCurState = &m_pObj->stateBufA.state.anyState;
	}
	else {							// odd frame
		cpCurState = &m_pObj->stateBufB.state.anyState;
	}
    COrientation ret;
    ret.SetUnitVectors(cpCurState->tangent,cpCurState->lateral);
    return ret;
}
//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Get the target position of the vehicle
///
///\remark This function will cause a failed assertion if it is called on an
/// 	invalid CObj instance. The Value from this function is only valid of
///     ADOs
///
///\return a current Target Position in feet
///
//////////////////////////////////////////////////////////////////////////////
COrientation CObj::GetEulerAnglesImm(void) const{
    cvTObjState::AnyObjState *cpCurState;
	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		cpCurState = &m_pObj->stateBufB.state.anyState;
	}
	else {							// odd frame
		cpCurState = &m_pObj->stateBufA.state.anyState;
	}
    COrientation ret;
    ret.SetUnitVectors(cpCurState->tangent,cpCurState->lateral);
    return ret;
}



//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Get the target position of the vehicle
///
///\remark This function will cause a failed assertion if it is called on an
/// 	invalid CObj instance. The Value from this function is only valid of
///     ADOs
///
///\return a current Target Position in feet
///
//////////////////////////////////////////////////////////////////////////////
CPoint3D		
CObj::GetPosTarget(void) const{
    AssertValid();
    cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
    //if (m_pObj->hcsmType != eCV_VEHICLE){
    //    return CPoint3D();
    //}
	if( (pH->frame & 1) == 0 ) 
	{
        CPoint3D tp( m_pObj->stateBufA.contInp.vehicleContInp.contInp.targPos );
		return tp;
	}
	else 
	{
		CPoint3D tp( m_pObj->stateBufB.contInp.vehicleContInp.contInp.targPos );
		return tp;
	}
}
//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Get the target speed of the vehicle
///
///\remark This function will cause a failed assertion if it is called on an
/// 	invalid CObj instance. The Value from this function is only valid of
///     ADOs
///
///\return a current Target Speed in Meters per second
///
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetAccelTarget(void) const{
    AssertValid();
    cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
    //if (m_pObj->hcsmType != eCV_VEHICLE){
    //    return CPoint3D();
    //}
	if( (pH->frame & 1) == 0 ) 
	{
		return m_pObj->stateBufA.contInp.vehicleContInp.contInp.targAccel;
	}
	else 
	{
		return m_pObj->stateBufB.contInp.vehicleContInp.contInp.targAccel;
	}
}
//////////////////////////////////////////////////////////////////////////////
///
///\brief 
/// 	Get the target speed of the vehicle
///
///\remark This function will cause a failed assertion if it is called on an
/// 	invalid CObj instance. The Value from this function is only valid of
///     ADOs
///
///\return a current Target Speed in Meters per second
///
//////////////////////////////////////////////////////////////////////////////
double		    
CObj::GetVelTarget(void) const{
    AssertValid();
    cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if( (pH->frame & 1) == 0 ) 
	{
        return m_pObj->stateBufA.contInp.vehicleContInp.contInp.targVel;
	}
	else 
	{
		return m_pObj->stateBufB.contInp.vehicleContInp.contInp.targVel;
	}
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPosImm
// 	Retrieves the immediate position of the current object, bypassing the
// 	double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate position of the current object
//
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CObj::GetPosImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CPoint3D pos(m_pObj->stateBufB.state.anyState.position);
		return pos;
	}
	else {							// odd frame
		CPoint3D pos(m_pObj->stateBufA.state.anyState.position);
		return pos;
	}
} // end of GetPosImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPosImmDelayCompensated
//  Only for dynamic objects, it returns position values compensated for the
//  visual delay.
//  
//  Two types of compensations are supported:
//    - a simple one, extrapolating positions in time;
//    - a detailed one, based on VRTC's algorithm
// 
// Arguments:
//
// Returns: a CPoint3D immediate compensated position of the current object
//
//////////////////////////////////////////////////////////////////////////////
CPoint3D
CObj::GetPosImmDelayCompensated(void) const
{
	AssertValid();
	bool simp_comp = true;
	double extrap_step = 0.06; // seconds

	if (simp_comp) {	// use simple compensation
		cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
		if ( (pH->frame & 1) == 0 ) {		// even frame
			// Obtain position and velocities
			CPoint3D pos(m_pObj->stateBufB.state.anyState.position);
			double vel_x = GetVelImm();
			double vel_y = GetVelLatImm();
			double vel_z = GetVelNormImm();
            // Compensate position
			pos.m_x += extrap_step * vel_x;
			pos.m_y += extrap_step * vel_y;
			pos.m_z += extrap_step * vel_z;

// Begin orientation compensation
// Everything is commented out, it should be moved into getOrientation function
// after the first tests...
			// Compute roll, pitch, and yaw (like in vehicle dynamics)
			//CVector3D lat = GetLatImm();
			//CVector3D tng = GetTanImm();
			//lat.m_i = -lat.m_i;
			//lat.m_j = -lat.m_j;
			//lat.m_k = -lat.m_k;
			//CVector3D nrm;
			//nrm.m_i = ( tng.m_j * lat.m_k ) - ( tng.m_k * lat.m_j );
			//nrm.m_j = ( tng.m_k * lat.m_i ) - ( tng.m_i * lat.m_k );
			//nrm.m_k = ( tng.m_i * lat.m_j ) - ( tng.m_j * lat.m_i );

			//double roll  = atan2( lat.m_k, nrm.m_k );
			//double pitch = asin( tng.m_k );
			//double yaw   = atan2( tng.m_j, tng.m_i );

			// Obtain roll, pitch, and yaw rates
			//double rollRate = GetRollRateImm();
			//double pitchRate = GetPitchRateImm();
			//double yawRate = GetYawRateImm();
			// Compensate orientation (NOT a recommended formula)
			//roll += extrap_step * rollRate;
			//pitch += extrap_step * pitchRate;
			//yaw += extrap_step * yawRate;
			// Compute the tangent and lateral vectors to obtain
			// the orientation (like in vehicle dynamics)
			//tng.m_i = cos(yaw) * cos(pitch);
			//tng.m_j = sin(yaw) * cos(pitch);
			//tng.m_k = sin(pitch);
			//lat.m_i = -( -sin(yaw) * cos(roll) - cos(yaw) * sin(pitch) * sin(roll));
			//lat.m_j = -( cos(yaw) * cos(roll) - sin(yaw) * sin(pitch) * sin(roll));
			//lat.m_k = -( cos(pitch) * sin(roll) );

// End of orientation compensation			

			return pos;
		}
		else {							// odd frame
			// Obtain position and velocities
			CPoint3D pos(m_pObj->stateBufA.state.anyState.position);
			double vel_x = GetVelImm();
			double vel_y = GetVelLatImm();
			double vel_z = GetVelNormImm();
			// Compensate position
			pos.m_x += extrap_step * vel_x;
			pos.m_y += extrap_step * vel_y;
			pos.m_z += extrap_step * vel_z;

			return pos;
		}
	}
	else { // use VRTC's algorithm
		// Initialize coefficients (done here for now)
		// It should be done only ONCE!
		double pi2 = 2.0 * 3.1415;
		double Vis_Comp_F0 = 2.5;		// Hz
		double Vis_Comp_Delay = 0.075;	// seconds
		double Vis_Comp_Filt0 = 6.0;		// Hz
		double Vis_Comp_m = 2.0;			// Hz
		double stepsize = 0.1;			// seconds (arbitrary value)
		// Compute compensater and filter coefficients
		double phi0 = pi2 * Vis_Comp_F0 * Vis_Comp_Delay;
		double theta0 = pi2 * Vis_Comp_F0 * stepsize;
        // Get compensator coefficients
		double st = sin(theta0);   // sine with samplig time
		double ct = cos(theta0);   // cosine with sampling time
		double sp = sin(phi0);     // sine with visual delay
		double cp = cos(phi0);     // cosine with visual delay

		double b1 = Vis_Comp_Delay * (st * (1. - cp) + ct * (phi0 - sp)) / (phi0 * (ct - 1.));
        double b2 = Vis_Comp_Delay * (st * (sp - phi0) + (1. - ct) * (cp - 1.)) / (2. * phi0 * st * (ct - 1.));
        double b0 = Vis_Comp_Delay - b1 - b2;
        // Get filter coefficients
		phi0   = pi2 * Vis_Comp_Filt0 * Vis_Comp_Delay;
		theta0 = pi2 * Vis_Comp_Filt0 * stepsize;
		st = sin(theta0);  // sine with sampling time
		ct = cos(theta0);  // cosine with sampling time
		sp = sin(phi0);    // sine with visual delay
		cp = cos(phi0);    // cosine with visual delay

		double q1 = Vis_Comp_Delay * (st * (1. - cp) + ct * (phi0 - sp)) / (phi0 * (ct - 1.));
        double q2 = Vis_Comp_Delay * (st * (sp - phi0) + (1. - ct) * (cp - 1.)) / (2. * phi0 * st * (ct - 1.));
        double q0 = Vis_Comp_Delay - q1 - q2;

        double d0  = (1. + Vis_Comp_Delay * Vis_Comp_m) / (1. + Vis_Comp_m * q0);
        double c1  = Vis_Comp_m * q1 / (1. + Vis_Comp_m * q0);
        double c2  = Vis_Comp_m * q2 / (1. + Vis_Comp_m * q0);
		// Here ends the initialization


		// Obtain position and velocities
		CPoint3D pos(m_pObj->stateBufA.state.anyState.position);
		double vel_x = GetVelImm();
		double vel_y = GetVelLatImm();
		double vel_z = GetVelNormImm();
		return pos;
	}
} // end of GetPosImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetTan
// 	Retrieves the tangent of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D tangent of the current object
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CObj::GetTan(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CVector3D vec(m_pObj->stateBufA.state.anyState.tangent);
		return vec;
	}
	else {							// odd frame
		CVector3D vec(m_pObj->stateBufB.state.anyState.tangent);
		return vec;
	}
} // end of GetTan

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetTanImm
// 	Retrieves the immediate tangent of the current object, bypassing the
// 	double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate tangent of the current object
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CObj::GetTanImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CVector3D tan(m_pObj->stateBufB.state.anyState.tangent);
		return tan;
	}
	else {							// odd frame
		CVector3D tan(m_pObj->stateBufA.state.anyState.tangent);
		return tan;
	}
} // end of GetTanImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLat
// 	Retrieves the lateral of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D lateral of the current object
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CObj::GetLat(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CVector3D vec(m_pObj->stateBufA.state.anyState.lateral);
		return vec;
	}
	else {							// odd frame
		CVector3D vec(m_pObj->stateBufB.state.anyState.lateral);
		return vec;
	}
} // end of GetLat

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLatImm
// 	Retrieves the immediate lateral of the current object, bypassing the
// 	double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate lateral of the current object
//
//////////////////////////////////////////////////////////////////////////////
CVector3D
CObj::GetLatImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CVector3D lat(m_pObj->stateBufB.state.anyState.lateral);
		return lat;
	}
	else {							// odd frame
		CVector3D lat(m_pObj->stateBufA.state.anyState.lateral);
		return lat;
	}
} // end of GetLatImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetNumOfOptions
// 	Retrieves the number of options this object has
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//
// Returns: returns the number of options of the object
//
//////////////////////////////////////////////////////////////////////////////
int
CObj::GetNumOfOptions(void) const
{
	AssertValid();
	return GetSolObj()->GetNumOptions();
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetAllOptions
// 	Retrieves all options of the object
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// Arguments:
//  objAttrVec  this vector contains all the options of the object
//
// Returns: 
//
//////////////////////////////////////////////////////////////////////////////
const vector <CSolOption>&
CObj::GetAllOptions() const
{
	return GetSolObj()->GetOptions();
}	


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetBoundBox
// 	Retrieves the bounding box of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: the CBoundingBox of the current object
//
//////////////////////////////////////////////////////////////////////////////
CBoundingBox
CObj::GetBoundBox(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CBoundingBox bb(m_pObj->stateBufA.state.anyState.boundBox[0].x, 
						m_pObj->stateBufA.state.anyState.boundBox[0].y, 
						m_pObj->stateBufA.state.anyState.boundBox[1].x, 
						m_pObj->stateBufA.state.anyState.boundBox[1].y);
		return bb;
	}
	else {							// odd frame
		CBoundingBox bb(m_pObj->stateBufB.state.anyState.boundBox[0].x, 
						m_pObj->stateBufB.state.anyState.boundBox[0].y, 
						m_pObj->stateBufB.state.anyState.boundBox[1].x, 
						m_pObj->stateBufB.state.anyState.boundBox[1].y);
		return bb;
	}
} // end of GetBoundBox

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetBoundBoxImm
// 	Retrieves the immediate bounding box of the current object, bypassing the
// 	double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: the immediate CBoundingBox of the current object
//
//////////////////////////////////////////////////////////////////////////////
CBoundingBox
CObj::GetBoundBoxImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		CBoundingBox bb(m_pObj->stateBufB.state.anyState.boundBox[0].x, 
						m_pObj->stateBufB.state.anyState.boundBox[0].y, 
						m_pObj->stateBufB.state.anyState.boundBox[1].x, 
						m_pObj->stateBufB.state.anyState.boundBox[1].y);
		return bb;
	}
	else {							// odd frame
		CBoundingBox bb(m_pObj->stateBufA.state.anyState.boundBox[0].x, 
						m_pObj->stateBufA.state.anyState.boundBox[0].y, 
						m_pObj->stateBufA.state.anyState.boundBox[1].x, 
						m_pObj->stateBufA.state.anyState.boundBox[1].y);
		return bb;
	}
} // end of GetBoundBoxImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVel
// 	Retrieves the velocity of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D velocity of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetVel(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.anyState.vel;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.anyState.vel;
	}
} // end of GetVel

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVelImm
// 	Retrieves the immediate velocity of the current object, bypassing 
//   the double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate velocity of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetVelImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.anyState.vel;
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.anyState.vel;
	}
} // end of GetVelImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVelLat
// 	Retrieves the lateral velocity of the current object, using 
//   double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D lateral velocity of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetVelLat(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.velLat;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.velLat;
	}
} // end of GetVelLat

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVelLatImm
// 	Retrieves the immediate lateral velocity of the current object, bypassing 
//   the double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate lateral velocity of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetVelLatImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.velLat;
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.velLat;
	}
} // end of GetVelLatImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVelNorm
// 	Retrieves the vertical (normal to the terrain surface) velocity of the 
//   current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D vertical velocity of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetVelNorm(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.velNorm;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.velNorm;
	}
} // end of GetVelLat

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVelNormImm
// 	Retrieves the immediate vertical (normal to the terrain surface) velocity 
//   of the current object, bypassing the double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate vertical velocity of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetVelNormImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.velNorm;
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.velNorm;
	}
} // end of GetVelNormImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRollRate
// 	Retrieves the roll rate velocity of the current object, using 
//  double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D roll rate of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetRollRate(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.rollRate;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.rollRate;
	}
} // end of GetRollRate

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetRollRateImm
// 	Retrieves the immediate roll rate of the current object, bypassing the 
//  double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate roll rate of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetRollRateImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.rollRate;
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.rollRate;
	}
} // end of GetRollRateImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPitchRate
// 	Retrieves the pitch rate of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D pitch rate of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetPitchRate(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.pitchRate;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.pitchRate;
	}
} // end of GetPitchRate

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetPitcRateImm
// 	Retrieves the immediate pitch rate of the current object, bypassing the 
//  double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate pitch rate of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetPitchRateImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.pitchRate;
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.pitchRate;
	}
} // end of GetPitchRateImm

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetYawRate
// 	Retrieves the yaw rate of the current object, using double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function will return consistant values whenever it is called within
// 	a frame.
//
// Arguments:
//
// Returns: a CPoint3D yaw rate of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetYawRate(void) const
{
	AssertValid();

	cvTHeader*	pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufA.state.vehicleState.vehState.yawRate;
	}
	else {							// odd frame
		return m_pObj->stateBufB.state.vehicleState.vehState.yawRate;
	}
} // end of GetYawRate

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetYawRateImm
// 	Retrieves the immediate yaw rate of the current object, bypassing the 
//  double-buffering.
//
// Remarks: This function will cause a failed assertion if it is called on an
// 	invalid CObj instance.
//
// 	This function may return inconsistant values if it is called multiple 
// 	times within a frame.
//
// Arguments:
//
// Returns: a CPoint3D immediate yaw rate of the current object
//
//////////////////////////////////////////////////////////////////////////////
double
CObj::GetYawRateImm(void) const
{
	AssertValid();

	cvTHeader* pH = static_cast<cvTHeader*>(GetInst());
	if ( (pH->frame & 1) == 0 ) {		// even frame
		return m_pObj->stateBufB.state.vehicleState.vehState.yawRate;
	}
	else {							// odd frame
		return m_pObj->stateBufA.state.vehicleState.vehState.yawRate;
	}
} // end of GetYawRateImm

} // namespace CVED
