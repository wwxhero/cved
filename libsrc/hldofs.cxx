//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: hldofs.cxx,v 1.8 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Lijen Tsao
// Date:		March, 2000
//
// Description:	The implementation of the CHldOfs class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"
#include "cvedstrc.h"

namespace CVED {

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator<<
// 	Prints the contents of cItem to the out ostream.
//
// Remarks: 
//
// Arguments:  
// 	out - ostream to print the CHldOfs to
// 	cItem - item to print
//
// Returns: Reference to the resulting ostream.
//
//////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const CHldOfs& cItem)
{
	out << "(HldOfs:" << cItem.GetObjId() << ", " << cItem.GetName() << ", " 
		<< cItem.GetReason() << ", " << cItem.GetDistance() << ", "
		<< cItem.GetOrientation() << ", " << cItem.GetThickness() << ")";
	return out;
} // end of operator<<

} // namespace CVED

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
// 
// Description: CHldOfs
// 	Constructor that takes a CCved instance and an attribute id
//
// Remarks: 
//
// Arguments:
//	cCved - reference to a CCved instance
//	id - id of a valid attribute
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CHldOfs::CHldOfs(const CCved& cCved, int id) 
	: CCvedItem(&cCved)
{
	if (id==0){
		m_pHldOfs=0;
	}
	else{
		cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
		char*		pOfs = static_cast<char*> (GetInst()) + pH->hldOfs;
		m_pHldOfs			 = (reinterpret_cast<THldOfs*>(pOfs)) + id;
	}
	
} // end of CHldOfs

//////////////////////////////////////////////////////////////////////////////
// 
// Description: CHldOfs
// 	Constructor that takes a cvTHldOfs
//
// Remarks: 
//
// Arguments:
//	cCved - reference to a CCved instance
//	pHldOfs - pointer to a valid cvTHldOfs
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CHldOfs::CHldOfs(const CCved& cCved, cvTHldOfs* pHldOfs) 
	: CCvedItem(&cCved), 
	  m_pHldOfs(pHldOfs)
{} // end of CHldOfs

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy of the parameter to the current instance.
//
// Remarks: 
//
// Arguments: 
// 	cRhs - const CHldOfs instance
//
// Returns: a reference to the current instance
//
//////////////////////////////////////////////////////////////////////////////
CHldOfs&
CHldOfs::operator=(const CHldOfs& cRhs) {

	if (&cRhs != this) {

		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pHldOfs = cRhs.m_pHldOfs;
	}

	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetObjId
// 	Returns the holdoffset's objId
//
// Remarks: this function will return 0 if this hldOfs is not a traffic sign.
// the reason it won't return negative number is because the object id type
// is unsigned, which means if we return negative number and the user of this
// function uses a variable of type TObjectPoolIdx to get return value, the
// user would end up getting a large positive number. 
//
// Arguments:
//
// Returns: hldofs's objIdx
//
//////////////////////////////////////////////////////////////////////////////
int
CHldOfs::GetObjId(void) const
{
	AssertValid();
	return m_pHldOfs->objIdx;
} // end of GetReason

//////////////////////////////////////////////////////////////////////////////
//
// Description: Returns the hold offset's solId.
//
// Remarks: 
//
// Arguments:
//
// Returns: hldofs's solIdx, or -1 if hldOfs is not a traffic sign
//
//////////////////////////////////////////////////////////////////////////////
int
CHldOfs::GetSolId( void ) const
{
	int objId = GetObjId();
	if( objId == 0 )  return -1;
 
	cvTObj* pObj = BindObj( GetObjId() );
	return pObj->attr.solId;
} 

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetName
// 	Returns the holdoffset's Name
//
// Remarks: 
//
// Arguments:
//
// Returns: hldofs's name
//
//////////////////////////////////////////////////////////////////////////////
string
CHldOfs::GetName(void) const
{
	AssertValid();
	cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
	char*		pOfs = static_cast<char*> (GetInst()) + pH->objectOfs;
	cvTObj*		pObj  = (reinterpret_cast<cvTObj*>(pOfs)) + m_pHldOfs->objIdx;
	return pObj->name;
} // end of GetName

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetReason
// 	Returns the holdoffset's reason
//
// Remarks: the returned char should be ORed with eSIGN, eTLIGHT, or eOVLAP 
//
// Arguments:
//
// Returns: hldofs's reason
//
//////////////////////////////////////////////////////////////////////////////
char
CHldOfs::GetReason(void) const
{
	AssertValid();
	return m_pHldOfs->reason;
} // end of GetReason

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetDistance
// 	Returns the distance of hldofs
//
// Remarks: 
//
// Arguments:
//
// Returns: the double the distance of the hldofs
//
//////////////////////////////////////////////////////////////////////////////
double 
CHldOfs::GetDistance(void) const
{
	AssertValid();
	return m_pHldOfs->distance;
} // end of GetDistance

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetOrientation
// 	Returns the hldofs's orientation
//
// Remarks: 
//
// Arguments:
//
// Returns: the double the orientation of the attribute
//
//////////////////////////////////////////////////////////////////////////////
double
CHldOfs::GetOrientation(void) const
{
	AssertValid();
	return m_pHldOfs->orientation;
} // end of GetOrientation


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetThickNess 
// 	Returns the attribute's ThickNess  
//
// Remarks: 
// 
// Arguments:
//
// Returns: the thickNess of the attribute 
//
//////////////////////////////////////////////////////////////////////////////
double
CHldOfs::GetThickness(void) const
{
	AssertValid();
	return m_pHldOfs->thickness;
} // end of GetTo

} // namespace CVED
