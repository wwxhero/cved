//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: objattr.cxx,v 1.7 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Lijen Tsao
// Date:		May, 2000
//
// Description:	The implementation of the CObjAttr class
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
// 	out - ostream to print the CObjAttr to
// 	cItem - item to print
//
// Returns: Reference to the resulting ostream.
//
//////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const CObjAttr& cItem)
{
	out << "(ObjAttr:";
	out << "SolId:"<<cItem.GetSolId()<<" "; 
	out << "HcsmId:"<<cItem.GetHcsmId()<<" "; 
	out << "XSize:"<<cItem.GetXSize()<<" "; 
	out << "YSize:"<<cItem.GetYSize()<<" "; 
	out << "ZSize:"<<cItem.GetZSize()<<" "; 

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
// Description: CObjAttr
// 	Constructor that takes a CCved instance and an object attribute id
//
// Remarks: 
//
// Arguments:
//	cCved - reference to a CCved instance
//	id - id of a valid object attribute
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CObjAttr::CObjAttr(const CCved& cCved, int id) 
	: CCvedItem(&cCved)
{
	m_pObjAttr = BindObjAttr(id);	
} // end of CObjAttr

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy of the parameter to the current instance.
//
// Remarks: 
//
// Arguments: 
// 	cRhs - const CObjAttr instance
//
// Returns: a reference to the current instance
//
//////////////////////////////////////////////////////////////////////////////
CObjAttr&
CObjAttr::operator=(const CObjAttr& cRhs) {

	if (&cRhs != this) {

		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pObjAttr = cRhs.m_pObjAttr;
	}

	return *this;
} // end of operator=


//////////////////////////////////////////////////////////////////////////////
//
// Description: GetSolId
// 	Returns the object attribute's sol id
//
// Remarks: 
//
// Arguments:
//
// Returns: the sol id of the attribute
//
//////////////////////////////////////////////////////////////////////////////
int
CObjAttr::GetSolId(void) const
{
	AssertValid();
	return m_pObjAttr->solId;
} // end of GetSolId

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetHcsmId
// 	Returns the attribute's HcsmId
//
// Remarks: 
//
// Arguments:
//
// Returns: the hcsm id of the attribute
//
//////////////////////////////////////////////////////////////////////////////
int 
CObjAttr::GetHcsmId(void) const
{
	AssertValid();
	return m_pObjAttr->hcsmId;
} // end of GetHcsmId

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetXSize
// 	Returns the attribute's size in x
//
// Remarks: 
//
// Arguments:
//
// Returns: the size in x
//
//////////////////////////////////////////////////////////////////////////////
double
CObjAttr::GetXSize(void) const
{
	AssertValid();
	return m_pObjAttr->xSize;
} // end of GetXSize

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetYSize
// 	Returns the attribute's size in x
//
// Remarks: 
//
// Arguments:
//
// Returns: the size in Y
//
//////////////////////////////////////////////////////////////////////////////
double
CObjAttr::GetYSize(void) const
{
	AssertValid();
	return m_pObjAttr->ySize;
} // end of GetYSize

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetZSize
// 	Returns the attribute's size in Z
//
// Remarks: 
//
// Arguments:
//
// Returns: the size in Z
//
//////////////////////////////////////////////////////////////////////////////
double
CObjAttr::GetZSize(void) const
{
	AssertValid();
	return m_pObjAttr->zSize;
} // end of GetZSize


} // namespace CVED
