//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: attr.cxx,v 1.16 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description:	The implementation of the CAttr class
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
// 	out - ostream to print the CAttr to
// 	cItem - item to print
//
// Returns: Reference to the resulting ostream.
//
//////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& out, const CAttr& cItem)
{
	if ( cItem.GetFrom() < 0 ) {
		out << "(Attr:" << cItem.GetName() << ", " 
				<< cItem.GetVal1() << ", " << cItem.GetVal2()
				<< ")";
	}
	else {
		out << "(Attr:" << cItem.GetName() << ", " 
				<< cItem.GetVal1() << ", " << cItem.GetVal2()
				<< cItem.GetFrom() << "->" << cItem.GetTo()
				<< ")";
	}
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
// Description: CAttr
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
CAttr::CAttr(const CCved& cCved, int id) 
	: CCvedItem(&cCved)
{
	if (id==0){
		m_pAttr=0;
	}
	else{
		cvTHeader*	pH   = static_cast<cvTHeader*>  (GetInst());
		char*		pOfs = static_cast<char*> (GetInst()) + pH->attrOfs;
		m_pAttr			 = (reinterpret_cast<TAttr*>(pOfs)) + id;
	}
	
} // end of CAttr

//////////////////////////////////////////////////////////////////////////////
// 
// Description: CAttr
// 	Constructor that takes a cvTAttr
//
// Remarks: 
//
// Arguments:
//	cCved - reference to a CCved instance
//	pAttr - pointer to a valid cvTAttr
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CAttr::CAttr(const CCved& cCved, cvTAttr* pAttr) 
	: CCvedItem(&cCved), 
	  m_pAttr(pAttr)
{} // end of CAttr

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
// 	Performs a deep copy of the parameter to the current instance.
//
// Remarks: 
//
// Arguments: 
// 	cRhs - const CAttr instance
//
// Returns: a reference to the current instance
//
//////////////////////////////////////////////////////////////////////////////
CAttr&
CAttr::operator=(const CAttr& cRhs) {

	if (&cRhs != this) {

		// Assign superclass members
		this->CCvedItem::operator=(cRhs);

		m_pAttr = cRhs.m_pAttr;
	}

	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetName
// 	Returns a string that holds the name of the current attribute.  
//
// Remarks: 
// 	If no name has been associated with the attribute, the string will be 
// 	empty.
//
// Arguments:
//
// Returns: a string representing the attribute's name.
//
//////////////////////////////////////////////////////////////////////////////
string
CAttr::GetName(void) const
{
	AssertValid();

	void*	p = GetInst();

	cvTHeader*	pH		= static_cast<cvTHeader*>(GetInst());
	char*		pOfs	= static_cast<char*>(GetInst()) + pH->charOfs;

	string s(pOfs + m_pAttr->nameIdx);

	return s;
} // end of GetName

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetId
// 	Returns the attribute's identifier
//
// Remarks: 
//
// Arguments:
//
// Returns: the int id of the attribute
//
//////////////////////////////////////////////////////////////////////////////
int
CAttr::GetId(void) const
{
	AssertValid();
	return m_pAttr->myId;
} // end of GetId

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVal1
// 	Returns the attribute's first value
//
// Remarks: 
//
// Arguments:
//
// Returns: the double first value of the attribute
//
//////////////////////////////////////////////////////////////////////////////
double 
CAttr::GetVal1(void) const
{
	AssertValid();
	return m_pAttr->value1;
} // end of GetVal1

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetVal2
// 	Returns the attribute's second value
//
// Remarks: 
//
// Arguments:
//
// Returns: the double second value of the attribute
//
//////////////////////////////////////////////////////////////////////////////
double
CAttr::GetVal2(void) const
{
	AssertValid();
	return m_pAttr->value2;
} // end of GetVal2

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetFrom
// 	Returns the attribute's beginning range.  
//
// Remarks: Attributes can be associated with part of a road.  The part of 
// 	the road is described by two distances the 'from' and the 'to' distances.  
// 	This function returns the distance along the road at which the attribute 
// 	becomes valid.
// 
// Arguments:
//
// Returns: the distance after which the attribute applies
//
//////////////////////////////////////////////////////////////////////////////
double
CAttr::GetFrom(void) const
{
	AssertValid();
	return m_pAttr->from;
} // end of GetFrom

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetTo 
// 	Returns the attribute's end range.  
//
// Remarks: Attributes can be associated with part of a road.  The part of the 
// 	road is described by two distances the 'from' and the 'to' distances.  
// 	This function returns the distance along the road up to which the 
// 	attribute becomes valid.
// 
// Arguments:
//
// Returns: the distance up to which the attribute applies
//
//////////////////////////////////////////////////////////////////////////////
double
CAttr::GetTo(void) const
{
	AssertValid();
	return m_pAttr->to;
} // end of GetTo

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetLaneMask 
// 	Returns the attribute's LaneMask  
//
// Remarks: Attributes can be applied on lanes that lanemask specifies
// 
// Arguments:
//
// Returns: the distance up to which the attribute applies
//
//////////////////////////////////////////////////////////////////////////////
int
CAttr::GetLaneMask(void) const
{
	AssertValid();
	return m_pAttr->laneMask;
} // end of GetTo

} // namespace CVED
