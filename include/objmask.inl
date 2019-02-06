/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: $Id: objmask.inl,v 1.2 2018/12/10 16:01:15 IOWA\dheitbri Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	Inline functions for the object type mask class
//
//////////////////////////////////////////////////////////////////////////////
#ifndef __OBJ_MASK_INL
#define __OBJ_MASK_INL		// {secret}

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator =
// 	Assignment operator allows assignment of one class to another.  
//
// Remarks:
//
// Arguments:
//    rhs - the object on the right hand side of assignment
//
// Returns: an l-value to allow cascaded assignments
//
//////////////////////////////////////////////////////////////////////////////
inline const CObjTypeMask& 
CObjTypeMask::operator=(const CObjTypeMask& cRhs)
{	
	if ( this != &cRhs )
		for (int i=0; i<cNUM_OBJECT_TYPES/8+1; i++) 
			m_data[i] = cRhs.m_data[i];
	return *this;
} // end of operator=

//////////////////////////////////////////////////////////////////////////////
//
// Description: CObjTypeMask
// 	Default constructor initializes the mask to empty.
//
// Remarks:
//
// Arguments: 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CObjTypeMask::CObjTypeMask()
{
	for (int i=0; i<(cNUM_OBJECT_TYPES+1)/8+1; i++)
		m_data[i] = 0;
} // end of CObjTypeMask

//////////////////////////////////////////////////////////////////////////////
//
// Description: CObjTypeMask
// 	Copy constructor creates a replica of an existing class.
//
// Remarks: 
//
// Arguments: 
//  copy - object to be copied 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CObjTypeMask::CObjTypeMask(const CObjTypeMask& cCopy)
{
	*this = cCopy;
} // end of COjTypeMask

//////////////////////////////////////////////////////////////////////////////
//
// Description: CObjTypeMask
// 	Constructor that takes an unsigned integer.
//
// Remarks: 
//	The contructor with the unsigned integer initializes the
//	class with the contents of that integer.  Given that the
//	actual values of the various object types can change, it
//	is best if this constructor is used only as a convenient 
//	converter of the integer 0 to an empty mask.
//
// Arguments: 
//	bits - an unsinged integer that initializes the class bit by bit
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CObjTypeMask::CObjTypeMask(TU32b  bits)
{
	int i;

	for (i=0; i<(cNUM_OBJECT_TYPES+1)/8+1; i++) 
		m_data[i] = 0;
	if ( bits  ) {
		int lim;		// how many elements of the mask array to initialize

		lim = (cNUM_OBJECT_TYPES/8+1) < 4 ? cNUM_OBJECT_TYPES/8+1 : 4;
		memcpy(m_data, &bits, lim * sizeof(TU8b));
	}
} // end of CObjTypeMask

///////////////////////////////////////////////////////////////////////////////
///
/// Description: CObjTypeMask
///\brief 	Constructor that takes an object type.
///
/// Remarks: 
///	The contructor with the unsigned integer initializes the
///	class with the contents of that integer.  Given that the
///	actual values of the various object types can change, it
///	is best if this constructor is used only as a convenient 
///	converter of the integer 0 to an empty mask.
///
/// Arguments: 
///	type
/// 
/// Returns: void
///
///////////////////////////////////////////////////////////////////////////////
inline
CObjTypeMask::CObjTypeMask(cvEObjType type) {
    for (int i=0; i<(cNUM_OBJECT_TYPES+1)/8+1; i++) 
        m_data[i] = 0;
    Set(type);
}
//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CObjTypeMask 
// 	Default destructor has very little work to do.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline
CObjTypeMask::~CObjTypeMask()
{} // end of ~CObjTypeMask

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetAll
// 	Sets all the types in the type mask.
//
// Remarks: Following this call, the has() function will return true for all
//	valid lane identifiers.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CObjTypeMask::SetAll(void)
{
	for (int i=0; i<(cNUM_OBJECT_TYPES+1)/8+1; i++) 
		m_data[i] = 0xFF;
} // end of SetAll

//////////////////////////////////////////////////////////////////////////////
//
// Description: Set
// 	Adds a type to the mask.
//
// Remarks: 
// 	This member function adds the specified type in the list of types 
// 	contained in the mask.  The membership of the remaining types is not 
// 	affected.
//
// Arguments:
// 	type - which type to add to the mask
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CObjTypeMask::Set(cvEObjType type)
{
	if ( type <= cNUM_OBJECT_TYPES )
		m_data[type/8] |= (1 << (type % 8));	
} // end of Set


//////////////////////////////////////////////////////////////////////////////
//
// Description: Clear
// 	Remove all types from the mask
//
// Remarks: Leaves the mask empty.
// 
// Arguments:
// 
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CObjTypeMask::Clear(void)
{
	for (int i=0; i<(cNUM_OBJECT_TYPES+1)/8+1; i++) 
		m_data[i] = 0;
} // end of Clear

//////////////////////////////////////////////////////////////////////////////
//
// Description: Clear
// 	Remove a type from the mask
//
// Remarks: 
//	This function removes the specified type from the list of types contained 
//	in the mask.  If the type is invalid, the class remains unchanged
//
// Arguments:
//	type  - which type to remove from the mask
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
inline void 
CObjTypeMask::Clear(cvEObjType type)
{
	if ( type <= cNUM_OBJECT_TYPES )
		m_data[type/8] &= ~(1 << type%8);
} // end of Clear

//////////////////////////////////////////////////////////////////////////////
//
// Description: Has
// 	Find out if a type is in the mask
//
// Remarks: 
// 	If the type is invalid or is not part of the mask, the function returns 
// 	false otherwise it returns true.
//
// Arguments:
//    type - which type to consider
//
// Returns: true or false 
//
//////////////////////////////////////////////////////////////////////////////
inline bool 
CObjTypeMask::Has(cvEObjType type) const
{
	if ( type <= cNUM_OBJECT_TYPES )
		return (m_data[type/8] & (1 << (type % 8))) != 0;
	else
		return false;
} // end of Has

#endif // __OBJ_MASK_INL

