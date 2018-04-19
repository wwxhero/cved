

/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: intrsctn.h,v 1.36 2016/01/05 22:58:28 IOWA\dheitbri Exp $
//
// Author(s):	
// Date:		September, 1998
//
// Description:	The declaration of the CIntrsctn class.
//
//////////////////////////////////////////////////////////////////////
#ifndef __INTRSCTN_H
#define __INTRSCTN_H	// {secret}

#include "cvedpub.h"

////////////////////////////////////////////////////////////////////////
//
// forward declarations for the intersectin structure.  We define them here
// so we can reference them but we don't include the actual 
// header file since it's private to CVED.
//
struct cvTIntrsctn;
struct cvTElevMap;
struct cvTElevPost;
struct cvTBorderSeg;

namespace CVED {

typedef struct cvTIntrsctn TIntrsctn;	// {secret}
typedef struct cvTElevMap TElevMap;		// {secret}
typedef struct cvTElevPost TElevPost;   // {secret}
typedef struct cvTBorderSeg TBorderSeg; // {secret}
typedef struct 
{
	vector<int> intrsctingCrdrs;
	double       hldOfsDist;
} TCrdrPriorityList;

//
// This class represents an intersection in the virtual environment
// represented by an instance of the CCved class.  Intersections
// provide the only way to travel between roads.  Each intersection
// is adjacent to one or more roads and contains zero or more
// corridors.  Corridors allow directed travel between specific
// lanes on adjacent roads.
//
// The class provides a variety of convenience functions for 
// accessing all associated information.
//
class CIntrsctn : public  CCvedItem 
{
public:
	CIntrsctn();
	CIntrsctn( const CIntrsctn& );
	CIntrsctn& operator=( const CIntrsctn& );
    bool operator==( const CIntrsctn& );
	virtual ~CIntrsctn();

	explicit CIntrsctn( const CCved& );
	CIntrsctn( const CCved&, const string& );
	CIntrsctn( const CCved&, TU32b id );

	string		GetName( void ) const;
	int			GetCrdrIdx( void ) const;
	int			GetNumCrdrs( void ) const;

	void		GetRoads( vector<CRoad>& ) const;
	void		GetAllCrdrs( TCrdrVec& ) const;
	void		GetCrdrs( const CLane&, TCrdrVec& ) const;
	void		GetCrdrs( const CLane&, TCrdrVec&, TRoadVec& ) const;

	CCrdr		GetCrdr( const CLane&, const CLane& ) const;

	void		GetCrdrsLeadingTo( const CRoad&, TCrdrVec& ) const;
	void		GetCrdrsLeadingTo( const CLane&, TCrdrVec& ) const;
    void		GetCrdrsControlledByLight( int TrafLightId, TCrdrVec& ) const;

	void        GetCrdrsStartingFrom( const CRoad&, TCrdrVec& ) const;
	void        GetCrdrsStartingFrom( const CLane&, TCrdrVec& ) const;

	void		GetRoadAccessList( const CRoad&, TRoadVec& ) const;

	void		GetRoadAccessListThrghIntrsctn( const CRoad&, TRoadVec& ) const;
	void		GetRoadAccessListThrghIntrsctn( const CRoad&, const bitset<cCV_MAX_CRDRS>&, TRoadVec& ) const;
	void		GetReverseRoadAccessListThrghIntrsctn( const CRoad&, TRoadVec& ) const;

	bool		DoRoadsConnect( const CRoad&, const CRoad& ) const;

	void		GetBorder( CPolygon2D& ) const;

	int			GetId( void ) const;

	bool		QryAttr( int, CAttr& ) const;
	
	bool		QryAttr( vector<CAttr>& ) const;

	void		PrioritizeCorridors( 
						vector<TCrdrPriorityList>& out 
						) const;
	virtual bool IsValid( void ) const;
	bool        IsTwoRoad() const;
    bool        IsControlled() const;
protected:
	void		AssertValid( void ) const;

private:
	TIntrsctn*	m_pIntrs;
};

typedef vector<CIntrsctn> TIntrsctnVec;

/////////////////////////////////////////////////////////////////////////////
// Inline functions
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: Default constructor.
//
// Remarks:
// 	Initializes an invalid CIntrsctn instance.
//
/////////////////////////////////////////////////////////////////////////////
inline
CIntrsctn::CIntrsctn() 
	: CCvedItem(0), 
	  m_pIntrs(0) 
{}

/////////////////////////////////////////////////////////////////////////////
//
// Description:  Constructo that takes a CCved instance
//
// Remarks: Initializes the superclass with the CCved instance, but leaves
// 	the CIntrsctn invalid, because no intersection was specified.
//
// Arguments:
// 	cCved - CCved instance to initialize the CCvedItem
//
/////////////////////////////////////////////////////////////////////////////
inline
CIntrsctn::CIntrsctn(const CCved& cved) 
	: CCvedItem(&cved), 
	  m_pIntrs(0) 
{}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Copy constructor
//
// Remarks:
// 	Performs a deep copy from the parameter to the current instance
// 	
// Arguments:
//  cCopy - the copy of a class to initialize the current instance
//
///////////////////////////////////////////////////////////////////////////
inline
CIntrsctn::CIntrsctn(const CIntrsctn& cCopy)
{
	*this = cCopy;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Destructor
//
// Remarks: Does nothing.
//
/////////////////////////////////////////////////////////////////////////////
inline
CIntrsctn::~CIntrsctn()
{}

///////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is valid
// 
// Remarks: This function may be used by outside elements which want to insure
// 	that the current instance is valid, without risking a failed assertion.
//
// Returns: true or false  
// 	
///////////////////////////////////////////////////////////////////////////////
inline bool
CIntrsctn::IsValid(void) const
{
	return CCvedItem::IsValid() && m_pIntrs != 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// {secret}
// Description: verify the class is valid
// Remarks:
// This is a "guard" function that should be called before the
// class touches its internal data structures.  It verifies that
// the object is bound and has a valid pointer to a road in the
// virtual environment.
//
///////////////////////////////////////////////////////////////////////////////
inline void
CIntrsctn::AssertValid(void) const
{
	CCvedItem::AssertValid();
	if ( m_pIntrs == 0 )
	{
		string msg = "CIntrsctn::AssertValid: null intersection";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}
}

}		// namespace CVED

#endif	// #ifdef __INTRSCTN_H
