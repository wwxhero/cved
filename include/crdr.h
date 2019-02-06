/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: $Id: crdr.h,v 1.40 2018/12/07 21:16:41 IOWA\dheitbri Exp $
//
// Author(s): 
// Date:       September, 1998
//
// Description:	The definition of the CCrdr class
//
/////////////////////////////////////////////////////////////////////////////
#ifndef __CRDR_H
#define __CRDR_H	// {secret}

#include "cvedpub.h"
#include <pi_string>
using namespace std;

struct cvTIntrsctn;
struct cvTCrdr;
struct cvTCrdrCntrlPnt;

namespace CVED {

typedef struct cvTIntrsctn		TIntrsctn;	// {secret}
typedef struct cvTCrdr			TCrdr;		// {secret}
typedef struct cvTCrdrCntrlPnt	TCrdrCntrlPnt;
typedef vector<CPoint3D>		TCPoint3DVec;

class CIntrsctn;

///////////////////////////////////////////////////////////////////
///\class CCrd
///\brief
///		This class represents a corridor
///\remark
///		This class represents a corridor.  Corridors allow traveling
///		between roads that are adjacent to an intersection.
///		Corridors have associated with them traffic control objects 
///		and hold offsets.  Hold ofsets indicate how far along
///		a corridor autonomous vehicles have to stop to yield to
///		other entities that are crossing on conflicting corridors.
/////////////////////////////////////////////////////////////////
class CCrdr : public CCvedItem {
public:
	struct TCntrlPnt{
		CPoint2D		location;
		TVector2D		rightVecLinear;
		double			width;
		double			distance;
		unsigned char	flagStyle;	/* OR operation with eLEFT_FLAG, */
									/* eLEFT_DOTTED, eRIGHT_FLAG, or */
									/* eRIGHT_DOTTED, eNO_FLAG */
	};

	typedef enum { eLEFT = -1, eSTRAIGHT = 0, eRIGHT = 1} ECrdrDirection;
	typedef enum { 
				eNONE = -1, 
				eTOTHELEFT = 0,
				eNOTLEFTNORRIGHT = 1, 
				eTOTHERIGHT = 2 
				} ECrdrGeometry;
	typedef enum {
				eUNCONTROLLED = 1,
				eLIGHT_FLASH_GREEN_STRAIGHT = 2,
			    eLIGHT_FLASH_GREEN_TURN_RIGHT = 2,
			    eLIGHT_FLASH_GREEN_TURN_LEFT = 2,
				eLIGHT_GREEN_STRAIGHT = 2,
			    eLIGHT_GREEN_TURN_RIGHT = 2,
			    eLIGHT_GREEN_TURN_LEFT = 2,
				eLIGHT_GREEN = 3,
                eLIGHT_FLASH_GREEN = 3,
				eYIELD = 4,
				eLIGHT_FLASH_YELLOW_STRAIGHT = 4,
			    eLIGHT_FLASH_YELLOW_TURN_RIGHT = 4,
			    eLIGHT_FLASH_YELLOW_TURN_LEFT = 4,
				eLIGHT_FLASH_YELLOW = 4,
				eLIGHT_YELLOW_STRAIGHT = 5,
			    eLIGHT_YELLOW_TURN_RIGHT = 5,
			    eLIGHT_YELLOW_TURN_LEFT = 5,
				eLIGHT_YELLOW = 6,
				eSTOP = 7,
				eLIGHT_FLASH_RED_STRAIGHT = 7,
			    eLIGHT_FLASH_RED_TURN_LEFT = 7,
			    eLIGHT_FLASH_RED_TURN_RIGHT = 7,
				eLIGHT_FLASH_RED = 7,
				eLIGHT_RED_STRAIGHT = 8,
			    eLIGHT_RED_TURN_LEFT = 8,
			    eLIGHT_RED_TURN_RIGHT = 8,
				eLIGHT_RED = 8
				} ECrdrPriority;

	typedef vector<TCntrlPnt> TCntrlPntVec;
	void		GetCntrlPnts(TCntrlPntVec&) const;
	void		GetBoundaryPnts(TCPoint3DVec&) const;
 
	CCrdr();
	CCrdr(const CCrdr&);
	CCrdr&		operator=(const CCrdr&);
	virtual		~CCrdr();

	explicit CCrdr(const CCved&);
 	CCrdr(const CCved&, TCrdr*);
	CCrdr(const CIntrsctn&, const CLane&, const CLane&);
	CCrdr(const CCved&, int);
	CCrdr(const CIntrsctn&, int);

	void		GetAllHldOfs(vector<CHldOfs>&) const; 
	bool		GetHldOfByDist( CHldOfs& hldofs, double dist= -1.0 ) const;
	double       GetFirstHldOfsDist() const;
	int 		GetIntrsctnId(void) const;
	int			GetSrcLnIdx(void) const;
	int			GetDstntnLnIdx(void) const;
	int			GetSrcRdIdx(void) const;
	int			GetDstntnRdIdx(void) const;

	CIntrsctn	GetIntrsctn(void) const;
	CLane		GetSrcLn(void) const;
	CLane		GetDstntnLn(void) const;
	CRoad		GetSrcRd(void) const;
	CRoad		GetDstntnRd(void) const;

	int			GetId(void) const;
	int			GetRelativeId(void) const;
	double		GetLength(void) const;

	bool        GetLeftCrdrAlongDir(CCrdr&) const;

	void		GetMrgDstFirst(double& dist, int& crdrId) const;
	void		GetMrgDstLast(double& dist, int& crdrId) const;
	void		GetMrgDstFirst(vector<double>& dists) const;
	double		GetFirstMrgDist( int crdrId ) const;
	double		GetLastMrgDist( int crdrId ) const;
	void		GetMrgDstLast(vector<double>& dists) const;
	void		GetIntersectingCrdrs( vector<int>& out ) const;

	ECrdrPriority GetCrdrPriority( void ) const;
	ECrdrDirection GetCrdrDirection( void ) const;
	double      GetCrdrDirectionK( void ) const;
	ECrdrGeometry DecideCrdrByGeometry( const CCrdr& otherCrdr ) const;
	string      GetCrdrDirectionStr( void );
	string      DirectionToStr( const ECrdrDirection ) const;
	string      GeometryToStr( const ECrdrGeometry ) const;

	void		QryAttr(vector<CAttr>& attrs) const;
	double		GetWidth( double distance ) const;


    bool		IsOnRamp(void) const;
    bool		IsOffRamp(void) const;
    bool        IsInterstate(void) const;

	string      GetString( void ) const;
	bool        SetString( const string& );

	bool		IsValid(void) const;

protected:
	void		AssertValid(void) const;

private:	
	TCrdr*		m_pCrdr;
};

ostream& operator<<(ostream &, const CCrdr &);

typedef vector<CCrdr> TCrdrVec;

/////////////////////////////////////////////////////////////////////////////
//
// Inline implementations.
//

//////////////////////////////////////////////////////////////////////////
///\brief  default constructor
///
///\remark
///		The default constructor creates an unbound and unitialized
///		object.  The copy constructor makes a copy of the provided
///		object.
///
/// 
///\param	copy - the object to duplicate.
///\param	intrsctn - the containing intersection
///\param	src - source lane
///\param	dst - destination lane
/////////////////////////////////////////////////////////////////////////
inline
CCrdr::CCrdr() 
	: CCvedItem(0), m_pCrdr(0) 
{}

inline
CCrdr::CCrdr(const CCrdr &cCopy)
{
	*this = cCopy;
}

inline
CCrdr::~CCrdr()
{}

inline
CCrdr::CCrdr(const CCved& cCved) 
	: CCvedItem(&cCved), m_pCrdr(0) 
{}

inline
CCrdr::CCrdr(const CCved& cCved, TCrdr* pCrdr) 
	: CCvedItem(&cCved), m_pCrdr(pCrdr) 
{}

/////////////////////////////////////////////////////////////////////////////
//
// {secret}
// Description: verify the class is valid
//
// Remarks:
// This is a "guard" function that should be called before the
// class touches its internal data structures.  It verifies that
// the object is bound and has a valid pointer to a road in the
// virtual environment.
//
/////////////////////////////////////////////////////////////////////////////

inline void
CCrdr::AssertValid(void) const
{
    CCvedItem::AssertValid();
    if( m_pCrdr == 0 )
	{
		string msg = "CCrdr::AssertValid: null crdr";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is valid
// 
// Remarks: This function may be used by outside elements which want to insure
// 	that the current instance is valid, without risking a failed assertion.
//
// Returns: true or false  
// 	
inline bool 
CCrdr::IsValid(void) const
{
    return (CCvedItem::IsValid() && (m_pCrdr != 0) );
}

}	// namespace

#endif	// #ifdef __CRDR_H

