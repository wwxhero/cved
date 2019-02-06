// Version: 		$Id: lane.h,v 1.39 2018/12/07 21:15:33 IOWA\dheitbri Exp $
//
// Lane.h: interface for the CLane class.
//
//////////////////////////////////////////////////////////////////////
#ifndef __LANE_H
#define __LANE_H

#include <randnumgen.h>
#include "cvedpub.h"

////////////////////////////////////////////////////////////////////////
//
// forward declarations for the lane structure.  We define them here
// so we can reference them but we don't include the actual 
// header file since it's private to CVED.
//
struct cvTLane;

namespace CVED {

class CCved;
typedef struct cvTLane TLane;

/////////////////////////////////////////////////////////////////////////
///
/// Lane class definition.
///
///
class CLane  : public CCvedItem
{
public:
	CLane();
	CLane(const CLane&);
	CLane&		operator=(const CLane& rhs);
	virtual		~CLane();
	explicit CLane(const CCved&); 

	CLane(const CCved&, const CRoad&, int);
	CLane(const CRoad&, int);
	CLane(const CCved&, int, int);
	CLane(const CCved&, TLane*);

	void		SetRandNumGen( CRandNumGen* pRng, int streamId );

	CLane		GetLeft(void) const;
	CLane		GetRight(void) const;
	bool		IsLeftMost(void) const;
	bool		IsRightMost(void) const;
    bool        IsRightMostAlongDir(void) const;
	bool        IsLeftMostAlongDir(void) const;
	bool		IsLeftOpposite(void) const;
    bool		IsRightOpposite(void) const;
	bool        GetOppositeLane(CLane&) const;
	void        GetOppositeLane( vector<CLane>& oppositeLanes, bool includeVehicleRestrictedLanes = false, bool includeTurnLanes = false ) const;
	bool		IsVehicleRestrictedLane(void) const;
	bool		IsTurnLane(void) const;
	bool		IsOnRamp(void) const;
    bool		IsOffRamp(void) const;
	bool		IsDrivingLane(void) const;
    bool        IsInterstate(void) const;
	double		GetWidth(double dist = 0.0) const;
	double		GetOffset(double dist = 0.0) const;
	TU8b		GetId(void) const;
	TU32b		GetIndex(void) const;
	TU8b		GetRelativeId(void) const;
	cvELnDir	GetDirection(void) const;

	string		GetName(void) const;
	CRoad		GetRoad(void) const;

	CIntrsctn   GetNextIntrsctn() const;
	CIntrsctn   GetPrevIntrsctn() const;

	double		GetLateralDistance(const CLane&) const;

	bool		operator==(const CLane&);
	bool		IsValid(void) const;

	friend ostream& operator<<(ostream&, const CLane&);

private:
	int			Search(double dist = 0.0) const; //get the control point index
												//with respect to the road

protected:
	void		AssertValid(void) const;

private:
	TLane*		m_pLane;

	// random nubmer generator
	CRandNumGen*    m_pRng;
	int             m_rngStreamId;
};

////////////////////////////////////////////////////////////////////////////
// Inline member functions.
////////////////////////////////////////////////////////////////////////////

inline
CLane::CLane() 
	: 
	CCvedItem(0), 
	m_pLane(0),
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{}

inline
CLane::~CLane() 
{}

inline
CLane::CLane(const CCved& cCved) 
	: 
	CCvedItem(&cCved), 
	m_pLane(0) ,
	m_pRng( NULL ),
	m_rngStreamId( -1 )
{}

inline void
CLane::AssertValid(void) const
{
	CCvedItem::AssertValid();
	if( m_pLane == 0 )
	{
		string msg = "CLane::AssertValid: null lane";
		cvCInternalError e(msg, __FILE__, __LINE__);
		throw e;
	}
}

///////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is valid
// 
// Remarks: This function may be used by outside elements which want to insure
// 	that the current instance is valid, without risking a failed assertion.
//
// Returns: true or false  
// 	
inline bool
CLane::IsValid(void) const
{
	return (CCvedItem::IsValid() && (m_pLane != 0) );
}

}		// namespace CVED

#endif // __LANE_H

