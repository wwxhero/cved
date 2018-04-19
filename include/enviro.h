
/**********************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of
 *     Iowa.  All rights reserved.
 *
 * Version:         $Id: enviro.h,v 1.5 2013/05/08 15:17:50 IOWA\vhorosewski Exp $ *
 * Author(s):  Lijen Tsao
 * Date:       Januaray, 2000
 *
 * Description: The declaration of the CEnvCndtn
 *
 **************************************************************************/
#ifndef __ENVIRO_CONDITION_H
#define __ENVIRO_CONDITION_H

#include "cvedpub.h"

struct cvTEnviroArea;

namespace CVED {

class CEnvArea : public CCvedItem{
public:
	CEnvArea();
	CEnvArea(const CEnvArea&);
	CEnvArea(const CCved&, int);
	explicit CEnvArea(const CCved&);
	CEnvArea& operator=(const CEnvArea&);
	virtual ~CEnvArea();

	void                  SetEnvCndtn(vector<cvTEnviroInfo>&);
	void                  GetCndtns(vector<cvTEnviroInfo>&) const;

	bool                  Enclose(CPoint3D&) const;
	bool                  Enclose(CPoint2D&) const;
	bool                  Enclose(double, double) const;

	CPoint2D              GetOrigin(void) const;

	void                  DumpEnvArea(void) const;
	void                  DumpEnvArea(vector<cvTEnviroInfo>&) const;

	virtual bool IsValid(void) const;

protected:
	void                  AssertValid(void) const;

private:
	cvTEnviroArea*        m_pEnvArea;
	CPolygon2D            m_area;
	cvTEnviroInfo*        m_pEnvInfo;
};

///////////////////////////////////////////////////////////////////////////////
//
// Description: Indicates whether the current instance is valid
//
// Remarks: This function may be used by outside elements which want to insure
//  that the current instance is valid, without risking a failed assertion.
//
// Returns: true or false
//
///////////////////////////////////////////////////////////////////////////////
inline bool
CEnvArea::IsValid(void) const
{
    return CCvedItem::IsValid() && m_pEnvArea != 0;
}

/////////////////////////////////////////////////////////////////////////////
//
// {secret}
// Description: verify the class is valid
// Remarks:
// This is a "guard" function that should be called before the
// class touches its internal data structures.  It verifies that
// the object is bound and has a valid pointer.
//
///////////////////////////////////////////////////////////////////////////////
inline void
CEnvArea::AssertValid(void) const
{
    CCvedItem::AssertValid();
    if ( m_pEnvArea == 0 )
        assert(0);
}



}      // namespace CVED
#endif // #ifdef __ENVIRO_CONDITION_H
