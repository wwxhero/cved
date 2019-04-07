/////////////////////////////////////////////////////////////////////////////
//
// Version: 		$Id: obj.h,v 1.35 2018/01/22 18:42:03 IOWA\dheitbri Exp $
//
// Obj.h: interface for the CObj class.
//
//////////////////////////////////////////////////////////////////////
#ifndef __OBJ_H
#define __OBJ_H

#include "cvedpub.h"

struct  cvTObj;
typedef struct cvTObj TObj;	// {secret}

namespace CVED {

class   CCved;

/////////////////////////////////////////////////////////////////////////////
//
// Description:
// This class represents a static object in the virtual environment.  
// It provides various access methods to the object's attribute's and 
// state.
//
// Before using an object, it must be bound to a CCved instance and
// associated with an object in the virtual environment.  An instance
// of this class that is bound but not associated with an object
// is called _invalid_.  Calling any of the access functions on an
// invalid object will throw an exception.  Use the isValid member function
// to determine if an object is valid.
// 
class CObj : public CCvedItem
{
public:
	CObj();
	virtual ~CObj();
	CObj(const CObj&);
	CObj& operator=(const CObj&);
	CObj(const CCved&);

	CObj(const CCved&, int);

	// public interface
	virtual bool			IsValid(void) const;
	virtual cvEObjType		GetType(void) const;
	int                     GetColorIndex(void) const;

	int						GetId(void) const;
	const char*				GetName(void) const;
	TS32b					GetSolId(void) const;
	const CSolObj*          GetSolObj(void) const;
	TS32b					GetHcsmId(void) const;
	double					GetXSize(void) const;
	double					GetYSize(void) const;
	double					GetZSize(void) const;
	const int*              GetCompositePieces(void) const;
	int                     GetNumCompositePieces(void) const;

	int						GetNumOfOptions(void) const;
	const vector<CSolOption>& GetAllOptions() const;	

    virtual COrientation    GetEulerAngles(void) const;
    virtual COrientation    GetEulerAnglesImm(void) const;
	virtual CPoint3D		GetPos(void) const;
    virtual CPoint3D		GetPosTarget(void) const;
    virtual double		    GetVelTarget(void) const;
    virtual double		    GetAccelTarget(void) const;
	virtual CPoint3D		GetPosImm(void) const;
	virtual CPoint3D		GetPosImmDelayCompensated(void) const;
	virtual CVector3D		GetTan(void) const;
	virtual CVector3D       GetTanImm(void) const;
	virtual CVector3D		GetLat(void) const;
	virtual CVector3D       GetLatImm(void) const;
	virtual CBoundingBox	GetBoundBox(void) const;
	virtual CBoundingBox	GetBoundBoxImm(void) const;
	virtual double          GetVel(void) const;
	virtual double          GetVelImm(void) const;
	virtual double          GetVelLat(void) const;
	virtual double          GetVelLatImm(void) const;
	virtual double          GetVelNorm(void) const;
	virtual double          GetVelNormImm(void) const;
	virtual double          GetRollRate(void) const;
	virtual double          GetRollRateImm(void) const;
	virtual double          GetPitchRate(void) const;
	virtual double          GetPitchRateImm(void) const;
	virtual double          GetYawRate(void) const;
	virtual double          GetYawRateImm(void) const;

	void					SetHcsmId(int id);

protected:
	CObj(const CCved&, TObj*);

	TObj*					m_pObj;
};


/////////////////////////////////////////////////////////////////////////////
//
// Inline functions
//
/////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////
//
// Description: constructors.
//
// Remarks:
// The various constructors initialize an object instance.  The 
// default constructor builds an unbound object.  When a CCved pointer 
// is provided, the class
// is initialized as bound, but is not associated with an actual object
// in the virtual environment.  The constructor with a string searches
// all static and dynamic objects for one whose name matches the argument.
// If none is found, the object is bound but not associated.
//
// The constuctor with an integer binds the object and associates
// it with an object instance based on the provided identifier.  If
// the identifier is invalid (i.e., does not refer to an existing object)
// then object will also be invalid.
//
// Arguments:
//  pCved - pointer to CCved instance in which the object is bound
//  name - the name of the object in the environment to associate with this
//         instance
//  copy - the copy of a class to initialize the current instance
//	id   - object identifier
//
inline
CObj::CObj() 
	: CCvedItem(0), 
	  m_pObj(0) 
{}

inline
CObj::CObj(const CCved& cCved) 
	: CCvedItem(&cCved), 
	  m_pObj(0) 
{}

inline
CObj::CObj(const CCved& cCved, TObj* pSlot) 
	: CCvedItem(&cCved), 
	  m_pObj(pSlot) 
{}

/////////////////////////////////////////////////////////////////////////////
//
// Description: check if an object is bound and valid.
//
// Remarks:
// The function provides information about an object's state.  The class
// instance is valid or invalid.  To be valid, an object should be
// bound to a virtual environment and associated with an object.
//
// Returns:
// If the class is bound to a virtual environment and it has been associated
// with an object, the function returns true else it returns false.
//
inline bool
CObj::IsValid(void) const
{
	if ( IsBound() && m_pObj != 0 )
		return true;
	else
		return false;
}


}		// namespace CVED

#endif // __OBJ_H

