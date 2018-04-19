//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 	$Id: cveditem.cxx,v 1.18 2004/04/07 20:01:41 oahmad Exp $
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// Description: implementation of the CCvedItem class.
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
//
// Description: CCvedItem
// 	Constructor that takes a pointer to a CCved instance
//
// Remarks:  
//
// Arguments:
// 	cpCved - const pointer to a CCved instance
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CCvedItem::CCvedItem(const CCved* cpCved) 
	: m_cpCved(cpCved)
{} // end of CCvedItem

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetInst
// 	Return the bound CCved header associated with the current instance.
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	unbound (invalid) CCvedItem instance.
//
// Arguments:
// 
// Returns: The function returns a pointer to the CCved instance, or 0 if the 
// 	class is not bound.
//
//////////////////////////////////////////////////////////////////////////////
void*
CCvedItem::GetInst(void) const 
{ 
	CCvedItem::AssertValid();
	return m_cpCved->m_pHdr; 
} // end of GetInst

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetInst
// 	Return the bound CCved header associated with the current instance.
//
// Remarks: 
//
// Arguments:
// 
// Returns: The function returns a pointer to the CCved instance, or 0 if the 
// 	class is not bound.
//
//////////////////////////////////////////////////////////////////////////////
void*
CCvedItem::GetInst(const CCved* pCved) const
{
	return pCved->m_pHdr;
} // end of GetInst

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetCved
// 	Return the bound CCved instance associated with the current instance.
//
// Remarks: This function will cause a failed assertion if it is run on an
// 	unbound (invalid) CCvedItem instance.
//
// Arguments:
// 
// Returns: The function returns a pointer to the CCved instance, or 0 if the 
// 	class is not bound.
//
//////////////////////////////////////////////////////////////////////////////
const CCved&
CCvedItem::GetCved(void) const 
{ 
	CCvedItem::AssertValid();
	return *m_cpCved; 
} // end of GetCved

//////////////////////////////////////////////////////////////////////////////
//
// Description: SetCved
// 	associate the given CCved instance with this item
//
// Remarks: 
//
// Arguments:
// 
// Returns: The function returns a pointer to the CCved instance, or 0 if the 
// 	class is not bound.
//
//////////////////////////////////////////////////////////////////////////////
void
CCvedItem::SetCved(const CCved* pCved)
{ 
	m_cpCved = pCved; 
} // end of GetCved

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindObjAttr (protected)
// 	Binds the indicated object attribute to a cvTObjAttr structure.
//
// Remarks: Used by some of the subclasses of CCvedItem to access the 
// 	attributes in the current CCved instance.
//
//	This function will cause a failed assertion if it is run on an unbound 
//	(invalid) CCvedItem instance.
//
// Arguments:
// 	slot - identifier of the desired object attribute
//
// Returns: A pointer to the cvTObjAttr structure associated with slot.
//
//////////////////////////////////////////////////////////////////////////////
struct cvTObjAttr*
CCvedItem::BindObjAttr(int slot) const
{
	cvTObj*	pObj = BindObj( slot );
	return &pObj->attr;
} // end of BindObjAttr

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindObj (protected)
// 	Binds the indicated object to a cvTObj structure.
//
// Remarks: Used by some of the subclasses of CCvedItem to access the 
// 	objects in the current CCved instance.
//
//	This function will cause a failed assertion if it is run on an unbound 
//	(invalid) CCvedItem instance.
//
// Arguments:
// 	slot - identifier of the desired object 
//
// Returns: A pointer to the cvTObj structure associated with slot.
//
//////////////////////////////////////////////////////////////////////////////
struct cvTObj*
CCvedItem::BindObj(int slot) const
{
	CCvedItem::AssertValid();

	cvTHeader*	pH			= static_cast<cvTHeader*>(m_cpCved->m_pHdr);
	TU32b		ofs			= pH->objectOfs;
	char*		pObjPool	= reinterpret_cast<char*>(m_cpCved->m_pHdr) + ofs;
	cvTObj*		pObj		= reinterpret_cast<cvTObj*>(pObjPool);

	return pObj + slot;
} // end of BindObj

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindCntrlPnt (protected)
// 	Return a pointer to the indicated control point.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CCvedItem instance.
//
// Arguments:
// 	id - index of the control point
//
// Returns:
// 	Pointer to the indicated control point.
//
//////////////////////////////////////////////////////////////////////////////
struct cvTCntrlPnt*
CCvedItem::BindCntrlPnt(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) + 
						pH->longitCntrlOfs;
	cvTCntrlPnt*pCp  = reinterpret_cast<TCntrlPnt*>(pOfs) + id;

	return pCp;
} // end of BindCntrlPnt

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindCrdrPnt (protected)
// 	Return a pointer to the indicated corridor control point.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CCvedItem instance.
//
// Arguments:
// 	id - index of the control point
//
// Returns:
// 	Pointer to the indicated control point.
//
//////////////////////////////////////////////////////////////////////////////
struct cvTCrdrCntrlPnt*
CCvedItem::BindCrdrPnt(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) +
						pH->crdrCntrlPntOfs;
	cvTCrdrCntrlPnt* pCp  = reinterpret_cast<TCrdrCntrlPnt*>(pOfs) + id;

	return pCp;
} // end of BindCrdrPnt

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindRoadPiece (protected)
// 	Return a pointer to the indicated road piece.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CCvedItem instance.
//
// Arguments:
// 	id - index of the road piece
//
// Returns: Pointer to the indicated road piece. 
//
//////////////////////////////////////////////////////////////////////////////
struct cvTRoadPiece*
CCvedItem::BindRoadPiece(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) +
	   					pH->roadPieceOfs;
	cvTRoadPiece*	p  = reinterpret_cast<TRoadPiece*>(pOfs) + id;

	return p;
} // end of BindRoadPiece

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindRoad (protected)
// 	Return a pointer to the indicated road.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CCvedItem instance.
//
// Arguments:
// 	id - index of the road 
//
// Returns: Pointer to the indicated road. 
//
//////////////////////////////////////////////////////////////////////////////
struct cvTRoad*
CCvedItem::BindRoad(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) + 
						pH->roadOfs;
	cvTRoad*	p  = reinterpret_cast<TRoad*>(pOfs) + id;

	return p;
} // end of BindRoad

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindLane (protected)
// 	Return a pointer to the indicated lane.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CLanePos instance.
//
// Arguments:
// 	id - index of the lane 
//
// Returns: Pointer to the indicated lane. 
//
//////////////////////////////////////////////////////////////////////////////
struct cvTLane*
CCvedItem::BindLane(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) + 
						pH->laneOfs;
	cvTLane*	p  = reinterpret_cast<TLane*>(pOfs) + id;

	return p;
} // end of BindLane

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindIntrsctn (protected)
// 	Return a pointer to the indicated intrsctn.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CCvedItem instance.
//
// Arguments:
// 	id - index of the intrsctn 
//
// Returns: Pointer to the indicated intrsctn. 
//
//////////////////////////////////////////////////////////////////////////////
struct cvTIntrsctn*
CCvedItem::BindIntrsctn(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) +
	   					pH->intrsctnOfs;
	cvTIntrsctn*p  = reinterpret_cast<TIntrsctn*>(pOfs) + id;

	return p;
} // end of BindIntrsctn

//////////////////////////////////////////////////////////////////////////////
// 
// Description: BindCrdr (protected)
// 	Return a pointer to the indicated crdr.  
//
// Remarks: This function causes a failed assertion if it is called on an
// 	invalid CCrdrPos instance.
//
// Arguments:
// 	id - index of the crdr 
//
// Returns: Pointer to the indicated crdr. 
//
//////////////////////////////////////////////////////////////////////////////
struct cvTCrdr*
CCvedItem::BindCrdr(int id) const
{
	THeader*	pH   = static_cast<THeader*>(m_cpCved->m_pHdr);
	char*		pOfs = reinterpret_cast<char*>(m_cpCved->m_pHdr) +
	   					pH->crdrOfs;
	cvTCrdr*	p  = reinterpret_cast<TCrdr*>(pOfs) + id;

	return p;
} // end of BindCrdr

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetIntrsctnGrid
// 	Returns the terrain grid associate with the given intersection id.  This
// 	data is stored within CCved.
//
// Remarks: Note that the resulting pointer might be NULL, if the given 
// 	intersection has no TerrainGrid associated with it.
//
// Arguments:
// 	intrsctnId - identifier of the intersection of interest
//
// Returns: A const pointer to the CTerrainGrid instance stored in CCved.
//
//////////////////////////////////////////////////////////////////////////////
const CTerrainGrid<Post>*
CCvedItem::GetIntrsctnGrid(int intrsctnId) const
{
	return m_cpCved->m_intrsctnGrids[intrsctnId];
} // end of GetIntsctnGrid
} // namespace CVED
