//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: enviro.cxx,v 1.11 2013/12/03 20:34:28 IOWA\dheitbri Exp $
//
// Author(s):   Lijen Tsao
// Date:        Feb, 2000
//
// Description: The implementation of the CEnvArea class
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"        // all needed for the class
#include "cvedstrc.h"       // private CVED data structs

// for Docjet to recognize the namespace
/*
using namespace CVED;
*/
namespace CVED{

//////////////////////////////////////////////////////////////////////////////
//
// Description: Default constructor
//
// Remarks:
//  Initializes an invalid CIntrsctn instance.
//
// Arguments:
//
//////////////////////////////////////////////////////////////////////////////
CEnvArea::CEnvArea() : CCvedItem(0), m_pEnvArea(0),m_pEnvInfo(0)
{}

/////////////////////////////////////////////////////////////////////////////
//
// Description:  Constructor that takes a CCved instance
//
// Remarks: Initializes the superclass with the CCved instance, but leaves
//  the CEnvArea invalid, because no envarea was specified.
//
// Arguments:
//  cCved - CCved instance to initialize the CCvedItem
//
/////////////////////////////////////////////////////////////////////////////
CEnvArea::CEnvArea(const CCved& cved) : CCvedItem(&cved), m_pEnvArea(0),m_pEnvInfo(0)
{}

//////////////////////////////////////////////////////////////////////////////
//
// Description: CEnvArea constructor
//  Constructor that takes a CCved instance and an id.
//
// Remarks:
//
// Arguments:
//  cCved - const reference to a CCved instance
//  id   - id of envArea
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CEnvArea::CEnvArea(const CCved& cCved, int id)
    : CCvedItem(&cCved)
{
    cvTHeader*  pH   = static_cast<cvTHeader*> (GetInst());
    char*       pOfs = static_cast<char*>  (GetInst()) + pH->envAreaOfs;
    m_pEnvArea       = reinterpret_cast<cvTEnviroArea*>(pOfs) + id;

	pOfs     = static_cast<char*>  (GetInst()) + pH->envInfoOfs;
	m_pEnvInfo = reinterpret_cast<cvTEnviroInfo*>(pOfs) + m_pEnvArea->infoIdx;

	int i = 0;
	for (; i<m_pEnvArea->numOfPolyPts; i++){
		m_area.AddPoint(m_pEnvArea->polyPt[i]);
	}
} // end of CIntrsctn




//////////////////////////////////////////////////////////////////////////////
//
// Description: Copy constructor
//
// Remarks:
//  Performs a deep copy from the parameter to the current instance
//
// Arguments:
//  r - the copy of a class to initialize the current instance
//
//////////////////////////////////////////////////////////////////////////////
CEnvArea::CEnvArea(const CEnvArea& copy)
{
	*this = copy;
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: operator=
//  Perform a deep copy of the parameter to the current instance.
//
// Remarks:
//
// Arguments:
//  c - a reference to an object intended to be to the right of the =
//
// Returns: a reference to the current instance
//
/////////////////////////////////////////////////////////////////////////////
CEnvArea&
CEnvArea::operator=(const CEnvArea& r)
{
	if ( this != &r){
		this->CCvedItem::operator=(r);

		m_area = r.m_area;
		m_pEnvArea = r.m_pEnvArea;
		m_pEnvInfo = r.m_pEnvInfo;
	}

	return *this;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description: Destructor
//
// Remarks: Does nothing.
//
/////////////////////////////////////////////////////////////////////////////
CEnvArea::~CEnvArea()
{}

//////////////////////////////////////////////////////////////////////////////
//
// Description:SetEnvCndtn
//  Set the environmental information of this environment area
//
// Remarks: This function takes as parameter a vector of environmental
//  informaiotn, and write them to appropriate place in memory pool
//
// Arguments:
//  info - of type vector<cvTEnviroInfo> as the value to be set
//
// Return value: 
//
//////////////////////////////////////////////////////////////////////////////
void
CEnvArea::SetEnvCndtn(vector<cvTEnviroInfo>& info)
{
    cvTHeader* pH = static_cast<cvTHeader*> (GetInst());
	int preSize = m_pEnvArea->numOfInfo + ( (int) info.size() );
	
	if (preSize >= (int)pH->envInfoSize){
		cvCInternalError err(
				"over the limit of envinfo pool",
				__FILE__,
				__LINE__);
		throw err;
	}
	if (preSize > m_pEnvArea->maxNumOfInfo) {
		cvCInternalError err(
				"over the limit of number of envinfo an envArea can have",
				__FILE__,
				__LINE__);
		throw err;
	}

	int i = m_pEnvArea->numOfInfo;
	for (; i<preSize; i++){
		m_pEnvInfo[i] = info[i - m_pEnvArea->numOfInfo];
	}

	m_pEnvArea->numOfInfo = preSize; 
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: 
//  return the all of the environmental information of this environment area
//
// Remarks: This function takes as a pamameter vector of type cvTEnviroInfo,
//  and copy the whole environmental information into it. 
//
// Arguments:
//  info - of type vector<cvTEnviroInfo> as a place holder holding the whole
//         environmental information
//
// Return value: 
//
//////////////////////////////////////////////////////////////////////////////
void 
CEnvArea::GetCndtns(vector<cvTEnviroInfo>& info) const
{
	info.clear();
	int i = 0;
	for(; i<m_pEnvArea->numOfInfo; i++){
		info.push_back(m_pEnvInfo[i]);
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: 
//  This function returns if the point is inside the area
//
// Remarks: This function takes as a pamameter point and check if the point is
//  in side the polygon the the environment area.
//
// Arguments:
//  pt - a 2d or 3d point 
//
// Return value: 
//   bool to indicate if the point is inside the environment area
//
//////////////////////////////////////////////////////////////////////////////
bool
CEnvArea::Enclose(CPoint3D& pt) const
{
	return m_area.Contains(pt.m_x, pt.m_y);
}

bool
CEnvArea::Enclose(CPoint2D& pt) const
{
	return m_area.Contains(pt);
}

bool
CEnvArea::Enclose(double x, double y) const
{
	return m_area.Contains(x, y);
}

//////////////////////////////////////////////////////////////////////////////
//
// Description: 
//  This function returns the origin of the environment area
//
// Remarks: 
//
// Arguments:
//
// Return value: 
//  a 2d point of the origin of the environment area
//
//////////////////////////////////////////////////////////////////////////////
CPoint2D
CEnvArea::GetOrigin(void) const
{
	return m_pEnvArea->origin;
}


//////////////////////////////////////////////////////////////////////////////
//
// Description:
//  dump the whole environmental information of specific type
//
// Remarks: This function simply print out the contents of the
//          environmental area
//
// Arguments:
//
// Return value:
//
//////////////////////////////////////////////////////////////////////////////
void
CEnvArea::DumpEnvArea(void) const
{


	gout<<"\n============ Dump EnvArea Info ================";
	if (m_area.GetSize() == 0){
		gout<<"\n This is global environment";
	} else {
		gout<<"\n This is local environment";
		gout<<"\n origin "<<GetOrigin();
		gout<<"\n area ";
		int p = 0;
		for(; p<m_area.GetSize(); p++){
			gout<<m_area.GetPoint(p);
		}
	}

	int num = m_pEnvArea->numOfInfo;
	gout<<"\n there are "<<num<<" envInfos";
	int i   = 0;
	for (; i<num; i++){
		switch(m_pEnvInfo[i].type){
			case eLIGHTNING:
				gout<<"\nLightning degree : \t";
				gout<<cvLMHScaleToString(m_pEnvInfo[i].info.Lightning.degree);
				break;
			case eVISIBILITY:
				gout<<"\nVisibility dist : \t";
				gout<<m_pEnvInfo[i].info.Visibility.dist;
				break;
			case eHAZE:
				gout<<"\nHaze dist : \t";
				gout<<m_pEnvInfo[i].info.Haze.dist;
				break;
			case eFOG:
				gout<<"\nFog dist :  \t";
				gout<<m_pEnvInfo[i].info.Fog.dist;
				break;
			case eSMOKE:
				gout<<"\nSmoke degree : \t";
				gout<<m_pEnvInfo[i].info.Smoke.degree;
				break;
			case eCLOUDS:
				gout<<"\nClouds altitude : \t";
				gout<<m_pEnvInfo[i].info.Clouds.altitude;
				gout<<"\t type : \t"<<m_pEnvInfo[i].info.Clouds.type;
				break;
			case eGLARE:
				gout<<"\nGlare degree : \t";
				gout<<m_pEnvInfo[i].info.Glare.degree;
				break;
			case eSNOW:
				gout<<"\nSnow degree : \t";
				gout<<cvLMHScaleToString(m_pEnvInfo[i].info.Snow.degree);
				break;
			case eRAIN:
				gout<<"\nRain degree : \t";
				gout<<cvLMHScaleToString(m_pEnvInfo[i].info.Rain.degree);
				break;
			case eWIND:
				gout<<"\nWind dir : \t";
				gout<<m_pEnvInfo[i].info.Wind.dir_i<<" ";
				gout<<m_pEnvInfo[i].info.Wind.dir_j;
				gout<<"\tvel : \t"<<m_pEnvInfo[i].info.Wind.vel;
				gout<<"\tgust : \t"<<m_pEnvInfo[i].info.Wind.gust;
				break;
			default:
				gout<<"\nThe type is: "<<m_pEnvInfo[i].type<<endl;
		}
	}
	gout<<endl;
}


} // namespace CVED
