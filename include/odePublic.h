#ifndef __ODE_PUBLIC_H
#define __ODE_PUBLIC_H
/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 2016 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: odePublic.h,v 1.2 2016/08/02 20:58:03 IOWA\dheitbri Exp $
//
// Author(s):	Joshua Svenson
// Date:		June 2016
//
// Description: Defines the structures used by ODE      
// 
//
//////////////////////////////////////////////////////////////////////
namespace odePublic
{
	////////////////////////////////////////////
	///\brief 
	////	def for a solid body used by ode
	struct SBodyProps {
		int	   solId;
		int    cvedId;
		int    cigiId;
		double frictionCoeff;  
		double bounceEnergyLoss;
		double contactSoftERP;
		double contactSoftCFM;
	};

	enum EODEObjType {
		e_SPHERE = 0,
		e_CAPSULE,
		e_CYLINDER,
		e_BOX,
		e_COMPOSITE};
}


#endif /* __ODE_PUBLIC_H */