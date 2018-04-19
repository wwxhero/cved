/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: sharedmem.h,v 1.8 2014/09/15 18:30:40 IOWA\vhorosewski Exp $
//
// Author(s):	Yiannis Papelis
// Date:		September, 1998
//
// Description:	The declaration of the CSharedMem class
//
/////////////////////////////////////////////////////////////////////////////
#ifndef __SHARED_MEM_H
#define __SHARED_MEM_H		// {secret}

#include <stdio.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Winsock2.h>
#include <windows.h>
#endif

//
// This class provides some basic encapsulation for shared
// memory usage.  The class has two goals, one to abstract
// dealing with shared memory segments so it's easier to do,
// and second, to allow a uniform interface in UNIX and Win32
// applications.
//
// The class creates persistent shared segments, i.e., segments
// that don't cease to exist when the class instance who created
// them is destroyed.
//
class CSharedMem {
public:
	CSharedMem();
	explicit CSharedMem(const char *logfile);
	~CSharedMem();

	enum { eANY_SIZE = -1 };

	void*		Connect(const char *cpName,
						bool create=false,
						int size=eANY_SIZE);		// attach/create as needed
	void		Disconnect(void);					// detach from segment
	bool		Delete(void);						// delete segment
	const char*	GetErrorMsg(void) const;			// return ptr to err msg

private:
	// declared private to disallow their use

	CSharedMem(const CSharedMem &);
	CSharedMem &operator=(const CSharedMem &);

	FILE*	m_pLog;				// log file to which to write debugging 
								//	messages
	void*	m_pData;			// ptr to actual shared memory
	char*	m_pErrMsg;			// ptr to error string
	int		m_shmid;			// in UNIX holds the shared seg id
};

#endif	// __SHARED_MEM_H

