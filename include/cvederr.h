//////////////////////////////////////////////////////////////////////////////
//
//(C) Copyright 1998 by NADS & Simulation Center, The University of
//   Iowa.  All rights reserved.
//
// Author(s):	Yiannis Papelis
// Date:		August, 1998
//
// $Id: cvederr.h,v 1.17 2013/05/08 15:17:50 IOWA\vhorosewski Exp $
//
// Description:	Header file for the CVED error classes.  These classes
// are to be "thrown" for exceptions.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef __CVEDERR_H
#define __CVEDERR_H // {secret}

#include <cvedpub.h>

namespace CVED {

///////////////////////////////////////////////////////////////////////
///\class cvCError
///\brief
///		This class is for "thrown" for exceptions.
///\par
/// This class helps manage error conditions.  It is designed so
/// it can be "thrown" by various CVED functions and classes when
/// serious errors occur.  Note that this class is not a replacement
/// for return codes or non standard conditions such as invalid user
/// provided arguments.  In thoses cases, the functions should return
/// appropriate error codes.  A CVED operation throws this class (or
/// any of its derivatives) to indicate some serious conditions that
/// cannot be dealt where it occurred.
//////////////////////////////////////////////////////////////////////
class cvCError {
public:
	cvCError();
	virtual ~cvCError();
	virtual void Notify(void) const;
	cvCError(const cvCError&);
	string m_msg;
};


////////////////////////////////////////////////////////////////////////
///
/// This class represents an internal memory overflow.  It takes place
/// when an overflow occurs in statically allocated arrays, or 
/// arrays allocated dynamically but whose size is a compile time constant 
///////////////////////////////////////////////////////////////////////////
class cvCArrayOverflow : public cvCError {
public:
	cvCArrayOverflow();
	cvCArrayOverflow(string&);
	~cvCArrayOverflow();
};

/////////////////////////////////////////////////////////////////////////
/// This class represents an internal consistency error.  It takes place
/// when an internal condition that is expected to be true turns out
/// false.  Almost always, this error indicates corruption in the
/// internal data structures or some coding error
/////////////////////////////////////////////////////////////////////////
class cvCInternalError : public cvCError {
public:
	cvCInternalError();
	cvCInternalError(const char* pFile, 
					int lineNum);
	cvCInternalError(const string& msg, 
					const char* pFile, 
					int lineNum);
	~cvCInternalError();
};

/////////////////////////////////////////////////////////////////////////
//
// This class represents an error that occurred during self-tests.  It
// takes place when quality assurance software determines that a called
// function does not operate as specified.
class cvCTestError : public cvCError {
public:
	cvCTestError();
	explicit cvCTestError(const char *pMsg);
	cvCTestError(const char *pMsg, 
				const char *pFile, 
				int lineNum);
	~cvCTestError();
};

///////////////////////////////////////////////////////////////////////
//	Inline functions
///////////////////////////////////////////////////////////////////////
inline
cvCError::~cvCError() {}

inline
cvCArrayOverflow::~cvCArrayOverflow() {}

inline
cvCInternalError::~cvCInternalError() {}

inline
cvCTestError::~cvCTestError() {}

}	// namespace CVED

#endif	// __CVED_ERR_H

