//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: cvederr.cxx,v 1.17 2013/11/26 23:49:58 iowa\dheitbri Exp $
//
// Author(s):   Yiannis Papelis
// Date:		August, 1998
//
// Description:	Implementation of the various error classes for CVED.
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
// Description: cvCError
// 	This default constructor clears the message buffer.
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCError::cvCError()
{
	string empty;
	m_msg = empty;
} // end of cvCError

//////////////////////////////////////////////////////////////////////////////
//
// Description: cvCError
// 	This copy constructor copies the message in the parameter to the current 
// 	error message.
//
// Remarks: 
//
// Arguments:  
// 	cErrorSource - cvCError instance
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCError::cvCError(const cvCError& cErrorSource) {
         m_msg = cErrorSource.m_msg  ;
} // end of cvCError

//////////////////////////////////////////////////////////////////////////////
//
// Description: Notify
// 	This function gives the user some notification regarding the error 
// 	condition represented by the error class.  
//
// Remarks: In an MFC window, the function pops up a dialog box explaining the 
// 	error whereas in text based implementations, the class simply prints an 
// 	error message to stderr.
//
// Arguments: 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void cvCError::Notify(void) const
{
	fprintf(stderr, "%s\n", m_msg.c_str());
} // end of Notify

//////////////////////////////////////////////////////////////////////////////
//
// Description: cvCArrayOverflow
// 	This default constructor initializes the message buffer.
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCArrayOverflow::cvCArrayOverflow()
{
	m_msg.append("CVED Error: Array Overflow: ");
} // end of cvCArrayOverflow

//////////////////////////////////////////////////////////////////////////////
//
// Description: cvCTestError
// 	This default constructor initializes the message buffer.
//
// Remarks: 
//
// Arguments: 
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCTestError::cvCTestError()
{
	m_msg.append("CVED Error: Error in tests\n");
} // end of cvCTestError

//////////////////////////////////////////////////////////////////////////////
//
// Description: cvCTestError
// 	This constructor initializes the message buffer.
//
// Remarks: 
//
// Arguments: 
// 	cpText - pointer to the text message
// 	cpFileName - pointer to the file name
//	lineno - line number of the error
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCTestError::cvCTestError(
			const char*	cpText,
			const char*	cpFileName, 
			int			lineno)
{
	char buf[10];

	m_msg.append("CVED Error: Error in tests\n");

	string file(cpFileName);
	int npos = (int)file.rfind('\\');
#if 0
	if ( npos > -1 )
		// the following statement didn't compile under SGI
		// file.erase(0, npos+1);
#endif

	m_msg.append("Message: ");
	m_msg.append(cpText);
	m_msg.append("\nError occured @ ");
	m_msg.append(file);
	m_msg.append(":");

	sprintf_s(buf, "%d\n", lineno);
	m_msg.append(buf);
} // end of cvCTestError

//////////////////////////////////////////////////////////////////////////////
//
// Description: cvCInternalError
// 	This constructor initializes the message buffer.
//
// Remarks: 
//
// Arguments: 
// 	cpFileName - pointer to the file name
//	lineno - line number of the error
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCInternalError::cvCInternalError(
			const char*	cpFile,
			int			lineno) 
{
	char buf[30];
	string empty;

	m_msg = empty;
	m_msg.append("Error occured @ ");
	m_msg.append(cpFile);
	m_msg.append(":");

	sprintf_s(buf, "%d\n", lineno);
	m_msg.append(buf);
} // end of cvCInternalError

//////////////////////////////////////////////////////////////////////////////
//
// Description: cvCInternalError
// 	This constructor initializes the message buffer.
//
// Remarks: 
//
// Arguments: 
// 	cUserMsg - string containing the user's message
// 	cpFileName - pointer to the file name
//	lineno - line number of the error
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
cvCInternalError::cvCInternalError(
			const string&	cUserMsg, 
			const char*		cpFile, 
			int				lineno)
{
	char buf[30];
	string empty;

	m_msg = empty;
	m_msg.append("Additional message: ");
	m_msg += cUserMsg;
	m_msg.append("\nError occured @ ");
	m_msg.append(cpFile);
	m_msg.append(":");

	sprintf_s(buf, "%d\n", lineno);
	m_msg.append(buf);
} // end of cvCInternalError

} // namespace CVED
