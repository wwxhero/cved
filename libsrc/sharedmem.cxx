//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version: 		$Id: sharedmem.cxx,v 1.10 2016/09/20 19:42:33 IOWA\dheitbri Exp $
//
// Author(s):	Yiannis Papelis
// Date:		September, 1998
//
// Description:	The implementation of the CSharedMem class for
//              unix and Win32
//
//////////////////////////////////////////////////////////////////////////////
#include "cvedpub.h"

#ifdef __sgi
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#elif _PowerMAXOS
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#else
#pragma warning(disable:4996)
#endif

//////////////////////////////////////////////////////////////////////////////
//
// Description: CSharedMem
// 	Default constructor initializes the local variables to null.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CSharedMem::CSharedMem()
{
	m_pLog    = 0;
	m_pData   = 0;
	m_pErrMsg = new char[256];
	m_pErrMsg[0] = 0;
	m_shmid   = -1;
} // end of CSharedMem

//////////////////////////////////////////////////////////////////////////////
//
// Description: CSharedMem
// 	This constructor initializes the local variables with the parameters.
//
// Remarks: If 0 is passed to the parameter, then no debugging file is used. 
//
// Arguments:
// 	cpLogFile - name of the log file to which to print the debugging messages
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CSharedMem::CSharedMem(const char *cpLogFile)
{
	m_pLog    = fopen(cpLogFile, "w");
	m_pData   = 0;
	m_pErrMsg = new char[256];
	m_pErrMsg[0] = 0;
	m_shmid   = -1;
} // end of CSharedMem

//////////////////////////////////////////////////////////////////////////////
//
// Description: ~CSharedMem
// 	Default destructor deletes the allocated memory and released the shared
// 	memory.
//
// Remarks:
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
CSharedMem::~CSharedMem()
{
	if ( m_pLog ) {
		fprintf(m_pLog, "Shm%p: destructor\n", this);
		fclose(m_pLog);
		m_pLog = 0;
	}
	delete[] m_pErrMsg;

	if ( m_pData ) {
#ifdef _WIN32
		UnmapViewOfFile(m_pData);
#elif __sgi
		shmdt(m_pData);
#elif _PowerMAXOS
        shmdt(m_pData);
#endif
		m_pData = 0;
	}
} // end of CSharedMem

/////////////////////////////////////////////////////////////////////////////
//
// Description: Disconnect
// 	Release the connection to a shared segment of memory
//
// Remarks: This function should be called to release a connection to a
// 	shared memory segment.  After this function is called, the memory 
// 	pointed to by the pointer returned by Connect will be invalid.
//
// Arguments:
//
// Returns: void
//
//////////////////////////////////////////////////////////////////////////////
void
CSharedMem::Disconnect(void)
{
	if ( m_pLog ) {
		fprintf(m_pLog, "Shm%p: disconnect\n", this);
	}
	if ( m_pData )  {
#ifdef _WIN32
		UnmapViewOfFile(m_pData);
#elif __sgi
		shmdt(m_pData);
#elif _PowerMAXOS
        shmdt(m_pData);

#endif
		m_pData = 0;
	}
} // end of Disconnect

//////////////////////////////////////////////////////////////////////////////
//
// Description: Connect
// 	Find or create a shared segment and get a pointer to it
// 
// Remarks: This function searches for a shared memory segment that matches
// 	the specified key, attaches to that segmet and if all goes well, returns 
// 	a pointer to the segment.
//
// 	To be useful for interprocess communication, all processes must agree on 
// 	a common key to globally identify the shared memory segment.  This 
// 	function allows the user to specify this key as a string in the cpName 
// 	parameter.
//
// 	If a segment using the specified key exists, the function will return a 
// 	pointer to that segment.  If one doesn't exist, and the create flag is 
// 	false, the function will return 0.
//
// 	If a segment using the specified key doesn't exist and the create flag 
// 	is true, the function will create that segment using the specified key, 
// 	at the specified size, then return a pointer to it.
//
// Arguments:
//  cpName - a string to be used to identify the segment
//	size - the size of the segment, used only when create is true.
//		when create is false, set this value to CSharedMem::eANY_SIZE.
//  create - if true and the segment doesn't exist it is created
//
// Returns: a pointer to the shared segment or 0 to indicate it could not 
// 	connect or create the segment.  In case of failure the GetErrorMessage 
// 	can be used to gain access to a description of the error.
//
//////////////////////////////////////////////////////////////////////////////
void *
CSharedMem::Connect(const char *cpName, bool create, int size)
{
#ifdef _WIN32
	HANDLE h;
#endif

	if ( cpName == NULL ) {
		sprintf(m_pErrMsg, "NULL pointer specified for segment name.\n");
		return 0;
	}

	if ( create == false ) {
#ifdef _WIN32
		h = OpenFileMapping(FILE_MAP_WRITE, true, cpName);
		if ( h == 0 ) {
			sprintf(m_pErrMsg, "OpenFileMapping failed.\n");
			return 0;
		}
#elif __sgi
		if ( size == eANY_SIZE ) size = 0;
		m_shmid = shmget(atoi(cpName), size, 0666);
		if ( m_shmid < 0 ) {
			sprintf(m_pErrMsg, "shmget(%d, %d, 0x%X) failed.\n",
						atoi(cpName), size, 0666);
			return 0;
		}
#elif _PowerMAXOS
		if ( size == eANY_SIZE ) size = 0;
		m_shmid = shmget(atoi(cpName), size, 0666);
		if ( m_shmid < 0 ) {
			sprintf(m_pErrMsg, "shmget(%d, %d, 0x%X) failed.\n",
										atoi(cpName), size, 0666);
			return 0;
		}
#endif
	}
	else {
#ifdef _WIN32
		h = CreateFileMapping((HANDLE)0xFFFFFFFF,
				NULL,
				PAGE_READWRITE,
				0,
				size,
				cpName);
		if ( h == NULL ) {
			// if we failed because segment exists, simply attach to it
			if ( GetLastError() == ERROR_ALREADY_EXISTS ) {
				h = OpenFileMapping(FILE_MAP_WRITE, true, cpName);
				if ( h == 0 ) {
					sprintf(m_pErrMsg, "OpenFileMapping failed.\n");
					return 0;
				}
			}
			else {
				sprintf(m_pErrMsg, "CreateFileMapping failed.\n");
				return 0;
			}
		}
#elif __sgi
		m_shmid = shmget(atoi(cpName), size, IPC_CREAT | 0666);
		if ( m_shmid < 0 ) {
			sprintf(m_pErrMsg, "shmget(%d, %d, 0x%X) failed.\n",
						atoi(cpName), size, IPC_CREAT | 0666);
			return 0;
		}
#elif _PowerMAXOS
                m_shmid = shmget(atoi(cpName), size, IPC_CREAT | 0666);
                if ( m_shmid < 0 ) {
                        sprintf(m_pErrMsg, "shmget(%d, %d, 0x%X) failed.\n",
                                                atoi(cpName), size, IPC_CREAT | 0666);
                        return 0;
                }

#endif
	}


#ifdef _WIN32
	void *p = MapViewOfFile(h, FILE_MAP_WRITE, 0, 0, 0);
	if ( p == 0 ) {
		sprintf(m_pErrMsg, "MapViewOfFile failed.\n");
		return 0;
	}
#elif __sgi
	void *p = shmat(m_shmid, 0, SHM_RND);
	if ( (int)p == -1 ) {
		sprintf(m_pErrMsg, "shmat failed.\n");
		return 0;
	}
#elif _PowerMAXOS
        void *p = shmat(m_shmid, 0, SHM_RND);
        if ( (int)p == -1 ) {
                sprintf(m_pErrMsg, "shmat failed.\n");
                return 0;
        }

#endif

	m_pData = p;
	return p;
} // end of Connect

//////////////////////////////////////////////////////////////////////////////
//
// Description: Delete
// 	Delete a shared segment
//
// Remarks: This function deletes the shared segment that this class instance 
// 	is currently connected to.  The function has no effect if the instance is 
// 	not connected to a shared segment.
//
// 	After deleting the shared segment, any pointers to the memory returned by 
// 	the Connect function are invalid and if accessed could cause unpredictable 
// 	results.
//
// Arguments:
//
// Returns: True to indicate that deletion worked, or false.
//
//////////////////////////////////////////////////////////////////////////////
bool
CSharedMem::Delete(void)
{
#ifdef __sgi
	if ( m_shmid != -1 ) {
		shmctl(m_shmid, IPC_RMID);
		m_shmid = -1;
	}
#elif _PowerMAXOS
        if ( m_shmid != -1 ) {
                shmctl(m_shmid, IPC_RMID,NULL);
                m_shmid = -1;
        }

#endif
	return true;		// in Win32, deletion is auto after all detach
} // end of Delete

//////////////////////////////////////////////////////////////////////////////
//
// Description: GetErrorMsg
// 	Get error message describing most recent error
// 
// Remarks: This function returns a pointer to the error message corresponding
// 	to the most recent error or failure.
// 
// Arguments:
// 
// Returns: The function retuns a pointer to a zero terminated string.  The
// 	function will never return a null pointer, but if no error has ocurred, 
// 	the string will be empty.
//
//////////////////////////////////////////////////////////////////////////////
const char *
CSharedMem::GetErrorMsg(void) const
{
	return m_pErrMsg;
} // end of GetErrorMsg

