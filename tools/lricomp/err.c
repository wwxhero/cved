/***************************************************************************
 *
 * (C) Copyright 1998 by NADS & Simulation Center, The University of Iowa,
 * and The University of Iowa.  All rights reserved.
 *
 * Version: 		$Id: err.c,v 1.1 2005/06/10 17:43:32 yhe Exp $
 *
 * Author(s):   
 * Date:        
 * 
 * Contains the error checking utilities used by the LRI 
 * compiler.
 *
 */
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include "err.h"

/**************************************************************************
 * 
 * Table with error messages.  Each message correpsonds to one of
 * the error coces in err.h
 */
static char *errMsgs[] = {
		"Bad Road!",
	 	"Bad Lane!",
	 	"Bad Intersection!",
		"Unknown object type. ",
		"Unknown sol identifier. ",
		"Memory allocation for road pool failed.",
		"Memory allocation for char pool failed.",
		"Memory allocation for attribute pool failed.",
		"Memory allocation for repeated object pool failed.",
		"Memory allocation for lane pool failed.",
		"Memory allocation for control point pool failed.",
		"Memory allocation for lateral control point pool failed.",
		"Memory allocation for intersection pool failed.",
		"Memory allocation for border segment pool failed.",
		"Memory allocation for corridor pool failed.",
		"Memory allocation for corridor control point pool failed.",
		"Memory allocation for hold offset pool failed.",
		"Memory allocation for lat pool failed.",
		"Memory write failed",
		"Semantics error:" ,
		"Memory allocation for road piece failed.",
		"Inconsistent number of elevation posts in elevation map.",
		"Unknown elevation map",
		"Incomplete intersection border polygon.",
		"Object whose name equates repeat obj is missing",
		"Memory allocation for dynObjRef pool failed.",
		"Memory allocation for roadRef pool failed.",
		"Memory allocation for intrsctn ref pool failed.",
		"Name of either sign or light in crdr can not be found in object pool.",
		"Memory allocation for envArea pool failed.",
		"Memory allocation for envInfo pool failed.",
		"Memory allocation for corridor merge distance point pool failed."
};

FILE *logFile = 0;

/**************************************************************************
*
* This function maybe used to prompt errors message in parsing lri files.
*
* Variable argument list has been implemented.
*
*/
void lrierr( TErrorType err_id, char * fmt, ... )
{
	va_list ap;

	if ( logFile == 0 ) logFile = stderr;

	if ( (int)err_id < 0 || (int)err_id > sizeof(errMsgs) / sizeof(char*) ) {
		fprintf(logFile,"Lri compilation error: code %d.\n", (int)err_id);
		exit(-1);
	}

	fprintf(logFile, "Lri compilation error: %s", errMsgs[err_id]);
	
	va_start(ap, fmt );
	vfprintf(logFile,fmt, ap);
	va_end(ap);

	fprintf(logFile,"\n");
	exit(-1);
}

int yywrap(void)
{
	return 1;
}


/* 
 * The gettxt function is not found anywhere when on the PC
 * side so we simply add it here.
 */
#ifdef _WIN32
char *
gettxt(const char *pMsg1, const char *pMsg2)
{
	printf("%s:%s\n", pMsg1, pMsg2);
	return "Error";
}
#endif


void yyerror(char *s)
{
	extern FILE *yyin;
	extern int yylineno;
	extern char yytext[];
	int    yyinput(void);

	fprintf(stderr, "%s near line %d.\nUnexpected token =>\"%s\", ", 
	         s, yylineno, yytext);
}

