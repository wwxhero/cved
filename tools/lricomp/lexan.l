%{
/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: lexan.l,v 1.1 2005/06/10 17:43:34 yhe Exp $
 *
 * Author(s):    Jennifer Galvez, Yiannis Papelis
 * Date:         July, 1998
 *
 * Description:  lexical analyzer for LRI compiler
 *
 ****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>

#include "parser.h" 
#include "y.tab.h"

#undef YY_INPUT
/*
 *converts string to uppercase
 */ 
#define  UpperCase(str)  \
{ int i; \
  for(i=0; i<strlen(str);i++)\
      str[i] = toupper(str[i]);  }


#define YY_INPUT()\
	yyinput()

	
#ifdef LRI_DEBUG
#define _lriDBG(stmt)\
	if (ve_debug_level > 2)  stmt 
#else
#define _lriDBG(stmt)  
#endif 

/*Forward declarations*/
void skip_comments(void);
char *GetString( void );
char *ReturnStrTillEOL( void );

int  yywrap(void);
extern char yysbuf[];
extern char lri_errstr[];
int yylex(void);

%}

%p 3500
%%

[ \t]                       { 
                              _lriDBG(_vePrintf("  ")); 
                            }

"\n"                        { }

";"							{
							 return LRI_SEMI;
							}

"{"							{ 
                              _lriDBG(_vePrintf("{"));
                              return (LRI_CURLY_OPEN); 
                            }

"}"				   			{ 
                              _lriDBG(_vePrintf("}")); 
                              return (LRI_CURLY_CLOSE);
                            }

INTERSECTIONS               {
								return LRI_INTERSECTIONS;
							}

ELEVMAP                     {
								return LRI_ELEVMAP;
							}

ROADS						{ 
                              _lriDBG(_vePrintf("ROADS"));	
                              return LRI_ROADS; 
                            }

LANES						{ 
                              _lriDBG(_vePrintf("LANES"));
                              return LRI_LANES; 
                            }

BORDER          			{
                    			_lriDBG(_vePrintf("BORDER"));
                    			return LRI_BORDER;
                			}

CRDR            			{
                    			_lriDBG(_vePrintf("CRDR"));
                    			return LRI_CRDR;
                			}

comment            			{
                    			_lriDBG(_vePrintf("COMMENT"));
                    			return LRI_COMMENT;
                			}

HOLDOFS         			{
                    			_lriDBG(_vePrintf("HOLDOFS"));
                    			return LRI_HOLDOFS;
                			}

CRDRCURVE       			{
                    			_lriDBG(_vePrintf("CRDRCURVE"));
                    			return LRI_CRDR_CURVE;
                			}

HOLDLINE        			{
                    			_lriDBG(_vePrintf("HOLDLINE"));
                    			return LRI_HOLDLINE;
                			}

HOLDSIGN        			{
                    			_lriDBG(_vePrintf("HOLDSIGN"));
                    			return LRI_HOLDSIGN;
                			}

LINES           			{
                    			_lriDBG(_vePrintf("LINES"));
                    			return LRI_LINES;
                			}

REPOBJS           			{
                    			_lriDBG(_vePrintf("REPOBJS"));
                    			return LRI_REPOBJS;
                			}


P 							{	
                              _lriDBG(_vePrintf("POSITIVE"));
							  yylval.ival = LRI_POSITIVE;
							  return LRI_DIRECTION; 
							}

N							{ 	
                              _lriDBG(_vePrintf("NEGATIVE"));
							  yylval.ival = LRI_NEGATIVE;
							  return LRI_DIRECTION; 
							}

y 							{
								yylval.ival = cYes;
								return LRI_LFLAG;
							}

n 							{
								yylval.ival = cNo;
								return LRI_LFLAG;
							}

aligned						{
                            	_lriDBG(_vePrintf("ALIGNED"));
								return LRI_ALIGNED;
							}

plant						{
								_lriDBG(_vePrintf("PLANT"));
								return LRI_PLANT;
							}

OBJECTS						{
								_lriDBG(_vePrintf("OBJECTS"));
								return LRI_OBJECTS;
							}

LAT_CURVES					{
                              _lriDBG(_vePrintf("LAT_CURVES" )); 
                              return LRI_LAT_CURVES; 
                            }

LATCURVE					{
                              _lriDBG(_vePrintf("LATCURVE" )); 
                              return LRI_LATCURVE; 
                            }

LANEWIDTH					{
                              _lriDBG(_vePrintf("LANEWIDTH" )); 
                              return LRI_LANEWIDTH; 
                            }

ATTR						{
                              _lriDBG(_vePrintf("ATTR" )); 
                              return LRI_ATTR; 
                            }

ATTR_DICT					{
                              _lriDBG(_vePrintf("ATTR_DICT" )); 
                              return LRI_ATTR_DICT; 
                            }

LONGCURVE					{
                              _lriDBG(_vePrintf("LONGCURVE" )); 
                              return LRI_LONGCURVE; 
                            }
SIGN            			{
								yylval.strval = (char *) calloc(1,(strlen((const
char *)yytext)+1)*sizeof(char));
                              	strcpy(yylval.strval,(const char *)yytext);
                    			_lriDBG(_vePrintf("SIGN"));
                    			return LRI_CRDR_REASON;
                			}

TLIGHT          			{
								yylval.strval = (char *) calloc(1,(strlen((const char *)yytext)+1)*sizeof(char));
                              	strcpy(yylval.strval,(const char *)yytext);
                    			_lriDBG(_vePrintf("TLIGHT"));
                    			return LRI_CRDR_REASON;
                			}

OVLAP           			{
								yylval.strval = (char *) calloc(1,(strlen((const
char *)yytext)+1)*sizeof(char));
                              	strcpy(yylval.strval,(const char *)yytext);
                    			_lriDBG(_vePrintf("OVLAP"));
                    			return LRI_CRDR_REASON;
                			}

SOLID           			{
                    			_lriDBG(_vePrintf("SOLID"));
	
								yylval.strval = "SOLID";
                    			return LRI_LSTYLE;
                			}

DOTTED          			{
                    			_lriDBG(_vePrintf("DOTTED"));
								yylval.strval = "DOTTED";
                    			return LRI_LSTYLE;
                			}

HEADER          			{
                    			_lriDBG(_vePrintf("HEADER"));
                    			return LRI_HEADER;
                			}

=               			{
                    			_lriDBG(_vePrintf("="));
                    			return LRI_EQUALTO;
                			}

zdown             			{
                    			_lriDBG(_vePrintf("zdown"));
                    			return LRI_ZDOWN;
                			}

solchecksum        			{
                    			_lriDBG(_vePrintf("solchecksum"));
                    			return LRI_SOLCHECKSUM;
                			}

-?(([0-9]+)|([0-9]*\.[0-9]+)([eE][+-]?[0-9]+)?) {
							 yylval.dval = (double)atof((const char *)yytext);
							 return LRI_REAL;
							}

[a-zA-Z\_][\-\.a-zA-Z0-9_]* {
							  yylval.strval = (char *) calloc(1,(strlen((const char *)yytext)+1)*sizeof(char));
					 		  strcpy(yylval.strval,(const char *)yytext);
					 		  _lriDBG(_vePrintf("%s",yylval.strval));
				         	  return(LRI_IDENTIFIER);
                            }

"/*"                        { 
                              skip_comments();
                            }

"\""						{
								yylval.strval = GetString();
								return LRI_QUOTED_STRING;
							}
%%



/*
 * copy string to output
 */
char* GetString( void )
{
	char  ch;
	char *pStr = NULL;
	int   size = 0; 		/* available storage */
	int   i    = 0;			/* actual size */
	
	/* collect all chars till close quote */
	for (;;) {
		if ( i >= size-1 ) {
			size += 128;
			pStr = realloc(pStr, size * sizeof(char)); 
		}

		ch = yyinput();
		if (ch == '\"'){
			pStr[i]  = '\0';
			break;	
		}
		else if (feof(yyin)) {
  			fprintf(stderr, "String not closed %d \n", yylineno);
			exit(-1);
		}
		else if (ch == '\n') {
			yylineno++;
		}
		pStr[i++]  = ch;
	}	

	_lriDBG(_vePrintf(" %s",str));
	return pStr;
}


/*
 * Collects all charactes till end of line and returns it 
 */
char *ReturnStrTillEOL( void )
{
	char ch, *str;
	int  i = 0;

	
	/* Remove leading blanks */
	do {
		ch = yyinput();
	}while ( ch == ' ' || ch == '\t' || ch =='\n' );

  for (;;) {
	
		if ( ch == '\n' ){
			str[i]  = '\0';

			/* strcpy(str, (const char *)tmp_buf); */

			break;
		}
		str[i++]  = ch;

		ch = yyinput();
	}

	_lriDBG(_vePrintf( " %s \n", str));
  	_lriDBG(_vePrintf("%d : ",yylineno)); 
	return str;
}

/* 
 * skips comments in the input file, no nested commenting 
 */
void skip_comments(void)
{
	char ch;

    for (;;) {
			ch = yyinput();
			if (feof(yyin)){
  			fprintf(stderr,"Comment not closed at %d\n", yylineno+1);
				break;
			}
			if (ch == '*'){
				if ((ch = yyinput()) == '/')
					break;	
			}
			else if (ch == '\n'){ 
				; 
			}
    }

}
