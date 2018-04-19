# include "stdio.h"
# define U(x) ((unsigned char)(x))
# define NLSTATE yyprevious=YYNEWLINE
# define BEGIN yybgin = yysvec + 1 +
# define INITIAL 0
# define YYLERR yysvec
# define YYSTATE (yyestate-yysvec-1)
# define YYOPTIM 1
# define YYLMAX 2048
# define output(c) (void)putc(c,yyout)
#if defined(__cplusplus) || defined(__STDC__)

#ifdef __cplusplus
extern "C" {
#endif
	int yylex(void);
	int yyback(int *, int);
	int yyinput(void);
	int yylook(void);
	void yyoutput(int);
	int yyracc(int);
	int yyreject(void);
	void yyunput(int);
#ifndef yyless
	void yyless(long int);
#endif
#ifndef yywrap
	int yywrap(void);
#endif
#ifdef __cplusplus
}
#endif

#endif

# define input() (((yytchar=yysptr>yysbuf?U(*--yysptr):getc(yyin))==10?(yylineno++,yytchar):yytchar)==EOF?0:yytchar)
# define unput(c) {yytchar= (c);if(yytchar=='\n')yylineno--;*yysptr++=yytchar;}
# define yymore() (yymorfg=1)
# define ECHO (void)fprintf(yyout, "%s",yytext)
# define REJECT { nstr = yyreject(); goto yyfussy;}
int yyleng; extern char yytext[];
int yymorfg;
extern char *yysptr, yysbuf[];
int yytchar;
FILE *yyin = 0, *yyout = 0;
extern int yylineno;
struct yysvf { 
	struct yywork *yystoff;
	struct yysvf *yyother;
	int *yystops;};
struct yysvf *yyestate;
extern struct yysvf yysvec[], *yybgin;

# line 2 "lexan.l"

# line 3 "lexan.l"
/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: lex.yy.c,v 1.1 2005/06/10 17:43:34 yhe Exp $
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

# line 27 "lexan.l"
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


# line 47 "lexan.l"
/*Forward declarations*/
void skip_comments(void);
char *GetString( void );
char *ReturnStrTillEOL( void );

int  yywrap(void);
extern char yysbuf[];
extern char lri_errstr[];
int yylex(void);

# define YYNEWLINE 10
yylex(void){
int nstr; extern int yyprevious;
while((nstr = yylook()) >= 0)
yyfussy: switch(nstr){
case 0:
if(yywrap()) return(0); break;
case 1:

# line 61 "lexan.l"
                      { 
                              _lriDBG(_vePrintf("  ")); 
                            }
break;
case 2:

# line 65 "lexan.l"
                       { }
break;
case 3:

# line 67 "lexan.l"
						{
							 return LRI_SEMI;
							}
break;
case 4:

# line 71 "lexan.l"
						{ 
                              _lriDBG(_vePrintf("{"));
                              return (LRI_CURLY_OPEN); 
                            }
break;
case 5:

# line 76 "lexan.l"
			   			{ 
                              _lriDBG(_vePrintf("}")); 
                              return (LRI_CURLY_CLOSE);
                            }
break;
case 6:

# line 81 "lexan.l"
              {
								return LRI_INTERSECTIONS;
							}
break;
case 7:

# line 85 "lexan.l"
                    {
								return LRI_ELEVMAP;
							}
break;
case 8:

# line 89 "lexan.l"
					{ 
                              _lriDBG(_vePrintf("ROADS"));	
                              return LRI_ROADS; 
                            }
break;
case 9:

# line 94 "lexan.l"
					{ 
                              _lriDBG(_vePrintf("LANES"));
                              return LRI_LANES; 
                            }
break;
case 10:

# line 99 "lexan.l"
         			{
                    			_lriDBG(_vePrintf("BORDER"));
                    			return LRI_BORDER;
                			}
break;
case 11:

# line 104 "lexan.l"
           			{
                    			_lriDBG(_vePrintf("CRDR"));
                    			return LRI_CRDR;
                			}
break;
case 12:

# line 109 "lexan.l"
           			{
                    			_lriDBG(_vePrintf("COMMENT"));
                    			return LRI_COMMENT;
                			}
break;
case 13:

# line 114 "lexan.l"
        			{
                    			_lriDBG(_vePrintf("HOLDOFS"));
                    			return LRI_HOLDOFS;
                			}
break;
case 14:

# line 119 "lexan.l"
      			{
                    			_lriDBG(_vePrintf("CRDRCURVE"));
                    			return LRI_CRDR_CURVE;
                			}
break;
case 15:

# line 124 "lexan.l"
       			{
                    			_lriDBG(_vePrintf("HOLDLINE"));
                    			return LRI_HOLDLINE;
                			}
break;
case 16:

# line 129 "lexan.l"
       			{
                    			_lriDBG(_vePrintf("HOLDSIGN"));
                    			return LRI_HOLDSIGN;
                			}
break;
case 17:

# line 134 "lexan.l"
          			{
                    			_lriDBG(_vePrintf("LINES"));
                    			return LRI_LINES;
                			}
break;
case 18:

# line 139 "lexan.l"
          			{
                    			_lriDBG(_vePrintf("REPOBJS"));
                    			return LRI_REPOBJS;
                			}
break;
case 19:

# line 145 "lexan.l"
							{	
                              _lriDBG(_vePrintf("POSITIVE"));
							  yylval.ival = LRI_POSITIVE;
							  return LRI_DIRECTION; 
							}
break;
case 20:

# line 151 "lexan.l"
						{ 	
                              _lriDBG(_vePrintf("NEGATIVE"));
							  yylval.ival = LRI_NEGATIVE;
							  return LRI_DIRECTION; 
							}
break;
case 21:

# line 157 "lexan.l"
							{
								yylval.ival = cYes;
								return LRI_LFLAG;
							}
break;
case 22:

# line 162 "lexan.l"
							{
								yylval.ival = cNo;
								return LRI_LFLAG;
							}
break;
case 23:

# line 167 "lexan.l"
					{
                            	_lriDBG(_vePrintf("ALIGNED"));
								return LRI_ALIGNED;
							}
break;
case 24:

# line 172 "lexan.l"
					{
								_lriDBG(_vePrintf("PLANT"));
								return LRI_PLANT;
							}
break;
case 25:

# line 177 "lexan.l"
					{
								_lriDBG(_vePrintf("OBJECTS"));
								return LRI_OBJECTS;
							}
break;
case 26:

# line 182 "lexan.l"
				{
                              _lriDBG(_vePrintf("LAT_CURVES" )); 
                              return LRI_LAT_CURVES; 
                            }
break;
case 27:

# line 187 "lexan.l"
				{
                              _lriDBG(_vePrintf("LATCURVE" )); 
                              return LRI_LATCURVE; 
                            }
break;
case 28:

# line 192 "lexan.l"
				{
                              _lriDBG(_vePrintf("LANEWIDTH" )); 
                              return LRI_LANEWIDTH; 
                            }
break;
case 29:

# line 197 "lexan.l"
					{
                              _lriDBG(_vePrintf("ATTR" )); 
                              return LRI_ATTR; 
                            }
break;
case 30:

# line 202 "lexan.l"
				{
                              _lriDBG(_vePrintf("ATTR_DICT" )); 
                              return LRI_ATTR_DICT; 
                            }
break;
case 31:

# line 207 "lexan.l"
				{
                              _lriDBG(_vePrintf("LONGCURVE" )); 
                              return LRI_LONGCURVE; 
                            }
break;
case 32:

# line 211 "lexan.l"
           			{
								yylval.strval = (char *) calloc(1,(strlen((const
char *)yytext)+1)*sizeof(char));
                              	strcpy(yylval.strval,(const char *)yytext);
                    			_lriDBG(_vePrintf("SIGN"));
                    			return LRI_CRDR_REASON;
                			}
break;
case 33:

# line 219 "lexan.l"
         			{
								yylval.strval = (char *) calloc(1,(strlen((const char *)yytext)+1)*sizeof(char));
                              	strcpy(yylval.strval,(const char *)yytext);
                    			_lriDBG(_vePrintf("TLIGHT"));
                    			return LRI_CRDR_REASON;
                			}
break;
case 34:

# line 226 "lexan.l"
          			{
								yylval.strval = (char *) calloc(1,(strlen((const
char *)yytext)+1)*sizeof(char));
                              	strcpy(yylval.strval,(const char *)yytext);
                    			_lriDBG(_vePrintf("OVLAP"));
                    			return LRI_CRDR_REASON;
                			}
break;
case 35:

# line 234 "lexan.l"
          			{
                    			_lriDBG(_vePrintf("SOLID"));
	
								yylval.strval = "SOLID";
                    			return LRI_LSTYLE;
                			}
break;
case 36:

# line 241 "lexan.l"
         			{
                    			_lriDBG(_vePrintf("DOTTED"));
								yylval.strval = "DOTTED";
                    			return LRI_LSTYLE;
                			}
break;
case 37:

# line 247 "lexan.l"
         			{
                    			_lriDBG(_vePrintf("HEADER"));
                    			return LRI_HEADER;
                			}
break;
case 38:

# line 252 "lexan.l"
              			{
                    			_lriDBG(_vePrintf("="));
                    			return LRI_EQUALTO;
                			}
break;
case 39:

# line 257 "lexan.l"
            			{
                    			_lriDBG(_vePrintf("zdown"));
                    			return LRI_ZDOWN;
                			}
break;
case 40:

# line 262 "lexan.l"
       			{
                    			_lriDBG(_vePrintf("solchecksum"));
                    			return LRI_SOLCHECKSUM;
                			}
break;
case 41:

# line 267 "lexan.l"
{
							 yylval.dval = (double)atof((const char *)yytext);
							 return LRI_REAL;
							}
break;
case 42:

# line 272 "lexan.l"
{
							  yylval.strval = (char *) calloc(1,(strlen((const char *)yytext)+1)*sizeof(char));
					 		  strcpy(yylval.strval,(const char *)yytext);
					 		  _lriDBG(_vePrintf("%s",yylval.strval));
				         	  return(LRI_IDENTIFIER);
                            }
break;
case 43:

# line 279 "lexan.l"
                       { 
                              skip_comments();
                            }
break;
case 44:

# line 283 "lexan.l"
					{
								yylval.strval = GetString();
								return LRI_QUOTED_STRING;
							}
break;
case -1:
break;
default:
(void)fprintf(yyout,"bad switch yylook %d",nstr);
} return(0); }
/* end of yylex */

# line 288 "lexan.l"



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
int yyvstop[] = {
0,

1,
0,

2,
0,

44,
0,

41,
0,

3,
0,

38,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

20,
42,
0,

42,
0,

19,
42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

22,
42,
0,

42,
0,

42,
0,

21,
42,
0,

42,
0,

4,
0,

5,
0,

41,
0,

43,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

41,
0,

29,
42,
0,

42,
0,

11,
42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

32,
42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

9,
42,
0,

42,
0,

42,
0,

42,
0,

17,
42,
0,

42,
0,

42,
0,

34,
42,
0,

42,
0,

8,
42,
0,

35,
42,
0,

42,
0,

42,
0,

42,
0,

24,
42,
0,

42,
0,

39,
42,
0,

42,
0,

10,
42,
0,

42,
0,

36,
42,
0,

42,
0,

37,
42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

33,
42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

7,
42,
0,

42,
0,

13,
42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

42,
0,

25,
42,
0,

18,
42,
0,

23,
42,
0,

12,
42,
0,

42,
0,

42,
0,

42,
0,

15,
42,
0,

16,
42,
0,

42,
0,

42,
0,

27,
42,
0,

42,
0,

42,
0,

42,
0,

30,
42,
0,

14,
42,
0,

42,
0,

28,
42,
0,

42,
0,

31,
42,
0,

42,
0,

42,
0,

26,
42,
0,

42,
0,

42,
0,

40,
42,
0,

42,
0,

6,
42,
0,
0};
# define YYTYPE unsigned char
struct yywork { YYTYPE verify, advance; } yycrank[] = {
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	1,3,	1,4,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	1,3,	0,0,	1,5,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	8,37,	
0,0,	0,0,	1,6,	1,7,	
1,8,	1,9,	1,9,	1,9,	
1,9,	1,9,	1,9,	1,9,	
1,9,	1,9,	1,9,	0,0,	
1,10,	0,0,	1,11,	0,0,	
0,0,	0,0,	1,12,	1,13,	
1,14,	1,15,	1,16,	1,17,	
1,17,	1,18,	1,19,	1,17,	
1,17,	1,20,	1,17,	1,21,	
1,22,	1,23,	1,17,	1,24,	
1,25,	1,26,	1,17,	1,17,	
1,17,	1,17,	1,17,	1,17,	
13,39,	14,40,	17,17,	14,17,	
1,17,	13,17,	1,27,	1,17,	
1,28,	1,17,	1,17,	1,17,	
1,17,	1,17,	1,17,	1,17,	
1,17,	1,17,	1,17,	1,29,	
1,17,	1,30,	1,17,	1,17,	
1,31,	1,17,	1,17,	1,17,	
1,17,	1,17,	1,32,	1,33,	
1,34,	6,7,	1,35,	6,9,	
6,9,	6,9,	6,9,	6,9,	
6,9,	6,9,	6,9,	6,9,	
6,9,	7,36,	7,36,	7,36,	
7,36,	7,36,	7,36,	7,36,	
7,36,	7,36,	7,36,	12,17,	
12,17,	21,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
15,41,	23,17,	29,17,	32,17,	
36,61,	15,17,	28,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,38,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	28,57,	38,62,	41,65,	
36,61,	12,17,	45,69,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	12,17,	12,17,	12,17,	
12,17,	16,42,	18,43,	22,49,	
19,45,	20,46,	26,55,	27,17,	
30,17,	16,17,	19,17,	31,17,	
18,44,	20,47,	26,17,	24,51,	
25,53,	18,17,	33,17,	20,48,	
40,64,	22,17,	25,54,	22,50,	
20,17,	24,52,	39,63,	25,17,	
39,17,	42,66,	24,17,	27,56,	
30,58,	43,67,	33,60,	44,68,	
40,17,	52,77,	31,59,	46,70,	
47,72,	48,73,	50,75,	44,17,	
42,17,	46,71,	47,17,	48,17,	
49,74,	51,76,	50,17,	53,78,	
43,17,	51,17,	54,79,	55,80,	
52,17,	56,17,	49,17,	57,17,	
58,17,	59,17,	54,17,	60,17,	
53,17,	62,88,	55,17,	62,17,	
64,90,	65,91,	64,17,	63,89,	
66,17,	58,83,	66,92,	76,103,	
78,105,	95,122,	56,81,	95,17,	
76,17,	105,17,	78,17,	123,17,	
57,82,	59,84,	61,86,	63,17,	
61,86,	67,93,	60,85,	61,87,	
61,87,	61,87,	61,87,	61,87,	
61,87,	61,87,	61,87,	61,87,	
61,87,	68,94,	73,100,	69,95,	
70,96,	67,17,	71,97,	72,99,	
75,102,	74,101,	77,104,	81,17,	
82,17,	83,17,	79,106,	73,17,	
80,107,	68,17,	69,17,	70,17,	
84,17,	85,17,	72,17,	71,17,	
74,17,	79,17,	77,17,	75,17,	
127,17,	80,17,	81,108,	97,17,	
97,125,	129,154,	71,98,	84,111,	
89,114,	82,109,	90,115,	83,110,	
86,87,	86,87,	86,87,	86,87,	
86,87,	86,87,	86,87,	86,87,	
86,87,	86,87,	88,17,	89,17,	
91,116,	92,117,	93,118,	90,17,	
85,112,	98,126,	96,123,	96,17,	
92,17,	88,113,	96,124,	94,119,	
99,127,	99,17,	94,120,	91,17,	
100,128,	93,17,	94,121,	94,17,	
101,129,	102,130,	98,17,	103,131,	
106,133,	102,17,	104,132,	104,17,	
108,17,	109,17,	107,134,	110,17,	
111,17,	100,17,	112,17,	113,140,	
114,141,	101,17,	114,17,	116,143,	
106,17,	103,17,	107,17,	115,17,	
115,142,	117,144,	109,136,	118,145,	
119,146,	118,17,	121,148,	113,17,	
111,138,	120,147,	108,135,	116,17,	
122,149,	122,17,	124,150,	119,17,	
112,139,	121,17,	130,17,	110,137,	
117,17,	131,155,	125,151,	120,17,	
125,17,	124,17,	126,17,	126,152,	
128,17,	128,153,	132,17,	131,17,	
133,17,	134,156,	135,17,	136,17,	
137,17,	138,17,	139,17,	140,160,	
141,17,	142,161,	143,17,	142,17,	
144,162,	145,17,	146,163,	148,165,	
144,17,	149,166,	140,17,	135,157,	
146,17,	150,167,	138,159,	147,164,	
147,17,	151,17,	156,17,	151,168,	
148,17,	136,158,	157,17,	152,169,	
149,17,	152,17,	153,170,	158,17,	
153,17,	150,17,	154,171,	154,17,	
155,172,	155,17,	159,17,	160,176,	
162,17,	161,17,	157,173,	161,177,	
163,178,	164,17,	165,179,	166,180,	
167,181,	169,17,	168,182,	169,183,	
165,17,	159,175,	171,17,	170,17,	
160,17,	170,184,	172,17,	163,17,	
173,17,	174,17,	175,17,	158,174,	
166,17,	168,17,	176,186,	177,187,	
178,17,	179,17,	180,188,	181,189,	
182,17,	183,190,	184,191,	185,17,	
186,17,	187,17,	189,17,	188,193,	
190,194,	190,17,	177,17,	181,17,	
191,17,	175,185,	192,17,	193,196,	
183,17,	184,17,	188,17,	194,17,	
193,17,	195,17,	196,198,	197,17,	
198,199,	198,17,	199,17,	0,0,	
196,17,	0,0,	0,0,	0,0,	
0,0,	0,0,	185,192,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	0,0,	0,0,	
0,0,	0,0,	195,197,	192,195,	
0,0};
struct yysvf yysvec[] = {
0,	0,	0,
yycrank+1,	0,		0,	
yycrank+0,	yysvec+1,	0,	
yycrank+0,	0,		yyvstop+1,
yycrank+0,	0,		yyvstop+3,
yycrank+0,	0,		yyvstop+5,
yycrank+79,	0,		0,	
yycrank+89,	0,		0,	
yycrank+1,	0,		0,	
yycrank+0,	yysvec+6,	yyvstop+7,
yycrank+0,	0,		yyvstop+9,
yycrank+0,	0,		yyvstop+11,
yycrank+102,	0,		yyvstop+13,
yycrank+13,	yysvec+12,	yyvstop+15,
yycrank+11,	yysvec+12,	yyvstop+17,
yycrank+81,	yysvec+12,	yyvstop+19,
yycrank+149,	yysvec+12,	yyvstop+21,
yycrank+10,	yysvec+12,	yyvstop+23,
yycrank+157,	yysvec+12,	yyvstop+25,
yycrank+150,	yysvec+12,	yyvstop+27,
yycrank+164,	yysvec+12,	yyvstop+29,
yycrank+65,	yysvec+12,	yyvstop+31,
yycrank+161,	yysvec+12,	yyvstop+34,
yycrank+77,	yysvec+12,	yyvstop+36,
yycrank+170,	yysvec+12,	yyvstop+39,
yycrank+167,	yysvec+12,	yyvstop+41,
yycrank+154,	yysvec+12,	yyvstop+43,
yycrank+147,	yysvec+12,	yyvstop+45,
yycrank+82,	yysvec+12,	yyvstop+47,
yycrank+78,	yysvec+12,	yyvstop+49,
yycrank+148,	yysvec+12,	yyvstop+52,
yycrank+151,	yysvec+12,	yyvstop+54,
yycrank+79,	yysvec+12,	yyvstop+56,
yycrank+158,	yysvec+12,	yyvstop+59,
yycrank+0,	0,		yyvstop+61,
yycrank+0,	0,		yyvstop+63,
yycrank+95,	yysvec+7,	yyvstop+65,
yycrank+0,	0,		yyvstop+67,
yycrank+110,	yysvec+12,	yyvstop+69,
yycrank+168,	yysvec+12,	yyvstop+71,
yycrank+176,	yysvec+12,	yyvstop+73,
yycrank+111,	yysvec+12,	yyvstop+75,
yycrank+184,	yysvec+12,	yyvstop+77,
yycrank+192,	yysvec+12,	yyvstop+79,
yycrank+183,	yysvec+12,	yyvstop+81,
yycrank+114,	yysvec+12,	yyvstop+83,
yycrank+185,	yysvec+12,	yyvstop+85,
yycrank+186,	yysvec+12,	yyvstop+87,
yycrank+187,	yysvec+12,	yyvstop+89,
yycrank+198,	yysvec+12,	yyvstop+91,
yycrank+190,	yysvec+12,	yyvstop+93,
yycrank+193,	yysvec+12,	yyvstop+95,
yycrank+196,	yysvec+12,	yyvstop+97,
yycrank+204,	yysvec+12,	yyvstop+99,
yycrank+202,	yysvec+12,	yyvstop+101,
yycrank+206,	yysvec+12,	yyvstop+103,
yycrank+197,	yysvec+12,	yyvstop+105,
yycrank+199,	yysvec+12,	yyvstop+107,
yycrank+200,	yysvec+12,	yyvstop+109,
yycrank+201,	yysvec+12,	yyvstop+111,
yycrank+203,	yysvec+12,	yyvstop+113,
yycrank+267,	0,		0,	
yycrank+207,	yysvec+12,	yyvstop+115,
yycrank+227,	yysvec+12,	yyvstop+117,
yycrank+210,	yysvec+12,	yyvstop+119,
yycrank+209,	yysvec+12,	yyvstop+121,
yycrank+212,	yysvec+12,	yyvstop+123,
yycrank+245,	yysvec+12,	yyvstop+125,
yycrank+257,	yysvec+12,	yyvstop+127,
yycrank+258,	yysvec+12,	yyvstop+129,
yycrank+259,	yysvec+12,	yyvstop+131,
yycrank+263,	yysvec+12,	yyvstop+133,
yycrank+262,	yysvec+12,	yyvstop+135,
yycrank+255,	yysvec+12,	yyvstop+137,
yycrank+264,	yysvec+12,	yyvstop+139,
yycrank+267,	yysvec+12,	yyvstop+141,
yycrank+220,	yysvec+12,	yyvstop+143,
yycrank+266,	yysvec+12,	yyvstop+145,
yycrank+222,	yysvec+12,	yyvstop+147,
yycrank+265,	yysvec+12,	yyvstop+149,
yycrank+269,	yysvec+12,	yyvstop+151,
yycrank+251,	yysvec+12,	yyvstop+153,
yycrank+252,	yysvec+12,	yyvstop+155,
yycrank+253,	yysvec+12,	yyvstop+157,
yycrank+260,	yysvec+12,	yyvstop+159,
yycrank+261,	yysvec+12,	yyvstop+161,
yycrank+316,	0,		0,	
yycrank+0,	yysvec+86,	yyvstop+163,
yycrank+290,	yysvec+12,	yyvstop+165,
yycrank+291,	yysvec+12,	yyvstop+168,
yycrank+295,	yysvec+12,	yyvstop+170,
yycrank+307,	yysvec+12,	yyvstop+173,
yycrank+300,	yysvec+12,	yyvstop+175,
yycrank+309,	yysvec+12,	yyvstop+177,
yycrank+311,	yysvec+12,	yyvstop+179,
yycrank+219,	yysvec+12,	yyvstop+181,
yycrank+299,	yysvec+12,	yyvstop+183,
yycrank+271,	yysvec+12,	yyvstop+185,
yycrank+314,	yysvec+12,	yyvstop+187,
yycrank+305,	yysvec+12,	yyvstop+189,
yycrank+325,	yysvec+12,	yyvstop+191,
yycrank+329,	yysvec+12,	yyvstop+193,
yycrank+317,	yysvec+12,	yyvstop+195,
yycrank+333,	yysvec+12,	yyvstop+197,
yycrank+319,	yysvec+12,	yyvstop+199,
yycrank+221,	yysvec+12,	yyvstop+201,
yycrank+332,	yysvec+12,	yyvstop+204,
yycrank+334,	yysvec+12,	yyvstop+206,
yycrank+320,	yysvec+12,	yyvstop+208,
yycrank+321,	yysvec+12,	yyvstop+210,
yycrank+323,	yysvec+12,	yyvstop+212,
yycrank+324,	yysvec+12,	yyvstop+214,
yycrank+326,	yysvec+12,	yyvstop+216,
yycrank+343,	yysvec+12,	yyvstop+218,
yycrank+330,	yysvec+12,	yyvstop+220,
yycrank+335,	yysvec+12,	yyvstop+222,
yycrank+347,	yysvec+12,	yyvstop+224,
yycrank+356,	yysvec+12,	yyvstop+226,
yycrank+341,	yysvec+12,	yyvstop+228,
yycrank+351,	yysvec+12,	yyvstop+230,
yycrank+359,	yysvec+12,	yyvstop+232,
yycrank+353,	yysvec+12,	yyvstop+234,
yycrank+349,	yysvec+12,	yyvstop+236,
yycrank+223,	yysvec+12,	yyvstop+238,
yycrank+361,	yysvec+12,	yyvstop+241,
yycrank+360,	yysvec+12,	yyvstop+243,
yycrank+362,	yysvec+12,	yyvstop+245,
yycrank+268,	yysvec+12,	yyvstop+247,
yycrank+364,	yysvec+12,	yyvstop+250,
yycrank+273,	yysvec+12,	yyvstop+252,
yycrank+354,	yysvec+12,	yyvstop+254,
yycrank+367,	yysvec+12,	yyvstop+257,
yycrank+366,	yysvec+12,	yyvstop+259,
yycrank+368,	yysvec+12,	yyvstop+262,
yycrank+369,	yysvec+12,	yyvstop+265,
yycrank+370,	yysvec+12,	yyvstop+267,
yycrank+371,	yysvec+12,	yyvstop+269,
yycrank+372,	yysvec+12,	yyvstop+271,
yycrank+373,	yysvec+12,	yyvstop+274,
yycrank+374,	yysvec+12,	yyvstop+276,
yycrank+386,	yysvec+12,	yyvstop+279,
yycrank+376,	yysvec+12,	yyvstop+281,
yycrank+379,	yysvec+12,	yyvstop+284,
yycrank+378,	yysvec+12,	yyvstop+286,
yycrank+384,	yysvec+12,	yyvstop+289,
yycrank+381,	yysvec+12,	yyvstop+291,
yycrank+388,	yysvec+12,	yyvstop+294,
yycrank+392,	yysvec+12,	yyvstop+296,
yycrank+396,	yysvec+12,	yyvstop+298,
yycrank+400,	yysvec+12,	yyvstop+300,
yycrank+405,	yysvec+12,	yyvstop+302,
yycrank+393,	yysvec+12,	yyvstop+304,
yycrank+401,	yysvec+12,	yyvstop+306,
yycrank+404,	yysvec+12,	yyvstop+308,
yycrank+407,	yysvec+12,	yyvstop+310,
yycrank+409,	yysvec+12,	yyvstop+312,
yycrank+394,	yysvec+12,	yyvstop+314,
yycrank+398,	yysvec+12,	yyvstop+317,
yycrank+403,	yysvec+12,	yyvstop+319,
yycrank+410,	yysvec+12,	yyvstop+321,
yycrank+428,	yysvec+12,	yyvstop+323,
yycrank+413,	yysvec+12,	yyvstop+325,
yycrank+412,	yysvec+12,	yyvstop+327,
yycrank+431,	yysvec+12,	yyvstop+330,
yycrank+417,	yysvec+12,	yyvstop+332,
yycrank+424,	yysvec+12,	yyvstop+335,
yycrank+436,	yysvec+12,	yyvstop+337,
yycrank+420,	yysvec+12,	yyvstop+339,
yycrank+437,	yysvec+12,	yyvstop+341,
yycrank+421,	yysvec+12,	yyvstop+343,
yycrank+427,	yysvec+12,	yyvstop+345,
yycrank+426,	yysvec+12,	yyvstop+347,
yycrank+430,	yysvec+12,	yyvstop+350,
yycrank+432,	yysvec+12,	yyvstop+353,
yycrank+433,	yysvec+12,	yyvstop+356,
yycrank+434,	yysvec+12,	yyvstop+359,
yycrank+438,	yysvec+12,	yyvstop+361,
yycrank+454,	yysvec+12,	yyvstop+363,
yycrank+440,	yysvec+12,	yyvstop+365,
yycrank+441,	yysvec+12,	yyvstop+368,
yycrank+442,	yysvec+12,	yyvstop+371,
yycrank+455,	yysvec+12,	yyvstop+373,
yycrank+444,	yysvec+12,	yyvstop+375,
yycrank+460,	yysvec+12,	yyvstop+378,
yycrank+461,	yysvec+12,	yyvstop+380,
yycrank+447,	yysvec+12,	yyvstop+382,
yycrank+448,	yysvec+12,	yyvstop+384,
yycrank+449,	yysvec+12,	yyvstop+387,
yycrank+462,	yysvec+12,	yyvstop+390,
yycrank+450,	yysvec+12,	yyvstop+392,
yycrank+453,	yysvec+12,	yyvstop+395,
yycrank+456,	yysvec+12,	yyvstop+397,
yycrank+458,	yysvec+12,	yyvstop+400,
yycrank+464,	yysvec+12,	yyvstop+402,
yycrank+463,	yysvec+12,	yyvstop+404,
yycrank+465,	yysvec+12,	yyvstop+407,
yycrank+472,	yysvec+12,	yyvstop+409,
yycrank+467,	yysvec+12,	yyvstop+411,
yycrank+469,	yysvec+12,	yyvstop+414,
yycrank+470,	yysvec+12,	yyvstop+416,
0,	0,	0};
struct yywork *yytop = yycrank+575;
struct yysvf *yybgin = yysvec+1;
unsigned char yymatch[] = {
00  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,011 ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
011 ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,'+' ,01  ,'-' ,'.' ,01  ,
'0' ,'0' ,'0' ,'0' ,'0' ,'0' ,'0' ,'0' ,
'0' ,'0' ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,'A' ,'A' ,'A' ,'A' ,'E' ,'A' ,'A' ,
'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,
'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,
'A' ,'A' ,'A' ,01  ,01  ,01  ,01  ,'A' ,
01  ,'A' ,'A' ,'A' ,'A' ,'E' ,'A' ,'A' ,
'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,
'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,'A' ,
'A' ,'A' ,'A' ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
01  ,01  ,01  ,01  ,01  ,01  ,01  ,01  ,
0};
char yyextra[] = {
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,
0};
#ident	"$Revision: 1.1 $"

int yylineno =1;
# define YYU(x) x
# define NLSTATE yyprevious=YYNEWLINE
char yytext[YYLMAX];
struct yysvf *yylstate [YYLMAX], **yylsp, **yyolsp;
char yysbuf[YYLMAX];
char *yysptr = yysbuf;
int *yyfnd;
extern struct yysvf *yyestate;
int yyprevious = YYNEWLINE;
#if defined(__cplusplus) || defined(__STDC__)
int yylook(void)
#else
yylook()
#endif
{
	register struct yysvf *yystate, **lsp;
	register struct yywork *yyt;
	struct yysvf *yyz;
	int yych, yyfirst;
	struct yywork *yyr;
# ifdef LEXDEBUG
	int debug;
# endif
	char *yylastch;
	/* start off machines */
# ifdef LEXDEBUG
	debug = 0;
# endif
	yyfirst=1;
	if (!yymorfg)
		yylastch = yytext;
	else {
		yymorfg=0;
		yylastch = yytext+yyleng;
		}
	for(;;){
		lsp = yylstate;
		yyestate = yystate = yybgin;
		if (yyprevious==YYNEWLINE) yystate++;
		for (;;){
# ifdef LEXDEBUG
			if(debug)fprintf(yyout,"state %d\n",yystate-yysvec-1);
# endif
			yyt = yystate->yystoff;
			if(yyt == yycrank && !yyfirst){  /* may not be any transitions */
				yyz = yystate->yyother;
				if(yyz == 0)break;
				if(yyz->yystoff == yycrank)break;
				}
			*yylastch++ = yych = input();
#ifndef LONGLINES
			if(yylastch > &yytext[YYLMAX]) {
				fprintf(yyout,"Input string too long, limit %d\n",YYLMAX);
				exit(1);
			}
#endif
			yyfirst=0;
		tryagain:
# ifdef LEXDEBUG
			if(debug){
				fprintf(yyout,"char ");
				allprint(yych);
				putchar('\n');
				}
# endif
			yyr = yyt;
			if ( (void *)yyt > (void *)yycrank){
				yyt = yyr + yych;
				if (yyt <= yytop && yyt->verify+yysvec == yystate){
					if(yyt->advance+yysvec == YYLERR)	/* error transitions */
						{unput(*--yylastch);break;}
					*lsp++ = yystate = yyt->advance+yysvec;
#ifndef LONGLINES
					if(lsp > &yylstate[YYLMAX]) {
						fprintf(yyout,"Input string too long, limit %d\n",YYLMAX);
						exit(1);
					}
#endif
					goto contin;
					}
				}
# ifdef YYOPTIM
			else if((void *)yyt < (void *)yycrank) {	/* r < yycrank */
				yyt = yyr = yycrank+(yycrank-yyt);
# ifdef LEXDEBUG
				if(debug)fprintf(yyout,"compressed state\n");
# endif
				yyt = yyt + yych;
				if(yyt <= yytop && yyt->verify+yysvec == yystate){
					if(yyt->advance+yysvec == YYLERR)	/* error transitions */
						{unput(*--yylastch);break;}
					*lsp++ = yystate = yyt->advance+yysvec;
#ifndef LONGLINES
					if(lsp > &yylstate[YYLMAX]) {
						fprintf(yyout,"Input string too long, limit %d\n",YYLMAX);
						exit(1);
					}
#endif
					goto contin;
					}
				yyt = yyr + YYU(yymatch[yych]);
# ifdef LEXDEBUG
				if(debug){
					fprintf(yyout,"try fall back character ");
					allprint(YYU(yymatch[yych]));
					putchar('\n');
					}
# endif
				if(yyt <= yytop && yyt->verify+yysvec == yystate){
					if(yyt->advance+yysvec == YYLERR)	/* error transition */
						{unput(*--yylastch);break;}
					*lsp++ = yystate = yyt->advance+yysvec;
#ifndef LONGLINES
					if(lsp > &yylstate[YYLMAX]) {
						fprintf(yyout,"Input string too long, limit %d\n",YYLMAX);
						exit(1);
					}
#endif
					goto contin;
					}
				}
			if ((yystate = yystate->yyother) && (yyt= yystate->yystoff) != yycrank){
# ifdef LEXDEBUG
				if(debug)fprintf(yyout,"fall back to state %d\n",yystate-yysvec-1);
# endif
				goto tryagain;
				}
# endif
			else
				{unput(*--yylastch);break;}
		contin:
# ifdef LEXDEBUG
			if(debug){
				fprintf(yyout,"state %d char ",yystate-yysvec-1);
				allprint(yych);
				putchar('\n');
				}
# endif
			;
			}
# ifdef LEXDEBUG
		if(debug){
			fprintf(yyout,"stopped at %d with ",*(lsp-1)-yysvec-1);
			allprint(yych);
			putchar('\n');
			}
# endif
		while (lsp-- > yylstate){
			*yylastch-- = 0;
			if (*lsp != 0 && (yyfnd= (*lsp)->yystops) && *yyfnd > 0){
				yyolsp = lsp;
				if(yyextra[*yyfnd]){		/* must backup */
					while(yyback((*lsp)->yystops,-*yyfnd) != 1 && lsp > yylstate){
						lsp--;
						unput(*yylastch--);
						}
					}
				yyprevious = YYU(*yylastch);
				yylsp = lsp;
				yyleng = (int)(yylastch-yytext+1);
				yytext[yyleng] = 0;
# ifdef LEXDEBUG
				if(debug){
					fprintf(yyout,"\nmatch ");
					sprint(yytext);
					fprintf(yyout," action %d\n",*yyfnd);
					}
# endif
				return(*yyfnd++);
				}
			unput(*yylastch);
			}
		if (yytext[0] == 0  /* && feof(yyin) */)
			{
			yysptr=yysbuf;
			return(0);
			}
		yyprevious = yytext[0] = input();
		if (yyprevious>0)
			output(yyprevious);
		yylastch=yytext;
# ifdef LEXDEBUG
		if(debug)putchar('\n');
# endif
		}
	}
#if defined(__cplusplus) || defined(__STDC__)
int yyback(int *p, int m)
#else
yyback(p, m)
	int *p;
#endif
{
	if (p==0) return(0);
	while (*p) {
		if (*p++ == m)
			return(1);
	}
	return(0);
}
	/* the following are only used in the lex library */
#if defined(__cplusplus) || defined(__STDC__)
int yyinput(void)
#else
yyinput()
#endif
{
	return(input());
	}
#if defined(__cplusplus) || defined(__STDC__)
void yyoutput(int c)
#else
yyoutput(c)
  int c; 
#endif
{
	output(c);
	}
#if defined(__cplusplus) || defined(__STDC__)
void yyunput(int c)
#else
yyunput(c)
   int c; 
#endif
{
	unput(c);
	}
