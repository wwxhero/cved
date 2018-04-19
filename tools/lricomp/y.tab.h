
typedef union
#ifdef __cplusplus
	YYSTYPE
#endif

{
	int      			ival;
	double   			dval;
	char     			*strval;
	unsigned 			uval;
	TLatCurve			latcurvedef;
	TLatCurve*			latcurveptr;
	TLatCpoint			latcpointdef;
	TCurveDef*			curvedefptr;
	TAttribute			attributedef;
	TAttribute*			attributeptr;
	TLaneDef 			lanedef;
	TLaneList			lanelistdef;
	TControlPoint		controlpointdef;
	TControlPoint*  	controlpointptr;
	TCpointList			cpointlistdef;
	TCpointInfo			cpointinfodef;
	TCpointInfo*		cpointinfoptr;
	TLaneWidth			lanewidthdef;
	TLaneWidth*			lanewidthptr;
	TIntersection*     intersectionsptr;
	TElevMap*           elevmapptr;
	TElevMap            elevmap;
	TElevMap2*          elevmap2ptr;
	TElevInfo           elevinfodef;
	TRoadName*          roadptr;
	TBorderPt           borderptdef;
	TBorder             borderdef;
	TCrdr               crdrdef;
	TCrdr               *crdrdefptr;
	THoldOfs*           holdofsptr;
	THoldOfs            holdofs;
	TCrdrCurveList      crdrcurvelist;
	TCrdrCurve          crdrcurvedef;
	TCrdrLineInfo*      crdrlineinfoptr;
	THoldOfsInfo*       holdofsinfoptr;
	THoldOfsInfo        holdofsinfo;
	TRepObj*            repobjptr;
	TObject             objdef;
} YYSTYPE;
extern YYSTYPE yylval;
# define LRI_ROADS 500
# define LRI_LANES 501
# define LRI_POSITIVE 503
# define LRI_NEGATIVE 504
# define LRI_CURLY_OPEN 505
# define LRI_CURLY_CLOSE 506
# define LRI_DIRECTION 507
# define LRI_IDENTIFIER 513
# define LRI_LAT_CURVES 514
# define LRI_LATCURVE 515
# define LRI_LANEWIDTH 516
# define LRI_ATTR 517
# define LRI_LONGCURVE 518
# define LRI_SEMI 519
# define LRI_INTERSECTIONS 520
# define LRI_ELEVMAP 521
# define LRI_BORDER 522
# define LRI_CRDR 523
# define LRI_HOLDOFS 524
# define LRI_CRDR_CURVE 525
# define LRI_HOLDLINE 526
# define LRI_HOLDSIGN 527
# define LRI_EQUALTO 528
# define LRI_LINES 529
# define LRI_LFLAG 530
# define LRI_CRDR_REASON 531
# define LRI_LSTYLE 532
# define LRI_REAL 533
# define LRI_REPOBJS 534
# define LRI_ALIGNED 535
# define LRI_OBJECTS 536
# define LRI_PLANT 537
# define LRI_ZDOWN 539
# define LRI_SOLCHECKSUM 540
# define LRI_HEADER 541
# define LRI_QUOTED_STRING 542
# define LRI_COMMENT 543
# define LRI_ATTR_DICT 544
