##############################################################################
#
# (C) Copyright 1998 by National Advanced Driving Simulator and
# Simulation Center, the University of Iowa and The University
# of Iowa. All rights reserved.
#
# $Id: Makefile,v 1.47 2000/12/21 22:53:21 schikore Exp $
#
# Author(s):    Yiannis Papelis
# Date:         August, 1998
#
# Description:  Makefile for CVED software.
#
##############################################################################

.SUFFIXES: .cxx .c .cpp

##############################################################################
SHELL      = /bin/sh
BINS       = bin/vepeek  bin/vesizes bin/lritool bin/writebin

#### standard rule for compilation
.cxx.o:
	$(CXXOPT) $(CFLAGS) $(INCLUDES) $<
#--implicit_include 
#	/people/papelis/bin/cc++ -g --instantiate used --force_vtbl $(CFLAGS) $(INCLUDES) $<
.c.o:
	$(CCOPT) $(CFLAGS) $<

INCDIR     = ../include
LIBDIR     = ../../lib
CVED       = libcved.a
TARGET     = $(LIBDIR)/$(CVED)

CXXFLAGS     = -I$(INCDIR) -I../../include 
#-woff 1355,1209,1155,1174,3201
CFLAGS       = -I$(INCDIR) -I../../include 

# library object files.  Append new files in this list and add
# dependencies to header files later.
OBJ =  lanemask.o cvederr.o attr.o cved.o cveditem.o obj.o dynobj.o \
lane.o road.o roadpos.o intrsctn.o sharedmem.o crdr.o objtype.o enumtostring.o \
cntrlpnt.o dynserv.o terrain.o objmask.o cvedversionnum.o dynobjreflist.o  \
vehicledynamics.o path.o pathpoint.o pathnetwork.o enviro.o hldofs.o \
objattr.o collision.o objreflistUtl.o

HEADERS = $(INCDIR)/attr.h $(INCDIR)/crdr.h $(INCDIR)/enumtostring.h \
		$(INCDIR)/cved.h $(INCDIR)/cveddecl.h $(INCDIR)/cvederr.h \
		$(INCDIR)/cveditem.h $(INCDIR)/cvedpub.h $(INCDIR)/cvedstrc.h \
		$(INCDIR)/dynobj.h $(INCDIR)/intrsctn.h $(INCDIR)/lane.h \
		$(INCDIR)/lanemask.h $(INCDIR)/obj.h $(INCDIR)/objlayout.h \
		$(INCDIR)/objmask.h $(INCDIR)/objtypes.h $(INCDIR)/reconfobj.h \
		$(INCDIR)/road.h $(INCDIR)/roadpos.h  $(INCDIR)/sharedmem.h \
		$(INCDIR)/cntrlpnt.h $(INCDIR)/cvedversionnum.h \
		$(INCDIR)/objreflistUtl.h \
		$(INCDIR)/terrain.h $(INCDIR)/dynobjreflist.h $(INCDIR)/dynobj.inl \
		$(INCDIR)/objmask.inl $(INCDIR)/road.inl $(INCDIR)/path.h \
		$(INCDIR)/pathpoint.h $(INCDIR)/enviro.h $(INCDIR)/hldofs.h \
		$(INCDIR)/pathnetwork.h $(INCDIR)/objattr.h

##### default target is the library in the cved/lib directory
all: $(TARGET) #dyntest


install: $(TARGET) #dyntest

#$(TARGET): $(CVED)
#	/bin/cp -p $(CVED) $(LIBDIR)

clean:
#/bin/rm -f $(OBJ) $(CVED)
	/bin/rm -f $(OBJ) $(TARGET)
#/bin/rm -f dyntest
	/bin/rm -rf ii_files
	/bin/rm -rf *.ti *.ii

##### tell 'make' that we are building components of a library
$(TARGET): $(OBJ)
	$(AR) $(TARGET) $?

###### dynamics test exe
 dyntest: $(OBJ)
	$(LINKCPP) -o dyntest $(OBJ) -L../../lib -L$(LIBDIR) -lsol -lmisc -lm

#TESTOBJ = \
#  lanemask.o\
# cvederr.o attr.o cved.o cveditem.o obj.o \
#lane.o road.o roadpos.o intrsctn.o sharedmem.o crdr.o objtype.o \
#cntrlpnt.o terrain.o dynserv.o\
#dynobj.o objmask.o
#
#testMisc: cleanup  testMisc.o 
#	$(LINKCPP) -o  test  testMisc.o  -L. -L../../lib -lmisc -lsol
#testMisc.o:
#	rm -f testMisc.o
#	/people/papelis/bin/cc++ -c -g  --auto_instantiation $(CFLAGS)  --implicit_include $(INCLUDES) testMisc.cxx
#/people/papelis/bin/cc++ -c -g  --force_vtbl $(CFLAGS)  --implicit_include $(INCLUDES) testMisc.cxx       
#cleanup:
#	rm -f testMisc.o testMisc
#
#testCved: libLudo.a $(TESTOBJ)
#	$(LINKCPP) -o  test testMisc.o  -L. -lLudo -L../../lib -lmisc -lm
#
#
#libLudo.a: $(TESTOBJ)
#	$(AR)  libLudo.a $(TESTOBJ)

########################################################################
#
# Individual dependencies follow here.  No need to put rules on
# how to build them, just the necessary header files.  Remember that
# header files reside in the cved/include directory.
#
attr.o      : attr.cxx        $(HEADERS)
crdr.o      : crdr.cxx        $(HEADERS)
cved.o      : cved.cxx        $(HEADERS)
terrain.o   : terrain.cxx     $(HEADERS)
cvederr.o   : cvederr.cxx     $(HEADERS)
cveditem.o  : cveditem.cxx    $(HEADERS)
dynobj.o    : dynobj.cxx      $(HEADERS)
objmask.o   : objmask.cxx     $(HEADERS)
	$(CXXSPEOPT2) $(CFLAGS) $(INCLUDES) objmask.cxx 
#	/people/papelis/bin/cc++ -c -g  --force_vtbl $(CFLAGS)  --implicit_include $(INCLUDES) objmask.cxx 
intrsctn.o  : intrsctn.cxx    $(HEADERS)
lane.o      : lane.cxx        $(HEADERS)
lanemask.o  : lanemask.cxx    $(HEADERS)
obj.o       : obj.cxx         $(HEADERS)
reconfobj.o : reconfobj.cx    $(HEADERS)
road.o      : road.cxx        $(HEADERS)
roadpos.o   : roadpos.cxx     $(HEADERS)
sharedmem.o : sharedmem.cxx   $(HEADERS)
cntrlpnt.o  : cntrlpnt.cxx    $(HEADERS)
path.o      : path.cxx		  $(HEADERS)
pathpoint.o : pathpoint.cxx   $(HEADERS)
pathnetwork.o : pathnetwork.cxx   $(HEADERS)
enviro.o    : enviro.cxx      $(HEADERS)
hldofs.o    : hldofs.cxx      $(HEADERS)
objattr.o   : objattr.cxx     $(HEADERS)
collision.o : collision.cxx   $(HEADERS)
objreflistUtl.o : objreflistUtl.cxx $(HEADERS)
dynobjreflist.o  : dynobjreflist.cxx    $(HEADERS)
	$(CXXSPEOPT) $(CFLAGS) $(INCLUDES) dynobjreflist.cxx

