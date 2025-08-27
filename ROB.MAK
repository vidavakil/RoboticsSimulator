#
# Borland C++ IDE generated makefile
# Generated 1/20/2002 at 7:02:44 PM 
#
.AUTODEPEND


#
# Borland C++ tools
#
IMPLIB  = Implib
BCC     = Bcc +BccW16.cfg 
TLINK   = TLink
TLIB    = TLib
BRC     = Brc
TASM    = Tasm
#
# IDE macros
#


#
# Options
#
IDE_LinkFLAGS =  -LC:\BC5\LIB;C:\BC5\BIN -c
IDE_ResFLAGS = 
LinkerLocalOptsAtW16_CcbROBOT2bFINALbrobdexe =  -Twe -c -C -LC:\BC5\LIB;C:\BC5\BIN -n -i -3
ResLocalOptsAtW16_CcbROBOT2bFINALbrobdexe =  -31
BLocalOptsAtW16_CcbROBOT2bFINALbrobdexe = 
CompInheritOptsAt_CcbROBOT2bFINALbrobdexe = -IC:\BC5\INCLUDE;C:\ROBOT2 -D_RTLDLL;
LinkerInheritOptsAt_CcbROBOT2bFINALbrobdexe = 
LinkerOptsAt_CcbROBOT2bFINALbrobdexe = $(LinkerLocalOptsAtW16_CcbROBOT2bFINALbrobdexe)
ResOptsAt_CcbROBOT2bFINALbrobdexe = $(ResLocalOptsAtW16_CcbROBOT2bFINALbrobdexe)
BOptsAt_CcbROBOT2bFINALbrobdexe = $(BLocalOptsAtW16_CcbROBOT2bFINALbrobdexe)

#
# Dependency List
#
Dep_rob = \
   C:\ROBOT2\FINAL\rob.exe

rob : BccW16.cfg $(Dep_rob)
  echo MakeNode

Dep_CcbROBOT2bFINALbrobdexe = \
   C:\ROBOT2\INTERMEDIATE\robot.res\
   C:\ROBOT2\INTERMEDIATE\win-rob.obj\
   C:\ROBOT2\INTERMEDIATE\sw-volum.obj\
   C:\ROBOT2\INTERMEDIATE\rhino.obj\
   C:\ROBOT2\INTERMEDIATE\plan3d.obj\
   C:\ROBOT2\INTERMEDIATE\plan2d.obj\
   C:\ROBOT2\INTERMEDIATE\intface.obj\
   C:\ROBOT2\INTERMEDIATE\display.obj\
   C:\ROBOT2\INTERMEDIATE\col-det.obj\
   C:\ROBOT2\INTERMEDIATE\atlas.obj\
   rob.exe

C:\ROBOT2\FINAL\rob.exe : $(Dep_CcbROBOT2bFINALbrobdexe)
  $(TLINK)   @&&|
 /v $(IDE_LinkFLAGS) $(LinkerOptsAt_CcbROBOT2bFINALbrobdexe) $(LinkerInheritOptsAt_CcbROBOT2bFINALbrobdexe) +
C:\BC5\LIB\c0wl.obj+
C:\ROBOT2\win-rob.obj+
C:\ROBOT2\sw-volum.obj+
C:\ROBOT2\rhino.obj+
C:\ROBOT2\plan3d.obj+
C:\ROBOT2\plan2d.obj+
C:\ROBOT2\intface.obj+
C:\ROBOT2\display.obj+
C:\ROBOT2\col-det.obj+
C:\ROBOT2\atlas.obj
$<,$*
C:\BC5\LIB\bwcc.lib+
C:\BC5\LIB\import.lib+
C:\BC5\LIB\crtldll.lib

|
   $(BRC) C:\ROBOT2\INTERMEDIATE\robot.res $<

C:\ROBOT2\INTERMEDIATE\robot.res :  robot.rc
  $(BRC) -R @&&|
 $(IDE_ResFLAGS) $(ROptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe)  -FO$@ robot.rc
|
C:\ROBOT2\INTERMEDIATE\win-rob.obj :  win-rob.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ win-rob.cpp
|

C:\ROBOT2\INTERMEDIATE\sw-volum.obj :  sw-volum.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ sw-volum.cpp
|

C:\ROBOT2\INTERMEDIATE\rhino.obj :  rhino.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ rhino.cpp
|

C:\ROBOT2\INTERMEDIATE\plan3d.obj :  plan3d.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ plan3d.cpp
|

C:\ROBOT2\INTERMEDIATE\plan2d.obj :  plan2d.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ plan2d.cpp
|

C:\ROBOT2\INTERMEDIATE\intface.obj :  intface.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ intface.cpp
|

C:\ROBOT2\INTERMEDIATE\display.obj :  display.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ display.cpp
|

C:\ROBOT2\INTERMEDIATE\col-det.obj :  col-det.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ col-det.cpp
|

C:\ROBOT2\INTERMEDIATE\atlas.obj :  atlas.cpp
  $(BCC)   -c @&&|
 $(CompOptsAt_CcbROBOT2bFINALbrobdexe) $(CompInheritOptsAt_CcbROBOT2bFINALbrobdexe) -o$@ atlas.cpp
|

# Compiler configuration file
BccW16.cfg : 
   Copy &&|
-w
-R
-v
-vi
-H
-H=rob.csm
-C
-ml
-WS
-K
-ff-
-v-
-R-
-H-
-4
-Vb
-Ot
-Ff
-C
-g200
-j30
| $@


