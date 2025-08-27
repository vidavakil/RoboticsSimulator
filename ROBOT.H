
© 2025 Vida Vakilotojar

/**************************************************************************/
//
//                               Attention
//                                                                        
//  This software was designed and written by Vida Vakilotojar, as part of 
//  her M.Sc. thesis in the field of AI & Robotics, in Tehran Polytechnic 
//  University. It is a simulation  software for single or multiple robot 
//  environments, originally  supporting an  algorithm for hidden surface
//  removal of the scene, a command language for the operation of robots,
//  an   off-line   collision-free   and  deadlock- free  motion-planning
//  algorithm for two or three robots, and an on-line collision-detection
//  algorithm.
//
//  Any  portion  of  this  software  can  be  used  for  academical  and
//  non-commercial purposes provided that the name of the original author 
//  -- Vida Vakilotojar --  is  mentioned,  both in source and exe files.
//  If  the  features  of  this software are enhanced, or it is ported to
//  other platforms, a version of it should be available to the  original
//  author on demand.
//
/**************************************************************************/

#ifndef  ROBOTH
#define  ROBOTH
#include <time.h>
#include <stdio.h>
#include <string.h>
#include "winrob.h"

#define  XMAX        20  // max number of path segments
#define  YMAX        20
#define  ZMAX        20

#define  DEG_TO_RAD  0.017453292
#define  D2R         0.017453292
#define  PRISMATIC   0
#define  REVOLUTE    1
#define  MaxRobot    3

#define  dof         5
#define  MaxLink     7
#define  MaxSurface  10   
#define  MaxVertex   8
#define  MaxPoints   37
#define  MaxPart     3
#define  MaxVelocity 9
#define  LinkNo      7   /* here dof+2 */
#define  MAXCHAR     150
#define  MAX_COMMAND 27  // with kinematics
#define  MaxParNo    15

#define  MaxPlane    14
#define  MaxEdges    30

#define  sign(X)     ((X>=0)?1:-1)
#define  max(X1,X2)  ((X1>X2)?X1:X2)
#define  min(X1,X2)  ((X1<X2)?X1:X2)

#define  START       0
#define  END         1

#define  _ATLAS       0
#define  _RHINO       1


#define ID_REC	1034
#define ID_SEG	1038
#define Teach	  2000
#define ID_ROB1	1035
#define ID_ROB2	1036
#define ID_ROB3	1037
#define FWD2	  1040
#define BCK2	  1041
#define FAST2	  1042
#define SLOW2	  1043
#define FWD3	  1050
#define BCK3	  1051
#define FAST3	  1052
#define SLOW3	  1053
#define FWD4	  1060
#define BCK4	  1061
#define FAST4	  1062
#define SLOW4	  1063
#define FWD5	  1070
#define BCK5	  1071
#define FAST5	  1072
#define SLOW5	  1073
#define FWD6	  1080
#define BCK6	  1081
#define FAST6	  1082
#define SLOW6	  1083
#define TCH_END	1090
#define FWD1	  1030
#define BCK1	  1031
#define FAST1	  1032
#define SLOW1	  1033

typedef int (*fptr)(void);


typedef struct
  {
    char name[10];
    fptr function;
    int  Single;
    int  parno;
    int  CommandType; // 2 : on-line commands which are not allowed in Files
  } CommandType;      // 1 : timed-commands which read the time
                      // 0 : untimed and general commands
                      // -1: timed commands which advance the timer

typedef struct
  {
    int          NoSurface;
    int          Surfaces[MaxSurface][MaxVertex];
  } ConvexType;

enum ProjectionType {XY, XZ, YZ};
enum AxisType {X=1, Y, Z};

typedef struct
  {
    BYTE R;
    BYTE G;
    BYTE B;
  } RGBColor;

typedef struct
  {
    RGBColor        Color;
    float           Radius;
    ProjectionType  PType;
    AxisType        AType;
    int             NoPart;
    ConvexType      Part[MaxPart];
  } LinkType;

typedef struct
  {
		int    RobNo;
		int    SNo;
		int    link_num;
		int   *connection;
		float  Min, Max;
		float  MaxX, MinX, MaxY, MinY;
		int    PXMax, PXMin, PYMax, PYMin;
		int    PartNumber;
		int    N;
  } visible_surface;

typedef float frame[5][5];
typedef float point[5];

struct projected_coordinates
  {
    float x;
    float y;
  };

struct VType
  {
    float X1, Y1, X2, Y2;
    float Radius;
  };

typedef struct
  {
    double  X, Y, Z;
  } Point;

typedef struct
  {
    unsigned long X, Y, Z;
  } Dimensions;

typedef struct
  {
    int  EdgeNo;
    int  FaceNo;
    int  VertNo;
    int  DcelTable[MaxEdges][9];
  } DCELType;

class cRobot
  {
    public :
      int     RobNo;
      int     Vox;
      int     Start_End;
      int     Filling;
      int     LineNo;
      int     CurrentFile;
			int     correction, UPdate;
      int     AnyMoreChange, UpdateDisplay;
      char    FirstMove[MAXCHAR]; 
      char    InFileName[13];
      FILE    *InFile, *OFile;
      time_t  Start, End, ElapsedTime;

      float     velocity_table [MaxLink][9];
      DCELType  *DcelP;
      LinkType  *LinkArray;
      int       *joint_status;
      int       *max_point;
      int       *max_surface;
      float     *ref;
      float     *length;
      float     *max_joint;
      float     *min_joint;

      point   relative_point[MaxLink][MaxPoints];
      point   absolute_point[MaxLink][MaxPoints];
      frame   map[MaxLink];
      frame   inv_map[MaxLink];
      frame   t;
      float   theta[MaxLink]; 	     /* current joint values */
      float   LastRecord[MaxLink]; 	 
      float   change[MaxLink];
      int     step[MaxLink];

      cRobot (void) {correction = Filling = Vox = CurrentFile = Start_End = 0;
		                 strcpy (InFileName,"robot .dat");}

      virtual void initialize_points (void) {return;}
      virtual void initialize_maps (void) {return;}
      virtual void update_maps (float *joint) {return;}
      virtual void compute_end_device (float tv) {return;}
      virtual int  kinematics (frame t,float *j) {return 0;}
      virtual int  inverse_kinematics (frame t,float *j) {return 0;}

      void    draw_link (int link_num);
      void    display_frame (frame t,float *joint);
      void    compute_points (frame t,int link_num);
      void    update_inv_map (frame t,int link_num);
      void    update_points (float *joint,float tv,frame t);
      void    compute_point (frame t,char link_num, char PType, VType *PC);
      void    specify (visible_surface *a, struct projected_coordinates P1[]);

      void    Update (void);
      void    GetNextCommand (void);
      void    Init (float *parameter);
      void    startfill (void), endfill (void);
      void    OpenFile (void), CloseFile (void);
      void    movej (float *f), move(float *f);
      void    refer (void), tp (WORD key), start (void);
      void    movea (float *f), movew (float *f), moveg (float *f);
      void    speed (float *f), drkin (float *f), inkin (float *f);

      int     surface_in_sight (int link_num,int *connection);
      int     Cylinder (LinkType *Link1, point P11, point P12,
		                    LinkType *Link2, point P21, point P22, int PType);
  };
#endif
