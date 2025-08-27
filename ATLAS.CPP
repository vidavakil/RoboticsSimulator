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

#include <windows.h>
#include <bios.h>
#include <ctype.h>
#include <math.h>
#include "robot.h"
#include "atlas.h"

extern int RecordFlag;
float Atlas_velocity_table[MaxLink][9] =
    {{0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0,1.60,.350,.300,.250,.200,.150,.100,.020},
		{0,1.60,.119,.102,.085,.068,.051,.034,.020},
		{0,20,7,6,5,4,3,2,.2},
		{0,1.60,.350,.300,.250,.200,.150,.100,.020},
		{0,3.142,.350,.300,.250,.200,.150,.100,.020},
		{0,4,3.5,3,2.5,2,1.5,1,.2},};

LinkType Atlas_LinkArray[LinkNo] = {{{255,0,0}, 14.3, XY, Z, 1, {{8,
	    {{1,3,4,2,1,-1},       // Robot Base
	     {1,2,6,5,1,-1},
	     {2,4,8,12,10,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,9,11,7,3,1,-1},
	     {5,6,10,9,5,-1},
	     {8,7,11,12,8,-1},
	     {9,10,12,11,9,-1},},},},},
				                            {{255,255,0}, 4.0, XZ, Y, 1, {{5,
	    {{1,2,6,5,1,-1},       // Robot Arm rotator
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,7,3,1,-1},
	     {6,8,7,5,6,-1},},},},},
				                            {{0,255,0}, 22.3, XY, Z, 2, {{7,
	    {{1,2,3,13,15,1,-1},   // Robot Arm Elevator
	     {7,16,14,9,8,7,-1},
	     {1,7,8,2,1,-1},
	     {2,8,9,3,2,-1},
	     {9,14,13,3,9,-1},
	     {1,15,16,7,1,-1},
	     {13,14,16,15,13,-1},},},
					                                               {7,
	    {{19,17,4,5,6,19,-1},
	     {20,12,11,10,18,20,-1},
	     {11,12,6,5,11,-1},
	     {10,11,5,4,10,-1},
	     {17,18,10,4,17,-1},
	     {6,12,20,19,6,-1},
	     {17,19,20,18,17,-1},},},},},
				                            {{0,255,255}, 8.0, XY, Z, 1, {{2,
	    {{1,2,3,4,1,-1},       // Robot Arm Extender
	     {4,3,2,1,4,-1},},},},},
				                            {{0,0,255}, 2.85, XY, Z, 1, {{6,
	    {{1,3,4,2,1,-1},       // Robot Wrist Elevator
	     {1,2,6,5,1,-1},
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,7,3,1,-1},
	     {5,6,8,7,5,-1},},},},},
				                            {{255,0,255}, 2.14, XY, Z, 1, {{6,
	    {{1,3,4,2,1,-1},       // Robot Wrist Rotator
	     {1,2,6,5,1,-1},
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {3,1,5,7,3,-1},
	     {5,6,8,7,5,-1},},},},},
				                            {{128,128,128}, 3.34, YZ, X, 2, {{6,
	    {{1,4,3,2,1,-1},       // Robot Gripper
	     {1,2,6,5,1,-1},
	     {2,3,7,6,2,-1},
	     {3,4,8,7,3,-1},
	     {4,1,5,8,4,-1},
	     {5,6,7,8,5,-1},},},
					                                              {6,
	    {{9,12,11,10,9,-1},
	     {9,10,14,13,9,-1},
	     {10,11,15,14,10,-1},
	     {11,12,16,15,11,-1},
	     {12,9,13,16,12,-1},
	     {13,14,15,16,13,-1},},},},}};

int Atlas_joint_status[MaxLink] = {-1, REVOLUTE, REVOLUTE, PRISMATIC,
			                                 REVOLUTE, REVOLUTE, PRISMATIC};

/* # of points on each link, starting from base (link 0)
   to the last link (the end device) */
int Atlas_max_point[MaxLink] = {14,12,22,36,10,10,18};

/* # of surfaces on each link, starting from base
   (link 0) to the last link (the end device) */
int Atlas_max_surface[MaxLink]={8,5,14,2,6,6,12};

float Atlas_length[41]={-1.0,
      13.0,9.5,25.5,25.5,14.0,2.0,1.0,0.75,
      5.0,6.5,8.0,15.0,
      15.0,10.5,12.5,29.5,13.0,24.0,10.0,5.6,10.6,16.8,11.0,4.5,2.5,
      0.0,32.0,0.6,2.5,16.0,8.5,7.0,/*^was 19.0*/            /* ^was 3.5*/
      1.5,5.5,6.0,
      12.0,2.0,11.0,6.5,4.0};

DCELType  Atlas_Dcel[MaxLink] =
                    {
                     {18, 8, 12,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
/* Robot Base */       {1, 3, 1, 6, 4, 6, 0, 0, 0},
                       {3, 4, 2, 6, 1, 7, 0, 0, 0},
                       {4, 2, 3, 6, 2, 8, 0, 0, 0},
                       {2, 1, 4, 6, 3, 5, 0, 0, 0},
                       {1, 5, 4, 1, 1, 10, 0, 0, 0},
                       {3, 7, 1, 2, 2, 17, 0, 0, 0},
                       {4, 8, 2, 3, 3, 12, 0, 0, 0},
                       {2, 6, 3, 4, 4, 16, 0, 0, 0},
                       {10, 12, 3, 5, 13, 18, 0, 0, 0},
                       {6, 5, 8, 4, 8, 15, 0, 0, 0},
                       {9, 11, 5, 1, 15, 14, 0, 0, 0},
                       {7, 8, 7, 2, 6, 18, 0, 0, 0},
                       {9, 10, 8, 5, 11, 16, 0, 0, 0},
                       {11, 12, 5, 7, 17, 9, 0, 0, 0},
                       {5, 9, 8, 1, 5, 13, 0, 0, 0},
                       {6, 10, 3, 8, 10, 9, 0, 0, 0},
                       {7, 11, 1, 7, 12, 11, 0, 0, 0},
                       {8, 12, 7, 3, 7, 14, 0, 0, 0}}},

                     {12, 6, 8,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {1, 3, 1, 6, 4, 6, 0, 0, 0},
                       {3, 4, 2, 6, 1, 7, 0, 0, 0},
/* Arm Rotator */      {4, 2, 3, 6, 2, 8, 0, 0, 0},
                       {2, 1, 4, 6, 3, 5, 0, 0, 0},
                       {1, 5, 4, 1, 1, 10, 0, 0, 0},
                       {3, 7, 1, 2, 2, 11, 0, 0, 0},
                       {4, 8, 2, 3, 3, 12, 0, 0, 0},
                       {2, 6, 3, 4, 4, 9, 0, 0, 0},
                       {8, 6, 5, 3, 7, 10, 0, 0, 0},
                       {6, 5, 5, 4, 8, 11, 0, 0, 0},
                       {5, 7, 5, 1, 5, 12, 0, 0, 0},
                       {7, 8, 5, 2, 6, 9, 0, 0, 0}}},

                     {18, 8, 12,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {2, 1, 1, 7, 18, 2, 0, 0, 0},
/* Arm Elevator */     {1, 6, 1, 2, 13, 3, 0, 0, 0},
                       {6, 5, 1, 3, 14, 4, 0, 0, 0},
                       {5, 4, 1, 4, 15, 5, 0, 0, 0},
                       {4, 3, 1, 5, 16, 6, 0, 0, 0},
                       {3, 2, 1, 6, 17, 1, 0, 0, 0},
                       {8, 7, 7, 8, 12, 13, 0, 0, 0},
                       {7, 12, 2, 8, 7, 14, 0, 0, 0},
                       {12, 11, 3, 8, 8, 15, 0, 0, 0},
                       {11, 10, 4, 8, 9, 16, 0, 0, 0},
                       {10, 9, 5, 8, 10, 17, 0, 0, 0},
                       {9, 8, 6, 8, 11, 18, 0, 0, 0},
                       {1, 7, 2, 7, 1, 8, 0, 0, 0},
                       {6, 12, 3, 2, 2, 9, 0, 0, 0},
                       {5, 11, 4, 3, 3, 10, 0, 0, 0},
                       {4, 10, 5, 4, 4, 11, 0, 0, 0},
                       {3, 9, 6, 5, 5, 12, 0, 0, 0},
                       {2, 8, 7, 6, 6, 7, 0, 0, 0}}},

                     {4, 2, 4,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {3, 4, 1, 2, 4, 2, 0, 0, 0},
/* Arm Extender */     {4, 1, 1, 2, 1, 3, 0, 0, 0},
                       {1, 2, 1, 2, 2, 4, 0, 0, 0},
                       {2, 3, 1, 2, 3, 1, 0, 0, 0}}},

                     {12, 6, 8,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {1, 3, 1, 6, 4, 6, 0, 0, 0},
                       {3, 4, 2, 6, 1, 7, 0, 0, 0},
/* wrist Elevator */   {4, 2, 3, 6, 2, 8, 0, 0, 0},
                       {2, 1, 4, 6, 3, 5, 0, 0, 0},
                       {1, 5, 4, 1, 1, 10, 0, 0, 0},
                       {3, 7, 1, 2, 2, 11, 0, 0, 0},
                       {4, 8, 2, 3, 3, 12, 0, 0, 0},
                       {2, 6, 3, 4, 4, 9, 0, 0, 0},
                       {8, 6, 5, 3, 7, 10, 0, 0, 0},
                       {6, 5, 5, 4, 8, 11, 0, 0, 0},
                       {5, 7, 5, 1, 5, 12, 0, 0, 0},
                       {7, 8, 5, 2, 6, 9, 0, 0, 0}}},

                     {12, 6, 8,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {1, 3, 1, 6, 4, 6, 0, 0, 0},
                       {3, 4, 2, 6, 1, 7, 0, 0, 0},
/* Wrist Rotator */    {4, 2, 3, 6, 2, 8, 0, 0, 0},
                       {2, 1, 4, 6, 3, 5, 0, 0, 0},
                       {1, 5, 4, 1, 1, 10, 0, 0, 0},
                       {3, 7, 1, 2, 2, 11, 0, 0, 0},
                       {4, 8, 2, 3, 3, 12, 0, 0, 0},
                       {2, 6, 3, 4, 4, 9, 0, 0, 0},
                       {8, 6, 5, 3, 7, 10, 0, 0, 0},
                       {6, 5, 5, 4, 8, 11, 0, 0, 0},
                       {5, 7, 5, 1, 5, 12, 0, 0, 0},
                       {7, 8, 5, 2, 6, 9, 0, 0, 0}}},

/* Tool */           {24, 12, 16,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {1, 2, 2, 1, 4, 5, 0, 0, 0},
                       {2, 3, 5, 1, 1, 6, 0, 0, 0},
                       {3, 4, 4, 1, 2, 7, 0, 0, 0},
                       {4, 1, 3, 1, 3, 8, 0, 0, 0},
                       {2, 6, 2, 5, 2, 9, 0, 0, 0},
                       {3, 7, 5, 4, 3, 10, 0, 0, 0},
                       {4, 8, 4, 3, 4, 11, 0, 0, 0},
                       {1, 5, 3, 2, 1, 12, 0, 0, 0},
                       {5, 6, 6, 2, 8, 10, 0, 0, 0},
                       {6, 7, 6, 5, 5, 11, 0, 0, 0},
                       {7, 8, 6, 4, 6, 12, 0, 0, 0},
                       {8, 5, 6, 3, 7, 9, 0, 0, 0},

                       {9, 10, 8, 7, 16, 17, 0, 0, 0},
                       {10, 11, 11, 7, 13, 18, 0, 0, 0},
                       {11, 12, 10, 7, 14, 19, 0, 0, 0},
                       {12, 9, 9, 7, 15, 20, 0, 0, 0},
                       {10, 14, 8, 11, 14, 21, 0, 0, 0},
                       {11, 15, 11, 10, 15, 22, 0, 0, 0},
                       {12, 16, 10, 9, 16, 23, 0, 0, 0},
                       {9, 13, 9, 8, 13, 24, 0, 0, 0},
                       {13, 14, 12, 8, 20, 22, 0, 0, 0},
                       {14, 15, 12, 11, 17, 23, 0, 0, 0},
                       {15, 16, 12, 10, 18, 24, 0, 0, 0},
                       {16, 13, 12, 9, 19, 21, 0, 0, 0}}}};

/***************************************************************************/
/* Workspace of the robot (limits are in radians for revolute joints)      */
/***************************************************************************/

float Atlas_max_joint[10]={0.0,180.0*D2R, 25.0*D2R,  0.0,
                           65.0*D2R,270.0*D2R,6.5};
float Atlas_min_joint[10]={0.0,-180.0*D2R,-60.0*D2R,-32.0,
                           -65.0*D2R,-90.0*D2R,0.0};
float Atlas_ref[10]={0.0,0.0,25.0,0.0,35.0,90.0,6.5}; // 35 was 65

extern float atan3(float y,float x),atan4(float y,float x);

extern int   Hide;
extern int   ParNo;
extern int   Segment;
extern int   simulation_mode;
extern int   teach_pendant_mode;
extern FILE  *Out;
extern float parameter[]; 
extern char  command_part[], param_part[], mess[]; 
extern CommandType   command[]; 
extern int   OnLineTest;
extern HWND	 HwndMap, HwndView, HwndPannel;

extern int   Record (void);
extern int   get_param (void);
extern void  preprocess(void);
extern void  send (float *qqd);
extern void  message (char *Text);
extern void  initializehard(void);
extern void  display_robot (void);
extern void  process_command (void);
extern void  VFinalize (int RobotNo);
extern void  Vinit (int RobotNo, int Status);
extern void  VSave (int RobotNo, char *FileName);
extern void  frame_multiply(frame a,frame b,frame c);
extern float fmin (float a, float b);

extern void Alarm (int Flag);
extern int  MultiCollision_Detect (void);
extern void VUpdate (int RobotNo);

extern void BackGround (void);
extern void EnableAllButtons (BOOL Enable);
/***************************************************************************/
// This is the initialization routine for robot Atlas
/***************************************************************************/
Atlas::Atlas (void)
{
  register int i, j;

  length = Atlas_length;
  ref = Atlas_ref;
  max_joint = Atlas_max_joint;
  min_joint = Atlas_min_joint;
  joint_status = Atlas_joint_status;
  max_point = Atlas_max_point;
  max_surface = Atlas_max_surface;
  LinkArray = Atlas_LinkArray;
  DcelP = Atlas_Dcel;
  for (i = 0; i < 7; i++)
    change[i] = 0;
  for (i = 0; i < 7; i++)  // MaxL = 7
    for (j = 0; j <= 8; j++)
      velocity_table[i][j] = Atlas_velocity_table[i][j];
} // Atlas::Atlas

/***************************************************************************/
/*                 Atlas-Specific Functions                                */
/*                                                                         */
/***************************************************************************/
/***************************************************************************/
// Very Important : The last two points of any Link are the endpoints
// of its axis
/***************************************************************************/
/***************************************************************************/
/* Initialize the last row of coordinate transformation matrices.          */
/* It also initializes the tool-tip and robot-base coordinate frames.      */
/***************************************************************************/
void Atlas::initialize_maps (void)
{
  frame    Temp;
  float    T, CT, ST;
  register int i;

  for (i=0;i<dof+2;i++)
    {   
      map[i][4][1]=map[i][4][2]=map[i][4][3]=0.0;
      map[i][4][4]=1.0;
    } // for
  map[0][1][1] = map[0][2][2] = map[0][3][3] = 1;
  map[0][1][2] = map[0][1][3] = map[0][2][1] =
  map[0][2][3] = map[0][3][1] = map[0][3][2] = 0;
  map[0][1][4] = map[0][3][4] = 0;
  map[0][2][4] = -55;

  T = 2*M_PI*RobNo/MaxRobot;
  CT = cos(T);            ST = sin (T);
  Temp[1][1] = CT;
  Temp[1][2] = ST;
  Temp[2][1] = -ST;
  Temp[2][2] = CT;
  Temp[1][3] = Temp[1][4] = Temp[2][3] = Temp[2][4] = Temp[3][1] =
  Temp[3][2] = Temp[3][4] =
  Temp[4][1] = Temp[4][2] = Temp[4][3] = 0.0;
  Temp[3][2] = 0/*-50*/;
  Temp[3][3] = Temp[4][4] = 1.0;
  frame_multiply (Temp, map[0], map[0]);

  /* define tool_tip frame here */

  map[6][1][1]= 0.0; map[6][1][2]= 0.0;
  map[6][1][3]=-1.0; map[6][1][4]=-length[40]-length[39]/2.0;
  map[6][2][1]= 0.0; map[6][2][2]=-1.0;
  map[6][2][3]= 0.0; map[6][2][4]= 0.0;
  map[6][3][1]=-1.0; map[6][3][2]= 0.0;
  map[6][3][3]= 0.0; map[6][3][4]= 0.0;
} // Atlas::initialize_maps

/***************************************************************************/
/* Initialize all relative points. This function is called only once,      */
/* and therefore there is no need to optimize it for speed.                */
/* The last two points of each link specify the endpoints of link axis     */
/* which is used for the on-line collision-detection algorithm.            */
/***************************************************************************/
void Atlas::initialize_points (void)
{
  /* basic angular values used in describing the robot */
  float alpha[7]={-1.0,
  108.0,155.0,115.0,122.0,90.0,130.0};

  register int i,j;

  /***********************/
  /* Base of the robot : */
  /***********************/
  absolute_point[0][ 1][1]=relative_point[0][ 1][1]= length[3]/2;
  absolute_point[0][ 1][2]=relative_point[0][ 1][2]=-length[4]/2;
  absolute_point[0][ 1][3]=relative_point[0][ 1][3]= 0.0;

  absolute_point[0][ 2][1]=relative_point[0][ 2][1]= length[3]/2;
  absolute_point[0][ 2][2]=relative_point[0][ 2][2]= length[4]/2;
  absolute_point[0][ 2][3]=relative_point[0][ 2][3]= 0.0;

  absolute_point[0][ 3][1]=relative_point[0][ 3][1]=-length[3]/2;
  absolute_point[0][ 3][2]=relative_point[0][ 3][2]=-length[4]/2;
  absolute_point[0][ 3][3]=relative_point[0][ 3][3]= 0.0;

  absolute_point[0][ 4][1]=relative_point[0][ 4][1]=-length[3]/2;
  absolute_point[0][ 4][2]=relative_point[0][ 4][2]= length[4]/2;
  absolute_point[0][ 4][3]=relative_point[0][ 4][3]= 0.0;

  absolute_point[0][ 5][1]=relative_point[0][ 5][1]= length[3]/2;
  absolute_point[0][ 5][2]=relative_point[0][ 5][2]=-length[4]/2;
  absolute_point[0][ 5][3]=relative_point[0][ 5][3]= length[2];

  absolute_point[0][ 6][1]=relative_point[0][ 6][1]= length[3]/2;
  absolute_point[0][ 6][2]=relative_point[0][ 6][2]= length[4]/2;
  absolute_point[0][ 6][3]=relative_point[0][ 6][3]= length[2];

  absolute_point[0][ 7][1]=relative_point[0][ 7][1]=-length[3]/2;
  absolute_point[0][ 7][2]=relative_point[0][ 7][2]=-length[4]/2;
  absolute_point[0][ 7][3]=relative_point[0][ 7][3]= length[2];

  absolute_point[0][ 8][1]=relative_point[0][ 8][1]=-length[3]/2;
  absolute_point[0][ 8][2]=relative_point[0][ 8][2]= length[4]/2;
  absolute_point[0][ 8][3]=relative_point[0][ 8][3]= length[2];

  absolute_point[0][ 9][1]=relative_point[0][ 9][1]= length[5]/2;
  absolute_point[0][ 9][2]=relative_point[0][ 9][2]=-length[4]/2;
  absolute_point[0][ 9][3]=relative_point[0][ 9][3]= length[1];

  absolute_point[0][10][1]=relative_point[0][10][1]= length[5]/2;
  absolute_point[0][10][2]=relative_point[0][10][2]= length[4]/2;
  absolute_point[0][10][3]=relative_point[0][10][3]= length[1];

  absolute_point[0][11][1]=relative_point[0][11][1]=-length[5]/2;
  absolute_point[0][11][2]=relative_point[0][11][2]=-length[4]/2;
  absolute_point[0][11][3]=relative_point[0][11][3]= length[1];

  absolute_point[0][12][1]=relative_point[0][12][1]=-length[5]/2;
  absolute_point[0][12][2]=relative_point[0][12][2]= length[4]/2;
  absolute_point[0][12][3]=relative_point[0][12][3]= length[1];

  // two newly added points for link axis

  absolute_point[0][13][1]=relative_point[0][13][1]= length[3]/2;
  absolute_point[0][13][2]=relative_point[0][13][2]= 0;
  absolute_point[0][13][3]=relative_point[0][13][3]= 0;

  absolute_point[0][14][1]=relative_point[0][14][1]= -length[3]/2;
  absolute_point[0][14][2]=relative_point[0][14][2]= 0;
  absolute_point[0][14][3]=relative_point[0][14][3]= 0;

  /***********************/
  /* Arm rotation part : */
  /***********************/
  relative_point[1][ 1][1]= length[9]/2;
  relative_point[1][ 1][2]=-length[10]/2;
  relative_point[1][ 1][3]= 0.0;

  relative_point[1][ 2][1]=-length[9]/2;
  relative_point[1][ 2][2]=-length[10]/2;
  relative_point[1][ 2][3]= 0.0;

  relative_point[1][ 3][1]= length[9]/2;
  relative_point[1][ 3][2]= length[10]/2;
  relative_point[1][ 3][3]= 0.0;

  relative_point[1][ 4][1]=-length[9]/2;
  relative_point[1][ 4][2]= length[10]/2;
  relative_point[1][ 4][3]= 0.0;

  relative_point[1][ 5][1]= length[9]/2;
  relative_point[1][ 5][2]=-length[10]/2;
  relative_point[1][ 5][3]=-length[11];

  relative_point[1][ 6][1]=-length[9]/2;
  relative_point[1][ 6][2]=-length[10]/2;
  relative_point[1][ 6][3]=-length[11];

  relative_point[1][ 7][1]= length[9]/2;
  relative_point[1][ 7][2]= length[10]/2;
  relative_point[1][ 7][3]=-length[11];

  relative_point[1][ 8][1]=-length[9]/2;
  relative_point[1][ 8][2]= length[10]/2;
  relative_point[1][ 8][3]=-length[11];

  relative_point[1][ 9][1]= 0.0;
  relative_point[1][ 9][2]=-length[10]/2;
  relative_point[1][ 9][3]=-length[12];

  relative_point[1][10][1]= 0.0;
  relative_point[1][10][2]= length[10]/2;
  relative_point[1][10][3]=-length[12];

  // two newly added points for link axis
  relative_point[1][11][1]= 0;
  relative_point[1][11][2]= 0;
  relative_point[1][11][3]= 0;

  relative_point[1][12][1]= 0;
  relative_point[1][12][2]= 0;
  relative_point[1][12][3]= -length[12];

  /************************/
  /* Arm elevation part : */
  /************************/
  relative_point[2][ 1][1]=-length[21]+length[13]+length[14]*
			                     sin(DEG_TO_RAD*alpha[3]);
  relative_point[2][ 1][2]=-length[19]/2;
  relative_point[2][ 1][3]=-length[20]+length[24]-length[14]*
			                     cos(DEG_TO_RAD*alpha[3]);

  relative_point[2][ 2][1]=-length[21]+length[14]*sin(DEG_TO_RAD*alpha[3]);
  relative_point[2][ 2][2]=-length[19]/2;
  relative_point[2][ 2][3]=-length[20]+length[24]-length[14]*
			                     cos(DEG_TO_RAD*alpha[3]);

  relative_point[2][ 3][1]=-length[21];
  relative_point[2][ 3][2]=-length[19]/2;
  relative_point[2][ 3][3]=-length[20]+length[24];

  relative_point[2][13][1]=-length[21];
  relative_point[2][13][2]=-length[19]/2;
  relative_point[2][13][3]=-length[20]+0.3;

  relative_point[2][15][1]=length[22];
  relative_point[2][15][2]=-length[19]/2;
  relative_point[2][15][3]=-length[20]+0.3;

  relative_point[2][17][1]=-length[21];
  relative_point[2][17][2]=-length[19]/2;
  relative_point[2][17][3]=-length[20]-0.3;

  relative_point[2][19][1]=length[22];
  relative_point[2][19][2]=-length[19]/2;
  relative_point[2][19][3]=-length[20]-0.3;
  
  relative_point[2][ 4][1]=-length[21];
  relative_point[2][ 4][2]=-length[19]/2;
  relative_point[2][ 4][3]=-length[20]-length[15]+length[24];

  relative_point[2][ 5][1]=-length[21]+length[16]*sin(DEG_TO_RAD*alpha[4]);
  relative_point[2][ 5][2]=-length[19]/2;
  relative_point[2][ 5][3]=-length[20]-length[15]+length[24]+length[16]*
			                     cos(DEG_TO_RAD*alpha[4]);

  relative_point[2][ 6][1]=-length[21]+length[16]*sin(DEG_TO_RAD*alpha[4])-
			                      length[17]*sin(DEG_TO_RAD*(alpha[4]+alpha[5]));
  relative_point[2][ 6][2]=-length[19]/2;
  relative_point[2][ 6][3]=-length[20]-length[15]+length[24]+length[16]*
			                     cos(DEG_TO_RAD*alpha[4])-length[17]*
			                     cos(DEG_TO_RAD*(alpha[4]+alpha[5]));

  relative_point[2][ 7][1]=-length[21]+length[14]*sin(DEG_TO_RAD*alpha[3])+
			                      length[13];
  relative_point[2][ 7][2]= length[19]/2;
  relative_point[2][ 7][3]=-length[20]+length[24]-length[14]*
			                     cos(DEG_TO_RAD*alpha[3]);

  relative_point[2][ 8][1]=-length[21]+length[14]*sin(DEG_TO_RAD*alpha[3]);
  relative_point[2][ 8][2]= length[19]/2;
  relative_point[2][ 8][3]=-length[20]+length[24]-length[14]*
			                     cos(DEG_TO_RAD*alpha[3]);

  relative_point[2][ 9][1]=-length[21];
  relative_point[2][ 9][2]= length[19]/2;
  relative_point[2][ 9][3]=-length[20]+length[24];

  relative_point[2][14][1]=-length[21];
  relative_point[2][14][2]= length[19]/2;
  relative_point[2][14][3]=-length[20]+0.3;

  relative_point[2][16][1]=length[22];
  relative_point[2][16][2]= length[19]/2;
  relative_point[2][16][3]=-length[20]+0.3;

  relative_point[2][18][1]=-length[21];
  relative_point[2][18][2]= length[19]/2;
  relative_point[2][18][3]=-length[20]-0.3;

  relative_point[2][20][1]=length[22];
  relative_point[2][20][2]= length[19]/2;
  relative_point[2][20][3]=-length[20]-0.3;

  relative_point[2][10][1]=-length[21];
  relative_point[2][10][2]= length[19]/2;
  relative_point[2][10][3]=-length[20]-length[15]+length[24];

  relative_point[2][11][1]=-length[21]+length[16]*sin(DEG_TO_RAD*alpha[4]);
  relative_point[2][11][2]= length[19]/2;
  relative_point[2][11][3]=-length[20]-length[15]+length[24]+length[16]*
			                     cos(DEG_TO_RAD*alpha[4]);

  relative_point[2][12][1]=-length[21]+length[16]*sin(DEG_TO_RAD*alpha[4])-
			                      length[17]*sin(DEG_TO_RAD*(alpha[4]+alpha[5]));
  relative_point[2][12][2]= length[19]/2;
  relative_point[2][12][3]=-length[20]-length[15]+length[24]+length[16]*
			                     cos(DEG_TO_RAD*alpha[4])-length[17]*
			                     cos(DEG_TO_RAD*(alpha[4]+alpha[5]));

  // two newly added points for link axis
  relative_point[2][21][1]= 4.55;
  relative_point[2][21][2]= -9.5;
  relative_point[2][21][3]= -9.24;

  relative_point[2][22][1]= 4.55;
  relative_point[2][22][2]= 9.5;
  relative_point[2][22][3]= -9.24;

  /************************/
  /* Arm extension part : */
  /************************/
  relative_point[3][ 9][1]= length[28]/2;
  relative_point[3][ 9][2]=-length[30]/2;
  relative_point[3][ 9][3]= length[29]/2;

  relative_point[3][22][1]=-length[28]/2;
  relative_point[3][22][2]=-length[30]/2;
  relative_point[3][22][3]= length[29]/2;

  relative_point[3][26][1]= length[28]/2;
  relative_point[3][26][2]= length[30]/2;
  relative_point[3][26][3]= length[29]/2;

  relative_point[3][13][1]=-length[28]/2;
  relative_point[3][13][2]= length[30]/2;
  relative_point[3][13][3]= length[29]/2;

  relative_point[3][ 5][1]= length[28]/2;
  relative_point[3][ 5][2]=-length[30]/2;
  relative_point[3][ 5][3]=-length[29]/2;

  relative_point[3][ 6][1]=-length[28]/2;
  relative_point[3][ 6][2]=-length[30]/2;
  relative_point[3][ 6][3]=-length[29]/2;

  relative_point[3][ 7][1]= length[28]/2;
  relative_point[3][ 7][2]= length[30]/2;
  relative_point[3][ 7][3]=-length[29]/2;

  relative_point[3][ 8][1]=-length[28]/2;
  relative_point[3][ 8][2]= length[30]/2;
  relative_point[3][ 8][3]=-length[29]/2;

  relative_point[3][ 1][1]= length[28]/2-length[32]+1.0; /* new */
  relative_point[3][ 1][2]=-2*length[25];
  relative_point[3][ 1][3]= 0.0;

  relative_point[3][10][1]= length[28]/2;
  relative_point[3][10][2]=-length[25];
  relative_point[3][10][3]= 0.0;

  relative_point[3][11][1]= length[28]/2;
  relative_point[3][11][2]= 0.0;
  relative_point[3][11][3]= 0.0;

  relative_point[3][12][1]= length[28]/2;
  relative_point[3][12][2]= length[25];
  relative_point[3][12][3]= 0.0;

  relative_point[3][ 4][1]= length[28]/2-length[32]+1.0; /* new */
  relative_point[3][ 4][2]= 2*length[25];
  relative_point[3][ 4][3]= 0.0;

  relative_point[3][14][1]= 3*length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][14][2]=-length[30]/2;
  relative_point[3][14][3]= length[29]/2;

  relative_point[3][15][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][15][2]=-length[30]/2;
  relative_point[3][15][3]= length[29]/2;

  relative_point[3][16][1]= 3*length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][16][2]= length[30]/2;
  relative_point[3][16][3]= length[29]/2;

  relative_point[3][17][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][17][2]= length[30]/2;
  relative_point[3][17][3]= length[29]/2;

  relative_point[3][18][1]= 3*length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][18][2]=-length[30]/2;
  relative_point[3][18][3]=-length[29]/2;

  relative_point[3][19][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][19][2]=-length[30]/2;
  relative_point[3][19][3]=-length[29]/2;

  relative_point[3][20][1]= 3*length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][20][2]= length[30]/2;
  relative_point[3][20][3]=-length[29]/2;

  relative_point[3][21][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][21][2]= length[30]/2;
  relative_point[3][21][3]=-length[29]/2;

  relative_point[3][ 2][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][ 2][2]=-2*length[25];
  relative_point[3][ 2][3]= 0.0;

  relative_point[3][23][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][23][2]=-length[25];
  relative_point[3][23][3]= 0.0;

  relative_point[3][24][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][24][2]= 0.0;
  relative_point[3][24][3]= 0.0;

  relative_point[3][25][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][25][2]= length[25];
  relative_point[3][25][3]= 0.0;

  relative_point[3][ 3][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][ 3][2]= 2*length[25];
  relative_point[3][ 3][3]= 0.0;

  relative_point[3][27][1]=-length[28]/2;
  relative_point[3][27][2]=-length[31]/2;
  relative_point[3][27][3]= length[29]/2;

  relative_point[3][28][1]=-length[28]/2-length[32];
  relative_point[3][28][2]=-length[31]/2;
  relative_point[3][28][3]= length[29]/2;

  relative_point[3][29][1]=-length[28]/2;
  relative_point[3][29][2]= length[31]/2;
  relative_point[3][29][3]= length[29]/2;

  relative_point[3][30][1]=-length[28]/2-length[32];
  relative_point[3][30][2]= length[31]/2;
  relative_point[3][30][3]= length[29]/2;

  relative_point[3][31][1]=-length[28]/2;
  relative_point[3][31][2]=-length[31]/2;
  relative_point[3][31][3]=-length[29]/2;

  relative_point[3][32][1]=-length[28]/2-length[32];
  relative_point[3][32][2]=-length[31]/2;
  relative_point[3][32][3]=-length[29]/2;

  relative_point[3][33][1]=-length[28]/2;
  relative_point[3][33][2]= length[31]/2;
  relative_point[3][33][3]=-length[29]/2;

  relative_point[3][34][1]=-length[28]/2-length[32];
  relative_point[3][34][2]= length[31]/2;
  relative_point[3][34][3]=-length[29]/2;

  // two newly added points for link axis
  relative_point[3][35][1]= length[28]/2-length[32]+1.0; /* new */
  relative_point[3][35][2]= 0;
  relative_point[3][35][3]= 0;

  relative_point[3][36][1]= length[28]/2+length[27]+length[21]+length[22]+
			                      length[26];
  relative_point[3][36][2]= 0;
  relative_point[3][36][3]= 0;

  /**************************/
  /* Wrist elevation part : */
  /**************************/
  relative_point[4][ 1][1]= 0.0;
  relative_point[4][ 1][2]=-length[34]/2;
  relative_point[4][ 1][3]= length[33]/2;

  relative_point[4][ 2][1]=-length[35];
  relative_point[4][ 2][2]=-length[34]/2;
  relative_point[4][ 2][3]= length[33]/2;

  relative_point[4][ 3][1]= 0.0;
  relative_point[4][ 3][2]= length[34]/2;
  relative_point[4][ 3][3]= length[33]/2;

  relative_point[4][ 4][1]=-length[35];
  relative_point[4][ 4][2]= length[34]/2;
  relative_point[4][ 4][3]= length[33]/2;

  relative_point[4][ 5][1]= 0.0;
  relative_point[4][ 5][2]=-length[34]/2;
  relative_point[4][ 5][3]=-length[33]/2;

  relative_point[4][ 6][1]=-length[35];
  relative_point[4][ 6][2]=-length[34]/2;
  relative_point[4][ 6][3]=-length[33]/2;

  relative_point[4][ 7][1]= 0.0;
  relative_point[4][ 7][2]= length[34]/2;
  relative_point[4][ 7][3]=-length[33]/2;

  relative_point[4][ 8][1]=-length[35];
  relative_point[4][ 8][2]= length[34]/2;
  relative_point[4][ 8][3]=-length[33]/2;

  // two newly added points for link axis
  relative_point[4][ 9][1]= 0.0;
  relative_point[4][ 9][2]= 0.0;
  relative_point[4][ 9][3]= 0.0;

  relative_point[4][10][1]=-length[35];
  relative_point[4][10][2]= 0.0;
  relative_point[4][10][3]= 0.0;

  /*************************/
  /* Wrist rotation part : */
  /*************************/
  relative_point[5][ 1][1]= -0.1 /*0.0*/;    /* -.1 is added */
  relative_point[5][ 1][2]=-length[36]/2;
  relative_point[5][ 1][3]= length[33]/2;

  relative_point[5][ 2][1]=-(length[40]-0.1);
  relative_point[5][ 2][2]=-length[36]/2;
  relative_point[5][ 2][3]= length[33]/2;

  relative_point[5][ 4][1]=-(length[40]-0.1);
  relative_point[5][ 4][2]= length[36]/2;
  relative_point[5][ 4][3]= length[33]/2;

  relative_point[5][ 3][1]= -0.1 /* 0.0 */;
  relative_point[5][ 3][2]= length[36]/2;
  relative_point[5][ 3][3]= length[33]/2;

  relative_point[5][ 5][1]= -0.1 /* 0.0 */;
  relative_point[5][ 5][2]=-length[36]/2;
  relative_point[5][ 5][3]=-length[33]/2;

  relative_point[5][ 6][1]=-(length[40]-0.1);
  relative_point[5][ 6][2]=-length[36]/2;
  relative_point[5][ 6][3]=-length[33]/2;

  relative_point[5][ 8][1]=-(length[40]-0.1);
  relative_point[5][ 8][2]= length[36]/2;
  relative_point[5][ 8][3]=-length[33]/2;

  relative_point[5][ 7][1]= -0.1 /* 0.0 */;
  relative_point[5][ 7][2]= length[36]/2;
  relative_point[5][ 7][3]=-length[33]/2;

  // two newly added points for link axis

  relative_point[5][ 9][1]= 0;
  relative_point[5][ 9][2]= -length[36]/2;
  relative_point[5][ 9][3]= 0;

  relative_point[5][10][1]= 0;
  relative_point[5][10][2]= length[36]/2;
  relative_point[5][10][3]= 0;

  /* Fill the last entry of homogeneous vector coordinates. */
  for (i=0;i<dof+2;i++)
    for (j=1;j<=max_point[i];j++)
      relative_point[i][j][4]=absolute_point[i][j][4]=1.0;
} // Atlas::initialize_points

/***************************************************************************/
/* Compute end device points; since movement of the end device may be more */
/* complicated than a pure rotation or translation a separate routine is   */
/* provided to describe the end device motion. Describe end device         */
/* points relative to tool frame in terms of tool_variable(tv).            */
/***************************************************************************/
void Atlas::compute_end_device (float tv)
{
  relative_point[6][ 1][1]=-(length[33]/2);
  relative_point[6][ 1][2]= length[37]+tv/2;
  relative_point[6][ 1][3]=-length[39]/2;

  relative_point[6][ 2][1]=-(length[33]/2);
  relative_point[6][ 2][2]= length[37]+tv/2;
  relative_point[6][ 2][3]= length[39]/2;

  relative_point[6][ 3][1]=-(length[33]/2);
  relative_point[6][ 3][2]= tv/2;
  relative_point[6][ 3][3]= length[39]/2;

  relative_point[6][ 4][1]=-(length[33]/2);
  relative_point[6][ 4][2]= tv/2;
  relative_point[6][ 4][3]=-length[39]/2;

  relative_point[6][ 5][1]= length[33]/2;
  relative_point[6][ 5][2]= length[37]+tv/2;
  relative_point[6][ 5][3]=-length[39]/2;

  relative_point[6][ 6][1]= length[33]/2;
  relative_point[6][ 6][2]= length[37]+tv/2;
  relative_point[6][ 6][3]= length[39]/2;

  relative_point[6][ 7][1]= length[33]/2;
  relative_point[6][ 7][2]= tv/2;
  relative_point[6][ 7][3]= length[39]/2;

  relative_point[6][ 8][1]= length[33]/2;
  relative_point[6][ 8][2]= tv/2;
  relative_point[6][ 8][3]=-length[39]/2;

  relative_point[6][ 9][1]=-(length[33]/2);
  relative_point[6][ 9][2]=-tv/2;
  relative_point[6][ 9][3]=-length[39]/2;

  relative_point[6][10][1]=-(length[33]/2);
  relative_point[6][10][2]=-tv/2;
  relative_point[6][10][3]= length[39]/2;

  relative_point[6][11][1]=-(length[33]/2);
  relative_point[6][11][2]=-length[37]-tv/2;
  relative_point[6][11][3]= length[39]/2;

  relative_point[6][12][1]=-(length[33]/2);
  relative_point[6][12][2]=-length[37]-tv/2;
  relative_point[6][12][3]=-length[39]/2;

  relative_point[6][13][1]= length[33]/2;
  relative_point[6][13][2]=-tv/2;
  relative_point[6][13][3]=-length[39]/2;

  relative_point[6][14][1]= length[33]/2;
  relative_point[6][14][2]=-tv/2;
  relative_point[6][14][3]= length[39]/2;

  relative_point[6][15][1]= length[33]/2;
  relative_point[6][15][2]=-length[37]-tv/2;
  relative_point[6][15][3]= length[39]/2;

  relative_point[6][16][1]= length[33]/2;
  relative_point[6][16][2]=-length[37]-tv/2;
  relative_point[6][16][3]=-length[39]/2;

  // two newly added points for link axis

  relative_point[6][17][1]= 0;
  relative_point[6][17][2]= length[37]+tv/2;
  relative_point[6][17][3]= 0;

  relative_point[6][18][1]= 0;
  relative_point[6][18][2]=-length[37]-tv/2;
  relative_point[6][18][3]= 0;

} // Atlas::compute_end_device

/***************************************************************************/
/* Update coordinate transformation matrices using joint vector.           */
/* These matrices may be found by any method and are not necessarily       */
/* obtained by Denavit-Hartenberg method.                                  */
/***************************************************************************/
void Atlas::update_maps (float *joint)
{
  float c1,s1,c2,s2,d3,c4,s4,c5,s5;

  c1=cos(joint[1]); s1=sin(joint[1]);
  c2=cos(joint[2]); s2=sin(joint[2]);
  d3=joint[3];
  c4=cos(joint[4]); s4=sin(joint[4]);
  c5=cos(joint[5]); s5=sin(joint[5]);

  map[1][1][1]=-s1;  map[1][1][2]=-c1;
  map[1][1][3]= 0.0;  map[1][1][4]= 0.0;
  map[1][2][1]=-c1;  map[1][2][2]= s1;
  map[1][2][3]= 0.0;  map[1][2][4]= 0.0;
  map[1][3][1]= 0.0; map[1][3][2]= 0.0;
  map[1][3][3]=-1.0;  map[1][3][4]= length[1];

  map[2][1][1]= c2;  map[2][1][2]= 0.0;
  map[2][1][3]=-s2;   map[2][1][4]= 0.0;
  map[2][2][1]= 0.0; map[2][2][2]= 1.0;
  map[2][2][3]= 0.0;  map[2][2][4]= 0.0;
  map[2][3][1]= s2;  map[2][3][2]= 0.0;
  map[2][3][3]= c2;   map[2][3][4]=-length[12];

  map[3][1][1]= 1.0; map[3][1][2]= 0.0;
  map[3][1][3]= 0.0;  map[3][1][4]=-d3-length[27]-length[21];
  map[3][2][1]= 0.0; map[3][2][2]= 1.0;
  map[3][2][3]= 0.0;  map[3][2][4]= 0.0;
  map[3][3][1]= 0.0; map[3][3][2]= 0.0;
  map[3][3][3]= 1.0;  map[3][3][4]=-length[20];

  map[4][1][1]= c4;  map[4][1][2]= 0.0;
  map[4][1][3]=-s4;   map[4][1][4]=-length[32];
  map[4][2][1]= 0.0; map[4][2][2]= 1.0;
  map[4][2][3]= 0.0;  map[4][2][4]= 0.0;
  map[4][3][1]= s4;  map[4][3][2]= 0.0;
  map[4][3][3]= c4;   map[4][3][4]= 0.0;

  map[5][1][1]= 1.0; map[5][1][2]= 0.0; 
  map[5][1][3]= 0.0;  map[5][1][4]=-length[35];
  map[5][2][1]= 0.0; map[5][2][2]= c5;  
  map[5][2][3]=-s5;   map[5][2][4]= 0.0;
  map[5][3][1]= 0.0; map[5][3][2]= s5;
  map[5][3][3]= c5;   map[5][3][4]= 0.0;
} // Atlas::update_maps

/***************************************************************************/
/**************************************************************************/
/*                       ATLAS cRobot Functions                           */
/**************************************************************************/
/* The following functions must be seperately defined for each kind of    */
/* robot. That is they are robot-specific. But for the case of the two    */
/* types of robots curruntly supported, i.e. Atlas and Rhino, they are    */
/* the same.                                                              */
/**************************************************************************/

void cRobot::movea (float *parameter)
{
  int i;

  for (i=1;i<=3;i++)
    change[i]=parameter[i]*((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1);
} // cRobot::movea

/***************************************************************************/
void cRobot::movew (float *parameter)
{
  int i;

  for (i=1;i<=2;i++) change[i+3]=parameter[i]*DEG_TO_RAD;
} // cRobot::movew

/***************************************************************************/
void cRobot::moveg (float *parameter)
{
  change[dof+1]=parameter[1]-theta[dof+1];
} // cRobot::moveg

/***************************************************************************/
void cRobot::movej (float *parameter)
{
  int i;

  for (i=1;i<=dof;i++)
    change[i]=parameter[i]*((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1);
  change[dof+1]=parameter[dof+1]-theta[dof+1];
} // cRobot::movej

/***************************************************************************/
void cRobot::move (float *parameter)
{
  int i;

  for (i=1;i<=dof+1;i++)
    change[i]=parameter[i]*((joint_status[i]==REVOLUTE) ?
                             DEG_TO_RAD : 1)-theta[i];
} // cRobot::move

/***************************************************************************/
void cRobot::speed (float *parameter)
{
  int i;

  for (i=1;i<=dof+1;i++)
    {
      if (parameter[i] < 1)
	      step[i] = 1;
      else if (parameter[i] > 8)
	      step[i] = 8;
      else
	      step[i]=parameter[i];
    } // for
  if (Filling || RecordFlag)    // '|| RecordFlag' is new
    {
      fprintf (OFile, "speed(%d,", RobNo);
      for (i = 1; i < dof+1; i++)
        fprintf (OFile, "%d, ", step[i]);
      fprintf (OFile, "%d)\n", step[dof+1]);
    } // for
} // cRobot::speed

/***************************************************************************/
/* This member function accepts the joint vector for this robot and after  */
/* computing the transformation matrix for the joint vector, shows it on   */
/* the screen.                                                             */
/***************************************************************************/
void cRobot::drkin (float *parameter)
{
  int i,j,out_of_range,y=30,color;
  float joint[10];
  frame t;
  char  str[50];

	RECT		 rc;
	HDC			 hdc;
	HPEN		 hpen, hpenOld;


  for (i=1;i<=dof;i++)
    joint[i]=parameter[i]*((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1);

  out_of_range=kinematics(t,joint);   /* obtain forward kinematics */

  if (!out_of_range)
    {
	    EnableAllButtons (FALSE);
	    HwndMap = HwndView;
	    GetClientRect (HwndMap, &rc);
	    InvalidateRect (HwndMap, &rc, TRUE);
      UpdateWindow (HwndMap);
	    hdc = GetDC (HwndMap);
      hpen = CreatePen (PS_SOLID, 0, RGB (0, 0, 255));
	    hpenOld = SelectObject (hdc, hpen);

      for (j=1;j<=4;j++)
        {
          for (i=1;i<=4;i++)
            {
	            sprintf(mess,"T[%d][%d]=%f",i,j,t[i][j]);
	            TextOut(hdc, 15, y, mess, strlen (mess));
	            y+=15;
	          } // for
          y+=10;
        } // for

	    SelectObject (hdc, hpenOld);
	    DeleteObject (hpen);
	    ReleaseDC (HwndMap, hdc);

	    EnableWindow (GetDlgItem (HwndPannel, ID_BACK), TRUE);
	    while (HwndMap != 0)
		    BackGround ();
	    GetClientRect (HwndView, &rc);
	    InvalidateRect (HwndView, &rc, TRUE);
	    UpdateWindow (HwndView);
	    EnableAllButtons (TRUE);
    } // if
  else
    {
      sprintf(mess," Joint%d out of range,press any key ...",out_of_range);
      message(mess);
    } // else
//  setcolor(color);
} // cRobot::drkin

/***************************************************************************/
/* This member function accepts the first two orientation vectors of the   */
/* tool-tip, together with its position vector, and after computing the    */
/* corresponding joint-vector shows it on the screen.                      */
/***************************************************************************/
void cRobot::inkin (float *parameter)
{
  int i,out_of_range,y=30,color;
  float joint[10],v[4],norm;
  frame t;
  char  str[50];

	RECT		 rc;
	HDC			 hdc;
	HPEN		 hpen, hpenOld;


  norm=0.0;
  for (i=1;i<=3;i++) v[i]=parameter[i];
  for (i=1;i<=3;i++) norm+=(v[i]*v[i]);
  for (i=1;i<=3;i++) v[i]/=sqrt(norm);
  for (i=1;i<=3;i++) t[i][1]=v[i];         /* 1st orientation vector */
  t[4][1]=0.0;

  norm=0.0;
  for (i=1;i<=3;i++) v[i]=parameter[i+3];
  for (i=1;i<=3;i++) norm+=(v[i]*v[i]);
  for (i=1;i<=3;i++) v[i]/=sqrt(norm);
  for (i=1;i<=3;i++) t[i][2]=v[i];         /* 2nd orientation vector */
  t[4][2]=0.0;

  norm=0.0;
  v[1]=t[2][1]*t[3][2]-t[3][1]*t[2][2];
  v[2]=t[3][1]*t[1][2]-t[1][1]*t[3][2];
  v[3]=t[1][1]*t[2][2]-t[2][1]*t[1][2];
  for (i=1;i<=3;i++) norm+=(v[i]*v[i]);
  for (i=1;i<=3;i++) v[i]/=sqrt(norm);
  for (i=1;i<=3;i++) t[i][3]=v[i];         /* 3rd orientation vector */
  t[4][3]=0.0;

  t[1][4]=parameter[7];                    /* position vector        */
  t[2][4]=parameter[8];
  t[3][4]=parameter[9];
  t[4][4]=1.0;

  out_of_range=inverse_kinematics(t,joint);

  if (!out_of_range)
    {

	    EnableAllButtons (FALSE);
	    HwndMap = HwndView;
	    GetClientRect (HwndMap, &rc);
	    InvalidateRect (HwndMap, &rc, TRUE);
      UpdateWindow (HwndMap);
	    hdc = GetDC (HwndMap);
      hpen = CreatePen (PS_SOLID, 0, RGB (0, 0, 255));
	    hpenOld = SelectObject (hdc, hpen);
      for (i=1;i<=dof;i++)
        {
          joint[i]/=((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1);
          sprintf(mess,"Joint[%d]=%f",i,joint[i]);
          TextOut (hdc, 15, y, mess, strlen (mess));
          y+=15;
        } // for

	    SelectObject (hdc, hpenOld);
	    DeleteObject (hpen);
	    ReleaseDC (HwndMap, hdc);

	    EnableWindow (GetDlgItem (HwndPannel, ID_BACK), TRUE);
	    while (HwndMap != 0)
		    BackGround ();
	    GetClientRect (HwndView, &rc);
	    InvalidateRect (HwndView, &rc, TRUE);
	    UpdateWindow (HwndView);
	    EnableAllButtons (TRUE);
    } // if
  else
    {
      sprintf(mess," Joint%d out of range,press any key ...",out_of_range);
      message(mess);
     } // else
//  setcolor(color);
} // cRobot::inkin

/***************************************************************************/
void cRobot::refer (void)
{
  register int i;

  for (i=1;i<=dof+1;i++)
    theta[i]=ref[i]*((joint_status[i]==REVOLUTE) ? DEG_TO_RAD:1);
  for (i=1;i<=dof+1;i++)
    step[i]=8; // was 4; now is slowest
  correction = 1; // new
} // cRobot::refer

/***************************************************************************/
void cRobot::Init (float *parameter)
{
  register int i;

  for (i=1;i<=dof+1;i++)
    theta[i]=parameter[i]*((joint_status[i]==REVOLUTE) ? DEG_TO_RAD:1);
  correction = 1; // new
} // cRobot::refer

/***************************************************************************/
/* This function accepts an input key from the teach-pendant device and    */
/* acts upon it accordingly.                                               */
/***************************************************************************/
void cRobot::tp (WORD key)
{
  register int i;
  int clear=1, esc=0, StepChange = 0;

  switch (key)
    {
	    case ID_REC : ParNo = 1;
			              parameter[0] = 0;
                    Record ();
		                break;

	    case ID_SEG : if (RecordFlag)
                      {
                        if (Start_End == START)
                          {
                            fprintf (Out, "startfill(%d)\n", RobNo);
                            Start_End = END;
                          } // if
                        else
                          fprintf (Out, "endfill(%d)\nstartfill(%d)\n",
                                        RobNo, RobNo);

                      } // if
		                break;

      case FWD1 : change[1]= velocity_table[1][step[1]];
			            break;

      case BCK1 : change[1]= -velocity_table[1][step[1]];
                  break;

      case FWD2 : change[2]= velocity_table[2][step[2]];
                  break;

      case BCK2 : change[2]= -velocity_table[2][step[2]];
                  break;

      case FWD3 : change[3]= velocity_table[3][step[3]];
                  break;

      case BCK3 : change[3]=-velocity_table[3][step[3]];
                  break;

      case FWD4 : change[4]= velocity_table[4][step[4]];
                  break;

      case BCK4 : change[4]=-velocity_table[4][step[4]];
                  break;

      case FWD5 : change[5]= velocity_table[5][step[5]];
                  break;

      case BCK5 : change[5]=-velocity_table[5][step[5]];
                  break;

      case FWD6 : change[6]= velocity_table[6][step[6]];
                  break;

      case BCK6 : change[6]=-velocity_table[6][step[6]];
                  break;

	    case SLOW1 : step[1]=min(8,step[1]+1);
                   StepChange = 1;
			             break;

	    case SLOW2 : step[2]=min(8,step[2]+1);
                   StepChange = 1;
			             break;

	    case SLOW3 : step[3]=min(8,step[3]+1);
                   StepChange = 1;
			             break;

	    case SLOW4 : step[4]=min(8,step[4]+1);
                   StepChange = 1;
			             break;

	    case SLOW5 : step[5]=min(8,step[5]+1);
                   StepChange = 1;
			             break;

	    case SLOW6 : step[6]=min(8,step[6]+1);
                   StepChange = 1;
			             break;

	    case FAST1 : step[1]=max(1,step[1]-1);
                   StepChange = 1;
			             break;

	    case FAST2 : step[2]=max(1,step[2]-1);
                   StepChange = 1;
			             break;

	    case FAST3 : step[3]=max(1,step[3]-1);
                   StepChange = 1;
			             break;

	    case FAST4 : step[4]=max(1,step[4]-1);
                   StepChange = 1;
			             break;

	    case FAST5 : step[5]=max(1,step[5]-1);
                   StepChange = 1;
			             break;

	    case FAST6 : step[6]=max(1,step[6]-1);
                   StepChange = 1;
			             break;

      default  : clear = 0;
    } // switch
  if (clear)
    start();
  if (StepChange && RecordFlag) // RecordFlag was Filling
    {
      fprintf (Out, "speed(%d,", RobNo);   // all Out's were OFile's
      for (i = 1; i < dof+1; i++)
        fprintf (Out, "%d, ", step[i]);
      fprintf (Out, "%d)\n", step[dof+1]);
    } // if
  StepChange = 0;
} // cRobot::tp

/***************************************************************************/
/*
void cRobot::start(void)
{
  register int i,j,end, k;
  int again,correction;
  float s,t,delta[10];

  clock_t Start, End, ElapsedTime;

  Start = End = clock();

  do
   {
     again=correction=0;
     /* find incremental correction for all joints */
     ElapsedTime = 1; /*(End - Start) / CLK_TCK;*/
     for (i=1;i<=dof+1;i++)
       {
         delta[i]=0.0;
         if (change[i])
           {
	           correction=1;
	           t=fmin(fabs(change[i]),ElapsedTime * velocity_table[i][step[i]])*sign(change[i]);
	           s=theta[i]+t;
	           if (s>max_joint[i])
               {          /* upper limit */
	               delta[i]=max_joint[i]-theta[i];
	               theta[i]=max_joint[i];
	               change[i]=0.0;
	             } // if
	           else if (s<min_joint[i])
               {     /* lower limit */
	               delta[i]=min_joint[i]-theta[i];
	               theta[i]=min_joint[i];
	               change[i]=0.0;
               } // else
	           else
               {                    /* within the limits */
	               delta[i]=t;
	               change[i]-=t;
	               theta[i]=s;
	               if (change[i]) again=1;
               } // else
	         } // if
       } // for
     display_robot();

     if (OnLineTest)
       {
         if (MultiCollision_Detect ())
           Alarm (1);
         else
           Alarm (0);
       } // if

     for (i = 0; i < MaxRobot; i++)
       VUpdate (i);

     End = clock();
   }
  while(again);
}
*/

/***************************************************************************/
/* This procedure starts animation and real system operation for a         */
/* previously supplied motion command. The array 'change' contatins the    */
/* values by which every joint of this robot must change. In each animation*/
/* step, each joint moves towards its final desired value with respect to  */
/* its current joint speed. The new configuration is then displayed.       */
/* If on-line collision detect is turned on, then the alarm light is       */
/* changed to red in case there is any collision. 'VUpdate' is called to   */
/* update the voxel structure of the swept-volume of the robot. This       */
/* happens only when the 'Filling' mode is on. If so, the current joint    */
/* vector of the robot is written into the corresponding .pat file.        */
/***************************************************************************/
void cRobot::start (void)
{
  register int i;
	int      again/*, UPdate*/;
  float    s,t,delta[10];

  clock_t Start, End, ElapsedTime;

  UPdate = 0;
  Start = End = clock();
  do
    {
      again=correction=0;
      /* find incremental correction for all joints */
      ElapsedTime = 1; // (End - Start) / CLK_TCK;
      for (i=1;i<=dof+1;i++)
        {
	        delta[i]=0.0;
	        if (change[i])
            {
	            correction=1;
	            t=fmin(fabs(change[i]),ElapsedTime * velocity_table[i][step[i]])*sign(change[i]);
	            s=theta[i]+t;
	            if (s>max_joint[i])           /* upper limit */
                {
		              delta[i]=max_joint[i]-theta[i];
		              theta[i]=max_joint[i];
		              change[i]=0.0;
   	            } // if
	            else if (s<min_joint[i])      /* lower limit */
                {
		              delta[i]=min_joint[i]-theta[i];
		              theta[i]=min_joint[i];
		              change[i]=0.0;
		            } // else
	            else                     /* within the limits */
                {
		              delta[i]=t;
		              change[i]-=t;
		              theta[i]=s;
		              if (change[i]) again=1;
                } // else
	          } // if
	      } // for
      if (correction)
        {
          UPdate = 1;
          display_robot();
          if (OnLineTest)
            {
              if (MultiCollision_Detect ())
                Alarm (1);
              else
                Alarm (0);
            } // if
          VUpdate (RobNo);

/*        // Notice, New
          for (i = 0; i < MaxRobot; i++)
            VUpdate (i);
*/
        } // if
      End = clock();
    } // do
  while(again);
  if (UPdate/*correction*/ && Filling) // new
    {
      fprintf (OFile, "move(%d,", RobNo);
      for (i = 1; i < dof+1; i++)
        {
          fprintf (OFile, "%- 10.4g, ", theta[i]/((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1.0));
        } /* for */
      fprintf (OFile, "%- 10.4g)\n", theta[dof+1]);
      fprintf (OFile, "start(%d)\n", RobNo);
    } // if
} // cRobot::start

/***************************************************************************/
/* This member function updates the current joint vector of this robot,    */
/* when the robots' movements are read out of their files concurrently.    */
/* The array 'change' contatins the values by which every joint of this    */
/* robot must change. Each joint moves towards its final desired value     */
/* with respect to its current joint speed. If this robot has reached its  */
/* final position, 'AnyMoreChange' is set to 0, and otherwise to 1.        */                                
/***************************************************************************/

void cRobot::Update (void)
{
  register int i;
  float    s,t,delta[10];

  UpdateDisplay = 0;
  End = time(NULL);
  ElapsedTime = difftime (End, Start);
  if (!AnyMoreChange || (ElapsedTime < 0.0))
    return;
  AnyMoreChange = 0;
  for (i=1;i<=dof+1;i++)
    {
      delta[i]=0.0;
      if (change[i])
        {
          correction = 1; // new
	        UpdateDisplay=1;
	        t=fmin(fabs(change[i]),ElapsedTime * velocity_table[i][step[i]])*sign(change[i]);
	        s=theta[i]+t;
	        if (s>max_joint[i])           /* upper limit */
            {
	            delta[i]=max_joint[i]-theta[i];
	            theta[i]=max_joint[i];
	            change[i]=0.0;
 	          } // if
	        else if (s<min_joint[i])      /* lower limit */
            {
	            delta[i]=min_joint[i]-theta[i];
	            theta[i]=min_joint[i];
	            change[i]=0.0;
	          } // else
	        else                     /* within the limits */
            {
	            delta[i]=t;
	            change[i]-=t;
	            theta[i]=s;
	            if (change[i]) AnyMoreChange=1;
            } // else
	      } // if
    } // for
  if (!UpdateDisplay)
    AnyMoreChange = 0;
  else
    Start = End;
} // cRobot::Update

/***************************************************************************/
/* This member function opens this robot's file to be used in a 'multiread'*/
/* command.                                                                */
/***************************************************************************/
void cRobot::OpenFile (void)  // Must return a value????
{
  AnyMoreChange = 1;
  LineNo = 0;
  InFileName[5] = '0' + RobNo;
  if ((InFile = fopen (InFileName, "rt"))==NULL)
    AnyMoreChange = 0;
} // cRobot::OpenFile

/***************************************************************************/
/* This member function closes this robot's file which was opened          */
/* previously.                                                             */
/***************************************************************************/
void cRobot::CloseFile(void)
{
  AnyMoreChange = 0;
  if (InFile != NULL)
    fclose (InFile);
} // cRobot::CloseFile

/***************************************************************************/
/* This member function reads the next command out of this robot's file,   */
/* and acts upon it properly.                                              */
/***************************************************************************/
void cRobot::GetNextCommand(void)
{
  register int i, error;

  if (!InFile)
    return;
  while (!feof(InFile))
    {
      fgets(command_part,MAXCHAR,InFile);
      command_part[strlen(command_part)-1]=0;
      i=0;
      while(command_part[i]==' ')         /* ignore preceeding spaces */
        i++;
      if (i)
        strcpy(command_part,&command_part[i]);
      preprocess();
      error = 0;
      for (i=0;i<MAX_COMMAND;i++)
        if (!strcmp(command_part,command[i].name) &&
            (command[i].CommandType < 2) &&
            !(get_param()))
	        {
              if ((command[i].Single == 1) && (parameter[0] != RobNo))
                break;
              switch (command[i].CommandType)
                {
                  case -1 : if (ParNo != 1)
                              error = 1;
                            else
                              {
                                AnyMoreChange = 1;
//                              Start = clock() + CLK_TCK * parameter[1];
                                Start = time(NULL) + parameter[0];
                                return;
                              } // else
                            break;

                  case 0  : error = command[i].function();
                            if (!strcmp (command[i].name, "sync"))
                              {
                                AnyMoreChange = 0;
				                        return;
                              } // if
                            break;

                  case 1  : AnyMoreChange = 1;
//                          Start = clock ();
                            Start = time(NULL);
                            if (!strcmp (command[i].name, "refer"))
                              refer();
                            return;
                } // switch
              break;
            } // if
      if (error && !feof(InFile))
        {
	        sprintf(param_part,"Syntax error on line #%d,press any key ...",LineNo);
          message (param_part);
	      } // if
      LineNo++;
    } // while
  if (feof (InFile))
    AnyMoreChange = 0; 
} // cRobot::GetNextCommand

/***************************************************************************/
/***************************************************************************/
void cRobot::startfill (void)
{
  char FileName[13] = "robot .pat";
  char StrCat[20];
  register int i;

  if (Filling)
    {
      message("Already in Fill Mode, press any key ...");
      return;
    } /* if */
  Filling = 1;
  Vinit (RobNo, 0);
  sprintf (FileName, "rob%d-%d.pat", RobNo, CurrentFile);
  OFile = fopen (FileName, "wt");
  sprintf (FirstMove, "move(%d,", RobNo);
  for (i = 1; i < dof+1; i++)
    {
      sprintf (StrCat, "%- 10.4g, ", theta[i]/((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1.0));
      strcat (FirstMove, StrCat);
    } /* for */
  sprintf (StrCat, "%- 10.4g)\n", theta[dof+1]);
  strcat (FirstMove, StrCat);

  fprintf (OFile, FirstMove);
  fprintf (OFile, "start(%d)\n", RobNo);

  fprintf (OFile, "speed(%d,", RobNo);
  for (i = 1; i < dof+1; i++)
    fprintf (OFile, "%d, ", step[i]);
  fprintf (OFile, "%d)\n", step[dof+1]);
} // cRobot::startfill

/***************************************************************************/
/***************************************************************************/
void cRobot::endfill (void)
{
  char FileName[13] = "robot .vol";
  if (!Filling)
    {
      message("Not in Fill Mode, press any key ...");
      return;
    } /* if */
  Filling = 0;
  fclose (OFile);
  VFinalize (RobNo);

  Hide = 1;
  strcpy (command_part, FirstMove);
  preprocess ();
  process_command ();

  sprintf (command_part, "start(%d)", RobNo);
  preprocess ();
  process_command ();

  Hide = 1;
  Vinit (RobNo, 1);   // with no filling; at this point the required volume
                      // is determined for each link
  sprintf (command_part, "fread(rob%d-%d.pat)", RobNo, CurrentFile);
  preprocess ();
  process_command ();
  sprintf (FileName, "rob%d-%d.vol", RobNo, CurrentFile);
  VSave (RobNo, FileName);
  VFinalize (RobNo);  // must write to an output file and before
                      // exit show all the volumes
  CurrentFile++;
  Hide = 0;
} // cRobot::endfill

/***************************************************************************/
/* This member function solves direct kinematic equations for this robot;  */
/* returns 0 if all joints are in the work space, and the # of the first   */
/* out of range joint otherwise.                                           */
/* This function expects all revolute joint values in radians.             */
/***************************************************************************/
int Atlas::kinematics (frame t,float *joint)
{
  int i;
  float s1,c1,s2,c2,d3,s5,c5,s24,c24;
  float l1,l12,l20,l21,l27,l32,l35,l39,l40;

  /* check workspace */
  for (i=1;i<=dof;i++)
    if (joint[i]<min_joint[i] || joint[i]>max_joint[i])
      return i;

  l1 =length[ 1]   ;
  l12=length[12]   ;
  l20=length[20]   ;
  l21=length[21]   ;
  l27=length[27]   ;
  l32=length[32]   ;
  l35=length[35]   ;
  l39=length[39]   ;
  l40=length[40]   ;

  d3 =    joint[3] ;
  s1 =sin(joint[1]);
  c1 =cos(joint[1]);
  s2 =sin(joint[2]);
  c2 =cos(joint[2]);
  s5 =sin(joint[5]);
  c5 =cos(joint[5]);
  s24=sin(joint[2]+joint[4]);
  c24=cos(joint[2]+joint[4]);

  t[1][1]=-c1*s5-s1*s24*c5;
  t[2][1]= s1*s5-c1*s24*c5;
  t[3][1]= c5*c24;
  t[4][1]= 0.0;

  t[1][2]=-s1*s24*s5+c1*c5;
  t[2][2]=-c1*s24*s5-s1*c5;
  t[3][2]= s5*c24;
  t[4][2]= 0.0;

  t[1][3]= s1*c24;
  t[2][3]= c1*c24;
  t[3][3]= s24;
  t[4][3]= 0.0;

  t[1][4]= s1*c2*(d3+l27+l21)-l20*s1*s2+l32*s1*c2+(l35+l39/2.0+l40)*s1*c24;
  t[2][4]= c1*c2*(d3+l27+l21)-l20*c1*s2+l32*c1*c2+(l35+l39/2.0+l40)*c1*c24;
  t[3][4]=    s2*(d3+l27+l21)+l20   *c2+l32   *s2+(l35+l39/2.0+l40)   *s24+l1+l12;
  t[4][4]= 1.0;

  frame_multiply (map[0], t, t);
  return 0;
} // Atlas::kinematics

/***************************************************************************/
/* This member function solves inverse kinematic equations of this robot;  */
/* returns 0 if all joints are in the work space, and the # of the first   */
/* out of range joint otherwise.                                           */
/* This function returns all revolute joint values in radians.             */
/***************************************************************************/
int Atlas::inverse_kinematics (frame t,float *joint)
{
  int i;
  float t24,w1,w2,r,phi,s1,s2,c1,c2,c4,s24,c24;
  float l1,l12,l20,l21,l27,l32,l35,l39,l40;
  float ll1,ll2,ll3,mm1,mm2,mm3,pp1,pp2,pp3;

  frame_multiply (inv_map[0], t, t);

  ll1=t[1][1]; ll2=t[2][1]; ll3=t[3][1];
  mm1=t[1][2]; mm2=t[2][2]; mm3=t[3][2];
  pp1=t[1][4]; pp2=t[2][4]; pp3=t[3][4];

  l1 =length[ 1];
  l12=length[12];
  l20=length[20];
  l21=length[21];
  l27=length[27];
  l32=length[32];
  l35=length[35];
  l39=length[39];
  l40=length[40];

  joint[1]=atan3(pp1,pp2);
  s1=sin(joint[1]);
  c1=cos(joint[1]);

  t24=atan3(-s1*ll1-c1*ll2,ll3); s24=sin(t24);      c24=cos(t24);
  w1=(l35+l39/2.0+l40)*c24-(s1*pp1+c1*pp2);
  w2=(l35+l39/2.0+l40)*s24-pp3+l1+l12;
  r=sqrt(w1*w1+w2*w2);  phi=atan4(w2,w1);

  joint[2]=phi+atan4(l20/r,-sqrt(1.0-l20*l20/(r*r)));
  c2=cos(joint[2]);
  s2=sin(joint[2]);
  joint[4]=t24-joint[2];
  c4=cos(joint[4]);
  joint[3]=c2*(s1*pp1+c1*pp2)+s2*(pp3-l1-l12)-(l35+l39/2.0+l40)*c4-l21-l27-l32;
  joint[5]=atan4(s1*ll2-c1*ll1,c1*mm1-s1*mm2);

  /* check workspace */
  /* for (i=1;i<=dof;i++)
       if (joint[i]<min_joint[i] || joint[i]>max_joint[i]) return i; */

  return 0;
} // Atlas::inverse_kinematics
