
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
#include "rhino.h"

#define  J_ANGLE   0
#define  LINK_TW   1
#define  J_DIST    2
#define  LINK_LEN  3

extern int RecordFlag;
float Rhino_velocity_table[MaxLink][9] = // Not Known for Rhino!!!
    {{0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0,1.60,.350,.300,.250,.200,.150,.100,.020},
		{0,1.60,.119,.102,.085,.068,.051,.034,.020},
		{0,1.60,.119,.102,.085,.068,.051,.034,.020},
		{0,1.60,.350,.300,.250,.200,.150,.100,.020},
		{0,3.142,.350,.300,.250,.200,.150,.100,.020},
		{0,1.60,.119,.102,.085,.068,.051,.034,.020},};

LinkType Rhino_LinkArray[LinkNo] = {{{255,0,0}, 7.185, XZ, Y, 1, {{6,
	    {{1,3,4,2,1,-1},       // Robot Base
	     {1,2,6,5,1,-1},
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,7,3,1,-1},
	     {5,6,8,7,5,-1},},},},},
				                            {{255,255,0}, /*7.65*/ 6.75, XY, Z, 1, {{6,
	    {{1,3,4,2,1,-1},       // Robot Mobile Part
	     {1,2,6,5,1,-1},
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,7,3,1,-1},
	     {5,6,8,7,5,-1},},},},},
				                            {{0,255,0}, 5.12, XZ, Y, 1, {{6,
	    {{1,3,4,2,1,-1},       // Robot Shoulder
	     {1,2,6,5,1,-1},
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,7,3,1,-1},
	     {5,6,8,7,5,-1},},},},},
				                            {{0,255,255}, 4.58, XZ, Y, 1, {{6,
	    {{1,3,4,2,1,-1},       // Robot Elbow
	     {1,2,6,5,1,-1},
	     {2,4,8,6,2,-1},
	     {4,3,7,8,4,-1},
	     {1,5,7,3,1,-1},
	     {5,6,8,7,5,-1},},},},},
				                            {{0,0,255}, 3.175, YZ, X, 1, {{5,
	    {{1,3,4,2,1,-1},       // Robot Tool Pitch
	     {1,5,3,1,-1},
	     {2,4,6,2,-1},
	     {1,2,6,5,1,-1},
	     {3,5,6,4,3,-1},},},},},
				                            {{255,0,255}, 2.29, YZ, X, 1, {{2,
       {{1,2,3,4,1,-1},      // Robot Roll
        {4,3,2,1,4,-1},},},},},
                            
				                            {{128,128,128}, 8, YZ, X, 2, {{2,
       {{1,2,3,4,1,-1},      // Tool
        {4,3,2,1,4,-1},},},  
                                                        {2,
       {{5,6,7,8,5,-1},
        {8,7,6,5,8,-1},},},},}};

int Rhino_joint_status[MaxLink] = {-1, REVOLUTE, REVOLUTE, REVOLUTE,
			                                 REVOLUTE, REVOLUTE, REVOLUTE};

/* # of points on each link, starting from base (link 0)
   to the last link (the end device) */
int Rhino_max_point[MaxLink] = {10,10,10,10,8,6,10}; // NOTICE

/* # of surfaces on each link, starting from base
   (link 0) to the last link (the end device) */
int Rhino_max_surface[MaxLink]={6,6,6,6,5,4,8}; // NOTICE

float Rhino_length[1];

DCELType  Rhino_Dcel[MaxLink] =
                    {
                     {12, 6, 8,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {1, 3, 1, 6, 4, 6, 0, 0, 0},
                       {3, 4, 2, 6, 1, 7, 0, 0, 0},
/* Robot Base    */    {4, 2, 3, 6, 2, 8, 0, 0, 0},
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
/* Robot Mobile Part */{4, 2, 3, 6, 2, 8, 0, 0, 0},
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
/* Robot Shoulder*/    {4, 2, 3, 6, 2, 8, 0, 0, 0},
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
/* Robot Elbow   */    {4, 2, 3, 6, 2, 8, 0, 0, 0},
                       {2, 1, 4, 6, 3, 5, 0, 0, 0},
                       {1, 5, 4, 1, 1, 10, 0, 0, 0},
                       {3, 7, 1, 2, 2, 11, 0, 0, 0},
                       {4, 8, 2, 3, 3, 12, 0, 0, 0},
                       {2, 6, 3, 4, 4, 9, 0, 0, 0},
                       {8, 6, 5, 3, 7, 10, 0, 0, 0},
                       {6, 5, 5, 4, 8, 11, 0, 0, 0},
                       {5, 7, 5, 1, 5, 12, 0, 0, 0},
                       {7, 8, 5, 2, 6, 9, 0, 0, 0}}},

                     {9, 5, 6,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {1, 2, 1, 4, 5, 4, 0, 0, 0},
                       {1, 3, 2, 1, 1, 6, 0, 0, 0},
/* Robot Tool Pitch*/  {3, 4, 5, 1, 2, 8, 0, 0, 0},
                       {2, 4, 1, 3, 7, 3, 0, 0, 0},
                       {1, 5, 4, 2, 2, 9, 0, 0, 0},
                       {3, 5, 2, 5, 3, 5, 0, 0, 0},
                       {2, 6, 3, 4, 1, 8, 0, 0, 0},
                       {4, 6, 5, 3, 4, 9, 0, 0, 0},
                       {5, 6, 4, 5, 6, 7, 0, 0, 0}}},

                     {4, 2, 4,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {3, 4, 1, 2, 4, 2, 0, 0, 0},
/* Tool Roll    */     {4, 1, 1, 2, 1, 3, 0, 0, 0},
                       {1, 2, 1, 2, 2, 4, 0, 0, 0},
                       {2, 3, 1, 2, 3, 1, 0, 0, 0}}},

                     {8, 4, 8,
                      {{0, 0, 0, 0, 0, 0, 0, 0, 0},
                       {3, 4, 1, 2, 4, 2, 0, 0, 0},
/* Tool Gripper */     {4, 1, 1, 2, 1, 3, 0, 0, 0},
                       {1, 2, 1, 2, 2, 4, 0, 0, 0},
                       {2, 3, 1, 2, 3, 1, 0, 0, 0},

                       {7, 8, 3, 4, 8, 6, 0, 0, 0},
                       {8, 5, 3, 4, 5, 7, 0, 0, 0},
                       {5, 6, 3, 4, 6, 8, 0, 0, 0},
                       {6, 7, 3, 4, 7, 5, 0, 0, 0}}}};

float RHINO_DH_MATRIX[MaxLink][4] = {{0, 0, 0, 0},
                                     {0.0, -M_PI_2, 26.04, 0.0},
                                     {-M_PI_2, 0.0, 0.0, 22.86},
                                     {M_PI_2, 0.0, 0.0, 22.86},
                                     {0.0, -M_PI_2, 0.0, .95},
                                     {-M_PI_2, 0.0, 16.83, 0.0},
                                     {0.0, 0.0, 0.0, 0.0}};


/***************************************************************************/
/* Workspace of the robot (limits are in radians for revolute joints)      */
/***************************************************************************/

float Rhino_max_joint[10]={0.0,180.0*D2R, 60.0*D2R,  150.0*D2R,
                           70.0*D2R,9999.9*D2R,90.0*D2R};
float Rhino_min_joint[10]={0.0,-180.0*D2R,-160.0*D2R,-90.0*D2R,
                           -200.0*D2R,-9999.9*D2R,-60.0*D2R};
float Rhino_ref[10]={0.0,0.0,-90.0,90.0,0.0,-90.0,-60};

extern float atan3(float y,float x),atan4(float y,float x);

void D_H (frame Map, float *Vars);

extern int   Hide;
extern int   ParNo;
extern int   Segment;
extern int   simulation_mode;
extern int   teach_pendant_mode;
extern FILE  *Out;
extern float parameter[]; 
extern char  command_part[], param_part[], mess[]; 
extern CommandType   command[]; 

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
extern void frame_multiply(frame a,frame b,frame c);
extern float fmin (float a, float b);

/***************************************************************************/
// This is the initialization routine for robot Rhino.                     */
/***************************************************************************/
Rhino::Rhino (void)
{
  register int i, j;

  length = Rhino_length;
  ref = Rhino_ref;
  max_joint = Rhino_max_joint;
  min_joint = Rhino_min_joint;
  joint_status = Rhino_joint_status;
  max_point = Rhino_max_point;
  max_surface = Rhino_max_surface;
  LinkArray = Rhino_LinkArray;
  DcelP = Rhino_Dcel;
  for (i = 0; i < 7; i++)
    change[i] = 0;
  for (i = 0; i < 7; i++)  // MaxL = 7
    for (j = 0; j <= 8; j++)
      velocity_table[i][j] = Rhino_velocity_table[i][j];
} // Rhino::Rhino

/***************************************************************************/
/*                 Rhino-Specific Functions                                */
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
/* Notice : The Direct-Kinematic and Inverse-Kinematic functions for       */
/* Rhino robot are to be written!                                          */
/***************************************************************************/
void Rhino::initialize_maps (void)
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
  map[0][1][4] =  map[0][3][4] = 0;
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
  D_H (map[6], RHINO_DH_MATRIX[6]);
/*
  map[6][1][1]= 0.0; map[6][1][2]= 1.0;
  map[6][1][3]= 0.0; map[6][1][4]= 0.0;
  map[6][2][1]=-1.0; map[6][2][2]= 0.0;
  map[6][2][3]= 0.0; map[6][2][4]= 0.0;
  map[6][3][1]= 0.0; map[6][3][2]= 0.0;
  map[6][3][3]= 1.0; map[6][3][4]= 17.15;
*/
} // Rhino::initialize_maps

/***************************************************************************/
/* Initialize all relative points. This function is called only once,      */
/* and therefore there is no need to optimize it for speed.                */
/* The last two points of each link specify the endpoints of link axis     */
/* which is used for the on-line collision-detection algorithm.            */
/***************************************************************************/
void Rhino::initialize_points (void)
{
  /* basic angular values used in describing the robot */

  register int i,j;

  /***********************/
  /* Base of the robot : */
  /***********************/
  absolute_point[0][ 1][1]=relative_point[0][ 1][1]= 2.54;
  absolute_point[0][ 1][2]=relative_point[0][ 1][2]= 5.08;
  absolute_point[0][ 1][3]=relative_point[0][ 1][3]= 0.0;

  absolute_point[0][ 2][1]=relative_point[0][ 2][1]=-7.62;
  absolute_point[0][ 2][2]=relative_point[0][ 2][2]= 5.08;
  absolute_point[0][ 2][3]=relative_point[0][ 2][3]= 0.0;

  absolute_point[0][ 3][1]=relative_point[0][ 3][1]= 2.54;
  absolute_point[0][ 3][2]=relative_point[0][ 3][2]=-5.08;
  absolute_point[0][ 3][3]=relative_point[0][ 3][3]= 0.0;

  absolute_point[0][ 4][1]=relative_point[0][ 4][1]=-7.62;
  absolute_point[0][ 4][2]=relative_point[0][ 4][2]=-5.08;
  absolute_point[0][ 4][3]=relative_point[0][ 4][3]= 0.0;

  absolute_point[0][ 5][1]=relative_point[0][ 5][1]= 2.54;
  absolute_point[0][ 5][2]=relative_point[0][ 5][2]= 5.08;
  absolute_point[0][ 5][3]=relative_point[0][ 5][3]= 17;

  absolute_point[0][ 6][1]=relative_point[0][ 6][1]=-7.62;
  absolute_point[0][ 6][2]=relative_point[0][ 6][2]= 5.08; 
  absolute_point[0][ 6][3]=relative_point[0][ 6][3]= 17;

  absolute_point[0][ 7][1]=relative_point[0][ 7][1]= 2.54;
  absolute_point[0][ 7][2]=relative_point[0][ 7][2]=-5.08;
  absolute_point[0][ 7][3]=relative_point[0][ 7][3]= 17;

  absolute_point[0][ 8][1]=relative_point[0][ 8][1]=-7.62;
  absolute_point[0][ 8][2]=relative_point[0][ 8][2]=-5.08;
  absolute_point[0][ 8][3]=relative_point[0][ 8][3]= 17;

  // two newly added points for link axis
  absolute_point[0][ 9][1]=relative_point[0][ 9][1]=-2.54;
  absolute_point[0][ 9][2]=relative_point[0][ 9][2]= 0.0;
  absolute_point[0][ 9][3]=relative_point[0][ 9][3]= 0.0;

  absolute_point[0][10][1]=relative_point[0][10][1]=-2.54;
  absolute_point[0][10][2]=relative_point[0][10][2]= 0.0;
  absolute_point[0][10][3]=relative_point[0][10][3]= 17.0;

  /***********************/
  /* Robot Mobile :      */
  /***********************/
  relative_point[1][ 1][1]= 2.54;
  relative_point[1][ 1][2]= 8.89;
  relative_point[1][ 1][3]=-5.08;

  relative_point[1][ 2][1]=-13.97;
  relative_point[1][ 2][2]= 8.89;
  relative_point[1][ 2][3]=-5.08;

  relative_point[1][ 3][1]= 2.54;
  relative_point[1][ 3][2]= 0.0; //1.27; //-2.54;
  relative_point[1][ 3][3]=-5.08;

  relative_point[1][ 4][1]=-13.97;
  relative_point[1][ 4][2]= 0.0; //1.27; //-2.54;
  relative_point[1][ 4][3]=-5.08;

  relative_point[1][ 5][1]= 2.54;
  relative_point[1][ 5][2]= 8.89;
  relative_point[1][ 5][3]= 5.08;

  relative_point[1][ 6][1]=-13.97;
  relative_point[1][ 6][2]= 8.89;
  relative_point[1][ 6][3]= 5.08;

  relative_point[1][ 7][1]= 2.54;
  relative_point[1][ 7][2]= 0.0; //1.27; //-2.54;
  relative_point[1][ 7][3]= 5.08;

  relative_point[1][ 8][1]=-13.97;
  relative_point[1][ 8][2]= 0.0; //1.27; //-2.54;
  relative_point[1][ 8][3]= 5.08;

  // two newly added points for link axis
  relative_point[1][ 9][1]= 2.54;
  relative_point[1][ 9][2]= 4.445;
  relative_point[1][ 9][3]= 0.0; //0.0;

  relative_point[1][10][1]=-13.97;
  relative_point[1][10][2]= 4.445;
  relative_point[1][10][3]= 0.0; //0.0;

  /************************/
  /* Robot Shoulder :     */
  /************************/
  relative_point[2][ 1][1]= 0.0;
  relative_point[2][ 1][2]= 0.0;
  relative_point[2][ 1][3]= -4.445;

  relative_point[2][ 2][1]=-11.43;
  relative_point[2][ 2][2]= 2.54;
  relative_point[2][ 2][3]=-4.445;

  relative_point[2][ 3][1]=-11.43;
  relative_point[2][ 3][2]=-2.54;
  relative_point[2][ 3][3]=-4.445;

  relative_point[2][ 4][1]=-22.86;
  relative_point[2][ 4][2]= 0.0;
  relative_point[2][ 4][3]=-4.445;

  relative_point[2][ 5][1]= 0.0;
  relative_point[2][ 5][2]= 0.0;
  relative_point[2][ 5][3]= 4.445;

  relative_point[2][ 6][1]=-11.43;
  relative_point[2][ 6][2]= 2.54;
  relative_point[2][ 6][3]= 4.445;

  relative_point[2][ 7][1]=-11.43;
  relative_point[2][ 7][2]=-2.54; 
  relative_point[2][ 7][3]= 4.445;

  relative_point[2][ 8][1]=-22.86;
  relative_point[2][ 8][2]= 0.0; 
  relative_point[2][ 8][3]= 4.445;

  // two newly added points for link axis
  relative_point[2][ 9][1]= 0.0;
  relative_point[2][ 9][2]= 0.0; 
  relative_point[2][ 9][3]= 0.0;

  relative_point[2][10][1]=-22.86;
  relative_point[2][10][2]= 0.0; 
  relative_point[2][10][3]= 0.0;

  /************************/
  /* Robot Elbow :        */  
  /************************/
  relative_point[3][ 1][1]=-11.43;
  relative_point[3][ 1][2]= 2.54;
  relative_point[3][ 1][3]=-3.81; 

  relative_point[3][ 2][1]=-22.86; 
  relative_point[3][ 2][2]= 0.0;
  relative_point[3][ 2][3]=-3.81; 

  relative_point[3][ 3][1]= 0.0; 
  relative_point[3][ 3][2]= 0.0; 
  relative_point[3][ 3][3]=-3.81; 

  relative_point[3][ 4][1]=-11.43;
  relative_point[3][ 4][2]=-2.54; 
  relative_point[3][ 4][3]=-3.81; 

  relative_point[3][ 5][1]=-11.43; 
  relative_point[3][ 5][2]= 2.54;
  relative_point[3][ 5][3]= 3.81;

  relative_point[3][ 6][1]=-22.86;
  relative_point[3][ 6][2]= 0.0;
  relative_point[3][ 6][3]= 3.81;

  relative_point[3][ 7][1]= 0.0; 
  relative_point[3][ 7][2]= 0.0; 
  relative_point[3][ 7][3]= 3.81;

  relative_point[3][ 8][1]=-11.43;
  relative_point[3][ 8][2]=-2.54; 
  relative_point[3][ 8][3]= 3.81;

  relative_point[3][ 9][1]= 0.0; 
  relative_point[3][ 9][2]= 0.0;
  relative_point[3][ 9][3]= 0.0; 

  relative_point[3][10][1]=-22.86; 
  relative_point[3][10][2]= 0.0;
  relative_point[3][10][3]= 0.0; 

  /**************************/
  /* Tool Pitch :           */
  /**************************/
  relative_point[4][ 1][1]= 3.175; 
  relative_point[4][ 1][2]= 3.175;
  relative_point[4][ 1][3]=-7.62;

  relative_point[4][ 2][1]= 3.175;
  relative_point[4][ 2][2]=-3.175;
  relative_point[4][ 2][3]=-7.62;

  relative_point[4][ 3][1]= 3.175;
  relative_point[4][ 3][2]= 3.175;
  relative_point[4][ 3][3]= 7.62;

  relative_point[4][ 4][1]= 3.175;
  relative_point[4][ 4][2]=-3.175;
  relative_point[4][ 4][3]= 7.62;

  relative_point[4][ 5][1]=-0.9525; //-5.08;
  relative_point[4][ 5][2]= 3.175;
  relative_point[4][ 5][3]= 0.0;

  relative_point[4][ 6][1]=-0.9525; //5.08;
  relative_point[4][ 6][2]=-3.175;
  relative_point[4][ 6][3]= 0.0;

  // two newly added points for link axis
  relative_point[4][ 7][1]= 3.175; 
  relative_point[4][ 7][2]= 0.0;
  relative_point[4][ 7][3]= 7.62;

  relative_point[4][ 8][1]= 3.175; 
  relative_point[4][ 8][2]= 0.0;
  relative_point[4][ 8][3]= -7.62;

  /*************************/
  /* Tool Roll :           */
  /*************************/
  relative_point[5][ 1][1]= 1.27; 
  relative_point[5][ 1][2]= 1.905;
  relative_point[5][ 1][3]=-8.255;

  relative_point[5][ 2][1]= 1.27;
  relative_point[5][ 2][2]=-1.905;
  relative_point[5][ 2][3]=-8.255;

  relative_point[5][ 3][1]=-1.27;
  relative_point[5][ 3][2]=-1.905;
  relative_point[5][ 3][3]=-8.255;

  relative_point[5][ 4][1]=-1.27;
  relative_point[5][ 4][2]= 1.905;
  relative_point[5][ 4][3]=-8.255;

  relative_point[5][ 5][1]= 0.0;
  relative_point[5][ 5][2]= 0.0;
  relative_point[5][ 5][3]=-19.3675;

  relative_point[5][ 6][1]= 0.0;
  relative_point[5][ 6][2]= 0.0;
  relative_point[5][ 6][3]=-8.255;

  /* Fill the last entry of homogeneous vector coordinates. */
  for (i=0;i<dof+2;i++)
    for (j=1;j<=max_point[i];j++)
      relative_point[i][j][4]=absolute_point[i][j][4]=1.0;
} // Rhino::initialize_points

/***************************************************************************/
/* Compute end device points; since movement of the end device may be more */
/* complicated than a pure rotation or translation we have provided a      */
/* separate routine to describe the end device motion.Describe end device  */
/* points relative to tool frame in terms of tool_variable(tv).            */
/***************************************************************************/
void Rhino::compute_end_device (float tv)
{
  float C, S;

  tv = M_PI_2 - tv/2.0;
  C = cos (tv);
  S = sin (tv);

  relative_point[6][ 1][1]= 1.27;
  relative_point[6][ 1][2]= 1.905;
  relative_point[6][ 1][3]= -8.255;

  relative_point[6][ 2][1]=-1.27;
  relative_point[6][ 2][2]= 1.905;
  relative_point[6][ 2][3]=-8.255;

  relative_point[6][ 3][1]=-1.27;
  relative_point[6][ 3][2]= 1.905+3.81*C;
  relative_point[6][ 3][3]= 3.81*S;

  relative_point[6][ 4][1]= 1.27;
  relative_point[6][ 4][2]= 1.905+3.81*C;
  relative_point[6][ 4][3]= 3.81*S;

  relative_point[6][ 5][1]= 1.27;
  relative_point[6][ 5][2]=-1.905; 
  relative_point[6][ 5][3]=-8.255;

  relative_point[6][ 6][1]=-1.27;
  relative_point[6][ 6][2]=-1.905;
  relative_point[6][ 6][3]=-8.255;

  relative_point[6][ 7][1]=-1.27;
  relative_point[6][ 7][2]= -relative_point[6][4][2]; 
  relative_point[6][ 7][3]= relative_point[6][4][3];

  relative_point[6][ 8][1]= 1.27;
  relative_point[6][ 8][2]= -relative_point[6][4][2];
  relative_point[6][ 8][3]= relative_point[6][4][3];

  relative_point[6][ 9][1]= 0;
  relative_point[6][ 9][2]= 0;
  relative_point[6][ 9][3]= 0;

  relative_point[6][10][1]= 0;
  relative_point[6][10][2]= 0;
  relative_point[6][10][3]=-8.48;
} // Rhino::compute_end_device

/***************************************************************************/
/* Update coordinate transformation matrices using joint vector.           */
/* These matrices may be found by any method and are not necessarily       */
/* obtained by Denavit-Hartenberg method. Anyway, for the case of Rhino    */
/* robots, they 'are' computed usinf D-H method!                           */                                 
/***************************************************************************/
void Rhino::update_maps (float *joint)
{
  register int i;

  for (i = 1; i < MaxLink-1; i++)
    {
      if (joint_status[i] == REVOLUTE)
        RHINO_DH_MATRIX[i][J_ANGLE] = joint[i];
      else
        RHINO_DH_MATRIX[i][J_DIST] = joint[i];
      D_H (map[i], RHINO_DH_MATRIX[i]);
    } // for
} // Rhino::update_maps

/***************************************************************************/
/* THis procedure accepts the joint parameters of a joint, and computes    */
/* the D-H matrix of that joint.                                           */
/***************************************************************************/
void D_H (frame Map, float *Vars)
{
  float Ct, Ca, St, Sa;

  Ct = cos (Vars[J_ANGLE]);
  St = sin (Vars[J_ANGLE]);
  Ca = cos (Vars[LINK_TW]);
  Sa = sin (Vars[LINK_TW]);
  Map[1][1] = Ct;       Map[1][2] = -Ca * St;
  Map[1][3] = Sa * St;  Map[1][4] = Ct * Vars[LINK_LEN];
  Map[2][1] = St;       Map[2][2] = Ca * Ct;
  Map[2][3] =-Sa * Ct; Map[2][4] = St * Vars[LINK_LEN];
  Map[3][1] = 0;        Map[3][2] = Sa;
  Map[3][3] = Ca;       Map[3][4] = Vars[J_DIST];
  Map[4][1] = 0;        Map[4][2] = 0;
  Map[4][3] = 0;        Map[4][4] = 1;
} // D_H

/***************************************************************************/
/* Not yet implemented.                                                    */
/***************************************************************************/
int Rhino::kinematics (frame t,float *joint)
{
  return 0;
} // Rhino::kinematics

/***************************************************************************/
/* Not yet implemented.                                                    */
/***************************************************************************/
int Rhino::inverse_kinematics (frame t,float *joint)
{
  return 0;

} // Rhino::inverse_kinematics
