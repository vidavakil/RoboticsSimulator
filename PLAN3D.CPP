
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
#include <conio.h>
#include <dos.h>
#include <process.h>
#include <stdlib.h>
#include <stdio.h>
#include "robot.h"

#define  E1       0
#define  E2       1
#define  E3       2
#define  E12      3
#define  E23      4
#define  E13      5
#define  E123     6


int  Times, MaxTimes;
char TDOutMap[XMAX+1][YMAX+1][ZMAX+1][7], OutMap[3][XMAX+1][YMAX+1];
char Route[XMAX+YMAX+ZMAX+3][3];

extern char  mess[];
extern int   File[MaxRobot];
extern FILE  *OutF[MaxRobot];
extern int   XMax, YMax, ZMax;
extern void  message (char *Text);
extern void  WriteWait (int RobNo);
extern void  CopyFile (int RobNo, int FileNo);

int  NoFanOut (int i, int j, int k);
int  RecursiveFind3D (int i, int j, int k);
void Plan3D (void);
void Write3D (void);
void InitFiles3D (void);
void Initialize3D (void);
void Eliminate_Deadlocks (void);
void WriteFiles3D (char *Route);
void Mark (int i, int j, int k);

/***************************************************************************/
//  This function constructs the 3D-TC (TDOutMap) by means of the 2D-Tc's
//  (OutMap). TDOutMap is a 3D array, each of its components is an array
//  with seven elements corresponding to the seven fan-out edges of a node
//  in a 3D TC-Graph.
//
//  1- TDOutMap is initialized to all ZEROs.
//  2- Each of the 2D-TC's are completed by first augmenting them
//     (manipuling the last row and columns of them to remove deadlocks in
//      the finall path-segments), and then finding their SW-Closure (to
//      remove any deadlocks between pairs of robots).
//  3- The completed 2D-TC's are copied to the 3D-TC. If any cell of a
//     2D-TC is filled then the corresponding 2nd-order edges of the 3D-TC
//     are all non-feasible. If any two adjacent cells in a 2D-TC are filled,
//     then their commomn edge which corresponds to a single robot's
//     movement (or equally a 1st-order edge in the 3D-TC) is also
//     non-feasible, then any combination of the three robots' movements
//     which includes this robot's movement is also non-feasible.
//  4- By now, 1st-order and 2nd-order edges of the 3D TC-Graph are computed.
//     3rd-order edges are non-feasible iff any 2nd-order edge is non-feasible
//  If the second version of 'MakeMap' is used, which computes the
//  SW-Closure of the 2D-TCs in an on-line fashion, then step 2 is not
//  needed.  
/***************************************************************************/
void Initialize3D (void)
{
  register int i, j, k, l;

/*
  XMax = YMax = ZMax = 10;
  for (i = 0; i <= XMax; i++)
    for (j = 0; j <= YMax; j++)
      OutMap[0][i][j] = 0;
  for (i = 0; i <= XMax; i++)
    for (k = 0; k <= ZMax; k++)
      OutMap[1][i][k] = 0;
  for (j = 0; j <= YMax; j++)
    for (k = 0; k <= ZMax; k++)
      OutMap[0][j][k] = 1;
*/

/*   // Illustrative  

  // Initialize the 3D Map1_2_3 with all zeros, TDOutMap is Map1_2_3
  for (i = 0; i <= XMax; i++) 
    for (j = 0; j <= YMax; j++)
      for (k = 0; k <= ZMax; k++)
        for (l = 0; l < 7; l++)
	        TDOutMap[i][j][k][l] = 0;


  // Augment Map1_2
  for (i = XMax - 1; i >= 0; i--)
    if (OutMap[0][i][YMax-1])
      {
        for (l = i; l >= 0; l--)
          OutMap[0][l][YMax] = 1;  
        break;
      } // if
  for (j = YMax - 1; j >= 0; j--)
    if (OutMap[0][XMax-1][j])
      {
        for (l = j; l >= 0; l--)
          OutMap[0][XMax][l] = 1;
        break;
      } // if

  // Compute SW-closure of Map1_2
  for (i = XMax-1; i >= 0; i--)  
    for (j = YMax-1; j >= 0; j--)
      if (!OutMap[0][i][j] && OutMap[0][i+1][j] && OutMap[0][i][j+1] &&
           OutMap[0][i+1][j+1])
        OutMap[0][i][j] = 1;

  // Augment Map1_3
  for (k = ZMax - 1; k >= 0; k--)
    if (OutMap[1][XMax-1][k])
      {
        for (l = k; l >= 0; l--)
          OutMap[1][XMax][l] = 1;
        break;
      } // if
  for (i = XMax - 1; i >= 0; i--)
    if (OutMap[1][i][ZMax-1])
      {
        for (l = i; l >= 0; l--)
          OutMap[1][l][ZMax] = 1;
        break;
      } // if

  // Compute SW-closure of Map1_3
  for (i = XMax-1; i >= 0; i--)
    for (k = ZMax-1; k >= 0; k--)
      if (!OutMap[1][i][k] && OutMap[1][i+1][k] && OutMap[1][i][k+1] &&
           OutMap[1][i+1][k+1])
        OutMap[1][i][k] = 1;

  // Augment Map2_3
  for (k = ZMax - 1; k >= 0; k--)
    if (OutMap[2][YMax-1][k])
      {
        for (l = k; l >= 0; l--)
          OutMap[2][YMax][l] = 1;
        break;
      } // if
  for (j = YMax - 1; j >= 0; j--)
    if (OutMap[2][j][ZMax-1])
      {
        for (l = j; l >= 0; l--)
          OutMap[2][l][ZMax] = 1;
        break;
      } // if

  // Compute SW-closure of Map2_3
  for (j = YMax-1; j >= 0; j--)
    for (k = ZMax-1; k >= 0; k--)
      if (!OutMap[2][j][k] && OutMap[2][j+1][k] && OutMap[2][j][k+1] &&
           OutMap[2][j+1][k+1])
        OutMap[2][j][k] = 1;

*/


  // Copy Map1_2 in Map1_2_3
  for (k = 0; k <= ZMax; k++) 
    for (i = 0; i <= XMax; i++)  
      for (j = 0; j <= YMax; j++)
	      if (OutMap[0][i][j])
          {
            TDOutMap[i][j][k][E12] = 1;
            if (!i || OutMap[0][i-1][j])
              {
                TDOutMap[i][j][k][E2] = 1;
                TDOutMap[i][j][k][E23] = 1;
              } // if
            if (!j || OutMap[0][i][j-1])
              {
                TDOutMap[i][j][k][E1] = 1;
                TDOutMap[i][j][k][E13] = 1;
              } // if
          } // if

  // Copy Map1_3 in Map1_2_3
  for (j = 0; j <= YMax; j++) 
    for (i = 0; i <= XMax; i++)
      for (k = 0; k <= ZMax; k++)
        if (OutMap[1][i][k])
          {
            TDOutMap[i][j][k][E13] = 1;
            if (!i || OutMap[1][i-1][k])
              {
                TDOutMap[i][j][k][E3] = 1;
                TDOutMap[i][j][k][E23] = 1;
              } // if
            if (!k || OutMap[1][i][k-1])
              {
                TDOutMap[i][j][k][E1] = 1;
                TDOutMap[i][j][k][E12] = 1;
              } // if
          } // if

  // Copy Map2_3 in Map1_2_3
  for (i = 0; i <= XMax; i++) 
    for (j = 0; j <= YMax; j++)
      for (k = 0; k <= ZMax; k++)  
	      if (OutMap[2][j][k])
          {
	          TDOutMap[i][j][k][E23] = 1;
	          if (!j || OutMap[2][j-1][k])
              {
	              TDOutMap[i][j][k][E3] = 1;
                TDOutMap[i][j][k][E13] = 1;
              } // if
	          if (!k || OutMap[2][j][k-1])
              {
	              TDOutMap[i][j][k][E2] = 1;
                TDOutMap[i][j][k][E12] = 1;
              } // if
          } // if

  // Compute 3rd-order fan-out edges of the 3D TC-Graph
  for (i = 0; i <= XMax; i++) 
    for (j = 0; j <= YMax; j++)
      for (k = 0; k <= ZMax; k++)
        if (TDOutMap[i][j][k][E12] || TDOutMap[i][j][k][E23] ||
            TDOutMap[i][j][k][E13])
          TDOutMap[i][j][k][E123] = 1;

} // Initialize3D

/***************************************************************************/
// This function checks whether the intended node of the 3D TC-Graph has
// no fan-out edges.  
/***************************************************************************/
int NoFanOut (int i, int j, int k)
{
  register int l;

  for (l = 0; l < 7; l++)
    if (!TDOutMap[i][j][k][l])
      break;
  if (l==7)
    return 1;
  else
    return 0;

} // NoFanOut

/***************************************************************************/
// This procedure marks as non-feasible every fan-in edge of the intended.
// node of the 3D TC_Graph 
/***************************************************************************/
void Mark (int i, int j, int k)
{
  register int l;

  if (i)
    TDOutMap[i-1][j][k][E1] = 1;
  if (j)
    TDOutMap[i][j-1][k][E2] = 1;
  if (k)
    TDOutMap[i][j][k-1][E3] = 1;
  if (i && j)
    TDOutMap[i-1][j-1][k][E12] = 1;
  if (i && k)
    TDOutMap[i-1][j][k-1][E13] = 1;
  if (j && k)
    TDOutMap[i][j-1][k-1][E23] = 1;
  if (i && j && k)
    TDOutMap[i-1][j-1][k-1][E123] = 1;

} // Mark

/***************************************************************************/
// This procedure eliminates any deadlock nodes/paths from the 3D TC_Graph. 
/***************************************************************************/
void Eliminate_Deadlocks (void)
{
  register int i, j, k;

  for (k = ZMax; k >= 0; k--)
    for (j = YMax; j >= 0; j--)
      for (i = XMax; i >= 0; i--)
	      if (NoFanOut (i, j, k))
          Mark (i, j, k);
} // Eliminate_Deadlocks

/***************************************************************************/
// This procedure finds a safe schedule in the 3D TC-Graph.
// It first checks to ensure that the set of fan-out edges of the source
// node, and the set of fan-in edges of the destination node are both
// non-empty; that is a safe schedule exists. Then it calls the function
// RecursiveFind3D, given the source node of the graph.
/***************************************************************************/
void Plan3D (void)
{
  register int i, l;

  for (i = 0; i < XMax+YMax+ZMax; i++)
    for (l = 0; l < 3; l++)
      Route[i][l] = 0;
 
  Times = -1;
  MaxTimes = 0;

  if (NoFanOut (0, 0, 0) ||
      (TDOutMap[XMax-1][YMax][ZMax][E1] &&
       TDOutMap[XMax][YMax-1][ZMax][E2] &&
       TDOutMap[XMax][YMax][ZMax-1][E3] &&
       TDOutMap[XMax-1][YMax-1][ZMax][E12] &&
       TDOutMap[XMax-1][YMax][ZMax-1][E13] &&
       TDOutMap[XMax][YMax-1][ZMax-1][E23] &&
       TDOutMap[XMax-1][YMax-1][ZMax-1][E123]) ||
      !RecursiveFind3D (0, 0, 0) || !MaxTimes)
    message ("Can not Plan a Collision-Free Path For The Three Robots");

} // Plan3D

/***************************************************************************/
// This function recursively finds the next feasible move in the 3D TC-Graph
// until the destination node is visited. The path through the graph is
// saved in the array 'Route', the number of scheduling steps is saved in
// 'MaxTimes' and the current step number is saved in 'Times'.
// In each visited node, the fan-out edges of the node are visited from the
// highest-order to the lowest-order, and the highest-order feasible node
// is chosen to move to the next node.
/***************************************************************************/
int RecursiveFind3D (int i, int j, int k)
{    

  Times++;
  if ((i == XMax) && (j == YMax) && (k == ZMax))
    {
      MaxTimes = Times;
      Times--;
      return 1;
    } // if

  if (!TDOutMap[i][j][k][E123] && i < XMax && j < YMax && k < ZMax) 
    if (RecursiveFind3D (i+1, j+1, k+1))
      {
        Route[Times][0] = 1;
	      Route[Times][1] = 1;
	      Route[Times][2] = 1;
        Times--;
        return 1;
      } // if

  if (!TDOutMap[i][j][k][E12] && i < XMax && j < YMax)
    if (RecursiveFind3D (i+1, j+1, k))
      {
        Route[Times][0] = 1;
	      Route[Times][1] = 1;
	      Route[Times][2] = 0;
        Times--;
        return 1;
      } // if

  if (!TDOutMap[i][j][k][E23] && j < YMax && k < ZMax)
    if (RecursiveFind3D (i, j+1, k+1))
      {
        Route[Times][0] = 0;
	      Route[Times][1] = 1;
	      Route[Times][2] = 1;
        Times--;
        return 1;
      } // if

  if (!TDOutMap[i][j][k][E13] && i < XMax && k < ZMax)
    if (RecursiveFind3D (i+1, j, k+1))
      {
        Route[Times][0] = 1;
	      Route[Times][1] = 0;
	      Route[Times][2] = 1;
        Times--;
        return 1;
      } // if

  if (!TDOutMap[i][j][k][E1] && i < XMax)
    if (RecursiveFind3D (i+1, j, k))
      {
        Route[Times][0] = 1;
	      Route[Times][1] = 0;
	      Route[Times][2] = 0;
        Times--;
        return 1;
      } // if

  if (!TDOutMap[i][j][k][E2] && j < YMax)
    if (RecursiveFind3D (i, j+1, k))
      {
        Route[Times][0] = 0;
	      Route[Times][1] = 1;
	      Route[Times][2] = 0;
        Times--;
        return 1;
      } // if

  if (!TDOutMap[i][j][k][E3] && k < ZMax)
    if (RecursiveFind3D (i, j, k+1))
      {
        Route[Times][0] = 0;
	      Route[Times][1] = 0;
	      Route[Times][2] = 1;
        Times--;
        return 1;
      } // if

	else
		{
		  sprintf (mess, "DeadLock : i = %d, j = %d, k = %d", i, j, k);
		  message (mess);
		  return 0;
		} // if
} // RecursiveFind3D


/***************************************************************************/
// This procedure initializes the three output files of the scheduler.
// The first command of each file moves the corresponding robot to the
// initial point of its path.
/***************************************************************************/
void InitFiles3D (void)
{

  FILE *In;
  char FileName[13];
  char Line[MAXCHAR];
  register int i;

  for (i = 0; i < MaxRobot; i++)
    {
      File[i] = 0;
      sprintf (FileName, "rob%d-0.pat", i);
      if ((In = fopen (FileName, "rt")) == NULL)
        continue;
      sprintf (FileName, "move(%d", i);
      do
	      {
	        fgets (Line, MAXCHAR, In);
	      } // do
      while (strncmp (Line, FileName, 6));
      sprintf (Line, "init");
      Line[4] = '(';
      fprintf (OutF[i], "%s", Line);
      fclose (In);
    } // for
} // InitFiles3D

/***************************************************************************/
// This procedure creates the three output files of the scheduler.
// It first opens the files, then calls InitFiles3D to Initialize them,
// then using the array 'Route', which is the output of the scheduler,
// it appends consecutive path-segments of each robot to eachother, using
// 'sync' commands to synchronize the robots. Finally the files are closed.  
/***************************************************************************/
void Write3D (void)
{
  register int i;
  char     RobotFile[13];

  for (i = 0; i < MaxRobot; i++)
    {
      sprintf (RobotFile, "robot%d.dat", i);
      OutF[i] = fopen (RobotFile, "wt");
    } // for
  InitFiles3D ();
  for (i = 0; i < MaxTimes; i++) //was i < Times
    WriteFiles3D (Route[i]);
  for (i = 0; i < MaxRobot; i++)
    {
      fprintf (OutF[i], "sync(%d)\n", i);
      fclose (OutF[i]);
    } // for
} // Write3D

/***************************************************************************/
// This procedure, using the array 'Route', reflects the next step of
// scheduling in the three output files. If the corresponding 'Route' entry
// of a robot is 0, then the robot has to wait, that is a 'sync' command
// is written in its output file. Otherwise this robot can move to
// its next path-segment, that is the file corresponding to this path-segment
// is appended to this robot's output file by calling 'CopyFile'.
/***************************************************************************/
void WriteFiles3D (char *Route)
{
  register int i;
  for (i = 0; i < MaxRobot; i++)
    {
      if (Route[i])
	      {
	        CopyFile (i, File[i]);
	        File[i]++;
	      } // if
      else
	      WriteWait (i);
    } // for
} // WriteFiles3D

