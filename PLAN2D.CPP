
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
#include <io.h>
#include <process.h>
#include <stdlib.h>
#include <stdio.h>
#include "robot.h"
#include "winrob.h"

#define  XMAX        20
#define  YMAX        20
#define  ZMAX        20


extern char OutMap[3][XMAX+1][YMAX+1];

char   CollisionMap[3][XMAX+1][YMAX+1];

int    MaxX, MaxY;
int    File[MaxRobot], Map;
char   Path[XMAX+YMAX+ZMAX+3][3];
FILE   *Test, *OutF[MaxRobot];
int    XMax = 14, YMax = 13, ZMax = 14;

extern int Times, MaxTimes;
extern int ParNo;
extern int IfMessage;
extern int active_page;
extern char  mess[];
extern char param_part[];
extern char Route[][3];
extern char Route[XMAX+YMAX+ZMAX+3][3];
extern HWND		 HwndMap, HwndView, HwndPannel;
extern float parameter[11];

extern int  VRead (void);
extern int  CheckVols (int Rob0, int Rob1);
extern void Plan3D (void);
extern void FreeVol(int Rob);
extern void BackGround (void);
extern void Initialize3D (void);
extern void message (char *Text);
extern void Eliminate_Deadlocks (void);
extern void EnableAllButtons (BOOL Enable);
extern FILE *HeaderRead (void);


int  RecursiveFind2D (int i, int j, int Map);
char Check2D (int i, int j, int l, int Map);
void Show (void);
void Plan (void);
void Write3D (void);
void Plan2D (int Map);
void InitFiles3D (void);
void WriteWait (int RobNo);
void SW_Closure2D (int Map);
void MakeFiles3D (char *Route);
int  MakeMap (int Rob1, int Rob2);
void Write2D (int Rob0, int Rob1);
void DrawMaps (int Rob1, int Rob2);
void CopyFile (int RobNo, int FileNo);
void InitFiles2D (int Rob0, int Rob1);
void WriteFiles2D (int Rob0, int Rob1, char *Route);

/***************************************************************************/
// This function constructs the 2D-TC of the two input robots.
// First it initializes the corresponding arrays 'OutMap' and CollisionMap
// to all ZEROs. Then for each consecutive path-segment of the first robot
// (Rob1) its swept-volume is read from that segment's file. The intersection
// of this swept-volume with each of the second robot's path-segment
// swept-volumes is checked, reading the corresponding swept-volume of Rob2's
// path-segment. To check intersections, the function 'CheckVols' is called.
// If there is any intersection between two path-segments swept-volumes, the
// corresponding cell of 'OutMap' and 'CollisionMap' is set.  
// Rob1 and Rob2 determine which of the three 2D-TCs should be filled, and
// what is the dimensions of the final 3D-TC (for the case of three robots). 
/***************************************************************************/
/*
void MakeMap (int Rob1, int Rob2)     // First Version (Illustrative)
{
  register int i, j;

  IfMessage = 0;

  Map = Rob1+Rob2-1;

  for (i = 0; i < XMAX; i++)
    for (j = 0; j < YMAX; j++)
      OutMap[Map][i][j] = CollisionMap[Map][i][j] = 0;

  for(i = 0, j = 0;;i++)
    {
      ParNo = 1;
      sprintf (param_part, "rob%d-%d.vol)", Rob1, i);
      if (VRead ())
	      break;
      for (j = 0;;j++)
	      {
	        ParNo = 1;
	        sprintf (param_part, "rob%d-%d.vol)", Rob2, j);
	        if (VRead ()) 
	          break;
	        ParNo = 2;
	        if (CheckVols (Rob1, Rob2))
	          OutMap[Map][i][j] = CollisionMap[Map][i][j] = '\1';
	        FreeVol (Rob2);
	      } // for
      FreeVol (Rob1);
    } // for

  MaxX = i;
  MaxY = j;

  IfMessage = 1;

  switch (Map)  // Very Important
    {
      case 0 : XMax = MaxX; YMax = MaxY; break;  // Map1_2
      case 1 : XMax = MaxX; ZMax = MaxY; break;  // Map1_3
      case 2 : YMax = MaxX; ZMax = MaxY; break;  // Map2_3
    } // switch

} // MakeMap
*/
/***************************************************************************/
// This is a new version of the above MakeMap which solves the problem of
// the borders of the 2D-TC, and computes the SW-Closure of it in an on-line
// fashion, that is during the construction of the actual 2D-TC. If a cell
// of the 2D-TC is to be filled due to any kind of deadlock, then it is not
// necessary to check the intersection of the two corresponding
// swept-volumes. This way, the speed of computation of the final TC is
// improved.
/***************************************************************************/

int MakeMap (int Rob1, int Rob2)  // Second Version (Practical)
{
  register int i, j, k;

  IfMessage = 0;

  Map = Rob1+Rob2-1;

  for (i = 0; i <= XMAX; i++)  // <= was <
    for (j = 0; j <= YMAX; j++)  // <= was <
      OutMap[Map][i][j] = CollisionMap[Map][i][j] = 0;

  MaxX = MaxY = 0;
  for(i = XMAX-1; i >= 0; i--)
    {
      ParNo = 1;
      sprintf (param_part, "rob%d-%d.vol)", Rob1, i);
      if (VRead ())
        continue;
      else if (!MaxX)
        MaxX = i+1;        
      for (j = (MaxY) ? (MaxY - 1) : (YMAX - 1); j >= 0; j--)
	      {
          if (OutMap[Map][i+1][j+1] && OutMap[Map][i][j+1] &&
              OutMap[Map][i+1][j])
            {
              OutMap[Map][i][j] = CollisionMap[Map][i][j] = 1;
              continue;
            }
	        ParNo = 1;
	        sprintf (param_part, "rob%d-%d.vol)", Rob2, j);
	        if (VRead ()) 
            continue;
          else if (!MaxY)
            MaxY = j+1;
	        ParNo = 2;
	        if (CheckVols (Rob1, Rob2))
            {
	            OutMap[Map][i][j] = CollisionMap[Map][i][j] = '\1';
              if (i == MaxX - 1)
                for (k = j - 1; k >= 0; k--)
	                OutMap[Map][MaxX][k] = CollisionMap[Map][MaxX][k] = '\1';
              if (j == MaxY - 1)
                for (k = i - 1; k >= 0; k--)
	                OutMap[Map][k][MaxY] = CollisionMap[Map][k][MaxY] = '\1';
            } // if
	        FreeVol (Rob2);
	      } // for
      FreeVol (Rob1);
    } // for

  if (OutMap[Map][0][0] && OutMap[Map][MaxX][MaxY])
    return 1;
  IfMessage = 1;

  switch (Map)  // Very Important
    {
      case 0 : XMax = MaxX; YMax = MaxY; break;  // Map1_2
      case 1 : XMax = MaxX; ZMax = MaxY; break;  // Map1_3
      case 2 : YMax = MaxX; ZMax = MaxY; break;  // Map2_3
    } // switch
  return 0;

} // MakeMap

/***************************************************************************/
//                      The following are 2D functions                     
/***************************************************************************/

/***************************************************************************/
// This routine computes the 2D SW_Closure of a 2D-TC which is identified
// by 'Map'. Before computing the SW_Closure, the 2D-TC is augmented with an
// extra row and an extra column. These extra row and columns are filled so
// that any deadlocks which are due to collisions in the last path-segment of
// one robot and a middle path-segment of the other robot are prevented.
// If the second version of 'MakeMap' is used, this procedure need not be
// called, as the SW-Closure of the TC is already computed. 
/***************************************************************************/
/*
void SW_Closure2D (int Map)
{
  register int i, j, l;

  for (i = MaxX - 1; i >= 0; i--)
    if (OutMap[Map][i][MaxY-1])
      {
        for (l = i; l >= 0; l--)
          OutMap[Map][l][MaxY] = 1;
        break;
      } // if
  for (j = MaxY - 1; j >= 0; j--)
    if (OutMap[Map][MaxX-1][j])
      {
        for (l = j; l >= 0; l--)
          OutMap[Map][MaxX][l] = 1;
        break;
      } // if

  for (i = MaxX-1; i >=0; i--)
	  for (j = MaxY-1; j >=0 ; j--)
	    if ((OutMap[Map][i][j+1] == 1) && (OutMap[Map][i+1][j] == 1) &&
	        (OutMap[Map][i+1][j+1] == 1))
	      OutMap[Map][i][j] = 1;
} // SW_Closure2D
*/
/***************************************************************************/
// This procedure finds a path in MapI_J (OutMap[Map]) by first initializing 
// 'Path' and 'Route' arrays to all ZEROs and then calling RecursiveFind2D
// for the starting cell of the path, i.e. MapI_J(0,0).                                     
// Each column j of 'Path' indicates whether the jth edge of the cell is   
// a feasible edge; that is if the jth robot can move.                                                        
// Each row of 'Route' indicates the next feasible moves for each of the
// two robots; a '1' means move, and a '0' means no-move.
/***************************************************************************/
void Plan2D (int Map)
{
  register int i, l;

  for (i = 0; i < XMax+YMax; i++)
    for (l = 0; l < 2; l++)
      {
	      Path[i][l] = 0;
	      Route[i][l] = 0;
      } // for

  Times = -1;
  if (!RecursiveFind2D (0, 0, Map) || !MaxTimes)
    message ("Can not Plan a Collision-Free Path For The Two Robots");
} // Plan2D

/***************************************************************************/
// This recursive function is called to find the next move from vertex
// (i,j) in MapI_J (OutMap[Map]). First, it checks whether the cell        
// (i,j) is empty, and if so it recursively calls RecursiveFind2D for       
// (i+1,j+1). Otherwise, it finds the 1st-order feasible edges of    
// vertex (i,j) by calling 'Check2D'; then using that information, it       
// searches for a feasible edge recursively, and advances to the
// corresponding vertex of MapI_J.                                  
/***************************************************************************/
int RecursiveFind2D (int i, int j, int Map)
{
  Times++;
  if ((i == XMax) && (j == YMax))
    {
      MaxTimes = Times;
      Times--;
      return 1;
    } // if

	if (!OutMap[Map][i][j] && i < XMax && j < YMax)
    if (RecursiveFind2D (i+1, j+1, Map))
      {
        Route[Times][0] = 1;
	      Route[Times][1] = 1;
        Times--;
        return 1;
      }
	for (int l = 0; l < 2; l++)
		Path[Times][l] = Check2D (i, j, l, Map);

	if (Path[Times][0] && i < XMax)
    if (RecursiveFind2D (i+1, j, Map))
		  {
		    Route[Times][0] = 1;
		    Route[Times][1] = 0;
        Times--;
        return 1;
		  } // else
	if (Path[Times][1] && j < YMax)
    if (RecursiveFind2D (i, j+1, Map))
		  {
		    Route[Times][0] = 0;
		    Route[Times][1] = 1;
        Times--;
        return 1;
  		} // else
	else
		{
		  sprintf (mess, "DeadLock : i = %d, j = %d", i, j);
		  message (mess);
		  return 0;
		} // if
} // RecursiveFind2D

/***************************************************************************/
// This function checks whether edge 'l' of cell (i,j) is a feasible edge.  
/***************************************************************************/
char Check2D (int i, int j, int l, int Map)
{
  switch (l)
    {
      case 0 : if (!OutMap[Map][i][j] || !OutMap[Map][i][j-1])
		             return 1;
	             else
		             return 0;
      case 1 : if (!OutMap[Map][i][j] || !OutMap[Map][i-1][j])
		             return 1;
	             else
		             return 0;
    } // switch

} // Check2D

/***************************************************************************/
// This procedure creates the two output files of the scheduler.
// It first opens the files, then calls InitFiles2D to Initialize them,
// then using the array 'Route', which is the output of the scheduler,
// it appends consecutive path-segments of each robot to eachother, using
// 'sync' commands to synchronize the robots. Finally the files are closed.  
/***************************************************************************/
void Write2D (int Rob0, int Rob1)
{
  register int i, j;
  char     RobotFile[13];

  for (j = 0, i = Rob0; j < 2; i = Rob1, j++)
    {
      sprintf (RobotFile, "robot%d.dat", i);
      OutF[i] = fopen (RobotFile, "wt");
    } // for
  InitFiles2D (Rob0, Rob1);
  for (i = 0; i < MaxTimes; i++)
    WriteFiles2D (Rob0, Rob1, Route[i]);
  for (j = 0, i = Rob0; j < 2; i = Rob1, j++)
    {
      fprintf (OutF[i], "sync(%d)\n", i);
      fclose (OutF[i]);
    } // for
} // Write2D

/***************************************************************************/
// This procedure, using the array 'Route', reflects the next step of
// scheduling in the two output files. If the corresponding 'Route' entry
// of a robot is 0, then the robot has to wait, that is a 'sync' command
// is written in its output file. Otherwise this robot can move to its next
// path-segment, that is the file corresponding to this path-segment is
// appended to this robot's output file by calling 'CopyFile'.
/***************************************************************************/
void WriteFiles2D (int Rob0, int Rob1, char *Route)
{
  if (Route[0] == 1)
    {
      CopyFile (Rob0, File[Rob0]);
      File[Rob0]++;
    } // if
  else
    WriteWait (Rob0);
  if (Route[1] == 1)
    {
      CopyFile (Rob1, File[Rob1]);
      File[Rob1]++;
    } // if
  else
    WriteWait (Rob1);
} // WriteFiles2D

/***************************************************************************/
// This procedure initializes the two output files of the scheduler.
// The first command of each file moves the corresponding robot to the
// initial point of its path.
/***************************************************************************/
void InitFiles2D (int Rob0, int Rob1)
{

  FILE *In;
  char FileName[13];
  char Line[MAXCHAR];
  register int i;
  int k;

  for (k = 0, i = Rob0; k < 2; i = Rob1, k++)
    {
      File[i] = 0;
      sprintf (FileName, "rob%d-0.pat", i);
      In = fopen (FileName, "rt");
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
} // InitFiles2D

/***************************************************************************/
// The following are common functions and procedures for 2D and 3D cases.
/***************************************************************************/

/***************************************************************************/
// This procedure reads the path-segment file 'FileNo' of robot 'RobNo' and
// appends it to the end of that robot's output file. Before doing this, it
// adds a 'sync' command to the output file, so that robots are synchronized
// at the beginning of each of their path-segmetns.
/***************************************************************************/
void CopyFile (int RobNo, int FileNo)
{
  FILE *In;
  char FileName[13];
  char Line[MAXCHAR];

  sprintf (FileName, "rob%d-%d.pat", RobNo, FileNo);
  if ((In = fopen (FileName, "rt")) == NULL)
    {
      sprintf (Line, "Warning : File %s Can not be Opened", FileName);
      message (Line);
      return;
    } // if
  fprintf (OutF[RobNo], "sync(%d)\n", RobNo);
  while (!feof(In))
    {
      if (fgets (Line, MAXCHAR, In) != NULL)
	      fprintf (OutF[RobNo], "%s", Line);
    } // while
  fclose (In);
} // CopyFile

/***************************************************************************/
// This procedure writes a 'sync' command into the output file of robot
// 'RobNo'.
/***************************************************************************/
void WriteWait (int RobNo)
{
  fprintf (OutF[RobNo], "sync(%d)\n", RobNo);
} // WriteWait

/***************************************************************************/
// This procedure schedules the collision-free and deadlock-free movements
// of two or three robots. First it makes the required 2D-TC's by calling
// 'MakeMap'. For the case of two robots, it then computes the SW-Closure
// of the 2D-TC and uses it to find a safe schedule by calling 'Plan2D'.
// For the case of three robots, it first constructs the 3D-TC from the
// 2D-TCs by calling 'Initialize3D' and then 'EliminateDeadlocks', then
// finds a safe schedule by calling 'Plan3D'. In any case, the result of
// scheduling is reflected in output files of the robots. 
/***************************************************************************/
void Plan (void)
{
  register int i, j, k;
  int      Flag = 0;


  MaxTimes = 0;
  if (MaxRobot == 2)
    {
      if (MakeMap (0, 1))
        {
          message ("Can not Plan a Collision-Free Path For The Two Robots");
          return;
        } // if
//    SW_Closure2D (0);  // Not needed if the 2nd version of MakeMap is used
      DrawMaps (0, 1);
      Plan2D (0);
      Write2D (0, 1);
      return;
    } // if
  else
    {
      for (i = 0; i < XMAX; i++)
        for (j = 0; j < YMAX; j++)
          for (k = 0; k < 3; k++)
            OutMap[k][i][j] = CollisionMap[k][i][j] = '\0';

      for (i = 0; i < MaxRobot; i++)
	      {
	        if (i == MaxRobot-1)
            {
              Flag = MakeMap (i, 0);
              DrawMaps (i, 0);
            } // if
	        else
            {
              Flag = MakeMap (i, i+1);
              DrawMaps (i, i+1);
            } // else
          if (Flag)
            {
              message ("Can not Plan a Collision-Free Path For The Three Robots");
              return;
            } // if

	      } // for

      Initialize3D ();
      Eliminate_Deadlocks ();
      Plan3D ();
      Write3D ();
    } // else
} // Plan

/***************************************************************************/
// This procedure draws the 2D-TC of two robots (Rob1 and Rob2) both with
// and without its SW-Closure. To return to the normal screen the 'OK' key
// on the pannel must be pressed.
/***************************************************************************/
void DrawMaps (int Rob1, int Rob2)
{
	register int i, j;
	char     str[40];
	RECT		 rc;
	HDC			 hdc;
	HBRUSH   hbr, hbrOld;
	HPEN		 hpen, hpenOld;

  EnableAllButtons (FALSE);
	HwndMap = HwndView;
	GetClientRect (HwndMap, &rc);
	InvalidateRect (HwndMap, &rc, TRUE);
	UpdateWindow (HwndMap);
	hdc = GetDC (HwndMap);
	hpen = CreatePen (PS_SOLID, 0, RGB (255, 0, 0));
	hpenOld = SelectObject (hdc, hpen);
	hbr = CreateSolidBrush (RGB(255, 0, 0));
	hbrOld = SelectObject (hdc, hbr);
  sprintf (str, "Original Collision Map of Rob%d and Rob%d", Rob1, Rob2);
	TextOut (hdc, 20, 100, str, strlen (str));
  if (MaxRobot == 2)
    {
      sprintf (str, "South West Closure Map of Rob%d and Rob%d", Rob1, Rob2);
	    TextOut (hdc, 250, 100, str, strlen (str));
    } // if
  for (i = 0; i < MaxX; i++)
    {
      for (j = 0; j < MaxY; j++)
	      {
					if (CollisionMap[Map][i][j]=='\1')
						Rectangle (hdc, i*10+10, j*10+140, (i+1)*10+10, (j+1)*10+140);
					if ((MaxRobot == 2) && (OutMap[Map][i][j] =='\1'))
						Rectangle (hdc, i*10+240, j*10+140, (i+1)*10+240, (j+1)*10+140);
	      } // for
    } // for
  for (j = 0; j <= MaxY; j++)
		{
			MoveTo (hdc, 10, j*10+140);
			LineTo (hdc, MaxX*10+10, j*10+140);
      if (MaxRobot == 2)
        {
			    MoveTo (hdc, 240, j*10+140);
			    LineTo (hdc, MaxX*10+240, j*10+140);
        } // if
		} // for
	for (i = 0; i <= MaxX; i++)
		{
			MoveTo (hdc, i*10+10, 140);
			LineTo (hdc, i*10+10, MaxY*10+140);
      if (MaxRobot == 2)
        {
			    MoveTo (hdc, i*10+240, 140);
			    LineTo (hdc, i*10+240, MaxY*10+140);
        } // if  
    } // for
	SelectObject (hdc, hbrOld);
  DeleteObject (hbr);
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
} // DrawMaps
