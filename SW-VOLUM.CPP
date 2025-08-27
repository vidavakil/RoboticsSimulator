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
#include <alloc.h>
#include <conio.h>
#include <dos.h>
#include <math.h>
#include <mem.h>
#include <search.h>
#include <stdlib.h>
#include <string.h>
#include <toolhelp.h>
#include "robot.h"
#include "winrob.h"
#include "sw-volum.h"


int            Coord[13];
//int            Planes[MaxPlane+1][7]; 
float          Planes[MaxPlane+1][23];
BoxType        Box[13];
PresentType    Presents[MaxRobot][MaxLink];
Voxel          SweptVolume[MaxRobot][MaxLink+1];

extern FILE    *Test;
extern char    mess[];
extern int     ParNo;
extern int     IfMessage;
extern int     active_page;
extern float   parameter[];
extern char    param_part[];
extern cRobot huge *Robot[];
extern MainWindow *MainWnd;
extern HWND		 HwndMap, HwndView, HwndPannel;

extern void BackGround (void);
extern void message (char *Text);
extern void EnableAllButtons (BOOL Enable);

int  VRead (void);
int  FreeVols (void);
void FreeVol(int Rob);
void VShow (int RobotNo);
void VUpdate (int RobotNo);
void VFinalize (int RobotNo);
void Exchange (Point *a, Point *b);
void Vinit (int RobotNo, int Status);
void VSave (int RobotNo, char *FileName);
void SaveStarts (int RobotNo, HFILE Out);
int  BoxSort( const void *a, const void *b);
int  VoxelIntersect (Voxel *Vx1, Voxel *Vx2);
int  CheckVols (int Rob0, int Rob1);
int  Sort_Function( const void *a, const void *b);
int  VoxelCollision (int Rob0, int Rob1, int L0, int L1);

/***************************************************************************/
// Finalize frees the memory allocated to the voxel structure of a link and
// resets all other corresponding varibles.
/***************************************************************************/
void Voxel::Finalize (void)
{
  if (IfInit && (Volume != NULL))
    {
      GlobalUnlock (hglb);
      GlobalFree (hglb);
    } // if
  IfInit = Copy = IfUpdate = 0;
  BBrush = 1;
  Volume = NULL;
  hglb = NULL;
} // Voxel::Finalize

/***************************************************************************/
// Initialize initilizes the Voxel structure for the corresponding link.
// To compute the swept volume of a link, this function is called twice.
// First, this function is called with Status=0. Now, the DCEL structure of
// of the link is initialized and then the bounding box of the link in its
// current position is computed. 'Copy' is set to 0, so that when 'Update'
// is called in consecutive positions of the link along the corresponding
// path-segment, the bounding box is updated to encompass the whole swept
// volume.
// Once the path-segment is executed, the bounding box of the whole swept
// volume is computed. Now this same function is called with Status=1, while
// the robot is put back on its initial position along its path-segment.
// This time, memory for the voxel structure with the computed dimensions is
// allocated, and the occupied voxels in the current position of the link
// (initial position along the path-segment) are marked by calling
// 'FillVolume'. Also 'Copy' is set to 1, so that when 'Update' is called
// in consecutive positions of the link along the path-segment (which is
// re-played) newly occupied voxels of the structure are also marked. 
/***************************************************************************/
int Voxel::Initialize (int RobotNo, int LkNo, int Status)
{

  point      *P;
  LinkType   *Link;
  register   unsigned int i, j;
  int        *connection, index, k;
  double     MinX = 1000,  MinY = 1000,  MinZ = 1000,
             MaxX = -1000, MaxY = -1000, MaxZ = -1000;

  RobNo = RobotNo;
  LNo = LkNo;
  IfInit = 1;
  IfUpdate = 1;

  if (!Status)
    {
      InitDCEL (&Robot[RobNo]->DcelP[LNo]);
      Copy = 0;
      Link = &Robot[RobNo]->LinkArray[LNo];
      for (k = 0; k < Link->NoPart; k++)
        for (i = 0; i < Link->Part[k].NoSurface; i++)
          {
            connection = Link->Part[k].Surfaces[i];
            for (j = 0; (index=connection[j])>0; j++)
              {
                P = Robot[RobNo]->absolute_point[LNo];
                if (P[index][1] > MaxX)
                  MaxX = P[index][1];
                if (P[index][1] < MinX)
                  MinX = P[index][1];
                if (P[index][2] > MaxY)
                  MaxY = P[index][2];
                if (P[index][2] < MinY)
                  MinY = P[index][2];
                if (P[index][3] > MaxZ)
                  MaxZ = P[index][3];
                if (P[index][3] < MinZ)
                  MinZ = P[index][3];
              } // for
          } // for
        Origin.X = (int) floor (MinX) - 3; 
        Origin.Y = (int) floor (MinY) - 3;
        Origin.Z = (int) floor (MinZ) - 3;
        Dim.X = (unsigned int) ceil (MaxX - Origin.X) + 4; 
        Dim.Y = (unsigned int) ceil (MaxY - Origin.Y) + 4;
        Dim.Z = (unsigned int) ceil (MaxZ - Origin.Z) + 4;
      } // if
    else
      {
        Size = Dim.X * Dim.Y * Dim.Z;
        if (((hglb = GlobalAlloc( GPTR | GMEM_DISCARDABLE, (DWORD)Size)) == NULL) ||
            ((Volume = (LPBYTE) GlobalLock (hglb)) == NULL) )

          {
            if (hglb != NULL)
              GlobalFree (hglb);
            hglb = NULL;
            IfInit = 0;
            return 1;
          } // if
        Select = SELECTOROF ((void FAR *) Volume);
        Copy = 1;
        BBrush = 1;
        FillVolume ();
//      Show ();
        Robot[RobNo]->Vox = 1;
      } // else
    return 0;
} // Voxel::Initialize

/***************************************************************************/
// Depending on the value of 'Copy', this function either updates the
// dimensions and origin of the bounding box which encompasses the swept
// volume of this link, or calls 'FillVolume' to mark the voxels which are
// occupied by the link in the current posiition of the link. 
/***************************************************************************/
int Voxel::Update (void)
{

  point       *P;
  LPBYTE      Temp;
  LinkType    *Link;
  Dimensions  NewDim;
  register    int i, j;
  Point       NewOrigin;
  unsigned long  l, Size;
  int         *connection, index, k;
  double      MinX = 1000,  MinY = 1000,  MinZ = 1000,
              MaxX = -1000, MaxY = -1000, MaxZ = -1000;

  if (!(IfInit && IfUpdate)) return 0;
  if (!Copy)
    {
      MaxX = Origin.X+Dim.X-4; 
      MaxY = Origin.Y+Dim.Y-4;
      MaxZ = Origin.Z+Dim.Z-4;
      MinX = Origin.X+3;
      MinY = Origin.Y+3;
      MinZ = Origin.Z+3;
      Link = &Robot[RobNo]->LinkArray[LNo];
      for (k = 0; k < Link->NoPart; k++)
        for (i = 0; i < Link->Part[k].NoSurface; i++)
          {
            connection = Link->Part[k].Surfaces[i];
            for (j = 0; (index=connection[j])>0; j++)
              {
                P = Robot[RobNo]->absolute_point[LNo];
                if (P[index][1] > MaxX)
                  MaxX = P[index][1];
                if (P[index][1] < MinX)
                  MinX = P[index][1];
                if (P[index][2] > MaxY)
                  MaxY = P[index][2];
                if (P[index][2] < MinY)
                  MinY = P[index][2];
                if (P[index][3] > MaxZ)
                  MaxZ = P[index][3];
                if (P[index][3] < MinZ)
                  MinZ = P[index][3];
              } // for
          } // for
        NewOrigin.X = (int) floor (MinX) - 3;
        NewOrigin.Y = (int) floor (MinY) - 3;
        NewOrigin.Z = (int) floor (MinZ) - 3;
        NewDim.X = (unsigned int) ceil (MaxX - NewOrigin.X) + 4; 
        NewDim.Y = (unsigned int) ceil (MaxY - NewOrigin.Y) + 4;
        NewDim.Z = (unsigned int) ceil (MaxZ - NewOrigin.Z) + 4;
        if ((NewDim.X != Dim.X) || (NewDim.Y != Dim.Y) || (NewDim.Z != Dim.Z)
           || (NewOrigin.X != Origin.X) || (NewOrigin.Y != Origin.Y) ||
              (NewOrigin.Z != Origin.Z))
          {
            Origin.X = NewOrigin.X;
            Origin.Y = NewOrigin.Y;
            Origin.Z = NewOrigin.Z;
            Dim.X = NewDim.X;
            Dim.Y = NewDim.Y;
            Dim.Z = NewDim.Z;
          } // if
      } // if
    else
      {
        // this code is needed for scanfill
        if ((int)BBrush+2*(int)MaxPlane >= 255)
          {
            BBrush = 1;
            for (l = 0, Size = Dim.X * Dim.Y * Dim.Z, Temp = Volume;
                 l < Size; l++, Temp++)
              if (*Temp)
                *Temp = 1;
          }
        else
          BBrush += MaxPlane;
        FillVolume ();
      } // esle
    return 0;
} // Voxel::Update

/***************************************************************************/
// 'FillVolume' marks all the voxels which are occupied in the current
// position of the link. It first sorts the vertices of the link with respect
// to their Z values. Then it scans the voxel layers from bottom to top.
// In each voxel layer, there are a couple of faces of the link which
// intersect with this layer. Any of such faces, which are called active
// faces, has at least two active edges at any time which intersect with the
// current voxel layer. The intersection polygon of each active face
// is computed by calling 'ProcessPlane'. If any vertex is visited in the
// current layer, then the list of active faces may change, that is some
// active faces will go inactive, and some others may go active when this
// vertex is visited. On the other hand, the active edges of a previously
// active face may change. 'CheckPlane' checks the faces adjacent to a
// visited vertex, and performs the necessary operations to update the list
// of active faces and their active edges.  
/***************************************************************************/
void Voxel::FillVolume (void)
{

  point    *Index;
  SORTType Sorted[MaxPoints+1];
  int      i, j, Begin, End, NextVertex, First, Vert1, k;

  BackGround ();

  for (i = 0; i <= Dcel.EdgeNo; i++)
    {
      Dcel.DcelTable[i][Visit] = NotVisited;
      Dcel.DcelTable[i][LastVertex] = 0;   
      Dcel.DcelTable[i][Date] = 0;         
    } // for
  for (i = 0; i <= Dcel.FaceNo; i++)
    {
      Planes[i][PVisit] = NotVisited; 
    } // for
  SortVertices (Sorted);
  Begin = floor (Sorted[1].Z);
  End = ceil (Sorted[Dcel.VertNo].Z);
/*
  if (Begin <= 0.0)
    {
      message ("Earth Collision");
      getch ();
      return;
    } // if
*/
  NextVertex = 1;
  for (i = Begin; i <= End; i++)
		{
      Height = i;
      for (j = 1; j <= Dcel.FaceNo; j++)
	if (Planes[j][PVisit] == Visiting)  
          {
						ProcessPlane (j);
          } // if
      Index = Robot[RobNo]->absolute_point[LNo];
      while ((NextVertex <= Dcel.VertNo) &&
             (Index[Sorted[NextVertex].VertNo][3] <= i+1))
        {
          k = First = FindFirstEdge (Sorted[NextVertex].VertNo);
          Vert1 = Sorted[NextVertex].VertNo;
          do
            {
              if (Dcel.DcelTable[k][V1] == Vert1)
                {
                  CheckPlane (Vert1, Dcel.DcelTable[k][F2], k,
															Dcel.DcelTable[k][P1]);
                  k = Dcel.DcelTable[k][P1];
                } // if
              else
                {
                  CheckPlane (Vert1, Dcel.DcelTable[k][F1], k,
															Dcel.DcelTable[k][P2]);
                  k = Dcel.DcelTable[k][P2];
                } // else
            } // do
          while (k != First);
          NextVertex++;
        } // while
    } // for
} // Voxel::FillVolume

/***************************************************************************/
// 'CheckPlane' updates the activity status of the face 'Plane', a vertex
// of which 'Vert' has just been visited. E1 and E2 are the edges of 'Plane'
// intersecting at 'Vert'. If it is the first time that any of the edges,
// E1 or E2, are visited, then that edge has just become active. Otherwise,
// this edge has just become inactive, as its second vertex has been
// visited. If none of E1 or E2 are active edges anymore, then so is 'Plane'. 
// If both of E1 and E2 are active, then so has become 'Plane'.
// If only one of E1 or E2 is active, then the active one has just replaced
// a previously active edge of 'Plane'. The two active edges of a face
// intersect with the current voxel layer in four points which define a
// trapezoid. Finally 'ProcessPlane' is called to fill the intersection
// trapezoid, only if 'Plane' is active.
/***************************************************************************/
void Voxel::CheckPlane (int Vert, int Plane, int E1, int E2)
{
  register int E, P;
  int      E1V1, E1V2, E2V1, E2V2, Pp;

	Brush = Plane+BBrush;
	if (Dcel.DcelTable[E1][LastVertex] == NotVisited)
    {
      if (Dcel.DcelTable[E1][V1] == Vert)
        {
          E1V2 = Dcel.DcelTable[E1][LastVertex] = Dcel.DcelTable[E1][V2];
          E1V1 = Dcel.DcelTable[E1][V1];
        } // if
      else
        {
          E1V2 = Dcel.DcelTable[E1][LastVertex] = Dcel.DcelTable[E1][V1];
          E1V1 = Dcel.DcelTable[E1][V2];
        } // else
      Dcel.DcelTable[E1][Visit] = Visiting;
    } // if
  else if (Dcel.DcelTable[E1][LastVertex] == Vert)
    Dcel.DcelTable[E1][Visit] = Visited;
  else
    {
      E1V1 = Vert;
      E1V2 = Dcel.DcelTable[E1][LastVertex];
    } // else
  if (Dcel.DcelTable[E2][LastVertex] == NotVisited) 
    {
      if (Dcel.DcelTable[E2][V1] == Vert)
        {
          E2V2 = Dcel.DcelTable[E2][LastVertex] = Dcel.DcelTable[E2][V2];
          E2V1 = Dcel.DcelTable[E2][V1];
        } // if
      else
        {
          E2V2 = Dcel.DcelTable[E2][LastVertex] = Dcel.DcelTable[E2][V1];
          E2V1 = Dcel.DcelTable[E2][V2];
        } // else
      Dcel.DcelTable[E2][Visit] = Visiting;
    } // if
  else if (Dcel.DcelTable[E2][LastVertex] == Vert)
    Dcel.DcelTable[E2][Visit] = Visited;
  else
    {
      E2V1 = Vert;
      E2V2 = Dcel.DcelTable[E2][LastVertex];
    } // else

  if ((Dcel.DcelTable[E1][Visit] == Visited) &&
      (Dcel.DcelTable[E2][Visit] == Visited))
    Planes[Plane][PVisit] = Visited;  
  else if ((Dcel.DcelTable[E1][Visit] == Visiting) &&
          (Dcel.DcelTable[E2][Visit] == Visiting))
    {
      Planes[Plane][e1] = E1;  
      Planes[Plane][e2] = E2;  
      Planes[Plane][e1V1] = E1V1;
      Planes[Plane][e1V2] = E1V2;
      Planes[Plane][e2V1] = E2V1;
      Planes[Plane][e2V2] = E2V2;
      Planes[Plane][PVisit] = Visiting; 
    } // if
  else if ((Planes[Plane][PVisit] != Visited) &&
	   !((Dcel.DcelTable[(int)Planes[Plane][e1]][Visit] == Visiting) &&
	    (Dcel.DcelTable[(int)Planes[Plane][e2]][Visit] == Visiting)))
    {
      E = (Dcel.DcelTable[E1][Visit] == Visiting) ? E1 : E2;
      P = (Dcel.DcelTable[(int)Planes[Plane][e1]][Visit] == Visited) ? e1 : e2;
/* debug
      Pp = (P == e1) ? e2 : e1;
      if (Planes[Plane][Pp] == E)
	message ("Bad Edge Found");

*/
      Planes[Plane][P] = E;
      if (E == E1)
        {
          Planes[Plane][P+1] = E1V1;
          Planes[Plane][P+2] = E1V2;
        } // if
      else
        {
          Planes[Plane][P+1] = E2V1;
          Planes[Plane][P+2] = E2V2;
        } // else
      Planes[Plane][PVisit] = Visiting;
//    PreprocessPlane (Plane);
    } // else
	ProcessPlane (Plane);
} // Voxel::CheckPlane

/***************************************************************************/
// 'Address' computes the address of the given voxel 'P' within the
// corresponding voxel structure. 
/***************************************************************************/
DWORD Voxel::Address (Point P)
{
  DWORD Temp;
  Temp = (DWORD)((floor (P.X)) * Dim.Y * Dim.Z +
								 (DWORD)(floor(P.Y)) * Dim.Z +
                 (DWORD)(floor (P.Z)));
  return (Temp);
} // Voxel::Address

/***************************************************************************/
// 'SetVoxel' sets the voxel with dimensions 'X' and 'Y' in the current
// voxel layer to the value given in 'Brush'. Since the voxel structure
// can be larger that 64k, special functions must be used to write to an
// element of it.  
/***************************************************************************/
void Voxel::SetVoxel (double X, double Y)
{

  Point P;
  DWORD Add;

  P.X = (X - Origin.X); // Always Positive
  P.Y = (Y - Origin.Y);
  P.Z = (Height - Origin.Z);
/* debug
if (P.X < 0 || P.X >= Dim.X || P.Y < 0 || P.Y >= Dim.Y ||
    P.Z < 0 || P.Z >= Dim.Z)
    {
//      message ("Range Error in SetVoxel");
      return;
    } 
*/
  Add = (DWORD)((floor (P.X)) * Dim.Y * Dim.Z + (DWORD)(floor(P.Y)) * Dim.Z +
                                                (DWORD)(floor (P.Z)));
	if (Add < 0xffff)  
		Volume[Add] = Brush;
  else
    MemoryWrite (Select, Add, (void FAR *)&Brush, 1l);
} // Voxel::SetVoxel

/***************************************************************************/
// 'GetVoxel' returns the value of a voxel in the current layer with
// dimensions 'X' and 'Y'. Again, special functions must be called to read
// the elements of the voxel structure.
/***************************************************************************/
char Voxel::GetVoxel (double X, double Y)
{

  Point P;
  DWORD Add;
  char Color;

  P.X = (X - Origin.X);
  P.Y = (Y - Origin.Y);
  P.Z = (Height - Origin.Z);
/* debug
  if (P.X < 0 || P.X >= Dim.X || P.Y < 0 || P.Y >= Dim.Y || P.Z < 0 || P.Z >= Dim.Z)
    {
//      message ("Range Error in GetVoxel");
      return 0;
    } 
*/
  Add = (DWORD)((floor (P.X)) * Dim.Y * Dim.Z + (DWORD)(floor(P.Y)) * Dim.Z +
                                                (DWORD)(floor (P.Z)));
  if (Add < 0xffff)
    return (Volume[Add]);
  else
    MemoryRead (Select, Add, (void FAR *)&Color, 1);
  return (Color);
} // Voxel::GetVoxel

/***************************************************************************/
// 'SortVertices' sorts the vertisec of the corresponding link, with respect
// to their 'Z' coordinate, and puts them in array 'Sorted'.
/***************************************************************************/
void Voxel::SortVertices (SORTType *Sorted)
{
  register int i;
  for (i = 1; i <= Dcel.VertNo; i++)
    {
      Sorted[i].VertNo = i;
      Sorted[i].Z = Robot[RobNo]->absolute_point[LNo][i][3];
    } // for
  qsort ((void *) &Sorted[1], Dcel.VertNo, sizeof (SORTType), Sort_Function);

} // Voxel::SortVertices

/***************************************************************************/
/***************************************************************************/
void Voxel::PreprocessPlane (int PlaneNo)
{
  int      E1, E2, Common1, Common2, E, V;

	if ((Planes[PlaneNo][PVisit] == Visiting) &&
      (Dcel.DcelTable[(int)Planes[PlaneNo][e1]][Visit] == Visiting) && 
      (Dcel.DcelTable[(int)Planes[PlaneNo][e2]][Visit] == Visiting))   
    {
      E1 = Planes[PlaneNo][e1];  
      E2 = Planes[PlaneNo][e2];  

      Common1 = Common2 = 0;
      if ((Dcel.DcelTable[E1][V1] == Dcel.DcelTable[E2][V1]) ||
          (Dcel.DcelTable[E1][V1] == Dcel.DcelTable[E2][V2]))
        Common1 = Dcel.DcelTable[E1][V1];
      if ((Dcel.DcelTable[E1][V2] == Dcel.DcelTable[E2][V1]) ||
          (Dcel.DcelTable[E1][V2] == Dcel.DcelTable[E2][V2]))
        Common2 = Dcel.DcelTable[E1][V2];
      if (Common2 != 0)
        {
          Planes[PlaneNo][e1V1] = V2;   
          Planes[PlaneNo][e1V2] = V1;
          if (Dcel.DcelTable[E1][V2] == Dcel.DcelTable[E2][V1])
            {
              Planes[PlaneNo][e2V1] = V1;
              Planes[PlaneNo][e2V2] = V2;
            } // if
          else
            {
              Planes[PlaneNo][e2V1] = V2;
              Planes[PlaneNo][e2V2] = V1;
            } // else
        } // if
      else if (Common1 != 0)
        {
          Planes[PlaneNo][e1V1] = V1;
          Planes[PlaneNo][e1V2] = V2;
          if (Dcel.DcelTable[E1][V1] == Dcel.DcelTable[E2][V1])
            {
              Planes[PlaneNo][e2V1] = V1;
              Planes[PlaneNo][e2V2] = V2;
	          } // if
          else
            {
              Planes[PlaneNo][e2V1] = V2;
              Planes[PlaneNo][e2V2] = V1;
            } // else
        } // if
      else
        {
          if (Dcel.DcelTable[E1][F1] == PlaneNo)
            {
              V = Dcel.DcelTable[E1][V2];
              E = Dcel.DcelTable[E1][P2];
              Planes[PlaneNo][e1V1] = V2;
              Planes[PlaneNo][e1V2] = V1;
            } // if
          else
            {
              V = Dcel.DcelTable[E1][V1];
              E = Dcel.DcelTable[E1][P1];
              Planes[PlaneNo][e1V1] = V1;
              Planes[PlaneNo][e1V2] = V2;
            } // if

          while (E != E2)
            {
              if (Dcel.DcelTable[E][V1] == V)
                {
		              V = Dcel.DcelTable[E][V2];
                  E = Dcel.DcelTable[E][P2];
                } // if
              else
                {
                  V = Dcel.DcelTable[E][V1];
                  E = Dcel.DcelTable[E][P1];
                } // else
            } // while
          if (Dcel.DcelTable[E][V1] == V)
            {
              Planes[PlaneNo][e2V1] = V1;
              Planes[PlaneNo][e2V2] = V2;
            } // if
          else
            {
              Planes[PlaneNo][e2V1] = V2;
              Planes[PlaneNo][e2V2] = V1;
            } // else
        } // else
    } // if
} // Voxel::PreprocessPlane

/***************************************************************************/
// 'ProcessPlane' is called to mark any voxels which are occupied by the
// face 'PlaneNo' in the current voxel layer. Both the face, and its current
// edges must be active. Each active edge intersects the upper and lower
// surfaces of the current voxel layer in two points. These four points make
// a trapezoid. The border voxels of the trapezoid are marked by calling
// Bresenham algorithms. The interior of the trapezoid is filled by calling
// 'ScanFill'. If any of the active edges is horizontal, and does not
// intersect with the upper and lower surfaces of the current voxel layer,
// then another active edge of this plane --the horizontal edge is between
// which and the other active edge-- is found by calling 'FindAnotherEdge'.
// If the intersection trapezoid is too thin, that is the given plane is
// almost vertical, then only its upper and lower edges are drawn by
// calling Bres procedures.      
/***************************************************************************/
void Voxel::ProcessPlane (int PlaneNo)
{
  double   t, t1;
  point    *Index;
  Point    Rect[6];
  register int i, j;
  Point    Vertices[4];
  int      E1, E2, Flag=0;
  double   MinY = 1000, MaxY = -1000, MinX = 1000, MaxX = -1000;

	Brush = PlaneNo+BBrush;
	if ((Planes[PlaneNo][PVisit] == Visiting) &&
      (Dcel.DcelTable[(int)Planes[PlaneNo][e1]][Visit] == Visiting) &&
      (Dcel.DcelTable[(int)Planes[PlaneNo][e2]][Visit] == Visiting))
    {
      E1 = Planes[PlaneNo][e1];
      E2 = Planes[PlaneNo][e2];
/* debug
      if (E1 == E2)
        message ("Equal Edges!");
*/
      Index = Robot[RobNo]->absolute_point[LNo];
      Vertices[0].X = Index[(int)Planes[PlaneNo][e1V1]][1];
      Vertices[0].Y = Index[(int)Planes[PlaneNo][e1V1]][2];
      Vertices[0].Z = Index[(int)Planes[PlaneNo][e1V1]][3];
      Vertices[1].X = Index[(int)Planes[PlaneNo][e1V2]][1];
      Vertices[1].Y = Index[(int)Planes[PlaneNo][e1V2]][2];
      Vertices[1].Z = Index[(int)Planes[PlaneNo][e1V2]][3];
      Vertices[2].X = Index[(int)Planes[PlaneNo][e2V1]][1];
      Vertices[2].Y = Index[(int)Planes[PlaneNo][e2V1]][2];
      Vertices[2].Z = Index[(int)Planes[PlaneNo][e2V1]][3];
      Vertices[3].X = Index[(int)Planes[PlaneNo][e2V2]][1];
      Vertices[3].Y = Index[(int)Planes[PlaneNo][e2V2]][2];
      Vertices[3].Z = Index[(int)Planes[PlaneNo][e2V2]][3];
      for (i = 0; i < 4; i+=2)
        {
	        if (fabs(t = Vertices[i].Z-Vertices[i+1].Z) < 0.000000001)
            {
							if (Vertices[i].Z >= Height && Vertices[i+1].Z <= Height+1)
                {  // newly changed from i to i+1     ^^^
                  if (i == 0)
										Flag = FindAnotherEdge (&E1, E2, &Vertices[0],
                                            PlaneNo, e1);
                  else
										Flag = FindAnotherEdge (&E2, E1, &Vertices[2],
                                            PlaneNo, e2);
                  if (Flag)
                    {
                      Planes[PlaneNo][PVisit] = Visited;
                      return;
                    } // else
                  else
                    {
                      t = Vertices[i].Z-Vertices[i+1].Z;
                      for (j = 0; j < 2; j++)
		                    {
                          t1 = (Height+j - Vertices[i+1].Z) / t;
                          if (t1 < 0.0)
                            t1 = 0.0;
                          if (t1 > 1.0)
                            t1 = 1.0;
                          Rect[i+j].X = t1 * (Vertices[i].X - Vertices[i+1].X)
                                           +  Vertices[i+1].X;
                          Rect[i+j].Y = t1 * (Vertices[i].Y - Vertices[i+1].Y)
                                           +  Vertices[i+1].Y;
                        } // for
                    } // else
                } // if
              else
                return;
            } // if
          else
            {
              for (j = 0; j < 2; j++)
                {
                  t1 = (Height+j - Vertices[i+1].Z) / t;
                  if (t1 < 0.0)
                    t1 = 0.0;
                  if (t1 > 1.0)
		                t1 = 1.0;
                  Rect[i+j].X = t1 * (Vertices[i].X - Vertices[i+1].X) +
                                      Vertices[i+1].X;
                  Rect[i+j].Y = t1 * (Vertices[i].Y - Vertices[i+1].Y) +
                                      Vertices[i+1].Y;
                } // for
            } // else
        } // for
      Exchange (&Rect[2], &Rect[3]);
      Rect[4].X = Rect[0].X;
      Rect[4].Y = Rect[0].Y;

      if ((sqrt (pow (Rect[0].X - Rect[1].X, 2) +
                 pow (Rect[0].Y - Rect[1].Y, 2)) < 1.0) &&
					(sqrt (pow (Rect[2].X - Rect[3].X, 2) +
                 pow (Rect[2].Y - Rect[3].Y, 2)) < 1.0))
				{
					Bres (Rect[1], Rect[2]);
					Bres (Rect[0], Rect[3]);
					return;
				} // if
			for (i = 0; i < 4; i++)
				Bres (Rect[i], Rect[i+1]);
      for (i = 1, MinY = MaxY = Rect[0].Y, MinX = MaxX = Rect[0].X;
           i < 4; i++)
				{
					MinY = min(MinY, Rect[i].Y);
					MaxY = max(MaxY, Rect[i].Y);
					MinX = min(MinX, Rect[i].X);
					MaxX = max(MaxX, Rect[i].X);
				} // for
			ScanFill ((int)floor (MinX-1), (int)ceil (MaxX+1), (int)floor (MinY-1), (int)ceil (MaxY+1));
    } // if                   
} // Voxel::ProcessPlane

/***************************************************************************/
// ScanFill fills the interior of a polygon which is already drawn on
// a Voxel layer. First the bounding box of the polygon is found. The
// right-most and left-most voxels in each row of the bounding box are found
// using those of the previous row. Then, all the voxels between these two
// voxels are marked.
/***************************************************************************/
void Voxel::ScanFill (int MinX, int MaxX, int MinY, int MaxY)
{
  int i, j, k, l;
  int LeftMost, Left, RightMost, Right, L, R;

	// Find the topmost point
  for (i = MinY; i <= MaxY; i++)
    {
      for (j = MinX; j <= MaxX; j++)
        if (GetVoxel (j, i) == Brush)
          break;
      if (j > MaxX)
        continue;
      else
        break;
    } // for
  if (i > MaxY)
    return;
  for (k = MaxX; k >= MinX; k--)
	  if (GetVoxel (k, i) == Brush)
	    break;
  for (l = j; l <= k; l++)
		SetVoxel (l, i);

  for (LeftMost = Left = j, RightMost = Right = k, i+=1; i <= MaxY; i++)
    {
      for (j = Left-1; j <= Right+1; j++)
        if (GetVoxel (j, i) == Brush)
          break;
      if (j > Right+1)
        return;
      L = j;
      for (j = L-1; j >= MinX; j--)
        if (GetVoxel (j, i) != Brush)
          break;
      LeftMost = j+1;

      for (k = Right+1; k >= Left-1; k--)
        if (GetVoxel (k, i) == Brush)
          break;
      if (k < Left-1)
        return;
      R = k;
      for (k = R+1; k <= MaxX; k++)
        if (GetVoxel (k, i) != Brush)
          break;
      RightMost = k-1;
      for (j = L; j <= R; j++)
				SetVoxel (j, i);
      Left = LeftMost;
      Right = RightMost;
    } // for

} // Voxel::ScanFill

/***************************************************************************/
// 'FindFirstEdge' searches the DCEL structure of this link for the first
// edge that 'Vertex' is a vertex of which. 
/***************************************************************************/
int Voxel::FindFirstEdge (int Vertex)
{
  register int i;

  for (i = 1; i <= Dcel.EdgeNo; i++)
    if ((Dcel.DcelTable[i][V1] == Vertex) ||
        (Dcel.DcelTable[i][V2] == Vertex))
      return (i);
/* debug
  if (i > Dcel.EdgeNo)
    {
      message ("Bad Vertex");
      exit (0);
    } // if
*/
  return 0;
} // Voxel::FindFirstEdge

/***************************************************************************/
// FindAnotherEdge is called by ProcessPlane to find an active edge for
// the given active plane which is not horizontal. 'E1' and 'E2' are two
// active edges of 'Plane' which have a vertex in common. 'E1' is a horizontal
// edge. 'FindAnotherEdge' searches the DCEL structure of this link for
// the other edge of 'Plane' which is connected to 'E2'. This edge has to be
// non-horizontal. If it is horizontal, then 'Plane' is horizontal itself.
// In this case the intersection polygon of 'Plane' and the current voxel
// layer is not a trapezoid but it is 'Plane' itself. Therefore 'DrawPlane'
// is called to fill the voxels occupied by the whole border and interior
// of 'Plane'. 
/***************************************************************************/
int Voxel::FindAnotherEdge (int *E1, int E2, Point *Vertices, int Plane,
                            int p)
{
  int       E, V, W;
  point     *Index;

  V = Dcel.DcelTable[*E1][LastVertex];
  if (Dcel.DcelTable[*E1][V1] == V)
    {
      if (Dcel.DcelTable[*E1][F1] == Plane)
        E = ClockWise (*E1, Dcel.DcelTable[*E1][V1]);
      else
        E = Dcel.DcelTable[*E1][P1];
    } // if
  else
    {
      if (Dcel.DcelTable[*E1][F1] == Plane)
        E = Dcel.DcelTable[*E1][P2];
      else
        E = ClockWise (*E1, Dcel.DcelTable[*E1][V2]);
    } // else
  if ((Dcel.DcelTable[E][Visit] == Visited) || (E == E2))
    {
      Planes[Plane][PVisit] = Visited;  // the last row of the plane
      return (1);
    } // if
  else
    {

      Index = Robot[RobNo]->absolute_point[LNo];
      W = (Dcel.DcelTable[E][V1] == V) ? Dcel.DcelTable[E][V2] :
                                         Dcel.DcelTable[E][V1];
      if (fabs (Index[V][3] - Index[W][3]) < 0.00000000001)
        {
					DrawPlane (Plane, E);
          Planes[Plane][PVisit] = Visited;  // the last row of the plane
          return (1);
        }
      Dcel.DcelTable[E][Visit] = Visiting;
      Dcel.DcelTable[E][LastVertex] = W;
      Planes[Plane][p] = *E1 = E;
      Planes[Plane][p+1] = V;
      Planes[Plane][p+2] = W;
      Vertices[0].X = Index[V][1];
      Vertices[0].Y = Index[V][2];
      Vertices[0].Z = Index[V][3];
      Vertices[1].X = Index[W][1];
      Vertices[1].Y = Index[W][2];
      Vertices[1].Z = Index[W][3];
      return (0);
    } // else
} // Voxel::FindAnotherEdge

/***************************************************************************/
// 'DrawPlane' is called to fill the border and interior voxels of a plane
// 'PlaneNo' which is a horizontal plane. For a horizontal plane, the
// intersection with the current voxel layer is not a trapezoid, but is the
// horizontal plane itself.
/***************************************************************************/
void Voxel::DrawPlane (int PlaneNo, int Edge)
{
  point *Index;
  Point Rect[2];
  int   E, V, W;
  double   MinY = 1000, MaxY = -1000, MinX = 1000, MaxX = -1000;

  E = Edge;
  Index = Robot[RobNo]->absolute_point[LNo];
  do
    {
      V = Dcel.DcelTable[E][V1];
      W = Dcel.DcelTable[E][V2];
      Rect[0].X = Index[V][1];
      Rect[0].Y = Index[V][2];
      Rect[0].Z = Index[V][3];
      Rect[1].X = Index[W][1];
      Rect[1].Y = Index[W][2];
      Rect[1].Z = Index[W][3];
	    MinY = min(MinY, Rect[0].Y);
	    MaxY = max(MaxY, Rect[0].Y);
	    MinX = min(MinX, Rect[0].X);
	    MaxX = max(MaxX, Rect[0].X);
			Bres (Rect[0], Rect[1]);
      // walk Clock Wise around PlaneNo
      if (Dcel.DcelTable[E][F1] == PlaneNo)
        E = Dcel.DcelTable[E][P2];
      else
        E = Dcel.DcelTable[E][P1];
    }
  while (E != Edge);
	ScanFill ((int)floor (MinX-1), (int)ceil (MaxX+1), (int)floor (MinY-1), (int)ceil (MaxY+1));
} // Voxel::DrawPlane

/***************************************************************************/
// Here, 'Center' is a vertex and 'Edge' is an edge connected to it.
// 'ClockWise' returns the next edge, visited after 'Edge', when moving
// around 'Center' in a clockwise fashion.
/***************************************************************************/
int Voxel::ClockWise (int Edge, int Center)
{
  int E, Temp;

  E = Edge;
  while ((Temp = CCW (E, Center)) != Edge)
    E = Temp;
  return (E);
} // Voxel::ClockWise

/***************************************************************************/
// 'CCW' returns the next edge, visited after 'Edge', when moving around
// 'Center' in a counter clockwise fashion.
/***************************************************************************/
int Voxel::CCW (int Edge, int Center)
{
  if (Dcel.DcelTable[Edge][V1] == Center)
    return (Dcel.DcelTable[Edge][P1]);
  else
    return (Dcel.DcelTable[Edge][P2]);
} // Voxel::CCW

/***************************************************************************/
// This routine draws a line between the two endpoints A and B.
// If the line is horizontal or vertical, it marks consecutive voxels
// between the initial and final points. Otherwise, based on the slope
// of the line, that is whether it is in interval [0,45] or [45,90],
// either 'Bres1' or 'Bres2' are called to fill the voxels that have any
// intersection with the actual line. 'Bres1' and 'Bres2' use floating-point
// additions/subtractions.
// 'BresAll' is another function which can be called instead of 'Bres1' or
// 'Bres2'. 'BresAll' draws a thick line which covers the actual line
// completely but some additional voxels are also marked. This function uses
// only integer additions/subtractions.
/***************************************************************************/
void Voxel::Bres (Point B, Point A)
{
  double T, i, Xa, Ya, Xb, Yb;

	Xb = B.X; Yb = B.Y; Xa = A.X; Ya = A.Y;
  if (fabs (Xb - Xa) < 0.0000001)
    {
      if (Yb > Ya)
        {
          T = Yb;
          Yb = Ya;
          Ya = T;
        } // if
      Xb = floor (Xb);
      for (i = floor (Yb); i <= floor (Ya); i++)
				SetVoxel (Xb, i);
    } // if
  else if (fabs (Yb - Ya) < 0.0000001)
    {
      if (Xb > Xa)
        {
          T = Xb;
          Xb = Xa;
          Xa = T;
        } // if
      Yb = floor (Yb);
      for (i = floor (Xb); i <= floor (Xa); i++)
				SetVoxel (i, Yb);
    } // if
  else
    {
      BresAll (Xb, Yb, Xa, Ya); // instead of the following commented code
/*
      if (Xb < Xa)
        {
          if (Yb < Ya)
            Bres1 (Xb, Yb, Xa, Ya);
          else
            Bres2 (Xb, Yb, Xa, Ya);
        } // if
      else
        {
          if (Ya < Yb)
            Bres1 (Xa, Ya, Xb, Yb);
          else
            Bres2 (Xa, Ya, Xb, Yb);
        } // else
*/
    } // else

} // Voxel::Bres

/***************************************************************************/
// 'Bres1' is used to draw lines with slopes in interval [0,45].
/***************************************************************************/
void Voxel::Bres1 (double Xb, double Yb, double Xa, double Ya)
{
  double DelX, DelY, X1, Y1, T, X2, Y2;
  char   f1, f2;

  DelY = (Ya - Yb) / (Xa - Xb);
  DelX = (Xa - Xb) / (Ya - Yb);
  X2 = floor (Xb); Y2 = DelY * X2 + (Yb - DelY * Xb);
  Y1 = floor (Yb); X1 = DelX * (Y1 - (Yb - DelY * Xb));
  f1 = f2 = 0;
  while (!f1)
    if (X1 + DelX > floor (X1))
      {
        T = max (floor (X1), floor (Xb));
        while ((T < X1 + DelX) && (T < Xa))
          {
						SetVoxel (T, Y1);
            T += 1;
          } // while
        if (T > Xa)
          f1 = 1;
        else
          {
            X1 += DelX;
            Y1 += 1;
          } // else
      } // if
  while (!f2)
    if (Y2 + DelY > floor (Y2))
      {
        T = max (floor (Y2), floor (Yb));
        while ((T < Y2 + DelY) && (T < Ya))
          {
						SetVoxel (X2, T);
            T += 1;
          } // while
        if (T > Ya)
          f2 = 1;
        else
          {
            Y2 += DelY;
            X2 += 1;
          } // else
      } // if
  if (floor (Xa) == Xa)
		SetVoxel (Xa, floor (Ya));
  if (floor (Ya) == Ya)
		SetVoxel (floor (Xa), Ya);
  if (floor (Xb) == Xb)
		SetVoxel (Xb-1, floor (Yb));
  if (floor (Yb) == Yb)
		SetVoxel (floor (Xb), Yb-1);

} // Voxel::Bres1

/***************************************************************************/
// 'Bres2' is used to draw lines with slopes in interval [45,90].
/***************************************************************************/
void Voxel::Bres2 (double Xb, double Yb, double Xa, double Ya)
{
  double DelX, DelY, X1, Y1, T, X2, Y2;
  char   f1, f2;

  DelY = (Ya - Yb) / (Xa - Xb);
  DelX = (Xa - Xb) / (Ya - Yb);
  X2 = floor (Xb); Y2 = DelY * X2 + (Yb - DelY * Xb);
  Y1 = ceil (Yb); X1 = DelX * (Y1 - (Yb - DelY * Xb));
  f1 = f2 = 0;
  while (!f1)
    if (X1 - DelX > floor (X1))
      {
        T = max (floor (X1), floor (Xb));
        while ((T < X1 - DelX) && (T < Xa))
          {
						SetVoxel (T, Y1-1);
            T += 1;
          } // while
        if (T > Xa)
          f1 = 1;
        else
          {
            X1 -= DelX;
            Y1 -= 1;
          } // else
      } // if
  while (!f2)
    if (Y2 + DelY < ceil (Y2))
      {
        T = min (ceil (Y2), ceil(Yb));
        while ((T > Y2 + DelY) && (T > Ya))
          {
						SetVoxel (X2, T-1);
            T -= 1;
          } // while
        if (T < Ya)
          f2 = 1;
        else
          {
            Y2 += DelY;
            X2 += 1;
          } // else
      } // if
  if (floor (Xa) == Xa)
		SetVoxel (Xa, floor (Ya));
  if (floor (Ya) == Ya)
		SetVoxel (floor (Xa), Ya-1);
  if (floor (Xb) == Xb)
		SetVoxel (Xb-1, floor (Yb));
  if (floor (Yb) == Yb)
		SetVoxel (floor (Xb), Yb);
} // Voxel::Bres2

/***************************************************************************/
// This version of Bresenham Algorithm draws a thick line between two points
// Only integer additions/subtractions are used. For each voxel which is
// marked by the original Bresenham algorithm, two adjacent voxels are also
// marked. In fractures of the original Bresenham line, two additional
// voxels are also marked. 
/***************************************************************************/
void Voxel::BresAll (double Xb, double Yb, double Xa, double Ya)
{
  int DelX, DelY, X1, Y1, S1, S2, temp, P, incr1, incr2, d;
  register int i;

  X1 = (int)floor(Xb);
  Y1 = (int)floor(Yb);
  DelX = (int)floor(Xa) - X1;
  DelY = (int)floor(Ya) - Y1;
  S1 = Sign (DelX);
  S2 = Sign (DelY);
  DelX = abs (DelX);
  DelY = abs (DelY);
  P = 0;
  if (DelY > DelX)
    {
      temp = DelY;
      DelY = DelX;
      DelX = temp;
      P = 1;
    } // if
  d = 2 * DelY - DelX;
  incr1 = -2 * DelX;
  incr2 = 2 * DelY;
  for (i = 0; i <= DelX; i++)
    {
			SetVoxel (X1, Y1);
      if (P)
				{
					SetVoxel (X1-1, Y1);// increasing the width of the line by 2 pixels
					SetVoxel (X1+1, Y1);
				} // if
			else
				{
					SetVoxel (X1, Y1+1);
					SetVoxel (X1, Y1-1);
				} // else
			if (d >= 0)
				{
					if (P)
            {
						  X1 += S1;
              SetVoxel (X1+S1, Y1);       // Filling fractures by
              SetVoxel (X1-2*S1, Y1+S2);  // additional pixels
            }
					else
            {
  					  Y1 += S2;
              SetVoxel (X1, Y1+S2);       // Filling fractures by
              SetVoxel (X1+S1, Y1-2*S2);  // additional pixels
            }
					d += incr1;
				} // if
			if (P)
				Y1 += S2;
      else
				X1 += S1;
      d += incr2;
    } // for
} // Voxel::BresAll

/***************************************************************************/
// 'Show' draws consecutive voxel layers of this link. 
/***************************************************************************/
void Voxel::Show (void)
{
	RECT		 rc;
	HDC			 hdc;
	char     str[40];
  register int i, j, k;
	HBRUSH   hbr, hbrOld;
	HPEN		 hpen, hpenOld;

  if (!IfInit)
    return;

	GetClientRect (HwndMap, &rc);
	InvalidateRect (HwndMap, &rc, TRUE);
  UpdateWindow (HwndMap);
	hdc = GetDC (HwndMap);
  hpen = CreatePen (PS_SOLID, 0, RGB (0, 0, 255));
	hpenOld = SelectObject (hdc, hpen);
	sprintf (str, "Link%d of Robot%d Swept Volume", LNo, RobNo);
	TextOut (hdc, 200, 400, str, strlen (str));
	hbr = CreateSolidBrush (RGB(0, 0, 255));
	SelectObject (hdc, hbr);
	for (k = 0; k < Dim.Z; k++)
    {
      DrawGrid (hdc);
      Height = k + Origin.Z;
      for (i = 0; i < Dim.X; i++)
        for (j = 0; j < Dim.Y; j++)
          {
            if (GetVoxel (i+Origin.X, j+Origin.Y))
							Rectangle (hdc, j*5+1, i*5+1, j*5+4, i*5+4);
          } // if
			SetRect (&rc, 0, 0, j*5+1, i*5+1);
			for (j = 0; j < 30000; j++);

			InvalidateRect (HwndMap, &rc, TRUE);
			UpdateWindow (HwndMap);
		} // for
	SelectObject (hdc, hbrOld);
  DeleteObject (hbr);
	SelectObject (hdc, hpenOld);
	DeleteObject (hpen);
	ReleaseDC (HwndMap, hdc);
} // Voxel::Show

/***************************************************************************/
// 'DrawGrid' draws an empty grid of the current voxel layer of this link,
// the occupied voxels of which are filled later.
/***************************************************************************/
void Voxel::DrawGrid (HDC hdc)
{ // DrawGrid
	register int i;

	for (i = 0; i <= (int)(Dim.Y*5); i += 5)
		{
		  MoveTo (hdc, i, 0);
			LineTo (hdc, i, (int)(Dim.X*5));
    } // for
	for (i = 0; i <= (int)(Dim.X*5); i += 5)
		{
			MoveTo (hdc, 0, i);
			LineTo (hdc, (Dim.Y*5), i);
		} // for
} // Voxel::DrawGrid
//
/***************************************************************************/
// 'InitDCEL' initializes the DCEL structure of this link with the given
// DCEL structure.
/***************************************************************************/
void Voxel::InitDCEL (DCELType *DC)
{
  register int i, j;
  Dcel.EdgeNo = DC->EdgeNo;
  Dcel.FaceNo = DC->FaceNo;
  Dcel.VertNo = DC->VertNo;
  for (i = 0; i <= DC->EdgeNo; i++)
    for (j = V1; j <= Date; j++)
      Dcel.DcelTable[i][j] = DC->DcelTable[i][j];
} // Voxel::InitDCEL

/***************************************************************************/
// 'Save' copies the voxel structure of this link into the given file (Out).
/***************************************************************************/
void Voxel::Save(HFILE Out)
{
	if (!IfInit)
    return;
  Size = Dim.X * Dim.Y * Dim.Z;
  _lwrite (Out, &LNo, sizeof(LNo)+sizeof(Origin)+sizeof(Dim)+sizeof(Size));
  _hwrite (Out, Volume, Size);
} // Voxel::Save

/***************************************************************************/
// 'Read' first reads the Origin, Dimensions and size of the voxel structure
// of this link from the given file (In). Then it copies the voxel contents
// from the file into an allocated space for this link's voxel structure.   
/***************************************************************************/
int Voxel::Read (HFILE In, int RobotNo, int L)
{
 	RobNo = RobotNo;
  LNo = L;
  _lread (In, &Origin, sizeof(Origin)+sizeof(Dim)+sizeof(Size));
  if (((hglb = GlobalAlloc( GPTR | GMEM_DISCARDABLE, (DWORD)Size)) == NULL) ||
      ((Volume = (LPBYTE) GlobalLock (hglb)) == NULL) )
    {
      if (hglb != NULL)
        GlobalFree(hglb);
      hglb = NULL;
      _llseek (In, Size, 1);
      return 1;
    } // if

  _hread (In, Volume, Size);
  Select = SELECTOROF ((void FAR *) Volume);
  IfInit = 1;
  return 0;
} // Voxel::Read

/***************************************************************************/
// 'SortFunction' is the sorting function which is called by qsort in
// 'SortVertices'. It acts upon the Z coordinate of the two given points.  
/***************************************************************************/
int Sort_Function (const void *a, const void *b)
{
   if (((SORTType *) a)->Z > ((SORTType *) b)->Z)
     return (1);
   else if (((SORTType *) a)->Z < ((SORTType *) b)->Z)
     return (-1);
   else
     return(0);
} // Sort_Function

/***************************************************************************/
// This procedure exchanges the coordinates of the two given points.
// It is called in ProcessPlane to order the vertices of the intersection
// trapezoid of an active plane and the current voxel layer. 
/***************************************************************************/
void Exchange (Point *a, Point *b)
{

  double Temp;

  Temp = a->X;
  a->X = b->X;
  b->X = Temp;
  Temp = a->Y;
  a->Y = b->Y;
  b->Y = Temp;
  Temp = a->Z;
  a->Z = b->Z;
  b->Z = Temp;
} // Exchange

/***************************************************************************/
// This procedure initializes the voxel structures of the links of the given
// robot. 'Status' determines whether this is the first or second step of
// voxel generation for the given path-segment. In the first step, only the
// dimensions of the voxel structures are computed, by moving the robot
// in the path-segment. In the second step, voxel structures of the
// computed dimensions are allocated, the path-segment is played once again,
// and the voxels visited during the motion are marked.
/***************************************************************************/
void Vinit (int RobotNo, int Status)
{
  register int i;

  for (i = 0; i < MaxLink; i++)
    if (SweptVolume[RobotNo][i].Initialize (RobotNo, i, Status))
      {
        message ("memory error on link");
        exit (0);
      } // if
} // Vinit

/***************************************************************************/
// This procedure is called to update the voxel structures of the links of
// the given robot. It is called on consecutive points of the given robot's
// path-segment.
/***************************************************************************/
void VUpdate (int RobotNo)
{
  register int i;

  for (i = 1; i < MaxLink; i++) // i = 1 because the base doesn't move
    if (SweptVolume[RobotNo][i].Update ())
      {
        message ("memory error");
        exit (0);
      } // if
} // VUpdate

/***************************************************************************/
// This procedure draws the voxel structures of the links of the given
// robot, layer by layer, on the screen. For each link of the given robot,
// 'Show' is called to draw its voxel layers.
/***************************************************************************/
void VShow (int RobotNo)
{
	register int j;
	RECT		 rc;

	EnableAllButtons (FALSE);
	HwndMap = HwndView;
	for (j = 0; j < MaxLink; j++)
    SweptVolume[RobotNo][j].Show ();
	EnableWindow (GetDlgItem (HwndPannel, ID_BACK), TRUE);
	while (HwndMap != 0)
		BackGround ();
	GetClientRect (HwndView, &rc);
	InvalidateRect (HwndView, &rc, TRUE);
	UpdateWindow (HwndView);
	EnableAllButtons (TRUE);
} // VShow

/***************************************************************************/
// This procedure is called when the user enters a 'readvol(filename)'
// command to examine and see the swept volumes of the links of a robot.
// The given file must have been generated by an 'fread' command. Such a file
// has the number of the robot and the number of its links in its header.
// The voxel structures of each link of the robot are read into their
// corresponding structures by calling the method 'Read'. Then by calling
// 'VShow', the swept volumes of the links are shown consecutively. 
/***************************************************************************/
int VRead (void)
{
  HFILE     In;
  OFSTRUCT  OFS;
  char far  *ptr;
  register int j;
  int      N, L, RobNo, TempMess;

  BackGround (); // notice

  if (ParNo != 1)
    return (1);
  ptr=_fstrchr(param_part,')');
  if (ptr==NULL)
    {
      message ("Syntax Error");
      return(0);
    } /* if */
  *ptr=0;

  if (OpenFile (param_part, &OFS, OF_READ | OF_EXIST) == HFILE_ERROR)
    {
       sprintf (mess, "File %s not found", param_part);
       message (mess);
       return(1); // must be 0
     } // if
   else
     In = OpenFile (param_part, &OFS, OF_READ);

  _lread (In, &RobNo, sizeof (RobNo));
  if (Robot[RobNo]->Vox == 1)
    {
      TempMess = IfMessage;
      IfMessage = 1;
      message ("Volume is already filled");
      _lclose (In);
      IfMessage = TempMess;
      return 0;
    } // if

  _lread (In, &N, sizeof (N));

  _lread (In, &Presents[RobNo][0], sizeof (PresentType)*N);

  for (j = 0; j < N; j++) /*MaxLink-1*/
    {
      _lread (In, &L, sizeof (L));
      if (SweptVolume[RobNo][L].Read(In, RobNo, L))
        {
	        TempMess = IfMessage;
	        IfMessage = 1;
	        sprintf (mess, "Out of memory for Robot%d, Link%d, %ld",
		               RobNo, L, SweptVolume[RobNo][L].Dim.X *
			             SweptVolume[RobNo][L].Dim.Y *
			             SweptVolume[RobNo][L].Dim.Z);
	        message (mess);
	        IfMessage = TempMess;
          _lclose (In);
	        return 0;
        } // if
      j = L;
    } // for
  _lclose (In);
  Robot[RobNo]->Vox = 1;
  if (IfMessage)
    VShow (RobNo);

  return 0;
} // VRead

/***************************************************************************/
// This procedure saves the swept volume data (the voxel structures) of the
// given robot in the given file.
/***************************************************************************/
void VSave (int RobotNo, char *FileName)
{
  int      j;
  HFILE     Out;
  OFSTRUCT  OFS;

  BackGround (); 

  if (!Robot[RobotNo]->Vox)
    {
      message ("Volume is not filled");
    } // if
  j = MaxLink; // number of total considered links

  if ((Out = OpenFile (FileName, &OFS, OF_WRITE | OF_CREATE)) == NULL)
    {
      message ("Can not create file");
      return;
    } // if
  _lwrite (Out, &RobotNo, sizeof (RobotNo));
  _lwrite (Out, &j, sizeof (j));

  SaveStarts (RobotNo, Out);
  _lclose (Out);
} // VSave

/***************************************************************************/
// This procedure which is called by 'VSave' generates the header of the
// .vol file and then copies the voxel structures of the links of the given
// robot into it. The header gives the link number, origin and dimensions
// of every present link of the robot, together with the address of the
// actual voxel structure of that link in the file.
/***************************************************************************/
void SaveStarts (int RobotNo, HFILE Out)
{
	unsigned long Next;
  int      i;

  Next = sizeof (RobotNo) + sizeof (int) + MaxLink * (sizeof(PresentType));
  for (i = 0; i < MaxLink; i++)
    {
      if (SweptVolume[RobotNo][i].IfInit)
        {
          Presents[RobotNo][i].Present = 1;
          Presents[RobotNo][i].FSeek = Next;
          Next += (SweptVolume[RobotNo][i].Dim.X *
                   SweptVolume[RobotNo][i].Dim.Y *
                   SweptVolume[RobotNo][i].Dim.Z +
                   sizeof(int)+sizeof(Point)+sizeof(Dimensions)+sizeof(unsigned long) );
                   //     LNo         Origin        Dim                Size                          
        } // if
      else
        Presents[RobotNo][i].Present = 0;
    } // for
  _lwrite (Out, &Presents[RobotNo][0], sizeof (PresentType)*MaxLink);

  for (i = 0; i < MaxLink; i++)
    if (Presents[RobotNo][i].Present)
      SweptVolume[RobotNo][i].Save (Out);
} // SaveStarts

/***************************************************************************/
// This procedure calls 'Finalize' for each link of the given robot, to
// reset the voxel structures of all of them.
/***************************************************************************/
void VFinalize (int RobotNo)
{
  register int j;

  for (j = 0; j < MaxLink; j++)
    SweptVolume[RobotNo][j].Finalize ();
  Robot[RobotNo]->Vox = 0;

} // VFinalize

/***************************************************************************/
// This is the sorting function called by qsort in 'VoxelIntersect'. It
// acts upon its two inputs due to their 'Coord' fields.
/***************************************************************************/
int BoxSort (const void *a, const void *b)
{
  if (((BoxType *)a)->Coord > ((BoxType *)b)->Coord)
    return 1;
  else if (((BoxType *)a)->Coord < ((BoxType *)b)->Coord)
    return -1;
  else return 0;
} // BoxSort

/***************************************************************************/
// This function determines whether its two given swept volumes, which are
// actually non-empty voxel structures intersect. It first checks whether
// the voxel structures overlap. If not, then the two swept volumes are
// non-intersecting. Otherwise the common intersection of the two voxel
// structures is searcehd for any voxel which is occupied in both of the
// structures. If any such a voxel is found, then the two swept volumes are
// intersecting and a '1' is returned. Otherwise a '0' is returned.
/***************************************************************************/
int VoxelIntersect (Voxel *Vx1, Voxel *Vx2)
{
  unsigned  i, j, k; 
  int       MinX1, MinY1, MinZ1,
            MaxX1, MaxY1, MaxZ1,
            MinX2, MinY2, MinZ2,
            MaxX2, MaxY2, MaxZ2,
            MinX, MinY, MinZ,
            MaxX, MaxY, MaxZ,
            DimX, DimY, DimZ, pX, pY;

  BackGround (); 

  if (!(Vx1->IfInit && Vx2->IfInit))
    return 0;

  MinX1 = Box[0].Coord = (int)Vx1->Origin.X;
  MaxX1 = Box[1].Coord = (int)Vx1->Dim.X + MinX1-1;
  MinX2 = Box[2].Coord = (int)Vx2->Origin.X;
  MaxX2 = Box[3].Coord = (int)Vx2->Dim.X + MinX2-1;
  MinY1 = Box[4].Coord = (int)Vx1->Origin.Y;
  MaxY1 = Box[5].Coord = (int)Vx1->Dim.Y + MinY1-1;
  MinY2 = Box[6].Coord = (int)Vx2->Origin.Y;
  MaxY2 = Box[7].Coord = (int)Vx2->Dim.Y + MinY2-1;
  MinZ1 = Box[8].Coord = (int)Vx1->Origin.Z;
  MaxZ1 = Box[9].Coord = (int)Vx1->Dim.Z + MinZ1-1;
  MinZ2 = Box[10].Coord = (int)Vx2->Origin.Z;
  MaxZ2 = Box[11].Coord = (int)Vx2->Dim.Z + MinZ2-1;

  if ((MinX1 > MaxX2) || (MinX2 > MaxX1) ||
      (MinY1 > MaxY2) || (MinY2 > MaxY1) ||
      (MinZ1 > MaxZ2) || (MinZ2 > MaxZ1))
    return (0);
  else
    {
      for (i = 0; i < 12; i++)
        {
          Coord[i] = Box[i].Coord;
          Box[i].Index = i;
        } // for
      for (i = 0; i < 12; i+=4)
        qsort ((void *)&Box[i], 4, sizeof (BoxType), BoxSort);
      MinX = Coord[Box[1].Index];
      MaxX = Coord[Box[2].Index];
      MinY = Coord[Box[5].Index];
      MaxY = Coord[Box[6].Index];
      MinZ = Coord[Box[9].Index];
      MaxZ = Coord[Box[10].Index];
      DimX = MaxX - MinX;
      DimY = MaxY - MinY;
      DimZ = MaxZ - MinZ;

      if (DimX < 0.0 || DimY < 0.0 || DimZ < 0.0)
        {
        printf ("Error");
        getch ();
        exit (-1);
        } // if
      for (k = 0, Vx1->Height = Vx2->Height = MinZ; k <= DimZ;
           k++, Vx1->Height += 1, Vx2->Height += 1)
        for (i = 0, pX = MinX; i <= DimX; pX+=1, i++)
          for (j = 0, pY = MinY; j <= DimY; pY+=1, j++)
            {
              if ((Vx2->GetVoxel ((double)pX, (double)pY)) == 0)
                continue;
              if ((Vx1->GetVoxel ((double)pX, (double)pY)) == 0)
                continue;
              else
                return (1);
            } // for
      return 0;
    } // else
} // VoxelIntersect

/***************************************************************************/
// This routine calls 'FreeVol' for each robot of the workcell, to free all
// the allocated voxel structures. 
/***************************************************************************/
int FreeVols (void)
{
  register int i;

  if (ParNo != 0)
    return (1);
  for (i = 0; i < MaxRobot; i++)
    FreeVol(i);
  return (0);
} // FreeVols

/***************************************************************************/
// This procedure calls 'Finalize' for each link of the given robot to
// reset the corresponding voxel structures and free the allocated memories. 
/***************************************************************************/
void FreeVol (int Rob)
{
	register int j;
  for (j = 0; j < MaxLink; j++)
    SweptVolume[Rob][j].Finalize ();
  Robot[Rob]->Vox = 0;
} // FreeVol

/***************************************************************************/
// CheckVols checks for any collision between the voxel structures of the
// swept volumes of two robots.
// The swept volume of each link of 'Rob0' is checked for a collision against
// the swept volume of each link of 'Rob1'.
/***************************************************************************/
int CheckVols (int Rob0, int Rob1)
{
  int i, j;

  if (ParNo != 2)
    return 1;
  if (!(Robot[Rob0]->Vox && Robot[Rob1]->Vox))
    {
      message ("Please first read SweptVolumes of both Robots!");
      return 0;
    } // if
  for (i = MaxLink - 1; i >= 0; i--)
    {
      for (j = MaxLink - 1; j >= 0; j--)
	    if (VoxelIntersect (&SweptVolume[Rob0][j], &SweptVolume[Rob1][i]))
        return 1;  // must be 0
    } // for
//  message ("No Collisions between the two Volumes");
  return 0;
} // CheckVols

/***************************************************************************/
// VoxelCollision checks for any collision between two Voxel structures.
// It first checks if the two structures have a common intersection, and   
// if so, searches for any voxel in the common intersection which is filled
// in both of the Voxel structures
/***************************************************************************/
int VoxelCollision (int Rob0, int Rob1, int L0, int L1)
{
  int ReturnValue = 0;

  SweptVolume[Rob0][L0].Initialize (Rob0, L0, 0);
  SweptVolume[Rob0][L0].Initialize (Rob0, L0, 1);
  SweptVolume[Rob1][L1].Initialize (Rob1, L1, 0);
  SweptVolume[Rob1][L1].Initialize (Rob1, L1, 1);

  if (VoxelIntersect (&SweptVolume[Rob0][L0], &SweptVolume[Rob1][L1]))
    ReturnValue = 1;
  SweptVolume[Rob0][L0].Finalize ();
  SweptVolume[Rob1][L1].Finalize ();
  return (ReturnValue);
} // VoxelCollision