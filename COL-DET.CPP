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
#include <ctype.h>
#include <dos.h>
#include <math.h>
#include <process.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot.h"

#define  Sign(X)      ((X>0.0)?1:-1)

void  SetIntersection (void);
int 	MultiCollision_Detect (void);
char  Cylinder(VType V1, VType V2);
char  Exact_Collision (char LNo1, char LNo2);
void  MakeViewMap (char LNo1, char LNo2, frame map);
char  Third_Plane (signed char *LastLink1, signed char *LastLink2,
                   signed char *LastLink2For1, signed char *LastLink1For2);
char 	Collision_Detect(signed char *LastLink1, signed char *LastLink2,
                       signed char *LastLink2For1, signed char *LastLink1For2);
void  SetEnd (VType *PC, float X1, float Y1, float X2, float Y2);
void  ShowThirdProjection (char LNo1, char LNo2, VType *V, VType *W);
void  ShowProjection (char Robot1, char Robot2, char LNo1,
                      char LNo2, VType *V, VType *W);

extern  HDC  hdc;
extern  char    mess[];
extern  int active_page;
extern  cRobot huge *Robot[];
extern  HWND HwndMap, HwndView, HwndPannel;
extern  float display_gain_x, display_gain_y;

extern  void ResetColor (void);
extern  void BackGround (void);
extern  void  Report (char *Text);
extern  void  message (char *Text);
extern  void SetColor (RGBColor *C);
extern  void EnableAllButtons (BOOL Enable);
extern  float *cross_product (float *a,float *b,float *c);
extern  int VoxelCollision (int Rob0, int Rob1, int L0, int L1);

char    IfShowProj = 0;
char    Robot1, Robot2;
char    Intersection[MaxLink][MaxLink];
LOGBRUSH logBrush = {BS_HOLLOW, 0, 0};

double P[4][2];
double DelX1, DelY1, DelX2, DelY2, Delta,
       X11, Y11, X12, Y12,
       X21, Y21, X22, Y22,
       D1, D2,
       Phi, 
       XVar, YVar,
       T1, T2,
       L1, L2,
       Dist, Temp,
       R1, R2,
       DelX, DelY;


/***************************************************************************/
//     On-Line Collision Deterction Routines
/***************************************************************************/

/***************************************************************************/
// SetIntersection initializes the matrix 'Intersection'. Those elements
// of this matrix which correspond to links of the two robots that are known
// to always be non-intersecting, regardless of the joint values, are set
// to 1.
/***************************************************************************/
void SetIntersection (void)
{ // SetIntersection

  char i, j;

  for (i = 0; i < MaxLink; i++)
    for (j = 0; j < MaxLink; j++)
      Intersection[i][j] = 0;
  // notice for this special case of robot positions ????????????
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      Intersection[i][j] = 1;

} // SetIntersection 

/***************************************************************************/
// ShowProjection projects the scene of two robots (Robot1 and Robot2) on
// the projection plane of link LNo1 of Robot1.
// Only links 0 through LNo1 of Robot1, and links 0 through LNo2 of Robot2
// are projected. 
// The projection data for Robot1 is written in array V, and that of Robot2
// is written in array W  
/***************************************************************************/
void ShowProjection (char Robot1, char Robot2, char LNo1, char LNo2,
		                 VType *V, VType *W)
{ // ShowProjection

  RECT     rc;
  VType    *VT;
  char     No[2];
  register int i;
  HBRUSH   hbr, hbrOld;

  No[1] = 0;
  if (IfShowProj)
    {
	    HwndMap = HwndView;
	    GetClientRect (HwndMap, &rc);
	    InvalidateRect (HwndMap, &rc, TRUE);
	    UpdateWindow (HwndMap);
	    hdc = GetDC (HwndMap);
	    hbr = CreateBrushIndirect (&logBrush);
	    SelectObject (hdc, hbr);
    } // if
  for (i = 0; i <= LNo1; i++) 
    Robot[Robot1]->compute_point (Robot[Robot1]->inv_map[LNo1], i,
			                        Robot[Robot1]->LinkArray[LNo1].PType, &V[i]);
  for (i = 0; i <= LNo2; i++)
    Robot[Robot2]->compute_point (Robot[Robot1]->inv_map[LNo1], i,
				                         Robot[Robot1]->LinkArray[LNo1].PType, &W[i]);
  if (IfShowProj) 
    {
	    No[0] = '0' + Robot1;
      TextOut (hdc, 200+(int)floor(V[0].X1*display_gain_x)*.8,
		                170-(int)floor(V[0].Y1*display_gain_y)*.8, No, 1);
	    No[0] = '0' + Robot2;
      TextOut (hdc, 200+(int)floor(W[0].X1*display_gain_x)*.8,
		                170-(int)floor(W[0].Y1*display_gain_y)*.8, No, 1);
	    SelectObject (hdc, hbrOld);
      DeleteObject (hbr);
	    ReleaseDC (HwndMap, hdc);
	    EnableWindow (GetDlgItem (HwndPannel, ID_BACK), TRUE);
	    while (HwndMap != 0)
		    BackGround ();
	    GetClientRect (HwndView, &rc);
	    InvalidateRect (HwndView, &rc, TRUE);
	    UpdateWindow (HwndView);
    } // if
} // ShowProjection

/***************************************************************************/
// SetEnd projects the end points of the axis of a cylindrically modeled link
// onto the projection plane of this link. 'PC' will hold the projected
// coordinates, and the other parameters hold the corresponding values. 
/***************************************************************************/
void SetEnd (VType *PC, float X1, float Y1, float X2, float Y2)
{ // SetEnd

  PC->X1 = X1;
  PC->Y1 = Y1;
  PC->X2 = X2;
  PC->Y2 = Y2;

} // SetEnd

/***************************************************************************/
// Collision_Detect in a parallel fashion checks whether
//  1) there is any collision between link0 through LastLink1 of Robot1 and
//     link0 through LastLink2For1 of Robot2,
//  2) there is any collision between link0 through LastLink2 of Robot2 and
//     link0 through LastLink1For2 of Robot1.
// For each set of the above checkings, a different projection plane is used
// i.e. the projection plane of LastLink1 for the first set, and the
// projection plane of LastLink2 for the second set.
/***************************************************************************/
char Collision_Detect (signed char *LastLink1, signed char *LastLink2,
		                   signed char *LastLink2For1, signed char *LastLink1For2)
{ // Collision_Detect

  signed   char     CurLink1, CurLink2, CurLink2For1, CurLink1For2;
  signed   char     InitialCurLink1, InitialCurLink2;

  char     IfCollision1, IfCollision2;
  VType    V11[MaxLink], V12[MaxLink], V21[MaxLink], V22[MaxLink];

  IfCollision1 = IfCollision2 = 0;
  InitialCurLink1 = CurLink1 = *LastLink1;
  InitialCurLink2 = CurLink2 = *LastLink2;
  CurLink1For2 = *LastLink1For2;
  CurLink2For1 = *LastLink2For1;
  ShowProjection (Robot1, Robot2,
                  InitialCurLink1, InitialCurLink2, V11, V12);
  ShowProjection (Robot2, Robot1,
                  InitialCurLink2, InitialCurLink1, V22, V21);
  for (;;)
    {
      if (!IfCollision1)
        {
	        if (CurLink1 >= 0)
	          {
	            if (CurLink2For1 < 0)
		            {
		              CurLink2For1 = CurLink2;
		              --CurLink1;
		            } // if
	            if ((CurLink1 == -1) || (CurLink2For1 == -1))
		            IfCollision1 = 1;
	            else
		            {
		              if (Intersection[CurLink1][CurLink2For1] ||
                      !Cylinder (V11[CurLink1], V12[CurLink2For1]))
                    {
                    // First projection on CurPlane1 with CurLinkink1 of
                    // Robot1 and CurLink2For1 of Robot2 
		                  Intersection[CurLink1][CurLink2For1] = 1;
		                  --CurLink2For1;
                    } // if
                  else
	                  IfCollision1 = 1;
                } // else
	          } // if
        } // if
      if (!IfCollision2)
        {
	        if (CurLink2 >= 0)
            {
	            if (CurLink1For2 < 0)
                {
		              CurLink1For2 = CurLink1;
                  --CurLink2;
		            }  // if
	            if ((CurLink2 == -1) || (CurLink1For2 == -1))
		            IfCollision2 = 1;
              else
		            {
		              if (Intersection[CurLink1For2][CurLink2] ||
                      !Cylinder (V22[CurLink2], V21[CurLink1For2]))
		                {
                    // Second projection on CurPlane2 with CurLinkink1For2
                    // of Robot1 and CurLink2 of Robot2 
		                  Intersection [CurLink1For2][CurLink2] = 1;
		                  --CurLink1For2;
		                } // if
		              else
	                  IfCollision2 = 1;
                } // else
	          } // if
	      } // if
      if ((CurLink1 <= -1) || (CurLink2 <= -1))
        return (0);
      if (IfCollision1 && IfCollision2)
        {
	        if (CurLink2For1 > CurLink2)
            {
              CurLink2For1 = CurLink2;
	            IfCollision1 = 0;
            } // if
	        if (CurLink1For2 > CurLink1)
	          {
	            CurLink1For2 = CurLink1;
	            IfCollision2 = 0;
	          } // if
	        if (Intersection[CurLink1][CurLink2For1] ||
              !Cylinder (V21[CurLink1], V22[CurLink2For1]))
	          {
	            IfCollision1 = 0;
	            Intersection[CurLink1][CurLink2For1] = 1;
	            --CurLink2For1;
	          } // if
	        if (Intersection[CurLink1For2][CurLink2] ||
              !Cylinder (V11[CurLink1For2], V12[CurLink2]))
            {
	            IfCollision2 = 0;
	            Intersection[CurLink1For2][CurLink2] = 1;
              --CurLink1For2;
            } // if
          if (IfCollision1 && IfCollision2)
	          {
              *LastLink1 = CurLink1;
              *LastLink2 = CurLink2;
              *LastLink1For2 = CurLink1For2;
              *LastLink2For1 = CurLink2For1;
	            if ((CurLink1 == InitialCurLink1) &&
                  (CurLink2 == InitialCurLink2))
                {
		              if (Third_Plane (LastLink1, LastLink2,
                                   LastLink2For1, LastLink1For2))
                    return (1);
                  else
                    {
                      if (CurLink1 != *LastLink1)
                        {
                          CurLink1 = *LastLink1;
		                      IfCollision1 = 0;
                        } // if
                      if (CurLink2 != *LastLink2)
                        {
                          CurLink2 = *LastLink2;
		                      IfCollision2 = 0;
                        } // if
                      if (CurLink1For2 != *LastLink1For2)
                        {
                          CurLink1For2 = *LastLink1For2;
		                      IfCollision2 = 0;
		                    } // if
                      if (CurLink2For1 != *LastLink2For1)
                        {
		                      CurLink2For1 = *LastLink2For1;
		                      IfCollision1 = 0;
                        } // if
                    } // else
		            } // if
              else
		            {
                  IfCollision1 = IfCollision2 = 0;
                  InitialCurLink1 = CurLink1 = *LastLink1;
                  InitialCurLink2 = CurLink2 = *LastLink2;
                  CurLink1For2 = *LastLink1For2;
                  CurLink2For1 = *LastLink2For1;
                  ShowProjection (Robot1, Robot2,
                                  InitialCurLink1, InitialCurLink2, V11, V12);
                  ShowProjection (Robot2, Robot1,
                                  InitialCurLink2, InitialCurLink1, V22, V21);
                } // else
            } // if
        } // if
    } // for

} // Collision_Detect

/***************************************************************************/
// ShowThirdProjection projects the scene of two robots (Robot1 and Robot2)
// on a third plane which is computed based on the two projection planes of
// link LNo1 of Robot1, and link LNo2 of Robot2.
// Only links 0 through LNo1 of Robot1, and links 0 through LNo2 of Robot2
// are projected.
// The projection data for Robot1 is written in array V, and that of Robot2
// is written in array W.
/***************************************************************************/
void ShowThirdProjection (char LNo1, char LNo2, VType *V, VType *W)
{ // ShowThirdProjection

  char     No[2];
  frame    inv_map;
  register int i;
  VType    *VT;
  HBRUSH   hbr, hbrOld;
  RECT     rc;

  MakeViewMap (LNo1, LNo2, inv_map);
  No[1] = 0;
  if (IfShowProj)
    {
	    HwndMap = HwndView;
	    GetClientRect (HwndMap, &rc);
	    InvalidateRect (HwndMap, &rc, TRUE);
	    UpdateWindow (HwndMap);
	    hdc = GetDC (HwndMap);
	    hbr = CreateBrushIndirect (&logBrush);
	    SelectObject (hdc, hbr);
    } // if
  for (i = 0; i <= LNo1; i++) 
    Robot[Robot1]->compute_point (inv_map, i, XY, &V[i]);
  for (i = 0; i <= LNo2; i++)
    Robot[Robot2]->compute_point (inv_map, i, XY, &W[i]);

  if (IfShowProj)   
    {		   
	    No[0] = '0' + Robot1;
      TextOut (hdc, 200+(int)floor(V[0].X1*display_gain_x)*.8,
		                170-(int)floor(V[0].Y1*display_gain_y)*.8, No, 1);
	    No[0] = '0' + Robot2;
      TextOut (hdc, 200+(int)floor(W[0].X1*display_gain_x)*.8,
		                170-(int)floor(W[0].Y1*display_gain_y)*.8, No, 1);
    } // if
  if (IfShowProj)
    {
	    SelectObject (hdc, hbrOld);
      DeleteObject (hbr);
	    ReleaseDC (HwndMap, hdc);
	    EnableWindow (GetDlgItem (HwndPannel, ID_BACK), TRUE);
	    while (HwndMap != 0)
		    BackGround ();
	    GetClientRect (HwndView, &rc);
	    InvalidateRect (HwndView, &rc, TRUE);
	    UpdateWindow (HwndView);
    } // if
} // ShowThirdProjection

/***************************************************************************/
// ThirdPlane in a parallel fashion checks whether
//   1) there is any collision between link 0 through LastLink1 of Robot1 and
//     link 0 through LastLink2For1 of Robot2,
//   2) there is any collision between link0 through LastLink2 of Robot2 and
//     link0 through LastLink1For2 of Robot1.
// For both sets of the above checkings, a same projection plane is used
// If this projection plane does not help identifying no-collision between
// the two robots, 'Exact_Collision' is called to check for any collisions
// between the current links.
/***************************************************************************/
char Third_Plane (signed char *LastLink1, signed char *LastLink2,
		              signed char *LastLink2For1, signed char *LastLink1For2)
{ // Third_Plane

  char   IfCollision1, IfCollision2;
  signed  char    CurLink1, CurLink2, CurLink2For1, CurLink1For2;
  signed  char    InitialCurLink1, InitialCurLink2;
/*
  signed  char    InitialCurLink2For1, InitialCurLink1For2;
*/

  VType  V31[MaxLink], V32[MaxLink];

  IfCollision1 = IfCollision2 = 0;
  InitialCurLink1 = CurLink1 = *LastLink1;
// InitilCurLink1For2 = *LastLink1For2; 
  CurLink1For2 = *LastLink1For2;
  InitialCurLink2 = CurLink2 = *LastLink2;
// InitialCurLink2For1 = LastLink2For1;
  CurLink2For1 = *LastLink2For1;
  ShowThirdProjection (CurLink1, CurLink2, V31, V32);
  for (;;)
    {
      if (!IfCollision1)
        {
	        if (CurLink1 >= 0)
	          {
	            if (CurLink2For1 < 0)
		            {
		              CurLink2For1 = CurLink2;
		              --CurLink1;
		            } // if
	            if ((CurLink1 == -1) || (CurLink2For1 == -1))
		            IfCollision1 = 1;
	            else
		            {
		              if (Intersection[CurLink1][CurLink2For1] ||
                      !Cylinder (V31[CurLink1], V32[CurLink2For1]))
		                {
                    // First projection with linki of rob0 and linkj of rob1
		                  Intersection[CurLink1][CurLink2For1] = 1;
		                  --CurLink2For1;
                    } // if
                  else
	                  IfCollision1 = 1;
                } // else
            } // if
        } // if
      if (!IfCollision2)
        {
	        if (CurLink2 >= 0)
            {
	            if (CurLink1For2 < 0)
                {
		              CurLink1For2 = CurLink1;
		              --CurLink2;
		            }  // if
	            if ((CurLink2 == -1) || (CurLink1For2 == -1))
		            IfCollision2 = 1;
              else
                {
		              if (Intersection[CurLink1For2][CurLink2] ||
                      !Cylinder (V32[CurLink2], V31[CurLink1For2]))
		                {
                    // Second projection with linkl of rob0 and linkk of rob1
		                  Intersection [CurLink1For2][CurLink2] = 1;
		                  --CurLink1For2;
		                } // if
		              else
	                  IfCollision2 = 1;
		            } // else
	          } // if
	      } // if
      if ((CurLink1 <= -1) || (CurLink2 <= -1))
        return (0);
      if (IfCollision1 && IfCollision2)
	      {
	        if (CurLink2For1 > CurLink2)
            {
              CurLink2For1 = CurLink2;
	            IfCollision1 = 0;
            } // if
	        if (CurLink1For2 > CurLink1)
            {
              CurLink1For2 = CurLink1;
	            IfCollision2 = 0;
            } // if
          if (IfCollision1 && IfCollision2)
	          {
              *LastLink1 = CurLink1;
              *LastLink2 = CurLink2;
              *LastLink1For2 = CurLink1For2;
              *LastLink2For1 = CurLink2For1;
              if ((CurLink1 == InitialCurLink1) &&
                  (CurLink2 == InitialCurLink2)
/*                &&
                  (CurLink2For1 == InitilCurLink2For1) &&
                  (CurLink1For2 == InitialCurLink1For2)
*/                 )
                {
		              if (!(IfCollision1 = Exact_Collision (*LastLink1,
                                                        *LastLink2For1)))
                    {
		                  Intersection [CurLink1][CurLink2For1] = 1;
		                  *LastLink2For1 = --CurLink2For1;
                    } // if  
		              if (!(IfCollision2 = Exact_Collision (*LastLink1For2,
                                                        *LastLink2)))
                    {
                      Intersection [CurLink1For2][CurLink2] = 1;
                      *LastLink1For2 = --CurLink1For2;
                    } // if
                  if (IfCollision1 || IfCollision2)
                    return (1);
                } // if
              else
	              return (0);
            } // if
        } // if
    } // for

} // Third_Plane

/***************************************************************************/
// Cylinder checks whether the projections of two cylinders (V1 and V2)
// collide.
/***************************************************************************/
char Cylinder (VType V1, VType V2)
{ // Cylinder

  char    In_Seg_1 = 0, In_Seg_2 = 0;

  R1 = V1.Radius;
  R2 = V2.Radius;
  Dist = R1 + R2;
  X11 = V1.X1;  X12 = V1.X2;  Y11 = V1.Y1;  Y12 = V1.Y2;
  X21 = V2.X1;  X22 = V2.X2;  Y21 = V2.Y1;  Y22 = V2.Y2;

  DelX1 = X11 - X12;   DelY1 = Y11 - Y12;
  DelX2 = X21 - X22;   DelY2 = Y21 - Y22;

  P[0][0] = X12;  P[0][1] = Y12;
  P[1][0] = X11;  P[1][1] = Y11;
  P[2][0] = X22;  P[2][1] = Y22;
  P[3][0] = X21;  P[3][1] = Y21;

  if (((DelX1 == 0.0) && (DelY1 == 0.0)) && ((DelX2 == 0.0) && (DelY2 == 0.0)))
    {
      if (sqrt (pow (X11 - X22, 2) + pow (Y11 - Y22, 2)) <= Dist)
	      return (1);
      else
	      return (0);
    } // if
  if ((Delta = DelX1 * DelY2 - DelY1 * DelX2) == 0.0)  // Parallel
    {
      if (!((DelX2 == 0.0) && (DelY2 == 0.0)))
	      {
	        if (DelX2 < 0)
            Phi = atan2 (-DelX2, -DelY2);
          else
            Phi = atan2 (DelX2, DelY2);
          D1 = fabs ((Y11 - Y22) * cos (Phi) + (X22 - X11) * sin (Phi));
	      } // if
      else
        {
          if (DelX1 < 0)
            Phi = atan2 (-DelX1, -DelY1);
          else
            Phi = atan2 (DelX1, DelY1);
	          D1 = fabs ((Y22 - Y11) * cos (Phi) + (X11 - X22) * sin (Phi));
        } // else

      if (D1 >= Dist)
	      return (0);   // no intersection
      else
        {
          // rotate all points by -Phi
	        Phi = -Phi;
	        XVar = cos (Phi) * X21 - sin (Phi) * Y21;
          Y21 = sin (Phi) * X21 + cos (Phi) * Y21;
	        X21 = XVar;
	        XVar = cos (Phi) * X22 - sin (Phi) * Y22;
          Y22 = sin (Phi) * X22 + cos (Phi) * Y22;
	        X22 = XVar;
	        XVar = cos (Phi) * X12 - sin (Phi) * Y12;
	        Y12 = sin (Phi) * X12 + cos (Phi) * Y12;
	        X12 = XVar;
	        XVar = cos (Phi) * X11 - sin (Phi) * Y11;
          Y11 = sin (Phi) * X11 + cos (Phi) * Y11;
	        X11 = XVar;
	        if (X11 > X12)
	          {
	            XVar = X11;
              X11 = X12;
	            X12 = XVar;
            } // if
          if (X21 > X22)
	          {
	            XVar = X21;
              X21 = X22;
	            X22 = XVar;
            } // if
          X11 -= R1;
	        X12 += R1;
	        X21 -= R2;
	        X22 += R2;
          if (((X11 >= X21) && (X11 <= X22)) ||
              ((X12 >= X21) && (X12 <= X22)) ||
              ((X22 >= X11) && (X22 <= X12)) ||
	            ((X21 >= X11) && (X21 <= X12)) )
	          return (1);
	        else
	          return (0);
        } // else
    } // if
  else
    {
      DelX = X22 - X12;
      DelY = Y22 - Y12;
      T1 = (DelY2 * DelX - DelX2 * DelY) / Delta;
      T2 = (DelY1 * DelX - DelX1 * DelY) / Delta;
      XVar = T1 * X11 + (1 - T1) * X12;
      YVar = T1 * Y11 + (1 - T1) * Y12;
      if ((T1 <= 1.0) && (T1 >= 0))
	      In_Seg_1 = 1;
      if ((T2 <= 1.0) && (T2 >= 0))
	      In_Seg_2 = 1;
      if (In_Seg_1 && In_Seg_2)
        return (1);  // Intersected, but must check circles too
      else
	      {
	        L1 = sqrt (pow (DelX1, 2) + pow (DelY1, 2));
          L2 = sqrt (pow (DelX2, 2) + pow (DelY2, 2));
          if (!In_Seg_1)
            {
              if (T1 < 0)
                {
                  DelX1 = T1 * (X11 - X12);
		              DelY1 = T1 * (Y11 - Y12);
		              P[0][0] = X12;
		              P[0][1] = Y12;
                  P[1][0] = X11;
                  P[1][1] = Y11;
                } // if
	            else if (T1 > 1)
		            {
		              DelX1 = (T1 - 1) * (X11 - X12);
                  DelY1 = (T1 - 1) * (Y11 - Y12);
                  P[0][0] = X11;
                  P[0][1] = Y11;
                  P[1][0] = X12;
                  P[1][1] = Y12;
		            } // else
              D1 = sqrt (pow (DelX1, 2) + pow (DelY1, 2));
            } // if
          if (!In_Seg_2)
            {
              if (T2 < 0)
                {
		              DelX2 = T2 * (X21 - X22);
		              DelY2 = T2 * (Y21 - Y22);
		              P[2][0] = X22;
                  P[2][1] = Y22;
                  P[3][0] = X21;
                  P[3][1] = Y21;
                } // if
              else if (T2 > 1)
		            {
                  DelX2 = (T2 - 1) * (X21 - X22);
                  DelY2 = (T2 - 1) * (Y21 - Y22);
                  P[2][0] = X21;
                  P[2][1] = Y21;
                  P[3][0] = X22;
		              P[3][1] = Y22;
		            } // else
	            D2 = sqrt (pow (DelX2 , 2) + pow (DelY2 , 2));
            } // if
          if (!In_Seg_1 && !In_Seg_2)
            {
	            if (sqrt (pow (P[0][0] - P[2][0], 2) +
			            pow (P[0][1] - P[2][1], 2)) <= Dist)
		             return (1);
              Phi =((double)DelX1 * (double)DelX2 +
                    (double)DelY1 * (double)DelY2)/((double)D1 * (double)D2);
              if (fabs (Phi) > 1.0)
                Phi = 1.0 * Sign (Phi);
              Phi = acos (Phi);
	            if (Phi < M_PI_2)
                {
                  if (D1 * sin (Phi) <= Dist)
                    {
                      if (D1 * cos (Phi) <= D2)
                        return (0); // no intersection
		                  else if (D1 * cos (Phi) >= D2 + L2)
			                  {
			                    if (sqrt (pow (P[0][0] - P[3][0], 2) +
                                    pow (P[0][1] - P[3][1], 2)) <= Dist)
                            return (1); // intersection
                        } // else
		                  else
			                  return (1);
		                } // if
                  if (D2 * sin (Phi) <= Dist)
                    {
                      if (D2 * cos (Phi) <= D1)
                        return (0); // no intersection
                      else if (D2 * cos (Phi) >= D1 + L1)
			                  {
                          if (sqrt (pow (P[2][0] - P[1][0], 2) +
                                    pow (P[2][1] - P[1][1], 2)) <= Dist)
                            return (1); // intersection
                        } // else
                      else
			                  return (1);
		                } // if
		            } // if
              return (0);
            } // if
          else
            {
	            if (!In_Seg_1)
		            {
		              D2 = sqrt (pow (XVar - P[2][0], 2) +
                             pow (YVar - P[2][1], 2));
                  if (D2 != 0.0)
                    {
		                  Temp = ((XVar - P[0][0]) * (XVar - P[2][0]) +
			                        (YVar - P[0][1]) * (YVar - P[2][1])) /
                              (D1 * D2);
		                  if ((Phi = acos (Temp)) > M_PI_2) // concave
                        {
			                    D2 = sqrt (pow (XVar - P[3][0], 2) +
                                     pow (YVar - P[3][1], 2));
                          Phi = M_PI - Phi;
                        } // if
                    } // if
		              else
		                {
		                  D2 = sqrt (pow (XVar - P[3][0], 2) +
                                 pow (YVar - P[3][1], 2));
		                  Temp = ((XVar - P[0][0]) * (XVar - P[3][0]) +
			                        (YVar - P[0][1]) * (YVar - P[3][1])) /
                              (D1 * D2);
                      if ((Phi = acos (Temp)) > M_PI_2) // concave
                        Phi = M_PI - Phi;
		                  D2 = 0;
		                } // else
                  if (sqrt (pow (P[0][0] - P[2][0], 2) +
                            pow (P[0][1] - P[2][1], 2)) <= Dist)
                    return (1);
                  else if ((D1 * sin (Phi) <= Dist) &&
                           (D1 * cos (Phi) <= D2))
		                return (1);
                  else
                    return (0);
                } // if
              if (!In_Seg_2)
                {
		              D1 = sqrt (pow (XVar - P[0][0], 2) +
                             pow (YVar - P[0][1], 2));
		              if (D1 != 0.0)
		                {
		                  Temp = ((XVar - P[2][0]) * (XVar - P[0][0]) +
			                        (YVar - P[2][1]) * (YVar - P[0][1])) /
                              (D1 * D2);
                      if ((Phi = acos (Temp)) > M_PI_2) // concave
                        {
			                    D1 = sqrt (pow (XVar - P[1][0], 2) +
                                     pow (YVar - P[1][1], 2));
			                    Phi = M_PI - Phi;
                        } // if
                    } // if
                  else
                    {
		                  D1 = sqrt (pow (XVar - P[1][0], 2) +
                                 pow (YVar - P[1][1], 2));
		                  Temp = ((XVar - P[2][0]) * (XVar - P[1][0]) +
			                        (YVar - P[2][1]) * (YVar - P[1][1])) /
                              (D1 * D2);
                      if ((Phi = acos (Temp)) > M_PI_2) // concave
                        Phi = M_PI - Phi;
                      D1 = 0;
                    } // else
		              if (sqrt (pow (P[2][0] - P[0][0], 2) +
			                pow (P[2][1] - P[0][1], 2)) <= Dist)
		                return (1);
                  else if ((D2 * sin (Phi) <= Dist) &&
                           (D2 * cos (Phi) <= D1))
                    return (1);
                  else
		                return (0);
		            } // if
            } // else
        } // else
    } // else
   return (0);   // Dummy
} // Cylinder

/***************************************************************************/
// MakeViewMap finds the third projection plane for the two links, LNo1
// and LNo2 of Robot1 and Robot2. It computes the matrix 'map' which will be
// used to project the scene on the selected third-projection-plane. 
/***************************************************************************/
void MakeViewMap (char LNo1, char LNo2, frame map)
{ // MakeViewMap

  float    V1[4], V2[4], V3[4], Sum1, Sum2;
  char     Col1, Col2;
  register int i;

  // Must use another projection plane perpendicular
  // to the two previous ones.
  // If the two ones are parallel we use a projection
  // plane which is perpedicular to them and
  // the Seperating plane of the two robots.
  // if at the same time the two parallel planes are
  // parallel to the Seperating plane we use the
  // xy plane as the projection plane.
  // we have got to have another projection matrices
  // and a seperate collision-detect routine which
  // uses only one projection and V matrix

  Col1 = Robot[Robot1]->LinkArray[LNo1].AType;
  Col2 = Robot[Robot2]->LinkArray[LNo2].AType;
  for (i = 1; i < 4; i++)
    {
      V1[i] = Robot[Robot1]->map[LNo1][i][Col1];
      V2[i] = Robot[Robot2]->map[LNo2][i][Col2];
      map[i][1] = V1[i];
      map[i][4] = 0.0;
    } // for
  cross_product (V1, V2, V3);
  if ((V3[1]==0.0) && (V3[2]==0.0) && (V3[3]==0.0)) // parallel planes
    {
      for (Sum1 = 0, i = 1; i < 4; i++)
        {
	        V2[i] = Robot[Robot1]->map[0][i][4] - Robot[Robot2]->map[0][i][4];
          Sum1 += pow (V2[i], 2);
        } // for
      Sum1 = sqrt (Sum1);
      for (i = 1; i < 4; i++)
        V2[i] /= Sum1;
      cross_product (V1, V2, V3);
      if ((V3[1]==0.0) && (V3[2]==0.0) && (V3[3]==0.0)) // parallel planes
        { // use xy plane temporarily
          // shall we choose a vertical plane instead of a horizontal one?
	        V3[1] = V3[2] = 0.0;
	        V3[3] = 1.0;
	      } // if
    } // if
  cross_product(V3, V1, V2);
  for (Sum1 = Sum2 = 0.0, i = 1; i < 4; i++)
    {
      Sum1 += pow (V2[i], 2);
      Sum2 += pow (V3[i], 2);
    } // for
  Sum1 = sqrt (Sum1);
  Sum2 = sqrt (Sum2);
  for (i = 1; i < 4; i++)
    {
      V2[i] /= Sum1;
      V3[i] /= Sum2;
    } // for
  for (i = 1; i < 4; i++)
    {
      map[i][2] = V2[i];
      map[i][3] = V3[i];
    } // for

} // MakeViewMap

/***************************************************************************/
// Exact_Collision checks whether the two links LNo1 and LNo2 --of robots
// 1 and 2 respectively-- actually intersect. It first checks any
// intersection between the bounding boxes of the two links. If they are
// disjoint the the two links don't collide. Otherwise, 'VoxelCollision' is
// called to check any collisions between the voxel structures of the two
// links.
/***************************************************************************/
char Exact_Collision (char LNo1, char LNo2)
{ // Exact_Collision

  float MinX1 = 1000, MinY1 = 1000, MinZ1 = 1000,
	MinX2 = 1000, MinY2 = 1000, MinZ2 = 1000,
	MaxX1 = -1000, MaxY1 = -1000, MaxZ1 = -1000,
	MaxX2 = -1000, MaxY2 = -1000, MaxZ2 = -1000;
  float *Temp;
  register int i, j, k;
  int   *connection, index;

  for (k = 0; k < Robot[Robot1]->LinkArray[LNo1].NoPart; k++)
    for (i = 0; i < Robot[Robot1]->LinkArray[LNo1].Part[k].NoSurface; i++)
      {
	      connection = Robot[Robot1]->LinkArray[LNo1].Part[k].Surfaces[i];
	      for (j = 0; (index=connection[j])>0; j++)
          {
            Temp = &Robot[Robot1]->absolute_point[LNo1][index][0];
	          if (Temp[1] > MaxX1)
	            MaxX1 = Temp[1];
	          if (Temp[1] < MinX1)
	            MinX1 = Temp[1];
	          if (Temp[2] > MaxY1)
	            MaxY1 = Temp[2];
	          if (Temp[2] < MinY1)
	            MinY1 = Temp[2];
	          if (Temp[3] > MaxZ1)
	            MaxZ1 = Temp[3];
	          if (Temp[3] < MinZ1)
	            MinZ1 = Temp[3];
	        } // for
      } // for
  for (k = 0; k < Robot[Robot2]->LinkArray[LNo2].NoPart; k++)
    for (i = 0; i < Robot[Robot2]->LinkArray[LNo2].Part[k].NoSurface; i++)
      {
	      connection = Robot[Robot2]->LinkArray[LNo2].Part[k].Surfaces[i];
	      for (j = 0; (index=connection[j])>0; j++)
          {
            Temp = &Robot[Robot2]->absolute_point[LNo2][index][0];
	          if (Temp[1] > MaxX2)
	            MaxX2 = Temp[1];
	          if (Temp[1] < MinX2)
	            MinX2 = Temp[1];
	          if (Temp[2] > MaxY2)
	            MaxY2 = Temp[2];
	          if (Temp[2] < MinY2)
	            MinY2 = Temp[2];
	          if (Temp[3] > MaxZ2)
	            MaxZ2 = Temp[3];
	          if (Temp[3] < MinZ2)
	            MinZ2 = Temp[3];
          } // for
      } // for
    if ((MinX1 > MaxX2) || (MinX2 > MaxX1) ||
        (MinY1 > MaxY2) || (MinY2 > MaxY1) ||
        (MinZ1 > MaxZ2) || (MinZ2 > MaxZ1))
      {
	      Intersection[LNo1][LNo2] = 1;
	      return (0);
      } // if
    else
      return VoxelCollision (Robot1, Robot2, LNo1, LNo2);

} // Exact_Collision

/***************************************************************************/
// MultiCollision_Detect checks whether there are any collisions between
// any two pair of robots. 
/***************************************************************************/
int MultiCollision_Detect (void)
{
  register int i, j;
  int      IfCollision;

  signed char     C1, C2, C3, C4;

  IfCollision = 0;
  for (i = 0; i < MaxRobot; i++)
    for (j = i+1; j < MaxRobot; j++)
      {
	      C1 = C2 = C3 = C4 = MaxLink-1;  
	      SetIntersection ();
	      Robot1 = i;
	      Robot2 = j;
	      if (Collision_Detect(&C1, &C2, &C3, &C4))
          {
            return (1);
            IfCollision = 1;
          } // if
      } // for
  return (0);
  if (IfCollision)
    return 1;
  else
    return 0;
} // MultiCollision_Detect

/***************************************************************************/
/*                 cRobot-Specific Functions                               */
/*                                                                         */
/***************************************************************************/

/***************************************************************************/
// This member function of cRobot draws the axis of the cylinderical model
// of link 'link_num. It also draws two circles centered at the end points
// of the axis, with a radius such that the cylinder completely encompasses
// the actual polygonal link. The last two points in array 'absolute_point'
// of a link contain the coordinates of the two endpoints of the axis of
// that link.
/***************************************************************************/
void cRobot::compute_point (frame t, char link_num, char PType, VType *PC)
{ // cRobot::compute_point

  char  n;
  float sum;
  char  i, j, k;
  point vertice[2];


  n = max_point[link_num];
  for (i = n-1; i <= n; i++)
    for (j = 1; j < 4; j++)
      {
	      sum=0.0;
	      for (k=1;k<=4;k++) sum+=absolute_point[link_num][i][k]*t[j][k];
	      vertice[i-(n-1)][j]=sum;
      } // for
  PC->Radius = LinkArray[link_num].Radius;
  if (PType == XY)
    SetEnd (PC, vertice[0][1], vertice[0][2], vertice[1][1], vertice[1][2]);
  else if (PType == XZ)
    SetEnd (PC, vertice[0][1], vertice[0][3], vertice[1][1], vertice[1][3]);
  else
    SetEnd (PC, vertice[0][2], vertice[0][3], vertice[1][2], vertice[1][3]);
  if (IfShowProj)
    {
      SetColor (&LinkArray[link_num].Color);
      MoveTo (hdc, 200+(int)floor(PC->X1*display_gain_x)*.8,
	                 170-(int)floor(PC->Y1*display_gain_y)*.8);
      LineTo (hdc, 200+(int)floor(PC->X2*display_gain_x)*.8,
	                 170-(int)floor(PC->Y2*display_gain_y)*.8);
      Ellipse (hdc, 200+(int)floor((PC->X1 - PC->Radius)*display_gain_x)*.8,
	                  170-(int)floor((PC->Y1 - PC->Radius)*display_gain_y)*.8,
                    200+(int)floor((PC->X1 + PC->Radius)*display_gain_x)*.8,
	                  170-(int)floor((PC->Y1 + PC->Radius)*display_gain_y)*.8);
      Ellipse (hdc, 200+(int)floor((PC->X2 - PC->Radius)*display_gain_x)*.8,
	                  170-(int)floor((PC->Y2 - PC->Radius)*display_gain_y)*.8,
                    200+(int)floor((PC->X2 + PC->Radius)*display_gain_x)*.8,
	                  170-(int)floor((PC->Y2 + PC->Radius)*display_gain_y)*.8);
      ResetColor ();
    } // if
} // cRobot::compute_point

