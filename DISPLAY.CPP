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

#include <alloc.h>
#include <bios.h>
#include <conio.h>
#include <ctype.h>
#include <dos.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "robot.h"

#define ZeroP      0.0 
#define ZeroPQ     0.0 
#define First      0
#define Extent     (9*MaxRobot) // Notice

#define ID_DOWN	   1014
#define ID_UP	     1015
#define ID_LEFT	   1016
#define ID_RIGHT	 1017
#define ID_HOME	   1018
#define ID_END	   1019
#define ID_PRIOR	 1020
#define ID_NEXT	   1021
#define ID_STATUS  1002

HDC              hdc;
HBRUSH           hbr, hbrOld;
HPEN             hpen, hpenOld;
RECT             Rigeon;

int              Parts = 0;
int              Light = 0;
int   		       teach_pendant_mode=0;
int              InDisplay = 0;
int              next_surface, NextColor;
int              drawn[Extent], Sequence[Extent][2];
char             SortTable[Extent+1][Extent+1];
struct           projected_coordinates Pr[70*MaxRobot][10];
visible_surface  surfaces[70*MaxRobot];

float Inter[2][10];
float display_gain_x=4.3-MaxRobot;
float display_gain_y=4.3-MaxRobot;
float vision_vector[4]={-1,1,0,0};
float vision_matrix [3][4] = {{-1,-1,-1,-1},
                              {-1,0,1,0},
                              {-1,0,0,1}};

void Draw1 (void);
void Draw2 (void);
void Present (void);
void ShowBase (void);
void Alarm (int Flag);
void DrawStatus (void);
void ResetColor (void);
void ResetBrush (void);
void ProcessRow (int row);
void display_robot (void);
int  Order (int S1, int S2);
void SetColor (RGBColor *C);
void SetBrush (RGBColor *C);
void change_vision (WORD key);
void draw_surface (visible_surface *a);
float inner_product (float *a,float *b);
void compute_points (frame t,int link_num);
int  InRow (int row, int status, int column);
void frame_multiply (frame a,frame b,frame c);
float *cross_product (float *a,float *b,float *c);
void compute_normal (visible_surface *a, float *v);
void project (point p,struct projected_coordinates *pc);
void project2d (point p,struct projected_coordinates *pc);
int  SortPolygon (visible_surface *a, visible_surface *b);
int  Is_Inside (struct projected_coordinates P1[],
	              struct projected_coordinates P2[], int N1, int N2);
int  EveryOtherSide (struct projected_coordinates P1[],
		                 struct projected_coordinates P2[], int N1, int N2);
void draw_poly (visible_surface *a, struct projected_coordinates P1[],
                int *N1, float *MaxX, float *MinX, float *MaxY, float *MinY);

extern  void ShowLight (void);
extern  void BackGround (void);
extern  void SetInterest (void);
extern  void VShow(int RobotNo);
extern  void message (char *str);
extern  void Vinit (int RobotNo);
extern  void VFinilize (int RobotNo);
extern  void update_maps(float *joint);
extern  void Collision_Detect(int *MaxLink1, int *MaxLink2, int *Maxj, int *Maxl);


extern   int  InBack;
extern   int  Hide;
extern   int  Count;
extern   int  Recursion;
extern   int  IfShowProj;
extern   int  OnLineTest;
extern   cRobot huge *Robot[];
extern   HWND HwndView, HwndPannel;

/***************************************************************************/
/* The following functions either create and select a new Pen/Brush for    */
/* the device context of the scene, or select the old Pen/Brush and delete */
/* the new one.                                                            */
/***************************************************************************/
void SetColor (RGBColor *C)
{
  hpen = CreatePen(PS_SOLID, 1, RGB(C->R, C->G, C->B));
  hpenOld = SelectObject(hdc, hpen);
} // SetColor

/***************************************************************************/
void ResetColor (void)
{
  SelectObject(hdc, hpenOld);
  DeleteObject(hpen);
} // ResetColor

/***************************************************************************/
void SetBrush (RGBColor *C)
{
  hbr = CreateSolidBrush (RGB (C->R, C->G, C->B));
  hbrOld = SelectObject (hdc, hbr);
} // SetBrush

/***************************************************************************/
void ResetBrush (void)
{
  SelectObject(hdc, hbrOld);
  DeleteObject(hbr);
} // ResetBrush

/***************************************************************************/
/* FillPoly draws and fills a polygon with 'numpoints' vertices, whose     */
/* coordinates are given in the array 'p'.                                 */
/***************************************************************************/
void far FillPoly (int numpoints, struct projected_coordinates p[])
{
  POINT    pp[MaxVertex];
  register int i;

  for (i = 0; i < numpoints; i++)
     {
       pp[i].x = p[i].x;
       pp[i].y = p[i].y;
     }
  Polygon(hdc, pp, numpoints);
} // FillPoly

/***************************************************************************/
/* Draw1 determines the order of drawing of the convex polyhedral objects  */
/* which constitute the scene of the workcell. Each link of a robot is     */
/* modeled by a number of convex polyhedral objects. If the links do not   */
/* intersect, the order can be determined easily!                          */
/* 'SortTable' is a two dimensional array, with its dimensions being equal */
/* to the maximum number of convex polyhedrals of the scene. Each cell     */
/* (i,j) of 'SortTable' indicates the position of objext(i) relative to    */
/* object(j) as the observer views. If Object(i) has to be drawn before    */
/* object(j), then SortTable(i,j) is marked as 'B'; if it has to be drawn  */
/* after object(j), then SortTable(i,j) is marked as 'F', and if the order */
/* of drawing is not important, then the cell is marked as 'X'.            */
/* It is not necessary to check the order of drawing for every pair of     */
/* objects, because we are dealing with an 'ordered relation', e.g. if     */
/* SortTable(i,j)='B' and SortTable(j,k)='B' then SortTable(i,k)='B'.      */
/* Therefore filling each cell of SortTable will help us fill even more    */
/* cells of it.                                                            */
/* SortTable is filled by calling 'ProcessRow' for each row of it.         */
/* Then until all the objects are drawn, the first undrawn object j whose  */
/* row does not have any 'F', i.e. no other undrawn object is behind it,   */
/* is drawn and marked as so. Then all the elements of row(j) and          */
/* column(j) of SortTable are marked as 'X' to let the process go on.      */
/* Each object has a number of visible facets which have already been      */
/* determined and placed in array 'surfaces'. When the next object to be   */
/* drawn is determined, the address of the first facet of it in 'surfaces' */
/* is put in the next location of an array called 'Sequence'. This latter  */
/* array is used in the routine 'Draw2' to draw the complete scene.        */
/***************************************************************************/
void Draw1 (void)
{
  register int i, j, k, found;

  for (i = First; i < Extent; i++)
    {
      drawn [i] = 0;
      for (j = First; j < Extent; j++)
        SortTable[i][j] = ' ';
      SortTable[i][j] = 0;
    } // for
  for (i = First; i < Extent; i++)
    ProcessRow (i);

  Parts = 0;
  for (i = First; i < Extent; i++)
    {
    for (j = First; j < Extent; j++)
      {
        if (!drawn [j])
          {
            for (found = 0, k = First; k < Extent; k++)
              {
                if (SortTable[j][k] == 'F')
                  {
                    found = 1;
                    break;
                  } /* if */
              } /* for */
            if (!found)
              {
                drawn[j] = 1;
                for (k = 0; (k <next_surface) &&
                            (surfaces[k].PartNumber != j); k++);
                if (k < next_surface)
                  {
										Sequence[Parts][0] = k;
										Sequence[Parts][1] = j;
										Parts++;
                  } // if
                for (k = First; k < Extent; k++)
                  SortTable[k][j] = SortTable[j][k] = 'X';
                break;
              } /* if */
          } /* if */
      } /* for */
    } // for
} // Draw1

/***************************************************************************/
/* Draw2 uses the information put in array 'Sequence' to draw the objects  */
/* of the scene from the furthest to the closest one relative to the viwer.*/
/* 'Draw1' puts the total number of visible objects in the variable        */
/* 'Parts'. The first column of Sequence[i] indicates the address of the   */
/* first facet of object(i) in array 'Surfaces'. All the facets of this    */
/* object are drawn consecutively by calling 'draw_surface'.               */
/***************************************************************************/
void Draw2 (void)
{
  register int i, k;

  ShowBase ();
  for (i = First; i < Parts; i++)
    for (k = Sequence[i][0]; (k <next_surface) &&
                             (surfaces[k].PartNumber == Sequence[i][1]); k++)
      draw_surface(&surfaces[k]);
} // Draw2

/***************************************************************************/
/* Initially 'SortTable' is filled with all blanks. Then in each call of   */
/* ProcessRow for each row of 'SortTable', every column of that row is     */
/* checked for being blank. Any blank cell is filled by calling 'Order'.   */
/* When cell(row,i) is filled, cell(i,row) is found to be the inverse of   */
/* it. This added information can help filling more cells of 'SortTable'   */
/* by using the properties of the 'ordered relation'. This is done by      */
/* iteratively calling 'InRow' for the corresponding row and column.       */
/* (See 'InRow').                                                          */
/***************************************************************************/
void ProcessRow (int row)
{
  register int i;
  for (i = First; i < Extent; i++)
    {
      if ((i != row) && (SortTable[row][i] == ' '))
	      {
	        SortTable[row][i] = Order (row, i);
	        if (SortTable[row][i] != 'X')
	          {
	            SortTable[i][row] = (SortTable[row][i] == 'B') ? 'F' : 'B';
	            while (InRow (row, SortTable[i][row], i) +
		                 InRow (i, SortTable[row][i], row));
	          } /* if */
	        else
	          SortTable[i][row] = 'X';
	      } /* if */
    } /* for */
} /* ProcessRow */

/***************************************************************************/
/* Assume that SortTable(row,column)=~status, where status='B' or 'F', and */
/* '~' indicates negation. As an ordered relation implies, if for some i,  */
/* SortTable(i,row)=SortTable(row,column)=~status, then                    */
/* SortTable(i,column)=~status, and SortTable(column,i)=status. If any     */
/* such i is found, InRow will be called recursively to fill even more     */
/* cells of SortTable. This is what is implied by the following recursive  */
/* function of InRow.                                                      */
/***************************************************************************/
int InRow (int row, int status, int column)
{
  register int i, Ret;
  Ret = 0;
  for (i = First; i < Extent; i++)
    if ((i != row) && (SortTable[row][i] == status) &&
	      ((SortTable[i][column] == 'X') || (SortTable[i][column] == ' ')) )
      {
	      Ret = 1;
	      SortTable[i][column] = SortTable[row][column];
	      SortTable[column][i] = status;
	      while (InRow(i, status, column) +
               InRow(column, SortTable[row][column], i));
      } /* if */
  return (Ret);
} /* InRow */

/***************************************************************************/
/* Order determines the drawing order of two objects S1 and S2 relative to */
/* the observer. At first, the first facet of each object in the array     */
/* 'Surfaces' is found. Then for each pair of facets of the two objects,   */
/* the drawing order of the facets relative to the observer is computed by */
/* calling 'SortPolygon'. If any of such calls reveals any significant     */
/* order (i.e. 'B' or 'F'), this order is the same drawing order of the    */
/* two objects. This property is due to the convexity of the objects.      */
/***************************************************************************/
int Order (int S1, int S2)
{
  register int i, j, k, Ret;
  Ret = 'X';

  for (i = 0; i < next_surface; i++)
    if (surfaces[i].PartNumber == S1) break;
  if (i == next_surface)
    return ('B'); 
  for (j = 0; j < next_surface; j++)
    if (surfaces[j].PartNumber == S2) break;
  if (j == next_surface)
		return ('F');
  for (; (i < next_surface) && (surfaces[i].PartNumber == S1); i++)
    {
      for (k = j; (k < next_surface) && (surfaces[k].PartNumber == S2); k++)
	      if ((Ret = SortPolygon (&surfaces[i], &surfaces[k])) != 'X')
	        return (Ret);
    } /* for */
  return (Ret);
} /* Order */

/***************************************************************************/
// draw_poly copies the projected coordinates of the vertices of a visible
// surface, a, into P1. The Max and Min coordinates of this surface, and
// also its number of coordinates are extracted from the given structure
/***************************************************************************/
void draw_poly (visible_surface *a, struct projected_coordinates P1[],
                int *N1, float *MaxX, float *MinX, float *MaxY, float *MinY)
{
  register int i;

  for (i = 0; i <= a->N; i++)
    {
      P1[i].x = Pr[a->SNo][i].x;
      P1[i].y = Pr[a->SNo][i].y;
    } // for
  *MinX = a->MinX;
  *MinY = a->MinY;
  *MaxX = a->MaxX;
  *MaxY = a->MaxY;
  *N1 = a->N;
} /* draw_poly */

/***************************************************************************/
// Distance determines the relative distances of two polygons in 3D space,
// with respect to the viewer. If polygon 'a' is infront of polygon 'b',
// then a positive number is returned. If 'a' is behind 'b' then a negative
// number is returned. Otherwise the returned value will be zero.
/***************************************************************************/
float Distance (visible_surface *a, visible_surface *b)
{
  register int i, j, index;
  float P1[4], P2[4], Ret1, Ret2;

  compute_normal (b, P2); // Since b is visible, its normal vector points
                          // towards the viewr
  for (i = 1; i < 4; i++)
    P1[i] = Robot[b->RobNo]->absolute_point[b->link_num][b->connection[0]][i];
  Ret2 = inner_product (P1, P2);   /* -d in the eqution of surface b */
  for (i = 1; (index = a->connection[i]) != -1; i++)
    {
      for (j = 1; j < 4; j++)
	      P1[j] = Robot[a->RobNo]->absolute_point[a->link_num][index][j];
      if (fabs ((Ret1 = inner_product (P1, P2) - Ret2)) > ZeroP) // > was >=
	      return (Ret1);
    } /* for */
  return (0);
} /* Distance */

/***************************************************************************/
// Is_Inside takes two non-intersecting 2D polygons as input and
// determines whether the first one (P1) is inside the second one (P2).
// If so, it returns 1. Otherwise it returns 0.
// Is_Inside first finds an interior point of P1. Then checks whether this
// point is in inside P2. Since all the polygons are convex, an interior
// point is computed by taking the average of the vertices of P1. 
/***************************************************************************/
int Is_Inside (struct projected_coordinates P1[],
	             struct projected_coordinates P2[], int N1, int N2)
{
  register int i;
  float P0[4], P3[4], P4[4], P5[4];

  for (i = 1; i < 4; i++)
    P0[i] = 0;
  for (i = 0; i < N1; i++)
    {
      P0[1] += P1[i].x;
      P0[2] += P1[i].y;
    } /* for */
  P0[1] /= N1;
  P0[2] /= N1;
  for (i = 0; i < N2; i++)
    {
      P3[1] = P2[i+1].x - P2[i].x;
      P3[2] = P2[i+1].y - P2[i].y;
      if ((P3[1] == 0.0) && (P3[2] == 0.0))
	      continue;
      P3[3] = 0.0;
      P4[1] = P0[1] - P2[i].x;
      P4[2] = P0[2] - P2[i].y;
      P4[3] = 0.0;
      cross_product (P3, P4, P5);
      if (fabs (P5[3]) <= ZeroPQ) 
	      continue;
      if (P5[3] > 0.0)
	      return (0);
    } /* for */
  return (1);
} /* Is_Inside */

/***************************************************************************/
// EveryOtherSide determines whether its two input polygons (P1 and P2) each
// completely lie on one side of a line which connects two intersection
// points of the two polygons. If the polygons lie on the same side then
// 1 is returned, if they lie in different sides of the line, then -1 is
// returned. If any of the polygons intersects with the connecting line
// then 0 is returned.
/***************************************************************************/
int EveryOtherSide (struct projected_coordinates P1[],
		                struct projected_coordinates P2[], int N1, int N2)
{
  float P3[4], P4[4], P5[4], P6[4], Product1, Product2;
  register int i;

  P3[1] = Inter[0][3] - Inter[1][3];
  P3[2] = Inter[0][4] - Inter[1][4];
  P3[3] = 0;
  P4[3] = 0;
  P6[1] = (Inter[0][3] + Inter[1][3]) / 2;
  P6[2] = (Inter[0][4] + Inter[1][4]) / 2;
  for (i = 0, Product1 = 0.0; i < N1; i++)
    {
      P4[1] = P6[1] - P1[i].x;
      P4[2] = P6[2] - P1[i].y;
      cross_product (P3, P4, P5);
      if (fabs (P5[3]) <= ZeroPQ)        
	      continue;
      if (Product1 == 0.0)
	      Product1 = (P5[3] > 0.0) ? 1.0 : -1.0;
      else if ((Product1 * P5[3]) < 0.0)
	      return (0);
    } /* for */
  for (i = 0, Product2 = 0.0; i < N2; i++)
    {
      P4[1] = P6[1] - P2[i].x;
      P4[2] = P6[2] - P2[i].y;
      cross_product (P3, P4, P5);
      if (fabs (P5[3]) <= ZeroPQ)
	      continue;
      if (Product2 == 0.0)
	      Product2 = (P5[3] > 0.0) ? 1.0 : -1.0;
      else if ((Product2 * P5[3]) < 0.0)
	      return (0);
    } /* for */
  return (Product1*Product2);
} /* EveryOtherSide */

/***************************************************************************/
// SortPolygon determines which of its two input 3D polygons (a and b)
// should be drawn first. It first computes the projected coordintes of the
// vertices of the polygons on the viewing plane. 
// 1- If the bounding boxes of the two projections are disjoint, then the
//    order of drawing is immaterial (a 'X' is returned).
// 2- Two intersecting points of the two projections are tried to be found.
// 3- If there is no intersection, then one of the projections can be inside
//    the other one only if its bounding box is equal to the intersection of
//    the two corresponding bounding boxes. If so, by calling 'Distance' the
//    closer polygon is determined.
// 4- If two intersection points are found, and both of the projections are
//    found to be completely on one side of the line connecting the two
//    intersection points (by calling 'EveryOtherSide'), then either they
//    are on different sides, and therefor the order of drawing is immaterial,
//    or they are on the same side. In the latter case, another intersection
//    point is tried to be found.
// 5- If no more intersection points is found, then the two projections
//    colapse, having an edge in common. The containment of each of them in
//    the other one is checked and the proper order of drawing is determined
//    as in 3.
// 6- If two intersecting points are found which don't satisfy the properties
//    of 4, then the middle point of the line which connects this two points
//    is computed. This point is the projection of two points, each of which
//    lies on yet another one of the original 3D polygons. These points are
//    computed and the one closer to the viewer is determined, thus the
//    proper order of drawing the polygons is found.
/***************************************************************************/
int SortPolygon (visible_surface *a, visible_surface *b)
{
  register    int i, j;
  int         N1, N2, No, OverWrite = 0;
  struct      projected_coordinates P1[MaxVertex], P2[MaxVertex];
  float       XMin, XMax, YMin, YMax,
	            XMin1, XMax1, YMin1, YMax1, XMin2, XMax2, YMin2, YMax2,
              X, Y, P3[4], P4[4], P5[4], P6[4],
	            *ptr1, *ptr2, *ptr3, *ptr4, VTemp, Dist,
	            DelX1, DelX2, DelY1, DelY2, T1, T2, DelX, DelY, Delta;

  draw_poly(a, P1, &N1, &XMax1, &XMin1, &YMax1, &YMin1);
  draw_poly(b ,P2, &N2, &XMax2, &XMin2, &YMax2, &YMin2);

  XMin = max (XMin1, XMin2);
  XMax = min (XMax1, XMax2);
  YMin = max (YMin1, YMin2);
  YMax = min (YMax1, YMax2);

  if ((XMin >= XMax) || (YMin >= YMax))
    return ('X');
  No = 0;
  for (i = 0; i < N1; i++)
    {
      DelX1 = P1[i].x - P1[i+1].x;
      DelY1 = P1[i].y - P1[i+1].y;
      for (j = 0; j < N2; j++)
	      {
	        DelX2 = P2[j].x - P2[j+1].x;
	        DelY2 = P2[j].y - P2[j+1].y;
	        if ((Delta = DelX1 * DelY2 - DelY1 * DelX2) == 0.0)  // Parallel
	          continue;
	        else
	          {
	            DelX = P2[j+1].x - P1[i+1].x;
	            DelY = P2[j+1].y - P1[i+1].y;
	            T1 = (DelY2 * DelX - DelX2 * DelY) / Delta;
	            T2 = (DelY1 * DelX - DelX1 * DelY) / Delta;
	            X = T1 * P1[i].x + (1 - T1) * P1[i+1].x;
	            Y = T1 * P1[i].y + (1 - T1) * P1[i+1].y;
	            if ((T1 <= 1.0) && (T1 >= 0) && (T2 <= 1.0) && (T2 >= 0))
		            {
		              Inter[No][3] = X;
		              Inter[No][4] = Y;
		              Inter[No][0] = i;
		              Inter[No][2] = j;
		              Inter[No][1] = 1-T1;
		              Inter[No][5] = 1-T2;
		              if ((No == 1) &&
		                  (fabs(Inter[1][3] - Inter[0][3]) <= ZeroP) &&
                      (fabs(Inter[1][4] - Inter[0][4]) <= ZeroP))
		                continue;
		              No++;
		              if (No == 2)
		                {
		                  if (VTemp = EveryOtherSide (P1, P2, N1, N2))
			                  {
			                    if (VTemp == -1.0)
			                      return ('X');
			                    else
			                      {
			                        OverWrite = 1;
			                        No = 1;
			                      } /* else */
			                  } /* if */
		                  else
			                  break;
		                } /* if */
		            } /* if */
	            else
                continue;
	          } // else
        } /* for */
      if (No == 2)
        break;
    } /* for */
  if (No <= 1)
    {
      if ((XMin == XMin1) && (XMax == XMax1) &&
          (YMin == YMin1) && (YMax == YMax1))
	      {  /* P1 may be inside P2 */
	        if (Is_Inside (P1, P2, N1, N2))
	          {
	            if ((Dist = Distance(a, b)) > 0.0)
		            return ('F');
	            else if (Dist < 0.0)
		            return ('B');
	            else return ('X');
	          } /* if */
	        else
	          return ('X');
	      } // if
      if ((XMin == XMin2) && (XMax == XMax2) &&
          (YMin == YMin2) && (YMax == YMax2))
	      {  /* P2 may be inside P1 */
	        if (Is_Inside (P2, P1, N2, N1))
	          {
	            if ((Dist = Distance (b, a)) > 0.0)
	              return ('B');
	            else if (Dist < 0.0)
		            return ('F');
	            else return ('X');
	          } /* if */
	        else
	          return ('X');
	      } /* if */
      else
        return ('X');
    } /* if */

  else if (No == 2)
    {
      if (EveryOtherSide (P1, P2, N1, N2))
	      return ('X');
      i = Inter[0][0];
      ptr1 = Robot[a->RobNo]->absolute_point[a->link_num][a->connection[i]];
      ptr2 = Robot[a->RobNo]->absolute_point[a->link_num][a->connection[i+1]];
      P3[1] = ptr1[1] - (ptr1[1] - ptr2[1]) * Inter[0][1];
      P3[2] = ptr1[2] - (ptr1[2] - ptr2[2]) * Inter[0][1];
      P3[3] = ptr1[3] - (ptr1[3] - ptr2[3]) * Inter[0][1];
      i = Inter[1][0];
      ptr3 = Robot[a->RobNo]->absolute_point[a->link_num][a->connection[i]];
      ptr4 = Robot[a->RobNo]->absolute_point[a->link_num][a->connection[i+1]];
      P4[1] = ptr3[1] - (ptr3[1] - ptr4[1]) * Inter[1][1];
      P4[2] = ptr3[2] - (ptr3[2] - ptr4[2]) * Inter[1][1];
      P4[3] = ptr3[3] - (ptr3[3] - ptr4[3]) * Inter[1][1];
      for (i = 1; i < 4; i++)
	    P5[i] = (P3[i]+P4[i])/2;

      i = Inter[0][2];
      ptr1 = Robot[b->RobNo]->absolute_point[b->link_num][b->connection[i]];
      ptr2 = Robot[b->RobNo]->absolute_point[b->link_num][b->connection[i+1]];
      P3[1] = ptr1[1] - (ptr1[1] - ptr2[1]) * Inter[0][5];
      P3[2] = ptr1[2] - (ptr1[2] - ptr2[2]) * Inter[0][5];
      P3[3] = ptr1[3] - (ptr1[3] - ptr2[3]) * Inter[0][5];
      i = Inter[1][2];
      ptr3 = Robot[b->RobNo]->absolute_point[b->link_num][b->connection[i]];
      ptr4 = Robot[b->RobNo]->absolute_point[b->link_num][b->connection[i+1]];
      P4[1] = ptr3[1] - (ptr3[1] - ptr4[1]) * Inter[1][5];
      P4[2] = ptr3[2] - (ptr3[2] - ptr4[2]) * Inter[1][5];
      P4[3] = ptr3[3] - (ptr3[3] - ptr4[3]) * Inter[1][5];
      for (i = 1; i < 4; i++)
	    P6[i] = (P3[i]+P4[i])/2;
      if ((Dist = inner_product (P5, vision_vector) - inner_product (P6, vision_vector)) < 0.0)
	      return ('B');
      else if (Dist > 0.0)
	      return ('F');
      else return ('X'); /* new */
    } /* else */
  else
    return ('X');
} /* SortPolygon */

/***************************************************************************/
// display_robot draws the scene of the workcell.
// It first computes the new coordinates of the vertices of any robot which
// has recently moved (by calling 'update_points').
// Then for each robot, the projection of the robot's vertices on the view
// plan , and aslo the visible surfaces are computed by calling 'draw_link'.
// By calling 'Draw1', the order of drawing visible surface are determined.
// Then by invalidating the viewing window, the window procedure of the
// viewing window will call 'present' which in return calls 'Draw2' to
// actualy update the scene.
/***************************************************************************/
void display_robot (void)
{
  MSG   msg;
  register int link_num, j;

  InDisplay = 1;
  if (!InBack)
    BackGround (); // notice

  next_surface = 0;
  NextColor = 0;

  for (j = 0; j < MaxRobot; j++)
    {
      Robot[j]->update_points(Robot[j]->theta,Robot[j]->theta[dof+1],Robot[j]->t);
      for (link_num=0; link_num<dof+2; link_num++)
        Robot[j]->draw_link(link_num);
    } /* for */

  if (!Hide)
    {
      Draw1 ();
      InvalidateRect (HwndView, NULL /*&Rigeon*/, FALSE/*TRUE*/);
			UpdateWindow (HwndView);
    } // if

  InDisplay = 0;
/*
  if (OnLineTest)
    {
      if (MultiCollision_Detect ())
        Alarm (1);
      else
        Alarm (0);
    } // if

  for (j = 0; j < MaxRobot; j++)
    VUpdate (j);
*/
} // display_robot

/***************************************************************************/
// This procedure is called by the viewing window's message processor when
// it needs to be redrawn. The bitmap of the scene is first constructed in
// memory by calling 'Draw2', and then selected to the window's device
// context. Finally the status window which shows current joint values and 
// cartesian coordinates is redrawn by calling 'DrawStatus'.
/***************************************************************************/
void Present (void)
{
  HBITMAP hbmpOld, hbmpNew;
  HDC     Oldhdc;
  RECT    Rect;
  HBRUSH  hbr;

  if (Count)
    {
      Oldhdc = GetDC (HwndView);

      GetClientRect (HwndView, &Rect);

      hdc = CreateCompatibleDC (Oldhdc);
      hbmpNew = CreateCompatibleBitmap (Oldhdc, Rect.right, Rect.bottom);
      hbmpOld = SelectObject (hdc, hbmpNew); 

      hbr = (HBRUSH) GetStockObject (WHITE_BRUSH);
      FillRect (hdc, &Rect, hbr);

      Draw2 ();

      BitBlt (Oldhdc, 0, 0, Rect.right, Rect.bottom, hdc, 0, 0, SRCCOPY);


      SelectObject (hdc, hbmpOld);
      DeleteObject (hbmpNew);
      DeleteDC (hdc);

			ReleaseDC (HwndView, Oldhdc);

			DrawStatus ();
      Count = 1;
    } // if
} // Present

/***************************************************************************/
// DrawStatus updates the status window which shows the joint values and
// cartesian coordinates of the robots tool-tips.
/***************************************************************************/
void DrawStatus (void)
{
  register int i;
  PAINTSTRUCT  ps;
  char     temp1[20];
  RECT     Rect;
  HBRUSH   hbr, hbrOld;

  if (Count)
    {
      hdc = GetDC (GetDlgItem (HwndPannel, ID_STATUS));
/*
      hbr = GetStockObject (WHITE_BRUSH);
      hbrOld = SelectObject (hdc, hbr);
      GetClientRect (GetDlgItem (HwndPannel, ID_STATUS), &Rect);
      FillRect (hdc, &Rect, hbr);
      SelectObject (hdc, hbrOld);
      DeleteObject (hbr);
*/
      for (i = 0; i < MaxRobot; i++) 
	      Robot[i]->display_frame(Robot[i]->t, Robot[i]->theta);
      for (i=1; i <= dof+1; i++)     
        {
          sprintf (temp1,"j%d", i);
          TextOut (hdc, 5, 20+i*15, temp1, strlen (temp1));
        } // for         
      for (i=1; i < 4; i++)
        {
          sprintf(temp1, "%c ", 'w'+i);
          TextOut (hdc, 5, 20+(i+dof+1)*15, temp1, strlen (temp1));
        } // for         
      ReleaseDC (GetDlgItem (HwndPannel, ID_STATUS), hdc);
    } // if
} // DrawStatus

/***************************************************************************/
// frame_multiply multiplies two coordinate transformation matrices 'a' and
// 'b', that is c=a*b                        
/***************************************************************************/
void frame_multiply (frame a,frame b,frame c)
{
  frame t;
  float sum;
  register int i,j,k;

  for (i=1;i<=3;i++) // 3 was 4
    {
      for (j=1;j<=4;j++) 
        {
	        sum=0.0;
	        for (k=1;k<=3;k++) sum+=a[i][k]*b[k][j]; // 3 was 4
	        t[i][j]=sum;
        } // for
      t[i][4]+=a[i][4];
    } // for
  for (i=1;i<=3;i++) // 3 was 4
    for (j=1;j<=4;j++)
      c[i][j]=t[i][j];
} // frame_multiply

/***************************************************************************/
// compute_normal computes the normal vector 'v' of a surface 'a'. 
/***************************************************************************/
void compute_normal (visible_surface *a, float *v)
{
  register int i;
  float v1[4],v2[4],*p0,*p1,*p2;

  p0=Robot[a->RobNo]->absolute_point[a->link_num][a->connection[0]];
  p1=Robot[a->RobNo]->absolute_point[a->link_num][a->connection[1]];
  p2=Robot[a->RobNo]->absolute_point[a->link_num][a->connection[2]];

  for (i=1;i<4;i++)
    {
      v1[i]=p1[i]-p0[i];
      v2[i]=p2[i]-p1[i];
    } // for
  cross_product(v1,v2,v);
} // compute_normal

/***************************************************************************/
// cross_product computes the cross product of two vectors 'a' and 'b',
// that is c=a*b                               
/***************************************************************************/
float *cross_product (float *a,float *b,float *c)
{
  c[1]=a[2]*b[3]-a[3]*b[2];
  c[2]=a[3]*b[1]-a[1]*b[3];
  c[3]=a[1]*b[2]-a[2]*b[1];

  return c;
} // cross_product

/***************************************************************************/
// inner_product computes and returns the inner product of two vectors 'a'
// and 'b'.                               
/***************************************************************************/
float inner_product (float *a,float *b)
{
  return (a[1]*b[1]+a[2]*b[2]+a[3]*b[3]);
} // inner_product

/***************************************************************************/
// draw_surface draws a visible surface 'a', by first projecting it on the
// viewing scene, with the specified zooming factor, and then drawing and
// filling the resulted polygon.                                           
/***************************************************************************/
void draw_surface (visible_surface *a)
{
  point *vertice;
  register int i,index;
  RGBColor C = {0,0,0};
  struct projected_coordinates p[MaxVertex];

  SetColor (&C);
  SetBrush (&Robot[a->RobNo]->LinkArray[a->link_num].Color);
  vertice=Robot[a->RobNo]->absolute_point[a->link_num];
  for (i = 0; (index=a->connection[i])>0; i++)
	  project2d(vertice[index],&p[i]);
  FillPoly(i, p);
  ResetColor ();
  ResetBrush ();
} // draw_surface

/***************************************************************************/
// project2d projects a point in 3d space 'p' onto the viewing plane,
// applying the zooming factors for X and Y coordiantes and aslo the required
// offsets. 
/***************************************************************************/
void project2d (point p,struct projected_coordinates *pc)
{
  float sum;
  register int i;

  for (i=1,sum=0.0;i<=3;i++) sum+=vision_matrix[1][i]*p[i];
  pc->x=220+(int)floor(sum*display_gain_x);

  for (i=1,sum=0.0;i<=3;i++) sum+=vision_matrix[2][i]*p[i];
  pc->y=250-(int)floor(sum*display_gain_y);
} // project2d

/***************************************************************************/
// project computes the projected coordinates of a point 'p' in 3D space
// onto the viewing plane.
/***************************************************************************/
void project (point p,struct projected_coordinates *pc)
{
  float sum;
  register int i;

  for (i=1,sum=0.0;i<=3;i++) sum+=vision_matrix[1][i]*p[i];
  pc->x= sum;

  for (i=1,sum=0.0;i<=3;i++) sum+=vision_matrix[2][i]*p[i];
  pc->y= - sum;
} // project

/***************************************************************************/
// change_vision takes the input value 'key' and changes the viewing
// direction and zooming factor accordingly. Then it recomputed the
// vision vector and vision projection matrix.
/***************************************************************************/
void change_vision (WORD key)
{
  int RNo, LNo;
  float c1,c2,s1,s2;
  static float phi_max=360, phi=0.0, theta=90, delta_phi=18, delta_theta=18,
               PHI, THETA;

    switch (key)
      {
	      case ID_PRIOR : display_gain_x*=1.1; /* page up */
		                    display_gain_y*=1.1;
		                    break;

	      case ID_NEXT : display_gain_x/=1.1;  /* page dn */
		                   display_gain_y/=1.1;
		                   break;

	      case ID_HOME : phi=0;                /* home  */
		                   theta=90;
		                   break;

	      case ID_LEFT : phi-=delta_phi;       /* left  */
		                 if (phi < 0.0)
                       phi+=phi_max;
		                 break;

	      case ID_RIGHT : phi+=delta_phi;      /* right */
		                    if (phi>phi_max)
                          phi-=phi_max;
		                    break;


	      case ID_UP : if (theta>0.0)          /* up    */
                       theta-=delta_theta;
		                 break;


	      case ID_DOWN : if (theta < 180)
                         theta+=delta_theta; /* down  */
		                   break;

	      default : return;
      } // switch
  PHI = phi * DEG_TO_RAD;
  THETA = theta * DEG_TO_RAD;
  c1=cos(PHI);   s1=sin(PHI);
  c2=cos(THETA); s2=sin(THETA);

  vision_vector[1]=c1*s2;
  vision_vector[2]=s1*s2;
  vision_vector[3]=c2;

  vision_matrix[1][1]=-s1   ;
  vision_matrix[1][2]= c1   ;
  vision_matrix[2][1]=-c1*c2;
  vision_matrix[2][2]=-s1*c2;
  vision_matrix[2][3]=    s2;
} // change_vision

/***************************************************************************/
// ShowBase draws the coordinate axes on the screen.
/***************************************************************************/
void ShowBase (void)
{
  point  V[4];
  char   Str[2];
  register int i, j;
  RGBColor C = {0,0,0};
  projected_coordinates VP[4];

  Str[1] = 0;
  for (i = 0; i < 4; i++)
    {
      for (j = 1; j < 4; j++)
        V[i][j] = 0.0;
      V[i][i] = 15.0;
      project2d (V[i], &VP[i]);
    } // for
  SetColor(&C);
  for (Str[0] = 'x', i = 1; i < 4; i++, Str[0]++)
    {
      MoveTo (hdc, VP[0].x, VP[0].y);
      LineTo (hdc,VP[i].x, VP[i].y);
      TextOut(hdc, VP[i].x, VP[i].y, Str, strlen (Str));
    } // for
  ResetColor ();
} // ShowBase

/***************************************************************************/
// Alarm changes the color of the light for collision-detect. Red indicates
// a collision and Blue indicates no collision.
/***************************************************************************/
void Alarm (int Flag)
{
  if (Flag)
    Light = 1;
  else
    Light = 0;
  ShowLight ();
}
/***************************************************************************/
/*                 cRobot-Specific Functions                               */
/*                                                                         */
/***************************************************************************/

/***************************************************************************/
// specify computes the projected coordinates of the vertices of a visible
// polygon 'a', putting them in 'P1'. It also computes the Max and Min
// projected coordinate values of the vertices.
/***************************************************************************/
void cRobot::specify (visible_surface *a, struct projected_coordinates P1[])
{
  float    Temp;
  point *vertice;
  register int i, index;

  vertice=absolute_point[a->link_num];
  for (i = 0; (index=a->connection[i])>0; i++)
	project(vertice[index],&P1[i]);
  a->N = i-1;

  a->MinX = a->MaxX = P1[0].x;
  a->MinY = a->MaxY = P1[0].y;

  a->PXMin = a->PXMax = a->PYMin = a->PYMax = 0;
  for (i = 0; i < a->N; i++)
    {
      if ((Temp = P1[i].x) > a->MaxX)
	      {
	        a->MaxX = Temp;
	        a->PXMax = i;
	      } // if
      else if (Temp < a->MinX)
	      {
	        a->MinX = Temp;
	        a->PXMin = i;
	      } // else
      if ((Temp = P1[i].y) > a->MaxY)
	      {
	        a->MaxY = Temp;
	        a->PYMax = i;
	      } // if
      else if (Temp < a->MinY)
	      {
	        a->MinY = Temp;
	        a->PYMin = i;
	      } // else
    } /* for */
} /* specify */

/***************************************************************************/
// update_points computes new coordinates of the vertices of a robot's links
// relative to the base, using the joint vector, and tool variable value.
// This procedure computes all of the transformation matrices of the
// corresponding robot (map[]) and also inverse transformation matrices
// (inv_map[]). map[i] is used to transform the points on link i to world
// coordinates, while inv_map[i] is used to transform the coordinates of a
// point in the world coordinate system to the local frame of link i.
// inv_map[i] is used to project the entire scene on to the projection plane
// of link i.
/***************************************************************************/
void cRobot::update_points (float *joint, float tv, frame t)
{
  register int i,j;

  if (!correction && Count)  // notice, new
    return;

  for (i=1;i<=4;i++)
    for (j=1;j<=4;j++)
      t[i][j]=(i==j);

  compute_end_device(tv);
  update_maps(joint);
  for (i=0;i<dof+2;i++)
    {  
      frame_multiply(t,map[i],t);
      compute_points(t,i);
      update_inv_map(t,i);
    } // for
  correction = 0;
} // cRobot::update_points

/***************************************************************************/
// compute_points multiplies all point coordinates of link 'link_num' by a
// transformation matrix 't'.
/***************************************************************************/
void cRobot::compute_points (frame t,int link_num)
{
  int n;
  float sum;
  register int i,j,k;
  point *vertice1,*vertice2;

  n=max_point[link_num];
  vertice1=relative_point[link_num];
  vertice2=absolute_point[link_num];

  for (i=1;i<=n;i++)
    for (j=1;j<4;j++)
      {
	      sum=0.0;
	      for (k=1;k<=3;k++) sum+=vertice1[i][k]*t[j][k];  // 3 was 4
	      vertice2[i][j]=sum+t[j][4];
      } // for
} // cRobot::compute_points

/***************************************************************************/
// draw_link adds to the array of visible surfaces ('surface') each surface
// of link 'link_num' which is visible. All the relevant information is
// copied into 'surface' and by calling 'specify', the projected polygon
// and also its bounding box are computed. 'next_surface' keeps the number
// of visible surfaces of the array 'surface'.
/***************************************************************************/
void cRobot::draw_link (int link_num)
{
  int *connection;
  register int i, k;

  for (k = 0; k < LinkArray[link_num].NoPart; k++, NextColor++)
    {
      for (i = 0; i < LinkArray[link_num].Part[k].NoSurface; i++)
	      {
	        connection = LinkArray[link_num].Part[k].Surfaces[i];
	          if (surface_in_sight(link_num,connection))
	            {
		            surfaces[next_surface].RobNo = RobNo;
		            surfaces[next_surface].PartNumber = NextColor;
		            surfaces[next_surface].link_num = link_num;
		            surfaces[next_surface].connection = connection;
		            surfaces[next_surface].SNo= next_surface;
		            specify(&surfaces[next_surface], Pr[next_surface]);
		            next_surface++;
	            } /* if */
	      } /* for */
    } /* for */
} // cRobot::draw_link

/***************************************************************************/
// surface_in_sight checks whether a surface is visible or hidden. If the
// normal vector of a surface is pointing towards the viewer then the
// surface is visible. The norml vector is computed by the cross-product of
// two edges of the polygon surface. The inner-product of the normal-vector
// and the vision-vector indicates the direction of the normal-vector.                               
/***************************************************************************/
int cRobot::surface_in_sight (int link_num,int *connection)
{
  register int i;
  float v1[4],v2[4],v[4],*p0,*p1,*p2;

  p0=absolute_point[link_num][connection[0]];
  p1=absolute_point[link_num][connection[1]];
  p2=absolute_point[link_num][connection[2]];

  for (i=1;i<4;i++)
    {
      v1[i]=p1[i]-p0[i];
      v2[i]=p2[i]-p1[i];
    } // for
  cross_product(v1,v2,v);
  return (inner_product(v,vision_vector)>0.0);
} // cRobot::surface_in_sight

/***************************************************************************/
// display_frame draws the tool_tip coordinates and joint values of a robot
// on the status window.
/***************************************************************************/
void cRobot::display_frame (frame t,float *joint)
{
  register int i;
  int   Interval;
  char  temp1[20];
  RECT  Rect;

  if (MaxRobot == 1)
    Interval = 0;
  else
    Interval = 100/(MaxRobot - 1); 
  sprintf (temp1, "Rob%d", RobNo);
  TextOut(hdc, 25+RobNo*Interval, 10, temp1, strlen (temp1));

  SetRect (&Rect, 30+RobNo*Interval, 35, 30+(RobNo+1)*Interval-1, 49);
  for (i=1; i <= dof+1; i++)         
    {
      sprintf (temp1,"%- 8.3g  ",
		           joint[i]/((joint_status[i]==REVOLUTE) ? DEG_TO_RAD : 1.0));
      DrawText (hdc, temp1, -1, &Rect, DT_SINGLELINE | DT_LEFT);
      Rect.top += 15;
      Rect.bottom += 15; 
//      TextOut (hdc, 30+RobNo*Interval, 20+i*15, temp1, strlen (temp1));
    } // for                         
  for (i=1; i < 4; i++)
    {
      sprintf(temp1, "%- 8.3g  ", t[i][4]);
      DrawText (hdc, temp1, -1, &Rect, DT_SINGLELINE | DT_LEFT);
      Rect.top += 15;
      Rect.bottom += 15; 
//      TextOut (hdc, 30+RobNo*Interval, 20+(i+dof+1)*15, temp1, strlen (temp1));
    } // for                             

} // cRobot::display_frame

/***************************************************************************/
// update_inv_map computes the transformation matrix which must be used
// to transform the coordinates of a point in the world-coordinate system to
// its coordinate in the coordinate system of link 'link_num'.
// This is done by computing the inverse matrix of the matrix which performs
// the inverse task, that is 't'.
/***************************************************************************/
void cRobot::update_inv_map (frame t,int link_num)
{
	float sum;
  register int j, k;

  for (j = 1; j <= 3; j++) // 3 was 4
    for (k = 1 ; k <= 3; k++) // 3 was 4
      inv_map[link_num][j][k] = t[k][j];
  for (j = 1; j < 4; j++)
    {
      for (sum = 0,k = 1; k < 4; k++)
	    sum += inv_map[link_num][j][k] * t[k][4];
      inv_map[link_num][j][4] = -sum;
    } // for
} // cRobot::update_inv_map

