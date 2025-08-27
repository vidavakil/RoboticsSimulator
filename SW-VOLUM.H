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

#define  V1          0
#define  V2          1
#define  F1          2
#define  F2          3
#define  P1          4
#define  P2          5
#define  Visit       6
#define  LastVertex  7
#define  Date        8

#define  PVisit      0
#define  e1          1
#define  e1V1        2
#define  e1V2        3
#define  e2          4
#define  e2V1        5
#define  e2V2        6
#define  x1          7
#define  y1          8
#define  z1          9
#define  Dx1         10
#define  Dy1         11
#define  Dz1         12
#define  Lastz1      13
#define  Just1       14
#define  x2          15
#define  y2          16
#define  z2          17
#define  Dx2         18
#define  Dy2         19
#define  Dz2         20
#define  Lastz2      21
#define  Just2       22

#define  NotVisited  0
#define  Visiting    1
#define  Visited     2
#define  Sign(X)     ((X>0)?1:-1)

typedef struct
  {
    int    VertNo;
    double Z;
  } SORTType;

typedef struct
  {
    int   Coord;
    int   Index;
  } BoxType;

typedef struct
  {
    char   Present;
    unsigned long FSeek;
  } PresentType;

class Voxel {
  private:
    LPBYTE      Volume;
    HGLOBAL     hglb;
    DCELType    Dcel;
    WORD        Select;
    int         Copy;
    int         IfUpdate;

  public  :
    int         RobNo;
    int         LNo;
    Point       Origin;
    Dimensions  Dim;
    unsigned long Size;
    int         IfInit;
    int         Height;
    char        Brush, BBrush;

    Voxel  (void) {IfInit = IfUpdate = Copy = 0; Volume = NULL; hglb = NULL; BBrush = 1;}
    void   Show (void);
    int    Update (void);
    void   Finalize (void);
    void   Save (HFILE Out);
    void   DrawGrid (HDC hdc);
    int    Read(HFILE In, int RobotNo, int L);
    int    Initialize (int RobotNo, int LkNo, int Status);
    friend VoxelIntersect (Voxel *Vx1, Voxel *Vx2);

  private :
    void InitDCEL (DCELType *DC);
    void FillVolume (void);
    void SortVertices (SORTType *Sorted);
		void CheckPlane (int Vert, int Plane, int E1, int E2);
		void ProcessPlane (int PlaneNo);
    void PreprocessPlane (int PlaneNo);
		void DrawPlane (int PlaneNo, int Edge);
    void Bres (Point B, Point A);
    void AddressTest (void);
    DWORD Address (Point AbsP);
    int  CCW (int Edge, int Center);
    int  ClockWise (int Edge, int Center);
    int  TestMem (unsigned long Size);
    void SetVoxel (double X, double Y);
    char GetVoxel (double X, double Y);
		void ScanFill (int MinX, int MaxX, int MinY, int MaxY);
    void Bres1 (double Xb, double Yb, double Xa, double Ya);
    void Bres2 (double Xb, double Yb, double Xa, double Ya);
    void BresAll (double Xb, double Yb, double Xa, double Ya);
    int  FindFirstEdge (int Vertex);
    int  FindAnotherEdge (int *E1, int E2, Point *Vertices, int Plane, int p);
};
