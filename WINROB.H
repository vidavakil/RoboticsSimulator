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
#ifndef  WINROBH
#define  WINROBH


#define ID_EDIT	                1000
#define ID_EDIT_STATIC	        1001
#define ID_STATUS               1002
#define ID_FLAG         				1003
#define ID_DETECT         			1004
#define ID_OK	                	1005
#define ID_BACK               	1006
#define MyDialog	        			1010
#define ID_EXIT	                1013
#define ID_DOWN	                1014
#define ID_UP	                	1015
#define ID_LEFT	                1016
#define ID_RIGHT	        			1017
#define ID_HOME	                1018
#define ID_TCH	                1019
#define ID_PRIOR	        			1020
#define ID_NEXT	                1021
#define TCH_END                 1090
#define ID_ROB1                 1035
#define ID_ROB2                 1036
#define ID_ROB3                 1037
#define SHOW_END                102
#define ID_PATH                 101
#define KILL_TEACH	        		(WM_USER+1)
#define KILL_SHOW 	        		(WM_USER+2)
#define ID_ATLAS1               132
#define ID_ATLAS2               142
#define ID_ATLAS3               152
#define ID_RHINO1               131
#define ID_RHINO2               141
#define ID_RHINO3               151
////////////////////////////////////////////////////////////////////////////
class Main
{
  public:
    static HINSTANCE hInstance;
    static HINSTANCE hPrevInstance;
    static int nCmdShow;
    static int MessageLoop( void );
};

////////////////////////////////////////////////////////////////////////////
class Window
{
protected:
    static HWND hWnd, InputWin, StatusWin, MessageWin, InVisible;
public:
    HWND GetHandle( void ) { return hWnd; }
    BOOL Show( int nCmdShow ) { return ShowWindow( hWnd, nCmdShow ); }
    void Update( void ) { UpdateWindow( hWnd ); }
    virtual long WndProc( UINT iMessage, WPARAM wParam, LPARAM lParam ) = 0;
};


////////////////////////////////////////////////////////////////////////////
class MainWindow : public Window
{
  private:
    static char szClassName[30];
  public:
    MainWindow( void );
    void Size (void);
    void OnCreate (void);
    void MakeDialog (DLGPROC *dlgprc, FARPROC lpProc, LPCSTR lpszNmae, HGLOBAL *hglb, HWND *hwnd);
    void StartPannel (void), EndPannel (void);
    void StartTeach (void), EndTeach (void);
    void StartConfig(void), EndConfig (void);
    void StartShow (void), EndShow (void);
    static void Register( void );
		long WndProc( UINT iMessage, WPARAM wParam, LPARAM lParam );
};
#endif
////////////////////////////////////////////////////////////////////////////