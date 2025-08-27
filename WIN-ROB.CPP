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

#define  STRICT
#include <windows.h>
#include <stdlib.h>
#include <process.h>
#include <string.h>
#include "winrob.h"
#include "robot.h"


extern cRobot *Robot[];
extern int    Count, Light;
extern int    InDisplay;
extern int    teach_pendant_mode;
//extern int    RXMin, RYMin, RXMax, RYMax;
extern int    IfNextCommand, active_page;
extern char   IfShowProj;
extern char   ProgramName[];
extern char   NextCommand[];

extern int    quit(void); 
extern void   Resume (void);
extern void   Robmain (void);
extern void   Present (void);
extern void   DrawStatus (void);
extern void   display_robot(void);
extern void   change_vision (WORD wParam);

BOOL    FAR CALLBACK _export PannelDialogProc (HWND hwnd, UINT iMessage,
                                               WPARAM wParam, LPARAM lParam);
BOOL    FAR CALLBACK _export CFGDialogProc(HWND hwnd, UINT iMessage,
                                           WPARAM wParam, LPARAM lParam);
BOOL    FAR CALLBACK _export TCHDialogProc(HWND hwnd, UINT iMessage,
                                           WPARAM wParam, LPARAM lParam);
LRESULT FAR CALLBACK _export ViewWndProc  (HWND hwnd, UINT iMessage,
                                           WPARAM wParam, LPARAM lParam);
LRESULT FAR CALLBACK _export WndProc      (HWND hWnd, UINT iMessage,
                                           WPARAM wParam, LPARAM lParam);

HWND HwndPannel = 0, HwndTeach = 0, HwndView = 0, HwndMap = 0;
HGLOBAL HGPannel, HGTeach, HGConfig;
DLGPROC TeachDlgPrc, PannelDlgPrc;
int     Back = 1;
int     Finished = 0;
int     ButtonEnable = 1;
int     RobNos[3];
int     Show_Mode = 0;
int     InBack = 0;

void ShowLight (void);
void BackGround (void);
void Paint( HWND Hwnd);
void RegisterView ( void );
void EnableButtons (int Enable);
void EnableAllButtons (BOOL Enable);

////////////////////////////////////////////////////////////////////////////

MainWindow *MainWnd;

////////////////////////////////////////////////////////////////////////////
HWND Window::hWnd = 0;
int  Main::nCmdShow = 0;
HINSTANCE Main::hInstance = 0;
HINSTANCE Main::hPrevInstance = 0;
char MainWindow::szClassName[] = "Robot Simulation Software";

////////////////////////////////////////////////////////////////////////////
// This is the message loop of this Windows software
////////////////////////////////////////////////////////////////////////////
int Main::MessageLoop( void )
{
  MSG msg;

  while( GetMessage( &msg, NULL, 0, 0 ))
    if ( !((HwndPannel && IsDialogMessage (HwndPannel, &msg)) ||
           (HwndTeach && IsDialogMessage (HwndTeach, &msg)) ))
      {
        TranslateMessage( &msg );
        DispatchMessage( &msg );
      }
  return msg.wParam;
} // Main::MessageLoop

////////////////////////////////////////////////////////////////////////////
// This member function creates the main window of this Windows software
////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow( void )
{                                                  
  RECT rc;
  WORD wWidth;

  GetWindowRect(GetDesktopWindow(), &rc);
  wWidth = rc.right - rc.left;
  hWnd = CreateWindow( szClassName,
                       szClassName,
                       WS_DLGFRAME | WS_MAXIMIZE | WS_VISIBLE |
                       WS_CLIPCHILDREN, 
                       0,                      // horizontal position
                       0,                      // vertical position
                       wWidth,                 // width
                       rc.bottom,              // height
                       NULL,                   
                       NULL,
                       Main::hInstance,
                       (LPSTR) this );
  if (! hWnd )
    exit( FALSE );
  Show( Main::nCmdShow );
  Update();
  OnCreate ();
} // MainWindow::MainWindow

////////////////////////////////////////////////////////////////////////////
// This member function registers the window class for the main window of
// this Windows software.
////////////////////////////////////////////////////////////////////////////
void MainWindow::Register( void )
{
  WNDCLASS wndclass;   

  wndclass.style         = 0; 
  wndclass.lpfnWndProc   = ::WndProc;
  wndclass.cbClsExtra    = 0;
  // Reserve extra bytes for each instance of the window;
  // we will use these bytes to store a pointer to the C++
  // (MainWindow) object corresponding to the window.
  // the size of a 'this' pointer depends on the memory model.
  wndclass.cbWndExtra    = sizeof( MainWindow * );
  wndclass.hInstance     = Main::hInstance;
  wndclass.hIcon         = LoadIcon( Main::hInstance, "whello" );
  wndclass.hCursor       = LoadCursor( NULL, IDC_ARROW );
  wndclass.hbrBackground = (HBRUSH)GetStockObject( WHITE_BRUSH );
  wndclass.lpszMenuName  = NULL;
  wndclass.lpszClassName = szClassName;

  if ( ! RegisterClass( &wndclass ) )
    exit( FALSE );
	RegisterView ();
} // MainWindow::Register

////////////////////////////////////////////////////////////////////////////
// After the main window is created, this member function is called to
// to create the following windows :
//   View, which shows the robots
//   Pannel, which shows the input and status boxes and holds the buttons
//           for inputting commands
// Then 'Robmain' is called to initialize the robots
////////////////////////////////////////////////////////////////////////////
void MainWindow::OnCreate (void)
{
  RECT rc;
  WORD wWidth;

  MessageBox (NULL, "Written by Vida Vakilotojar",
              "Robot Simulation Software",
              MB_ICONINFORMATION | MB_OK | MB_SYSTEMMODAL);

  StartConfig ();
  GetWindowRect(hWnd, &rc);
  wWidth = rc.right - rc.left;
  HwndView = CreateWindow( "View",
		                       "view",
                            WS_DLGFRAME | WS_CHILD | WS_VISIBLE,
                            200, 
                            0,
                            wWidth-200,
                            rc.bottom,
                            hWnd,
                            NULL,
                            Main::hInstance,
                            (LPSTR) this);
  ShowWindow (HwndView, SW_SHOW);
  GetClientRect (HwndView, &rc);
//  RXMin = rc.left; RXMax = rc.right; RYMin = rc.top; RYMax = rc.bottom;

//  StartConfig ();
  StartPannel ();
	Robmain ();
  SetFocus (GetDlgItem (HwndPannel, ID_EDIT));
} // MainWindow::OnCreate

////////////////////////////////////////////////////////////////////////////
// This member function activates the teach-pendant dialog box.
////////////////////////////////////////////////////////////////////////////
void MainWindow::StartTeach (void)
{
  MakeDialog (&TeachDlgPrc, (FARPROC) TCHDialogProc, "#2000", &HGTeach, &HwndTeach);
  SetFocus (GetDlgItem (HwndTeach, TCH_END));
  teach_pendant_mode = 1;
  EnableButtons (FALSE);
} // MainWindow::StartTeach

////////////////////////////////////////////////////////////////////////////
// This member function disactivates the teach-pendant dialog box.
////////////////////////////////////////////////////////////////////////////
void MainWindow::EndTeach (void)
{
  DestroyWindow (HwndTeach);
  FreeProcInstance ((FARPROC) TeachDlgPrc);
  FreeResource (HGTeach);
  teach_pendant_mode = 0;
  HwndTeach = 0;
  EnableButtons (TRUE);
  SetFocus (GetDlgItem (HwndPannel, ID_EDIT));
} // MainWindow::EndTeach

////////////////////////////////////////////////////////////////////////////
// This member function activates the pannel device. 
////////////////////////////////////////////////////////////////////////////
void MainWindow::StartPannel (void)
{
  MakeDialog (&PannelDlgPrc, (FARPROC) PannelDialogProc, "#1010", &HGPannel, &HwndPannel);
	EnableWindow (GetDlgItem (HwndPannel, ID_BACK), FALSE);
  ShowLight ();
}

////////////////////////////////////////////////////////////////////////////
// This member function disactivates the pannel device before the software
// is exited.
////////////////////////////////////////////////////////////////////////////
void MainWindow::EndPannel (void)
{
  DestroyWindow (HwndPannel);
  FreeProcInstance ((FARPROC) PannelDlgPrc);
  FreeResource (HGPannel);
} // MainWindow::EndPannel

////////////////////////////////////////////////////////////////////////////
// This member function creates a non-modal dialog box.
////////////////////////////////////////////////////////////////////////////
void MainWindow::MakeDialog (DLGPROC *dlgprc, FARPROC lpProc, LPCSTR lpszName,
                        HGLOBAL *hglb, HWND *hwnd)
{
  HRSRC FR;
  void FAR *FP;

  if ((*dlgprc = (DLGPROC) MakeProcInstance ((FARPROC) lpProc,
                                             Main::hInstance)) == NULL)
    MessageBox (hWnd, "FirstNull", NULL, MB_OK);
  if ((FR = FindResource (Main::hInstance, lpszName, "#5")) == NULL)
    MessageBox (hWnd, "SecondNull", NULL, MB_OK);
  if ((*hglb = LoadResource (Main::hInstance, FR)) == NULL)
    MessageBox (hWnd, "ThirdNull", NULL, MB_OK);
  if ((FP = LockResource (*hglb)) == NULL)
    MessageBox (hWnd, "FourthNull", NULL, MB_OK);
  if ((*hwnd = CreateDialogIndirect (Main::hInstance, FP, hWnd,
                                         (DLGPROC) *dlgprc)) == NULL)
    MessageBox (hWnd, "NULL", NULL, MB_OK);

} // MainWindow::MakeDialog

////////////////////////////////////////////////////////////////////////////
// This member function opens the configuration dialog boxe which asks
// the types of each of the robots of the workcell.
////////////////////////////////////////////////////////////////////////////
void MainWindow::StartConfig (void)
{
  HRSRC FR;
  DLGPROC ConfigDlgPrc;

  if ((ConfigDlgPrc = (DLGPROC) MakeProcInstance ((FARPROC) CFGDialogProc,
                                            Main::hInstance)) == NULL)
    MessageBox (hWnd, "FirstNull", NULL, MB_OK);

	if ((FR = FindResource (Main::hInstance, "#6000", "#5")) == NULL)
    MessageBox (hWnd, "SecondNull", NULL, MB_OK);
  if ((HGConfig = LoadResource (Main::hInstance, FR)) == NULL)
    MessageBox (hWnd, "ThirdNull", NULL, MB_OK);
  if (DialogBoxIndirect (Main::hInstance, HGConfig, hWnd, ConfigDlgPrc) == -1)
    MessageBeep (-1);
  FreeProcInstance ((FARPROC) ConfigDlgPrc);
  FreeResource (HGConfig);
} // MainWindow::StartConfig


////////////////////////////////////////////////////////////////////////////
// This member function is the message processor of the main window of this
// Windows software.
////////////////////////////////////////////////////////////////////////////
long MainWindow::WndProc( UINT iMessage, WPARAM wParam, LPARAM lParam )
{
  switch (iMessage)
    {
      case KILL_TEACH : EndTeach ();
                        return 0;

      case WM_DESTROY: EndPannel ();
                       PostQuitMessage( 0 );
                       return 0;

      case WM_CLOSE  :  if (MessageBox (MainWnd->GetHandle (),
                            "Really Want to Leave This Program?",
                            ProgramName, MB_YESNO | MB_DEFBUTTON1 |
                                         MB_ICONQUESTION) == IDYES)
                          {
                            if (HwndTeach)
                              EndTeach ();
                            Finished = 1;
//                            quit ();
                            DestroyWindow (MainWnd->GetHandle ());
                            return 0;
                          } // if
                        else
                          if (HwndTeach)
                            SetFocus (GetDlgItem (HwndTeach, TCH_END));
                          else
                            SetFocus (GetDlgItem (HwndPannel, ID_EDIT));
                        break;

      default: return DefWindowProc( hWnd, iMessage, wParam, lParam );
    } // switch
  return 0;
}

////////////////////////////////////////////////////////////////////////////
// This procedure registers the window class of the 'View' window.
////////////////////////////////////////////////////////////////////////////
void RegisterView( void )
{
  WNDCLASS wndclass;   

  wndclass.style         = CS_BYTEALIGNWINDOW;
  wndclass.lpfnWndProc   = ViewWndProc;
  wndclass.cbClsExtra    = 0;
  wndclass.cbWndExtra    = 0;
  wndclass.hInstance     = Main::hInstance;
  wndclass.hIcon         = LoadIcon( Main::hInstance, "whello" );
  wndclass.hCursor       = LoadCursor( NULL, IDC_ARROW );
  wndclass.hbrBackground = (HBRUSH)GetStockObject( WHITE_BRUSH );
  wndclass.lpszMenuName  = NULL;
  wndclass.lpszClassName = "View";

  if ( ! RegisterClass( &wndclass ) )
      exit( FALSE );
} // RegisterView

////////////////////////////////////////////////////////////////////////////
// This procedure changes the color of the collision signal accordingly.
////////////////////////////////////////////////////////////////////////////
void ShowLight (void)
{
  HBRUSH hbr, hbrOld;
  RECT   Rect;
  HDC    hdc;

  if (Light == 1)
    hbr = CreateSolidBrush (RGB(255, 0, 0));
  else
    hbr = CreateSolidBrush (RGB(0, 0, 255));
  hdc = GetDC (GetDlgItem (HwndPannel, ID_FLAG));
  hbrOld = (HBRUSH) SelectObject (hdc, hbr);
  GetClientRect (GetDlgItem (HwndPannel, ID_FLAG), &Rect);
  FillRect (hdc, &Rect, hbr);
  SelectObject (hdc, hbrOld);
  DeleteObject (hbr);
  ReleaseDC (GetDlgItem (HwndPannel, ID_FLAG), hdc);

}

////////////////////////////////////////////////////////////////////////////
// This procedure redraws the 'View' window by calling 'Present'.
////////////////////////////////////////////////////////////////////////////
void Paint( HWND Hwnd)
{
  PAINTSTRUCT ps;

  BeginPaint( Hwnd, &ps );
  Present ();
  EndPaint(Hwnd, &ps);
} // Paint

////////////////////////////////////////////////////////////////////////////
// This procedure is called to enable/disable any of the following Pannel's
// items the status of which can be changed: Input-box, Teach-pendant button,
// OK button, Exit button.
////////////////////////////////////////////////////////////////////////////
void EnableButtons (int Enable)
{
  EnableWindow (GetDlgItem (HwndPannel, ID_EDIT), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_TCH), Enable);
  EnableWindow (GetDlgItem (HwndPannel, ID_OK), Enable);
  EnableWindow (GetDlgItem (HwndPannel, ID_EXIT), Enable);
  ButtonEnable = Enable;
} // EnableButtons

////////////////////////////////////////////////////////////////////////////
// This procedure is called to enable/disable any of the Pannel's items
// the status of which can be changed. 
////////////////////////////////////////////////////////////////////////////
void EnableAllButtons (BOOL Enable)
{
	EnableWindow (GetDlgItem (HwndPannel, ID_DOWN), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_UP), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_LEFT), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_RIGHT), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_HOME), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_PRIOR), Enable);
	EnableWindow (GetDlgItem (HwndPannel, ID_NEXT), Enable);
} // EnableAllButtons

////////////////////////////////////////////////////////////////////////////
// This is the window message processor of the Pannel window. 
////////////////////////////////////////////////////////////////////////////
BOOL FAR CALLBACK PannelDialogProc (HWND hwndDlg, UINT msg, WPARAM wParam,
                                    LPARAM lParam)
{
  switch (msg)
    {

			case WM_CTLCOLOR : if (Count)
                           {
  												   DrawStatus ();
                             ShowLight ();
                           } // if
			                   break;

      case WM_COMMAND :
        switch (wParam)
          {
						case ID_DETECT  : if (HIWORD (lParam) == BN_CLICKED)
															  {
                                  IfShowProj = (IfShowProj) ? 0 : 1;
																  return 0;
															  } // if
														  break;

						case ID_BACK  : if (HIWORD (lParam) == BN_CLICKED)
															{
																HwndMap = 0;
													      EnableWindow (GetDlgItem (HwndPannel,
                                              ID_BACK), FALSE);
																return 0;
															} // if
														break;

            case ID_EXIT : if (HIWORD (lParam) == BN_CLICKED)
                             PostMessage (MainWnd->GetHandle (), WM_CLOSE,
                                          wParam, lParam);
                           break;

            case ID_OK   : if (HIWORD (lParam) == BN_CLICKED)
                             {
                               NextCommand[GetWindowText (GetDlgItem
                                           (hwndDlg, ID_EDIT), NextCommand,
                                           80)] = 0;
                               IfNextCommand = 1;
                               SetWindowText(GetDlgItem (hwndDlg, ID_EDIT),"");
                               EnableButtons (FALSE);
                               Resume ();
                               EnableButtons (TRUE);
                               if (!Show_Mode)
                                 SetFocus (GetDlgItem (hwndDlg, ID_EDIT));
                               else
                                 Show_Mode = 0;
                               return 0;
                             } // if
                           break;

            case ID_TCH  : if (HIWORD (lParam) == BN_CLICKED)
                             {
                               MainWnd->StartTeach ();
                               return 0;
                             } // if
                           else
                             break;

            case ID_LEFT :
            case ID_RIGHT:
            case ID_UP   :
            case ID_DOWN :
            case ID_HOME :
            case ID_PRIOR:
            case ID_NEXT : if (HIWORD (lParam) == BN_CLICKED)
                             {
                               change_vision (wParam);
                               if (!InDisplay)
                                 display_robot ();
                               return 0;
                             } // if
                           else
                             break;
            default      :;
          } // switch
        break;
      default: ;
    } // switch
  return 0; 

} // PannelDialogProc

////////////////////////////////////////////////////////////////////////////
// This is the window message processor of the Teach-Pendant window.
////////////////////////////////////////////////////////////////////////////
BOOL FAR CALLBACK TCHDialogProc (HWND hwndDlg, UINT msg, WPARAM wParam,
                                 LPARAM lParam)
{
  static int RobNo = 0;

  switch (msg)
    {
      case WM_INITDIALOG :  // enable proper number of robots radio buttons
                           SendDlgItemMessage (hwndDlg, ID_ROB1, BM_SETCHECK,
                                               TRUE, 0);
                           RobNo = 0;
                           return TRUE;

      case WM_COMMAND    :
                           if (HIWORD (lParam) == BN_CLICKED)
                             switch (wParam)
                               {
                                 case TCH_END : SendMessage (MainWnd->GetHandle(),
                                                             KILL_TEACH, 0, 0);
                                                return 0;
                                 case ID_ROB1 : RobNo = 0; break;
                                 case ID_ROB2 : RobNo = 1; break;
                                 case ID_ROB3 : RobNo = 2; break;
                                 default      : Robot[RobNo]->tp (wParam);
                                                SetFocus (GetDlgItem (hwndDlg,
                                                          TCH_END));
                                                return 0;
                               } // if
                             break;
      default: ;
    } // switch
  return 0;   
} // TCHDialogProc

////////////////////////////////////////////////////////////////////////////
// This is the window message processor of the robot-configuration window.
////////////////////////////////////////////////////////////////////////////
BOOL FAR CALLBACK CFGDialogProc (HWND hwndDlg, UINT msg, WPARAM wParam,
                                 LPARAM lParam)
{
  switch (msg)
    {
      case WM_INITDIALOG :  // enable proper number of robots radio buttons
                           RobNos[0] = RobNos[1] = RobNos[2] = _ATLAS;
                           SendDlgItemMessage (hwndDlg, ID_ATLAS1, BM_SETCHECK,
                                               TRUE, 0);
                           SendDlgItemMessage (hwndDlg, ID_ATLAS2, BM_SETCHECK,
                                               TRUE, 0);
                           SendDlgItemMessage (hwndDlg, ID_ATLAS3, BM_SETCHECK,
                                               TRUE, 0);
                           return 1;

      case WM_COMMAND    :
                           if (HIWORD (lParam) == BN_CLICKED)
                             switch (wParam)
                               {
                                 case ID_OK   : EndDialog (hwndDlg, 0);
                                                return 1;
                                 case ID_RHINO1 : RobNos[0] = _RHINO; return 1;
                                 case ID_RHINO2 : RobNos[1] = _RHINO; return 1;
                                 case ID_RHINO3 : RobNos[2] = _RHINO; return 1;
                                 case ID_ATLAS1 : RobNos[0] = _ATLAS; return 1;
                                 case ID_ATLAS2 : RobNos[1] = _ATLAS; return 1;
                                 case ID_ATLAS3 : RobNos[2] = _ATLAS; return 1;
                                 default        :;
                               } // if
                             break;
      default: ;
    } // switch
//  return DefDlgProc( hwndDlg, msg, wParam, lParam );
  return 0;
} // TCHDialogProc

////////////////////////////////////////////////////////////////////////////
// This is the window message processor of the View window.
////////////////////////////////////////////////////////////////////////////
LRESULT CALLBACK ViewWndProc (HWND hwnd, UINT iMessage, WPARAM wParam,
                              LPARAM lParam)
{
  switch (iMessage)
    {
			case WM_PAINT  : if (Count && !HwndMap)
                         {
                           Paint(hwnd);
                           return 0;
                         } // if
                       else
                         break;

      case WM_DESTROY: PostQuitMessage( 0 );
                       return 0;
                       break;

      default        :;
    } // switch

  return DefWindowProc( hwnd, iMessage, wParam, lParam );
} // ViewWndProc

////////////////////////////////////////////////////////////////////////////
// This procedure handles any Windows' events during heavy-load computation
// phases of the software. 
////////////////////////////////////////////////////////////////////////////
void BackGround (void)
{
  MSG msg;

  if (InBack)
    return;
  if (Finished)
    exit(0);
  if (!Back)
    return;
  InBack = 1;
  if (PeekMessage (&msg, NULL, 0, 0, PM_REMOVE))
/*    if ( !((HwndPannel && IsDialogMessage (HwndPannel, &msg)) ||
           (HwndTeach && IsDialogMessage (HwndTeach, &msg)) ))*/
      {
        TranslateMessage (&msg);
        DispatchMessage (&msg);
      } // if
  InBack = 0;
} // BackGround

////////////////////////////////////////////////////////////////////////////
inline Window *GetPointer( HWND hWnd )
{
  return (Window *) GetWindowLong( hWnd, 0 );
} // GetPointer

////////////////////////////////////////////////////////////////////////////
inline void SetPointer( HWND hWnd, Window *pWindow )
{
  SetWindowLong( hWnd, 0, (LONG) pWindow );
} // SetPointer

////////////////////////////////////////////////////////////////////////////
LRESULT CALLBACK _export WndProc( HWND hWnd, UINT iMessage, WPARAM wParam,
                                 LPARAM lParam )
{
  Window *pWindow = GetPointer( hWnd );

  if ( pWindow == 0 )
    {
      if ( iMessage == WM_CREATE )
        {
          LPCREATESTRUCT lpcs;

          lpcs = (LPCREATESTRUCT) lParam;
          pWindow = (Window *) lpcs->lpCreateParams;
          SetPointer( hWnd, pWindow );
	        return pWindow->WndProc( iMessage, wParam, lParam );
        } // if
      else
        return DefWindowProc( hWnd, iMessage, wParam, lParam );
    } // if
  else
    return pWindow->WndProc( iMessage, wParam, lParam );
} // WndProc

////////////////////////////////////////////////////////////////////////////
#pragma argsused
#pragma option -w-aus

////////////////////////////////////////////////////////////////////////////
// This is the start point (main) of this Windows software!
////////////////////////////////////////////////////////////////////////////
int PASCAL WinMain( HINSTANCE hInstance, HINSTANCE hPrevInstance,
                    LPSTR lpszCmdLine, int nCmdShow )
{
	Main::nCmdShow = nCmdShow;
  Main::hInstance = hInstance;
  Main::hPrevInstance = hPrevInstance;
  HINSTANCE hinstBWCC;

	int len;
	int RetVal;
	char szBuf[80];
	DWORD dwVersion;

  hinstBWCC = LoadLibrary("BWCC.DLL");
	if ((UINT) hinstBWCC <= 32)
		{
			MessageBox (NULL, "Error in Loading BWCC Lib",
									"Robot Simulation Software", MB_OK | MB_ICONEXCLAMATION);
		  exit (0);
		} // if
	dwVersion = GetVersion();
	if (HIBYTE(LOWORD(dwVersion)) != 10)
    {
			MessageBox(NULL, "Must Be Run Under Windows Version 3.1",
								 "Robot Simulation Software", MB_ICONEXCLAMATION | MB_OK);

//			exit (0);
    } // if
  SetMessageQueue (30);
  if ( ! Main::hPrevInstance )
    MainWindow::Register();
  MainWnd = (MainWindow *) new MainWindow();
	RetVal = Main::MessageLoop();
	if ((UINT) hinstBWCC > 32)
		FreeLibrary(hinstBWCC);
  Finished = 1;
  return RetVal;
} // WinMain
