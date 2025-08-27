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
#include <conio.h>
#include <ctype.h>
#include <dir.h>
#include <dos.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "winrob.h"
#include "robot.h"
#include "atlas.h"
#include "Rhino.h"

int   speed (void);
int   drkin (void), inkin (void);
int   start_fill(void), end_fill(void);
int   process_file(void), process_files(void);
int   quit (void);
int   Open (void), Close (void), Record (void);
int   get_key (void), synchronize (void);
int   start (void), refer (void), Init (void), wait (void);
int   move (void), movea (void), movew (void), moveg (void), movej (void);
int   NotePad (void);

void  Resume (void);
void  Robmain (void);
int   CheckVox (void);
int   ChangeDir (void);
void  Report (char *Text);
void  message (char *Text);
float fmin (float a, float b);
int   get_param (void), process_command (void);
void  get_command (char *str), preprocess(void);
void  Beep (void);
float atan3(float y,float x),atan4(float y,float x);

extern void Plan (void);
extern int VRead (void);
extern int FreeVols (void);
extern int CheckVols (int Rob0, int Rob1);
/*
typedef int (*fptr)(void);
typedef struct
  {
    char name[10];
    fptr function;
    int  Single;
    int  parno;
    int  CommandType; // 2 : on-line commands which are not allowed in Files
  } CommandType;      // 1 : timed-commnds which read the time
                      // 0 : untimed and general commands
                      // -1: timed commands which advance the timer
*/
CommandType command[MAX_COMMAND]={
				                         {"wait"  ,(fptr)wait,              0, 1, -1},
				                         {"init"  ,(fptr)Init,              1, 7, 0},
				                         {"sync"  ,(fptr)synchronize,       1, 1, 0},
				                         {"move" ,(fptr)move,               1, 7, 0},
                                 {"movea" ,(fptr)movea,             1, 4, 0},
                                 {"movej" ,(fptr)movej,             1, 7, 0},
				                         {"moveg" ,(fptr)moveg,             1, 2, 0},
				                         {"movew" ,(fptr)movew,             1, 3, 0},
				                         {"speed" ,(fptr)speed,             1, 7, 0},
				                         {"drkin" ,(fptr)drkin,             1, 6, 0},
				                         {"inkin" ,(fptr)inkin,             1, 10,0},
				                         {"startfill" ,(fptr)start_fill,    1, 1, 0},
                                 {"endfill", (fptr)end_fill,        1, 1, 0},
				                         {"start" ,(fptr)start,             1, 1, 1},
				                         {"refer" ,(fptr)refer,             0, 0, 1},
                                 {"chdir", (fptr)ChangeDir,         0, 0, 2},
                                 {"readvol", (fptr)VRead,           0, 0, 2},
                                 {"checkvols", (fptr)CheckVox,      0, 2, 2},
                                 {"freevols", (fptr)FreeVols,       0, 0, 2},
				                         {"plan", (fptr)Plan,               0, 0, 2},
				                         {"fread" ,(fptr)process_file,      0, 0, 2},
				                         {"multiread" ,(fptr)process_files, 0, 1, 2},
				                         {"record"  ,(fptr)Record,          0, 1, 2},
				                         {"getch" ,(fptr)get_key,           0, 0, 2},
				                         {"fopen"  ,(fptr)Open,             0, 2, 2},
                                 {"showfile", (fptr)NotePad,        0, 1, 2},
				                         {"fclose"  ,(fptr)Close,           0, 0, 2}};

FILE  *Out = NULL, *ShowFile;
int   Hide = 0;
int   Count = 0;
int   ParNo = 0;
int   SyncExe = 0;
int   IfMessage = 1;
int   DirChanged = 0;
int   RecordFlag = 0;
int   OnLineTest = 0;
int   Sync[MaxRobot];
int   IfNextCommand = 0;

float parameter[11];

char  mess[MAXCHAR];
char  NextCommand[MAXCHAR+1] = "";
char  command_part[MAXCHAR], param_part[MAXCHAR];
char  ProgramName[] = "Robot Simulation Software";

cRobot  *Robot[3];
extern  int        RobNos[];
extern  int        teach_pendant_mode;
extern  MainWindow *MainWnd;
extern  int        Show_Mode;

extern  void Alarm (int Flag);
extern  int  MultiCollision_Detect (void);
extern  void display_robot(void);
extern  void VShow (int RobotNo);
extern  void VUpdate (int RobotNo);
extern  void VFinalize (int RobotNo);
extern  void Vinit (int RobotNo, int Status);
extern  void VSave (int RobotNo, char *FileName);

/***************************************************************************/
/* RobMain is the initialization function for the Robots which is called   */
/* only once by the window owner.                                          */
/***************************************************************************/
void Robmain (void)
{
  int j;

  for (j = 0; j < MaxRobot; j++)
    {
      if (RobNos[j] == _ATLAS)
        Robot[j] = (cRobot *) new Atlas;
      else
        Robot[j] = (cRobot *) new Rhino;
      Robot[j]->RobNo = j;
      Robot[j]->initialize_maps ();
      Robot[j]->initialize_points ();
      Sync[j] = 0;
    } /* for */
  Count = 1;
  refer ();
} // Robmain

/***************************************************************************/
/* Resume is called to process the next user command.                      */
/***************************************************************************/
void Resume (void)
{
  get_command (command_part);
  preprocess ();
  if (process_command ())
    message ("Syntax Error");
} // Resume

/***************************************************************************/
/* get_command returns the next user command in 'str', by first removing   */
/* any initial blanks.                                                     */
/***************************************************************************/
void get_command (char *str)
{
  register int i;
  if (IfNextCommand)
    {
      for (i = 0; i < strlen (NextCommand); i++)
        if (NextCommand[i] != ' ')
          break;
      strcpy (str, &NextCommand[i]);
      IfNextCommand = 0;
    }
  else str[0] = 0;
  return;
} // get_command

/***************************************************************************/
/* get_param extracts the parameters of a command from 'param_part', and   */
/* places them in the array 'parameter'; the # of parameters is placed in  */
/* 'ParNo'.                                                                  */
/***************************************************************************/
int get_param (void)
{
  int i;
  char *i1, *i2;

  i2=param_part;
  for (i=0; i <= MaxParNo; i++)
    {            
      i1=_fstrchr(i2,',');
      if ((i1!=NULL) && (i==MaxParNo))
        return (1);
      if (i1==NULL)
	      {
	        if ((i1=_fstrchr(i2, ')'))==NULL)
            {
              if (i==0)
                {
                  ParNo = 0;
                  return (0);
                } /* if */
              else
                return (1);
            } /* if */
          else
            {
              parameter[i]=atof(i2);
              ParNo = i+1;  /* number of read parameters */
              return (0);
            } /* else */
        } /* if */
      parameter[i]=atof(i2);
      i2=i1+1;
    } // for
  return (i);
} // get_param

/***************************************************************************/
/* preprocess separates the parameter's part of the user command -which is */
/* embraced in parentheses- from the command part, and places it in        */
/* 'param_part'.                                                           */
/***************************************************************************/
void preprocess (void)
{
  char *ptr;

  ptr=_fstrchr(command_part,'(');
  if (ptr!=NULL)
    strncpy(param_part,ptr+1,MAXCHAR);
  else
    {
      param_part[0]=0;
      ptr=&command_part[strlen(command_part)];
    } // else
  if (strlen(command_part) != 0)
    {
      ptr--;
      while (*ptr==' ')
        ptr--;  /* remove additional spaces at the end of command_part*/
      *(ptr+1)=0;
    } // if
} // preprocess

/***************************************************************************/
/* process_command decodes the command in 'command_part', and if it is a   */
/* valid command the related function is called whose return value is      */
/* returned to the caller. Otherwise a '1' indicating error is returned.   */                        
/***************************************************************************/
int process_command (void)
{
  int i, error=0;

  if (command_part[0] == 0)
    return (0);
  for (i=0;i<MAX_COMMAND;i++)
    if (!strcmp(command_part,command[i].name) && !(error=get_param()))
      return (command[i].function());  /* execute the relevant command */
  return (1);
} // process_command

/***************************************************************************/
/* process_file reads a program file, and executes its commands line by    */
/* line. The name of the command, which is the command's parameter, is in  */
/* param_part.                                                             */
/***************************************************************************/
int process_file (void)
{
  FILE *in;
  char far *ptr;
  char File[20];
  int i,line=1;

  if (ParNo != 1)
    return (1);
  ptr=_fstrchr(param_part,')');
  if (ptr==NULL)
    {
      message ("Syntax Error");
      return(0);
    } /* if */
  *ptr=0;
  if ((in=fopen(param_part,"rt"))==NULL)
    {
      message ("File not found");
      return(0);
    } // if
  strcpy (File, param_part);
  while (!feof(in))
    {
      fgets(command_part,MAXCHAR,in);
      command_part[strlen(command_part)-1]=0;
      i=0;
      while(command_part[i]==' ')
        i++;        /* ignore preceeding spaces */
      if (i)
        strcpy(command_part,&command_part[i]);
      preprocess();
      if (process_command()&&!feof(in))
        {
	        sprintf(mess,"Syntax error on line #%d",line);
	        message (mess);
	        break;
        } // if
      line++;
    } // while
  fclose(in);
  return (0);
} // process_file

/***************************************************************************/
/* The following functions are each called for a specific user command.    */
/***************************************************************************/

/***************************************************************************/
/* If the command is a Robot-specific command (i.e. not a general command),*/
/* then the # of the commanded robot is always the first parameter of the  */
/* commad, i.e. it is in parameter[0]. The commanded joint values or other */
/* command-specific parameters are the next parameters of the command.     */
/* Since different kinds of robots can be used and simulated, for each     */
/* kind of robot, its corresponding function with a same name is called,   */
/* passing all the parameters to it.                                       */
/* Robot-specific commands are indicated by 'R-S'.                         */
/***************************************************************************/

/***************************************************************************/
/* Init initializes the commanded robot to the commanded position          */
/* instantly (in one step and not step by step). 'R-S'                     */
/***************************************************************************/
int Init (void)
{
  if (ParNo != dof+2)
    return (1);
  Robot[(int)parameter[0]]->Init(&parameter[0]);
  display_robot();
  return (0);
} // Init

/***************************************************************************/
/* movea moves the commanded robot's arm (joints 1,2,3). 'R-S'             */
/* The move is relative to the current joint values.                       */
/***************************************************************************/
int movea (void)
{
  if (ParNo != 4)
    return (1);
  Robot[(int)parameter[0]]->movea(&parameter[0]);
  return (0);
} // movea

/***************************************************************************/
/* movew moves the commanded robot's wrist (joints 4,5). 'R-S'             */
/* The move is relative to the current joint values.                       */
/***************************************************************************/
int movew (void)
{
  if (ParNo != 3)
    return (1);
  Robot[(int)parameter[0]]->movew(&parameter[0]);
  return (0);
} // movew

/***************************************************************************/
/* moveg moves the commanded robot's gripper. 'R-S'                        */
/* The move is relative to the current joint values.                       */
/***************************************************************************/
int moveg (void)
{
  if (ParNo != 2)
    return (1);
  Robot[(int)parameter[0]]->moveg(&parameter[0]);
  return (0);
} // moveg

/***************************************************************************/
/* movej moves all the commanded robot's joints and robot gripper together.*/
/* The move is relative to the current joint values. 'R-S'                 */
/***************************************************************************/
int movej (void)
{
  if (ParNo != dof+2)
    return (1);
  Robot[(int)parameter[0]]->movej(&parameter[0]);
  return (0);
} // movej

/***************************************************************************/
/* move moves all the commanded robot's joints and robot gripper together. */
/* The move is an absolute move. 'R-S'                                     */
/***************************************************************************/
int move (void)
{
  if (ParNo != dof+2)
    return (1);
  Robot[(int)parameter[0]]->move(&parameter[0]);
  return (0);
} // move

/***************************************************************************/
/* speed sets joint speeds : 1 -> fastest (default) , 8 -> slowest.        */
/* 'R-S'.                                                                  */
/***************************************************************************/
int speed (void)
{
  if (ParNo != dof+2)
    return (1);
  Robot[(int)parameter[0]]->speed(&parameter[0]);
  return (0);
} // speed

/***************************************************************************/
/* drkin solves direct kinematics equations for the commanded robot, and   */
/* displays the result. 'R-S'                                              */
/* The input parameter is the joint vector, and the output is the          */
/* position and orientation of the tooltip, i.e. (x,y,z,yaw,pitch,roll).   */
/***************************************************************************/
int drkin (void)
{
  if (ParNo != dof+1)
    return (1);
  Robot[(int)parameter[0]]->drkin(&parameter[0]);
  return (0);
} // drkin

/***************************************************************************/
/* inkin solves the inverse kinematics equations for the commanded robot   */
/* and displays the results. 'R-S'                                         */
/* The input parameter is the tooltip position and orientation, i.e.       */
/* (x,y,z,yaw,pitch,roll), and the output is the corresponding joint       */
/* vector.                                                                 */
/***************************************************************************/
int inkin (void)
{
  if (ParNo != 10)
    return (1);
  Robot[(int)parameter[0]]->inkin(&parameter[0]);
  return (0);
} // inkin

/***************************************************************************/
/* start starts moving the commanded robot. 'R-S'                          */
/***************************************************************************/
int start (void)
{
  if (ParNo != 1)
    return (1);
  Robot[(int)parameter[0]]->start();
  return (0);
} // start

/***************************************************************************/
/* synchronize takes the commanded robot to its wait state. 'R-S'          */
/***************************************************************************/
int synchronize (void)
{
  if (!SyncExe)
    return (0);
  if (ParNo != 1)
    return (1);
  Sync[(int)parameter[0]] = 0;
  return (0);
} // synchronize

/***************************************************************************/
/* refer returns all of the robots to their home positions.                */
/* It calls the actual 'refer' function for each kind of robot.            */
/***************************************************************************/
int refer (void)
{
  register int i;

  if (ParNo != 0)
    return (1);
  for (i = 0; i < MaxRobot; i++)
    Robot[i]->refer();
  display_robot();
  return (0);
} // refer

/***************************************************************************/
/* wait waits for the specified number of seconds.                         */
/***************************************************************************/
int wait (void)
{
  if (ParNo != 1)
    return (1);
//     delay (parameter[1]*1000);  // notice
  return (0);
} // wait

/***************************************************************************/
/* process_files moves each robot with the robot's own move-profile -which */
/* is given in a separate file. The robots move asynchronously, i.e. each  */
/* of them executes its own path and trajectory independently.             */              
/* Each move command in a move-profile corresponds to a sequence of moving */
/* steps by the robot. In each time step of simulation, the next moving    */
/* step of each robot is executed; therefore the next move command of a    */
/* robot is read from its profile only when the previous move command is   */
/* completed.                                                              */
/* When all the move profiles are executed, the function returns.          */
/* 'sync' commands are used to synchronize the motions of the robots; i.e. */
/* when a 'sync' command is read from a robot's profile, the robot enters  */
/* a wait state. Only after all the robots have entered their wait states, */
/* all of them are restarted again.                                        */
/* 'sync' commands are ignored for 'multiread(0)'. They are considered for */
/* 'multiread(1)'.                                                         */
/* The move-profiles are named 'ROBOTi.DAT' where 'i' is the robot number. */
/***************************************************************************/
int process_files (void)
{
  register int i;
  int      IfAnyChange, IfUpdate;

  if (ParNo != 1)
    return (1);
  for (i = 0; i < MaxRobot; i++)
    {
      Robot[i]->OpenFile();
      Sync[i] = 1;
    } // for
  OnLineTest = 1;
  SyncExe = parameter[0];
  while(1)
    {
      for (i = 0; i < MaxRobot; i++)
        if (!Robot[i]->AnyMoreChange && Sync[i])
          Robot[i]->GetNextCommand();  // if next command is a delay command
                                      // then Start will be set to a time in
                                      // the future and no move must
                                      // happen. Read Next Command until
				                              // a moving command and set Start=clock()
                                      // and AnyMoreChange=1
      for (IfAnyChange = 0, i = 0; i < MaxRobot; i++)
        if (Robot[i]->InFile != NULL)
          IfAnyChange += ((feof(Robot[i]->InFile)) ? 0 : 1);
      if (!IfAnyChange)
        break;
      for (IfUpdate = 0, i = 0; i < MaxRobot; i++)
        {
          Robot[i]->Update();
          IfUpdate += Robot[i]->UpdateDisplay;
        } // for
      if (IfUpdate != 0)
        {
          display_robot();
          if (OnLineTest)
            {
              if (MultiCollision_Detect ())
                Alarm (1);
              else
                Alarm (0);
            } // if
        } // if

      for (i = 0; i < MaxRobot; i++)
        if (Sync[i] == 1)
          break;
      if (i == MaxRobot)
	      {
          for (i = 0; i < MaxRobot; i++)
	          Sync[i] = 1;
//	        Report ("Synchronized");
	      } // for
    } // while
  for (i = 0; i < MaxRobot; i++)
    if (Robot[i]->InFile != NULL)
      Robot[i]->CloseFile();
  OnLineTest = 0;
  SyncExe = 0;
  return (0);
} // process_files

/***************************************************************************/
/* Wait for a key stroke.                                                  */
/***************************************************************************/
int get_key (void)
{
  return (0);
} // get_key

/***************************************************************************/
/* quit exits the program.                                                 */
/***************************************************************************/
int quit (void)
{
  register int i;

  if (RecordFlag)
    Close();
  for (i = 0; i < MaxRobot; i++)
    VFinalize (i);
  if (DirChanged)
    chdir("..");
  for (i = 0; i < MaxRobot; i++)
    delete Robot[i];
  return 0;
} // quit

/***************************************************************************/
/* Beep makes a beep sound.                                                */
/***************************************************************************/
void Beep (void)
{
  MessageBeep (MB_ICONEXCLAMATION);
  return;
} // beep

/***************************************************************************/
/* Open opens a file for recording desired sequences of motions of the     */
/* robots. Initially, the configurations of all robots are saved in the    */
/* file. Then with each 'record' command, the current configuration of the */
/* commanded robot is saved. Record commands can also be issued in teach-  */
/* pendant mode. The name of the output file is the first parameter.       */
/***************************************************************************/
int Open (void)
{
  char *ptr;
  register int i, j;

  if (ParNo != 1)
    return (1);
  if (RecordFlag)
    {
      message("Previous Teach File must be closed");
      return(0);
    } /* if */
  ptr=_fstrchr(param_part,')');    
  if (ptr==NULL)
    {
      message ("Syntax Error");
      return(0);
    } /* if */

  for (*ptr = ' '; *ptr == ' '; ptr--);
  ptr++;

  *ptr=0;
  if ((Out=fopen(param_part,"wt"))==NULL)
    {
      message("File can not be created");
      return(0);
    } /* if */
  RecordFlag = 1;
  for (j=0; j<MaxRobot; j++)
    {
      fprintf (Out, "speed(%d,", j);
      for (i = 1; i < dof+1; i++)
        fprintf (Out, "%d, ", Robot[j]->step[i]);
      fprintf (Out, "%d)\n", Robot[j]->step[dof+1]);

//      fprintf (Out, "speed(%d, 8, 8, 8, 8, 8, 8)\n", j);  // notice
      fprintf (Out, "move(%d,", j);
      for (i = 1; i < dof+1; i++)
        {
          fprintf (Out, "%- 10.4g, ",
          Robot[j]->theta[i]/((Robot[j]->joint_status[i]==REVOLUTE) ?
          DEG_TO_RAD : 1.0));
          Robot[j]->LastRecord[i] = Robot[j]->theta[i];
        } /* for */
      Robot[j]->LastRecord[dof+1] = Robot[j]->theta[dof+1];
      fprintf (Out, "%- 10.4g)\n", Robot[j]->LastRecord[dof+1]);
      fprintf (Out, "start(%d)\n", j);
    } /* for */
  return (0);
} // Open

/***************************************************************************/
/* Close closes the file opened for saving positions. If no file was       */
/* opened, an error message is produced.                                   */
/***************************************************************************/
int Close (void)
{
  register int i;

  if (ParNo != 0)
    return (1);
  if (RecordFlag)
    {
      RecordFlag = 0;
      for (i = 0; i < MaxRobot; i++)
        if (Robot[i]->Start_End == END)
          {
            fprintf (Out, "endfill(%d)\n", i);
            Robot[i]->Start_End = START;
          } // if
      fclose (Out);
      Out = NULL;
    } /* if */
  else
    {
      message("No Teach File is Open");
      return(0);
    } /* else */
  return (0);
} // Close

/***************************************************************************/
/* Record records current configurtion of any robot which has moved since  */
/* the last recording action, in the file previously opened by 'fopen'     */
/* command. The single parameter of this command indicates whether path    */
/* segmentation should also be done. Path segmentation is done by adding   */
/* 'startfill(i)' and 'endfill(i)' for moved robot i to the output file.   */
/***************************************************************************/
int Record (void)
{
  register int i, j;

  if (ParNo != 1)
    return (1);
  if (RecordFlag)
    {
      for (j = 0; j < MaxRobot; j++)
				if (Robot[j]->UPdate/*correction*/)
          {
            fprintf (Out, "move(%d, ", j);
            for (i = 1; i < dof+1; i++)
              {
                fprintf (Out, "%- 10.4g, ",
                Robot[j]->theta[i]/((Robot[j]->joint_status[i]==REVOLUTE) ?
                DEG_TO_RAD : 1.0));
                Robot[j]->LastRecord[i] = Robot[j]->theta[i];
              } /* for */
            Robot[j]->LastRecord[dof+1] = Robot[j]->theta[dof+1];
            fprintf (Out, "%- 10.4g)\n", Robot[j]->LastRecord[dof+1]);
            fprintf (Out, "start(%d)\n", j);
						Robot[j]->UPdate/*correction*/ = 0;
            if (parameter[0]) // If operator wants to define a new segment
              {
                if (Robot[j]->Start_End == START)
                  {
                    fprintf (Out, "startfill(%d)\n", j);
                    Robot[j]->Start_End = END;
                  } // if
                else
                  fprintf (Out, "endfill(%d)\nstartfill(%d)\n", j, j);
              } // if
          } /* if */
    } /* if */
  else
    {
      message("No Teach File is Open");
      return(0);
    } /* else */
  return (0);
} // Record

/***************************************************************************/
/* message prints the message 'Text' in a Message-Box.                     */
/***************************************************************************/
void message (char *Text)
{
  if (!IfMessage)
    return;
  Beep ();
  MessageBox (MainWnd->GetHandle (), Text, ProgramName, MB_OK | MB_ICONEXCLAMATION);
} // message

/***************************************************************************/
/* Report prints the message 'Text' in a Message-Box.                      */
/***************************************************************************/
void Report (char *Text)
{
  message (Text);
} // message

/***************************************************************************/
/* start_fill indicates that swept volume generation for the next path     */
/* segment of the commanded robot should be started. In fact, this command */
/* specifies the starting point of a path segment.                         */
/***************************************************************************/
int start_fill (void)
{
  if (ParNo != 1)
    return (1);
  Robot[(int)parameter[0]]->startfill();
  return (0);
} // start_fill

/***************************************************************************/
/* end_fill indicates that swept volume generation for the current path    */
/* segment of the commanded robot must be ended. In fact, this command     */
/* specifies the ending point of a path segment.                           */
/***************************************************************************/
int end_fill (void)
{
  if (ParNo != 1)
    return (1);
  Robot[(int)parameter[0]]->endfill();
  return (0);
} // end_fill

/***************************************************************************/
/* CheckVox checks whether the swept volumes of two robots, which are read */
/* by 'readvol' commands, collide. The two parameters of this command are  */
/* the # of the two commanded robots.                                      */
/***************************************************************************/
int CheckVox (void)
{
  if (ParNo != 2)
    return (1);
  CheckVols (parameter[0], parameter[1]);
  return 0;
} // CheckVox

/***************************************************************************/
/* ChangeDir changes to the commanded directory, or if it does not exist,  */
/* creates it. Since the names of the files generated during an execution  */
/* of the motion planning algorithm are fixed, this function if usefull to */
/* create different subdirectories for different motion planning tasks.    */
/* This command can be called only once during each session of the program.*/
/* When the program quits, the directory is automatically changed to '..', */
/* if a 'ChangeDir' command has been already issued.                       */
/***************************************************************************/
int ChangeDir (void)
{
  char *ptr;

  if (ParNo != 1)
   return (1);
  if (DirChanged)
    {
      message ("Directory is once already changed");
      return(0);
    } /* if */
  ptr=_fstrchr(param_part,')');
  if (ptr==NULL)
    {
      message ("Syntax Error");
      return(0);
    } /* if */
  *ptr=0;
  mkdir(param_part);
  if (chdir(param_part))
    {
      message ("Can't change Directory");
      return(0);
    } // if
  DirChanged = 1;
  return 0;
} // ChangeDir

/***************************************************************************/
/* fmin returns the minimum of two floating-point numbers.                 */
/***************************************************************************/
float fmin (float a,float b)
{
  if (a<b) return a;
  return b;
} // fmin

/***************************************************************************/
/* Return a value in the range 0 to 2*PI                                   */
/***************************************************************************/
float atan3 (float y,float x)
{
  float temp;

  if ((temp=atan4(y,x))>=0)
    return(temp);
  else
    return (2.0*M_PI+temp);
} // atan3

/***************************************************************************/
/* Return a value in the range -PI to PI                                   */
/***************************************************************************/
float atan4 (float y,float x)
{
  if (fabs(x)<1.0e-7)
    return( ((y>0.0) ? M_PI_2 : -M_PI_2) );
  else
    return(atan2(y,x));
} // atan4

/***************************************************************************/
// This function employs windows' Notepad to show the file the name of
// which is in 'param_part'. This function is called when the user calls
// the 'ShowFile' command.
/***************************************************************************/
int NotePad (void)
{
  char *ptr;
  register int i, j;
  char CmdLine[30]; 

  if (ParNo != 1)
    return (1);
  if (Show_Mode)
    {
      message ("File Show is already active");
      return (0);
    } // if
  ptr=_fstrchr(param_part,')');
  if (ptr==NULL)
    {
      message ("Syntax Error");
      return(0);
    } /* if */

  for (*ptr = ' '; *ptr == ' '; ptr--);
  ptr++;

  *ptr=0;

  sprintf (CmdLine, "notepad %s", param_part);
  WinExec (CmdLine , SW_SHOWMAXIMIZED);
  Show_Mode = 1;
  return (0);
} // NotePad
