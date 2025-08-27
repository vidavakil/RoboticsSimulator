
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

#ifndef RHINO
#include "robot.h"

class Rhino: cRobot
  {
    public :
      Rhino (void);

      void    initialize_maps(void);
      void    initialize_points(void);
      void    update_maps(float *joint);
      void    compute_end_device(float tv);
      int     kinematics(frame t,float *j);
      int     inverse_kinematics(frame t,float *j);
  };
#endif
#define RHINO
