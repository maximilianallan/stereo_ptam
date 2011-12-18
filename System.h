// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "VideoSource.h"
#include "GLWindow2.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

class StereoCamera;
class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;

class System
{
public:
  System();
  void Run();
  
private:
  VideoSource mVideoSource;
  GLWindow2 mGLWindow_L;
  GLWindow2 mGLWindow_R;
  //left and right rgb images
  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameR_RGB;
  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameL_RGB;
  //left and right b&w images
  CVD::Image<CVD::byte> mimFrameR_BW;
  CVD::Image<CVD::byte> mimFrameL_BW;
  
  Map *mpMap; 
  MapMaker *mpMapMaker; 
  Tracker *mpTracker;
  StereoCamera *mpCamera;
  //ATANCamera *mpCamera_Left;
  //ATANCamera *mpCamera_Right;
  //ATANCamera *mprCamera_Left;
  //ATANCamera *mprCamera_Right;
  ARDriver *mpARDriver;
  MapViewer *mpMapViewer;
  
  bool mbDone;

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};



#endif
