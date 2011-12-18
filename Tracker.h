//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H
#include "System.h"
#include "MapMaker.h"
#include "StereoCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"
#include "GLWindow2.h"
#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <sstream>
#include <vector>
#include <list>


class TrackerData;
struct Trail    
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos; 
  CVD::ImageRef irInitialPos;
};

class Tracker
{
public:
  Tracker(CVD::ImageRef irVideoSize, const StereoCamera &c, Map &m, MapMaker &mm, GLWindow2 &l, GLWindow2 &r);
  void TrackFrame(CVD::Image<CVD::byte> &imFrame_L, CVD::Image<CVD::byte> &imFrame_R, bool bDraw); 

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}
  
  std::string GetMessageForUser();
  
protected:
  KeyFrame mCurrentKF;
  GLWindow2 &mWindowL;
  GLWindow2 &mWindowR; 
  
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  StereoCamera mCamera;           //Projection model

  Relocaliser mRelocaliser;       

  CVD::ImageRef mirSize;          
  
  void Reset();                   
  void RenderGrid();             


  void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;
  void TrailTracking_Start();     
  int  TrailTracking_Advance();   
  std::list<Trail> mlTrails;      
  KeyFrame mFirstKF;              
  KeyFrame mPreviousFrameKF;      
  
  // track map over stereo pair
  void TrackMap();                
  void AssessTrackingQuality();   
  void ApplyMotionModel();        
  void UpdateMotionModel();    
  // find  left coarse points
  int SearchForCoarsePoints(std::vector<TrackerData*> &vTD, 
			    int nRange, 
			    int nFineIts);  
  // find in both images
  int SearchForPoints(std::vector<TrackerData*> &vTD, 
		      int nRange, 
		      int nFineIts);
  // get  update based onboth image
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD, 
			   double dOverrideSigma = 0.0, 
			   bool bMarkOutliers = false); 
  // get update on just left
  Vector<6> CalcPoseUpdateFromCoarse(std::vector<TrackerData*> vTD, double dOverrideSigma = 0.0, bool bMarkOutliers = false);
  SE3<> mse3CamFromWorld;   
  SE3<> mse3StartPos;       
  Vector<6> mv6CameraVelocity;
  double mdVelocityMagnitude; 
  double mdMSDScaledVelocityMagnitude;
  bool mbDidCoarse;               
  
  bool mbDraw;                    
  
  int mnFrame;
  int mnLastKeyFrameDropped;
  void AddNewKeyFrame();    

  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;

  bool AttemptRecovery();         
  bool mbJustRecoveredSoUseCoarse;

  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Vector<6> mv6SBIRot;
  bool mbUseSBIInit;
  
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;
  
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
};

#endif






