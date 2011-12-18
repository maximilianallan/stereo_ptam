// Copyright 2008 Isis Innovation Limited
#include "StereoCamera.h"
#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"
#include <TooN/se3.h>
#include <TooN/TooN.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxmisc.h>
#include "HelperFunctions.h"
#include "ATANRectifiedCamera.h"
using namespace CVD;
using namespace std;
using namespace GVars3;


System::System(): mGLWindow_L(mVideoSource.Size(), "Left Camera"), mGLWindow_R(mVideoSource.Size(), "Right Camera")
{
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  mimFrameR_BW.resize(mVideoSource.Size());
  mimFrameL_BW.resize(mVideoSource.Size());
  mimFrameR_RGB.resize(mVideoSource.Size());
  mimFrameL_RGB.resize(mVideoSource.Size());

  Vector<NUMTRACKERCAMPARAMETERS> vTest_LEFT;
  Vector<NUMTRACKERCAMPARAMETERS> vTest_RIGHT;
  Vector<6> L2R_Extrinsic;
  Vector<6> R2L_Extrinsic;
  cout << "getting gvars" << endl;
  //Get internal and external calibration parameters for both cameras
  vTest_RIGHT = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("CameraRight.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
  vTest_LEFT = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("CameraLeft.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
  L2R_Extrinsic = GV3::get<Vector<6> >("CameraLeftExtrinsic.Parameters", ATANCamera::mvDefaultExtrinsicVector,HIDDEN);
  R2L_Extrinsic = GV3::get<Vector<6> >("CameraRightExtrinsic.Parameters",ATANCamera::mvDefaultExtrinsicVector,HIDDEN);
  
  //Create a stereo camera  
  mpCamera = new StereoCamera("CameraLeft","CameraRight");
  Vector<2> v2;
  if(v2==v2) ;
  if(vTest_LEFT == ATANCamera::mvDefaultParams || vTest_RIGHT == ATANCamera::mvDefaultParams){
      cout << endl;
      cout << "Camera intrinsic parameters are not set, need to run the CameraCalibrator tool" << endl;
      cout << "  and/or put the Camera_[Left|Right].Parameters= line into the appropriate .cfg file." << endl;
      exit(1);
  }
  // load the extrinsic matrices
  SE3<> L2R = SE3<>::exp(L2R_Extrinsic);
  SE3<> R2L = SE3<>::exp(R2L_Extrinsic);
  // determine of the translation vector is set - test to see if calibration
  if(!( L2R.get_translation()*L2R.get_translation()) || !(R2L.get_translation()*R2L.get_translation()) ){
    cout << endl;
    cout << "Camera extrinsic parameters are not set, need to run the CameraCalibrator tool "<< endl;
    cout << " and/or put the Camera[Left|Right]Extrinsic.Paramters= line into the appropriate .cfg file " << endl;
    exit(1);
  }
 
  /*  OLD CODE FOR PERFORMING RECTIFICATION
      
      ATANCamera &l = *mpCamera_Left;
      ATANCamera &r = *mpCamera_Right;
      
      cv::Mat &lMapX = *l.getMap1();
      cv::Mat &lMapY = *l.getMap2();
      cv::Mat &rMapX = *r.getMap1();
      cv::Mat &rMapY = *r.getMap2();
      
      UndistortRectifyMap(l.getCvCameraMatrix(),
      l.getmdW(),
      l.getCvRectifyRotate(),
      l.getCvRectifyProject(),
      mVideoSource.getCVSize(),
      CV_32FC1,
      lMapX, lMapY);
      
      UndistortRectifyMap(r.getCvCameraMatrix(),
      r.getmdW(),
      r.getCvRectifyRotate(),
      r.getCvRectifyProject(),
      mVideoSource.getCVSize(),
      CV_32FC1,
      rMapX, rMapY);
      l.SetParametersToRectified();
      r.SetParametersToRectified();
      
      mprCamera_Left = 
      new ATANRectifiedCamera(l.getCvRectifyProject(),
      l.getCvRectifyReProject(),
      l.getMap1(), 
      l.getMap2());
      
      mprCamera_Right = 
      new ATANRectifiedCamera(r.getCvRectifyProject(),
      r.getCvRectifyReProject(),
      r.getMap1(), 
      r.getMap2());
      
      assert(l.getPrincipalPoint(0) == r.getPrincipalPoint(0) && 
      l.getFocal(0) == r.getFocal(0) && 
      l.getPrincipalPoint(1) == r.getPrincipalPoint(1) && 
      r.getFocal(1) == l.getFocal(1));
      
      assert(mprCamera_Right->getFocal(0) == r.getFocal(0) 
      && mprCamera_Left->getFocal(1) == l.getFocal(1) 
      && mprCamera_Right->getPrincipalPoint(0) == 
      r.getPrincipalPoint(0) 
      && mprCamera_Right->getPrincipalPoint(1) == 
      r.getPrincipalPoint(1) 
      && mprCamera_Left->getPrincipalPoint(0) == 
      l.getPrincipalPoint(0) 
      && mprCamera_Left->getPrincipalPoint(1) == 
	 l.getPrincipalPoint(1)); 
  */
  
  mpMap = new Map;
  //Each function now passes a stereo camera
  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
  mpTracker = new Tracker(mVideoSource.Size(), *mpCamera, *mpMap, *mpMapMaker, mGLWindow_L, mGLWindow_R);
  mpARDriver = new ARDriver(*mpCamera, mVideoSource.Size(), mGLWindow_L);
  mpMapViewer = new MapViewer(*mpMap, mGLWindow_L);
  mGLWindow_L.make_current();
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawAR=0");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");
  
  mbDone = false;
};

void System::Run()
{
  while(!mbDone)
    {
      /*Computing rectification remap 
	
	ATANRectifiedCamera &l = *mprCamera_Left;
	ATANRectifiedCamera &r = *mprCamera_Right;
	cv::Mat &lMapX = *l.getMap1();
	cv::Mat &lMapY = *l.getMap2();
	cv::Mat &rMapX = *r.getMap1();
	cv::Mat &rMapY = *r.getMap2();
      */
      
      // Send 4 empty frames to be filled by the videosource
      // 2 b&w frames and 2 rgb frames are returned
      
      mVideoSource.GetAndFillFrameBWandRGB(mimFrameL_BW, mimFrameR_BW, mimFrameL_RGB, mimFrameR_RGB);
      
      static bool bFirstFrame = true;
      
      if(bFirstFrame)
	{
	  mpARDriver->Init();
	  bFirstFrame = false;
	}
      
      mGLWindow_L.SetupViewport();
      mGLWindow_L.SetupVideoOrtho();
      mGLWindow_L.SetupVideoRasterPosAndZoom();

      mGLWindow_R.SetupViewport();
      mGLWindow_R.SetupVideoOrtho();
      mGLWindow_R.SetupVideoRasterPosAndZoom();
      
      if(!mpMap->IsGood())
	mpARDriver->Reset();
      
      static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
      static gvar3<int> gvnDrawAR("DrawAR", 0, HIDDEN|SILENT);
      
      bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
      bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;
      
      mpTracker->TrackFrame(mimFrameL_BW, mimFrameR_BW, !bDrawAR && !bDrawMap);
      
      if(bDrawMap)
	mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
      else if(bDrawAR)
	mpARDriver->Render(mimFrameL_RGB, mpTracker->GetCurrentPose());

      string sCaption;
      if(bDrawMap)
	sCaption = mpMapViewer->GetMessageForUser();
      else
	sCaption = mpTracker->GetMessageForUser();
      mGLWindow_L.DrawCaption(sCaption);
      mGLWindow_L.DrawMenus();
      mGLWindow_L.swap_buffers();
      mGLWindow_L.HandlePendingEvents();
      mGLWindow_R.DrawCaption(sCaption);
      mGLWindow_R.DrawMenus();
      mGLWindow_R.swap_buffers();
      mGLWindow_R.HandlePendingEvents();
    }
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
}








