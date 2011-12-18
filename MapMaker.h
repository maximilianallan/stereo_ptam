// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxmisc.h>
#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "StereoCamera.h"
#include <queue>

struct MapMakerData
{
  std::set<KeyFrame*> sMeasurementKFs; 
  std::set<KeyFrame*> sNeverRetryKFs;  
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};

class MapMaker : protected CVD::Thread
{
public:
  // constructed with stereo cam
  MapMaker(Map &m, const StereoCamera &camL);
  ~MapMaker();
  
  // Make a map from stereo cam. Called by the tracker.
  bool InitFromStereo(KeyFrame &kFirst, 
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
		      SE3<> &se3CameraPos);

  bool InitFromStereo_OLD(KeyFrame &kFirst, KeyFrame &kSecond,  // EXPERIMENTAL HACK
		      std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
		      SE3<> &se3CameraPos);
  
  
  void AddKeyFrame(KeyFrame &k);   
  void RequestReset();
  bool ResetDone();   
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} 
  bool NeedNewKeyFrame(KeyFrame &kCurrent);            
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent); 
protected:
  Map &mMap;
  // the stereo camera used by teh map maker
  StereoCamera mCamera;
  virtual void run();  
  // Triangulate a point in a rectified setup
  Vector<3> TriangulatePoint(const Vector<2> &v2A, const Vector<2> &v2B);
  SE3<> CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  void ApplyGlobalScaleToMap(double dScale);
  
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame &k, Level &l, int nLevel);
  void AddSomeMapPoints(int nLevel);
  void InitAddMapPoints(int nLevel);
  // add map point from true stereo epipolar search
  bool InitAddPointEpipolar(KeyFrame &kf, int nLevel, int nCandidate);
  // add map points just using initaddpointepipolar rather than choosing
  bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);
  // Returns point in ref frame B
  Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);

  void BundleAdjust(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustAll();
  void BundleAdjustRecent();
  void BundleAdjustSingleKeyframe();
  
  // refinds all now over stereo pair
  int ReFindInSingleKeyFrame(KeyFrame &k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame &k, MapPoint &p);
  
  void SubPixelRefineMatches(KeyFrame &k, int nLevel);
  
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(KeyFrame &kCurrent);
  double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
  KeyFrame* ClosestKeyFrame(KeyFrame &k);
  std::vector<KeyFrame*> NClosestKeyFrames(KeyFrame &k, unsigned int N);
  void RefreshSceneDepth(KeyFrame *pKF);
  

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
  

  Matrix<4> ReProjectionMatrix; // Q reprojection matrix l2r for rectified.

  std::vector<KeyFrame*> mvpKeyFrameQueue; 
  std::vector<std::pair<KeyFrame*, MapPoint*> > mvFailureQueue; 
  std::queue<MapPoint*> mqNewQueue;
  
  double mdBaseLine;
  double mdBaseLineOverSceneDepth;
    
  bool mbBundleConverged_Full;
  bool mbBundleConverged_Recent;
  
  bool mbResetRequested;
  bool mbResetDone;     
  bool mbBundleAbortRequested;
  bool mbBundleRunning;       
  bool mbBundleRunningIsRecent;
  
};

#endif


















