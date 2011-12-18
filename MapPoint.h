// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This file declares the MapPoint class
// 
// The map is made up of a bunch of mappoints.
// Each one is just a 3D point in the world;
// it also includes information on where and in which key-frame the point was
// originally made from, so that pixels from that keyframe can be used
// to search for that point.
// Also stores stuff like inlier/outlier counts, and privat information for 
// both Tracker and MapMaker.

#ifndef __MAP_POINT_H
#define __MAP_POINT_H
#include <TooN/TooN.h>
using namespace TooN;
#include <cvd/image_ref.h>
#include <cvd/timer.h>
#include <set>

class KeyFrame;
class TrackerData;
class MapMakerData;

struct MapPoint
{
  // Constructor inserts sensible defaults and zeros pointers.
  inline MapPoint()
  {
    bBad = false;
    pTData = NULL;
    pMMData = NULL;
    nMEstimatorOutlierCount = 0;
    nMEstimatorInlierCount = 0;
    dCreationTime = CVD::timer.get_time();
  };
  
  // Where in the world is this point? The main bit of information, really.
  Vector<3> v3WorldPos;
  // Is it a dud? In that case it'll be moved to the trash soon.
  bool bBad;
  
  // What pixels should be used to search for this point?
  KeyFrame *pPatchSourceKF; // The KeyFrame the point was originally made in
  int nSourceLevel;         // Pyramid level in source KeyFrame
  CVD::ImageRef irLeftFrameCenter;   // This is in level-coords in the source pyramid level
  CVD::ImageRef irRightFrameCenter;
  
  // What follows next is a bunch of intermediate vectors - they all lead up
  // to being able to calculate v3Pixel{Down,Right}_W, which the PatchFinder
  // needs for patch warping!
  
  // NC refers to vectors in camera frame 
  // W refers to vectors in world frame
  Vector<3> v3LeftFrameCenter_NC;
  Vector<3> v3RightFrameCenter_NC;
  // Unit vector in Source-KF coords
  // pointing at the patch center
  Vector<3> v3OneDownFromLeftFrameCenter_NC;
  Vector<3> v3OneDownFromRightFrameCenter_NC;
  // Unit vector in Source-KF coords pointing 
  // towards one pixel down of the patch center
  Vector<3> v3OneRightFromLeftFrameCenter_NC;
  Vector<3> v3OneRightFromRightFrameCenter_NC;
  // Unit vector in Source-KF coords pointing
  // towards one pixel right of the patch center
  Vector<3> v3Normal_NC;
  // Unit vector in Source-KF coords indicating patch normal
  Vector<3> v3LeftFramePixelDown_W;
  Vector<3> v3RightFramePixelDown_W;
  // 3-Vector in World coords corresponding to 
  // a one-pixel move down the source image - 
  // should not be too different from right
  Vector<3> v3LeftFramePixelRight_W;
  Vector<3> v3RightFramePixelRight_W;
  // 3-Vector in World coords corresponding to
  // a one-pixel move right the source image 
  void RefreshPixelVectors();        
  // Calculates above two vectors
  
  // Info for the Mapmaker (not to be trashed by the tracker:)
  MapMakerData *pMMData;
  
  // Info for the Tracker (not to be trashed by the MapMaker:)
  TrackerData *pTData;
  
  // Info provided by the tracker for the mapmaker:
  int nMEstimatorOutlierCount;
  int nMEstimatorInlierCount;
  
  // Random junk (e.g. for visualisation)
  double dCreationTime; //timer.get_time() time of creation
};

#endif
