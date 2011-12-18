// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
// 
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H
#include <TooN/TooN.h>
#include "constants.h"
using namespace TooN;
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <vector>
#include <set>
#include <map>

class MapPoint;
class SmallBlurryImage;


// Candidate: a feature in an image which could be made into a map point
struct Candidate
{
  CVD::ImageRef irLevelPos;
  Vector<2> v2RootPos;
  double dSTScore;
};

// Measurement: contains pixels from both frames now

struct Measurement
{
  int nLevel; 
  bool bSubPix; 
  Vector<2> v2RootPos;  
  Vector<2> v2CorrespondingPos;
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; 
};


struct Level
{
  inline Level()
  {
    bImplaneCornersCached = false;
  };
  
  CVD::Image<CVD::byte> im;               
  std::vector<CVD::ImageRef> vCorners;    
  std::vector<int> vCornerRowLUT;         
  std::vector<CVD::ImageRef> vMaxCorners; 
  Level& operator=(const Level &rhs);
  std::vector<Candidate> vCandidates;   
  bool bImplaneCornersCached; 
  std::vector<Vector<2> > vImplaneCorners; 
};

struct KeyFrame
{
  inline KeyFrame()
  {
    pSBI = NULL;
  }
  SE3<> se3LeftCfromW;   // left camera pose 
  SE3<> se3RightCfromW;  // right camera pose
  bool bFixed; 
  Level aCamLeftLevels[LEVELS];  // Images, corners, etc for the left image pyramid
  Level aCamRightLevels[LEVELS]; // same for the right
  std::map<MapPoint*, Measurement> mMeasurements; 
  void MakeKeyFrame_Lite(CVD::BasicImage<CVD::byte> &im); 
  // This takes an image and calculates pyramid levels etc to fill the
  void MakeKeyFrameStereo_Lite(CVD::BasicImage<CVD::byte> &im);
  // just generates the pyramid and candidates - no ordering of fast corners - test
  
  void MakeKeyFrame_Rest();
  // ...  this calculates the rest of the data which the mapmaker needs.
  double dSceneDepthMean;
  double dSceneDepthSigma;
  SmallBlurryImage *pSBI; 
};

typedef std::map<MapPoint*, Measurement>::iterator meas_it; 


#endif

