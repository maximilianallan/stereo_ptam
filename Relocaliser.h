// -*- c++ -*- 
// Copyright 2008 Isis Innovation Limited
//
// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#ifndef __RELOCALISER_H
#define __RELOCALISER_H
#include <TooN/se2.h>
#include "StereoCamera.h"
#include "SmallBlurryImage.h"
#include "StereoCamera.h"

#include "Map.h"


class Relocaliser
{
public:
  Relocaliser(Map &map, StereoCamera &camera);
  bool AttemptRecovery(KeyFrame &k);
  SE3<> BestPose();
  
protected:
  void ScoreKFs(KeyFrame &kCurrentF);
  Map &mMap;
  StereoCamera mCamera;
  int mnBest;
  double mdBestScore;
  SE2<> mse2;
  SE3<> mse3Best;

};
#endif









