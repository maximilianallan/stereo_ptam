// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// MiniPatch.h
//
// Declares MiniPatch class
// 
// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map

#ifndef __MINI_PATCH_H
#define __MINI_PATCH_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/utility.h>
#include <cvd/fast_corner.h>
#include <TooN/TooN.h>
using namespace TooN;
#include <vector>
#include "constants.h"
struct MiniPatch
{
  void SampleFromImage(CVD::ImageRef irPos, CVD::BasicImage<CVD::byte> &im);  // Copy pixels out of source image
  bool FindPatch(CVD::ImageRef &irPos,           // Find patch in a new image
		 CVD::BasicImage<CVD::byte> &im, 
		 int nRange, int ,int ,int,
		 std::vector<CVD::ImageRef> &vCorners,
		 std::vector<int> *pvRowLUT = NULL);
  
  bool FindPatchEpipolar(CVD::ImageRef &irPos,
			 CVD::BasicImage<CVD::byte> &im, 
			 Vector<3> EpipolarLine,
			 std::vector<CVD::ImageRef> &vCorners);
  //finds the corresponding point in the right image
  //bool FindCorrespondingPoint(CVD::ImageRef &irPos,
  //			      CVD::BasicImage<CVD::byte> &im,
  //			      CVD::ImageRef &irPosEnd,
  //			      bool left = true, std::vector<CVD::ImageRef> *fc = NULL);
  inline int SSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir); // Score function
  static int mnHalfPatchSize;     // How big is the patch?
  static int mnRange;             // How far to search? 
  static int mnMaxSSD;            // Max SSD for matches?
  CVD::Image<CVD::byte> mimOrigPatch;  // Original pixels
};

#endif
