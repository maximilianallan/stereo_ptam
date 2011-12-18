// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __TRACKERDATA_H
#define __TRACKERDATA_H

#include "PatchFinder.h"
#include "ATANCamera.h"
#include "ATANCamera.h"

struct TrackerData
{
  TrackerData(MapPoint *pMapPoint) 
    : Point(*pMapPoint)
  {};
  
  MapPoint &Point;
  // now contains a stereo patch finder for finding both points in images
  StereoPatchFinder StereoFinder;
  
  // Projection itermediates:
  // left frame and right frame camera coordiantes
  Vector<3> v3LeftCam;        
  Vector<3> v3RightCam;
  Vector<2> v2LeftImPlane;    
  Vector<2> v2RightImPlane;
  Vector<2> v2LeftImage;
  Vector<2> v2RightImage;
  Matrix<2> m2LeftCamDerivs;
  Matrix<2> m2RightCamDerivs;
  Matrix<4> m4CamDerivs;

  bool bInBothImages;
  bool bInLeftImage;
  bool bInRightImage;
  bool bPotentiallyVisible; 
  
  int nSearchLevel;
  bool bSearched;
  bool bFound; 
  bool bFoundEpipolar;
  bool bDidSubPix;
  bool bDidSubPixEpipolar;
  Vector<2> v2LeftFound; 
  Vector<2> v2RightFound;
  double dSqrtInvNoise; 
  
  
  Vector<2> v2Error_CovScaledLeft;
  Vector<2> v2Error_CovScaledRight;
  // 2x6 jacobian for coarse pose update - only over left camera
  Matrix<2,6> m26Jacobian; 
  // 4x6 jac and covariance for fine pose update - stereo
  Matrix<4,6> m46Jacobian;
  Vector<4> v4Error_CovScaled;
  
  // Project point into both images

  inline void Project(const SE3<> &se3CFromW, ATANCamera &LeftCam, ATANCamera &RightCam)
  {
    bInLeftImage = bInRightImage = bInBothImages = bPotentiallyVisible = false;
    // left camera and right camera coords
    v3LeftCam = se3CFromW * Point.v3WorldPos;
    v3RightCam = LeftCam.Extrinsic * v3LeftCam; 
    
    if(v3LeftCam[2] < 0.001 || v3RightCam[2] < 0.001){
      return;
    }
    
    // project onto both image planes - where does it end up?
    v2LeftImPlane = project(v3LeftCam); 
    v2RightImPlane = project(v3RightCam);
    bInLeftImage = (v2LeftImPlane*v2LeftImPlane) < (LeftCam.LargestRadiusInImage() * LeftCam.LargestRadiusInImage());
    bInRightImage = (v2RightImPlane*v2RightImPlane) < (RightCam.LargestRadiusInImage() * RightCam.LargestRadiusInImage());

    // check if it is in both images, if not bail
    if(!bInLeftImage)
      {
	bInLeftImage = false;
	bInRightImage = false;
	bInBothImages = false;
	return;
      }
    if(!bInRightImage)
      {
	bInLeftImage = false;
	bInRightImage = false;
	bInBothImages = false;
	return;
      }
    
    // project to both images
    if(bInLeftImage){
      v2LeftImage = LeftCam.Project(v2LeftImPlane);
      if(LeftCam.Invalid()){
	bInLeftImage = false;
	bInRightImage = false;
	bInBothImages = false;
	return;
      }
    }
    
    if(bInRightImage){
      v2RightImage = RightCam.Project(v2RightImPlane);
      if(RightCam.Invalid()){
	bInRightImage = false;
	bInLeftImage = false;
	bInBothImages = false;
	return;
      }
    }
    
    if(bInLeftImage && 
       (v2LeftImage[0] < 0 || v2LeftImage[1] < 0 || v2LeftImage[0] > irLeftImageSize[0] || v2LeftImage[1] > irLeftImageSize[1])
       ){
      bInLeftImage = false;
      bInRightImage = false;
      bInBothImages = false;
      return;
    }
    
    if(bInRightImage && 
       (v2RightImage[0] < 0 || v2RightImage[1] < 0 || v2RightImage[0] > irRightImageSize[0] || v2RightImage[1] > irRightImageSize[1])
       ){
      bInRightImage = false;
      bInLeftImage = false;
      bInBothImages = false;
      return;
    }
    
    if(bInRightImage && bInLeftImage)
      bInBothImages = true;
  }
  
  // get projection derivs for both cameras
  inline void GetDerivsUnsafe(ATANCamera &LeftCam, ATANCamera &RightCam) 
  {
    if(bInLeftImage){
      m2LeftCamDerivs = LeftCam.GetProjectionDerivs();
      m4CamDerivs.slice(0,0,2,2) = m2LeftCamDerivs;
    }
    if(bInRightImage){
      m2RightCamDerivs = RightCam.GetProjectionDerivs();
      m4CamDerivs.slice(2,2,2,2) = m2RightCamDerivs;
    }
  }
  
  inline void ProjectAndDerivs(SE3<> &se3, ATANCamera &LeftCam, ATANCamera &RightCam)
  {
    Project(se3, LeftCam, RightCam);
    if(bFound || bFoundEpipolar)
      GetDerivsUnsafe(LeftCam, RightCam);
  }
  
  // get old 2x6 jac for coarse pose update
  inline void CalcCoarseJacobian()
  {

    double dOneOverLeftCameraZ = 1.0 / v3LeftCam[2];
    for(int m=0; m<6; m++)
      {
	const Vector<4> v4Motion1 = SE3<>::generator_field(m, unproject(v3LeftCam));
	Vector<2> v2CamFrameMotion;
	v2CamFrameMotion[0] = (v4Motion1[0] - v3LeftCam[0] * v4Motion1[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
	v2CamFrameMotion[1] = (v4Motion1[1] - v3LeftCam[1] * v4Motion1[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
		
	m26Jacobian.T()[m] = m2LeftCamDerivs * v2CamFrameMotion;
      };
  }

  // get 4x6 jac for fine pose update
  // same method as bundle adjuster
  inline void CalcFineJacobian()
  {
    double dOneOverLeftCameraZ = 1.0 / v3LeftCam[2];
    double dOneOverRightCameraZ = 1.0 / v3RightCam[2];
    for(int m=0; m<6; m++)
      {
	const Vector<4> v4Motion1 = SE3<>::generator_field(m, unproject(v3LeftCam));
	const Vector<4> v4Motion2 = SE3<>::generator_field(m, unproject(v3RightCam));
	Vector<2> v2LeftCamFrameMotion;
	Vector<2> v2RightCamFrameMotion;
	
	v2LeftCamFrameMotion[0] = (v4Motion1[0] - v3LeftCam[0] * v4Motion1[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
	v2LeftCamFrameMotion[1] = (v4Motion1[1] - v3LeftCam[1] * v4Motion1[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
	
	v2RightCamFrameMotion[0] = (v4Motion2[0] - v3RightCam[0] * v4Motion2[2] * dOneOverRightCameraZ) * dOneOverRightCameraZ;
	v2RightCamFrameMotion[1] = (v4Motion2[1] - v3RightCam[1] * v4Motion2[2] * dOneOverRightCameraZ) * dOneOverRightCameraZ;
	
	m46Jacobian.T()[m].slice(0,2) = m2LeftCamDerivs * v2LeftCamFrameMotion;
	m46Jacobian.T()[m].slice(2,2) = m2RightCamDerivs * v2RightCamFrameMotion;
      };
  }
  // linear update for coarse
  inline void LinearUpdate(const Vector<6> &v6)
  {
    v2LeftImage += m26Jacobian * v6;
  }
  
  // linear update for fine
  inline void LinearUpdateBoth(const Vector<6> &v6)
  {
    Vector<4> lin = m46Jacobian * v6;
    v2LeftImage += lin.slice<0,2>();
    v2RightImage += lin.slice<2,2>();
  }
  
  static CVD::ImageRef irLeftImageSize;
  static CVD::ImageRef irRightImageSize;
};






#endif




