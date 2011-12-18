#ifndef __STEREO__CAMERA_H
#define __STEREO__CAMERA_H

#include "VideoSource.h"
#include <cmath>
#include "HelperFunctions.h"
#include <TooN/se3.h>
#include <TooN/TooN.h>
using namespace TooN;
#include <cvd/vector_image_ref.h>
#include <gvars3/gvars3.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "constants.h"
#include "ATANCamera.h"
#include <TooN/LU.h>
#include <TooN/SVD.h>

class StereoCamera{
 public:
  StereoCamera(string sLeftName, string sRightName);
  StereoCamera();
  
  inline ATANCamera &Left(){ return mLeftCam; };
  inline ATANCamera &Right(){ return mRightCam; };
  inline SE3<> GetRelativePose(){ return RelativePose; }
  
  Vector<3> UnProjectToWorld(Vector<2> v2A, Vector<2> v2B, const SE3<> &CameraPose);
  Vector<3> UnProjectToWorldFromPixels(Vector<2> v2A, Vector<2> v2B, const SE3<> &CameraPose);
  inline Vector<2> UnProjectToLeft(Vector<2> v2){
    return mLeftCam.UnProject(v2);
  }
  inline Vector<2> UnProjectToLeft(CVD::ImageRef ir){
    return mLeftCam.UnProject(vec(ir));
  }
  inline Vector<2> UnProjectToRight(Vector<2> v2){
    return mRightCam.UnProject(v2);
  }
  inline Vector<2> UnProjectToRight(CVD::ImageRef ir){
    return mRightCam.UnProject(vec(ir));
  }
  inline Vector<2> ProjectToLeft(Vector<2> v2){
    return mLeftCam.Project(v2);
  }
  inline Vector<2> ProjectToLeft(CVD::ImageRef ir){
    return mLeftCam.Project(vec(ir));
  }
  inline Vector<2> ProjectToRight(Vector<2> v2){
    return mRightCam.Project(v2);
  }
  inline Vector<2> ProjectToRight(CVD::ImageRef ir){
    return mRightCam.Project(vec(ir));
  }
  
  inline Vector<3> GetEpipolarLine(Vector<2> v2){
    return FundamentalMatrix * unproject(v2);
  }
  
  inline Vector<3> GetEpipolarLine(CVD::ImageRef ir){
    return GetEpipolarLine(vec(ir));
  }
						   
  
static const Vector<6> mvDefaultExtrinsicVector;
 
 protected:
  
  ATANCamera mLeftCam;
  ATANCamera mRightCam;
  
  GVars3::gvar3<Vector<6> >mgvvRelativePose_Vector;
  SE3<> RelativePose;
  Matrix<3> EssentialMatrix;
  Matrix<3> FundamentalMatrix;
  
  
};

#endif


