#include <gvars3/instances.h>
#include "StereoCamera.h"
#include "HelperFunctions.h"

using namespace GVars3;

StereoCamera::StereoCamera(string sLeftName, string sRightName):mLeftCam(sLeftName),mRightCam(sRightName)
{
  GV2.Register(mgvvRelativePose_Vector,sLeftName+"Extrinsic.Parameters", mvDefaultExtrinsicVector,HIDDEN|FATAL_IF_NOT_DEFINED);
  RelativePose = SE3<>::exp(*mgvvRelativePose_Vector);
  EssentialMatrix = SkewSymmetric(RelativePose.get_translation())*RelativePose.get_rotation().get_matrix();
  
  // Perform LU decomposition to get pseudo inverse
  LU<3> LU_r = mRightCam.GetCameraMatrix();
  LU<3> LU_l = mLeftCam.GetCameraMatrix();
  // F = KR^ E KL^ 
  FundamentalMatrix = LU_r.get_inverse().T() * EssentialMatrix * LU_l.get_inverse();
  
}

//Direct implementation of MVG p 312: Linear Triangulation 
//Methods 
//Reproject to world coordinates given a stereo camera rig
//of pose se3CameraPose viewing a point in the left frame v2Left
//and the right frame v2B
Vector<3> StereoCamera::UnProjectToWorld(Vector<2> v2Left, Vector<2> v2Right, const SE3<> &se3LCameraPose)
{
  SE3<> se3RCameraPose = RelativePose*se3LCameraPose;
  Matrix<3,4> P,Q; // projection matricies
  
  P.slice<0,0,3,3>()=se3LCameraPose.get_rotation().get_matrix();
  P.slice<0,3,3,1>()=se3LCameraPose.get_translation().as_col();
  Q.slice<0,0,3,3>()=se3RCameraPose.get_rotation().get_matrix();
  Q.slice<0,3,3,1>()=se3RCameraPose.get_translation().as_col();
  
  Matrix<4> A;
  A[0] = v2Left[0]*P[2] - P[0];
  A[1] = v2Left[1]*P[2] - P[1];
  A[2] = v2Right[0]*Q[2] - Q[0];
  A[3] = v2Right[1]*Q[2] - Q[1];
  
  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}

Vector<3> StereoCamera::UnProjectToWorldFromPixels(Vector<2> v2A, Vector<2> v2B, const SE3<> &CameraPose)
{
  Vector<2> v2UA = mLeftCam.UnProject(v2A);
  Vector<2> v2UB = mRightCam.UnProject(v2B);
  return UnProjectToWorld(v2UA,v2UB,CameraPose);
}
  

const Vector<6> StereoCamera::mvDefaultExtrinsicVector = makeVector(0.5,0.5,0.5,0.5,0.5,0.5);
