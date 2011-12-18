// Copyright 2008 Isis Innovation Limited
#include "MapPoint.h"
#include "KeyFrame.h"
void MapPoint::RefreshPixelVectors()
{
  KeyFrame &k = *pPatchSourceKF;
  
  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Vector<3> v3LeftPlanePoint_C = k.se3LeftCfromW * v3WorldPos;
  Vector<3> v3RightPlanePoint_C = k.se3RightCfromW * v3WorldPos;
  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  double dLeftCamHeight = fabs(v3LeftPlanePoint_C * v3Normal_NC);
  double dRightCamHeight = fabs(v3RightPlanePoint_C * v3Normal_NC);
  double dLeftFramePixelRate = fabs(v3LeftFrameCenter_NC * v3Normal_NC);
  double dRightFramePixelRate = fabs(v3RightFrameCenter_NC * v3Normal_NC);
  double dLeftFrameOneRightRate = fabs(v3OneRightFromLeftFrameCenter_NC * v3Normal_NC);
  double dRightFrameOneRightRate = fabs(v3OneRightFromRightFrameCenter_NC * v3Normal_NC);
  double dLeftFrameOneDownRate = fabs(v3OneDownFromLeftFrameCenter_NC * v3Normal_NC);
  double dRightFrameOneDownRate = fabs(v3OneDownFromRightFrameCenter_NC * v3Normal_NC);
  
  // Find projections onto plane
  Vector<3> v3CenterOnLeftPlane_C = v3LeftFrameCenter_NC * dLeftCamHeight / dLeftFramePixelRate;
  Vector<3> v3OneRightOnLeftPlane_C = v3OneRightFromLeftFrameCenter_NC * dLeftCamHeight / dLeftFrameOneRightRate;
  Vector<3> v3OneDownOnLeftPlane_C = v3OneDownFromLeftFrameCenter_NC * dLeftCamHeight / dLeftFrameOneDownRate;

  Vector<3> v3CenterOnRightPlane_C = v3RightFrameCenter_NC * dRightCamHeight / dRightFramePixelRate;
  Vector<3> v3OneRightOnRightPlane_C = v3OneRightFromRightFrameCenter_NC * dRightCamHeight / dRightFrameOneRightRate;
  Vector<3> v3OneDownOnRightPlane_C = v3OneDownFromRightFrameCenter_NC * dRightCamHeight / dRightFrameOneDownRate;
  
  // Find differences of these projections in the world frame
  v3LeftFramePixelRight_W = k.se3LeftCfromW.get_rotation().inverse() * (v3OneRightOnLeftPlane_C - v3CenterOnLeftPlane_C);
  v3LeftFramePixelDown_W = k.se3LeftCfromW.get_rotation().inverse() * (v3OneDownOnLeftPlane_C - v3CenterOnLeftPlane_C);
  
  v3RightFramePixelRight_W = k.se3RightCfromW.get_rotation().inverse() * (v3OneRightOnRightPlane_C - v3CenterOnRightPlane_C);
  v3RightFramePixelDown_W = k.se3RightCfromW.get_rotation().inverse() * (v3OneDownOnRightPlane_C - v3CenterOnRightPlane_C);

}  
