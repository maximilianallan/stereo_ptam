#ifndef __RECTIFIED__CAMERA_H
#define __RECTIFIED__CAMERA_H

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

class CameraCalibrator;
class CalibImage;

// The parameters are:
// 0 - normalized x focal length
// 1 - normalized y focal length
// 2 - normalized x offset
// 3 - normalized y offset
// 4 - w (distortion parameter)


//class for a rectified ATANCamera rig
class ATANRectifiedCamera{
  
 public:
  ATANRectifiedCamera();
  ATANRectifiedCamera(CvMat *rectifiedProject, CvMat *ReProject, cv::Mat *mapX, cv::Mat *mapY);
  void deleteCv();
  
  //set image size

  void SetImageSize(Vector<2> v2ImageSize);
  inline void SetImageSize(CVD::ImageRef irImageSize) {SetImageSize(vec(irImageSize));};
  inline Vector<2> GetImageSize() {return mvImageSize;};

  //refresh params

  void RefreshParams();

  //project and unproject

  Vector<2> Project(const Vector<2>& camframe); 
  Vector<2> RectifiedProject(const Vector<3>pointCoord);
  inline Vector<2> Project(CVD::ImageRef ir) { return Project(vec(ir)); }
  Vector<2> UnProject(const Vector<2>& imframe); // Inverse operation
  inline Vector<2> UnProject(CVD::ImageRef ir)  { return UnProject(vec(ir)); }
  Vector<2> UFBProject(const Vector<2>& camframe);
  Vector<2> UFBUnProject(const Vector<2>& camframe);
  inline Vector<2> UFBLinearProject(const Vector<2>& camframe);
  inline Vector<2> UFBLinearUnProject(const Vector<2>& fbframe);

  //and their rectified versions
  //Vector<2> RectifiedProject(const Vector<3> pointcoord);
  Vector<3> UnProjectToWorld(const Vector<2> imPoint, float disparity);
  Vector<3> UnProjectToWorld(const CVD::ImageRef imPoint, float disparity);
  //unproject to z = 1 plane
  //Vector<2> RectifiedUnProject(const Vector<2> imPoint);
  //Vector<2> RectifiedUnProject(const CVD::ImageRef imPoint);
   
  Matrix<2,2> GetProjectionDerivs(); // Projection jacobian
  
  //some getters
  inline double getStereoBaseline(){
    return stereoBaseline;
  }
  inline bool Invalid() {  return mbInvalid;}
  inline double LargestRadiusInImage() {  return mdLargestRadius; }
  inline double OnePixelDist() { return mdOnePixelDist; }
  inline double getFocal(int n){
    if(n==0||n==1)
      return mvFocal[n];
    else return 0;
    }
  // The z=1 plane bounding box of what the camera can see
  inline Vector<2> ImplaneTL(); 
  inline Vector<2> ImplaneBR(); 
  inline double getPrincipalPoint(int n){
    if(n == 0 || n == 1)
      return mvCenter[n];
    else
      return 0;
  }
  SE3<> Rt;
  //openCV getters
  cv::Mat *getMap1();
  cv::Mat *getMap2();
  CvRect *getCvRectangle();

  inline double getFT(){
    return mvProjectMatrix(0,3);
  }
  // OpenGL helper function
  Matrix<4> MakeUFBLinearFrustumMatrix(double near, double far);

  // Feedback for Camera Calibrator
  double PixelAspectRatio() { return mvFocal[1] / mvFocal[0];}
  
  // Useful for gvar-related reasons (in case some external func tries to read the camera params gvar, and needs some defaults.)
 protected:
  //the Gvars
  
  Matrix<2, NUMTRACKERCAMPARAMETERS> GetCameraParameterDerivs();
  
  // Cached from the last project/unproject:
  Vector<2> mvLastCam;      // Last z=1 coord
  Vector<2> mvLastIm;       // Last image/UFB coord
  Vector<3> mvLastWorld;    // Last world coordiante
  double mdLastR;           // Last z=1 radius
  bool mbInvalid;           // Was the last projection invalid?
  
  double stereoBaseline;
  // Cached from last RefreshParams:
  double mdLargestRadius; // Largest R in the image
  double mdMaxR;          // Largest R for which we consider projection valid
  double mdOnePixelDist;  // z=1 distance covered by a single pixel offset (a rough estimate!)
    
  //OPENCV MATRICES - duplicates a lot of functionality... //
  
  cv::Mat *map1;
  cv::Mat *map2;
  CvRect *cvRectangle;

  //CVD and TooN parameters
  Matrix<3> mvCameraMatrix;
  Matrix<4> mvReProjectMatrix;
  Matrix<3,4> mvProjectMatrix;
  SE3<> Extrinsic;
  Vector<2> mvCenter;     // Pixel projection center
  Vector<2> mvFocal;      // Pixel focal length
  Vector<2> mvInvFocal;   // Inverse pixel focal length
  Vector<2> mvImageSize;  
  Vector<2> mvUFBLinearFocal;
  Vector<2> mvUFBLinearInvFocal;
  Vector<2> mvUFBLinearCenter;
  Vector<2> mvImplaneTL;   
  Vector<2> mvImplaneBR;
  
  std::string msName;

  friend class CameraCalibrator;   // friend declarations allow access to calibration jacobian and camera update function.
  friend class CalibImage;
};

// Some inline projection functions:
inline Vector<2> ATANRectifiedCamera::UFBLinearProject(const Vector<2>& camframe)
{
  Vector<2> v2Res;
  v2Res[0] = camframe[0] * mvUFBLinearFocal[0] + mvUFBLinearCenter[0];
  v2Res[1] = camframe[1] * mvUFBLinearFocal[1] + mvUFBLinearCenter[1];
  return v2Res;
}

inline Vector<2> ATANRectifiedCamera::UFBLinearUnProject(const Vector<2>& fbframe)
{
  Vector<2> v2Res;
  v2Res[0] = (fbframe[0] - mvUFBLinearCenter[0]) * mvUFBLinearInvFocal[0];
  v2Res[1] = (fbframe[1] - mvUFBLinearCenter[1]) * mvUFBLinearInvFocal[1];
  return v2Res;
}


#endif


