// *-* c++ *-*
// Copyright 2008 Isis Innovation Limited


//
// BEWARE: This camera model caches intermediate results in member variables
// Some functions therefore depend on being called in order: i.e.
// GetProjectionDerivs() uses data stored from the last Project() or UnProject()
// THIS MEANS YOU MUST BE CAREFUL WITH MULTIPLE THREADS
// Best bet is to give each thread its own version of the camera!
//
// Camera parameters are stored in a GVar, but changing the gvar has no effect
// until the next call to RefreshParams() or SetImageSize().
//
// Pixel conventions are as follows:
// For Project() and Unproject(),
// round pixel values - i.e. (0.0, 0.0) - refer to pixel centers
// I.e. the top left pixel in the image covers is centered on (0,0)
// and covers the area (-.5, -.5) to (.5, .5)
//
// Be aware that this is not the same as what opengl uses but makes sense
// for acessing pixels using ImageRef, especially ir_rounded.
//
// What is the UFB?
// This is for projecting the visible image area
// to a unit square coordinate system, with the top-left at 0,0,
// and the bottom-right at 1,1
// This is useful for rendering into textures! The top-left pixel is NOT
// centered at 0,0, rather the top-left corner of the top-left pixel is at 
// 0,0!!! This is the way OpenGL thinks of pixel coords.
// There's the Linear and the Distorting version - 
// For the linear version, can use 
// glMatrixMode(GL_PROJECTION); glLoadIdentity();
// glMultMatrix(Camera.MakeUFBLinearFrustumMatrix(near,far));
// To render un-distorted geometry with full frame coverage.
//

#ifndef __ATAN_CAMERA_H
#define __ATAN_CAMERA_H
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

//#define NUMTRACKERCAMPARAMETERS 5

class CameraCalibrator;
class CalibImage;

// The parameters are:
// 0 - normalized x focal length
// 1 - normalized y focal length
// 2 - normalized x offset
// 3 - normalized y offset
// 4 - w (distortion parameter);

class ATANCamera{
 public:
  // constructors
  ATANCamera();
  ATANCamera(std::string sName);
  
  void SetImageSize(Vector<2> v2ImageSize);
  inline void SetImageSize(CVD::ImageRef irImageSize) {SetImageSize(vec(irImageSize));};
  inline Vector<2> GetImageSize() {return mvImageSize;};
  
  //change parameters according to gVars etc
  void RefreshParams();
  
  // Various projection functions
  Vector<2> Project(const Vector<2>& camframe); 
  inline Vector<2> Project(CVD::ImageRef ir) { return Project(vec(ir)); }
  Vector<2> UnProject(const Vector<2>& imframe); // Inverse operation
  inline Vector<2> UnProject(CVD::ImageRef ir)  { return UnProject(vec(ir)); }
  Vector<2> UFBProject(const Vector<2>& camframe);
  Vector<2> UFBUnProject(const Vector<2>& camframe);
  inline Vector<2> UFBLinearProject(const Vector<2>& camframe);
  inline Vector<2> UFBLinearUnProject(const Vector<2>& fbframe);
  
  // Returns the fundamental matrix
  Matrix<3> GetFundamentalMatrix(Vector<3> v3C, ATANCamera &Cam, Vector<3> LeftCamCenter);
  
  // Return the epipolar line for a given point in view of the camera
  inline Vector<3> GetEpipolarLine(Vector<3> v3A, Matrix<3> F)
  {
    return F*v3A;
  }

  Vector<3> UnProjectToWorld(const Vector<2> imLPoint, const Vector<2> imRPoint);
  inline Vector<3> UnProjectToWorld(const CVD::ImageRef imLPoint, const CVD::ImageRef imRPoint){
    return UnProjectToWorld(vec(imLPoint), vec(imRPoint));
  }
  
  Matrix<2,2> GetProjectionDerivs(); 

  // return the tranformation for this camera to the other
  inline SE3<> GetExtrinsic(){ return Extrinsic;}

  // return just the x-baseline
  inline double getStereoBaseline(){
    return sqrt(Extrinsic.get_translation()[0]*Extrinsic.get_translation()[0]);
  }
  // return the camera matrix
  inline Matrix<3> GetCameraMatrix(){ return mvCameraMatrix;}
  
  inline bool Invalid() {  return mbInvalid;}
  inline double LargestRadiusInImage() {  return mdLargestRadius; }
  inline double OnePixelDist() { return mdOnePixelDist; }
  inline double getmdW(){ return mdW;}  
  // return the focal length
  inline double getFocal(int n){
    if(n==0||n==1)
      return mvFocal[n];
    else return 0;
    }
  // return the principal point
  inline double getPrincipalPoint(int n){
    if(n == 0 || n == 1)
      return mvCenter[n];
    else
      return 0;
  }
  
  inline Vector<2> ImplaneTL(); 
  inline Vector<2> ImplaneBR(); 
  Matrix<4> MakeUFBLinearFrustumMatrix(double near, double far);
  double PixelAspectRatio() { return mvFocal[1] / mvFocal[0];}
  static const Vector<NUMTRACKERCAMPARAMETERS> mvDefaultParams;
  // defaul vector for stereo transformation
  static const Vector<6> mvDefaultExtrinsicVector;
  // the relative pose of this camera to its stereo pair
  SE3<> Extrinsic;
protected:
  //the Gvars
  GVars3::gvar3<Vector<NUMTRACKERCAMPARAMETERS> > mgvvCameraParams; 
  GVars3::gvar3<Vector<6> >mgvvSE3Parameters_Vector;

  Matrix<2, NUMTRACKERCAMPARAMETERS> GetCameraParameterDerivs();
  
  void UpdateParams(Vector<NUMTRACKERCAMPARAMETERS> vUpdate);
  //update the extrinsic parameters in the calibrator
  void UpdateExtrinsicParams(SE3<> extrinsic);
  void DisableRadialDistortion();

  Vector<2> mvLastCam;      
  Vector<2> mvLastIm;       
  Vector<2> mvLastDistCam;  
  Vector<3> mvLastWorld;    
  double mdLastR;           
  double mdLastDistR;       
  double mdLastFactor;      
  bool mbInvalid;           
  
  double mdLargestRadius; 
  double mdMaxR;          
  double mdOnePixelDist;  
  double md2Tan;          
  double mdOneOver2Tan;   
  double mdW;             
  double mdWinv;          
  double mdDistortionEnabled; 
  
  //CVD and TooN parameters

  Matrix<3> mvCameraMatrix;
  Matrix<4> mvReProjectMatrix;
  Matrix<3,4> mvProjectMatrix;
  
  Vector<2> mvCenter;
  Vector<2> mvFocal; 
  Vector<2> mvInvFocal;
  Vector<2> mvImageSize;  
  Vector<2> mvUFBLinearFocal;
  Vector<2> mvUFBLinearInvFocal;
  Vector<2> mvUFBLinearCenter;
  Vector<2> mvImplaneTL;   
  Vector<2> mvImplaneBR;
  
  //CvMat *cvR;
  //CvMat *cvT;
  
  inline double rtrans_factor(double r)
  {
    if(r < 0.001 || mdW == 0.0)
      return 1.0;
    else 
      return (mdWinv* atan(r * md2Tan) / r);
  };

  inline double invrtrans(double r)
  {
    if(mdW == 0.0)
      return r;
    return(tan(r * mdW) * mdOneOver2Tan);
  };
  
  std::string msName;

  friend class CameraCalibrator;   
  friend class CalibImage;
};

inline Vector<2> ATANCamera::UFBLinearProject(const Vector<2>& camframe)
{
  Vector<2> v2Res;
  v2Res[0] = camframe[0] * mvUFBLinearFocal[0] + mvUFBLinearCenter[0];
  v2Res[1] = camframe[1] * mvUFBLinearFocal[1] + mvUFBLinearCenter[1];
  return v2Res;
}

inline Vector<2> ATANCamera::UFBLinearUnProject(const Vector<2>& fbframe)
{
  Vector<2> v2Res;
  v2Res[0] = (fbframe[0] - mvUFBLinearCenter[0]) * mvUFBLinearInvFocal[0];
  v2Res[1] = (fbframe[1] - mvUFBLinearCenter[1]) * mvUFBLinearInvFocal[1];
  return v2Res;
}


#endif

