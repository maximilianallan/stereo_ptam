// Implementation of a rectified camera - as this is no longer used its presence
// is for reference only 
// contains functions for performing projections and unprojections with rectified
// cameras using opencv

#include "ATANRectifiedCamera.h"
#include <TooN/helpers.h>
#include <cvd/vector_image_ref.h>
#include <iostream>
#include <gvars3/instances.h>
using namespace std;
using namespace CVD;
using namespace GVars3;

ATANRectifiedCamera::ATANRectifiedCamera(CvMat *rectifiedProject, CvMat *ReProject, cv::Mat *mapX, cv::Mat *mapY)
{
  //  se3BetweenCams = Rt;
  
  
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      mvCameraMatrix(i,j) = cvmGet(rectifiedProject,i,j);
  
  for(int i=0;i<3;i++)
    for(int j=0;j<4;j++)
      mvProjectMatrix(i,j) = cvmGet(rectifiedProject,i,j);
  
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      mvReProjectMatrix(i,j) = cvmGet(ReProject,i,j);
  
  map1 = mapX;
  map2 = mapY;
  
  stereoBaseline = 1/mvReProjectMatrix(3,2);
  //stereoBaseline = sqrt(stereoBaseline*stereoBaseline);
  Rt.get_translation() = makeVector(-1*stereoBaseline,0,0);
  mvImageSize[0] = OPENCV_VIDEO_W;
  mvImageSize[1] = OPENCV_VIDEO_H;
  
  mvFocal[0] = mvFocal[1] = mvCameraMatrix(0,0);
  mvInvFocal[0] = 1/mvFocal[0];
  mvInvFocal[1] = 1/mvFocal[1];
  mvCenter[0] = mvProjectMatrix(0,2);
  mvCenter[1] = mvProjectMatrix(1,2);
  RefreshParams();
  
}

void ATANRectifiedCamera::RefreshParams(){

  Vector<2> v2;
  double prX_N = mvCenter[0]/mvImageSize[0];
  double prY_N = mvCenter[1]/mvImageSize[1];
  double fcX_N = mvFocal[0]/mvImageSize[0];
  double fcY_N = mvFocal[1]/mvImageSize[1];
  v2[0]= max(prX_N, 1.0 - prX_N) / fcX_N;
  v2[1]= max(prY_N, 1.0 - prY_N) / fcY_N;
  mdLargestRadius = sqrt(v2*v2);
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    Vector<2> v2Center = UnProject(mvImageSize / 2);
    Vector<2> v2RootTwoAway = UnProject(mvImageSize / 2 + vec(ImageRef(1,1)));
    Vector<2> v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = sqrt(v2Diff * v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<Vector<2> > vv2Verts;
    vv2Verts.push_back(UnProject(makeVector( -0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, mvImageSize[1]-0.5)));
    vv2Verts.push_back(UnProject(makeVector( -0.5, mvImageSize[1]-0.5)));
    Vector<2> v2Min = vv2Verts[0];
    Vector<2> v2Max = vv2Verts[0];
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++)
	{
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    Vector<2> v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }
}
//project to screen coordinates from world coordinates
Vector<2> ATANRectifiedCamera::RectifiedProject(const Vector<3> pointCoord){
  
  Vector<3> tmp = pointCoord;
  normalizeLast(tmp);
  mvLastCam = tmp.slice(0,2);
  mdLastR = sqrt(mvLastCam * mvLastCam);
  cout << "mdLastR is set to " << mdLastR << " in rectifiedProjet "<< endl;
  mbInvalid = (mdLastR > mdMaxR);
  
  Vector<4> point4Coord = unproject(pointCoord);
 
  tmp = mvProjectMatrix*point4Coord;
  normalizeLast(tmp);
  mvLastIm[0] = tmp[0];
  mvLastIm[1] = tmp[1];
  return mvLastIm;
}
//rectified unproject takes a point in the left image and projects in into
//a 3 vector representing the point's location in 3d space.
Vector<3> ATANRectifiedCamera::UnProjectToWorld(const Vector<2> imgPoint, float disparity){

  //modify x,y to x-c_x and y-c_y
  mvLastIm = imgPoint;
  Vector<4> inV;
  inV.slice(0,2) = imgPoint;
  inV[2] = disparity;
  inV[3] = 1;
  if((inV[2]*inV[2]) < 0.00001){
    cout << "possible error disparity v low! " << endl;
    inV[2] = 0.001;
  }
  Vector<4> outV = mvReProjectMatrix*inV;
  Vector<4> outV3 = normalizeLast(outV);
      
  if(outV3[3] != 1){
    cout << outV3[3] << " is the value of out " << endl;
    cout << "ERROR - normalizeLast is not doing its job"<< endl;
  }
  mvLastCam[0] = outV3[0]/outV3[2];
  mvLastCam[1] = outV3[1]/outV3[2];
  
  mdLastR = sqrt(mvLastCam[0]*mvLastCam[0] + mvLastCam[1]*mvLastCam[1]);
  mvLastWorld = outV3.slice(0,3);
  return mvLastWorld;
}

Vector<3> ATANRectifiedCamera::UnProjectToWorld(const ImageRef imPoint, float disparity){
  return UnProjectToWorld(vec(imPoint),disparity);
}

void ATANRectifiedCamera::SetImageSize(Vector<2> vImageSize)
{
  mvImageSize = vImageSize;
  RefreshParams();
};

// Project from the camera z=1 plane to image pixels,
// while storing intermediate calculation results in member variables
Vector<2> ATANRectifiedCamera::Project(const Vector<2>& vCam){
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam); 
  mbInvalid = (mdLastR > mdMaxR);
  
  mvLastIm[0] = mvCenter[0] + (mvFocal[0] * mvLastCam[0]);
  mvLastIm[1] = mvCenter[1] + (mvFocal[1] * mvLastCam[1]);
  
  return mvLastIm;
}


// Un-project from image pixel coords to the camera z=1 plane
// while storing intermediate calculation results in member variables

Vector<2> ATANRectifiedCamera::UnProject(const Vector<2>& v2Im)
{
  
  mvLastIm = v2Im;
  
  mvLastCam[0] = (mvLastIm[0]-mvCenter[0]) * mvInvFocal[0];
  mvLastCam[1] = (mvLastIm[1]-mvCenter[1]) * mvInvFocal[1];
    
 //subtracts principle points and divides by focal lengths
  mdLastR = sqrt(mvLastCam * mvLastCam);
  return mvLastCam;
}

// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANRectifiedCamera.h
Matrix<4> ATANRectifiedCamera::MakeUFBLinearFrustumMatrix(double near, double far)
{
  Matrix<4> m4 = Zeros;
  

  double left = mvImplaneTL[0] * near;
  double right = mvImplaneBR[0] * near;
  double top = mvImplaneTL[1] * near;
  double bottom = mvImplaneBR[1] * near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4[0][0] = (2 * near) / (right - left);
  m4[1][1] = (2 * near) / (top - bottom);
  
  m4[0][2] = (right + left) / (left - right);
  m4[1][2] = (top + bottom) / (bottom - top);
  m4[2][2] = (far + near) / (far - near);
  m4[3][2] = 1;
  
  m4[2][3] = 2*near*far / (near - far);

  return m4;
};

Matrix<2,2> ATANRectifiedCamera::GetProjectionDerivs()
{
  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (du/dx, du/dx)
  //             (dv/dy, dv/dy)
  //This is done for the z=1 plane. As there is no distortion 
  //this reduces to just the focal length
  Matrix<2> m2Derivs;
  m2Derivs[0][0] = mvFocal[0];
  m2Derivs[1][0] = 0;
  m2Derivs[0][1] = 0;
  m2Derivs[1][1] = mvFocal[1];  
  return m2Derivs;
}


Vector<2> ATANRectifiedCamera::UFBProject(const Vector<2>& vCam)
{
  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam);
  mbInvalid = (mdLastR > mdMaxR);
    
  mvLastIm[0] = (mvCenter[0]/mvImageSize[0]) + (mvFocal[0]/mvImageSize[0]) * mvLastCam[0];
  mvLastIm[1] = (mvCenter[1]/mvImageSize[1]) + (mvFocal[1]/mvImageSize[1]) * mvLastCam[1];
  return mvLastIm;
}

Vector<2> ATANRectifiedCamera::UFBUnProject(const Vector<2>& v2Im)
{
  mvLastIm = v2Im;
  mvLastCam[0] = (mvLastIm[0] - (mvCenter[0]/mvImageSize[0])) / (mvCenter[0]/mvImageSize[0]);
  mvLastCam[1] = (mvLastIm[1] - (mvCenter[1]/mvImageSize[1])) / (mvCenter[1]/mvImageSize[1]);
  mdLastR = sqrt(mvLastCam * mvLastCam);
  return mvLastCam;
}

cv::Mat *ATANRectifiedCamera::getMap1(){
  return map1;
}

cv::Mat *ATANRectifiedCamera::getMap2(){
  return map2;
}

CvRect *ATANRectifiedCamera::getCvRectangle(){
  return cvRectangle;
}



