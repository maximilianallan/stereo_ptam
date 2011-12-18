// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __BUNDLE_H
#define __BUNDLE_H
// Bundle.h
// 
// This file declares the Bundle class along with a few helper classes.
// Bundle is the bundle adjustment core of the mapping system; instances
// of Bundle are generated by MapMaker to adjust the positions of 
// keyframes (called Cameras in this file) and map points.
//
// It's a pretty straight-forward Levenberg-Marquardt bundle adjustment 
// implementation closely following Hartley and Zisserman's MVG book, with
// the addition of a robust M-Estimator.
//
// Unfortunately, having undergone a few tweaks, the code is now
// not the easiest to read!
//
// Basic operation: MapMaker creates a new Bundle object;
// then adds map points and keyframes to adjust;
// then adds measurements of map points in keyframes;
// then calls Compute() to do bundle adjustment;
// then reads results back to update the map.

#include "ATANCamera.h"
#include "ATANCamera.h"
#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <vector>
#include <map>
#include <set>
#include <list>

struct Camera
{
  bool bFixed;
  SE3<> se3CfW;
  SE3<> se3CfWNew;
  Matrix<6> m6U;          
  Vector<6> v6EpsilonA;   
  int nStartRow;
};

struct OffDiagScriptEntry
{
  int j;
  int k;
};

struct Point
{
  inline Point()
  { nMeasurements = 0; nOutliers = 0;}
  Vector<3> v3Pos;
  Vector<3> v3PosNew;
  Matrix<3> m3V;
  Vector<3> v3EpsilonB;
  Matrix<3> m3VStarInv;
  
  int nMeasurements;
  int nOutliers;
  std::set<int> sCameras;
  std::vector<OffDiagScriptEntry> vOffDiagonalScript; 
};


struct Meas
{
  inline Meas()
  {bBad = false;}
  
  int p;
  int c;

  inline bool operator<(const Meas &rhs) const
  {  return(c<rhs.c ||(c==rhs.c && p < rhs.p)); }
  
  bool bBad;
  // Data structures for stereo measurements
  Vector<2> v2FoundInLeft;
  Vector<2> v2EpsilonLeft;
  Vector<2> v2FoundInRight;
  Vector<2> v2EpsilonRight;
  Matrix<4,6> m46A;
  Matrix<4,3> m43B;
  Matrix<6,3> m63W; 
  Matrix<6,3> m63Y;
  double dSqrtInvNoise;
  
  Vector<3> v3Cam;
  double dErrorSquared;
  // 4x4 matrix of camera derivs
  Matrix<4> m4CamDerivs;
};

// Core bundle adjustment class
class Bundle
{
public:

  Bundle(const ATANCamera &TCamL, const ATANCamera &TCamR);
  int AddCamera(SE3<> se3CamFromWorld, bool bFixed); 
  int AddPoint(Vector<3> v3Pos);       
  void AddMeas(int nCam, int nPoint, Vector<4> v4Pos, double dSigmaSquared); // Add a measurement as a 4-vector
  int Compute(bool *pbAbortSignal);    
  inline bool Converged() { return mbConverged;}  
  Vector<3> GetPoint(int n);
  SE3<> GetCamera(int n); 
  std::vector<std::pair<int,int> > GetOutlierMeasurements();  
  std::set<int> GetOutliers();                               
  
protected:

  inline void ProjectAndFindSquaredError(Meas &meas); // Project a single point in a single view, compare to measurement this is performed over stereo meas
  template<class MEstimator> bool Do_LM_Step(bool *pbAbortSignal);
  template<class MEstimator> double FindNewError();
  void GenerateMeasLUTs();
  void GenerateOffDiagScripts();
  void ClearAccumulators(); 
  void ModifyLambda_GoodStep();
  void ModifyLambda_BadStep();
  
  std::vector<Point> mvPoints;
  std::vector<Camera> mvCameras;
  std::list<Meas> mMeasList; //all measurements (both frames)
  std::vector<std::pair<int,int> > mvOutlierMeasurementIdx; 
  std::vector<std::vector<Meas*> > mvMeasLUTs;  
  //Each camera gets a per-point table of pointers to valid measurements
  
  ATANCamera mCamera_Left; //two camera 
  ATANCamera mCamera_Right;
  int mnCamsToUpdate;
  int mnStartRow;
  double mdSigmaSquared;
  double mdLambda;
  double mdLambdaFactor;
  bool mbConverged;
  bool mbHitMaxIterations;
  int mnCounter;
  int mnAccepted;
  
  GVars3::gvar3<int> mgvnMaxIterations;
  GVars3::gvar3<double> mgvdUpdateConvergenceLimit;
  GVars3::gvar3<int> mgvnBundleCout;
  
  bool *mpbAbortSignal;
};





#endif
