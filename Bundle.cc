// Copyright 2008 Isis Innovation Limited
// The bundle adjuster class implemented with stereo measurements
// measurements are passed as a four-vector and the camera pose is updated to 
// minimise the reproejction error over all 4 parameters

#include "Bundle.h"
#include "MEstimator.h"
#include <TooN/helpers.h>
#include <TooN/Cholesky.h>
#include <fstream>
#include <iomanip>
#include <gvars3/instances.h>

using namespace GVars3;
using namespace std;

#ifdef WIN32
inline bool isnan(double d) {return !(d==d);}
#endif

#define cout if(*mgvnBundleCout) cout

// Requires 2nd arg to now be 4x6 matrix due to new meas structure
inline void BundleTriangle_UpdateM6U_LL(Matrix<6> &m6U, Matrix<4,6> &m46A)
{
  for(int r=0; r<6; r++)
    for(int c=0; c<=r; c++)
      m6U(r,c)+= m46A.T()(r,0)*m46A(0,c) + m46A.T()(r,1)*m46A(1,c);
}
// requires second arg to be a 4x3 due to new meas structure
inline void BundleTriangle_UpdateM3V_LL(Matrix<3> &m3V, Matrix<4,3> &m43B)
{
  for(int r=0; r<3; r++) 
    for(int c=0; c<=r; c++) 
      m3V(r,c)+= m43B.T()(r,0)*m43B(0,c) + m43B.T()(r,1)*m43B(1,c); 
}

Bundle::Bundle(const ATANCamera &TCamL, const ATANCamera &TCamR)
  : mCamera_Left(TCamL), mCamera_Right(TCamR)
{
  mnCamsToUpdate = 0;
  mnStartRow = 0;
  GV3::Register(mgvnMaxIterations, "Bundle.MaxIterations", 20,  SILENT);
  GV3::Register(mgvdUpdateConvergenceLimit, "Bundle.UpdateSquaredConvergenceLimit", 1e-06, SILENT);
  GV3::Register(mgvnBundleCout, "Bundle.Cout", 0, SILENT);
};

// Add a camera to the system, return value is the bundle adjuster's ID for the camera
// the camera pose if the tracker pose - left camera
int Bundle::AddCamera(SE3<> se3CamFromWorld, bool bFixed)
{
  int n = mvCameras.size();
  Camera c;
  c.bFixed = bFixed;
  c.se3CfW = se3CamFromWorld;
  if(!bFixed)
    {
      c.nStartRow = mnStartRow;
      mnStartRow += 6;
      mnCamsToUpdate++;
    }
  else
    c.nStartRow = -999999999; 
  mvCameras.push_back(c);
   
  return n;
}

int Bundle::AddPoint(Vector<3> v3Pos)
{
  int n = mvPoints.size();
  Point p;
  if(isnan(v3Pos * v3Pos))
    {
      cerr << " You sucker, tried to give me a nan " << v3Pos << endl;
      v3Pos = Zeros;
    }
  p.v3Pos = v3Pos;
  mvPoints.push_back(p);
  return n;
}

// Add a measurement of one point with one camera - this is now a 4 vector
void Bundle::AddMeas(int nCam, int nPoint, Vector<4> v4Pos, double dSigmaSquared)
{
  assert(nCam < (int) mvCameras.size());
  assert(nPoint < (int) mvPoints.size());
  mvPoints[nPoint].nMeasurements++;
  mvPoints[nPoint].sCameras.insert(nCam);
  Meas m;
  m.p = nPoint;
  m.c = nCam;
  // set the measurement image parameters
  m.v2FoundInLeft = v4Pos.slice(0,2);
  m.v2FoundInRight = v4Pos.slice(2,2);
  m.dSqrtInvNoise = sqrt(1.0 / dSigmaSquared);
  mMeasList.push_back(m);
}

void Bundle::ClearAccumulators()
{
  for(size_t i=0; i<mvPoints.size(); ++i)
    {
      mvPoints[i].m3V = Zeros;
      mvPoints[i].v3EpsilonB = Zeros;
    }
  for(size_t i=0; i<mvCameras.size(); ++i)
    {
      mvCameras[i].m6U = Zeros;
      mvCameras[i].v6EpsilonA = Zeros;
    }
}


int Bundle::Compute(bool *pbAbortSignal)
{
  mpbAbortSignal = pbAbortSignal;

  GenerateMeasLUTs();
  GenerateOffDiagScripts();

  mdLambda = 0.0001;
  mdLambdaFactor = 2.0;
  mbConverged = false;
  mbHitMaxIterations = false;
  mnCounter = 0;
  mnAccepted = 0;
  
  static gvar3<string> gvsMEstimator("BundleMEstimator", "Tukey", SILENT);
  
  while(!mbConverged  && !mbHitMaxIterations && !*pbAbortSignal)
    {
      bool bNoError;
      if(*gvsMEstimator == "Cauchy")
	bNoError = Do_LM_Step<Cauchy>(pbAbortSignal);  
      else if(*gvsMEstimator == "Tukey")
	bNoError = Do_LM_Step<Tukey>(pbAbortSignal);
      else if(*gvsMEstimator == "Huber")
	bNoError = Do_LM_Step<Huber>(pbAbortSignal);
      else
	{
	  cout << "Invalid BundleMEstimator selected !! " << endl;
	  cout << "Defaulting to Tukey." << endl;
	  *gvsMEstimator = "Tukey";
	  bNoError = Do_LM_Step<Tukey>(pbAbortSignal);
	};
      
      if(!bNoError)
	return -1;
    }
  
  if(mbHitMaxIterations)
    cout << "  Hit max iterations." << endl;
  cout << "Final Sigma Squared: " << mdSigmaSquared << " (= " << sqrt(mdSigmaSquared) / 4.685 << " pixels.)" << endl;
  return mnAccepted;
};

// Reproject a single measurement, find error
// do this into the correct camera frame!
inline void Bundle::ProjectAndFindSquaredError(Meas &meas)
{
  Camera &cam = mvCameras[meas.c];
  Point &point = mvPoints[meas.p];
  meas.v3Cam = cam.se3CfW * point.v3Pos;
  if(meas.v3Cam[2] <= 0)
    {
      meas.bBad = true;
      return;
    }
  meas.bBad = false;
  // project to the left camera first
  Vector<2> v2ImPlane = project(meas.v3Cam);
  Vector<2> v2Image   = mCamera_Left.Project(v2ImPlane);
  // check the matrix is zero...
  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      meas.m4CamDerivs(i,j) = 0;
  // set the 4x4 of camera derivs with the left cam derivs
  meas.m4CamDerivs.slice(0,0,2,2) = mCamera_Left.GetProjectionDerivs();
  meas.v2EpsilonLeft = meas.dSqrtInvNoise * (meas.v2FoundInLeft - v2Image);
  meas.dErrorSquared = meas.v2EpsilonLeft * meas.v2EpsilonLeft;
  // perform the same proccess with the right camera
  v2ImPlane = project(mCamera_Left.GetExtrinsic()*meas.v3Cam);
  v2Image = mCamera_Right.Project(v2ImPlane);
  meas.m4CamDerivs.slice(2,2,2,2) = mCamera_Right.GetProjectionDerivs();
  meas.v2EpsilonRight = meas.dSqrtInvNoise * (meas.v2FoundInRight - v2Image);
  meas.dErrorSquared += meas.v2EpsilonRight * meas.v2EpsilonRight;
  // set the measurement error to the mean error between the measurements
  meas.dErrorSquared /= 2;
}

template<class MEstimator>
bool Bundle::Do_LM_Step(bool *pbAbortSignal)
{
  ClearAccumulators();

  vector<double> vdErrorSquared;
  for(list<Meas>::iterator itr = mMeasList.begin(); itr!=mMeasList.end(); itr++)
    {
      Meas &meas = *itr;
      //finds error over both views
      ProjectAndFindSquaredError(meas);
      if(!meas.bBad)
	vdErrorSquared.push_back(meas.dErrorSquared);
    };
  
  mdSigmaSquared = MEstimator::FindSigmaSquared(vdErrorSquared);
  static gvar3<double> gvdMinSigma("Bundle.MinTukeySigma", 0.4, SILENT);
  const double dMinSigmaSquared = *gvdMinSigma * *gvdMinSigma;
  if(mdSigmaSquared < dMinSigmaSquared)
    mdSigmaSquared = dMinSigmaSquared;
  double dCurrentError = 0.0;
  for(list<Meas>::iterator itr = mMeasList.begin(); itr!=mMeasList.end(); itr++)
    {
      Meas &meas = *itr;
      Camera &cam = mvCameras[meas.c];
      Point &point = mvPoints[meas.p];
      
      if(meas.bBad)
	{
	  dCurrentError += 1.0;
	  continue;
	};
      
      double dWeight= MEstimator::SquareRootWeight(meas.dErrorSquared, mdSigmaSquared);
      
      // reweight both left and right measurements
      meas.v2EpsilonLeft = dWeight * meas.v2EpsilonLeft;
      meas.v2EpsilonRight = dWeight * meas.v2EpsilonRight;
      
      if(dWeight == 0)
	{
	  meas.bBad = true;  
	  dCurrentError += 1.0;
	  continue;
	}
      
      dCurrentError += MEstimator::ObjectiveScore(meas.dErrorSquared, mdSigmaSquared);
      
      //
      //calculate the jacobian for camera pose over the 4 measurement values
      //

      //get the camera derivtives for both cameras 
      Matrix<4> m4CamDerivs = dWeight * meas.m4CamDerivs;
      
      // get the point coords in left camera frame
      const double dOneOverLeftCameraZ = 1.0 / meas.v3Cam[2];
      const Vector<4> v4LeftCam = unproject(meas.v3Cam);

      // get the point coods in right camera frame
      const double dOneOverRightCameraZ = 1.0 / (mCamera_Left.GetExtrinsic()*meas.v3Cam)[2];
      const Vector<4> v4RightCam = unproject(mCamera_Left.GetExtrinsic()*meas.v3Cam);
      
      if(cam.bFixed)
	meas.m46A = Zeros;
      else 
	{
	  for(int m=0;m<6;m++)
	    {
	      // get the generators for both left and right cameras
	      const Vector<4> v4LeftMotion = SE3<>::generator_field(m, v4LeftCam);
	      const Vector<4> v4RightMotion = SE3<>::generator_field(m, v4RightCam);
	      // calculat teh derivative dLeft/d mu_{i}
 	      Vector<4> v4CamFrameMotion;
 	      v4CamFrameMotion[0] = (v4LeftMotion[0] - v4LeftCam[0] * v4LeftMotion[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
 	      v4CamFrameMotion[1] = (v4LeftMotion[1] - v4LeftCam[1] * v4LeftMotion[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;

	      // calculate the derivative dRight/d mu_{i}
	      Vector<2> v2RightCamFrameMotion;
 	      v4CamFrameMotion[2] = (v4RightMotion[0] - v4RightCam[0] * v4RightMotion[2] * dOneOverRightCameraZ) * dOneOverRightCameraZ;
 	      v4CamFrameMotion[3] = (v4RightMotion[1] - v4RightCam[1] * v4RightMotion[2] * dOneOverRightCameraZ) * dOneOverRightCameraZ;
	      meas.m46A.T()[m] = meas.dSqrtInvNoise * m4CamDerivs * v4CamFrameMotion;
	    };
	}
      
      // calculate the derivatives for the left view of the point
      
      Matrix<3> m3LeftMotion = 
	cam.se3CfW.get_rotation().get_matrix();

      // calculate the derivatives of right camera view of point
      Matrix<3> m3RightMotion = 
	(mCamera_Left.Extrinsic * cam.se3CfW).get_rotation().get_matrix();
      
      for(int m=0;m<3;m++)
	{
	  // get the matrix rows
	  const Vector<3> v3LeftMotion = m3LeftMotion.T()[m]; 
	  const Vector<3> v3RightMotion = m3RightMotion.T()[m]; 
	  
	  // derivs for left cam in each degree of (x,y,z) freedom
	  Vector<4> v4CamFrameMotion;
	  v4CamFrameMotion[0] = (v3LeftMotion[0] - v4LeftCam[0] * v3LeftMotion[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
	  v4CamFrameMotion[1] = (v3LeftMotion[1] - v4LeftCam[1] * v3LeftMotion[2] * dOneOverLeftCameraZ) * dOneOverLeftCameraZ;
	  
	  // derivs for right cam in (x,y,z) parmeters
	  v4CamFrameMotion[2] = (v3RightMotion[0]-v4RightCam[0] * v3RightMotion[2] * dOneOverRightCameraZ)*dOneOverRightCameraZ;
	  v4CamFrameMotion[3] = (v3RightMotion[1]-v4RightCam[1] * v3RightMotion[2] * dOneOverRightCameraZ)*dOneOverRightCameraZ;

	  // multiply each deriv by the respective camera projection derivs
	  meas.m43B.T()[m] = meas.dSqrtInvNoise * m4CamDerivs * v4CamFrameMotion;
	};
      
      // get the error vectors epsilon
      Vector<4> v4Ep;
      v4Ep.slice(0,2) = meas.v2EpsilonLeft;
      v4Ep.slice(2,2) = meas.v2EpsilonRight;

      if(!cam.bFixed)
	{
	  // add the 4-vector measurement
	  BundleTriangle_UpdateM6U_LL(cam.m6U, meas.m46A);
	  cam.v6EpsilonA += meas.m46A.T() * v4Ep;
	}

      // add the 4-vector measurement
      BundleTriangle_UpdateM3V_LL(point.m3V, meas.m43B);
      // add the 4-vector error
      point.v3EpsilonB += meas.m43B.T() * v4Ep;
      
      if(cam.bFixed)
	meas.m63W = Zeros;
      else
	meas.m63W = meas.m46A.T() * meas.m43B;
    } // end for all measurements
  
  double dNewError = dCurrentError + 9999;
  while(dNewError > dCurrentError && !mbConverged && !mbHitMaxIterations && !*pbAbortSignal)
    {
      for(vector<Point>::iterator itr = mvPoints.begin(); itr!=mvPoints.end(); itr++)
	{
	  Point &point = *itr;
	  Matrix<3> m3VStar = point.m3V;
	  if(m3VStar[0][0] * m3VStar[1][1] * m3VStar[2][2] == 0)
	    point.m3VStarInv = Zeros;
	  else
	    {
	      m3VStar[0][1] = m3VStar[1][0];
	      m3VStar[0][2] = m3VStar[2][0];
	      m3VStar[1][2] = m3VStar[2][1];
	       
	      for(int i=0; i<3; i++)
		m3VStar[i][i] *= (1.0 + mdLambda);
	      Cholesky<3> chol(m3VStar);
	      point.m3VStarInv = chol.get_inverse();
	    };
	}


      Matrix<> mS(mnCamsToUpdate * 6, mnCamsToUpdate * 6);
      mS = Zeros;
      Vector<> vE(mnCamsToUpdate * 6);
      vE = Zeros;

      Matrix<6> m6; // Temp working space
      Vector<6> v6; // Temp working space
      
      for(unsigned int j=0; j<mvCameras.size(); j++)
	{
	  Camera &cam_j = mvCameras[j];
	  if(cam_j.bFixed) continue;
	  int nCamJStartRow = cam_j.nStartRow;
	  
	  for(int r=0; r<6; r++)
	    {
	      for(int c=0; c<r; c++)
		m6[r][c] = m6[c][r] = cam_j.m6U[r][c];
	      m6[r][r] = cam_j.m6U[r][r];
	    };
	  
	  for(int nn = 0; nn< 6; nn++)
	    m6[nn][nn] *= (1.0 + mdLambda);
	  
	  v6 = cam_j.v6EpsilonA;
	  
	  vector<Meas*> &vMeasLUTj = mvMeasLUTs[j];
	  for(unsigned int i=0; i<mvPoints.size(); i++)
	    {
	      Meas* pMeas = vMeasLUTj[i];
	      if(pMeas == NULL || pMeas->bBad)
		continue;
	      m6 -= pMeas->m63W * mvPoints[i].m3VStarInv * pMeas->m63W.T();
	      v6 -= pMeas->m63W * (mvPoints[i].m3VStarInv * mvPoints[i].v3EpsilonB);
	    }
	  mS.slice(nCamJStartRow, nCamJStartRow, 6, 6) = m6;
	  vE.slice(nCamJStartRow,6) = v6;
	}
      
      for(unsigned int i=0; i<mvPoints.size(); i++)
	{
	  Point &p = mvPoints[i];
	  int nCurrentJ = -1;
	  int nJRow = -1;
	  Meas* pMeas_ij;
	  Meas* pMeas_ik;
	  Matrix<6,3> m63_MIJW_times_m3VStarInv;
	  
	  for(vector<OffDiagScriptEntry>::iterator it=p.vOffDiagonalScript.begin();
	      it!=p.vOffDiagonalScript.end();
	      it++)
	    {
	      OffDiagScriptEntry &e = *it;
	      pMeas_ik = mvMeasLUTs[e.k][i];
	      if(pMeas_ik == NULL || pMeas_ik->bBad)
		continue;
	      if(e.j != nCurrentJ)
		{
		  pMeas_ij = mvMeasLUTs[e.j][i];
		  if(pMeas_ij == NULL || pMeas_ij->bBad)
		    continue;
		  nCurrentJ = e.j;
		  nJRow = mvCameras[e.j].nStartRow;
		  m63_MIJW_times_m3VStarInv = pMeas_ij->m63W * p.m3VStarInv;
		}
	      int nKRow = mvCameras[pMeas_ik->c].nStartRow;
#ifndef WIN32
		  mS.slice(nJRow, nKRow, 6, 6) -= m63_MIJW_times_m3VStarInv * pMeas_ik->m63W.T();
#else
		  Matrix<6> m = mS.slice(nJRow, nKRow, 6, 6);
		  m -= m63_MIJW_times_m3VStarInv * pMeas_ik->m63W.T();
          mS.slice(nJRow, nKRow, 6, 6) = m;
#endif
	      assert(nKRow < nJRow);
	    }
	}
      
      for(int i=0; i<mS.num_rows(); i++)
	for(int j=0; j<i; j++)
	  mS[j][i] = mS[i][j];
      
      Vector<> vCamerasUpdate(mS.num_rows());
      vCamerasUpdate = Cholesky<>(mS).backsub(vE);
      
      Vector<> vMapUpdates(mvPoints.size() * 3);
      for(unsigned int i=0; i<mvPoints.size(); i++)
	{
	  Vector<3> v3Sum;
	  v3Sum = Zeros;
	  for(unsigned int j=0; j<mvCameras.size(); j++)
	    {
	      Camera &cam = mvCameras[j];
	      if(cam.bFixed)
		continue;
	      Meas *pMeas = mvMeasLUTs[j][i];
	      if(pMeas == NULL || pMeas->bBad)
		continue;
	      v3Sum+=pMeas->m63W.T() * vCamerasUpdate.slice(cam.nStartRow,6);
	    }
	  Vector<3> v3 = mvPoints[i].v3EpsilonB - v3Sum;
	  vMapUpdates.slice(i * 3, 3) = mvPoints[i].m3VStarInv * v3;
	  if(isnan(vMapUpdates.slice(i * 3, 3) * vMapUpdates.slice(i * 3, 3)))
	    {
	      cerr << "NANNERY! " << endl;
	      cerr << mvPoints[i].m3VStarInv << endl;
	    };
	}
      
       double dSumSquaredUpdate = vCamerasUpdate * vCamerasUpdate + vMapUpdates * vMapUpdates;
      if(dSumSquaredUpdate< *mgvdUpdateConvergenceLimit)
	mbConverged = true;
      
       for(unsigned int j=0; j<mvCameras.size(); j++)
	{
	  if(mvCameras[j].bFixed)
	    mvCameras[j].se3CfWNew = mvCameras[j].se3CfW;
	  else
	    mvCameras[j].se3CfWNew = SE3<>::exp(vCamerasUpdate.slice(mvCameras[j].nStartRow, 6)) * mvCameras[j].se3CfW;
	}
      for(unsigned int i=0; i<mvPoints.size(); i++)
	mvPoints[i].v3PosNew = mvPoints[i].v3Pos + vMapUpdates.slice(i*3, 3);
      
      dNewError = FindNewError<MEstimator>();
      
      cout <<setprecision(1) << "L" << mdLambda << setprecision(3) <<  "\tOld " << dCurrentError << "  New " << dNewError << "  Diff " << dCurrentError - dNewError << "\t";
      
      // Was the step good? If not, modify lambda and try again!!
      // (if it was good, will break from this loop.)
      if(dNewError > dCurrentError)
	{
	  cout << " TRY AGAIN " << endl;
	  ModifyLambda_BadStep();
	};
      
      mnCounter++;
      if(mnCounter >= *mgvnMaxIterations)
	mbHitMaxIterations = true;
    }   // End of while error too big loop
  
  if(dNewError < dCurrentError) // Was the last step a good one?
    {
      cout << " WINNER            ------------ " << endl;
      // Woo! got somewhere. Update lambda and make changes permanent.
      ModifyLambda_GoodStep();
      for(unsigned int j=0; j<mvCameras.size(); j++)
	mvCameras[j].se3CfW = mvCameras[j].se3CfWNew;
      for(unsigned int i=0; i<mvPoints.size(); i++)
	mvPoints[i].v3Pos = mvPoints[i].v3PosNew; 
      mnAccepted++;
    }
  
  // Finally, ditch all the outliers.
  vector<list<Meas>::iterator> vit;
  for(list<Meas>::iterator itr = mMeasList.begin(); itr!=mMeasList.end(); itr++)
    if(itr->bBad)
      {
	vit.push_back(itr);
	mvOutlierMeasurementIdx.push_back(make_pair(itr->p, itr->c));
	mvPoints[itr->p].nOutliers++;
	mvMeasLUTs[itr->c][itr->p] = NULL;
      };
  
  for(unsigned int i=0; i<vit.size(); i++)
    mMeasList.erase(vit[i]);
  
  cout << "Nuked " << vit.size() << " measurements." << endl;
  return true;
}

// Find the new total error if cameras and points used their 
// new coordinates- this is done with both camera new positions
template<class MEstimator>
double Bundle::FindNewError()
{
  ofstream ofs;
  double dNewError = 0;
  vector<double> vdErrorSquared;
  for(list<Meas>::iterator itr = mMeasList.begin(); itr!=mMeasList.end(); itr++)
    {
      Meas &meas = *itr;
      SE3<> se3Cam  = mvCameras[meas.c].se3CfWNew;
      Vector<3> v3Cam = se3Cam * mvPoints[meas.p].v3PosNew;
      if(v3Cam[2] <= 0)
	{
	  dNewError += 1.0;
	  cout << ".";
	  continue;
	};

      // project to image plane of the two cameras
      // and find the new projection error
      Vector<2> v2ImPlane = project(v3Cam);
      Vector<2> v2Image   = mCamera_Left.Project(v2ImPlane);
      Vector<2> v2Error = meas.dSqrtInvNoise * (meas.v2FoundInLeft - v2Image);
      double dErrorSquared = v2Error * v2Error;
      v2ImPlane = project(mCamera_Left.GetExtrinsic()*v3Cam);
      v2Image   = mCamera_Right.Project(v2ImPlane);
      v2Error = meas.dSqrtInvNoise * (meas.v2FoundInRight - v2Image);
      // average error over the two views
      dErrorSquared += v2Error * v2Error;
      dErrorSquared /= 2;
      dNewError += MEstimator::ObjectiveScore(dErrorSquared, mdSigmaSquared);
    }
  return dNewError;
}


void Bundle::GenerateMeasLUTs()
{
  mvMeasLUTs.clear();
  for(unsigned int nCam = 0; nCam < mvCameras.size(); nCam++)
    {
      mvMeasLUTs.push_back(vector<Meas*>());
      mvMeasLUTs.back().resize(mvPoints.size(), NULL);
    };
  for(list<Meas>::iterator it = mMeasList.begin(); it!=mMeasList.end(); it++)
    mvMeasLUTs[it->c][it->p] =  &(*it);
 
}

void Bundle::GenerateOffDiagScripts()
{
  for(unsigned int i=0; i<mvPoints.size(); i++)
    {
      Point &p = mvPoints[i];
      p.vOffDiagonalScript.clear();
      for(set<int>::iterator it_j = p.sCameras.begin(); it_j!=p.sCameras.end(); it_j++)
	{
	  int j = *it_j;
	  if(mvCameras[j].bFixed)
	    continue;
	  Meas *pMeas_j = mvMeasLUTs[j][i];
	  assert(pMeas_j != NULL);
	  
	  for(set<int>::iterator it_k = p.sCameras.begin(); it_k!=it_j; it_k++)
	    {
	      int k = *it_k;
	      if(mvCameras[k].bFixed)
		continue;
	      
	      Meas *pMeas_k = mvMeasLUTs[k][i];
	      assert(pMeas_k != NULL);
	      
	      OffDiagScriptEntry e;
	      e.j = j;
	      e.k = k;
	      p.vOffDiagonalScript.push_back(e);
	    }
	}
    }
}

void Bundle::ModifyLambda_GoodStep()
{
  mdLambdaFactor = 2.0;
  mdLambda *= 0.3;
};

void Bundle::ModifyLambda_BadStep()
{
  mdLambda = mdLambda * mdLambdaFactor;
  mdLambdaFactor = mdLambdaFactor * 2;
};


Vector<3> Bundle::GetPoint(int n)
{
  return mvPoints.at(n).v3Pos;
}

SE3<> Bundle::GetCamera(int n)
{
  return mvCameras.at(n).se3CfW;
}

set<int> Bundle::GetOutliers()
{
  set<int> sOutliers;
  set<int>::iterator hint = sOutliers.begin();
  for(unsigned int i=0; i<mvPoints.size(); i++)
    {
      Point &p = mvPoints[i];
      if(p.nMeasurements > 0 && p.nMeasurements == p.nOutliers)
	hint = sOutliers.insert(hint, i);
    }
  return sOutliers;
};


vector<pair<int, int> > Bundle::GetOutlierMeasurements()
{
  return mvOutlierMeasurementIdx;
}









