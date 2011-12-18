// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HelperFunctions.h"
#include "constants.h"
#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>
#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>

#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <fstream>
#include <algorithm>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace CVD;
using namespace std;
using namespace GVars3;


// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, const StereoCamera &cam)
  : mMap(m), mCamera(cam)
{
  mbResetRequested = false;
  Reset();
  start(); 
  GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
};

void MapMaker::Reset()
{
  mMap.Reset();
  mvFailureQueue.clear();
  while(!mqNewQueue.empty()) mqNewQueue.pop();
  mMap.vpKeyFrames.clear(); 
  mvpKeyFrameQueue.clear(); 
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;
}

#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run()
{

#ifdef WIN32
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

  while(!shouldStop()) 
    {
      CHECK_RESET;
      sleep(5);
      CHECK_RESET;
      
      while(!mvQueuedCommands.empty())
	{
	  GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
	  mvQueuedCommands.erase(mvQueuedCommands.begin());
	}
      
      if(!mMap.IsGood())
	continue;
      
      CHECK_RESET;

      if(!mbBundleConverged_Recent && QueueSize() == 0)  
	BundleAdjustRecent();   
      
      CHECK_RESET;

      if(mbBundleConverged_Recent && QueueSize() == 0)
	ReFindNewlyMade();  
      
      CHECK_RESET;

      if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
	BundleAdjustAll();
      
      CHECK_RESET;

      if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
	ReFindFromFailureQueue();
      
      CHECK_RESET;
      HandleBadPoints();
      
      CHECK_RESET;
      
      if(QueueSize() > 0)
	AddKeyFrameFromTopOfQueue();
    }
}

void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

void MapMaker::HandleBadPoints()
{
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      MapPoint &p = *mMap.vpPoints[i];
      if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount)
	p.bBad = true;
    }
  
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    if(mMap.vpPoints[i]->bBad)
      {
	MapPoint *p = mMap.vpPoints[i];
	for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++)
	  {
	    KeyFrame &k = *mMap.vpKeyFrames[j];
	    if(k.mMeasurements.count(p))
	      k.mMeasurements.erase(p);
	  }
      }
  mMap.MoveBadPointsToTrash();
}

MapMaker::~MapMaker()
{
  mbBundleAbortRequested = true;
  stop(); 
  cout << "Waiting for mapmaker to die.." << endl;
  join();
  cout << " .. mapmaker has died." << endl;
}

Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();
  
  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}

/* Triangulate point in a rectified camera rig */

Vector<3> MapMaker::TriangulatePoint(const Vector<2> &v2A, const Vector<2> &v2B){
  Vector<4> point;
  point.slice(0,2) = v2A;
  point[2] = (mCamera.Left().getPrincipalPoint(0)-v2A[0]) - (mCamera.Right().getPrincipalPoint(0)-v2B[0]);
  point[3] = 1;
  Vector<4> projected = ReProjectionMatrix*point;
  normalizeLast(projected);
  return projected.slice(0,3);
}

// InitFromStereo() generates the initial match from single keyframes and two views
// and a vector of image correspondences. 

bool MapMaker::InitFromStereo(KeyFrame &kF,
			      vector<pair<ImageRef, ImageRef> > &vTrailMatches,SE3<> &se3TrackerPose)
{
  mCamera.Left().SetImageSize(kF.aCamLeftLevels[0].im.size());
  mCamera.Right().SetImageSize(kF.aCamRightLevels[0].im.size());
  
  //create the first keyframe in the MapMaker
  KeyFrame *pkFirst = new KeyFrame();
  *pkFirst = kF;
  pkFirst->bFixed = true;
  pkFirst->se3LeftCfromW = SE3<>();
  pkFirst->se3RightCfromW = mCamera.GetRelativePose();
  
  // Construct map from the stereo matches.
  // PatchFinder is used for sub-pixel iteration here as we
  // already have screen coords
  PatchFinder finder;
  
  for(unsigned int i=0; i<vTrailMatches.size(); i++)
    {
      MapPoint *p = new MapPoint();
      
      // Patch source stuff:
      p->pPatchSourceKF = pkFirst;
      p->nSourceLevel = 0;
      p->v3Normal_NC = makeVector( 0,0,-1);
      p->irLeftFrameCenter = vTrailMatches[i].first;
      p->irRightFrameCenter = vTrailMatches[i].second;
        
      //set the vectors for patch warping for the left frameview
      p->v3LeftFrameCenter_NC = unproject(mCamera.Left().UnProject(p->irLeftFrameCenter));
      p->v3OneDownFromLeftFrameCenter_NC = unproject(mCamera.Left().UnProject(p->irLeftFrameCenter + ImageRef(0,1)));
      p->v3OneRightFromLeftFrameCenter_NC = unproject(mCamera.Left().UnProject(p->irLeftFrameCenter + ImageRef(1,0)));            
      normalize(p->v3LeftFrameCenter_NC);
      normalize(p->v3OneDownFromLeftFrameCenter_NC);
      normalize(p->v3OneRightFromLeftFrameCenter_NC);
      
      //set the vectors for patch warping for right frameview - not really necessary
      p->v3RightFrameCenter_NC = unproject(mCamera.Right().UnProject(p->irRightFrameCenter));
      p->v3OneDownFromRightFrameCenter_NC = unproject(mCamera.Right().UnProject(p->irRightFrameCenter + ImageRef(0,1)));
      p->v3OneRightFromRightFrameCenter_NC = unproject(mCamera.Right().UnProject(p->irRightFrameCenter + ImageRef(1,0)));
      normalize(p->v3RightFrameCenter_NC);
      normalize(p->v3OneDownFromRightFrameCenter_NC);
      normalize(p->v3OneRightFromRightFrameCenter_NC);
      
      p->RefreshPixelVectors();

      // Do sub-pixel alignment on the second image of stereo pair
      finder.MakeTemplateCoarseNoWarp(*p);
      finder.MakeSubPixTemplate();
      finder.SetSubPixPos(vec(vTrailMatches[i].second));
      bool bGood = finder.IterateSubPixToConvergence((*pkFirst).aCamRightLevels,10);
      
      if(!bGood)
      	{ 
	  delete p; continue;
	}

      
      // Triangulate point:
      // Get the sub pixel matching of the first point

      Vector<2> v2SecondPos = finder.GetSubPixPos();
      
      // unproject to world using stereo pair
      p->v3WorldPos = mCamera.UnProjectToWorld(mCamera.UnProjectToLeft(vTrailMatches[i].first),mCamera.UnProjectToRight(vTrailMatches[i].second), pkFirst->se3LeftCfromW);

      if(p->v3WorldPos[2] < 0.0)
       	{
	  cout << "z less than zero for points "<< endl << vTrailMatches[i].first << "," << vTrailMatches[i].second << endl;
	  delete p; continue;
 	}
      
      p->pMMData = new MapMakerData();
      mMap.vpPoints.push_back(p);
      
      // add both pixel locations to stereo match
      Measurement mFirst;
      mFirst.nLevel = 0;
      mFirst.Source = Measurement::SRC_ROOT;
      mFirst.v2RootPos = vec(vTrailMatches[i].first);
      mFirst.v2CorrespondingPos = finder.GetSubPixPos();
      mFirst.bSubPix = true;
      pkFirst->mMeasurements[p] = mFirst; 
      p->pMMData->sMeasurementKFs.insert(pkFirst);
    }
 
  //add the first keyframe
  mMap.vpKeyFrames.push_back(pkFirst);
  
  pkFirst->MakeKeyFrame_Rest();
  
  RefreshSceneDepth(pkFirst);
  
  mdBaseLine = mCamera.Left().getStereoBaseline();
  mdBaseLineOverSceneDepth = mCamera.Left().getStereoBaseline()/pkFirst->dSceneDepthMean;

  cout << "estimated scene depth is " << pkFirst->dSceneDepthMean << endl;
  
  cout << "current size of map is " << mMap.vpPoints.size() << " points before InitAddMapPoint" << endl;
  
  // add map points by true epipolar search
  InitAddMapPoints(0);
  InitAddMapPoints(3);
  InitAddMapPoints(1);
  InitAddMapPoints(2);
  
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  
  ApplyGlobalTransformationToMap(CalcPlaneAligner());
  mMap.bGood = true;
  se3TrackerPose = pkFirst->se3LeftCfromW;
  
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;
  return true; 
}

// ThinCandidates() Thins out a key-frame's candidate list.
void MapMaker::ThinCandidates(KeyFrame &k, Level &l, int nLevel)
{
  vector<Candidate> &vCSrc = l.vCandidates;
  vector<Candidate> vCGood;
  vector<ImageRef> irBusyLevelPos;
  for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
    {
      if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
	continue;
      irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
    }
  
  unsigned int nMinMagSquared = 10*10;
  for(unsigned int i=0; i<vCSrc.size(); i++)
    {
      ImageRef irC = vCSrc[i].irLevelPos;
      bool bGood = true;
      for(unsigned int j=0; j<irBusyLevelPos.size(); j++)
	{
	  ImageRef irB = irBusyLevelPos[j];
	  if((irB - irC).mag_squared() < nMinMagSquared)
	    {
	      bGood = false;
	      break;
	    }
	}
      if(bGood)
	vCGood.push_back(vCSrc[i]);
    } 
  vCSrc = vCGood;
}

// Add map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. 

void MapMaker::InitAddMapPoints(int nLevel){
  KeyFrame &kf = (*mMap.vpKeyFrames.front());
  Level *l_l = kf.aCamLeftLevels;
  ThinCandidates(kf,l_l[nLevel], nLevel);
  //generate the map points from candidates
  for(unsigned int n = 0; n<l_l[nLevel].vCandidates.size();n++)
    InitAddPointEpipolar(kf,nLevel,(int)n);
}

void MapMaker::AddSomeMapPoints(int nLevel)
{
  KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); 
  KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));   
  Level &l = kSrc.aCamLeftLevels[nLevel];

  ThinCandidates(kSrc, l, nLevel);
  

  // if scene depth is huge- use the false stereo pair
  if(kSrc.dSceneDepthMean/mdBaseLine > BASELINE_SCENE_DEPTH){ 
    for(unsigned int i = 0; i<l.vCandidates.size(); i++)
      AddPointEpipolar(kSrc, kTarget, nLevel, i);
  }else{ // else use the good real stereo pair
  for(unsigned int i=0; i<l.vCandidates.size(); i++)
    InitAddPointEpipolar(kSrc,nLevel,(int)i);
  }
}

void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
    mMap.vpKeyFrames[i]->se3LeftCfromW = mMap.vpKeyFrames[i]->se3LeftCfromW * se3NewFromOld.inverse();
    mMap.vpKeyFrames[i]->se3RightCfromW = mMap.vpKeyFrames[i]->se3RightCfromW * se3NewFromOld.inverse();
  }
  
  SO3<> so3Rot = se3NewFromOld.get_rotation();
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      mMap.vpPoints[i]->v3WorldPos = 
	se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      mMap.vpPoints[i]->RefreshPixelVectors();
    }
}

void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
    mMap.vpKeyFrames[i]->se3LeftCfromW.get_translation() *= dScale;
    mMap.vpKeyFrames[i]->se3RightCfromW.get_translation() *= dScale;
  }
    
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;
      (*mMap.vpPoints[i]).v3LeftFramePixelRight_W *= dScale;
      (*mMap.vpPoints[i]).v3LeftFramePixelDown_W *= dScale;
      (*mMap.vpPoints[i]).v3RightFramePixelRight_W *= dScale;
      (*mMap.vpPoints[i]).v3RightFramePixelDown_W *= dScale;
      (*mMap.vpPoints[i]).RefreshPixelVectors();
    }
}

void MapMaker::AddKeyFrame(KeyFrame &k)
{
  KeyFrame *pK = new KeyFrame;
  *pK = k;
  pK->pSBI = NULL; 
  mvpKeyFrameQueue.push_back(pK);
  if(mbBundleRunning) 
    mbBundleAbortRequested = true;
}


void MapMaker::AddKeyFrameFromTopOfQueue()
{
  if(mvpKeyFrameQueue.size() == 0)
    return;
  
  KeyFrame *pK = mvpKeyFrameQueue[0];
  mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
  pK->MakeKeyFrame_Rest();
  mMap.vpKeyFrames.push_back(pK);

  for(meas_it it = pK->mMeasurements.begin();
      it!=pK->mMeasurements.end();
      it++)
    {
      it->first->pMMData->sMeasurementKFs.insert(pK);
      it->second.Source = Measurement::SRC_TRACKER;
    }
  
  ReFindInSingleKeyFrame(*pK);
  
  AddSomeMapPoints(3); 
  AddSomeMapPoints(0);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);
  
  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;
}

// add points from a single keyframe using an epipolar search across the two frames

bool MapMaker::InitAddPointEpipolar(KeyFrame &first,
				    int nLevel,
				    int nCandidate){
  
  // get a candidate
  Candidate &candidate = first.aCamLeftLevels[nLevel].vCandidates[nCandidate];
  ImageRef irLeftFramePos = candidate.irLevelPos;
  Vector<2> v2RootLeftPos = LevelZeroPos(irLeftFramePos,nLevel);
  PatchFinder Finder;
  // make a unwarped template of the nth level of the left image
  // at the position of the candidate in that level
  Finder.MakeTemplateCoarseNoWarp(first.aCamLeftLevels, nLevel, irLeftFramePos);
  if(Finder.TemplateBad())
    return false;
    
  //all the corners in the right image to search for matches
  vector<ImageRef> &vIR =first.aCamRightLevels[nLevel].vCorners;
  
  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  
  int nLevelScale = LevelScale(nLevel);
  // set a maximum distance to search in pixels (taking into account image res)
  int SearchDistMag = (200 + nLevelScale -1)/nLevelScale;
  
  /* for level 0 search x = 100
     for level 0 search y = 15
     for level 1 search x = 50
     for level 1 search y = 10
     for level 2 search x = 25
     for level 2 search y = 5
     for level 3 search x = 13
     for level 3 search y = 3
  */
  
  // get the epipolar line
  Vector<3> EpipolarLine = mCamera.GetEpipolarLine(irLeftFramePos);
  // seach all corners and compare to epipolar line
  for(int i=0;i<(int)first.aCamRightLevels[nLevel].vCorners.size();i++){
    ImageRef irMatch = vIR[i];
    if(unproject(vec(irMatch))*EpipolarLine > /*0.05 0.1*/0.01*nLevelScale || 
       sqrt( (vec(irMatch)-vec(irLeftFramePos))* (vec(irMatch)-vec(irLeftFramePos))) > SearchDistMag ||
       irMatch.x < irLeftFramePos.x) continue;
    // attempt to match if on epipolar line
    int nZMSSD = Finder.ZMSSDAtPoint(first.aCamRightLevels[nLevel].im,irMatch);
    if(nZMSSD < nBestZMSSD){
      nBest = i;
      nBestZMSSD = nZMSSD;
    }
  }
  
  ImageRef irRightFramePos;

  // if we missed a point, try along the line - not on fast corners
  if(nBest == -1){
    Vector<3> NormEpipolar = EpipolarLine;
    normalize(NormEpipolar);
    ImageRef irBest(-1,-1);
    for(int x = irLeftFramePos.x;x<irLeftFramePos.x+SearchDistMag;x++) {
      ImageRef OnLine(x,((-EpipolarLine[0]/EpipolarLine[1])*x)-(EpipolarLine[2]/EpipolarLine[1]));
      int nZMSSD = Finder.ZMSSDAtPoint(first.aCamRightLevels[nLevel].im,OnLine);
      if(nZMSSD < nBestZMSSD){
	irBest = OnLine;
	nBestZMSSD = nZMSSD;
      }
    }
    if(irBest.x == -1)
      return false;
    irRightFramePos = irBest;
    }else{
    irRightFramePos = vIR[nBest];
  }
  
  // now finder has found a matching point, try to 
  // sub-pixel match it in the image level
 
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(/*vIR[nBest]*/irRightFramePos,nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(first.aCamRightLevels,10);
  if(!bSubPixConverges)
    return false;
  
  Vector <2> v2RootRightPos = Finder.GetSubPixPos();
  
  // Triangulate this 3D point
  Vector<3> v3WorldPos = mCamera.UnProjectToWorldFromPixels(v2RootLeftPos,v2RootRightPos,first.se3LeftCfromW);
  
  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3WorldPos;
  pNew->pMMData = new MapMakerData();
  
  // Set position of point in keyframe and level found
  pNew->pPatchSourceKF = &first;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector(0,0,-1);
  pNew->irLeftFrameCenter = irLeftFramePos;
  pNew->irRightFrameCenter = /*vIR[nBest]*/irRightFramePos; 
  
  // Calcuate vectors used for warping patch in left image
  pNew->v3LeftFrameCenter_NC = unproject(mCamera.UnProjectToLeft(v2RootLeftPos));
  pNew->v3OneRightFromLeftFrameCenter_NC = unproject(mCamera.UnProjectToLeft(v2RootLeftPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromLeftFrameCenter_NC  = unproject(mCamera.UnProjectToLeft(v2RootLeftPos + vec(ImageRef(0,nLevelScale))));
  normalize(pNew->v3LeftFrameCenter_NC);
  normalize(pNew->v3OneDownFromLeftFrameCenter_NC);
  normalize(pNew->v3OneRightFromLeftFrameCenter_NC);

  // Calcuate vectors used for warping patch in right image
  pNew->v3RightFrameCenter_NC = unproject(mCamera.UnProjectToRight(v2RootRightPos));
  pNew->v3OneRightFromRightFrameCenter_NC = unproject(mCamera.UnProjectToRight(v2RootRightPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromRightFrameCenter_NC  = unproject(mCamera.UnProjectToRight(v2RootRightPos + vec(ImageRef(0,nLevelScale))));
  normalize(pNew->v3RightFrameCenter_NC);
  normalize(pNew->v3OneDownFromRightFrameCenter_NC);
  normalize(pNew->v3OneRightFromRightFrameCenter_NC);

  pNew->RefreshPixelVectors();
  // generate sterem measurements  
  mMap.vpPoints.push_back(pNew);
  mqNewQueue.push(pNew);
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootLeftPos;
  m.v2CorrespondingPos = v2RootRightPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  first.mMeasurements[pNew] = m;
  pNew->pMMData->sMeasurementKFs.insert(&first);
  
  return true;
}

// PTAM's original map point adder - now gets guess depth from the predicted true
//stereo depth

bool MapMaker::AddPointEpipolar(KeyFrame &kSrc, 
				KeyFrame &kTarget, 
				int nLevel,
				int nCandidate)
{
  static Image<Vector<2> > imUnProj;
  static bool bMadeCache = false;
  if(!bMadeCache)
    {
      imUnProj.resize(kSrc.aCamLeftLevels[0].im.size());
      ImageRef ir;
      do imUnProj[ir] = mCamera.UnProjectToLeft(ir);
      while(ir.next(imUnProj.size()));
      bMadeCache = true;
    }
  //levelScale is bitwise left shift 1 << nLevel so returns 1,2,4,8 etc.
  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aCamLeftLevels[nLevel].vCandidates[nCandidate];
  ImageRef irSrcLeftLevelPos = candidate.irLevelPos;
  Vector<2> v2SrcLeftRootPos = LevelZeroPos(irSrcLeftLevelPos, nLevel);

  //find depth of point by stereo match
  
  //make a template of the left image point
  PatchFinder EpiFinder_A;
  EpiFinder_A.MakeTemplateCoarseNoWarp(kSrc.aCamLeftLevels, nLevel, irSrcLeftLevelPos); //coarse no warp takes level n pos
  
  if(EpiFinder_A.TemplateBad())
    return false;
  
  Vector<3> EpipolarLine = mCamera.GetEpipolarLine(irSrcLeftLevelPos);
  vector<Candidate> &vCan = kSrc.aCamRightLevels[nLevel].vCandidates;
  int nBestIndex = -1;
  int nBestSSD = EpiFinder_A.mnMaxSSD + 1; 
  for(int i=0;i<(int)kSrc.aCamRightLevels[nLevel].vCandidates.size();i++){
    ImageRef IR = vCan[i].irLevelPos;
    int nZMSSD = EpiFinder_A.ZMSSDAtPoint(kSrc.aCamRightLevels[nLevel].im, IR);
    if(nZMSSD < nBestSSD){
      nBestIndex = i;
      nBestSSD = nZMSSD;
    }
  }
  if(nBestIndex == -1)
    return false;


  EpiFinder_A.MakeSubPixTemplate();
  EpiFinder_A.SetSubPixPos(LevelZeroPos(vCan[nBestIndex].irLevelPos,nLevel));
  if(!EpiFinder_A.IterateSubPixToConvergence(kSrc.aCamRightLevels,10))
    return false;
  Vector<2> v2SrcRightRootPos_A = EpiFinder_A.GetSubPixPos();
  ImageRef irSrcRightLevelPos_A = ir(v2SrcRightRootPos_A/LevelScale(nLevel));
  //get the depth
  double Depth = mCamera.UnProjectToWorldFromPixels(v2SrcLeftRootPos,v2SrcRightRootPos_A,kSrc.se3LeftCfromW)[2];
  


  Vector<3> v3Ray_SC = unproject(mCamera.UnProjectToLeft(v2SrcLeftRootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC = kTarget.se3LeftCfromW.get_rotation() * (kSrc.se3LeftCfromW.get_rotation().inverse() * v3Ray_SC);

  
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  // set start and end depth via stereo guess
  double dStartDepth = max(Depth/2, dMean - dSigma);
  double dEndDepth = min(Depth*2, dMean + dSigma);
  
  Vector<3> v3CamCenter_TC = kTarget.se3LeftCfromW * kSrc.se3LeftCfromW.inverse().get_translation();
  Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                  
  
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])  
    return false;
  if(v3RayEnd_TC[2] <= 0.0 )  return false;
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);
  
  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;
  
  if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
    {
      return false;
    }
  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];
  
  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > mCamera.Left().LargestRadiusInImage() )
    return false;
  
  double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;

  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(kSrc.aCamLeftLevels, nLevel, irSrcLeftLevelPos);
  if(Finder.TemplateBad())  return false;
  
  vector<Vector<2> > &vv2Corners = kTarget.aCamLeftLevels[nLevel].vImplaneCorners;
  vector<ImageRef> &vIR = kTarget.aCamLeftLevels[nLevel].vCorners;
  if(!kTarget.aCamLeftLevels[nLevel].bImplaneCornersCached)
    {
      for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
	vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
      kTarget.aCamLeftLevels[nLevel].bImplaneCornersCached = true;
    }
  
  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  double dMaxDistDiff = mCamera.Left().OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;
  
  for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
    {
      Vector<2> v2Im = vv2Corners[i];
      double dDistDiff = dNormDist - v2Im * v2Normal;
      if(dDistDiff * dDistDiff > dMaxDistSq)	continue;
      if(v2Im * v2AlongProjectedLine < dMinLen)	continue;
      if(v2Im * v2AlongProjectedLine > dMaxLen)	continue;
      int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aCamLeftLevels[nLevel].im, vIR[i]);
      if(nZMSSD < nBestZMSSD)
	{
	  nBest = i;
	  nBestZMSSD = nZMSSD;
	}
    } 
  
  if(nBest == -1)   return false;   // Nothing found.
  
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget.aCamLeftLevels,10);
  if(!bSubPixConverges)
    return false;
  
  Vector<2> v2DstLeftRootPos = Finder.GetSubPixPos();
  Vector<3> v3New;
  v3New = kTarget.se3LeftCfromW.inverse() *  
    ReprojectPoint(kSrc.se3LeftCfromW * 
		   kTarget.se3LeftCfromW.inverse(),
		   mCamera.UnProjectToLeft(v2SrcLeftRootPos), 
		   mCamera.UnProjectToLeft(v2DstLeftRootPos));
  

  
  //confirm find with stereo search in second keyframe - adds robustness
  Vector<3> v3RightCam = (kSrc.se3LeftCfromW * v3New);
  v3RightCam = mCamera.Left().Extrinsic * v3RightCam;
  Vector<2> v2RightImPos = mCamera.Right().Project(project(v3RightCam));
  
  PatchFinder EpiFinder;
  EpiFinder.MakeTemplateCoarseNoWarp(kSrc.aCamLeftLevels, nLevel, irSrcLeftLevelPos); 
  
  if(EpiFinder.TemplateBad()){
    return false;
  }
  
  // find the point with epipolar search
  bool bFoundEpipolar = EpiFinder.FindPatchCoarse(ir(v2RightImPos), kSrc.aCamRightLevels, 15);
  
  if(!bFoundEpipolar){
    return false;
  }

  
  EpiFinder.MakeSubPixTemplate();
  if(!EpiFinder.IterateSubPixToConvergence(kSrc.aCamRightLevels,10))
    return false;
  Vector<2> v2SrcRightRootPos = EpiFinder.GetSubPixPos();
  ImageRef irSrcRightLevelPos = ir(v2SrcRightRootPos/LevelScale(nLevel));
  
  Vector<3> v3RightCam2 = kTarget.se3LeftCfromW * v3New;
  v3RightCam2 = mCamera.Left().Extrinsic * v3RightCam2;
  Vector<2> v2RightImPos2 = mCamera.Right().Project(project(v3RightCam2));
  PatchFinder EpiFinder2;
  EpiFinder2.MakeTemplateCoarseNoWarp(kTarget.aCamLeftLevels, nLevel, vIR[nBest]);
  if(EpiFinder2.TemplateBad()){
    return false;
  }
  
  bool bFoundEpipolar2 = EpiFinder2.FindPatchCoarse(ir(v2RightImPos2), kTarget.aCamRightLevels, 15);
  if(!bFoundEpipolar2){
    return false;
  }

  EpiFinder2.MakeSubPixTemplate();
  if(!EpiFinder2.IterateSubPixToConvergence(kTarget.aCamRightLevels,10))
    return false;
  
  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3New;
  pNew->pMMData = new MapMakerData();
  
  

  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irLeftFrameCenter = irSrcLeftLevelPos;
  pNew->irRightFrameCenter = irSrcRightLevelPos;
  
  // get the unit vectors for unprojection in both images 
  pNew->v3LeftFrameCenter_NC = unproject(mCamera.Left().UnProject(v2SrcLeftRootPos));
  pNew->v3OneRightFromLeftFrameCenter_NC = unproject(mCamera.Left().UnProject(v2SrcLeftRootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromLeftFrameCenter_NC  = unproject(mCamera.Left().UnProject(v2SrcLeftRootPos + vec(ImageRef(0,nLevelScale))));
  normalize(pNew->v3LeftFrameCenter_NC);
  normalize(pNew->v3OneDownFromLeftFrameCenter_NC);
  normalize(pNew->v3OneRightFromLeftFrameCenter_NC);
  
  pNew->v3RightFrameCenter_NC = unproject(mCamera.Right().UnProject(v2SrcRightRootPos));
  pNew->v3OneRightFromRightFrameCenter_NC = unproject(mCamera.Right().UnProject(v2SrcRightRootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromRightFrameCenter_NC  = unproject(mCamera.Right().UnProject(v2SrcRightRootPos + vec(ImageRef(0,nLevelScale))));
  normalize(pNew->v3RightFrameCenter_NC);
  normalize(pNew->v3OneDownFromRightFrameCenter_NC);
  normalize(pNew->v3OneRightFromRightFrameCenter_NC);
  
  pNew->RefreshPixelVectors();
    
  mMap.vpPoints.push_back(pNew);
  mqNewQueue.push(pNew);
  
  //gen the stereo measurements
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2SrcLeftRootPos;
  m.v2CorrespondingPos = v2SrcRightRootPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  kSrc.mMeasurements[pNew] = m;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder.GetSubPixPos();
  m.v2CorrespondingPos = EpiFinder2.GetSubPixPos();
  kTarget.mMeasurements[pNew] = m;
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);
  return true;
}

double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
  Vector<3> v3KF1_CamPos = k1.se3LeftCfromW.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.se3LeftCfromW.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}

vector<KeyFrame*> MapMaker::NClosestKeyFrames(KeyFrame &k, unsigned int N)
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
    }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());
  
  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}

KeyFrame* MapMaker::ClosestKeyFrame(KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      if(dDist < dClosestDist)
	{
	  dClosestDist = dDist;
	  nClosest = i;
	}
    }
  assert(nClosest != -1);
  return mMap.vpKeyFrames[nClosest];
}

double MapMaker::DistToNearestKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  return dDist;
}

bool MapMaker::NeedNewKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  dDist *= (1.0 / kCurrent.dSceneDepthMean);
  
  if(dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdBaseLineOverSceneDepth)
    return true;
  return false;  double a[] = {0,0,0,0,0};
  CvMat dist = cvMat(5,1,CV_64FC1,a);
  cv::Mat dist_(&dist);// = dist;

}

void MapMaker::BundleAdjustSingleKeyframe()
{
  //not used
}

void MapMaker::BundleAdjustAll()
{
  set<KeyFrame*> sAdj;
  set<KeyFrame*> sFixed;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    if(mMap.vpKeyFrames[i]->bFixed)
      sFixed.insert(mMap.vpKeyFrames[i]);
    else
      sAdj.insert(mMap.vpKeyFrames[i]);
  
  set<MapPoint*> sMapPoints;
  for(unsigned int i=0; i<mMap.vpPoints.size();i++)
    sMapPoints.insert(mMap.vpPoints[i]);
  
  BundleAdjust(sAdj, sFixed, sMapPoints, false);
}


void MapMaker::BundleAdjustRecent()
{
  if(mMap.vpKeyFrames.size() < 8)  
    { 
      mbBundleConverged_Recent = true;
      return;
    }

  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFrames(*pkfNewest, 4);
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);
  
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };
  
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }
  
  BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true);
}

void MapMaker::BundleAdjust(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, bool bRecent)
{
  Bundle b(mCamera.Left(), mCamera.Right()); 
  mbBundleRunning = true;
  mbBundleRunningIsRecent = bRecent;
  
  map<MapPoint*, int> mPoint_BundleID;
  map<int, MapPoint*> mBundleID_Point;
  map<KeyFrame*, int> mView_BundleID;
  map<int, KeyFrame*> mBundleID_View;
  
  for(set<KeyFrame*>::iterator it = sAdjustSet.begin(); it!= sAdjustSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3LeftCfromW, (*it)->bFixed);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }
  for(set<KeyFrame*>::iterator it = sFixedSet.begin(); it!= sFixedSet.end(); it++)
    {
      int nBundleID = b.AddCamera((*it)->se3LeftCfromW, true);
      mView_BundleID[*it] = nBundleID;
      mBundleID_View[nBundleID] = *it;
    }
  
  for(set<MapPoint*>::iterator it = sMapPoints.begin(); it!=sMapPoints.end(); it++)
    {
      int nBundleID = b.AddPoint((*it)->v3WorldPos);
      mPoint_BundleID[*it] = nBundleID;
      mBundleID_Point[nBundleID] = *it;
    }
  
  // Add the relevant stereo measurments  measurements
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mView_BundleID.count(mMap.vpKeyFrames[i]) == 0)
	continue;
      
      int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[i]];
      for(meas_it it= mMap.vpKeyFrames[i]->mMeasurements.begin();
	  it!= mMap.vpKeyFrames[i]->mMeasurements.end();
	  it++)
	{
	  if(mPoint_BundleID.count(it->first) == 0)
	    continue;
	  int nPoint_BundleID = mPoint_BundleID[it->first];
	  Vector<4> meas;
	  meas.slice(0,2) = it->second.v2RootPos;
	  meas.slice(2,2) = it->second.v2CorrespondingPos;
	  b.AddMeas(nKF_BundleID, nPoint_BundleID,meas, LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel));
	}
    }
  
  int nAccepted = b.Compute(&mbBundleAbortRequested);
  
  if(nAccepted < 0)
    {
      cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
	   << "   The map is probably corrupt: Ditching the map. " << endl;
      mbResetRequested = true;
      return;
    }
  
  if(nAccepted > 0)
    {
      
      for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
	  itr!=mPoint_BundleID.end();
	  itr++)
	itr->first->v3WorldPos = b.GetPoint(itr->second);
      
      for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
	  itr!=mView_BundleID.end();
	  itr++)
	itr->first->se3LeftCfromW = b.GetCamera(itr->second);
      if(bRecent)
	mbBundleConverged_Recent = false;
      mbBundleConverged_Full = false;
    };
  
  if(b.Converged())
    {
      mbBundleConverged_Recent = true;
      if(!bRecent)
	mbBundleConverged_Full = true;
    }
  
  mbBundleRunning = false;
  mbBundleAbortRequested = false;

  vector<pair<int,int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
  for(unsigned int i=0; i<vOutliers_PC_pair.size(); i++)
    {
      MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
      KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
      Measurement &m = pk->mMeasurements[pp];
      if(pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT)
	pp->bBad = true;
      else
	{
	  if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
	    mvFailureQueue.push_back(pair<KeyFrame*,MapPoint*>(pk,pp));
	  else
	    pp->pMMData->sNeverRetryKFs.insert(pk);
	  pk->mMeasurements.erase(pp);
	  pp->pMMData->sMeasurementKFs.erase(pk);
	}
    }
}

// refind now over stereo pair - as usual
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p)
{

  if(p.pMMData->sMeasurementKFs.count(&k)
     || p.pMMData->sNeverRetryKFs.count(&k))
    return false;

  // find in one image
  static PatchFinder Finder;
  Vector<3> v3LeftCam = k.se3LeftCfromW*p.v3WorldPos;
  Vector<3> v3RightCam = mCamera.GetRelativePose() * v3LeftCam;
  if(v3LeftCam[2] < 0.001 || v3RightCam[2] < 0.001)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  // project to both 
  Vector<2> v2LeftImPlane = project(v3LeftCam);
  Vector<2> v2RightImPlane = project(v3RightCam);
  if( (v2LeftImPlane * v2LeftImPlane > mCamera.Left().LargestRadiusInImage() * mCamera.Left().LargestRadiusInImage()) || 
      (v2RightImPlane * v2RightImPlane > mCamera.Right().LargestRadiusInImage() * mCamera.Right().LargestRadiusInImage()) )
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  // if not in both return false
  Vector<2> v2LeftImage = mCamera.Left().Project(v2LeftImPlane);
  Vector<2> v2RightImage = mCamera.Right().Project(v2RightImPlane);
  if(mCamera.Left().Invalid() || mCamera.Right().Invalid())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  ImageRef irImageSize = k.aCamLeftLevels[0].im.size();
  if( (v2LeftImage[0] < 0 || v2LeftImage[1] < 0 || v2LeftImage[0] > irImageSize[0] || v2LeftImage[1] > irImageSize[1] ) ||
      (v2LeftImage[0] < 0 || v2LeftImage[1] < 0 || v2LeftImage[0] > irImageSize[0] || v2LeftImage[1] > irImageSize[1]))
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  Matrix<2> m2LeftCamDerivs = mCamera.Left().GetProjectionDerivs();
 
  Finder.MakeTemplateCoarse(p, k.se3LeftCfromW, m2LeftCamDerivs);
  
  if(Finder.TemplateBad())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  
  bool bFound = Finder.FindPatchCoarse(ir(v2LeftImage), k.aCamLeftLevels, 4);  // Very tight search radius!
  if(!bFound)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
  // find in second image of stereo pair
  static PatchFinder EpiFinder;
  ImageRef irLevPos = ir(v2LeftImage)/Finder.GetLevelScale();
  EpiFinder.MakeTemplateCoarseNoWarp(k.aCamLeftLevels, Finder.GetLevel(), irLevPos);
  if(EpiFinder.TemplateBad()){
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }
  // found in second image?
  bool bFoundEpipolar = EpiFinder.FindPatchCoarse(ir(v2RightImage), k.aCamRightLevels, 4);
  if(!bFoundEpipolar){
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }
  // gen  a stereo measurement struct and put it in the map
  Measurement m;
  m.nLevel = Finder.GetLevel();
  m.Source = Measurement::SRC_REFIND;
  
  if(Finder.GetLevel() > 0)
    {
      Finder.MakeSubPixTemplate();
      Finder.IterateSubPixToConvergence(k.aCamLeftLevels,8);
      EpiFinder.MakeSubPixTemplate();
      EpiFinder.IterateSubPixToConvergence(k.aCamRightLevels,8);
      m.v2RootPos = Finder.GetSubPixPos();
      m.v2CorrespondingPos = EpiFinder.GetSubPixPos();
      m.bSubPix = true;
    }
  else
    {
      m.v2RootPos = Finder.GetCoarsePosAsVector();
      m.v2CorrespondingPos = EpiFinder.GetCoarsePosAsVector();
      m.bSubPix = false;
    };
  
  if(k.mMeasurements.count(&p))
    {
      assert(0); 
    }
  k.mMeasurements[&p] = m;
  p.pMMData->sMeasurementKFs.insert(&k);
  return true;
}

int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k)
{
  vector<MapPoint*> vToFind;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    vToFind.push_back(mMap.vpPoints[i]);
  
  int nFoundNow = 0;
  for(unsigned int i=0; i<vToFind.size(); i++)
    if(ReFind_Common(k,*vToFind[i]))
      nFoundNow++;

  return nFoundNow;
};


void MapMaker::ReFindNewlyMade()
{
  if(mqNewQueue.empty())
    return;
  int nFound = 0;
  int nBad = 0;
  while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0)
    {
      MapPoint* pNew = mqNewQueue.front();
      mqNewQueue.pop();
      if(pNew->bBad)
	{
	  nBad++;
	  continue;
	}
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	if(ReFind_Common(*mMap.vpKeyFrames[i], *pNew))
	  nFound++;
    }
};

void MapMaker::ReFindFromFailureQueue()
{
  if(mvFailureQueue.size() == 0)
    return;
  sort(mvFailureQueue.begin(), mvFailureQueue.end());
  vector<pair<KeyFrame*, MapPoint*> >::iterator it;
  int nFound=0;
  for(it = mvFailureQueue.begin(); it!=mvFailureQueue.end(); it++)
    if(ReFind_Common(*it->first, *it->second))
      nFound++;
  
  mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent)
{
  return DistToNearestKeyFrame(kCurrent) > mdBaseLine * 1000.0;
}

SE3<> MapMaker::CalcPlaneAligner()
{
  unsigned int nPoints = mMap.vpPoints.size();
  if(nPoints < 10)
    {
      cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
      return SE3<>();
    };
  
  int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 400, HIDDEN|SILENT);
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;
  Vector<3> a,b,c;
  for(int i=0; i<nRansacs; i++)
    {
      int nA = rand()%nPoints;
      int nB = nA;
      int nC = nA;
      while(nB == nA)
	nB = rand()%nPoints;
      while(nC == nA || nC==nB)
	nC = rand()%nPoints;
      
      Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos + 
				       mMap.vpPoints[nB]->v3WorldPos + 
				       mMap.vpPoints[nC]->v3WorldPos);
      //3 random map points, find a plane normal
      Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3Normal = v3CA ^ v3BA;
      if(v3Normal * v3Normal  == 0)
	continue;
      normalize(v3Normal);
      //find the different between the average of these 3 points
      // and how far that is from the other points
      double dSumError = 0.0;
      for(unsigned int i=0; i<nPoints; i++)
	{
	  Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
	  double dDistSq = v3Diff * v3Diff;
	  if(dDistSq == 0.0)
	    continue;
	  double dNormDist = fabs(v3Diff * v3Normal);
	  
	  if(dNormDist > 0.05)
	    dNormDist = 0.05;
	  dSumError += dNormDist;
	}
      if(dSumError < dBestDistSquared)
	{
	  a = mMap.vpPoints[nA]->v3WorldPos;
	  b = mMap.vpPoints[nB]->v3WorldPos;
	  c = mMap.vpPoints[nC]->v3WorldPos;
	  dBestDistSquared = dSumError;
	  v3BestMean = v3Mean;
	  v3BestNormal = v3Normal;
	}
    }
  vector<Vector<3> > vv3Inliers;
  for(unsigned int i=0; i<nPoints; i++)
    {
      Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
      double dDistSq = v3Diff * v3Diff;
      if(dDistSq == 0.0)
	continue;
      double dNormDist = fabs(v3Diff * v3BestNormal);
      if(dNormDist < 0.05)
	vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
    }
  
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    v3MeanOfInliers+=vv3Inliers[i];
  v3MeanOfInliers *= (1.0 / vv3Inliers.size());
  
  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
      Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
      m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };
  
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];
  
  if(v3Normal[2] > 0)
    v3Normal *= -1.0;
  
  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];
  
  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;
  return se3Aligner;
}

void MapMaker::RefreshSceneDepth(KeyFrame *pKF)
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
      MapPoint &point = *it->first;
      Vector<3> v3PosK = pKF->se3LeftCfromW * point.v3WorldPos;
      dSumDepth += v3PosK[2];
      dSumDepthSquared += v3PosK[2] * v3PosK[2];
      nMeas++;
    }
 
  assert(nMeas > 2); 
  pKF->dSceneDepthMean = dSumDepth / nMeas;
  pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  
{
  if(sCommand=="SaveMap")
    {
      cout << "  MapMaker: Saving the map.... " << endl;
      ofstream ofs("map.dump");
      for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
	  ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
	  ofs << mMap.vpPoints[i]->nSourceLevel << endl;
	}
      ofs.close();
      
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	  ostringstream ost1;
	  ost1 << "keyframes/" << i << ".jpg";
 	  ostringstream ost2;
	  ost2 << "keyframes/" << i << ".info";
	  ofstream ofs2;
	  ofs2.open(ost2.str().c_str());
	  ofs2 << mMap.vpKeyFrames[i]->se3LeftCfromW << endl;
	  ofs2.close();
	}
      cout << "  ... done saving map." << endl;
      return;
    }
  
  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 












