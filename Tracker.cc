// Copyright 2008 Isis Innovation Limited
// now performs stereo tracking

#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "StereoPatchFinder.h"
#include "TrackerData.h"

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <fstream>
#include <fcntl.h>


using namespace CVD;
using namespace std;
using namespace GVars3;

// construct wiht stereo cameras
Tracker::Tracker(ImageRef irVideoSize, const StereoCamera &c, Map &m, MapMaker &mm, GLWindow2 &w_l, GLWindow2 &w_r) : 
  mWindowL(w_l),
  mWindowR(w_r),
  mMap(m),
  mMapMaker(mm),
  mCamera(c),
  mRelocaliser(mMap, mCamera),
  mirSize(irVideoSize)
{
  mCurrentKF.bFixed = false;
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
  TrackerData::irLeftImageSize = mirSize;
  TrackerData::irRightImageSize = mirSize;
  
  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;
  
  Reset();
}


void Tracker::Reset()
{
  mbDidCoarse = false;
  mbUserPressedSpacebar = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.Left().SetImageSize(mirSize);
  mCamera.Right().SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = false;
  

  mMapMaker.RequestReset();
  while(!mMapMaker.ResetDone())
#ifndef WIN32
	  usleep(10);
#else
	  Sleep(1);
#endif
}

// Track a single stereo pair

void Tracker::TrackFrame(Image<byte> &imFrame_L,Image<byte> &imFrame_R, bool bDraw)
{
  mbDraw = bDraw;
  mMessageForUser.str("");     

  mCurrentKF.mMeasurements.clear();
  mCurrentKF.MakeKeyFrame_Lite(imFrame_L);
  mCurrentKF.MakeKeyFrameStereo_Lite(imFrame_R);

  static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = *gvnUseSBI;
  if(!mpSBIThisFrame)
    {
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
      mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  else
    {
      delete  mpSBILastFrame;
      mpSBILastFrame = mpSBIThisFrame;
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }
  
  mnFrame++;
  
  if(mbDraw) 
    {
      
      mWindowL.make_current();
      glDrawPixels(mCurrentKF.aCamLeftLevels[0].im);
      mWindowR.make_current();
      glDrawPixels(mCurrentKF.aCamRightLevels[0].im);
      mWindowL.make_current();
      if(GV2.GetInt("Tracker.DrawFASTCorners",0, SILENT))
	{
	  glColor3f(1,0,1);  glPointSize(1); glBegin(GL_POINTS);
	  for(unsigned int i=0; i<mCurrentKF.aCamLeftLevels[0].vCorners.size(); i++) 
	    glVertex(mCurrentKF.aCamLeftLevels[0].vCorners[i]);
	  glEnd();
	}
    }
  
  if(mMap.IsGood())
    {
      if(mnLostFrames < 3)  
	{
	  if(mbUseSBIInit){
	    CalcSBIRotation();
	  }
	  ApplyMotionModel();
	  TrackMap();
	  UpdateMotionModel();      // 
	  
	  AssessTrackingQuality();  
	  
	  { 
	    mMessageForUser << "Tracking Map, quality ";
	    if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
	    if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
	    if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
	    mMessageForUser << " Found:";
	    for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
	    
	    mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
	  }
	  

	  if(mTrackingQuality == GOOD &&
	     mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
	     mnFrame - mnLastKeyFrameDropped > 20  &&
	     mMapMaker.QueueSize() < 3)
	    {
	      mMessageForUser << " Adding key-frame.";
	      AddNewKeyFrame();
	    };
	}
      else 
	{
	  mMessageForUser << "** Attempting recovery **.";
	  if(AttemptRecovery())
	    {
	      TrackMap();
	      AssessTrackingQuality();
	    }
	}
      // dont both with the grid in pelvis search 
      //if(mbDraw)
	//RenderGrid();
    }
  else 
    TrackForInitialMap(); 
    
  // GUI interface
  while(!mvQueuedCommands.empty())
    {
      GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
      mvQueuedCommands.erase(mvQueuedCommands.begin());
    }
};

bool Tracker::AttemptRecovery()
{
  bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
  if(!bRelocGood)
    return false;
  
  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}

// Draw the reference grid to give the user an idea of wether tracking is OK or not.
// project into both images
void Tracker::RenderGrid()
{
  if(mbDidCoarse)
    glColor4f(.0, 0.5, .0, 0.6);
  else
    glColor4f(0,0,0,0.6);
  
  int nHalfCells = 8;
  int nTot = nHalfCells * 2 + 1;
  Image<Vector<2> >  imLeftVertices(ImageRef(nTot,nTot));
  Image<Vector<2> > imRightVertices(ImageRef(nTot,nTot));
  for(int i=0; i<nTot; i++)
    for(int j=0; j<nTot; j++)
      {
	Vector<3> v3;
	v3[0] = (i - nHalfCells) * 0.25;
	v3[1] = (j - nHalfCells) * 0.25;
	v3[2] = 0.0;
	// get grid in both camera frames
	Vector<3> v3CamLeft = mse3CamFromWorld * v3;
	Vector<3> v3CamRight = mCamera.GetRelativePose() * v3CamLeft;
	if(v3CamLeft[2] < 0.001)
	  v3CamLeft[2] = 0.001;
	if(v3CamRight[2] < 0.001)
	  v3CamRight[2] = 0.001;
	imLeftVertices[i][j] = mCamera.ProjectToLeft(project(v3CamLeft)); 
	imRightVertices[i][j] = mCamera.ProjectToRight(project(v3CamRight));
      }
  
  mWindowL.make_current();
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(imLeftVertices[i][j]);
      glEnd();
      
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(imLeftVertices[j][i]);
      glEnd();
    };
  
  mWindowR.make_current();
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(2);
  for(int i=0; i<nTot; i++)
    {
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(imRightVertices[i][j]);
      glEnd();
      
      glBegin(GL_LINE_STRIP);
      for(int j=0; j<nTot; j++)
	glVertex(imRightVertices[j][i]);
      glEnd();
    };
  
  glLineWidth(1);
  glColor3f(1,0,0);
  mWindowL.make_current();
}

void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}

void Tracker::GUICommandHandler(string sCommand, string sParams) 
{
  if(sCommand=="Reset")
    {
      Reset();
      return;
    }

  if(sCommand=="KeyPress")
    {
      if(sParams == "Space")
	{
	  mbUserPressedSpacebar = true;
	}
      else if(sParams == "r")
	{
	  Reset();
	}
      else if(sParams == "q" || sParams == "Escape")
	{
	  GUI.ParseLine("quit");
	}
      return;
    }
  if((sCommand=="PokeTracker"))
    {
      mbUserPressedSpacebar = true;
      return;
    }
    
  
  cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 

// Routine for establishing the initial map over stereo paid
void Tracker::TrackForInitialMap()
{
  static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;
  
  if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED) 
    {
      if(mbUserPressedSpacebar)  
	{
	  mbUserPressedSpacebar = false;
	  // start tracking in left image
	  TrailTracking_Start();
	  // track to right...
	  int nGoodTrails = TrailTracking_Advance();
	  if(nGoodTrails < 10) 
	    {
	      cout << "tracking bad, only " << nGoodTrails << "found, reset" << endl;
	      Reset();
	      return;
	    }
	  vector<pair<ImageRef, ImageRef> > vMatches; 
	  for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); i++)
	    vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos,i->irCurrentPos));
	  // map maker init from new stereo pair
	  mMapMaker.InitFromStereo(mCurrentKF, vMatches, mse3CamFromWorld);  // This will take some time!
	  mnInitialStage = TRAIL_TRACKING_COMPLETE;
	} else {
	mMessageForUser << "Press spacebar to initialise map from stereo pair " << endl;
      }
    }
}

// The current frame is to be the first keyframe!
void Tracker::TrailTracking_Start()
{
  mCurrentKF.MakeKeyFrame_Rest();  
  mFirstKF = mCurrentKF; 
  vector<pair<double,ImageRef> > vCornersAndSTScores;
  for(unsigned int i=0; i<mCurrentKF.aCamLeftLevels[0].vCandidates.size(); i++) 
    {       
      Candidate &c = mCurrentKF.aCamLeftLevels[0].vCandidates[i];
      if(!mCurrentKF.aCamLeftLevels[0].im.in_image_with_border(c.irLevelPos, MiniPatch::mnHalfPatchSize))
	continue;
      vCornersAndSTScores.push_back(pair<double,ImageRef>(-1.0 * c.dSTScore, c.irLevelPos)); 
    };
  sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  
  int nToAdd = GV2.GetInt("MaxInitialTrails", 1000, SILENT);
  for(unsigned int i = 0; i<vCornersAndSTScores.size() && nToAdd > 0; i++)
    {
      if(!mCurrentKF.aCamLeftLevels[0].im.in_image_with_border(vCornersAndSTScores[i].second, MiniPatch::mnHalfPatchSize))
	continue;
      
      Trail t;
      t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aCamLeftLevels[0].im);
      t.irInitialPos = vCornersAndSTScores[i].second; //sets the image ref as the init pos
      t.irCurrentPos = t.irInitialPos;
      mlTrails.push_back(t);
      nToAdd--;
    }
  mPreviousFrameKF = mFirstKF;
}

// Steady-state trail tracking: Advance from the previous frame, remove duds.
int Tracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;
  if(mbDraw)
    {
      glPointSize(5);
      glLineWidth(2);
      glEnable(GL_POINT_SMOOTH);
      //glEnable(GL_LINE_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glEnable(GL_BLEND);
      glBegin(GL_POINTS);
    }
  
  MiniPatch BackwardsPatch;
  
  // get ref to both frames
  Level &LeftFrame = mCurrentKF.aCamLeftLevels[0];
  Level &RightFrame = mCurrentKF.aCamRightLevels[0];
  
  list<Trail>::iterator i;
  int count = 0;
  for(i = mlTrails.begin(); i!=mlTrails.end();)
    {
      
	  list<Trail>::iterator next = i; next++;

	  Trail &trail = *i;
	  ImageRef irStart = trail.irCurrentPos;
	  ImageRef irEnd = irStart;
	  // search the epipolar line in the right image for the left point
	  bool bFound = 
	    trail.mPatch.FindPatchEpipolar(irEnd,RightFrame.im,mCamera.GetEpipolarLine(irStart),RightFrame.vCorners);
	  if(bFound)
	    {
	      // the same again backwrads
	      BackwardsPatch.SampleFromImage(irEnd, RightFrame.im);
	      ImageRef irBackWardsFound = irEnd;
	      bFound = BackwardsPatch.FindPatchEpipolar(irBackWardsFound, LeftFrame.im, mCamera.GetEpipolarLine(irStart),LeftFrame.vCorners);

	      if((irBackWardsFound - irStart).mag_squared() > 2)
		bFound = false;
	      trail.irCurrentPos = irEnd;
	      nGoodTrails++;
	    }
	  
	  if(mbDraw)
	    {
	      if(!bFound){
		glColor3f(0,1,1);
	      }else{
		if(count==3){
		  glColor3f(1,1,0);
		  count = 0;
		}
		if(count == 2){
		  glColor3f(0.5,0.5,0);
		  count++;
		}
		if(count == 1){
		  glColor3f(0,0.5,1);
		  count++;
		}
		if(count == 0){
		  glColor3f(0.5,1,0);
		  count++;
		}
		glVertex(trail.irInitialPos);
	      }
	    }
	  if(!bFound) 
	    {
	      mlTrails.erase(i);
	    }
	  i = next;
    }
  if(mbDraw){
    glEnd();
    mWindowR.make_current();
    glPointSize(5);
    glColor3f(1,0,0);
    glBegin(GL_POINTS);
    list<Trail>::iterator n;
    int count = 0;
    for(n = mlTrails.begin();n!=mlTrails.end();n++){
      if(count==3){
	glColor3f(1,1,0);
	count = 0;
      }
      if(count == 2){
	glColor3f(0.5,0.5,0);
	count++;
      }
      if(count == 1){
	glColor3f(0,0.5,1);
	count++;
      }
      if(count == 0){
	glColor3f(0.5,1,0);
	count++; 
      }
      glVertex( (*n).irCurrentPos);
    }
    glEnd();
    mWindowL.make_current();
  }
  mPreviousFrameKF = mCurrentKF;
  cout << "number of good trails is " << nGoodTrails << endl;
  return nGoodTrails;
}

// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap()
{
  // Some accounting which will be used for tracking quality assessment:
  for(int i=0; i<LEVELS; i++)
     manMeasAttempted[i] = manMeasFound[i] = 0;
  
  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  vector<TrackerData*> avPVS[LEVELS]; 
  for(int i=0; i<LEVELS; i++)
    avPVS[i].reserve(500); //reserves space for each vector 

  // For all points in the map..
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++) 
    {

      MapPoint &p= *(mMap.vpPoints[i]);  
      if(!p.pTData) p.pTData = new TrackerData(&p);   
      TrackerData &TData = *p.pTData;
      
      // Project according to current view, 
      // if its not in both views skip
      TData.Project(mse3CamFromWorld, mCamera.Left(), mCamera.Right()); 
      
      // TO ADD: just left image, just right image - lower 
      // weight to these points
      if(!TData.bInBothImages){
	continue;   
      }
      
      // get both cam derivs.
      TData.GetDerivsUnsafe(mCamera.Left(), mCamera.Right());

      // use the stereo patch finder to get the warp matrices and search level of this featu8re
      TData.nSearchLevel = TData.StereoFinder.CalcSearchLevelAndBothPatchWarpMatrix(TData.Point, mse3CamFromWorld, mCamera.GetRelativePose()*mse3CamFromWorld, TData.m2LeftCamDerivs, TData.m2RightCamDerivs);
      
      if(TData.nSearchLevel == -1)
	continue;   
      
      TData.bSearched = false;
      TData.bFound = false;
      TData.bFoundEpipolar = false;
      avPVS[TData.nSearchLevel].push_back(&TData);
    };
   
  /* now avPVS contains 4 levels. each point that is determined to be 'in view'
   * is added to a pyramid level where the patch image is likely to be the same
   * size as the record of the patch. the determinant of the warping matrix (jacobian?) is used to determine
   * the size of the patch after the position change, this is matched against how large each pyramid level
   * thinks the patch should be.
   */

  for(int i=0; i<LEVELS; i++)
    random_shuffle(avPVS[i].begin(), avPVS[i].end());

  vector<TrackerData*> vNextToSearch;
  vector<TrackerData*> vIterationSet;
  
  // Tunable parameters to do with the coarse tracking stage:
  static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 5, SILENT);   // Min number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
  static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
  static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
  static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.03, SILENT);  // Speed above which coarse stage is used.

  unsigned int nCoarseMax = *gvnCoarseMax;
  unsigned int nCoarseRange = *gvnCoarseRange;
  
  mbDidCoarse = false;

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(*gvnCoarseDisabled || 
     mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
     nCoarseMax == 0)
    bTryCoarse = false;
  if(mbJustRecoveredSoUseCoarse)
    {
      bTryCoarse = true;
      nCoarseMax *=2;
      nCoarseRange *=2;
      mbJustRecoveredSoUseCoarse = false;
    };

  // DO COARSE PHASE OVER LEFT IMAGE ONLY
  
  if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin ) 
    {
      if(avPVS[LEVELS-1].size() <= nCoarseMax) 
	{ 
	  vNextToSearch = avPVS[LEVELS-1];
	  avPVS[LEVELS-1].clear();
	}
      else
	{ 
	  for(unsigned int i=0; i<nCoarseMax; i++)
	    vNextToSearch.push_back(avPVS[LEVELS-1][i]);
	  avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
	}
      
      if(vNextToSearch.size() < nCoarseMax)
	{
	  unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
	  if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
	    {
	      vNextToSearch = avPVS[LEVELS-2];
	      avPVS[LEVELS-2].clear();
	    }
	  else
	    {
	      for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
		vNextToSearch.push_back(avPVS[LEVELS-2][i]);
	      avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
	    }
	}

      //////////POTENTIALLY VISIBLE SETS NOW MADE///////////////
      
      unsigned int nFound = SearchForCoarsePoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
      vIterationSet = vNextToSearch; 
      if(nFound >= *gvnCoarseMin)  
	{
	  mbDidCoarse = true;
	  for(int iter = 0; iter<10; iter++) 
	    {
	      if(iter != 0)
		{ 
		  for(unsigned int i=0; i<vIterationSet.size(); i++)
		    if(vIterationSet[i]->bFound)  
		      vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera.Left(), mCamera.Right());
		}
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound)
		  vIterationSet[i]->CalcCoarseJacobian();
	      double dOverrideSigma = 0.0;
	      if(iter > 5)
		dOverrideSigma = 1.0;
	      
	      Vector<6> v6Update = 
		CalcPoseUpdateFromCoarse(vIterationSet, 
					 dOverrideSigma);
	      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
	    };
	}
    };
  
  // So, at this stage, we may or may not have done a coarse tracking stage.
  // now do the fine pose update over both images
  
  int nFineRange = 10;  // Pixel search range for the fine stage. 
  if(mbDidCoarse)      
    nFineRange = 5;
  
  {
    int l = LEVELS - 1;
    // project points to both images
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera.Left(), mCamera.Right());
    
    // search for points in both images
    SearchForPoints(avPVS[l],nFineRange,8);
   
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vIterationSet.push_back(avPVS[l][i]);
  };
  

  vNextToSearch.clear();
  for(int l=LEVELS - 2; l>=0; l--)
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vNextToSearch.push_back(avPVS[l][i]);
  

  static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
  if(nFinePatchesToUse < 0)
    nFinePatchesToUse = 0;
  if((int) vNextToSearch.size() > nFinePatchesToUse)
    {
      random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
      vNextToSearch.resize(nFinePatchesToUse); 
    };
  
  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse)
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
      vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera.Left(),mCamera.Right());
  
  // Find fine points in both images
  
  SearchForPoints(vNextToSearch, nFineRange, 0);

  // And attach them all to the end of the optimisation-set.
  for(unsigned int i=0; i<vNextToSearch.size(); i++)
    vIterationSet.push_back(vNextToSearch[i]);
    
  Vector<6> v6LastUpdate;
  v6LastUpdate = Zeros;
  for(int iter = 0; iter<15; iter++)
    {

      bool bNonLinearIteration; 
      if(iter == 0 || iter == 4 || iter == 9)
	bNonLinearIteration = true;   
      else                            
	bNonLinearIteration = false;  
      
      if(iter != 0)   
        {
	  if(bNonLinearIteration)
	    {
	      // genreate the derivs after projection into both images
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound && vIterationSet[i]->bFoundEpipolar)
		  vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera.Left(),mCamera.Right());
	    }
	  else
	    {
	      // do linear update to both points
	      for(unsigned int i=0; i<vIterationSet.size(); i++)
		if(vIterationSet[i]->bFound && vIterationSet[i]->bFoundEpipolar)
		  vIterationSet[i]->LinearUpdateBoth(v6LastUpdate);
	    };
	}
      // calc the jacobian over both images
      if(bNonLinearIteration)
	for(unsigned int i=0; i<vIterationSet.size(); i++)
	  if(vIterationSet[i]->bFound)
	    vIterationSet[i]->CalcFineJacobian();
      
      double dOverrideSigma = 0.0;
      if(iter > 5)
	dOverrideSigma = 16.0;
      
      // do the pose update on both images
      Vector<6> v6Update = 
	CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      v6LastUpdate = v6Update;
    };//end 10 iter updates
  
  
  // draw points into both views
  if(mbDraw)
    {
      mWindowL.make_current();
      glPointSize(6);
      glEnable(GL_BLEND);
      glEnable(GL_POINT_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBegin(GL_POINTS);
      for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
	  it!= vIterationSet.rend(); 
	  it++)
	{
	  if(! (*it)->bFound)
	    continue;
	  glColor(gavLevelColors[(*it)->nSearchLevel]);
	  glVertex((*it)->v2LeftImage);
	}
      glEnd();
      glDisable(GL_BLEND);
      mWindowR.make_current();
      glPointSize(6);
      glEnable(GL_BLEND);
      glEnable(GL_POINT_SMOOTH);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
      glBegin(GL_POINTS);
      for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
	  it!= vIterationSet.rend(); 
	  it++)
	{
	  if(! ((*it)->bFoundEpipolar))
	    continue;
	  glColor(gavLevelColors[(*it)->nSearchLevel]);
	  glVertex((*it)->v2RightImage);
	}
      glEnd();
      glDisable(GL_BLEND);
      mWindowL.make_current();
    }
  
 
  mCurrentKF.se3LeftCfromW = mse3CamFromWorld;
  mCurrentKF.se3RightCfromW = mCamera.GetRelativePose()*mse3CamFromWorld;
  mCurrentKF.mMeasurements.clear();
  for(vector<TrackerData*>::iterator it = vIterationSet.begin();
      it!= vIterationSet.end(); 
      it++)
    {
      if(! (*it)->bFound)
	continue;  
      // create stereo meas
      Measurement m; 
      m.v2RootPos = (*it)->v2LeftFound; 
      m.v2CorrespondingPos = (*it)->v2RightFound;
      m.nLevel = (*it)->nSearchLevel;
      m.bSubPix = (*it)->bDidSubPix; 
      mCurrentKF.mMeasurements[& ((*it)->Point)] = m; 
    }
  
  {
    double dSum = 0;
    double dSumSq = 0;
    int nNum = 0;
    for(vector<TrackerData*>::iterator it = vIterationSet.begin();
	it!= vIterationSet.end(); 
	it++)
      if((*it)->bFound)
	{
	  double z = (*it)->v3LeftCam[2];
	  dSum+= z;
	  dSumSq+= z*z;
	  nNum++;
	};
    if(nNum > 20)
      {
	mCurrentKF.dSceneDepthMean = dSum/nNum;
	mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
      }
  }
}

// Find points in the image. Uses the PatchFiner struct stored in TrackerData
/*
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts){
  //counter for the number of found points
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)
    {
      TrackerData &TD = *vTD[i];
      StereoPatchFinder &Finder = TD.StereoFinder;
      //Finder.CalcSearchLevelAndBothPatchWarpMatrix(TD.Point,mse3CamFromWorld, mCamera.Left().Extrinsic * mse3CamFromWorld, TD.m2LeftCamDerivs, TD.m2RightCamDerivs);
      Finder.MakeTemplateCoarseCont(TD.Point);
      TD.bFound = TD.bFoundEpipolar = false;
      
      if(Finder.TemplateBad())
	continue;
      manMeasAttempted[Finder.GetLevel()]++;
      bool bFound = Finder.FindPatchesInBothImagesCoarse(ir(TD.v2LeftImage), ir(TD.v2RightImage), mCurrentKF.aCamLeftLevels, mCurrentKF.aCamRightLevels, nRange);
      
      TD.bSearched = true;
      if(!bFound)
	continue;
      TD.bFound = TD.bFoundEpipolar = true;
      TD.dSqrtInvNoise = 1.0/Finder.GetLevelScale();
      nFound++;
      manMeasFound[Finder.GetLevel()]++;
      if(nSubPixIts > 0)
	{
	  TD.bDidSubPix = true;
	  Finder.MakeSubPixTemplate();
	  bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF.aCamLeftLevels, mCurrentKF.aCamRightLevels, nSubPixIts);
	  if(!bSubPixConverges){
	    TD.bFound = TD.bFoundEpipolar = false;
	    nFound--;
	    manMeasFound[Finder.GetLevel()]--;
	    continue;
	  }
	  TD.v2LeftFound = Finder.GetLeftSubPixPos();
	  TD.v2RightFound = Finder.GetRightSubPixPos();
	}else{
	TD.v2LeftFound = Finder.GetLeftCoarsePosAsVector();
	TD.v2RightFound = Finder.GetRightCoarsePosAsVector();
	TD.bDidSubPix = false;
      }
    }
  return nFound;
}
*/
/*

int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   
  {
      
  TrackerData &TD = *vTD[i];
  
  //////////left image search /////////////////////
  PatchFinder &Finder= TD.StereoFinder.GetLeftPatchFinder();
  //PatchFinder &Finder= TD.NewPointFinder;
  // For the map point, using the current location
  // of the tracker calculates a warped view of the
  // map point 
  Finder.MakeTemplateCoarseCont(TD.Point);
  TD.bFound = TD.bFoundEpipolar = false;
  if(!Finder.TemplateBad())
  {
  manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta
      
  bool bFound = 
  Finder.FindPatchCoarse(ir(TD.v2LeftImage), mCurrentKF.aCamLeftLevels, nRange);
      
      
  TD.bSearched = true;
      
  if(!bFound) 
  {
    TD.bFoundEpipolar = false;
    TD.bFound = false;
    continue;
  }else{
	    
  TD.bFound = true;
  TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
	  
  //nFound++;
  //manMeasFound[Finder.GetLevel()]++;
	    
  if(nSubPixIts > 0)
  {
  TD.bDidSubPix = true;
  Finder.MakeSubPixTemplate();
  bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF.aCamLeftLevels, nSubPixIts);
  if(!bSubPixConverges)
  { 
		    
  TD.bFound = false;
  TD.bFoundEpipolar = false;
  //nFound--;
  //manMeasFound[Finder.GetLevel()]--;
  continue;
  }
  else{
  TD.v2LeftFound = Finder.GetSubPixPos();
  }
  }
  else
  {
  TD.v2LeftFound = Finder.GetCoarsePosAsVector();
  TD.bDidSubPix = false;
  }
  }
  }//endif
      
  //NOW FIND PATCH IN RIGHT IMAGE
      
  //PatchFinder &EpiFinder = TD.EpipolarFinder;
  PatchFinder &EpiFinder = TD.StereoFinder.GetRightPatchFinder();
  // For the map point, using the current location
  // of the tracker calculates a warped view of the
  // map point 
		
	
  if(TD.bFound){
    EpiFinder.CalcWarpMatrix(TD.Point, mCamera.Left().Extrinsic * mse3CamFromWorld, TD.m2RightCamDerivs, TD.nSearchLevel);
    EpiFinder.MakeTemplateCoarseCont(TD.Point,false);
    
    //ImageRef irLevelPos = ir(TD.v2LeftFound)/LevelScale(TD.nSearchLevel);
    //EpiFinder.MakeTemplateCoarseNoWarp(mCurrentKF.aCamLeftLevels, TD.nSearchLevel,irLevelPos);
    if(EpiFinder.TemplateBad()){
      TD.bFound = TD.bFoundEpipolar = false;
    continue;
    }
  }else{
    continue;
    EpiFinder.CalcWarpMatrix(TD.Point, mCamera.Left().Extrinsic * mse3CamFromWorld, TD.m2RightCamDerivs, TD.nSearchLevel);
    EpiFinder.MakeTemplateCoarseCont(TD.Point,false);
    if(EpiFinder.TemplateBad())
      {
	TD.bFound = false;
	TD.bFoundEpipolar = false;
	continue;
      }
  }
  
     
  bool bFoundEpipolar = 
    EpiFinder.FindPatchCoarse(ir(TD.v2RightImage), mCurrentKF.aCamRightLevels, nRange);
      
  TD.bSearched = true;
  if(!bFoundEpipolar) 
    {
      TD.bFound = false;
      TD.bFoundEpipolar = false;
      continue;
    }else{
    TD.bFoundEpipolar = true;
    nFound++;
    manMeasFound[EpiFinder.GetLevel()]++;
    if(nSubPixIts > 0)
      {
	TD.bDidSubPix = true;
	EpiFinder.MakeSubPixTemplate();
	bool bSubPixConverges = EpiFinder.IterateSubPixToConvergence(mCurrentKF.aCamRightLevels, nSubPixIts);
	if(!bSubPixConverges)
	  { 
	    TD.bFound = false;
	    TD.bFoundEpipolar = false;
	    nFound--;
	    manMeasFound[Finder.GetLevel()]--;
	    continue;
	  }
	else{
	  TD.v2RightFound = EpiFinder.GetSubPixPos();
	}
      }else{
      TD.v2RightFound = EpiFinder.GetCoarsePosAsVector();
    }
    
  }  
  }
  return nFound;
};
*/


 // search for points in boht images
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   
    {
      
      TrackerData &TD = *vTD[i];
  
      // create two patch finders refs for later bit
      PatchFinder &Finder= TD.StereoFinder.GetLeftPatchFinder();
      PatchFinder &EpiFinder = TD.StereoFinder.GetRightPatchFinder();

      // use the stereo patch finder to gen the template
      TD.StereoFinder.MakeTemplateCoarseCont(TD.Point);
      TD.bFound = TD.bFoundEpipolar = false;
      if(Finder.TemplateBad() || EpiFinder.TemplateBad()) 
	continue;
      
      // find the patch ith a coarse search
      bool bFound = 
	Finder.FindPatchCoarse(ir(TD.v2LeftImage), mCurrentKF.aCamLeftLevels, nRange);
      manMeasAttempted[Finder.GetLevel()]++;
      
      TD.bSearched = true;
      
      if(!bFound) 
	{
	  TD.bFoundEpipolar = false;
	  TD.bFound = false;
	  continue;
	}
	    
	TD.bFound = true;
	TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
	
	if(nSubPixIts > 0)
	  {
	    TD.bDidSubPix = true;
	    Finder.MakeSubPixTemplate();
	    bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF.aCamLeftLevels, nSubPixIts);
	    if(!bSubPixConverges)
	      { 
		
		TD.bFound = false;
		TD.bFoundEpipolar = false;
		continue;
	      }
	    else{
	      TD.v2LeftFound = Finder.GetSubPixPos();
	    }
	  }
	else
	  {
	    TD.v2LeftFound = Finder.GetCoarsePosAsVector();
	    TD.bDidSubPix = false;
	  }
	
	// find the point in the right image now
	bool bFoundEpipolar = 
	  EpiFinder.FindPatchCoarse(ir(TD.v2RightImage), mCurrentKF.aCamRightLevels, nRange);
	TD.bSearched = true;
	if(!bFoundEpipolar) 
	  {
	  TD.bFound = false;
	  TD.bFoundEpipolar = false;
	  continue;
	}
	
	TD.bFoundEpipolar = true;
	nFound++;
	manMeasFound[EpiFinder.GetLevel()]++;
	if(nSubPixIts > 0)
	  {
	    TD.bDidSubPix = true;
	    EpiFinder.MakeSubPixTemplate();
	    bool bSubPixConverges = EpiFinder.IterateSubPixToConvergence(mCurrentKF.aCamRightLevels, nSubPixIts);
	    if(!bSubPixConverges)
	      { 
		TD.bFound = false;
		TD.bFoundEpipolar = false;
		nFound--;
		manMeasFound[Finder.GetLevel()]--;
		continue;
	      }
	    else{
	      TD.v2RightFound = EpiFinder.GetSubPixPos();
	    }
	  }else{
	  TD.v2RightFound = EpiFinder.GetCoarsePosAsVector();
	}
    }
  return nFound;
};

// 
// Find the points in the left image
//
int Tracker::SearchForCoarsePoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   
    {
      TrackerData &TD = *vTD[i];
      PatchFinder &Finder= TD.StereoFinder.GetLeftPatchFinder();
      Finder.MakeTemplateCoarseCont(TD.Point);
      if(Finder.TemplateBad())
	{
	  TD.bInBothImages = TD.bPotentiallyVisible = TD.bFound = false;
	  continue;
	}
      manMeasAttempted[Finder.GetLevel()]++;  
      
      bool bFound = 
	Finder.FindPatchCoarse(ir(TD.v2LeftImage), mCurrentKF.aCamLeftLevels, nRange);
      TD.bSearched = true;
      if(!bFound) 
	{
	  TD.bFound = false;
	  continue;
	}
      
      TD.bFound = true;
      TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());
      
      nFound++;
      manMeasFound[Finder.GetLevel()]++;
      
      if(nSubPixIts > 0)
	{
	  TD.bDidSubPix = true;
	  Finder.MakeSubPixTemplate();
	  bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF.aCamLeftLevels, nSubPixIts);
	  if(!bSubPixConverges)
	    { 
	      TD.bFound = false;
	      nFound--;
	      manMeasFound[Finder.GetLevel()]--;
	      continue;
	    }
	  TD.v2LeftFound = Finder.GetSubPixPos();
	}
      else
	{
	  TD.v2LeftFound = Finder.GetCoarsePosAsVector();
	  TD.bDidSubPix = false;
	}
    }
  return nFound;
};

Vector<6> Tracker::CalcPoseUpdateFromCoarse(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
  int nEstimator = 0;
  static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  if(*gvsEstimator == "Tukey")
    nEstimator = 0;
  else if(*gvsEstimator == "Cauchy")
    nEstimator = 1;
  else if(*gvsEstimator == "Huber")
    nEstimator = 2;
  else 
    {
      cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
      nEstimator = 0;
      *gvsEstimator = "Tukey";
    };
  
  vector<double> vdErrorSquared;
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
	continue;
      TD.v2Error_CovScaledLeft = TD.dSqrtInvNoise* (TD.v2LeftFound - TD.v2LeftImage);
      vdErrorSquared.push_back(TD.v2Error_CovScaledLeft * TD.v2Error_CovScaledLeft);
    };
  
  if(vdErrorSquared.size() == 0)
    return makeVector( 0,0,0,0,0,0);
  
  double dSigmaSquared;
  if(dOverrideSigma > 0)
    dSigmaSquared = dOverrideSigma; 
  else
    {
      if (nEstimator == 0)
	dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      else if(nEstimator == 1)
	dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else 
	dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
    }

  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
	continue;
      Vector<2> &v2 = TD.v2Error_CovScaledLeft;
      double dErrorSq = v2 * v2;
      double dWeight;
      
      if(nEstimator == 0)
	dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
	dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else 
	dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
      
      if(dWeight == 0.0)
	{
	  if(bMarkOutliers)
	    TD.Point.nMEstimatorOutlierCount++;
	  continue;
	}
      else
	if(bMarkOutliers)
	  TD.Point.nMEstimatorInlierCount++;
      
      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); 
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); 
    }
  
  wls.compute();
  return wls.get_mu();
}

/* Do a fine tracking pose update, this uses teh points in both
   of the images for the guass newton step - calls CalcFineJac
   and LinearUpdateBoth 
*/

Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
  int nEstimator = 0;
  static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  if(*gvsEstimator == "Tukey")
    nEstimator = 0;
  else if(*gvsEstimator == "Cauchy")
    nEstimator = 1;
  else if(*gvsEstimator == "Huber")
    nEstimator = 2;
  else 
    {
      cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
      nEstimator = 0;
      *gvsEstimator = "Tukey";
    };
  
  
  vector<double> vdErrorSquaredA;
  vector<double> vdErrorSquaredB;
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      for(int i=0;i<2;i++){
	if( (TD.v2LeftImage[i] != TD.v2LeftImage[i]) ||
	    (TD.v2LeftFound[i] != TD.v2LeftFound[i]) ||
	    (TD.v2RightImage[i]!=TD.v2RightImage[i]) ||
	    (TD.v2RightFound[i]!=TD.v2RightFound[i]) ){
	  TD.bFound = TD.bFoundEpipolar = false;
	}
      }
      if(!TD.bFound && !TD.bFoundEpipolar)
	continue;
  
      // get the covars from both images
      Vector<2> covarianceLeft;
      covarianceLeft = (TD.v2LeftFound-TD.v2LeftImage);
      Vector<2> covarianceRight;
      covarianceRight = (TD.v2RightFound-TD.v2RightImage);
      // if not found - skip
      for(int i=0;i<2;i++)
	if((covarianceLeft[i]*covarianceLeft[i]) > 200 || (covarianceLeft[i]*covarianceLeft[i]) < -200 || (covarianceRight[i]*covarianceRight[i]) > 200 || (covarianceRight[i]*covarianceRight[i]) < -200){
	  TD.bFoundEpipolar = false;
	  TD.bFound = false;
	  continue;
	}
      
      // scale the covars
      TD.v2Error_CovScaledLeft = TD.dSqrtInvNoise*(covarianceLeft);
      TD.v2Error_CovScaledRight = TD.dSqrtInvNoise*(covarianceRight);
      vdErrorSquaredA.push_back(TD.v2Error_CovScaledLeft*TD.v2Error_CovScaledLeft);
      vdErrorSquaredB.push_back(TD.v2Error_CovScaledRight*TD.v2Error_CovScaledRight);
    };
  

  if(vdErrorSquaredA.size() == 0)
    return makeVector( 0,0,0,0,0,0);
  
  // What is the distribution of error of both vectors
  double dSigmaSquaredA;
  double dSigmaSquaredB;
  if(dOverrideSigma > 0)
    dSigmaSquaredA = dSigmaSquaredB = dOverrideSigma; 
  else
    {
      if(nEstimator == 0){
	dSigmaSquaredA = Tukey::FindSigmaSquared(vdErrorSquaredA);
	dSigmaSquaredB = Tukey::FindSigmaSquared(vdErrorSquaredB);
      }

      else if(nEstimator == 1){
	dSigmaSquaredA = Cauchy::FindSigmaSquared(vdErrorSquaredA);
	dSigmaSquaredB = Cauchy::FindSigmaSquared(vdErrorSquaredB);
      }
      else {
	dSigmaSquaredA = Huber::FindSigmaSquared(vdErrorSquaredA);
	dSigmaSquaredB = Huber::FindSigmaSquared(vdErrorSquaredB);
      }
    }
  
  WLS<6> wls;
  wls.add_prior(300.0);
  for(unsigned int f=0; f<vTD.size(); f++)
    {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound && !TD.bFoundEpipolar)
	continue;
      // get the covars
      Vector<2> &v2Left = TD.v2Error_CovScaledLeft;
      Vector<2> &v2Right = TD.v2Error_CovScaledRight;
      // get the error
      double dErrorSqA = v2Left * v2Left;
      double dErrorSqB = v2Right * v2Right;
      double dWeightA;
      double dWeightB;
      
      if(nEstimator == 0){
	dWeightA = Tukey::Weight(dErrorSqA, dSigmaSquaredA);
	dWeightB = Tukey::Weight(dErrorSqB, dSigmaSquaredB);
      }
      else if(nEstimator == 1){
	dWeightA = Cauchy::Weight(dErrorSqA, dSigmaSquaredA);
	dWeightB = Cauchy::Weight(dErrorSqB, dSigmaSquaredB);
      }else{
	dWeightA = Huber::Weight(dErrorSqA, dSigmaSquaredA);
	dWeightB = Huber::Weight(dErrorSqB, dSigmaSquaredB);
      }

      if(dWeightA == 0.0 || dWeightB == 0.0)
	{
	  if(bMarkOutliers)
	    TD.Point.nMEstimatorOutlierCount++;
	  continue;
	}
      else
	if(bMarkOutliers)
	  TD.Point.nMEstimatorInlierCount++;
      
      // the jacs
      Matrix<4,6> &m46Jac = TD.m46Jacobian;
      
      // add to the wls class
      wls.add_mJ(v2Left[0], TD.dSqrtInvNoise * m46Jac[0], dWeightA); 
      wls.add_mJ(v2Left[1], TD.dSqrtInvNoise * m46Jac[1], dWeightA); 
      wls.add_mJ(v2Right[0], TD.dSqrtInvNoise * m46Jac[2], dWeightB); 
      wls.add_mJ(v2Right[1], TD.dSqrtInvNoise * m46Jac[3], dWeightB); 
      
    }
    
    wls.compute();
    return wls.get_mu();
}

void Tracker::ApplyMotionModel()
{
  mse3StartPos = mse3CamFromWorld;
  Vector<6> v6Velocity = mv6CameraVelocity;
  if(mbUseSBIInit)
    {
      v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>(); 
      v6Velocity[0] = 0.0;
      v6Velocity[1] = 0.0;
    }
  mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
};


void Tracker::UpdateMotionModel()
{
  SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse(); 
  Vector<6> v6Motion = SE3<>::ln(se3NewFromOld); 
  Vector<6> v6OldVel = mv6CameraVelocity;
  
  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
  mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);
  
  Vector<6> v6 = mv6CameraVelocity;
  v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}


void Tracker::AddNewKeyFrame()
{
  mMapMaker.AddKeyFrame(mCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
}


void Tracker::AssessTrackingQuality()
{
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;
  
  for(int i=0; i<LEVELS; i++)
    {
      nTotalAttempted += manMeasAttempted[i];
      nTotalFound += manMeasFound[i];
      if(i>=2) nLargeAttempted += manMeasAttempted[i];
      if(i>=2) nLargeFound += manMeasFound[i];
    }
  
  if(nTotalFound == 0 || nTotalAttempted == 0)
    mTrackingQuality = BAD;
  else
    {
      double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
      double dLargeFracFound;
      if(nLargeAttempted > 10)
	dLargeFracFound = (double) nLargeFound / nLargeAttempted;
      else
	dLargeFracFound = dTotalFracFound;

      static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.15, SILENT);
      static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.05, SILENT);
      
      
      if(dTotalFracFound > *gvdQualityGood)
	mTrackingQuality = GOOD;
      else if(dLargeFracFound < *gvdQualityLost)
	mTrackingQuality = BAD;
      else
	mTrackingQuality = DODGY;
    }
  
  if(mTrackingQuality == DODGY)
    {
      if(mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
	mTrackingQuality = BAD;
    }
  
  if(mTrackingQuality==BAD)
    mnLostFrames++;
  else
    mnLostFrames = 0;
}

string Tracker::GetMessageForUser()
{
  return mMessageForUser.str();
}

void Tracker::CalcSBIRotation()
{
  mpSBILastFrame->MakeJacs(); 
  pair<SE2<>, double> result_pair;
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  mv6SBIRot = se3Adjust.ln();
}

ImageRef TrackerData::irLeftImageSize;  // Static member of TrackerData lives here
ImageRef TrackerData::irRightImageSize;








