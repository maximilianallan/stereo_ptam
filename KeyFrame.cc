// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"
#include "ShiTomasi.h"
#include "SmallBlurryImage.h"
#include <cvd/vision.h>
#include <cvd/fast_corner.h>

using namespace CVD;
using namespace std;
using namespace GVars3;


// this is called on both images in stereo pair
void KeyFrame::MakeKeyFrame_Lite(BasicImage<byte> &im)
{
  aCamLeftLevels[0].im.resize(im.size());
  copy(im, aCamLeftLevels[0].im);
  
  for(int i=0; i<LEVELS; i++)
    {
      Level &lev = aCamLeftLevels[i];
      if(i!=0)
	{  
	  lev.im.resize(aCamLeftLevels[i-1].im.size() / 2);
	  halfSample(aCamLeftLevels[i-1].im, lev.im);
	}
      
      lev.vCorners.clear();
      lev.vCandidates.clear();
      lev.vMaxCorners.clear();
      if(i == 0)
	fast_corner_detect_10(lev.im, lev.vCorners, 10);
      if(i == 1)
	fast_corner_detect_10(lev.im, lev.vCorners, 15);
      if(i == 2)
	fast_corner_detect_10(lev.im, lev.vCorners, 15);
      if(i == 3)
	fast_corner_detect_10(lev.im, lev.vCorners, 10);
      
      
      unsigned int v=0;
      lev.vCornerRowLUT.clear();
      for(int y=0; y<lev.im.size().y; y++)
	{
	  while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
	    v++;
	  lev.vCornerRowLUT.push_back(v);
	}
    };
}

//just create the blurry images 
void KeyFrame::MakeKeyFrameStereo_Lite(CVD::BasicImage<CVD::byte> &im){
  aCamRightLevels[0].im.resize(im.size());
  copy(im, aCamRightLevels[0].im);
  
  // Then, for each level...
  for(int i=0; i<LEVELS; i++)
    {
      Level &lev = aCamRightLevels[i];
      if(i!=0)
	{  // .. make a half-size image from the previous level..
	  lev.im.resize(aCamRightLevels[i-1].im.size() / 2);
	  halfSample(aCamRightLevels[i-1].im, lev.im);
	}
      lev.vCorners.clear();
      lev.vCandidates.clear();
      lev.vMaxCorners.clear();
      if(i == 0)
	fast_corner_detect_10(lev.im, lev.vCorners, 10); 
      //fast corner detect takes an image and a container for the corner references... then it populates that container.
      if(i == 1)
	fast_corner_detect_10(lev.im, lev.vCorners, 15);
      if(i == 2)
	fast_corner_detect_10(lev.im, lev.vCorners, 15);
      if(i == 3)
	fast_corner_detect_10(lev.im, lev.vCorners, 10);
      
      unsigned int v=0;
      lev.vCornerRowLUT.clear();
      for(int y=0; y<lev.im.size().y; y++)
	{
	  while(v < lev.vCorners.size() && y > lev.vCorners[v].y)
	    v++;
	  lev.vCornerRowLUT.push_back(v);
	}

    }   
};


void KeyFrame::MakeKeyFrame_Rest()
{
  static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  
  // For each level...
  for(int l=0; l<LEVELS; l++)
    {
      // find the non-max corners in both images
      Level &levLeft = aCamLeftLevels[l]; 
      Level &levRight = aCamRightLevels[l];
      fast_nonmax(levLeft.im,levLeft.vCorners,10,levLeft.vMaxCorners);
      fast_nonmax(levRight.im,levRight.vCorners,10,levRight.vMaxCorners);
      // score the left features
      
      for(vector<ImageRef>::iterator i=levLeft.vMaxCorners.begin(); i!=levLeft.vMaxCorners.end(); i++)
	{
	  if(!levLeft.im.in_image_with_border(*i, 10))
	    continue;
	  double dSTScore = FindShiTomasiScoreAtPoint(levLeft.im, 3, *i);
	  if(dSTScore > *gvdCandidateMinSTScore)
	    {
	      Candidate c;
	      c.irLevelPos = *i;
	      c.dSTScore = dSTScore;
	      levLeft.vCandidates.push_back(c);
	    }
	}

      // score the right features
      
      for(vector<ImageRef>::iterator i=levRight.vMaxCorners.begin(); i!=levRight.vMaxCorners.end(); i++)
	{
	  if(!levRight.im.in_image_with_border(*i, 10))
	    continue;
	  double dSTScore = FindShiTomasiScoreAtPoint(levRight.im, 3, *i);
	  if(dSTScore > *gvdCandidateMinSTScore)
	    {
	      Candidate c;
	      c.irLevelPos = *i;
	      c.dSTScore = dSTScore;
	      levRight.vCandidates.push_back(c);
	    }
	}
   };
 
  
  pSBI = new SmallBlurryImage(*this);  
  pSBI->MakeJacs();
}

Level& Level::operator=(const Level &rhs)
{
  im.resize(rhs.im.size());
  copy(rhs.im, im);
  
  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}
Vector<3> gavLevelColors[LEVELS];
struct LevelHelpersFiller 
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
      {
	if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
	else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
	else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
	else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 0.7);
	else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7);
      }
  }
};
static LevelHelpersFiller foo;







