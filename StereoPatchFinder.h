#ifndef __STEREO_PATCHFINDER_H
#define __STEREO_PATCHFINDER_H

#include "PatchFinder.h"
#include<iostream>
using namespace std;
struct PatchImageData{
  CVD::Image<CVD::byte> mimTemplate;
  CVD::Image<std::pair<float,float> > mimJacs;
  Matrix<3> mm3HInv;
  double mdMeanDiff;
  //these two are the sum of the pixel intensity over the patch
  int mnTemplateSum;
  int mnTemplateSumSq;
};

class StereoPatchFinder : public PatchFinder
{
 public:
  StereoPatchFinder(int nPatchSize = 8);
  
  //step 1: calculate the search level and warp matrix for
  //        both patches
  int CalcSearchLevelAndBothPatchWarpMatrix(MapPoint &p, SE3<> se3LeftCFromW, SE3<> se3RightCFromW, Matrix<2> &m2LeftCamDerivs, Matrix<2> &m2RightCamDerivs);

  // wrapper functions for making templates across stereo pair
  void MakeTemplatesCoarse(MapPoint &p, SE3<> se3LeftCFromW, SE3<> se3RightCFromW, Matrix<2> &m2LeftCamDerivs, Matrix<2> &m2RightCamDerivs);
  void MakeTemplateCoarseCont(MapPoint &p);
  //void CopyTemplate();
  
  //step 3: find patches in image
  bool FindPatchesInBothImagesCoarse(CVD::ImageRef irLeft, 
				     CVD::ImageRef irRight,
				     Level LeftLevs[],
				     Level RightLevs[],
				     unsigned int nRange);

  inline CVD::ImageRef GetLeftCoarsePos(){
    return mLeftImageFinder.GetCoarsePos();
  }
  inline Vector<2> GetLeftCoarsePosAsVector(){
    return mLeftImageFinder.GetCoarsePosAsVector();
  }
  inline CVD::ImageRef GetRightCoarsePos(){
    return mRightImageFinder.GetCoarsePos();
  }
  inline Vector<2> GetRightCoarsePosAsVector(){
    return mRightImageFinder.GetCoarsePosAsVector();
  }
  inline int GetLevel(){
    return mLeftImageFinder.GetLevel();
  }

  //step 4: make the templates for sub pixel matching
  inline void MakeSubPixTemplate(){
    mLeftImageFinder.MakeSubPixTemplate();
    mRightImageFinder.MakeSubPixTemplate();
  }
  //step 5: do sub pixel matching
  inline bool IterateSubPixToConvergence(Level *LeftLev, Level *RightLev, int nMaxIts){
    bool a = mLeftImageFinder.IterateSubPixToConvergence(LeftLev,nMaxIts);
    bool b = mRightImageFinder.IterateSubPixToConvergence(RightLev,nMaxIts);
    if(!a) 
      cout << "failed at a" <<endl;
    if(!b)
      cout << "failed at b" << endl;
    return a && b;
  }
  
  inline Vector<2> GetLeftSubPixPos()  { return mLeftImageFinder.GetSubPixPos();   }  // Get result
  void SetLeftSubPixPos(Vector<2> v2)  { mLeftImageFinder.SetSubPixPos(v2);     }  // Set starting point

  inline Vector<2> GetRightSubPixPos()  { return mRightImageFinder.GetSubPixPos();   }  // Get result
  void SetRightSubPixPos(Vector<2> v2)  { mRightImageFinder.SetSubPixPos(v2);     }  // Set starting point
   
  inline bool TemplateBad(){
    return mRightImageFinder.TemplateBad() || mLeftImageFinder.TemplateBad();
  }

  inline PatchFinder &GetLeftPatchFinder(){
    return mLeftImageFinder;
  }
  inline PatchFinder &GetRightPatchFinder(){
    return mRightImageFinder;
  }
 protected:
  
  PatchFinder mLeftImageFinder;
  PatchFinder mRightImageFinder;

  PatchImageData *mPatchInLeft;
  PatchImageData *mPatchInRight; // this may point to the left patch if the patch looks similar enough in the two images
  
  
};
#endif
