#include "StereoPatchFinder.h"

StereoPatchFinder::StereoPatchFinder(int nPatchSize):mLeftImageFinder(nPatchSize),mRightImageFinder(nPatchSize){
}

// Calculate the search level and warp matrix for both views. If the search level is not determined as the same for both views, 
// return false
int StereoPatchFinder::CalcSearchLevelAndBothPatchWarpMatrix(MapPoint &p, SE3<> se3LeftCFromW, SE3<> se3RightCFromW, Matrix<2> &m2LeftCamDerivs, Matrix<2> &m2RightCamDerivs){
  mLeftImageFinder.CalcSearchLevelAndWarpMatrix(p,se3LeftCFromW,m2LeftCamDerivs);
  //calculate the warp matrix for the right view
  //mRightImageFinder.CalcWarpMatrix(p,se3RightCFromW,m2RightCamDerivs, mLeftImageFinder.GetLevel());
  mRightImageFinder.CalcSearchLevelAndWarpMatrix(p,se3RightCFromW,m2RightCamDerivs);
  if(mLeftImageFinder.GetLevel() != mRightImageFinder.GetLevel())
    return -1;
  return mLeftImageFinder.GetLevel();
}

void StereoPatchFinder::MakeTemplateCoarseCont(MapPoint &p){
  mLeftImageFinder.MakeTemplateCoarseCont(p);
  if(!mRightImageFinder.IsTemplateSimilar(mLeftImageFinder))
    mRightImageFinder.MakeTemplateCoarseCont(p,false);
  else
    mRightImageFinder.CopyTemplateFrom(mLeftImageFinder);
}

bool StereoPatchFinder::FindPatchesInBothImagesCoarse(CVD::ImageRef irLeft, CVD::ImageRef irRight, Level LeftLevs[], Level RightLevs[], unsigned int nRange)
{
  bool a = mLeftImageFinder.FindPatchCoarse(irLeft, LeftLevs, nRange);
  bool b = mRightImageFinder.FindPatchCoarse(irRight,RightLevs,nRange);
  if(!a)
    cout << " left find was bad "<< endl;
  if(!b)
    cout << " right find was bad " << endl;
  return a && b;
}



void StereoPatchFinder::MakeTemplatesCoarse(MapPoint &p, SE3<> se3LeftCFromW, SE3<> se3RightCFromW, Matrix<2> &m2LeftCamDerivs, Matrix<2> &m2RightCamDerivs){
  CalcSearchLevelAndBothPatchWarpMatrix(p, se3LeftCFromW, se3RightCFromW, m2LeftCamDerivs, m2RightCamDerivs);
  MakeTemplateCoarseCont(p);
}
  
