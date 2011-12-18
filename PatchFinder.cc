// Copyright 2008 Isis Innovation Limited
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "KeyFrame.h"

#include <cvd/vision.h>
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>
#include <TooN/Cholesky.h>

#if CVD_HAVE_XMMINTRIN
#include <tmmintrin.h>
#endif

using namespace CVD;
using namespace std;

PatchFinder::PatchFinder(int nPatchSize)
  : mimTemplate(ImageRef(nPatchSize,nPatchSize))
{
  mnPatchSize = nPatchSize; 
  mirCenter = ImageRef(nPatchSize/2, nPatchSize/2);
  int nMaxSSDPerPixel = 400; 
  mnMaxSSD = mnPatchSize * mnPatchSize * nMaxSSDPerPixel;
  mm2LastWarpMatrix = 9999.9 * Identity;
  mpLastTemplateMapPoint = NULL;
};


// Find the warping matrix and search level
// this is used to find patches in the left frame
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint &p,
					      SE3<> se3CFromW,
					      Matrix<2> &m2CamDerivs)
{
  Vector<3> v3Cam = se3CFromW * p.v3WorldPos;
  double dOneOverCameraZ = 1.0 / v3Cam[2];
  Vector<3> v3MotionRight = se3CFromW.get_rotation() * p.v3LeftFramePixelRight_W;
  Vector<3> v3MotionDown = se3CFromW.get_rotation() * p.v3LeftFramePixelDown_W;
  mm2WarpInverse.T()[0] = m2CamDerivs * (v3MotionRight.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionRight[2] * dOneOverCameraZ) * dOneOverCameraZ;
  mm2WarpInverse.T()[1] = m2CamDerivs * (v3MotionDown.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionDown[2] * dOneOverCameraZ) * dOneOverCameraZ;
  double dDet = mm2WarpInverse[0][0] * mm2WarpInverse[1][1] - mm2WarpInverse[0][1] * mm2WarpInverse[1][0];
  mnSearchLevel = 0;
  
  while(dDet > 3 && mnSearchLevel < LEVELS-1)
    {
      mnSearchLevel++;
      dDet *= 0.25;
    };
  
  
  if(dDet > 3 || dDet < 0.25)
    {
      mbTemplateBad = true;
      return -1;
    }
  else
    return mnSearchLevel;
}

// just calc the warp matrix - this is used by the second stereo view
void PatchFinder::CalcWarpMatrix(MapPoint &p,  SE3<> se3CFromW, Matrix<2> &m2CamDerivs, int searchLevel){
  Vector<3> v3Cam = se3CFromW * p.v3WorldPos;
  double dOneOverCameraZ = 1.0 / v3Cam[2];
  Vector<3> v3MotionRight = se3CFromW.get_rotation() * p.v3RightFramePixelRight_W;
  Vector<3> v3MotionDown = se3CFromW.get_rotation() * p.v3RightFramePixelDown_W;
  mm2WarpInverse.T()[0] = m2CamDerivs * (v3MotionRight.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionRight[2] * dOneOverCameraZ) * dOneOverCameraZ;
  mm2WarpInverse.T()[1] = m2CamDerivs * (v3MotionDown.slice<0,2>() - v3Cam.slice<0,2>() * v3MotionDown[2] * dOneOverCameraZ) * dOneOverCameraZ;
  
  mnSearchLevel = searchLevel;
}

void PatchFinder::MakeTemplateCoarse(MapPoint &p,
				     SE3<> se3CFromW,
				     Matrix<2> &m2CamDerivs)
{
  CalcSearchLevelAndWarpMatrix(p, se3CFromW, m2CamDerivs);
  MakeTemplateCoarseCont(p);
};

// if true set - generate template from left image - if not do right 
void PatchFinder::MakeTemplateCoarseCont(MapPoint &p, bool left /*defaults true */)
{

  Matrix<2> m2 = M2Inverse(mm2WarpInverse) * LevelScale(mnSearchLevel); 
  bool bNeedToRefreshTemplate = false;
  if(&p != mpLastTemplateMapPoint)
    bNeedToRefreshTemplate = true;
  for(int i=0; !bNeedToRefreshTemplate && i<2; i++)
    {
      Vector<2> v2Diff = m2.T()[i] - mm2LastWarpMatrix.T()[i];
      const double dRefreshLimit = 0.07; 
      if(v2Diff * v2Diff > dRefreshLimit * dRefreshLimit)
	bNeedToRefreshTemplate = true;
    }
  
  if(bNeedToRefreshTemplate)
    {
      int nOutside;  
      // choose the right template size
      if(left)
	nOutside = CVD::transform(p.pPatchSourceKF->aCamLeftLevels[p.nSourceLevel].im, 
				  mimTemplate, 
				  m2,
				  vec(p.irLeftFrameCenter),
				  vec(mirCenter)); 
      else
	nOutside = CVD::transform(p.pPatchSourceKF->aCamRightLevels[p.nSourceLevel].im, 
				  mimTemplate, 
				  m2,
				  vec(p.irRightFrameCenter),
				  vec(mirCenter)); 
      if(nOutside)
	mbTemplateBad = true;
      else
	mbTemplateBad = false;
      
      MakeTemplateSums();
      
      mpLastTemplateMapPoint = &p;
      mm2LastWarpMatrix = m2;
    }
};

bool PatchFinder::IsTemplateSimilar(const PatchFinder &PFinder){
  Matrix<2> m2 = M2Inverse(mm2WarpInverse) * LevelScale(mnSearchLevel);
  for(int i=0; i<2; i++)
    {
      Vector<2> v2Diff = m2.T()[i] - PFinder.mm2LastWarpMatrix.T()[i];
      const double dRefreshLimit = 0.07; 
      if(v2Diff * v2Diff > dRefreshLimit * dRefreshLimit){
	return false;
      }
    }
  return true;
}

void PatchFinder::CopyTemplateFrom(const PatchFinder &Finder){
  mimTemplate.copy_from(Finder.mimTemplate);
  mdMeanDiff = Finder.mdMeanDiff;
  mnTemplateSum = Finder.mnTemplateSum;
  mnTemplateSumSq = Finder.mnTemplateSumSq;
}


void PatchFinder::MakeTemplateCoarseNoWarp(Level Lev[], int nLevel, CVD::ImageRef irLevelPos)
{
  mnSearchLevel = nLevel;
  Image<byte> &im = Lev[nLevel].im;
  if(!im.in_image_with_border(irLevelPos, mnPatchSize / 2 + 1))
    {
      mbTemplateBad = true;
      return;
    }
  mbTemplateBad = false;
  copy(im,
       mimTemplate,
       mimTemplate.size(),
       irLevelPos - mirCenter);
  
  MakeTemplateSums();
}

// make a template for the finder for left or right frame
// depending on if left or right is set
void PatchFinder::MakeTemplateCoarseNoWarp(MapPoint &p, bool left) //left defaults to true
{
  if(left)
    MakeTemplateCoarseNoWarp(p.pPatchSourceKF->aCamLeftLevels, p.nSourceLevel,  p.irLeftFrameCenter);
  else
    MakeTemplateCoarseNoWarp(p.pPatchSourceKF->aCamRightLevels, p.nSourceLevel,  p.irRightFrameCenter);    
};

inline void PatchFinder::MakeTemplateSums()
{
  int nSum = 0;
  int nSumSq = 0;
  ImageRef ir;
  do
    {
      int b = mimTemplate[ir];
      nSum += b;
      nSumSq +=b * b;
    }      
  while(ir.next(mimTemplate.size()));
  mnTemplateSum = nSum;
  mnTemplateSumSq = nSumSq;
}

bool PatchFinder::FindPatchCoarse(ImageRef irPos, Level lV[], unsigned int nRange)
{
  mbFound = false;
  if(mnSearchLevel < 0 || mnSearchLevel > 3){
    cout << "search level wrong " << endl;
    return false;
  }
  int nLevelScale = LevelScale(mnSearchLevel);
  mirPredictedPos = irPos;
  irPos = irPos / nLevelScale;
  nRange = (nRange + nLevelScale - 1) / nLevelScale;
  
  int nTop = irPos.y - nRange;
  int nBottomPlusOne = irPos.y + nRange + 1;
  int nLeft = irPos.x - nRange;
  int nRight = irPos.x + nRange;
  
  Level &L = lV[mnSearchLevel];
  
  if(nTop < 0)
    nTop = 0;
  if(nTop >= L.im.size().y)
    return false;
  if(nBottomPlusOne <= 0)
    return false;
  
  vector<ImageRef>::iterator i;
  vector<ImageRef>::iterator i_end;
  
  i = L.vCorners.begin() + L.vCornerRowLUT[nTop];
  
  if(nBottomPlusOne >= L.im.size().y)
    i_end = L.vCorners.end();
  else 
    i_end = L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];
  
  ImageRef irBest; 
  int nBestSSD = mnMaxSSD + 1;
  int count = 0;
  for(; i<i_end; i++)         
    {                         
      if( i->x < nLeft || i->x > nRight)
        continue;
      if((irPos - *i).mag_squared() > nRange * nRange)
	continue;             

      int nSSD; 
      count++;
      nSSD = ZMSSDAtPoint(L.im, *i);
      if(nSSD < nBestSSD)      
	{
	  irBest = *i;
	  nBestSSD = nSSD;
	}
    } // done looping over corners
  if(nBestSSD < mnMaxSSD)      // Found a valid match?
    {
      mv2CoarsePos= LevelZeroPos(irBest, mnSearchLevel);
      mbFound = true;
    }
  else
    mbFound = false;
  return mbFound;
}

void PatchFinder::MakeSubPixTemplate()
{
  mimJacs.resize(mimTemplate.size() - ImageRef(2,2));
  Matrix<3> m3H = Zeros; 
  ImageRef ir;
  for(ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
    for(ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
      {
	Vector<2> v2Grad;
	v2Grad[0] = 0.5 * (mimTemplate[ir + ImageRef(1,0)] - mimTemplate[ir - ImageRef(1,0)]);
	v2Grad[1] = 0.5 * (mimTemplate[ir + ImageRef(0,1)] - mimTemplate[ir - ImageRef(0,1)]);
	mimJacs[ir-ImageRef(1,1)].first = v2Grad[0];
	mimJacs[ir-ImageRef(1,1)].second = v2Grad[1];
	Vector<3> v3Grad = unproject(v2Grad); 
	m3H += v3Grad.as_col() * v3Grad.as_row();
      }
  
  // Invert JTJ..
  Cholesky<3> chol(m3H);
  mm3HInv = chol.get_inverse();
  mv2SubPixPos = mv2CoarsePos; 
  mdMeanDiff = 0.0;
}


bool PatchFinder::IterateSubPixToConvergence(Level *level, int nMaxIts)
{
  const double dConvLimit = 0.17;
  bool bConverged = false;
  int nIts;
  double tmp = 0;
  for(nIts = 0; nIts < nMaxIts && !bConverged; nIts++)
    {
      double dUpdateSquared = IterateSubPix(level); 
      if(dUpdateSquared < 0) 
	return false;
      if(dUpdateSquared < dConvLimit*dConvLimit)
	return true;
      tmp = dUpdateSquared;
    }
  return false;
}

double PatchFinder::IterateSubPix(Level *levels)
{
  Vector<2> v2Center = LevelNPos(mv2SubPixPos, mnSearchLevel);
  BasicImage<byte> &im = levels[mnSearchLevel].im;
  if(!im.in_image_with_border(ir_rounded(v2Center), mnPatchSize / 2 + 1))
    return -1.0;
  

  Vector<2> v2Base = v2Center - vec(mirCenter);
  
  Vector<3> v3Accum = Zeros;
  
  ImageRef ir;
  
  byte* pTopLeftPixel;
  
  double dX = v2Base[0]-floor(v2Base[0]); // Distances from pixel center of TL pixel
  double dY = v2Base[1]-floor(v2Base[1]);
  float fMixTL = (1.0 - dX) * (1.0 - dY);
  float fMixTR = (dX)       * (1.0 - dY);
  float fMixBL = (1.0 - dX) * (dY);
  float fMixBR = (dX)       * (dY);
  
  unsigned long nRowOffset = levels[mnSearchLevel].im[ImageRef(0,1)] - levels[mnSearchLevel].im[ImageRef(0,0)];
  for(ir.y = 1; ir.y < mnPatchSize - 1; ir.y++)
    {
      pTopLeftPixel = &im[::ir(v2Base) + ImageRef(1,ir.y)]; 
      for(ir.x = 1; ir.x < mnPatchSize - 1; ir.x++)
	{
	  // Calc target interpolated pixel
	  float fPixel = fMixTL * pTopLeftPixel[0] + fMixTR 
	    * pTopLeftPixel[1] + fMixBL * 
	    pTopLeftPixel[nRowOffset] + fMixBR * 
	    pTopLeftPixel[nRowOffset + 1];
	  pTopLeftPixel++;
	  double dDiff = fPixel - mimTemplate[ir] + mdMeanDiff;
	  v3Accum[0] += dDiff * mimJacs[ir - ImageRef(1,1)].first;
	  v3Accum[1] += dDiff * mimJacs[ir - ImageRef(1,1)].second;
	  v3Accum[2] += dDiff;  // Update JT*d
	};
    }
  
  // All done looping over image - find JTJ^-1 * JTd:
  Vector<3> v3Update = mm3HInv * v3Accum; 
  mv2SubPixPos -= v3Update.slice<0,2>() * LevelScale(mnSearchLevel);
  mdMeanDiff -= v3Update[2];
  
  double dPixelUpdateSquared = v3Update.slice<0,2>() * v3Update.slice<0,2>();
  return dPixelUpdateSquared;
}

#if CVD_HAVE_XMMINTRIN

inline int SumXMM_16(__m128i &target)
{
  unsigned short int sums_store[8];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
    sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
}
inline int SumXMM_32(__m128i &target)
{
  unsigned int sums_store[4];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif

int PatchFinder::ZMSSDAtPoint(CVD::BasicImage<CVD::byte> &im, const CVD::ImageRef &ir)
{
  if(!im.in_image_with_border(ir, mirCenter[0]))
    return mnMaxSSD + 1;
  
  ImageRef irImgBase = ir - mirCenter;
  byte *imagepointer;
  byte *templatepointer;
  
  int nImageSumSq = 0;
  int nImageSum = 0;
  int nCrossSum = 0;

#if CVD_HAVE_XMMINTRIN
  if(mnPatchSize == 8)
    {
      long unsigned int imagepointerincrement;

      __m128i xImageAsEightBytes;
      __m128i xImageAsWords;
      __m128i xTemplateAsEightBytes;
      __m128i xTemplateAsWords;
      __m128i xZero;
      __m128i xImageSums; // These sums are 8xuint16
      __m128i xImageSqSums; // These sums are 4xint32
      __m128i xCrossSums;   // These sums are 4xint32
      __m128i xProduct;

      
      xImageSums = _mm_setzero_si128();
      xImageSqSums = _mm_setzero_si128();
      xCrossSums = _mm_setzero_si128();
      xZero = _mm_setzero_si128();
      
      imagepointer = &im[irImgBase + ImageRef(0,0)];
      templatepointer = &mimTemplate[ImageRef(0,0)];
      imagepointerincrement = &im[irImgBase + ImageRef(0,1)] - imagepointer;
      
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      imagepointer += imagepointerincrement;
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsEightBytes=_mm_load_si128((__m128i*) templatepointer);
      templatepointer += 16;
      xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
      xImageAsEightBytes=_mm_loadl_epi64((__m128i*) imagepointer);
      xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes,xZero);
      xImageSums = _mm_adds_epu16(xImageAsWords,xImageSums);
      xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
      xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
      xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes,xZero);
      xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
      xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

      nImageSum = SumXMM_16(xImageSums);
      nCrossSum = SumXMM_32(xCrossSums);
      nImageSumSq = SumXMM_32(xImageSqSums);
    }
  else
#endif 
    {    
      for(int nRow = 0; nRow < mnPatchSize; nRow++)
	{
	  imagepointer = &im[irImgBase + ImageRef(0,nRow)];
	  templatepointer = &mimTemplate[ImageRef(0,nRow)];
	  for(int nCol = 0; nCol < mnPatchSize; nCol++)
	    {
	      int n = imagepointer[nCol];
	      nImageSum += n;
	      nImageSumSq += n*n;
	      nCrossSum += n * templatepointer[nCol];
	    };
	}
    };
  
  int SA = mnTemplateSum;
  int SB = nImageSum;
  
  int N = mnPatchSize * mnPatchSize;
  return ((2*SA*SB - SA*SA - SB*SB)/N + nImageSumSq + mnTemplateSumSq - 2*nCrossSum);
}






