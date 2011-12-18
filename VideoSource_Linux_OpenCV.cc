/*
* Autor : Arnaud GROSJEAN (VIDE SARL)
* This implementation of VideoSource allows to use OpenCV as a source for the video input
* I did so because libCVD failed getting my V4L2 device
*
* INSTALLATION :
* - Copy the VideoSource_Linux_OpenCV.cc file in your PTAM directory
* - In the Makefile:
*	- set the linkflags to
	LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lblas -llapack -lGVars3 -lcvd -lcv -lcxcore -lhighgui
*	- set the videosource to 
	VIDEOSOURCE = VideoSource_Linux_OpenCV.o
* - Compile the project
* - Enjoy !
* 
* Notice this code define two constants for the image width and height (OPENCV_VIDEO_W and OPENCV_VIDEO_H)
*/

#include "VideoSource.h"
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>


using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace cv;

VideoSource::VideoSource()
{
    cout << "  VideoSource_Linux: Opening video source..." << endl;
    mptr1 = new VideoCapture("pelvis.avi");    
    //mptr1 = new VideoCapture("calibration_video.avi");
    VideoCapture* cap1 = (VideoCapture*) mptr1;
    if (!cap1->isOpened()) {
        cerr << "Unable to get the camera" << endl;
        exit(-1);
    }
    mirSize = ImageRef(OPENCV_VIDEO_W, OPENCV_VIDEO_H);
    cv_Size = cvSize(OPENCV_VIDEO_W,OPENCV_VIDEO_H);
    cout << "  ... got video source." << endl;    
    
};


ImageRef VideoSource::Size()
{ 
  return mirSize;
};

CvSize VideoSource::getCVSize(){
  return cv_Size;
}

//version for converting single image into Image<byte>
void conversionNB(Mat frame, Image<byte> &imBW){
  Mat clone = frame.clone();
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++){
    for (int j = 0; j < OPENCV_VIDEO_W; j++){	
      imBW[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
    }
  }
}

void conversionRGB(Mat frame, Image<Rgb<byte> > &imRGB){
	Mat clone = frame.clone();
	Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
	for (int i = 0; i < OPENCV_VIDEO_H; i++){
		for (int j = 0; j < OPENCV_VIDEO_W; j++){	
		imRGB[i][j].red = frame_p(i,j)[2];
		imRGB[i][j].green = frame_p(i,j)[1];
		imRGB[i][j].blue = frame_p(i,j)[0];
		}
	}
}

//version for converting large stereo image into two Image<byte>
void conversionNB(Mat frame, Image<byte> &imL_BW, Image<byte> &imR_BW){
  Mat clone = frame.clone(); 
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++){
    for (int j = 0; j < OPENCV_VIDEO_W; j++){	
      imL_BW[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
    }
                for (int a = OPENCV_VIDEO_W; a<(OPENCV_VIDEO_W*2);a++){
		  imR_BW[i][a-OPENCV_VIDEO_W] = (frame_p(i,a)[0] + frame_p(i,a)[1] + frame_p(i,a)[2]) / 3;
		}
	}

}

//version for converting large stereo image into two Image<byte>
void conversionRGB(Mat frame, Image<Rgb<byte> > &imL_RGB, Image<Rgb<byte> > &imR_RGB){
  Mat clone = frame.clone();
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++){
    for (int j = 0; j < OPENCV_VIDEO_W; j++){	
      imL_RGB[i][j].red = frame_p(i,j)[2];
      imL_RGB[i][j].green = frame_p(i,j)[1];
      imL_RGB[i][j].blue = frame_p(i,j)[0];
    }
    for (int a = OPENCV_VIDEO_W; a < (OPENCV_VIDEO_W*2); a++){
      imR_RGB[i][a-OPENCV_VIDEO_W].red = frame_p(i,a)[2];
      imR_RGB[i][a-OPENCV_VIDEO_W].green = frame_p(i,a)[1];
      imR_RGB[i][a-OPENCV_VIDEO_W].blue = frame_p(i,a)[0];
    }
  }
}


//void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imL_BW, Image<byte> &imR_BW, Image<Rgb<byte> > &imL_RGB,Image<Rgb<byte> > &imR_RGB){
  /*  Mat frame;
  VideoCapture* cap = (VideoCapture*)mptr;
  *cap >> frame;
  conversionNB(frame, imL_BW, imR_BW);
  conversionRGB(frame, imL_RGB, imR_RGB);
  */// }

void preprocessFrames2(Image<byte> &frameL, Image<byte> &frameR){
  cout << "#### new frame ####" << endl;
  //Mat_<Vec3b>& frameL = (Mat_<Vec3b>&)leftFrame;
  //Mat_<Vec3b>& frameR = (Mat_<Vec3b>&)rightFrame;
  double meanLeft = 0;
  double meanRight = 0;
  double varianceLeft = 0;
  double varianceRight = 0;
  double sumLeft = 0;
  double sumRight = 0;
  double sumSquareLeft = 0;
  double sumSquareRight = 0;
  for(int i=0;i<OPENCV_VIDEO_H;i++)
    {
      for(int j=0;j<OPENCV_VIDEO_W;j++)
	{
	  sumLeft += frameL[i][j];
	  sumRight += frameR[i][j];
	  sumSquareLeft += frameL[i][j]*frameL[i][j];
	  sumSquareRight += frameR[i][j]*frameR[i][j];
	}
    }
  meanLeft = sumLeft / (OPENCV_VIDEO_W*OPENCV_VIDEO_H);
  meanRight = sumRight / (OPENCV_VIDEO_H*OPENCV_VIDEO_W);
  double meanFactor = meanLeft/meanRight;
  varianceLeft = (sumSquareLeft-(sumLeft*meanLeft))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  varianceRight = (sumSquareRight-(sumRight*meanRight))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  float SDLeft = sqrt(varianceLeft);
  float SDRight = sqrt(varianceRight);
  //cout << "sd left is " << SDLeft << " and sd right is " << SDRight << endl;
  //float varianceFactor = varianceLeft/varianceRight;
  if(meanLeft < meanRight){
  //cout << "called 1 " << endl;
    //    varianceRight = varianceLeft;
    for(int i=0;i<OPENCV_VIDEO_H;i++)
      for(int j=0;j<OPENCV_VIDEO_W;j++){
	frameR[i][j] = ((meanLeft/meanRight)*(frameR[i][j]-SDRight))+SDLeft;
	//	if(frameR[i][j] 
	
      }
  }
  else{
    for(int i=0;i<OPENCV_VIDEO_H;i++)
      for(int j=0;j<OPENCV_VIDEO_W;j++){
	frameL[i][j] = ((meanRight/meanLeft)*(frameL[i][j]-SDLeft))+SDRight; 
      }
  }
  
  sumLeft = sumRight = sumSquareLeft = sumSquareRight = 0.0;
  for(int i=0;i<OPENCV_VIDEO_H;i++)
    {
      for(int j=0;j<OPENCV_VIDEO_W;j++)
	{
	  sumLeft += frameL[i][j];
	  sumRight += frameR[i][j];
	  sumSquareLeft += frameL[i][j]*frameL[i][j];
	  sumSquareRight += frameR[i][j]*frameR[i][j];
	}
    }
  meanLeft = sumLeft / (OPENCV_VIDEO_W*OPENCV_VIDEO_H);
  meanRight = sumRight / (OPENCV_VIDEO_W*OPENCV_VIDEO_H);
  varianceLeft = (sumSquareLeft-(sumLeft*meanLeft))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  varianceRight = (sumSquareRight-(sumRight*meanRight))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  cout << "mean left = " << meanLeft << endl 
       << "mean right = " << meanRight << endl 
       << "sd left = " << sqrt(varianceLeft) << endl
       << "sd right = " << sqrt(varianceRight) << endl;
}


void preprocessFrames(cv::Mat &leftFrame, cv::Mat &rightFrame){
  //cout << "#### new frame ####" << endl;
  Mat_<Vec3b>& frameL = (Mat_<Vec3b>&)leftFrame;
  Mat_<Vec3b>& frameR = (Mat_<Vec3b>&)rightFrame;
  double meanLeft = 0;
  double meanRight = 0;
  double varianceLeft = 0;
  double varianceRight = 0;
  double sumLeft = 0;
  double sumRight = 0;
  double sumSquareLeft = 0;
  double sumSquareRight = 0;
  for(int i=0;i<OPENCV_VIDEO_H;i++)
    {
      for(int j=0;j<OPENCV_VIDEO_W;j++)
	{
	  sumLeft += (frameL(i,j)[0] + frameL(i,j)[1] + frameL(i,j)[2])/3;
	  sumRight += (frameR(i,j)[0] + frameR(i,j)[1] + frameR(i,j)[2])/3;
	  sumSquareLeft += pow(((frameL(i,j)[0]+frameL(i,j)[1]+frameL(i,j)[2])/3),2);
	  sumSquareRight += pow(((frameR(i,j)[0]+frameR(i,j)[1]+frameR(i,j)[2])/3),2);
	}
    }
  meanLeft = sumLeft / (OPENCV_VIDEO_W*OPENCV_VIDEO_H);
  meanRight = sumRight / (OPENCV_VIDEO_H*OPENCV_VIDEO_W);
  double meanFactor = meanLeft/meanRight;
  varianceLeft = (sumSquareLeft-(sumLeft*meanLeft))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  varianceRight = (sumSquareRight-(sumRight*meanRight))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  float SDLeft = sqrt(varianceLeft);
  float SDRight = sqrt(varianceRight);
  cout << "sd left is " << SDLeft << " and sd right is " << SDRight << endl;
  //float varianceFactor = varianceLeft/varianceRight;
  if(SDLeft < SDRight){
    //    varianceRight = varianceLeft;
    for(int i=0;i<OPENCV_VIDEO_H;i++)
      for(int j=0;j<OPENCV_VIDEO_W;j++){
	frameR(i,j)[0] = (meanLeft/meanRight)*(frameR(i,j)[0] - SDRight) + SDLeft;
	frameR(i,j)[1] = (meanLeft/meanRight)*(frameR(i,j)[1] - SDRight) + SDLeft;
	frameR(i,j)[2] = (meanLeft/meanRight)*(frameR(i,j)[2] - SDRight) + SDLeft;
      }
  }else{
    // varianceFactor = 1.0/varianceFactor;
    //varianceLeft = varianceRight;
    for(int i=0;i<OPENCV_VIDEO_H;i++)
      for(int j=0;j<OPENCV_VIDEO_W;j++){
	frameL(i,j)[0] = (meanRight/meanLeft)*(frameL(i,j)[0]-SDLeft) + SDRight; 
	frameL(i,j)[1] = (meanRight/meanLeft)*(frameL(i,j)[1]-SDLeft) + SDRight;
	frameL(i,j)[2] = (meanRight/meanLeft)*(frameL(i,j)[2]-SDLeft) + SDRight; 
      }
  }
  sumLeft = sumRight = sumSquareLeft = sumSquareRight = 0.0;
  for(int i=0;i<OPENCV_VIDEO_H;i++)
    {
      for(int j=0;j<OPENCV_VIDEO_W;j++)
	{
	  sumLeft += (frameL(i,j)[0] + frameL(i,j)[1] + frameL(i,j)[2])/3;
	  sumRight += (frameR(i,j)[0] + frameR(i,j)[1] + frameR(i,j)[2])/3;
	  sumSquareLeft += ((frameL(i,j)[0]+frameL(i,j)[1]+frameL(i,j)[2])/3)*((frameL(i,j)[0]+frameL(i,j)[1]+frameL(i,j)[2])/3);
	  sumSquareRight += ((frameR(i,j)[0]+frameR(i,j)[1]+frameR(i,j)[2])/3)*((frameR(i,j)[0]+frameR(i,j)[1]+frameR(i,j)[2])/3);
	 }
    }
  meanLeft = sumLeft / (OPENCV_VIDEO_W*OPENCV_VIDEO_H);
  meanRight = sumRight / (OPENCV_VIDEO_W*OPENCV_VIDEO_H);
  varianceLeft = (sumSquareLeft-(sumLeft*meanLeft))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  varianceRight = (sumSquareRight-(sumRight*meanLeft))/((OPENCV_VIDEO_W*OPENCV_VIDEO_H)-1);
  cout << "mean left = " << meanLeft << endl 
       << "mean right = " << meanRight << endl 
       << "variance left = " << varianceLeft << endl
       << "variance right = " << varianceRight << endl;
}

void VideoSource::GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imL_BW,CVD::Image<CVD::byte> &imR_BW,CVD::Image<CVD::Rgb<CVD::byte> > &imL_RGB,CVD::Image<CVD::Rgb<CVD::byte> > &imR_RGB/*,const cv::Mat &leftX, const cv::Mat &leftY, const cv::Mat &rightX, const cv::Mat &rightY*/){
  Mat frame;
  VideoCapture* cap1 = (VideoCapture*)mptr1;
  *cap1 >> frame;
  CvRect l = cvRect(0,0,OPENCV_VIDEO_W,OPENCV_VIDEO_H);
  CvRect r = cvRect(OPENCV_VIDEO_W-1,0,OPENCV_VIDEO_W,OPENCV_VIDEO_H);
  
  Mat frame_left(l.height,l.width,CV_8UC3);
  Mat frame_right(l.height,l.width,CV_8UC3);
 
  frame_left = frame(l);
  frame_right = frame(r);
  //Mat rectified_left, rectified_right;
  //VideoCapture *cap1 = (VideoCapture*)mptr1;
  //VideoCapture *cap2 = (VideoCapture*)mptr2;
  
  //*cap1 >> rectified_left;
  //*cap2 >> rectified_right;


  
  //preprocessFrames(frame_left,frame_right);
  //Mat rectified_left(l.height,l.width,CV_8UC3);
  //Mat rectified_right(l.height,l.width,CV_8UC3);
  //remap(frame_left,rectified_left,leftX,leftY, CV_INTER_LINEAR);
  //remap(frame_right,rectified_right,rightX,rightY, CV_INTER_LINEAR);
  conversionNB(frame_left, imL_BW);
  //cout << "here" << endl;
  conversionNB(frame_right, imR_BW);
  //preprocessFrames2(imL_BW,imR_BW);
  conversionRGB(frame_left, imL_RGB);
  //cout << "here" << endl;
  conversionRGB(frame_right, imR_RGB);
  //cout << "here" << endl;
}
