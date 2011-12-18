#ifndef __CONSTANTS__H
#define __CONSTANTS__H

//the maximum disparity that will be searched - avoid searching the whole image across
#define MAX_DISPARITY 100
//number of internal params in camera calibrator - can't see why this would change but whatever...
#define NUMTRACKERCAMPARAMETERS 5
//input video source width
#define OPENCV_VIDEO_W 720
//input video source height
#define OPENCV_VIDEO_H 576
//number of squares in the calibration grid across
#define X_GRID 9
//number of squares in the calibration grid down
#define Y_GRID 7
//size of grid squares in m 
#define GRID_SQUARE_SIZE 0.008
//number of levels in keyframe
#define LEVELS 4
//minimum scene depth for epipolar search
#define MIN_SCENE_DEPTH 10
//maximum scene depth for epipolar search
#define MAX_SCENE_DEPTH 400
//scene depth to baseline ratio for choosing epipolar search across stereo pair or closest keyframes
#define BASELINE_SCENE_DEPTH 100
	

#endif
