/*
* Voxar Labs - CIn-UFPE
* Copyright(c) 2016 by LG Voxar Labs
*
* All rights reserved. No part of this work may be reproduced, stored in a retrieval
* system, or transmitted by any means without prior written Permission of Voxar Labs.
*
* Description:
*
* @author Ermano Arruda <eaa3@cin.ufpe.br>,
*         Francisco Simões <fpms@cin.ufpe.br>,
*         João Paulo Lima <jpsml@cin.ufpe.br>,
*         João Marcelo Teixeira <jmxnt@cin.ufpe.br>,
*         Lucas Figueiredo <lsf@cin.ufpe.br>,
*         Mozart Almeida <mwsa@cin.ufpe.br>,
*         Rafael Roberto <rar3@cin.ufpe.br>,
*         Thulio Araujo <tsla@cin.ufpe.br>,
*         Veronica Teichrieb <vt@cin.ufpe.br>.
*
* @since creation date(2016-02-19)
*/

#ifndef _VOODOOMETRY_
#define _VOODOOMETRY_

// C++ includes
#include <map>
#include <list>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <iostream>
#include <algorithm>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// Project includes
#include "header/stam/types.h"
#include "header/stam/utils.h"
#include "header/stam/config.h"
#include "header/stam/Defines.h"
#include "header/stam/profiler.h"
#include "header/stam/YUVDrawing.h"

extern int SCENE;

namespace visual_odometry {

class STAM {

public:

	class Params {
	public:
		double baseline_thr;
		std::string POINTS_2D_INIT_FILE;
		std::string POINTS_3D_INIT_FILE;
		std::string INTRINSICS_FILE;
		std::string NEXT_FRAME_FMT;
	};

	// Public Variables
	Params params;

	cv::Mat intrinsics_;
	
	cv::Point3d refPoint;
	
	std::list<Frame::Ptr> key_frames_;
	
	std::vector<int> tracked_keyframe_pts_ids_;
	std::vector<cv::Point2f> tracked_keyframe_pts_;

	bool competitonMode = false;

	static int currentChallengePoint;
	
	static std::vector<int> challengePointsIds;
	static std::vector<cv::Point3d> challengePoints;

	static bool calibrate;
	static bool reinitialize;

	cv::Rect leftButton;
	cv::Rect rightButton;

	// Public Functions
	void init(std::string resourcesPath);

	void loadIntrinsicsFromFile(const std::string& filename);
	
	void initFromFiles(cv::Mat& image, const std::string& p2D_filename, const std::string& p3D_filename);

	bool initFromChessboard(cv::Mat& yImage, cv::Mat& uvImage, const cv::Size& chessBoardSize, int squareSize);
	//bool initFromChessboard(const cv::Mat& colorImage, const cv::Mat& depthImage, const cv::Size& chessBoardSize, int squareSize);

	void loadChallengePoints(const std::string& file);

	Frame::Ptr process(cv::Mat& yImage, cv::Mat& uvImage);
	//Frame::Ptr process(const cv::Mat &image, const cv::Mat &depth);

	void dump();

	void optimise();

	bool isCalibrated() { return trackset_.points2D_.size() > 0; }

private:
	// Private Variables
	Memory memory_;
	
	TrackSet trackset_;
	
	cv::Mat distortion_;
	
	int keyframeSize = 3;
	
	Frame::Ptr previous_frame_;

	visual_odometry::utils::GenericMatcher matcher;

	// Private Functions
	cv::Mat calcProjMatrix(bool use_guess, cv::Mat& guess_r, cv::Mat& guess_t);
	
	bool hasEnoughBaseline(const cv::Mat& pose1, const cv::Mat& pose2, double thr_baseline);
	
	void updateUsingKLT(cv::Mat image);

	void updateUsingDetection(Frame::Ptr& key_frame, Frame::Ptr& current_frame);

	void mapping(Frame::Ptr& key_frame, Frame::Ptr& current_frame);

	bool matchAndTriangulate(Frame::Ptr& key_frame, Frame::Ptr& current_frame, const cv::Mat& intrinsics, const cv::Mat& distortion);

	bool matchAndTriangulateKlt(Frame::Ptr& key_frame, Frame::Ptr& current_frame, const cv::Mat& intrinsics, const cv::Mat& distortion);
	
	bool unprojectAndAlign(Frame::Ptr& key_frame, Frame::Ptr& current_frame, cv::Mat& intrinsics);

	void projectAndDrawTargetPoint(const cv::Mat& projMatrix, cv::Mat& image);

	void drawCompass(cv::Mat& image, const cv::Point& targetPoint);

	void updateOptimisedKF();
		
	cv::Rect getButtonRect(const std::string& text, cv::Point& point, bool toTheLeft);

	static void onMouse(int event, int x, int y, int, void*);

	void clearKeyframes();
};

}

#endif // _VOODOOMETRY_
