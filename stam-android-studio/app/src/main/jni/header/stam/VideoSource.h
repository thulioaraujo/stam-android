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

#ifndef _VIDEO_SOURCE_
#define _VIDEO_SOURCE_

// C++ includes
#include <string>
#include <iostream>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// OpenNI includes
//#include <OpenNI.h>

class VideoSource {

public:
	//int width_;		// Image width (only used for RGB-D)
	//int height_;	// Image height (only used for RGB-D)
	//int fps_;		// Depth sensor fps (only used for RGB-D)

	// Default contructor
	VideoSource();

	// RGB constructor
	VideoSource(int device);

	// Image sequence constructor
	VideoSource(std::string &filename);

	// RGB-D constructor
	//VideoSource(int width, int height, int fps);


	// Frame grabber when using RGB camera
	cv::Mat readNextFrame();
	
	// Frame grabber when using image sequence
	cv::Mat readNextFrame(const std::string& NEXT_FRAME_FMT);

	// Frame grabber when using RGB-D camera
	//void readNextFrame(cv::Mat &rgbImage, cv::Mat &depthImage);

private:
	cv::VideoCapture capture_;			// OpenCV camera
	
	//openni::Device device_;				// OpenNI camera (only used for RGB-D)
	//openni::VideoStream depthStream_;	// OpenNI depth stream (only used for RGB-D)
	//openni::VideoStream colorStream_;	// OpenNI color stream (only used for RGB-D)
	//openni::VideoFrameRef depthFrame_;	// OpenNI depth image (only used for RGB-D)
	//openni::VideoFrameRef colorFrame_;	// OpenNI color image (only used for RGB-D)
};

#endif // _VIDEO_SOURCE_