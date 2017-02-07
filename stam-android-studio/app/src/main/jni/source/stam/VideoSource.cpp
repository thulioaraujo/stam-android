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

// Project includes
#include "header/stam/VideoSource.h"

// Default contructor
VideoSource::VideoSource(){}

// RGB constructor
VideoSource::VideoSource(int device) {
	capture_.open(device);
	if (!capture_.isOpened()) {
		std::cout << "Error: Cannot open video capture from device " << device << std::endl;
		exit(1);
	}
}

// Image sequence constructor
VideoSource::VideoSource(std::string &filename) {
	capture_.open(filename);
	if (!capture_.isOpened()) {
		std::cout << "error: can't open video capture from file " << filename << std::endl;
		exit(1);
	}
}

//// RGB-D constructor
//VideoSource::VideoSource(int width, int height, int fps) : width_(width), height_(height), fps_(fps) {
//	openni::OpenNI::initialize();
//	device_.open(openni::ANY_DEVICE);
//
//	depthStream_.create(device_, openni::SENSOR_DEPTH);
//
//	openni::VideoMode vgaDepthVideoMode;
//	vgaDepthVideoMode.setResolution(width, height);
//	vgaDepthVideoMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
//	vgaDepthVideoMode.setFps(fps);
//
//	depthStream_.setVideoMode(vgaDepthVideoMode);
//	depthStream_.start();
//
//	colorStream_.create(device_, openni::SENSOR_COLOR);
//
//	openni::VideoMode vgaColorVideoMode;
//	vgaColorVideoMode.setResolution(width, height);
//	vgaColorVideoMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
//	vgaColorVideoMode.setFps(fps);
//
//	colorStream_.setVideoMode(vgaColorVideoMode);
//	colorStream_.start();
//
//	device_.setImageRegistrationMode(openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
//}

// Frame grabber when using RGB camera
cv::Mat VideoSource::readNextFrame() {
	cv::Mat image;
	capture_ >> image;
	return image;
}

// Frame grabber when using image sequence
cv::Mat VideoSource::readNextFrame(const std::string& NEXT_FRAME_FMT) {
	static int findex = 0;
	cv::Mat image;
	char buf[256];
	sprintf(buf, NEXT_FRAME_FMT.c_str(), findex++);
	image = cv::imread(buf);
	
	return image;
}

//// Frame grabber when using RGB-D camera
//void VideoSource::readNextFrame(cv::Mat &rgbImage, cv::Mat &depthImage) {
//	int changedIndex;
//	openni::VideoStream* streams[1];
//	openni::Status rc;
//
//	streams[0] = &colorStream_;
//
//	rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedIndex);
//
//	colorStream_.readFrame(&colorFrame_);
//	memcpy(rgbImage.data, colorFrame_.getData(), width_ * height_ * 3);
//	cv::cvtColor(rgbImage, rgbImage, CV_BGR2RGB);
//
//	streams[0] = &depthStream_;
//
//	rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedIndex);
//
//	depthStream_.readFrame(&depthFrame_);
//	memcpy(depthImage.data, depthFrame_.getData(), width_* height_ * 2);
//
//	cv::flip(rgbImage, rgbImage, 1);
//	cv::flip(depthImage, depthImage, 1);
//}