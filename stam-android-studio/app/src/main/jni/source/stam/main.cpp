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

// C++ includes
#include <fstream>
#include <string>

// Project includes
#include "header/stam/ConversionUtil.h"
#include "header/stam/VideoSource.h"
#include "header/stam/config.h"
#include "header/stam/STAM.h"

// Namespace definition
namespace vo = visual_odometry;

int main(int argc, char** argv) {

	/* Use two forward slash to choose scene number or one to bypass it ans use only the camera
	if (argc < 2) {
		printf("Usage: ./stam <scene_number>\n"
			"where <scene_number> = 1|2|3|4|5\n\n");
		exit(1);
	}
	else {
		SCENE = atoi(argv[1]);

		if (SCENE > 5 || SCENE < 1) {
			printf("Usage: ./stam <scene_number>\n"
				"where <scene_number> = 1|2|3|4|5 (4 is RGB OnSite, 5 is RGBD OnSite)\n\n");
			exit(1);
		}
	}
	/*/
	//* Use two forward slash to choose RGB or one to use only RGB-D
	SCENE = 4; /*/ SCENE = 5; //*/
	//*/

	VideoSource video_source;
	if(SCENE == 4) {
		video_source = VideoSource(0);
	}
	//else if (SCENE == 5) {
	//	video_source = VideoSource(640, 480, 30);
	//}
		
	while (true)
	{
		int i = 0;
		cv::Mat frame;
		vo::STAM STAM;
		std::ofstream traj_out;
		std::string next_frame;
		std::stringstream traj_name;
		visual_odometry::Frame::Ptr current_frame;

		STAM.init("../../assets/");

		// Initializa the scene
		if (SCENE < 4) {
			traj_name << "../resource/trajectory_scene" << argv[1] << ".txt";
			//traj_out = std::ofstream(traj_name.str());

			std::string next_frame_format[] = { "../resource/S01L03_VGA/S01L03_VGA_%04d.png",
				"../resource/S02L03_VGA/S02L03_VGA_%04d.png",
				"../resource/S03L03_VGA/S03L03_VGA_%04d.png" };

			next_frame = next_frame_format[SCENE - 1];

			STAM.initFromFiles(video_source.readNextFrame(next_frame),
				STAM.params.POINTS_2D_INIT_FILE,
				STAM.params.POINTS_3D_INIT_FILE);
		}
		else if (SCENE == 4) {
			bool calibrated = false;
			while (!calibrated) {

				// Get the input BGR frame
				cv::Mat originalInput = video_source.readNextFrame();

				// Create the YUV frame
				cv::Size inputSize = originalInput.size();
				inputSize.height += inputSize.height / 2;
				cv::Mat inputYUV420sp = cv::Mat::zeros(inputSize, CV_8UC1);

				// Create the Y and UV frame
				cv::Mat inputY = cv::Mat(inputYUV420sp.rows * 2 / 3, inputYUV420sp.cols, CV_8UC1);
				inputY.data = inputYUV420sp.data;
				cv::Mat inputUV = cv::Mat(inputY.rows / 2, inputY.cols / 2, CV_8UC2);
				inputUV.data = &inputYUV420sp.data[inputY.rows * inputY.cols];

				// Convert BGR image to YUV
				parallel_for_(cv::Range(0, originalInput.rows - 1), ConversionUtil::Parallel_bgr2yuv(originalInput, inputY, inputUV, false));

				// Calibrate the STAM

				Profiler profilerInitFromChessboard;
				profilerInitFromChessboard.startSampling();
				calibrated = STAM.initFromChessboard(inputY, inputUV, cv::Size(9, 6), 100);
				profilerInitFromChessboard.endSampling();
				profilerInitFromChessboard.print("initFromChessboard", -1);

				//// Convert YUV image to BGR
				//cv::Mat output = cv::Mat::zeros(originalInput.size(), originalInput.type());
				//parallel_for_(cv::Range(0, output.rows - 1), ConversionUtil::Parallel_yuv2bgr(output, inputY, inputUV, false));
				//cv::imshow("YUV Render", output);
				//cv::waitKey(1);
			}
		}
		//else if (SCENE == 5) {
		//	frame.create(video_source.height_, video_source.width_, CV_8UC3);
		//	cv::Mat depthFrame(video_source.height_, video_source.width_, CV_16U);
		//	video_source.readNextFrame(frame, depthFrame);
		//	while (!STAM.initFromChessboard(frame, depthFrame, cv::Size(9, 6), 100))
		//	{
		//		video_source.readNextFrame(frame, depthFrame);
		//	}
		//}

		bool imageIsOver = false;
		while (!imageIsOver) {

			if (SCENE < 4) {
				frame = video_source.readNextFrame(next_frame);
			}
			else if (SCENE == 4) {
				frame = video_source.readNextFrame();
			}
			//else if (SCENE == 5) {
			//	video_source.readNextFrame(frame, depthFrame);
			//}

			if (!frame.empty()) {

				// Create the YUV frame
				cv::Size inputSize = frame.size();
				inputSize.height += inputSize.height / 2;
				cv::Mat inputYUV420sp = cv::Mat::zeros(inputSize, CV_8UC1);

				// Create the Y and UV frame
				cv::Mat inputY = cv::Mat(inputYUV420sp.rows * 2 / 3, inputYUV420sp.cols, CV_8UC1);
				inputY.data = inputYUV420sp.data;
				cv::Mat inputUV = cv::Mat(inputY.rows / 2, inputY.cols / 2, CV_8UC2);
				inputUV.data = &inputYUV420sp.data[inputY.rows * inputY.cols];

				// Convert BGR image to YUV
				parallel_for_(cv::Range(0, frame.rows - 1), ConversionUtil::Parallel_bgr2yuv(frame, inputY, inputUV, false));

				if (SCENE < 5) {

					static Profiler profilerProcess;
					profilerProcess.startSampling();
					current_frame = STAM.process(inputY, inputUV);
					profilerProcess.endSampling();
					profilerProcess.print("process", 350);
				}
				//else {
				//	current_frame = STAM.process(frame, depthFrame);
				//}

#ifdef WINDOWS_VS
				if (SCENE > 1 && i % 100 == 0) {
					static Profiler profilerOptimise;
					profilerOptimise.startSampling();
					STAM.optimise();
					profilerOptimise.endSampling();
					profilerOptimise.print("optimise", -1);
				}
#endif

				i++;
				cv::Mat p;

				cv::Mat pM = STAM.intrinsics_*current_frame->projMatrix;//.mul(1.0/274759.971);

				for (int j = 0; j < 3; j++)
					traj_out << pM.at<double>(j, 0) << "," << pM.at<double>(j, 1) << "," << pM.at<double>(j, 2) << "," << pM.at<double>(j, 3) << std::endl;

				if (vo::STAM::reinitialize)
				{
					vo::STAM::calibrate = false;
					vo::STAM::reinitialize = false;
					vo::Memory::next_p3d_id_s_ = 0;
					vo::Memory::next_p2d_id_s_ = 0;
					break;
				}
			}
			else {
				imageIsOver = true;
			}
		}
	}
	return 0;
}
