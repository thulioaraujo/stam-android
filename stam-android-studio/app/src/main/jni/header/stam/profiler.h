#ifndef _PROFILER_
#define _PROFILER_

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <numeric>
#include <vector>
#include <opencv2\opencv.hpp>

#include "header\stam\Defines.h"

class Profiler
{
private:
	double delta;
	double totalTime;
	int counter;
	std::vector<uint64> samples;
	std::ofstream outfile;

public:

	Profiler(){
#ifdef WINDOWS_VS
		//if (!outfile.is_open()) {
		//	outfile.open("stam-test.txt", std::ofstream::out | std::ofstream::ate | std::ofstream::app);
		//}
#endif
		delta = (double)cv::getTickCount();
		totalTime = 0;
		counter = 0;
	}

	void startSampling(){
		delta = (double)cv::getTickCount();
	}

	void endSampling(){
		delta = (double)(cv::getTickCount() - delta);
		totalTime = totalTime + delta;
		counter++;
		samples.push_back(delta);
	}

	void print(const std::string& text, int maxSamples)
	{
		if (counter == maxSamples || maxSamples == -1){

			double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
			double mean = sum / samples.size();
			double sqSum = std::inner_product(samples.begin(), samples.end(), samples.begin(), 0.0);
			double stdDev = std::sqrt(sqSum / samples.size() - mean*mean);

			std::stringstream stream;
			stream
				<< text
				<< "\t"
				<< std::setprecision(3)
				<< "mean: "
				<< (mean * 1000.0) / cv::getTickFrequency()
				<< " ms"
				<< " (FPS: "
				<< 1000.0 / ((mean * 1000.0) / cv::getTickFrequency())
				<< ") "
				<< "SD: "
				<< (stdDev * 1000.0) / cv::getTickFrequency()
				<< " ms "
				<< "["
				<< counter
				<< " samples]";

#ifdef ANDROID
			LOGI("%s", stream.str().c_str());
#else
			
			std::cout << stream.str() << std::endl;
			//outfile.close();
#endif
		}
	}
};

#endif // _PROFILER_
