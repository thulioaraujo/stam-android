#pragma once

#include <opencv2\opencv.hpp>

#define CV_BGR -1

namespace ConversionUtil
{
	// http://stackoverflow.com/questions/10566668/lossless-rgb-to-ycbcr-transformation
	//inline static void forwardLift(uchar x, uchar y, uchar& average, uchar& diff)
	//{
	//	diff = (y - x) & 0xFF;
	//	average = (x + (diff >> 1)) & 0xFF;
	//}
	//inline static void reverseLift(uchar average, uchar diff, uchar& x, uchar& y)
	//{
	//	x = (average - (diff >> 1)) & 0xFF;
	//	y = (x + diff) & 0xFF;
	//}

	inline static void rgb2yuv(uchar r, uchar g, uchar b, uchar& y, uchar& u, uchar& v)
	{
		// http://stackoverflow.com/questions/10566668/lossless-rgb-to-ycbcr-transformation
		//uchar temp;
		//forwardLift(r, b, temp, u);
		//forwardLift(g, temp, y, v);
		//u += 128;
		//v += 128;

		// http://research.microsoft.com/pubs/102040/2008_colortransforms_malvarsullivansrinivasan.pdf
		//y = (r + (g << 1) + b) >> 2;
		//u = std::max(0, r - g);
		//v = std::max(0, b - g);

		// standard jpeg conversion
		//y = std::min(255.0, std::max(0.0, 0.299 * r + 0.587 * g + 0.114 * b));
		//u = std::min(255.0, std::max(0.0, -0.168736 * r - 0.331264 * g + 0.5 * b + 128));
		//v = std::min(255.0, std::max(0.0, 0.5 * r - 0.418688 * g - 0.081312 * b + 128));

		// https://msdn.microsoft.com/en-us/library/aa451258.aspx?f=255&MSPPError=-2147217396
		y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
		u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
		v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
	}
	inline static void rgb2yuv(cv::Vec3b& vec)
	{
		uchar y, u, v;
		rgb2yuv(vec[0], vec[1], vec[2], y, u, v);
		vec[0] = y;
		vec[1] = u;
		vec[2] = v;
	}
	inline static void yuv2rgb(uchar y, uchar u, uchar v, uchar& r, uchar& g, uchar& b)
	{
		// http://stackoverflow.com/questions/10566668/lossless-rgb-to-ycbcr-transformation
		//u -= 128;
		//v -= 128;
		//uchar temp;
		//reverseLift(y, v, g, temp);
		//reverseLift(temp, u, r, b);

		// http://research.microsoft.com/pubs/102040/2008_colortransforms_malvarsullivansrinivasan.pdf
		//g = y - ((u + v) >> 2);
		//r = u + g;
		//b = v + g;

		// standard jpeg conversion
		//r = std::min(255.f, std::max(0.f, float(y) + 1.402 * (float(v) - 128.0)));
		//g = std::min(255.f, std::max(0.f, float(y) - 0.34414 * (float(u) - 128.0) - 0.71414 * (float(v) - 128.0)));
		//b = std::min(255.f, std::max(0.f, float(y) + 1.772 * (float(u) - 128.0)));

		// https://msdn.microsoft.com/en-us/library/aa451258.aspx?f=255&MSPPError=-2147217396
		int c = int(y) - 16;
		int d = int(u) - 128;
		int e = int(v) - 128;
		r = std::min(255, std::max(0, (298 * c + 409 * e + 128) >> 8));
		g = std::min(255, std::max(0, (298 * c - 100 * d - 208 * e + 128) >> 8));
		b = std::min(255, std::max(0, (298 * c + 516 * d + 128) >> 8));
	}
	inline static void yuv2rgb(cv::Vec3b& vec)
	{
		uchar y, u, v;
		yuv2rgb(vec[0], vec[1], vec[2], y, u, v);
		vec[0] = y;
		vec[1] = u;
		vec[2] = v;
	}

	class Parallel_bgr2yuv : public cv::ParallelLoopBody
	{

	private:
		cv::Mat& bgrImage;
		cv::Mat& yImage;
		cv::Mat& uvImage;
		bool hasAlpha;

	public:
		Parallel_bgr2yuv(
			cv::Mat& bgrImage, 
			cv::Mat& yImage, 
			cv::Mat& uvImage, 
			bool hasAlpha) :
			bgrImage(bgrImage), 
			yImage(yImage), 
			uvImage(uvImage), 
			hasAlpha(hasAlpha) {}

		virtual void operator()(const cv::Range &range) const {


			register uchar y, u, v, r, g, b;
			register int yId = range.start * bgrImage.cols;

			int bgrId;

			for (register int i = range.start; i != range.end; ++i)
			{
				for (register int j = 0; j < bgrImage.cols; j++)
				{
					if (hasAlpha)
					{
						bgrId = yId << 2;
					}
					else
					{
						bgrId = yId * 3;
					}

					b = bgrImage.data[bgrId];
					g = bgrImage.data[bgrId + 1];
					r = bgrImage.data[bgrId + 2];

					rgb2yuv(r, g, b, y, u, v);

					yImage.data[yId++] = y;

					if ((i & 1) == 0 && (j & 1) == 0)
					{
						uvImage.at<cv::Vec2b>(i >> 1, j >> 1) = cv::Vec2b(u, v);
					}
				}
			}
		}
	};

	class Parallel_yuv2bgr : public cv::ParallelLoopBody
	{

	private:
		cv::Mat& bgrImage;
		cv::Mat& yImage;
		cv::Mat& uvImage;
		bool hasAlpha;

	public:
		Parallel_yuv2bgr(
			cv::Mat& bgrImage, 
			cv::Mat& yImage, 
			cv::Mat& uvImage, 
			bool hasAlpha) :
			bgrImage(bgrImage), 
			yImage(yImage), 
			uvImage(uvImage), 
			hasAlpha(hasAlpha) {}

		virtual void operator()(const cv::Range &range) const {


			register uchar y, u, v, r, g, b;
			register int yId = range.start * bgrImage.cols;

			int bgrId;

			for (register int i = range.start; i != range.end; ++i)
			{
				for (register int j = 0; j < bgrImage.cols; j++)
				{
					if (hasAlpha)
					{
						bgrId = yId << 2;
					}
					else
					{
						bgrId = yId * 3;
					}

					y = yImage.data[yId++];
					cv::Vec2b uv = uvImage.at<cv::Vec2b>(i >> 1, j >> 1);
					u = uv[0];
					v = uv[1];

					yuv2rgb(y, u, v, r, g, b);

					bgrImage.data[bgrId] = b;
					bgrImage.data[bgrId + 1] = g;
					bgrImage.data[bgrId + 2] = r;
					if (hasAlpha)
					{
						bgrImage.data[bgrId + 3] = 255;
					}
				}
			}
		}
	};

	inline static void convertRGB2YUV2RGB(const cv::Mat& inputRGB, int loopCount = 4)
	{
		cv::Mat bgraImage;
		cv::cvtColor(inputRGB, bgraImage, CV_BGR2BGRA);
		cv::Mat yImage(bgraImage.rows, bgraImage.cols, CV_8UC1);
		cv::Mat uvImage = cv::Mat(bgraImage.rows / 2, bgraImage.cols / 2, CV_8UC2);

		for (int i = 0; i < loopCount; ++i)
		{
			parallel_for_(cv::Range(0, inputRGB.rows - 1), Parallel_bgr2yuv(bgraImage, yImage, uvImage, true));
			parallel_for_(cv::Range(0, inputRGB.rows - 1), Parallel_yuv2bgr(bgraImage, yImage, uvImage, true));
		}

		cv::Mat inputResized;
		cv::resize(inputRGB, inputResized, cv::Size(), 0.25, 0.25, 0);
		cv::imshow("input resized", inputResized);

		cv::resize(bgraImage, bgraImage, cv::Size(), 0.25, 0.25, 0);
		cv::imshow("conversion result", bgraImage);
	}

}
