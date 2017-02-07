/*
* Voxar Labs - CIn-UFPE
* Copyright(c) 2016 by LG Voxar Labs
*
* All rights reserved. No part of this work may be reproduced, stored in a retrieval
* system, or transmitted by any means without prior written Permission of Voxar Labs.
*
* Description: Use OpenCV drawing functions to draw on YUV Images 
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

#ifndef _YUV_DRAWING_
#define _YUV_DRAWING_

// Projet includes
#include "header/stam/ConversionUtil.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace YUVDrawing {

	/** @brief Draws a circle.

	The function circle draws a simple or filled circle with a given center and radius.
	@param img Image where the circle is drawn.
	@param center Center of the circle.
	@param radius Radius of the circle.
	@param color Circle color.
	@param thickness Thickness of the circle outline, if positive. Negative thickness means that a
	filled circle is to be drawn.
	@param lineType Type of the circle boundary. See the line description.
	@param shift Number of fractional bits in the coordinates of the center and in the radius value.
	*/
	inline static void circle(cv::Mat& yImage, cv::Mat& uvImage, const cv::Point &center, int radius,
							  const cv::Scalar& rgbColor, int thickness = 1, int lineType = 8,
							  int shift = 0) {
		
		// Convert RGB color space to YUV
		uchar r = rgbColor.val[0];
		uchar g = rgbColor.val[1];
		uchar b = rgbColor.val[2];
		uchar y = (( 66 * r + 129 * g + 25  * b + 128) >> 8) + 16;
		uchar u = ((-38 * r - 74  * g + 112 * b + 128) >> 8) + 128;
		uchar v = ((112 * r - 94  * g - 18  * b + 128) >> 8) + 128;

		cv::circle(yImage, center, radius, cv::Scalar(y), thickness, lineType, shift);
		cv::circle(uvImage, cv::Point(center.x / 2, center.y / 2), radius / 2, cv::Scalar(u, v), thickness, lineType, shift);
	}

	/** @brief Draws a text string.

	The function putText renders the specified text string in the image. Symbols that cannot be rendered
	using the specified font are replaced by question marks. See getTextSize for a text rendering code
	example.

	@param img Image.
	@param text Text string to be drawn.
	@param org Bottom-left corner of the text string in the image.
	@param fontFace Font type, see cv::HersheyFonts.
	@param fontScale Font scale factor that is multiplied by the font-specific base size.
	@param color Text color.
	@param thickness Thickness of the lines used to draw a text.
	@param lineType Line type. See the line for details.
	@param bottomLeftOrigin When true, the image data origin is at the bottom-left corner. Otherwise,
	it is at the top-left corner.
	*/
	inline static void textButton(cv::Mat& yImage, cv::Mat& uvImage, const cv::Rect& rect,
								  const std::string& text, int fontFace = CV_FONT_HERSHEY_COMPLEX,
								  double fontScale = 1,
								  const cv::Scalar& backgroundColor = cv::Scalar(255, 255, 255),
								  const cv::Scalar& foregroundColor = cv::Scalar(0, 0, 0),
								  int thickness = 4, int lineType = 8, bool bottomLeftOrigin = false) {

		// Convert RGB color space to YUV
		uchar r = backgroundColor.val[0];
		uchar g = backgroundColor.val[1];
		uchar b = backgroundColor.val[2];
		uchar bgY = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
		uchar bgU = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
		uchar bgV = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;

		r = foregroundColor.val[0];
		g = foregroundColor.val[1];
		b = foregroundColor.val[2];
		uchar fgY = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
		uchar fgU = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
		uchar fgV = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;

		// Draw background text
		cv::putText(yImage, text, cv::Point(rect.x, rect.y + rect.height), fontFace, fontScale, cv::Scalar(bgY), thickness, lineType, bottomLeftOrigin);
		cv::putText(uvImage, text, cv::Point(rect.x / 2, (rect.y + rect.height) / 2), fontFace, fontScale / 2, cv::Scalar(bgU, bgV), thickness / 2, lineType, bottomLeftOrigin);

		// Draw foreground text
		cv::putText(yImage, text, cv::Point(rect.x, rect.y + rect.height), fontFace, fontScale, cv::Scalar(fgY), thickness / 2, lineType, bottomLeftOrigin);
		cv::putText(uvImage, text, cv::Point(rect.x / 2, (rect.y + rect.height) / 2), fontFace, fontScale / 2, cv::Scalar(fgU, fgV), thickness / 4, lineType, bottomLeftOrigin);
	}

	inline static void yuvMat2bgrMat(cv::Mat& yImage, cv::Mat& uvImage, cv::Mat& bgrImage) {
		bgrImage = cv::Mat::zeros(yImage.size(), CV_8UC3);
		parallel_for_(cv::Range(0, bgrImage.rows - 1), ConversionUtil::Parallel_yuv2bgr(bgrImage, yImage, uvImage, false));
	}

	inline static void bgrMat2yuvMat(cv::Mat& bgrImage, cv::Mat& yImage, cv::Mat& uvImage) {
		parallel_for_(cv::Range(0, bgrImage.rows - 1), ConversionUtil::Parallel_bgr2yuv(bgrImage, yImage, uvImage, false));
	}

	inline static void render(cv::Mat& yImage, cv::Mat& uvImage, const std::string& windowName, int delay = 1) {
		cv::Mat rgbMat, bgrMat;
		yuvMat2bgrMat(yImage, uvImage, bgrMat);
		//cv::cvtColor(bgrMat, rgbMat, CV_BGR2RGB);
		cv::imshow(windowName, bgrMat);
		cv::waitKey(delay);
	}
}

#endif // _YUV_DRAWING_
