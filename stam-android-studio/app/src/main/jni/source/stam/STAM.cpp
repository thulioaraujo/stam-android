/** @file STAM.cpp
 *
 * @author	Ermano A Arruda (eaa3@cin.ufpe.br)
 * @author	Joao Marcelo Teixeira (jmxnt@cin.ufpe.br)
 * @author  Joao Paulo Lima (jpsml@cin.ufpe.br)
 *
 * @version 1.2
 *
 * Adding STAM::initFromChessboard overload to handle with RGB-D images
 *
 */

#include "header/stam/STAM.h"

int SCENE = 1;

double baseline[] = { 175, 50, 35, 175, 50};
std::string points2d_init_file[] = { "S01L03/S01_2Ddata_dst_init.csv", "S02L03/S02_2Ddata_dst_init.csv", "S03L03/S03_2Ddata_dst_init.csv" };
std::string points3d_init_file[] = { "S01L03/S01_3Ddata_dst_init.csv", "S02L03/S02_3Ddata_dst_init.csv", "S03L03/S03_3Ddata_dst_init.csv" };
std::string intrinsics_file[] = { "S01L03/intrinsicsS01.xml", "S02L03/intrinsicsS02.xml", "S03L03/intrinsicsS03.xml", "intrinsicsOnSite.xml" , "intrinsicsXtion_vga.xml"};
std::string next_frame_fmt[] = { "S01L03/S01L03_VGA/S01L03_VGA_%04d.png", "S02L03/S02L03_VGA/S02L03_VGA_%04d.png", "S03L03/S03L03_VGA/S03L03_VGA_%04d.png"};

const double THR_BASELINE = baseline[SCENE-1];
const std::string POINTS_2D_INIT_FILE = points2d_init_file[SCENE-1];
const std::string POINTS_3D_INIT_FILE = points3d_init_file[SCENE-1];
const std::string INTRINSICS_FILE = intrinsics_file[SCENE-1];
const std::string NEXT_FRAME_FMT = next_frame_fmt[SCENE-1];

namespace visual_odometry {

	int STAM::currentChallengePoint = 0;
	std::vector<int> STAM::challengePointsIds = std::vector<int>();
	std::vector<cv::Point3d> STAM::challengePoints = std::vector<cv::Point3d>();
	bool STAM::calibrate = false;
	bool STAM::reinitialize = false;

void STAM::init(std::string resourcesPath) {

	params.baseline_thr = baseline[SCENE - 1];
	params.INTRINSICS_FILE = resourcesPath + intrinsics_file[SCENE - 1];

	loadIntrinsicsFromFile(params.INTRINSICS_FILE);
	loadChallengePoints(resourcesPath + CHALLENGE_FILE);

#ifdef WINDOWS_VS
	if (SCENE < 4) {
		params.POINTS_2D_INIT_FILE = points2d_init_file[SCENE - 1];
		params.POINTS_3D_INIT_FILE = points3d_init_file[SCENE - 1];
		params.NEXT_FRAME_FMT = next_frame_fmt[SCENE - 1];
	}
#endif
}

Frame::Ptr STAM::process(cv::Mat& yImage, cv::Mat& uvImage) {
	
	cv::Mat bgrImage;

	YUVDrawing::yuvMat2bgrMat(yImage, uvImage, bgrImage);

	if (bgrImage.empty()) {
		return Frame::Ptr();
	}

	Frame::Ptr current_frame = Frame::Ptr(new Frame(bgrImage));

	//if STAM_VIDEO_SOURCE_RGBD_SENSOR
	//Frame::Ptr current_frame = Frame::Ptr(new Frame(rgbImage, depth));

	//cv::resize(bgrImage, bgrImage, cv::Size(bgrImage.cols/2, bgrImage.rows/2));

	updateUsingKLT(bgrImage);

	auto itId = trackset_.ids_.begin();
	for (auto it = trackset_.points2D_.begin(); it != trackset_.points2D_.end() && itId != trackset_.ids_.end(); it++, itId++){
		if (*itId >= 0) {
			current_frame->keypoints.push_back( cv::KeyPoint(it->x, it->y, 1) );
			current_frame->ids_.push_back(*itId);
		}
	}

	// adapt to use guess
	current_frame->projMatrix = calcProjMatrix(false, current_frame->r, current_frame->t);
	
	cv::Mat R1 = current_frame->projMatrix(cv::Range::all(), cv::Range(0, 3));
	cv::Mat T1 = current_frame->projMatrix(cv::Range::all(), cv::Range(3, 4));

	cv::Mat Pose(3, 4, CV_64FC1);
	cv::Mat pos, R;
	R = R1.inv();
	pos = R * T1;

	R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
	pos.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
	current_frame->pose = Pose;

	// last keyframe
	Frame::Ptr key_frame = key_frames_.back();

	bool enough_baseline = hasEnoughBaseline(key_frame->projMatrix, current_frame->projMatrix, params.baseline_thr);
	
	if (enough_baseline) {
		current_frame->detectAndDescribe();
		mapping(key_frame, current_frame);
	}

	// Project target point and draw it on the image
	projectAndDrawTargetPoint(current_frame->projMatrix, bgrImage);
	
	YUVDrawing::bgrMat2yuvMat(bgrImage, yImage, uvImage);

#ifdef WINDOWS_VS
	// Create and draw previous button to render
	std::string prev = "<";
	cv::Point prevPoint = cv::Point(5, yImage.rows * 0.5);
	cv::Rect prevButton = getButtonRect(prev, prevPoint, false);
	YUVDrawing::textButton(yImage, uvImage, prevButton, prev);

	// Create and draw next button to render
	std::string next = ">";
	cv::Point nextPoint = cv::Point(yImage.cols - 5, yImage.rows * 0.5);
	cv::Rect nextButton = getButtonRect(next, nextPoint, true);
	YUVDrawing::textButton(yImage, uvImage, nextButton, next);

	// Create and draw recalibrate button
	std::string reinit = "r";
	cv::Size sizeR = cv::getTextSize("r", CV_FONT_HERSHEY_COMPLEX, 1, 1, 0);
	cv::Point reinitializePoint = cv::Point(yImage.cols * 0.5 - sizeR.width, 10);
	cv::Rect reinitializeButton = cv::Rect(reinitializePoint.x, reinitializePoint.y, sizeR.width, sizeR.height);
	YUVDrawing::textButton(yImage, uvImage, reinitializeButton, reinit);

	// Render image 
	YUVDrawing::render(yImage, uvImage, "STAM");

	// Keyboard event grabber
	int key = cv::waitKey(10);
	if (key == 27) {// esc
		exit(0);
	}
	else if (key == 'r' || key == 'R') {
		reinitialize = true;
	}
	else if (key == 2555904) {// Right arrow
		currentChallengePoint = (currentChallengePoint + 1) % challengePoints.size();
	}
	else if (key == 2424832) {//Left arrow
		currentChallengePoint = (currentChallengePoint - 1) % challengePoints.size();
	}
#endif

	return current_frame;
}

bool STAM::hasEnoughBaseline(const cv::Mat &pose1, const cv::Mat &pose2, double thr_baseline) {

	cv::Mat R1 = pose1(cv::Range::all(), cv::Range(0, 3));
	cv::Mat T1 = pose1(cv::Range::all(), cv::Range(3, 4));

	cv::Mat R2 = pose2(cv::Range::all(), cv::Range(0, 3));
	cv::Mat T2 = pose2(cv::Range::all(), cv::Range(3, 4));

	cv::Mat pos1, pos2;
	pos1 = -R1.inv() * T1;
	pos2 = -R2.inv() * T2;

	double baseline = cv::norm(pos1 - pos2);

	return baseline >= thr_baseline;
}

void STAM::loadIntrinsicsFromFile(const std::string& filename){

	cv::FileStorage cvfs(filename.c_str(),CV_STORAGE_READ);
	if( cvfs.isOpened() ){
		cvfs["mat_intrinsicMat"] >> intrinsics_;
		cvfs["mat_distortionMat"] >> distortion_;
	}
	else {
#ifdef WINDOWS_VS
		printf("Error: could not load intrinsic file");
		exit(1);
#elif ANDROID
		LOGE("Error: could not load intrinsic file");
#endif
	}
}

void STAM::initFromFiles(cv::Mat& image, const std::string& p2D_filename, const std::string& p3D_filename){
	FILE *p2D_f = fopen(p2D_filename.c_str(), "r");
	FILE *p3D_f = fopen(p3D_filename.c_str(), "r");

	cv::Point3f p3D;
	cv::Point2f p2D;

	Frame::Ptr frame(new Frame(image));
	trackset_.image_ = image;

	while( fscanf(p3D_f, "%f,%f,%f", &p3D.x, &p3D.y, &p3D.z) == 3 && fscanf(p2D_f, "%f,%f", &p2D.x, &p2D.y) == 2 ){

		// return: (p3d_id, p2d_id)
		auto added_ids = memory_.addCorrespondence(frame->id_, p2D, p3D);

		frame->keypoints.push_back( cv::KeyPoint(p2D.x, p2D.y, 1) );
		// Indicate the 3D point this keypoint refers to
		frame->ids_.push_back( added_ids.first );

		trackset_.points2D_.push_back( p2D );
		trackset_.ids_.push_back( added_ids.first );
	}

	frame->detectAndDescribe();

	for (int i = 0; i < frame->keypoints.size(); ++i)
	{
		tracked_keyframe_pts_.push_back(frame->keypoints[i].pt);
		tracked_keyframe_pts_ids_.push_back(i);
	}

	frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
	trackset_.projMatrix = frame->projMatrix;

	key_frames_.push_back(frame);

	// Not used
	previous_frame_ = frame;

	memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));

	fclose(p2D_f);
	fclose(p3D_f);
}

bool STAM::initFromChessboard(cv::Mat& yImage, cv::Mat& uvImage, const cv::Size& chessBoardSize, int squareSize)
{
	cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F);
	cv::Mat tvec = cv::Mat(cv::Size(3, 1), CV_64F);

	std::vector<cv::Point2d> imagePoints, imageBoardPoints;
	std::vector<cv::Point3d> boardPoints;

	for (int i = 0; i < chessBoardSize.height; i++) {
		for (int j = 0; j < chessBoardSize.width; j++) {
			boardPoints.push_back(cv::Point3d(j*squareSize, i*squareSize, 0.0));
		}
	}

	bool found = findChessboardCorners(yImage, chessBoardSize, imagePoints, cv::CALIB_CB_FAST_CHECK);

#ifdef WINDOWS_VS
	printf("Number of chessboard points: %d\n", imagePoints.size());
#elif ANDROID
	//LOGE("Number of chessboard points: %d", imagePoints.size());
#endif
	
	cv::Mat bgrImage;
	YUVDrawing::yuvMat2bgrMat(yImage, uvImage, bgrImage);

	for (int i = 0; i < imagePoints.size(); i++) {
		YUVDrawing::circle(yImage, uvImage, imagePoints[i], 6, cv::Scalar(0, 255, 0), -1);
	}

#ifdef WINDOWS_VS
	// Create button to render
	std::string calib = "c";
	cv::Size sizeC = cv::getTextSize(calib, CV_FONT_HERSHEY_COMPLEX, 1, 1, 0);
	cv::Point calibratePoint = cv::Point(yImage.size().width - 5 - sizeC.width, yImage.size().height * 0.5 + sizeC.height);
	cv::Rect calibrateButton = cv::Rect(calibratePoint.x, calibratePoint.y, sizeC.width, sizeC.height);
	
	// Draw button on image
	YUVDrawing::textButton(yImage, uvImage, calibrateButton, calib);
	
	// Render image 
	YUVDrawing::render(yImage, uvImage, "Calibration");

	// Keyboard event grabber
	int key = cv::waitKey(10);
	if (key == 'c' || key == 'C') {
		calibrate = true;
	}
	else if (key == 27) {
		exit(0);
	}
#endif

	if (imagePoints.size() != chessBoardSize.height * chessBoardSize.width || !calibrate) {
		return false;
	}

	Frame::Ptr frame(new Frame(bgrImage));
	trackset_.image_ = bgrImage;

	for (int i = 0; i < imagePoints.size(); i++) {
		auto added_ids = memory_.addCorrespondence(frame->id_, imagePoints[i], boardPoints[i]);

		// Indicate the 3D point this keypoint refers to
		frame->ids_.push_back(added_ids.first);

		trackset_.points2D_.push_back(imagePoints[i]);
		trackset_.ids_.push_back(added_ids.first);
	}

	std::vector<cv::Point3d> cornersPoints3d;
	cornersPoints3d.push_back(cv::Point3d(-100, -100, 0));
	cornersPoints3d.push_back(cv::Point3d(-100,  600, 0));
	cornersPoints3d.push_back(cv::Point3d( 900,  600, 0));
	cornersPoints3d.push_back(cv::Point3d( 900, -100, 0));

	std::cout << intrinsics_ << std::endl;
	std::cout << distortion_ << std::endl;

	cv::solvePnP(cv::Mat(boardPoints), cv::Mat(imagePoints), intrinsics_, distortion_, rvec, tvec, false);
	cv::projectPoints(cornersPoints3d, rvec, tvec, intrinsics_, distortion_, imageBoardPoints);

	std::vector<cv::Point> imageBoardPoint2d;
	for (int i = 0; i < imageBoardPoints.size(); i++) {
		imageBoardPoint2d.push_back(imageBoardPoints[i]);
	}

	cv::Mat roi = cv::Mat(yImage.size(), CV_8UC1, cv::Scalar::all(255));
	cv::fillConvexPoly(roi, imageBoardPoint2d, cv::Scalar::all(0));

	frame->detectAndDescribe(roi);

	for (int i = 0; i < frame->keypoints.size(); ++i) {
		tracked_keyframe_pts_.push_back(frame->keypoints[i].pt);
		tracked_keyframe_pts_ids_.push_back(i);
	}

	frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
	trackset_.projMatrix = frame->projMatrix;

	key_frames_.push_back(frame);

	memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0, 1), cv::Range(0, 5)));

	calibrate = false;

	return found;
}

//bool STAM::initFromChessboard(const cv::Mat& colorImage, const cv::Mat& depthImage, const cv::Size& chessBoardSize, int squareSize)
//{
//	cv::Mat rvec = cv::Mat(cv::Size(3, 1), CV_64F);
//	cv::Mat tvec = cv::Mat(cv::Size(3, 1), CV_64F);
//
//	std::vector<cv::Point2d> imagePoints, imageBoardPoints;
//	std::vector<cv::Point3d> boardPoints;
//
//	cv::Mat gray;
//	cvtColor(colorImage, gray, cv::COLOR_BGR2GRAY);
//
//	bool found = findChessboardCorners(gray, chessBoardSize, imagePoints, cv::CALIB_CB_FAST_CHECK);
//
//	printf("%d\n", imagePoints.size());
//
//	cv::imshow("frame", gray);
//	cv::Mat depthScaled;
//	depthImage.convertTo(depthScaled, CV_8U, 255.0 / 4000.0, 0.0);
//	cv::imshow("depth", depthScaled);
//	cv::waitKey(1);
//
//	if (imagePoints.size() != chessBoardSize.width * chessBoardSize.height) {
//		return false;
//	}
//
//	cvtColor(gray, gray, cv::COLOR_GRAY2BGR);
//	for (int i = 0; i < imagePoints.size(); i++)
//	{
//		cv::circle(gray, imagePoints[i], 4, cv::Scalar(0, 255, 0), 2);
//	}
//
//	cv::imshow("frame", gray);
//	int key = cv::waitKey(30);
//
//	if (key != 'c' && key != 'C')
//	{
//		return false;
//	}//*/
//
//	for (int i = 0; i < imagePoints.size(); i++)
//	{
//		// compute 3D chessboard points
//		ushort d = depthImage.at<ushort>(imagePoints[i].y, imagePoints[i].x);
//
//		if (d)
//		{
//			double f = intrinsics_.at<double>(0, 0);
//			double factor = d / f;
//
//			double cx = intrinsics_.at<double>(0, 2);
//			double cy = intrinsics_.at<double>(1, 2);
//
//			boardPoints.push_back(cv::Point3d((imagePoints[i].x - cx) * factor, (imagePoints[i].y + cy - colorImage.rows) * factor, d));
//			imageBoardPoints.push_back(imagePoints[i]);
//		}
//	}
//
//	refPoint = boardPoints[0];
//
//	cv::Mat chessImage = gray.clone();
//	for (int i = 0; i < imagePoints.size(); i++) {
//		cv::circle(chessImage, imagePoints[i], 3, cv::Scalar(0, 255, 0));
//	}
//	cv::imwrite("resource/chessImage.png", chessImage);
//
//	std::vector<cv::Point2d> projectedPoints;
//	projectPoints(boardPoints, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), intrinsics_, distortion_, projectedPoints);
//
//	for (int i = 0; i < projectedPoints.size(); i++) {
//		cv::circle(gray, projectedPoints[i], 3, cv::Scalar(128), -1);
//	}
//
//	cv::imwrite("resource/initialization.png", gray);
//	cv::imshow("initialization", gray);
//	cv::waitKey(1);//*/
//
//	Frame::Ptr frame(new Frame(colorImage, depthImage));
//	trackset_.image_ = colorImage;
//
//	for (int i = 0; i < imageBoardPoints.size(); i++)
//	{
//		// return: (p3d_id, p2d_id)
//		auto added_ids = memory_.addCorrespondence(frame->id_, imageBoardPoints[i], boardPoints[i]);
//
//		frame->keypoints.push_back(cv::KeyPoint(imageBoardPoints[i].x, imageBoardPoints[i].y, 1));
//		// Indicate the 3D point this keypoint refers to
//		frame->ids_.push_back(added_ids.first);
//
//		trackset_.points2D_.push_back(imageBoardPoints[i]);
//		trackset_.ids_.push_back(added_ids.first);
//	}
//
//	frame->detectAndDescribe();
//
//	for (int i = 0; i < frame->keypoints.size(); ++i)
//	{
//		tracked_keyframe_pts_.push_back(frame->keypoints[i].pt);
//		tracked_keyframe_pts_ids_.push_back(i);
//	}
//
//	frame->projMatrix = calcProjMatrix(false, frame->r, frame->t);
//	trackset_.projMatrix = frame->projMatrix;
//
//	key_frames_.push_back(frame);
//
//	// Not used
//	previous_frame_ = frame;
//
//	memory_.addKeyFrame(frame->id_, frame->r, frame->t, intrinsics_, distortion_(cv::Range(0, 1), cv::Range(0, 5)));
//
//	// TODO
//	// compute transform from local coordinate system to global coordinate system
//	// absolute orientation
//
//	return found;
//}

void STAM::loadChallengePoints(const std::string &file)
{
	std::ifstream ifs(file);
	if (ifs.is_open()) {
		int numberOfPoints;

		ifs >> numberOfPoints;

		for (int i = 0; i < numberOfPoints; i++)
		{
			int id;
			ifs >> id;
			challengePointsIds.push_back(id);

			double x, y, z;
			ifs >> y >> x >> z;
			challengePoints.push_back(cv::Point3d(x, y, -z));
		}
		ifs.close();
	}
	else {
#ifdef WINDOWS_VS
		printf("Error: could not load challenge points file");
		exit(1);
#elif ANDROID
		LOGE("Error: could not load challenge points file");
#endif
	}
}

cv::Mat STAM::calcProjMatrix(bool use_guess, cv::Mat& guess_r, cv::Mat& guess_t) {

	std::vector<cv::Point3f> points3d;
	std::vector<cv::Point2f> points2d;

	for (int i = std::max<int>(0,(trackset_.points2D_.size()-100)); i < trackset_.points2D_.size(); i++) {

		if( trackset_.ids_[i] >= 0 ){
			points3d.push_back(memory_.map_[trackset_.ids_[i]]);
			points2d.push_back(trackset_.points2D_[i]);
		}
	}

#ifdef WINDOWS_VS
	printf("Number of Points %d\n", points3d.size());
#elif ANDROID
	LOGE("Number of Points %d\n", points3d.size());
#endif

	if (points3d.size() >= 4) {
		cv::solvePnPRansac(points3d, points2d, intrinsics_, distortion_, guess_r, guess_t, use_guess, 100, 5.0, 0.99);
	}

	cv::Mat projMatrix = cv::Mat::eye(3, 4, CV_64FC1);
	if (!guess_r.empty() && !guess_t.empty()) {

		cv::Mat rvec = guess_r, tvec = guess_t;
		cv::Mat R(3, 3, CV_64FC1);
		cv::Rodrigues(rvec, R);

		cv::Mat Pose(3,4, R.type());
		R.copyTo(Pose(cv::Rect(0, 0, 3, 3)));
		tvec.copyTo(Pose(cv::Rect(3, 0, 1, 3)));
		projMatrix = Pose;
	}
	return projMatrix;
}

void STAM::updateUsingKLT(cv::Mat rgbImage) {
	std::vector<cv::Point2f> currentPoints;
	std::vector<uchar> status;
	cv::Mat errors;

	if (trackset_.points2D_.size() > 0)	{
		cv::calcOpticalFlowPyrLK(trackset_.image_, rgbImage, trackset_.points2D_, currentPoints, status, errors);
	}

	std::vector<cv::Point2f> filteredP2d;
	std::vector<int> filteredIds;
	for (int i = 0; i < trackset_.points2D_.size(); i++) {
		if (status[i] != 0 && errors.at<float>(i) < 12.0f) {
			// Meaning: f is visible in this frame
			filteredP2d.push_back(currentPoints[i]);
			filteredIds.push_back(trackset_.ids_[i]);
		}
	}

	trackset_.points2D_ = filteredP2d;
	trackset_.ids_ = filteredIds;
	trackset_.image_ = rgbImage.clone();
}

void STAM::mapping(Frame::Ptr &key_frame, Frame::Ptr &current_frame){

	//* Use 2 '/' to track using descriptor or 1 to use optical flow
	bool success = matchAndTriangulate(key_frame, current_frame, intrinsics_, distortion_);
	/*/
	bool success = matchAndTriangulateKlt(key_frame, current_frame, intrinsics_, distortion_);
	//*/

	//#elif defined(STAM_VIDEO_SOURCE_RGBD_SENSOR)
	//	bool success = unprojectAndAlign(key_frame, current_frame, intrinsics_);
	//#endif

	if(success) {
		tracked_keyframe_pts_.clear();
		tracked_keyframe_pts_ids_.clear();

		for (int i = 0; i < current_frame->keypoints.size(); ++i) {
			tracked_keyframe_pts_.push_back(current_frame->keypoints[i].pt);
			tracked_keyframe_pts_ids_.push_back(i);
		}

		// This frame becomes a keyframe
		key_frames_.push_back(current_frame);

		memory_.addKeyFrame(current_frame->id_, current_frame->r, current_frame->t, intrinsics_, distortion_(cv::Range(0,1),cv::Range(0,5)));
	}
}

bool STAM::matchAndTriangulate(Frame::Ptr& key_frame, Frame::Ptr& current_frame, const cv::Mat& intrinsics, const cv::Mat& distortion) {

	std::vector<cv::DMatch> matches;
	std::vector<bool> mask(current_frame->keypoints.size(), false);
	matcher.match(current_frame->descriptors, key_frame->descriptors, matches, current_frame->keypoints, key_frame->keypoints);

#ifdef WINDOWS_VS
	printf("NMatches %d\n", matches.size());
#elif ANDROID
	LOGE("NMatches %d\n", matches.size());
#endif
	
	if (!matches.size()) {
		return false;
	}

	cv::Mat	outputTriangulate;
	std::vector<cv::Point2f> previousTriangulate, currentTriangulate;
	outputTriangulate.create(cv::Size(4, matches.size()), CV_32FC1);

	// Points for triangulation (i.e. points in current_frame that doesn't have 3D points already)
	std::vector<int> triangulation_indices;
	std::vector<cv::Point3f> points3D;
	std::vector<cv::Point2f> points2D;
	for (int i = 0; i < matches.size(); i++) {

		if( key_frame->ids_[matches[i].trainIdx] < 0 ){
			cv::Point pt1 = key_frame->keypoints[matches[i].trainIdx].pt;
			cv::Point pt2 = current_frame->keypoints[matches[i].queryIdx].pt;

			previousTriangulate.push_back(pt1);
			currentTriangulate.push_back(pt2);

			triangulation_indices.push_back(matches[i].queryIdx);
		}
		// Otherwise, we need to update a new observation for this point which already has a 3D correspondence
		else {
			cv::Point2f pt2 = current_frame->keypoints[matches[i].queryIdx].pt;
			int p3d_id = key_frame->ids_[matches[i].trainIdx];

			memory_.addCorrespondence(current_frame->id_, pt2, p3d_id);

			// we also add those points to the trackingset_
			trackset_.points2D_.push_back(pt2);
			trackset_.ids_.push_back(p3d_id);

			points2D.push_back(pt2);
			points3D.push_back(memory_.map_[p3d_id]);
		}
		// marking points which had matches.
		// Later we can use the negation of this mask to extract the points that had no matchings
		mask[matches[i].queryIdx] = true;
	}

	if( previousTriangulate.size() == 0 || currentTriangulate.size() == 0 ){
		return false;
	}

	// undistort
	std::vector<cv::Point2f> previousTriangulateUnd, currentTriangulateUnd;
	cv::undistortPoints(previousTriangulate, previousTriangulateUnd, intrinsics, distortion);
	cv::undistortPoints(currentTriangulate, currentTriangulateUnd, intrinsics, distortion);

	cv::triangulatePoints(key_frame->projMatrix, current_frame->projMatrix, previousTriangulateUnd, currentTriangulateUnd, outputTriangulate);

	for (int i = 0; i < triangulation_indices.size(); i++) {

		cv::Point2f p2D = currentTriangulate[i];
		cv::Point3f p3D = cv::Point3f(outputTriangulate.at<float>(0, i) / outputTriangulate.at<float>(3, i),
									  outputTriangulate.at<float>(1, i) / outputTriangulate.at<float>(3, i),
									  outputTriangulate.at<float>(2, i) / outputTriangulate.at<float>(3, i)
									  );



		auto added_ids = memory_.addCorrespondence(current_frame->id_, p2D, p3D);
		int p3d_id = added_ids.first;

		// Updating the fact that this 2D point now has a 3D correspondence
		current_frame->ids_[triangulation_indices[i]] = p3d_id;

		// we also add those points to the trackingset_
		trackset_.points2D_.push_back(p2D);
		trackset_.ids_.push_back(p3d_id);

		points2D.push_back(p2D);
		points3D.push_back(memory_.map_[p3d_id]);
	}
	return true;
}

bool STAM::matchAndTriangulateKlt(Frame::Ptr& key_frame, Frame::Ptr& current_frame, const cv::Mat& intrinsics, const cv::Mat& distortion) {
	
	cv::Mat	outputTriangulate;
	std::vector<cv::Point2f> previousTriangulate, currentTriangulate;
	outputTriangulate.create(cv::Size(4, tracked_keyframe_pts_.size()), CV_32FC1);

	// Points for triangulation (i.e. points in current_frame that doesn't have 3D points already)
	std::vector<cv::Point3f> points3D;
	std::vector<cv::Point2f> points2D;
	for (int i = 0; i < tracked_keyframe_pts_.size(); i++) {
		cv::Point pt1 = key_frame->keypoints[tracked_keyframe_pts_ids_[i]].pt;
		cv::Point pt2 = tracked_keyframe_pts_[i];

		previousTriangulate.push_back(pt1);
		currentTriangulate.push_back(pt2);
	}

	if (previousTriangulate.size() == 0 || currentTriangulate.size() == 0) {
		return false;
	}

	// undistort
	std::vector<cv::Point2f> previousTriangulateUnd, currentTriangulateUnd;
	cv::undistortPoints(previousTriangulate, previousTriangulateUnd, intrinsics, distortion);
	cv::undistortPoints(currentTriangulate, currentTriangulateUnd, intrinsics, distortion);

	cv::triangulatePoints(key_frame->projMatrix, current_frame->projMatrix, previousTriangulateUnd, currentTriangulateUnd, outputTriangulate);

	current_frame->keypoints.clear();
	current_frame->ids_.clear();

	for (int i = 0; i < tracked_keyframe_pts_.size(); i++) {
		cv::Point2f p2D = currentTriangulate[i];
		cv::Point3f p3D = cv::Point3f(outputTriangulate.at<float>(0, i) / outputTriangulate.at<float>(3, i),
			outputTriangulate.at<float>(1, i) / outputTriangulate.at<float>(3, i),
			outputTriangulate.at<float>(2, i) / outputTriangulate.at<float>(3, i)
			);

		auto added_ids = memory_.addCorrespondence(current_frame->id_, p2D, p3D);
		int p3d_id = added_ids.first;

		// Updating the fact that this 2D point now has a 3D correspondence
		current_frame->keypoints.push_back(cv::KeyPoint(p2D, 1.0f));
		current_frame->ids_.push_back(p3d_id);

		// we also add those points to the trackingset_
		trackset_.points2D_.push_back(p2D);
		trackset_.ids_.push_back(p3d_id);
	}
	return true;
}

bool STAM::unprojectAndAlign(Frame::Ptr& key_frame, Frame::Ptr& current_frame, cv::Mat& intrinsics)
{
	std::vector<cv::DMatch> matches;

	matcher.match(current_frame->descriptors, key_frame->descriptors, matches, current_frame->keypoints, key_frame->keypoints);
	
	if (!matches.size())
		return false;

	std::vector<cv::Point3d> unprojectedPoints;

	//for (int i = 0; i < current_frame->keypoints.size(); i++)
	for (int i = 0; i < matches.size(); i++)
	{
		//ushort d = current_frame->depth.at<ushort>(current_frame->keypoints[i].pt.y, current_frame->keypoints[i].pt.x);
		ushort d = current_frame->depth.at<ushort>(current_frame->keypoints[matches[i].queryIdx].pt.y, current_frame->keypoints[matches[i].queryIdx].pt.x);

		if (d)
		{
			double f = intrinsics_.at<double>(0, 0);
			double factor = d / f;

			double cx = intrinsics_.at<double>(0, 2);
			double cy = intrinsics_.at<double>(1, 2);

			/*unprojectedPoints.push_back(cv::Point3d((current_frame->keypoints[i].pt.x - cx) * factor,
				(current_frame->keypoints[i].pt.y + cy - current_frame->depth.rows) * factor, d));//*/
			unprojectedPoints.push_back(cv::Point3d((current_frame->keypoints[matches[i].queryIdx].pt.x - cx) * factor,
				(current_frame->keypoints[matches[i].queryIdx].pt.y + cy - current_frame->depth.rows) * factor, d));//*/
		}
	}

	std::vector<cv::Point3d> alignedPoints;

	cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);
	current_frame->pose.copyTo(pose(cv::Rect(0, 0, 4, 3)));

	cv::Mat inversePose = pose.inv();

	cv::transform(unprojectedPoints, alignedPoints, inversePose(cv::Rect(0, 0, 4, 3)));


	cv::Mat rvec;
	cv::Rodrigues(current_frame->pose(cv::Rect(0, 0, 3, 3)), rvec);
	cv::Mat tvec = current_frame->pose.col(3);
	std::vector<cv::Point2d> projectedPoints;
	projectPoints(alignedPoints, rvec, tvec, intrinsics_, distortion_, projectedPoints);

	cv::Mat projPointsImage = current_frame->image.clone();

	for (int i = 0; i < projectedPoints.size(); i++) {
		cv::circle(projPointsImage, projectedPoints[i], 3, cv::Scalar(128), -1);
	}

	cv::Mat keypointsImage;
	cv::drawKeypoints(current_frame->image, current_frame->keypoints, keypointsImage);

	cv::imwrite("resource/projPointsImage.png", projPointsImage);
	cv::imwrite("resource/keypointsImage.png", keypointsImage);//*/

	//for (int i = 0; i < alignedPoints.size(); i++)
	for (int i = 0; i < matches.size(); i++)
	{		
		if (key_frame->ids_[matches[i].trainIdx] < 0)
		{
			cv::Point2f p2D = current_frame->keypoints[i].pt;
			cv::Point3f p3D = alignedPoints[i];

			auto added_ids = memory_.addCorrespondence(current_frame->id_, p2D, p3D);
			int p3d_id = added_ids.first;

			// Updating the fact that this 2D point now has a 3D correspondence
			current_frame->ids_[i] = p3d_id;

			// we also add those points to the trackingset_
			trackset_.points2D_.push_back(p2D);
			trackset_.ids_.push_back(p3d_id);
		}
		// Otherwise, we need to update a new observation for this point which already has a 3D correspondence
		else {
			cv::Point2f pt2 = current_frame->keypoints[matches[i].queryIdx].pt;
			int p3d_id = key_frame->ids_[matches[i].trainIdx];

			memory_.addCorrespondence(current_frame->id_, pt2, p3d_id);

			// we also add those points to the trackingset_
			trackset_.points2D_.push_back(pt2);
			trackset_.ids_.push_back(p3d_id);
		}
	}

	return true;
}

void STAM::projectAndDrawTargetPoint(const cv::Mat& projMatrix, cv::Mat& image) {

	cv::Mat target;
	//cv::Mat imcopy = image.clone();

	cv::Mat point3D;
	point3D.create(cv::Size(1, 4), CV_64FC1);

	point3D.at<double>(0) = STAM::challengePoints[STAM::currentChallengePoint].x;
	point3D.at<double>(1) = STAM::challengePoints[STAM::currentChallengePoint].y;
	point3D.at<double>(2) = STAM::challengePoints[STAM::currentChallengePoint].z;
	point3D.at<double>(3) = 1;

	target.create(cv::Size(1, 3), CV_64FC1);
	cv::gemm(intrinsics_*projMatrix, point3D, 1, 0, 0, target);
	cv::Point targetPoint(target.at<double>(0) / target.at<double>(2), target.at<double>(1) / target.at<double>(2));
	
	if (!competitonMode) {
		cv::Mat result;
		result.create(cv::Size(1, 3), CV_64FC1);

		for (auto it = memory_.map_.begin(); it != memory_.map_.end(); it++) {

			point3D.at<double>(0) = it->second.x;
			point3D.at<double>(1) = it->second.y;
			point3D.at<double>(2) = it->second.z;
			point3D.at<double>(3) = 1;

			cv::Mat p3D;
			p3D.create(cv::Size(1, 4), CV_64FC1);
			p3D.at<double>(0) = it->second.x;
			p3D.at<double>(1) = it->second.y;
			p3D.at<double>(2) = it->second.z;
			p3D.at<double>(2) = 1;
			cv::Mat p3DCam = (projMatrix)*p3D;

			float r = 40.0 / (abs(p3D.at<double>(1)) + 1);
			int ri = 255 * r;
			cv::gemm(intrinsics_*projMatrix, point3D, 1, 0, 0, result);
			cv::Point resultPoint(result.at<double>(0) / result.at<double>(2), result.at<double>(1) / result.at<double>(2));

			//cv::circle(image, resultPoint, 4, cv::Scalar(255, 0, ri), -1);
			//cv::circle(image, resultPoint, 3, cv::Scalar(255, 0, ri), -1);
			cv::circle(image, resultPoint, 1, cv::Scalar(255, 0, ri), -1);
		}
	}

	// Render target point
	cv::circle(image, targetPoint, 5, cv::Scalar(0, 255, 0), -1);
	cv::circle(image, targetPoint, 3, cv::Scalar(255, 255, 255), -1);
	cv::circle(image, targetPoint, 2, cv::Scalar(0, 255, 0), -1);

	// Render target point identification
#ifdef WINDOWS_VS
	cv::putText(image, "Target Point: " + std::to_string(STAM::challengePointsIds[STAM::currentChallengePoint]), cv::Point2i(10, 15), CV_FONT_NORMAL, 0.50, cv::Scalar(255, 255, 255), 1);
#endif

	// Draw compass on the image
	drawCompass(image, targetPoint);
}

void STAM::drawCompass(cv::Mat& image, const cv::Point& targetPoint) {
	
	cv::Point2f compass(image.cols - 50, image.rows - 50);
	cv::Point2f target(targetPoint.x, targetPoint.y);

	float compassradius = 20;
	float length = sqrtf((target.x - compass.x)*(target.x - compass.x) + (target.y - compass.y)*(target.y - compass.y));
	cv::Point2f compasstarget = compass + compassradius * cv::Point2f((target.x - compass.x) / length, (target.y - compass.y) / length);

	cv::line(image, compass, compasstarget, cvScalar(255, 0, 0), 1);
	cv::circle(image, compass, 2, cvScalarAll(0), -1);
}

void STAM::optimise() {
	printf("RODOU O BUNDLE ADJUSTMENT!\n");
	memory_.optimise();
	updateOptimisedKF();
}

void STAM::dump() {
	memory_.dumpMapAndKFExtrinsics();
}

void STAM::updateOptimisedKF(){
	auto it = key_frames_.begin();
	for(;it!=key_frames_.end(); it++)
		memory_.updateKF(*it);
}

cv::Rect STAM::getButtonRect(const std::string& text, cv::Point& point, bool toTheLeft) {
	cv::Size size = cv::getTextSize(text, CV_FONT_HERSHEY_COMPLEX, 1, 1, 0);
	if (toTheLeft) {
		point.x -= size.width;
	}
	cv::Rect result = cv::Rect(point.x, point.y, size.width, size.height);

	return result;
}

void STAM::onMouse(int event, int x, int y, int, void*)
{
	if (event != CV_EVENT_LBUTTONDOWN)
		return;

	cv::Size size = cv::getTextSize("<", CV_FONT_HERSHEY_COMPLEX, 1, 1, 0);
	cv::Point leftPoint = cv::Point(5, 480 * 0.5);
	cv::Rect leftButton = cv::Rect(leftPoint.x, leftPoint.y, size.width, size.height);

	cv::Point rightPoint = cv::Point(640 - 5 - size.width, 480 * 0.5);
	cv::Rect rightButton = cv::Rect(rightPoint.x, rightPoint.y, size.width, size.height);

	cv::Size sizeC = cv::getTextSize("c", CV_FONT_HERSHEY_COMPLEX, 1, 1, 0);
	cv::Point calibratePoint = cv::Point(640 - 5 - sizeC.width, 480 * 0.5 + sizeC.height);
	cv::Rect calibrateButton = cv::Rect(calibratePoint.x, calibratePoint.y, sizeC.width, sizeC.height);

	cv::Size sizeR = cv::getTextSize("r", CV_FONT_HERSHEY_COMPLEX, 1, 1, 0);
	cv::Point reinitializePoint = cv::Point(640 * 0.5 - sizeR.width, 10);
	cv::Rect reinitializeButton = cv::Rect(reinitializePoint.x, reinitializePoint.y, sizeR.width, sizeR.height);


	// check if one of the buttons was clicked
	if (leftButton.contains(cv::Point(x, y)))
	{
		STAM::currentChallengePoint = (STAM::currentChallengePoint - 1) % STAM::challengePoints.size();
	}
	else if (rightButton.contains(cv::Point(x, y)))
	{
		STAM::currentChallengePoint = (STAM::currentChallengePoint + 1) % STAM::challengePoints.size();
	}
	else if (calibrateButton.contains(cv::Point(x, y)))
	{
		STAM::calibrate = true;
	}
	else if (reinitializeButton.contains(cv::Point(x, y)))
	{
		STAM::reinitialize = true;
	}
}

}
