/*
 * BALProblem.h
 *
 *  Created on: 17/03/2016
 *      Author: thulioaraujo
 */

#ifndef HEADER_STAM_BALPROBLEM_H_
#define HEADER_STAM_BALPROBLEM_H_

#include "Defines.h"

// Read a Bundle Adjustment in the Large dataset.
class BALProblem {
public:
	~BALProblem() {
		delete[] point_index_;
		delete[] camera_index_;
		delete[] observations_;
		delete[] parameters_;
	}

	std::map<int,int> kf_map()         { return kf_map_;                          }
	int num_observations()       const { return num_observations_;               }
	const double* observations() const { return observations_;                   }
	double* mutable_cameras()          { return parameters_;                     }
	double* mutable_points()           { return parameters_  + 9 * num_cameras_; }
	double* mutable_camera_for_observation(int i) {
		return mutable_cameras() + camera_index_[i] * 9;
	}
	double* mutable_point_for_observation(int i) {
		return mutable_points() + point_index_[i] * 3;
	}
	bool LoadCeresSBAParams(
		std::map<int, cv::Point3d> map_, // point3D id -> point3D
		std::map<int, cv::Point2d> points2D_, // point2D id -> point2D

		std::map<int, cv::Mat> Rs_, // keyframe id -> Rotation Matrix
		std::map<int, cv::Mat> Ts_, // keyframe id -> Translation Vector

		// keyframe id -> [ pair(point3D id, point2D id) ]
		// => Projection (point2D id) of point3D on Keyframe with given keyframe id
		std::multimap<int, std::pair< int, int > > projections_,

		// keyframe id -> [ pair(point3D id, True|False) ]
		// => True: point3D id is visible from keyframe id
		// => False: point3D id is not visible from keyframe id
		std::multimap<int, std::pair< int, bool > > visibility_,

		// keyframe id -> intrinsic matrix, distortion coefficients
		std::map<int, cv::Mat> cam_matrix_list,

		std::map<int, cv::Mat> dist_coeff_list
	) {

		num_points_ = map_.size();
		num_observations_ = projections_.size();

		point_index_ = new int[num_observations_];
		camera_index_ = new int[num_observations_];
		observations_ = new double[2 * num_observations_];

		int i = 0;
		int kf_index = 0;
		int kf = 0;
		for(auto it = projections_.begin(); it != projections_.end(); it++, i++) {

			auto kf_it = kf_map_.find(it->first);

			if(kf_it == kf_map_.end()) {
				kf = kf_index++;
				kf_map_.insert(std::make_pair(it->first,kf));

			} else {
				kf = kf_it->second;
			}

			camera_index_[i] = it->first;
			point_index_[i] = it->second.first;
			int p2d_index  = it->second.second;
			observations_[2*i] = points2D_[p2d_index].x;
			observations_[2*i + 1] = points2D_[p2d_index].y;
		}

		num_cameras_ = kf_map_.size();
		num_parameters_ = 9 * num_cameras_ + 3 * num_points_;
		parameters_ = new double[num_parameters_];

		int img_index;
		for(auto it = kf_map_.begin(); it != kf_map_.end(); it++) {

			img_index = it->first;
			kf_index = it->second;

			// Load Rs_
			for (int j = 0; j < 3; j++) {
				parameters_[9*kf_index+j] = Rs_[img_index].at<double>(j,0);
				parameters_[9*kf_index+3+j] = Ts_[img_index].at<double>(j,0);
			}

			// Focal lenght
			parameters_[kf_index+6] = cam_matrix_list[img_index].at<double>(0,0);

			// dist_coeff_list
			parameters_[kf_index+7] = dist_coeff_list[img_index].at<double>(0,0);
			parameters_[kf_index+8] = dist_coeff_list[img_index].at<double>(0,1);
		}

		i = 0;
		for (auto it = map_.begin(); it != map_.end(); it++, i=i+3) {
			parameters_[num_cameras_*9+i] = it->second.x;
			parameters_[num_cameras_*9+i+1] = it->second.y;
			parameters_[num_cameras_*9+i+2] = it->second.z;
		}

		return true;
	}
private:
	std::map<int,int> kf_map_;
	int num_cameras_;
	int num_points_;
	int num_observations_;
	int num_parameters_;
	int* point_index_;
	int* camera_index_;
	double* observations_;
	double* parameters_;
};

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
	SnavelyReprojectionError(double observed_x, double observed_y)
	: observed_x(observed_x), observed_y(observed_y) {}
	template <typename T>
	bool operator()(const T* const camera,
			const T* const point,
			T* residuals) const {

		// camera[0,1,2] are the angle-axis rotation.
		// TODO Represents the R of the SBA code!
		T p[3];
		ceres::AngleAxisRotatePoint(camera, point, p);

		// camera[3,4,5] are the translation.
		// TODO Represents the T of the SBA code!
		p[0] += camera[3];
		p[1] += camera[4];
		p[2] += camera[5];

		// Compute the center of distortion. The sign change comes from
		// the camera model that Noah Snavely's Bundler assumes, whereby
		// the camera coordinate system has a negative z axis.
		// TODO Check if it's necessary to invert the points projection
		T xp = - p[0] / p[2];
		T yp = - p[1] / p[2];

		// Apply second and fourth order radial distortion.
		// TODO represents the distCoeffs of the SBA code
		const T& l1 = camera[7];
		const T& l2 = camera[8];

		T r2 = xp*xp + yp*yp;
		T distortion = T(1.0) + r2  * (l1 + l2  * r2);
		// Compute final projected point position.
		const T& focal = camera[6];
		T predicted_x = focal * distortion * xp;
		T predicted_y = focal * distortion * yp;
		// The error is the difference between the predicted and observed position.
		residuals[0] = predicted_x - T(observed_x);
		residuals[1] = predicted_y - T(observed_y);
		return true;
	}
	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction* Create(const double observed_x,
			const double observed_y) {
		return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
				new SnavelyReprojectionError(observed_x, observed_y)));
	}
	double observed_x;
	double observed_y;
};

#endif /* HEADER_STAM_BALPROBLEM_H_ */
