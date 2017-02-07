/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tango-gl/conversions.h>
#include "header/stam/STAM.h"
#include "header/tango/augmented_reality_app.h"

namespace {
const int kVersionStringLength = 128;

// Far clipping plane of the AR camera.
const float kArCameraNearClippingPlane = 0.1f;
const float kArCameraFarClippingPlane = 100.0f;

// This function routes onTangoEvent callbacks to the application object for
// handling.
//
// @param context, context will be a pointer to a AugmentedRealityApp
//        instance on which to call callbacks.
// @param event, TangoEvent to route to onTangoEventAvailable function.
void onTangoEventAvailableRouter(void* context, const TangoEvent* event) {
	using namespace tango_augmented_reality;
	AugmentedRealityApp* app = static_cast<AugmentedRealityApp*>(context);
	app->onTangoEventAvailable(event);
}

// This function routes texture callbacks to the application object for
// handling.
//
// @param context, context will be a pointer to a AugmentedRealityApp
//        instance on which to call callbacks.
// @param id, id of the updated camera..
void onTextureAvailableRouter(void* context, TangoCameraId id) {
	using namespace tango_augmented_reality;
	AugmentedRealityApp* app = static_cast<AugmentedRealityApp*>(context);
	app->onTextureAvailable(id);
}

void OnFrameAvailableRouter(void* context, TangoCameraId,
		const TangoImageBuffer* buffer) {
	using namespace tango_augmented_reality;
	AugmentedRealityApp* app = static_cast<AugmentedRealityApp*>(context);
	app->OnFrameAvailable(buffer);
}

// We could do this conversion in a fragment shader if all we care about is
// rendering, but we show it here as an example of how people can use RGB data
// on the CPU.
inline void Yuv2Rgb(uint8_t yValue, uint8_t uValue, uint8_t vValue, uint8_t* r,
		uint8_t* g, uint8_t* b) {
	*r = yValue + (1.370705 * (vValue - 128));
	*g = yValue - (0.698001 * (vValue - 128)) - (0.337633 * (uValue - 128));
	*b = yValue + (1.732446 * (uValue - 128));
}
}  // namespace

namespace tango_augmented_reality {

void AugmentedRealityApp::onTangoEventAvailable(const TangoEvent* event) {
	std::lock_guard<std::mutex> lock(tango_event_mutex_);
	tango_event_data_.UpdateTangoEvent(event);
}

void AugmentedRealityApp::onTextureAvailable(TangoCameraId id) {
	if (id == TANGO_CAMERA_COLOR) {
		RequestRender();
	}
}

AugmentedRealityApp::AugmentedRealityApp() {}

AugmentedRealityApp::~AugmentedRealityApp() {
	TangoSupport_freeImageBufferManager(yuv_manager_);
	TangoConfig_free(tango_config_);
	JNIEnv* env;
	java_vm_->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
	env->DeleteGlobalRef(calling_activity_obj_);
	is_yuv_texture_available_ = false;
	swap_buffer_signal_ = false;
	yuv_drawable_ = NULL;
}

int AugmentedRealityApp::TangoInitialize(JNIEnv* env, jobject caller_activity) {
	// The first thing we need to do for any Tango enabled application is to
	// initialize the service. We'll do that here, passing on the JNI environment
	// and jobject corresponding to the Android activity that is calling us.
	int ret = TangoService_initialize(env, caller_activity);

	// We want to be able to trigger rendering on demand in our Java code.
	// As such, we need to store the activity we'd like to interact with and the
	// id of the method we'd like to call on that activity.
	jclass cls = env->GetObjectClass(caller_activity);
	on_demand_render_ = env->GetMethodID(cls, "requestRender", "()V");

	calling_activity_obj_ =
			reinterpret_cast<jobject>(env->NewGlobalRef(caller_activity));

	return ret;
}

int AugmentedRealityApp::TangoSetupConfig() {

	// Here, we'll configure the service to run in the way we'd want. For this
	// application, we'll start from the default configuration
	// (TANGO_CONFIG_DEFAULT). This enables basic motion tracking capabilities.
	tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
	if (tango_config_ == nullptr) {
		LOGE("AugmentedRealityApp: Failed to get default config form");
		return TANGO_ERROR;
	}

	int ret = TangoSupport_createImageBufferManager(TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP, 1280, 720, &yuv_manager_);
	if (ret != TANGO_SUCCESS) {
		LOGE("AugmentedRealityApp: Failed to create an image buffer manager"
				"code: %d", ret);
		return ret;
	}

	// Set auto-recovery for motion tracking as requested by the user.
	ret = TangoConfig_setBool(tango_config_, "config_enable_auto_recovery",
			true);
	if (ret != TANGO_SUCCESS) {
		LOGE("AugmentedRealityApp: config_enable_auto_recovery() failed with error"
				"code: %d", ret);
		return ret;
	}

	// Enable color camera from config.
	ret = TangoConfig_setBool(tango_config_, "config_enable_color_camera", true);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: config_enable_color_camera() failed with error"
				"code: %d",
				ret);
		return ret;
	}

	// Low latency IMU integration enables aggressive integration of the latest
	// inertial measurements to provide lower latency pose estimates. This will
	// improve the AR experience.
	ret = TangoConfig_setBool(tango_config_,
			"config_enable_low_latency_imu_integration", true);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: config_enable_low_latency_imu_integration() "
				"failed with error code: %d",
				ret);
		return ret;
	}

	ret = TangoService_connectOnFrameAvailable(TANGO_CAMERA_COLOR, this,
	                                             OnFrameAvailableRouter);
	if (ret != TANGO_SUCCESS) {
		LOGE("AugmentedRealityApp: Error connecting color frame %d", ret);
		return ret;
	}

	// Get TangoCore version string from service.
	char tango_core_version[kVersionStringLength];
	ret = TangoConfig_getString(
			tango_config_, "tango_service_library_version",
			tango_core_version, kVersionStringLength);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: get tango core version failed with error"
				"code: %d",
				ret);
		return ret;
	}
	tango_core_version_string_ = tango_core_version;

	return ret;
}

void AugmentedRealityApp::OnFrameAvailable(const TangoImageBuffer* buffer) {

	TangoSupport_updateImageBuffer(yuv_manager_, buffer);

	if (yuv_drawable_ == NULL){
		return;
	}

	if (yuv_drawable_->GetTextureId() == 0) {
		LOGE("AugmentedRealityApp::yuv texture id not valid");
		return;
	}

	if (buffer->format != TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP) {
		LOGE("AugmentedRealityApp::yuv texture format is not supported by this app");
		return;
	}

	// The memory needs to be allocated after we get the first frame because we
	// need to know the size of the image.
	if (!is_yuv_texture_available_) {
		yuv_width_ = buffer->width;
		yuv_height_ = buffer->height;
		uv_buffer_offset_ = yuv_width_ * yuv_height_;

		yuv_size_ = yuv_width_ * yuv_height_ + yuv_width_ * yuv_height_ / 2;

		// Reserve and resize the buffer size for RGB and YUV data.
		yuv_buffer_.resize(yuv_size_);
		yuv_temp_buffer_.resize(yuv_size_);
		rgb_buffer_.resize(yuv_width_ * yuv_height_ * 3);

		AllocateTexture(yuv_drawable_->GetTextureId(), yuv_width_, yuv_height_);
		is_yuv_texture_available_ = true;
	}
}

void AugmentedRealityApp::AllocateTexture(GLuint texture_id, int width,
		int height) {

	glBindTexture(GL_TEXTURE_2D, texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
			GL_UNSIGNED_BYTE, rgb_buffer_.data());
}

void AugmentedRealityApp::DeleteDrawables() {
	delete yuv_drawable_;
	yuv_drawable_ = NULL;
}

int AugmentedRealityApp::TangoConnectCallbacks() {
	// Attach onEventAvailable callback.
	// The callback will be called after the service is connected.
	int ret = TangoService_connectOnTangoEvent(onTangoEventAvailableRouter);
	if (ret != TANGO_SUCCESS) {
		LOGE("AugmentedRealityApp: Failed to connect to event callback with error"
				"code: %d", ret);
		return ret;
	}

	return ret;
}

// Connect to Tango Service, service will start running, and
// pose can be queried.
int AugmentedRealityApp::TangoConnect() {
	TangoErrorType ret = TangoService_connect(this, tango_config_);
	if (ret != TANGO_SUCCESS) {
		LOGE("AugmentedRealityApp: Failed to connect to the Tango service with"
				"error code: %d", ret);
		return ret;
	}

	ret = UpdateExtrinsics();
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: Failed to query sensor extrinsic with error "
				"code: %d",
				ret);
		return ret;
	}
	return ret;
}

void AugmentedRealityApp::TangoDisconnect() {
	// When disconnecting from the Tango Service, it is important to make sure to
	// free your configuration object. Note that disconnecting from the service,
	// resets all configuration, and disconnects all callbacks. If an application
	// resumes after disconnecting, it must re-register configuration and
	// callbacks with the service.
	TangoConfig_free(tango_config_);
	tango_config_ = nullptr;
	TangoService_disconnect();
}

void AugmentedRealityApp::TangoResetMotionTracking() {
	TangoService_resetMotionTracking();
}

void AugmentedRealityApp::InitializeGLContent() {

	if (yuv_drawable_ != NULL) {
		this->DeleteDrawables();
	}
	yuv_drawable_ = new YUVDrawable();

	// Connect color camera texture. TangoService_connectTextureId expects a valid
	// texture id from the caller, so we will need to wait until the GL content is
	// properly allocated.
	TangoErrorType ret = TangoService_connectTextureId(
			TANGO_CAMERA_COLOR, yuv_drawable_->GetTextureId(), this,
			onTextureAvailableRouter);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: Failed to connect the texture id with error"
				"code: %d",
				ret);
	}
}

void AugmentedRealityApp::SetViewPort(int width, int height) {
	// Query intrinsics for the color camera from the Tango Service, because we
	// want to match the virtual render camera's intrinsics to the physical
	// camera, we will compute the actually projection matrix and the view port
	// ratio for the render.
	TangoErrorType ret = TangoService_getCameraIntrinsics(
			TANGO_CAMERA_COLOR, &color_camera_intrinsics_);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: Failed to get camera intrinsics with error"
				"code: %d",
				ret);
	}

	float image_width = static_cast<float>(color_camera_intrinsics_.width);
	float image_height = static_cast<float>(color_camera_intrinsics_.height);
	float fx = static_cast<float>(color_camera_intrinsics_.fx);
	float fy = static_cast<float>(color_camera_intrinsics_.fy);
	float cx = static_cast<float>(color_camera_intrinsics_.cx);
	float cy = static_cast<float>(color_camera_intrinsics_.cy);

	float image_plane_ratio = image_height / image_width;
	float image_plane_distance = 2.0f * fx / image_width;

	glm::mat4 projection_mat_ar =
			tango_gl::Camera::ProjectionMatrixForCameraIntrinsics(
					image_width, image_height, fx, fy, cx, cy, kArCameraNearClippingPlane,
					kArCameraFarClippingPlane);

	float screen_ratio = static_cast<float>(height) / static_cast<float>(width);
	// In the following code, we place the view port at (0, 0) from the bottom
	// left corner of the screen. By placing it at (0,0), the view port may not
	// be exactly centered on the screen. However, this won't affect AR
	// visualization as the correct registration of AR objects relies on the
	// aspect ratio of the screen and video overlay, but not the position of the
	// view port.
	//
	// To place the view port in the center of the screen, please use following
	// code:
	//
	// if (image_plane_ratio < screen_ratio) {
	//   glViewport(-(h / image_plane_ratio - w) / 2, 0,
	//              h / image_plane_ratio, h);
	// } else {
	//   glViewport(0, -(w * image_plane_ratio - h) / 2, w,
	//              w * image_plane_ratio);
	// }

	if (image_plane_ratio < screen_ratio) {
		glViewport(0, 0, height / image_plane_ratio, height);
	} else {
		glViewport(0, 0, width, width * image_plane_ratio);
	}
}

void AugmentedRealityApp::Render(visual_odometry::STAM* stam) {

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	TangoImageBuffer* yuv = new TangoImageBuffer();
	TangoSupport_getLatestImageBuffer(yuv_manager_, &yuv);

	cv::Mat yuv_frame;
	yuv_frame.create(yuv->height*3/2, yuv->width, CV_8UC1);
	memcpy(yuv_frame.data, yuv->data, yuv->height*3/2*yuv->width); // yuv image

	//getting only the Y channel (many of the functions like face detect and align only needs the grayscale image)
	cv::Mat y_frame(yuv->height, yuv->width, CV_8UC1);
	y_frame.data = yuv_frame.data;

	cv::Mat uv_frame = cv::Mat((y_frame.rows / 2), (y_frame.cols / 2), CV_8UC2);

	uv_frame.data = &yuv_frame.data[y_frame.rows * y_frame.cols];

	cv::resize(uv_frame, uv_frame, cv::Size(uv_frame.cols/2, uv_frame.rows/2));
	cv::resize(y_frame, y_frame, cv::Size(y_frame.cols/2, y_frame.rows/2));

	///
	if (!stam->isCalibrated()) {
		Profiler profiler;
		profiler.startSampling();
		stam->initFromChessboard(y_frame, uv_frame, cv::Size(7, 3), 100);
		profiler.endSampling();
		profiler.print("initFromChessboard ", -1);
	}
	else {
		static Profiler profilerProcess;
		profilerProcess.startSampling();
		stam->process(y_frame, uv_frame);
		profilerProcess.endSampling();
		profilerProcess.print("process ", -1);

		static int i = 1;

		if (i % 100 == 0) {
			static Profiler profilerOptmise;
			profilerOptmise.startSampling();
			stam->optimise();
			profilerOptmise.endSampling();
			profilerOptmise.print("optimise ", -1);
		}

		i++;
	}
	///

	cv::resize(uv_frame, uv_frame, cv::Size(uv_frame.cols*2, uv_frame.rows*2));
	cv::resize(y_frame, y_frame, cv::Size(y_frame.cols*2, y_frame.rows*2));

	int yFrameSize = y_frame.rows * y_frame.cols;
	memcpy(yuv_frame.data, y_frame.data, yFrameSize);

	int uvFrameSize = uv_frame.rows * uv_frame.cols;
	memcpy(&yuv_frame.data[yFrameSize], uv_frame.data, uvFrameSize*2);

	memcpy(&yuv_temp_buffer_[0], yuv_frame.data, yuv_size_);
	swap_buffer_signal_ = true;
	RenderYUV();
}

void AugmentedRealityApp::RenderYUV() {
  if (!is_yuv_texture_available_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(yuv_buffer_mutex_);
    if (swap_buffer_signal_) {
      std::swap(yuv_buffer_, yuv_temp_buffer_);
      swap_buffer_signal_ = false;
    }
  }

  for (size_t i = 0; i < yuv_height_; ++i) {
    for (size_t j = 0; j < yuv_width_; ++j) {
      size_t x_index = j;
      if (j % 2 != 0) {
        x_index = j - 1;
      }

      size_t rgb_index = (i * yuv_width_ + j) * 3;

      // The YUV texture format is NV21,
      // yuv_buffer_ buffer layout:
      //   [y0, y1, y2, ..., yn, v0, u0, v1, u1, ..., v(n/4), u(n/4)]
      Yuv2Rgb(
          yuv_buffer_[i * yuv_width_ + j],
          yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index + 1],
          yuv_buffer_[uv_buffer_offset_ + (i / 2) * yuv_width_ + x_index],
          &rgb_buffer_[rgb_index], &rgb_buffer_[rgb_index + 1],
          &rgb_buffer_[rgb_index + 2]);
    }
  }

  glBindTexture(GL_TEXTURE_2D, yuv_drawable_->GetTextureId());
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, yuv_width_, yuv_height_, 0, GL_RGB,
               GL_UNSIGNED_BYTE, rgb_buffer_.data());

  yuv_drawable_->Render(glm::mat4(1.0f), glm::mat4(1.0f));
}

void AugmentedRealityApp::FreeGLContent() {
	is_yuv_texture_available_ = false;
	swap_buffer_signal_ = false;
	rgb_buffer_.clear();
	yuv_buffer_.clear();
	yuv_temp_buffer_.clear();
	TangoSupport_freeImageBufferManager(yuv_manager_);
	this->DeleteDrawables();
}

std::string AugmentedRealityApp::GetPoseString() {
	std::lock_guard<std::mutex> lock(pose_mutex_);
	return pose_data_.GetPoseDebugString();
}

std::string AugmentedRealityApp::GetEventString() {
	std::lock_guard<std::mutex> lock(tango_event_mutex_);
	return tango_event_data_.GetTangoEventString().c_str();
}

std::string AugmentedRealityApp::GetVersionString() {
	return tango_core_version_string_.c_str();
}

glm::mat4 AugmentedRealityApp::GetPoseMatrixAtTimestamp(double timstamp) {
	TangoPoseData pose_start_service_T_device;
	TangoCoordinateFramePair frame_pair;
	frame_pair.base = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
	frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
	TangoErrorType status = TangoService_getPoseAtTime(
			timstamp, frame_pair, &pose_start_service_T_device);
	if (status != TANGO_SUCCESS) {
		LOGE(
				"AugmentedRealityApp: Failed to get transform between the Start of "
				"service and device frames at timstamp %lf",
				timstamp);
	}

	{
		std::lock_guard<std::mutex> lock(pose_mutex_);
		pose_data_.UpdatePose(&pose_start_service_T_device);
	}

	if (pose_start_service_T_device.status_code != TANGO_POSE_VALID) {
		return glm::mat4(1.0f);
	}
	return pose_data_.GetMatrixFromPose(pose_start_service_T_device);
}

TangoErrorType AugmentedRealityApp::UpdateExtrinsics() {
	TangoErrorType ret;
	TangoPoseData pose_data;
	TangoCoordinateFramePair frame_pair;

	// TangoService_getPoseAtTime function is used for query device extrinsics
	// as well. We use timestamp 0.0 and the target frame pair to get the
	// extrinsics from the sensors.
	//
	// Get device with respect to imu transformation matrix.
	frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
	frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
	ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"PointCloudApp: Failed to get transform between the IMU frame and "
				"device frames");
		return ret;
	}
	pose_data_.SetImuTDevice(pose_data_.GetMatrixFromPose(pose_data));

	// Get color camera with respect to imu transformation matrix.
	frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
	frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
	ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_data);
	if (ret != TANGO_SUCCESS) {
		LOGE(
				"PointCloudApp: Failed to get transform between the color camera frame "
				"and device frames");
		return ret;
	}
	pose_data_.SetImuTColorCamera(pose_data_.GetMatrixFromPose(pose_data));
	return ret;
}

void AugmentedRealityApp::RequestRender() {
	if (calling_activity_obj_ == nullptr || on_demand_render_ == nullptr) {
		LOGE("Can not reference Activity to request render");
		return;
	}

	// Here, we notify the Java activity that we'd like it to trigger a render.
	JNIEnv* env;
	java_vm_->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
	env->CallVoidMethod(calling_activity_obj_, on_demand_render_);
}

}  // namespace tango_augmented_reality
