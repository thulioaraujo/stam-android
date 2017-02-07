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

#ifndef HEADER_TANGO_AUGMENTED_REALITY_APP_H_
#define HEADER_TANGO_AUGMENTED_REALITY_APP_H_

#include <jni.h>
#include <memory>
#include <atomic>
#include <mutex>

#include <tango_client_api.h>  // NOLINT
#include <tango_support_api.h>
#include <tango-gl/util.h>
#include <tango-gl/camera.h>
#include <tango-gl/video_overlay.h>

#include <header/stam/STAM.h>
#include <header/tango/yuv_drawable.h>
#include <header/tango/pose_data.h>
#include <header/tango/tango_event_data.h>


namespace tango_augmented_reality {

// AugmentedRealityApp handles the application lifecycle and resources.
class AugmentedRealityApp {
 public:
  // Constructor and deconstructor.
  AugmentedRealityApp();
  ~AugmentedRealityApp();

  // Initialize Tango Service, this function starts the communication
  // between the application and Tango Service.
  // The activity object is used for checking if the API version is outdated.
  int TangoInitialize(JNIEnv* env, jobject caller_activity);

  // Setup the configuration file for the Tango Service. We'll also se whether
  // we'd like auto-recover enabled.
  int TangoSetupConfig();

  // YUV data callback.
  void OnFrameAvailable(const TangoImageBuffer* buffer);

  // Connect the onPoseAvailable callback.
  int TangoConnectCallbacks();

  // Connect to Tango Service.
  // This function will start the Tango Service pipeline, in this case, it will
  // start Motion Tracking.
  int TangoConnect();

  // Disconnect from Tango Service, release all the resources that the app is
  // holding from Tango Service.
  void TangoDisconnect();

  // Explicitly reset motion tracking and restart the pipeline.
  // Note that this will cause motion tracking to re-initialize.
  void TangoResetMotionTracking();

  // Tango service event callback function for pose data. Called when new events
  // are available from the Tango Service.
  //
  // @param event: Tango event, caller allocated.
  void onTangoEventAvailable(const TangoEvent* event);

  // Tango service texture callback. Called when the texture is updated.
  //
  // @param id: camera Id of the updated camera.
  void onTextureAvailable(TangoCameraId id);

  // Allocate OpenGL resources for rendering, mainly initializing the Scene.
  void InitializeGLContent();

  // Setup the view port width and height.
  void SetViewPort(int width, int height);

  // Main render loop.
  void Render(visual_odometry::STAM* stam);

  // Release all OpenGL resources that allocate from the program.
  void FreeGLContent();

  // Retrun pose debug string.
  std::string GetPoseString();

  // Retrun Tango event debug string.
  std::string GetEventString();

  // Retrun Tango Service version string.
  std::string GetVersionString();

  // Cache the Java VM
  //
  // @JavaVM java_vm: the Java VM is using from the Java layer.
  void SetJavaVM(JavaVM* java_vm) { java_vm_ = java_vm; }

 private:
  // Get a pose in matrix format with extrinsics in OpenGl space.
  //
  // @param: timstamp, timestamp of the target pose.
  //
  // @return: pose in matrix format.
  glm::mat4 GetPoseMatrixAtTimestamp(double timstamp);

  // Query sensor/camera extrinsic from the Tango Service, the extrinsic is only
  // available after the service is connected.
  //
  // @return: error code.
  TangoErrorType UpdateExtrinsics();

  // Request the render function from Java layer.
  void RequestRender();

  // pose_data_ handles all pose onPoseAvailable callbacks, onPoseAvailable()
  // in this object will be routed to pose_data_ to handle.
  PoseData pose_data_;

  // Mutex for protecting the pose data. The pose data is shared between render
  // thread and TangoService callback thread.
  std::mutex pose_mutex_;

  // tango_event_data_ handles all Tango event callbacks,
  // onTangoEventAvailable() in this object will be routed to tango_event_data_
  // to handle.
  TangoEventData tango_event_data_;

  // tango_event_data_ is share between the UI thread we start for updating
  // debug
  // texts and the TangoService event callback thread. We keep event_mutex_ to
  // protect tango_event_data_.
  std::mutex tango_event_mutex_;

  // Tango configration file, this object is for configuring Tango Service setup
  // before connect to service. For example, we set the flag
  // config_enable_auto_recovery based user's input and then start Tango.
  TangoConfig tango_config_;

  // Device color camera intrinsics, these intrinsics value is used for
  // calculate the camera frustum and image aspect ratio. In the AR view, we
  // want to match the virtual camera's intrinsics to the actual physical camera
  // as close as possible.
  TangoCameraIntrinsics color_camera_intrinsics_;

  TangoSupportImageBufferManager *yuv_manager_;

  // Tango service version string.
  std::string tango_core_version_string_;

  // Cached Java VM, caller activity object and the request render method. These
  // variables are used for on demand render request from the onTextureAvailable
  // callback.
  JavaVM* java_vm_;
  jobject calling_activity_obj_;
  jmethodID on_demand_render_;

  // video_overlay_ render the camera video feedback onto the screen.
  YUVDrawable* yuv_drawable_;

  std::vector<uint8_t> yuv_buffer_;
  std::vector<uint8_t> yuv_temp_buffer_;
  std::vector<GLubyte> rgb_buffer_;

  std::atomic<bool> is_yuv_texture_available_;
  std::atomic<bool> swap_buffer_signal_;
  std::mutex yuv_buffer_mutex_;

  size_t yuv_width_;
  size_t yuv_height_;
  size_t yuv_size_;
  size_t uv_buffer_offset_;

  void AllocateTexture(GLuint texture_id, int width, int height);
  void RenderYUV();
  void DeleteDrawables();
};
}  // namespace tango_augmented_reality

#endif  // HEADER_TANGO_AUGMENTED_REALITY_APP_H_
