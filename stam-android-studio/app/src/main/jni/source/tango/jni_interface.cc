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

#define GLM_FORCE_RADIANS

#include <jni.h>
#include <header/stam/STAM.h>
#include <header/tango/augmented_reality_app.h>

static tango_augmented_reality::AugmentedRealityApp app;

#ifdef __cplusplus
extern "C" {
#endif
jint JNI_OnLoad(JavaVM* vm, void*) {
	// We need to store a reference to the Java VM so that we can call into the
	// Java layer to trigger rendering.
	app.SetJavaVM(vm);
	return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_initialize(
		JNIEnv* env, jobject, jobject activity) {
	app.TangoInitialize(env, activity);
}

JNIEXPORT jlong JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_initializeStam(
		JNIEnv* env, jobject, jstring resourcesPath) {

	const char *filePathNative = env->GetStringUTFChars(resourcesPath, 0);
	SCENE = 4;
	visual_odometry::STAM *stam = new visual_odometry::STAM();
	stam->init(filePathNative);

	env->ReleaseStringUTFChars(resourcesPath, filePathNative);
	return (long) stam;
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_deleteStamPointer(
		JNIEnv* env, jobject, jlong stamPointer) {

	visual_odometry::STAM *stam = (visual_odometry::STAM*) stamPointer;
	delete stam;
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_calibrateChessBoard(
		JNIEnv* env, jobject, jlong stamPointer, jboolean calibrate) {

	visual_odometry::STAM *stam = (visual_odometry::STAM*) stamPointer;
	stam->calibrate = calibrate;
}

JNIEXPORT jint JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_setupConfig(
		JNIEnv*, jobject) {
	return app.TangoSetupConfig();
}

JNIEXPORT jint JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_connect(
		JNIEnv*, jobject) {
	return app.TangoConnect();
}

JNIEXPORT jint JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_connectCallbacks(
		JNIEnv*, jobject) {
	int ret = app.TangoConnectCallbacks();
	return ret;
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_disconnect(
		JNIEnv*, jobject) {
	app.TangoDisconnect();
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_resetMotionTracking(
		JNIEnv*, jobject) {
	app.TangoResetMotionTracking();
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_initGlContent(
		JNIEnv*, jobject) {
	app.InitializeGLContent();
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_setupGraphic(
		JNIEnv*, jobject, jint width, jint height) {
	app.SetViewPort(width, height);
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_render(
		JNIEnv*, jobject, jlong stamPointer) {

	visual_odometry::STAM *stam = (visual_odometry::STAM*) stamPointer;
	app.Render(stam);
}

JNIEXPORT void JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_freeGLContent(
		JNIEnv*, jobject) {
	app.FreeGLContent();
}

JNIEXPORT jstring JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_getPoseString(
		JNIEnv* env, jobject) {
	return (env)->NewStringUTF(app.GetPoseString().c_str());
}

JNIEXPORT jstring JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_getEventString(
		JNIEnv* env, jobject) {
	return (env)->NewStringUTF(app.GetEventString().c_str());
}

JNIEXPORT jstring JNICALL
Java_com_voxar_stam_util_StamTangoJNINative_getVersionNumber(
		JNIEnv* env, jobject) {
	return (env)->NewStringUTF(app.GetVersionString().c_str());
}

#ifdef __cplusplus
}
#endif
