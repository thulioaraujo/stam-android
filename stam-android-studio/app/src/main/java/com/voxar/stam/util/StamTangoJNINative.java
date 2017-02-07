package com.voxar.stam.util;

import com.voxar.stam.views.MainStamTangoActivity;

//Interfaces between native C++ code and Java code.
public class StamTangoJNINative {
	static {
		System.loadLibrary("stam_tango");
	}

	// Initialize the Tango Service, this function starts the communication
	// between the application and Tango Service.
	// The activity object is used for checking if the API version is outdated.
	public static native int initialize(MainStamTangoActivity activity);
	
	// Initialize the Tango Service, this function starts the communication
	// between the application and Tango Service.
	// The activity object is used for checking if the API version is outdated.
	public static native int initializeStam(String resourcesPath);

	// Setup the configuration file of the Tango Service. We are also setting up
	// the auto-recovery option from here.
	public static native int setupConfig();

	// Connect the onPoseAvailable callback.
	public static native int connectCallbacks();

	// Connect to the Tango Service.
	// This function will start the Tango Service pipeline, in this case, it
	// will
	// start Motion Tracking.
	public static native int connect();

	// Disconnect from the Tango Service, release all the resources that the app
	// is
	// holding from the Tango Service.
	public static native void disconnect();

	// Release all OpenGL resources that are allocated from the program.
	public static native void freeGLContent();

	// Allocate OpenGL resources for rendering.
	public static native void initGlContent();

	// Setup the view port width and height.
	public static native void setupGraphic(int width, int height);

	// Main render loop.
	public static native void render();

	// Set the render camera's viewing angle:
	// first person, third person, or top down.
	public static native void setCamera(int cameraIndex);

	// Explicitly reset motion tracking and restart the pipeline.
	// Note that this will cause motion tracking to re-initialize.
	public static native void resetMotionTracking();

	// Get the latest pose string from our application for display in our debug
	// UI.
	public static native String getPoseString();

	// Get the latest event string from our application for display in our debug
	// UI.
	public static native String getEventString();

	// Get the TangoCore version from our application for display in our debug
	// UI.
	public static native String getVersionNumber();

	// Pass touch events to the native layer.
	public static native void onTouchEvent(int touchCount, int event0,
			float x0, float y0, float x1, float y1);
}
