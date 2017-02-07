package com.voxar.stam.controllers;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.opengl.GLSurfaceView;

import com.voxar.stam.util.StamTangoJNINative;

//StamTangoRender renders graphic content. This includes the ground grid,
//camera frustum, camera axis, and trajectory based on the Tango device's pose.
public class StamTangoRender implements GLSurfaceView.Renderer {
	// Render loop of the Gl context.
	public void onDrawFrame(GL10 gl) {
		StamTangoJNINative.render();
	}

	// Called when the surface size changes.
	public void onSurfaceChanged(GL10 gl, int width, int height) {
		StamTangoJNINative.setupGraphic(width, height);
	}

	// Called when the surface is created or recreated.
	public void onSurfaceCreated(GL10 gl, EGLConfig config) {
		StamTangoJNINative.initGlContent();
	}
}
