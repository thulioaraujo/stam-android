package com.voxar.stam.views;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import android.app.Activity;
import android.content.Intent;
import android.content.res.AssetManager;
import android.graphics.Point;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.view.Display;
import android.view.WindowManager;
import android.widget.Toast;

import com.voxar.stam.R;
import com.voxar.stam.controllers.StamTangoRender;
import com.voxar.stam.util.StamTangoJNINative;
import com.voxar.stam.util.Util;

/**
 * @author Th?lio Ara?jo(tsla@cin.ufpe.br)
 * @version 1.0
 * @since 2015/10/29
 */
public class MainStamTangoActivity extends Activity {

	// Constants to facilitate the file loading throughout the application
	private static final String CHALLENGE_FILE = "challenge.txt";
	private static final String INTRINSICS_ONSITE_FILE = "intrinsicsOnSite.xml";

	// The user has not given permission to use Motion Tracking functionality.
	// private static final int TANGO_NO_MOTION_TRACKING_PERMISSION = -3;
	// The input argument is invalid.
	// private static final int TANGO_INVALID = -2;
	// This error code denotes some sort of hard error occurred.
	// private static final int TANGO_ERROR = -1;
	// This code indicates success.
	// private static final int TANGO_SUCCESS = 0;

	// Tag for debug logging.
	// private static final String TAG = "STAM VoxarLabs";

	// Motion Tracking permission request action.
	private static final String MOTION_TRACKING_PERMISSION_ACTION = "android.intent.action.REQUEST_TANGO_PERMISSION";

	// Key string for requesting and checking Motion Tracking permission.
	private static final String MOTION_TRACKING_PERMISSION = "MOTION_TRACKING_PERMISSION";

	// GLSurfaceView and its renderer, all of the graphic content is rendered
	// through OpenGL ES 2.0 in the native code.
	private StamTangoRender mRenderer;
	private GLSurfaceView mGLView;

	// A flag to check if the Tango Service is connected. This flag avoids the
	// program attempting to disconnect from the service while it is not
	// connected.This is especially important in the onPause() callback for the
	// activity class.
	private boolean mIsConnectedService = false;

	// Screen size for normalizing the touch input for orbiting the render
	// camera.
	private Point mScreenSize = new Point();

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setTitle(R.string.app_name);

		getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
				WindowManager.LayoutParams.FLAG_FULLSCREEN);

		// Querying screen size, used for computing the normalized touch point.
		Display display = getWindowManager().getDefaultDisplay();
		display.getSize(mScreenSize);

		setContentView(R.layout.activity_main_stam);

		// OpenGL view where all of the graphics are drawn
		mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

		// Configure OpenGL renderer
		mGLView.setEGLContextClientVersion(2);
		
		// Configure OpenGL renderer. The RENDERMODE_WHEN_DIRTY is set
		// explicitly
		// for reducing the CPU load. The request render function call is
		// triggered
		// by the onTextureAvailable callback from the Tango Service in the
		// native
		// code.
		mRenderer = new StamTangoRender();
		mGLView.setRenderer(mRenderer);
		mGLView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);

		// Initialize Tango Service, this function starts the communication
		// between the application and Tango Service.
		// The activity object is used for checking if the API version is
		// outdated.
		StamTangoJNINative.initialize(this);
		try {
			moveConfigFileToLocalStorage();
		} catch (IOException e) {
			e.printStackTrace();
			Toast.makeText(this,
					"There was a problem when configuring this app",
					Toast.LENGTH_SHORT).show();
		}
		StamTangoJNINative.initializeStam(this.getFilesDir() + "/");
	}

	@Override
	protected void onResume() {
		super.onResume();
		mGLView.onResume();

		// In the onResume function, we first check if the
		// MOTION_TRACKING_PERMISSION is
		// granted to this application, if not, we send a permission intent to
		// the Tango Service to launch the permission activity.
		// Note that the onPause() callback will be called once the permission
		// activity is foregrounded.
		if (!Util.hasPermission(getApplicationContext(),
				MOTION_TRACKING_PERMISSION)) {
			getMotionTrackingPermission();
		} else {
			// If motion tracking permission is granted to the application, we
			// can
			// connect to the Tango Service. For this example, we'll be calling
			// through the JNI to the C++ code that actually interfaces with the
			// service.

			// Setup the configuration for the TangoService.
			StamTangoJNINative.setupConfig();

			// Connect the onPoseAvailable callback.
			StamTangoJNINative.connectCallbacks();

			// Connect to Tango Service.
			// This function will start the Tango Service pipeline, in this
			// case,
			// it will start Motion Tracking.
			StamTangoJNINative.connect();

			// Set the connected service flag to true.
			mIsConnectedService = true;
		}
	}

	@Override
	protected void onPause() {
		super.onPause();
		mGLView.onPause();
		StamTangoJNINative.freeGLContent();

		// If the service is connected, we disconnect it here.
		if (mIsConnectedService) {
			mIsConnectedService = false;
			// Disconnect from Tango Service, release all the resources that the
			// app is
			// holding from Tango Service.
			StamTangoJNINative.disconnect();
		}
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		if (mIsConnectedService) {
			mIsConnectedService = false;
			StamTangoJNINative.disconnect();
		}
	}

	// Call the permission intent for the Tango Service to ask for motion
	// tracking
	// permissions. All permission types can be found here:
	// https://developers.google.com/project-tango/apis/c/c-user-permissions
	private void getMotionTrackingPermission() {
		Intent intent = new Intent();
		intent.setAction(MOTION_TRACKING_PERMISSION_ACTION);
		intent.putExtra("PERMISSIONTYPE", MOTION_TRACKING_PERMISSION);

		// After the permission activity is dismissed, we will receive a
		// callback
		// function onActivityResult() with user's result.
		startActivityForResult(intent, 0);
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		// The result of the permission activity.
		//
		// Note that when the permission activity is dismissed, the
		// MotionTrackingActivity's onResume() callback is called. As the
		// TangoService is connected in the onResume() function, we do not call
		// connect here.
		if (requestCode == 0) {
			if (resultCode == RESULT_CANCELED) {
				mIsConnectedService = false;
				finish();
			}
		}
	}

	// Request render on the glSurfaceView. This function is called from the
	// native code, and it is triggered from the onTextureAvailable callback
	// from
	// the Tango Service.
	public void requestRender() {
		mGLView.requestRender();
	}

	/**
	 * Assembles and returns the path to the LBF configuration file.
	 * 
	 * @return String The path to the LBF file.
	 */
	public String getLbfPath() {
		return this.getFilesDir() + "/" + CHALLENGE_FILE;
	}

	/**
	 * Assembles and returns the path to the LBF regressor configuration file.
	 * 
	 * @return String The path to the LBF regressor file.
	 */
	public String getLbfRegressorPath() {
		return this.getFilesDir() + "/" + INTRINSICS_ONSITE_FILE;
	}

	/**
	 * Moves the application configuration files from the compressed apk to the
	 * device storage, so they can be read at JNI level.
	 * 
	 * @throws IOException
	 *             If something goes wrong when opening or reading the files.
	 */
	public void moveConfigFileToLocalStorage() throws IOException {

		File fileChallenge = new File(getLbfPath());
		File fileIntrinsincsOnSite = new File(getLbfRegressorPath());

		AssetManager assetManager = this.getAssets();

		checkFileOnLocalStorage(assetManager, fileChallenge, CHALLENGE_FILE);
		checkFileOnLocalStorage(assetManager, fileIntrinsincsOnSite,
				INTRINSICS_ONSITE_FILE);
	}

	/**
	 * Check if the file has already been transferred from the assets to a local
	 * storage folder.
	 * 
	 * @param assetManager
	 *            The application's asset manager.
	 * @param file
	 *            The file that will be checked.
	 * @throws IOException
	 *             If something goes wrong when using the files.
	 */
	private void checkFileOnLocalStorage(AssetManager assetManager, File file,
			String fileName) throws IOException {

		if (!file.exists()) {
			FileOutputStream fos = new FileOutputStream(file.getPath());

			InputStream inputStream = null;
			final byte[] buffer = new byte[16384];
			int read;

			try {
				inputStream = assetManager.open(fileName);

				while ((read = inputStream.read(buffer)) != -1) {
					fos.write(buffer, 0, read);
				}

				fos.flush();
			} finally {
				fos.close();
				inputStream.close();
			}
		}
	}
}