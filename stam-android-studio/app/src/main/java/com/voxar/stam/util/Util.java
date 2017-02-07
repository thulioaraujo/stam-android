package com.voxar.stam.util;

import android.content.Context;
import android.database.Cursor;
import android.net.Uri;

//Util class provides handy utility functions.
public class Util {

	// Checks if the calling app has the specified permission.
	// It is recommended that an app check if it has a permission before trying
	// to request it; this will save time by avoiding re-requesting permissions
	// that have already been granted.

	// @param context The context of the calling app.
	// @param permissionType The type of permission to request; either
	// PERMISSIONTYPE_MOTION_TRACKING or PERMISSIONTYPE_ADF_LOAD_SAVE.
	// @return boolean Whether or not the permission was already granted.
	public static boolean hasPermission(Context context, String permissionType) {
		Uri uri = Uri
				.parse("content://com.google.atap.tango.PermissionStatusProvider/"
						+ permissionType);
		Cursor cursor = context.getContentResolver().query(uri, null, null,
				null, null);
		if (cursor == null) {
			return false;
		} else {
			return true;
		}
	}
}