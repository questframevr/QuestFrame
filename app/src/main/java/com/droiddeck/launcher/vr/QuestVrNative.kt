package com.droiddeck.launcher.vr

import android.app.Activity

internal object QuestVrNative {
    val available: Boolean = runCatching { System.loadLibrary("questvr") }.isSuccess

    @JvmStatic external fun nativeStart(activity: Activity, mode: Int, theaterWidth: Int, theaterHeight: Int): Boolean
    @JvmStatic external fun nativeStop()
    @JvmStatic external fun nativeRecenter()
    @JvmStatic external fun nativeSetScreen(distance: Float, width: Float)
}
