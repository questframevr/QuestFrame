package com.droiddeck.launcher.vr

import android.app.Activity
import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.view.Surface
import android.view.WindowManager
import android.widget.Toast
import com.droiddeck.launcher.MainActivity
import com.droiddeck.launcher.R
import com.droiddeck.launcher.SessionActivity
import com.droiddeck.launcher.input.PadBridge
import com.droiddeck.launcher.input.PadState
import com.droiddeck.launcher.runtime.LinuxRuntime
import com.droiddeck.launcher.session.SessionPrefs
import com.droiddeck.launcher.session.SessionService
import com.droiddeck.launcher.session.SessionState
import com.droiddeck.launcher.ui.Themes
import com.droiddeck.launcher.wayland.CompositorHost
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

class QuestVrActivity : Activity() {
    private val ui = Handler(Looper.getMainLooper())
    private var mode = MODE_HOME
    private var started = false
    private var theaterSurface: Surface? = null
    private var panel: VrPanel? = null

    private var pad: PadBridge? = null
    private var ownPad = false
    private val lastPad = FloatArray(8)

    private var screenDistance = 2.6f
    private var screenWidth = 3.2f

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        mode = intent.getIntExtra(EXTRA_MODE, MODE_HOME)
        val prefs = getSharedPreferences(PREFS, MODE_PRIVATE)
        screenDistance = prefs.getFloat("screenDistance", screenDistance)
        screenWidth = prefs.getFloat("screenWidth", screenWidth)
        when {
            !supported(this) -> fail(getString(R.string.vr_not_headset))
            !QuestVrNative.available -> fail(getString(R.string.vr_lib_missing))
        }
    }

    override fun onResume() {
        super.onResume()
        if (started || isFinishing || !QuestVrNative.available || !supported(this)) return
        val (w, h) = SessionState.outputSize
        started = QuestVrNative.nativeStart(this, mode, w, h)
        Log.i(TAG, "native session start (${when (mode) { MODE_THEATER -> "theater ${w}x$h"; MODE_STEAMVR -> "SteamVR"; else -> "home" }}): $started")
        if (!started) fail(getString(R.string.vr_start_failed))
        else {
            QuestVrNative.nativeSetScreen(screenDistance, screenWidth)
            if (mode != MODE_HOME) hold(this)
            if (mode == MODE_STEAMVR) startSteamVr()
        }
    }

    override fun onDestroy() {
        releaseSurfaces()
        releasePad()
        if (started) {
            QuestVrNative.nativeStop()
            started = false
            if (mode != MODE_HOME) release(this)
        }
        super.onDestroy()
    }

    @Suppress("unused")
    fun onTheaterSurface(surface: Surface, width: Int, height: Int) = onMain {
        if (CompositorHost.attachOverlay(surface)) {
            theaterSurface = surface
            openPad()
            Log.i(TAG, "theater screen ${width}x$height attached to the compositor")
        } else {
            Log.w(TAG, "no compositor running: nothing to show on the theater screen")
            Toast.makeText(this, R.string.vr_no_session, Toast.LENGTH_LONG).show()
        }
    }

    @Suppress("unused")
    fun onPanelSurface(surface: Surface, width: Int, height: Int) {
        ui.post { showPanel(surface, width, height) }
    }

    private fun showPanel(surface: Surface, width: Int, height: Int) {
        if (isDestroyed) return
        val palette = Themes.byId(SessionPrefs.theme(this))
        val buttons = buildList {
            if (mode == MODE_THEATER) {
                add(VrPanel.Button(getString(R.string.vr_panel_steam)) { tapSystem(guide = true) })
                add(VrPanel.Button(getString(R.string.vr_panel_qam)) { tapSystem(guide = false) })
            }
            add(VrPanel.Button(getString(R.string.vr_panel_recenter)) { QuestVrNative.nativeRecenter() })
            if (mode == MODE_THEATER) {
                add(VrPanel.Button(getString(R.string.vr_panel_closer)) { adjustScreen(distance = -0.3f) })
                add(VrPanel.Button(getString(R.string.vr_panel_farther)) { adjustScreen(distance = 0.3f) })
                add(VrPanel.Button(getString(R.string.vr_panel_smaller)) { adjustScreen(width = -0.3f) })
                add(VrPanel.Button(getString(R.string.vr_panel_bigger)) { adjustScreen(width = 0.3f) })
            }
            add(VrPanel.Button(getString(R.string.vr_panel_exit), primary = true) { leaveVr() })
        }
        panel = VrPanel(surface, width, height, palette, buttons).also { it.draw() }
    }

    @Suppress("unused")
    fun onSurfacesGone() = onMainAndWait { releaseSurfaces() }

    @Suppress("unused")
    fun onPanelPointer(state: Int, u: Float, v: Float) {
        ui.post { panel?.pointer(state, u, v) }
    }

    @Suppress("unused")
    fun onPad(state: FloatArray) {
        val bridge = pad ?: return
        val changed = !state.contentEquals(lastPad)
        state.copyInto(lastPad)
        val buttons = state[6].toInt()
        bridge.applyExternal({ s ->
            s.leftX = state[0]; s.leftY = state[1]; s.rightX = state[2]; s.rightY = state[3]
            s.leftTrigger = state[4]; s.rightTrigger = state[5]
            for (b in PAD_BUTTONS) s.press(b, buttons and (1 shl b) != 0)
        }, changed)
    }

    @Suppress("unused")
    fun onVrError(message: String) {
        Log.e(TAG, "VR error: $message")
        ui.post { fail(message) }
    }

    @Suppress("unused")
    fun onVrExit() {
        Log.i(TAG, "VR session over")
        ui.post { if (!isFinishing && !isDestroyed) finish() }
    }

    // --- Here -----------------------------------------------------------------------------

    private fun releaseSurfaces() {
        panel?.release()
        panel = null
        theaterSurface?.let { CompositorHost.detachOverlay(it) }
        theaterSurface = null
    }

    private fun openPad() {
        if (pad != null || !SessionState.running) return
        val shared = SessionState.padBridge
        if (shared != null) {
            pad = shared.also { it.start() }
        } else {
            pad = PadBridge(File(LinuxRuntime.sessionRoot(this), "dev/input")).also { it.start() }
            ownPad = true
        }
    }

    private fun releasePad() {
        val bridge = pad ?: return
        bridge.releaseAll()
        if (ownPad) bridge.stop()
        pad = null
        ownPad = false
    }

    private fun tapSystem(guide: Boolean) {
        val bridge = pad ?: return
        if (guide) bridge.setSystemButtons(true, false) else bridge.triggerQam()
        if (guide) ui.postDelayed({ pad?.setSystemButtons(false, false) }, 150)
    }

    private fun adjustScreen(distance: Float = 0f, width: Float = 0f) {
        screenDistance = (screenDistance + distance).coerceIn(1.2f, 6f)
        screenWidth = (screenWidth + width).coerceIn(1.2f, 8f)
        QuestVrNative.nativeSetScreen(screenDistance, screenWidth)
        getSharedPreferences(PREFS, MODE_PRIVATE).edit()
            .putFloat("screenDistance", screenDistance).putFloat("screenWidth", screenWidth).apply()
    }

    private fun onMain(block: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) block() else ui.post { if (!isDestroyed) block() }
    }

    private fun onMainAndWait(block: () -> Unit) {
        if (Looper.myLooper() == Looper.getMainLooper()) { block(); return }
        val done = CountDownLatch(1)
        ui.post { try { block() } finally { done.countDown() } }
        if (!done.await(2, TimeUnit.SECONDS)) Log.w(TAG, "main thread did not release the VR surfaces in time")
    }

    private fun leaveVr() {
        val back = if (SessionState.running) {
            Intent(this, SessionActivity::class.java).setAction(SessionService.ACTION_RESUME)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_SINGLE_TOP)
        } else {
            Intent(this, MainActivity::class.java).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP)
        }
        runCatching { startActivity(back) }.onFailure { Log.w(TAG, "could not return to QuestDeck", it) }
        finish()
    }

    private fun startSteamVr() {
        val ctx = applicationContext
        Thread {
            val cmd = "setsid sh /root/.local/share/questdeck-vr/run-steamvr.sh 1800 " +
                "> /root/.local/share/questdeck-vr/logs/launch.out 2>&1 < /dev/null &"
            val request = org.json.JSONObject().put("kind", "exec").put("timeout", 20)
                .put("argv", org.json.JSONArray(listOf("sh", "-c", cmd)))
            for (attempt in 1..20) {
                val result = runCatching { com.droiddeck.launcher.agent.AgentGuest.call(ctx, request, 30_000) }
                if (result.isSuccess) {
                    Log.i(TAG, "SteamVR start requested (attempt $attempt)")
                    return@Thread
                }
                Log.w(TAG, "SteamVR start not possible yet: ${result.exceptionOrNull()?.message}")
                Thread.sleep(3_000)
            }
        }.start()
    }

    private fun fail(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_LONG).show()
        finish()
    }

    companion object {
        private const val TAG = "QuestVR"
        private const val PREFS = "questvr"
        private const val EXTRA_MODE = "mode"
        const val MODE_HOME = 0
        const val MODE_THEATER = 1
        const val MODE_STEAMVR = 2

        @Volatile private var holds = 0

        val holdingSession: Boolean get() = holds > 0
        private fun hold(context: Context) {
            if (holds++ == 0) SessionService.setVrHolding(context, true)
        }
        private fun release(context: Context) {
            if (holds > 0 && --holds == 0) SessionService.setVrHolding(context, false)
        }

        private val PAD_BUTTONS = intArrayOf(
            PadState.A, PadState.B, PadState.X, PadState.Y, PadState.LB, PadState.RB,
            PadState.SELECT, PadState.START, PadState.L3, PadState.R3,
        )

        fun supported(context: Context): Boolean = context.packageManager.let {
            it.hasSystemFeature("oculus.hardware.standalone_vr") || it.hasSystemFeature("android.hardware.vr.headtracking")
        }

        fun intent(context: Context, mode: Int): Intent =
            Intent(context, QuestVrActivity::class.java).putExtra(EXTRA_MODE, mode)
                .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
    }
}
