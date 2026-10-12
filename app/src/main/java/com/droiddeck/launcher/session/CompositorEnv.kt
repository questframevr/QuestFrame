package com.droiddeck.launcher.session

import android.content.Context
import com.droiddeck.launcher.core.FileUtils

/**
 * Compositor switches the session's extra-environment file (droiddeck-env, see
 * [SessionService.envSwitchFile]) can set. The compositor lives in the app process, so the
 * guest environment that file feeds cannot reach it: these are read here, before it starts.
 */
object CompositorEnv {
    const val UBWC = "DROIDDECK_WAYLAND_UBWC"

    /**
     * What the last `DROIDDECK_WAYLAND_UBWC=` line in [lines] says: false for 0 (the compositor
     * advertises LINEAR only, so games hand it linear buffers), true for 1, null when no line
     * gives a 0 or a 1 (the default, compressed buffers on, stays as it is).
     */
    fun ubwc(lines: List<String>): Boolean? {
        var result: Boolean? = null
        for (raw in lines) {
            val line = raw.trim()
            if (!line.startsWith("$UBWC=")) continue
            when (line.substringAfter('=').trim()) {
                "0" -> result = false
                "1" -> result = true
            }
        }
        return result
    }

    fun ubwc(context: Context): Boolean? {
        val file = SessionService.envSwitchFile(context)?.takeIf { it.isFile } ?: return null
        return ubwc(FileUtils.readString(file)?.lines().orEmpty())
    }

    /** Lines that set a variable for the compositor's own Vulkan driver, not the guest. */
    const val DRIVER_PREFIX = "DROIDDECK_COMPOSITOR_"

    /**
     * What `DROIDDECK_COMPOSITOR_<NAME>=value` lines in [lines] ask for, as <NAME> to value (the
     * last line for a name wins). The compositor's Turnip reads its options (TU_DEBUG,
     * FD_DEV_FEATURES, ...) from the app process's environment, which nothing else can set.
     */
    fun driverEnv(lines: List<String>): Map<String, String> {
        val result = LinkedHashMap<String, String>()
        for (raw in lines) {
            val line = raw.trim()
            if (!line.startsWith(DRIVER_PREFIX) || '=' !in line) continue
            val name = line.substring(DRIVER_PREFIX.length).substringBefore('=').trim()
            if (name.isNotEmpty() && name.all { it.isLetterOrDigit() || it == '_' }) {
                result[name] = line.substringAfter('=').trim()
            }
        }
        return result
    }

    /** Sets the [driverEnv] variables in this process; call before the compositor loads its driver. */
    fun applyDriverEnv(context: Context): Map<String, String> {
        val file = SessionService.envSwitchFile(context)?.takeIf { it.isFile } ?: return emptyMap()
        val env = driverEnv(FileUtils.readString(file)?.lines().orEmpty())
        for ((name, value) in env) android.system.Os.setenv(name, value, true)
        return env
    }
}
