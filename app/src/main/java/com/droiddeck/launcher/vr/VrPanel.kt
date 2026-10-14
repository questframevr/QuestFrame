package com.droiddeck.launcher.vr

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.PorterDuff
import android.graphics.RectF
import android.graphics.Typeface
import android.view.Surface
import androidx.compose.ui.graphics.toArgb
import com.droiddeck.launcher.ui.Palette

internal class VrPanel(
    private val surface: Surface,
    private val width: Int,
    private val height: Int,
    private val palette: Palette,
    private val buttons: List<Button>,
) {
    class Button(val label: String, val primary: Boolean = false, val onClick: () -> Unit)

    private var hover = -1
    private var pressed = -1
    private var pointerU = -1f
    private var pointerV = -1f
    private var released = false

    private val bg = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = palette.surface.toArgb() }
    private val border = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.STROKE; strokeWidth = 3f; color = palette.line2.toArgb() }
    private val fill = Paint(Paint.ANTI_ALIAS_FLAG)
    private val text = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textAlign = Paint.Align.CENTER
        textSize = height * 0.26f
        typeface = Typeface.create(Typeface.DEFAULT, Typeface.BOLD)
    }
    private val dot = Paint(Paint.ANTI_ALIAS_FLAG).apply { color = Color.WHITE }

    fun pointer(state: Int, u: Float, v: Float) {
        pointerU = u
        pointerV = v
        val at = if (state == 0) -1 else buttonAt(u, v)
        when (state) {
            2 -> pressed = at
            3 -> {
                val target = pressed
                pressed = -1
                if (target >= 0 && target == at) buttons[target].onClick()
            }
            0 -> pressed = -1
        }
        if (at != hover || state != 1) {
            hover = at
            draw()
        } else draw()
    }

    private fun slot(i: Int): RectF {
        val pad = height * 0.14f
        val gap = height * 0.08f
        val w = (width - 2 * pad - gap * (buttons.size - 1)) / buttons.size
        val left = pad + i * (w + gap)
        return RectF(left, pad, left + w, height - pad)
    }

    private fun buttonAt(u: Float, v: Float): Int {
        val x = u * width
        val y = v * height
        return buttons.indices.firstOrNull { slot(it).contains(x, y) } ?: -1
    }

    fun draw() {
        if (released || !surface.isValid) return
        val canvas: Canvas = try {
            surface.lockHardwareCanvas()
        } catch (e: Exception) {
            return
        }
        try {
            canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR)
            val r = height * 0.22f
            val body = RectF(1.5f, 1.5f, width - 1.5f, height - 1.5f)
            canvas.drawRoundRect(body, r, r, bg)
            canvas.drawRoundRect(body, r, r, border)
            val signal = palette.signal.toArgb()
            buttons.forEachIndexed { i, b ->
                val box = slot(i)
                val br = box.height() * 0.24f
                fill.color = when {
                    i == pressed -> palette.primary2.toArgb()
                    b.primary -> palette.primary.toArgb()
                    i == hover -> withAlpha(signal, 0.28f)
                    else -> palette.surfaceVariant.toArgb()
                }
                canvas.drawRoundRect(box, br, br, fill)
                if (i == hover) {
                    fill.color = signal
                    fill.style = Paint.Style.STROKE
                    fill.strokeWidth = 4f
                    canvas.drawRoundRect(box, br, br, fill)
                    fill.style = Paint.Style.FILL
                }
                text.color = if (b.primary || i == pressed) palette.onPrimary.toArgb() else palette.onBackground.toArgb()
                canvas.drawText(b.label, box.centerX(), box.centerY() - (text.descent() + text.ascent()) / 2, text)
            }
            if (pointerU in 0f..1f && pointerV in 0f..1f) canvas.drawCircle(pointerU * width, pointerV * height, height * 0.035f, dot)
        } finally {
            try {
                surface.unlockCanvasAndPost(canvas)
            } catch (e: Exception) {
            }
        }
    }

    fun release() {
        released = true
    }

    private fun withAlpha(argb: Int, alpha: Float): Int = (argb and 0x00FFFFFF) or ((alpha * 255).toInt() shl 24)
}
