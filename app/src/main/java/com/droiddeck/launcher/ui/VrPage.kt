package com.droiddeck.launcher.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.outlined.ViewInAr
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.droiddeck.launcher.R

@Composable
internal fun VrPage(s: FrontEndState, a: FrontEndActions, modifier: Modifier) {
    val narrow = LocalNarrowPane.current
    val colors = MaterialTheme.colorScheme
    val tabs = modeSheetTabs(steam = true).map { it to sheetTabLabel(it) }
    LiftHero(
        key = "vr", tabs = tabs,
        onOpen = a.onSteamSettings, onPrimary = a.onSteamVr,
        modifier = modifier.clipToBounds(),
        start = if (narrow) 20.dp else 44.dp, bottom = if (narrow) 20.dp else 44.dp,
        background = { VrWall() },
        scrim = { WallFade() },
        words = {
            Column(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                Text(
                    stringResource(R.string.vr_steamvr_title), fontSize = if (narrow) 30.sp else 42.sp, lineHeight = if (narrow) 34.sp else 46.sp,
                    fontWeight = FontWeight.Black, color = colors.onBackground,
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Chip(stringResource(R.string.vr_chip_headset), ok = true)
                    Chip(stringResource(R.string.vr_chip_experimental), ok = false)
                }
                BusyChip(s)
            }
        },
        button = { click ->
            PrimaryButton(stringResource(R.string.vr_steamvr_enter), enabled = !s.busy, main = true, large = true, icon = Icons.Outlined.ViewInAr, onClick = click)
        },
        rows = { tab -> a.modeSheet(com.droiddeck.launcher.session.SessionService.MODE_STEAM, tab, SheetExtras()) },
    )
}

@Composable
private fun VrWall() {
    val colors = MaterialTheme.colorScheme
    val pal = LocalPalette.current
    Box(
        Modifier.fillMaxSize().background(
            Brush.radialGradient(listOf(pal.signal.copy(alpha = 0.22f), colors.background.copy(alpha = 0f))),
        ),
        contentAlignment = Alignment.CenterEnd,
    ) {
        Icon(
            Icons.Outlined.ViewInAr, contentDescription = null, tint = colors.onBackground.copy(alpha = 0.08f),
            modifier = Modifier.size(420.dp).offset(x = 60.dp).padding(24.dp),
        )
    }
}
