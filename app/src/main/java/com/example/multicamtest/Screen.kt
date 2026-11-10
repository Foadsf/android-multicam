package com.example.multicamtest

import android.content.Intent
import android.net.Uri
import android.provider.Settings
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxScope
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.navigationBarsPadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.window.Dialog
import androidx.lifecycle.compose.collectAsStateWithLifecycle

// Layout modelled on Foadsf/android-microscope (viewfinder on top; mode tabs, option chips and a
// shutter row underneath) and Foadsf/parallax (capture -> result flow, permission prompt).

@Composable
fun MultiCamScreen(vm: MultiCamViewModel, onRequestPermission: () -> Unit) {
    val s by vm.ui.collectAsStateWithLifecycle()
    Box(Modifier.fillMaxSize().background(Palette.Background)) {
        if (!s.hasPermission) {
            PermissionPrompt(onRequestPermission)
        } else {
            if (s.showInfo) InfoDialog { vm.showInfo(false) }
            if (s.showLog) LogDialog(s.log) { vm.showLog(false) }
            Column(Modifier.fillMaxSize()) {
                Box(Modifier.weight(1f).fillMaxWidth().background(Color.Black), contentAlignment = Alignment.Center) {
                    Viewfinder(s, vm)
                }
                Controls(s, vm)
            }
        }
    }
}

@Composable
private fun BoxScope.Viewfinder(s: UiState, vm: MultiCamViewModel) {
    val img = s.image
    if (img != null) {
        Image(img.asImageBitmap(), "Result", contentScale = ContentScale.Fit, modifier = Modifier.fillMaxSize())
    } else {
        Column(horizontalAlignment = Alignment.CenterHorizontally, modifier = Modifier.padding(32.dp)) {
            Text(if (s.mode == Mode.MACRO) "MACRO DETAIL" else "3D SCAN", style = Type.Label)
            Spacer(Modifier.height(8.dp))
            Text(
                if (s.mode == Mode.MACRO) "The finished image appears here after the burst."
                else "The live depth map appears here while the scan runs.",
                style = Type.Small.copy(fontSize = 14.sp), textAlign = TextAlign.Center,
            )
        }
    }
    Row(
        Modifier.align(Alignment.TopStart).fillMaxWidth().statusBarsPadding().padding(horizontal = 12.dp, vertical = 8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Chip("ⓘ  What is this?", active = false, description = "About these modes") { vm.showInfo(true) }
        Spacer(Modifier.weight(1f))
        Chip("Log", active = false, description = "Show log") { vm.showLog(true) }
    }
    val tone = when (s.level) {
        Level.OK -> Palette.Accent
        Level.ERROR -> Palette.Error
        Level.BUSY -> Palette.Warn
        Level.INFO -> Palette.Text
    }
    Text(
        s.message,
        style = Type.Value.copy(color = tone),
        textAlign = TextAlign.Center,
        modifier = Modifier.align(Alignment.BottomCenter).padding(12.dp).widthIn(max = 340.dp)
            .clip(RoundedCornerShape(16.dp)).background(Palette.Scrim).padding(horizontal = 16.dp, vertical = 9.dp),
    )
}

@Composable
private fun Controls(s: UiState, vm: MultiCamViewModel) {
    Column(
        Modifier.fillMaxWidth().navigationBarsPadding().padding(start = 16.dp, end = 16.dp, top = 10.dp, bottom = 10.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
    ) {
        ModeTabs(s, vm)
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.Center, verticalAlignment = Alignment.CenterVertically) {
            if (s.mode == Mode.MACRO) {
                Chip("RAW", active = s.rawMacro, description = "Use raw sensor data", enabled = !s.running) { if (!s.rawMacro) vm.toggleRaw() }
                Spacer(Modifier.size(8.dp))
                Chip("COLOUR", active = !s.rawMacro, description = "Use colour frames", enabled = !s.running) { if (s.rawMacro) vm.toggleRaw() }
            } else {
                Chip("FULL-RES TEST", active = s.fullRes, description = "Test full resolution dual streams", enabled = !s.running) { vm.toggleFullRes() }
                Spacer(Modifier.size(8.dp))
                Chip("SAVE PAIR", active = false, description = "Save left and right image", enabled = s.running) { vm.savePair() }
                Spacer(Modifier.size(8.dp))
                Chip("EXPORT MESH", active = false, description = "Export STL and PLY mesh", enabled = !s.running) { vm.exportMesh() }
            }
        }
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.Center) { Shutter(s, vm) }
    }
}

@Composable
private fun ModeTabs(s: UiState, vm: MultiCamViewModel) {
    Column(Modifier.fillMaxWidth(), horizontalAlignment = Alignment.CenterHorizontally) {
        Row(horizontalArrangement = Arrangement.spacedBy(28.dp)) {
            for ((mode, name) in listOf(Mode.MACRO to "MACRO", Mode.SCAN to "3D SCAN")) {
                val selected = s.mode == mode
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    modifier = Modifier.clip(RoundedCornerShape(8.dp))
                        .clickable(enabled = !s.running, role = Role.Tab) { vm.setMode(mode) }
                        .padding(horizontal = 6.dp, vertical = 4.dp),
                ) {
                    Text(name, style = Type.Tab.copy(color = if (selected) Palette.Accent else Palette.TextDim))
                    Spacer(Modifier.height(4.dp))
                    Box(Modifier.size(4.dp).clip(CircleShape).background(if (selected) Palette.Accent else Color.Transparent))
                }
            }
        }
        Spacer(Modifier.height(4.dp))
        Text(
            if (s.mode == Mode.MACRO) "${CaptureEngine.MICROSCOPE_BURST_FRAMES} frames, gyro-aligned, 2× resolution"
            else "wide + ultra-wide together, live depth map",
            style = Type.Small,
        )
    }
}

@Composable
private fun Shutter(s: UiState, vm: MultiCamViewModel) {
    val progress by animateFloatAsState(s.progress ?: 0f, label = "progress")
    Box(
        Modifier.size(78.dp).clip(CircleShape).clickable(role = Role.Button) { vm.shutter() }
            .semantics { contentDescription = if (s.running) "Stop" else "Start" },
        contentAlignment = Alignment.Center,
    ) {
        Canvas(Modifier.fillMaxSize()) {
            val stroke = 3.dp.toPx()
            drawCircle(Palette.Text, size.minDimension / 2 - stroke / 2, style = Stroke(stroke))
            if (s.progress != null) {
                drawArc(
                    Palette.Accent, -90f, 360f * progress, false,
                    topLeft = Offset(stroke / 2, stroke / 2), size = Size(size.width - stroke, size.height - stroke),
                    style = Stroke(stroke, cap = StrokeCap.Round),
                )
            }
            val inner = size.minDimension / 2 - 8.dp.toPx()
            if (s.running) {
                // Running: the shutter becomes a stop square.
                val half = inner * 0.55f
                drawRoundRect(
                    Palette.Accent, topLeft = Offset(size.width / 2 - half, size.height / 2 - half), size = Size(2 * half, 2 * half),
                    cornerRadius = androidx.compose.ui.geometry.CornerRadius(half / 4, half / 4),
                )
            } else {
                drawCircle(Palette.Text, inner)
            }
        }
    }
}

@Composable
private fun Chip(text: String, active: Boolean, description: String, enabled: Boolean = true, onClick: () -> Unit) {
    Text(
        text,
        style = Type.Value.copy(
            fontSize = 12.sp, fontWeight = FontWeight.SemiBold,
            color = if (active) Palette.Background else if (enabled) Palette.Text else Palette.TextDim,
        ),
        modifier = Modifier.clip(RoundedCornerShape(50)).background(if (active) Palette.Accent else Palette.Scrim)
            .border(1.dp, if (active) Palette.Accent else Palette.Outline, RoundedCornerShape(50))
            .clickable(enabled = enabled, role = Role.Button, onClick = onClick)
            .semantics { contentDescription = description }
            .padding(horizontal = 12.dp, vertical = 8.dp),
    )
}

@Composable
private fun InfoDialog(onDismiss: () -> Unit) {
    Dialog(onDismissRequest = onDismiss) {
        Column(
            Modifier.clip(RoundedCornerShape(16.dp)).background(Palette.Surface).padding(20.dp).verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(10.dp),
        ) {
            Text("MACRO", style = Type.Label)
            Text(
                "Hold the phone steady about 10 cm from a flat, well-lit subject (PCB, fabric, print). The app takes " +
                    "${CaptureEngine.MICROSCOPE_BURST_FRAMES} frames from the main camera, aligns them with the gyroscope and " +
                    "merges them into a sharper 2× image. Expect cleaner texture and edges, about 60–80 µm detail. " +
                    "This is not true microscope resolution without extra optics.",
                style = Type.Value,
            )
            Text("3D SCAN", style = Type.Label)
            Text(
                "Streams the wide and ultra-wide cameras together and shows a depth map. Some phones refuse to run " +
                    "both at once; the status line says so. Depth is relative, not metric, until a stereo calibration is added. " +
                    "SAVE PAIR stores left/right images for calibration; EXPORT MESH writes STL and PLY.",
                style = Type.Value,
            )
            Text("Files are saved in the app's folder: Android/data/com.example.multicamtest/files", style = Type.Small)
            Chip("Close", active = true, description = "Close") { onDismiss() }
        }
    }
}

@Composable
private fun LogDialog(lines: List<String>, onDismiss: () -> Unit) {
    Dialog(onDismissRequest = onDismiss) {
        Column(Modifier.clip(RoundedCornerShape(16.dp)).background(Palette.Surface).padding(16.dp)) {
            Text("LOG", style = Type.Label)
            Spacer(Modifier.height(8.dp))
            Column(Modifier.heightIn(max = 380.dp).verticalScroll(rememberScrollState())) {
                Text(
                    if (lines.isEmpty()) "(empty)" else lines.joinToString("\n"),
                    style = Type.Small.copy(fontFamily = FontFamily.Monospace, fontSize = 11.sp, color = Palette.Text),
                )
            }
            Spacer(Modifier.height(10.dp))
            Chip("Close", active = true, description = "Close") { onDismiss() }
        }
    }
}

@Composable
private fun PermissionPrompt(onRequest: () -> Unit) {
    val context = LocalContext.current
    Column(
        Modifier.fillMaxSize().padding(32.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("MultiCam Macro & 3D", style = Type.Value.copy(fontSize = 26.sp, fontWeight = FontWeight.SemiBold))
        Spacer(Modifier.height(12.dp))
        Text(
            "Hand-held macro super-resolution and two-camera 3D scanning. It needs the camera; nothing leaves your phone.",
            style = Type.Small.copy(fontSize = 14.sp), textAlign = TextAlign.Center,
        )
        Spacer(Modifier.height(28.dp))
        Chip("Allow camera", active = true, description = "Allow camera") { onRequest() }
        Spacer(Modifier.height(14.dp))
        Text(
            "Open app settings",
            style = Type.Small,
            modifier = Modifier.clickable {
                context.startActivity(
                    Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.fromParts("package", context.packageName, null)),
                )
            }.padding(8.dp),
        )
    }
}
