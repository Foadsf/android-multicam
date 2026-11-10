package com.example.multicamtest

import android.content.Intent
import android.graphics.SurfaceTexture
import android.net.Uri
import android.provider.Settings
import android.view.Surface
import android.view.TextureView
import androidx.compose.animation.core.animateFloatAsState
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
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.widthIn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.LinearProgressIndicator
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
import androidx.compose.ui.viewinterop.AndroidView
import androidx.compose.ui.window.Dialog
import androidx.lifecycle.compose.collectAsStateWithLifecycle

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
                // Live Viewfinder / Result Canvas
                Box(
                    Modifier.weight(1f).fillMaxWidth().background(Color.Black),
                    contentAlignment = Alignment.Center
                ) {
                    ViewfinderArea(s, vm)
                }
                // Interactive Controls
                ControlsArea(s, vm)
            }
        }
    }
}

@Composable
private fun BoxScope.ViewfinderArea(s: UiState, vm: MultiCamViewModel) {
    // 1. Continuous Live Camera Preview Layer
    AndroidView(
        factory = { ctx ->
            TextureView(ctx).apply {
                surfaceTextureListener = object : TextureView.SurfaceTextureListener {
                    override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) {
                        vm.setPreviewSurface(Surface(st))
                    }
                    override fun onSurfaceTextureSizeChanged(st: SurfaceTexture, w: Int, h: Int) {}
                    override fun onSurfaceTextureDestroyed(st: SurfaceTexture): Boolean {
                        vm.setPreviewSurface(null)
                        return true
                    }
                    override fun onSurfaceTextureUpdated(st: SurfaceTexture) {}
                }
            }
        },
        modifier = Modifier.fillMaxSize()
    )

    // 2. Overlaid Results / Heatmaps
    if (s.showComparison && s.beforeBitmap != null && s.afterBitmap != null) {
        val displayBitmap = if (s.showAfter) s.afterBitmap else s.beforeBitmap
        Image(
            bitmap = displayBitmap.asImageBitmap(),
            contentDescription = if (s.showAfter) "2x Super-Resolution Result" else "1x Baseline Frame",
            contentScale = ContentScale.Fit,
            modifier = Modifier.fillMaxSize().background(Palette.Background)
        )
        // Before / After Comparison Toggle Chip
        Row(
            Modifier.align(Alignment.TopCenter).statusBarsPadding().padding(top = 48.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Chip("1\u00d7 Before", active = !s.showAfter, description = "Show original 1x frame") {
                if (s.showAfter) vm.toggleComparisonView()
            }
            Chip("2\u00d7 Super-Res", active = s.showAfter, description = "Show 2x super-resolution") {
                if (!s.showAfter) vm.toggleComparisonView()
            }
        }
    } else if (s.image != null && s.mode == Mode.SCAN_3D) {
        // Live Stereo Depth Heatmap Overlay
        Image(
            bitmap = s.image.asImageBitmap(),
            contentDescription = "Live Stereo Depth Map",
            contentScale = ContentScale.Fit,
            modifier = Modifier.fillMaxSize().background(Color(0xBB000000))
        )
    }

    // 3. Center Framing Reticle (shown during live preview)
    if (!s.showComparison && (s.image == null || s.mode != Mode.SCAN_3D)) {
        Reticle()
    }

    // 4. Top Action & Navigation Bar
    Row(
        Modifier.align(Alignment.TopStart).fillMaxWidth().statusBarsPadding().padding(horizontal = 14.dp, vertical = 8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Chip("ⓘ Guide", active = false, description = "How it works") { vm.showInfo(true) }
        Spacer(Modifier.weight(1f))
        // Real-time Gyro Stability Indicator
        StabilityBadge(s.stability)
        Spacer(Modifier.width(8.dp))
        Chip("Console", active = false, description = "Show log") { vm.showLog(true) }
    }

    // 5. Active Capture Progress Indicator
    if (s.progress != null) {
        val cur = (s.progress * CaptureEngine.MICROSCOPE_BURST_FRAMES).toInt()
        Column(
            Modifier.align(Alignment.Center).clip(RoundedCornerShape(16.dp))
                .background(Palette.Scrim).padding(horizontal = 24.dp, vertical = 16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text("Capturing Burst", style = Type.Label.copy(color = Palette.Accent))
            Spacer(Modifier.height(6.dp))
            LinearProgressIndicator(
                progress = { s.progress },
                modifier = Modifier.width(180.dp).height(6.dp).clip(RoundedCornerShape(3.dp)),
                color = Palette.Accent,
                trackColor = Palette.Outline
            )
            Spacer(Modifier.height(6.dp))
            Text("Frame $cur / ${CaptureEngine.MICROSCOPE_BURST_FRAMES} \u2014 Hold Steady", style = Type.Small)
        }
    }

    // 6. 3D Model Export Card
    if (s.lastStlFile != null && s.lastTriangles > 0) {
        val context = LocalContext.current
        Column(
            Modifier.align(Alignment.Center).clip(RoundedCornerShape(20.dp))
                .background(Palette.Surface).border(1.dp, Palette.Accent, RoundedCornerShape(20.dp))
                .padding(20.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text("\u2705 3D Model Ready", style = Type.Value.copy(fontSize = 17.sp, fontWeight = FontWeight.Bold, color = Palette.Accent))
            Spacer(Modifier.height(6.dp))
            Text("${s.lastTriangles} triangles \u2022 ${s.lastStlFile.name}", style = Type.Small)
            Spacer(Modifier.height(14.dp))
            Row(horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                Chip("Share STL", active = true, description = "Share 3D Model") {
                    vm.shareFile(context, s.lastStlFile, "application/octet-stream")
                }
                Chip("Open in 3D Viewer", active = false, description = "Open in 3D Viewer") {
                    vm.openFile(context, s.lastStlFile, "application/octet-stream")
                }
            }
            Spacer(Modifier.height(8.dp))
            Text("Dismiss", style = Type.Small.copy(color = Palette.TextDim), modifier = Modifier.clickable { vm.dismissResult() }.padding(6.dp))
        }
    }

    // 7. Context Status Banner
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
        modifier = Modifier.align(Alignment.BottomCenter).padding(bottom = 12.dp).widthIn(max = 360.dp)
            .clip(RoundedCornerShape(16.dp)).background(Palette.Scrim).padding(horizontal = 16.dp, vertical = 8.dp),
    )
}

@Composable
private fun ControlsArea(s: UiState, vm: MultiCamViewModel) {
    val context = LocalContext.current
    Column(
        Modifier.fillMaxWidth().navigationBarsPadding().padding(start = 16.dp, end = 16.dp, top = 8.dp, bottom = 10.dp),
        verticalArrangement = Arrangement.spacedBy(10.dp),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // Mode Selector Tabs
        Row(horizontalArrangement = Arrangement.spacedBy(24.dp)) {
            for ((mode, name) in listOf(Mode.INSPECT to "INSPECT", Mode.SCAN_3D to "3D SCAN", Mode.LABS to "LABS")) {
                val selected = s.mode == mode
                Column(
                    horizontalAlignment = Alignment.CenterHorizontally,
                    modifier = Modifier.clip(RoundedCornerShape(8.dp))
                        .clickable(enabled = !s.running, role = Role.Tab) { vm.setMode(mode) }
                        .padding(horizontal = 8.dp, vertical = 4.dp),
                ) {
                    Text(name, style = Type.Tab.copy(color = if (selected) Palette.Accent else Palette.TextDim))
                    Spacer(Modifier.height(4.dp))
                    Box(Modifier.size(4.dp).clip(CircleShape).background(if (selected) Palette.Accent else Color.Transparent))
                }
            }
        }

        // Secondary Options / Result Actions
        when {
            s.showComparison -> {
                // Post-capture Action Row for Inspect Result
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Chip("Share Image", active = true, description = "Share super-resolution image") {
                        vm.shareFile(context, s.lastResultFile, "image/bmp")
                    }
                    Spacer(Modifier.width(10.dp))
                    Chip("Open Gallery", active = false, description = "Open in photo gallery") {
                        vm.openFile(context, s.lastResultFile, "image/bmp")
                    }
                    Spacer(Modifier.width(10.dp))
                    Chip("Retake", active = false, description = "Back to live viewfinder") {
                        vm.dismissResult()
                    }
                }
            }
            s.mode == Mode.INSPECT -> {
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Chip("Best Quality (RAW)", active = s.rawMacro, description = "Use raw sensor data", enabled = !s.running) {
                        if (!s.rawMacro) vm.toggleRaw()
                    }
                    Spacer(Modifier.width(10.dp))
                    Chip("Fast (YUV)", active = !s.rawMacro, description = "Use color frames", enabled = !s.running) {
                        if (s.rawMacro) vm.toggleRaw()
                    }
                }
            }
            s.mode == Mode.SCAN_3D -> {
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text("Move phone in a slow arc around subject", style = Type.Small)
                }
            }
            s.mode == Mode.LABS -> {
                Row(
                    Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Chip("Full-Res Test", active = s.fullRes, description = "Test full resolution dual streams", enabled = !s.running) {
                        vm.toggleFullRes()
                    }
                    Spacer(Modifier.width(8.dp))
                    Chip("Save Pair", active = false, description = "Save left/right calibration pair", enabled = s.running) {
                        vm.savePair()
                    }
                    Spacer(Modifier.width(8.dp))
                    Chip("Export Mesh", active = false, description = "Export current TSDF mesh", enabled = !s.running) {
                        vm.exportMesh()
                    }
                }
            }
        }

        // Shutter / Action Button
        if (!s.showComparison) {
            ShutterButton(s, vm)
        }
    }
}

@Composable
private fun ShutterButton(s: UiState, vm: MultiCamViewModel) {
    val progress by animateFloatAsState(s.progress ?: 0f, label = "progress")
    val buttonText = when {
        s.running -> "STOP"
        s.mode == Mode.INSPECT -> "CAPTURE 2\u00d7"
        s.mode == Mode.SCAN_3D -> "START SCAN"
        else -> "CAPTURE"
    }

    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Box(
            Modifier.size(76.dp).clip(CircleShape).clickable(role = Role.Button) { vm.shutter() }
                .semantics { contentDescription = buttonText },
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
                    val half = inner * 0.55f
                    drawRoundRect(
                        Palette.Accent, topLeft = Offset(size.width / 2 - half, size.height / 2 - half), size = Size(2 * half, 2 * half),
                        cornerRadius = androidx.compose.ui.geometry.CornerRadius(half / 4, half / 4),
                    )
                } else {
                    drawCircle(Palette.Accent, inner)
                }
            }
        }
        Spacer(Modifier.height(4.dp))
        Text(buttonText, style = Type.Label.copy(fontSize = 10.sp, color = Palette.Text))
    }
}

@Composable
private fun StabilityBadge(stability: Float) {
    val isStable = stability >= 0.70f
    val color = if (isStable) Palette.Accent else Palette.Warn
    val text = if (isStable) "● STABLE" else "▲ HOLD STEADY"
    Row(
        Modifier.clip(RoundedCornerShape(50)).background(Palette.Scrim)
            .border(1.dp, color.copy(alpha = 0.5f), RoundedCornerShape(50))
            .padding(horizontal = 10.dp, vertical = 6.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text, style = Type.Label.copy(color = color, fontSize = 10.sp))
    }
}

@Composable
private fun Reticle() {
    Canvas(Modifier.size(160.dp)) {
        val strokeWidth = 2.dp.toPx()
        val cornerLen = 22.dp.toPx()
        val col = Palette.Accent.copy(alpha = 0.55f)
        // Top-left
        drawLine(col, Offset(0f, 0f), Offset(cornerLen, 0f), strokeWidth)
        drawLine(col, Offset(0f, 0f), Offset(0f, cornerLen), strokeWidth)
        // Top-right
        drawLine(col, Offset(size.width, 0f), Offset(size.width - cornerLen, 0f), strokeWidth)
        drawLine(col, Offset(size.width, 0f), Offset(size.width, cornerLen), strokeWidth)
        // Bottom-left
        drawLine(col, Offset(0f, size.height), Offset(cornerLen, size.height), strokeWidth)
        drawLine(col, Offset(0f, size.height), Offset(0f, size.height - cornerLen), strokeWidth)
        // Bottom-right
        drawLine(col, Offset(size.width, size.height), Offset(size.width - cornerLen, size.height), strokeWidth)
        drawLine(col, Offset(size.width, size.height), Offset(size.width, size.height - cornerLen), strokeWidth)
        // Center crosshair
        val cx = size.width / 2f
        val cy = size.height / 2f
        drawLine(col, Offset(cx - 5.dp.toPx(), cy), Offset(cx + 5.dp.toPx(), cy), strokeWidth)
        drawLine(col, Offset(cx, cy - 5.dp.toPx()), Offset(cx, cy + 5.dp.toPx()), strokeWidth)
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
            .padding(horizontal = 14.dp, vertical = 9.dp),
    )
}

@Composable
private fun InfoDialog(onDismiss: () -> Unit) {
    Dialog(onDismissRequest = onDismiss) {
        Column(
            Modifier.clip(RoundedCornerShape(16.dp)).background(Palette.Surface).padding(20.dp).verticalScroll(rememberScrollState()),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Text("SURFACE INSPECTION", style = Type.Label.copy(color = Palette.Accent))
            Text(
                "Hold phone steady about 10 cm from a flat subject (PCB solder joints, prints, textiles, watch mechanics). " +
                    "The camera captures a ${CaptureEngine.MICROSCOPE_BURST_FRAMES}-frame burst, integrates gyro angular motion, " +
                    "and performs 2\u00d7 sub-pixel Drizzle reconstruction to recover optical edge sharpness and remove Bayer demosaicing artifacts.",
                style = Type.Value,
            )
            Text("3D SCANNER", style = Type.Label.copy(color = Palette.Accent))
            Text(
                "Synchronously streams the Wide and Ultra-Wide camera sensors together under a unified multi-camera handle. " +
                    "Computes stereo disparity in real-time and fuses depth into a Truncated Signed Distance Field (TSDF) voxel grid. " +
                    "Export models directly as binary STL or indexed PLY for 3D printing and CAD inspection.",
                style = Type.Value,
            )
            Text("LABS & CALIBRATION", style = Type.Label.copy(color = Palette.Accent))
            Text(
                "For metric-scale accuracy, stereo extrinsics (baseline and rotation) require calibration using a ChArUco board. " +
                    "Place a calibrated 'stereo_calibration.json' file into the app storage directory to replace empirical defaults.",
                style = Type.Value,
            )
            Text("Privacy Contract: 100% on-device processing. Zero network access. No telemetry.", style = Type.Small.copy(color = Palette.Accent))
            Chip("Close", active = true, description = "Close dialog") { onDismiss() }
        }
    }
}

@Composable
private fun LogDialog(lines: List<String>, onDismiss: () -> Unit) {
    Dialog(onDismissRequest = onDismiss) {
        Column(Modifier.clip(RoundedCornerShape(16.dp)).background(Palette.Surface).padding(16.dp)) {
            Text("SYSTEM LOG", style = Type.Label.copy(color = Palette.Accent))
            Spacer(Modifier.height(8.dp))
            Column(Modifier.heightIn(max = 380.dp).verticalScroll(rememberScrollState())) {
                Text(
                    if (lines.isEmpty()) "(empty)" else lines.joinToString("\n"),
                    style = Type.Small.copy(fontFamily = FontFamily.Monospace, fontSize = 11.sp, color = Palette.Text),
                )
            }
            Spacer(Modifier.height(10.dp))
            Chip("Close", active = true, description = "Close log") { onDismiss() }
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
            "Close-up surface inspection and dual-camera 3D scanning. All processing runs 100% on-device; zero network permissions.",
            style = Type.Small.copy(fontSize = 14.sp), textAlign = TextAlign.Center,
        )
        Spacer(Modifier.height(28.dp))
        Chip("Allow Camera Access", active = true, description = "Allow camera") { onRequest() }
        Spacer(Modifier.height(14.dp))
        Text(
            "Open App Settings",
            style = Type.Small,
            modifier = Modifier.clickable {
                context.startActivity(
                    Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.fromParts("package", context.packageName, null)),
                )
            }.padding(8.dp),
        )
    }
}
