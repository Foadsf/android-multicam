package com.example.multicamtest

import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.content.Intent
import android.graphics.SurfaceTexture
import android.net.Uri
import android.os.SystemClock
import android.provider.Settings
import android.view.Surface
import android.view.TextureView
import androidx.activity.compose.BackHandler
import androidx.compose.animation.AnimatedContent
import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.tween
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.scaleIn
import androidx.compose.animation.scaleOut
import androidx.compose.animation.slideInVertically
import androidx.compose.animation.slideOutVertically
import androidx.compose.animation.togetherWith
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
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
import androidx.compose.foundation.Image
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.ModalBottomSheet
import androidx.compose.material3.Text
import androidx.compose.material3.rememberModalBottomSheetState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableLongStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.hapticfeedback.HapticFeedbackType
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalHapticFeedback
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import kotlinx.coroutines.delay

private const val MIME_BMP = "image/bmp"
private const val MIME_BIN = "application/octet-stream"

@Composable
fun MultiCamScreen(vm: MultiCamViewModel, onRequestPermission: () -> Unit) {
    val s by vm.ui.collectAsStateWithLifecycle()
    val view = LocalView.current

    // Keep the display awake while a burst or scan is running.
    DisposableEffect(s.running) {
        view.keepScreenOn = s.running
        onDispose { view.keepScreenOn = false }
    }
    BackHandler(enabled = s.hasPermission && (s.showGuide || s.showConsole || s.showComparison || s.lastStlFile != null || s.running)) {
        vm.handleBack()
    }

    Box(Modifier.fillMaxSize().background(Palette.Background)) {
        if (!s.hasPermission) {
            PermissionScreen(onRequestPermission)
        } else {
            val accent by animateColorAsState(Palette.accentFor(s.mode), tween(300), label = "accent")
            Column(Modifier.fillMaxSize().statusBarsPadding()) {
                TopBar(s, vm, accent)
                Box(
                    Modifier.weight(1f).fillMaxWidth().padding(horizontal = 12.dp),
                    contentAlignment = Alignment.Center,
                ) {
                    Viewfinder(s, vm, accent)
                }
                ControlPanel(s, vm, accent)
            }
            if (s.showGuide) GuideSheet(accent) { vm.showGuide(false) }
            if (s.showConsole) ConsoleSheet(s.log) { vm.showConsole(false) }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Top bar
// ---------------------------------------------------------------------------------------------

@Composable
private fun TopBar(s: UiState, vm: MultiCamViewModel, accent: Color) {
    Row(
        Modifier.fillMaxWidth().height(64.dp).padding(start = 20.dp, end = 12.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Column(Modifier.weight(1f)) {
            Text("MULTICAM", style = Type.Label.copy(color = accent, fontSize = 10.sp, letterSpacing = 2.sp))
            AnimatedContent(
                targetState = s.mode,
                transitionSpec = { fadeIn(tween(200)) togetherWith fadeOut(tween(120)) },
                label = "title",
            ) { m ->
                Text(
                    when (m) {
                        Mode.INSPECT -> "Surface Inspection"
                        Mode.SCAN_3D -> "3D Scanner"
                        Mode.LABS -> "Labs"
                    },
                    style = Type.Title, maxLines = 1,
                )
            }
        }
        IconBtn(Ic.Console, "Open console log", size = 44.dp) { vm.showConsole(true) }
        Spacer(Modifier.width(8.dp))
        IconBtn(Ic.Help, "How it works", size = 44.dp) { vm.showGuide(true) }
    }
}

// ---------------------------------------------------------------------------------------------
// Viewfinder
// ---------------------------------------------------------------------------------------------

@Composable
private fun Viewfinder(s: UiState, vm: MultiCamViewModel, accent: Color) {
    val frameShape = RoundedCornerShape(28.dp)
    val preview = vm.previewSize
    // Preview buffers are landscape; the phone is portrait-locked, so the view is h:w.
    val aspect = preview.height.toFloat() / preview.width.toFloat()
    val borderColor by animateColorAsState(if (s.running) accent else Palette.Outline, tween(300), label = "vfBorder")
    val showHeatmap = s.image != null && s.mode != Mode.INSPECT
    val hasModel = s.lastStlFile != null && s.lastTriangles > 0

    Box(
        Modifier
            .aspectRatio(aspect)
            .clip(frameShape)
            .background(Color.Black)
            .border(if (s.running) 2.dp else 1.dp, borderColor, frameShape),
    ) {
        // 1. Live camera preview (always attached so the surface survives mode switches).
        AndroidView(
            factory = { ctx ->
                TextureView(ctx).apply {
                    surfaceTextureListener = object : TextureView.SurfaceTextureListener {
                        override fun onSurfaceTextureAvailable(st: SurfaceTexture, w: Int, h: Int) {
                            st.setDefaultBufferSize(preview.width, preview.height)
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
            modifier = Modifier.fillMaxSize(),
        )

        // 2. Results / live depth map
        if (s.showComparison && s.beforeBitmap != null && s.afterBitmap != null) {
            CompareView(
                before = s.beforeBitmap, after = s.afterBitmap,
                split = s.comparisonSplit, onSplit = vm::setComparisonSplit,
                accent = accent, modifier = Modifier.fillMaxSize(),
            )
        } else if (showHeatmap) {
            Image(
                bitmap = s.image.asImageBitmap(),
                contentDescription = "Live stereo depth map",
                contentScale = ContentScale.Crop,
                modifier = Modifier.fillMaxSize().background(Color.Black),
            )
            if (!hasModel) DepthLegend(Modifier.align(Alignment.CenterEnd).padding(end = 10.dp))
        }

        // 3. Framing reticle on the live preview
        if (!s.showComparison && !showHeatmap) {
            Reticle(
                color = if (s.stability >= 0.7f) accent else Palette.Warn,
                modifier = Modifier.align(Alignment.Center).fillMaxWidth(0.52f).aspectRatio(1f),
            )
        }

        // 4. Top-of-viewfinder chips
        if (!s.showComparison) {
            StabilityPill(s.stability, Modifier.align(Alignment.TopStart).padding(12.dp))
            when {
                s.running && s.mode != Mode.INSPECT -> RecordingTag(s.runStartedAt, Modifier.align(Alignment.TopEnd).padding(12.dp))
                s.mode == Mode.INSPECT && !s.running -> Tag(if (s.rawMacro) "RAW16 · 20 FRAMES" else "YUV · 20 FRAMES", accent, Modifier.align(Alignment.TopEnd).padding(12.dp).background(Palette.Glass, RoundedCornerShape(50)))
            }
        }

        // 5. Burst progress
        AnimatedVisibility(
            visible = s.progress != null && s.mode == Mode.INSPECT,
            modifier = Modifier.align(Alignment.Center),
            enter = fadeIn() + scaleIn(initialScale = 0.9f), exit = fadeOut() + scaleOut(targetScale = 0.9f),
        ) {
            val p = s.progress ?: 0f
            val cur = (p * CaptureEngine.MICROSCOPE_BURST_FRAMES).toInt()
            Column(
                Modifier.clip(RoundedCornerShape(20.dp)).background(Palette.Scrim).padding(horizontal = 28.dp, vertical = 18.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                Text("$cur", style = Type.Headline.copy(fontSize = 44.sp, color = accent))
                Text("of ${CaptureEngine.MICROSCOPE_BURST_FRAMES} frames", style = Type.Small)
                Spacer(Modifier.height(6.dp))
                Text("Hold steady", style = Type.Value.copy(color = Palette.Text))
            }
        }

        // 6. Mesh-building spinner
        AnimatedVisibility(visible = s.exporting, modifier = Modifier.align(Alignment.Center), enter = fadeIn(), exit = fadeOut()) {
            Column(
                Modifier.clip(RoundedCornerShape(20.dp)).background(Palette.Scrim).padding(horizontal = 28.dp, vertical = 20.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                CircularProgressIndicator(color = accent, strokeWidth = 3.dp, modifier = Modifier.size(36.dp))
                Spacer(Modifier.height(12.dp))
                Text("Building mesh…", style = Type.Value)
            }
        }

        // 7. Bottom overlays: model card / labs console / status line
        Column(
            Modifier.align(Alignment.BottomCenter).fillMaxWidth().padding(12.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            if (hasModel) ModelCard(s, accent)
            if (s.mode == Mode.LABS && !s.showComparison) MiniConsole(s.log) { vm.showConsole(true) }
            if (!s.showComparison && !hasModel) StatusPill(s.message, s.level)
        }
    }
}

@Composable
private fun RecordingTag(startedAt: Long, modifier: Modifier = Modifier) {
    var now by remember { mutableLongStateOf(SystemClock.elapsedRealtime()) }
    LaunchedEffect(startedAt) {
        while (true) { now = SystemClock.elapsedRealtime(); delay(500) }
    }
    val secs = ((now - startedAt) / 1000).coerceAtLeast(0)
    Row(
        modifier.height(40.dp).clip(RoundedCornerShape(50)).background(Palette.Glass).padding(horizontal = 14.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(Modifier.size(8.dp).clip(CircleShape).background(Palette.Error))
        Spacer(Modifier.width(8.dp))
        Text("%02d:%02d".format(secs / 60, secs % 60), style = Type.Value.copy(fontSize = 13.sp))
    }
}

@Composable
private fun ModelCard(s: UiState, accent: Color) {
    Column(
        Modifier.fillMaxWidth().clip(RoundedCornerShape(20.dp)).background(Palette.Glass)
            .border(1.dp, accent.copy(alpha = 0.5f), RoundedCornerShape(20.dp)).padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Box(Modifier.size(32.dp).clip(CircleShape).background(accent.copy(alpha = 0.18f)), contentAlignment = Alignment.Center) {
                Icon(Ic.Check, null, tint = accent, modifier = Modifier.size(18.dp))
            }
            Spacer(Modifier.width(12.dp))
            Column {
                Text("3D model ready", style = Type.Title.copy(fontSize = 17.sp))
                Text("${"%,d".format(s.lastTriangles)} triangles", style = Type.Small)
            }
        }
        Row(horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            Tag(s.lastStlFile?.name ?: "STL", accent)
            Tag(s.lastPlyFile?.name ?: "PLY", accent)
        }
        Text(
            "Uncalibrated scale is approximate. Load stereo_calibration.json in Labs for metric accuracy.",
            style = Type.Small,
        )
    }
}

@Composable
private fun MiniConsole(lines: List<String>, onClick: () -> Unit) {
    Column(
        Modifier.fillMaxWidth().clip(RoundedCornerShape(16.dp)).background(Palette.Glass)
            .clickableNoRipple(onClick).padding(horizontal = 12.dp, vertical = 8.dp)
            .semantics { contentDescription = "Console preview. Tap to expand." },
    ) {
        val last = lines.takeLast(3)
        if (last.isEmpty()) Text("> waiting for stream…", style = Type.Small.copy(fontFamily = FontFamily.Monospace, fontSize = 11.sp))
        last.forEach { Text("> $it", style = Type.Small.copy(fontFamily = FontFamily.Monospace, fontSize = 11.sp, color = Palette.Text), maxLines = 1) }
    }
}

private fun Modifier.clickableNoRipple(onClick: () -> Unit): Modifier =
    this.then(Modifier.clickable(interactionSource = null, indication = null, onClick = onClick))

// ---------------------------------------------------------------------------------------------
// Control panel
// ---------------------------------------------------------------------------------------------

@Composable
private fun ControlPanel(s: UiState, vm: MultiCamViewModel, accent: Color) {
    val context = LocalContext.current
    val haptic = LocalHapticFeedback.current
    val hasModel = s.lastStlFile != null && s.lastTriangles > 0
    val hasImageResult = s.showComparison

    Column(
        Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(topStart = 28.dp, topEnd = 28.dp))
            .background(Palette.PanelBrush)
            .navigationBarsPadding()
            .padding(start = 16.dp, end = 16.dp, top = 14.dp, bottom = 12.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        AnimatedContent(
            targetState = when {
                hasImageResult -> PanelState.IMAGE_RESULT
                hasModel -> PanelState.MODEL_RESULT
                else -> PanelState.CAPTURE
            },
            transitionSpec = { (fadeIn(tween(220)) + slideInVertically(tween(220)) { it / 6 }) togetherWith fadeOut(tween(120)) },
            label = "panel",
        ) { state ->
            when (state) {
                PanelState.IMAGE_RESULT -> ResultActions(
                    caption = "Preview is downscaled — the full 8064×6048 image is saved on the device.",
                    accent = accent,
                    onShare = { vm.shareFile(context, s.lastResultFile, MIME_BMP) },
                    shareLabel = "Share",
                    onOpen = { vm.openFile(context, s.lastResultFile, MIME_BMP) },
                    onAgain = { vm.dismissResult() },
                    againLabel = "Retake",
                )
                PanelState.MODEL_RESULT -> ResultActions(
                    caption = "Binary STL for slicers, PLY for viewers. Saved in the app folder.",
                    accent = accent,
                    onShare = { vm.shareFile(context, s.lastStlFile, MIME_BIN) },
                    shareLabel = "Share STL",
                    onOpen = { vm.openFile(context, s.lastStlFile, MIME_BIN) },
                    onAgain = { vm.dismissResult() },
                    againLabel = "New scan",
                    extraLabel = "Share PLY",
                    onExtra = { vm.shareFile(context, s.lastPlyFile, MIME_BIN) },
                )
                PanelState.CAPTURE -> CaptureControls(s, vm, accent) { haptic.performHapticFeedback(HapticFeedbackType.LongPress) }
            }
        }

        Segmented(
            items = listOf(
                SegItem("INSPECT", Ic.Macro, "Surface inspection mode"),
                SegItem("3D SCAN", Ic.Cube, "3D scan mode"),
                SegItem("LABS", Ic.Labs, "Labs and diagnostics mode"),
            ),
            selected = s.mode.ordinal,
            accent = accent,
            enabled = !s.running && !s.exporting,
        ) {
            haptic.performHapticFeedback(HapticFeedbackType.TextHandleMove)
            vm.setMode(Mode.entries[it])
        }
    }
}

private enum class PanelState { CAPTURE, IMAGE_RESULT, MODEL_RESULT }

@Composable
private fun ResultActions(
    caption: String,
    accent: Color,
    onShare: () -> Unit, shareLabel: String,
    onOpen: () -> Unit,
    onAgain: () -> Unit, againLabel: String,
    extraLabel: String? = null, onExtra: (() -> Unit)? = null,
) {
    Column(horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(14.dp)) {
        Text(caption, style = Type.Small, textAlign = TextAlign.Center, modifier = Modifier.widthIn(max = 320.dp))
        Row(horizontalArrangement = Arrangement.spacedBy(10.dp), modifier = Modifier.fillMaxWidth()) {
            PillButton(shareLabel, Modifier.weight(1f), accent = accent, icon = Ic.Share, onClick = onShare)
            PillButton("Open", Modifier.weight(0.8f), accent = accent, filled = false, icon = Ic.Open, onClick = onOpen)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(10.dp), modifier = Modifier.fillMaxWidth()) {
            if (extraLabel != null && onExtra != null) {
                PillButton(extraLabel, Modifier.weight(1f), accent = accent, filled = false, icon = Ic.Download, onClick = onExtra)
            }
            PillButton(againLabel, Modifier.weight(1f), accent = Palette.TextDim, filled = false, icon = Ic.Retake, onClick = onAgain)
        }
    }
}

@Composable
private fun CaptureControls(s: UiState, vm: MultiCamViewModel, accent: Color, onTap: () -> Unit) {
    Column(horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(10.dp)) {
        // Context options
        Box(Modifier.heightIn(min = 44.dp), contentAlignment = Alignment.Center) {
            when (s.mode) {
                Mode.INSPECT -> Segmented(
                    items = listOf(
                        SegItem("RAW · BEST", description = "Use raw sensor data for best quality"),
                        SegItem("YUV · FAST", description = "Use processed colour frames, faster"),
                    ),
                    selected = if (s.rawMacro) 0 else 1, accent = accent, height = 44.dp,
                    enabled = !s.running, stacked = false,
                    modifier = Modifier.widthIn(max = 300.dp),
                ) { if ((it == 0) != s.rawMacro) vm.toggleRaw() }

                Mode.SCAN_3D -> Text(
                    if (s.running) "Move slowly in an arc around the object" else "Keep the object centred, 15–40 cm from the lenses",
                    style = Type.Small.copy(fontSize = 13.sp), textAlign = TextAlign.Center,
                )

                Mode.LABS -> Segmented(
                    items = listOf(
                        SegItem("STANDARD", description = "Standard stream resolution"),
                        SegItem("FULL-RES", description = "Full sensor resolution on both cameras"),
                    ),
                    selected = if (s.fullRes) 1 else 0, accent = accent, height = 44.dp,
                    enabled = !s.running, stacked = false,
                    modifier = Modifier.widthIn(max = 300.dp),
                ) { if ((it == 1) != s.fullRes) vm.toggleFullRes() }
            }
        }

        // Shutter row — optional secondary actions flank the shutter for thumb reach.
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceEvenly, verticalAlignment = Alignment.Top) {
            Box(Modifier.width(72.dp).height(88.dp), contentAlignment = Alignment.TopCenter) {
                if (s.mode == Mode.LABS) LabeledAction(Ic.Pair, "Save pair", enabled = s.running) { vm.savePair() }
            }
            Column(horizontalAlignment = Alignment.CenterHorizontally) {
                val label = when {
                    s.running && s.mode == Mode.SCAN_3D -> "Finish & build mesh"
                    s.running -> "Stop"
                    s.mode == Mode.INSPECT -> "Capture 2×"
                    s.mode == Mode.SCAN_3D -> "Start scan"
                    else -> "Start stream"
                }
                Shutter(
                    running = s.running, progress = s.progress, accent = accent, description = label,
                    enabled = s.hasPermission && !s.exporting,
                ) { onTap(); vm.shutter() }
                Spacer(Modifier.height(6.dp))
                Text(label.uppercase(), style = Type.Label.copy(color = Palette.Text, fontSize = 11.sp))
            }
            Box(Modifier.width(72.dp).height(88.dp), contentAlignment = Alignment.TopCenter) {
                if (s.mode == Mode.LABS) LabeledAction(Ic.Download, "Export mesh", enabled = !s.running && !s.exporting) { vm.exportMesh() }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Sheets
// ---------------------------------------------------------------------------------------------

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun GuideSheet(accent: Color, onDismiss: () -> Unit) {
    ModalBottomSheet(
        onDismissRequest = onDismiss,
        sheetState = rememberModalBottomSheetState(skipPartiallyExpanded = true),
        containerColor = Palette.Surface,
        contentColor = Palette.Text,
        dragHandle = null,
        shape = RoundedCornerShape(topStart = 28.dp, topEnd = 28.dp),
    ) {
        Column(
            Modifier.fillMaxWidth().verticalScroll(rememberScrollState()).navigationBarsPadding().padding(horizontal = 20.dp, vertical = 12.dp),
            verticalArrangement = Arrangement.spacedBy(14.dp),
        ) {
            Box(Modifier.align(Alignment.CenterHorizontally).width(40.dp).height(4.dp).clip(CircleShape).background(Palette.Outline))
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text("How it works", style = Type.Headline.copy(fontSize = 24.sp), modifier = Modifier.weight(1f))
                IconBtn(Ic.Close, "Close guide", size = 44.dp, background = Palette.SurfaceHigh, onClick = onDismiss)
            }
            GuideCard(Ic.Macro, Palette.Accent, "Surface Inspection",
                "Hold the phone about 10 cm from a flat subject — PCBs, prints, fabric, coins. A 20-frame burst is aligned with the gyroscope and merged onto a 2× sub-pixel grid. Slide the divider afterwards to compare with a single frame.")
            GuideCard(Ic.Cube, Palette.Violet, "3D Scanner",
                "Uses the wide and ultra-wide cameras together as a stereo pair. Move slowly around a textured object, then tap Finish to build an STL + PLY mesh you can share to a slicer or viewer.")
            GuideCard(Ic.Labs, Palette.Orange, "Labs",
                "Raw dual-camera stream, synchronized calibration-pair capture (PGM) and mesh export. Put a stereo_calibration.json in the app folder for metric scale.")
            Column(
                Modifier.fillMaxWidth().clip(RoundedCornerShape(16.dp)).background(Palette.SurfaceHigh).padding(16.dp),
                verticalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                Text("TIPS", style = Type.Label.copy(color = accent))
                Text("• Lock the phone against a surface or lean your elbows on the table.\n• Good light beats high ISO — the burst uses short exposures.\n• The STEADY badge turns green when the gyroscope sees little motion.\n• Double-tap a result to reset zoom.", style = Type.Body)
            }
            Row(Modifier.fillMaxWidth().padding(bottom = 8.dp), verticalAlignment = Alignment.CenterVertically) {
                Icon(Ic.Lock, null, tint = Palette.Ok, modifier = Modifier.size(20.dp))
                Spacer(Modifier.width(10.dp))
                Text("100% on-device. No network permission, no telemetry. Free software (GPL-3.0).", style = Type.Small.copy(color = Palette.Text))
            }
        }
    }
}

@Composable
private fun GuideCard(icon: ImageVector, tone: Color, title: String, body: String) {
    Row(
        Modifier.fillMaxWidth().clip(RoundedCornerShape(16.dp)).background(Palette.SurfaceHigh).padding(16.dp),
        horizontalArrangement = Arrangement.spacedBy(14.dp),
    ) {
        Box(Modifier.size(44.dp).clip(CircleShape).background(tone.copy(alpha = 0.16f)), contentAlignment = Alignment.Center) {
            Icon(icon, null, tint = tone, modifier = Modifier.size(22.dp))
        }
        Column(verticalArrangement = Arrangement.spacedBy(4.dp)) {
            Text(title, style = Type.Title.copy(fontSize = 16.sp))
            Text(body, style = Type.Body.copy(color = Palette.TextDim))
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
private fun ConsoleSheet(lines: List<String>, onDismiss: () -> Unit) {
    val context = LocalContext.current
    val scroll = rememberScrollState()
    LaunchedEffect(lines.size) { scroll.animateScrollTo(scroll.maxValue) }
    ModalBottomSheet(
        onDismissRequest = onDismiss,
        sheetState = rememberModalBottomSheetState(skipPartiallyExpanded = true),
        containerColor = Palette.Surface,
        contentColor = Palette.Text,
        dragHandle = null,
        shape = RoundedCornerShape(topStart = 28.dp, topEnd = 28.dp),
    ) {
        Column(
            Modifier.fillMaxWidth().navigationBarsPadding().padding(horizontal = 20.dp, vertical = 12.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            Box(Modifier.align(Alignment.CenterHorizontally).width(40.dp).height(4.dp).clip(CircleShape).background(Palette.Outline))
            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Column(Modifier.weight(1f)) {
                    Text("Console", style = Type.Headline.copy(fontSize = 24.sp))
                    Text("Files: Android/data/${context.packageName}/files", style = Type.Small.copy(fontSize = 11.sp), maxLines = 1)
                }
                IconBtn(Ic.Copy, "Copy log", size = 44.dp, background = Palette.SurfaceHigh, enabled = lines.isNotEmpty()) {
                    val cm = context.getSystemService(Context.CLIPBOARD_SERVICE) as ClipboardManager
                    cm.setPrimaryClip(ClipData.newPlainText("MultiCam log", lines.joinToString("\n")))
                }
                IconBtn(Ic.Close, "Close console", size = 44.dp, background = Palette.SurfaceHigh, onClick = onDismiss)
            }
            Box(
                Modifier.fillMaxWidth().heightIn(min = 160.dp, max = 420.dp).clip(RoundedCornerShape(16.dp))
                    .background(Palette.Background).border(1.dp, Palette.Outline, RoundedCornerShape(16.dp)),
            ) {
                Text(
                    if (lines.isEmpty()) "(no events yet)" else lines.joinToString("\n"),
                    style = Type.Small.copy(fontFamily = FontFamily.Monospace, fontSize = 11.sp, lineHeight = 15.sp, color = Palette.Text),
                    modifier = Modifier.verticalScroll(scroll).padding(14.dp).fillMaxWidth(),
                )
            }
            Spacer(Modifier.height(4.dp))
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Permission / welcome
// ---------------------------------------------------------------------------------------------

@Composable
private fun PermissionScreen(onRequest: () -> Unit) {
    val context = LocalContext.current
    Box(
        Modifier.fillMaxSize().background(
            Brush.radialGradient(
                listOf(Palette.AccentDim.copy(alpha = 0.55f), Color.Transparent),
                center = androidx.compose.ui.geometry.Offset(540f, 300f), radius = 900f,
            ),
        ),
    ) {
        Column(
            Modifier.fillMaxSize().statusBarsPadding().navigationBarsPadding().verticalScroll(rememberScrollState()).padding(horizontal = 28.dp, vertical = 24.dp),
            verticalArrangement = Arrangement.Center,
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            Box(
                Modifier.size(112.dp).clip(CircleShape).background(Palette.Surface).border(1.dp, Palette.Accent.copy(alpha = 0.5f), CircleShape),
                contentAlignment = Alignment.Center,
            ) { Icon(Ic.Lens, null, tint = Palette.Accent, modifier = Modifier.size(64.dp)) }
            Spacer(Modifier.height(24.dp))
            Text("MultiCam Macro & 3D", style = Type.Headline, textAlign = TextAlign.Center)
            Spacer(Modifier.height(8.dp))
            Text(
                "Turn your phone cameras into a close-up inspection instrument and a stereo 3D scanner.",
                style = Type.Body.copy(color = Palette.TextDim), textAlign = TextAlign.Center, modifier = Modifier.widthIn(max = 340.dp),
            )
            Spacer(Modifier.height(28.dp))
            Column(Modifier.widthIn(max = 340.dp), verticalArrangement = Arrangement.spacedBy(14.dp)) {
                Feature(Ic.Macro, Palette.Accent, "2× macro super-resolution", "Gyro-aligned burst, sub-pixel merge")
                Feature(Ic.Cube, Palette.Violet, "Dual-camera 3D scanning", "Export STL and PLY meshes")
                Feature(Ic.Lock, Palette.Ok, "Private by design", "Runs offline — no network permission")
            }
            Spacer(Modifier.height(32.dp))
            PillButton("Allow camera access", Modifier.fillMaxWidth().widthIn(max = 340.dp), icon = Ic.Lens, onClick = onRequest)
            Spacer(Modifier.height(10.dp))
            PillButton(
                "Open app settings", Modifier.fillMaxWidth().widthIn(max = 340.dp), accent = Palette.TextDim, filled = false,
                onClick = {
                    context.startActivity(
                        Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS, Uri.fromParts("package", context.packageName, null))
                            .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK),
                    )
                },
            )
        }
    }
}

@Composable
private fun Feature(icon: ImageVector, tone: Color, title: String, subtitle: String) {
    Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(14.dp)) {
        Box(Modifier.size(44.dp).clip(CircleShape).background(tone.copy(alpha = 0.15f)), contentAlignment = Alignment.Center) {
            Icon(icon, null, tint = tone, modifier = Modifier.size(22.dp))
        }
        Column {
            Text(title, style = Type.Value.copy(fontSize = 15.sp, fontWeight = androidx.compose.ui.text.font.FontWeight.SemiBold))
            Text(subtitle, style = Type.Small)
        }
    }
}
