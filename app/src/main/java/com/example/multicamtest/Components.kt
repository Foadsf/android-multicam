package com.example.multicamtest

import android.graphics.Bitmap
import androidx.compose.animation.animateColorAsState
import androidx.compose.animation.core.Spring
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.spring
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectHorizontalDragGestures
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.interaction.MutableInteractionSource
import androidx.compose.foundation.interaction.collectIsPressedAsState
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Icon
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.draw.drawWithContent
import androidx.compose.ui.draw.scale
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipRect
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.semantics.Role
import androidx.compose.ui.semantics.contentDescription
import androidx.compose.ui.semantics.selected
import androidx.compose.ui.semantics.semantics
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlin.math.roundToInt

// ---------------------------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------------------------

/** Round, 48dp (minimum recommended touch target) icon button on a translucent glass disc. */
@Composable
fun IconBtn(
    icon: ImageVector,
    description: String,
    modifier: Modifier = Modifier,
    tint: Color = Palette.Text,
    background: Color = Palette.Glass,
    size: Dp = 48.dp,
    enabled: Boolean = true,
    onClick: () -> Unit,
) {
    Box(
        modifier
            .size(size)
            .clip(CircleShape)
            .background(background)
            .border(1.dp, Palette.Outline.copy(alpha = 0.7f), CircleShape)
            .alpha(if (enabled) 1f else 0.4f)
            .clickable(enabled = enabled, role = Role.Button, onClick = onClick)
            .semantics { contentDescription = description },
        contentAlignment = Alignment.Center,
    ) {
        Icon(icon, contentDescription = null, tint = tint, modifier = Modifier.size(size * 0.46f))
    }
}

/** Labelled action: round icon button with a caption underneath (reachable with a thumb, 64dp wide). */
@Composable
fun LabeledAction(
    icon: ImageVector,
    label: String,
    modifier: Modifier = Modifier,
    tint: Color = Palette.Text,
    enabled: Boolean = true,
    onClick: () -> Unit,
) {
    Column(modifier.width(72.dp), horizontalAlignment = Alignment.CenterHorizontally) {
        IconBtn(icon, label, tint = tint, background = Palette.SurfaceHigh, size = 52.dp, enabled = enabled, onClick = onClick)
        Spacer(Modifier.height(5.dp))
        Text(
            label, style = Type.Small.copy(fontSize = 11.sp, color = if (enabled) Palette.TextDim else Palette.TextDim.copy(alpha = 0.5f)),
            textAlign = TextAlign.Center, maxLines = 1,
        )
    }
}

/** Primary / secondary pill button, 48dp tall. */
@Composable
fun PillButton(
    text: String,
    modifier: Modifier = Modifier,
    accent: Color = Palette.Accent,
    filled: Boolean = true,
    icon: ImageVector? = null,
    enabled: Boolean = true,
    onClick: () -> Unit,
) {
    val fg = if (filled) Palette.Background else accent
    Row(
        modifier
            .height(48.dp)
            .clip(RoundedCornerShape(50))
            .background(if (filled) accent else accent.copy(alpha = 0.12f))
            .border(1.dp, if (filled) accent else accent.copy(alpha = 0.45f), RoundedCornerShape(50))
            .alpha(if (enabled) 1f else 0.4f)
            .clickable(enabled = enabled, role = Role.Button, onClick = onClick)
            .padding(horizontal = 20.dp),
        horizontalArrangement = Arrangement.Center,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        if (icon != null) {
            Icon(icon, null, tint = fg, modifier = Modifier.size(20.dp))
            Spacer(Modifier.width(8.dp))
        }
        Text(text, style = Type.Button.copy(color = fg), maxLines = 1)
    }
}

// ---------------------------------------------------------------------------------------------
// Segmented switch with sliding highlight (mode switcher & option toggles)
// ---------------------------------------------------------------------------------------------

data class SegItem(val label: String, val icon: ImageVector? = null, val description: String = label)

@Composable
fun Segmented(
    items: List<SegItem>,
    selected: Int,
    accent: Color,
    modifier: Modifier = Modifier,
    height: Dp = 56.dp,
    enabled: Boolean = true,
    stacked: Boolean = true,
    onSelect: (Int) -> Unit,
) {
    val shape = RoundedCornerShape(50)
    BoxWithConstraints(
        modifier
            .fillMaxWidth()
            .height(height)
            .clip(shape)
            .background(Palette.Surface)
            .border(1.dp, Palette.Outline, shape)
            .alpha(if (enabled) 1f else 0.45f)
            .padding(4.dp),
    ) {
        val segW = maxWidth / items.size
        val x by animateDpAsState(segW * selected, spring(dampingRatio = 0.8f, stiffness = Spring.StiffnessMediumLow), label = "seg")
        Box(
            Modifier
                .offset { IntOffset(x.roundToPx(), 0) }
                .width(segW)
                .fillMaxHeight()
                .clip(shape)
                .background(accent.copy(alpha = 0.16f))
                .border(1.dp, accent.copy(alpha = 0.55f), shape),
        )
        Row(Modifier.fillMaxSize()) {
            items.forEachIndexed { i, item ->
                val on = i == selected
                val tone by animateColorAsState(if (on) accent else Palette.TextDim, tween(180), label = "segTone")
                val content: @Composable () -> Unit = {
                    if (item.icon != null) Icon(item.icon, null, tint = tone, modifier = Modifier.size(if (stacked) 22.dp else 18.dp))
                    if (item.icon != null && !stacked) Spacer(Modifier.width(8.dp))
                    Text(item.label, style = Type.Tab.copy(color = tone, fontSize = if (stacked) 11.sp else 13.sp), maxLines = 1)
                }
                Box(
                    Modifier
                        .weight(1f)
                        .fillMaxHeight()
                        .clip(shape)
                        .clickable(enabled = enabled, role = Role.Tab) { onSelect(i) }
                        .semantics { contentDescription = item.description; this.selected = on },
                    contentAlignment = Alignment.Center,
                ) {
                    if (stacked) Column(horizontalAlignment = Alignment.CenterHorizontally, verticalArrangement = Arrangement.spacedBy(1.dp)) { content() }
                    else Row(verticalAlignment = Alignment.CenterVertically) { content() }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Shutter
// ---------------------------------------------------------------------------------------------

/** 88dp shutter: ring + inner disc that morphs into a rounded "stop" square while running. */
@Composable
fun Shutter(
    running: Boolean,
    progress: Float?,
    accent: Color,
    description: String,
    enabled: Boolean = true,
    onClick: () -> Unit,
) {
    val source = remember { MutableInteractionSource() }
    val pressed by source.collectIsPressedAsState()
    val press by animateFloatAsState(if (pressed) 0.92f else 1f, spring(stiffness = Spring.StiffnessMedium), label = "press")
    val morph by animateFloatAsState(if (running) 1f else 0f, tween(260), label = "morph")
    val prog by animateFloatAsState(progress ?: 0f, tween(120), label = "prog")
    val color by animateColorAsState(accent, tween(250), label = "shutterColor")

    Box(
        Modifier
            .size(88.dp)
            .scale(press)
            .clip(CircleShape)
            .alpha(if (enabled) 1f else 0.4f)
            .clickable(interactionSource = source, indication = null, enabled = enabled, role = Role.Button, onClick = onClick)
            .semantics { contentDescription = description },
        contentAlignment = Alignment.Center,
    ) {
        Canvas(Modifier.fillMaxSize()) {
            val ring = 4.dp.toPx()
            drawCircle(Palette.Text.copy(alpha = 0.9f), size.minDimension / 2 - ring / 2, style = Stroke(ring))
            if (progress != null) {
                drawArc(
                    color, -90f, 360f * prog, false,
                    topLeft = Offset(ring / 2, ring / 2), size = Size(size.width - ring, size.height - ring),
                    style = Stroke(ring, cap = StrokeCap.Round),
                )
            }
            val full = size.minDimension / 2 - ring - 5.dp.toPx() // inner disc radius
            val half = full * (1f - 0.42f * morph)               // morph to a smaller square
            val corner = half * (1f - 0.72f * morph)             // round -> softly rounded
            drawRoundRect(
                color,
                topLeft = Offset(size.width / 2 - half, size.height / 2 - half),
                size = Size(2 * half, 2 * half),
                cornerRadius = CornerRadius(corner, corner),
            )
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Viewfinder furniture
// ---------------------------------------------------------------------------------------------

/** Gyro stability pill: tiny meter + word. Warns amber when the phone is moving too much. */
@Composable
fun StabilityPill(stability: Float, modifier: Modifier = Modifier) {
    val steady = stability >= 0.70f
    val tone by animateColorAsState(if (steady) Palette.Ok else Palette.Warn, tween(200), label = "stab")
    val v by animateFloatAsState(stability.coerceIn(0f, 1f), tween(150), label = "stabV")
    Row(
        modifier
            .height(40.dp)
            .clip(RoundedCornerShape(50))
            .background(Palette.Glass)
            .border(1.dp, tone.copy(alpha = 0.5f), RoundedCornerShape(50))
            .padding(horizontal = 14.dp)
            .semantics { contentDescription = if (steady) "Phone is steady" else "Hold the phone steadier" },
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(Modifier.width(34.dp).height(5.dp).clip(RoundedCornerShape(50)).background(Palette.Outline)) {
            Box(Modifier.fillMaxHeight().fillMaxWidth(v.coerceAtLeast(0.06f)).clip(RoundedCornerShape(50)).background(tone))
        }
        Spacer(Modifier.width(8.dp))
        Text(if (steady) "STEADY" else "HOLD STILL", style = Type.Label.copy(color = tone, fontSize = 10.sp, letterSpacing = 1.1.sp))
    }
}

/** Corner-bracket framing reticle whose colour follows stability. */
@Composable
fun Reticle(color: Color, modifier: Modifier = Modifier) {
    val tone by animateColorAsState(color, tween(200), label = "reticle")
    Canvas(modifier) {
        val sw = 2.5.dp.toPx()
        val len = 28.dp.toPx()
        val w = size.width
        val h = size.height
        val c = tone.copy(alpha = 0.85f)
        fun seg(a: Offset, b: Offset) = drawLine(c, a, b, sw, StrokeCap.Round)
        seg(Offset(0f, len), Offset(0f, 0f)); seg(Offset(0f, 0f), Offset(len, 0f))
        seg(Offset(w - len, 0f), Offset(w, 0f)); seg(Offset(w, 0f), Offset(w, len))
        seg(Offset(0f, h - len), Offset(0f, h)); seg(Offset(0f, h), Offset(len, h))
        seg(Offset(w - len, h), Offset(w, h)); seg(Offset(w, h), Offset(w, h - len))
        val cx = w / 2
        val cy = h / 2
        val t = 7.dp.toPx()
        seg(Offset(cx - t, cy), Offset(cx + t, cy)); seg(Offset(cx, cy - t), Offset(cx, cy + t))
    }
}

/** Translucent status line with a coloured dot (info / busy / ok / error). */
@Composable
fun StatusPill(text: String, level: Level, modifier: Modifier = Modifier) {
    val tone = when (level) {
        Level.OK -> Palette.Ok
        Level.ERROR -> Palette.Error
        Level.BUSY -> Palette.Warn
        Level.INFO -> Palette.TextDim
    }
    Row(
        modifier
            .clip(RoundedCornerShape(20.dp))
            .background(Palette.Glass)
            .border(1.dp, tone.copy(alpha = 0.35f), RoundedCornerShape(20.dp))
            .padding(horizontal = 14.dp, vertical = 9.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Box(Modifier.size(8.dp).clip(CircleShape).background(tone))
        Spacer(Modifier.width(10.dp))
        Text(text, style = Type.Value.copy(fontSize = 13.sp, color = Palette.Text), maxLines = 2)
    }
}

/** Near→far colour bar matching the stereo heat-map palette (red = near, blue = far). */
@Composable
fun DepthLegend(modifier: Modifier = Modifier) {
    Column(
        modifier.clip(RoundedCornerShape(12.dp)).background(Palette.Glass).padding(horizontal = 8.dp, vertical = 8.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("NEAR", style = Type.Label.copy(fontSize = 9.sp, letterSpacing = 0.8.sp))
        Spacer(Modifier.height(4.dp))
        Box(
            Modifier.width(8.dp).height(88.dp).clip(RoundedCornerShape(50)).background(
                Brush.verticalGradient(
                    listOf(Color(0xFFFF2D2D), Color(0xFFFFD02D), Color(0xFF3DFF8A), Color(0xFF2DD3FF), Color(0xFF2D3DFF)),
                ),
            ),
        )
        Spacer(Modifier.height(4.dp))
        Text("FAR", style = Type.Label.copy(fontSize = 9.sp, letterSpacing = 0.8.sp))
    }
}

/** Small "tag" used for key/value facts (e.g. triangle counts). */
@Composable
fun Tag(text: String, color: Color = Palette.TextDim, modifier: Modifier = Modifier) {
    Text(
        text,
        style = Type.Label.copy(color = color, fontSize = 10.sp, letterSpacing = 0.9.sp),
        modifier = modifier
            .clip(RoundedCornerShape(50))
            .background(color.copy(alpha = 0.12f))
            .padding(horizontal = 10.dp, vertical = 5.dp),
    )
}

// ---------------------------------------------------------------------------------------------
// Before / after comparison with pinch-zoom
// ---------------------------------------------------------------------------------------------

/**
 * Draggable before/after reveal. Pinch to zoom (up to 4x), drag to pan, double-tap to reset.
 * The divider handle is its own touch target so it never fights the pan gesture.
 */
@Composable
fun CompareView(
    before: Bitmap,
    after: Bitmap,
    split: Float,
    onSplit: (Float) -> Unit,
    accent: Color,
    modifier: Modifier = Modifier,
) {
    var scale by remember { mutableFloatStateOf(1f) }
    var offset by remember { mutableStateOf(Offset.Zero) }
    val splitState by rememberUpdatedState(split)
    val onSplitState by rememberUpdatedState(onSplit)

    BoxWithConstraints(
        modifier
            .clipToBounds()
            .background(Color.Black)
            .pointerInput(Unit) {
                detectTransformGestures { _, pan, zoom, _ ->
                    val ns = (scale * zoom).coerceIn(1f, 4f)
                    val maxX = size.width * (ns - 1f) / 2f
                    val maxY = size.height * (ns - 1f) / 2f
                    offset = Offset((offset.x + pan.x).coerceIn(-maxX, maxX), (offset.y + pan.y).coerceIn(-maxY, maxY))
                    scale = ns
                }
            }
            .pointerInput(Unit) {
                detectTapGestures(onDoubleTap = { scale = 1f; offset = Offset.Zero })
            },
    ) {
        val wPx = constraints.maxWidth.toFloat()
        val hitPx = with(androidx.compose.ui.platform.LocalDensity.current) { 56.dp.toPx() }
        val layer = Modifier.fillMaxSize().graphicsLayer {
            scaleX = scale; scaleY = scale; translationX = offset.x; translationY = offset.y
        }
        Image(after.asImageBitmap(), "2x super-resolution result", layer, contentScale = ContentScale.Fit, filterQuality = androidx.compose.ui.graphics.FilterQuality.Medium)
        Box(
            Modifier.fillMaxSize().drawWithContent {
                clipRect(right = size.width * splitState) { this@drawWithContent.drawContent() }
            },
        ) {
            Image(before.asImageBitmap(), "1x single-frame baseline", layer, contentScale = ContentScale.Fit, filterQuality = androidx.compose.ui.graphics.FilterQuality.Medium)
        }

        // Divider + handle
        Box(
            Modifier
                .fillMaxHeight()
                .width(56.dp)
                .offset { IntOffset((wPx * split - hitPx / 2f).roundToInt(), 0) }
                .pointerInput(Unit) {
                    detectHorizontalDragGestures { change, dx ->
                        change.consume()
                        onSplitState((splitState + dx / wPx).coerceIn(0.02f, 0.98f))
                    }
                }
                .semantics { contentDescription = "Before and after divider" },
            contentAlignment = Alignment.Center,
        ) {
            Box(Modifier.fillMaxHeight().width(2.dp).background(Color.White.copy(alpha = 0.9f)))
            Box(
                Modifier.size(44.dp).clip(CircleShape).background(Palette.Background.copy(alpha = 0.85f))
                    .border(2.dp, accent, CircleShape),
                contentAlignment = Alignment.Center,
            ) { Icon(Ic.Compare, null, tint = accent, modifier = Modifier.size(22.dp)) }
        }

        Tag("BEFORE · 1×", Color.White, Modifier.align(Alignment.TopStart).padding(12.dp).background(Palette.Scrim, RoundedCornerShape(50)))
        Tag("AFTER · 2×", accent, Modifier.align(Alignment.TopEnd).padding(12.dp).background(Palette.Scrim, RoundedCornerShape(50)))
        if (scale > 1.01f) {
            Tag("${"%.1f".format(scale)}×", Color.White, Modifier.align(Alignment.BottomEnd).padding(12.dp).background(Palette.Scrim, RoundedCornerShape(50)))
        }
    }
}
