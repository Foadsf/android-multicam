package com.example.multicamtest

import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Shapes
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * Dark "optical instrument" look: ink-black canvas, cool grey glass surfaces and one accent colour
 * per mode (teal = inspect, violet = 3D scan, orange = labs) so the active mode is obvious at a glance.
 */
object Palette {
    val Background = Color(0xFF07090C)
    val Surface = Color(0xFF10151C)
    val SurfaceHigh = Color(0xFF1A212B)
    val SurfaceHighest = Color(0xFF242D39)
    val Outline = Color(0xFF2A333F)
    val Text = Color(0xFFE9EEF4)
    val TextDim = Color(0xFF8E9BAD)
    val Accent = Color(0xFF5EEAD4)
    val AccentDim = Color(0xFF1F4F49)
    val Violet = Color(0xFFA5B4FC)
    val Orange = Color(0xFFFDBA74)
    val Warn = Color(0xFFFBBF24)
    val Error = Color(0xFFFF8A80)
    val Ok = Color(0xFF4ADE80)
    val Scrim = Color(0xAA000000)
    val Glass = Color(0xB3101419)

    val PanelBrush = Brush.verticalGradient(listOf(Color(0xFF131921), Color(0xFF0B0F14)))

    fun accentFor(mode: Mode): Color = when (mode) {
        Mode.INSPECT -> Accent
        Mode.SCAN_3D -> Violet
        Mode.LABS -> Orange
    }
}

object Type {
    val Title = TextStyle(fontSize = 20.sp, fontWeight = FontWeight.SemiBold, color = Palette.Text)
    val Headline = TextStyle(fontSize = 28.sp, fontWeight = FontWeight.Bold, letterSpacing = (-0.4).sp, color = Palette.Text)
    val Body = TextStyle(fontSize = 14.sp, lineHeight = 20.sp, color = Palette.Text)
    val Label = TextStyle(fontSize = 11.sp, fontWeight = FontWeight.SemiBold, letterSpacing = 1.4.sp, color = Palette.TextDim)
    val Value = TextStyle(fontSize = 13.sp, fontWeight = FontWeight.Medium, fontFeatureSettings = "tnum", color = Palette.Text)
    val Small = TextStyle(fontSize = 12.sp, lineHeight = 16.sp, fontFeatureSettings = "tnum", color = Palette.TextDim)
    val Button = TextStyle(fontSize = 14.sp, fontWeight = FontWeight.SemiBold, letterSpacing = 0.2.sp)
    val Tab = TextStyle(fontSize = 12.sp, fontWeight = FontWeight.SemiBold, letterSpacing = 0.8.sp)
}

@Composable
fun MultiCamTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = darkColorScheme(
            primary = Palette.Accent,
            onPrimary = Palette.Background,
            background = Palette.Background,
            surface = Palette.Surface,
            surfaceContainerLow = Palette.Surface,
            surfaceContainer = Palette.Surface,
            surfaceContainerHigh = Palette.SurfaceHigh,
            onSurface = Palette.Text,
            onBackground = Palette.Text,
            outline = Palette.Outline,
        ),
        shapes = Shapes(
            small = RoundedCornerShape(10.dp),
            medium = RoundedCornerShape(16.dp),
            large = RoundedCornerShape(24.dp),
            extraLarge = RoundedCornerShape(32.dp),
        ),
        content = content,
    )
}
