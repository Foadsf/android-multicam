package com.example.multicamtest

import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.SolidColor
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.graphics.vector.addPathNodes
import androidx.compose.ui.unit.dp

/** Tiny stroke-icon set (24x24 viewport) so the app needs no icon-font or extended-icons dependency. */
object Ic {
    private fun icon(name: String, vararg paths: String, width: Float = 1.8f): ImageVector =
        ImageVector.Builder(name, 24.dp, 24.dp, 24f, 24f).apply {
            for (d in paths) {
                addPath(
                    pathData = addPathNodes(d),
                    stroke = SolidColor(Color.White),
                    strokeLineWidth = width,
                    strokeLineCap = StrokeCap.Round,
                    strokeLineJoin = StrokeJoin.Round,
                )
            }
        }.build()

    val Macro by lazy { icon("macro", "M10.5,3.5 a7,7 0 1,0 0,14 a7,7 0 1,0 0,-14", "M16,16 L21,21", "M8,10.5 L13,10.5 M10.5,8 L10.5,13") }
    val Cube by lazy { icon("cube", "M12,2.5 L20.5,7.25 L20.5,16.75 L12,21.5 L3.5,16.75 L3.5,7.25 Z", "M3.5,7.25 L12,12 L20.5,7.25", "M12,12 L12,21.5") }
    val Labs by lazy {
        icon(
            "labs",
            "M4,7 L20,7", "M4,17 L20,17",
            "M9,4.5 a2.5,2.5 0 1,0 0.01,0", "M15,14.5 a2.5,2.5 0 1,0 0.01,0",
        )
    }
    val Help by lazy { icon("help", "M12,3 a9,9 0 1,0 0.01,0", "M9.6,9.4 a2.5,2.5 0 1,1 3.6,2.2 c-0.8,0.5 -1.2,1 -1.2,1.9", "M12,17 L12,17.01") }
    val Console by lazy {
        icon(
            "console",
            "M4,5 L20,5 a1,1 0 0 1 1,1 L21,18 a1,1 0 0 1 -1,1 L4,19 a1,1 0 0 1 -1,-1 L3,6 a1,1 0 0 1 1,-1 Z",
            "M7,10 L10,12.5 L7,15", "M12.5,15 L16.5,15",
        )
    }
    val Share by lazy { icon("share", "M12,15 L12,3.5", "M7.5,8 L12,3.5 L16.5,8", "M5,13 L5,18 a2,2 0 0 0 2,2 L17,20 a2,2 0 0 0 2,-2 L19,13") }
    val Open by lazy { icon("open", "M14,4 L20,4 L20,10", "M20,4 L11,13", "M18,14 L18,19 a1,1 0 0 1 -1,1 L5,20 a1,1 0 0 1 -1,-1 L4,7 a1,1 0 0 1 1,-1 L10,6") }
    val Retake by lazy { icon("retake", "M20,12 a8,8 0 1,1 -2.4,-5.7", "M20,4 L20,9 L15,9") }
    val Close by lazy { icon("close", "M6,6 L18,18", "M18,6 L6,18") }
    val Check by lazy { icon("check", "M5,12.5 L10,17.5 L19,7") }
    val Pair by lazy { icon("pair", "M3,6 L10.5,6 L10.5,18 L3,18 Z", "M13.5,6 L21,6 L21,18 L13.5,18 Z") }
    val Download by lazy { icon("download", "M12,3.5 L12,15", "M7,10.5 L12,15.5 L17,10.5", "M5,20 L19,20") }
    val Copy by lazy {
        icon(
            "copy",
            "M9,9 L19,9 a1,1 0 0 1 1,1 L20,20 a1,1 0 0 1 -1,1 L9,21 a1,1 0 0 1 -1,-1 L8,10 a1,1 0 0 1 1,-1 Z",
            "M5,15 L4,15 a1,1 0 0 1 -1,-1 L3,4 a1,1 0 0 1 1,-1 L14,3 a1,1 0 0 1 1,1 L15,5",
        )
    }
    val Compare by lazy { icon("compare", "M12,3 L12,21", "M7,8 L3,12 L7,16", "M17,8 L21,12 L17,16") }
    val Stop by lazy { icon("stop", "M7,7 L17,7 L17,17 L7,17 Z") }
    val Lens by lazy {
        icon(
            "lens",
            "M12,2.5 a9.5,9.5 0 1,0 0.01,0", "M12,7 a5,5 0 1,0 0.01,0",
            "M12,9.6 a2.4,2.4 0 1,0 0.01,0",
            width = 1.4f,
        )
    }
    val Lock by lazy { icon("lock", "M6,11 L18,11 a1,1 0 0 1 1,1 L19,20 a1,1 0 0 1 -1,1 L6,21 a1,1 0 0 1 -1,-1 L5,12 a1,1 0 0 1 1,-1 Z", "M8,11 L8,7.5 a4,4 0 0 1 8,0 L16,11") }
    val Chip by lazy { icon("chip", "M7,7 L17,7 L17,17 L7,17 Z", "M9.5,3.5 L9.5,7 M14.5,3.5 L14.5,7 M9.5,17 L9.5,20.5 M14.5,17 L14.5,20.5 M3.5,9.5 L7,9.5 M3.5,14.5 L7,14.5 M17,9.5 L20.5,9.5 M17,14.5 L20.5,14.5") }
}
