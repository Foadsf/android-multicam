package com.example.multicamtest

import android.app.Application
import android.content.Context
import android.content.Intent
import android.graphics.Bitmap
import android.graphics.Color as AColor
import android.net.Uri
import android.util.Log
import android.view.Surface
import androidx.core.content.FileProvider
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import java.io.File

enum class Mode { INSPECT, SCAN_3D, LABS }
enum class Level { INFO, BUSY, OK, ERROR }

data class UiState(
    val mode: Mode = Mode.INSPECT,
    val hasPermission: Boolean = false,
    val running: Boolean = false,
    val message: String = "Align subject in viewfinder, then tap capture.",
    val level: Level = Level.INFO,
    val stability: Float = 1.0f,
    val progress: Float? = null,
    val image: Bitmap? = null,
    val beforeBitmap: Bitmap? = null,
    val afterBitmap: Bitmap? = null,
    val showComparison: Boolean = false,
    val comparisonSplit: Float = 0.5f,
    val showAfter: Boolean = true,
    val lastResultFile: File? = null,
    val lastStlFile: File? = null,
    val lastPlyFile: File? = null,
    val lastTriangles: Int = 0,
    val log: List<String> = emptyList(),
    val rawMacro: Boolean = true,
    val fullRes: Boolean = false,
    val showGuide: Boolean = false,
    val showConsole: Boolean = false,
    val exporting: Boolean = false,
    val runStartedAt: Long = 0L,
    val pairsSaved: Int = 0,
)

/**
 * ViewModel driving CaptureEngine and maintaining state for Inspect, 3D Scan, and Labs modes.
 */
class MultiCamViewModel(app: Application) : AndroidViewModel(app), CaptureEngine.Listener {
    private val _ui = MutableStateFlow(UiState())
    val ui: StateFlow<UiState> = _ui.asStateFlow()
    private var engine: CaptureEngine? = null

    private fun engine(): CaptureEngine = engine ?: CaptureEngine(getApplication(), this).also { engine = it }

    fun setPreviewSurface(surface: Surface?) {
        engine().setPreviewSurface(surface)
    }

    fun setPermission(granted: Boolean) {
        _ui.update { it.copy(hasPermission = granted) }
        if (granted) {
            engine().resumePreview()
        }
    }

    fun setMode(m: Mode) {
        if (_ui.value.running) return
        _ui.update {
            it.copy(
                mode = m,
                image = null,
                beforeBitmap = null,
                afterBitmap = null,
                showComparison = false,
                progress = null,
                level = Level.INFO,
                message = when (m) {
                    Mode.INSPECT -> "Hold steady about 10 cm from a flat subject."
                    Mode.SCAN_3D -> "Aim both lenses at a textured object, 15\u201340 cm away."
                    Mode.LABS -> "Raw stereo stream, calibration pairs and diagnostics."
                }
            )
        }
        engine().resumePreview()
    }

    fun toggleRaw() = _ui.update { it.copy(rawMacro = !it.rawMacro) }
    fun toggleFullRes() = _ui.update { it.copy(fullRes = !it.fullRes) }
    fun showGuide(v: Boolean) = _ui.update { it.copy(showGuide = v) }
    fun showConsole(v: Boolean) = _ui.update { it.copy(showConsole = v) }

    /** Preview stream size chosen for the live viewfinder (landscape sensor orientation). */
    val previewSize: android.util.Size by lazy { CaptureEngine.choosePreviewSize(getApplication()) }

    /** Back press: close whatever is on top. Returns true if something was dismissed. */
    fun handleBack(): Boolean {
        val s = _ui.value
        return when {
            s.showGuide -> { showGuide(false); true }
            s.showConsole -> { showConsole(false); true }
            s.showComparison || s.lastStlFile != null -> { dismissResult(); true }
            s.running -> { shutter(); true }
            else -> false
        }
    }

    fun toggleComparisonView() {
        _ui.update { it.copy(showAfter = !it.showAfter) }
    }

    fun setComparisonSplit(split: Float) {
        _ui.update { it.copy(comparisonSplit = split.coerceIn(0f, 1f)) }
    }

    fun dismissResult() {
        _ui.update {
            it.copy(
                image = null,
                beforeBitmap = null,
                afterBitmap = null,
                showComparison = false,
                lastStlFile = null,
                lastPlyFile = null,
                lastTriangles = 0,
                message = "Viewfinder ready."
            )
        }
        engine().resumePreview()
    }

    fun shutter() {
        val s = _ui.value
        if (!s.hasPermission) return
        if (s.running) {
            engine().stop()
            _ui.update {
                it.copy(
                    running = false, progress = null, level = Level.INFO, message = "Stopped.",
                    image = if (s.mode == Mode.SCAN_3D) it.image else null,
                )
            }
            // In 3D mode, stopping the stream finishes the scan and builds the mesh.
            if (s.mode == Mode.SCAN_3D) exportMesh()
            return
        }
        _ui.update {
            it.copy(
                running = true,
                runStartedAt = android.os.SystemClock.elapsedRealtime(),
                progress = null,
                level = Level.BUSY,
                showComparison = false,
                image = null,
                beforeBitmap = null,
                afterBitmap = null,
                message = when (s.mode) {
                    Mode.INSPECT -> "Capturing burst\u2026"
                    Mode.SCAN_3D -> "Starting 3D scan\u2026"
                    Mode.LABS -> "Starting stereo stream\u2026"
                }
            )
        }
        if (s.mode == Mode.INSPECT) {
            engine().startMacro(s.rawMacro)
        } else {
            engine().startScan(s.fullRes)
        }
    }

    fun savePair() {
        if (_ui.value.running) {
            engine().savePair()
            _ui.update { it.copy(pairsSaved = it.pairsSaved + 1) }
        }
    }

    fun exportMesh() {
        if (_ui.value.exporting) return
        _ui.update { it.copy(exporting = true, level = Level.BUSY, message = "Building 3D mesh\u2026") }
        viewModelScope.launch(Dispatchers.IO) {
            val tris = engine().exportMesh()
            _ui.update {
                if (tris > 0) it.copy(exporting = false, level = Level.OK, message = "Mesh exported: $tris triangles")
                else it.copy(exporting = false, level = Level.ERROR, message = "No mesh yet \u2014 scan a textured object for a few seconds first.")
            }
        }
    }

    fun shareFile(context: Context, file: File?, mimeType: String) {
        if (file == null || !file.exists()) return
        try {
            val uri: Uri = FileProvider.getUriForFile(context, "${context.packageName}.provider", file)
            val intent = Intent(Intent.ACTION_SEND).apply {
                type = mimeType
                putExtra(Intent.EXTRA_STREAM, uri)
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
            }
            val chooser = Intent.createChooser(intent, "Share ${file.name}").apply {
                addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            context.startActivity(chooser)
        } catch (e: Exception) {
            Log.e("MultiCamVM", "Failed to share file: ${file.name}", e)
        }
    }

    fun openFile(context: Context, file: File?, mimeType: String) {
        if (file == null || !file.exists()) return
        try {
            val uri: Uri = FileProvider.getUriForFile(context, "${context.packageName}.provider", file)
            val intent = Intent(Intent.ACTION_VIEW).apply {
                setDataAndType(uri, mimeType)
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            context.startActivity(intent)
        } catch (e: Exception) {
            Log.e("MultiCamVM", "Failed to open file: ${file.name}", e)
        }
    }

    // ---- CaptureEngine.Listener ----
    override fun onState(message: String, color: Int) {
        val level = when (color) {
            AColor.rgb(0, 230, 118) -> Level.OK
            AColor.rgb(255, 138, 128) -> Level.ERROR
            AColor.rgb(255, 213, 79) -> Level.BUSY
            else -> Level.INFO
        }
        _ui.update { it.copy(message = message, level = level, running = if (level == Level.ERROR) false else it.running) }
    }

    override fun onProgress(value: Int, max: Int) {
        _ui.update {
            when {
                max > 0 -> it.copy(progress = value.toFloat() / max)
                it.mode == Mode.INSPECT -> it.copy(progress = null, running = false)
                else -> it.copy(progress = null)
            }
        }
    }

    override fun onStability(stability: Float) {
        _ui.update { it.copy(stability = stability) }
    }

    override fun onLog(line: String) {
        _ui.update { it.copy(log = (it.log + line).takeLast(200)) }
    }

    override fun onImage(bitmap: Bitmap) {
        _ui.update { it.copy(image = bitmap) }
    }

    override fun onMacroResult(before: Bitmap?, after: Bitmap?, file: File?) {
        _ui.update {
            it.copy(
                beforeBitmap = before,
                afterBitmap = after,
                image = after ?: before,
                lastResultFile = file,
                showComparison = (before != null && after != null),
                running = false,
                progress = null,
                level = Level.OK,
                message = "2\u00d7 Super-Resolution complete!"
            )
        }
    }

    override fun onScanResult(stlFile: File?, plyFile: File?, triangleCount: Int) {
        _ui.update {
            it.copy(
                lastStlFile = stlFile,
                lastPlyFile = plyFile,
                lastTriangles = triangleCount,
                image = if (triangleCount > 0) it.image else null,
                running = false,
                progress = null,
                level = if (triangleCount > 0) Level.OK else Level.ERROR,
                message = if (triangleCount > 0) "3D Model Ready ($triangleCount triangles)." else "No surface reconstructed."
            )
        }
    }

    override fun onCleared() {
        super.onCleared()
        engine?.release()
        engine = null
    }
}
