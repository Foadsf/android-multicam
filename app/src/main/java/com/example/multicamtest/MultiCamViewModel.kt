package com.example.multicamtest

import android.app.Application
import android.content.Context
import android.content.Intent
import android.graphics.Bitmap
import android.net.Uri
import android.os.SystemClock
import android.util.Log
import android.view.Surface
import androidx.core.content.FileProvider
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File

enum class Mode { INSPECT, SCAN_3D, LABS }
enum class Level { INFO, BUSY, OK, ERROR }

/** Coarse UI state. High-frequency values (stability, depth frames) have their own flows to avoid recomposing everything. */
data class UiState(
    val mode: Mode = Mode.INSPECT,
    val hasPermission: Boolean = false,
    val running: Boolean = false,
    val message: String = "Hold steady about 10 cm from a flat subject.",
    val level: Level = Level.INFO,
    val progress: Float? = null,
    val beforeBitmap: Bitmap? = null,
    val afterBitmap: Bitmap? = null,
    val showComparison: Boolean = false,
    val comparisonSplit: Float = 0.5f,
    val resultW: Int = 0,
    val resultH: Int = 0,
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
    // device capabilities / geometry (filled by the engine)
    val stereoAvailable: Boolean = true,
    val rawAvailable: Boolean = true,
    val baselineMm: Float = 0f,
    val zMinMm: Float = 0f,
    val zMaxMm: Float = 0f,
    val calibration: String = "",
)

class MultiCamViewModel(app: Application) : AndroidViewModel(app), CaptureEngine.Listener {
    private val _ui = MutableStateFlow(UiState())
    val ui: StateFlow<UiState> = _ui.asStateFlow()

    private val _stability = MutableStateFlow(1.0f)
    val stability: StateFlow<Float> = _stability.asStateFlow()

    private val _depth = MutableStateFlow<Bitmap?>(null)
    val depth: StateFlow<Bitmap?> = _depth.asStateFlow()

    val hasDepth: StateFlow<Boolean> = _depth.map { it != null }.distinctUntilChanged()
        .stateIn(viewModelScope, SharingStarted.Eagerly, false)

    private var engine: CaptureEngine? = null
    private fun engine(): CaptureEngine = engine ?: CaptureEngine(getApplication(), this).also { engine = it }

    /** Preview stream size (landscape sensor orientation); computed once from the camera metadata. */
    val previewSize: android.util.Size by lazy { CaptureEngine.choosePreviewSize(getApplication()) }

    fun setPreviewSurface(surface: Surface?) = engine().setPreviewSurface(surface)

    fun setPermission(granted: Boolean) {
        _ui.update { it.copy(hasPermission = granted) }
        if (granted) { engine().setActive(true); engine().resumePreview() }
    }

    /** UI went to the background: stop captures and release the camera. */
    fun onBackground() {
        engine?.pause()
        _ui.update { it.copy(running = false, progress = null, exporting = false, level = if (it.level == Level.BUSY) Level.INFO else it.level) }
    }

    fun onForeground() { if (_ui.value.hasPermission) { engine().setActive(true); engine().resumePreview() } }

    fun setMode(m: Mode) {
        if (_ui.value.running || _ui.value.exporting) return
        _depth.value = null
        _ui.update {
            it.copy(
                mode = m, beforeBitmap = null, afterBitmap = null, showComparison = false,
                lastStlFile = null, lastPlyFile = null, lastTriangles = 0, progress = null, level = Level.INFO,
                message = when (m) {
                    Mode.INSPECT -> "Hold steady about 10 cm from a flat subject."
                    Mode.SCAN_3D -> if (it.stereoAvailable) "Aim both lenses at a textured object." else "This phone does not expose a usable dual-camera pair."
                    Mode.LABS -> "Raw stereo stream, calibration pairs and diagnostics."
                },
            )
        }
        engine().resumePreview()
    }

    fun toggleRaw() = _ui.update { if (it.rawAvailable) it.copy(rawMacro = !it.rawMacro) else it }
    fun toggleFullRes() = _ui.update { it.copy(fullRes = !it.fullRes) }
    fun showGuide(v: Boolean) = _ui.update { it.copy(showGuide = v) }
    fun showConsole(v: Boolean) = _ui.update { it.copy(showConsole = v) }
    fun setComparisonSplit(split: Float) = _ui.update { it.copy(comparisonSplit = split.coerceIn(0f, 1f)) }

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

    fun dismissResult() {
        _depth.value = null
        _ui.update {
            it.copy(
                beforeBitmap = null, afterBitmap = null, showComparison = false,
                lastStlFile = null, lastPlyFile = null, lastTriangles = 0, message = "Viewfinder ready.", level = Level.INFO,
            )
        }
        engine().resumePreview()
    }

    fun shutter() {
        val s = _ui.value
        if (!s.hasPermission || s.exporting) return
        if (s.running) {
            engine().stop()
            _ui.update { it.copy(running = false, progress = null, level = Level.INFO, message = "Stopped.") }
            if (s.mode == Mode.SCAN_3D) exportMesh()   // stopping a 3D scan finishes it and builds the mesh
            else _depth.value = null
            return
        }
        if (s.mode != Mode.INSPECT && !s.stereoAvailable) {
            _ui.update { it.copy(level = Level.ERROR, message = "3D scanning needs two back cameras exposed as one logical camera — not available on this phone.") }
            return
        }
        _depth.value = null
        _ui.update {
            it.copy(
                running = true, runStartedAt = SystemClock.elapsedRealtime(), progress = null, level = Level.BUSY,
                showComparison = false, beforeBitmap = null, afterBitmap = null,
                message = when (s.mode) {
                    Mode.INSPECT -> "Preparing burst…"
                    Mode.SCAN_3D -> "Starting 3D scan…"
                    Mode.LABS -> "Starting stereo stream…"
                },
            )
        }
        if (s.mode == Mode.INSPECT) engine().startMacro(s.rawMacro) else engine().startScan(s.fullRes)
    }

    fun savePair() {
        if (_ui.value.running) { engine().savePair(); _ui.update { it.copy(pairsSaved = it.pairsSaved + 1) } }
    }

    fun exportMesh() {
        if (_ui.value.exporting) return
        _ui.update { it.copy(exporting = true, level = Level.BUSY, message = "Building 3D mesh…") }
        viewModelScope.launch(Dispatchers.Default) {
            val tris = engine().exportMesh()
            _ui.update {
                if (tris > 0) it.copy(exporting = false, level = Level.OK, message = "Mesh ready: $tris triangles")
                else it.copy(exporting = false, level = Level.ERROR, message = "No surface yet — scan a textured object for a few seconds first.")
            }
        }
    }

    /** Copies a user-selected stereo_calibration.json into the app folder (validated as JSON first). */
    fun importCalibration(uri: Uri) {
        viewModelScope.launch {
            val msg = withContext(Dispatchers.IO) {
                try {
                    val ctx = getApplication<Application>()
                    val text = ctx.contentResolver.openInputStream(uri)?.use { it.readBytes().toString(Charsets.UTF_8) }
                        ?: return@withContext "Could not read the selected file." to Level.ERROR
                    org.json.JSONObject(text)  // throws when malformed
                    File(engine().outputDir(), StereoCalibration.FILE_NAME).writeText(text)
                    "Calibration imported — it will be used by the next scan." to Level.OK
                } catch (e: Exception) {
                    "That file is not a valid calibration JSON: ${e.message}" to Level.ERROR
                }
            }
            _ui.update { it.copy(message = msg.first, level = msg.second) }
            onLog(msg.first)
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
            context.startActivity(Intent.createChooser(intent, "Share ${file.name}").addFlags(Intent.FLAG_ACTIVITY_NEW_TASK))
        } catch (e: Exception) {
            Log.e("MultiCamVM", "Failed to share ${file.name}", e)
            _ui.update { it.copy(level = Level.ERROR, message = "Nothing on this phone can handle that share.") }
        }
    }

    fun openFile(context: Context, file: File?, mimeType: String) {
        if (file == null || !file.exists()) return
        try {
            val uri: Uri = FileProvider.getUriForFile(context, "${context.packageName}.provider", file)
            val intent = Intent(Intent.ACTION_VIEW).apply {
                setDataAndType(uri, mimeType)
                addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            context.startActivity(intent)
        } catch (e: Exception) {
            Log.e("MultiCamVM", "Failed to open ${file.name}", e)
            _ui.update { it.copy(level = Level.ERROR, message = "No app installed can open ${file.extension.uppercase()} files — use Share instead.") }
        }
    }

    // ---- CaptureEngine.Listener ----
    override fun onState(message: String, level: Int) {
        val l = when (level) {
            CaptureEngine.Listener.LEVEL_OK -> Level.OK
            CaptureEngine.Listener.LEVEL_ERROR -> Level.ERROR
            CaptureEngine.Listener.LEVEL_BUSY -> Level.BUSY
            else -> Level.INFO
        }
        _ui.update { it.copy(message = message, level = l, running = if (l == Level.ERROR) false else it.running, progress = if (l == Level.ERROR) null else it.progress) }
    }

    override fun onProgress(value: Int, max: Int) {
        _ui.update { if (max > 0) it.copy(progress = (value.toFloat() / max).coerceIn(0f, 1f)) else it.copy(progress = null) }
    }

    override fun onStability(stability: Float) {
        // quantised so tiny sensor jitter does not cause recomposition
        val q = Math.round(stability * 20f) / 20f
        if (q != _stability.value) _stability.value = q
    }

    override fun onLog(line: String) { _ui.update { it.copy(log = (it.log + line).takeLast(300)) } }

    override fun onDepthFrame(bitmap: Bitmap) { _depth.value = bitmap }

    override fun onMacroResult(before: Bitmap?, after: Bitmap?, file: File?, outWidth: Int, outHeight: Int) {
        _ui.update {
            it.copy(
                beforeBitmap = before, afterBitmap = after, lastResultFile = file, resultW = outWidth, resultH = outHeight,
                showComparison = before != null && after != null, running = false, progress = null,
                level = Level.OK, message = "2× super-resolution ready.",
            )
        }
    }

    override fun onScanResult(stlFile: File?, plyFile: File?, triangleCount: Int) {
        if (triangleCount <= 0) _depth.value = null
        _ui.update {
            it.copy(
                lastStlFile = stlFile, lastPlyFile = plyFile, lastTriangles = triangleCount, running = false, progress = null,
                level = if (triangleCount > 0) Level.OK else Level.ERROR,
                message = if (triangleCount > 0) "3D model ready ($triangleCount triangles)." else "No surface reconstructed.",
            )
        }
    }

    override fun onStereoInfo(baselineMm: Float, zMinMm: Float, zMaxMm: Float, calibration: String) {
        _ui.update { it.copy(baselineMm = baselineMm, zMinMm = zMinMm, zMaxMm = zMaxMm, calibration = calibration) }
    }

    override fun onCapabilities(stereo: Boolean, raw: Boolean) {
        _ui.update { it.copy(stereoAvailable = stereo, rawAvailable = raw, rawMacro = it.rawMacro && raw) }
    }

    override fun onCleared() {
        super.onCleared()
        engine?.release()
        engine = null
    }
}
