package com.example.multicamtest

import android.app.Application
import android.graphics.Bitmap
import android.graphics.Color as AColor
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch

enum class Mode { MACRO, SCAN }
enum class Level { INFO, BUSY, OK, ERROR }

data class UiState(
    val mode: Mode = Mode.MACRO,
    val hasPermission: Boolean = false,
    val running: Boolean = false,
    val message: String = "Choose a mode, then press the shutter.",
    val level: Level = Level.INFO,
    val progress: Float? = null,
    val image: Bitmap? = null,
    val log: List<String> = emptyList(),
    val rawMacro: Boolean = true,
    val fullRes: Boolean = false,
    val showInfo: Boolean = false,
    val showLog: Boolean = false,
)

/** Holds UI state and drives [CaptureEngine], the Camera2 + NDK pipeline. */
class MultiCamViewModel(app: Application) : AndroidViewModel(app), CaptureEngine.Listener {
    private val _ui = MutableStateFlow(UiState())
    val ui: StateFlow<UiState> = _ui.asStateFlow()
    private var engine: CaptureEngine? = null

    private fun engine(): CaptureEngine = engine ?: CaptureEngine(getApplication(), this).also { engine = it }

    fun setPermission(granted: Boolean) = _ui.update { it.copy(hasPermission = granted) }
    fun setMode(m: Mode) {
        if (_ui.value.running) return
        _ui.update { it.copy(mode = m, image = null, progress = null, level = Level.INFO,
            message = if (m == Mode.MACRO) "Hold steady about 10 cm from a flat subject." else "Point both cameras at a textured scene.") }
    }
    fun toggleRaw() = _ui.update { it.copy(rawMacro = !it.rawMacro) }
    fun toggleFullRes() = _ui.update { it.copy(fullRes = !it.fullRes) }
    fun showInfo(v: Boolean) = _ui.update { it.copy(showInfo = v) }
    fun showLog(v: Boolean) = _ui.update { it.copy(showLog = v) }

    fun shutter() {
        val s = _ui.value
        if (!s.hasPermission) return
        if (s.running) {
            engine().stop()
            _ui.update { it.copy(running = false, progress = null, level = Level.INFO, message = "Stopped.") }
            return
        }
        _ui.update { it.copy(running = true, progress = null, level = Level.BUSY, message = "Starting...") }
        if (s.mode == Mode.MACRO) engine().startMacro(s.rawMacro) else engine().startScan(s.fullRes)
    }

    fun savePair() { if (_ui.value.running) engine().savePair() }

    fun exportMesh() {
        viewModelScope.launch(Dispatchers.IO) {
            val tris = engine().exportMesh()
            _ui.update {
                if (tris > 0) it.copy(level = Level.OK, message = "Mesh exported: $tris triangles")
                else it.copy(level = Level.ERROR, message = "No mesh yet - run a 3D scan first.")
            }
        }
    }

    // ---- CaptureEngine.Listener (called from camera / IMU threads) ----
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
                it.mode == Mode.MACRO -> it.copy(progress = null, running = false)
                else -> it.copy(progress = null)
            }
        }
    }

    override fun onLog(line: String) = _ui.update { it.copy(log = (it.log + line).takeLast(200)) }
    override fun onImage(bitmap: Bitmap) = _ui.update { it.copy(image = bitmap) }

    override fun onCleared() {
        engine?.release()
        engine = null
    }
}
