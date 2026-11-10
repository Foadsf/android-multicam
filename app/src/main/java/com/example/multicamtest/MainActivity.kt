package com.example.multicamtest

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.SystemBarStyle
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels

class MainActivity : ComponentActivity() {
    private val vm: MultiCamViewModel by viewModels()

    private val requestCamera =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { vm.setPermission(it) }

    private val pickCalibration =
        registerForActivityResult(ActivityResultContracts.OpenDocument()) { uri -> if (uri != null) vm.importCalibration(uri) }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge(
            statusBarStyle = SystemBarStyle.dark(android.graphics.Color.TRANSPARENT),
            navigationBarStyle = SystemBarStyle.dark(android.graphics.Color.TRANSPARENT),
        )
        setContent {
            MultiCamTheme {
                MultiCamScreen(
                    vm,
                    onRequestPermission = { requestCamera.launch(Manifest.permission.CAMERA) },
                    onImportCalibration = { pickCalibration.launch(arrayOf("application/json", "text/plain", "application/octet-stream")) },
                )
            }
        }
    }

    override fun onResume() {
        super.onResume()
        vm.setPermission(checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED)
        vm.onForeground()
    }

    override fun onPause() {
        vm.onBackground()   // release the camera and sensors whenever the UI is not visible
        super.onPause()
    }
}
