package com.example.multicamtest

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels

class MainActivity : ComponentActivity() {
    private val vm: MultiCamViewModel by viewModels()

    private val requestCamera =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { vm.setPermission(it) }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            MultiCamTheme {
                MultiCamScreen(vm) { requestCamera.launch(Manifest.permission.CAMERA) }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        vm.setPermission(checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED)
    }
}
