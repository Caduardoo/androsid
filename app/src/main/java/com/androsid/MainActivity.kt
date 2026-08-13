package com.androsid

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.Gravity
import android.view.ViewGroup
import android.widget.Button
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.ComponentActivity
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat

/**
 * Permission gate and an on/off switch. All the real work lives in [SensorService];
 * this activity can be closed once the service is running.
 */
class MainActivity : ComponentActivity() {

    private lateinit var status: TextView

    private val required: Array<String>
        get() {
            val perms = mutableListOf(
                Manifest.permission.CAMERA,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION
            )
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                perms += Manifest.permission.POST_NOTIFICATIONS
            }
            return perms.toTypedArray()
        }

    private val requestPermissions = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { startService() }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(buildUi())
        refreshStatus()
    }

    private fun buildUi(): ViewGroup {
        val pad = (24 * resources.displayMetrics.density).toInt()

        status = TextView(this).apply {
            textSize = 16f
            setPadding(0, 0, 0, pad)
        }

        val start = Button(this).apply {
            text = "Start streaming"
            setOnClickListener {
                val missing = required.filter {
                    ContextCompat.checkSelfPermission(this@MainActivity, it) !=
                        PackageManager.PERMISSION_GRANTED
                }
                if (missing.isEmpty()) startService() else requestPermissions.launch(missing.toTypedArray())
            }
        }

        val stop = Button(this).apply {
            text = "Stop"
            setOnClickListener {
                stopService(Intent(this@MainActivity, SensorService::class.java))
                status.text = "Stopped."
            }
        }

        return LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            gravity = Gravity.CENTER
            setPadding(pad, pad, pad, pad)
            addView(status)
            addView(start)
            addView(stop)
        }
    }

    private fun startService() {
        ContextCompat.startForegroundService(
            this, Intent(this, SensorService::class.java)
        )
        refreshStatus()
    }

    private fun refreshStatus() {
        val granted = required.filter {
            ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED
        }
        status.text = buildString {
            append("andROSid\n\n")
            append("Listening on port ${SensorService.PORT}\n")
            append("Connect from proot: 127.0.0.1:${SensorService.PORT}\n\n")
            append("Permissions granted: ${granted.size}/${required.size}")
        }
    }
}
