package com.example.pitchekf.src

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.widget.TextView

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null

    private lateinit var pitchTextView: TextView
    private lateinit var slopeTextView: TextView
    private lateinit var alphaTextView: TextView

    init {
        System.loadLibrary("yourlibraryname") // Load native C++ library
    }

    external fun initKalman()
    external fun updateKalmanState(accData: FloatArray, gyroData: FloatArray, timestamp: Long): PitchSlopeResult?

    private var accData = FloatArray(3)
    private var gyroData = FloatArray(3)
    private var accReady = false
    private var gyroReady = false

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        pitchTextView = findViewById(R.id.pitchTextView)
        slopeTextView = findViewById(R.id.slopeTextView)
        alphaTextView = findViewById(R.id.alphaTextView)

        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_FASTEST)
        sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_FASTEST)

        initKalman()
    }

    override fun onSensorChanged(event: SensorEvent?) {
        if (event == null) return

        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                accData = event.values.clone()
                accReady = true
            }
            Sensor.TYPE_GYROSCOPE -> {
                gyroData = event.values.clone()
                gyroReady = true
            }
        }

        if (accReady && gyroReady) {
            val timestamp = System.currentTimeMillis()
            val result = updateKalmanState(accData, gyroData, timestamp)
            result?.let {
                runOnUiThread {
                    pitchTextView.text = "Pitch: ${it.pitch}"
                    slopeTextView.text = "Slope: ${it.slope}"
                }
            }
            accReady = false
            gyroReady = false
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    override fun onDestroy() {
        super.onDestroy()
        sensorManager.unregisterListener(this)
    }
}
