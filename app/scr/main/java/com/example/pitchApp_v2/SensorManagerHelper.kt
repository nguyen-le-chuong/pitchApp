package com.example.myapp

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager

class SensorManagerHelper(private val context: Context) {

    private val sensorManager: SensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null

    // Store the most recent accelerometer and gyroscope values
    private var accelValues: FloatArray = FloatArray(3)
    private var gyroValues: FloatArray = FloatArray(3)

    private var listener: SensorDataListener? = null

    // Set the listener to receive sensor data
    fun setSensorDataListener(listener: SensorDataListener) {
        this.listener = listener
    }

    // Start listening to sensor events
    fun startListening() {
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        if (accelerometer != null) {
            sensorManager.registerListener(sensorEventListener, accelerometer, SensorManager.SENSOR_DELAY_UI)
        }
        if (gyroscope != null) {
            sensorManager.registerListener(sensorEventListener, gyroscope, SensorManager.SENSOR_DELAY_UI)
        }
    }

    // Stop listening to sensor events
    fun stopListening() {
        sensorManager.unregisterListener(sensorEventListener)
    }

    // SensorEventListener to handle accelerometer and gyroscope data
    private val sensorEventListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) return

            when (event.sensor.type) {
                Sensor.TYPE_ACCELEROMETER -> {
                    accelValues = event.values.clone()  // Update accelerometer values
                    listener?.onAccelerometerData(accelValues)
                }
                Sensor.TYPE_GYROSCOPE -> {
                    gyroValues = event.values.clone()  // Update gyroscope values
                    listener?.onGyroscopeData(gyroValues)
                }
            }
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            // Handle sensor accuracy changes if necessary
        }
    }

    // Interface to send data to the calling activity/fragment
    interface SensorDataListener {
        fun onAccelerometerData(accelValues: FloatArray)
        fun onGyroscopeData(gyroValues: FloatArray)
    }
}
