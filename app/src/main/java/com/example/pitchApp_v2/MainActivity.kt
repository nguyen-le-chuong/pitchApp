// package com.example.myapp

// import android.hardware.Sensor
// import android.hardware.SensorEvent
// import android.hardware.SensorEventListener
// import android.hardware.SensorManager
// import android.os.Bundle
// import android.util.Log
// import android.widget.Button
// import androidx.appcompat.app.AppCompatActivity

// class MainActivity : AppCompatActivity(), SensorEventListener {

//     // Native method to update the simulation
//     external fun updateKalman(acc: DoubleArray, gyro: DoubleArray, timeStamp: Long)

//     // Load the native library
//     init {
//         System.loadLibrary("yourlibraryname")
//     }

//     private lateinit var sensorManager: SensorManager
//     private var accelerometer: Sensor? = null
//     private var gyroscope: Sensor? = null

//     private var accData = DoubleArray(3) { 0.0 }
//     private var gyroData = DoubleArray(3) { 0.0 }

//     override fun onCreate(savedInstanceState: Bundle?) {
//         super.onCreate(savedInstanceState)
//         setContentView(R.layout.activity_main)

//         sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
//         accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
//         gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

//         val resetButton: Button = findViewById(R.id.resetButton)
//         resetButton.setOnClickListener {
//             Log.d("MainActivity", "Resetting Kalman Filter")
//             resetKalman()
//         }
//     }

//     override fun onResume() {
//         super.onResume()
//         accelerometer?.let {
//             sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_UI)
//         }
//         gyroscope?.let {
//             sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_UI)
//         }
//     }

//     override fun onPause() {
//         super.onPause()
//         sensorManager.unregisterListener(this)
//     }

//     override fun onSensorChanged(event: SensorEvent?) {
//         if (event == null) return

//         when (event.sensor.type) {
//             Sensor.TYPE_ACCELEROMETER -> {
//                 accData[0] = event.values[0].toDouble()
//                 accData[1] = event.values[1].toDouble()
//                 accData[2] = event.values[2].toDouble()
//             }
//             Sensor.TYPE_GYROSCOPE -> {
//                 gyroData[0] = event.values[0].toDouble()
//                 gyroData[1] = event.values[1].toDouble()
//                 gyroData[2] = event.values[2].toDouble()
//             }
//         }

//         // Call native method to update the Kalman filter with IMU data
//         updateKalman(accData, gyroData, System.currentTimeMillis())
//     }

//     override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
//         // No action needed for accuracy changes
//     }

//     // Native function to reset the Kalman filter
//     external fun resetKalman()
// }

package com.example.pitchApp_v2

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null

    private val accData = FloatArray(3)
    private val gyroData = FloatArray(3)
    private var lastTimestamp: Long = 0

    // Native methods
    private external fun initKalman(cor: Float)
    private external fun setKalmanParameters(accelStd: Float, gyroStd: Float, accelBias: Float, gyroBias: Float, initVel: Float, cA: Float, numR2: Float, numNG: Float, cor: Float)
    private external fun updateKalmanState(accData: FloatArray, gyroData: FloatArray, timestamp: Long): PitchSlopeResult

    init {
        System.loadLibrary("native-lib")
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize SensorManager and sensors
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

        val accelStdInput = findViewById<EditText>(R.id.accelStdInput)
        val gyroStdInput = findViewById<EditText>(R.id.gyroStdInput)
        val accelBiasInput = findViewById<EditText>(R.id.accelBiasInput)
        val gyroBiasInput = findViewById<EditText>(R.id.gyroBiasInput)
        val initVelInput = findViewById<EditText>(R.id.initVelStdInput)
        val cAInput = findViewById<EditText>(R.id.cAInput)
        val numR2Input = findViewById<EditText>(R.id.numR2Input)
        val numNGInput = findViewById<EditText>(R.id.numNGInput)
        val corInput = findViewById<EditText>(R.id.corInput)

        // val pitchTextView = findViewById<TextView>(R.id.pitchValue)
        // val slopeTextView = findViewById<TextView>(R.id.roadSlopeValue)
        // val pitchMinusSlopeTextView = findViewById<TextView>(R.id.pitchMinusSlopeValue)

        val btnReset = findViewById<Button>(R.id.resetButton)
        val btnIncAccelStd = findViewById<Button>(R.id.plusAccelStd)
        val btnDecAccelStd = findViewById<Button>(R.id.minusAccelStd)
        val btnIncGyroStd = findViewById<Button>(R.id.plusGyroStd)
        val btnDecGyroStd = findViewById<Button>(R.id.minusGyroStd)
        val btnIncAccelBias = findViewById<Button>(R.id.plusAccelBias)
        val btnDecAccelBias = findViewById<Button>(R.id.minusAccelBias)
        val btnIncGyroBias = findViewById<Button>(R.id.plusGyroBias)
        val btnDecGyroBias = findViewById<Button>(R.id.minusGyroBias)
        val btnIncInitVel = findViewById<Button>(R.id.plusInitVelStd)
        val btnDecInitVel = findViewById<Button>(R.id.minusInitVelStd)
        val btnIncCA = findViewById<Button>(R.id.plusCA)
        val btnDecCA = findViewById<Button>(R.id.minusCA)
        val btnIncNumR2 = findViewById<Button>(R.id.plusNumR2)
        val btnDecNumR2 = findViewById<Button>(R.id.minusNumR2)
        val btnIncNumNG = findViewById<Button>(R.id.plusNumNG)
        val btnDecNumNG = findViewById<Button>(R.id.minusNumNG)
        val btnIncCor = findViewById<Button>(R.id.plusCor)
        val btnDecCor = findViewById<Button>(R.id.minusCor)

        btnReset.setOnClickListener {
            val accelStd = accelStdInput.text.toString().toFloatOrNull() ?: 0.01f
            val gyroStd = gyroStdInput.text.toString().toFloatOrNull() ?: 0.01f
            val accelBias = accelBiasInput.text.toString().toFloatOrNull() ?: 0.0f
            val gyroBias = gyroBiasInput.text.toString().toFloatOrNull() ?: 0.0f
            val initVel = initVelInput.text.toString().toFloatOrNull() ?: 0.0f
            val cA = cAInput.text.toString().toFloatOrNull() ?: 0.0f
            val numR2 = numR2Input.text.toString().toFloatOrNull() ?: 0.0f
            val numNG = numNGInput.text.toString().toFloatOrNull() ?: 0.0f
            val cor = corInput.text.toString().toFloatOrNull() ?: 0.0f

            setKalmanParameters(accelStd, gyroStd, accelBias, gyroBias, initVel, cA, numR2, numNG, cor)
            initKalman(cor)
        }

        // Set up increment and decrement buttons
        setupIncrementDecrement(btnIncAccelStd, btnDecAccelStd, accelStdInput, 0.01f)
        setupIncrementDecrement(btnIncGyroStd, btnDecGyroStd, gyroStdInput, 0.01f)
        setupIncrementDecrement(btnIncAccelBias, btnDecAccelBias, accelBiasInput, 0.01f)
        setupIncrementDecrement(btnIncGyroBias, btnDecGyroBias, gyroBiasInput, 0.01f)
        setupIncrementDecrement(btnIncInitVel, btnDecInitVel, initVelInput, 0.01f)
        setupIncrementDecrement(btnIncCA, btnDecCA, cAInput, 0.01f)
        setupIncrementDecrement(btnIncNumR2, btnDecNumR2, numR2Input, 0.01f)
        setupIncrementDecrement(btnIncNumNG, btnDecNumNG, numNGInput, 0.01f)
        setupIncrementDecrement(btnIncCor, btnDecCor, corInput, 0.01f)

        // Register sensors
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME)
        sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME)
    }

    private fun setupIncrementDecrement(incButton: Button, decButton: Button, editText: EditText, step: Float) {
        incButton.setOnClickListener {
            val currentValue = editText.text.toString().toFloatOrNull() ?: 0.0f
            editText.setText(String.format("%.2f", currentValue + step))
        }
        decButton.setOnClickListener {
            val currentValue = editText.text.toString().toFloatOrNull() ?: 0.0f
            editText.setText(String.format("%.2f", currentValue - step))
        }
    }

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                accData[0] = event.values[0]
                accData[1] = event.values[1]
                accData[2] = event.values[2]
            }
            Sensor.TYPE_GYROSCOPE -> {
                gyroData[0] = event.values[0]
                gyroData[1] = event.values[1]
                gyroData[2] = event.values[2]
            }
        }

        // Get the current system time
        val currentTimestamp = System.currentTimeMillis()

        // Send sensor data to Kalman Filter if we have a valid timestamp
        if (lastTimestamp != 0L) {
            val result = updateKalmanState(accData, gyroData, currentTimestamp)
            displayKalmanResults(result)
        }

        lastTimestamp = currentTimestamp
    }

    private fun displayKalmanResults(result: PitchSlopeResult) {
        val pitchTextView = findViewById<TextView>(R.id.pitchValue)
        val slopeTextView = findViewById<TextView>(R.id.roadSlopeValue)
        val pitchMinusSlopeTextView = findViewById<TextView>(R.id.pitchMinusSlopeValue)

        pitchTextView.text = String.format("%.2f", result.pitch)
        slopeTextView.text = String.format("%.2f", result.slope)
        pitchMinusSlopeTextView.text = String.format("%.2f", result.pitchMinusSlope)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Not needed for this implementation
    }

    override fun onDestroy() {
        super.onDestroy()
        sensorManager.unregisterListener(this)
    }
}

