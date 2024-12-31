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

import android.annotation.SuppressLint
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.annotation.RequiresApi
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.app.AlertDialog
import java.io.File
import java.io.FileWriter
import java.io.BufferedWriter
import android.os.Environment
import androidx.core.content.ContextCompat
import android.Manifest
import android.content.pm.PackageManager
import androidx.core.app.ActivityCompat

class MainActivity : AppCompatActivity(), SensorEventListener {

    private lateinit var sensorManager: SensorManager
    private var accelerometer: Sensor? = null
    private var acceler_bias: Sensor? = null
    private var gyroscope: Sensor? = null
    private var gravity: Sensor? = null

    private val accData = FloatArray(3)
    private val gyroData = FloatArray(3)
    private var lastTimestamp: Long = 0


    // Native methods
    private external fun initKalman(cor: Float)
    private external fun setKalmanParameters(
        parameters: FloatArray
    )
    private external fun updateKalmanState(accData: FloatArray, gyroData: FloatArray, timestamp: Long): PitchSlopeResult

    init {
        System.loadLibrary("native-lib")
    }

    fun writeSensorDataToCSV(timestamp: Long, accData: FloatArray, gyroData: FloatArray) {
        try {
            // Ensure the file is on external storage (or internal if preferred)
            val fileName = "/sensor_data.csv"
            val filePath = File(Environment.getExternalStorageDirectory(), fileName)

            // Check if the file exists, if not create it and write headers
            if (!filePath.exists()) {
                filePath.createNewFile()
                val writer = BufferedWriter(FileWriter(filePath))
                writer.write("Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z\n")
                writer.close()
            }

            // Append data to the file
            val writer = BufferedWriter(FileWriter(filePath, true))
            writer.write("$timestamp,${accData[0]},${accData[1]},${accData[2]},${gyroData[0]},${gyroData[1]},${gyroData[2]}\n")
            writer.close()

            Log.d("CSV", "Data written to CSV: $timestamp")
        } catch (e: Exception) {
            Log.e("CSV", "Error writing data to CSV: ${e.message}")
        }
    }

    private fun showMissingSensorDialog() {
        val builder = AlertDialog.Builder(this)
        builder.setTitle("Missing Sensors")
        builder.setMessage("This device does not have the required sensors (accelerometer or gyroscope). The app may not function correctly.")
        builder.setPositiveButton("OK") { dialog, _ ->
            dialog.dismiss()
            //finish() // Optionally close the app
        }
        builder.setCancelable(false)
        builder.show()
    }
    @RequiresApi(Build.VERSION_CODES.O)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        // Check if permission is granted
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            // Request permission
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.WRITE_EXTERNAL_STORAGE), 1)
        }

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
        val btnIncCor = findViewById<Button>(R.id.plusCor)
        val btnDecCor = findViewById<Button>(R.id.minusCor)

        if (accelStdInput.text.isEmpty()) accelStdInput.setText(ACCEL_STD.toString())
        if (gyroStdInput.text.isEmpty()) gyroStdInput.setText(GYRO_STD.toString())
        if (accelBiasInput.text.isEmpty()) accelBiasInput.setText(ACCEL_BIAS.toString())
        if (gyroBiasInput.text.isEmpty()) gyroBiasInput.setText(GYRO_BIAS.toString())
        if (initVelInput.text.isEmpty()) initVelInput.setText(INIT_VEL_STD.toString())
        if (cAInput.text.isEmpty()) cAInput.setText(C_A.toString())
        if (numR2Input.text.isEmpty()) numR2Input.setText(NUM_R2.toString())
        if (corInput.text.isEmpty()) corInput.setText(COR.toString())

        btnReset.setOnClickListener {
            val accelStd = accelStdInput.text.toString().toFloatOrNull() ?: 0.01f
            val gyroStd = gyroStdInput.text.toString().toFloatOrNull() ?: 0.01f
            val accelBias = accelBiasInput.text.toString().toFloatOrNull() ?: 0.0f
            val gyroBias = gyroBiasInput.text.toString().toFloatOrNull() ?: 0.0f
            val initVel = initVelInput.text.toString().toFloatOrNull() ?: 0.0f
            val cA = cAInput.text.toString().toFloatOrNull() ?: 0.0f
            val numR2 = numR2Input.text.toString().toFloatOrNull() ?: 0.0f
            val cor = corInput.text.toString().toFloatOrNull() ?: 0.0f
            val paramsArray = floatArrayOf(accelStd, gyroStd, initVel, cA, numR2, accelBias, gyroBias, cor)

            setKalmanParameters(paramsArray)
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
        setupIncrementDecrement(btnIncCor, btnDecCor, corInput, 0.01f)

        // Register sensors
        if (accelerometer == null || gyroscope == null) {
            // Show a pop-up message
            showMissingSensorDialog()
        } else {
            // Register sensors
            sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME)
            sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_GAME)
        }
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

    @SuppressLint("ClickableViewAccessibility")
    override fun onSensorChanged(event: SensorEvent) {

        val currentTimestamp = System.currentTimeMillis()
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                accData[0] = event.values[0]
                accData[1] = event.values[1]
                accData[2] = event.values[2]
                Log.d(
                    "SensorChange",
                    "Timestamp: $currentTimestamp, Accelerometer: x=${accData[0]}, y=${accData[1]}, z=${accData[2]}"
                )
            }
            Sensor.TYPE_GYROSCOPE -> {
                gyroData[0] = event.values[0]
                gyroData[1] = event.values[1]
                gyroData[2] = event.values[2]
                Log.d(
                    "SensorChange",
                    "Timestamp: $currentTimestamp, Gyroscope: x=${gyroData[0]}, y=${gyroData[1]}, z=${gyroData[2]}"
                )
            }
        }

        // Get the current system time
//        writeSensorDataToCSV(currentTimestamp, accData, gyroData)
        // Send sensor data to Kalman Filter if we have a valid timestamp
        if (lastTimestamp != 0L) {
            val result = updateKalmanState(accData, gyroData, currentTimestamp)
            displayKalmanResults(result)
        }
//        val rootLayout: LinearLayout = findViewById(R.id.rootLayout) // Root layout id
//        rootLayout.setOnTouchListener { v, event ->
//            val currentFocus = currentFocus
//            if (currentFocus is EditText) {
//                val imm = getSystemService(INPUT_METHOD_SERVICE) as InputMethodManager
//                imm.hideSoftInputFromWindow(currentFocus.windowToken, 0)
//                currentFocus.clearFocus()
//            }
//            false
//        }
        lastTimestamp = currentTimestamp
    }

    private fun displayKalmanResults(result: PitchSlopeResult?) {
        val pitchTextView = findViewById<TextView>(R.id.pitchValue)
        val slopeTextView = findViewById<TextView>(R.id.roadSlopeValue)
        val pitchMinusSlopeTextView = findViewById<TextView>(R.id.pitchMinusSlopeValue)
        if (result == null) {
            Log.e("Error", "Cannot display results: PitchSlopeResult is null.")
            pitchTextView.text = "Pitch: N/A"
            slopeTextView.text = "Road Slope: N/A"
            pitchMinusSlopeTextView.text = "Pitch - Slope: N/A"
            return
        }
        pitchTextView.text = String.format("Pitch: %.5f", result.pitch)
        slopeTextView.text = String.format("Road Slope: %.5f", result.slope)
        pitchMinusSlopeTextView.text = String.format("Pitch - Slope: %.5f", result.pitchMinusSlope)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Not needed for this implementation
    }

    override fun onDestroy() {
        super.onDestroy()
        sensorManager.unregisterListener(this)
    }

    companion object {
        const val ACCEL_STD = 1.0
        const val GYRO_STD = 0.01
        const val INIT_VEL_STD = 10.0
        const val C_A = 0.1
        const val NUM_R2 = 0.009
        const val ACCEL_BIAS = 0.001
        const val GYRO_BIAS = 0.001
        const val ODO_STD = 0.001
        const val ODO_BIAS = 0.001
        const val COR = 0.01
    }
}


