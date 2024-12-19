package com.example.myapp

import android.os.Bundle
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity() {

    // Native methods to call C++ code
    external fun initKalman()
    external fun updateKalmanState(acc: FloatArray, gyro: FloatArray, timestamp: Long): PitchSlopeResult
    external fun resetSimulation(
        accelStd: Double,
        gyroStd: Double,
        initVelStd: Double,
        c_a: Double,
        numR2: Double,
        numNg: Double,
        accelBias: Double,
        gyroBias: Double,
        odoStd: Double,
        odoBias: Double,
        cor: Double
    )

    init {
        System.loadLibrary("yourlibraryname") // Replace with your actual .so library name
    }

    private lateinit var pitchTextView: TextView
    private lateinit var slopeTextView: TextView

    // Parameter EditTexts
    private lateinit var accelStdEditText: EditText
    private lateinit var gyroStdEditText: EditText
    private lateinit var initVelStdEditText: EditText
    private lateinit var cAEditText: EditText
    private lateinit var numR2EditText: EditText
    private lateinit var numNgEditText: EditText
    private lateinit var accelBiasEditText: EditText
    private lateinit var gyroBiasEditText: EditText
    private lateinit var odoStdEditText: EditText
    private lateinit var odoBiasEditText: EditText
    private lateinit var corEditText: EditText

    // Default parameter values
    private var accelStd = 1.0
    private var gyroStd = 0.02 / 180.0 * Math.PI
    private var initVelStd = 10.0
    private var cA = 0.1
    private var numR2 = 0.009
    private var numNg = 0.5 / 180 * Math.PI
    private var accelBias = 0.001
    private var gyroBias = 0.001 / 180.0 * Math.PI
    private var odoStd = 0.001 / 180.0 * Math.PI
    private var odoBias = 0.001 / 180.0 * Math.PI
    private var cor = 0.01

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // Initialize Kalman Filter
        initKalman()

        // Find Views
        pitchTextView = findViewById(R.id.pitchTextView)
        slopeTextView = findViewById(R.id.slopeTextView)

        // Parameter EditTexts
        accelStdEditText = findViewById(R.id.accelStdEditText)
        gyroStdEditText = findViewById(R.id.gyroStdEditText)
        initVelStdEditText = findViewById(R.id.initVelStdEditText)
        cAEditText = findViewById(R.id.cAEditText)
        numR2EditText = findViewById(R.id.numR2EditText)
        numNgEditText = findViewById(R.id.numNgEditText)
        accelBiasEditText = findViewById(R.id.accelBiasEditText)
        gyroBiasEditText = findViewById(R.id.gyroBiasEditText)
        odoStdEditText = findViewById(R.id.odoStdEditText)
        odoBiasEditText = findViewById(R.id.odoBiasEditText)
        corEditText = findViewById(R.id.corEditText)

        // Reset Button
        val resetButton: Button = findViewById(R.id.resetButton)
        resetButton.setOnClickListener {
            // Parse values from EditTexts
            accelStd = accelStdEditText.text.toString().toDoubleOrNull() ?: accelStd
            gyroStd = gyroStdEditText.text.toString().toDoubleOrNull() ?: gyroStd
            initVelStd = initVelStdEditText.text.toString().toDoubleOrNull() ?: initVelStd
            cA = cAEditText.text.toString().toDoubleOrNull() ?: cA
            numR2 = numR2EditText.text.toString().toDoubleOrNull() ?: numR2
            numNg = numNgEditText.text.toString().toDoubleOrNull() ?: numNg
            accelBias = accelBiasEditText.text.toString().toDoubleOrNull() ?: accelBias
            gyroBias = gyroBiasEditText.text.toString().toDoubleOrNull() ?: gyroBias
            odoStd = odoStdEditText.text.toString().toDoubleOrNull() ?: odoStd
            odoBias = odoBiasEditText.text.toString().toDoubleOrNull() ?: odoBias
            cor = corEditText.text.toString().toDoubleOrNull() ?: cor

            // Call resetSimulation to apply changes
            resetSimulation(
                accelStd, gyroStd, initVelStd, cA, numR2, numNg, accelBias, gyroBias, odoStd, odoBias, cor
            )
        }

        // Parameter Adjustment Buttons (Increase/Decrease)
        findViewById<Button>(R.id.increaseAccelStdButton).setOnClickListener {
            accelStd += 0.1
            accelStdEditText.setText(accelStd.toString())
        }
        findViewById<Button>(R.id.decreaseAccelStdButton).setOnClickListener {
            accelStd -= 0.1
            accelStdEditText.setText(accelStd.toString())
        }

        // Repeat for other parameters...
    }

    // Method to update Kalman state and display result
    fun updateKalman(acc: FloatArray, gyro: FloatArray, timestamp: Long) {
        val result = updateKalmanState(acc, gyro, timestamp)

        // Update the UI with the result
        pitchTextView.text = "Pitch: ${result.pitch}"
        slopeTextView.text = "Slope: ${result.slope}"
    }
}
