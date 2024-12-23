
#include "simulation.h"
#include "utils.h"
#include "math.h"

Simulation::Simulation()
: m_sim_parameters(SimulationParams()),
    m_is_paused(false),
    m_is_running(false),
    m_time_multiplier(1),
    m_view_size(100),
    m_time(0.0),
    m_time_till_gyro_measurement(0.0),
    m_time_till_accel_measurement(0.0),
    m_time_till_odo_measurement(0.0)
{}

void Simulation::reset(VectorXd RotationState, VectorXd SlopeState, MatrixXd cov)
{
    // Reset Simulation
    m_time = 0.0;
    m_time_till_gyro_measurement = 0.0;
    m_time_till_accel_measurement = 0.0;
    m_time_till_odo_measurement = 0.0;
  

    m_is_running = true;
    m_is_paused = false;
    
    m_kalman_filter.reset(RotationState, cov);
    m_road_slope.reset(SlopeState);


    m_gyro_sensor.reset();
    m_gyro_sensor.setGyroNoiseStd(m_sim_parameters.gyro_noise_std);
    m_gyro_sensor.setGyroBias(m_sim_parameters.gyro_bias);

    m_accel_sensor.reset();
    m_accel_sensor.setAccelNoiseStd(m_sim_parameters.accel_noise_std);
    m_accel_sensor.setAccelBias(m_sim_parameters.accel_bias);

    m_odo_sensor.reset();
    m_odo_sensor.setOdoNoiseStd(m_sim_parameters.odo_noise_std);
    m_odo_sensor.setOdoBias(m_sim_parameters.odo_bias);

    m_front_sensor.reset();
    m_rear_sensor.reset();


    m_car.reset(0, 0, 0);
    


    // Stats Variables
    m_filter_error_x_position_history.clear();
    m_filter_error_y_position_history.clear();
    m_filter_error_heading_history.clear();
    m_filter_error_velocity_history.clear();

    std::cout << "Simulation: Reset" << std::endl;
}

void Simulation::setKalmanParameters(double gyro_noise_std, double gyro_bias, double accel_noise_std, double accel_bias, double odo_noise_std, double odo_bias, double c_a, double num_R2, double num_nG, double accel_bias_std, double gyro_bias_std)
{
    m_sim_parameters.gyro_noise_std = gyro_noise_std;
    m_sim_parameters.gyro_bias = gyro_bias;
    m_sim_parameters.accel_noise_std = accel_noise_std;
    m_sim_parameters.accel_bias = accel_bias;
    m_kalman_filter.setKalmanParameters(c_a, num_R2, num_nG, accel_bias_std, gyro_bias_std);
    // m_sim_parameters.odo_noise_std = odo_noise_std;
    // m_sim_parameters.odo_bias = odo_bias;
    // m_sim_parameters.c_a = c_a;
    // m_sim_parameters.num_R2 = num_R2;
    // m_sim_parameters.num_nG = num_nG;
    // m_sim_parameters.accel_bias_std = accel_bias_std;
    // m_sim_parameters.gyro_bias_std = gyro_bias_std;
}

VectorXd Simulation::returnVehicleState()
{
    VehicleState filter_state = m_kalman_filter.getVehicleState();
    double R31 = filter_state.R31;
    double R32 = filter_state.R32;
    double R33 = filter_state.R33;

    VectorXd state(3);
    state << R31, R32, R33;
    return state;
}

MatrixXd Simulation::returnVehicleCovarience()
{
    return m_kalman_filter.getVehicleCovariance();
}

VectorXd Simulation::returnSlopeState()
{
    SlopeState filter_state = m_road_slope.getVehicleState();
    double deri_v = filter_state.deri_v;
    double v = filter_state.v;
    double g_x = filter_state.g_x;

    VectorXd state(3);
    state << deri_v, v, g_x;
    return state;
}

void Simulation::writeVectorsToCSV(const std::string& filename) {
    
    std::vector<time_t> vec1 = m_time_history;
    std::vector<double> vec2 = m_filter_pitch_history;
    // std::vector<double> vec3 = m_wheel_pitch_history;
    std::vector<double> vec3 = m_road_slope_history;
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    size_t maxSize = std::max({vec1.size(), vec2.size(), vec3.size()});

    for (size_t i = 0; i < maxSize; ++i) {
        if (i < vec1.size()) file << vec1[i];
        file << ","; // Column separator

        if (i < vec2.size()) file << vec2[i];
        file << ",";

        if (i < vec3.size()) file << vec3[i];
        file << "\n"; // Newline at the end of each row
    }

    file.close();
    std::cout << "Data successfully written to " << filename << std::endl;
}



void Simulation::update(Eigen::VectorXd acc, Eigen::VectorXd gyro, double odo, double h_rear, double h_front, time_t m_time, double delta_t, Eigen::Vector2d& alpha)
{
    m_sim_parameters.time_step = delta_t;
    if (m_is_running && !m_is_paused)
    {
            // Update Motion (for Front and Rear ) 
            m_car.update(h_rear, h_front);
            m_vehicle_pitch_history.push_back(m_car.getWheelState().pitch);
            if (m_kalman_filter.isInitialised())
            {
                WheelState wheel_state = m_car.getWheelState();
                VehicleState filter_state = m_kalman_filter.getVehicleState();
                m_filter_error_pitch_history.push_back(filter_state.pitch - wheel_state.pitch);
                m_filter_pitch_history.push_back(filter_state.pitch);
                m_wheel_pitch_history.push_back(wheel_state.pitch);
                m_time_history.push_back(m_time);
            }

            // Gyro Measurement / Prediction Step
            GyroMeasurement gyro_meas;  // Khai báo biến gyro_meas
            AccelMeasurement accel_meas; // Khai báo biến accel_meas
            OdoMeasurement v_meas;  // Khai báo biến v_meas

            if (m_sim_parameters.gyro_enabled)
            {
                    gyro_meas = m_gyro_sensor.generateGyroMeasurement(gyro);
                    m_kalman_filter.predictionStep(gyro_meas, m_sim_parameters.time_step);
                    m_time_till_gyro_measurement += 1.0/m_sim_parameters.gyro_update_rate;
     
            }

            if (m_sim_parameters.accel_enabled)
            {
     
                    accel_meas = m_accel_sensor.generateAccelMeasurement(acc);
                    m_time_till_accel_measurement += 1.0/m_sim_parameters.accel_update_rate;
    
            }

            if (m_sim_parameters.odo_enabled)
            {
         
                    v_meas = m_odo_sensor.generateOdoMeasurement(odo);
                    m_time_till_odo_measurement += 1.0/m_sim_parameters.odo_update_rate;

            }
            m_kalman_filter.measurementStep1(accel_meas, gyro_meas, v_meas.v, alpha);
            m_kalman_filter.measurementStep2();
            alpha = m_kalman_filter.calculateExAccel(accel_meas, gyro_meas, v_meas.v);

            // Save Filter History and Calculate Stats

            // Update Time
            //m_time += m_sim_parameters.time_step;
    }
}

void Simulation::updateRoadSlope(Eigen::VectorXd acc, Eigen::VectorXd gyro, double odo, double delta_t)
{

    if (m_is_running && !m_is_paused){
        GyroMeasurement gyro_meas;  // Khai báo biến gyro_meas
        AccelMeasurement accel_meas; // Khai báo biến accel_meas
        OdoMeasurement v_meas;  // Khai báo biến v_meas

        if (m_sim_parameters.accel_enabled)
        {
            accel_meas = m_accel_sensor.generateAccelMeasurement(acc);
            // m_time_till_accel_measurement += 1.0/m_sim_parameters.accel_update_rate;
        }
        if (m_sim_parameters.gyro_enabled) 
        {
            gyro_meas = m_gyro_sensor.generateGyroMeasurement(gyro);
            // m_time_till_gyro_measurement 
        }
        if (m_sim_parameters.odo_enabled)
        {
            v_meas = m_odo_sensor.generateOdoMeasurement(odo);
            // m_time_till_odo_measurement += 1.0/m_sim_parameters.odo_update_rate;
        }
        m_road_slope.predictionStep(delta_t);
        m_road_slope.measurementStep(accel_meas, gyro_meas, v_meas.v, delta_t);
        if (m_road_slope.isInitialised()){
            SlopeState slope_state = m_road_slope.getVehicleState();
            m_road_slope_history.push_back(slope_state.slope);
        }
    }

}
        
double Simulation::returnPitch(){
    return m_kalman_filter.getVehicleState().pitch;
}
double Simulation::returnSlope(){
    return m_road_slope.getVehicleState().slope;
}
void Simulation::reset(SimulationParams sim_params, VectorXd RotationState, VectorXd SlopeState, MatrixXd cov){m_sim_parameters = sim_params; reset(RotationState, SlopeState, cov);}
void Simulation::increaseTimeMultiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier << ")" << std::endl;
}
void Simulation::decreaseTimeMultiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")" << std::endl;
    }
}
void Simulation::setTimeMultiplier(unsigned int multiplier){m_time_multiplier = static_cast<int>(multiplier);}
void Simulation::increaseZoom()
{
    if (m_view_size > 25){m_view_size -= 25;}
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}
void Simulation::decreaseZoom()
{
    if (m_view_size < 400){m_view_size += 25;}
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}
void Simulation::togglePauseSimulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused?"True":"False") << ")" << std::endl;
}
bool Simulation::isPaused(){return m_is_paused;}
bool Simulation::isRunning(){return m_is_running;}
