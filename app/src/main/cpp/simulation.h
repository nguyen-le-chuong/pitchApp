#ifndef INCLUDE_AKFSFSIM_SIMULATION_H
#define INCLUDE_AKFSFSIM_SIMULATION_H

#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include "kalmanfilter.h"
#include "car.h"
#include "sensors.h"

struct SimulationParams
{
    std::string profile_name;
    double time_step;
    double end_time;

    bool gyro_enabled;
    double gyro_update_rate;
    double gyro_noise_std;
    double gyro_bias;

    bool accel_enabled;
    double accel_update_rate;
    double accel_noise_std;
    double accel_bias;

    bool odo_enabled;
    double odo_update_rate;
    double odo_noise_std;
    double odo_bias;

    double car_initial_R31;
    double car_initial_R32;
    double car_initial_R33;

    double car_initial_x;
    double car_initial_y;
    double car_initial_psi;
    double car_initial_velocity;

    // std::vector<std::shared_ptr<MotionCommandBase>> car_commands;

    SimulationParams()
    :profile_name(""),
     time_step(0.1),end_time(120),
     gyro_enabled(true), gyro_update_rate(10.0),gyro_noise_std(0.01), gyro_bias(0.01),
     accel_enabled(true), accel_update_rate(10.0),accel_noise_std(0.01), accel_bias(0.01),
     odo_enabled(true), odo_update_rate(10.0),odo_noise_std(0.01), odo_bias(0.01),
     car_initial_R31(0.0), car_initial_R32(0.0), car_initial_R33(1.0), car_initial_velocity(0.0)
    {}
};


class Simulation
{
    public:

        Simulation();
        void reset(VectorXd RotationState, VectorXd SlopeState, MatrixXd cov);
        void update(Eigen::VectorXd acc, Eigen::VectorXd gyro, double odo, time_t m_time, double delta_t, Eigen::Vector2d& alpha);
        void updateRoadSlope(Eigen::VectorXd acc, Eigen::VectorXd gyro, double odo, double delta_t);
        void setKalmanParameters(SimulationParams sim_params, double accel_std, double gyro_std, double init_vel_std, double c_a, double num_R2, double num_nG, double accel_bias, double gyro_bias, double cor);
        Eigen::Vector3d calculateVelocity(Eigen::VectorXd acc, Eigen::VectorXd gyro, double delta_t, Eigen::Vector3d velocity);

        double returnPitch();
        double returnSlope();
        VectorXd returnVehicleState();
        MatrixXd returnVehicleCovarience();
        VectorXd returnSlopeState();
        MatrixXd returnSlopeCovarience();
        // void render(Display& disp);
        void increaseTimeMultiplier();
        void decreaseTimeMultiplier();
        void setTimeMultiplier(unsigned int multiplier);
        void increaseZoom();
        void writeVectorsToCSV(const std::string& filename);
        void decreaseZoom();
        void togglePauseSimulation();
        bool isPaused();
        bool isRunning();

    private:

        SimulationParams m_sim_parameters;
        KalmanFilter m_kalman_filter;
        KalmanFilterRoadSlope m_road_slope;
        Car m_car;
        GyroSensor m_gyro_sensor;
        AccelSensor m_accel_sensor;
        OdoSensor m_odo_sensor;
        FrontHeightSensor m_front_sensor;
        RearHeightSensor m_rear_sensor;

        bool m_is_paused;
        bool m_is_running;
        int  m_time_multiplier;
        double m_view_size;

        double m_time;
        double m_time_till_gyro_measurement;
        double m_time_till_accel_measurement;
        double m_time_till_odo_measurement;

        // double m_time_till_gps_measurement;
        // double m_time_till_lidar_measurement;
        double m_time_till_pitch_measurement;

        std::vector<GyroMeasurement> m_gyro_measurement_history;
        std::vector<AccelMeasurement> m_accel_measurement_history;
        std::vector<OdoMeasurement> m_odo_measurement_history;
        std::vector<double> m_pitch_measurement_history;
        std::vector<double> m_vehicle_pitch_history;
        std::vector<double> m_filter_pitch_history;
        std::vector<double> m_wheel_pitch_history;
        std::vector<double> m_road_slope_history;
        std::vector<time_t> m_time_history;
        // std::vector<Eigen::Vector2> m_filter_position_history;

        std::vector<double> m_filter_error_x_position_history;
        std::vector<double> m_filter_error_y_position_history;

        std::vector<double> m_filter_error_pitch_history;

        std::vector<double> m_filter_error_heading_history;
        std::vector<double> m_filter_error_velocity_history;

};
SimulationParams loadSimulation4Parameters();


#endif  // INCLUDE_AKFSFSIM_SIMULATION_H
