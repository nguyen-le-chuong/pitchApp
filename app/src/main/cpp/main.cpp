// System Includes
#include <string>
#include <iostream>


#include "simulation.h"
#include "car.h"
#include "readData.h"
// #include "matplotlibcpp.h"

// Screen dimension constants
const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;
const double GRID_SIZE = 500;
const double GRID_SPACEING = 25;

// Function Prototypes
SimulationParams loadSimulation4Parameters();

// Main Loop
int main( int argc, char* args[] )
{
    // Display mDisplay;
    Simulation mSimulation;
    time_t ts;
    Eigen::VectorXd acc, gyro;
    double odo;
    double h_rear, h_front;
    // Read Data
    std::string accelerometer_file = "/home/chuongnl1/project/pitchekf/build/accelerometer.csv";
    std::string gyroscope_file = "/home/chuongnl1/project/pitchekf/data/gyroscope.csv";

    std::map<time_t, std::vector<double>> dataA = read_dataTimeStamp(accelerometer_file);
    std::map<time_t, std::vector<double>> dataG = read_dataTimeStamp(gyroscope_file);
    
    // Merge the data based on common timestamps
    std::vector<std::tuple<time_t, Eigen::VectorXd, Eigen::VectorXd>> merged_data = merge_dataTimeStamp(dataA, dataG);

    VectorXd RotationState(3);
    VectorXd SlopeState(3);

    // RotationState << 0.1908, 0, 0.9816;
    RotationState << 0, 0, 1;
    SlopeState << 0, 0, 0;
    MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
    

    mSimulation.reset(loadSimulation4Parameters(), RotationState, SlopeState, cov);
    time_t prev_ts = 0;
    double dt = 0.0;
    Eigen::Vector2d alpha;
    alpha << 0.001, 0.001;
    bool reset = false;
    double flag_ts = 0.0;
    int steps = 0;
    for (const auto &entry : merged_data) {
        time_t ts;

        std::tie(ts, acc, gyro) = entry;
        // if (reset) {
        //     RotationState = mSimulation.returnVehicleState();
        //     SlopeState = mSimulation.returnSlopeState();
        //     // cov = mSimulation.returnVehicleCovarience();
        //     mSimulation.reset(loadSimulation4Parameters(), RotationState, SlopeState, cov);
        //     reset = false;
        // }
        if (prev_ts != 0) {
            dt = static_cast<double>(ts - prev_ts)/1000;
        } else {
            prev_ts = ts;
            continue;
        }
        mSimulation.update(acc, gyro, 5, h_rear, h_front, ts, dt, alpha);
        mSimulation.updateRoadSlope(acc, gyro, 5, dt);

        prev_ts = ts;
    }
    pitch = mSimulation.returnPitch();
    slope = mSimulation.returnSlope();
    mSimulation.writeVectorsToCSV("output.csv");

    return 0;
}

SimulationParams loadSimulation4Parameters()
{    
    SimulationParams sim_params;
    sim_params.profile_name = "4 - Variable Speed Profile + GPS + GYRO";
    sim_params.end_time = 200;
    sim_params.car_initial_velocity = 0;
    sim_params.odo_enabled = true;
    sim_params.gyro_enabled = true;
    sim_params.accel_enabled = true;
    // sim_params.car_initial_psi = M_PI/180.0 * 45.0;

    return sim_params;
}
