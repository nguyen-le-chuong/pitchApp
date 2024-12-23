// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### ANSWER FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
double ACCEL_STD = 0.05;
double GYRO_STD = 0.001;
double INIT_VEL_STD = 1;
double INIT_PSI_STD = 45.0/180.0 * M_PI;
double GPS_POS_STD = 3.0;
double LIDAR_RANGE_STD = 3.0;
double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //
// MatrixXd R = Matrix2d::Identity() * GYRO_STD * INIT_VEL_STD + Matrix2d::Identity() * ACCEL_STD;
// MatrixXd R2 = MatrixXd::Constant(1, 1, 0.01);


void KalmanFilterRoadSlope::predictionStep(double dt)
{
    if (isInitialised()){
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        Matrix3d Q;
        Matrix3d F;
        MatrixXd F_noise(3, 2);
        MatrixXd W(2, 1);

        Q << ACCEL_STD, 0, 0,
            0, INIT_VEL_STD, 0,
            0, 0, GYRO_STD;
        
        F << 1, 0, 0,
            dt, 1, 0,
            0, 0, 1;

        F_noise << dt * dt / 2, 0,
                    dt, 0,
                    0, dt;

        W << ACCEL_STD, GYRO_STD;

        state = F * state + F_noise * W;

        cov = F * cov * F.transpose() + Q;

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilterRoadSlope::measurementStep(AccelMeasurement accel, GyroMeasurement gyro, double vt, double dt)
{
    if (isInitialised()){
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        double w_x = gyro.wx;
        double a_long = accel.ax ;
        MatrixXd z(2, 1);
        MatrixXd H(2, 3);

        z << vt, a_long;

        H << dt, 1, 0,
            1, 0, 1;

        VectorXd y = z - H * state;
        Matrix2d R;
        R << ACCEL_STD, 0,
            0, INIT_VEL_STD;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov * H.transpose() * S.inverse();

        state = state + K * y;

        MatrixXd I = MatrixXd::Identity(cov.rows(), cov.cols());
        cov = (I - K * H) * cov;

        setState(state);
        setCovariance(cov);
    }
}

Matrix2d KalmanFilterRoadSlope::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

SlopeState KalmanFilterRoadSlope::getVehicleState()
{
    if (isInitialised())
    {   
        VectorXd state = getState(); //
        // std::cout << state[0] << " " <<  state[1] << " " << state[2] << std::endl; 
        double slope = asin(state[2]/9.81);
        double slope_degree = -slope*180.0 / 3.14;
        return SlopeState(state[0], state[1], state[2], slope_degree);
    }
    return SlopeState();
}

