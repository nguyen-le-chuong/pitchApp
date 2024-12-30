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
// double getAccel_std() = 0.05;
// double getGyro_std() = 0.001;
// double getInit_vel_std() = 1;
// -------------------------------------------------- //
// MatrixXd R = Matrix2d::Identity() * getGyro_std() * getInit_vel_std() + Matrix2d::Identity() * getAccel_std();
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

        Q << getAccel_std(), 0, 0,
            0, getInit_vel_std(), 0,
            0, 0, getGyro_std();
        
        F << 1, 0, 0,
            dt, 1, 0,
            0, 0, 1;

        F_noise << dt * dt / 2, 0,
                    dt, 0,
                    0, dt;

        W << getAccel_std(), getGyro_std();

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
        R << getAccel_std(), 0,
            0, getInit_vel_std();
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
        double slope_degree = -slope*180.0 / M_PI;
        return SlopeState(state[0], state[1], state[2], slope_degree);
    }
    return SlopeState();
}

void KalmanFilterRoadSlope::setParameters(double accel_std, double gyro_std, double init_vel_std)
{
    setAccelNoiseStd(accel_std);   
    setGyroNoiseStd(gyro_std);
    setInitVelNoiseStd(init_vel_std);
}

