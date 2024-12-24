#ifndef INCLUDE_AKFSFSIM_KALMANFILTER_H
#define INCLUDE_AKFSFSIM_KALMANFILTER_H

#include <vector>

#include "car.h"
#include "sensors.h"


class KalmanFilterBase
{
    public:

        KalmanFilterBase():m_initialised(false){}
        virtual ~KalmanFilterBase(){}
        void reset(VectorXd RotationState, MatrixXd cov){
            // VectorXd initialState(3);
            // initialState << a, b, c;
            setState(RotationState);
            m_initialised = true;
            // MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
            setCovariance(cov);}
        bool isInitialised() const {return m_initialised;}

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}
        double getAccel_std() const {return ACCEL_STD;}
        double getGyro_std() const {return GYRO_STD;}
        double getInit_vel_std() const {return INIT_VEL_STD;}
        double getc_a() const {return c_a;}
        Eigen::Matrix2d getR1() const {return R1;}
        Eigen::MatrixXd getR2() const {return R2;}
        Eigen::MatrixXd getnG() const {return nG;}
        void setAccelNoiseStd(double std) {ACCEL_STD = std;}
        void setGyroNoiseStd(double std) {GYRO_STD = std;}
        void setInitVelNoiseStd(double std) {INIT_VEL_STD = std;}
        void setc_a(double std) {c_a = std;}
        void setR1(Eigen::Matrix2d std) {R1 = std;}
        void setR2(Eigen::MatrixXd std) {R2 = std;}
        void setnG(Eigen::MatrixXd std) {nG = std;}


    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
            double ACCEL_STD;
    double GYRO_STD;
    double INIT_VEL_STD;
    double c_a;
    Eigen::Matrix2d R1;
    Eigen::MatrixXd R2;
    Eigen::MatrixXd nG;
};

class KalmanFilterRoadSlopeBase
{
    public:

        KalmanFilterRoadSlopeBase():m_initialised(false){}
        virtual ~KalmanFilterRoadSlopeBase(){}
        void reset(VectorXd SlopeState){
            // VectorXd initialState(3);
            // initialState << 0, 0, 9.8;
            setState(SlopeState);
            m_initialised = true;
            MatrixXd cov = MatrixXd::Identity(3, 3) * 0.01;
            setCovariance(cov);}
        bool isInitialised() const {return m_initialised;}

    protected:
    
        VectorXd getState() const {return m_state;}
        MatrixXd getCovariance()const {return m_covariance;}
        void setState(const VectorXd& state ) {m_state = state; m_initialised = true;}
        void setCovariance(const MatrixXd& cov ){m_covariance = cov;}
        double getAccel_std() const {return ACCEL_STD;}
        double getGyro_std() const {return GYRO_STD;}
        double getInit_vel_std() const {return INIT_VEL_STD;}
        void setAccelNoiseStd(double std) {ACCEL_STD = std;}
        void setGyroNoiseStd(double std) {GYRO_STD = std;}
        void setInitVelNoiseStd(double std) {INIT_VEL_STD = std;}

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
                    double ACCEL_STD;
    double GYRO_STD;
    double INIT_VEL_STD;
};

class KalmanFilter : public KalmanFilterBase
{
    public:

        VehicleState getVehicleState();
        MatrixXd getVehicleCovariance();
        Matrix2d getVehicleStatePositionCovariance();
        Vector2d calculateExAccel(AccelMeasurement accel, GyroMeasurement gyro, double v_t);

        void predictionStep(double dt);
        void predictionStep(GyroMeasurement gyro, double dt);
        void measurementStep1(AccelMeasurement accel, GyroMeasurement gyro, double v_t, Vector2d alpha);
        void measurementStep2();
        void setParameters(double accel_std, double gyro_std, double init_vel_std, double c_a, double num_R2, double num_nG, double accel_bias, double gyro_bias);

};


class KalmanFilterRoadSlope : public KalmanFilterRoadSlopeBase
{
    public:
        SlopeState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void measurementStep(AccelMeasurement accel, GyroMeasurement gyro, double v_t, double dt);
        void setParameters(double accel_std, double gyro_std, double init_vel_std);

};



#endif  // INCLUDE_AKFSFSIM_KALMANFILTER_H