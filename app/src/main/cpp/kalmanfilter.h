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

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
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

    private:
        bool m_initialised;
        VectorXd m_state;
        MatrixXd m_covariance;
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
        void setKalmanParameters(double c_a, double num_R2, double num_nG, double ACCEL_BIAS, double GYRO_BIAS, double ODO_STD, double ODO_BIAS, double COR);

};


class KalmanFilterRoadSlope : public KalmanFilterRoadSlopeBase
{
    public:
        SlopeState getVehicleState();
        Matrix2d getVehicleStatePositionCovariance();

        void predictionStep(double dt);
        void measurementStep(AccelMeasurement accel, GyroMeasurement gyro, double v_t, double dt);
};



#endif  // INCLUDE_AKFSFSIM_KALMANFILTER_H