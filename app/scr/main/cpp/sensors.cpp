#include "sensors.h"
#include "utils.h"



// Gyro Sensor
GyroSensor::GyroSensor():m_rand_gen(std::mt19937()),m_noise_std(0.0),m_bias(0.0){}
void GyroSensor::reset(){m_rand_gen = std::mt19937();}
void GyroSensor::setGyroNoiseStd(double std){m_noise_std = std;}
void GyroSensor::setGyroBias(double bias){m_bias = bias;}
GyroMeasurement GyroSensor::generateGyroMeasurement(Eigen::VectorXd sensor_yaw_rate)
{
    GyroMeasurement meas;
    std::normal_distribution<double> gyro_dis(0.0,m_noise_std);
    meas.wx = sensor_yaw_rate(0) + m_bias + gyro_dis(m_rand_gen);
    meas.wy = sensor_yaw_rate(1) + m_bias + gyro_dis(m_rand_gen);
    meas.wz = sensor_yaw_rate(2) + m_bias + gyro_dis(m_rand_gen);
    return meas;
}

AccelSensor::AccelSensor():m_rand_gen(std::mt19937()), m_noise_std(0.0), m_bias(0.0){}
void AccelSensor::reset(){m_rand_gen = std::mt19937();}
void AccelSensor::setAccelNoiseStd(double std){m_noise_std = std;}
void AccelSensor::setAccelBias(double bias){m_bias = bias;}
AccelMeasurement AccelSensor::generateAccelMeasurement(Eigen::VectorXd sensor_rate){
    AccelMeasurement meas;
    std::normal_distribution<double> accel_dis(0.0, m_noise_std);
    meas.ax = sensor_rate(0) + m_bias + accel_dis(m_rand_gen);
    meas.ay = sensor_rate(1) + m_bias + accel_dis(m_rand_gen);
    meas.az = sensor_rate(2) + m_bias + accel_dis(m_rand_gen);
    return meas;
}

OdoSensor::OdoSensor():m_rand_gen(std::mt19937()), m_noise_std(0.0), m_bias(0.0){}
void OdoSensor::reset(){m_rand_gen = std::mt19937();}
void OdoSensor::setOdoNoiseStd(double std){m_noise_std = std;}
void OdoSensor::setOdoBias(double bias){m_bias = bias;}
OdoMeasurement OdoSensor::generateOdoMeasurement(double sensor){
    OdoMeasurement meas;
    std::normal_distribution<double> odo_dis(0.0, m_noise_std);
    meas.v = sensor + m_bias + odo_dis(m_rand_gen);
    return meas;
}


FrontHeightSensor::FrontHeightSensor(){}
void FrontHeightSensor::reset(){}
FrontHeightMeasurement getFrontHeightMeasurement(double a) {
    FrontHeightMeasurement meas;
    meas.a = a;
    return meas;
}

RearHeightSensor::RearHeightSensor(){}
void RearHeightSensor::reset(){}
RearHeightMeasurement getRearHeightMeasurement(double a) {
    RearHeightMeasurement meas;
    meas.a = a;
    return meas;
}


