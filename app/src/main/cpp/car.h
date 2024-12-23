#ifndef INCLUDE_AKFSFSIM_CAR_H
#define INCLUDE_AKFSFSIM_CAR_H

#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "utils.h"
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector4d;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Matrix3d;
struct VehicleState
{
    double R31, R32, R33;
    double pitch, roll;
    VehicleState():R31(0.0), R32(0.0), R33(1.0), pitch(0.0), roll(0.0) {}
    VehicleState(double setR31, double setR32, double setR33, double setPitch, double setRoll):R31(setR31), R32(setR32), R33(setR33), pitch(setPitch), roll(setRoll) {}
    // VehicleState(double setX, double setY, double setPsi, double setV, double setPsiDot, double setSteering):x(setX), y(setY), psi(setPsi), V(setV), yaw_rate(setPsiDot),steering(setSteering) {}
};

struct WheelState {
    double h_rear, h_front;
    double pitch;
    WheelState(): h_rear(0.0), h_front(0.0), pitch(0.0) {}
    WheelState(double setRear, double setFront, double setPitch): h_rear(setRear), h_front(setFront), pitch(setPitch) {}
};

struct SlopeState {
    double deri_v, v, g_x;
    double slope;
    SlopeState(): deri_v(0.0), v(0.0), g_x(9.8), slope(0.0) {}
    SlopeState(double setDeri_v, double setV, double setGx, double setSlope): deri_v(setDeri_v), v(setV), g_x(setGx), slope(setSlope) {}
};

class BicycleMotion
{
    public:

        BicycleMotion():m_initial_state(WheelState(0,0,0)),m_wheel_base(4.0),m_max_velocity(28.0),m_max_acceleration(2.0),m_max_steering(0.8) {reset();}
        BicycleMotion(double h_rear, double h_front, double pitch):m_initial_state(WheelState(h_rear, h_front, pitch)),m_wheel_base(4.0),m_max_velocity(28.0),m_max_acceleration(2.0),m_max_steering(0.8){reset();}

        void reset()
        {
            m_current_state = m_initial_state;
            // m_steering_command = m_initial_state.steering;
            // m_velocity_command = m_initial_state.V;
        }
        void reset(WheelState state)
        {
            m_initial_state = state;
            reset();
        }

        void update(double h_rear, double h_front)
        {

            double delta_h = abs(h_front - h_rear);
            double pitch = atan2(delta_h, m_wheel_base);
            m_current_state = WheelState(h_rear, h_front, pitch);
        }


        WheelState getWheelState() const {return m_current_state;}

    private:

        WheelState m_current_state;
        WheelState m_initial_state;


        double m_steering_command;
        double m_velocity_command;

        double m_wheel_base;

        double m_max_velocity;
        double m_max_acceleration;
        double m_max_steering;

};

class Car
{
    public:

        Car():m_vehicle_model() //,m_current_command(nullptr)
        {
            // Create Display Geometry
            m_car_lines_body = {{2,-1},{2,1},{-2,1},{-2,-1},{2,-1}};
            m_marker_lines = {{{0.5,0.5},{-0.5,-0.5}}, {{0.5,-0.5},{-0.5,0.5}}, {{0,0},{3.5,0}}};
            m_wheel_lines = {{-0.6,0.3},{0.6, 0.3},{0.6, -0.3},{-0.6, -0.3},{-0.6, 0.3}};
            m_wheel_fl_offset = Eigen::Vector2d(2, -1.6);
            m_wheel_fr_offset = Eigen::Vector2d(2, 1.6);
            m_wheel_rl_offset = Eigen::Vector2d(-2, -1.6);
            m_wheel_rr_offset = Eigen::Vector2d(-2, 1.6);
        }

        void reset(double h_front, double h_rear, double pitch)
        {
            m_vehicle_model.reset(WheelState(h_front, h_rear, pitch));
            // while (!m_vehicle_commands.empty()){m_vehicle_commands.pop();}
            // m_current_command = nullptr;
        }



        WheelState getWheelState() const {return m_vehicle_model.getWheelState();}

        bool update(double h_front, double h_rear)
        {

            // Update Vehicle
            m_vehicle_model.update(h_rear, h_front);

            return true;
        }

    private:

        BicycleMotion m_vehicle_model;

        std::vector<Eigen::Vector2d> m_car_lines_body;
        std::vector<Eigen::Vector2d> m_wheel_lines;
        std::vector<std::vector<Eigen::Vector2d>> m_marker_lines;
        Eigen::Vector2d m_wheel_fl_offset, m_wheel_fr_offset, m_wheel_rl_offset, m_wheel_rr_offset;
};


#endif  // INCLUDE_AKFSFSIM_CAR_H