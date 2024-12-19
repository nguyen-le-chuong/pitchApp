#include <iostream>
#include "car.h"
#include "sensors.h"


class RoadSlope 
{
    public:
        PositionState getPosition()
        RoadSlope() : p_x(0), p_y(0), p_z(0), v_x(0), v_y(0), v_z(0) {}
        void estimatePosition(AccelMeasurement accel, double dt);
        double estimateSlopeAngle(std::vector<VectorXd> m_position_history);

    private:
        double p_x, p_y, p_z;
        double v_x, v_y, v_z;
}