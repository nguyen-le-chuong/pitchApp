#include <iostream>
#include <roadslope.h>

void RoadSlope::estimatePosition(AccelMeasurement accel, double dt) 
{
    if(isInitialised()){
        double ax = accel.ax;
        double ay = accel.ay;
        double az = accel.az;
        
        VectorXd position = getPosition();

        double p_x = position(0); 
        double p_y = position(1);
        double p_z = position(2);
        
        
        v_x += ax * dt;
        v_y += ay * dt;
        v_z += az * dt;

        p_x += v_x * dt;
        p_y += v_y * dt;
        p_z += v_z * dt;

        VectorXd new_pos;
        new_pos >> p_x, p_y, p_z;

        setPosition(new_pos);
    }
}

void RoadSlope::estimateSlopeAngle(){
    if (isInitialised()){
        VectorXd position = getPosition();
                
        double p_x = position(0); 
        double p_y = position(1);
        double p_z = position(2);

        double initial_p_z = m_position_history[0](2)

        double delta_p_vertical = std::abs(p_z - initial_p_z);
        double delta_p_horizontal = 0.0;

        double prev_px = 0.0;
        double prev_py = 0.0;
        double curr_px = 0.0;
        double curr_py = 0.0;

        for (const auto &temp_pos : m_position_history){
            curr_px = temp_pos(0);
            curr_py = temp_pos(1);

            delta_p_horizontal += std::sprt(std::pow(curr_px - prev_px, 2) + std::pow(curr_px - prev_px, 2));
        }

        double road_slope = std::atan2(delta_p_vertical, delta_p_horizontal);
        return road_slope* (180.0 / M_PI);
    }
}