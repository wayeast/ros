#ifndef CREATE_AUTODRIVE_H
#define CREATE_AUTODRIVE_H

#include <cmath>
#include <sensor_msgs/LaserScan.h>


namespace create_autodrive
{
    // Constant value definitions
    const float SAFE_DIST = 0.75;  // min distance to nearest obstacle to allow forward motion
    const float FULL_SPEED = 2.0;  // robot full speed forward
    const float FUDGE_FACTOR = 0.01;  // accounting for imprecise laserscan readings

    // space representation structure
    struct create_autodrive_space
    {
        float left_border_theta;
        float right_border_theta;
        float left_border_dist;
        float right_border_dist;
        float nearest_obstacle_distance;
        float top_range;
        int size;
    };

    // related functions
    float select_linear_vel(struct create_autodrive_space);
    float select_angular_vel(struct create_autodrive_space);
    struct create_autodrive_space find_drive_space(const sensor_msgs::LaserScan &);

    float fill_nan(float, float, float, float);

}  // end namespace definition

#endif

