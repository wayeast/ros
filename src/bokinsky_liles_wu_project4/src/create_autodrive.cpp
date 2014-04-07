#include "create_autodrive.h"


/*
 * Linear velocity is a function of robot's distance from nearest obstacle.
 * If nearest obstacle is > create_autodrive::SAFE_DIST, robot moves forward
 * at full speed .  Otherwise it moves forward at speed = distance to nearest
 * obstacle.
 */
float
create_autodrive::select_linear_vel(struct create_autodrive::create_autodrive_space s)
{
    return fmin(create_autodrive::FULL_SPEED,
                      fmax((float) 0.0,
                                 (s.nearest_obstacle_distance - 
                                  create_autodrive::SAFE_DIST)
                                 )
                      );
}

/*
 * Angular velocity is a weighted average of thetas bordering selected space.
 * If the nearest obstacle on right side of space is closer than nearest obstacle
 * on left side, then robot should tend toward left side of center.
 */
float
create_autodrive::select_angular_vel(struct create_autodrive::create_autodrive_space s)
{
    return ((s.left_border_theta * s.left_border_dist) +
            (s.right_border_theta * s.right_border_dist)) /
           (s.left_border_dist + s.right_border_dist);
}


/*
 * Do sweep of laserscan ranges and return space with 1? largest top range,
 * and 2) widest swath.
 */
create_autodrive::create_autodrive_space
create_autodrive::find_drive_space(const sensor_msgs::LaserScan& msg)
{
    std::vector<create_autodrive::create_autodrive_space> candidates;
    create_autodrive::create_autodrive_space curr_space;
    float curr_range = fill_nan(msg.ranges[0], msg.ranges[1], msg.ranges[1], msg.range_max);
    float nearest_obstacle_distance = curr_range;
    float curr_theta = curr_space.right_border_theta = msg.angle_min;
    curr_space.right_border_dist = curr_range;
    curr_space.top_range = curr_range;
    curr_space.size = 1;
    float prev_range = curr_range;

    int back_ind = msg.ranges.size() - 1;
    for (int i=1; i<back_ind; i++)
    {
        prev_range = curr_range;
        curr_range = fill_nan(msg.ranges[i], msg.ranges[i+1], prev_range, msg.range_max);
        curr_theta += msg.angle_increment;

        // update nearest obstacle distance
        if (curr_range < nearest_obstacle_distance)
            nearest_obstacle_distance = curr_range;

        if (std::abs(curr_range - prev_range) < create_autodrive::FUDGE_FACTOR)
        {
            /* Update current space */
            curr_space.size++;
            if (curr_range > curr_space.top_range)
                curr_space.top_range = curr_range;
        }
        else
        {
            /* Add curr_space to candidates and refresh */
            curr_space.left_border_theta = curr_theta;
            curr_space.left_border_dist = curr_range;
            candidates.push_back(curr_space);

            curr_space = create_autodrive::create_autodrive_space();
            curr_space.right_border_theta = curr_theta - msg.angle_increment;
            curr_space.right_border_dist = prev_range;
            curr_space.size = 1;
            curr_space.top_range = curr_range;
        }
    }  // end scan of msg.ranges
    // get last space onto candidates
    prev_range = curr_range;
    curr_range = fill_nan(msg.ranges[back_ind], prev_range, prev_range, msg.range_max);
    curr_space.left_border_theta = msg.angle_max;
    curr_space.left_border_dist = curr_range;
    candidates.push_back(curr_space);

    // find space with largest max range -> biggest size
    std::vector<create_autodrive_space>::iterator It = candidates.begin();
    create_autodrive_space ret = *It;
    while (It != candidates.end())
    {
        if (It->top_range > ret.top_range)
            ret = *It;
        else if (It->top_range == ret.top_range && It->size > ret.size)
            ret = *It;
    }

    // fill in nearest_obstacle_distance
    ret.nearest_obstacle_distance = nearest_obstacle_distance;

    return ret;
}

float
create_autodrive::fill_nan(float middle, float post, float prev, float max)
{
    if (!std::isnan(middle))
        return middle;
    if (!std::isnan(post))
        return (post + prev) / 2.0;
    return max;
}
