/**************************************************
 * Name:         Huston Bokinsky
 * File:         trackers.cpp
 * Assignment:   count_people_bokinsky
 * Date:         23 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Description:  
 * 
 **************************************************/

#include "trackers.h"

EntityTracker::EntityTracker(const sensor_msgs::LaserScan & scan)
{
    entity_present = false;
    pc.header = scan.header;
}

void
EntityTracker::update(float theta, float range)
{
    if (std::isfinite(range))
    {
        if (!entity_present)  // first hit from void
        {
            resetVars(theta, range);
        }
        else if (likelyDifferentEntity(range))  // first hit from diff entity
        {
            pushPoint();
            resetVars(theta, range);
        }
        else  // contiguous hit likely on same entity
        {
            incrementVars(theta, range);
        }
    }
    else if (entity_present && std::isnan(range))
        incrementVars(theta, last_range);
    else if (entity_present)
        pushPoint();

}  // end update function

bool
EntityTracker::likelyDifferentEntity(float this_range)
{
    return (std::abs(this_range - last_range) >= DIFF_ENT_LIMIT);
}

void
EntityTracker::resetVars(float theta, float range)
{
    entity_present = true;
    theta_accum = theta;
    range_accum = last_range = range;
    theta_count = 1;
    range_count = 1;
}

void
EntityTracker::incrementVars(float theta, float range)
{
    theta_accum += theta;
    range_accum += range;
    last_range = range;
    theta_count++;
    range_count++;
}

void
EntityTracker::pushPoint()
{
    entity_present = false;
    if (range_count >= REAL_ENTITY_CONTIGUOUS_HIT_MIN)
    {
        geometry_msgs::Point32 pt;
        pt.x = range_accum / range_count
               * std::cos(theta_accum / theta_count);
        pt.y = range_accum / range_count
               * std::sin(theta_accum / theta_count);
        pc.points.push_back(pt);
    }
}

void
EntityTracker::close(ros::Publisher *pub)
{
    if (entity_present)
        pushPoint();
    pub->publish(pc);
}


/**************************************************************
 * A PersonTracker is created once for the duration of a ros program.
 * It examines sensor_msgs/Point/Cloud messages from the /entity_tracker
 * topic and determines which ones are people (moving) and which are
 * objects (stationary).  It publishes the locations of people to
 * the /person_locations topic.
 **************************************************************/

PersonTracker::PersonTracker(
        ros::Publisher *loc_p,
        ros::Publisher *new_p)
{
    next_key = 'A';
    count = 0;
    loc_pub = loc_p;
    new_pub = new_p;
}

void
PersonTracker::update(sensor_msgs::PointCloud cloud)
{
    ++count;

    /* Create local work buffers */
    std::set<geometry_msgs::Point32, pt_cmp> unplaced_points;
    for (pt_it np = cloud.points.begin();
            np != cloud.points.end();
            np++)
        unplaced_points.insert(geometry_msgs::Point32(*np));
    std::set<char> unmatched_keys;
    for (map_it mp = monitored_points.begin();
            mp != monitored_points.end();
            mp++)
        unmatched_keys.insert(mp->first);

    /* Align new points with monitored points */
    for (map_it mp = monitored_points.begin();
            mp != monitored_points.end();
            mp++)
        for (pt_it np = cloud.points.begin();
                np != cloud.points.end();
                np++)
            if (likelyIdentical(mp->second, *np))
                scramble(mp->first, *np,
                        &unplaced_points, &unmatched_keys);

    /* Add new points and remove disappeared ones */
    if (unplaced_points.size() > 0)
        for (std::set<geometry_msgs::Point32>::iterator up
                = unplaced_points.begin();
                up != unplaced_points.end();
                up++)
            registerNewPoint(*up);

    if (unmatched_keys.size() > 0)
        for (std::set<char>::iterator c = unmatched_keys.begin();
                c != unmatched_keys.end();
                c++)
        {
            monitored_points.erase(*c);
            displacements.erase(*c);
        }

    /* Check progress to publish */
    if (count % TICK_COUNT == 0)
        publish(cloud);
}

void
PersonTracker::scramble(
        char key,
        const geometry_msgs::Point32 new_point,
        std::set<geometry_msgs::Point32, pt_cmp> *unplaced_points,
        std::set<char> *unmatched_keys
        )
{
    displacements[key].x += new_point.x - monitored_points[key].x;
    displacements[key].y += new_point.y - monitored_points[key].y;
    monitored_points[key] = new_point;
    unmatched_keys->erase(key);
    unplaced_points->erase(new_point);
}

void
PersonTracker::publish(const sensor_msgs::PointCloud & current_entity_cloud)
{
    sensor_msgs::PointCloud loc_cloud;
    loc_cloud.header = current_entity_cloud.header;

    /* Look for moving people */
    for (map_it mp = monitored_points.begin();
            mp != monitored_points.end();
            mp++)
        if (isPerson(mp->first))
            addPersonLoc(mp->first, mp->second, &loc_cloud);

    /* publish a message */
    loc_pub->publish(loc_cloud);
}

void
PersonTracker::addPersonLoc(
        char key, 
        geometry_msgs::Point32 loc,
        sensor_msgs::PointCloud *cloud)
{
    if (persons.find(key) == persons.end())
    {
        persons.insert(key);
        new_pub->publish(loc);
    }
    cloud->points.push_back(loc);
}

bool
PersonTracker::isPerson(char key)
{
    double avg_x = displacements[key].x / TICK_COUNT;
    double avg_y = displacements[key].y / TICK_COUNT;
    double avg_d = sqrt(pow(avg_x, 2.0) + pow(avg_y, 2.0));
    return (avg_d >= MOVEMENT_THRESHOLD);
}

bool
PersonTracker::likelyIdentical(
        geometry_msgs::Point32 first,
        const geometry_msgs::Point32 second)
{
    return ( (abs(first.x - second.x) < IDENTITY_THRESHOLD) &&
             (abs(first.y - second.y) < IDENTITY_THRESHOLD) );
}

void
PersonTracker::registerNewPoint(geometry_msgs::Point32 point)
{
    monitored_points.insert(
            std::make_pair<char, geometry_msgs::Point32>
            (next_key, point)
            );
    geometry_msgs::Point32 p;
    displacements.insert(
            std::make_pair<char, geometry_msgs::Point32>
            (next_key, p)
            );
    next_key++;
}

PersonTracker::PersonTracker()
{ /* Default constructor; intentionally left blank */ }