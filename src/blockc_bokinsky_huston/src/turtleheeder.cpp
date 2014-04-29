/*
 * Name:         Huston Bokinsky
 * Date:         1 February, 2014
 * Course:       CSCE 574 - Robotics
 * Instructor:   Dr. O'Kane
 * Assignment:   Project 1 - Block "C"
 * Description:  Use ros::Publisher and ros::Subscriber nodes to make
 *               a turtlesim turtle describe a block C route.
 *
 * Disclaimer:   This code is for a class project and should not be
 *               used in production environments where people's lives
 *               are dependent upon its proper functioning.
 */


#include "turtleherder.h"


int main(int argc, char **argv)
{
    /* 0. Check that we know where our instructions are */
    std::string instruction_file;
    if (argc == 1) {
        std::cout << "Please provide as an argument to this function \n \
call a filename (full path) where we can find\n \
the instruction set for the turtle.  Block C instructions are\n \
included in the data/ directory of this package.\n \
Thanks for your attention!";
        return 1;
    }
    else
        instruction_file = argv[1];

    /* 1. Some ros initialization */
    ros::init(argc, argv, "blockc_node");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    /* 2. Instantiate our publisher... */
    TurtleHerder th(instruction_file, nh);
    /* 3. ...and a subscriber */
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,
            &TurtleHerder::checkPosition, &th);
    /* 4. Let roscore catch up on linking publishers and subscribers */
    rate.sleep();
    /* 5. And we're off! */
    ros::spin();
}
