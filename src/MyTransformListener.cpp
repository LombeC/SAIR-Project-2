//
// Created by lombe with help from the tutorials http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "tf/transform_listener.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv) {
    //initialize the node
    ros::init(argc, argv, "my_transform_listener");
    ros::NodeHandle node;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";

    bool location_one = true;
    bool location_two = true;

    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (node.ok()) {

        //start by moving to location one
        if (location_one) {
            //we'll send a goal to the robot to move to location one
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = -1.0; //hard coding the goal values
            goal.target_pose.pose.position.y = 1.0;
            goal.target_pose.pose.position.z = 0.0;

            goal.target_pose.pose.orientation.w = 1.0;
            location_one = false;
            ROS_INFO("Sending goal to move to location one");
            ac.sendGoal(goal);
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The robot moved to location one");
            } else {
                ROS_ERROR("The base failed to move to location one");
            }
        }
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        double x = transform.getOrigin().x();
        double y = transform.getOrigin().y();
        double z = transform.getOrigin().z();

        ROS_INFO("Current position: ( x: %.2f , y: %.2f, z: %.2f )\n", x, y, z);

        if (location_two) {
            //we'll send a goal to the robot to move to location two
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose.position.x = 0.5; //hard coding the goal values
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.position.z = 0.0;

            goal.target_pose.pose.orientation.w = -1;

            ROS_INFO("Sending goal to move to location two");
            ac.sendGoal(goal);
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("The robot moved to location two");
            } else {
                ROS_ERROR("The robot failed to move to location two");
            }

            location_two = false; // using booleans to ensure robot only moves once

        }
        rate.sleep();
    }
    return 0;
}

