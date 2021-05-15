#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
//#include "robot_node.cpp"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <robotics_hw1/MotorSpeed.h>
#include "robotics_odometry_project/resetOdometry.h"

/*  
    ROBOTICS 1st HOMEWORK 2020/2021 
    Student: Maitan Massimo - 10531426 - 944771 - Computer Science and Engineering, Politecnico di Milano
*/

/*
    This node was used to test the services
*/

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_utility_node");   
    //utility_node my_utility_node;
    ROS_INFO("I'm the utility node, used by Massimo to test the services =)");
    ros::NodeHandle n;
    ros::init(argc, argv, "project_test_client");
    robotics_odometry_project::resetOdometry srv;
    ros::ServiceClient client;

    if (argc != 4){

        ROS_INFO("The odometry will be reset to default (0,0,0)");
        ros::NodeHandle n;
        client = n.serviceClient<std_srvs::Empty>("reset_odometry_origin");
    }else{
        client = n.serviceClient<robotics_odometry_project::resetOdometry>("reset_odometry_custom");
        srv.request.x = atof(argv[1]);
        srv.request.y = atof(argv[2]);
        srv.request.w = atof(argv[3]);
        ROS_INFO("The odometry will be reset to custom value (%f,%f,%f)", srv.request.x, srv.request.y, srv.request.w);
    }   
    if (client.call(srv)){
        ROS_INFO("Odometry correctly reset!");
    }
    else {
        ROS_ERROR("Failed to reset odometry!");
        return 1;
    }

    ros::spin();
    
    return 0;
}
