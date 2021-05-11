#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include "nav_msgs/Odometry.h"
//#include "std_msgs/Float64.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#define WHEEL_RAY 0.1573
#define RPM_ON_RADS 0.10472
#define ROBOT_BASELINE 0.583

class robot_node {

private:
    ros::NodeHandle n; 

    /*ros::Subscriber sub;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    ros::Subscriber sub4;*/
    /*
    robotics_hw1::MotorSpeed message1;
    robotics_hw1::MotorSpeed message2;
    robotics_hw1::MotorSpeed message3;
    robotics_hw1::MotorSpeed message4;*/

    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rr;
    message_filters::Subscriber<nav_msgs::Odometry>* sub_scout_odom;
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,
                                                                nav_msgs::Odometry> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy>* sync;
    ros::Publisher pub; 
    ros::Publisher pub_float;
    ros::Timer timer1;

    double apparent_baseline_ratio;
    int gear_ratio = 30;
    
    /*
    void callback_m(const robotics_hw1::MotorSpeed::ConstPtr& msg){
        message1=*msg;
       // ROS_INFO("Callback1 triggered");
    }

    void callback_m2(const robotics_hw1::MotorSpeed ::ConstPtr& msg){
        message2=*msg;
       // ROS_INFO("Callback2 triggered");
    }

    void callback_m3(const robotics_hw1::MotorSpeed::ConstPtr& msg){
        message3=*msg;
        //ROS_INFO("Callback3 triggered");
    }

    void callback_m4(const robotics_hw1::MotorSpeed::ConstPtr& msg){
        message4=*msg;
       // ROS_INFO("Callback4 triggered");
    }*/


    /*
    void callback_t(const ros::TimerEvent&) {
        pub.publish(message1);
        pub.publish(message2);
        pub.publish(message3);
        pub.publish(message4);
        ROS_INFO("Timer callback triggered, publishing %f, %f, %f, %f", message1.rpm, message2.rpm, message3.rpm, message4.rpm);
    }*/
    
    void message_filter_callback(const robotics_hw1::MotorSpeed::ConstPtr& msg_fl, 
            const robotics_hw1::MotorSpeed::ConstPtr& msg_fr, 
            const robotics_hw1::MotorSpeed::ConstPtr& msg_rl, 
            const robotics_hw1::MotorSpeed::ConstPtr& msg_rr,
            const nav_msgs::Odometry::ConstPtr& msg_odom) 
    {
        ROS_INFO ("Received four rpm messages: (%f,%f,%f,%f)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)",  msg_fl->rpm, msg_fr->rpm, msg_rl->rpm, msg_rr->rpm);
        ROS_INFO ("Received five  messages: (%d,%d,%d,%d,%d)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!)",  msg_fl->header.stamp.sec, 
                                                                                                                    msg_fr->header.stamp.sec, 
                                                                                                                    msg_rl->header.stamp.sec, 
                                                                                                                    msg_rr->header.stamp.sec, 
                                                                                                                    msg_odom->header.stamp.sec);
        double estimated_x = msg_odom->pose.pose.position.x;
        double estimated_y = msg_odom->pose.pose.position.y;
        double estimated_z = msg_odom->pose.pose.position.z;
        double robot_angular_velocity = 0.0;
        double robot_linear_velocity = 0.0;
        //ROS_INFO ("Received scout odometry position: %f,%f,%f",  estimated_x, estimated_y, estimated_z);

        double scout_odom_x_lin_vel = msg_odom->twist.twist.linear.x;
        //double scout_odom_y_lin_vel = msg_odom->twist.twist.linear.y;          ALWAYS 0!
        //double scout_odom_z_lin_vel = msg_odom->twist.twist.linear.z;          ALWAYS 0!

        //double scout_odom_x_ang_vel = msg_odom->twist.twist.angular.x;         ALWAYS 0!
        //double scout_odom_y_ang_vel = msg_odom->twist.twist.angular.y;         ALWAYS 0!
        double scout_odom_z_ang_vel = msg_odom->twist.twist.angular.z;

        
        //ROS_INFO ("Received scout odometry linear velocities: %f m/s on local x axis",  scout_odom_x_lin_vel);

        
        //ROS_INFO ("Received scout odometry angular velocities: %f rad/s around z axis",  scout_odom_z_ang_vel);

        //now that we have the number of round per minutes, we have to compute a mean for the left and right tracks
        gear_ratio = 30;

        double error = 0;
        double linear_velocity_error = 0;
        double angular_velocity_error = 0;

        double min_error = -1;
        int min_gear_ratio = 0;
        double min_apparent_baseline_ratio = 0;
        for(gear_ratio = 30; gear_ratio <= 40; gear_ratio = gear_ratio + 1)
        {
            error = 0;
            double left_track_rad_s = ((double) (msg_fl->rpm/gear_ratio) + (double) (msg_rl->rpm/gear_ratio))*RPM_ON_RADS/2;
            double right_track_rad_s = ((double)(msg_fr->rpm/gear_ratio) + (double) (msg_rr->rpm/gear_ratio))*RPM_ON_RADS/2;

            double left_track_lin_vel = left_track_rad_s*WHEEL_RAY;
            double right_track_lin_vel = right_track_rad_s*WHEEL_RAY;

            for(apparent_baseline_ratio = 1; apparent_baseline_ratio <= 4; apparent_baseline_ratio = apparent_baseline_ratio + 0.1)
            {
                robot_angular_velocity = (right_track_lin_vel - left_track_lin_vel)/(ROBOT_BASELINE*apparent_baseline_ratio);
                robot_linear_velocity = (right_track_lin_vel + left_track_lin_vel)/2;
                linear_velocity_error = std::abs(robot_linear_velocity-scout_odom_x_lin_vel);
                angular_velocity_error = std::abs(robot_angular_velocity-scout_odom_z_ang_vel);
                error = linear_velocity_error + angular_velocity_error;

                if(min_error == -1 || error < min_error)
                {
                    min_error = error;
                    min_gear_ratio = gear_ratio;
                    min_apparent_baseline_ratio = apparent_baseline_ratio;
                }
                if(true)//(linear_velocity_error < 1 && angular_velocity_error < M_PI/2)
                {
                    ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
                    ROS_INFO("TESTING parameters -> gear ratio: %d, apparent baseline ratio: %f, -> Baseline = %f m ", gear_ratio, apparent_baseline_ratio, ROBOT_BASELINE*apparent_baseline_ratio);

                    ROS_INFO ("Tracks angular velocities -> Left: %f rad/s -> %f deg/s, Right: %f rad/s-> %f deg/s", left_track_rad_s, 
                                                                                                                    left_track_rad_s*180/M_PI,
                                                                                                                    right_track_rad_s, 
                                                                                                                    right_track_rad_s*180/M_PI);
                    ROS_INFO ("Tracks linear velocities --> Left: %f m/s, Right: %f m/s", left_track_lin_vel, right_track_lin_vel); 
                    ROS_INFO ("Computed velocities ------> linear: %f m/s, angular: %f rad/s = %f deg/s", robot_linear_velocity, robot_angular_velocity, robot_angular_velocity*180/M_PI);                                                                                                                                                                                              
                    ROS_INFO ("Scout odometry velocities-> linear %f m/s , angular %f rad/s", scout_odom_x_lin_vel, scout_odom_z_ang_vel);
                    ROS_INFO ("ERROR = %f, linear: %f, angular: %f", error, linear_velocity_error, angular_velocity_error);
                   
                }
            }
        }
        
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");


        ROS_INFO("Minimum error: %f found for  gear ratio: %d, apparent baseline ratio: %f", min_error, min_gear_ratio, min_apparent_baseline_ratio);
        
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        

        pub.publish(msg_fl);
        pub.publish(msg_fr);
        pub.publish(msg_rl);
        pub.publish(msg_rr);
    }

public:
    robot_node(){
        sub_fl = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_fl", 1);//FRONT LEFT!!!!!!!
        sub_fr = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_fr", 1);
        sub_rl = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_rl", 1);
        sub_rr = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_rr", 1); //REAR RIGHT!!

        sub_scout_odom = new message_filters::Subscriber<nav_msgs::Odometry>(n, "/scout_odom", 1);
        //just for test
        /*sub->registerCallback((boost::bind(&robot_node::callback_m, this,_1)));
        sub2->registerCallback((boost::bind(&robot_node::callback_m2, this, _1)));
        sub3->registerCallback((boost::bind(&robot_node::callback_m3, this, _1)));
        sub4->registerCallback((boost::bind(&robot_node::callback_m4, this, _1)));*/

        
        pub = n.advertise<robotics_hw1::MotorSpeed >("/rechatter", 1);
        //timer1 = n.createTimer(ros::Duration(1), &robot_node::callback_t, this);
        
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub_fl, *sub_fr, *sub_rl, *sub_rr, *sub_scout_odom);
        sync->registerCallback(boost::bind(&robot_node::message_filter_callback, this, _1, _2, _3, _4, _5));
    }

    
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");
  
  robot_node my_robot_node;
  
  ros::spin();
  
  return 0;
}
