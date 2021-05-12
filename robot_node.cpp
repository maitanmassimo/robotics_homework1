#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotics_hw1/MotorSpeed.h"
#include "nav_msgs/Odometry.h"
//#include "std_msgs/Float64.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#define WHEEL_RAY 0.1573      //ray of the wheels
#define RPM_ON_RADS 0.10472   //constant used to convert from RPM to Rad/s
#define ROBOT_BASELINE 0.583  //length of the (full) robot baseline

#define CALIBRATION_MODE 1 //if 1, the message filter callback will print information about the estimated parameters, such as the GEAR RATIO of the wheels and the APPARENT BASELINE for the skid steering
                            //if 0, the message filter callback will just compute the odometry
#define DEBUG_MODE 0 // if 1 the information useful for debugging purpose will be print on screen
class robot_node {

private:
    ros::NodeHandle n; 


    //here I create the message filters for each topic
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rr;
    message_filters::Subscriber<nav_msgs::Odometry>* sub_scout_odom;

    //define the policy
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,
                                                                nav_msgs::Odometry> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy>* sync;

    ros::Publisher pub;  //to publish the messages received from the 4 wheel topics in the /rechatter topic (for debugging purposes)
    ros::Publisher pub2; //to publish the scout odometry messages in the /my_odom topic (for debugging purposes)
    ros::Timer timer1; //never say never

    double apparent_baseline_ratio;
    double gear_ratio;

    double mean_apparent_baseline_ratio;
    double mean_gear_ratio;
    int calibration_samples_gear_ratio;
    int calibration_samples_apparent_baseline_ratio;
    /**
    function I created to estimate the gear ratio on the motors/wheels. Takes as input the 4 RPMs and the known linear velocity, and reverts the equation
    of the linear velocity in the differential drive kinematic model to compute the gear ratio

    Important: the RPMs values for the left wheels are already inverted, differently from the bag provided by the course instructors

    The angular velocity of each wheel is given by the angular velocity of the motor divided by the ratio, and the angular velocity of each track is given 
    by the mean of the two wheels angular velocities. The linear velocity of each "track" is given by its angular velocity multiplied for the radius of the wheel.

    Finally, the linear velocity of the robot is given by the mean of the two track velocities:

    lin_vel_robot = (lin_vel_track_l + lin_vel_track_r)/2   <--- substitute track linear velocities with the formula containing the track angular velocities
    lin_vel_robot = R*(ang_vel_track_l + ang_vel_track_r)/2 <--- substitute track angular velocities with the formulas containing the single wheels angular velocities (the mean)
    lin_vel_robot = R*((ang_vel_wheel_fl + ang_vel_wheel_rl)/2 + (ang_vel_wheel_fr + ang_vel_wheel_rr)/2)/2 = R * (ang_vel_wheel_fl + ang_vel_wheel_rl + ang_vel_wheel_fr + ang_vel_wheel_rr)/(2*2)

    since for each wheel the angular velocity is given by the motor angular velocity divided by the ratio:

    lin_vel_robot = R * (ang_vel_motor_fl + ang_vel_motor_rl + ang_vel_motor_fr + ang_vel_motor_rr)/(2*2*gear_ratio)

    if we solve the equation for the gear ratio, we obain that it is given by the sum of the motor RPM angular velocities (that must be converted into Rad/s) multiplied by the 
    ray of the wheel, divided by 4 times the linear velocity of the robot
    **/

    double estimate_gear_ratio(double m1_rpm,double m2_rpm,double m3_rpm, double m4_rpm, double linear_velocity){
        //ROS_INFO("Estimating gear ratio with data: %f, %f, %f, %f, %f", m1_rpm, m2_rpm, m3_rpm,  m4_rpm,  linear_velocity);
        if(std::abs(linear_velocity)!=0){
            return (((m1_rpm + m2_rpm + m3_rpm + m4_rpm)*WHEEL_RAY)/(4*linear_velocity))*RPM_ON_RADS;
        }
        else{ //when the linear velocity of the robot is 0, we cannot estimate the gear ratio, because we would have 0 as denominator, so we return an error value
            return -1;
        } 
    }

    /**
    function I created to estimate the ratio between the apparent baseline and the real one. Takes as input the 4 RPMs, the known angular velocity, a fixed gear ratio, and reverts the equation
    of the angular velocity in the differential drive kinematic model to compute the apparent baseline. 
    The reasoning is quite similar to the one of the other function, and the motors angular velocities are already inverted as well.
    **/
    double estimate_apparent_baseline_ratio(double m1_rpm,double m2_rpm,double m3_rpm, double m4_rpm, double angular_velocity, double gear_ratio)
    {
        //ROS_INFO("Estimating apparent baseline ratio with data: %f, %f, %f, %f, %f, %f", m1_rpm, m2_rpm, m3_rpm,  m4_rpm,  angular_velocity, gear_ratio);
        if(std::abs(angular_velocity)!=0){
            return (((m2_rpm + m4_rpm - m1_rpm - m3_rpm)*WHEEL_RAY*RPM_ON_RADS)/(2*angular_velocity*(gear_ratio)*ROBOT_BASELINE));
        }
        else{
            return -1; //when the robot is not turning it is not possible to estimate the apparent baseline, so we return an error value
        }
    }
    
    void update_average_apparent_baseline(double new_apparent_baseline){
        
        mean_apparent_baseline_ratio = (mean_apparent_baseline_ratio*calibration_samples_apparent_baseline_ratio + new_apparent_baseline)/(calibration_samples_apparent_baseline_ratio+1);
        calibration_samples_apparent_baseline_ratio++;
    }

    void update_average_gear_ratio(double new_gear_ratio){
        
        mean_gear_ratio = (mean_gear_ratio*calibration_samples_gear_ratio + new_gear_ratio)/(calibration_samples_gear_ratio+1);
        calibration_samples_gear_ratio++;
    }

    void message_filter_callback(const robotics_hw1::MotorSpeed::ConstPtr& msg_fl, 
            const robotics_hw1::MotorSpeed::ConstPtr& msg_fr, 
            const robotics_hw1::MotorSpeed::ConstPtr& msg_rl, 
            const robotics_hw1::MotorSpeed::ConstPtr& msg_rr,
            const nav_msgs::Odometry::ConstPtr& msg_odom) 
    {
        double rpm_fl_wheel = - msg_fl->rpm;
        double rpm_fr_wheel = msg_fr->rpm;
        double rpm_rl_wheel = - msg_rl->rpm;
        double rpm_rr_wheel = msg_rr->rpm;

        if(DEBUG_MODE){
            ROS_INFO ("Received four rpm messages: (%f,%f,%f,%f))",  rpm_fl_wheel,rpm_fr_wheel, rpm_rl_wheel,rpm_rr_wheel);
            //ROS_INFO ("Received five  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.nsec, msg_fr->header.stamp.nsec, msg_rl->header.stamp.nsec, msg_rr->header.stamp.nsec, msg_odom->header.stamp.nsec);
        }
        double estimated_x = msg_odom->pose.pose.position.x;
        double estimated_y = msg_odom->pose.pose.position.y;
        double estimated_z = msg_odom->pose.pose.position.z;
        double scout_odom_x_lin_vel = msg_odom->twist.twist.linear.x;
        double scout_odom_z_ang_vel = msg_odom->twist.twist.angular.z;

        double robot_angular_velocity = 0.0;
        double robot_linear_velocity = 0.0;

        double error = 0;
        double linear_velocity_error = 0;
        double angular_velocity_error = 0;

        if(DEBUG_MODE){
            ROS_INFO ("Received scout odometry position: %f,%f,%f",  estimated_x, estimated_y, estimated_z);
            ROS_INFO ("Received scout odometry linear velocities: %f m/s on local x axis",  scout_odom_x_lin_vel);
            ROS_INFO ("Received scout odometry angular velocities: %f rad/s around z axis",  scout_odom_z_ang_vel);
        }

        //now that we have the number of round per minutes, we have to compute a mean for the left and right tracks
      
        if(CALIBRATION_MODE){
            double estimated_gear_ratio = estimate_gear_ratio(rpm_fl_wheel, rpm_fr_wheel, rpm_rl_wheel, rpm_rr_wheel, scout_odom_x_lin_vel);
            double estimated_apparent_baseline_ratio = estimate_apparent_baseline_ratio(rpm_fl_wheel, rpm_fr_wheel, rpm_rl_wheel, rpm_rr_wheel, scout_odom_z_ang_vel, gear_ratio);
            ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            ROS_INFO ("CALIBRATION");
            if(estimated_gear_ratio == -1){
                ROS_INFO("unable to estimate gear ratio");
                gear_ratio = 40;
            }else{
                ROS_INFO("Estimated gear ratio: %f", estimated_gear_ratio);
                gear_ratio = estimated_gear_ratio;
                update_average_gear_ratio(gear_ratio);

                ROS_INFO("Average gear ratio: %f\t# samples = %d", mean_gear_ratio, calibration_samples_gear_ratio);
            }

            if(estimated_apparent_baseline_ratio == -1){
                ROS_INFO("unable to estimate apparent baseline ratio");
                apparent_baseline_ratio = 2;

            }else{
                ROS_INFO("Estimated apparent baseline ratio: %f,\tapparent baseline: %f m", estimated_apparent_baseline_ratio, estimated_apparent_baseline_ratio*ROBOT_BASELINE);
                apparent_baseline_ratio = estimated_apparent_baseline_ratio;
                update_average_apparent_baseline(apparent_baseline_ratio);

                ROS_INFO("Average apparent baseline ratio: %f,\tapparent baseline: %f m,\t# samples = %d", mean_apparent_baseline_ratio,mean_apparent_baseline_ratio*ROBOT_BASELINE, calibration_samples_gear_ratio);
            }
            
        }
        
        
        
        //compute tracks angular velocities
        double left_track_rad_s = ((double) (rpm_fl_wheel/gear_ratio) + (double) (rpm_rl_wheel/gear_ratio))*RPM_ON_RADS/2;
        double right_track_rad_s = ((double)(rpm_fr_wheel/gear_ratio) + (double) (rpm_rr_wheel/gear_ratio))*RPM_ON_RADS/2;

        //compute tracks linear velocities
        double left_track_lin_vel = left_track_rad_s*WHEEL_RAY;
        double right_track_lin_vel = right_track_rad_s*WHEEL_RAY;
        
        //compute robot's linear and angular velocities
        robot_angular_velocity = (right_track_lin_vel - left_track_lin_vel)/(ROBOT_BASELINE*apparent_baseline_ratio);
        robot_linear_velocity = (right_track_lin_vel + left_track_lin_vel)/2;

        //estimates the error between my velocity estimation and the one of the odometry provided by the constructor
        linear_velocity_error = std::abs(robot_linear_velocity-scout_odom_x_lin_vel);
        angular_velocity_error = std::abs(robot_angular_velocity-scout_odom_z_ang_vel);
        error = linear_velocity_error + angular_velocity_error;

        if(DEBUG_MODE)
        {
            ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            ROS_INFO ("VELOCITY ESTIMATION");
            ROS_INFO ("Tracks ANGULAR velocities -> \tLeft: %f rad/s -= %f deg/s,\tRight: %f rad/s = %f deg/s", left_track_rad_s, 
                                                                                                            left_track_rad_s*180/M_PI,
                                                                                                            right_track_rad_s, 
                                                                                                            right_track_rad_s*180/M_PI);
            ROS_INFO ("Tracks LINEAR velocities --> \tLeft: %f m/s,\tRight: %f m/s", left_track_lin_vel, right_track_lin_vel); 
            ROS_INFO ("ESTIMATED velocities ------> \tLinear: %f m/s,\tAngular: %f rad/s = %f deg/s", robot_linear_velocity, robot_angular_velocity, robot_angular_velocity*180/M_PI);                                                                                                                                                                                              
            ROS_INFO ("SCOUT odometry velocities--> \tLinear %f m/s,\tAngular %f rad/s", scout_odom_x_lin_vel, scout_odom_z_ang_vel);
            ROS_INFO ("ERROR = %f, \tLinear: %f m/s,\tAngular: %f rad/s = %f deg/s", error, linear_velocity_error, angular_velocity_error, angular_velocity_error*180/M_PI);
            ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        }
        
        
        pub.publish(msg_fl);
        pub.publish(msg_fr);
        pub.publish(msg_rl);
        pub.publish(msg_rr);
        pub2.publish(msg_odom);
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

        mean_apparent_baseline_ratio = 1;
        mean_gear_ratio = 30;
        calibration_samples_gear_ratio = 0;
        calibration_samples_apparent_baseline_ratio = 0;

        pub = n.advertise<robotics_hw1::MotorSpeed >("/rechatter", 1);
        pub2 = n.advertise<nav_msgs::Odometry >("/my_odom", 1);
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

