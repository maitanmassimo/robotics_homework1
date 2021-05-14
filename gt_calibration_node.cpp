#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <robotics_hw1/MotorSpeed.h>
#include <dynamic_reconfigure/server.h>
#include <robotics_odometry_project/parametersConfig.h>
#include <tf/transform_datatypes.h>
#include "robotics_odometry_project/resetOdometry.h"
#include "robotics_odometry_project/integratedOdom.h"
#include <math.h> 

#define WHEEL_RAY 0.1573      //ray of the wheels
#define RPM_ON_RADS 0.10472   //constant used to convert from RPM to Rad/s
#define ROBOT_BASELINE 0.583  //length of the (full) robot baseline
#define DEBUG_MODE 1
#define CALIBRATION_MODE 1
#define ROTATE_ON_SPOT_RATIO 0.45
/*  
    ROBOTICS 1st HOMEWORK 2020/2021 
    Student: Maitan Massimo - 10531426 - 944771 - Computer Science and Engineering, Politecnico di Milano
*/

/*
    In this node I put the calibration functionality that allows to estimate the apparent baseline of the robot using the data coming from the ground truth odometry
*/
class gt_calibration_node {

private:
    ros::NodeHandle n; 

    //here I create the message filters for each topic
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rr;
    message_filters::Subscriber<geometry_msgs::PoseStamped>* sub_gt_pose;

    double parameter_x_initial_position;
    double parameter_y_initial_position;
    double parameter_initial_orientation;
    bool parameter_integration_method;



    //define the policy
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,
                                                                geometry_msgs::PoseStamped> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy>* sync;

    double x_position;
    double y_position;
    double orientation;

    long int previous_time_sec;
    long int previous_time_nsec;

    double apparent_baseline_ratio;
    double gear_ratio;

    double mean_apparent_baseline_ratio;
    double mean_gear_ratio;
    int calibration_samples_gear_ratio;
    int calibration_samples_apparent_baseline_ratio;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    
    

    /**
    function I created to estimate the gear ratio on the motors/wheels. Takes as input the 4 RPMs, and reverts the equation
    of the linear velocity in the differential drive kinematic model to compute the gear ratio.
    The point is that now the linear velocity is not known, but has

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
        if(std::abs(linear_velocity)>0.01 ){
            double gear_ratio = (((m1_rpm + m2_rpm + m3_rpm + m4_rpm)*WHEEL_RAY)/(4*linear_velocity))*RPM_ON_RADS;
            if(gear_ratio > 0 && gear_ratio < 100)
                return gear_ratio;
            else
                return -1;
        }
        else{ //when the linear velocity of the robot is 0, we cannot estimate the gear ratio, because we would have 0 as denominator, so we return an error value
            return -1;
        } 
    }

    /**
    function I created to estimate the ratio between the apparent baseline and the real one. Takes as input the 4 RPMs and a fixed gear ratio, but differently from the procedure that
    exploits the scout odometry, this time we don't have an angular velocity provided by the manufacturer.
    So we have to estimate it measuring the difference in the orientation, and revert the equation of the angular velocity in the differential drive kinematic model 
    to compute the apparent baseline. 
    The reasoning is quite similar to the one of the other function, and the motors angular velocities are already inverted as well.
    **/
    double estimate_apparent_baseline_ratio(double m1_rpm,double m2_rpm,double m3_rpm, double m4_rpm, double angular_velocity, double gear_ratio)
    {
        //ROS_INFO("Estimating apparent baseline ratio with data: %f, %f, %f, %f, %f, %f", m1_rpm, m2_rpm, m3_rpm,  m4_rpm,  angular_velocity, gear_ratio);
        if(std::abs(angular_velocity)>0.01 && angular_velocity < M_PI){
            double ap_bl_ratio = (((m2_rpm + m4_rpm - m1_rpm - m3_rpm)*WHEEL_RAY*RPM_ON_RADS)/(2*angular_velocity*(gear_ratio)*ROBOT_BASELINE));
            if(ap_bl_ratio > 0 && ap_bl_ratio < 100)
                return ap_bl_ratio;
            else
                return -1;
            
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
            const geometry_msgs::PoseStamped::ConstPtr& msg_gt) 
    {
        if(DEBUG_MODE){
            ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            ROS_INFO ("PARAMETERS:");
            ROS_INFO("parameter_x_initial_position : %f",  parameter_x_initial_position);
            ROS_INFO("parameter_y_initial_position : %f",  parameter_y_initial_position);
            ROS_INFO("parameter_initial_orientation : %f",  parameter_initial_orientation);
            ROS_INFO("integration method : %f",  parameter_initial_orientation);
        }
        double rpm_fl_wheel = - msg_fl->rpm;
        double rpm_fr_wheel = msg_fr->rpm;
        double rpm_rl_wheel = - msg_rl->rpm;
        double rpm_rr_wheel = msg_rr->rpm;
        long int current_time_sec;
        long int current_time_nsec;
        double time_step;

        if(DEBUG_MODE){
            ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            ROS_INFO ("Received four rpm messages: (%f,%f,%f,%f))",  rpm_fl_wheel,rpm_fr_wheel, rpm_rl_wheel,rpm_rr_wheel);
            //ROS_INFO ("Received five  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.nsec, msg_fr->header.stamp.nsec, msg_rl->header.stamp.nsec, msg_rr->header.stamp.nsec, msg_odom->header.stamp.nsec);
            //ROS_INFO ("sec  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.sec, msg_fr->header.stamp.sec, msg_rl->header.stamp.sec, msg_rr->header.stamp.sec, msg_gt->header.stamp.sec);
            //ROS_INFO ("nsec  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.nsec, msg_fr->header.stamp.nsec, msg_rl->header.stamp.nsec, msg_rr->header.stamp.nsec, msg_gt->header.stamp.nsec);
        }

        double gt_pose_x = msg_gt->pose.position.x;
        double gt_pose_y = msg_gt->pose.position.y;

        tf::Quaternion q(
                    msg_gt->pose.orientation.x,
                    msg_gt->pose.orientation.y,
                    msg_gt->pose.orientation.z,
                    msg_gt->pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double gt_pose_orientation = yaw;

        

        double robot_angular_velocity = 0.0;
        double robot_linear_velocity = 0.0;

        double error = 0;
        double linear_velocity_error = 0;
        double angular_velocity_error = 0;


        //linear velocity of the robot
        double gt_x_lin_vel = 0.0;


        //angular velocity of the robot
        double gt_z_ang_vel = 0.0;

        if(DEBUG_MODE){
            ROS_INFO ("Received scout odometry position: %f,%f,%f",  gt_pose_x, gt_pose_y, gt_pose_orientation);
        }

        //now that we have the number of round per minutes, we have to compute a mean for the left and right tracks
        current_time_sec = msg_fl->header.stamp.sec;
        current_time_nsec = msg_fl->header.stamp.nsec;
      
        if(CALIBRATION_MODE){
            
            if(previous_time_sec!= -1){

                if(current_time_sec == previous_time_sec)
                    time_step = (current_time_nsec - previous_time_nsec)/1e+9;
                else
                    time_step = (current_time_nsec - previous_time_nsec)/1e+9 + 1;
                
                ROS_INFO("current_time_sec %ld, previous_time_sec %ld", current_time_sec, previous_time_sec);
                ROS_INFO("current_time_nsec %ld, previous_time_nsec %ld, time step %f", current_time_nsec, previous_time_nsec, time_step);
            
                /*
                //compute the linear and angular velocity of the robot given their trajectory
                double time_step_1 = time_step*ROTATE_ON_SPOT_RATIO; //this is the time in which the robot has to turn towards the position to reach

                //first we need to find the angular coefficient of the line that starts from the old position and ends in the new one
                double m = atan((gt_pose_y - y_position)/(gt_pose_x - x_position));

                //now we have to find the change in orientation needed for the robot to align with this orientation
                double delta_theta = m - orientation;
                //now we divide by the time_step_1 to obtain the angular velocity of the robot
                double omega_1 = delta_theta/time_step_1;
                ROS_INFO ("timestep_1 = %f, m = %f rad  = %f deg, delta_theta = %f rad = %f deg, omega_1 = %f rad/s= %f deg/s",time_step_1, m, m*180/M_PI, delta_theta,delta_theta*180/M_PI, omega_1, omega_1*180/M_PI);


                double time_step_2 = time_step - 2*time_step*ROTATE_ON_SPOT_RATIO; //this is the time in which the robot has to reach the position
                
                double traveled_distance = sqrt(pow((gt_pose_y - y_position), 2) + pow((gt_pose_x - x_position), 2));
                gt_x_lin_vel = traveled_distance/time_step_2;

                ROS_INFO ("timestep_2 = %f, traveled_distance = %f, gt_x_lin_vel = %f",time_step_2,traveled_distance, gt_x_lin_vel );

                
                //now we divide by the time_step_1 to obtain the angular velocity of the robot
                double delta_theta_2 = gt_pose_orientation -  m;
                double time_step_3 =  time_step_1;
                double omega_2 = delta_theta_2/time_step_3;
                ROS_INFO ("timestep_3 = %f, delta_theta = %f rad = %f deg, omega_1 rad/s = %f = %f deg/s",time_step_3, delta_theta_2,delta_theta_2*180/M_PI, omega_2, omega_2*180/M_PI);
                */
                double traveled_distance = sqrt(pow((gt_pose_y - y_position), 2) + pow((gt_pose_x - x_position), 2));
                double delta_theta = (gt_pose_orientation - orientation);
                if(std::abs(delta_theta) > M_PI)
                    if(delta_theta>0)
                        delta_theta = 2*M_PI - delta_theta; 
                    else delta_theta = delta_theta + 2*M_PI;
                gt_x_lin_vel = traveled_distance/time_step;
                gt_z_ang_vel = delta_theta/time_step;

                ROS_INFO ("timestep = %f, delta_theta = %f rad = %f deg, omega rad/s = %f = %f deg/s, linear velocity = %f m/s",time_step, delta_theta ,delta_theta*180/M_PI, gt_z_ang_vel, gt_z_ang_vel*180/M_PI, gt_x_lin_vel);
                double estimated_gear_ratio = estimate_gear_ratio(rpm_fl_wheel, rpm_fr_wheel, rpm_rl_wheel, rpm_rr_wheel, gt_x_lin_vel);
                double estimated_apparent_baseline_ratio = estimate_apparent_baseline_ratio(rpm_fl_wheel, rpm_fr_wheel, rpm_rl_wheel, rpm_rr_wheel, gt_z_ang_vel, gear_ratio);
                ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
                ROS_INFO ("CALIBRATION");
                if(estimated_gear_ratio == -1){
                    ROS_INFO("unable to estimate gear ratio");
                    gear_ratio = 40;
                }else{
                    ROS_INFO("Estimated gear ratio: %f", estimated_gear_ratio);
                    gear_ratio = estimated_gear_ratio;
                    update_average_gear_ratio(gear_ratio);
                }
                ROS_INFO("Average gear ratio: %f\t# samples = %d", mean_gear_ratio, calibration_samples_gear_ratio);

                if(estimated_apparent_baseline_ratio == -1){
                    ROS_INFO("unable to estimate apparent baseline ratio");
                    apparent_baseline_ratio = 2;

                }else{
                    ROS_INFO("Estimated apparent baseline ratio: %f,\tapparent baseline: %f m", estimated_apparent_baseline_ratio, estimated_apparent_baseline_ratio*ROBOT_BASELINE);
                    apparent_baseline_ratio = estimated_apparent_baseline_ratio;
                    update_average_apparent_baseline(apparent_baseline_ratio);
                } 
                ROS_INFO("Average apparent baseline ratio: %f,\tapparent baseline: %f m,\t# samples = %d", mean_apparent_baseline_ratio,mean_apparent_baseline_ratio*ROBOT_BASELINE, calibration_samples_gear_ratio);
                
            }
            previous_time_sec = current_time_sec;
            previous_time_nsec = current_time_nsec;
            x_position = gt_pose_x;
            y_position = gt_pose_y;
            orientation = gt_pose_orientation;
        }else{
        
        
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
            linear_velocity_error = std::abs(robot_linear_velocity-gt_x_lin_vel);
            angular_velocity_error = std::abs(robot_angular_velocity-gt_z_ang_vel);
            error = linear_velocity_error + angular_velocity_error;

            if(DEBUG_MODE)
            {
                ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
                ROS_INFO ("VELOCITY ESTIMATION");
                /*ROS_INFO ("Tracks ANGULAR velocities -> \tLeft: %f rad/s = %f deg/s,\tRight: %f rad/s = %f deg/s", left_track_rad_s, 
                                                                                                                left_track_rad_s*180/M_PI,
                                                                                                                right_track_rad_s, 
                                                                                                                right_track_rad_s*180/M_PI);
                ROS_INFO ("Tracks LINEAR velocities --> \tLeft: %f m/s,\tRight: %f m/s", left_track_lin_vel, right_track_lin_vel); */
                ROS_INFO ("ESTIMATED velocities ------> \tLinear: %f m/s,\tAngular: %f rad/s = %f deg/s", robot_linear_velocity, robot_angular_velocity, robot_angular_velocity*180/M_PI);                                                                                                                                                                                              
                ROS_INFO ("SCOUT odometry velocities--> \tLinear %f m/s,\tAngular %f rad/s", gt_x_lin_vel, gt_z_ang_vel);
                ROS_INFO ("ERROR = %f, \t\tLinear: %f m/s,\tAngular: %f rad/s = %f deg/s", error, linear_velocity_error, angular_velocity_error, angular_velocity_error*180/M_PI);
                //ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
            }    

            //given the robot velocities, let's compute the positions integrating
        
            current_time_sec = msg_fl->header.stamp.sec;
            current_time_nsec = msg_fl->header.stamp.nsec;


            if(previous_time_sec!= -1){

                if(current_time_sec == previous_time_sec)
                    time_step = (current_time_nsec - previous_time_nsec)/1e+9;
                else
                    time_step = (current_time_nsec - previous_time_nsec)/1e+9 + 1;
                
                ROS_INFO("current_time_sec %ld, previous_time_sec %ld", current_time_sec, previous_time_sec);
                ROS_INFO("current_time_nsec %ld, previous_time_nsec %ld, time step %f", current_time_nsec, previous_time_nsec, time_step);

                double x_position_euler = 0;
                double y_position_euler = 0;
                double orientation_euler = 0;
                double x_position_rk = 0;
                double y_position_rk = 0;
                double orientation_rk = 0;

                if(!parameter_integration_method){ //parameter_integration_method = 0 -> Euler
                    
                    x_position_euler = time_step*std::cos(orientation)*robot_linear_velocity + x_position;
                    y_position_euler = time_step*std::sin(orientation)*robot_linear_velocity + y_position;
                    orientation_euler = time_step*robot_angular_velocity + orientation;
                    if(DEBUG_MODE){
                        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
                        ROS_INFO ("ODOMETRY - EULER INTEGRATION METHOD");
                        ROS_INFO ("X position:\tEstimated: %f\tScout odometry: %f\tError: %f", x_position_euler, gt_pose_x, std::abs(x_position_euler-gt_pose_x));
                        ROS_INFO ("Y positison:\tEstimated: %f\tScout odometry: %f\tError: %f", y_position_euler, gt_pose_y, std::abs(y_position_euler-gt_pose_y));
                        ROS_INFO ("Orientation:\tEstimated: %f\tScout odometry: %f\tError: %f", orientation_euler, gt_pose_orientation, std::abs(orientation_euler-gt_pose_orientation));
                        }
                    transform.setOrigin( tf::Vector3(x_position_euler, y_position_euler, 0) );
                    tf::Quaternion q;
                    q.setRPY(0, 0, orientation_euler);
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "custom_odom"));
                }else{
                    
                    x_position_rk = time_step*std::cos(orientation+(time_step*robot_angular_velocity)/2)*robot_linear_velocity + x_position;
                    y_position_rk = time_step*std::sin(orientation+(time_step*robot_angular_velocity)/2)*robot_linear_velocity + y_position;
                    orientation_rk = time_step*robot_angular_velocity + orientation;
                    if(DEBUG_MODE){
                        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
                        ROS_INFO ("ODOMETRY - RUNGE-KUTTA INTEGRATION METHOD");
                        ROS_INFO ("X position:\tEstimated: %f\tScout odometry: %f\tError: %f", x_position_rk, gt_pose_x, std::abs(x_position_rk-gt_pose_x));
                        ROS_INFO ("Y position:\tEstimated: %f\tScout odometry: %f\tError: %f", y_position_rk, gt_pose_y, std::abs(y_position_rk-gt_pose_y)); 
                        ROS_INFO ("Orientation:\tEstimated: %f\tScout odometry: %f\tError: %f", orientation_rk, gt_pose_orientation, std::abs(orientation_rk-gt_pose_orientation));
                    }
                
                    transform.setOrigin( tf::Vector3(x_position_rk, y_position_rk, 0) );
                    tf::Quaternion q;
                    q.setRPY(0, 0, orientation_rk);
                    transform.setRotation(q);
                    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "custom_odom"));
                    
                }

                if(parameter_integration_method){

                    x_position = x_position_rk;
                    y_position = y_position_rk;
                    orientation = orientation_rk;
                }else{
                    x_position = x_position_euler;
                    y_position = y_position_euler;
                    orientation = orientation_euler;
                }

                ROS_INFO ("Position and orientation updated to %f, %f, %f", x_position, y_position, orientation );
                transform.setOrigin( tf::Vector3(gt_pose_x, gt_pose_y, 0) );
                q.setRPY(0, 0, gt_pose_orientation);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gt_pose"));
                
            }
            ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");

            previous_time_sec = current_time_sec;
            previous_time_nsec = current_time_nsec;
            
            /*pub_rechatter.publish(msg_fl);                -> for debugging purposes!
            pub_rechatter.publish(msg_fr);
            pub_rechatter.publish(msg_rl);
            pub_rechatter.publish(msg_rr);
            pub_scout_odometry.publish(msg_odom);*/

            /*
            geometry_msgs::TwistStamped twist_stamped;
            twist_stamped.header.stamp.sec = current_time_sec;
            twist_stamped.header.stamp.nsec = current_time_nsec;
            twist_stamped.header.frame_id = "twist_frame_id";
            twist_stamped.twist.linear.x = robot_linear_velocity; 
            twist_stamped.twist.linear.y = 0.0;
            twist_stamped.twist.linear.z = 0.0;
            twist_stamped.twist.angular.x = 0.0;
            twist_stamped.twist.angular.y = 0.0;
            twist_stamped.twist.angular.z = robot_angular_velocity;

            pub_my_odom_velocities.publish(twist_stamped); //TBD: create another node that reads from that topic and publishes on tf

            robotics_odometry_project::integratedOdom customMessage;

            customMessage.odom.header.stamp.sec = current_time_sec;
            customMessage.odom.header.stamp.nsec = current_time_nsec;
            customMessage.odom.header.frame_id = "custom_odom";
            customMessage.odom.pose.pose.position.x = x_position;
            customMessage.odom.pose.pose.position.y = y_position;
            customMessage.odom.pose.pose.position.z = 0.0;
            geometry_msgs::Quaternion q_2;
            q.setRPY(0, 0, orientation);
            quaternionTFToMsg(q , q_2);
            customMessage.odom.pose.pose.orientation.x = q_2.x;
            customMessage.odom.pose.pose.orientation.y = q_2.y;
            customMessage.odom.pose.pose.orientation.z = q_2.z;
            customMessage.odom.pose.pose.orientation.w = q_2.w;
            customMessage.odom.twist.twist.linear.x = robot_linear_velocity;
            customMessage.odom.twist.twist.linear.y = 0.0;
            customMessage.odom.twist.twist.linear.z = 0.0;
            customMessage.odom.twist.twist.angular.x = 0.0;
            customMessage.odom.twist.twist.angular.y = 0.0;
            customMessage.odom.twist.twist.angular.z = robot_angular_velocity;
        
            if(this->parameter_integration_method)
                customMessage.method.data = "rk";
            else
                customMessage.method.data = "Euler";

            pub_my_odom_position_integration_m.publish(customMessage);*/
        }
    }


public:
    gt_calibration_node(){


        ROS_INFO ("ROBOT GROUND TRUTH CALIBRATION NODE");      

        //in order to estimate the angular velocity of the wheel, the node must subscribe to the four motors topics 
        sub_fl = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_fl", 1);
        sub_fr = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_fr", 1);
        sub_rl = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_rl", 1);
        sub_rr = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_rr", 1); 

        //in order to estimate the gear ratio and the apparent baseline, the node must subscribe to the constructor's odometry topic
        sub_gt_pose = new message_filters::Subscriber<geometry_msgs::PoseStamped>(n, "/gt_pose", 1);

        mean_apparent_baseline_ratio = 1;
        mean_gear_ratio = 40;
        calibration_samples_gear_ratio = 0;
        calibration_samples_apparent_baseline_ratio = 0;
        previous_time_sec = -1;
        previous_time_nsec = -1;


        
        gear_ratio = 38.41;                 //computed through calibration over >2000 samples   
        apparent_baseline_ratio = 1.734;    //computed through calibration over >2000 samples

        n.getParam("/parameter_x_initial_position", parameter_x_initial_position);
        n.getParam("/parameter_y_initial_position", parameter_y_initial_position);
        n.getParam("/parameter_initial_orientation", parameter_initial_orientation);
        n.getParam("/int_meth", parameter_integration_method);

        x_position = parameter_x_initial_position;
        y_position = parameter_y_initial_position;
        orientation = parameter_initial_orientation;
        
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub_fl, *sub_fr, *sub_rl, *sub_rr, *sub_gt_pose);
        sync->registerCallback(boost::bind(&gt_calibration_node::message_filter_callback, this, _1, _2, _3, _4, _5));
    }    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gt_calibration_node");
    gt_calibration_node my_gt_calibration_node;
    ros::spin(); 
    return 0;
}
