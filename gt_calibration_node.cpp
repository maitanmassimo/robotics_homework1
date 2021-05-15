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

    The reasoning is explained in the robot_node class
    **/

    double estimate_gear_ratio(double m1_rpm,double m2_rpm,double m3_rpm, double m4_rpm, double linear_velocity){
        ROS_INFO("Estimating gear ratio with data: %f, %f, %f, %f, %f", m1_rpm, m2_rpm, m3_rpm,  m4_rpm,  linear_velocity);
        if(std::abs(linear_velocity)>0.01 ){    //it makes no sense to estimate the gear ratio with a too low linear velocity
            double gear_ratio = (((m1_rpm + m2_rpm + m3_rpm + m4_rpm)*WHEEL_RAY)/(4*linear_velocity))*RPM_ON_RADS;

            ROS_INFO("Estimated gear ratio: %f", gear_ratio);

            /*before committing the result we must filter, because there are cases in which the encoders tell that the robot is rotating on the spot
                but it is actually going straight. In these cases the term (m1_rpm + m2_rpm + m3_rpm + m4_rpm) will tend to 0 and so will the gear ratio
                so we must take only "valid" values, filtering the cases in which this happens
                On the other hand, there are cases in which the encoders tell that the robot is going straight but this is inconsistent with the linear velocity that is very small
                In this cases, with the divsion for such a small number the gear ratio value will explode with nonsense values.
                So, the idea is to constrain the gear ratio to have at least reasonable values
            */
            if(std::abs(gear_ratio) > 10 && std::abs(gear_ratio)< 60)
                return std::abs(gear_ratio);
            else{
                ROS_INFO("Unable to estimate a reasonable gear ratio, some inconsistency with sensor data");
                return -1;
            }
                
        }
        else{ //when the linear velocity of the robot is 0, we cannot estimate the gear ratio, because we would have 0 as denominator, so we return an error value
            ROS_INFO("Unable to estimate a reasonable gear ratio, the linear velocity is negligible");
            return -1;
        } 
    }

    /**
    function I created to estimate the ratio between the apparent baseline and the real one.
    The reasoning is quite similar to the one of the other function, and the motors angular velocities are already inverted as well.
    **/
    double estimate_apparent_baseline_ratio(double m1_rpm,double m2_rpm,double m3_rpm, double m4_rpm, double angular_velocity, double gear_ratio)
    {
        ROS_INFO("Estimating apparent baseline ratio with data: %f, %f, %f, %f, %f, %f", m1_rpm, m2_rpm, m3_rpm,  m4_rpm,  angular_velocity, gear_ratio);

        //we have to consider just relevant angular velocities, otherwise the apparent baseline will assume nonsense values
        if(std::abs(angular_velocity)>0.01 ){
            double ap_bl_ratio = (((m2_rpm + m4_rpm - m1_rpm - m3_rpm)*WHEEL_RAY*RPM_ON_RADS)/(2*angular_velocity*(gear_ratio)*ROBOT_BASELINE));
            ROS_INFO("Estimated apparent baseline ratio: %f,\tapparent baseline: %f m", ap_bl_ratio, ap_bl_ratio*ROBOT_BASELINE);
            
            /*
                Before committing the result we must filter, because there are cases in which the encoders tell that the robot is going straight (i.e.: same RPM for each wheel)
                but it is actually rotating. In these cases the term (m2_rpm + m4_rpm - m1_rpm - m3_rpm) will tend to 0 and so will the apparent baseline ratio
                So we must take only "valid" values
                In the same way, encoders that tell the robot is rotating on the spot while the computed angular velocity is quite negligible
                can lead to a ridiculously high apparent baseline ratios (this is in part filtered by the if(std::abs(angular_velocity)>0.01 ))
                Again, there are some cases in which the change of orientation in the data is affected by noise and the angular velocity is just not reasonable (e.g. > 500 deg/s)
                So, also in this case, the idea is to constrain the apparent baseline ratio to have at least reasonable values
            */
            if(std::abs(ap_bl_ratio) > 1 && std::abs(ap_bl_ratio) < 10) 
                return std::abs(ap_bl_ratio);
            else{
                ROS_INFO("Unable to estimate a reasonable apparent baseline ratio, some inconsistency with sensor data");
                return -1;
            }   
        }else{
            ROS_INFO("Unable to estimate a reasonable apparent baseline ratio, the angular velocity is negligible");
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
        if(false){//(DEBUG_MODE){
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
            //ROS_INFO ("Received four rpm messages: (%f,%f,%f,%f))",  rpm_fl_wheel,rpm_fr_wheel, rpm_rl_wheel,rpm_rr_wheel);
            //ROS_INFO ("Received five  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.nsec, msg_fr->header.stamp.nsec, msg_rl->header.stamp.nsec, msg_rr->header.stamp.nsec, msg_odom->header.stamp.nsec);
            ROS_INFO ("sec  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.sec, msg_fr->header.stamp.sec, msg_rl->header.stamp.sec, msg_rr->header.stamp.sec, msg_gt->header.stamp.sec);
            ROS_INFO ("nsec  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.nsec, msg_fr->header.stamp.nsec, msg_rl->header.stamp.nsec, msg_rr->header.stamp.nsec, msg_gt->header.stamp.nsec);
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
            ROS_INFO ("Received ground truth pose: x: %f, y: %f, theta: %f",  gt_pose_x, gt_pose_y, gt_pose_orientation);
        }
        
        current_time_sec = msg_fl->header.stamp.sec;
        current_time_nsec = msg_fl->header.stamp.nsec;
          
        if(previous_time_sec!= -1){

            //compute the time step
            if(current_time_sec == previous_time_sec)
                time_step = (current_time_nsec - previous_time_nsec)/1e+9;
            else
                time_step = (current_time_nsec - previous_time_nsec)/1e+9 + 1;
            
            if(DEBUG_MODE){
                ROS_INFO("current_time_sec %ld, previous_time_sec %ld", current_time_sec, previous_time_sec);
                ROS_INFO("current_time_nsec %ld, previous_time_nsec %ld, time step %f", current_time_nsec, previous_time_nsec, time_step);
            }
            
            //in order to estimate the gear ratio we need to know the linear and angular velocity in this step. 
            //the dumbest way is to just measure the traveled distance and the change in the orientation, and divide by the timestep to obtain the velocities
            /*
                NB: another possibility was to split the time step into 3 phases
                    //rotate on the spot to align the orientation towards the wanted position
                    //go straight
                    //rotate on the spot to aligh the robot in the wanted orientation
                this method was not so good by the way because it lead to very high angular velocities, while the "dumb" way brought at least some feasible values in general
            */

            double traveled_distance = sqrt(pow((gt_pose_y - y_position), 2) + pow((gt_pose_x - x_position), 2));
            double delta_theta = (gt_pose_orientation - orientation);

            //there are some points in which the orientation switches from (-PI + a small value) to (+PI - a small value)
            //so I decided to convert such angles in equivalent ones in the opposite direction, otherwise it would have lead to high angular velocities of the robot
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
                //ROS_INFO("unable to estimate gear ratio");
                gear_ratio = 40;
            }else{
                gear_ratio = estimated_gear_ratio;
                update_average_gear_ratio(gear_ratio);
            }
            ROS_INFO("Average gear ratio: %f\t# samples = %d", mean_gear_ratio, calibration_samples_gear_ratio);

            if(estimated_apparent_baseline_ratio == -1){
                //ROS_INFO("unable to estimate apparent baseline ratio");
                apparent_baseline_ratio = 2;

            }else{
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

        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
        
        transform.setOrigin( tf::Vector3(gt_pose_x, gt_pose_y, 0) );
        q.setRPY(0, 0, gt_pose_orientation);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gt_pose"));

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
