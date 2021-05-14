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
/*  
    ROBOTICS 1st HOMEWORK 2020/2021 
    Student: Maitan Massimo - 10531426 - 944771 - Computer Science and Engineering, Politecnico di Milano
*/

/*
    Notes about the project: I decided to do all the stuff in a single node in order to reduce the complexity due to communication among nodes and synchronization of messages
    The drawback is that this main node has a lot of work to do, but as long as it is able to process the requests and do all the computation in an acceptable time slot 
    in my opinion it is a good tradeoff between code complexity, workload and testability
*/


#define WHEEL_RAY 0.1573      //ray of the wheels
#define RPM_ON_RADS 0.10472   //constant used to convert from RPM to Rad/s
#define ROBOT_BASELINE 0.583  //length of the (full) robot baseline

#define CALIBRATION_MODE 0 //if 1, the message filter callback will print information about the estimated parameters, such as the GEAR RATIO of the wheels and the APPARENT BASELINE for the skid steering
                            //if 0, the message filter callback will just compute the odometry
#define DEBUG_MODE 1 // if 1 the information useful for debugging purpose will be print on screen


//the integration method constant was used to test the program BEFORE the implementation of dynamic reconfigure for the integration method
//#define INTEGRATION_METHOD 0 // if 1 the node uses Euler's integration method, if 2 it uses RUNGE-KUTTA. If 0, uses both and compares them
class robot_node {

private:
    ros::NodeHandle n; 

    //here I create the message filters for each topic
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fr;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_fl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rl;
    message_filters::Subscriber<robotics_hw1::MotorSpeed>* sub_rr;
    message_filters::Subscriber<nav_msgs::Odometry>* sub_scout_odom;

    double parameter_x_initial_position;
    double parameter_y_initial_position;
    double parameter_initial_orientation;
    bool parameter_integration_method;

    //services

    ros::ServiceServer service1;
    ros::ServiceServer service2;

    //define the policy
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,  
                                                                robotics_hw1::MotorSpeed,
                                                                nav_msgs::Odometry> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy>* sync;

    double x_position;
    double y_position;
    double orientation;
    ros::Publisher pub_rechatter;  //to publish the messages received from the 4 wheel topics in the /rechatter topic (for debugging purposes)
    ros::Publisher pub_scout_odometry; //to publish the scout odometry messages in the /my_odom topic (for debugging purposes)
    ros::Publisher pub_my_odom_velocities;
    ros::Publisher pub_my_odom;
    ros::Publisher pub_my_odom_position_integration_m;
    ros::Timer timer1; //never say never
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

    
    dynamic_reconfigure::Server<robotics_odometry_project::parametersConfig> server;
    dynamic_reconfigure::Server<robotics_odometry_project::parametersConfig>::CallbackType f;

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
            ROS_INFO ("sec  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.sec, msg_fr->header.stamp.sec, msg_rl->header.stamp.sec, msg_rr->header.stamp.sec, msg_odom->header.stamp.sec);
            ROS_INFO ("nsec  messages: (%d,%d,%d,%d,%d)", msg_fl->header.stamp.nsec, msg_fr->header.stamp.nsec, msg_rl->header.stamp.nsec, msg_rr->header.stamp.nsec, msg_odom->header.stamp.nsec);
        }

        double scout_odom_x = msg_odom->pose.pose.position.x;
        double scout_odom_y = msg_odom->pose.pose.position.y;
        tf::Quaternion q(
                    msg_odom->pose.pose.orientation.x,
                    msg_odom->pose.pose.orientation.y,
                    msg_odom->pose.pose.orientation.z,
                    msg_odom->pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double scout_odom_orientation = yaw;

        double scout_odom_x_lin_vel = msg_odom->twist.twist.linear.x;
        double scout_odom_z_ang_vel = msg_odom->twist.twist.angular.z;
        

        double robot_angular_velocity = 0.0;
        double robot_linear_velocity = 0.0;

        double error = 0;
        double linear_velocity_error = 0;
        double angular_velocity_error = 0;

        if(DEBUG_MODE){
            ROS_INFO ("Received scout odometry position: %f,%f",  scout_odom_x, scout_odom_y);
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
            /*ROS_INFO ("Tracks ANGULAR velocities -> \tLeft: %f rad/s = %f deg/s,\tRight: %f rad/s = %f deg/s", left_track_rad_s, 
                                                                                                            left_track_rad_s*180/M_PI,
                                                                                                            right_track_rad_s, 
                                                                                                            right_track_rad_s*180/M_PI);
            ROS_INFO ("Tracks LINEAR velocities --> \tLeft: %f m/s,\tRight: %f m/s", left_track_lin_vel, right_track_lin_vel); */
            ROS_INFO ("ESTIMATED velocities ------> \tLinear: %f m/s,\tAngular: %f rad/s = %f deg/s", robot_linear_velocity, robot_angular_velocity, robot_angular_velocity*180/M_PI);                                                                                                                                                                                              
            ROS_INFO ("SCOUT odometry velocities--> \tLinear %f m/s,\tAngular %f rad/s", scout_odom_x_lin_vel, scout_odom_z_ang_vel);
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
                    ROS_INFO ("X position:\tEstimated: %f\tScout odometry: %f\tError: %f", x_position_euler, scout_odom_x, std::abs(x_position_euler-scout_odom_x));
                    ROS_INFO ("Y positison:\tEstimated: %f\tScout odometry: %f\tError: %f", y_position_euler, scout_odom_y, std::abs(y_position_euler-scout_odom_y));
                    ROS_INFO ("Orientation:\tEstimated: %f\tScout odometry: %f\tError: %f", orientation_euler, scout_odom_orientation, std::abs(orientation_euler-scout_odom_orientation));
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
                    ROS_INFO ("X position:\tEstimated: %f\tScout odometry: %f\tError: %f", x_position_rk, scout_odom_x, std::abs(x_position_rk-scout_odom_x));
                    ROS_INFO ("Y position:\tEstimated: %f\tScout odometry: %f\tError: %f", y_position_rk, scout_odom_y, std::abs(y_position_rk-scout_odom_y)); 
                    ROS_INFO ("Orientation:\tEstimated: %f\tScout odometry: %f\tError: %f", orientation_rk, scout_odom_orientation, std::abs(orientation_rk-scout_odom_orientation));
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
            transform.setOrigin( tf::Vector3(scout_odom_x, scout_odom_y, 0) );
            q.setRPY(0, 0, scout_odom_orientation);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "scout_odom"));
               
        }
        ROS_INFO ("-------------------------------------------------------------------------------------------------------------------------------------------------------------------------");

        previous_time_sec = current_time_sec;
        previous_time_nsec = current_time_nsec;
        
        /*pub_rechatter.publish(msg_fl);                -> for debugging purposes!
        pub_rechatter.publish(msg_fr);
        pub_rechatter.publish(msg_rl);
        pub_rechatter.publish(msg_rr);
        pub_scout_odometry.publish(msg_odom);*/

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

        pub_my_odom_position_integration_m.publish(customMessage);
    }

    void callback(robotics_odometry_project::parametersConfig &config, uint32_t level) {
            ROS_INFO("Reconfigure Request: %d", config.int_meth);
            parameter_integration_method = config.int_meth;
            ROS_INFO ("%d",level);
    }

    bool reset_odometry_origin(robotics_odometry_project::resetOdometry::Request  &req, robotics_odometry_project::resetOdometry::Response &res)
    {
        ROS_INFO("Resetting odometry to (0,0,0)");
        this->x_position = 0.0;
        this->y_position = 0.0;
        this->orientation = 0.0;
        return true;
    }

    bool reset_odometry_custom(robotics_odometry_project::resetOdometry::Request  &req, robotics_odometry_project::resetOdometry::Response &res)
    {
        
        this->x_position = req.x;
        this->y_position = req.y;
        this->orientation = req.w;
        ROS_INFO("Resetting odometry to (%f,%f,%f)", x_position, y_position, orientation);
        return true;
    }

public:
    robot_node(){


        ROS_INFO ("ROBOT MAIN NODE");
        /*
            This node publishes on 2 different topics, in particular

                /my_local_velocities: here I publish geometry_msgs::TwistStamped messages that contain information about the estimation of 
                                        local velocities of the robot in its local frame

                /custom_odometry: here I publish the custom messages with the information about the Odometry computed in this node and the integration method used        
        */
        pub_my_odom_velocities= n.advertise<geometry_msgs::TwistStamped >("/my_local_velocities", 1); 
        pub_my_odom_position_integration_m = n.advertise<robotics_odometry_project::integratedOdom>("/custom_odometry", 1);

        //in order to estimate the angular velocity of the wheel, the node must subscribe to the four motors topics 
        sub_fl = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_fl", 1);
        sub_fr = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_fr", 1);
        sub_rl = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_rl", 1);
        sub_rr = new message_filters::Subscriber<robotics_hw1::MotorSpeed>(n, "/motor_speed_rr", 1); 

        //in order to estimate the gear ratio and the apparent baseline, the node must subscribe to the constructor's odometry topic
        sub_scout_odom = new message_filters::Subscriber<nav_msgs::Odometry>(n, "/scout_odom", 1);

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

        
        //pub_rechatter = n.advertise<robotics_hw1::MotorSpeed >("/rechatter", 1); //---> used for debug!
        //pub_scout_odometry = n.advertise<nav_msgs::Odometry >("/rechat_scout_odom", 1);//---> used for debug!
        
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub_fl, *sub_fr, *sub_rl, *sub_rr, *sub_scout_odom);
        sync->registerCallback(boost::bind(&robot_node::message_filter_callback, this, _1, _2, _3, _4, _5));

        f = boost::bind(&robot_node::callback, this, _1, _2);
        server.setCallback(f);

        service1 = n.advertiseService("reset_odometry_origin", &robot_node::reset_odometry_origin, this);
        service2 = n.advertiseService("reset_odometry_custom", &robot_node::reset_odometry_custom, this);
    }    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_main_node");
    robot_node my_robot_node;
    ros::spin(); 
    return 0;
}
