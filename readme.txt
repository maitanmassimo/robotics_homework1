10531426 - Massimo Maitan


FILES
The project is composed by the following files

/src
   /robot_node.cpp     		-> contains the code of the main project
   /gt_calibration_node.cpp	-> contains the code for the optional calibration feature


/srv/resetOdometry.srv		-> contains the format of the request for the (custom) odometry reset service

/cfg/parameters.cfg		-> configuration for dynamic reconfigure

/msg/integratedOdom.msg		-> contains the custom message definition (Odometry + integration method)

/launch/odom_proj.launch	-> starts the robot main node and the ground truth calibration node, setting a static transformation between world and odom and some parameters
					->NB: in the launch file there is a parameter also to set the configuration mode for the main node: if 1, this node just estimates gear ratio 
					apparent baseline, otherwise it computes and publishes the odometry


PARAMETERS

double parameter_x_initial_position -> initial x position of the robot in odom frame
double parameter_y_initial_position;-> initial y position of the robot in odom frame
double parameter_initial_orientation; -> initial orientation of the robot in odom frame
bool parameter_integration_method;   -> integration method, can be changed with dynamic reconfigure
bool parameter_calibration_mode; -> explained above


TF TREE: the frames hierarchy is

/world
	/odom: defined through a static transformation from /world (for the setup I just watched the initial gt_pose of the robot in world's frame)		
		/scout_odom: dynamic transformation from /odom, has the pose of the scout_odom position 
		/custom_odom: dynamic transformation from /odom, has the pose of my odometry

	/gt_pose: dynamic transformation from /world, assumes the pose of the ground truth pose estimation made with OptiTrack System


CUSTOM MESSAGE: the custom message robotics_odometry_project/integratedOdom  has the following structure

nav_msgs/Odometry odom			-> odometry message, has an header, pose (in odom reference frame) and linear/angular velocity of the robot (expressed in a local reference frame)
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
    float64[36] covariance
  geometry_msgs/TwistWithCovariance twist
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
    float64[36] covariance


std_msgs/String method    		-> string that identifies the integration method (Euler or rk = Runge Kutta)
  string data


TEST INSTRUCTIONS

	1) The launch file provided already starts both the nodes. Two empty windows will appear, and they will start printing information as soon as the bag starts publishing data
	2) The services can be invoked with the rosservice command
	3) The messages are published on:

            /my_local_velocities: here I publish geometry_msgs::TwistStamped messages that contain information about the estimation of 
                                    local velocities of the robot in its local frame

            /custom_odometry: here I publish the custom messages with the information about the Odometry computed in this node and the integration method used  


	4) The pose parameters are set to (0,0,0), which is the initial pose for bag1 file. I also tested the project with the other bags exploiting the custom odomety reset service


