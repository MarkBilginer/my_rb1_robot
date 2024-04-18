#include <my_rb1_ros/Rotate.h> // Include your custom service message
#include <nav_msgs/Odometry.h> // Include Odometry message
#include <ros/ros.h>
#include <tf/tf.h> // Include TF library for quaternion to RPY conversion

// Global variables to store the initial yaw and current yaw
double initial_yaw = 0.0;
double current_yaw = 0.0;
bool yaw_initialized = false; // To check if the initial yaw is set

ros::Publisher pub; // Declare publisher at a higher scope

// Callback function to update current_yaw based on Odometry data
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // Extract yaw from Quaternion orientation
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Update current_yaw
  current_yaw = yaw;
  if (!yaw_initialized) {
    initial_yaw = current_yaw;
    yaw_initialized = true;
    ROS_INFO("Initial Yaw set: %f", initial_yaw);
  }
}

// Callback function for rotate_robot service
bool rotateRobotCallback(my_rb1_ros::Rotate::Request &req,
                         my_rb1_ros::Rotate::Response &res) {
  if (!yaw_initialized) {
    res.result = "Error: Yaw not initialized.";
    ROS_ERROR("Yaw not initialized.");
    return false;
  }

  // Calculate the target yaw
  double target_yaw = initial_yaw + (req.degrees * M_PI / 180.0);

  // Publish command to rotate the robot
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z =
      (req.degrees > 0) ? -0.3 : 0.3; // Example angular velocity for rotation

  // Print debug info
  ROS_INFO("Initial Yaw: %f, Target Yaw: %f", initial_yaw, target_yaw);
  ROS_INFO("Publishing rotation command: angular velocity = %f",
           cmd_vel_msg.angular.z);

  pub.publish(cmd_vel_msg); // Publish initial rotation command
  ros::Rate loop_rate(20);  // Loop rate of 20 Hz

  // Wait until the robot reaches the target orientation
  bool rotation_completed = false; // Flag to track rotation completion
  while (ros::ok() && !rotation_completed) {
    ros::spinOnce(); // Allow callbacks to be called

    // Check if the robot's orientation is close to the target_yaw
    double tolerance = 0.05; // Tolerance for considering the rotation complete
    if (fabs(current_yaw - target_yaw) < tolerance) {
      cmd_vel_msg.angular.z = 0.0;
      pub.publish(cmd_vel_msg); // Publish rotation command to stop the rotation
      res.result = "Success: Rotation completed";
      rotation_completed = true; // Flag to track rotation completion
    } else {
      loop_rate.sleep();
    }
  }

  // If the rotation is not completed within the loop, update res.result
  // accordingly
  if (!rotation_completed) {
    res.result = "Error: Rotation failed or timed out";
    ROS_ERROR("Rotation failed or timed out");
  } else {
    ROS_INFO("Rotation completed successfully");
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");
  ros::NodeHandle nh;

  pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  // Subscribe to Odometry data to get current_yaw
  ros::Subscriber sub = nh.subscribe("/odom", 10, odomCallback);

  // Create a ROS service to rotate the robot
  ros::ServiceServer service =
      nh.advertiseService("/rotate_robot", rotateRobotCallback);

  ros::spin();

  return 0;
}
