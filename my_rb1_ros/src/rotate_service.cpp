#include <cmath>               // For fabs and other mathematical functions
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
    // ROS_INFO("Initial Yaw set: %f", initial_yaw);
  }
}

double normalizeAngle(double angle) {
  // Use atan2 to compute the angle in a circle and ensure it is between -π and
  // π
  return atan2(sin(angle), cos(angle));
}

// Callback function for rotate_robot service
bool rotateRobotCallback(my_rb1_ros::Rotate::Request &req,
                         my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("Service Requested: Rotate by %d degrees", req.degrees);

  if (!yaw_initialized) {
    res.result = "Error: Yaw not initialized.";
    ROS_ERROR("Yaw not initialized.");
    return false;
  }

  double degrees_rad = req.degrees * M_PI / 180.0; // Convert degrees to radians
  double target_yaw = normalizeAngle(
      current_yaw + degrees_rad); // Ensuring target is within -pi to pi

  double angular_velocity = 0.8; // Control rotation speed
  if (req.degrees > 0) {
    angular_velocity = -angular_velocity;
  }

  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z = angular_velocity;
  pub.publish(cmd_vel_msg);

  ros::Rate loop_rate(20);
  double accumulated_rotation = 0.0;
  double previous_yaw = current_yaw;

  while (ros::ok()) {
    ros::spinOnce();
    double delta_yaw = normalizeAngle(current_yaw - previous_yaw);
    accumulated_rotation += delta_yaw;

    if (fabs(accumulated_rotation) >=
        fabs(degrees_rad) - 0.05) { // Close to target, with tolerance
      cmd_vel_msg.angular.z = 0;    // Stop rotation
      pub.publish(cmd_vel_msg);
      res.result = "Success: Rotation completed";
      ROS_INFO("Rotation completed successfully.");
      return true;
    }

    previous_yaw = current_yaw;
    loop_rate.sleep();
  }

  res.result = "Error: Rotation failed or timed out";
  ROS_ERROR("Rotation failed or timed out");
  return false;
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

  ROS_INFO("Service Ready: /rotate_robot");

  ros::spin();

  return 0;
}
