#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <signal.h>
using namespace std;

#define PI 3.1415926

bool getRobotPose(tf::Stamped<tf::Pose>& robot_pose_odom, tf::TransformListener& tfListener);

tf::Stamped<tf::Pose> g_goal;
string g_name_space;
ros::Publisher g_twist_pub;

//odom based
void getGoal(const geometry_msgs::PoseStamped& goal_pose)
{
  ROS_INFO("Received goal.");
  tf::poseStampedMsgToTF(goal_pose, g_goal);
}

void shutdown(int sig)
{
  ros::Duration(1).sleep(); // sleep for one second
  ROS_INFO("location test node ended!");
  geometry_msgs::Twist twist;
  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
  twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
  g_twist_pub.publish(twist);
  ros::shutdown();
}

//need to realize use lider Lyapunov tf location
//costmap
int main(int argc, char **argv)
{
  ros::init(argc, argv, "location_test");
  ros::NodeHandle nh;
  g_name_space = nh.getNamespace();
  signal(SIGINT, shutdown);
  tf::TransformListener tfListener(ros::Duration(10));
  double k1 = 0.2, k2 = 1;

  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  g_twist_pub = twist_pub;
  ros::Subscriber get_goal_sub = nh.subscribe(g_name_space + "/goal", 10, &getGoal);
  geometry_msgs::Twist twist;
  tf::Stamped<tf::Pose> robot_pose_odom;
  double control_linear_vel, control_angular_vel;

  ros::Rate loop_rate(0.5);
  while (ros::ok())
  {
    getRobotPose(robot_pose_odom, tfListener);
    geometry_msgs::PoseStamped msg;
    tf::poseStampedTFToMsg(robot_pose_odom, msg);
    double yaw = tf::getYaw(msg.pose.orientation);
    tf::Vector3 goal_v = g_goal.getOrigin();
    tf::Vector3 robot_v = robot_pose_odom.getOrigin();
    double gx = goal_v.getX();
    double gy = goal_v.getY();
    double rx = robot_v.getX();
    double ry = robot_v.getY();
    double ux = cos(yaw);
    double uy = sin(yaw);
    Eigen::Matrix<double, 2,1> P;
    Eigen::Matrix<double, 2,1> U;
    Eigen::Matrix<double, 2,2> J;
    P << rx, ry;
    U << ux, uy;
    J << cos(PI/2), -sin(PI/2), sin(PI/2), cos(PI/2);
    ROS_INFO("gx:%f gy:%f rx:%f ry:%f", gx, gy, rx, ry);
    ROS_INFO("robot_yaw: %f, ux:%f, uy:%f", yaw, ux, uy);
    control_linear_vel = - k1 * P.transpose() * U;
    control_angular_vel = - k2 * P.transpose() * J * U;
    ROS_INFO("control_linear_vel:%f", control_linear_vel);
    ROS_INFO("control_angular_vel:%f", control_angular_vel);
    twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel;
    twist_pub.publish(twist);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

bool getRobotPose(tf::Stamped<tf::Pose>& robot_pose_odom, tf::TransformListener& tfListener)
{
  std::string odom_frame = g_name_space.substr(1, g_name_space.length() - 1) + "_odom";
  std::string base_frame;
  base_frame = g_name_space.substr(1, g_name_space.length() - 1) + "_base_footprint";
  robot_pose_odom.setIdentity();
  tf::Stamped < tf::Pose > robot_pose;
  robot_pose.setIdentity();
  robot_pose.frame_id_ = base_frame;
  robot_pose.stamp_ = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    tfListener.transformPose(odom_frame, robot_pose, robot_pose_odom);
  }
  catch (tf::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::InvalidArgumentException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "InvalidArgumentException: %s\n", ex.what());
    return false;
  }

  double transform_tolerance = 0.1;
  // check robot_pose_odom timeout
  if (current_time.toSec() - robot_pose_odom.stamp_.toSec() > transform_tolerance)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Costmap2DROS transform timeout. Current time: %.4f, robot_pose_odom stamp: %.4f, tolerance: %.4f",
                      current_time.toSec(), robot_pose_odom.stamp_.toSec(), transform_tolerance);
    return false;
  }
//  ROS_INFO("robot_pose_odom.frame_id_: %s", robot_pose_odom.frame_id_.c_str());
  return true;
}
