#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "turtlebot_test/NAV_GOAL.h"
#include <algorithm>

//math defines
#define PI 3.14159265358979323846
#define TWO_PI 6.28318530717958647693
#define HALF_PI 1.57079632679489661923

//bot defines
#define BURGER_MAX_LIN_VEL 0.22
#define BURGER_MAX_ANG_VEL 2.84

//control defines
#define KV 1.0;
#define KW 1.0;

//globals
nav_msgs::Odometry currentPose;
turtlebot_test::NAV_GOAL* current_goal = nullptr;
double CHASSIS_RADIUS_METERS = 0;
double WHEEL_RADIUS_METERS = 0;
ros::Publisher pub;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& input)
{
  memcpy(&currentPose, input.get(), sizeof(nav_msgs::Odometry));
  //ROS_INFO("Pose: [%f], [%f], [%f] | [%f], [%f], [%f], [%f]", currentPose.pose.pose.position.x, currentPose.pose.pose.position.y, currentPose.pose.pose.position.z, currentPose.pose.pose.orientation.x, currentPose.pose.pose.orientation.y, currentPose.pose.pose.orientation.z, currentPose.pose.pose.orientation.w);
}
double makeSimpleProfile(double output, double input, double slop){
  if (input>output){
    output = std::min(input, output+slop);
  } else if(input<output){
    output = std::max(input, output-slop);
  } else {
    output = input;
  }
  return output;
}
double normalRelativeAngle(double angle) {
    angle = fmod(angle, TWO_PI);
		return angle >= 0 ? (angle < PI) ? angle : angle - TWO_PI : (angle >= -PI) ? angle : angle + TWO_PI;
}
double getRotationToPoint(double current_theta, double current_x, double current_y, double target_x, double target_y)
{
  //translate to origin
  double x_diff = target_x - current_x;
  double y_diff = target_y - current_y;

  double theta_to_target = atan2(y_diff, x_diff); // returns in range (-pi/2, pi/2)
  double turn = normalRelativeAngle(theta_to_target-current_theta);
  return turn;
}
void driveToPoint(double target_x, double target_y)
{
    // update pose
    double x = currentPose.pose.pose.position.x;
    double y = currentPose.pose.pose.position.y;
    tf::Quaternion q(currentPose.pose.pose.orientation.x, currentPose.pose.pose.orientation.y, currentPose.pose.pose.orientation.z, currentPose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // calculate w and v, swapping directions if necessary to allow backwards pathing
    double w = getRotationToPoint(yaw, x, y, target_x, target_y) * KW;
    bool setBackAsFront = false;
    if(w<-HALF_PI){
      w=PI+w;
      setBackAsFront = true;
    } else if(w>HALF_PI){
      w=-PI+w;
      setBackAsFront = true;
    }
    if (w > BURGER_MAX_ANG_VEL)
    {
        w = BURGER_MAX_ANG_VEL;
    }
    double dist = sqrt(pow(x - target_x, 2) + pow(y - target_y, 2));
    double v = dist * KV;
    if (v > BURGER_MAX_LIN_VEL)
    {
        v = BURGER_MAX_LIN_VEL;
    }
    if(setBackAsFront){
      v=-v;
    }
    geometry_msgs::Twist twist = {};
    twist.linear.x = v;
    twist.angular.z = w;
    pub.publish(twist);
}
void navgoalCallback(const turtlebot_test::NAV_GOAL::ConstPtr& input){
  if(current_goal!=nullptr){
    free(current_goal);
  }
  current_goal = (turtlebot_test::NAV_GOAL*)malloc(sizeof(turtlebot_test::NAV_GOAL));
  memcpy(current_goal, input.get(), sizeof(turtlebot_test::NAV_GOAL));
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
  ros::Subscriber odometrysub = n.subscribe("/odom", 1000, odometryCallback);
  ros::Subscriber navgoalsub = n.subscribe("NAV_GOAL", 1, navgoalCallback);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  //get chassis radius parameter
  if (n.getParam("/CHASSIS_RADIUS", CHASSIS_RADIUS_METERS))
  {
    CHASSIS_RADIUS_METERS/=1000;
    ROS_INFO("Got chassis radius");
  }
  else{
    ROS_FATAL("Chassis radius parameter missing!");
    exit(1);
  }
  if (n.getParam("/WHEEL_RADIUS", WHEEL_RADIUS_METERS))
  {
    WHEEL_RADIUS_METERS/=1000;
    ROS_INFO("Got wheel radius");
  }
  else{
    ROS_FATAL("Wheel radius parameter missing!");
    exit(1);
  }

  currentPose = {};

  //set update rate to 10Hz
  ros::Rate loop_rate(10);
  while(ros::ok()){
    if(current_goal != nullptr){
      driveToPoint(current_goal->x, current_goal->y);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  free(current_goal);
  return 0;
}
