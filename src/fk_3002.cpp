#include "ros/ros.h"
#include "std_msgs/String.h"
#include "turtlebot_test/FK_INPUT.h"
#include "turtlebot_test/FK_OUTPUT.h"
ros::Publisher pub;
double CHASSIS_RADIUS = 0;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void fkCallback(const turtlebot_test::FK_INPUT::ConstPtr& input)
{
  turtlebot_test::FK_OUTPUT output;
      if (input->vl == input->vr)
      {
        output.x = input->start_x + input->vr * cos(input->start_theta) * input->delta_t;
        output.y = input->start_y + input->vr * sin(input->start_theta) * input->delta_t;
        output.theta = input->start_theta;
      }
      else
      {
        // radius to ICC
        double R = CHASSIS_RADIUS * ((input->vl + input->vr) / (input->vr - input->vl));
        // angular velocity
        double w = (input->vr - input->vl) / (2 * CHASSIS_RADIUS);
        // ICC
        double ICCx = input->start_x - (R * sin(input->start_theta));
        double ICCy = input->start_y + (R * cos(input->start_theta));
        // FK
        output.x = ICCx - cos(input->delta_t * w) * (ICCx - input->start_x) + sin(input->delta_t * w) * (ICCy - input->start_y);
        output.y = ICCy - cos(input->delta_t * w) * (ICCy - input->start_y) - sin(input->delta_t * w) * (ICCx - input->start_x);
        output.theta = input->start_theta + input->delta_t * w;
      }
  pub.publish(output);
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "fk_3002");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //get chassis radius parameter
  if (n.getParam("/CHASSIS_RADIUS", CHASSIS_RADIUS))
  {
    ROS_INFO("Got chassis radius");
  }
  else{
    ROS_FATAL("Chassis radius parameter missing!");
    exit(1);
  }

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("FK_INPUT", 1000, fkCallback);

  pub = n.advertise<turtlebot_test::FK_OUTPUT>("FK_OUTPUT", 1);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
