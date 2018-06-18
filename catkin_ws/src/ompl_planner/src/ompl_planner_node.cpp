#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include"ompl_planner.h"

nav_msgs::Odometry start;
nav_msgs::Odometry goal;
bool start_called = false, goal_called = false;

void start_cb (const nav_msgs::OdometryConstPtr& input)
{
  if(!start_called)
  {
    start.pose.pose.position = input -> pose.pose.position;
    start.pose.pose.orientation = input -> pose.pose.orientation;
    start_called = true;
  }
}

void goal_cb (const nav_msgs::OdometryConstPtr& input)
{
  if(!goal_called)
  {
    goal.pose.pose.position = input -> pose.pose.position;
    goal.pose.pose.orientation = input -> pose.pose.orientation;
    goal_called = true;
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Subscriber start_sub = n.subscribe ("/planner/start", 1, start_cb);
  ros::Subscriber goal_sub = n.subscribe ("/planner/goal", 1, goal_cb);

  Planner ompl;

  ros::Rate r(1.0);
  while(n.ok()){

    ros::spinOnce();
    // check for incoming messages

    if(start_called && goal_called)
    {
      ompl.planWithSimpleSetup(start, goal);
      start_called = false;
      goal_called = false;
    }

    r.sleep();
  }
}
