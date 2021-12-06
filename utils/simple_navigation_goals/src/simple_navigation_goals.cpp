#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/Odometry.h"
#include "iostream"
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

//Function prototype
int send_nav_goal(float x, float y);
void odomcallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float x_pos = msg->pose.pose.position.x;
    float y_pos = msg->pose.pose.position.y;
    // send_nav_goal(x_pos + 1.0,y_pos + 0.0);
}

//Sends the Navigation goal
int send_nav_goal(std::vector<vector<float>> goal_positions)
{
    move_base_msgs::MoveBaseGoal goal;
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("Goal positions.size() %d",goal_positions.size());
    for(int i=0; i<goal_positions.size(); ++i){
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = goal_positions[i][0];
      goal.target_pose.pose.position.y = goal_positions[i][1];
      goal.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
      ac.waitForResult();
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Successful!!");
      else
        ROS_INFO("Failed");
    }

}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n("~");
  //Get goal positions
  
  std::vector<vector<float>> goal_positions;
  std::vector<float> pos_a;
  std::vector<float> pos_b;
  std::vector<float> pos_c;
  n.getParam("/goal_positions/a", pos_a);
  n.getParam("/goal_positions/b", pos_b);
  n.getParam("/goal_positions/c", pos_c);
  goal_positions.push_back(pos_a);
  goal_positions.push_back(pos_b);
  goal_positions.push_back(pos_c);
  // ros::Subscriber sub = n.subscribe("/odometry/filtered",100,odomcallback);
  send_nav_goal(goal_positions);
  return 0;
}
