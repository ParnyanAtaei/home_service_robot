#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

double x_goal_1 = 6;
double y_goal_1 = 3;
double x_goal_2 = 0;
double y_goal_2 = 7;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n; 
  ros::spinOnce();

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);


  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pick_up_goal;


  // set up the frame parameters
  pick_up_goal.target_pose.header.frame_id = "map";
  pick_up_goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pick_up_goal.target_pose.pose.position.x = x_goal_1;
  pick_up_goal.target_pose.pose.position.y = y_goal_1;
  pick_up_goal.target_pose.pose.orientation.w = 1.0;



   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal( pick_up_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Hooray, the robot reach the Pick_up zone");
    ros::Duration(5.0).sleep();
    //ROS_INFO("start");

    MoveBaseClient ac1("move_base", true);

    move_base_msgs::MoveBaseGoal drop_off_goal; 

    drop_off_goal.target_pose.header.frame_id = "map";
    drop_off_goal.target_pose.header.stamp = ros::Time::now();

    drop_off_goal.target_pose.pose.position.x = x_goal_2;
    drop_off_goal.target_pose.pose.position.y = y_goal_2;
    drop_off_goal.target_pose.pose.orientation.w = 1.0;


    ROS_INFO("Sending goal");
    ac1.sendGoal(drop_off_goal);

    ac1.waitForResult();

    if(ac1.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the robot reach the Drop_off zone");
      ros::Duration(5.0).sleep();}

    else
      ROS_INFO("The robot failed to reach Drop_off zone for some reason");
  }
  

  else
    ROS_INFO("The robot failed to reach Pick_up zone for some reason");

  return 0;
}
