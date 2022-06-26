#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  double target_goal_x = 0;
  double target_goal_y = 0;
  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  bool run = true;
  char your_choice = 'Y';
  while (run)
  {
    move_base_msgs::MoveBaseGoal goal;
    std::cout << "\nEnter the position that you want robot move" << std::endl;
    std::cin >> target_goal_x;
    std::cin >> target_goal_y;
    std::cout << "\nRobot will move to the target position x: " << target_goal_x <<" y: " << target_goal_y << std::endl;
    // we'll send a goal to the robot to move
    // goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = target_goal_x;
    goal.target_pose.pose.position.y = target_goal_y;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the basae moved");
    }
    else
    {
      ROS_INFO("The base failed to move for some reason");
    }
    do
    {
      std::cout << "\nWould you like to go to a new target? (Y/N)" << std::endl;
      std::cin >> your_choice;
      your_choice = tolower(your_choice);
    }
    while (your_choice != 'n' && your_choice != 'y');
    if (your_choice == 'n')
    {
      run = false;
    }
  }
  return 0;
}
