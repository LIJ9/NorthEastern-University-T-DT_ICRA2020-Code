/****************************************************************************
 *@brief		
 *@version	1.0.0.0
 *@author		Way
 * 
 *@Change History:
 * -------------------------------------------------------------------------
 * <Date>	      |<version>	   |<Author>	|<Description>
 * -------------------------------------------------------------------------
 * 2019/10/22   |1.0.0.1       |Way		    |2019ICRA
 *-------------------------------------------------------------------------- 
 * 2020/08/24   |1.0.0.2       |Way		    |最终格式规范
 *--------------------------------------------------------------------------
 ***************************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "roborts_msgs/ArmorDetectionAction.h"
#include <actionlib/client/terminal_state.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "armor_detection_node_test_client");

  // create the action client
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> ac("armor_detection_node_action", true);
  //roborts_msgs::ArmorDetectionResult node_result;
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();
  ROS_INFO("Start.");
  roborts_msgs::ArmorDetectionGoal goal;

  char command = '0';
  
 // command = '1';

  while (command != '4') {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: start the action" << std::endl
              << "2: pause the action" << std::endl
              << "3: stop  the action" << std::endl
              << "4: exit the program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4') {
      std::cout << "please inpugain!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

    switch (command) {
      //start thread.
      case '1':
        goal.command = 1;
        ROS_INFO("I am running the request");
        ac.sendGoal(goal);
        break;
        //pause thread.
      case '2':
        goal.command = 2;
        ROS_INFO("Action server will pause.");
        ac.sendGoal(goal);
        //stop thread.
      case '3':
        goal.command = 3;
        ROS_INFO("I am cancelling the request");
        ac.cancelGoal();
        break;
      default:
        break;
    }
  }
  return 0;
}
