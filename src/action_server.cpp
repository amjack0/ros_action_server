#include "ros/ros.h"
#include <iostream>
#include "actionlib/server/simple_action_server.h"
#include "ros_action_server/MyMsgAction.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include "math.h"
#include <array>
#include <Eigen/Eigen>
#include <kdl/jntarray.hpp>

using namespace std;
#define N_JOINT 6


class MoveRobotAction{

protected:
  ros::NodeHandle nh;
  actionlib::SimpleActionServer<ros_action_server::MyMsgAction> action_server;
  std::string action_name;                    // name of my action server
  ros_action_server::MyMsgFeedback feedback; // variables stores the feedback
  ros_action_server::MyMsgResult result;    // variables stores the result

  //defining my subscribers

  ros::Subscriber pos_sub;
  sensor_msgs::JointState pos_info;

  //defining my publishers

  ros::Publisher tau_pub;
  ros::Publisher arm_pub;
  std::vector<ros::Publisher> tau_multiple_pub;


public:
  //constructor
  MoveRobotAction(std::string name) :
    action_server(nh, name, boost::bind(&MoveRobotAction::actionCb, this, _1),false),
    action_name(name)
  {
   initializeVariables();
   initializeSubscribers();
   initializePublishers();
   action_server.start();
  }

  ~MoveRobotAction(void){} // TODO: delete

private:

  // provide your controller topic names
  std::string tauTopicNames[N_JOINT] = { //  this controller is not in use here
    "/urbot/endeffector_frc_trq_controller_1/command",
    "/urbot/endeffector_frc_trq_controller_2/command",
    "/urbot/endeffector_frc_trq_controller_3/command",
    "/urbot/endeffector_frc_trq_controller_4/command",
    "/urbot/endeffector_frc_trq_controller_5/command",
    "/urbot/endeffector_frc_trq_controller_6/command",
  };

  std::array<int, N_JOINT> map_joint_states;

  void initializeVariables(void)
  {
    map_joint_states={2, 1, 0, 3, 4, 5}; // to read joint_states in order
  }

  // initialize subscriber
  void initializeSubscribers(void)
  {
    pos_sub = nh.subscribe("/joint_states", 100, &MoveRobotAction::subscriberCb, this);  //TODO:  1
    ROS_INFO("[AS] Subscriber Initialized");
  }

  // initialize publisher

  void initializePublishers(void)
  {
    //ros::Duration(30.0).sleep(); // Debug
    arm_pub = nh.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 100);

    /*for (short int j = 0; j < N_JOINT; j++)
    {
      tau_pub = nh.advertise<std_msgs::Float64>(tauTopicNames[j], 1); //TODO: 1
      tau_multiple_pub.push_back(tau_pub);
    }*/
    ROS_INFO("[AS] Publisher Initialized");
  }

  void subscriberCb(const sensor_msgs::JointStateConstPtr &msg)
  {
    //pos_info.position.clear(); pos_info.effort.clear();
    pos_info.position.resize(N_JOINT); pos_info.effort.resize(N_JOINT); //resize/clear velocity as well

    for(short int l=0; l< N_JOINT; l++){

      pos_info.position[l]= msg->position[map_joint_states[l]];
      pos_info.effort[l] = msg->effort[map_joint_states[l]];
      //cout << "[AS] position: " <<  pos_info.position[l] << endl;
    }
    ROS_INFO("[AS] Reading joint_states");
  }


  // function to calculate the tau error and decides the feedback

  std_msgs::Float64MultiArray calError(sensor_msgs::JointState current, const ros_action_server::MyMsgGoalConstPtr &goal)
  {
    std_msgs::Float64MultiArray err;
    //err.data.clear();
    err.data.resize(N_JOINT); // clear here is not required, reset the error

    for(short int l=0; l< N_JOINT; l++){
      err.data[l] = abs(goal->tau.data[l])  - abs(current.effort[l]) ; // error = desired - current
    }
    //  TODO: check the difference
    return err;
  }


  //  main action server callback

  void actionCb(const ros_action_server::MyMsgGoalConstPtr &goal)
  {
    ros::Rate rate(50); //50
    bool success = true;
    ROS_INFO("[AS] executing the action call_back");

    //std::vector<std_msgs::Float64> desired_tau(N_JOINT);
    std_msgs::Float64 a;
    std_msgs::Float64MultiArray arr; arr.data.resize(N_JOINT);

    for (short int j = 0; j < N_JOINT; j++){
      a.data=static_cast<float>(goal->tau.data[j]); // TODO: check
      //desired_tau[j].data = a.data;
      //cout << "desired_tau: " << desired_tau[j].data << endl;
      arr.data[j] = a.data; // check
    }

    do
    {

      /*for (short int j = 0; j < N_JOINT; j++)
      {
        tau_multiple_pub[j].publish(desired_tau[j]);
        arr.data[j] = desired_tau[j].data;  // store , arr = desired tau
      }*/

      for (short int j = 0; j < N_JOINT; j++){
        cout << "[AS] recieved from client: " << arr.data[j] << endl;
      }

      //goal->header.stamp;
      arm_pub.publish(arr);
      //feedback.error.data.clear();
      feedback.error.data.resize(N_JOINT);
      feedback.error = calError(pos_info, goal);
      action_server.publishFeedback(feedback);

      // take care of preemption

      if( action_server.isPreemptRequested() || !ros::ok() )
      {

        for (short int j = 0; j < N_JOINT; j++){
          a.data=static_cast<float>(pos_info.effort[j]);   // TODO: send current jointEffort as goals
          //desired_tau[j].data = a.data;
          arr.data[j] = a.data;
        }

//        for (short int j = 0; j < N_JOINT; j++)
//        {
//          tau_multiple_pub[j].publish(desired_tau[j]); bulli bulli gani ki

//          arr.data[j] = desired_tau[j].data;
//        }

        arm_pub.publish(arr);
        ROS_INFO("########## [AS] %s: Preempted", action_name.c_str());
        action_server.setPreempted();
        success = false;
        break;
      }

      rate.sleep();
    }

    // TODO: set the goal tolerance
    while ( abs(feedback.error.data[0]) > 40 || abs(feedback.error.data[1]) > 40 || abs(feedback.error.data[2]) > 40 || abs(feedback.error.data[3]) > 40 || abs(feedback.error.data[4]) > 40 || abs(feedback.error.data[5]) > 40 );

    // check if succeeded--yes-->return result
    if(success)
    {
      result.status = "FINISHED";
      ROS_INFO("[AS] %s: Succeeded", action_name.c_str());
      action_server.setSucceeded(result); //! set the action state to succeeded
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");
  MoveRobotAction robot("trajectory_action");
  ros::Rate rate(15);     // 10

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
