#include "ros/ros.h"
#include <iostream>
#include "actionlib/server/simple_action_server.h"
#include "ros_action_server/MyMsgAction.h"
#include "math.h"
#include <array>
#include <Eigen/Eigen>
#include <mutex>

/* ros msgs */
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "sensor_msgs/JointState.h"
#include <ros_action_server/tau.h>

/* Kinematics & Dynamics Library */
#include <kdl/jntarray.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/frames.hpp>

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

  std::vector<ros::Publisher> pose_multiple_pub;
  std::vector<ros::Publisher> data_multiple_pub;
  ros::Publisher pose_pub;
  ros::Publisher data_pub;

  KDL::Tree mytree; KDL::Chain mychain;
  KDL::JntArray jointPosCurrent, jointVelCurrent, jointEffort;
  Eigen::MatrixXf q_cur, qdot_cur;
  std::array<float, N_JOINT> k_p;
  std::array<float, N_JOINT> k_d;
  mutex m1;

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

  ~MoveRobotAction(void){} // delete

private:

  // provide your topic names

  std::string tauTopicNames[N_JOINT] = {
    "tau_1",
    "tau_2",
    "tau_3",
    "tau_4",
    "tau_5",
    "tau_6",
  };
  std::string dataTopicNames[12] = {
    "pose_error_1",
    "pose_error_2",
    "pose_error_3",
    "pose_error_4",
    "pose_error_5",
    "pose_error_6",
    "velo_error_1",
    "velo_error_2",
    "velo_error_3",
    "velo_error_4",
    "velo_error_5",
    "velo_error_6",
  };

  std::array<int, N_JOINT> map_joint_states;

  void initializeVariables(void)
  {
    jointPosCurrent.resize(N_JOINT), jointVelCurrent.resize(N_JOINT), jointEffort.resize(N_JOINT);
    q_cur.resize(N_JOINT,1); qdot_cur.resize(N_JOINT,1);
    map_joint_states={2, 1, 0, 3, 4, 5};      // to read joint_states in order
    k_p={16, 16, 16,  10,  10,  10};          // specify p and d gains
    k_d={5, 5, 5,  6,  6,  6};
    // BEGIN
    float mass = 5 ; double Ixx, Iyy, Izz; double l= 0.08, r = l/2.0; // from URDF

    Ixx = (1.0/12.0) * mass * ( 3*r*r + l*l);
    Iyy = Ixx;
    Izz =  (1.0/2.0) * mass * ( r*r );
    double I[6]={Ixx, Iyy, Izz, 0, 0, 0};   // Ixy = Ixz= Iyz = 0;
    double offset[6] = {0, 0, 0, 0, 0, 0} ; std::string tool_name = "new_tool";
    KDL::Vector r_cog(r, r, l/2.0); //! for a cylinder
    KDL::Joint fixed_joint = KDL::Joint(KDL::Joint::None);
    KDL::Frame tip_frame = KDL::Frame(KDL::Rotation::RPY(offset[0],offset[1],offset[2]),KDL::Vector(offset[3],offset[4],offset[5]));

    // rotational inertia in the cog
    KDL::RotationalInertia Inertia_cog = KDL::RotationalInertia(I[0], I[1], I[2], I[3], I[4], I[5]);
    KDL::RigidBodyInertia Inertia = KDL::RigidBodyInertia(mass, r_cog, Inertia_cog);
    KDL::Segment segment = KDL::Segment(tool_name, fixed_joint, tip_frame, Inertia);

    //parse kdl tree from Urdf
    if(!kdl_parser::treeFromFile("/home/mujib/test_ws/src/universal_robot/ur_description/urdf/model.urdf", mytree)){
      ROS_ERROR("[AS] Failed to construct kdl tree for elfin ! ");
    }

    if (!mytree.addSegment(segment, "wrist_3_link")) {  //! adding segment to the tree
      ROS_ERROR("[AS] Could not add segment to kdl tree");
    }

    if (!mytree.getChain("base_link", tool_name, mychain)){
      ROS_ERROR("[AS] Failed to construct kdl chain for elfin ! ");
    }
    // END

    unsigned int nj, ns; // resize variables using # of joints & segments
    nj =  mytree.getNrOfJoints(); ns = mychain.getNrOfSegments();
    if (ns == 0 || nj == 0){
      ROS_ERROR("[AS] Number of segments/joints are zero ! ");
    }
    if(jointPosCurrent.rows()!=nj || jointVelCurrent.rows()!=nj || jointEffort.rows() !=nj )
    {
      ROS_ERROR("[AS] ERROR in size of joint variables ! ");
    }
  }

  void initializeSubscribers(void)
  {
    pos_sub = nh.subscribe("/joint_states", 100, &MoveRobotAction::subscriberCb, this);  //TODO:  1
    ROS_INFO("[AS] Subscriber Initialized");
  }

  void initializePublishers(void)
  {
    arm_pub = nh.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 100);
    for (short int j = 0; j < N_JOINT; j++)
    {
      pose_pub = nh.advertise<std_msgs::Float64>(tauTopicNames[j], 1);
      pose_multiple_pub.push_back(pose_pub);
    }
    for (short int j = 0; j < 12; j++)
    {
      data_pub = nh.advertise<std_msgs::Float64>(dataTopicNames[j], 1);
      data_multiple_pub.push_back(data_pub);
    }
    ROS_INFO("[AS] Publisher Initialized");
  }

  void subscriberCb(const sensor_msgs::JointStateConstPtr &msg)
  {
    pos_info.position.resize(N_JOINT); pos_info.effort.resize(N_JOINT); //resize/clear velocity as well

    for(short int l=0; l< N_JOINT; l++){

      pos_info.position[l]= msg->position[map_joint_states[l]];  // joint_state
      pos_info.effort[l] = msg->effort[map_joint_states[l]];

      jointPosCurrent(l) = msg->position[map_joint_states[l]];
      jointVelCurrent(l) = msg->velocity[map_joint_states[l]];
      jointEffort(l) = msg->effort[map_joint_states[l]];
      q_cur(l,0) = msg->position[map_joint_states[l]];
      qdot_cur(l,0) = msg->velocity[map_joint_states[l]];

      //cout << "[AS] position: " <<  pos_info.position[l] << endl;
    }
    ROS_INFO("[AS] Reading joint_states");
  }

  // function to calculate the tau error and decides the feedback

  std_msgs::Float64MultiArray calError(sensor_msgs::JointState current, const std_msgs::Float64MultiArray arr)
  {
    std_msgs::Float64MultiArray err;
    err.data.resize(N_JOINT);

      for(short int l=0; l< N_JOINT; l++){    //msg2->goal[k].q.data[l]
        //err.data[l] = abs( goal->trajectory[k].angle_goal.data[l] )  - abs( current.effort[l] );
        err.data[l] = abs( arr.data[l] ) - abs( current.effort[l] ); // error = desired - current
      }

    return err;
  }


  //  main action server callback

  void actionCb(const ros_action_server::MyMsgGoalConstPtr &goal)
  {
    ros::Rate rate(50); //50
    bool success = true;
    ROS_INFO("[AS] executing the action call_back");

    KDL::JntArray C(N_JOINT), gravity(N_JOINT);
    Eigen::MatrixXf qdotdot_k(N_JOINT,1), c(N_JOINT,1), g(N_JOINT,1), M_(N_JOINT, N_JOINT), tau(N_JOINT,1);
    KDL::JntSpaceInertiaMatrix M(N_JOINT);
    KDL::ChainDynParam dyn_param(mychain, KDL::Vector(0, 0, -9.80665));
    std::vector<std_msgs::Float64> msg_(N_JOINT);
    std::vector<std_msgs::Float64> msg_error(12);
    Eigen::MatrixXf position_error(N_JOINT,1);  Eigen::MatrixXf velocity_error(N_JOINT,1);
    //std::vector<std_msgs::Float64> desired_tau(N_JOINT);
    std_msgs::Float64 a;
    std_msgs::Float64MultiArray q_des; q_des.data.resize(N_JOINT);
    std_msgs::Float64 b;
    std_msgs::Float64MultiArray qdot_des; qdot_des.data.resize(N_JOINT);
    std_msgs::Float64 v;
    std_msgs::Float64MultiArray qddot_des ; qddot_des.data.resize(N_JOINT);


    for(short int k = 0; k < goal->trajectory.size() ; k++){

      for (short int j = 0; j < N_JOINT; j++){
        a.data=static_cast<float>(goal->trajectory[k].angle_goal.data[j]);
        q_des.data[j] = a.data;

        b.data=static_cast<float>(goal->trajectory[k].vel_goal.data[j]);
        qdot_des.data[j] = b.data;

        v.data=static_cast<float>(goal->trajectory[k].acc_goal.data[j]);
        qddot_des.data[j] = v.data;
      }
      //m1.try_lock();//calculations see if Mutex Lock is required
      dyn_param.JntToMass(jointPosCurrent, M);
      dyn_param.JntToGravity(jointPosCurrent, gravity);
      dyn_param.JntToCoriolis(jointPosCurrent, jointVelCurrent, C);

      for(short int i = 0; i < N_JOINT; i++){

        c(i,0) = C(i);
        g(i,0) = gravity(i);
        qdotdot_k(i,0)=k_p[i]*(q_des.data[i]-q_cur(i,0))+k_d[i]*(qdot_des.data[i]-qdot_cur(i,0));

        position_error(i,0) = q_des.data[i] ;//-q_cur(i,0);
        velocity_error(i,0) = qdot_des.data[i];//-qdot_cur(i,0);

        for(int j = 0; j < N_JOINT; j++){
          M_(i,j) = M(i,j);}
      }

      tau = M_ * qdotdot_k + c + g;

      cout << "Applid tau: " << tau.transpose() << ", Current tau:" << jointEffort.data.transpose() << endl;

      for (short int j = 0; j < N_JOINT; j++)
      {
        msg_[j].data = tau(j,0);
        pose_multiple_pub[j].publish(msg_[j]);  // publish torques to plot
      }

      for (short int j = 0; j < N_JOINT; j++)
      {
        msg_error[j].data = position_error(j,0);
        msg_error[j+N_JOINT].data = velocity_error(j,0);
        data_multiple_pub[j].publish(msg_error[j]);                 // publish position error to plot
        data_multiple_pub[j+N_JOINT].publish(msg_error[j+N_JOINT]); // publish velocity error to plot
      }

      do
      {
        std_msgs::Float64MultiArray arr ; arr.data.resize(N_JOINT);
        for (short int j = 0; j < N_JOINT; j++){
          //cout << "[AS] recieved from client: " << arr.data[j] << endl;
          arr.data[j] = tau(j,0);
        }

        arm_pub.publish(arr); //goal->header.stamp;
        feedback.error.data.resize(N_JOINT);
        feedback.error = calError(pos_info, arr); // check if pos_info is updated
        action_server.publishFeedback(feedback);

        if( action_server.isPreemptRequested() || !ros::ok() ) // take care of preemption
        {

          for (short int j = 0; j < N_JOINT; j++){
            a.data=static_cast<float>(pos_info.effort[j]);   // send current jointEffort as goals
            arr.data[j] = a.data;
          }

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
      //m1.unlock();
    } //! for loop

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
  ros::Rate rate(50);     // 15,   10

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
