
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"

bool torso_left_arm          = false;  //size_group     7
bool torso_right_arm         = false;  //size_group     7
bool right_arm               = false;  //size_group     6
bool left_arm                = false;  //size_group     6
bool torso_head              = false;  //size_group     3
bool head                    = false;  //size_group     2
bool torso_tray              = false;  //size_group     2
bool left_hand               = false;  //size_group     1
bool right_hand              = false;  //size_group     1
bool tray                    = false;  //size_group     1

sensor_msgs::JointState::ConstPtr data_joint;

void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    data_joint = msg;

    int size_group = msg->name.size();

    switch(size_group)
    {
        case 1 :  if(msg->name[0].find("right_gripper_joint") == 0)      
                {
                    right_hand = true;
                    ROS_INFO("right_hand");
                }
                else if(msg->name[0].find("left_gripper_joint") == 0)
                     {
                        left_hand = true;
                        ROS_INFO("left_hand");
                     }
                     else 
                     {
                        tray = true;
                        ROS_INFO("tray");
                     }
                break;
                
    
        case 2 :  if(msg->name[0].find("torso_yaw_joint") == 0)
                {
                    torso_tray = true;
                    ROS_INFO("torso_tray");
                }
                else
                {
                    head = true;
                    ROS_INFO("head");
                }
                break;

        case 3 :  torso_head = true;
                ROS_INFO("torso_head");
                break;

        case 6 :  if(msg->name[0].find("left_arm_elbow_roll_joint") == 0)
                {
                    left_arm = true;
                    ROS_INFO("left_arm");
                }
                else
                {
                    right_arm = true;
                    ROS_INFO("right_arm");
                }
                break;
    
        case 7 :  if(msg->name[0].find("right_arm_elbow_roll_joint") == 0)
                {
                    torso_right_arm = true;
                    ROS_INFO("torso_right");
                }
                else
                {
                    torso_left_arm = true;
                    ROS_INFO("torso_left");
                }
                break;          
    }
}



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub_moveit = n.subscribe("/move_group/fake_controller_joint_states", 1000, chatterCallback);

  ros::spin();

  return 0;
}