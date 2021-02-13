
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"

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
bool torso                   = false; 

sensor_msgs::JointState::ConstPtr data_joint;

std_msgs::Float64   left_arm_elbow_roll, 
                    left_arm_forearm_yaw,
                    left_arm_shoulder_pitch,
                    left_arm_shoulder_roll,
                    left_arm_shoulder_yaw, 
                    left_arm_wrist_roll,
                    left_gripper,

                    right_arm_elbow_roll, 
                    right_arm_forearm_yaw,
                    right_arm_shoulder_pitch,
                    right_arm_shoulder_roll,
                    right_arm_shoulder_yaw, 
                    right_arm_wrist_roll,
                    right_gripper,

                    head_pitch,
                    head_yaw,

                    torso_yaw,

                    tray_pitch;

void move_group_identify(const sensor_msgs::JointState::ConstPtr& msg)
{
    data_joint = msg;

    int size_group = msg->name.size();

    switch(size_group)
    {
        case 1 :  if(msg->name[0].find("right_gripper_joint") == 0)      
                {
                    right_hand = true;
                    //ROS_INFO("right_hand");
                }
                else if(msg->name[0].find("left_gripper_joint") == 0)
                     {
                        left_hand = true;
                        //ROS_INFO("left_hand");
                     }
                     else if(msg->name[0].find("tray_pitch_joint") == 0) 
                            {
                                tray = true;
                                //ROS_INFO("tray");
                            }else
                            {
                                torso = true;
                                ROS_INFO("TORSO");
                            }
                break;
                
    
        case 2 :  if(msg->name[0].find("torso_yaw_joint") == 0)
                {
                    torso_tray = true;
                    //ROS_INFO("torso_tray");
                }
                else
                {
                    head = true;
                    //ROS_INFO("head");
                }
                break;

        case 3 :  torso_head = true;
               // ROS_INFO("torso_head");
                break;

        case 6 :  if(msg->name[0].find("left_arm_elbow_roll_joint") == 0)
                {
                    left_arm = true;
                    //ROS_INFO("left_arm");
                }
                else
                {
                    right_arm = true;
                    //ROS_INFO("right_arm");
                }
                break;
    
        case 7 :  if(msg->name[0].find("right_arm_elbow_roll_joint") == 0)
                {
                    torso_right_arm = true;
                    //ROS_INFO("torso_right");
                }
                else
                {
                    torso_left_arm = true;
                    //ROS_INFO("torso_left");
                }
                break;          
    }
}


int main(int argc, char **argv)
{
     
  ros::init(argc, argv, "Sub_for_dyna");
  
  ros::NodeHandle nh;
  
  ros::Subscriber sub_moveit = nh.subscribe("/move_group/fake_controller_joint_states", 1000, move_group_identify);

  ros::Publisher left_arm_shoulder_pitch_joint  = nh.advertise<std_msgs::Float64>("left_arm_shoulder_pitch_joint_controller/command", 1000);
  ros::Publisher left_arm_shoulder_roll_joint   = nh.advertise<std_msgs::Float64>("left_arm_shoulder_roll_joint_controller/command", 1000);
  ros::Publisher left_arm_shoulder_yaw_joint    = nh.advertise<std_msgs::Float64>("left_arm_shoulder_yaw_joint_controller/command", 1000);
  ros::Publisher left_arm_elbow_roll_joint      = nh.advertise<std_msgs::Float64>("left_arm_elbow_roll_joint_controller/command", 1000);
  ros::Publisher left_arm_forearm_yaw_joint     = nh.advertise<std_msgs::Float64>("left_arm_forearm_yaw_joint_controller/command", 1000);
  ros::Publisher left_arm_wrist_roll_joint      = nh.advertise<std_msgs::Float64>("left_arm_wrist_roll_joint_controller/command", 1000);
  ros::Publisher left_gripper_joint             = nh.advertise<std_msgs::Float64>("left_gripper_joint_controller/command", 1000);

  ros::Publisher right_arm_shoulder_pitch_joint = nh.advertise<std_msgs::Float64>("right_arm_shoulder_pitch_joint_controller/command", 1000);
  ros::Publisher right_arm_shoulder_roll_joint  = nh.advertise<std_msgs::Float64>("right_arm_shoulder_roll_joint_controller/command", 1000);
  ros::Publisher right_arm_shoulder_yaw_joint   = nh.advertise<std_msgs::Float64>("right_arm_shoulder_yaw_joint_controller/command", 1000);
  ros::Publisher right_arm_elbow_roll_joint     = nh.advertise<std_msgs::Float64>("right_arm_elbow_roll_joint_controller/command", 1000);
  ros::Publisher right_arm_forearm_yaw_joint    = nh.advertise<std_msgs::Float64>("right_arm_forearm_yaw_joint_controller/command", 1000);
  ros::Publisher right_arm_wrist_roll_joint     = nh.advertise<std_msgs::Float64>("right_arm_wrist_roll_joint_controller/command", 1000);
  ros::Publisher right_gripper_joint            = nh.advertise<std_msgs::Float64>("right_gripper_joint_controller/command", 1000);

  ros::Publisher torso_yaw_joint                = nh.advertise<std_msgs::Float64>("torso_yaw_joint_controller/command", 1000);
  ros::Publisher tray_pitch_joint               = nh.advertise<std_msgs::Float64>("tray_pitch_joint_controller/command", 1000);

  ros::Publisher head_yaw_joint                 = nh.advertise<std_msgs::Float64>("head_yaw_joint_controller/command", 1000);
  ros::Publisher head_pitch_joint               = nh.advertise<std_msgs::Float64>("head_pitch_joint_controller/command", 1000);

  ros::Rate loop_rate(50);

  while (ros::ok()) 
   {
     if(left_arm)
     {
        left_arm_elbow_roll.data        = data_joint->position[0];
        left_arm_forearm_yaw.data       = data_joint->position[1];
        left_arm_shoulder_pitch.data    = data_joint->position[2];
        left_arm_shoulder_roll.data     = data_joint->position[3];
        left_arm_shoulder_yaw.data      = data_joint->position[4];
        left_arm_wrist_roll.data        = data_joint->position[5];

        left_arm_shoulder_pitch_joint.publish(left_arm_shoulder_pitch);
        left_arm_shoulder_roll_joint.publish(left_arm_shoulder_roll);
        left_arm_shoulder_yaw_joint.publish(left_arm_shoulder_yaw);
	    left_arm_elbow_roll_joint.publish(left_arm_elbow_roll);
        left_arm_forearm_yaw_joint.publish(left_arm_forearm_yaw);
        left_arm_wrist_roll_joint.publish(left_arm_wrist_roll);

        ROS_INFO("LEFT_ARM OK");
     }else if(right_arm)
            {
                right_arm_elbow_roll.data        = data_joint->position[0];
                right_arm_forearm_yaw.data       = data_joint->position[1];
                right_arm_shoulder_pitch.data    = data_joint->position[2];
                right_arm_shoulder_roll.data     = data_joint->position[3];
                right_arm_shoulder_yaw.data      = data_joint->position[4];
                right_arm_wrist_roll.data        = data_joint->position[5];

                right_arm_shoulder_pitch_joint.publish(right_arm_shoulder_pitch);
                right_arm_shoulder_roll_joint.publish(right_arm_shoulder_roll);
                right_arm_shoulder_yaw_joint.publish(right_arm_shoulder_yaw);
                right_arm_elbow_roll_joint.publish(right_arm_elbow_roll);
                right_arm_forearm_yaw_joint.publish(right_arm_forearm_yaw);
                right_arm_wrist_roll_joint.publish(right_arm_wrist_roll);

                ROS_INFO("RIGHT_ARM OK");
            }else if(torso_left_arm)
                    {
                        left_arm_elbow_roll.data        = data_joint->position[0];
                        left_arm_forearm_yaw.data       = data_joint->position[1];
                        left_arm_shoulder_pitch.data    = data_joint->position[2];
                        left_arm_shoulder_roll.data     = data_joint->position[3];
                        left_arm_shoulder_yaw.data      = data_joint->position[4];
                        left_arm_wrist_roll.data        = data_joint->position[5];
                        torso_yaw.data                  = data_joint->position[6];

                        left_arm_shoulder_pitch_joint.publish(left_arm_shoulder_pitch);
                        left_arm_shoulder_roll_joint.publish(left_arm_shoulder_roll);
                        left_arm_shoulder_yaw_joint.publish(left_arm_shoulder_yaw);
                        left_arm_elbow_roll_joint.publish(left_arm_elbow_roll);
                        left_arm_forearm_yaw_joint.publish(left_arm_forearm_yaw);
                        left_arm_wrist_roll_joint.publish(left_arm_wrist_roll);
                        torso_yaw_joint.publish(torso_yaw);

                        ROS_INFO("TORSO_LEFT_ARM OK");
                    }else if(torso_right_arm)
                            {
                                right_arm_elbow_roll.data        = data_joint->position[0];
                                right_arm_forearm_yaw.data       = data_joint->position[1];
                                right_arm_shoulder_pitch.data    = data_joint->position[2];
                                right_arm_shoulder_roll.data     = data_joint->position[3];
                                right_arm_shoulder_yaw.data      = data_joint->position[4];
                                right_arm_wrist_roll.data        = data_joint->position[5];
                                torso_yaw.data                   = data_joint->position[6];

                                right_arm_shoulder_pitch_joint.publish(right_arm_shoulder_pitch);
                                right_arm_shoulder_roll_joint.publish(right_arm_shoulder_roll);
                                right_arm_shoulder_yaw_joint.publish(right_arm_shoulder_yaw);
                                right_arm_elbow_roll_joint.publish(right_arm_elbow_roll);
                                right_arm_forearm_yaw_joint.publish(right_arm_forearm_yaw);
                                right_arm_wrist_roll_joint.publish(right_arm_wrist_roll);
                                torso_yaw_joint.publish(torso_yaw);

                                ROS_INFO("TORSO_right_arm OK");
                            }else if(torso_head)
                                    {
                                        head_pitch.data     = data_joint->position[0];
                                        head_yaw.data       = data_joint->position[1];
                                        torso_yaw.data      = data_joint->position[2];

                                        head_pitch_joint.publish(head_pitch);
                                        head_yaw_joint.publish(head_yaw);
                                        torso_yaw_joint.publish(torso_yaw);

                                        ROS_INFO("TORSO_HEAD OK");
                                    }else if(head)
                                            {
                                                head_pitch.data     = data_joint->position[0];
                                                head_yaw.data       = data_joint->position[1];

                                                head_pitch_joint.publish(head_pitch);
                                                head_yaw_joint.publish(head_yaw);

                                                ROS_INFO("HEAD OK");                                               
                                            }else if(torso_tray)
                                                    {
                                                        torso_yaw.data      = data_joint->position[0];
                                                        tray_pitch.data     = data_joint->position[1];

                                                        torso_yaw_joint.publish(torso_yaw);
                                                        tray_pitch_joint.publish(tray_pitch);

                                                        ROS_INFO("TORSO_TRAY OK");
                                                    }else if(left_hand)
                                                            {
                                                                left_gripper.data   = data_joint->position[0];

                                                                left_gripper_joint.publish(left_gripper);

                                                                ROS_INFO("LEFT_HAND OK");
                                                            }else if(right_hand)
                                                                    {
                                                                        right_gripper.data      = data_joint->position[0];

                                                                        right_gripper_joint.publish(right_gripper);

                                                                        ROS_INFO("RIGHT_HAND OK");
                                                                    }else if(tray)
                                                                            {
                                                                                tray_pitch.data     = data_joint->position[0];

                                                                                tray_pitch_joint.publish(tray_pitch);

                                                                                ROS_INFO("TRAY OK");
                                                                            }else if(torso)
                                                                                    {
                                                                                        torso_yaw.data      = data_joint->position[0];

                                                                                        torso_yaw_joint.publish(torso_yaw);

                                                                                        ROS_INFO("TORSO OK");

                                                                                    }else ; 


    torso_left_arm          = false;  
    torso_right_arm         = false;  
    right_arm               = false;  
    left_arm                = false;  
    torso_head              = false;  
    head                    = false;  
    torso_tray              = false;  
    left_hand               = false;  
    right_hand              = false;  
    tray                    = false; 
    torso                   = false;
    
     
    ros::spinOnce();
    loop_rate.sleep();
   }

   ros::spin();
   return 0;
}