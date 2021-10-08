/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "turtlebot3_gazebo/turtlebot3_drive.h"

Turtlebot3::Turtlebot3()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3::~Turtlebot3()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
void CLaser::initLaser(ros::NodeHandle *nh_){
  //Laser subscribe 
  laser_scan_sub_  = nh_->subscribe("scan", 10, &CLaser::laserScanMsgCallBack, this);
}

void COdom::initOdom(ros::NodeHandle *nh_){
  //Initialise current and prev positions
  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;
  //Odom subscribe
  odom_sub_ = nh_->subscribe("odom", 10, &COdom::odomMsgCallBack, this);
}

bool Turtlebot3::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.5;
  check_side_dist_    = 0.55; //0.5

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  //Intialise laser
  Laser.initLaser(&nh_);
  //Intialise odom
  Odom.initOdom(&nh_);

  return true;
}

void COdom::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  
        
        //Calculate current position
	tb3_pose_ = atan2(siny, cosy);
}

void CLaser::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[14] = {0,15,30,45,60,75,90,180,270,285,300,315,330,345};

  //Scan location at different angles
  for (int num = 0; num < 14; num++)
  {
    //If data is infinity, then let it equal max range
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    //Store data
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Turtlebot3::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  //Update linear and angular vel
  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3::controlLoop()
{
  //Initialise the turtlebot's state
  static uint8_t turtlebot3_state_num = 0;
  //static bool FLAG =true;
  switch(turtlebot3_state_num)
  {
    //Determine which direction robot needs to move
    case GET_TB3_DIRECTION:
      //Check if the robot needs to make a right turn
      if(Laser.scan_data_[CENTER] >check_forward_dist_ && Laser.scan_data_[RIGHT_300] > check_side_dist_ && Laser.scan_data_[RIGHT_285]> check_side_dist_ && Laser.scan_data_[RIGHT_330] > check_side_dist_)     {  
        //Update prev pose  
        Odom.prev_tb3_pose_ = Odom.tb3_pose_;
        //Change state to right turn
        turtlebot3_state_num = TB3_RIGHT_TURN;
      }
      //Check if the robot needs to go forward
      else if(Laser.scan_data_[CENTER]>check_forward_dist_ ){
        //Update prev pose  
        Odom.prev_tb3_pose_ = Odom.tb3_pose_;
        //Change state to forward
        turtlebot3_state_num = TB3_DRIVE_FORWARD;
      }
      //Check if the robot needs to make a left turn
      else if (Laser.scan_data_[CENTER] < check_forward_dist_ || Laser.scan_data_[RIGHT_270]<check_side_dist_ || Laser.scan_data_[RIGHT_345]<check_side_dist_ || Laser.scan_data_[RIGHT_300]<check_side_dist_)  {
        //Update prev pose  
        Odom.prev_tb3_pose_ = Odom.tb3_pose_;
        //Change state to left turn
        turtlebot3_state_num = TB3_LEFT_TURN;
      }
      break;
 
    //Robot drives forward
    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    //Robot makes a right turn
    case TB3_RIGHT_TURN:
      if (fabs(Odom.prev_tb3_pose_ - Odom.tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    //Robot makes a left turn
    case TB3_LEFT_TURN:
      if (fabs(Odom.prev_tb3_pose_ - Odom.tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3 turtlebot3_drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
