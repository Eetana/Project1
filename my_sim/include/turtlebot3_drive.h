#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define DEG2RAD (M_PI / 180.0)     // transfer degree to radius
#define RAD2DEG (180.0 / M_PI)     // transfer radius to degree
#define Max_detected 20

// There are the angles which would be used to check the barrier, because the laser will detecte range around 360 degree.
// We don't need to check all of its.
#define CENTER    0
#define LEFT_15   1
#define LEFT_30   2
#define LEFT_45  3
#define LEFT_60  4
#define LEFT_75  5
#define LEFT_90  6
#define FOR_BACK 7
#define RIGHT_270  8
#define RIGHT_285  9
#define RIGHT_300  10
#define RIGHT_315  11
#define RIGHT_330  12
#define RIGHT_345  13

// Define the robot moving velocity and the cornering speed which call angular velocity
#define LINEAR_VELOCITY  0.2    // velocity of moving forward
#define ANGULAR_VELOCITY 1.3    // velocity of turn left or right

// Define the direction using there are 4 different instructions
#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1     // ask robot move forward
#define TB3_RIGHT_TURN    2     // ask robot turn right
#define TB3_LEFT_TURN     3     // ask robot turn left

// Create a class which content the Laser
class CLaser
{
 public:
  //Laser callback
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  //Initialise laser 
  void initLaser(ros::NodeHandle *nh_);
  //Datapoints scanned by laser
  double scan_data_[13] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
 private:
  //ROS topic laser subscriber
  ros::Subscriber laser_scan_sub_;
  
};

class COdom
{
 public:
  //Initialise function
  void initOdom(ros::NodeHandle *nh_);
  //Odom callback  
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);

  //Current and previous position which could be use to test.
  double tb3_pose_;
  double prev_tb3_pose_;
 private:
  //ROS topic Odom subscriber
  ros::Subscriber odom_sub_;
  
};

// The main class for control turtlebot
class Turtlebot3
{
 public:
  Turtlebot3();          // Constructor
  ~Turtlebot3();         // Destructor
  bool init();           // Initial function and return T/F
  bool controlLoop();    // Control robot moving and return ture

 private:
  CLaser Laser;          // call class CLaser and name it Laser
  COdom Odom;            // call class CLaser and name it Laser

  //ROS NodeHandle
  ros::NodeHandle nh_;   // create a node call nh
  ros::NodeHandle nh_priv_;    // create a node call nh_priv_

  //ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  //Variables
  double escape_range_;   
  double check_forward_dist_;   // It's a distance limiting of forward which will compare with laser check distance.
  double check_side_dist_;      // It's a distance limiting of left side and right side which will compare with laser check distance.

  //Function prototypes
  void updatecommandVelocity(double linear, double angular);
};
#endif // TURTLEBOT3_DRIVE_H_
