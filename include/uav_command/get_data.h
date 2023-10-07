#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
using namespace std;

namespace UAV_plan
{

    class odom_data
    {

    public:
        odom_data();
        ~odom_data();

        nav_msgs::Odometry odom;
        Eigen::Quaterniond odom_q;
        double yaw;
        void feed(const nav_msgs::Odometry::ConstPtr pMsg);//储存接收到的信息

        void fromQuaternion2yaw(Eigen::Quaterniond q);//四元数转化为偏航角
    };

    class tag2world_data
    {
    public:
        tag2world_data();
        ~tag2world_data();

        bool detection = 0; // 0: has not been deteted ;1: has not been deteted
        bool loop = 1;      // in whole loop of planner, there should be *** tag2world_data.loop = 1; ***
        ros::Duration duration = ros::Duration(0.05);

        tf::TransformListener listener;
        // get tf , tag bundle detections pose relative to world
        tf::StampedTransform transform_tag2world;
        geometry_msgs::PoseStamped tag2world;

        void waitfor_tag2world();
    };
    
    class ego2px4_data
    {
    public:
         ego2px4_data();
        ~ ego2px4_data();
             bool receive_ego_cal; // 检测是否接受到
        quadrotor_msgs::PositionCommand ego2px4;
        void feed(const quadrotor_msgs::PositionCommandConstPtr pMsg);//储存接收到的信息
    };
}
