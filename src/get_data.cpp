#include <uav_command/get_data.h>

using namespace std;

namespace UAV_plan
{
    odom_data::odom_data()
    {
    }

    odom_data::~odom_data()
    {
    }
    void odom_data::fromQuaternion2yaw(Eigen::Quaterniond q)
    {
        yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    }

    void odom_data::feed(const nav_msgs::Odometry::ConstPtr pMsg)
    {
        odom = *pMsg;
        odom_q.w() = odom.pose.pose.orientation.w;
        odom_q.x() = odom.pose.pose.orientation.x;
        odom_q.y() = odom.pose.pose.orientation.y;
        odom_q.z() = odom.pose.pose.orientation.z;
        fromQuaternion2yaw(odom_q);
        // ROS_WARN("yaw_angle: %.2f", yaw);
    }

    tag2world_data::tag2world_data()
    {
    }

    tag2world_data::~tag2world_data()
    {
    }

    void tag2world_data::waitfor_tag2world()
    {
        while (loop)
        {

            loop = 0;

            try
            {
                listener.waitForTransform("world", "tag_mark", ros::Time(0), duration); // 20 hz
                listener.lookupTransform("world", "tag_mark", ros::Time(0), transform_tag2world);
            }
            catch (tf::TransformException &ex)
            {
                detection = 0;
                ROS_ERROR("%s", ex.what());
                ROS_ERROR("tag has not been deteted !!! ");
                // ros::Duration(0.5).sleep();
                continue;
            }

            tag2world.pose.position.x = transform_tag2world.getOrigin().x();
            tag2world.pose.position.y = transform_tag2world.getOrigin().y();
            tag2world.pose.position.z = transform_tag2world.getOrigin().z();

            tag2world.pose.orientation.w = transform_tag2world.getRotation().w();
            tag2world.pose.orientation.x = transform_tag2world.getRotation().x();
            tag2world.pose.orientation.y = transform_tag2world.getRotation().y();
            tag2world.pose.orientation.z = transform_tag2world.getRotation().z();

            if (!detection)
            {
                ROS_INFO("tag has been deteted !!! ");
            }
            detection = 1;
        }
    }
 ego2px4_data::ego2px4_data(){
        receive_ego_cal=false;
    }
ego2px4_data::~ego2px4_data(){

     }
  //中继储存
    void ego2px4_data::feed(const quadrotor_msgs::PositionCommandConstPtr pMsg){
       ego2px4=*pMsg;
       receive_ego_cal=true;
    }
}