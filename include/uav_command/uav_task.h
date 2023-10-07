#include <uav_command/get_data.h>

#include <vision_server/circle.h>
#include <vision_server/cylinder.h>
#include <math.h>
#include<vector>

namespace UAV_plan
{
    class uav_command
    {
    private:

        // prameters
        double origin_x_;
        double origin_y_;
        double cylinder0_[2];//x,y
        double left;
        double right;
        // double y_distance_cylinder3_chunnel_;
        //double x_distance_cylinder3_chunnel_;
        //double chunnel_x_before_;
        //double chunnel_y_before_;
        //double chunnel_z_before_;
        //double x_distance_maze2_circle_static;
        double chunnel_before_[4];//x,y,z,yaw
        double chunnel_start_[4];
        double chunnel_end_[4];
        double cylinder_before_[4];
        double cylinder_z_0_;
        double circle_before_[4];
        double maze_0_[4];
        double maze_1_[4];
        double maze_2_[4];
        double maze_3_[4];
        double maze_4_[4];
        double maze_behind_[4];
        double circle_static0_x;
        double circle_static0_y;
        double circle_static0_z;
        double circle_static0_direction_x;
        double circle_static0_direction_y;
        double circle_static0_direction_z;

        double circle_move0_x;
        double circle_move0_y;
        double circle_move0_z;
        double circle_move0_direction_x;
        double circle_move0_direction_y;
        double circle_move0_direction_z;

        double land_[4];
        // task_flag
        //  1 -- takeoff
        //  2 -- pass through cylinder0
        //  3 -- pass through cylinder1
        //  4 -- pass through cylinder2
        // 5  -- pass through cylinder3
        //  6 -- pass through  chunnel
        // 7 --pass through maze
        //  8 -- circle_static_0
        //  9 -- circle_move_0
        //  10 -- land

    public:
        uav_command();
        ~uav_command();
        //wader中的点
        std::vector<double> x_tmp_vec=std::vector<double>(8);
        std::vector<double> y_tmp_vec= std::vector<double>(8);
        std::vector<double>yaw_tmp_vec = std::vector<double>(8);
        int  index_in_vector;
        void init(ros::NodeHandle &nh);
        enum task_flags
        {
            takeoff = 1,
            cylinder_before=2,
            cylinder0 = 3,
            wader = 4,
            chunnel_before=5,
            chunnel_start = 6,
            chunnel_end = 7,
            maze_0 = 8,
            maze_1=9,
            maze_2=10,
            maze_3=11,
            maze_4=12,
            maze_behind=13,
            circle_before=14,
            circle_static_0 = 15,
            circle_move_0 = 16,
            land = 17
        };
        odom_data rc_odom;
        tag2world_data rc_tag2world;

        //  TYPE_WALL = 1  TYPE_CHANNEL = 2  TYPE_1.5M_CIRCLE = 3  TYPE_1M_CIRCLE = 4
        ros::ServiceClient object_client;
        // 新增客户端
        ros::ServiceClient vision_cylinder_client;
        ros::ServiceClient vision_circle_client;
        //话题发布
        ros::Publisher TakeoffLand_pub;
        ros::Publisher ego_pub;
        ros::Publisher px4_pub;
        int task_flag;
        int last_task_flag;
        bool land_flag;
        bool landing;
        bool debug;
        // 根据视觉的信息知道是否到达能看到完整障碍物的位置
        bool ready;
        // 是否接收到视觉的服务响应
        bool receive_srvinfo;
        // 视觉检测到的障碍物信息
        vision_server::cylinder srv_cylinder_info;
        vision_server::circle srv_circle_info;
        //发布的消息，ego到点，px4转向，自动降落指令
        geometry_msgs::PoseStamped ego_setpoint;
        quadrotor_msgs::PositionCommand px4_setpoint;
        quadrotor_msgs::TakeoffLand Takeoff_land;
         // 中继修改
        ros::Subscriber ego_sub;
        ego2px4_data ego2px4_data_msg;
        bool px4_flag;
        bool ego_flag;
        bool next; // 到达指定点后会改为true
        int flag;//ego or px4
        double distance1;
        double distance2;
        double angle;
        double theta;
        
        // target
        double x;
        double y;
        double z;
        double yaw;
        bool msg_valid;
        void go2target(double x, double y, double z, double yaw);
        void ego(double x, double y, double z, double yaw, double dist);
        void px4(double x, double y, double z, double yaw, double dist);

        void land_planner();
        void task_planner();
        bool arrive1(double dst);
        bool arrive2(double dst);
    };
}
