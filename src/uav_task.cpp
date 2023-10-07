
#include <uav_command/uav_task.h>

namespace UAV_plan
{

    uav_command::uav_command()
    {
        land_flag = 0;
        landing = 0;
        task_flag = takeoff;
        Takeoff_land.takeoff_land_cmd = 1; // takeoff:1   land:2
        ego_setpoint.header.frame_id = "world";
        debug = 1;
        ready = false;
        receive_srvinfo = false;
        yaw = 0;
        // 圆环位置、方向初始化
        circle_static0_x = 0.0;
        circle_static0_y = 0.0;
        circle_static0_z = 0.0;
        circle_static0_direction_x = 0.0;
        circle_static0_direction_y = 0.0;
        circle_static0_direction_z = 0.0;

        circle_move0_x = 0.0;
        circle_move0_y = 0.0;
        circle_move0_z = 0.0;
        circle_move0_direction_x = 0.0;
        circle_move0_direction_y = 0.0;
        circle_move0_direction_z = 0.0;
        // init px4ctrl
        px4_setpoint.header.stamp = ros::Time::now();
        px4_setpoint.position.x = 0;
        px4_setpoint.position.y = 0;
        px4_setpoint.position.z = 2.5;

        px4_setpoint.velocity.x = 0;
        px4_setpoint.velocity.y = 0;
        px4_setpoint.velocity.z = 0;

        px4_setpoint.acceleration.x = 0;
        px4_setpoint.acceleration.y = 0;
        px4_setpoint.acceleration.z = 0;

        px4_setpoint.jerk.x = 0;
        px4_setpoint.jerk.y = 0;
        px4_setpoint.jerk.z = 0;

        px4_setpoint.yaw = 0;
        px4_setpoint.yaw_dot = 0;

        px4_flag = true;

        // init ego_planner
        ego_setpoint.pose.position.x = 0;
        ego_setpoint.pose.position.y = 0;
        ego_setpoint.pose.position.z = 2.5;
        theta = 0;
        ego_setpoint.pose.orientation.w = cos(theta / 2.0);
        ego_setpoint.pose.orientation.x = 0;
        ego_setpoint.pose.orientation.y = 0;
        ego_setpoint.pose.orientation.z = sin(theta / 2.0);

        ego_flag = true;
        msg_valid = false;
        next = false;
        // 数组索引值初始化
        index_in_vector = 0;
    }

    uav_command::~uav_command()
    {
    }

    void uav_command::init(ros::NodeHandle &nh)
    {
        // these parameters are in custom world frame
        //  x -- forward
        //  y -- right
        // y_sim --left
        //  original point of world frame is at the bottom left
        nh.param("/uav_command/task/origin_x", origin_x_, 0.65);
        nh.param("/uav_command/task/origin_y", origin_y_, 5.77);
        nh.param("/uav_command/task/right", right, 3.0);
        nh.param("/uav_command/task/left", left, 2.5);
        nh.param("/uav_command/task/cylinder_x_before", cylinder_before_[0], 24.0);
        nh.param("/uav_command/task/cylinder_y_before", cylinder_before_[1], 9.0);
        nh.param("/uav_command/task/cylinder_z_before",cylinder_before_[2],1.5);
        nh.param("/uav_command/task/cylinder_yaw_before",cylinder_before_[3],1.5);
        nh.param("/uav_command/task/chunnel_x_before", chunnel_before_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_before", chunnel_before_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_before", chunnel_before_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_before",chunnel_before_[3], 1.5);
         nh.param("/uav_command/task/chunnel_x_start", chunnel_start_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_start", chunnel_start_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_start", chunnel_start_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_start", chunnel_start_[3], 1.5);
         nh.param("/uav_command/task/chunnel_x_end", chunnel_end_[0], 15.0);
        nh.param("/uav_command/task/chunnel_y_end", chunnel_end_[1], 7.0);
        nh.param("/uav_command/task/chunnel_z_end", chunnel_end_[2], 1.5);
        nh.param("/uav_command/task/chunnel_yaw_end", chunnel_end_[3], 1.5);
        //nh.param("/uav_command/task/x_distance_cylinder3_chunnel", x_distance_cylinder3_chunnel_, 3.0);
        // nh.param("/uav_command/task/chunnel_x_before", chunnel_x_before_, 24.0);
        // nh.param("/uav_command/task/chunnel_y_before", chunnel_y_before_, 9.0);
        // nh.param("/uav_command/task/chunnel_z_before",chunnel_z_before_,1.5);

        nh.param("/uav_command/task/maze_x_0", maze_0_[0], 24.0);
        nh.param("/uav_command/task/maze_y_0", maze_0_[1], 9.0);
        nh.param("/uav_command/task/maze_z_0", maze_0_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_0", maze_0_[3], 1.5);
        nh.param("/uav_command/task/maze_x_1", maze_1_[0], 24.0);
        nh.param("/uav_command/task/maze_y_1", maze_1_[1], 9.0);
        nh.param("/uav_command/task/maze_z_1", maze_1_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_1", maze_1_[3], 1.5);
        nh.param("/uav_command/task/maze_x_2", maze_2_[0], 24.0);
        nh.param("/uav_command/task/maze_y_2", maze_2_[1], 9.0);
        nh.param("/uav_command/task/maze_z_2", maze_2_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_2", maze_3_[3], 1.5);
        nh.param("/uav_command/task/maze_x_3", maze_3_[0], 24.0);
        nh.param("/uav_command/task/maze_y_3", maze_3_[1], 9.0);
        nh.param("/uav_command/task/maze_z_3", maze_3_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_3", maze_3_[3], 1.5);
        nh.param("/uav_command/task/maze_x_4", maze_4_[0], 24.0);
        nh.param("/uav_command/task/maze_y_4", maze_4_[1], 9.0);
        nh.param("/uav_command/task/maze_z_4", maze_4_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_4", maze_4_[3], 1.5);
        nh.param("/uav_command/task/maze_x_behind", maze_behind_[0], 24.0);
        nh.param("/uav_command/task/maze_y_behind", maze_behind_[1], 9.0);
        nh.param("/uav_command/task/maze_z_behind", maze_behind_[2], 1.5);
        nh.param("/uav_command/task/maze_yaw_behind", maze_behind_[3], 1.5);
        nh.param("/uav_command/task/circle_x_before", circle_before_[0], 24.0);
        nh.param("/uav_command/task/circle_y_before", circle_before_[1], 9.0);
        nh.param("/uav_command/task/circle_z_before", maze_before_[2], 1.5);
        nh.param("/uav_command/task/circle_yaw_before", maze_before_[3], 1.5);
        nh.param("/uav_command/task/land_x", land_[0], 1.0);
        nh.param("/uav_command/task/land_y", land_[1], 26.0);
        nh.param("/uav_command/task/land_z", land_[2], 1.0);
        nh.param("/uav_command/task/land_yaw", land_[3], 26.0);
        nh.param("/uav_command/task/cylinder_z_0",cylinder_z_0_,1.5);
        // flag=1;
        // object_client = nh.serviceClient<object_all::object>("/obj_position");
        // srv.request.type = 1;

        // object_client.call(srv);
        // A_x_ = srv.response.Point.x;
        // B_y_ = srv.response.Point.y;
        // C_z_ = srv.response.Point.z;
    }

    // Does ego_planner arrive its target?
    bool uav_command::arrive1(double dst)
    {
        distance1 = pow(rc_odom.odom.pose.pose.position.x - ego_setpoint.pose.position.x, 2) + pow(rc_odom.odom.pose.pose.position.y - ego_setpoint.pose.position.y, 2);
        distance1 = sqrt(distance1);

        ROS_INFO("distance1: %.2f", distance1);

        if (distance1 < dst)
            return true;
        else
            return false;
    }

    // Does px4ctrl  arrive its target?
    bool uav_command::arrive2(double dst)
    {

        distance2 = pow(rc_odom.odom.pose.pose.position.x - px4_setpoint.position.x, 2) + pow(rc_odom.odom.pose.pose.position.y - px4_setpoint.position.y, 2);

        angle = abs(px4_setpoint.yaw - rc_odom.yaw);

        distance2 = sqrt(distance2);

        // ROS_INFO("distance2: %.2f", distance2);
        ROS_INFO("odom_yaw: %.2f", rc_odom.yaw);
        ROS_INFO("yaw_setpoint:%.2f", px4_setpoint.yaw);
        ROS_INFO("angle: %.2f", angle);

        if (px4_setpoint.yaw < 3.1 && px4_setpoint.yaw > -3.1)
        {
            // ROS_INFO("0 0 0");
            ROS_INFO("distance2: %.2f", distance2);
            if (distance2 < dst && angle < 0.05)
                return true;
            else
                return false;
        }

        if (3.1 < px4_setpoint.yaw || px4_setpoint.yaw < -3.1) // 0 --- M_PI  0 ---  -M_PI
        {
            // ROS_INFO("M_PI");
            if ((distance2 < dst && angle < 0.1) || (distance2 < dst && angle > 6.18))
                return true;
            else
                return false;
        }
    }

    // 启用前
    //  ego_flag = true;
    //  px4_flag = true;
    //  next = false;

    void uav_command::go2target(double x, double y, double z, double yaw)
    {
        ego_setpoint.header.stamp = ros::Time::now();
        ego_setpoint.pose.position.x = x;
        ego_setpoint.pose.position.y = y;
        ego_setpoint.pose.position.z = z;
        ego_setpoint.pose.orientation.w = cos(yaw / 2.0);
        ego_setpoint.pose.orientation.x = 0;
        ego_setpoint.pose.orientation.y = 0;
        ego_setpoint.pose.orientation.z = sin(yaw / 2.0);

        px4_setpoint.header.stamp = ros::Time::now();
        px4_setpoint.position.x = x;
        px4_setpoint.position.y = y;
        px4_setpoint.position.z = z;
        px4_setpoint.yaw = yaw;

        // 先用ego_planner 引导到目标位置
        // 再调用px4ctrl旋转到合适的角度
        // ego
        if (ego_flag)
        {
            ego_pub.publish(ego_setpoint);
        }
        ego_flag = false;
        if (arrive1(0.5) && next == false)
        {
            ROS_INFO("ego arrives at target!!!");
            px4_flag = false;
        }

        // px4
        if (!px4_flag)
        {
            px4_pub.publish(px4_setpoint);
        }
        if (arrive2(0.3) && next == false)
        {
            next = true;
            px4_flag = true;
            ros::Duration(1).sleep();
            ROS_INFO("px4 arrives at target!!!");
        }
    }

    // 启用前
    //  ego_flag = ture;
    //  next = false;

    void uav_command::ego(double x, double y, double z, double yaw, double dist)
    {
        ego_setpoint.header.stamp = ros::Time::now();
        ego_setpoint.pose.position.x = x;
        ego_setpoint.pose.position.y = y;
        ego_setpoint.pose.position.z = z;
        ego_setpoint.pose.orientation.w = cos(yaw / 2.0);
        ego_setpoint.pose.orientation.x = 0;
        ego_setpoint.pose.orientation.y = 0;
        ego_setpoint.pose.orientation.z = sin(yaw / 2.0);

        if (ego_flag)
        {
            ego_pub.publish(ego_setpoint);
        }
        ego_flag = false;

        if (arrive1(dist) && next == false)
        {
            ROS_INFO("ego arrives at target!!!");
            next = true;
        }
    }

    // 启用px4_flag = true;
    //  next = false;
    void uav_command::px4(double x, double y, double z, double yaw, double dist)
    {
        px4_setpoint.header.stamp = ros::Time::now();
        px4_setpoint.position.x = x;
        px4_setpoint.position.y = y;
        px4_setpoint.position.z = z;
        px4_setpoint.yaw = yaw;

        if (px4_flag)
        {
            px4_pub.publish(px4_setpoint);
        }

        if (arrive2(dist) && next == false)
        {
            next = true;
            px4_flag = false;
            ROS_INFO("px4 arrives at target!!!");
        }
    }

    void uav_command::land_planner()
    {

        rc_tag2world.waitfor_tag2world();

        rc_tag2world.loop = 1;

        if (rc_tag2world.detection == 1) // start langing process
        {
            if (!land_flag)
            {
                x = rc_tag2world.tag2world.pose.position.x;
                y = rc_tag2world.tag2world.pose.position.y;
                z = 1.5;
                yaw = rc_odom.yaw;

                px4(x, y, z, yaw, 0.1);

                if (debug)
                {
                    ROS_INFO("flying to land_tag: x:%0.1f  y:%0.1f  z:%0.1f", x, y, z);
                }

                debug = 0;

                if (next == true)
                {
                    land_flag = 1;
                    ROS_ERROR("lang_flag = 1");
                }
            }

            if (land_flag == 1 && !landing)
            {
                Takeoff_land.takeoff_land_cmd = 2;
                ROS_INFO("landing!!!");

                ros::Duration(1).sleep();
                TakeoffLand_pub.publish(Takeoff_land);
                landing = 1;

                ROS_ERROR("landing!!!");
            }
        }
    }

    void uav_command::task_planner()
    { // task_flag
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

        // take_off

        if (task_flag == takeoff)
        {
            Takeoff_land.takeoff_land_cmd = 1;
            // TakeoffLand_pub.publish(Takeoff_land);

            task_flag = cylinder_before;
            ROS_INFO("takeoff!!!");
            ego_flag = true;
            px4_flag = true;
            next = false;
        }
        //
        else if (task_flag == cylinder_before)
        {
            x = cylinder_before_[0]; // 6.00
            y = cylinder_before_[1]; // 0.00
            z = cylinder_before_[2];
            // z = cylinder_z_before_;
            yaw =cylinder_before_[3];

            if (ego_flag == true)
            {
                ROS_INFO("move to the position of  seeing  cylinder !!!");
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.6);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;
                task_flag = cylinder0;
                next = false;
            }
        }
        // cylinder0，第一、二、三、四个圆柱
        else if (task_flag == cylinder0)
        {
            srv_cylinder_info.request.obstacle_type = task_flag;
            last_task_flag = task_flag - 1; // 1、2、3
            receive_srvinfo = vision_cylinder_client.call(srv_cylinder_info);
            if (receive_srvinfo)
            {
                // ROS_INFO("get vision msg about cylinder now.");
                receive_srvinfo = false;
                // 接收视觉消息，圆柱的x,y位置
                ego_flag = true;
                px4_flag = true;

                cylinder0_[0] = srv_cylinder_info.response.cylinder_x;
                cylinder0_[1] = srv_cylinder_info.response.cylinder_y;

                // ROS_INFO("get  vision_msg about cylinder0 position!!! x:%.2f y:%.2f", cylinder0_x_, cylinder0_y_);
                // 飞到圆柱和起飞点连线上距离圆柱0.5m的位置
                // x = (1 - 0.5 / abs((-rc_odom.odom.pose.pose.position.x + cylinder0_x_))) * (-rc_odom.odom.pose.pose.position.x + cylinder0_x_) + rc_odom.odom.pose.pose.position.x;
                // y = (1 - 0.5 / abs((-rc_odom.odom.pose.pose.position.y + cylinder0_y_))) * (-rc_odom.odom.pose.pose.position.y + cylinder0_y_) + rc_odom.odom.pose.pose.position.y;
                x = cylinder0_[0];
                y = cylinder0_[1];
                z = cylinder_z_0_;

                if (ego_flag == true)
                {
                    yaw = -M_PI / 2; // 旋转90度
                    ROS_INFO("cylinder0 !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                }
                // go2target(x, y, z, yaw);
                ego(x, y, z, yaw, 0.5);
                // px4(x, y, z, yaw, 0.2);

                if (next == true)
                {
                    ego_flag = true;
                    px4_flag = true;

                    task_flag = wader; // 转到计算绕过圆柱的位置状态
                    int i = 0;
                    double x_tmp = x;
                    double y_tmp = y;
                    double yaw_tmp = 0;
                    while (i < 8)
                    {
                        switch (i)
                        {
                        case 0:
                            x_tmp = x_tmp + 0.5;
                            y_tmp = y_tmp + left;
                            yaw_tmp = 0;
                            break;
                        case 1:
                            x_tmp = x_tmp + 0.5;
                            yaw_tmp = - M_PI / 2;
                            break;
                        case 2:
                            y_tmp = y_tmp - right - left;
                            yaw_tmp = 0;
                            break;
                        case 3:
                            x_tmp = x_tmp + 1.0;
                            yaw_tmp = M_PI / 2;
                            break;
                        case 4:
                            y_tmp = y_tmp + right + left;
                            yaw_tmp = 0;
                            break;
                        case 5:
                            x_tmp = x_tmp + 1;
                            yaw_tmp = -M_PI / 2;
                            break;
                        case 6:
                            y_tmp = y_tmp - right - left;
                            yaw_tmp = 0;
                            break;
                        case 7:
                            x_tmp = x_tmp + 1;
                            yaw_tmp = -M_PI / 2;
                            break;
                        }
                        x_tmp_vec[i] = x_tmp;
                        y_tmp_vec[i] = y_tmp;
                        yaw_tmp_vec[i] = yaw_tmp;
                        i = i + 1;
                    }
                    next = false;
                }
            }
            else
            {
                // ROS_INFO("something wrong while get information from vision about cylinder0 or not the first time!");
            }
        }
        // 蛇形转弯过圆柱
        //第一个圆柱点为侧移，使用px4直接给点，后续都使用ego进行轨迹规划
        else if (task_flag == wader)
        {
            last_task_flag = task_flag - 1;
            ego(x_tmp_vec[index_in_vector], y_tmp_vec[index_in_vector], cylinder_z_0_, yaw_tmp_vec[index_in_vector], 0.5);

            // if (index_in_vector == 0)
            // {
            //     px4(x_tmp_vec[index_in_vector], y_tmp_vec[index_in_vector], 1.5, yaw_tmp_vec[index_in_vector], 0.3);
            // }
            // else
            // {
            //     // ROS_INFO("point%d !!! x:%.2f y:%.2f z:%.2f", index_in_vector, x_tmp_vec[index_in_vector], y_tmp_vec[index_in_vector], 1.50);
            //     // px4(x_tmp_vec[index_in_vector], y_tmp_vec[index_in_vector], 1.5, yaw_tmp_vec[index_in_vector], 0.2);
            //     // go2target(x_tmp_vec[index_in_vector], y_tmp_vec[index_in_vector], 1.5, yaw_tmp_vec[index_in_vector]);
            //     ego(x_tmp_vec[index_in_vector], y_tmp_vec[index_in_vector], 1.5, yaw_tmp_vec[index_in_vector], 0.5);
            // }

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;
                index_in_vector = index_in_vector + 1;
                if (index_in_vector == 8)
                {
                    task_flag = chunnel_before;
                }
                next = false;
            }

            // 前三个柱子的下一任务都是cylinder,最后一个柱子的任务是隧道
        }
        // chunnel
        else if (task_flag == chunnel_before)
        {
            // ROS_INFO("Task Flag = %d ", task_flag);
            x = chunnel_before_[0]; // 16.06
            y = chunnel_before_[1]; //-3.51
            z = chunnel_before_[2];
            yaw =chunnel_before_[3];

            if (ego_flag == true)
            {
                ROS_INFO("chunnel_before !!! x:%.2f y:%.2f z:%.2f angle:%.2f", x, y, z, yaw);
            }
            go2target(x, y, z, yaw);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = chunnel_start;
                next = false;
            }
        }
        else if (task_flag == chunnel_start)
        {
            // ROS_INFO("Task Flag = %d ", task_flag);
            x = chunnel_start_[0]; // 16.06
            y = chunnel_start_[1]; //-3.51
            z = chunnel_start_[2];
            yaw = chunnel_start_[3];

            if (ego_flag == true)
            {
                ROS_INFO("chunnel_start !!! x:%.2f y:%.2f z:%.2f angle:%.2f", x, y, z, yaw);
            }
            go2target(x, y, z, yaw);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = chunnel_end;
                next = false;
            }
        }
        else if (task_flag == chunnel_end)
        {
            x = chunnel_end_[0];
            y = chunnel_end_[1];
            z = chunnel_end_[2];
            yaw = chunnel_end_[3];
            if (ego_flag == true)
            {
                ROS_INFO("chunnel_end !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                ego_flag = false;
            }
            // go2target(x, y, z, yaw);
            px4(x, y, z, yaw, 0.5);
            //ego(x, y, z, yaw, 0.35);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_0;
                next = false;
            }
        }
        else if (task_flag == maze_0)
        {
            x = maze_0_[0];
            y = maze_0_[1];
            z = maze_0_[2];
            yaw =maze_0_[3];

            if (ego_flag == true)
            {
                ROS_INFO("go to the point brfore gate0!!! x:%.2f y:%.2f z:%.2f", x, y, z);
            }
            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.5);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_1;
                next = false;
            }
        }
        // maze_0
        else if (task_flag == maze_1)
        {
            x = maze_1_[0];
            y = maze_1_[1];
            z = maze_1_[2];
            yaw =maze_1_[3];

            if (ego_flag == true)
                ROS_INFO("go through gate0 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.5);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_1;
                next = false;
            }
        }
        else if (task_flag == maze_2)
        {
            x = maze_2_[0];
            y = maze_2_[1];
            z = maze_2_[3];
            yaw = maze_2_[4];

            if (ego_flag == true)
                ROS_INFO("go to the point brfore gate1 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.4);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_3;
                next = false;
            }
        }
        else if (task_flag == maze_3)
        {
            x = maze_3_[0];
            y = maze_3_[1];
            z = maze_3_[2];
            yaw = maze_3_[3];

            if (ego_flag == true)
                ROS_INFO("after go through gate1 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.3);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_4;
                next = false;
            }
        }
        else if (task_flag == maze_4)
        {
            x = maze_4_[0];
            y = maze_4_[1];
            z = maze_4_[2];
            yaw = maze_4_[3];

            if (ego_flag == true)
                ROS_INFO("go to the point brfore gate2 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw)
            ego(x, y, z, yaw, 0.3);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = maze_behind;
                next = false;
            }
        }
        else if (task_flag == maze_behind)
        {
            x = maze_behind_[0];
            y = maze_behind_[1];
            z = maze_behind_[2];
            yaw = maze_behind_[3];

            if (ego_flag == true)
                ROS_INFO("after go through gate2 !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.3);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = circle_before;
                next = false;
            }
        }
        else if (task_flag == circle_before)
        {
            x = circle_before_[0];
            y = circle_before_[1];
            z = circle_before_[2]; // 圆环圆心离地高度2-2.5m
            yaw = circle_before_[3];

            if (ego_flag == true)
                ROS_INFO("go to position that can see the circle!!! x:%.2f y:%.2f z:%.2f", x, y, z);

            // go2target(x, y, z, yaw);
            ego(x, y, z, yaw, 0.3);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                task_flag = circle_static_0;
                next = false;
            }
        }
        // circle_static
        else if (task_flag == circle_static_0)
        {
            srv_circle_info.request.obstacle_type = circle_static_0;

            receive_srvinfo = vision_circle_client.call(srv_circle_info);

            if (receive_srvinfo)
            {
                ROS_INFO("get vision msg about static circle now.");
                receive_srvinfo = false;
                ego_flag = true;
                px4_flag = true;
                msg_valid = srv_circle_info.response.msg_valid;
                if (msg_valid)
                {
                    circle_static0_x = srv_circle_info.response.pos_x;
                    circle_static0_y = srv_circle_info.response.pos_y;
                    circle_static0_z = srv_circle_info.response.pos_z;
                    circle_static0_direction_x = srv_circle_info.response.direction_x;
                    circle_static0_direction_y = srv_circle_info.response.direction_y;
                    circle_static0_direction_z = srv_circle_info.response.direction_z;

                    ROS_INFO("get  vision_msg about ciecle_0 position!!! x:%.2f y:%.2f z:%.2f", circle_static0_x, circle_static0_y, circle_static0_z);
                    ROS_INFO("get  vision_msg about ciecle_0 orientation!!! dir_x:%.2f dir_y:%.2f dir_z:%.2f", circle_static0_direction_x, circle_static0_direction_y, circle_static0_direction_z);

                    double dir_tmp = sqrt(circle_static0_x * circle_static0_x + circle_static0_y * circle_static0_y + circle_static0_z * circle_static0_z);

                    if (abs(circle_static0_direction_x + circle_static0_x - rc_odom.odom.pose.pose.position.x) > abs(circle_static0_x - rc_odom.odom.pose.pose.position.x))
                    {
                        // 方向背离无人机
                        x = circle_static0_x - 1.5 * circle_static0_direction_x / dir_tmp;
                        y = circle_static0_y - 1.5 * circle_static0_direction_y / dir_tmp;
                        z = circle_static0_z - 1.5 * circle_static0_direction_z / dir_tmp;
                        // 待计算
                        yaw = M_PI;
                    }
                    else
                    {
                        x = circle_static0_x + 1.5 * circle_static0_direction_x / dir_tmp;
                        y = circle_static0_y + 1.5 * circle_static0_direction_y / dir_tmp;
                        z = circle_static0_z + 1.5 * circle_static0_direction_z / dir_tmp;
                        yaw = M_PI;
                    }
                    if (ego_flag == true)
                    {
                        ROS_INFO("circle_static !!! x:%.2f y:%.2f z:%.2f", circle_static0_x, circle_static0_y, circle_static0_z);
                        ROS_INFO("move to circle_static !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                    }
                    ego(x, y, z, yaw, 0.3);
                    // px4(x, y, z, yaw, 0.2);
                    if (next == true)
                    {
                        next = false;
                        ego_flag = true;
                        px4_flag = true;

                        if (abs(circle_static0_direction_x + circle_static0_x - rc_odom.odom.pose.pose.position.x) > abs(circle_static0_x - rc_odom.odom.pose.pose.position.x))
                        {

                            x = circle_static0_x + 1.5 * circle_static0_direction_x / dir_tmp;
                            y = circle_static0_y + 1.5 * circle_static0_direction_y / dir_tmp;
                            z = circle_static0_z + 1.5 * circle_static0_direction_z / dir_tmp;
                            // 待计算
                            yaw = M_PI;
                        }
                        else
                        {

                            x = circle_static0_x - 1.5 * circle_static0_direction_x / dir_tmp;
                            y = circle_static0_y - 1.5 * circle_static0_direction_y / dir_tmp;
                            z = circle_static0_z - 1.5 * circle_static0_direction_z / dir_tmp;
                            yaw = M_PI;
                        }
                        // ego(x, y, z, yaw, 0.2);
                        px4(x, y, z, yaw, 0.2);

                        if (next == true)
                        {
                            ego_flag = true;
                            px4_flag = true;
                            task_flag = circle_move_0; // 下一状态为动态环穿过
                            next = false;
                        }
                    }
                }
                else
                {
                    ROS_INFO("get message of circle_static ,but the message is not correct.");
                }
            }
            else
            {
                ROS_INFO("something wrong while get information from vision about circle_static or not the first time!");
            }
        }
        // circle_move
        else if (task_flag == circle_move_0)
        {
            srv_circle_info.request.obstacle_type = circle_move_0;
            while (receive_srvinfo == false)
            { // 当没有接收到视觉消息一直循环
                receive_srvinfo = vision_circle_client.call(srv_circle_info);
                ROS_INFO("waiting for vision msg about move circle...");
            }
            ROS_INFO("get vision msg about static move now.");
            if (receive_srvinfo)
            {
                ego_flag = true;
                px4_flag = true;

                circle_move0_x = srv_circle_info.response.pos_x;
                circle_move0_y = srv_circle_info.response.pos_y;
                circle_move0_z = srv_circle_info.response.pos_z;
                circle_move0_direction_x = srv_circle_info.response.direction_x;
                circle_move0_direction_y = srv_circle_info.response.direction_y;
                circle_move0_direction_z = srv_circle_info.response.direction_z;

                ROS_INFO("get  vision_msg about ciecle_0 position!!! x:%.2f y:%.2f z:%.2f", circle_move0_x, circle_move0_y, circle_move0_z);
                ROS_INFO("get  vision_msg about ciecle_0 orientation!!! dir_x:%.2f dir_y:%.2f dir_z:%.2f", circle_move0_direction_x, circle_move0_direction_y, circle_move0_direction_z);
                // 先飞到圆环正前方
                double dir_tmp = sqrt(circle_move0_x * circle_move0_x + circle_move0_y * circle_move0_y + circle_move0_z * circle_move0_z);
                // 判断得到的圆环朝向是背离无人机一方还是指向无人机的
                if (abs(circle_move0_direction_x + circle_move0_x - rc_odom.odom.pose.pose.position.x) > abs(circle_move0_x - rc_odom.odom.pose.pose.position.x))
                {
                    // 方向背离无人机
                    x = circle_move0_x - 1.5 * circle_move0_direction_x / dir_tmp;
                    y = circle_move0_y - 1.5 * circle_move0_direction_y / dir_tmp;
                    z = circle_move0_z - 1.5 * circle_move0_direction_z / dir_tmp;
                    // 待计算
                    yaw = M_PI;
                }
                else
                {
                    // 方向指向无人机
                    x = circle_move0_x + 1.5 * circle_move0_direction_x / dir_tmp;
                    y = circle_move0_y + 1.5 * circle_move0_direction_y / dir_tmp;
                    z = circle_move0_z + 1.5 * circle_move0_direction_z / dir_tmp;
                    yaw = M_PI;
                }
                if (ego_flag == true)
                {
                    ROS_INFO("circle_move !!! x:%.2f y:%.2f z:%.2f", circle_move0_x, circle_move0_y, circle_move0_z);
                    ROS_INFO("move to circle_move !!! x:%.2f y:%.2f z:%.2f", x, y, z);
                }
                ego(x, y, z, yaw, 0.3);
                if (next == true)
                {
                    next = false;
                    ego_flag = true;
                    px4_flag = true;
                    // 穿过静态圆环给定点
                    if (abs(circle_static0_direction_x + circle_static0_x - rc_odom.odom.pose.pose.position.x) > abs(circle_static0_x - rc_odom.odom.pose.pose.position.x))
                    {
                        // 方向背离无人机
                        x = circle_static0_x + 1.5 * circle_static0_direction_x / dir_tmp;
                        y = circle_static0_y + 1.5 * circle_static0_direction_y / dir_tmp;
                        z = circle_static0_z + 1.5 * circle_static0_direction_z / dir_tmp;
                        // 待计算
                        yaw = M_PI;
                    }
                    else
                    {
                        // 方向指向无人机
                        x = circle_static0_x - 1.5 * circle_static0_direction_x / dir_tmp;
                        y = circle_static0_y - 1.5 * circle_static0_direction_y / dir_tmp;
                        z = circle_static0_z - 1.5 * circle_static0_direction_z / dir_tmp;
                        yaw = M_PI;
                    }
                    /////////////////////////////////////////////////////////////////////////////
                    ego(x, y, z, yaw, 0.2);

                    if (next == true)
                    {
                        ego_flag = true;
                        px4_flag = true;
                        task_flag = land; // 下一状态为落地
                        next = false;
                    }
                }
            }
        }
        // landing
        else if (task_flag == land)
        {
            x = land_[0];
            y = land_[1];
            z = land_[2];
            yaw = land_[3];

            if (ego_flag == true)
                ROS_INFO("go to apriltag !!! x:%.2f y:%.2f z:%.2f", x, y, z);

            ego(x, y, z, yaw, 0.3);

            if (next == true)
            {
                ego_flag = true;
                px4_flag = true;

                land_planner();
                next = false;
            }
        }
    }
}
