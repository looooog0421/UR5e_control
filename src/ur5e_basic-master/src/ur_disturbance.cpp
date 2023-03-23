//
// Created by lgx on 23-3-22.
//
#include "ros/ros.h"
#include "UR5e.hpp"


int main(int argc, char** argv)
{
    ros::init(argc,argv,"ur5e_server");
    ros::NodeHandle nh;

    double timeout = 0.005;
    double control_rate = 100;
    std::string urdf_param, UR_prefix;
    nh.param("urdf_param",urdf_param, std::string("/robot_description"));
    nh.param("ur_prefix", UR_prefix,std::string("UR5e"));
    UR5e *ur5e = new UR5e(nh, urdf_param, "base_link", "tool0", timeout, UR_prefix, control_rate);

    // 运动到初始位置
    std::vector<double> gravity_down_joints;
    nh.getParam("gravity_down_joints", gravity_down_joints); //[180, -90, -90, -90, 90, 90]//这个备注的角度似乎有问题，输出角度出来的第一个角应该是-90度
    KDL::JntArray gravity_down_jnt(6);
    for(unsigned int i = 0; i < 6; ++i)
        gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
    ROS_INFO_STREAM("Move start joint position");
    ur5e->servoj_moveto(gravity_down_jnt, 5, true);
    ros::Duration(0.5).sleep();
    ROS_INFO_STREAM(gravity_down_jnt.data);


    //直线运动测试
    ROS_INFO_STREAM("Move line test");
    KDL::Frame start_frame = ur5e->getCurrentFrame();
    KDL::Frame line_frame = start_frame;

    ros::Rate loop_rate(control_rate);
    uint64_t nsec0 = ros::Time::now().toNSec();
    while(ros::ok()){
        ros::spinOnce();
        uint64_t time_now = ros::Time::now().toNSec() - nsec0;
        double time_now_d = double(time_now) / 1e9;

        line_frame.p.data[1] += 0.01;
        ur5e->servoj_moveto(line_frame, 0.5, true);
        line_frame.p.data[1] -= 0.01;
        ur5e->servoj_moveto(line_frame, 1/control_rate, true);
        if (10<time_now_d)
            break;
        loop_rate.sleep();
    }


//    ros::Duration(0.5).sleep();
}
template<class T>
int length(T& arr)
{
//    std::cout << sizeof(arr[0]) << std::endl;
    //std::cout << sizeof(arr) << std::endl;
    return sizeof(arr) / sizeof(arr[0]);
}