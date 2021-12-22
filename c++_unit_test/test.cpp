#define _CRT_SECURE_NO_WARNINGS
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include "JAKAZuRobot.h"
#include <iostream>
#include <stdio.h>

BOOST_AUTO_TEST_SUITE(jaka_sdk_test)

std::string ipaddr = "192.168.2.76"; //161

#define PI 3.1415926

//获取机器人状态信息
BOOST_AUTO_TEST_CASE(get_robot_status)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    //RobotStatus结构体中存放机器人状态数据
    RobotStatus robotSts;
    test.get_robot_status(&robotSts);
    //当errcode不等于0时，说明机器人运行状态发生异常,发生异常时保存错误码，可根据错误码查询引起机器人异常的原因
    BOOST_CHECK_EQUAL(robotSts.errcode, 0);
    //检测机器人是否运动到指定位置
    BOOST_CHECK_EQUAL(robotSts.inpos, 0);
    //检测机器人是否上电
    BOOST_CHECK_EQUAL(robotSts.powered_on, 0);
    //检测机器人是否上使能
    BOOST_CHECK_EQUAL(robotSts.enabled, 0);
    //检测机器人是否发生碰撞
    BOOST_CHECK_EQUAL(robotSts.protective_stop, 0);
    //检测机器人急停开关是否被按下
    BOOST_CHECK_EQUAL(robotSts.emergency_stop, 0);
    //检测机器人是否处于拖拽模式
    BOOST_CHECK_EQUAL(robotSts.drag_status, 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//测试带加速度参数的运动控制
BOOST_AUTO_TEST_CASE(robot_move_test)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    CartesianPose end, mid;
    end.tran.x = 386.8; end.tran.y = -211.7; end.tran.z = 221;
    end.rpy.rx = -109 * PI / 180; end.rpy.ry = -85 * PI / 180; end.rpy.rz = -73 * PI / 180;
    mid.tran.x = 367; mid.tran.y = -192; mid.tran.z = 221;
    mid.rpy.rx = -109 * PI / 180; mid.rpy.ry = -85 * PI / 180; mid.rpy.rz = -73 * PI / 180;
    OptionalCond* q = nullptr;
    BOOST_CHECK_EQUAL(test.circular_move(&end, &mid, ABS, FALSE, 50, 30, 5, q), 0);
    JointValue jointPos;
    jointPos.jVal[0] = 0 * PI / 180; jointPos.jVal[1] = 90 * PI / 180; jointPos.jVal[2] = -90 * PI / 180;
    jointPos.jVal[3] = 90 * PI / 180; jointPos.jVal[4] = 90 * PI / 180; jointPos.jVal[5] = 0 * PI / 180;
    OptionalCond* p = nullptr;
    BOOST_CHECK_EQUAL(test.joint_move(&jointPos, ABS, TRUE, 1, 300, 1, p), 0);
    CartesianPose end_pos, mid_pos;
    end_pos.tran.x = 357; end_pos.tran.y = 120; end_pos.tran.z = 461;
    end_pos.rpy.rx = -167 * PI / 180; end_pos.rpy.ry = 0 * PI / 180; end_pos.rpy.rz = -90 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, ABS, TRUE, 50.0, 300, 1, p), 0);
    BOOST_CHECK_EQUAL(test.joint_move(&jointPos, ABS, TRUE, 1, 300, 1, p), 0);
    mid_pos = end_pos;
    end_pos.tran.x = 222; end_pos.tran.y = 268; end_pos.tran.z = 490;
    end_pos.rpy.rx = -162 * PI / 180; end_pos.rpy.ry = 0 * PI / 180; end_pos.rpy.rz = -60 * PI / 180;
    BOOST_CHECK_EQUAL(test.circular_move(&end_pos, &mid_pos, ABS, TRUE, 50, 300, 1, p), 0);
    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//连续点位运动功能测试
BOOST_AUTO_TEST_CASE(continue_move_test)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    //设置第一个点坐标
    CartesianPose firstPoint;
    firstPoint.tran.x = 423; firstPoint.tran.y = 121; firstPoint.tran.z = 373;
    firstPoint.rpy.rx = 180 * PI / 180; firstPoint.rpy.ry = 0 * PI / 180; firstPoint.rpy.rz = -90 * PI / 180;
    //设置逆解参考解空间
    JointValue refPos, resultPos;
    refPos.jVal[0] = 0 * PI / 180; refPos.jVal[1] = 90 * PI / 180; refPos.jVal[2] = -90 * PI / 180;
    refPos.jVal[3] = 90 * PI / 180; refPos.jVal[4] = 90 * PI / 180; refPos.jVal[5] = 0 * PI / 180;
    //逆解
    BOOST_CHECK_EQUAL(test.kine_inverse(&refPos, &firstPoint, &resultPos),0);
    //关节运动到第一个点
    BOOST_CHECK_EQUAL(test.joint_move(&resultPos, ABS, TRUE, 1), 0);

    //此处连续运行4个点位
    CartesianPose cart[4];
    for (int i = 0; i < 4; ++i)
    {
        cart[i].tran.x = firstPoint.tran.x ;
        cart[i].tran.y = firstPoint.tran.y ;
        cart[i].tran.z = firstPoint.tran.z;
        cart[i].rpy.rx = firstPoint.rpy.rx + (i + 1) * 0.5;
        cart[i].rpy.ry = firstPoint.rpy.ry + (i + 1) * 0.5;
        cart[i].rpy.rz = firstPoint.rpy.rz + (i + 1) * 0.5;
    }

    for (int i = 0; i < 4; ++i)
    {
        BOOST_CHECK_EQUAL(test.linear_move(&cart[i], ABS,false,20),0);
    }

    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//轨迹复现功能测试
BOOST_AUTO_TEST_CASE(track_test)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    RobotStatus status;
    //test.drag_mode_enable(1);
    test.get_robot_status(&status);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    //重命名第一个数据文件
    char name[20] = "test";
    //加载轨迹复现数据，轨迹复现数据的加载需要在文件夹名字前加上track/
    char load_data[100];
    sprintf(load_data, "track/%s", name);
    BOOST_CHECK_EQUAL(test.program_load(load_data), 0);
    //运行轨迹复现数据
    BOOST_CHECK_EQUAL(test.program_run(), 0);
    BOOST_CHECK_EQUAL(test.program_abort(), 0);
}

//轨迹复现功能测试
BOOST_AUTO_TEST_CASE(traj_reprduce)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    //关节阻塞绝对运动
    JointValue joint_pos;
    joint_pos.jVal[0] = 0 * PI / 180; joint_pos.jVal[1] = 90 * PI / 180; joint_pos.jVal[2] = -90 * PI / 180;
    joint_pos.jVal[3] = 90 * PI / 180; joint_pos.jVal[4] = 90 * PI / 180; joint_pos.jVal[5] = 0 * PI / 180;
    BOOST_CHECK_EQUAL(test.joint_move(&joint_pos, ABS, TRUE, 0.5), 0);
    //设置轨迹复现配置参数
    TrajTrackPara ttp;
    ttp.acc = 300;
    ttp.vel = 1000;
    ttp.xyz_interval = 0.1;
    ttp.rpy_interval = 0.1;
    BOOST_CHECK_EQUAL(test.set_traj_config(&ttp), 0);
    //查询轨迹复现配置参数
    BOOST_CHECK_EQUAL(test.get_traj_config(&ttp), 0);
    BOOST_CHECK_EQUAL(ttp.acc, 300);
    BOOST_CHECK_EQUAL(ttp.vel, 1000);
    BOOST_CHECK_EQUAL(ttp.xyz_interval, 0.1);
    BOOST_CHECK_EQUAL(ttp.rpy_interval, 0.1);
    //查询目前控制器中存储的轨迹复现文件的文件名
    MultStrStorType msst;
    BOOST_CHECK_EQUAL(test.get_exist_traj_file_name(&msst), 0);
    //显示文件名
    for (int i = 0; i < msst.len; ++i)
    {
        
        std::cout << msst.name[i] << std::endl;
    }

    //重命名第一个数据文件
    char name[20] = "jaka123";   
    BOOST_CHECK_EQUAL(test.rename_traj_file_name(msst.name[0],name), 0);
    //清空filename
    for (int i = 0; i < msst.len; ++i)
    {
        msst.name[i][0] = '\0';
    }
    BOOST_CHECK_EQUAL(test.get_exist_traj_file_name(&msst), 0);
    
    //删除名为jaka的数据文件
    BOOST_CHECK_EQUAL(test.remove_traj_file(name), 0);
    //清空filename
    for (int i = 0; i < msst.len; ++i)
    {
        msst.name[i][0] = '\0';
    }
    BOOST_CHECK_EQUAL(test.get_exist_traj_file_name(&msst), 0);
    
    //开始采集数据，应在数据采集开始后，打断点暂停，然后拖拽机器人，以测试轨迹复现功能
    BOOST_CHECK_EQUAL(test.set_traj_sample_mode(TRUE,name), 0);
    //查询数据采集状态，如果正在采集拖拽轨迹，不能再次开启数据采集开关set_traj_sample_mode
    BOOL sample_status;
    BOOST_CHECK_EQUAL(test.get_traj_sample_status(&sample_status), 0);
    BOOST_CHECK_EQUAL(sample_status, 1);
    //结束采集数据，应在数据采集开始后，打断点暂停，然后拖拽机器人，以测试轨迹复现功能
    BOOST_CHECK_EQUAL(test.set_traj_sample_mode(FALSE,name), 0);
    BOOST_CHECK_EQUAL(test.get_traj_sample_status(&sample_status), 0);
    BOOST_CHECK_EQUAL(sample_status, 0);

    //加载轨迹复现数据，轨迹复现数据的加载需要在文件夹名字前加上track/
    char load_data[100];
    sprintf(load_data, "track/%s", name);
    BOOST_CHECK_EQUAL(test.program_load(load_data), 0);
    //运行轨迹复现数据
    BOOST_CHECK_EQUAL(test.program_run(), 0);

    //修改速度和加速度
    ttp.acc = 500;
    ttp.vel = 1500;
    ttp.xyz_interval = 0.1;
    ttp.rpy_interval = 0.1;
    BOOST_CHECK_EQUAL(test.set_traj_config(&ttp), 0);
    
    //重新生成执行脚本
    BOOST_CHECK_EQUAL(test.generate_traj_exe_file(name), 0);

    //加载轨迹复现数据，轨迹复现数据的加载需要在文件夹名字前加上track/
    BOOST_CHECK_EQUAL(test.program_load(load_data), 0);
    //运行轨迹复现数据
    BOOST_CHECK_EQUAL(test.program_run(), 0);
    

    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//逆解与直线运动测试
BOOST_AUTO_TEST_CASE(kine_inverse_cart_move)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    //逆解
    JointValue joint_pos,ref_joint_pos;
    CartesianPose end_pos;
    end_pos.tran.x = -161; end_pos.tran.y = -325; end_pos.tran.z = 374;
    end_pos.rpy.rx = -178 * PI / 180; end_pos.rpy.ry = 1.8 * PI / 180; end_pos.rpy.rz = 179 * PI / 180;   

    ref_joint_pos.jVal[0] = 0 * PI / 180; ref_joint_pos.jVal[1] = 60 * PI / 180; ref_joint_pos.jVal[2] = -90 * PI / 180;
    ref_joint_pos.jVal[3] = 90 * PI / 180; ref_joint_pos.jVal[4] = 90 * PI / 180; ref_joint_pos.jVal[5] = 0 * PI / 180;
    BOOST_CHECK_EQUAL(test.kine_inverse(&ref_joint_pos,&end_pos,&joint_pos), 0);
    //关节阻塞绝对运动   
    BOOST_CHECK_EQUAL(test.joint_move(&joint_pos, ABS, TRUE, 0.5), 0);
    
    //直线阻塞绝对运动
    end_pos.tran.x = -624.8; end_pos.tran.y = -413.7; end_pos.tran.z = 374;
    end_pos.rpy.rx = 180 * PI / 180; end_pos.rpy.ry = 0 * PI / 180; end_pos.rpy.rz = 180 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, ABS, TRUE, 50.0), 0);

    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//关节与直线运动测试
BOOST_AUTO_TEST_CASE(joint_cart_move)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    //关节阻塞绝对运动
    JointValue joint_pos;
    joint_pos.jVal[0] = 0 * PI / 180; joint_pos.jVal[1] = 90 * PI / 180; joint_pos.jVal[2] = -90 * PI / 180;
    joint_pos.jVal[3] = 90 * PI / 180; joint_pos.jVal[4] = 90 * PI / 180; joint_pos.jVal[5] = 0 * PI / 180;
    BOOST_CHECK_EQUAL(test.joint_move(&joint_pos, ABS, TRUE, 0.5),0);
    //关节非阻塞绝对运动
    joint_pos.jVal[0] = 0 * PI / 180; joint_pos.jVal[1] = 60 * PI / 180; joint_pos.jVal[2] = -90 * PI / 180;
    joint_pos.jVal[3] = 90 * PI / 180; joint_pos.jVal[4] = 90 * PI / 180; joint_pos.jVal[5] = 0 * PI / 180;
    BOOST_CHECK_EQUAL(test.joint_move(&joint_pos, ABS, FALSE, 0.5), 0);
    //关节阻塞相对运动
    joint_pos.jVal[0] = 0 * PI / 180; joint_pos.jVal[1] = 120 * PI / 180; joint_pos.jVal[2] = 0 * PI / 180;
    joint_pos.jVal[3] = 0 * PI / 180; joint_pos.jVal[4] = 0 * PI / 180; joint_pos.jVal[5] = 0 * PI / 180;
    BOOST_CHECK_EQUAL(test.joint_move(&joint_pos, INCR, TRUE, 0.5), 0);
    //关节非阻塞相对运动
    joint_pos.jVal[0] = 0 * PI / 180; joint_pos.jVal[1] = 30 * PI / 180; joint_pos.jVal[2] = 0 * PI / 180;
    joint_pos.jVal[3] = 0 * PI / 180; joint_pos.jVal[4] = 0 * PI / 180; joint_pos.jVal[5] = 0 * PI / 180;
    BOOST_CHECK_EQUAL(test.joint_move(&joint_pos, INCR, FALSE, 0.5), 0);

    //直线阻塞绝对运动
    CartesianPose end_pos;
    end_pos.tran.x = -19.6010; end_pos.tran.y = -374.2140; end_pos.tran.z = 121.2920;
    end_pos.rpy.rx = -179.96862 * PI / 180; end_pos.rpy.ry = 1.087302 * PI / 180; end_pos.rpy.rz = -177.60179 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, ABS, TRUE, 50.0), 0);
    //直线非阻塞绝对运动
    end_pos.tran.x = -19.6010; end_pos.tran.y = -374.2140; end_pos.tran.z = 164;
    end_pos.rpy.rx = -179.96862 * PI / 180; end_pos.rpy.ry = 1.087302 * PI / 180; end_pos.rpy.rz = -177.60179 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, ABS, FALSE, 50.0), 0);
    //直线阻塞相对运动
    end_pos.tran.x = 0; end_pos.tran.y = 0; end_pos.tran.z = 50;
    end_pos.rpy.rx = 100 * PI / 180; end_pos.rpy.ry = 100 * PI / 180; end_pos.rpy.rz = 100 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, INCR, TRUE, 50.0), 0);
    //直线非阻塞相对运动
    end_pos.tran.x = 0; end_pos.tran.y = 0; end_pos.tran.z = -50;
    end_pos.rpy.rx = -100 * PI / 180; end_pos.rpy.ry = -100 * PI / 180; end_pos.rpy.rz = -100 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, INCR, FALSE, 50.0), 0);

    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//复现客户现场反馈指令阻塞模式下停留时间长的问题
BOOST_AUTO_TEST_CASE(test_block)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    //直线运动
    CartesianPose end_pos;
    end_pos.tran.x = -19.6010; end_pos.tran.y = -374.2140; end_pos.tran.z = 121.2920;
    end_pos.rpy.rx = -179.96862 * PI / 180; end_pos.rpy.ry = 1.087302 * PI / 180; end_pos.rpy.rz = -177.60179 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, ABS, TRUE, 50.0), 0);
    //设置控制器面板数字输出信号3号引脚为0
    BOOST_CHECK_EQUAL(test.set_digital_output(IO_CABINET, 3, 0), 0);

    end_pos.tran.x = -19.6014860; end_pos.tran.y = -364.2139570; end_pos.tran.z = 136.2916210;
    end_pos.rpy.rx = -179.9686248 * PI / 180; end_pos.rpy.ry = 1.087302 * PI / 180; end_pos.rpy.rz = -177.60179 * PI / 180;
    BOOST_CHECK_EQUAL(test.linear_move(&end_pos, ABS, TRUE, 50.0), 0);

    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

//上电上使能 下电下使能
BOOST_AUTO_TEST_CASE(power_enable)
{
    JAKAZuRobot test;
    BOOST_CHECK_EQUAL(test.login_in(ipaddr.c_str()), 0);
    BOOST_CHECK_EQUAL(test.power_on(), 0);
    BOOST_CHECK_EQUAL(test.enable_robot(), 0);
    BOOST_CHECK_EQUAL(test.disable_robot(), 0);
    BOOST_CHECK_EQUAL(test.power_off(), 0);
    BOOST_CHECK_EQUAL(test.login_out(), 0);
}

BOOST_AUTO_TEST_SUITE_END()