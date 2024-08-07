/**
 * @file elspider3.h
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2022-11-29
 *
 * @copyright Copyright (C) 2022.
 *
 */
#pragma once

/* related header files */
#include "robot/robot_param.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "rclcpp/rclcpp.hpp"

/* internal project header files */
#include "common/utilities/basic_func.hpp"
#include "common/utilities/basic_math.hpp"

inline RobotParam buildElspider3(void)
{
    using namespace basic_math;

    RobotParam param;
    std::string interface_type{};
    // try
    // {
    //     GET_ROS_PARAM("/hexapod/interface_type", interface_type);
    // }
    // catch (const std::exception &e)
    // {
    interface_type = "hardware";
    // }

    /* leg */
    param.leg_dof = 3;
    param.leg_num = 6;
    param.leg_mirror_coe.resize(param.leg_num, 1);
    param.leg_mirror_coe << 1, 1, 1, -1, -1, -1;

    /* robot */
    param.robot_name = "elspider3";
    param.robot_type = "hexapod";
    param.link_length << 0.3, 0.16, 0.3; // 0.26; // 0.32

    // rf--rm--rb--lf--lm--lb
    param.hip_offset.resize(param.leg_dof, param.leg_num);
    param.hip_offset.col(0) = Eigen::Vector3d(0.6, -0.3, 0.);
    param.hip_offset.col(1) = Eigen::Vector3d(0., -0.45, 0.);
    param.hip_offset.col(2) = Eigen::Vector3d(-0.6, -0.3, 0.);
    param.hip_offset.col(3) = Eigen::Vector3d(0.6, 0.3, 0.);
    param.hip_offset.col(4) = Eigen::Vector3d(0., 0.45, 0.);
    param.hip_offset.col(5) = Eigen::Vector3d(-0.6, 0.3, 0.);

    param.mass = 72;

    double truck_ixx = 2.673775;
    double truck_ixy = -0.361433135;
    double truck_ixz = 761.1E-06;
    double truck_iyy = 7.163591350;
    double truck_iyz = -15.915E-06;
    double truck_izz = 9.49691187;

    param.trunk_inertia << truck_ixx, truck_ixy, truck_ixz, truck_ixy, truck_iyy, truck_iyz,
        truck_ixz, truck_iyz, truck_izz;

    param.max_step_size << 0.3, 0.1, 0.15;

    param.with_foot_force_sensor = false;

    /* joint */
    if (interface_type == "hardware")
    {
        param.joint_calibration_pos.resize(param.leg_dof, param.leg_num);
        // param.joint_calibration_pos =
        //     Eigen::Matrix<double, 3, 1>(deg2rad(-95), deg2rad(120),
        //     deg2rad(-150))
        //         .replicate(1, param.leg_num);
        param.joint_calibration_pos =
            Eigen::Matrix<double, 3, 1>(deg2rad(-65), deg2rad(120), deg2rad(-160))
                .replicate(1, param.leg_num);
        // param.joint_calibration_pos(0, 0) = deg2rad(-69);
        // param.joint_calibration_pos(0, 3) = deg2rad(-69);
        param.joint_calibration_pos(0, 1) = deg2rad(-85);
        param.joint_calibration_pos(0, 4) = deg2rad(-85);

        param.joint_calibration_pos(0, 0) = deg2rad(-80);
        param.joint_calibration_pos(0, 3) = deg2rad(-80);

        // param.joint_calibration_pos(1, 4) = deg2rad(127.5);
        // param.joint_calibration_pos(2, 4) = deg2rad(-155);

        // param.joint_calibration_pos(1, 0) = deg2rad(127.5);
        // param.joint_calibration_pos(1, 2) = deg2rad(127.5);

        // param.joint_calibration_pos(1, 5) = deg2rad(115);
        // param.joint_calibration_pos(1, 3) = deg2rad(115);

        param.joint_protect_pos_max << deg2rad(97), deg2rad(130), deg2rad(0);
        param.joint_protect_pos_min << deg2rad(-97), deg2rad(-90), deg2rad(-162);
    }
    else if (interface_type == "gazebo")
    {
        param.joint_calibration_pos.resize(param.leg_dof, param.leg_num);
        param.joint_calibration_pos =
            Eigen::Matrix<double, 3, 1>(deg2rad(-95), deg2rad(120), deg2rad(-270))
                .replicate(1, param.leg_num);
        param.joint_protect_pos_max << deg2rad(95), deg2rad(120), deg2rad(0);
        param.joint_protect_pos_min << deg2rad(-95), deg2rad(-90), deg2rad(-270);
    }

    /* motor */
    param.external_reduction_ratio << 1, -1, 1;
    param.motor_reduction_ratio << 1, 1, 1;
    // SETZ90 motor, 108 * 0.75 = 81
    // SETZ120 motor, 180 * 0.75 = 135
    param.max_motor_torque << 135, 81, 81;
    param.motor_init_type = MotorInitType::kAutoCalibration;

    return param;
}
