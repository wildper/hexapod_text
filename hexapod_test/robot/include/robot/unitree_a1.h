/**
 * @file unitree_a1.h
 * @brief
 * @author Haoyu Wang (qrpucp@qq.com)
 * @date 2023-04-18
 *
 * @copyright Copyright (C) 2023.
 *
 */
#pragma once

/* related header files */
#include "robot/robot_param.h"

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "common/utilities/basic_math.hpp"

/*
                head
    (leg2)FL ----------- FR(leg0)
             |    x    |
             | y__|    |
             |         |
    (leg3)RL ----------- RR(leg1)
*/

inline RobotParam buildUnitreeA1(void)
{
    using namespace basic_math;

    RobotParam param;

    /* leg */
    param.leg_dof = 3;
    param.leg_num = 4;
    // param.leg_mirror_coe.resize(param.leg_num, 1);
    // param.leg_mirror_coe << -1, -1, 1, 1;

    /* robot */
    param.robot_name = "unitree_a1";
    param.robot_type = "quadruped";
    param.link_length << 0, 0.21, 0.21;

    // FR--RR--FL--RL
    param.hip_offset.resize(param.leg_dof, param.leg_num);
    param.hip_offset.col(0) = Eigen::Vector3d(0.1805, -0.047, 0);
    param.hip_offset.col(1) = Eigen::Vector3d(-0.1805, -0.047, 0);
    param.hip_offset.col(2) = Eigen::Vector3d(0.1805, 0.047, 0);
    param.hip_offset.col(3) = Eigen::Vector3d(-0.1805, 0.047, 0);

    param.mass = 12.2;

    double truck_ixx = 0.0158533;
    double truck_ixy = -0.0000366;
    double truck_ixz = -0.0000611;
    double truck_iyy = 0.0377999;
    double truck_iyz = -0.0000275;
    double truck_izz = 0.0456542;

    param.trunk_inertia << truck_ixx, truck_ixy, truck_ixz, truck_ixy, truck_iyy, truck_iyz,
        truck_ixz, truck_iyz, truck_izz;

    // param.max_step_size << 0.175, 0.05, 0.075;
    param.max_step_size << 0.175, 0.05, 0.1; // 0.15

    param.with_foot_force_sensor = true;

    /* joint */
    param.joint_calibration_pos = Eigen::MatrixXd::Zero(param.leg_dof, param.leg_num);
    param.joint_protect_pos_max << deg2rad(180), deg2rad(180), deg2rad(180);
    param.joint_protect_pos_min << deg2rad(-180), deg2rad(-180), deg2rad(-180);

    /* motor */
    // param.external_reduction_ratio << 1, 1, 1;
    // param.motor_reduction_ratio << 9.1, 9.1, 9.1;
    // Unitree A1 motor, 33.5 * 0.8 = 26.8
    param.max_motor_torque << 26.8, 26.8, 26.8;
    param.motor_init_type = MotorInitType::kAbsoluteEncoder;

    return param;
}
