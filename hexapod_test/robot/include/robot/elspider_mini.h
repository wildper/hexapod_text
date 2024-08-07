/**
 * @file elspider_mini.h
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

/* internal project header files */
#include "common/utilities/basic_math.hpp"

/*
               head
  (leg3)LF ------------- RF(leg0)
           |           |
           |     x     |
  (leg4)LM |  y__|     | RM(leg1)
           |           |
           |           |
  (leg5)LB ------------- RB(leg2)
*/

inline RobotParam buildElspiderMini(void)
{
    using namespace basic_math;

    RobotParam param;

    /* leg */
    param.leg_dof = 3;
    param.leg_num = 6;
    param.leg_mirror_coe.resize(param.leg_num, 1);
    param.leg_mirror_coe << 1, 1, 1, -1, -1, -1;

    /* robot */
    param.robot_name = "elspider_mini";
    param.robot_type = "hexapod";
    param.link_length << 0.15, 0.162, 0.27;

    // rf--rm--rb--lf--lm--lb
    param.hip_offset.resize(param.leg_dof, param.leg_num);
    param.hip_offset.col(0) = Eigen::Vector3d(0.42311, -0.2, 0.003);
    param.hip_offset.col(1) = Eigen::Vector3d(0., -0.24, 0.003);
    param.hip_offset.col(2) = Eigen::Vector3d(-0.42311, -0.2, 0.003);
    param.hip_offset.col(3) = Eigen::Vector3d(0.42311, 0.2, 0.003);
    param.hip_offset.col(4) = Eigen::Vector3d(0., 0.24, 0.003);
    param.hip_offset.col(5) = Eigen::Vector3d(-0.42311, 0.2, 0.003);
    // temp
    for (int i = 0; i < param.leg_num; ++i)
    {
        param.hip_offset(2, i) -= 0.175;
    }

    param.mass = 30; // 20; // 25;

    double truck_ixx = 0.383316503;
    double truck_ixy = 0.005624294;
    double truck_ixz = 0.000867617;
    double truck_iyy = 1.184788202;
    double truck_iyz = -0.000642783;
    double truck_izz = 1.510253542;

    param.trunk_inertia << truck_ixx, truck_ixy, truck_ixz, truck_ixy, truck_iyy, truck_iyz,
        truck_ixz, truck_iyz, truck_izz;

    // param.max_step_size << 0.175, 0.05, 0.075;
    param.max_step_size << 0.175, 0.05, 0.1; // 0.15

    param.with_foot_force_sensor = false;

    /* joint */
    param.joint_calibration_pos.resize(param.leg_dof, param.leg_num);
    param.joint_calibration_pos =
        Eigen::Matrix<double, 3, 1>(deg2rad(-53.88), deg2rad(96.187), deg2rad(-204.93))
            .replicate(1, 6);
    param.joint_protect_pos_max << deg2rad(53.88), deg2rad(96.187), deg2rad(-30);
    param.joint_protect_pos_min << deg2rad(-53.88), deg2rad(10), deg2rad(-204.93);

    /* motor */
    param.external_reduction_ratio << 1, 1, 1;
    param.motor_reduction_ratio << 9.1, 9.1, 9.1;
    // Unitree A1 motor, 33.5 * 0.8 = 26.8
    param.max_motor_torque << 26.8, 26.8, 26.8;
    param.motor_init_type = MotorInitType::kAutoCalibration;

    return param;
}
