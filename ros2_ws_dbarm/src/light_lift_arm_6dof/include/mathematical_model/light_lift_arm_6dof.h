#ifndef LIGHT_LIFT_ARM_6DOF_H
#define LIGHT_LIFT_ARM_6DOF_H

#include "math.h"
#include <vector>
#include <iostream>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
// #include "geometry_msgs/msg/wrench.hpp"
#include <Eigen/Core>
#include <Eigen/QR>  
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <urdf/model.h>

#include <cmath>


#define ARM_DOF 6

class LlArm6dof
{
public:
    LlArm6dof();
    ~LlArm6dof();

    float desir_joint_pos[3] = {0};
    float desir_joint_vel[3] = {0};
    float desir_joint_acc[3] = {0};
    float desir_joint_tau[3] = {0};
    float desir_motor_pos[3] = {0};
    float desir_motor_vel[3] = {0};
    float desir_motor_tau[3] = {0};
    float desir_end_efect_pos_default[ARM_DOF] = {0.4, 0.00, 0.2, 0, 0.0 ,0};
    float desir_joint_pos_default [ARM_DOF] = {0};
    float desir_end_efect_pos[ARM_DOF] = {0};
    float desir_end_efect_vel[ARM_DOF] = {0};
    float desir_end_efect_tau[ARM_DOF] = {0};
    float desir_end_efect_acc[ARM_DOF] = {0};
    float desir_joint_pos_protect_max[ARM_DOF] = { 1.4, 1.55, 1.50, 1.56, 1.4, 1.5};
    float desir_joint_pos_protect_min[ARM_DOF] = {-1.4,-1.5, -1.5, -1.56, -1.5, -1.5};

    float desir_grp_default = 100;
    float desir_grp = desir_grp_default;//0-230=闭合-张开
    bool is_load = false;
    bool init_load = false;

    float current_joint_pos[ARM_DOF] = {0};
    float current_joint_vel[ARM_DOF] = {0};
    float current_joint_acc[ARM_DOF] = {0};
    float current_joint_tau[ARM_DOF] = {0};
    float current_motor_pos[ARM_DOF] = {0};
    float current_motor_vel[ARM_DOF] = {0};
    float current_motor_tau[ARM_DOF] = {0};
    float current_motor_acc[ARM_DOF] = {0};
    float current_end_efect_pos[ARM_DOF] = {0};
    float current_end_efect_vel[ARM_DOF] = {0};
    float current_end_efect_tau[ARM_DOF] = {0};

    float end_force_value = -2.6;
    float end_force_orientation[6] = {0, 0, 1, 0, 0, 0};//x轴正方向,只能是正数

    float joint_control_tau[ARM_DOF] = {0};
    float joint_gravity_tau[ARM_DOF] = {0};
    float joint_STSMC_tau[ARM_DOF] = {0};
    float joint_impedance_tau[ARM_DOF] = {0};
    float joint_pid_tau[ARM_DOF] = {0};
    float joint_admittance_tau[ARM_DOF] = {0};
    float motor_direction[ARM_DOF] = {1, -1, 1, -1, 1, 1};

    float external_force[6] = {0.0};

    std::vector<float> motorState;

    float arm_link_lenth[6] = {0};

    std::string cmd = "nan";


    // 创建逆动力学求解器对象
    //    KDL::ChainDynParam dyn_param(chain, gravity);
    std::shared_ptr<KDL::ChainDynParam> dyn_param;

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_myarm;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_lma;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_myarm;
    std::shared_ptr<KDL::ChainJntToJacDotSolver> ik_solver_acc;

    KDL::Chain kdl_chain;
 
    KDL::JntArray desir_joint_positions;
    KDL::JntArray desir_joint_velocities;
    KDL::JntArray desir_joint_tauqes;
    KDL::JntArray desir_joint_acceleration;
    KDL::Frame desir_end_effector_frame;
    KDL::Twist desir_end_effector_dot;
    KDL::Twist desir_end_effector_ddot;
    KDL::Jacobian desir_jacobian;
    KDL::JntArray STSMC_joint_tauqes;
    KDL::JntArray gravity_joint_tauqes;
    KDL::JntArrayVel desir_q_pos_vel;//用于计算逆加速度函数的输入
    KDL::Wrench desir_end_effector_wrench;
  
    KDL::JntArray current_joint_positions;
    KDL::JntArray current_joint_velocities;
    KDL::JntArray current_joint_tauqes;
    KDL::JntArray current_joint_acceleration;
    KDL::Frame current_end_effector_frame;
    KDL::Twist current_end_effector_dot;
    KDL::Twist current_end_effector_ddot;
    KDL::Jacobian current_jacobian;
    KDL::JntArrayVel current_q_pos_vel;//用于计算逆加速度函数的输入
    KDL::Wrench current_end_effector_wrench;

    void joint2end_force();

    void joint2end_vel();

    void updatArmState(float motor_pos[ARM_DOF], float motor_vel[ARM_DOF], float motor_tau[ARM_DOF]);

    void joint2motor(float q[ARM_DOF],float dq[ARM_DOF], float ddq[ARM_DOF],  float tau_joint[ARM_DOF],  float motor_pos[ARM_DOF],float motor_vel[ARM_DOF], float motor_acc[ARM_DOF],  float tau_motor[ARM_DOF]);

    bool now2aim(float sta_pos[ARM_DOF], float end_pos[ARM_DOF], float now_pos[ARM_DOF], float steps, int delay);

    void update_end_effector_trajectory(float realtime);

    void computeInverseDynamics(const KDL::JntArray& q, const KDL::JntArray& q_dot, const KDL::JntArray& q_dotdot, KDL::JntArray& torques);

    void comput_joint_acceleration(const KDL::JntArray& q, const KDL::JntArray& q_dot, KDL::Twist& desir_end_effector_ddot,  KDL::JntArray& q_dotdot);

    void comput_ik( KDL::Frame &frame);

    void updateLinkMass(KDL::Chain& chain, const std::string& link_name, double new_mass);

private:


 
    
};

#endif // ARMCONTROL_H
