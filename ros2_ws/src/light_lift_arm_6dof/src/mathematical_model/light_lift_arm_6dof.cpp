#include "mathematical_model/light_lift_arm_6dof.h"
#include <cmath>

LlArm6dof::LlArm6dof(){

    motorState.reserve(ARM_DOF*3);
    
    // RCLCPP_INFO(this->get_logger(), "Orocos KDL Test Node Started");
    printf("Orocos KDL Test Node Started\n");

 
    // 加载URDF模型
    // std::string urdf_file = "src/kdl_test/urdf/testInertiaRPYmodel1.urdf";
    std::string urdf_file = "src/light_lift_arm_6dof/urdf/first_robot.urdf";
    

    urdf::Model model;
    if (!model.initFile(urdf_file))
    {
        // RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file");
        printf("Failed to parse URDF file\n");
        return;
    }

    // 解析URDF并生成KDL树
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        // RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
        printf("Failed to construct KDL tree\n");
        return;
    }

    // 从KDL树生成KDL链
    if (!kdl_tree.getChain("base_link", "link6", kdl_chain))
    {
        // RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain");
        printf("Failed to get KDL chain\n");
        return;
    }

    for (unsigned int i = 0; i < kdl_chain.getNrOfSegments(); ++i)
    {
        const KDL::Segment& segment = kdl_chain.getSegment(i);
        const KDL::Frame& frame = segment.getFrameToTip();
        double length = frame.p.Norm(); // 获取该段的长度
        std::cout << "Segment " << i << " length: " << length << " meters" << std::endl;
        arm_link_lenth[i] = length;
    }


    Eigen::Matrix<double, 6, 1> _L = Eigen::Matrix<double, 6, 1>::Identity();
    for (size_t i = 0; i < 6; i++)
    {
        _L(i) = 1;
    }
    
    printf("_L: [%f, %f, %f, %f, %f, %f]\n", _L(0), _L(1), _L(2), _L(3), _L(4), _L(5));

    // printf(":%f",_L);
    // 创建正运动学求解器
    fk_solver_myarm = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain);

    // 创建逆运动学求解器
    ik_solver_vel = std::make_shared<KDL::ChainIkSolverVel_pinv>(kdl_chain);

    ik_solver_lma = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain, _L);

    // 初始化雅可比矩阵求解器
    jac_solver_myarm = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain);

    dyn_param = std::make_shared<KDL::ChainDynParam>(kdl_chain, KDL::Vector(0.0, 0.0, -9.81));

    ik_solver_acc = std::make_shared<KDL::ChainJntToJacDotSolver>(kdl_chain);


 
    desir_joint_positions = KDL::JntArray(kdl_chain.getNrOfJoints());
    desir_joint_velocities = KDL::JntArray(kdl_chain.getNrOfJoints());
    desir_joint_tauqes = KDL::JntArray(kdl_chain.getNrOfJoints());
    STSMC_joint_tauqes = KDL::JntArray(kdl_chain.getNrOfJoints());
    gravity_joint_tauqes = KDL::JntArray(kdl_chain.getNrOfJoints());
    desir_joint_acceleration = KDL::JntArray(kdl_chain.getNrOfJoints());
    desir_q_pos_vel = KDL::JntArrayVel(kdl_chain.getNrOfJoints());
    // desir_end_effector_dot = KDL::Twist(kdl_chain.getNrOfJoints());
    // desir_end_effector_ddot = KDL::Twist(kdl_chain.getNrOfJoints());
   

    current_joint_positions = KDL::JntArray(kdl_chain.getNrOfJoints());
    current_joint_velocities = KDL::JntArray(kdl_chain.getNrOfJoints());
    current_joint_tauqes = KDL::JntArray(kdl_chain.getNrOfJoints());
    current_joint_acceleration = KDL::JntArray(kdl_chain.getNrOfJoints());
    current_q_pos_vel = KDL::JntArrayVel(kdl_chain.getNrOfJoints());

 
    desir_joint_positions.data.setZero();
    desir_joint_velocities.data.setZero();
    desir_joint_tauqes.data.setZero();
    desir_joint_acceleration.data.setZero();
     
    current_joint_positions.data.setZero();
    current_joint_velocities.data.setZero();
    current_joint_tauqes.data.setZero();
    current_joint_acceleration.data.setZero();

    current_jacobian = KDL::Jacobian(kdl_chain.getNrOfJoints());
    desir_jacobian = KDL::Jacobian(kdl_chain.getNrOfJoints());

    desir_end_effector_frame.p = KDL::Vector(desir_end_efect_pos_default[0], desir_end_efect_pos_default[1], desir_end_efect_pos_default[2]);
    desir_end_effector_frame.M = KDL::Rotation::RPY(0.0, 0 + 0.0, 0.0);

    for (unsigned int i = 0; i < kdl_chain.getNrOfJoints(); i++)
    {
        // 关节位置
        desir_q_pos_vel.q(i) = 0.0;
        // 关节速度
        desir_q_pos_vel.qdot(i) = 0.0;

        // 关节位置
        current_q_pos_vel.q(i) = 0.0;
        // 关节速度
        current_q_pos_vel.qdot(i) = 0.0;

    }

    desir_end_effector_dot.vel = KDL::Vector(0.0, 0.0, 0.0);
    desir_end_effector_dot.rot = KDL::Vector(0.0, 0.0, 0.0);
    current_end_effector_ddot.vel = KDL::Vector(0.0, 0.0, 0.0);
    current_end_effector_ddot.rot = KDL::Vector(0.0, 0.0, 0.0);
    current_end_effector_wrench.force = KDL::Vector(0, 0, 0);
    current_end_effector_wrench.torque = KDL::Vector(0, 0, 0);
    desir_end_effector_wrench.force = KDL::Vector(0, 0, 0);
    desir_end_effector_wrench.torque = KDL::Vector(0, 0, 0);
}

LlArm6dof::~LlArm6dof() {}

/// @brief 计算末端力矩
/**
 * joint_torqe = J^T * F
 * 
 * F = (J^T)^-1 * joint_torqe
 * 
 * 使用的雅可比矩阵的伪逆
 * */  
void LlArm6dof::joint2end_force()
{
    // 计算 J^T 的伪逆（这里采用的是 QR 分解法，或者你可以使用 completeOrthogonalDecomposition()）
    Eigen::MatrixXd Jt = current_jacobian.data.transpose();

    Eigen::MatrixXd Jt_pinv = Jt.completeOrthogonalDecomposition().pseudoInverse();

    // 用伪逆计算末端力（wrench），结果是一个6维向量（前三个分量是力，后三个是力矩）
    //current_joint_tauqes是电机反馈回来的力矩，gravity_joint_tauqes是重力补偿力矩
    Eigen::VectorXd wrench = Jt_pinv * (current_joint_tauqes.data - gravity_joint_tauqes.data);

    // 如果需要，可以将 wrench 转换为 KDL::Wrench 类型：
    current_end_effector_wrench.force = KDL::Vector(wrench(0), wrench(1), wrench(2));

    current_end_effector_wrench.torque = KDL::Vector(wrench(3), wrench(4), wrench(5));
}

/**
 * 计算末端速度
 * end_effector_dot = J * joint_vel
 * current_jacobian, current_joint_velocities 都是实时更新的
*/
void LlArm6dof::joint2end_vel()
{
    for (unsigned int i = 0; i < 6; i++)
    {
        double component = 0.0;
        for (unsigned int j = 0; j < 6; j++)
        {
            component += current_jacobian(i, j) * current_joint_velocities(j);
        }
        // 前 3 行对应线速度，后 3 行对应角速度
        if (i < 3)
        {
            current_end_effector_dot.vel(i) = component;
        }
        else
        {
            current_end_effector_dot.rot(i - 3) = component;
        }
        
    }
}
/*************************************************/

/**
 * 在主函数实时的更新关节位置、速度、加速度、力矩
*/
void LlArm6dof::updatArmState(float motor_pos[ARM_DOF], float motor_vel[ARM_DOF], float motor_tau[ARM_DOF])
{
    //不更新关节加速度
    current_joint_pos[0] = motor_direction[0]*motor_pos[0];
    current_joint_pos[1] = motor_direction[1]*motor_pos[1];
    current_joint_pos[2] = motor_direction[2]*motor_pos[2];
    current_joint_pos[3] = motor_direction[3]*motor_pos[3];
    current_joint_pos[4] = motor_direction[4]*motor_pos[4];
    current_joint_pos[5] = motor_direction[5]*motor_pos[5];

    current_joint_vel[0] = motor_direction[0]*motor_vel[0];
    current_joint_vel[1] = motor_direction[1]*motor_vel[1];
    current_joint_vel[2] = motor_direction[2]*motor_vel[2];
    current_joint_vel[3] = motor_direction[3]*motor_vel[3];
    current_joint_vel[4] = motor_direction[4]*motor_vel[4];
    current_joint_vel[5] = motor_direction[5]*motor_vel[5];

    current_joint_tau[0] = motor_direction[0]*motor_tau[0];
    current_joint_tau[1] = motor_direction[1]*motor_tau[1];
    current_joint_tau[2] = motor_direction[2]*motor_tau[2];
    current_joint_tau[3] = motor_direction[3]*motor_tau[3];
    current_joint_tau[4] = motor_direction[4]*motor_tau[4];
    current_joint_tau[5] = motor_direction[5]*motor_tau[5];


    for (size_t i = 0; i < 6; i++)
    {
        current_joint_positions(i) = current_joint_pos[i];
        current_joint_velocities(i) = current_joint_vel[i];
        current_joint_tauqes(i) = current_joint_tau[i];
    }
    // std::cout << "current_joint_positions: " << current_joint_positions(0) << " " << current_joint_positions(1) << " " << current_joint_positions(2)<< std::endl;
      
    // std::cout << "manualServo_STSMC-motor_control_tau: " << motor_pos[3] << " " << motor_pos[4] << " " << motor_pos[5] << std::endl;

}

/**
 * 因为urdf的关节方向和实际电机的方向可能不一致，所以要转换一下
*/
void LlArm6dof::joint2motor(float q[ARM_DOF], float dq[ARM_DOF], float ddq[ARM_DOF], float tau_joint[ARM_DOF], float motor_pos[ARM_DOF], float motor_vel[ARM_DOF], float motor_acc[ARM_DOF], float tau_motor[ARM_DOF])
{
    for (size_t i = 0; i < ARM_DOF; i++)
    {
        motor_pos[i] = motor_direction[i] * q[i];
        motor_vel[i] = motor_direction[i] * dq[i];
        motor_acc[i] = motor_direction[i] * ddq[i];
        tau_motor[i] = motor_direction[i] * tau_joint[i];
    }
}
//*****************************************************************

/**
 * 插值
 * 现在到期望的误差只算一次
 * 输入：
 *      关节位置
 * 
 * 输出：
 *      关节位置
 *
 */
bool  LlArm6dof::now2aim(float sta_pos[ARM_DOF], float end_pos[ARM_DOF], float now_pos[ARM_DOF], float steps, int delay)
{
    float e[6] = {(end_pos[0] - sta_pos[0]), (end_pos[1] - sta_pos[1]), (end_pos[2] - sta_pos[2]), (end_pos[3] - sta_pos[3]), (end_pos[4] - sta_pos[4]), (end_pos[5] - sta_pos[5])};

    static int couter = 0;

    float start_pos[6] = {sta_pos[0], sta_pos[1], sta_pos[2], sta_pos[3], sta_pos[4], sta_pos[5]};

    static float inter_step = 0;

    float time_couter = (float)couter / 1000.0;

    inter_step += steps*0.5 * 0.001 * sin(time_couter);

    for (size_t i = 0; i < 6; i++)
    {
        now_pos[i] = start_pos[i] + e[i] * inter_step;
    }

    // std::cout << "------------" <<std::endl;

    // std::cout << "sta_pos: " << sta_pos[0] << " " << sta_pos[1] << " " << sta_pos[2]<< " " << sta_pos[3] << " " << sta_pos[4] << " " << sta_pos[5]<<std::endl;

    // std::cout << "start_pos: " << start_pos[0] << " " << start_pos[1] << " " << start_pos[2]<< " " << start_pos[3] << " " << start_pos[4] << " " << start_pos[5]<<std::endl;

    // std::cout << "end_pos  : " << end_pos[0] << " " << end_pos[1] << " " << end_pos[2]<< " " << end_pos[3] << " " << end_pos[4] << " " << end_pos[5]<<std::endl;

    // std::cout << "now_pos  : " << now_pos[0] << " " << now_pos[1] << " " << now_pos[2]<< " " << now_pos[3] << " " << now_pos[4] << " " << now_pos[5]<<std::endl;

    // std::cout << "e  : " << e[0] << " " << e[1] << " " << e[2]<< " " << e[3] << " " << e[4] << " " << e[5]<<std::endl;

    // std::cout << "couter  : " << couter << " " << sin(time_couter) << " " << inter_step<< " " << 0 << " " << 0<< " " << 0<<std::endl;

    if (couter >= 3140)
    {
        couter = 0;
        inter_step = 0;
        return true;
    }
    else
    {
        couter += steps;
        return false;
    }

}


/**
 * 在主函数中实时的更新某端轨迹的位置、速度、加速度
 * 输入的是时间，单位是秒
 * 这个函数在使用话题控制机械臂时，不能调用
*/
void LlArm6dof::update_end_effector_trajectory(float realtime){
    realtime = realtime * 0.0;

    float rate = 2 , amplitude = 0.03 ;
    float amplitude_x = 0.0;
    float amplitude_y = 0.0;
    float amplitude_z = 0.0;
    float amplitude_rot_x = 0.0;
    float amplitude_rot_y = 0.0;
    float amplitude_rot_z = 0.0;

    //外部话题控制，要把内部轨迹更新关闭


   desir_end_efect_pos[0] =  desir_end_efect_pos_default[0] + amplitude_x * sin(rate * realtime);

   desir_end_efect_pos[1] =  desir_end_efect_pos_default[1]+ amplitude_y * cos(rate * realtime);

   desir_end_efect_pos[2] =  desir_end_efect_pos_default[2] + amplitude_z * cos(rate * realtime);

   desir_end_efect_vel[0] = amplitude_x * rate *cos(rate * realtime);

   desir_end_efect_vel[1] = -amplitude_y * rate *sin(rate * realtime);

   desir_end_efect_vel[2] = -amplitude_z * rate *sin(rate * realtime);

   desir_end_efect_acc[0] = -amplitude_x * rate *rate *sin(rate * realtime);

   desir_end_efect_acc[1] = -amplitude_y * rate *rate *cos(rate * realtime);

   desir_end_efect_acc[2] = -amplitude_z * rate *rate *cos(rate * realtime);

   for (size_t i = 0; i < 3; i++)
   {
    desir_end_efect_pos[3+i] = 0;
    desir_end_efect_vel[3+i] = 0;
    desir_end_efect_acc[3+i] = 0;
   }

   desir_end_efect_pos[3] = desir_end_efect_pos_default[3] + amplitude_rot_x*sin(rate * realtime);
   desir_end_efect_pos[4] = desir_end_efect_pos_default[4] + amplitude_rot_y*cos(rate * realtime);
   desir_end_efect_pos[5] = desir_end_efect_pos_default[5] + amplitude_rot_z*sin(rate * realtime);

   desir_end_efect_vel[3] = amplitude_rot_x*rate*cos(rate * realtime);
   desir_end_efect_vel[4] = amplitude_rot_y*rate*cos(rate * realtime);
   desir_end_efect_vel[5] = amplitude_rot_z*rate*cos(rate * realtime);

   desir_end_effector_frame.p = KDL::Vector(desir_end_efect_pos[0], desir_end_efect_pos[1], desir_end_efect_pos[2]);
   desir_end_effector_frame.M = KDL::Rotation::RPY(desir_end_efect_pos[3], desir_end_efect_pos[4], desir_end_efect_pos[5]);

   desir_end_effector_dot.vel = KDL::Vector(desir_end_efect_vel[0], desir_end_efect_vel[1], desir_end_efect_vel[2]);
   desir_end_effector_dot.rot = KDL::Vector(desir_end_efect_vel[3], desir_end_efect_vel[4], desir_end_efect_vel[5]);

}

/**
 * 计算逆动力学
 * 分成 M C G 三部分
 * 考虑的受外力的情况，函数内部外力为0
 * 
*/
void LlArm6dof::computeInverseDynamics(const KDL::JntArray& q, const KDL::JntArray& q_dot, const KDL::JntArray& q_dotdot, KDL::JntArray& torques)
{
    KDL::JntArray gravity_torques(kdl_chain.getNrOfJoints());
    KDL::JntArray coriolis_torques(kdl_chain.getNrOfJoints());
    KDL::JntArray mass_torques(kdl_chain.getNrOfJoints());
    KDL::JntSpaceInertiaMatrix inertia_matrix(kdl_chain.getNrOfJoints());

    dyn_param->JntToGravity(q, gravity_torques);
    dyn_param->JntToCoriolis(q, q_dot, coriolis_torques);
    dyn_param->JntToMass(q, inertia_matrix);//这时惯性矩阵M

    // for (unsigned int i = 0; i < 6; i++)
    // {
    //     std::cout << "关节 " << i + 1 << " gravity_torques: " << gravity_torques(i) << " Nm" << std::endl;
    // }

    // 设定末端外力：例如沿 X 轴的一个力（单位：N），无外部力矩
     external_force[2] = 0;

    // 计算雅可比矩阵

    // 计算外力对关节的影响：tau_ext = J^T * f_ext
    KDL::JntArray tau_ext(kdl_chain.getNrOfJoints());
    // KDL::Wrench 内部存储为一个 6 维数组：前3个为力，后3个为力矩
    //把末端力转换到关节力
    for (unsigned int i = 0; i < kdl_chain.getNrOfJoints(); i++)
    {
        tau_ext(i) = 0.0;
        for (unsigned int j = 0; j < 6; j++)
        {
            tau_ext(i) += current_jacobian(j, i) * external_force[j];
        }
    }


    for (unsigned int i = 0; i < kdl_chain.getNrOfJoints(); i++) {
        double inertia_term = 0.0;
        // 对每一行求和，考虑所有关节的耦合效应
        for (unsigned int j = 0; j < kdl_chain.getNrOfJoints(); j++) {
            inertia_term += inertia_matrix(i, j) * q_dotdot(j);
        }
        // 将惯性项、Coriolis项、重力项相加，再减去外力影响（tau_ext）
        torques(i) = inertia_term + coriolis_torques(i) + gravity_torques(i) - tau_ext(i);
    }
}

/// @brief 计算关节的加速度
/// @param q 关节角度
/// @param q_dot 关节角速度
/// @param desir_end_effector_ddot 期望轨迹的加速度
/// @param q_dotdot 计算出来的关节加速度
void LlArm6dof::comput_joint_acceleration(const KDL::JntArray &q, const KDL::JntArray &q_dot, KDL::Twist &desir_end_effector_ddot, KDL::JntArray &q_dotdot)
{
    // 1. 准备关节位置和速度
    KDL::JntArrayVel q_in(6);

    for (unsigned int i = 0; i < 6; i++)
    {
        // 关节位置
        q_in.q(i) = q(i);
        // 关节速度
        q_in.qdot(i) = q_dot(i);
    }

    // 3. 调用 JntToJacDot 得到 dot(J)*qdot (存储在一个 Twist 中)
    KDL::Twist jac_dot_q_dot; // 输出结果

    int ret = ik_solver_acc->JntToJacDot(q_in, jac_dot_q_dot);

    if (ret != 0)
    {
        std::cerr << "JntToJacDot 计算失败, 错误码 = " << ret << std::endl;
        return;
    }

    // 4. 计算关节加速度

    KDL::Twist delta_acc = desir_end_effector_ddot - jac_dot_q_dot;

    ret = ik_solver_vel->CartToJnt(q, delta_acc, q_dotdot);

    if (ret != 0)
    {
        std::cerr << "ik_solver_vel 计算失败, 错误码 = " << ret << std::endl;
        return;
    }
}

/// @brief ik计算成功更新关节角度，失败则不更新。期望轨迹超出工作空间则不更新。
/// ik计算出的角度超出关节限制也不会更新关节角度
///
void LlArm6dof::comput_ik( KDL::Frame &frame)
{
    KDL::JntArray desir_joint_positions_temp(6);

    fk_solver_myarm->JntToCart(current_joint_positions, frame, 4);

    float desir_arm_lenght = sqrt(frame.p.x() * frame.p.x() + frame.p.y() * frame.p.y() + (frame.p.z() - 0.1) * (frame.p.z() - 0.1));
 
    float arm_lenght = arm_link_lenth[2] + arm_link_lenth[3];

    // std::cerr << "lenght:  "<< arm_lenght << " "<< desir_arm_lenght<< " "<< current_joint_positions(4) << " "<< current_joint_positions(1) << " "<< current_joint_positions(2) << std::endl;

    if (arm_lenght < (desir_arm_lenght - 0.01))
    {
        std::cerr << "期望轨迹超出工作空间: " << std::endl;
        std::cerr << "arm_lenght:  "<< arm_lenght << std::endl;
        std::cerr << "desir_arm_lenght:  "<< desir_arm_lenght << std::endl;
        return;
    }

    // std::cout << "desir_end_effector_frame: " << desir_end_effector_frame.p.x() << " " << desir_end_effector_frame.p.y() << " " << desir_end_effector_frame.p.z()<< std::endl;


    KDL::JntArray ik_init_joint_positions;

    ik_init_joint_positions = KDL::JntArray(6);

    ik_init_joint_positions.data.setZero();
    
    //IKD的初始位置可以用当前的关节角度也可以用初始关节角度（全零）
    int ret = ik_solver_lma->CartToJnt(current_joint_positions, desir_end_effector_frame, desir_joint_positions_temp);

    // int ret = ik_solver_lma->CartToJnt(ik_init_joint_positions, desir_end_effector_frame, desir_joint_positions_temp);

    if (ret != 0)
    {
        std::cerr << "ik_solver_lma->JntToJacDot 计算失败, 错误码 = " << ret << std::endl;
        return;
    }
    else
    {

        for (size_t i = 0; i < 6; i++)
        {
            if (desir_joint_positions_temp(i) > desir_joint_pos_protect_max[i])
            {
                std::cerr << "关节 " << i << " 角度max超出工作空间"<< desir_joint_positions_temp(i)  << std::endl;
                return;
            }
            if (desir_joint_positions_temp(i) < desir_joint_pos_protect_min[i])
            {
                std::cerr << "关节 " << i << " 角度min超出工作空间"<< desir_joint_positions_temp(i)  << std::endl;
                return;
            }
        }

        desir_joint_positions = desir_joint_positions_temp;
    }
}

/***
 * 更新惯性参数
 * 用于估计末端负载
 */
void LlArm6dof::updateLinkMass(KDL::Chain& chain, const std::string& link_name, double new_mass) {

    KDL::Chain new_chain;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); i++) {
        KDL::Segment seg = chain.getSegment(i);
        // 判断是否为需要更新的连杆
        if (seg.getName() == link_name) {
            // 获取原来的惯性参数
            KDL::RigidBodyInertia inertia = seg.getInertia();
            // 使用原有的质心和旋转惯性，但更新质量
            KDL::RigidBodyInertia new_inertia(new_mass, inertia.getCOG(), inertia.getRotationalInertia());
            // 构造一个新的 Segment，其他参数与原来一致，只是 inertia 更新了
            KDL::Segment new_seg(seg.getName(), seg.getJoint(), seg.getFrameToTip(), new_inertia);
            new_chain.addSegment(new_seg);
            std::cout << "更新连杆 " << link_name << " 的质量为 " << new_mass << std::endl;
        } else {
            new_chain.addSegment(seg);
        }
    }
    // 用新的链替换旧的链
    chain = new_chain;
}