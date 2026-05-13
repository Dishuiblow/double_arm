#include <iostream>
#include "controllers/state_machine.h"

    

ArmState::ArmState() : currentState(State::AutoServo) {}

int signum(float x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void ArmState::modeTransition(State newState)
{
    switch (newState)
    {
    case State::AutoServo:
        std::cout << "Transitioning to AutoServo state.\n";
        break;
    case State::ManualServo:
        std::cout << "Transitioning to ManualServo state.\n";
        break;
    case State::Impdence:
        std::cout << "Transitioning to Impdence state.\n";
        break;
    case State::Admittance:
        std::cout << "Transitioning to Admittance state.\n";
        break;
    case State::JointAutoServo:
        std::cout << "Transitioning to JointAutoServo state.\n";
        break;
    case State::Gravity:
        std::cout << "Transitioning to Gravity state.\n";
        break;
    case State::Zero:
        std::cout << "Transitioning to Zero state.\n";
        break;
    case State::Planning:
        std::cout << "Transitioning to Planning state.\n";
        break;
    case State::Gohome:
        std::cout << "Transitioning to Gohome state.\n";
        break;
    }

    // 更新当前状态
    currentState = newState;
}

void ArmState::run(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{
     bool initialized = is_init_end;
     bool joint_initialized = is_init_joint;

    if (currentState == State::Zero)
        initialized = true; // 校零不用到期望位型

    if (!initialized)
    {
        std::cout << "Initializing Robot Arm.\n";
        if ((currentState == State::JointAutoServo))
        { // 关节空间位型初始化
            joint_initialize(port, arm, motor);
            initialized = true;
            is_init_end = true;
            is_init_joint = true;
        }
        else
        { // 工作空间位型初始化
            initialize(port, arm, motor);
            initialized = true;
            is_init_end = true;
            is_init_joint = true;
        }
    }

    switch (currentState)
    {
    case State::AutoServo:
        autoServo(port, arm, motor);
        break;

    case State::ManualServo:
        // manualServo(port, arm, motor);
        manualServo_STSMC(port, arm, motor);
        break;

    case State::Impdence:
        impdence(port, arm, motor);
        break;

    case State::Admittance:
        admittance(port, arm, motor);
        break;

    case State::JointAutoServo:
        joint_auto_servo(port, arm, motor);
        break;

    case State::Gravity:
        // std::cout << "Robot Arm has in Gravity state.\n";
        gravity(port, arm, motor);
        break;
    case State::Zero:
        std::cout << "Robot Arm has in Zero state.\n";
        motor.SetZeroMotors(port);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        motor.SetZeroMotors(port);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "ok.\n";
        break;
    case State::Planning:
        planning(port, arm, motor);
        break;
    case State::Gohome:
        joint_go_home(port, arm, motor);
        break;
    }
}

void ArmState::autoServo(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    // while (true)
    {

        // for (size_t i = 0; i < 6; i++)
        // {
        //     arm.current_joint_positions(i) = 0;
        //     arm.current_joint_velocities(i) = 0;
        //     arm.current_joint_acceleration(i) = 0;
        // }

        // std::cout << "逆动力学计算成功，关节力矩为：" << std::endl;
        // for (unsigned int i = 0; i < 6; i++)
        // {
        //     std::cout << "关节 " << i + 1 << " 力矩: " << arm.current_joint_tauqes(i) << " Nm" << std::endl;
        // }
        // 实时重力补偿
        // arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
        //                             arm.current_joint_vel,
        //                             arm.current_joint_acc,
        //                             arm.joint_gravity_tau);

        // float est_joint_load[3] = {15 * (arm.current_joint_tau[0] - arm.joint_gravity_tau[0]), 15 * (arm.current_joint_tau[1] - arm.joint_gravity_tau[1]), 15 * (arm.current_joint_tau[2] - arm.joint_gravity_tau[2])};

        // arm.joint2end_force(arm.current_joint_pos, est_joint_load, arm.current_end_efect_tau);

        // arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);
        arm.ik_solver_lma->CartToJnt(arm.current_joint_positions, arm.desir_end_effector_frame, arm.desir_joint_positions);

            // 计算逆动力学（即，求每个关节需要施加的力矩）
        arm.computeInverseDynamics(arm.desir_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        // std::cout << "autoServo-desir_joint_pos: " << arm.desir_joint_pos[0] << " " << arm.desir_joint_pos[1] << " " << arm.desir_joint_pos[2]<< std::endl;
       
        // float AV[6] = {0};

        // arm.Dof_3_Arm_iAV_Sca(arm.desir_joint_pos, arm.desir_end_efect_vel, arm.desir_end_efect_acc, AV);

        // for (size_t i = 0; i < 3; i++)
        // {
        //     arm.desir_joint_vel[i] = AV[i];
        //     arm.desir_joint_acc[i] = AV[3 + i];
        // }

        // arm.joint_control_tau[0] = 1.0 * arm.joint_gravity_tau[0];

        // arm.joint_control_tau[1] = 1.0 * arm.joint_gravity_tau[1];

        // arm.joint_control_tau[2] = 1.0 * arm.joint_gravity_tau[2];

        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
            arm.desir_joint_vel[i] = arm.desir_joint_velocities(i);
            arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
        }
        
    // 由于关节和电机的方向有可能不一致，所以要转换一下
        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 用期望的电机速度，提前补偿静摩擦
        // motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

        motor.motor_pid_tau[0] = 350 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0]) + 1.9 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);

        motor.motor_pid_tau[1] = 300 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1]) + 2.0 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);

        motor.motor_pid_tau[2] = 280 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2]) + 2.0 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);

        motor.motor_pid_tau[3] = 10 * (motor.desir_motor_pos[3] - motor.current_motor_pos[3]) + 0.6 * (motor.desir_motor_vel[3] - motor.current_motor_vel[3]);

        for (size_t i = 0; i < 4; i++)
        {
            motor.motor_control_tau[i] += motor.motor_pid_tau[i];
        }

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // motor.ControlMotors_g(port,
        //                       motor.desir_motor_pos,
        //                       motor.desir_motor_vel,
        //                       motor.kp,
        //                       motor.kd,
        //                       motor.motor_control_tau,
        //                       arm.desir_grp);

        // float force = sqrt(arm.current_end_efect_tau[0] * arm.current_end_efect_tau[0] + arm.current_end_efect_tau[1] * arm.current_end_efect_tau[1] + arm.current_end_efect_tau[2] * arm.current_end_efect_tau[2]);

        // float estimate_end_load = force / 0.00981;
        // std::cout << "autoServo-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "autoServo-current_end_pos  : " << arm.current_end_efect_pos[0] << " " << arm.current_end_efect_pos[1] << " " << arm.current_end_efect_pos[2]<< std::endl;
        // std::cout << "autoServo-current_motor_tau: " << motor.current_motor_tau[0] << " " << motor.current_motor_tau[1] << " " << motor.current_motor_tau[2]<< std::endl;
        // std::cout << std::endl;
        // std::cout << "autoServo-estimate_end_load: " << arm.current_end_efect_tau[2] << std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}
/// @brief 电机pid为零，仅用力矩控制，手动实现pid
/// @param port
/// @param arm
/// @param motor
void ArmState::manualServo(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    float kp1 = 315;
    float kp2 = 315;
    float kp3 = 312;
    float kp4 = 20;
    float kp5 = 20;
    float kp6 = 20;

    float kd1 = 1.43;
    float kd2 = 1.43;
    float kd3 = 1.73;
    float kd4 = 0.53;
    float kd5 = 0.43;
    float kd6 = 0.33;

    // kp1 = 0;
    // kp2 = 0;
    // kp3 = 0;

    // kd1 = 0.0;
    // kd2 = 0.0;
    // kd3 = 0.0;

    // while (true)
    {


         arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);


         for (size_t i = 0; i < 6; i++)
         {
             arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
             arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
             arm.desir_joint_vel[i] = arm.desir_joint_velocities(i);
         }
  

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 直接在电机空间运行pid

        motor.motor_pid_tau[0] = kp1 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0]) + kd1 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);

        motor.motor_pid_tau[1] = kp2 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1]) + kd2 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);

        motor.motor_pid_tau[2] = kp3 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2]) + kd3 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);

        motor.motor_pid_tau[3] = kp4 * (motor.desir_motor_pos[3] - motor.current_motor_pos[3]) + kd4 * (motor.desir_motor_vel[3] - motor.current_motor_vel[3]);

        motor.motor_pid_tau[4] = kp5 * (motor.desir_motor_pos[4] - motor.current_motor_pos[4]) + kd5 * (motor.desir_motor_vel[4] - motor.current_motor_vel[4]);

        motor.motor_pid_tau[5] = kp6 * (motor.desir_motor_pos[5] - motor.current_motor_pos[5]) + kd6 * (motor.desir_motor_vel[5] - motor.current_motor_vel[5]);

        for (size_t i = 0; i < 6; i++)
        {
            motor.motor_control_tau[i] += motor.motor_pid_tau[i];
        }

        // float est_joint_load[3] = {(arm.current_joint_tau[0] - arm.joint_gravity_tau[0]), (arm.current_joint_tau[1] - arm.joint_gravity_tau[1]), (arm.current_joint_tau[2] - arm.joint_gravity_tau[2])};

        // arm.joint2end_force(arm.current_joint_pos, motor.motor_pid_tau, arm.current_end_efect_tau);

        // 用期望的电机速度，提前补偿静摩擦
        // motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

        for (size_t i = 0; i < 6; i++)
        {
            motor.kp[i] = 0.0;
            motor.kd[i] = 0.0;
        }

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "manualServo-estimate_end_load: " << arm.current_end_efect_tau[2] << std::endl;
        // std::cout << "manualServo-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "manualServo-current_end_pos  : " << arm.current_end_efect_pos[0] << " " << arm.current_end_efect_pos[1] << " " << arm.current_end_efect_pos[2]<< std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}


/// @brief 电机pid为零，仅用力矩控制，但是使用超螺旋滑模算法
/// @param port
/// @param arm
/// @param motor
void ArmState::manualServo_STSMC(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    // while (true)
    {
        // 计算逆动力学（即，求每个关节需要施加的力矩）
        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        float JointState_r[6] = {0};
        float JointState_D[9] = {0};
        float JointState[9] = {0};
        float estimateT[6] = {0};
        float ControlT[6] = {0}; // 控制输入
        float s[6] = {0};
        float e[6] = {0};
        float de[6] = {0};
        float K[6] = {0};
        float S_K[6] = {0};
        float F_K1[6] = {0};
        float F_K2[6] = {0};
        float V[6] = {0};
        float Yd[6] = {0};
        float S_state[6] = {0};
        float Control_his[5] = {0};
        float detlta[6] = {6.7, 11.7, 11.6, 8.7, 4.7, 1.6};
        float A1 = -0.6;
        float A2 = + 110;
        float B1 = 0;
        float B2 = 0;
        float C1 = 0;
        float C2 = 0;

        K[0] = 280;
        S_K[0] = 0.9 ; // 适当的增大会增加精度
        F_K1[0] = 0.9;
        F_K2[0] = 0.8;

        K[1] = 290 + A2;
        S_K[1] = 1.39 + A1; // 但是会导致运动的波动，负载越大波动越严重
        F_K1[1] = 1.25 + A1;
        F_K2[1] = 1.1 + A1;

        K[2] = 210 + A2;     // 增大精确，与抖动无关
        S_K[2] = 1.1 + A1;  // 增大会增加控制力矩的振动频率，会更快恢复平稳状态
        F_K1[2] = 1.1 + A1; // 增大后显著抖动
        F_K2[2] = 1.1 + A1; // 调小了丝滑，增大抖动


        K[3] = 130;
        S_K[3] = 0.5; // 适当的增大会增加精度
        F_K1[3] = 0.2;
        F_K2[3] = 0.15;

        K[4] = 90;
        S_K[4] = 0.1; // 但是会导致运动的波动，负载越大波动越严重
        F_K1[4] = 0.1;
        F_K2[4] = 0.1;

        K[5] = 80;     // 增大精确，与抖动无关
        S_K[5] = 0.1;  // 增大会增加控制力矩的振动频率，会更快恢复平稳状态
        F_K1[5] = 0.1; // 增大后显著抖动
        F_K2[5] = 0.1; // 调小了丝滑，增大抖动




        // K[0] = 280;
        // S_K[0] = 0.8; // 适当的增大会增加精度
        // F_K1[0] = 0.4;
        // F_K2[0] = 0.2;

        // K[1] = 290;
        // S_K[1] = 0.89; // 但是会导致运动的波动，负载越大波动越严重
        // F_K1[1] = 0.25;
        // F_K2[1] = 0.1;

        // K[2] = 210;     // 增大精确，与抖动无关
        // S_K[2] = 0.85;  // 增大会增加控制力矩的振动频率，会更快恢复平稳状态
        // F_K1[2] = 0.1; // 增大后显著抖动
        // F_K2[2] = 0.2; // 调小了丝滑，增大抖动


        // K[3] = 80;
        // S_K[3] = 0.2; // 适当的增大会增加精度
        // F_K1[3] = 0.2;
        // F_K2[3] = 0.15;

        // K[4] = 50;
        // S_K[4] = 0.1; // 但是会导致运动的波动，负载越大波动越严重
        // F_K1[4] = 0.1;
        // F_K2[4] = 0.1;

        // K[5] = 50;     // 增大精确，与抖动无关
        // S_K[5] = 0.1;  // 增大会增加控制力矩的振动频率，会更快恢复平稳状态
        // F_K1[5] = 0.1; // 增大后显著抖动
        // F_K2[5] = 0.1; // 调小了丝滑，增大抖动

        /* 误 差 */
        for (size_t i = 0; i < 6; i++)
        {
            e[i] = arm.desir_joint_positions(i) - arm.current_joint_positions(i);

            de[i] = arm.desir_joint_velocities(i) - arm.current_joint_velocities(i);

            s[i] = 3 * de[i] + K[i] * e[i]; // 擅自在速度误差上加了增益，也算有点效果

            /*更 新 滑 模 状 态*/
            S_state[i] += S_K[i] * signum(s[i]) * 0.06;
        }

        arm.comput_joint_acceleration(arm.desir_joint_positions, arm.desir_joint_velocities, arm.desir_end_effector_ddot, arm.desir_joint_acceleration);

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.desir_joint_acceleration, arm.STSMC_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            /* code */
            estimateT[i] = arm.STSMC_joint_tauqes(i);

            ControlT[i] = estimateT[i] + F_K1[i] * s[i] + F_K2[i] * sqrt(abs(s[i])) * tanh(s[i]) + S_state[i];

            if (ControlT[i] > (arm.gravity_joint_tauqes(i) + detlta[i]))
                ControlT[i] = arm.gravity_joint_tauqes(i) + detlta[i];
            if (ControlT[i] < (arm.gravity_joint_tauqes(i) - detlta[i]))
                ControlT[i] = arm.gravity_joint_tauqes(i) - detlta[i];

            arm.joint_control_tau[i] = ControlT[i];
        }

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        for (size_t i = 0; i < 6; i++)
        {
            motor.kp[i] = 0.0;
            motor.kd[i] = 0.0;
        }

        // 用期望的电机速度，提前补偿静摩擦
        // motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "manualServo_STSMC-motor_control_tau: " << motor.motor_control_tau[0] << " " << motor.motor_control_tau[1] << " " << motor.motor_control_tau[2] << std::endl;
        // std::cout << "manualServo_STSMC-desir_joint_vel: " << arm.desir_joint_vel[0] << " " << arm.desir_joint_vel[1] << " " << arm.desir_joint_vel[2] << std::endl;

        // 超螺旋控制运算主程序
    }
}

/**
 *
 * 使用纯力矩控制，实现阻抗，
 * 比较丝滑，不容易抖动。
 * 但是，非阻抗的轴向，控制精度不如用电机内部pid的高
 * 这个模式用来估计末端负载非常合适
 * 仅有此模式增加了更新惯性参数（连杆2的质量）功能
 * 
 * 估计末端负载的方法，是重新加载chian，这个过程不能实时进行，只能通过判断加载一个次
 */
void ArmState::impdence(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{
    // while (true)
    {
        float e[6]={0}, de[6]={0}, K[6], C[6], F[6];
        float end_est_wrentch[6] = {0};

    // 计算 x_dot = J * q_dot 末端速度
    for (unsigned int i = 0; i < 6; i++)
    {
        double component = 0.0;
        for (unsigned int j = 0; j < 6; j++)
        {
            component += arm.current_jacobian(i, j) * arm.current_joint_velocities(j);
        }
        // 前 3 行对应线速度，后 3 行对应角速度
        if (i < 3)
        {
            arm.current_end_effector_dot.vel(i) = component;
        }
        else
        {
            arm.current_end_effector_dot.rot(i - 3) = component;
        }
        
    }

    e[0] = arm.desir_end_effector_frame.p.x() - arm.current_end_effector_frame.p.x();
    e[1] = arm.desir_end_effector_frame.p.y() - arm.current_end_effector_frame.p.y();
    e[2] = arm.desir_end_effector_frame.p.z() - arm.current_end_effector_frame.p.z();

    double desir_roll, desir_pitch, desir_yaw, current_roll, current_pitch, current_yaw;
    arm.desir_end_effector_frame.M.GetRPY(desir_roll, desir_pitch, desir_yaw);
    arm.current_end_effector_frame.M.GetRPY(current_roll, current_pitch, current_yaw);

    e[3] = desir_roll - current_roll;
    e[4] = desir_pitch- current_pitch;
    e[5] = desir_yaw - current_yaw;
    
    de[0] = 0.0*arm.desir_end_effector_dot.vel(0) - arm.current_end_effector_dot.vel(0);
    de[1] = 0.0*arm.desir_end_effector_dot.vel(1) - arm.current_end_effector_dot.vel(1);
    de[2] = 0.0*arm.desir_end_effector_dot.vel(2) - arm.current_end_effector_dot.vel(2);
    de[3] = 0.0*arm.desir_end_effector_dot.rot(0) - arm.current_end_effector_dot.rot(0);
    de[4] = 0.0*arm.desir_end_effector_dot.rot(1) - arm.current_end_effector_dot.rot(1);
    de[5] = 0.0*arm.desir_end_effector_dot.rot(2) - arm.current_end_effector_dot.rot(2);

    //硬
    // C[0] = 31.96;
    // C[1] = 20.96;
    // C[2] = 20.96;
    // C[3] = 0.13;
    // C[4] = 0.93;
    // C[5] = 0.93;

    //软
    C[0] = 0.36;
    C[1] = 0.36;
    C[2] = 0.36;
    C[3] = 0.13;
    C[4] = 0.03;
    C[5] = 0.03;

    //硬
    K[0] = 455;
    K[1] = 1095;
    K[2] = 1095;
    K[3] = 18;
    K[4] = 18;
    K[5] = 11;

    //软
    // K[0] = 255;
    // K[1] = 295;
    // K[2] = 95;
    // K[3] = 18;
    // K[4] = 18;
    // K[5] = 11;

    for (size_t i = 0; i < 6; i++)
    {
        F[i] = C[i]*de[i] + K[i]*e[i];
    }

    for (unsigned int i = 0; i < 6; i++)
    {
        double component = 0.0;
        for (unsigned int j = 0; j < 6; j++)
        {
            component += arm.current_jacobian(j, i) * F[j];
        }

        arm.joint_impedance_tau[i] = component;
        
    }

    // std::cout << "arm.current_end_effector_wrench: " << arm.current_end_effector_wrench.force(0) << " " <<arm.current_end_effector_wrench.force(1) << " " << arm.current_end_effector_wrench.force(2)<< std::endl;

    // std::cout << "arm.joint_impedance_tau: " << arm.joint_impedance_tau[0] << " " <<arm.joint_impedance_tau[1] << " " << arm.joint_impedance_tau[2]<< std::endl;

    // std::cout << "arm.e: " << e[0] << " " << e[1] << " " << e[2] << std::endl;

    // std::cout << "arm.desir_end_effector_frame: " << arm.desir_end_effector_frame.p.x() << " " << arm.desir_end_effector_frame.p.y() << " " << arm.desir_end_effector_frame.p.z() << std::endl;

    // arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

    // for (size_t i = 0; i < 6; i++)
    // {
    //     arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i) + 0.0*arm.joint_impedance_tau[i];
    // }

    arm.joint2end_force();

    double link_mass = 0.0015 * (arm.current_end_effector_wrench.force(2) / 0.00981);

    if(arm.cmd == "laod")
    {

        std::string link_name = "link6";

        arm.updateLinkMass(arm.kdl_chain, link_name, 0.5*link_mass);

        std::cout << "某端增加： " << link_mass << " kg" << std::endl;

        arm.cmd = "nan";
    }
    else if(arm.cmd ==  "clearlaod"){

        std::string link_name = "link6";

        arm.updateLinkMass(arm.kdl_chain, link_name, 0.0);

        std::cout << "某端增加： " << 0.0 << " kg" << std::endl;

        arm.cmd = "nan";

    }


    // std::cout << "arm.current_end_effector_wrench: " << arm.current_end_effector_wrench.force(0) << " " << arm.current_end_effector_wrench.force(1) << " " << arm.current_end_effector_wrench.force(2) << std::endl;

    arm.joint_control_tau[0] = arm.gravity_joint_tauqes(0) + 1.2*arm.joint_impedance_tau[0] + 0.0*signum(arm.joint_impedance_tau[0]) * 1 ;
    arm.joint_control_tau[1] = arm.gravity_joint_tauqes(1) + 1.3*arm.joint_impedance_tau[1] + 0.0*signum(arm.joint_impedance_tau[1]) * 1;
    arm.joint_control_tau[2] = arm.gravity_joint_tauqes(2) + 1.3*arm.joint_impedance_tau[2] + 0.0*signum(arm.joint_impedance_tau[2]) * 1;
    arm.joint_control_tau[3] = arm.gravity_joint_tauqes(3) + 1.0*arm.joint_impedance_tau[3];
    arm.joint_control_tau[4] = arm.gravity_joint_tauqes(4) + 1.0*arm.joint_impedance_tau[4];
    arm.joint_control_tau[5] = arm.gravity_joint_tauqes(5) + 1.0*arm.joint_impedance_tau[5];


        // 由于关节和电机的方向有可能不一致，所以要转换一下

        for (size_t i = 0; i < 3; i++)
        {
            arm.desir_joint_pos[3+i] = arm.desir_joint_positions(3+i);
        }
        
        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // 电机反馈回来的力矩是pid的控制力矩+重力补偿力矩

        // motor.ControlMotors(port,
        //                     motor.desir_motor_pos,
        //                     motor.desir_motor_vel,
        //                     motor.kp,
        //                     motor.kd,
        //                     motor.motor_control_tau);

        float gravity_kp[6] = {0.0};
        float gravity_kd[6] = {0.0};

        gravity_kp[3] = 30;
        // gravity_kp[4] = 20;
        // gravity_kp[5] = 10;

        gravity_kd[0] = 0.1;
        gravity_kd[1] = 0.2;
        gravity_kd[2] = 0.2;

        gravity_kd[3] = 0.9;
        gravity_kd[4] = 0.8;
        gravity_kd[5] = 0.5;

        // motor.motor_control_tau[0] = 0;
        // motor.motor_control_tau[1] = 0;
        // motor.motor_control_tau[2] = 0;
 
        // for (size_t i = 0; i < 6; i++)
        // {
        //     motor.desir_motor_pos[i] = 0;
        //     motor.desir_motor_vel[i] = 0;
        // }

        motor.ControlMotors_g(port,
                              motor.desir_motor_pos,
                              motor.desir_motor_vel,
                              gravity_kp,
                              gravity_kd,
                              motor.motor_control_tau,
                              arm.desir_grp);

        // std::cout << "desir_grp : " << arm.desir_grp << std::endl;

        // std::cout << "impdence-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "impdence-end_force  : " << arm.link_mass[1] << " " << end_force[1] << " " << end_force[2]<< std::endl;

        // float ts_tau1 = motor.current_motor_tau[2]/q3_e;
        // std::cout << "impdence-desir_grp: " << arm.desir_grp << " " << 0 << " " << motor.current_motor_tau[2]<< std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}

// 力约束的自由度，最好只有一个，不然会抽搐
void ArmState::admittance(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{
    static float ts_time = 0;

    float refer_mask[6] = {0.0};
    float end_force[6] = {0};

    // while (true)
    {

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            refer_mask[i] = 1 - abs(arm.end_force_orientation[i]);
        }
        // std::cout << "desir_motor_pos  : " << current_end_pos[0] << " " << current_end_pos[1] << " " << current_end_pos[2]<< std::endl;
        arm.desir_end_effector_frame.p.data[0] = arm.desir_end_effector_frame.p.x() * refer_mask[0] + arm.current_end_effector_frame.p.x() * abs(arm.end_force_orientation[0]);
        arm.desir_end_effector_frame.p.data[1] = arm.desir_end_effector_frame.p.y() * refer_mask[1] + arm.current_end_effector_frame.p.y() * abs(arm.end_force_orientation[1]);
        arm.desir_end_effector_frame.p.data[2] = arm.desir_end_effector_frame.p.z() * refer_mask[2] + arm.current_end_effector_frame.p.z()  * abs(arm.end_force_orientation[2]); 

        double desir_roll, desir_pitch, desir_yaw, current_roll, current_pitch, current_yaw;
        arm.desir_end_effector_frame.M.GetRPY(desir_roll, desir_pitch, desir_yaw);
        arm.current_end_effector_frame.M.GetRPY(current_roll, current_pitch, current_yaw);
    
        double R = desir_roll * refer_mask[3]  + current_roll  * abs(arm.end_force_orientation[3]); 
        double P = desir_pitch * refer_mask[4] + current_pitch * abs(arm.end_force_orientation[4]); 
        double Y = desir_yaw * refer_mask[5]   + current_yaw  * abs(arm.end_force_orientation[5]); 

        arm.desir_end_effector_frame.M = KDL::Rotation::RPY(R, P, Y);

        int ret = arm.ik_solver_lma->CartToJnt(arm.current_joint_positions, arm.desir_end_effector_frame, arm.desir_joint_positions);

        //在主函数里面进行逆运动学

        // std::cout << "imp_desir_pos    : " << imp_desir_pos[0] << " " << imp_desir_pos[1] << " " << imp_desir_pos[2]<< std::endl;
     // / 获取当前末端位速度
        for (unsigned int i = 0; i < 6; i++)
        {
            double component = 0.0;
            for (unsigned int j = 0; j < 6; j++)
            {
                component += arm.current_jacobian(i, j) * arm.current_joint_velocities(j);
            }
            // 前 3 行对应线速度，后 3 行对应角速度
            if (i < 3)
            {
                arm.current_end_effector_dot.vel(i) = component;
            }
            else
            {
                arm.current_end_effector_dot.rot(i - 3) = component;
            }
            
        }
    
        for (size_t i = 0; i < 6; i++)
        {
            end_force[i] = arm.end_force_orientation[i] * arm.end_force_value;
        }

        // 期望末端力，对应的关节的力矩

        for (unsigned int i = 0; i < 6; i++)
        {
            double component = 0.0;
            for (unsigned int j = 0; j < 6; j++)
            {
                component += arm.current_jacobian(j, i) * end_force[j];
            }
    
            arm.joint_admittance_tau[i] = component;
            
        }

        // arm.admittance(arm.desir_end_efect_pos,
        //                arm.current_joint_pos,
        //                arm.current_joint_vel,
        //                arm.end_force_orientation,
        //                arm.end_force_value,
        //                arm.desir_end_efect_acc,
        //                arm.desir_joint_pos,
        //                arm.desir_joint_vel,
        //                arm.joint_admittance_tau);

        // for (size_t i = 0; i < 3; i++)
        // {
        //     if (abs(arm.current_joint_vel[i]) <= 0.01)
                // arm.joint_admittance_tau[i] += signum(arm.joint_admittance_tau[i]) * 0.1;
        // }

        arm.joint_control_tau[0] = arm.joint_admittance_tau[0] + arm.gravity_joint_tauqes(0);

        arm.joint_control_tau[1] = arm.joint_admittance_tau[1] + arm.gravity_joint_tauqes(1);

        arm.joint_control_tau[2] = arm.joint_admittance_tau[2] + arm.gravity_joint_tauqes(2);

        arm.joint_control_tau[3] = arm.joint_admittance_tau[3] + arm.gravity_joint_tauqes(3);

        arm.joint_control_tau[4] = arm.joint_admittance_tau[4] + arm.gravity_joint_tauqes(4);

        arm.joint_control_tau[5] = arm.joint_admittance_tau[5] + arm.gravity_joint_tauqes(5);

        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
        }
        

        // 由于关节和电机的方向有可能不一致，所以要转换一下
        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // motor.compensate_static_friction_through_vel(motor.current_motor_vel, motor.motor_control_tau);
        // motor.kp[0] = 0.0;
        // motor.kd[0] = 0.0;
    
        // motor.kp[1] = 0.0;
        // motor.kd[1] = 0.0;
    
        // motor.kp[2] = 0.0;
        // motor.kd[2] = 0.0;

        // motor.kp[3] = 00.0;
        // motor.kd[3] = 0.00;
    
        // motor.kp[4] = 0.0;
        // motor.kd[4] = 0.0;
    
        // motor.kp[5] = 00.0;
        // motor.kd[5] = 0.0;


        motor.kp[0] = 350.0;
        motor.kd[0] = 1.51;
    
        motor.kp[1] = 270.0;
        motor.kd[1] = 2.01;
    
        motor.kp[2] = 270.0;
        motor.kd[2] = 2.01;
    
        // motor.kp[0] = 0.0;
        // motor.kd[0] = 0.0;
    
        // motor.kp[1] = 00.0;
        // motor.kd[1] = 0.0;
    
        // motor.kp[2] = 0.0;
        // motor.kd[2] = 0.0;
    
    
        motor.kp[3] = 55.0;
        motor.kd[3] = 0.9;
    
        motor.kp[4] = 20.0;
        motor.kd[4] = 0.51;
    
        motor.kp[5] = 10.0;
        motor.kd[5] = 0.51;

        //在z轴方向上超级丝滑，但是期望速度必须为0,不然会抖
        //x轴很涩，
        //y轴也很丝滑
        for (size_t i = 0; i < 6; i++)
        {
            motor.current_motor_vel[i] = 0;
        }

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.current_motor_vel, // 使用实时速度是最好的！！！！
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);
        // std::cout << "admittance-current_motor_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< std::endl;
        // std::cout << "admittance-desir_motor_pos  : " << motor.desir_motor_pos[0] << " " << motor.desir_motor_pos[1] << " " << motor.desir_motor_pos[2]<< std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}

void ArmState::gravity(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{
    // 配置滤波参数
    motor.KalmanFilter(1e-6, 1e-2, 1, 0, 0.03);

    // while (true)
    {

        // 卡尔曼滤波，处理电机速度值
        // for (size_t i = 0; i < 3; i++)
        // {
        //     motor.current_motor_vel[i] = motor.update(motor.current_motor_vel[i]);
        // }


        // std::cout << "current_joint_positions: " << arm.current_joint_positions(3) << " " << arm.current_joint_positions(4) << " " << arm.current_joint_positions(5)<< std::endl;
      
        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
        }


        
        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // motor.compensate_static_friction_through_vel(motor.current_motor_vel, motor.motor_control_tau);

        float gravity_kp[6] = {0.0};
        float gravity_kd[6] = {0.0};

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            gravity_kp,
                            gravity_kd,
                            motor.motor_control_tau);

        // std::cout << "gravity: " << motor.motor_control_tau[0] << " " << motor.motor_control_tau[1] << " " << motor.motor_control_tau[2]<< std::endl;
        // std::cout << "arm.current_joint_pos: " << arm.current_joint_pos[0] << " " << arm.current_joint_pos[1] << " " << arm.current_joint_pos[2] << std::endl;

        // std::this_thread::sleep_for(std::chrono::microseconds(900)); // 微秒
    }
}

void ArmState::initialize(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    float ts_time = 0;
    /**
     * 初始姿态
     */

    // arm.desir_end_efect_pos[0] = 0.21 + 0.03 * sin(3 * ts_time / 1000.0);

    // arm.desir_end_efect_pos[1] = 0.0 + 0.03 * cos(3 * ts_time / 1000.0);

    // arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);

    arm.update_end_effector_trajectory(0);

    arm.ik_solver_lma->CartToJnt(arm.current_joint_positions, arm.desir_end_effector_frame, arm.desir_joint_positions);
 

    std::cout << "获取当前关节位置>>>" << std::endl;

    /* 获取当前关节位置 */
    for (size_t i = 0; i < 50; i++)
    {
        auto parsedData_ = port.getReceivedData();

        motor.usbDataToMotorState(parsedData_, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        // arm.Ik_3Dof(arm.desir_end_efect_pos, arm.desir_joint_pos);
        arm.ik_solver_lma->CartToJnt(arm.current_joint_positions, arm.desir_end_effector_frame, arm.desir_joint_positions);

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);


        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
        }
        

        for (size_t i = 0; i < 6; i++)
        {
            motor.kp[i] = 0.0;
            motor.kd[i] = 0.0;
            motor.tau[i] = 0.0*arm.gravity_joint_tauqes(i);
        }
        motor.ControlMotors(port, motor.desir_motor_pos, motor.desir_motor_vel, motor.kp, motor.kd, motor.tau);

        std::cout << "current_motor_pos: " << arm.current_joint_positions(0) << " " << arm.current_joint_positions(1) << " " << arm.current_joint_positions(2)<< arm.current_joint_positions(3) << " " << arm.current_joint_positions(4) << " " << arm.current_joint_positions(5) << std::endl;
        std::cout << "desir_joint_pos  : " << arm.desir_joint_positions(0) << " " << arm.desir_joint_positions(1) << " " << arm.desir_joint_positions(2)<< arm.desir_joint_positions(3) << " " << arm.desir_joint_positions(4) << " " << arm.desir_joint_positions(5) << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 测试**********************************************************************************************************************
    float current_joint_pos_temp[6] = {arm.current_joint_positions(0), arm.current_joint_positions(1), arm.current_joint_positions(2),arm.current_joint_positions(3), arm.current_joint_positions(4), arm.current_joint_positions(5)};
    float desir_joint_pos_temp[6] = {arm.desir_joint_positions(0), arm.desir_joint_positions(1), arm.desir_joint_positions(2), arm.desir_joint_positions(3), arm.desir_joint_positions(4), arm.desir_joint_positions(5)};
    // 测试**********************************************************************************************************************

    motor.kp[0] = 450.0;
    motor.kd[0] = 4.51;

    motor.kp[1] = 500.0;
    motor.kd[1] = 4.81;

    motor.kp[2] = 500.0;
    motor.kd[2] = 4.51;


    motor.kp[3] = 90.0;
    motor.kd[3] = 1.0;

    motor.kp[4] = 40.0;
    motor.kd[4] = 0.51;

    motor.kp[5] = 10.0;
    motor.kd[5] = 0.51;


    motor.kp[0] = 450.0;
    motor.kd[0] = 4.51;

    motor.kp[1] = 300.0;
    motor.kd[1] = 4.81;

    motor.kp[2] = 300.0;
    motor.kd[2] = 4.51;


    motor.kp[3] = 40.0;
    motor.kd[3] = 1.0;

    motor.kp[4] = 40.0;
    motor.kd[4] = 0.51;

    motor.kp[5] = 10.0;
    motor.kd[5] = 0.51;


    // motor.kp[0] = 10.0;
    // motor.kd[0] = 2.01;

    // motor.kp[1] = 40.0;
    // motor.kd[1] = 2.01;

    // motor.kp[2] = 30.0;
    // motor.kd[2] = 1.71;

    std::cout << "arm.desir_end_effector_frame: " << arm.desir_end_effector_frame.p.x() << " " << arm.desir_end_effector_frame.p.y() << " " << arm.desir_end_effector_frame.p.z() << std::endl;
    

    /* 前往期望位置 */
    while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp, arm.desir_joint_pos, 1, 50))
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        // 实时重力补偿
        // arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
        //                             arm.current_joint_vel,
        //                             arm.current_joint_acc,
        //                             arm.joint_gravity_tau);

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
        }

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // std::cout << "init arm: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2];

        std::cout << "init arm: " << motor.desir_motor_pos[0] << " " << motor.desir_motor_pos[1] << " " << motor.desir_motor_pos[2]<< motor.desir_motor_pos[3] << " " << motor.desir_motor_pos[4] << " " << motor.desir_motor_pos[5];

        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "到达目标位置>>>" << std::endl;


    // std::string link = "link6";
    // arm.updateLinkMass(arm.kdl_chain, link, 0.6);
    // std::cout << "updateLinkMass>>>" << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(3));

}

void ArmState::joint_initialize(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    float ts_time = 0;

    arm.desir_joint_positions.data.setZero();

  
    std::cout << "获取当前关节位置>>>" << std::endl;

    /* 获取当前关节位置 */
    for (size_t i = 0; i < 50; i++)
    {
        auto parsedData_ = port.getReceivedData();

        motor.usbDataToMotorState(parsedData_, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        arm.desir_joint_positions.data.setZero();
        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_positions(i) = arm.desir_joint_pos_default[i];
            /* code */
        }
        

        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
        }
        

        for (size_t i = 0; i < 6; i++)
        {
            motor.kp[i] = 0.0;
            motor.kd[i] = 0.0;
            motor.tau[i] = 0.0;
        }
        motor.ControlMotors(port, motor.desir_motor_pos, motor.desir_motor_vel, motor.kp, motor.kd, motor.tau);

        // std::cout << "current_motor_pos: " << motor.current_joint_positions[0] << " " << motor.current_joint_positions[1] << " " << motor.current_joint_positions[2]<< motor.current_joint_positions[3] << " " << motor.current_joint_positions[4] << " " << motor.current_joint_positions[5] << std::endl;
        // std::cout << "desir_joint_pos  : " << arm.desir_joint_positions[0] << " " << arm.desir_joint_positions[1] << " " << arm.desir_joint_positions[2]<< arm.desir_joint_positions[3] << " " << arm.desir_joint_positions[4] << " " << arm.desir_joint_positions[5] << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 测试**********************************************************************************************************************
    float current_joint_pos_temp[6] = {arm.current_joint_positions(0), arm.current_joint_positions(1), arm.current_joint_positions(2),arm.current_joint_positions(3), arm.current_joint_positions(4), arm.current_joint_positions(5)};
    float desir_joint_pos_temp[6] = {arm.desir_joint_positions(0), arm.desir_joint_positions(1), arm.desir_joint_positions(2), arm.desir_joint_positions(3), arm.desir_joint_positions(4), arm.desir_joint_positions(5)};
    // 测试**********************************************************************************************************************

    motor.kp[0] = 450.0;
    motor.kd[0] = 1.51;

    motor.kp[1] = 500.0;
    motor.kd[1] = 4.81;

    motor.kp[2] = 500.0;
    motor.kd[2] = 4.51;


    motor.kp[3] = 90.0;
    motor.kd[3] = 1.0;

    motor.kp[4] = 40.0;
    motor.kd[4] = 0.51;

    motor.kp[5] = 10.0;
    motor.kd[5] = 0.51;


    /* 前往期望位置 */
    while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp, arm.desir_joint_pos, 1, 50))
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        // 实时重力补偿
        // arm.Closed_Arm_Modle_decoup(arm.current_joint_pos,
        //                             arm.current_joint_vel,
        //                             arm.current_joint_acc,
        //                             arm.joint_gravity_tau);

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
        }

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        motor.ControlMotors(port,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "到达目标位置>>>" << std::endl;


}

void ArmState::joint_auto_servo(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

    for (size_t i = 0; i < 6; i++)
    {
        arm.desir_joint_pos[i] = arm.desir_joint_positions(i);
        arm.desir_joint_vel[i] = arm.desir_joint_velocities(i);
        arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
    }
    
// 由于关节和电机的方向有可能不一致，所以要转换一下
    arm.joint2motor(arm.desir_joint_pos,
                    arm.desir_joint_vel,
                    arm.desir_joint_acc,
                    arm.joint_control_tau,
                    motor.desir_motor_pos,
                    motor.desir_motor_vel,
                    motor.desir_motor_acc,
                    motor.motor_control_tau);

    // 用期望的电机速度，提前补偿静摩擦
    // motor.compensate_static_friction_through_vel(motor.desir_motor_vel, motor.motor_control_tau);

    // motor.motor_pid_tau[0] = 150 * (motor.desir_motor_pos[0] - motor.current_motor_pos[0]) + 1.1 * (motor.desir_motor_vel[0] - motor.current_motor_vel[0]);

    // motor.motor_pid_tau[1] = 170 * (motor.desir_motor_pos[1] - motor.current_motor_pos[1]) + 1.4 * (motor.desir_motor_vel[1] - motor.current_motor_vel[1]);

    // motor.motor_pid_tau[2] = 170 * (motor.desir_motor_pos[2] - motor.current_motor_pos[2]) + 1.4 * (motor.desir_motor_vel[2] - motor.current_motor_vel[2]);

    for (size_t i = 0; i < 3; i++)
    {
        motor.motor_control_tau[i] += motor.motor_pid_tau[i];
    }


    motor.ControlMotors(port,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.kp,
                        motor.kd,
                        motor.motor_control_tau);
}

void ArmState::joint_go_home(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    // arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

    // float kp_temp[6] = {0.0};
    // float kd_temp[6] = {0.0};

    // for (size_t i = 0; i < 6; i++)
    // {
    //     motor.motor_control_tau[i] = arm.gravity_joint_tauqes(i);
    // }

    // motor.ControlMotors(port,
    //                     motor.current_joint_pos,
    //                     motor.current_joint_vel,
    //                     kp_temp,
    //                     kd_temp,
    //                     motor.motor_control_tau);

    if (is_go_home)
    {
        motor.kp[0] = 450.0;
        motor.kd[0] = 4.51;

        motor.kp[1] = 500.0;
        motor.kd[1] = 4.81;

        motor.kp[2] = 500.0;
        motor.kd[2] = 4.51;

        motor.kp[3] = 90.0;
        motor.kd[3] = 1.0;

        motor.kp[4] = 40.0;
        motor.kd[4] = 0.51;

        motor.kp[5] = 10.0;
        motor.kd[5] = 0.51;

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);
   

        for (size_t i = 0; i < 6; i++)
        {
            motor.motor_control_tau[i] = arm.gravity_joint_tauqes(i);
        }

        std::cout << "current_joint_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< " " << motor.current_motor_pos[3]<< " " << motor.current_motor_pos[4]<< " " << motor.current_motor_pos[5] << std::endl;


        motor.ControlMotors(port,
                            motor.current_motor_pos,
                            motor.current_joint_vel,
                            motor.kp,
                            motor.kd,
                            motor.motor_control_tau);

        // 测试**********************************************************************************************************************
        float current_joint_pos_temp[6] = {arm.current_joint_positions(0), arm.current_joint_positions(1), arm.current_joint_positions(2), arm.current_joint_positions(3), arm.current_joint_positions(4), arm.current_joint_positions(5)};
        float desir_joint_pos_temp[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 测试**********************************************************************************************************************

        // std::cout << "arm.desir_end_effector_frame: " << arm.desir_end_effector_frame.p.x() << " " << arm.desir_end_effector_frame.p.y() << " " << arm.desir_end_effector_frame.p.z() << std::endl;


        std::cout << "current_joint_pos_temp: " << arm.current_joint_positions(0) << " " << arm.current_joint_positions(1) << " " << arm.current_joint_positions(2)<< " " << arm.current_joint_positions(3)<< " " << arm.current_joint_positions(4)<< " " << arm.current_joint_positions(5) << std::endl;


        std::cout << "current_joint_pos_temp: " << current_joint_pos_temp[0] << " " << current_joint_pos_temp[1] << " " << current_joint_pos_temp[2]<< " " << current_joint_pos_temp[3]<< " " << current_joint_pos_temp[4]<< " " << current_joint_pos_temp[5] << std::endl;

        /* 前往期望位置 */
        while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp, arm.desir_joint_pos, 1, 50))
        {
            auto parsedData = port.getReceivedData();

            motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

            motor.updatMotorState(motor.motorState); // 更新电机实时状态

            arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

            /*所有的实时加速度都不更新，且为默认值0*/

            arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

            for (size_t i = 0; i < 6; i++)
            {
                arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
            }

            // 由于关节和电机的方向有可能不一致，所以要转换一下
            arm.joint2motor(arm.desir_joint_pos,
                            arm.desir_joint_vel,
                            arm.desir_joint_acc,
                            arm.joint_control_tau,
                            motor.desir_motor_pos,
                            motor.desir_motor_vel,
                            motor.desir_motor_acc,
                            motor.motor_control_tau);

            motor.ControlMotors(port,
                                motor.desir_motor_pos,
                                motor.desir_motor_vel,
                                motor.kp,
                                motor.kd,
                                motor.motor_control_tau);

            // std::cout << "init arm: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2];

            // std::cout << "init arm: " << motor.desir_motor_pos[0] << " " << motor.desir_motor_pos[1] << " " << motor.desir_motor_pos[2] << " " << motor.desir_motor_pos[3] << " " << motor.desir_motor_pos[4] << " " << motor.desir_motor_pos[5];

            // std::cout << std::endl;
            // std::cout << "init arm: " << arm.desir_joint_pos[0] << " " << arm.desir_joint_pos[1] << " " << arm.desir_joint_pos[2] << " " << arm.desir_joint_pos[3] << " " << arm.desir_joint_pos[4] << " " << arm.desir_joint_pos[5];

            // std::cout << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        std::cout << "到达目标位置>>>" << std::endl;

        std::cout << "重置关节角度>>>" << std::endl;

        for (size_t i = 0; i < 6; i++)
        {
            arm.desir_joint_positions(i) = 0;
        }
        
        std::cout << "切换到关节控制模式>>>" << std::endl;

        modeTransition(State::JointAutoServo);

        is_go_home = false;
    }
}

void ArmState::planning(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor)
{

    // while (true)
   if(is_run_planning) {

    for (size_t i = 0; i < 6; i++)
    {
        motor.motor_control_tau[i] = arm.gravity_joint_tauqes(i);
    }

    // std::cout << "current_joint_pos: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2]<< " " << motor.current_motor_pos[3]<< " " << motor.current_motor_pos[4]<< " " << motor.current_motor_pos[5] << std::endl;


    motor.ControlMotors_g(port,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.kp,
                        motor.kd,
                        motor.motor_control_tau,
                        arm.desir_grp);




    // 测试**********************************************************************************************************************
    float current_joint_pos_temp[6] = {arm.current_joint_positions(0), arm.current_joint_positions(1), arm.current_joint_positions(2),arm.current_joint_positions(3), arm.current_joint_positions(4), arm.current_joint_positions(5)};
    float desir_joint_pos_temp[6] = {arm.desir_joint_positions(0), arm.desir_joint_positions(1), arm.desir_joint_positions(2), arm.desir_joint_positions(3), arm.desir_joint_positions(4), arm.desir_joint_positions(5)};
    // 测试**********************************************************************************************************************

    std::cout << "desir_joint_pos_temp: " << desir_joint_pos_temp[0] << " " << desir_joint_pos_temp[1] << " " << desir_joint_pos_temp[2]<< " " << desir_joint_pos_temp[3]<< " " << desir_joint_pos_temp[4]<< " " << desir_joint_pos_temp[5] << std::endl;



    while (!arm.now2aim(current_joint_pos_temp, desir_joint_pos_temp, arm.desir_joint_pos, 8, 50))
    {
        auto parsedData = port.getReceivedData();

        motor.usbDataToMotorState(parsedData, motor.motorState); // 获取串口数据

        motor.updatMotorState(motor.motorState); // 更新电机实时状态

        arm.updatArmState(motor.current_motor_pos, motor.current_motor_vel, motor.current_motor_tau); // 更新关节实时状态

        /*所有的实时加速度都不更新，且为默认值0*/

        arm.computeInverseDynamics(arm.current_joint_positions, arm.current_joint_velocities, arm.current_joint_acceleration, arm.gravity_joint_tauqes);

        for (size_t i = 0; i < 6; i++)
        {
            arm.joint_control_tau[i] = arm.gravity_joint_tauqes(i);
        }

        // 由于关节和电机的方向有可能不一致，所以要转换一下

        arm.joint2motor(arm.desir_joint_pos,
                        arm.desir_joint_vel,
                        arm.desir_joint_acc,
                        arm.joint_control_tau,
                        motor.desir_motor_pos,
                        motor.desir_motor_vel,
                        motor.desir_motor_acc,
                        motor.motor_control_tau);

        // motor.ControlMotors(port,
        //                     motor.desir_motor_pos,
        //                     motor.desir_motor_vel,
        //                     motor.kp,
        //                     motor.kd,
        //                     motor.motor_control_tau);

        motor.ControlMotors_g(port,
                              motor.desir_motor_pos,
                              motor.desir_motor_vel,
                              motor.kp,
                              motor.kd,
                              motor.motor_control_tau,
                              arm.desir_grp);

        // std::cout << "init arm: " << motor.current_motor_pos[0] << " " << motor.current_motor_pos[1] << " " << motor.current_motor_pos[2];

        // std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "到达目标位置>>>" << std::endl;

    is_run_planning = false;
   }
}
