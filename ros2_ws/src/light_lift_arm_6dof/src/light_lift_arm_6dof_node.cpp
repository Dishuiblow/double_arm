#include "rclcpp/rclcpp.hpp"
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>

#include "mathematical_model/light_lift_arm_6dof.h"
#include "controllers/state_machine.h"
#include "communication/serial_port.h"
#include "drivers/motor_control.h"
#include "controllers/state_machine.h"

/**
 * 输入：
 *  time_step：此函数循环的间隔，单位是秒  。
 *  max_time：此函数运行的最大时间，单位是秒。
 *
 * 输出：
 *  此函数运行的时间，单位是秒。
 */
float get_duration_realtime(float time_step, float max_time)
{

    static float loop_count = 0;
    static float realtime = 0;

    realtime = (loop_count)*time_step;

    loop_count++;

    if (realtime > max_time)
    {
        loop_count = 0;
        realtime = 0;
    }

    return realtime;
}

class KDLTestNode : public rclcpp::Node
{
public:
    KDLTestNode() : Node("kdl_test_node"), motorControl(serial)
    {
        // 启动串口接收线程
        serial.startReceiving();

        RCLCPP_INFO(this->get_logger(), "Orocos KDL Test Node Started");

        //创建发布者
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        desir_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("desir_joint_states", 10);

        current_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("current_end_effector_pose", 10);

        current_twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("current_twist", 10);

        current_wrench_publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("current_wrench", 10);

       // 创建TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 创建定时器，以1ms的间隔发布关节状态
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&KDLTestNode::publish_joint_states, this));

        motorControl.EnableMotors(serial);

        std::cerr << "ENABLE MOTORS>>>>>>>>>>>>>>>>>>>>>>>." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));

        motorControl.EnableMotors(serial);


        //     AutoServo,      //电机内部mit
        //     ManualServo,    //手动通过纯力矩控制电机
        //     Impdence,       //阻抗
        //     Admittance,     //导纳
        //     Gravity,        //重力补偿
        //     JointAutoServo, //关节自动控制,注意要初始化关节位置
        //     Zero            //设置零点
        //     Teach        //示教
        //     Planning       //轨迹规划

        mode_ = State::AutoServo;

        control_mode.modeTransition(mode_);

        // 创建订阅者
        desir_end_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "topic_control_desir_end_pos", 10, std::bind(&KDLTestNode::desir_end_pos_callback, this, std::placeholders::_1));

        // // 创建订阅者：关节控制
        desir_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "topic_control_desir_joint_state", 10, std::bind(&KDLTestNode::desir_joint_state_callback, this, std::placeholders::_1));
   
         // 创建字符串话题的订阅者
         cmd_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "cmd_topic", 10, std::bind(&KDLTestNode::cmd_callback, this, std::placeholders::_1));
        }

    static void stopArm()
    {
        std::cout << "\n >>>>>>失能机械臂" << std::endl;
        // 需要一个实例来调用 motorControl 的成员函数
        if (instance)
        {
            instance->motorControl.DisableMotors(instance->serial);
        }
    }

    static void signalHandler(int signum)
    {
        std::cout << "\n >>>>>>接收到SIGINT(" << signum << ")，准备终止程序。" << std::endl;
        // 需要一个实例来调用 motorControl 和 serial 的成员函数
        if (instance)
        {
            instance->motorControl.DisableMotors(instance->serial);
            instance->serial.stopReceiving();
            std::this_thread::sleep_for(std::chrono::microseconds(2000)); // 微秒
        }
        std::exit(signum); // 程序正常退出，将触发atexit的调用
    }

    static KDLTestNode *instance;

private:
    void publish_joint_states()
    {

        float realtime = get_duration_realtime(0.001, 3.1415926 * 4);

        auto parsedData = serial.getReceivedData();

        motorControl.usbDataToMotorState(parsedData, motorControl.motorState); // 获取串口数据

        motorControl.updatMotorState(motorControl.motorState); // 更新电机实时状态

        // 电机保护
        motorControl.real_time_motor_protection(motorControl.current_motor_pos, motorControl.current_motor_vel, motorControl.current_motor_tau, serial);

        llarm6dof.updatArmState(motorControl.current_motor_pos, motorControl.current_motor_vel, motorControl.current_motor_tau); // 更新关节实时状态

        /*** 当 进 行 话 题 控 制 时 要 把 这 个 函 数 注释！！！！！！***/
        llarm6dof.update_end_effector_trajectory(realtime);/******/
        /**********************************************************/

 
        /***************************************
         ************ 实时的计算雅可比 **********
         ***************************************/
        if (llarm6dof.jac_solver_myarm->JntToJac(llarm6dof.current_joint_positions, llarm6dof.current_jacobian) >= 0)
        {

            // RCLCPP_INFO(this->get_logger(), "Jacobian: \n%s", ss.str().c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to calculate Jacobian");
        }

        /***************************************
         ************ 实时的计算关节速度 **********
         ***************************************/
        if (llarm6dof.ik_solver_vel->CartToJnt(llarm6dof.desir_joint_positions, llarm6dof.desir_end_effector_dot, llarm6dof.desir_joint_velocities) >= 0)
        {
            // RCLCPP_INFO(this->get_logger(), "Inverse velocity calculated using pinv solver");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to calculate inverse velocity using pinv solver");
        }

 
        // // 发布关节状态
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->get_clock()->now();
        joint_state_msg.header.frame_id = '1';
        joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        joint_state_msg.position = {llarm6dof.current_joint_positions(0), llarm6dof.current_joint_positions(1), llarm6dof.current_joint_positions(2), llarm6dof.current_joint_positions(3), llarm6dof.current_joint_positions(4), llarm6dof.current_joint_positions(5)};
        joint_state_msg.velocity = {llarm6dof.current_joint_velocities(0), llarm6dof.current_joint_velocities(1), llarm6dof.current_joint_velocities(2), llarm6dof.current_joint_velocities(3), llarm6dof.current_joint_velocities(4), llarm6dof.current_joint_velocities(5)};
        joint_state_msg.effort = {llarm6dof.current_joint_tauqes(0), llarm6dof.current_joint_tauqes(1), llarm6dof.current_joint_tauqes(2), llarm6dof.current_joint_tauqes(3), llarm6dof.current_joint_tauqes(4), llarm6dof.current_joint_tauqes(5)};
        joint_state_publisher_->publish(joint_state_msg);

        // 发布关节状态
        auto desir_joint_state_msg  = sensor_msgs::msg::JointState();
        desir_joint_state_msg.header.stamp = this->get_clock()->now();
        desir_joint_state_msg.header.frame_id = '1';
        desir_joint_state_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        desir_joint_state_msg.position = {llarm6dof.desir_joint_positions(0), llarm6dof.desir_joint_positions(1), llarm6dof.desir_joint_positions(2), llarm6dof.desir_joint_positions(3), llarm6dof.desir_joint_positions(4), llarm6dof.desir_joint_positions(5)};
        desir_joint_state_msg.velocity = {llarm6dof.desir_joint_velocities(0), llarm6dof.desir_joint_velocities(1), llarm6dof.desir_joint_velocities(2), llarm6dof.desir_joint_velocities(3), llarm6dof.desir_joint_velocities(4), llarm6dof.desir_joint_velocities(5)};
        desir_joint_state_msg.effort = {llarm6dof.desir_joint_tauqes(0), llarm6dof.desir_joint_tauqes(1), llarm6dof.desir_joint_tauqes(2), llarm6dof.desir_joint_tauqes(3), llarm6dof.desir_joint_tauqes(4), llarm6dof.desir_joint_tauqes(5)};
        desir_joint_state_publisher_->publish(desir_joint_state_msg);

        // // 发布TF变换
        KDL::Frame frame;
        geometry_msgs::msg::TransformStamped transform_stamped;

        llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 1);
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "link1"; // link6
        transform_stamped.transform.translation.x = frame.p.x();
        transform_stamped.transform.translation.y = frame.p.y();
        transform_stamped.transform.translation.z = frame.p.z();
        frame.M.GetQuaternion(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        // tf_broadcaster_->sendTransform(transform_stamped);

        llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 2);
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "link2"; // link6
        transform_stamped.transform.translation.x = frame.p.x();
        transform_stamped.transform.translation.y = frame.p.y();
        transform_stamped.transform.translation.z = frame.p.z();
        frame.M.GetQuaternion(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        // tf_broadcaster_->sendTransform(transform_stamped);

        llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 3);
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "link3"; // link6
        transform_stamped.transform.translation.x = frame.p.x();
        transform_stamped.transform.translation.y = frame.p.y();
        transform_stamped.transform.translation.z = frame.p.z();
        frame.M.GetQuaternion(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        // tf_broadcaster_->sendTransform(transform_stamped);

        /***************************************
         ************ 实时的计算逆运动学 **********
         ***************************************/
        /**当进行话题 关 节 控制的时候要把这个函数注释！！！！！！*/
        if(control_mode.currentState != State::JointAutoServo)
        {
            llarm6dof.comput_ik(frame);
        }

        llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 4);
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "link4"; // link6
        transform_stamped.transform.translation.x = frame.p.x();
        transform_stamped.transform.translation.y = frame.p.y();
        transform_stamped.transform.translation.z = frame.p.z();
        frame.M.GetQuaternion(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        // tf_broadcaster_->sendTransform(transform_stamped);

        llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 5);
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "link5"; // link6
        transform_stamped.transform.translation.x = frame.p.x();
        transform_stamped.transform.translation.y = frame.p.y();
        transform_stamped.transform.translation.z = frame.p.z();
        frame.M.GetQuaternion(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        // tf_broadcaster_->sendTransform(transform_stamped);

        llarm6dof.fk_solver_myarm->JntToCart(llarm6dof.current_joint_positions, frame, 6);
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "base_link";
        transform_stamped.child_frame_id = "link6"; // link6
        transform_stamped.transform.translation.x = frame.p.x();
        transform_stamped.transform.translation.y = frame.p.y();
        transform_stamped.transform.translation.z = frame.p.z();
        frame.M.GetQuaternion(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        // tf_broadcaster_->sendTransform(transform_stamped);
        /***************************************
         ************ 实时的计算正运动学 **********
         ***************************************/
        llarm6dof.current_end_effector_frame = frame;

        /***************************************
         ************ 实时的发布末端状态 **********
         但是发布语句和相关计算被注释掉了
         ***************************************/
        // 末端位姿，相对世界坐标系（或者说：固定坐标系、基坐标系）
        geometry_msgs::msg::Pose pose;
        pose.position.x = llarm6dof.current_end_effector_frame.p.x();
        pose.position.y = llarm6dof.current_end_effector_frame.p.y();
        pose.position.z = llarm6dof.current_end_effector_frame.p.z();
        double x_, y_, z_, w_;
        llarm6dof.current_end_effector_frame.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        // current_pose_publisher_->publish(pose);//这里被注释了!!!!!!!!

        // 末端速度，相对世界坐标系（或者说：固定坐标系、基坐标系）
        // llarm6dof.joint2end_vel();;//这里被注释了!!!!!!!!
        geometry_msgs::msg::Twist twist;
        twist.linear.x = llarm6dof.current_end_effector_dot.vel(0);
        twist.linear.y = llarm6dof.current_end_effector_dot.vel(1);
        twist.linear.z = llarm6dof.current_end_effector_dot.vel(2);
        twist.angular.x = llarm6dof.current_end_effector_dot.rot(0);
        twist.angular.y = llarm6dof.current_end_effector_dot.rot(1);
        twist.angular.z = llarm6dof.current_end_effector_dot.rot(2);
        // current_twist_publisher_->publish(twist);
        // 末端力，相对世界坐标系（或者说：固定坐标系、基坐标系）
        // llarm6dof.joint2end_force();;//这里被注释了!!!!!!!!
        geometry_msgs::msg::Wrench wrench;
        wrench.force.x = llarm6dof.current_end_effector_wrench.force(0);
        wrench.force.y = llarm6dof.current_end_effector_wrench.force(1);
        wrench.force.z = llarm6dof.current_end_effector_wrench.force(2);
        wrench.torque.x = llarm6dof.current_end_effector_wrench.torque(0);
        wrench.torque.y = llarm6dof.current_end_effector_wrench.torque(1);
        wrench.torque.z = llarm6dof.current_end_effector_wrench.torque(2);
        // current_pose_publisher_->publish(wrench);;//这里被注释了!!!!!!!!

        /*******************************
         ************ 运行状态机 **********
         *******************************/
        try
        {
            control_mode.run(serial, llarm6dof, motorControl); // 循环运行
        }
        catch (const std::exception &e)
        {
            std::cerr << "程序异常：" << e.what() << std::endl;
        }
    }

    //某端轨迹控制话题的回调函数
    //鼠标和键盘控制就是使用的这个函数
    void desir_end_pos_callback(const geometry_msgs::msg::Pose msg)
    {

        llarm6dof.desir_end_effector_frame.p.data[1] = msg.position.y;

        llarm6dof.desir_end_effector_frame.p.data[0] = msg.position.x;

        llarm6dof.desir_end_effector_frame.p.data[2] = msg.position.z;

        llarm6dof.desir_end_effector_frame.M = KDL::Rotation::RPY(msg.orientation.x, msg.orientation.y, msg.orientation.z);

        llarm6dof.desir_grp = msg.orientation.w;

        // 这个模式会自动规划一个轨迹到期望位置
        // 使用这个模式时一定要注释掉末端轨迹更新函数
        if (mode_ == State::Planning)
        {
            control_mode.is_run_planning = true;

            RCLCPP_INFO(this->get_logger(), "Planning");

            RCLCPP_INFO(this->get_logger(), "BE SURE YOU ARE closed: llarm6dof.update_end_effector_trajectory(realtime);");
        }

        // RCLCPP_INFO(this->get_logger(), "BE SURE YOU ARE closed: llarm6dof.update_end_effector_trajectory(realtime);");
    }

    // 使用话题进行关节控制时的回调函数
    // 一般用来拖动示教机械臂
    void desir_joint_state_callback(const sensor_msgs::msg::JointState msg)
    {
        RCLCPP_INFO(this->get_logger(), "desir_joint_state_callback Received ");

        for (size_t i = 0; i < 6; i++)
        {
            llarm6dof.desir_joint_positions(i) = msg.position[i];
            // if(set_joint_init)
            // {//让示教的第一个关节位置赋值到关节初始化的目标位置
            //     llarm6dof.desir_joint_pos_default[i] = msg.position[i];
            // }
            /* code */
        }

        // 处理接收到的命令
    }

    // 接受命令的回调函数
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        llarm6dof.cmd = msg->data.c_str();//判断是否自适应负载

        RCLCPP_INFO(this->get_logger(), "Received string message: '%s'", llarm6dof.cmd);

        // 控制程序进入重力补偿模式
        //  一般用来拖动示教机械臂
        if (llarm6dof.cmd == "Gravity")
        {

            mode_ = State::Gravity;

            control_mode.modeTransition(mode_);
        }

        if (llarm6dof.cmd == "Impdence")
        {

            mode_ = State::Impdence;

            control_mode.modeTransition(mode_);
        }

        // 控制程序进入关节自动控制模式
        // 但是之前会过渡到各个关节的零点
        //   一般用来拖动示教机械臂
        if (llarm6dof.cmd == "JointAutoServo")
        {
            mode_ = State::Gohome;

            control_mode.modeTransition(mode_);

            control_mode.is_go_home = true;
        }

        std::cout << llarm6dof.cmd << std::endl;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desir_joint_state_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr current_pose_publisher_;//末端的位置向量

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr current_twist_publisher_;//末端的速度向量

    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr current_wrench_publisher_;//末端的力向量

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr desir_end_pos_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desir_joint_state_subscriber_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscriber_; // 字符串话题的订阅者
    
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_myarm;

    LlArm6dof llarm6dof;

    serialport::SerialPortWrapper serial;

    MotorControl motorControl;

    ArmState control_mode;

    State mode_ ;

    bool set_joint_init = false;
};

KDLTestNode *KDLTestNode::instance = nullptr;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<KDLTestNode>();

    KDLTestNode::instance = node.get();

    // 注册结束时执行的函数
    std::atexit(KDLTestNode::stopArm);

    // 注册SIGINT信号处理函数
    std::signal(SIGINT, KDLTestNode::signalHandler);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}