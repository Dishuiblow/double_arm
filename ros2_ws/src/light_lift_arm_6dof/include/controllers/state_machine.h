#ifndef MODESTSTEMSCHINE_H
#define MODESTSTEMSCHINE_H

#include <iostream>
#include "communication/serial_port.h"
#include "mathematical_model/light_lift_arm_6dof.h"
#include "drivers/motor_control.h"


enum class State {
    AutoServo,      //电机内部mit
    ManualServo,    //手动通过纯力矩控制电机
    Impdence,       //阻抗
    Admittance,     //导纳
    Gravity,        //重力补偿
    JointAutoServo, //关节自动控制,注意要初始化关节位置
    Zero ,          //设置零点
    Planning,       //以规划的轨迹到达期望点
    Gohome          //关节归零
};

class ArmState {
public:
    ArmState();

    void modeTransition(State newState);
    void run(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void autoServo(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void manualServo(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void manualServo_STSMC(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void impdence(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void admittance(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void gravity(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void initialize(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void joint_initialize(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void joint_auto_servo(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void planning(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);
    void joint_go_home(serialport::SerialPortWrapper &port, LlArm6dof &arm, MotorControl &motor);

    bool is_init_joint = false;
    bool is_init_end = false;
    bool is_run_planning = false;
    bool is_go_home = false;

    State currentState;

private:
     
};




#endif // ARMCONTROL_H

