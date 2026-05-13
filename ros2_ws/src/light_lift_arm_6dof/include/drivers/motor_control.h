#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <cstdint>
#include <vector>
#include <stdexcept>
#include "communication/serial_port.h"
#include <math.h>

#define MOTOR_NUM 6

class MotorControl {
public:
    // 构造函数与析构函数
    MotorControl(serialport::SerialPortWrapper &s);
    ~MotorControl();

    serialport::SerialPortWrapper &serial;

    int frame_date_lenth = 0;
    float kp[MOTOR_NUM]={0};
    float kd[MOTOR_NUM]={0};
    float pos[MOTOR_NUM]={0};
    float vel[MOTOR_NUM]={0};
    float tau[MOTOR_NUM]={0};

    float current_motor_pos[MOTOR_NUM] = {0};
    float current_joint_pos[MOTOR_NUM] = {0};
    float current_motor_vel[MOTOR_NUM] = {0};
    float current_joint_vel[MOTOR_NUM] = {0};
    float current_motor_acc[MOTOR_NUM] = {0};
    float current_joint_acc[MOTOR_NUM] = {0};
    float current_motor_tau[MOTOR_NUM] = {0};
    float current_joint_tau[MOTOR_NUM] = {0};

    float desir_motor_pos[MOTOR_NUM] = {0};
    float desir_joint_pos[MOTOR_NUM] = {0};
    float desir_motor_vel[MOTOR_NUM] = {0};
    float desir_joint_vel[MOTOR_NUM] = {0};
    float desir_motor_tau[MOTOR_NUM] = {0};
    float desir_joint_tau[MOTOR_NUM] = {0};
    float desir_motor_acc[MOTOR_NUM] = {0};
    float desir_joint_acc[MOTOR_NUM] = {0};

    static constexpr int record_count = 6280;
    float motor1_record_pos[record_count] = {0};
    float motor2_record_pos[record_count] = {0};
    float motor3_record_pos[record_count] = {0};
    float grp_record_pos[record_count] = {0};


    float vel_protection_level = 2;
    float tau_protection_level = 13;

    float desir_motor_pos_protect_max[MOTOR_NUM] = { 1.5, 1.45, 1.50, 1.57, 1.57, 1.57};
    float desir_motor_pos_protect_min[MOTOR_NUM] = {-1.5, -1.5, -1.5, -1.57, -1.57, -1.57};

    float desir_motor_vel_protect_max[MOTOR_NUM] = { 2.57,  2.57,  2.57,  2.57,  2.57,  2.57};
    float desir_motor_vel_protect_min[MOTOR_NUM] = {-1.57, -2.57, -2.57, -2.57, -2.57, -2.57};

    float desir_motor_tau_protect_max[MOTOR_NUM] = { 6,  11,  9, 5, 4, 3};
    float desir_motor_tau_protect_min[MOTOR_NUM] = {-6, -11, -9,-5,-4,-3};

    float current_motor_pos_protect_max[MOTOR_NUM] = { 1.5, 1.5, 1.50, 1.5, 1.5, 1.57};
    float current_motor_pos_protect_min[MOTOR_NUM] = {-1.5, -1.3, -1.5, -1.5, -1.5, -1.57};

    float current_motor_vel_protect_max[MOTOR_NUM] = { 2.57,  2.57,  2.57,  4.57,  4.57,  4.57};
    float current_motor_vel_protect_min[MOTOR_NUM] = {-1.57, -2.57, -2.57, -4.57, -4.57, -4.57};

    float current_motor_tau_protect_max[MOTOR_NUM] = { 4,  8,  7, 3, 3, 3};
    float current_motor_tau_protect_min[MOTOR_NUM] = {-4, -8, -7,-3,-3,-3};


    float motor_impedance_tau[MOTOR_NUM] = {0};
    float motor_pid_tau[MOTOR_NUM] = {0};
    float motor_admittance_tau[MOTOR_NUM] = {0};
    float motor_gravity_tau[MOTOR_NUM] = {0};
    float motor_control_tau[MOTOR_NUM] = {0};

    float static_friction_in_gravity_max[MOTOR_NUM] = { 0.08,  0.2,   0.15, 0.08,  0.2,   0.15};
    float static_friction_in_gravity_min[MOTOR_NUM] = {-0.08, -0.04, -0.05,-0.08, -0.04, -0.05};


    std::vector<float> motorState;


    int floatToUint(float value, float min, float max, int bits);
    float uintToFloat(int value, float min, float max, int bits);
    void MitCtrl( float pos, float vel, float kp, float kd, float torq, uint8_t data[8]);
    void MitCtrl4340( float pos, float vel, float kp, float kd, float torq, uint8_t data[8]);
    void usbDataToMotorState(const std::vector<uint8_t>& data, std::vector<float>& motorState);
    void ControlMotors(serialport::SerialPortWrapper &port,float pos[6], float vel[6], float kp[6], float kd[6], float tau[6]);
    void ControlMotors_g(serialport::SerialPortWrapper &port,float pos[6], float vel[6], float kp[6], float kd[6], float tau[6], uint8_t grp);


    void EnableMotors(serialport::SerialPortWrapper &port);
    void DisableMotors(serialport::SerialPortWrapper &port);
    void SetZeroMotors(serialport::SerialPortWrapper &port);


    void updatMotorState(std::vector<float> motorState);
    void real_time_motor_protection(float pos_[MOTOR_NUM], float vel_[MOTOR_NUM], float tau_[MOTOR_NUM], serialport::SerialPortWrapper &port);
    

    void KalmanFilter(float process_noise, float measurement_noise, float estimation_error, float initial_value, float limit);
      
    float update(float measurement);

    void ButterworthFilter(float cutoff_frequency, float sampling_rate);

    float Butt_update(float new_value);

    void compensate_static_friction_through_vel(float filtered_motor_vel[MOTOR_NUM], float motor_tor[MOTOR_NUM]);

    uint8_t enable_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    uint8_t disable_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    // uint8_t clear_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    uint8_t setzero_motor_data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};

    std::vector<uint8_t> enable_data; // 示例数据
    std::vector<uint8_t> disable_data; // 示例数据
    std::vector<uint8_t> setzero_data; // 示例数据


private:


    // 物理量范围
    static constexpr float P_MIN = -12.5f;
    static constexpr float P_MAX = 12.5f;

    static constexpr float V_MIN = -30.0f;
    static constexpr float V_MAX = 30.0f;

    static constexpr float KP_MIN = 0.0f;
    static constexpr float KP_MAX = 500.0f;

    static constexpr float KD_MIN = 0.0f;
    static constexpr float KD_MAX = 5.0f;

    static constexpr float T_MIN = -10.0f;
    static constexpr float T_MAX = 10.0f;


    static constexpr float P_MIN_4340 = -12.5f;
    static constexpr float P_MAX_4340 = 12.5f;

    static constexpr float V_MIN_4340 = -10.0f;
    static constexpr float V_MAX_4340 = 10.0f;

    static constexpr float KP_MIN_4340 = 0.0f;
    static constexpr float KP_MAX_4340 = 500.0f;

    static constexpr float KD_MIN_4340 = 0.0f;
    static constexpr float KD_MAX_4340 = 5.0f;

    static constexpr float T_MIN_4340 = -28.0f;
    static constexpr float T_MAX_4340 = 28.0f;

 
    float Q = 1e-6; // 过程噪声协方差
    float R = 1e-2; // 测量噪声协方差
    float P = 1; // 估计误差协方差
    float K; // 卡尔曼增益
    float X = 0; // 状态估计
    float threshold= 0.03;//阈值

    float a0, a1, a2, b1, b2;
    float x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

};

#endif // MOTOR_CONTROL_H
