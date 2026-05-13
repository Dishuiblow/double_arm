#include "drivers/motor_control.h"


MotorControl::MotorControl(serialport::SerialPortWrapper &s)
    : serial(s),
      enable_data(s.FRAME_DATA_LENGTH, 0x01),
      disable_data(s.FRAME_DATA_LENGTH, 0x01),
      setzero_data(s.FRAME_DATA_LENGTH, 0x01)
{

    motorState.reserve(MOTOR_NUM*3);
    int cout = 0;
    for (size_t i = 0; i < 8; i++)
    {
        enable_data[i] = enable_motor_data[i];
        disable_data[i] = disable_motor_data[i];
        setzero_data[i] = setzero_motor_data[i];
        cout = 8;
        enable_data[cout + i] = enable_motor_data[i];
        disable_data[cout + i] = disable_motor_data[i];
        setzero_data[cout + i] = setzero_motor_data[i];
        cout = 16;
        enable_data[cout + i] = enable_motor_data[i];
        disable_data[cout + i] = disable_motor_data[i];
        setzero_data[cout + i] = setzero_motor_data[i];
        cout = 24;
        enable_data[cout + i] = enable_motor_data[i];
        disable_data[cout + i] = disable_motor_data[i];
        setzero_data[cout + i] = setzero_motor_data[i];
        cout = 32;
        enable_data[cout + i] = enable_motor_data[i];
        disable_data[cout + i] = disable_motor_data[i];
        setzero_data[cout + i] = setzero_motor_data[i];
        cout = 40;
        enable_data[cout + i] = enable_motor_data[i];
        disable_data[cout + i] = disable_motor_data[i];
        setzero_data[cout + i] = setzero_motor_data[i];
    }
    frame_date_lenth = s.FRAME_DATA_LENGTH;
    // 构造函数逻辑

    for (size_t i = 0; i < 6; i++)
    {
        current_motor_vel_protect_max[i] += vel_protection_level;
        current_motor_tau_protect_max[i] += tau_protection_level;

        current_motor_vel_protect_min[i] -= vel_protection_level;
        current_motor_tau_protect_min[i] -= tau_protection_level;
    }
    



}

MotorControl::~MotorControl()
{

    // 析构函数逻辑
}

int MotorControl::floatToUint(float value, float min, float max, int bits)
{
    float span = max - min;
    return static_cast<int>((value - min) * ((1 << bits) - 1) / span);
}

float MotorControl::uintToFloat(int value, float min, float max, int bits)
{
    float span = max - min;
    return value * span / ((1 << bits) - 1) + min;
}

void MotorControl::usbDataToMotorState(const std::vector<uint8_t> &data, std::vector<float> &motorState_)
{
    if (data.size() < 8)
    {
        throw std::invalid_argument("Insufficient data length for motor state extraction.");
    }
    int is_motor_conect = 0;

    motorState_.resize(18);
    //判断哪个电机的数据全为0
    for (size_t j = 0; j < 6; j++)
    {
        int motor_num = j*8;
        is_motor_conect = 0;
        for (size_t i = 0; i < 8; i++)
        {
            if(data[motor_num+i]==0) is_motor_conect += 1;
            else is_motor_conect = 0;
    
            if(i == 7)
            {
                if(is_motor_conect>6) 
                {
                    std::cout << "motor "<< j <<" diconnect!!!! " << std::endl;
                    return;
                }
            }
        }
    }
    

    
    int m1Int = (data[1] << 8) | data[2];
    int m2Int = (data[3] << 4) | (data[4] >> 4);
    int m3Int = ((data[4]&0xF)<<8)|data[5];
    motorState_[0] = uintToFloat(m1Int, P_MIN_4340, P_MAX_4340, 16);
    motorState_[1] = uintToFloat(m2Int, V_MIN_4340, V_MAX_4340, 12);
    motorState_[2] = uintToFloat(m3Int, T_MIN_4340, T_MAX_4340, 12);

    int offset = 8;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[3] = uintToFloat(m1Int, P_MIN_4340, P_MAX_4340, 16);
    motorState_[4] = uintToFloat(m2Int, V_MIN_4340, V_MAX_4340, 12);
    motorState_[5] = uintToFloat(m3Int, T_MIN_4340, T_MAX_4340, 12);

    offset = 16;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[6] = uintToFloat(m1Int, P_MIN_4340, P_MAX_4340, 16);
    motorState_[7] = uintToFloat(m2Int, V_MIN_4340, V_MAX_4340, 12);
    motorState_[8] = uintToFloat(m3Int, T_MIN_4340, T_MAX_4340, 12);

    /****************************************************************/
    offset = 24;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[9] = uintToFloat(m1Int, P_MIN, P_MAX, 16);
    motorState_[10] = uintToFloat(m2Int, V_MIN, V_MAX, 12);
    motorState_[11] = uintToFloat(m3Int, T_MIN, T_MAX, 12);

    offset = 32;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[12] = uintToFloat(m1Int, P_MIN, P_MAX, 16);
    motorState_[13] = uintToFloat(m2Int, V_MIN, V_MAX, 12);
    motorState_[14] = uintToFloat(m3Int, T_MIN, T_MAX, 12);

    offset = 40;
    m1Int = (data[offset + 1] << 8) | data[offset + 2];
    m2Int = (data[offset + 3] << 4) | (data[offset + 4] >> 4);
    m3Int = ((data[offset + 4]&0xF)<<8)|data[offset + 5];
    motorState_[15] = uintToFloat(m1Int, P_MIN, P_MAX, 16);
    motorState_[16] = uintToFloat(m2Int, V_MIN, V_MAX, 12);
    motorState_[17] = uintToFloat(m3Int, T_MIN, T_MAX, 12);




}

void MotorControl::EnableMotors(serialport::SerialPortWrapper &port)
{

    std::vector<uint8_t> frame;
    port.packFrame(enable_data, frame);

    // 发送数据
    port.sendData(frame);
}

void MotorControl::DisableMotors(serialport::SerialPortWrapper &port)
{

    std::vector<uint8_t> frame;
    port.packFrame(disable_data, frame);

    // 发送数据
    port.sendData(frame);
}

void MotorControl::SetZeroMotors(serialport::SerialPortWrapper &port)
{

    std::vector<uint8_t> frame;
    port.packFrame(setzero_data, frame);

    // 发送数据
    port.sendData(frame);
}

/// <summary>
/// 返回8个字节，对应位置速kp kd和力矩
/// </summary>
/// <param name="pos"></param>
/// <param name="vel"></param>
/// <param name="kp"></param>
/// <param name="kd"></param>
/// <param name="torq"></param>
/// <returns></returns>
void MotorControl::MitCtrl(float pos_, float vel_, float kp_, float kd_, float torq_, uint8_t data[8])
{
    // byte[] data = new byte[8];
    // uint8_t data[8] = {0};
    int pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = floatToUint(pos_, P_MIN, P_MAX, 16);
    vel_tmp = floatToUint(vel_, V_MIN, V_MAX, 12);
    kp_tmp = floatToUint(kp_, KP_MIN, KP_MAX, 12);
    kd_tmp = floatToUint(kd_, KD_MIN, KD_MAX, 12);
    tor_tmp = floatToUint(torq_, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = (((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = (((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    data[7] = tor_tmp;

    // return data;
}

void MotorControl::MitCtrl4340(float pos_, float vel_, float kp_, float kd_, float torq_, uint8_t data[8])
{
    // byte[] data = new byte[8];
    // uint8_t data[8] = {0};
    int pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    pos_tmp = floatToUint(pos_, P_MIN_4340, P_MAX_4340, 16);
    vel_tmp = floatToUint(vel_, V_MIN_4340, V_MAX_4340, 12);
    kp_tmp = floatToUint(kp_, KP_MIN_4340, KP_MAX_4340, 12);
    kd_tmp = floatToUint(kd_, KD_MIN_4340, KD_MAX_4340, 12);
    tor_tmp = floatToUint(torq_, T_MIN_4340, T_MAX_4340, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = (((vel_tmp & 0xF) << 4) | (kp_tmp >> 8));
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = (((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    data[7] = tor_tmp;

    // return data;
}

/// <summary>
/// 所有电机 MIT 控制
/// </summary>
/// <returns></returns>
void MotorControl::ControlMotors(serialport::SerialPortWrapper &port, float pos[6], float vel[6], float kp[6], float kd[6], float tau[6])
{
    // uint8_t enableData[8] = {0};
    int motorIndex = 0;
    uint8_t motorDate_1[8] = {0};
    uint8_t motorDate_2[8] = {0};
    uint8_t motorDate_3[8] = {0};
    uint8_t motorDate_4[8] = {0};
    uint8_t motorDate_5[8] = {0};
    uint8_t motorDate_6[8] = {0};

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        if(pos[i]>desir_motor_pos_protect_max[i]){
            std::cout << "joint" << i << " desir pos out range:" << " " << pos[i]<<std::endl;
            pos[i] = desir_motor_pos_protect_max[i];
        }
        if (pos[i] < desir_motor_pos_protect_min[i])
        {
            std::cout << "joint" << i << " desir pos out range:" << " " << pos[i] << std::endl;
            pos[i] = desir_motor_pos_protect_min[i];
        }

        if (vel[i] > desir_motor_vel_protect_max[i])
        {
            std::cout << "joint" << i << " desir vel out range:" << " " << vel[i] << std::endl;
            vel[i] = desir_motor_vel_protect_max[i];
        }
        if (vel[i] < desir_motor_vel_protect_min[i])
        {
            std::cout << "joint" << i << " desir vel out range:" << " " << vel[i] << std::endl;
            vel[i] = desir_motor_vel_protect_min[i];
        }

        if (tau[i] > desir_motor_tau_protect_max[i])
        {
            std::cout << "joint" << i << " desir tau out range:" << " " << tau[i] << std::endl;
            tau[i] = desir_motor_tau_protect_max[i];
        }
        if (tau[i] < desir_motor_tau_protect_min[i])
        {
            std::cout << "joint" << i << " desir tau out range:" << " " << tau[i] << std::endl;
            tau[i] = desir_motor_tau_protect_min[i];
        }

    }

    motorIndex = 0;
    MitCtrl4340(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_1);
    motorIndex = 1;
    MitCtrl4340(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_2);
    motorIndex = 2;
    MitCtrl4340(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_3);
    motorIndex = 3;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_4);
    motorIndex = 4;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_5);
    motorIndex = 5;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_6);

    std::vector<uint8_t> data;
    data.resize(port.FRAME_DATA_LENGTH);
    std::vector<uint8_t> frame;

    for (int i = 0; i < 8; i++)
    {
        motorIndex = 0;
        data[motorIndex + i] = motorDate_1[i]; // 电机1
        motorIndex = 8;
        data[motorIndex + i] = motorDate_2[i]; // 电机2
        motorIndex = 16;
        data[motorIndex + i] = motorDate_3[i]; // 电机3
        motorIndex = 24;
        data[motorIndex + i] = motorDate_4[i]; // 电机4
        motorIndex = 32;
        data[motorIndex + i] = motorDate_5[i]; // 电机5
        motorIndex = 40;
        data[motorIndex + i] = motorDate_6[i]; // 电机6
    }

    port.packFrame(data, frame);

    // 发送数据
    port.sendData(frame);
}

/// @brief 可以控制夹爪
/// @param port 
/// @param pos 
/// @param vel 
/// @param kp 
/// @param kd 
/// @param tau 
/// @param grp 加爪0-250：闭合-张开
void MotorControl::ControlMotors_g(serialport::SerialPortWrapper &port, float pos[6], float vel[6], float kp[6], float kd[6], float tau[6], uint8_t grp)
{
    // uint8_t enableData[8] = {0};
    int motorIndex = 0;
    uint8_t motorDate_1[8] = {0};
    uint8_t motorDate_2[8] = {0};
    uint8_t motorDate_3[8] = {0};
    uint8_t motorDate_4[8] = {0};
    uint8_t motorDate_5[8] = {0};
    uint8_t motorDate_6[8] = {0};
    uint8_t motorDate_7[8] = {0};

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {
        if(pos[i]>desir_motor_pos_protect_max[i]){
            std::cout << "joint" << i << " desir pos out range:" << " " << pos[i]<<std::endl;
            pos[i] = desir_motor_pos_protect_max[i];
        }
        if (pos[i] < desir_motor_pos_protect_min[i])
        {
            std::cout << "joint" << i << " desir pos out range:" << " " << pos[i] << std::endl;
            pos[i] = desir_motor_pos_protect_min[i];
        }

        if (vel[i] > desir_motor_vel_protect_max[i])
        {
            std::cout << "joint" << i << " desir vel out range:" << " " << vel[i] << std::endl;
            vel[i] = desir_motor_vel_protect_max[i];
        }
        if (vel[i] < desir_motor_vel_protect_min[i])
        {
            std::cout << "joint" << i << " desir vel out range:" << " " << vel[i] << std::endl;
            vel[i] = desir_motor_vel_protect_min[i];
        }

        if (tau[i] > desir_motor_tau_protect_max[i])
        {
            std::cout << "joint" << i << " desir tau out range:" << " " << tau[i] << std::endl;
            tau[i] = desir_motor_tau_protect_max[i];
        }
        if (tau[i] < desir_motor_tau_protect_min[i])
        {
            std::cout << "joint" << i << " desir tau out range:" << " " << tau[i] << std::endl;
            tau[i] = desir_motor_tau_protect_min[i];
        }

    }

    motorIndex = 0;
    MitCtrl4340(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_1);
    motorIndex = 1;
    MitCtrl4340(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_2);
    motorIndex = 2;
    MitCtrl4340(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_3);
    motorIndex = 3;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_4);
    motorIndex = 4;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_5);
    motorIndex = 5;
    MitCtrl(pos[motorIndex], vel[motorIndex], kp[motorIndex], kd[motorIndex], tau[motorIndex], motorDate_6);

    std::vector<uint8_t> data;
    data.resize(port.FRAME_DATA_LENGTH);
    std::vector<uint8_t> frame;

    motorDate_7[0] = grp;

    for (int i = 0; i < 8; i++)
    {
        motorIndex = 0;
        data[motorIndex + i] = motorDate_1[i]; // 电机1
        motorIndex = 8;
        data[motorIndex + i] = motorDate_2[i]; // 电机2
        motorIndex = 16;
        data[motorIndex + i] = motorDate_3[i]; // 电机3
        motorIndex = 24;
        data[motorIndex + i] = motorDate_4[i]; // 电机4
        motorIndex = 32;
        data[motorIndex + i] = motorDate_5[i]; // 电机5
        motorIndex = 40;
        data[motorIndex + i] = motorDate_6[i]; // 电机6
        motorIndex = 48;
        data[motorIndex + i] = motorDate_7[i]; // 电机6
    }

    port.packFrame(data, frame);

    // 发送数据
    port.sendData(frame);
}

void MotorControl::updatMotorState(std::vector<float> motorState)
{
    // 不更新电机加速度
    current_motor_pos[0] = motorState[0];
    current_motor_pos[1] = motorState[3];
    current_motor_pos[2] = motorState[6];
    current_motor_pos[3] = motorState[9];
    current_motor_pos[4] = motorState[12];
    current_motor_pos[5] = motorState[15];

    current_motor_vel[0] = motorState[1];
    current_motor_vel[1] = motorState[4];
    current_motor_vel[2] = motorState[7];
    current_motor_vel[3] = motorState[10];
    current_motor_vel[4] = motorState[13];
    current_motor_vel[5] = motorState[16];

    current_motor_tau[0] = motorState[2];
    current_motor_tau[1] = motorState[5];
    current_motor_tau[2] = motorState[8];
    current_motor_tau[3] = motorState[11];
    current_motor_tau[4] = motorState[14];
    current_motor_tau[5] = motorState[17];

    // std::cout << "motorState:" << motorState[9]<< motorState[12]<< motorState[15] << std::endl;

}

void MotorControl::real_time_motor_protection(float pos_[MOTOR_NUM], float vel_[MOTOR_NUM], float tau_[MOTOR_NUM], serialport::SerialPortWrapper &port)
{
    static int tau_dengerous_cout[MOTOR_NUM] = {0};
    static int vel_dengerous_cout[MOTOR_NUM] = {0};
    static int pos_dengerous_cout[MOTOR_NUM] = {0};

    int vel_max_dengerous_cout = 10;
    int tau_max_dengerous_cout = 10;
    int pos_max_dengerous_cout = 5;

    int bad_motor[MOTOR_NUM] = {0};

    static bool out_flag = false;

    if(out_flag) return;

    // std::cout << "real_time_motor_protection: " << tau_[0] << " " << tau_[1] << " " << tau_[2]<< std::endl;

    for (size_t i = 0; i < MOTOR_NUM; i++)
    {

         /***  位置警告保护  ***/
        if ((pos_[i]) > current_motor_pos_protect_max[i])
        {
            pos_dengerous_cout[i]++;
            std::cout << "joint pos " << i << " out range must Ctr + c kill programe!!!!" << pos_[i]<< std::endl;

            if (pos_dengerous_cout[i] >= pos_max_dengerous_cout)
            {
                //  dengerous_cout = 0;
                for (size_t i = 0; i < 10; i++)
                {
                    // DisableMotors(port);
                    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                bad_motor[i] = 1;
                out_flag = true;
            }
        }else{
            //标志位置零
            pos_dengerous_cout[i] = 0;
        }

        if ((pos_[i]) < current_motor_pos_protect_min[i])
        {
            pos_dengerous_cout[i]++;
            std::cout << "joint pos " << i << " out range must Ctr + c kill programe!!!!" << pos_[i]<< std::endl;

            if (pos_dengerous_cout[i] >= pos_max_dengerous_cout)
            {
                //  dengerous_cout = 0;
                for (size_t i = 0; i < 10; i++)
                {
                    // DisableMotors(port);
                    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                bad_motor[i] = 1;
                out_flag = true;
            }
        }else{
            //标志位置零
            pos_dengerous_cout[i] = 0;
        }

        /***  速度力矩保护  ***/

        if (abs(vel_[i]) > current_motor_vel_protect_max[i])
        {
            vel_dengerous_cout[i]++;
            std::cout << "joint vel " << i << " out range must Ctr + c kill programe!!!!" << vel_[i]<< std::endl;

            if (vel_dengerous_cout[i] >= vel_max_dengerous_cout)
            {
                //  dengerous_cout = 0;
                for (size_t i = 0; i < 10; i++)
                {
                    DisableMotors(port);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                bad_motor[i] = 1;
                out_flag = true;
            }
        }
        else
        {
            //标志位置零
            vel_dengerous_cout[i] = 0;
        }

        if (abs(tau_[i]) > current_motor_tau_protect_max[i])
        {
            tau_dengerous_cout[i]++;
            std::cout << "joint tau " << i << " dengerous must Ctr + c kill programe!!!!"<< tau_[i] << std::endl;

            if (tau_dengerous_cout[i] >= tau_max_dengerous_cout)
            {
                // dengerous_cout = 0;
                //加循环，失能电机
                for (size_t i = 0; i < 10; i++)
                {
                    DisableMotors(port);
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                bad_motor[i] = 1;
                out_flag = true;
            }
        }
        else
        {
            // 标志位置零
            tau_dengerous_cout[i] = 0;
        }

        // if (print_massege)
        // {
        //     DisableMotors(port);
        // }
    }







}

void MotorControl::KalmanFilter(float process_noise, float measurement_noise, float estimation_error, float initial_value, float limit) {
    Q = process_noise;
    R = measurement_noise;
    P = estimation_error;
    X = initial_value;
    threshold = limit;
}

float MotorControl::update(float measurement) {
    // 预测步骤
    P = P + Q;

    // 计算卡尔曼增益
    K = P / (P + R);

    // 更新状态估计
    X = X + K * (measurement - X);

    // 更新状态协方差
    P = (1 - K) * P;

    // std::cout << "threshold: " << threshold<< std::endl;

    //阈值滤波
    if (std::abs(X) < threshold)
    {
        X = 0.0;
    }

    return X;
}

void MotorControl::compensate_static_friction_through_vel(float filtered_motor_vel[MOTOR_NUM], float motor_tor[MOTOR_NUM])
{
    for (size_t i = 0; i < 3; i++)
    {
        if (filtered_motor_vel[i] > 0)
            motor_tor[i] += static_friction_in_gravity_max[i];
        if (filtered_motor_vel[i] < 0)
            motor_tor[i] += static_friction_in_gravity_min[i];
    }
}

void MotorControl::ButterworthFilter(float cutoff_frequency, float sampling_rate) {
    float wc = tan(M_PI * cutoff_frequency / sampling_rate);
    float k1 = sqrt(2.0) * wc;
    float k2 = wc * wc;
    a0 = k2 / (1.0 + k1 + k2);
    a1 = 2.0 * a0;
    a2 = a0;
    b1 = 2.0 * (k2 - 1.0) / (1.0 + k1 + k2);
    b2 = (1.0 - k1 + k2) / (1.0 + k1 + k2);
}

float MotorControl::Butt_update(float new_value) {
    float result = a0 * new_value + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
    x2 = x1;
    x1 = new_value;
    y2 = y1;
    y1 = result;
    return result;
}





