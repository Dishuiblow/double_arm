# 轻擎六轴力控机械臂

稳定的1k的控制频率，可跑各种动力学算法，比如超螺旋滑模算法

基于电流环的力控：末端阻抗（刚度阻尼控制），导纳和力位混控

可以自动估计末端负载

重力补偿

预留夹爪控制数据位

ROS2



# 安装串口库，直接安装到系统环境里

```bash
sudo apt-get install libserial-dev
```
 
 
 # 安装eigen库，直接安装到系统环境里
```bash
sudo apt-get update
sudo apt-get install libeigen3-dev libcppunit-dev
```
 # 源码安装kdl库，直接在ros2工作空间里安装（也可以直接安装到系统环境中）

项目中已经自带了，不用再下载了直接colcon build即可，

只需要将前面需要安装到系统环境的包安装即可。


如果直接安装到系统，cmake的配置可能会简单些。
本项目的cmake是按照kdl安装在工作空间的情况设置的。

所以需要配置 cmake 的 orocos_kdl_DIR 变量让他在此工作空间的install目录中寻找库（cmake默认是在系统路径寻找的），不过你也可以先把库都配好，直接编译运行一下试试。

*****************
工作空间中已经下载好了，不用再下载了

 先建立ros2的项目文件目录

git clone https://github.com/orocos/orocos_kinematics_dynamics.git

下载到工作空间的src里面，然后在上一级目录进行
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
*****************
  # 安装urdf库，直接安装到系统环境里
```bash
sudo apt-get update
sudo apt-get install ros-<your_ros_distro>-urdf
```

# 安装kdl解析urdf的库，直接安装到系统环境里
```bash
 sudo apt install ros-humble-kdl-parser
```

所有环境配置好后，试试能不能通过编译。

# 第一次启动程序

**前提是先把下位机的代码烧到下位机里。**

在工作空间中执行：
```cpp
source install/setup.bash 
```
确认 light_lift_arm_6dof_node.cpp 运行的模式是什么：
```cpp
        //     AutoServo,      //电机内部mit
        //     ManualServo,    //手动通过纯力矩控制电机
        //     Impdence,       //阻抗
        //     Admittance,     //导纳
        //     Gravity,        //重力补偿
        //     JointAutoServo, //关节自动控制,注意要初始化关节位置
        //     Zero            //设置零点
        //     Teach        //示教
        //     Planning       //示教

        mode_ = State::Zero;

        control_mode.modeTransition(mode_);
```
上述代码表示程序以校零模式运行：
# 在第一次使用机械臂之前，必须要给各个电机校零！

因为不确定电机当前的角度是多少，所以有可能超过关节限制的角度，所以需要注释如下代码：
```cpp
//158行左右
 motorControl.real_time_motor_protection(motorControl.current_motor_pos, motorControl.current_motor_vel, motorControl.current_motor_tau, serial);
```
上述函数的作用是在机械臂关节超过限制的角度，速度和力矩时失能电机。注释掉之后就不会因为电机角度超限而失能电机了，但是注意《 完 成 校 零 后 要 取 消 注 释 》。

再执行，注意一定要**插上下位机（达妙的h7开发版）**：
```cpp
//前面的配置过程，应该已经成功编译通过了
 ./run.sh 
```

机械臂电机灯都为绿色，即表示机械臂启动成功，手动将机械臂拖到零位即可。

ctrl c退出程序。注意在退出程序前，机械臂的位置都不要变，因为校零是循环运行的。

完成校零后就可以运行其他模式了。




*****************

如果想用foxglove显示话题，必须要运行发消息的插件：
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```



 



