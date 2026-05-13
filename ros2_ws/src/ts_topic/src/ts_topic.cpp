#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include <cmath>
#include "ts_topic/mousekey.h"


using namespace std::chrono_literals;

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


/***
 *   cat /proc/bus/input/devices
    这将列出所有的输入设备信息，输出格式类似：

    sudo cat /dev/input/event10
    然后移动鼠标或点击按钮，查看是否有输出。如果有输出，则设备正在生成事件。

    sudo chmod 666 /dev/input/event10
    更改设备文件的权限：
 * */
class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
  : Node("joint_state_publisher"),
  mouse("/dev/input/event12"),
  key("/dev/input/event9")
  {
    // 创建发布者，发布到 "joint_states" 话题上，队列大小为 10
    topic_control_desir_joint_state_publisher = this->create_publisher<sensor_msgs::msg::JointState>("topic_control_desir_joint_state", 10);

    topic_control_desir_end_pos_publisher = this->create_publisher<geometry_msgs::msg::Pose>("topic_control_desir_end_pos", 10);

    cmd_publisher = this->create_publisher<std_msgs::msg::String>("cmd_topic", 10);

    // 定时器：每 1 秒调用一次回调函数
    timer_ = this->create_wall_timer(1ms, std::bind(&JointStatePublisher::end_pos, this));

  }

private:

  void joint_state()
  {
    float realtime = get_duration_realtime(0.001, 3.1415926 * 4);

    auto message = sensor_msgs::msg::JointState();
    // 设置时间戳
    message.header.stamp = this->now();
    // 填写关节数据：名称、位置、速度、力矩（此处示例两个关节）
    message.name = {"joint1", "joint2","joint3", "joint4","joint5", "joint6"};
    message.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.5*sin(realtime)};
    message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    message.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    topic_control_desir_joint_state_publisher->publish(message);

    RCLCPP_INFO(this->get_logger(), "Publishing JointState message");
  }

  void end_pos()
  {
      float realtime = get_duration_realtime(0.001, 3.1415926 * 4);

      auto message = geometry_msgs::msg::Pose();

      auto cmd_ = std_msgs::msg::String();

      static bool is_control = false;

      // 初始化发送的数据值,注意这里的初始值一定要和机械臂的初始位置一致
      message.position.x = key.current_postion[0];

      message.position.y = key.current_postion[1];

      message.position.z = key.current_postion[2];

      //这里是用四元数的格式储存欧拉角，所以w不赋值。
      message.orientation.w = key.current_postion[3]; 

      message.orientation.x = mouse.current_rpy[0]; 

      message.orientation.y = mouse.current_rpy[1]; 

      message.orientation.z = -mouse.current_rpy[2];

      topic_control_desir_end_pos_publisher->publish(message);

      mouse.get_mouse_data();

      key.get_key_data();

      if (is_control)
      {
   

        mouse.mouse2rotation();

        key.key2position();

      }

      // RCLCPP_INFO(this->get_logger(), "xyz rpy; %f, %f, %f, %f, %f, %f", key.current_postion[0],  key.current_postion[1], key.current_postion[2], mouse.current_rpy[0], mouse.current_rpy[1], mouse.current_rpy[2]);

      // RCLCPP_INFO(this->get_logger(), "mouse rpy, %f, %f, %f,", mouse.current_rpy[0], mouse.current_rpy[1], mouse.current_rpy[2]);

      // RCLCPP_INFO(this->get_logger(), "key xyz, %f, %f, %f,",  key.current_postion[0],  key.current_postion[1], key.current_postion[2]);



      //复位
      if (key.key_state)
      {
         std::cout << "键盘按下：" << key.last_key << std::endl;

        if (key.last_key == 19)//r：复位姿态
        {
          for (size_t i = 0; i < 3; i++)
          {
            mouse.current_rpy[i] = mouse.init_rpy[i];
          }
        }
        if (key.last_key == 33)//f：复位位置
        {
          for (size_t i = 0; i < 3; i++)
          {
            key.current_postion[i] = key.init_postion[i];
          }
        }

        if (key.last_key == 38)//l：加载估计的末端负载
        {
          cmd_.data = "laod";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cmd_publisher->publish(cmd_);
        }
        if (key.last_key == 37)//k:取消加载估计的末端负载
        {
          cmd_.data = "clearlaod";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cmd_publisher->publish(cmd_);
        }

        if (key.last_key == 20)//t：进入重力补偿模式
        {
          cmd_.data = "Gravity";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cmd_publisher->publish(cmd_);
        }
        if (key.last_key == 21)//t：进入阻抗
        {
          cmd_.data = "Impdence";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cmd_publisher->publish(cmd_);
        }

        if (key.last_key == 25)//p 回到关节零点，并进入关节控制模式
        {
          cmd_.data = "JointAutoServo";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cmd_publisher->publish(cmd_);
        }

        if (key.last_key == 57)//空格：解锁鼠标控制
        {
          is_control = !is_control;
          std::cout << "空格按下：" << key.last_key << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }


      }

  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_control_desir_joint_state_publisher;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr topic_control_desir_end_pos_publisher;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_publisher;

  rclcpp::TimerBase::SharedPtr timer_;

  MouseKey mouse;
  MouseKey key;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
