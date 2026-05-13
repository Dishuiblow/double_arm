#ifndef TS_TOPIC_MOUSEKEY_H
#define TS_TOPIC_MOUSEKEY_H

#include <libinput.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <cstdlib>

class MouseKey
{
public:
    MouseKey(const char* device_path);
    ~MouseKey();

    void initialize();
    // 事件处理循环
    void processEvents();

      // 获取鼠标当前移动坐标 (dx, dy)
    void get_mouse_data();

    void get_key_data();

    void mouse2rotation();

    void key2position();

     //这里的数值是鼠标每次移动时，当前位置与上一次位置的差值
    float mouse_dlt_x = 0;
    float mouse_dlt_y = 0;

    bool is_click_left = false;
    bool is_click_right = false;

    float mouse_scroll = 0;

    int last_key=0;      // 存储最近按下的键值
    bool key_state=false;

    float init_rpy[3] = {0.0, 0.0, 0.0}; // 初始化欧拉角
    float current_rpy[3] = {init_rpy[0], init_rpy[1], init_rpy[2]}; 

    float init_postion[4] = {0.4, 0.0, 0.2, 30}; 
    float current_postion[4] = {init_postion[0], init_postion[1], init_postion[2],init_postion[3]}; // 初始化欧拉角



    
private:
    struct libinput *li_;     // libinput 上下文
    const char *device_path_; // 设备路径

    // 静态回调函数：打开设备文件
    static int openRestricted(const char *path, int flags, void *user_data);

    // 静态回调函数：关闭设备文件
    static void closeRestricted(int fd, void *user_data);

    // 静态 libinput 接口结构体
    static const struct libinput_interface interface_;
};

#endif // TS_TOPIC_MOUSEKEY_H