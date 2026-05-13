#include "ts_topic/mousekey.h"

// 静态 libinput 接口结构体
const struct libinput_interface MouseKey::interface_ = {
    .open_restricted = MouseKey::openRestricted,
    .close_restricted = MouseKey::closeRestricted,
};

MouseKey::MouseKey(const char *device_path) : li_(nullptr), device_path_(device_path)
{
    // 创建 libinput 上下文，传入自定义的接口
    int device_;
    struct libinput *libinput_;
    struct libinput_device *de;

    /*********************************************************************************************/
    /*********************************************************************************************/
    // 创建libinput上下文
    // 1. 创建 libinput 上下文（此处未传递自定义 log_handler，传递空指针即可）
    li_ = libinput_path_create_context(&interface_, this);
    // struct libinput *li = libinput_path_create_context(nullptr, nullptr);
    if (!li_)
    {
        std::cerr << "无法创建 libinput 上下文" << std::endl;
        return;
    }

    /*********************************************************************************************/
    /*********************************************************************************************/
    // 添加设备
    if (!libinput_path_add_device(li_, device_path_))
    {
        std::cerr << "无法添加设备: " << device_path_ << std::endl;
        libinput_unref(li_);
        std::exit(EXIT_FAILURE);
    }
    std::cout << "成功创建 libinput 上下文，并添加设备 " << device_path_ << std::endl;

    std::cout << "设备打开成功！！" << std::endl;
}

MouseKey::~MouseKey()
{
    // 如果不需要特别的清理工作，可以留空
    if (li_)
    {
        libinput_unref(li_);
    }
}

// 静态回调函数：打开设备文件
int MouseKey::openRestricted(const char *path, int flags, void *user_data)
{
    int fd = open(path, flags);
    if (fd < 0)
        return -errno;
    return fd;
}

// 静态回调函数：关闭设备文件
void MouseKey::closeRestricted(int fd, void *user_data)
{
    close(fd);
}

void MouseKey::get_mouse_data()
{

    libinput_dispatch(li_);
    struct libinput_event *event = nullptr;

    if ((event = libinput_get_event(li_)) != nullptr)
    {
        uint32_t type = libinput_event_get_type(event);

        // 检测坐标
        if (type == LIBINPUT_EVENT_POINTER_MOTION)
        {
            auto pointer_event = libinput_event_get_pointer_event(event);
            mouse_dlt_x = libinput_event_pointer_get_dx(pointer_event);
            mouse_dlt_y = libinput_event_pointer_get_dy(pointer_event);
        }

        // 检测左右键
        if (type == LIBINPUT_EVENT_POINTER_BUTTON)
        {
            auto pointer_event = libinput_event_get_pointer_event(event);
            uint32_t button = libinput_event_pointer_get_button(pointer_event);
            uint32_t state = libinput_event_pointer_get_button_state(pointer_event);
            if (button == BTN_LEFT)
            {
                is_click_left = (state == LIBINPUT_BUTTON_STATE_PRESSED);
            }
            else if (button == BTN_RIGHT)
            {
                is_click_right = (state == LIBINPUT_BUTTON_STATE_PRESSED);
            }
        }

        // 检测滚轮
        if (type == LIBINPUT_EVENT_POINTER_AXIS) {
            auto pointer_event = libinput_event_get_pointer_event(event);
            
            if (libinput_event_pointer_has_axis(pointer_event, LIBINPUT_POINTER_AXIS_SCROLL_VERTICAL)) {
                mouse_scroll = libinput_event_pointer_get_axis_value(pointer_event, LIBINPUT_POINTER_AXIS_SCROLL_VERTICAL);
                // std::cout << "mouse_scroll_y" <<mouse_scroll_y<< std::endl;
            }
            // if (libinput_event_pointer_has_axis(pointer_event, LIBINPUT_POINTER_AXIS_SCROLL_HORIZONTAL)) {
            //     mouse_scroll_x = libinput_event_pointer_get_axis_value(pointer_event, LIBINPUT_POINTER_AXIS_SCROLL_HORIZONTAL);
            // }
        }

        libinput_event_destroy(event);
    }
}

void MouseKey::get_key_data()
{

    libinput_dispatch(li_);
    struct libinput_event *event = nullptr;

    if ((event = libinput_get_event(li_)) != nullptr)
    {
        uint32_t type = libinput_event_get_type(event);

        if (type == LIBINPUT_EVENT_KEYBOARD_KEY)
        {
            auto keyboard_event = libinput_event_get_keyboard_event(event);
            last_key = libinput_event_keyboard_get_key(keyboard_event);
            uint32_t state = libinput_event_keyboard_get_key_state(keyboard_event);

            key_state = (state == LIBINPUT_KEY_STATE_PRESSED) ? true : false;
        
        }

        libinput_event_destroy(event);
    }
}

void MouseKey::mouse2rotation()
{

    float rpy_step[3] = {0.003,0.00051,0.00051};
    float rpy_limit[3] = {0.6,0.6,0.6};

    current_rpy[1] += rpy_step[1] * mouse_dlt_y;

    current_rpy[2] += rpy_step[2] * mouse_dlt_x;

    current_rpy[0] += rpy_step[0] * mouse_scroll;

    mouse_scroll = 0.0;//复位滚轮
    mouse_dlt_x = 0.0;//复位x
    mouse_dlt_y = 0.0;//复位y

    if (current_rpy[0]>rpy_limit[0])
    {
        current_rpy[0] = rpy_limit[0];
    }
    if (current_rpy[0]<-rpy_limit[0])
    {
        current_rpy[0] = -rpy_limit[0];
    }

    if (current_rpy[1]>rpy_limit[1])
    {
        current_rpy[1] = rpy_limit[1];
    }
    if (current_rpy[1]<-rpy_limit[1])
    {
        current_rpy[1] = -rpy_limit[1];
    }

    if (current_rpy[2]>rpy_limit[2])
    {
        current_rpy[2] = rpy_limit[2];
    }
    if (current_rpy[2]< -rpy_limit[2])
    {
        current_rpy[2] = -rpy_limit[2];
    }

}

void MouseKey::key2position()
{
    float xyz_step[4] = {0.0001,0.0001,0.0001 ,0.2};

    float xyz_max[4] = {0.58,0.2, 0.3 ,200};
    float xyz_min[4] = {0.28,-0.2,0.0, 0};


    if (key_state)
    {
         
        if(last_key==17)//w x轴
        {
            current_postion[0] += xyz_step[0];
        }

        if(last_key==31)//s x轴
        {
            current_postion[0] -= xyz_step[0];
        }

        if(last_key==30)//a y轴
        {
            current_postion[1] += xyz_step[1];
        }

        if(last_key==32)//d y轴
        {
            current_postion[1] -= xyz_step[1];
        }

        if(last_key==16)//q z轴
        {
            current_postion[2] += xyz_step[2];
        }

        if(last_key==18)//e z轴
        {
            current_postion[2] -= xyz_step[2];
        }

        //控制加爪开合
        if(last_key==44)//z
        {
            current_postion[3] += xyz_step[3];
        }

        if(last_key==46)//c
        {
            current_postion[3] -= xyz_step[3];
        }



        if (current_postion[0] > xyz_max[0])
        {
            current_postion[0] = xyz_max[0];
        }
        if (current_postion[0] < xyz_min[0])
        {
            current_postion[0] = xyz_min[0];
        }

        if (current_postion[1] > xyz_max[1])
        {
            current_postion[1] = xyz_max[1];
        }
        if (current_postion[1] < xyz_min[1])
        {
            current_postion[1] = xyz_min[1];
        }

        if (current_postion[2] > xyz_max[2])
        {
            current_postion[2] = xyz_max[2];
        }
        if (current_postion[2] < xyz_min[2])
        {
            current_postion[2] = xyz_min[2];
        }

        if (current_postion[3] > xyz_max[3])
        {
            current_postion[3] = xyz_max[3];
        }
        if (current_postion[3] < xyz_min[3])
        {
            current_postion[3] = xyz_min[3];
        }



        
        



    }
}
