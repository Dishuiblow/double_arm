#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdbool.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>



using namespace LibSerial;
// // 串口设置
// const std::string PORT_NAME = "/dev/ttyACM0"; // 根据实际情况修改
// const BaudRate BAUD_RATE = BaudRate::BAUD_115200;
// const int DATA_SIZE = FRAME_SIZE; // 每次收发数据的字节数

namespace serialport
{

    class SerialPortWrapper
    {
    public:
        // 常量定义
        static constexpr uint8_t FRAME_HEAD = 0xAA;
        static constexpr uint8_t FRAME_TAIL = 0x55;
        static constexpr uint16_t FRAME_DATA_LENGTH = 59;
        static constexpr uint16_t FRAME_SIZE = 1 + 1 + FRAME_DATA_LENGTH + 2 + 1;
        // static const size_t FRAME_DATA_LENGTH = 64;

        // SerialPortWrapper(const std::string& port_name, unsigned int baud_rate);
        SerialPortWrapper();
        ~SerialPortWrapper();
        using DataBuffer = std::vector<uint8_t>;

        void writeData(const std::string &data);
        void sendData(const DataBuffer &buffer);
        std::string readData();
        std::vector<uint8_t> receiveData();

        // 公共方法
        void packFrame(const std::vector<uint8_t> &data, std::vector<uint8_t> &frame);
        bool parseFrame(const std::vector<uint8_t> &frame, std::vector<uint8_t> &data);

        uint16_t calculateCRC16(const uint8_t *data, uint16_t length);

        void stopReceiving();
        void startReceiving();
        std::vector<uint8_t> getReceivedData() ;

    private:
        // 私有方法

        LibSerial::SerialPort serialPort;

        // 串口设置
        const std::string PORT_NAME = "/dev/ttyACM0"; // 根据实际情况修改
        const BaudRate BAUD_RATE = BaudRate::BAUD_921600;
        const unsigned char DATA_SIZE = FRAME_SIZE; // 每次收发数据的字节数

        std::atomic<bool> running;         // 用于控制线程运行的标志
        std::thread receiveThread;         // 接收线程
        std::mutex dataMutex;              // 数据保护互斥锁
        std::vector<uint8_t> receivedData;      // 存储接收到的数据（共享资源）
        std::vector<uint8_t> receivedmotorData; // 存储接收到的数据（共享资源）

        // CRC 查找表
        static const uint16_t crc16_table[256];

        void receiveLoop();

    };

} // namespace libserial_example

#endif // LIBSERIAL_EXAMPLE_SERIALPORT_H
