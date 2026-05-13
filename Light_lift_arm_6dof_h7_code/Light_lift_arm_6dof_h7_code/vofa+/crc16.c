#include "crc16.h"
#include <stdbool.h>


#define RX_BUFFER_SIZE 256 
uint8_t rx_buffer[RX_BUFFER_SIZE]; 
volatile uint16_t rx_write_index = 0; 
volatile uint16_t rx_read_index = 0;  
uint8_t frame_buffer[FRAME_SIZE];     

volatile uint8_t USB_Rcive_Data[64]; 



// 计算 CRC16 的函数
uint16_t crc16_calculate(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    for (uint16_t i = 0; i < length; i++) {
        uint8_t index = (crc >> 8) ^ data[i]; // 高字节异或
        crc = (crc << 8) ^ crc16_table[index];
    }
    return crc;
}


void Frame_Pack(uint8_t *frame, const uint8_t *data) {
    frame[0] = FRAME_HEAD; // 帧头
    frame[1] = FRAME_DATA_LENGTH; // 数据长度

    // 拷贝数据区
    for (int i = 0; i < FRAME_DATA_LENGTH; i++) {
        frame[2 + i] = data[i];
    }

    // 计算 CRC16 校验
    uint16_t crc = crc16_calculate(data, FRAME_DATA_LENGTH);
    frame[2 + FRAME_DATA_LENGTH] = crc & 0xFF;         // CRC 低字节
    frame[2 + FRAME_DATA_LENGTH + 1] = (crc >> 8) & 0xFF; // CRC 高字节
    frame[2 + FRAME_DATA_LENGTH + 2] = FRAME_TAIL; // 帧尾
}




void RingBuffer_Write(uint8_t *data, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        rx_buffer[rx_write_index] = data[i];
        rx_write_index = (rx_write_index + 1) % RX_BUFFER_SIZE;
    }
}

uint16_t RingBuffer_Read(uint8_t *data, uint16_t length) {
    uint16_t count = 0;
    while (rx_read_index != rx_write_index && count < length) {
        data[count++] = rx_buffer[rx_read_index];
        rx_read_index = (rx_read_index + 1) % RX_BUFFER_SIZE;
    }
    return count;
}


void Process_Frame(void) {
    static uint8_t temp_buffer[FRAME_SIZE];
    static uint16_t temp_index = 0;

    while (rx_read_index != rx_write_index) {

        temp_buffer[temp_index++] = rx_buffer[rx_read_index];
        rx_read_index = (rx_read_index + 1) % RX_BUFFER_SIZE;

        if (temp_index == 1 && temp_buffer[0] != FRAME_HEAD) {
            temp_index = 0; 
        }

        if (temp_index == FRAME_SIZE) {
            if (temp_buffer[FRAME_SIZE - 1] == FRAME_TAIL) {
          
                uint8_t received_data[FRAME_DATA_LENGTH];
                if (Frame_Parse(temp_buffer, received_data)) {

											for(int i=0; i<FRAME_DATA_LENGTH; i++){
															USB_Rcive_Data[i] = received_data[i];
											}

                }
            }
            temp_index = 0; 
        }
    }
}






bool Frame_Parse(const uint8_t *frame, uint8_t *data) {
    // 校验帧头和帧尾
    if (frame[0] != FRAME_HEAD || frame[FRAME_SIZE - 1] != FRAME_TAIL) {
        return false; // 帧头或帧尾错误
    }

    // 获取数据长度
    uint8_t length = frame[1];
    if (length != FRAME_DATA_LENGTH) {
        return false; // 数据长度错误
    }

    // 校验 CRC
    uint16_t received_crc = frame[2 + FRAME_DATA_LENGTH] | (frame[2 + FRAME_DATA_LENGTH + 1] << 8);
    uint16_t calculated_crc = crc16_calculate(frame + 2, FRAME_DATA_LENGTH);
    if (received_crc != calculated_crc) {
        return false; // 校验码错误
    }

    // 提取数据区
    for (int i = 0; i < FRAME_DATA_LENGTH; i++) {
        data[i] = frame[2 + i];
    }

    return true;
}











