#pragma once
#include <libserial/SerialPort.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <thread>

#include "../road/imgprocess.hpp"
#include "serial_packets.hpp"
#include "stop_watch.hpp"

#undef LOG_TAG
#define LOG_TAG "Serial"

#define PWM_MIDDLE 1500

using namespace LibSerial;

// 引用图像处理
extern ImageProcess imgprocess;
#define IMG imgprocess

typedef union
{
    uint8_t U8_Buff[2];
    uint16_t U16;
} Bint16_Union;

typedef union
{
    uint8_t U8_Buff[4];
    float Float;
} Bint32_Union;

class SerialDriver
{
public:
    SerialDriver(const std::string& port_name, BaudRate bps) : _port_name(port_name), _bps(bps) {};
    ~SerialDriver() {};

private:
    // 通讯协议结构体

    // OLD send packet struct
    // struct Packet {
    //     uint8_t dirty;     // 脏数据
    //     uint8_t head[2];   // 头
    //     uint8_t data[5];   // 数据
    //     uint8_t check[2];  // 校验
    //     uint8_t tail;      // 尾
    // };

    // NEW send packet strucrt
    struct Packet
    {
        uint8_t head;
        uint8_t addr;
        uint8_t length;
        uint8_t buf_speed[4];
        uint8_t buf_direction[2];
        uint8_t sumcheck = 0;
    };

    // 4字节转int32
    typedef union
    {
        unsigned char uc[4];
        int32_t i;
    } byte2int32;

private:
    std::shared_ptr<SerialPort> _serial_port = nullptr;
    std::string _port_name; // 端口号
    BaudRate _bps;          // 波特率

    bool _loop;
    bool _can_close;

    std::thread _worker_thread_send;
    std::thread _worker_thread_rece;

    // BlockingQueue<Payload_t> _queue;
    BlockingQueue<Payload> _queue;
private:

    void run_send()
    {
        while (_loop)
        {
            if (_queue.Size() >= 1)
            {
                Payload Payload_Data;
                Payload_Data = _queue.Take();
                DataBuffer pkt_buf(Payload_Data.size + 4);
                uint8_t* packet = (uint8_t*)&pkt_buf[0];
                packet[0] = 0x42;
                packet[1] = Payload_Data.address;
                packet[2] = Payload_Data.size + 4;
                memcpy(&packet[3], Payload_Data.pData, Payload_Data.size);
                packet[Payload_Data.size + 4 - 1] = packet[0] + packet[1] + packet[2];
                for (uint8_t i = 0; i < Payload_Data.size; i++)
                {
                    packet[Payload_Data.size + 4 - 1] += packet[3 + i];
                }
                try
                {
                    _serial_port->Write(pkt_buf);
                }
                catch (const std::runtime_error&)
                {
                    log_e("The Write() runtime_error.");
                    continue;
                }
                catch (const NotOpen&)
                {
                    log_e("Port not open.");
                    continue;
                }
                _serial_port->DrainWriteBuffer(); // 直到写缓冲区耗尽
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        // Payload_t payload;
        // DataBuffer pkt_buf(sizeof(Packet));
        // Packet *packet = (Packet *)&pkt_buf[0];

        // Bint32_Union speed_union;
        // Bint16_Union direction_union;

        // while (_loop)
        // {
        //     if (_queue.Size() >= 1)
        //     {
        //         _can_close = false;

        //         payload = _queue.Take();

        //         // NEW send preration

        //         speed_union.Float = payload.tSpeed / 1000.0;

        //         direction_union.U16 = (uint16_t)(payload.tAngle * (-(IMG._config->send_k)) + PWM_MIDDLE);

        //         packet->head = 0x42;
        //         packet->addr = 0x01;
        //         packet->length = 10;

        //         packet->buf_speed[0] = speed_union.U8_Buff[0];
        //         packet->buf_speed[1] = speed_union.U8_Buff[1];
        //         packet->buf_speed[2] = speed_union.U8_Buff[2];
        //         packet->buf_speed[3] = speed_union.U8_Buff[3];

        //         packet->buf_direction[0] = direction_union.U8_Buff[0];
        //         packet->buf_direction[1] = direction_union.U8_Buff[1];

        //         packet->sumcheck = packet->head +
        //                            packet->addr +
        //                            packet->length;

        //         for (uint8_t i = 0; i < 4; i++)
        //         {
        //             packet->sumcheck += packet->buf_speed[i];
        //         }
        //         for (uint8_t i = 0; i < 2; i++)
        //         {
        //             packet->sumcheck += packet->buf_direction[i];
        //         }

        //         try
        //         {
        //             _serial_port->Write(pkt_buf);
        //         }
        //         catch (const std::runtime_error &)
        //         {
        //             log_e("The Write() runtime_error.");
        //             continue;
        //         }
        //         catch (const NotOpen &)
        //         {
        //             log_e("Port not open.");
        //             continue;
        //         }

        //         _serial_port->DrainWriteBuffer(); // 直到写缓冲区耗尽

        //         _can_close = true;
        //     }
        //     else
        //     {
        //         _can_close = true;
        //         std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //     }
        // }
    };

    void run_rece()
    {
#define FRAME_LENGTH_RECE 11
        uint8_t datas[FRAME_LENGTH_RECE] = { 0 };
        uint8_t counter = 0;
        uint8_t recv_data;
        uint16_t check;

        while (_loop)
        {
            // 循环阻塞接收一字节数据
            if (!recv(recv_data, 10))
            {
                datas[counter++] = recv_data;

                if (0xF2 != datas[0])
                {
                    counter = 0;
                    continue;
                }
                if (0xE2 != datas[1] && counter >= 2)
                {
                    counter = 0;
                    continue;
                }
                if ((FRAME_LENGTH_RECE - 1) < counter)
                {
                    if (0xE3 == datas[(FRAME_LENGTH_RECE - 1)])
                    {
                        check = (uint16_t)(((uint16_t)datas[FRAME_LENGTH_RECE - 3] << 8) | datas[FRAME_LENGTH_RECE - 2]);

                        if (check == CRC_CHECK(&datas[2], 6))
                        {
                            byte2int32 b2i;
                            b2i.uc[0] = (unsigned char)datas[7];
                            b2i.uc[1] = (unsigned char)datas[6];
                            b2i.uc[2] = (unsigned char)datas[5];
                            b2i.uc[3] = (unsigned char)datas[4];

                            IMG.encoder.route = (float)(b2i.i);
                            IMG.encoder.speed = (int16_t)(((int16_t)datas[2] << 8) | datas[3]) / 100.0f;
                        }
                    }
                    counter = 0;
                }
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    };

public:
    int open()
    {
        _serial_port = std::make_shared<SerialPort>();
        CHECK(_serial_port != nullptr) << "Create Failed." << LOG_END;

        try
        {
            _loop = true;
            _can_close = true;

            _serial_port->Open(_port_name);
            _serial_port->SetBaudRate(_bps);
            _serial_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);   // 8位数据位
            _serial_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
            _serial_port->SetParity(Parity::PARITY_NONE);                 // 无校验
            _serial_port->SetStopBits(StopBits::STOP_BITS_1);             // 1个停止位

            _worker_thread_send = std::thread(&SerialDriver::run_send, this);
            _worker_thread_rece = std::thread(&SerialDriver::run_rece, this);


            LOG_I << "[" << _port_name << "] Open Success." << LOG_END;
            return 0;
        }
        catch (const OpenFailed&)
        {
            LOG_E << "[" << _port_name << "] Open Failed." << LOG_END;
            return -2;
        }
        catch (const AlreadyOpen&)
        {
            LOG_E << "[" << _port_name << "] Already Open." << LOG_END;
            return -3;
        }
        catch (...)
        {
            LOG_E << "[" << _port_name << "] Exception." << LOG_END;
            return -4;
        }

        return -1;
    };

    void buzzerSound(const uint8_t sound)
    {
        transmit_packet(0x04, &sound, 1);

        // DataBuffer pkt_buf(sizeof(uint8_t) * 7);
        // uint8_t *sendBuff = (uint8_t *)&pkt_buf[0];

        // if (_loop)
        // {
        //     // uint8_t sendBuff[7];
        //     uint8_t check = 0;

        //     sendBuff[0] = 0x42;  // 帧头
        //     sendBuff[1] = 0x04;  // 地址
        //     sendBuff[2] = 5;     // 帧长
        //     sendBuff[3] = sound; // 音效

        //     for (size_t i = 0; i < 4; i++)
        //     {
        //         check += sendBuff[i];
        //     }
        //     sendBuff[4] = check;

        //     try
        //     {
        //         _serial_port->Write(pkt_buf);
        //     }
        //     catch (const std::runtime_error &)
        //     {
        //         log_e("The Write() runtime_error.");
        //     }
        //     catch (const NotOpen &)
        //     {
        //         log_e("Port not open.");
        //     }
        // }
    }

    int close()
    {
        _loop = false;
        _can_close = false;
        _worker_thread_send.join();
        _worker_thread_rece.join();
        _queue.ShutDown();

        if (_serial_port != nullptr)
        {
            _serial_port->Close();
            _serial_port = nullptr;
        }

        log_i("Stop.");
        return 0;
    };

    int sendPack(Payload_t payloadss)
    {
        // if (_queue.Size() < 5)
        // {
        //     _queue.Put(payloadss);
        // }
        uint8_t buf[6];
        Bint32_Union speed_union;
        Bint16_Union direction_union;
        speed_union.Float = payloadss.tSpeed / 1000.0;
        direction_union.U16 = (uint16_t)(payloadss.tAngle * (-(IMG._config->send_k)) + PWM_MIDDLE);
        buf[0] = speed_union.U8_Buff[0];
        buf[1] = speed_union.U8_Buff[1];
        buf[2] = speed_union.U8_Buff[2];
        buf[3] = speed_union.U8_Buff[3];
        buf[4] = direction_union.U8_Buff[0];
        buf[5] = direction_union.U8_Buff[1];
        transmit_packet(0x01, buf, 6);
        
        return 0;
    }
    void transmit_packet(uint8_t address, const uint8_t* pData, uint8_t size)
    {
        if (_queue.Size() < 5)
        {
            Payload Payload_Data;
            Payload_Data.address = address;
            memcpy(Payload_Data.pData, pData, size);
            Payload_Data.size = size;
            _queue.Put(Payload_Data);
        }
    }

private:
    int recv(unsigned char& charBuffer, size_t msTimeout = 0)
    {
        try
        {
            _serial_port->ReadByte(charBuffer, msTimeout);
        }
        catch (const ReadTimeout&)
        {
            // CHECK(false) << " The ReadByte() call has timed out." << LOG_END;
            return -2;
        }
        catch (const NotOpen&)
        {
            CHECK(false) << "port not open." << LOG_END;
        }

        return 0;
    };

private:
    uint16_t CRC_CHECK(uint8_t* Buf, uint8_t CRC_CNT)
    {
        uint16_t CRC_Temp = 0xffff;
        uint8_t i, j;

        for (i = 0; i < CRC_CNT; i++)
        {
            CRC_Temp ^= Buf[i];
            for (j = 0; j < 8; j++)
            {
                if (CRC_Temp & 0x01)
                    CRC_Temp = (CRC_Temp >> 1) ^ 0xa001;
                else
                    CRC_Temp = CRC_Temp >> 1;
            }
        }

        return (CRC_Temp);
    };
};
