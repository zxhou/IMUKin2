
/**
 * Copyright 2018 Nanjing University of Science and Technology
 * Author: Zhixing Hou <zxhou@njust.edu.cn>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef MPU6050_H
#define MPU6050_H

#include <string>
#include <sstream>
#include <fstream>
#include <memory>
#include <mutex>


class MPU
{
public:
    bool save;
    bool running;
    std::mutex lockSave;
private:
    int fd;                            //文件描述符
    int err;                           //返回调用函数的状态
    int len;
    double T;
    std::unique_ptr<double[]> a,w,Angle;

    struct timeval tv;

    std::string baseName;
    std::string timeStamp;

    std::ofstream fs;
    std::ostringstream oss;
       

public:
    MPU(char* serialNum, std::string writePath);
    ~MPU();
    
    void start();

private:
    int UART0_Open(char* port);
    void UART0_Close();
    int UART0_Set(int speed,int flow_ctrl,int databits,int stopbits,int parity);
    int UART0_Init(int speed,int flow_ctrl,int databits,int stopbits,int parity);
    int UART0_Recv(unsigned char *rcv_buf,int data_len);
    void DecodeIMUData(unsigned char chrTemp[]);
    void MPU_Receiver();
    void saveImuData();

};

#endif