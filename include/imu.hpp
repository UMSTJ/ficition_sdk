//
// Created by anaple on 24-3-22.
//


#ifndef IMU_H
#define IMU_H
#include <vector>
#include <cstdint>
#include "entity.hpp"



/**********************************************************************
函数功能：IMU数据解析发布
入口参数：无
返回  值：无
**********************************************************************/
ImuInfo ImuDataProcess(std::vector<uint8_t> ImuData);



#endif // UMS_FICTION_DRIVER_MAIN_IMU_H
