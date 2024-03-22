//
// Created by anaple on 24-3-22.
//

#ifndef UMS_SERIAL_METHODS_H
#define UMS_SERIAL_METHODS_H

#include <vector>
#include <cstdint>
#include <string>
#include <memory>
#include "serial.h"
#include "crc.hpp"

int Rfid(std::vector<uint8_t> &byteVector);

std::string magneticDataProcess(std::vector<uint8_t> NativeData);

int32_t HexArrayToInt32(uint8_t *hexArray, size_t size);

float HexArrayToFloat32(uint8_t *hexArray, size_t size);

/**********************************************************************
函数功能：消息帧内容转义
入口参数：std::vector<std::string>& byteVector
返回  值：byteVector
**********************************************************************/
std::vector<uint8_t> CompoundVector(std::vector<uint8_t> &byteVector);

/**********************************************************************
函数功能：帧内容合成发送
入口参数：uint8_t signbit 标志位        std::vector<uint8_t>& Vector 数据
返回  值：无
**********************************************************************/
std::vector<uint8_t> DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector);

void LowerParameterOperationInt(std::string readOrwrite, uint8_t address, int32_t data, std::shared_ptr<serial::Serial> Sp);

/**********************************************************************
函数功能：下位参数读写操作 FLOAT
入口参数：red_write读取或写入  address写入的地址  data写入的数据
返回  值：无
**********************************************************************/
void LowerParameterOperation(std::string red_write, uint8_t address, float data, std::shared_ptr<serial::Serial> Sp);

/**********************************************************************
函数功能：数据校验
入口参数：校验数据
返回  值：是否通过校验
**********************************************************************/
bool DataCheck(std::vector<uint8_t> &data);

/**********************************************************************
函数功能：计算并返回转换后的 double 数值
入口参数：startIndex 开始下标    count 截取的元素个数   byteData 1组8bytes数据
返回  值：double result
**********************************************************************/
double DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData);

/**********************************************************************
函数功能：将二进制数据转换为 double 类型
入口参数：std::vector<uint8_t>& byteData
返回  值：double result
**********************************************************************/
double BinaryToDouble(const std::vector<uint8_t> &byteData);

/**********************************************************************
函数功能：函数将 double 转换为 std::vector<uint8_t> 表示
入口参数：double value
返回  值：无
**********************************************************************/
std::vector<uint8_t> DoubleToBytes(double value);

void test(uint8_t a);

#endif // UMS_FICTION_DRIVER_MAIN_UMS_SERIAL_METHODS_H
