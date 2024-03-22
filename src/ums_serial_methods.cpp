#include <iomanip>
#include "ums_serial_methods.hpp"



int Rfid(std::vector<uint8_t> &byteVector)
{
    int rfid_d = -1;
    // 使用 std::stringstream 构建十六进制字符串
    std::stringstream ss;
    for (const auto &byte : byteVector)
    {
        // 将 uint8_t 格式化为十六进制并追加到 stringstream 中
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    // 从 stringstream 中获取字符串
    std::string hexString = ss.str();
    if (hexString.size() == 46)
    {

        if (hexString.substr(0, 8) == "0d3a5210")
        {
            std::string rfidHex = hexString.substr(8, 2);
            rfid_d =  std::stoi(rfidHex, 0, 16);;
        }
    }
    return rfid_d;
}

std::string magneticDataProcess(std::vector<uint8_t> NativeData)
{
    std::string MagneticSensorData;
    std::stringstream ss;
    for (uint8_t hexValue : NativeData)
    {
        ss << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(hexValue);
    }
    std::string FullHexString = ss.str();
    // 截取字符串的一部分并赋值给 MagneticSensorData
    size_t startIndex = 8; // 从第三个字符开始（索引从0开始）
    size_t length = 32;    // 截取32个字符
    MagneticSensorData = FullHexString.substr(startIndex, length);
    return MagneticSensorData;
}

int32_t HexArrayToInt32(uint8_t *hexArray, size_t size)
{
    // 检查数组大小
    if (size != sizeof(int32_t))
    {
        return 0;
    }

    // 将数组中的每个字节转换为uint32_t类型
    int32_t value = 0;
    memcpy(&value, hexArray, sizeof(int32_t));

    // 返回结果
    return value;
}

float HexArrayToFloat32(uint8_t *hexArray, size_t size)
{
    // 检查数组大小
    if (size != sizeof(float))
    {
        return 0.0;
    }
    float value;
    memcpy(&value, hexArray, sizeof(float));
    // 返回结果
    return value;
}

/**********************************************************************
函数功能：消息帧内容转义
入口参数：std::vector<std::string>& byteVector
返回  值：byteVector
**********************************************************************/
std::vector<uint8_t> CompoundVector(std::vector<uint8_t> &byteVector)
{
    std::vector<uint8_t> modifiedVector; // 用于存储修改后的值

    for (uint8_t byte : byteVector)
    {
        if (byte == 0x5C)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x00);
        }
        else if (byte == 0x3A)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x01);
        }
        else if (byte == 0x0A)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x02);
        }
        else if (byte == 0x0D)
        {
            modifiedVector.push_back(0x5C);
            modifiedVector.push_back(0x03);
        }
        else
        {
            modifiedVector.push_back(byte);
        }
    }

    return modifiedVector; // 将原始向量替换为修改后的向量
}

/**********************************************************************
函数功能：帧内容合成发送
入口参数：uint8_t signbit 标志位        std::vector<uint8_t>& Vector 数据
返回  值：无
**********************************************************************/
std::vector<uint8_t> DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector)
{
    std::vector<uint8_t> Send_data;
    std::vector<uint8_t> ResultBytes;
    // 数据长度计算
    std::size_t DataLength = Vector.size();
    // 处理数据长度
    std::vector<uint8_t> Data_length_;
    Data_length_.push_back(DataLength & 0xFF);
    if ((DataLength >> 8) != 0x00)
    {
        Data_length_.push_back((DataLength >> 8) & 0xFF);
    }
    Send_data.push_back(signbit);
    Send_data.insert(Send_data.end(), Data_length_.begin(), Data_length_.end());
    Send_data.insert(Send_data.end(), Vector.begin(), Vector.end());

    uint16_t result = CalCRC16(Send_data);
    ResultBytes.push_back(result >> 8);
    ResultBytes.push_back(result & 0xFF);

    Send_data.insert(Send_data.end(), ResultBytes.begin(), ResultBytes.end());
    Send_data = CompoundVector(Send_data);

    Send_data.push_back(0x0A);
    Send_data.push_back(0x0D);
    Send_data.insert(Send_data.begin(), 0x3A);

    ResultBytes.clear();
    Data_length_.clear();

    return Send_data;
}

void LowerParameterOperationInt(std::string readOrwrite, uint8_t address, int32_t data, std::shared_ptr<serial::Serial> Sp)
{
    std::vector<uint8_t> read;
    std::vector<uint8_t> write_in;
    std::vector<uint8_t> datTransformed_date;

    // 获取高字节
    uint8_t highByte = (address >> 8) & 0xFF;
    // 获取低字节
    uint8_t lowByte = address & 0xFF;

    if (readOrwrite == "write" && Sp != nullptr)
    {
        write_in.push_back(highByte);
        write_in.push_back(lowByte);
        // 将浮点数的字节表示转换为 std::vector<uint8_t>
        std::vector<uint8_t> byteVector(sizeof(int32_t));
        std::memcpy(byteVector.data(), &data, sizeof(int32_t));
        datTransformed_date = byteVector;
        write_in.insert(write_in.end(), datTransformed_date.begin(), datTransformed_date.end());
        write_in = DataDelivery(0x57, write_in);
        byteVector.clear();
        Sp->write(write_in);
    }
}
/**********************************************************************
函数功能：下位参数读写操作 FLOAT
入口参数：red_write读取或写入  address写入的地址  data写入的数据
返回  值：无
**********************************************************************/
void LowerParameterOperation(std::string red_write, uint8_t address, float data, std::shared_ptr<serial::Serial> Sp)
{
    std::vector<uint8_t> read;
    std::vector<uint8_t> write_in;
    std::vector<uint8_t> datTransformed_date;

    // 获取高字节
    uint8_t highByte = (address >> 8) & 0xFF;
    // 获取低字节
    uint8_t lowByte = address & 0xFF;

    if (red_write == "read" && Sp != nullptr)
    {
        read.push_back(0x00);
        read.push_back(0x00);
        read.push_back(0x00);
        read.push_back(0x2C);

        read = DataDelivery(0x52, read);
        Sp->write(read);
    }
    else if (red_write == "write" && Sp != nullptr)
    {
        write_in.push_back(highByte);
        write_in.push_back(lowByte);
        // 将浮点数的字节表示转换为 std::vector<uint8_t>
        std::vector<uint8_t> byteVector(sizeof(float));
        std::memcpy(byteVector.data(), &data, sizeof(float));
        datTransformed_date = byteVector;
        write_in.insert(write_in.end(), datTransformed_date.begin(), datTransformed_date.end());
        write_in = DataDelivery(0x57, write_in);
        byteVector.clear();
        Sp->write(write_in);
        // for (const auto &element : write_in)
        // {
        //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(element) << " ";
        // }
        // std::cout << std::dec << std::endl;
    }
}
/**********************************************************************
函数功能：数据校验
入口参数：校验数据
返回  值：是否通过校验
**********************************************************************/
bool DataCheck(std::vector<uint8_t> &data)
{
    std::vector<uint8_t> extractedData;

    if (data.size() >= 7)
    {
        //        printf("1 if");
        extractedData.assign(data.begin() + 2, data.end() - 3);
    }
    else
    {
        // std::cerr << "Data doesn't have enough elements." << std::endl;
        return false; // 返回 false 表示数据不符合要求
    }

    uint16_t result = CalCRC16(extractedData);
    uint16_t result_l = (result & 0xFF);
    uint16_t result_h = (result >> 8);

    if (result_h == data[data.size() - 3] && result_l == data[data.size() - 2])
    {
        //    {   printf("2 if");

        return 1;
    }
    else
    {

        return 0;
    }
}
/**********************************************************************
函数功能：计算并返回转换后的 double 数值
入口参数：startIndex 开始下标    count 截取的元素个数   byteData 1组8bytes数据
返回  值：double result
**********************************************************************/
double DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData)
{
    size_t startIndexs = startIndex;
    size_t counts = count;
    std::vector<uint8_t> subVector(byteData.begin() + startIndexs, byteData.begin() + startIndexs + counts);
    double result = BinaryToDouble(subVector);
    return result;
}

/**********************************************************************
函数功能：将二进制数据转换为 double 类型
入口参数：std::vector<uint8_t>& byteData
返回  值：double result
**********************************************************************/
double BinaryToDouble(const std::vector<uint8_t> &byteData)
{
    if (byteData.size() != sizeof(double))
    {

        return 0.0;
    }
    double result;
    std::memcpy(&result, byteData.data(), sizeof(double));

    return result;
}

/**********************************************************************
函数功能：函数将 double 转换为 std::vector<uint8_t> 表示
入口参数：double value
返回  值：无
**********************************************************************/
std::vector<uint8_t> DoubleToBytes(double value)
{
    std::vector<uint8_t> bytes;
    uint8_t *ptr = reinterpret_cast<uint8_t *>(&value);

    for (size_t i = 0; i < sizeof(double); i++)
    {
        bytes.push_back(*ptr);
        ptr++;
    }

    return bytes;
}



void test(uint8_t a)
{

    printf("hello %d\r\n", a);
}