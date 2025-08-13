#include "ums_serial_methods.hpp"
#include <iostream>
#include <vector>
#include <unordered_map>

using namespace std;
using namespace serial;

class CustomLayout : public log4cpp::Layout
{
public:
    virtual ~CustomLayout() {}

    virtual std::string format(const log4cpp::LoggingEvent &event)
    {
        std::ostringstream stream;

        // 获取当前时间
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];

        time(&rawtime);
        timeinfo = localtime(&rawtime);

        strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
        std::string strTime(buffer);

        // 设定自定义格式
        stream << "[" << event.threadName << event.categoryName << event.ndc << event.priority << "] " << strTime << " [UMS_FICTION_SDK] : " << event.message << "\n";
        return stream.str();
    }
};

bool UmsSerialMethods::checkSignValue(uint8_t sign)
{
    for (uint8_t regSignValue : signedValue)
    {

        if (regSignValue == sign)
            return true;
    }
    return false;
}
bool UmsSerialMethods::checkDataLength(uint8_t signLength, size_t size)
{

    if ((size - signLength) - 7 == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void UmsSerialMethods::sendEditParamsData()
{
    try
    {
        while (!ParamDataWrite())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待写入
            /* code */
        }
    }
    catch (const std::exception &e)
    {
        root.error("UmsSerialMethods::sendEditParamsData() error: " + std::string(e.what()));
    }
}

UmsSerialMethods::UmsSerialMethods()
{
    circularQueue = std::make_shared<CircularQueue>(85);
    // 创建一个输出到标准输出的Appender
    log4cpp::OstreamAppender *osAppender = new log4cpp::OstreamAppender("osAppender", &std::cout);

    // 创建布局并设置模式
    osAppender->setLayout(new CustomLayout());

    root.setPriority(log4cpp::Priority::DEBUG);
    root.addAppender(osAppender);
    root.info("UmsSerialSDK Init");
}

UmsSerialMethods::UmsSerialMethods(const std::string &portName, int baudRate, bool isDebug, int queueSize, AgreementVersion agreementVersion)
{

    circularQueue = std::make_shared<CircularQueue>(queueSize);
    // 创建一个输出到标准输出的Appender
    log4cpp::OstreamAppender *osAppender;
    osAppender = new log4cpp::OstreamAppender("osAppender", &std::cout);

    // 将布局设置给Appender
    osAppender->setLayout(new CustomLayout());
    if (isDebug)
    {
        root.setPriority(log4cpp::Priority::DEBUG);
    }
    if (!isDebug)
    {
        root.setPriority(log4cpp::Priority::INFO);
    }
    // 设置IMU协议版本
    runtimeVersion = agreementVersion;

    root.addAppender(osAppender);
    root.info("UmsSerialSDK Init start serial");
    startSerial(portName, baudRate);
}
size_t UmsSerialMethods::findElement(const vector<uint8_t> &buffer, uint8_t element, size_t startPos = 0)
{
    for (size_t i = startPos; i < buffer.size(); ++i)
    {
        if (buffer[i] == element)
        {
            return i;
        }
    }
    return string::npos; // 没有找到时返回npos
}

std::vector<uint8_t> UmsSerialMethods::comFrameReduction(std::vector<uint8_t> &arr)
{
    std::unordered_map<uint8_t, uint8_t> fix{
        {0x00, 0x5C}, {0x01, 0x3A}, {0x02, 0x0A}, {0x03, 0x0D}};

    for (size_t index = 0; index < arr.size(); ++index)
    {
        if (arr[index] == 0x5C)
        {
            // 检查是否有足够的空间来查找和替换
            if (index + 1 < arr.size() && fix.find(arr[index + 1]) != fix.end())
            {
                arr[index] = fix[arr[index + 1]];
                arr.erase(arr.begin() + index + 1); // 删除下一个元素
            }
        }
    }
    return arr;
}

bool UmsSerialMethods::extractPacket(vector<uint8_t> &buffer, vector<uint8_t> &packet)
{
    size_t start, end;
    start = findElement(buffer, 0x3A);
    if (start == string::npos)
    {
        buffer.clear();
        root.warn("未找到包起始头0x3A");
        return false;
    }

    // 从包起始头后开始寻找0x0A
    end = findElement(buffer, 0x0A, start);
    if (end == string::npos)
    {
        root.warn("未找到0x0A");
        buffer.clear();
        return false;
    }
    if (buffer[end + 1] != 0x0D)
    {
        root.warn("未找到0x0D");
        buffer.erase(buffer.begin() + start, buffer.begin() + end);
        return false;
    }

    // 提取从0x3A到0x0A之间的数据
    //    packet.clear();
    buffer.erase(buffer.begin() + start, buffer.begin() + end + 1);
    packet.insert(packet.end(), buffer.begin() + start, buffer.begin() + end + 1);

    return true;
}

int UmsSerialMethods::Rfid(std::vector<uint8_t> &byteVector)
{
    int rfid_d = -1;
    // 使用 std::string stream 构建十六进制字符串
    std::stringstream ss;
    for (const auto &byte : byteVector)
    {
        // 将 uint8_t 格式化为十六进制并追加到 stringstream 中
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
    }
    // 从 string stream 中获取字符串
    std::string hexString = ss.str();
    if (hexString.size() == 46)
    {

        if (hexString.substr(0, 8) == "0d3a5210")
        {
            std::string rfidHex = hexString.substr(8, 2);
            rfid_d = std::stoi(rfidHex, 0, 16);
        }
    }
    return rfid_d;
}

std::string UmsSerialMethods::magneticDataProcess(const std::vector<uint8_t> &NativeData)
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

int32_t UmsSerialMethods::HexArrayToInt32(uint8_t *hexArray, size_t size)
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

float UmsSerialMethods::HexArrayToFloat32(uint8_t *hexArray, size_t size)
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
std::vector<uint8_t> UmsSerialMethods::CompoundVector(std::vector<uint8_t> &byteVector)
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
std::vector<uint8_t> UmsSerialMethods::DataDelivery(uint8_t signbit, const std::vector<uint8_t> &Vector)
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

void UmsSerialMethods::LowerParameterOperationInt(const std::string &basicString, uint8_t address, int32_t data, const std::shared_ptr<serial::Serial> &Sp)
{
    std::vector<uint8_t> read;
    std::vector<uint8_t> write_in;
    std::vector<uint8_t> datTransformed_date;

    // 获取高字节
    uint8_t highByte = (address >> 8) & 0xFF;
    // 获取低字节
    uint8_t lowByte = address & 0xFF;

    if (basicString == "write" && Sp != nullptr)
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
void UmsSerialMethods::LowerParameterOperation(const std::string &red_write, uint8_t address, float data, const std::shared_ptr<serial::Serial> &Sp)
{
    std::vector<uint8_t> read;
    std::vector<uint8_t> write_in;
    std::vector<uint8_t> datTransformed_date;

    // 获取高字节
    uint8_t highByte = (address >> 8) & 0xFF;
    // 获取低字节
    uint8_t lowByte = address & 0xFF;

    if (red_write == "read" && sp != nullptr)
    {
        read.push_back(0x00);
        read.push_back(0x00);
        read.push_back(0x00);
        read.push_back(0x2C);

        read = DataDelivery(0x52, read);
        Sp->write(read);
    }
    else if (red_write == "sys" && Sp != nullptr)
    {
        read.push_back(0x00);
        read.push_back(0x04);
        read.push_back(0x00);
        read.push_back(0x04);
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
    }
}
/**********************************************************************
函数功能：数据校验
入口参数：校验数据
返回  值：是否通过校验
**********************************************************************/
bool UmsSerialMethods::DataCheck(std::vector<uint8_t> &data)
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
        return true;
    }
    else if (data[2] == 0x51 && data[4] == 0x04)
    {
        return true;
    }
    else
    {
        root.debug("CRC16校验失败");
        root.debug("chigh : %02x clow: %02x", result_h, result_l);
        root.debug("high: %02x low: %02x", data[data.size() - 3], data[data.size() - 2]);
        //  for (int i = 0; i < data.size(); i++)
        //  {
        //      printf("%02x", data[i]);
        //  }
        //  printf("\n");

        return false;
    }
}
/**********************************************************************
函数功能：计算并返回转换后的 double 数值
入口参数：startIndex 开始下标    count 截取的元素个数   byteData 1组8bytes数据
返回  值：double result
**********************************************************************/
double UmsSerialMethods::DirectionalInterception(int startIndex, int count, const std::vector<uint8_t> &byteData)
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
double UmsSerialMethods::BinaryToDouble(const std::vector<uint8_t> &byteData)
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
std::vector<uint8_t> UmsSerialMethods::DoubleToBytes(double value)
{
    std::vector<uint8_t> bytes;
    auto *ptr = reinterpret_cast<uint8_t *>(&value);

    for (size_t i = 0; i < sizeof(double); i++)
    {
        bytes.push_back(*ptr);
        ptr++;
    }

    return bytes;
}
void UmsSerialMethods::EscapeVector(std::vector<uint8_t> &byteVector)
{
    int index5c = findElement(byteVector, 0x5C);
    if (index5c != -1)
    {
        int indexSign = index5c + 1;
        if (byteVector[index5c] == 0x00)
        {
            byteVector.erase(byteVector.begin() + indexSign - 1);
            byteVector[index5c - 1] = 0x5C;
        }
        else if (byteVector[index5c] == 0x01)
        {
            byteVector.erase(byteVector.begin() + indexSign - 1);
            byteVector[index5c - 1] = 0x3A;
        }
        else if (byteVector[index5c] == 0x02)
        {
            byteVector.erase(byteVector.begin() + indexSign - 1);
            byteVector[index5c - 1] = 0x0A;
        }
        else if (byteVector[index5c] == 0x03)
        {
            byteVector.erase(byteVector.begin() + indexSign - 1);
            byteVector[index5c - 1] = 0x0D;
        }
    }
}
PowerInfo UmsSerialMethods::PowerDataProcess(const std::vector<uint8_t> &NativeData)
{
    // 总线电流
    double bus = DirectionalInterception(4, 8, NativeData);
    // 5v输出
    double output = DirectionalInterception(12, 8, NativeData);
    // 输入电压
    double input = DirectionalInterception(20, 8, NativeData);
    // 19v输出
    double output19 = DirectionalInterception(28, 8, NativeData);

    PowerInfo result{};
    result.bus = bus;
    result.output5v = output;
    result.input = input;
    result.output19v = output19;
    return result;
}
ImuInfo UmsSerialMethods::ImuDataProcess(std::vector<uint8_t> ImuData)
{

    try
    {
        //        for(int i = 0; i < ImuData.size(); i++){
        //            printf("%02x", ImuData[i]);
        //        }
        //        printf("\n");
        if (ImuData[4] == 0x04 && runtimeVersion == AgreementVersion::V2)
        {
            for (int i = 0; i < 13; i = i + 1)
            {
                std::vector<uint8_t> subVector;
                subVector.insert(subVector.begin(), ImuData.begin() + 5 + (i * 8), ImuData.begin() + 13 + (i * 8));
                float value = BinaryToDouble(subVector);
                //                printf("%f", value);
                switch (i)
                {
                case 0:
                {
                    ImuStructural.axaxis = value;
                    break;
                }
                case 1:
                {
                    ImuStructural.ayaxis = value;
                    break;
                }
                case 2:
                {
                    ImuStructural.azaxis = value;
                    break;
                }
                case 3:
                {
                    ImuStructural.gxaxis = value;
                    break;
                }
                case 4:
                {
                    ImuStructural.gyaxis = value;
                    break;
                }
                case 5:
                {
                    ImuStructural.gzaxis = value;
                    break;
                }
                case 6:
                {
                    ImuStructural.q0 = value;
                    break;
                }
                case 7:
                {
                    ImuStructural.q1 = value;
                    break;
                }
                case 8:
                {
                    ImuStructural.q2 = value;
                    break;
                }
                case 9:
                {
                    ImuStructural.q3 = value;
                    break;
                }
                case 10:
                {
                    ImuStructural.pitch = value;
                    break;
                }
                case 11:
                {
                    ImuStructural.roll = value;
                    break;
                }
                case 12:
                {
                    ImuStructural.yaw = value;
                    break;
                }
                }
            }
            //            printf("\n");
            //            printf("ImuDataProcess: %f, %f, %f, %f, %f, %f, %f, %f, \n", ImuStructural.axaxis, ImuStructural.ayaxis, ImuStructural.azaxis, ImuStructural.gxaxis, ImuStructural.gyaxis, ImuStructural.gzaxis, ImuStructural.pitch, ImuStructural.roll);
        }
        else if (runtimeVersion == AgreementVersion::V1)
        {
            std::vector<uint8_t> subVector;
            if (ImuData[4] == 0x00)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.axaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.ayaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.azaxis = BinaryToDouble(subVector);
                subVector.clear();
            }
            else if (ImuData[4] == 0x01)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.gxaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.gyaxis = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.gzaxis = BinaryToDouble(subVector);
                subVector.clear();
            }
            else if (ImuData[4] == 0x02)
            {
                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.q0 = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.q1 = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.q2 = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 29, ImuData.begin() + 37);
                ImuStructural.q3 = BinaryToDouble(subVector);
                subVector.clear();
            }
            else if (ImuData[4] == 0x03)
            {

                subVector.insert(subVector.begin(), ImuData.begin() + 5, ImuData.begin() + 13);
                ImuStructural.pitch = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 13, ImuData.begin() + 21);
                ImuStructural.roll = BinaryToDouble(subVector);
                subVector.clear();

                subVector.insert(subVector.begin(), ImuData.begin() + 21, ImuData.begin() + 29);
                ImuStructural.yaw = BinaryToDouble(subVector);
                subVector.clear();
            }
        }
    }
    catch (const std::exception &e)
    {
        // std::cerr << e.what() << '\n';
    }
    return ImuStructural;
}
std::shared_ptr<serial::Serial> UmsSerialMethods::getSerial()
{

    if (spThread.joinable())
    {
        spThread.join(); // 确保之前的线程已经完成
    }
    return sp;
}
void UmsSerialMethods::createSerial(const std::string &portName, int baudRate)
{
    while (!stopFlag)
    {
        try
        {
            sp = std::make_shared<serial::Serial>(portName, baudRate, serial::Timeout::simpleTimeout(1000));
            if (sp->isOpen())
            {
                lastReceiveTime = std::chrono::steady_clock::now();
                timeoutOccurred = false;
                bool checkFin = false;
                std::thread timeoutThread(&UmsSerialMethods::monitorTimeout, this);
                root.info("start1");
                root.info(std::to_string(sp->available()));

                if(sp->available() == 0 ){
                    auto uplhz = generateUplhzPacket(80,true,true,true);
                    
                    LowerParameterOperationInt("write", 64, uplhz, sp);
                    sleep(5);
                }
                while (sp->available() != 0 && !stopFlag && !timeoutOccurred)
                {
                    root.info("in while start");
                    stopFlag = false;
                    std::string data;

                    data = sp->readline();
                    std::vector<uint8_t> package(data.begin(), data.end());
                    if (DataCheck(package))
                    {
                        lastReceiveTime = std::chrono::steady_clock::now();
                        root.info("serial port " + portName + " checked successfully");
                        checkFin = true;
                        break;
                    }
                }
                if (timeoutOccurred)
                {
                    root.warn("serial port " + portName + " timeout");
                    if (timeoutThread.joinable())
                        timeoutThread.join();
                    sp = nullptr;
                }
                if (checkFin)
                {
                    root.info("serial port " + portName + " opened successfully");
                    if (timeoutThread.joinable())
                        timeoutThread.join();
                    break;
                }
                if (!checkFin)
                {
                    root.warn("serial port " + portName + " open failed");
                    sp = nullptr;
                }
            }
            else
            {
                root.warn("serial port " + portName + " open failed");
                sp = nullptr;
            }
        }
        catch (const std::exception &e)
        {
            sp = nullptr;
            root.error("Exception thrown: " + std::string(e.what()) + "  port:" + portName);
            sleep(3);
        }
    }
    if (stopFlag)
    {
        root.warn("serial thread shotdown  port: %s", portName.c_str());
    }
}
OdomInfo UmsSerialMethods::OdomDataProcess(const std::vector<uint8_t> &ImuData)
{
    // x方向速度 vx ，y 方向速度 vy ，角速度 ωz
    double vx_chassis;
    double vy_chassis;
    double wz_chassis;
    OdomInfo result{};

    vx_chassis = DirectionalInterception(4, 8, ImuData);
    vy_chassis = DirectionalInterception(12, 8, ImuData);
    wz_chassis = DirectionalInterception(20, 8, ImuData);
    result.vx = vx_chassis;
    result.vy = vy_chassis;
    result.vth = wz_chassis;

    return result;
}
bool UmsSerialMethods::ParamDataWrite()
{
    try
    {
        if (sp->isOpen())
        {
            std::string LC_read_write = "write";
            if (inputParam.KP != 0)
                LowerParameterOperation(LC_read_write, 8, inputParam.KP, sp);
            if (inputParam.KI != 0)
                LowerParameterOperation(LC_read_write, 12, inputParam.KI, sp);
            if (inputParam.KD != 0)
                LowerParameterOperation(LC_read_write, 16, inputParam.KD, sp);
            if (inputParam.MPE != 0)
                LowerParameterOperation(LC_read_write, 20, inputParam.MPE, sp);
            if (inputParam.MPC != 0)
                LowerParameterOperation(LC_read_write, 24, inputParam.MPC, sp);
            if (inputParam.LA != 0)
                LowerParameterOperation(LC_read_write, 28, inputParam.LA, sp);
            if (inputParam.LB != 0)
                LowerParameterOperation(LC_read_write, 32, inputParam.LB,
                                        sp);
            if (inputParam.KMTT != 0)
                LowerParameterOperationInt(LC_read_write, 36, inputParam.KMTT, sp);
            if (inputParam.IMU_ZOFS != 0)
                LowerParameterOperation(LC_read_write, 40, inputParam.IMU_ZOFS, sp);
            if (inputParam.IMU_YOFS != 0)
                LowerParameterOperation(LC_read_write, 44, inputParam.IMU_YOFS, sp);
            if (inputParam.IMU_XOFS != 0)
                LowerParameterOperation(LC_read_write, 48, inputParam.IMU_XOFS, sp);
            if (inputParam.MTDIR != 0)
                LowerParameterOperation(LC_read_write, 52, inputParam.MTDIR, sp);
            if (inputParam.MSNUM != 0)
                LowerParameterOperation(LC_read_write, 56, inputParam.IMU_YOFS, sp);
            if (inputParam.ROSID != 0)
                LowerParameterOperation(LC_read_write, 60, inputParam.ROSID, sp);
            if (inputParam.UPLHZ != 0)
                LowerParameterOperationInt(LC_read_write, 64, inputParam.UPLHZ, sp);
            if (inputParam.FUNC1 != 0)
                LowerParameterOperation(LC_read_write, 68, inputParam.FUNC1, sp);
            if (inputParam.FUNC2 != 0)
                LowerParameterOperation(LC_read_write, 72, inputParam.FUNC1, sp);
            if (inputParam.FUNC3 != 0)
                LowerParameterOperation(LC_read_write, 76, inputParam.FUNC1, sp);
            return true;
        }
        else
        {
            return false;
        }
    }
    catch (const std::exception &e)
    {
        root.error("Exception ParamWrite thrown: " + std::string(e.what()));
        return false;
    }
}
ParamsData UmsSerialMethods::ParamDataRead(uint8_t *data)
{
    uint8_t msg_len = data[3];
    ParamsData result{};
    try
    {
        uint8_t *p = data + 4;
        if (msg_len == 4)
        {
            int32_t intValue = HexArrayToInt32(p, 4);
            result.sysStatusData = static_cast<SysStatus>(intValue);
            result.sysStatusFrame = true;
            return result;
        }
        if (msg_len == 44)
        {
            for (int index = 0; index <= msg_len; index = index + 4)
            {
                int32_t intValue = HexArrayToInt32(p + index, 4);
                float floatValue = HexArrayToFloat32(p + index, 4);

                switch (index)
                {
                case 8:
                {
                    result.KP = floatValue;
                    break;
                }
                case 12:
                {
                    result.KI = floatValue;
                    break;
                }
                case 16:
                {
                    result.KD = floatValue;
                    break;
                }
                case 20:
                {
                    result.MPE = floatValue;
                    break;
                }
                case 24:
                {
                    result.MPC = floatValue;
                    break;
                }
                case 28:
                {
                    result.LA = floatValue;
                    break;
                }
                case 32:
                {
                    result.LB = floatValue;
                    break;
                }
                case 36:
                {
                    result.KMTT = intValue;

                    break;
                }
                case 40:
                {
                    result.IMU_ZOFS = floatValue;

                    break;
                }
                case 44:
                {
                    result.IMU_YOFS = floatValue;
                }
                case 48:
                {
                    result.IMU_XOFS = floatValue;

                }
                case 52:
                {
                    result.MTDIR = intValue;
                }
                case 56:
                {
                    result.MSNUM = intValue;
                }
                case 60:
                {
                    result.ROSID = intValue;
                }
                case 64:
                {
                    result.UPLHZ = intValue;
                }
                case 68:
                {
                    result.FUNC1 = intValue;
                }
                case 72:
                {
                    result.FUNC2 = intValue;
                }
                case 76:
                {
                    result.FUNC3 = intValue;
                }
                default:
                    //
                    break;
                }
            }
            result.sysStatusFrame = false;
        }
    }
    catch (const std::exception &e)
    {
        root.error("Exception ParamRead thrown: " + std::string(e.what()));
    }
    return result;
}
ICDRemote UmsSerialMethods::convertBackDataToControl(int channel1Value, int channel2Value, int channel3Value)
{
    float maxOutputValue = 50.0; // 最大输出值

    ICDRemote icdRemoteData{};

    // 自转角速度转换
    float yawRate = ((float)channel1Value / maxOutputValue) * 180; // 假设最大自转角速度为±180度/秒

    // Vx线速度转换
    float vxSpeed = ((float)channel2Value / maxOutputValue) * 1; // 假设Vx的最大速度为±1m/s

    // Vy线速度转换
    float vySpeed = ((float)channel3Value / maxOutputValue) * 1; // 假设Vy的最大速度为±1m/s
    icdRemoteData.az = yawRate;
    icdRemoteData.vx = vxSpeed;
    icdRemoteData.vy = vySpeed;
    return icdRemoteData;
}

RCSBUSRemote UmsSerialMethods::convertRCBusRemote(std::vector<uint8_t> &byteVector)
{
    RCSBUSRemote rcsbusRemote{};
    rcsbusRemote.len = byteVector[3];
    for (int index = 0; index < 8; index = index + 2)
    {
        uint16_t result = (static_cast<uint16_t>(byteVector[index + 5]) << 8) | byteVector[index + 4];
        rcsbusRemote.axes[index / 2] = result;
    }
    return rcsbusRemote;
}
void UmsSerialMethods::loopUmsFictionData(const std::shared_ptr<FictionData> &FictionData)
{
    rdThread = std::thread(&UmsSerialMethods::tdLoopUmsFictionData, this, sp, FictionData);
    fictionData = FictionData;
}

void UmsSerialMethods::monitorTimeout()
{
    while (!stopFlag)
    {
        if (std::chrono::steady_clock::now() - lastReceiveTime > std::chrono::seconds(10))
        {
            timeoutOccurred = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void UmsSerialMethods::readSerialData()
{
    try
    {
        if (sp != nullptr)
        {
            while (!stopFlag)
            {
                if (stopFlag) break;
                std::string data;
                try
                {
                    data = sp->readline();
                }
                catch (const serial::SerialException &e)
                {
                    if (stopFlag)
                        break; // 如果是因为停止而抛出异常，直接退出
                    throw;
                }

                // 添加超时控制的入队
                auto start = std::chrono::steady_clock::now();
                while (!circularQueue->enqueue(data))
                {
                    if (stopFlag)
                        return;

                    // 避免死循环，添加超时检查
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now - start).count() > 5)
                    {
                        root.warn("Queue enqueue timeout");
                        break;
                    }

                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        if (!stopFlag)
        {
            root.error("readSerialData exception: " + std::string(e.what()));
            // 可能需要重启串口或其他错误处理
        }
    }
}
void UmsSerialMethods::tdLoopUmsFictionData(const std::shared_ptr<serial::Serial> &Sp, const std::shared_ptr<FictionData> &FictionData)
{
    lastReceiveTime = std::chrono::steady_clock::now();
    timeoutOccurred = false;
    std::thread readThread(&UmsSerialMethods::readSerialData, this);
    try
    {
        if (FictionData != nullptr)
        {
            //            root.debug("Serial port is open");
            //            root.debug("Params sending");
            getSysStatus();
            ParamDataWrite();
            while (!stopFlag)
            {
                std::string data;
                if (circularQueue->dequeue(data))
                {
                    std::vector<uint8_t> packet(data.begin(), data.end());

                    packet = comFrameReduction(packet);
                    if (!packet.empty())
                    {
                        if (packet[1] == 0x3a && packet[packet.size() - 1] == 0x0a && packet[0] == 0x0d && checkSignValue(packet[2]) && checkDataLength(packet[3], packet.size()))
                        {
                            if (DataCheck(packet))
                            {
                                switch (packet[2])
                                {
                                case 0x41: // 电源树 总线电流、5V 输出、输入电压、19V 输出。
                                    FictionData->powerData = PowerDataProcess(packet);
                                    break;
                                case 0x42: // 参数读取操作返回内容
                                {
                                    FictionData->paramsData = ParamDataRead(&packet[0]);
                                    break;
                                }
                                case 0x45: // 磁条数据
                                    FictionData->magneticData = magneticDataProcess(packet);
                                    break;
                                case 0x51: // imu数据
                                    FictionData->imuStructural = ImuDataProcess(packet);
                                    break;
                                case 0x52: // rfid数据
                                    FictionData->rfidData = std::to_string(Rfid(packet));
                                    break;
                                case 0x46: // 遥控数据RC SBUS
                                {
                                    FictionData->rcsBusData = convertRCBusRemote(packet);
                                }
                                case 0x49: // ICD遥控
                                {
                                    FictionData->icdData = convertBackDataToControl(packet[4], packet[5],
                                                                                    packet[6]);
                                    break;
                                }
                                case 0x4b: // 里程计
                                {
                                    FictionData->odomData = OdomDataProcess(packet);
                                    break;
                                }
                                case 0x53: // 四个电机速度数据
                                    break;
                                case 0x54: // 数字式温度传感器
                                    FictionData->temperature = static_cast<float>((packet[5] << 8) | packet[4]);
                                    break;
                                case 0x55: // 超声数据
                                    FictionData->ultrasonic = static_cast<float>((packet[5] << 8) | packet[4]);
                                    break;
                                default:
                                    // 处理默认情况
                                    break;
                                }
                                packet.clear();
                            }
                        }
                    }
                }
                else
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 等待队列有数据
                }
            }

            //            stopFlag = true;
            //            if (timeoutThread.joinable()) timeoutThread.join();
            //            if (reThread.joinable()){
            //                root.warn("reThread running");
            //                reThread.detach();
            //            }
            //            reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
            //            if(timeoutOccurred){
            //                root.warn("serial port data:  %ld , timeout", sp->available());
            //            }
            //            root.warn("read serial end \n");
        }
        else
        {
            root.warn("read serial fail");
        }
    }

    catch (const std::exception &e)
    {
        root.error("read serial base exception " + std::string(e.what()));
        stopFlag = true;
        if (readThread.joinable())
            readThread.join();
        if (reThread.joinable())
            reThread.detach();
        reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
    }
}

std::vector<uint8_t> UmsSerialMethods::generatePwmPacket(uint8_t channel, uint16_t pulseWidth) {
        // --- 1. 输入参数校验 ---
        if (channel > 7) {
            std::cerr << "Error: Channel number " << static_cast<int>(channel) << " is out of the valid range [0, 7]." << std::endl;
            return {}; // 返回空vector表示失败
        }
        if (pulseWidth > 50000) {
            std::cerr << "Error: Pulse width " << pulseWidth << " is out of the valid range [0, 50000]." << std::endl;
            return {}; // 返回空vector表示失败
        }
        // --- 2. 创建协议数据包 ---
        std::vector<uint8_t> packet;
        packet.reserve(3); // 预分配5字节空间以提高效率
        // packet.push_back(0x53); // 'S'
        // packet.push_back(0x03);
        packet.push_back(channel);
        packet.push_back(static_cast<uint8_t>(pulseWidth & 0xFF));
        packet.push_back(static_cast<uint8_t>((pulseWidth >> 8)));
        return packet;
}

int32_t  UmsSerialMethods::generateUplhzPacket(uint16_t frequency, bool enableUsbCom, bool enableCom1, bool enableCom2) {
        if (frequency > 1000) {
            std::cerr << "Error: Frequency " << frequency << " is out of the valid range [0, 1000]." << std::endl;
            return -1; // 返回-1表示错误
        }

        // --- 2. 构建32位的配置值 ---
        // 使用 uint32_t 进行位运算是最清晰和安全的方式
        uint32_t configValue = frequency; // 低16位为频率

        // 使用位掩码设置端口
        if (enableUsbCom) {
            configValue |= (1U << 19); // 置第16位为1
        }
        if (enableCom1) {
            configValue |= (1U << 17); // 置第17位为1
        }
        if (enableCom2) {
            configValue |= (1U << 16); // 置第18位为1
        }

        // --- 3. 将结果转换为 int32_t 并返回 ---
        return static_cast<int32_t>(configValue);
    }

void UmsSerialMethods::sendPwmPacket(const std::vector<uint8_t> &pwmData){
    auto write_in = DataDelivery(0x53, pwmData);
    sp->write(write_in);
    
    // if (pwmData.empty()) {
    //     std::cout << "Packet is empty (generation failed)." << std::endl;
    //     return;
    // }
    // if (sp != nullptr && !stopFlag){
    //     sp->write(pwmData);
    // }
}
void UmsSerialMethods::sendTwistData(const std::shared_ptr<TwistCustom> &twistData)
{

    if (sp != nullptr && twistData != nullptr && !stopFlag)
    {
        std::vector<uint8_t> SendHexData;
        std::vector<uint8_t> LeftWheelSpeed = DoubleToBytes(twistData->linear_x);
        std::vector<uint8_t> RightWheelSpeed = DoubleToBytes(twistData->linear_y);
        std::vector<uint8_t> AngularVelocity = DoubleToBytes(twistData->angular_z);
        SendHexData.insert(SendHexData.end(), LeftWheelSpeed.begin(), LeftWheelSpeed.end());
        SendHexData.insert(SendHexData.end(), RightWheelSpeed.begin(), RightWheelSpeed.end());
        SendHexData.insert(SendHexData.end(), AngularVelocity.begin(), AngularVelocity.end());
        SendHexData = DataDelivery(0x4b, SendHexData);
        try
        {

            sp->write(SendHexData);
        }
        catch (const std::exception &e)
        {
            root.error("sendTwistData exception %s", e.what());
            stopFlag = true;
            if (reThread.joinable())
                reThread.detach();
            reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
        }
    }
}
void UmsSerialMethods::sendMessageToGetParamData()
{
    try
    {

        if (sp != nullptr)
        {
            LowerParameterOperation("read", 0, 0, sp);
        }
    }
    catch (const std::exception &e)
    {

        root.error("sendMessageToGetParamData exception %s", e.what());
    }
}
void UmsSerialMethods::reStartSerial(const std::string &portName, int baudRate)
{
    stopFlag = true;
    if (spThread.joinable())
    {
        spThread.join(); // 确保之前的线程已经完成
    }
    if (sp != nullptr)
    {
        sp->close();
    }
    stopFlag = false;
    spThread = std::thread(&UmsSerialMethods::createSerial, this, portName, baudRate);
    if (spThread.joinable())
        spThread.join();
    if (rdThread.joinable())
        rdThread.join();
    loopUmsFictionData(fictionData);
}
void UmsSerialMethods::startSerial(const std::string &portName, int baudRate)
{
    stopFlag = false;
    try
    {
        root.info("wait %s,%d", portName.c_str(), baudRate);
        spThread = std::thread(&UmsSerialMethods::createSerial, this, portName, baudRate);
    }
    catch (const std::exception &e)
    {
        root.error("Exception thrown: %s", e.what());
    }
}

void UmsSerialMethods::setParamsData(ParamsData paramsData)
{
    inputParam = paramsData;
}

void UmsSerialMethods::getSysStatus()
{
    if (sysStatusThread.joinable())
    {
        sysStatusThread.detach();
    }
    sysStatusThread = std::thread(&UmsSerialMethods::loopToGetSysStatus, this);
}

void UmsSerialMethods::loopToGetSysStatus()
{
    while (sp != nullptr && !stopFlag)
    {
        try
        {
            LowerParameterOperation("sys", 0, 0, sp);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        catch (const std::exception &e)
        {
            if (!stopFlag)
            {
                root.error("loopToGetSysStatus exception: " + std::string(e.what()));
            }
            break;
        }
    }
}
std::string UmsSerialMethods::stringToHex(const std::string &input)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (unsigned char c : input)
    {
        ss << std::setw(2) << static_cast<int>(c);
    }
    return ss.str();
}

void UmsSerialMethods::cleanup()
{

    if (sp != nullptr)
    {
        sp->close();
    }
    if (reThread.joinable())
        reThread.join();
    if (rdThread.joinable())
        rdThread.join();
    if (spThread.joinable())
        spThread.join();
    if (sysStatusThread.joinable())
        sysStatusThread.join();
}

void UmsSerialMethods::refuseController()
{
    if (sp != nullptr)
    {
        // 清除警报
        LowerParameterOperationInt("write", 0, 2, sp);
        sleep(static_cast<unsigned int>(0.1));
        // 使能电机
        LowerParameterOperationInt("write", 0, 4, sp);
    }
}
