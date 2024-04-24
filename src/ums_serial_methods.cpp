#include "ums_serial_methods.hpp"
#include <iostream>


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

std::string UmsSerialMethods::magneticDataProcess(const std::vector<uint8_t>& NativeData)
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
std::vector<uint8_t> UmsSerialMethods::DataDelivery(uint8_t signbit, std::vector<uint8_t> &Vector)
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

void UmsSerialMethods::LowerParameterOperationInt(const std::string& basicString, uint8_t address, int32_t data, const std::shared_ptr<serial::Serial>& Sp)
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
void UmsSerialMethods::LowerParameterOperation(const std::string& red_write, uint8_t address, float data, const std::shared_ptr<serial::Serial>& Sp)
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
    else if (red_write == "sys" && Sp != nullptr){
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
    else
    {
        root.warn("CRC16 check failed");
        root.warn("CRC16 rgl_data: %02x , %02x org_data: %02x ,%02x", result_h,result_l,data[data.size() - 3],data[data.size() - 2]);
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
    for (size_t i = 0; i < byteVector.size() - 1; ++i)
    {
        if (byteVector[i] == 0x5c)
        {
            if (byteVector[i + 1] == 0x00)
            {
                // 将 5c 00 替换为 5c
                byteVector.erase(byteVector.begin() + i + 1);
            }
            else if (byteVector[i + 1] == 0x01)
            {
                // 将 5c 01 替换为 3a
                byteVector[i] = 0x3a;
                byteVector.erase(byteVector.begin() + i + 1);
            }
            else if (byteVector[i + 1] == 0x02)
            {
                // 将 5c 02 替换为 0a
                byteVector[i] = 0x0a;
                byteVector.erase(byteVector.begin() + i + 1);
            }
            else if (byteVector[i + 1] == 0x03)
            {
                // 将 5c 03 替换为 0d
                byteVector[i] = 0x0d;
                byteVector.erase(byteVector.begin() + i + 1);
            }
        }
    }
}
PowerInfo UmsSerialMethods::PowerDataProcess(const std::vector<uint8_t>& NativeData)
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
    std::vector<uint8_t> subVector;


    try
    {
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
void UmsSerialMethods::createSerial(const std::string& portName, int baudRate)
{
    while (!stopFlag)
    {

        try
        {
            sp = std::make_shared<serial::Serial>(portName, baudRate, serial::Timeout::simpleTimeout(1000));
            if(sp->isOpen()){
                lastReceiveTime = std::chrono::steady_clock::now();
                timeoutOccurred = false;
                bool checkFin = false;
                std::thread timeoutThread(&UmsSerialMethods::monitorTimeout, this);
                while (sp->available() != 0 &&!stopFlag && !timeoutOccurred){
                    std::string str = sp->readline();
                    stopFlag = false;
                    std::vector<uint8_t> buffer(str.begin(), str.end());
                    EscapeVector(buffer);
                    if (DataCheck(buffer)) {
                        lastReceiveTime = std::chrono::steady_clock::now();
                        root.info("serial port "+portName+" checked successfully");
                        checkFin = true;
                        break;
                    }
                }
                if(timeoutOccurred){
                    root.warn("serial port "+portName+" timeout");
                    if(timeoutThread.joinable())timeoutThread.join();
                    sp = nullptr;
                }
                if(checkFin){
                    root.info("serial port "+portName+" opened successfully");
                    if(timeoutThread.joinable())timeoutThread.join();
                    break;
                }
                if(!checkFin){
                    root.warn("serial port "+portName+" open failed");
                    sp = nullptr;
                }
            } else {
                root.warn("serial port "+portName+" open failed");
                sp = nullptr;
            }
        }
        catch (const std::exception &e)
        {
            sp = nullptr;
            root.error("Exception thrown: %s \n port: %s", e.what(), portName.c_str());
            sleep(3);
        }
    }
    if (stopFlag)
    {
        root.warn("serial thread shotdown \n port: %s", portName.c_str());
    }
}
OdomInfo UmsSerialMethods::OdomDataProcess(const std::vector<uint8_t>& ImuData)
{
    // x方向速度 vx ，y 方向速度 vy ，角速度 ωz
    double vx_chassis;
    double vy_chassis;
    double wz_chassis;
    OdomInfo result{};

    vx_chassis = DirectionalInterception(4, 8, ImuData);
    vy_chassis = DirectionalInterception(12, 8, ImuData);
    wz_chassis = DirectionalInterception(20, 8, ImuData);
    result.delta_x = vx_chassis;
    result.delta_y = vy_chassis;
    result.delta_th = wz_chassis;

    return result;
}
bool UmsSerialMethods::ParamDataWrite() {
    try {
        if(sp->isOpen()){
            std::string LC_read_write = "write";
            if(inputParam.KP != 0)
            LowerParameterOperation(LC_read_write, 8, inputParam.KP,sp);
             if(inputParam.KI != 0)
            LowerParameterOperation(LC_read_write, 12, inputParam.KI,sp);
             if(inputParam.KD != 0)
            LowerParameterOperation(LC_read_write, 16, inputParam.KD,sp);
             if(inputParam.MPE != 0)
            LowerParameterOperation(LC_read_write, 20, inputParam.MPE,sp);
             if(inputParam.MPC != 0)
            LowerParameterOperation(LC_read_write, 24, inputParam.MPC,sp);
             if(inputParam.LA != 0)
            LowerParameterOperation(LC_read_write, 28, inputParam.LA,sp
            );
             if(inputParam.LB != 0)
            LowerParameterOperation(LC_read_write, 32, inputParam.LB,
                                    sp);
             if(inputParam.KMTT != 0)
            LowerParameterOperationInt(LC_read_write, 36, inputParam.KMTT,sp);
             if(inputParam.IMU_Z != 0)
            LowerParameterOperation(LC_read_write, 40, inputParam.IMU_Z,sp);
            return true;
        } else{
            return false;
        }
    }catch (const std::exception &e){
        root.error("Exception thrown: %s \n", e.what());
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
        if(msg_len == 4)
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
                    result.IMU_Z = floatValue;

                    break;
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

    }
    return result;
}
ICDRemote UmsSerialMethods::convertBackDataToControl(int channel1Value, int channel2Value, int channel3Value) {
    float maxOutputValue = 50.0; // 最大输出值

    ICDRemote icdRemoteData{};

    // 自转角速度转换
    float yawRate = ((float )channel1Value / maxOutputValue) * 180; // 假设最大自转角速度为±180度/秒

    // Vx线速度转换
    float vxSpeed = ((float )channel2Value / maxOutputValue) * 1; // 假设Vx的最大速度为±1m/s

    // Vy线速度转换
    float vySpeed = ((float )channel3Value / maxOutputValue) * 1; // 假设Vy的最大速度为±1m/s
    icdRemoteData.az =yawRate;
    icdRemoteData.vx = vxSpeed;
    icdRemoteData.vy = vySpeed;
    return icdRemoteData;


}

RCSBUSRemote UmsSerialMethods::convertRCBusRemote(std::vector<uint8_t> &byteVector) {
    RCSBUSRemote rcsbusRemote{};
    rcsbusRemote.len = byteVector[3];
    for (int index = 0;index <8; index = index+2){
        uint16_t result = (static_cast<uint16_t>(byteVector[index + 5]) << 8) | byteVector[index + 4];
        rcsbusRemote.axes[index / 2] = result;
    }
    return rcsbusRemote;
}
void UmsSerialMethods::loopUmsFictionData(const std::shared_ptr<FictionData>& FictionData) {
    rdThread = std::thread(&UmsSerialMethods::tdLoopUmsFictionData, this, sp, FictionData);
    fictionData = FictionData;


}

void UmsSerialMethods::monitorTimeout() {
    while (!stopFlag) {
        if (std::chrono::steady_clock::now() - lastReceiveTime > std::chrono::seconds(2)) {
            timeoutOccurred = true;
            if(timeoutOccurred){
                break;
            }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
void UmsSerialMethods::tdLoopUmsFictionData(const std::shared_ptr<serial::Serial>& Sp, const std::shared_ptr<FictionData>&  FictionData) {
    lastReceiveTime = std::chrono::steady_clock::now();

    timeoutOccurred = false;

    std::thread timeoutThread(&UmsSerialMethods::monitorTimeout, this);
    try {
        if (sp != nullptr && FictionData != nullptr)
        {
            root.info("Serial port is open");
            root.info("Params sending");
            getSysStatus();
            ParamDataWrite();
            while (!stopFlag && !timeoutOccurred)
            {
                /* code */
                std::string str = sp->readline();
                stopFlag = false;
                std::vector<uint8_t> buffer(str.begin(), str.end());
                EscapeVector(buffer);
                if (DataCheck(buffer))
                {

                    lastReceiveTime = std::chrono::steady_clock::now();
                    switch (buffer[2])
                    {
                        case 0x41: // 电源树 总线电流、5V 输出、输入电压、19V 输出。
                            FictionData->powerData = PowerDataProcess(buffer);
                            break;
                        case 0x42: // 参数读取操作返回内容
                        {
                            FictionData->paramsData = ParamDataRead(&buffer[0]);
                            break;
                        }
                        case 0x45: // 磁条数据
                            FictionData->magneticData = magneticDataProcess(buffer);
                            break;
                        case 0x51: // imu数据
                            FictionData->
                            imuStructural = ImuDataProcess(buffer);
                            break;
                        case 0x52: // rfid数据
                            FictionData->rfidData = std::to_string(Rfid(buffer));
                            break;
                        case 0x46: // 遥控数据RC SBUS
                        {
                            FictionData->rcsBusData = convertRCBusRemote(buffer);
                        }
                        case 0x49: //ICD遥控
                        {
                            FictionData->icdData = convertBackDataToControl(buffer[4],buffer[5],buffer[6]);
                            break;
                        }
                        case 0x4b: // 里程计
                        {
                            FictionData->odomData = OdomDataProcess(buffer);
                            break;
                        }
                        case 0x53: // 四个电机速度数据

                            break;
                        case 0x54: // 数字式温度传感器
                            FictionData->temperature =static_cast<float>((buffer[4] << 8) | buffer[5]);
                            break;
                        case 0x55: // 超声数据
                            FictionData->ultrasonic = static_cast<float>((buffer[4] << 8) | buffer[5]);
                            break;
                        default:
                            // 处理默认情况
                            break;
                    }
                } else {
                    root.warn("data error");
                    root.info("Hexadecimal: "+ stringToHex(str));
                }

            }

            stopFlag = true;
            if (timeoutThread.joinable()) timeoutThread.join();
            if (reThread.joinable()){
                root.warn("reThread running");
                reThread.detach();
            }
            reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
            if(timeoutOccurred){
                root.warn("serial port data:  %ld , timeout", sp->available());
            }
            root.warn("read serial end \n");
        } else{
            root.warn("read serial fail");
        }
    }
    catch (const serial::IOException &e) {
        root.error("read serial exception IO %s",e.what());
        stopFlag = true;
        if (timeoutThread.joinable()) timeoutThread.join();
        if(reThread.joinable()) reThread.detach();
        reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
    }
    catch (const serial::SerialException &e) {
        root.error("read serial exception %s",e.what());
        stopFlag = true;
        if (timeoutThread.joinable()) timeoutThread.join();
        if(reThread.joinable()) reThread.detach();
        reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
    }
    catch (const std::exception &e) {
        root.error("read serial exception %s",e.what());
        stopFlag = true;
        if (timeoutThread.joinable()) timeoutThread.join();
        if(reThread.joinable()) reThread.detach();
        reThread = std::thread(&UmsSerialMethods::reStartSerial, this, sp->getPort(), sp->getBaudrate());
    }

}
void UmsSerialMethods::sendTwistData( const std::shared_ptr<TwistCustom>& twistData)
{
    if (sp != nullptr && twistData != nullptr)
    {
        std::vector<uint8_t> SendHexData;
        std::vector<uint8_t> LeftWheelSpeed = DoubleToBytes(twistData->linear_x);
        std::vector<uint8_t> RightWheelSpeed = DoubleToBytes(twistData->linear_y);
        std::vector<uint8_t> AngularVelocity = DoubleToBytes(twistData->angular_z);
        SendHexData.insert(SendHexData.end(), LeftWheelSpeed.begin(), LeftWheelSpeed.end());
        SendHexData.insert(SendHexData.end(), RightWheelSpeed.begin(), RightWheelSpeed.end());
        SendHexData.insert(SendHexData.end(), AngularVelocity.begin(), AngularVelocity.end());
        SendHexData = DataDelivery(0x4b, SendHexData);

        sp->write(SendHexData);

    }
}
void UmsSerialMethods::sendMessageToGetParamData()
{   try{

        if (sp != nullptr)
        {
            LowerParameterOperation("read", 0, 0, sp);
        }
}catch (const std::exception &e) {

    root.error("sendMessageToGetParamData exception %s",e.what());
}
}
void UmsSerialMethods::reStartSerial(const std::string& portName, int baudRate)
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
    if(spThread.joinable())spThread.join();
    if(rdThread.joinable())rdThread.join();
    loopUmsFictionData(fictionData);
}
void UmsSerialMethods::startSerial(const std::string& portName, int baudRate)
{
    stopFlag = false;
    try
    {
        root.info("wait %s,%d",portName.c_str(),baudRate);
        spThread = std::thread(&UmsSerialMethods::createSerial, this, portName, baudRate);

    }
    catch (const std::exception &e)
    {
        root.error("Exception thrown: %s", e.what());
    }
}

void UmsSerialMethods::setParamsData(ParamsData paramsData) {
    inputParam = paramsData;
}


void UmsSerialMethods::getSysStatus() {
    if(sysStatusThread.joinable()){
        sysStatusThread.detach();
    }
    sysStatusThread = std::thread(&UmsSerialMethods::loopToGetSysStatus, this);

}

void UmsSerialMethods::loopToGetSysStatus() {
    while (sp!= nullptr && !stopFlag){
        try{
            LowerParameterOperation("sys", 0, 0, sp);

        }catch (const std::exception &e){
            root.error("loopToGetSysStatus exception %s",e.what());
        }
        sleep(static_cast<unsigned int>(0.1));

    }
}
std::string UmsSerialMethods::stringToHex(const std::string& input) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (unsigned char c : input) {
        ss << std::setw(2) << static_cast<int>(c);
    }
    return ss.str();
}

void UmsSerialMethods::refuseController() {
    if(sp != nullptr){
        //清除警报
        LowerParameterOperationInt("write", 0, 2, sp);
        sleep(static_cast<unsigned int>(0.1));
        //使能电机
        LowerParameterOperationInt("write", 0, 4, sp);

    }

}


