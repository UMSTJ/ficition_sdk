
#include <crc.hpp>


uint16_t CalCRC16(const std::vector<uint8_t> &data)
{
    unsigned char uchCRCHi = 0xFF; // 高CRC字节初始化
    unsigned char uchCRCLo = 0xFF; // 低CRC 字节初始化
    unsigned char uIndex;          // CRC循环中的索引

    for (auto byte : data)
    {

        uIndex = uchCRCHi ^ byte; // 计算CRC
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}