def crc16(data: bytes, poly=0xA001):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ poly if (crc & 0x0001) else crc >> 1
    return crc

# 将字符串数据转换为字节
data_hex = "532003000000000000000000000000000000004f76cee6eec34a3f00000000000000"
data_bytes = bytes.fromhex(data_hex)

# 计算 CRC
crc_result = crc16(data_bytes)
print("CRC-16:", hex(crc_result))
