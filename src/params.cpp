#include <params.hpp>
#include <ums_serial_methods.hpp>

ParamsData ParamDataRead(uint8_t *data)
{
    uint8_t msg_len = data[3];
    ParamsData result;
    try
    {
        uint8_t *p = data + 4;
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
        }
    }
    catch (const std::exception &e)
    {
        printf("err");
        std::cerr << e.what() << '\n';
    }
    return result;
}
