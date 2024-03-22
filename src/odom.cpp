#include <odom.hpp>

OdomInfo OdomDataProcess(std::vector<uint8_t> ImuData)
{
    // x方向速度 vx ，y 方向速度 vy ，角速度 ωz
    double vx_chassis = 0;
    double vy_chassis = 0;
    double wz_chassis = 0;
    OdomInfo result;

    vx_chassis = DirectionalInterception(4, 8, ImuData);
    vy_chassis = DirectionalInterception(12, 8, ImuData);
    wz_chassis = DirectionalInterception(20, 8, ImuData);
    result.delta_x = vx_chassis;
    result.delta_y = vy_chassis;
    result.delta_th = wz_chassis;

    return result;
}
