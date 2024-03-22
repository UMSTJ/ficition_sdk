#include "battery.hpp"
#include "ums_serial_methods.hpp"


PowerInfo PowerDataProcess(std::vector<uint8_t>  NativeData){
    // 总线电流
    double bus = DirectionalInterception(4, 8, NativeData);
    // 5v输出
    double output = DirectionalInterception(12, 8, NativeData);
    // 输入电压
    double input = DirectionalInterception(20, 8, NativeData);
    // 19v输出
    double output19 = DirectionalInterception(28, 8, NativeData);

    PowerInfo result;
    result.bus = bus;
    result.output5v = output;
    result.input= input;
    result.output19v = output19;
    return result;

}