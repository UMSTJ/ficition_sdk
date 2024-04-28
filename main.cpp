#include "ums_serial_methods.hpp"
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>

int main()
{

    UmsSerialMethods umsSerialMethods;
    FictionData fictionData;
    umsSerialMethods.test(1);

    return 0;
}

// g++ main.cpp -Iinclude/ -Iinclude/serial -L build/ -lums_sdk -lserial