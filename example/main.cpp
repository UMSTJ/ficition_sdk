//
// Created by anaple on 28/04/24.
//

#include "ums_serial_methods.hpp"

int main()
{
    std::shared_ptr<UmsSerialMethods> umsSerialMethods = make_shared<UmsSerialMethods>("/dev/ttyUSB0", 921600, true, 40, AgreementVersion::V1);
    umsSerialMethods->getSerial();
    std::shared_ptr<FictionData> data = make_shared<FictionData>();
    umsSerialMethods->loopUmsFictionData(data);

    while (true)
    {
        umsSerialMethods->sendTwistData(std::make_shared<TwistCustom>());
        sleep(0.001);
        // printf("battery: %f sleep(static_cast<unsigned int>(0.8)); \n",data->powerData.input);
    }
}