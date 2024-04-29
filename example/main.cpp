//
// Created by anaple on 28/04/24.
//

#include "ums_serial_methods.hpp"

int main(){

     UmsSerialMethods thisUms = UmsSerialMethods("/dev/ttyUSB0",921600, true,40);
     thisUms.getSerial();
     std::shared_ptr<FictionData> data = make_shared<FictionData>();

     thisUms.loopUmsFictionData(data);


     while (true){
         thisUms.sendTwistData(std::make_shared<TwistCustom>());
         sleep(0.001);
         //printf("battery: %f sleep(static_cast<unsigned int>(0.8)); \n",data->powerData.input);
     }


}