#include "ums_serial_methods.hpp"

int main(){
    UmsSerialMethods thisUms = UmsSerialMethods("/dev/ttyUSB0",20000000, true,40);
    thisUms.getSerial();
    std::shared_ptr<FictionData> fictionData =  std::make_shared<FictionData>();
    thisUms.loopUmsFictionData(fictionData);
    while (true){

       // printf("battery %f \n",fictionData->powerData.input);
    };

}