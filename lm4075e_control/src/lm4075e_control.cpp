#include <lm4075e_control/dmc_protocol.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Test");
    
    uint8_t ResponseTime = 0x05;

    uint16_t CPR = 16;
    uint16_t ReductionRatio = 191;
    uint32_t baudrate = 115200;


    DMC_Protocol dmc_protocol;

    std::__cxx11::string dev = "/dev/ttyXRUSB0";

    dmc_protocol.serial_open(dev,baudrate);

    //dmc_protocol.FactorRest(ID1);
    dmc_protocol.BaudrateSet(ID1,Baudrate115200);
    dmc_protocol.PositionReset(ID1);
    dmc_protocol.ResolutionSet(ID1,CPR);
    dmc_protocol.ReductionSet(ID1,ReductionRatio);
    dmc_protocol.ResponseTimeSet(ID1,ResponseTime);

    //dmc_protocol.PositionControlModeSet(ID1,AbsolutePositionControl);
    dmc_protocol.PositionControlSet(ID1);
    dmc_protocol.ControlOnSet(ID1);

    float data;
    data = 100;

    dmc_protocol.PositionControl(ID1,data);
    dmc_protocol.PositionRequest(ID1);
    dmc_protocol.PositionRead();

    while(ros::ok())
    {
        ros::spinOnce();
    }

}