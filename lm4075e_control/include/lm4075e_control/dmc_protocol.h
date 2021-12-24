#ifndef DMCPROTOCOL_H
#define DMCPROTOCOL_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <unistd.h>
#include <lm4075e_control/dmc_definition.h>
#include <lm4075e_control/packet_info.h>

unsigned int delay_usec = 200;

using serial::Serial;
using std_msgs::Float32;

class DMC_Protocol{
    
    public:
    // Constructor
    DMC_Protocol();
    
    int serial_open(const std::__cxx11::string & filename,uint32_t baudrate);
    
    void PositionCallback(const std_msgs::Float32::ConstPtr & msg);
    void FactorRest(uint8_t id);
    void PositionReset(uint8_t id);

    void IdSet(uint8_t ID_prev, uint8_t ID_set);
    void BaudrateSet(uint8_t id,uint8_t baudrate);
    void ResolutionSet(uint8_t id,uint16_t resolution);
    void ReductionSet(uint8_t id, uint16_t ratio);
    void ResponseTimeSet(uint8_t id, uint8_t response_time);
    void PositionControlModeSet(uint8_t id, uint8_t ControlMode);

    void PositionRequest(uint8_t id);
    void PositionRead();

    void VelocityRequest(uint8_t id);
    void VelocityRead();

    void ReductionRequest(uint8_t id);
    void ReductionRead();

    void ControlOnSet(uint8_t id);
    void PositionControlSet(uint8_t id);
    void PositionControl(uint8_t id, float position);
    void VelocityControl(uint8_t id, float velocity);

    void wait();
    // Destructor
    ~DMC_Protocol();


    private:
    ros::NodeHandle nh;
    ros::Subscriber position_data_subscriber;
    
    Serial ser;

    uint8_t packet[12];
    uint8_t packet_;

    pos_vel_data data;

};

DMC_Protocol::DMC_Protocol()
{
    position_data_subscriber = nh.subscribe("Position",1,&DMC_Protocol::PositionCallback,this);
}

int DMC_Protocol::serial_open(const std::__cxx11::string & filename, uint32_t baudrate)
{
    try
    {
        ser.setPort(filename);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setTimeout(to);
        ser.open();
        return 0;
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port Initialized");
    }else{
        return -1;
    }
}

void DMC_Protocol::PositionCallback(const std_msgs::Float32::ConstPtr & msg)
{
    ControlOnSet(ID1);
    std::cout<<"Position Control"<<std::endl;
    PositionControl(ID1,msg->data);
    std::cout<<"Request"<<std::endl;
    PositionRequest(ID1);
    PositionRead();
    std::cout<<"\n";
}

void DMC_Protocol::FactorRest(uint8_t id)
{
    uint8_t data_size = 0x02;
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size + FactoryMode);
    std::cout<<"Factor Reset"<<std::endl;
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&FactoryMode,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::PositionReset(uint8_t id)
{
    uint8_t data_size = 0x02;

    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size + PosInitMode);
    std::cout<<"Position Reset"<<std::endl;
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&PosInitMode,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::BaudrateSet(uint8_t id, uint8_t baudrate)
{
    uint8_t data_size = 0x03;
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size + BaudrateSetMode + baudrate);
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&BaudrateSetMode,1);
    ser.write(&baudrate,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::ResolutionSet(uint8_t id, uint16_t resolution)
{
    uint8_t data_size = 0x04;
    uint8_t crc;
    crc = 0x00;
    uint8_t resolution_low = resolution;
    uint8_t resolution_high = uint8_t (resolution<<8);
    crc = ~(id + data_size + ResolutionSetMode + resolution_low + resolution_high);
    std::cout<<"Resolution Set: "<<(unsigned)resolution<<std::endl;
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&ResolutionSetMode,1);
    ser.write(&resolution_high,1);
    ser.write(&resolution_low,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::ReductionSet(uint8_t id, uint16_t ratio)
{
    uint8_t data_size = 0x04;
    uint8_t crc;
    crc = 0x00;
    uint8_t ratio_low = ratio;
    uint8_t ratio_high = uint8_t (ratio>>8);
    crc = ~(id + data_size + ReductionSetMode + ratio_low + ratio_high);
    std::cout<<"Reduction Ratio Set: "<<(unsigned)ratio<<std::endl;
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&ReductionSetMode,1);
    ser.write(&ratio_high,1);
    ser.write(&ratio_low,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::ResponseTimeSet(uint8_t id, uint8_t response_time)
{
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size3 + ResponseTimeSetMode + response_time);
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size3,1);
    ser.write(&crc,1);
    ser.write(&ResponseTimeSetMode,1);
    ser.write(&response_time,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::PositionControlModeSet(uint8_t id, uint8_t ControlMode)
{
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size3 + PositionControlSetMode13 + ControlMode);
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size3,1);
    ser.write(&crc,1);
    ser.write(&PositionControlSetMode13,1);
    ser.write(&ControlMode,1);
    ser.flush();
    std::cout<<"Position Control Mode Setup"<<std::endl;
    usleep(delay_usec);
}

void DMC_Protocol::PositionRequest(uint8_t id)
{
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size2 + RequestPos);

    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size2,1);
    ser.write(&crc,1);
    ser.write(&RequestPos,1);
    ser.flush();
    usleep(delay_usec);    
}

void DMC_Protocol::PositionRead()
{
    int idx = 0;

    uint16_t data1;
    uint16_t data2;
    uint16_t result1;

    uint16_t data3;
    uint16_t data4;
    uint16_t result2;

    while(ser.available())
    {
        ser.read(&position_packet_,1);

        if(idx == position_feedback_packet_size)
            break;

        position_packet[idx] = position_packet_;
        idx++;
    }
    ser.flush();
    
    for (int i = 0; i < position_feedback_packet_size;i++)
        std::cout<<std::hex<<(unsigned)position_packet[i]<<"  ";
    std::cout<<"\n";
    
    // Position Data(Degree)    
    data1 = (uint16_t) position_packet[7];
    data2 = (uint16_t) position_packet[8];
    result1 = (data1<<8) + data2;

    // Velocity Data(RPM)
    data3 = (uint16_t) position_packet[9];
    data4 = (uint16_t) position_packet[10];
    result2 = (data3<<8) + data4;

    data.id = position_packet[2];
    data.velocity_direction = position_packet[6];
    data.position = (float) result1;
    data.position = 0.01 * data.position;
    data.velocity = (float) 0.1*result2;
    data.current = 100.0*position_packet[11];

    if (packet[6] == 0x01)
        data.position = (float)-data.position;
    
    std::cout<<"Position data: "<<data.position<<std::endl;

}


void DMC_Protocol::ReductionRequest(uint8_t id)
{
    uint8_t data_size = 0x02;
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size + RequestRed);

    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&RequestRed,1);
    ser.flush();
}

void DMC_Protocol::ReductionRead()
{
    int packet_size = 9;
    int idx = 0;
    uint8_t packet[packet_size];
    uint8_t packet_;

    while(ser.available())
    {
        ser.read(&packet_,1);
        if(idx == packet_size)
            break;

        packet[idx] = packet_;
        
        idx++;
    }

    for (int i = 0; i < packet_size;i++)
        std::cout<<std::hex<<(unsigned)packet[i]<<"  ";
    std::cout<<"\n";

}

void DMC_Protocol::ControlOnSet(uint8_t id)
{
    uint8_t control_on = 0x00;
    uint8_t crc;
    crc = 0x00;
    crc = ~(id + data_size3 + ControlOnSetMode);

    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size3,1);
    ser.write(&crc,1);
    ser.write(&ControlOnSetMode,1);
    ser.write(&control_on,1);
    ser.flush();
    usleep(delay_usec);
}

void DMC_Protocol::PositionControlSet(uint8_t id)
{
    uint8_t crc;
    uint8_t Kp, Ki, Kd, current;
    Kp = 0xFE;
    Ki = 0xFE;
    Kd = 0xFE;
    current = 0x50;
    crc = ~(id + data_size6 + PositionControlSetMode+ Kp + Ki + Kd + current);
    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size6,1);
    ser.write(&crc,1);
    ser.write(&PositionControlSetMode,1);
    ser.write(&Kp,1);
    ser.write(&Ki,1);
    ser.write(&Kd,1);
    ser.write(&current,1);
    ser.flush();
    usleep(delay_usec);    

}

void DMC_Protocol::PositionControl(uint8_t id, float position)
{
    uint8_t crc;

    crc = 0x00;
    position_data = uint16_t (abs(position*100));
    position_data_l = (uint8_t) position_data;
    position_data_h = (uint8_t) (position_data>>8);
    
    if (position < 0)
        control_direction = 0x01;
    else
        control_direction = 0x00;

/**
    std::cout<<"Position Sign: ";
    std::cout<<std::hex<<(unsigned)position_sign<<std::endl;

    std::cout<<"Position Byte: ";
    std::cout<<std::hex<<(unsigned)position_data_h<<"  ";
    std::cout<<std::hex<<(unsigned)position_data_l<<std::endl;
**/    
    crc = ~(id + data_size7 + PositionMode1 + control_direction + position_data_h + position_data_l + RPM_h + RPM_l);

    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size7,1);
    ser.write(&crc,1);
    ser.write(&PositionMode1,1);
    ser.write(&control_direction,1);
    ser.write(&position_data_h,1);
    ser.write(&position_data_l,1);
    ser.write(&RPM_h,1);
    ser.write(&RPM_l,1);
    ser.flush();
    usleep(delay_usec);

}

void DMC_Protocol::VelocityControl(uint8_t id,float velocity)
{
    uint8_t data_size = 0x06;
    uint8_t velocity_sign = 0x00;
    uint8_t velocity_data_l, velocity_data_h;
    uint8_t time;
    uint16_t velocity_data;

    velocity_data = uint16_t (abs(velocity*10));
    velocity_data_l = (uint8_t) velocity_data;
    velocity_data_h = (uint8_t) (velocity_data>>8);
    time = 0x01;

    std::cout<<(unsigned)velocity_data_h<<"  ";
    std::cout<<(unsigned)velocity_data_l<<std::endl;
    if (velocity < 0)
        velocity_sign = 0x01;

    
    uint8_t crc;
    crc = 0x00;
    
    crc = ~(id + data_size + VelocityMode + velocity_sign + velocity_data_h + velocity_data_l + time);

    ser.write(&H1,1);
    ser.write(&H2,1);
    ser.write(&id,1);
    ser.write(&data_size,1);
    ser.write(&crc,1);
    ser.write(&VelocityMode,1);
    ser.write(&velocity_sign,1);
    ser.write(&velocity_data_h,1);
    ser.write(&velocity_data_l,1);
    ser.write(&time,1);
    ser.flush();
    usleep(delay_usec);
}

DMC_Protocol::~DMC_Protocol()
{
    position_data_subscriber.~Subscriber();
    ser.close();
    std::cout<<"Destructor"<<std::endl;
}

#endif
