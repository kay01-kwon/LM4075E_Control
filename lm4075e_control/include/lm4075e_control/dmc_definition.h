#ifndef DMC_DEFINITION_H
#define DMC_DEFINITION_H
#include <iostream>
// Header
const uint8_t H1 = 0xFF;
const uint8_t H2 = 0xFE;

// ID
const uint8_t ID1 = 0x00;
const uint8_t ID2 = 0x01;

// Control Mode
const uint8_t PositionMode1 = 0x01;
const uint8_t PositionMode2 = 0x02;
const uint8_t VelocityMode = 0x03;

// Set Mode 
const uint8_t IDSetMode = 0x06;
const uint8_t BaudrateSetMode = 0x07;
const uint8_t ResolutionSetMode = 0x04;
const uint8_t ReductionSetMode = 0x0B;
const uint8_t ResponseTimeSetMode = 0x08;
const uint8_t ControlOnSetMode = 0x0C;
const uint8_t PositionControlSetMode = 0x04;
const uint8_t AbsolutePositionControl = 0x00;
const uint8_t RelativePositionControl = 0x01;
const uint8_t PositionControlSetMode13 = 0x0D;

// Init Mode
const uint8_t PosInitMode = 0x0F;
const uint8_t FactoryMode = 0x02;

// Request Mode
const uint8_t RequestPing = 0xA0;
const uint8_t RequestPos = 0xA1;
const uint8_t RequestVel = 0xA2;
const uint8_t RequestPositionControl = 0xA3;
const uint8_t RequestVelocityControl = 0xA4;
const uint8_t RequestResponseTime = 0xA5;
const uint8_t RequestMotorRatingVelocity = 0xA6;
const uint8_t RequestResolutionFeedback = 0xA7;
const uint8_t RequestReductionRatio = 0xA8;
const uint8_t RequestOnOffFeedback = 0xA9;
const uint8_t RequestPosControlModeFeedback = 0xAA;
const uint8_t RequestControlDirectionFeedback = 0xAB;
const uint8_t RequestFirmwareVersion = 0xCD;

// Baudrate Info
const uint8_t Baudrate600 = 0x02;
const uint8_t Baudrate9600 = 0x06;
const uint8_t Baudrate19200 = 0x08;
const uint8_t Baudrate76800 = 0x0C;
const uint8_t Baudrate115200 = 0x0D;
const uint8_t Baudrate250000 = 0x0F;

struct pos_vel_data{
    int id;
    float position;
    int velocity_direction;
    float velocity;
    float current;
};

#endif