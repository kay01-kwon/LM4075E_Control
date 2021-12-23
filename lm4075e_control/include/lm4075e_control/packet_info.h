#ifndef PACKET_INFO_H
#define PACKET_INFO_H
#include <stdint.h>
#include <iostream>

const uint8_t data_size2 = 0x02;
const uint8_t data_size3 = 0x03;
const uint8_t data_size4 = 0x04;

const uint8_t data_size6 = 0x06;
const uint8_t data_size7 = 0x07;
const uint8_t data_size8 = 0x08;

// Position velocity control
uint8_t control_direction = 0x00;
uint16_t control_position = 0x0000;
uint16_t control_velocity = 0x0000;

uint8_t position_sign = 0x00;
uint8_t position_data_l = 0x00;
uint8_t position_data_h = 0x00;

uint16_t position_data = 0x0000;
uint16_t velocity_data = 0x0000;

const uint8_t RPM_h = 0x03;
const uint8_t RPM_l = 0xEB;

// Position feedback
int position_feedback_packet_size = 12;
uint8_t position_packet[12];
uint8_t position_packet_;

uint8_t velocity_h, velocity_l;


uint16_t position_data_H, position_data_L;

pos_vel_data data;



#endif