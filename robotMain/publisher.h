#ifndef PUBLISHER_H
#define PUBLISHER_H
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>

void pubSetup();
void pubLoop(int CO2Data);

#endif