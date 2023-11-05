#include "publisher.h"
ros::NodeHandle  nh1;

std_msgs::Int32 int_msg;
ros::Publisher testInt("testInt", &int_msg);


void pubSetup()
{
  nh1.initNode();
  nh1.advertise(testInt);
}
void pubLoop(int CO2Data)
{
  int CO2 = CO2Data;
  int_msg.data = CO2;
  testInt.publish( &int_msg );
  nh1.spinOnce();
  delay(1000);
}