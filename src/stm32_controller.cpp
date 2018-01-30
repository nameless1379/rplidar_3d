#include "ros/ros.h"
#include "rplidar_3d/stm32_cmd.h"
#include <cstdlib>
#include <math.h>

#define STEPPER_MAX_SPEED 8*M_PI
#define STEPPER_MIN_SPEED 0.8f
#define STEPPER_SPEED_PSC M_PI/30
int main(int argc, char **argv)
{
  ros::init(argc, argv, "stm32_controller");

  if (argc < 2)
  {
    ROS_INFO("usage: set RPM for stepper motor");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client =
    n.serviceClient<rplidar_3d::stm32_cmd>("stepper_set_speed");

  rplidar_3d::stm32_cmd srv;
  int16_t RPM = atoi(argv[1]);

  if(!strcmp(argv[2],"ccw"))
    RPM = -RPM;

  float speed = (float)RPM* STEPPER_SPEED_PSC;

  if(fabsf(speed) > STEPPER_MAX_SPEED)
  {
    fprintf(stderr, "E: Too high speed\r\n");
    return -1;
  }

  if(fabsf(speed) < STEPPER_MIN_SPEED && RPM != 0)
  {
    fprintf(stderr, "E: Too low speed\r\n");
    return -1;
  }

  ROS_INFO("Setting stepper speed to %d RPM\r\n", RPM);
  srv.request.speed = speed;

  if (client.call(srv) && !srv.response.success)
      ROS_INFO("E: Transmission failed\r\n");

  return 0;
}
