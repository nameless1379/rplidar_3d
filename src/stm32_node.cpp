/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "ros/ros.h"

#include "rplidar_3d/stm32_cmd.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "stm32_serial.h"
#include "rptypes.h"

#include <tf2/LinearMath/Transform.h>

#define DEG2RAD(x) ((x)*M_PI/180.)

static stm32_serial* serial;

/**
  * TODO: Modify stm32 to transmit quaternion
  *       call this function to publish attitude data
  */
void stm32_serial::publish_pos_msg(ros::Publisher *pub, stm32_serial_packet_t *node, std::string frame_id)
{
    geometry_msgs::QuaternionStamped pos_msg;

    ros::Duration duration((node->timeStamp - timeStamp_start)*1e-3);

    pos_msg.header.stamp = ros_start + duration;
    pos_msg.header.frame_id = frame_id;

    tf2::Quaternion q1,q2;
    q2.setEulerZYX(-node->stepper_angle,DEG2RAD(22.5),0);

    if (node->imu_data[0] != 100.0f)
      q1 = tf2::Quaternion(node->imu_data[0], node->imu_data[1], node->imu_data[2], node->imu_data[3]);
    else
      q1  = tf2::Quaternion::getIdentity();

    //q2*= q1;

    pos_msg.quaternion.x = (double)(q2.x());
    pos_msg.quaternion.y = (double)(q2.y());
    pos_msg.quaternion.z = (double)(q2.z());
    pos_msg.quaternion.w = (double)(q2.w());

    pub->publish(pos_msg);
}

#define STEPPER_MAX_SPEED 8*M_PI
#define STEPPER_MIN_SPEED 0.8f
#define STEPPER_SPEED_PSC M_PI/30
bool stepper_set_speed(rplidar_3d::stm32_cmd::Request  &req,
                       rplidar_3d::stm32_cmd::Response &res)
{
  return IS_FAIL(serial->transmit_stepper_cmd(req.speed));
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "stm32_serial_node");

    std::string serial_port;
    int serial_baudrate = 115200;
    int stepper_RPM = 0;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/lidar_pos", 1000);

    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh_private.param<int>("stepper_RPM", stepper_RPM, 0);

    serial = new stm32_serial;

    printf("3D scanning RPLIDAR by Edward ZHANG\n");

    u_result op_result;
    // make connection...

    if (IS_FAIL(serial->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
      fprintf(stderr, "E: cannot bind to the specified serial port %s.\n"
          , serial_port.c_str());
      return -1;
    }

    if(IS_FAIL(serial->transmit_handshake()))
    {
      fprintf(stderr, "E: failed to establish transmission with STM32\n");
      return -2;
    }

    float stepper_speed = (float)stepper_RPM* STEPPER_SPEED_PSC;
    if(fabsf(stepper_speed) > STEPPER_MAX_SPEED)
    {
      fprintf(stderr, "E: Too high stepper speed\r\n");
      return -3;
    }

    if(fabsf(stepper_speed) < STEPPER_MIN_SPEED && stepper_RPM != 0)
    {
      fprintf(stderr, "E: Too low stepper speed\r\n");
      return -3;
    }

    ros::ServiceServer set_speed_srv = nh.advertiseService("stepper_set_speed", stepper_set_speed);

    serial->transmit_stepper_cmd(stepper_speed);
    serial->start_rx(DEFAULT_TIMEOUT);
    printf("Started receiving data...\n");

    ros::Rate r(250);
    while (ros::ok())
    {
        stm32_serial_packet_t nodes[360*2];
        size_t   count = 720;

        op_result = serial->grabPacket(nodes, count, DEFAULT_TIMEOUT);

        if (op_result != RESULT_OK)
        {
          printf("W: Connection lost!\n");
          return -1;
        }
/*
        if (nodes[0].imu_data[0] == 100.0f)
        {
          printf("W: Failed to obtain imu data!\n");
          //return -2;
        }
*/
        printf("stepper angle: %f\n", nodes[0].stepper_angle * 180.0f/M_PI);
        serial->publish_pos_msg(&pos_pub, nodes, "PCL2_frame");

        r.sleep();
        ros::spinOnce();
    }

    delete serial;

    return 0;
}
