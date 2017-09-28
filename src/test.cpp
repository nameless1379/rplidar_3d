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

#include "stm32_serial.h"
#include "rptypes.h"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "stm32_serial_node");

    std::string serial_port;
    int serial_baudrate = 115200;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);

    stm32_serial* serial = new stm32_serial;

    printf("3D scanning RPLIDAR by Edward ZHANG\n");

    u_result op_result;
    // make connection...

    if (IS_FAIL(serial->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , serial_port.c_str());
        return -1;
    }

    serial->transmit_test();

    printf("Sent data string\n");

    while (ros::ok())
    {
/*
        stm32_serial_packet_t nodes[360*2];
        size_t   count = 720;

        op_result = serial.grabPacket(nodes, count, DEFAULT_TIMEOUT);

        if (op_result == RESULT_OK)
        {
          printf("Time:%d\n", nodes[count].timeStamp);
        }
*/
        ros::spin();
    }

    delete serial;

    return 0;
}
