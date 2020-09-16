#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__author__ = "mferguson@willowgarage.com (Michael Ferguson)"

import rclpy
from rclpy.node import Node
from ros2serial_python.serial_client import SerialClient
from ros2serial_python.serial_server import SerialServer
from serial import SerialException
from time import sleep
import multiprocessing

import sys

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.get_logger().info('Initialized serial_node')
        self.declare_parameters(namespace='',
                                parameters=[ ('port', '/dev/ttyUSB0'),
                                             ('baudrate', 115200),
                                             ('tcp_port', 11411),
                                             ('fork_server', True),
                                             ('mode', 0)])
        self.port_name = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.fork_server = self.get_parameter('fork_server').get_parameter_value().integer_value
        
        # can be used to write some code for simulated ports
        self.mode = self.get_parameter('mode').get_parameter_value().integer_value
    
        self.get_logger().info('port: ' + self.port_name)
        self.get_logger().info('baudrate: ' + str(self.baudrate))
        self.get_logger().info('tcp_port: ' + str(self.tcp_port) + ' [optional]')
        self.get_logger().info('fork_server: ' + str(self.fork_server) + ' [optional]')

        self.startup()


    def startup(self):
        # TODO: should we accept cli params and ros params?
        if len(sys.argv) >= 2 :
            self.port_name  = sys.argv[1]
        if len(sys.argv) == 3 :
            self.tcp_portnum = int(sys.argv[2])
    
        if self.port_name == "tcp" :
            server = SerialServer(self.tcp_portnum, self.fork_server)
            self.get_logger().info("Waiting for socket connections on port %d" % self.tcp_portnum)
            try:
                server.listen()
            except KeyboardInterrupt:
                self.get_logger().info("got keyboard interrupt")
            finally:
                self.get_logger().info("Shutting down")
                for process in multiprocessing.active_children():
                    self.get_logger().info("Shutting down process %r", process)
                    process.terminate()
                    process.join()
                rclpy.loginfo("All done")
    
        else :          # Use serial port
            while rclpy.ok():
                self.get_logger().info("Connecting to %s at %d baudrate" % (self.port_name, self.baudrate) )
                try:
                    client = SerialClient(self.port_name, self.baudrate, mode=self.mode)
                    client.run()
                except KeyboardInterrupt:
                    rclpy.shutdown()
                    break
                except SerialException:
                    sleep(1.0)
                    continue
                except OSError:
                    sleep(1.0)
                    continue
                except Exception as e:
                    self.get_logger().warn('Unexpected Error: ' + str(e))
                    sleep(1.0)
                    continue
        

def main():
    rclpy.init(args=sys.argv)
    serial_node = SerialNode()

    rclpy.spin(serial_node)

    rclpy.shutdown()

if __name__=="__main__":
    main()
