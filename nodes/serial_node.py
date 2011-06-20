#!/usr/bin/env python

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

import roslib; roslib.load_manifest("serial_node")
import rospy

import thread
from serial import *
import StringIO

MODE_FIRST_FF = 0
MODE_SECOND_FF = 1

class Publisher:
    """ 
        Prototype of a forwarding publisher.
    """

    def __init__(self, topic, message_type):
        """ Create a new publisher. """ 
        self.topic = topic
        
        # find message type
        package, message = message_type.split('/')
        try:
            m = __import__( package +'.msg')
        except:
            roslib.load_manifest(package)
            m = __import__( package +'.msg')
        m2 = getattr(m, 'msg')
        self.message = getattr(m2, message)
        self.publisher = rospy.Publisher(topic, self.message)
    
    def publish(self, msg):
        """ Publish a message """ 
        m = self.message()
        m.deserialize(msg)
        self.publisher.publish(m)
                         

class Subscriber:
    """ 
        Prototype of a forwarding subscriber.
    """

    def __init__(self, topic, message_type, parent):
        self.topic = topic
        self.parent = parent
        
        # find message type
        package, message = message_type.split('/')
        try:
            m = __import__( package +'.msg')
        except:
            roslib.load_manifest(package)
            m = __import__( package +'.msg')
        m2 = getattr(m, 'msg')
        self.message = getattr(m2, message)
        rospy.Subscriber(topic, self.message, self.callback)

    def callback(self, msg):
        """ Forward a message """
        data_buffer = StringIO.StringIO()
        msg.serialize(data_buffer)
        self.parent.send(self.topic, data_buffer.getvalue())


class SerialNode:
    """
        Prototype of node to connect to serial bus.
    """
    
    def __init__(self, port=None, baud=57600):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        self.mutex = thread.allocate_lock()
        rospy.init_node("serial_node")

        if port == None:
            # no port specified, listen for any new port?
            pass
        else:
            # open a specific port
            self.port = Serial(port, baud)
                
        self.publishers = dict()
        self.subscribers = dict()
        self.requestTopics()
        self.spin()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        self.port.flushInput()
        # request topic sync
        self.port.write("\xff\xff\x00\x00\x00\x010")

    def spin(self):
        """ Forward recieved messages to appropriate publisher. """
        mode = MODE_FIRST_FF
        while not rospy.is_shutdown():
            if mode == MODE_FIRST_FF:
                if self.port.read() == '\xff':
                    mode += 1
            elif mode == MODE_SECOND_FF:
                x = self.port.read()
                if x == '\xff':
                    topic = ord(self.port.read())
                    if topic == 0:
                        # parse a topic
                        l = ord(self.port.read())
                        print l
                        topic_id = ord(self.port.read())
                        print topic_id
                        l1 = ord(self.port.read())
                        print l1
                        topic_name = self.port.read(l1)
                        print topic_name            
                        l2 = ord(self.port.read())
                        print l2
                        topic_type = self.port.read(l2)            
                        print topic_type
                        # checksum?
                        #chk = self.port.read()
                        if topic_id < 128:
                            self.publishers[topic_id] = Publisher(topic_name, topic_type)
                        else:
                            self.subscribers[topic_name] = [topic_id, Subscriber(topic_name, topic_type, self)]
                    else:   
                        chk   = topic
                        ttype = ord(self.port.read())
                        chk   += ttype
                        bytes = ord(self.port.read())<<8
                        chk   += bytes >> 8
                        bytes += ord(self.port.read())
                        chk   += bytes&255                        
                        msg   = self.port.read(bytes-1)
                        chk   += sum([ord(x) for x in msg] )
                        chk   += ord(self.port.read())   
                        if chk%256 == 255:
                            try:
                                self.publishers[topic].publish(msg)
                            except KeyError:
                                rospy.loginfo("Tried to publish before configured, topic id %d" % topic)
                        else:
                            rospy.logerr("Failed checksum")
                else:       
                    rospy.loginfo("Failed packet")
                mode = MODE_FIRST_FF
            
    def send(self, topic, msg):
        """ Send a message on a particular topic to the device. """
        with self.mutex:
            length = 1 + len(msg)
            checksum = 255 - ( (self.subscribers[topic][0] + (length>>8) + (length&255) + sum([ord(x) for x in msg]))%256 )
            self.port.write('\xff\xff')
            self.port.write(chr(self.subscribers[topic][0]))
            self.port.write('\x00')
            self.port.write(chr(length>>8))
            self.port.write(chr(length&255))
            self.port.write(msg)
            self.port.write(chr(checksum))

if __name__=="__main__":
    SerialNode("/dev/ttyUSB0")

