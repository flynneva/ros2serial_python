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

import array
import errno
import imp
import io
import multiprocessing
import queue
import socket
import struct
import sys
import threading
import time

from serial import Serial, SerialException, SerialTimeoutException

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from ros2serial_msgs.msg import TopicInfo, Log
from ros2serial_msgs.srv import RequestParam

import diagnostic_msgs.msg


class SerialClient(object):
    """
        ServiceServer responds to requests from the serial device.
    """
    header = b'\xff'

    ERROR_MISMATCHED_PROTOCOL = 'Mismatched protocol version in packet: ' \
                                'lost sync or rosserial_python is from different ' \
                                'ros release than the rosserial client'
    ERROR_NO_SYNC = 'no sync with device'
    ERROR_PACKET_FAILED = 'Packet Failed : Failed to read msg data'

    # hydro introduces protocol ver2 which must match node_handle.h
    # The protocol version is sent as the 2nd sync byte emitted by each end
    protocol_ver1 = b'\xff'
    protocol_ver2 = b'\xfe'
    protocol_ver = protocol_ver2


    def __init__(self, port=None, baud=57600, timeout=5.0, mode=0):
        """ Initialize node, connect to bus, attempt to negotiate topics. """
        # get time immediately at start of init
        now = Time()

        self.read_lock = threading.RLock()

        self.write_lock = threading.RLock()
        self.write_queue = queue.Queue()
        self.write_thread = None

        self.lastsync = now
        self.lastsync_lost = now
        self.lastsync_success = now
        self.last_read = now
        self.last_write = now 
        self.timeout = timeout
        self.synced = False
        self.mode = mode

        self.publishers = dict()  # id:Publishers
        self.subscribers = dict() # topic:Subscriber
        self.services = dict()    # topic:Service

        #self.pub_diagnostics = rclpy.Publisher('/diagnostics',
        #                                       diagnostic_msgs.msg.DiagnosticArray,
        #                                       queue_size=10)

        if port is None:
            # no port specified, listen for any new port?
            pass
        elif hasattr(port, 'read'):
            #assume its a filelike object
            self.port=port
        else:
            # open a specific port
            while rclpy.ok():
                try:
                    if (self.mode != 0):
                        # see https://github.com/pyserial/pyserial/issues/59
                        self.port = Serial(port, baud, timeout=self.timeout,
                                           write_timeout=10, rtscts=True, dsrdtr=True)
                    else:
                        self.port = Serial(port, baud, timeout=self.timeout, write_timeout=10)
                    break
                except SerialException as e:
                    rclpy.logging.get_logger('serial_node').error('Error opening serial: ' + str(e))
                    time.sleep(3)

        if not rclpy.ok():
            return

        time.sleep(0.1)           # Wait for ready (patch for Uno)

        self.buffer_out = -1
        self.buffer_in = -1

        self.callbacks = dict()
        # endpoints for creating new pubs/subs
        self.callbacks[TopicInfo.ID_PUBLISHER] = self.setupPublisher
        self.callbacks[TopicInfo.ID_SUBSCRIBER] = self.setupSubscriber
        # service client/servers have 2 creation endpoints (a publisher and a subscriber)
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_PUBLISHER] = self.setupServiceServerPublisher
        self.callbacks[TopicInfo.ID_SERVICE_SERVER+TopicInfo.ID_SUBSCRIBER] = self.setupServiceServerSubscriber
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_PUBLISHER] = self.setupServiceClientPublisher
        self.callbacks[TopicInfo.ID_SERVICE_CLIENT+TopicInfo.ID_SUBSCRIBER] = self.setupServiceClientSubscriber
        # custom endpoints
        self.callbacks[TopicInfo.ID_PARAMETER_REQUEST] = self.handleParameterRequest
        self.callbacks[TopicInfo.ID_LOG] = self.handleLoggingRequest
        self.callbacks[TopicInfo.ID_TIME] = self.handleTimeRequest

        time.sleep(2.0)
        self.requestTopics()
        self.lastsync = Time()

    def requestTopics(self):
        """ Determine topics to subscribe/publish. """
        rclpy.logging.get_logger('serial_node').info('Requesting topics...')

        # TODO remove if possible
        if (self.mode != 0):
            with self.read_lock:
                self.port.flushInput()

        # request topic sync
        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x00\x00\xff")

    def txStopRequest(self):
        """ Send stop tx request to client before the node exits. """
        if (self.mode != 0):
            with self.read_lock:
                self.port.flushInput()

        self.write_queue.put(self.header + self.protocol_ver + b"\x00\x00\xff\x0b\x00\xf4")
        rclpy.logging.get_logger('serial_node').info("Sending tx stop request")

    def tryRead(self, length):
        try:
            read_start = time.time()
            bytes_remaining = length
            result = bytearray()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    self.last_read = Time()
                    result.extend(received)
                    bytes_remaining -= len(received)

            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))

            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)

    def run(self):
        """ Forward recieved messages to appropriate publisher. """

        # Launch write thread.
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self.processWriteQueue)
            self.write_thread.daemon = True
            self.write_thread.start()

        # Handle reading.
        data = ''
        read_step = None
        while self.write_thread.is_alive() and rclpy.ok():
            if (Time() - self.lastsync).to_msg() > (self.timeout * 3):
                if self.synced:
                    rclpy.logging.get_logger('serial_node').error("Lost sync with device, restarting...")
                else:
                    rclpy.logging.get_logger('serial_node').error("Unable to sync with device; possible link problem or link software version mismatch such as hydro ros2serial_python with groovy Arduino")
                self.lastsync_lost = Time()
                self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_NO_SYNC)
                self.requestTopics()
                self.lastsync = Time()

            # This try-block is here because we make multiple calls to read(). Any one of them can throw
            # an IOError if there's a serial problem or timeout. In that scenario, a single handler at the
            # bottom attempts to reconfigure the topics.
            try:
                with self.read_lock:
                    if self.port.inWaiting() < 1:
                        time.sleep(0.001)
                        continue

                # Find sync flag.
                flag = [0, 0]
                read_step = 'syncflag'
                flag[0] = self.tryRead(1)
                if (flag[0] != self.header):
                    continue

                # Find protocol version.
                read_step = 'protocol'
                flag[1] = self.tryRead(1)
                if flag[1] != self.protocol_ver:
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_MISMATCHED_PROTOCOL)
                    rclpy.logging.get_logger('serial_node').error("Mismatched protocol version in packet (%s): lost sync or rosserial_python is from different ros release than the rosserial client" % repr(flag[1]))
                    protocol_ver_msgs = {
                            self.protocol_ver1: 'Rev 0 (rosserial 0.4 and earlier)',
                            self.protocol_ver2: 'Rev 1 (rosserial 0.5+)',
                            b'\xfd': 'Some future rosserial version'
                    }
                    if flag[1] in protocol_ver_msgs:
                        found_ver_msg = 'Protocol version of client is ' + protocol_ver_msgs[flag[1]]
                    else:
                        found_ver_msg = "Protocol version of client is unrecognized"
                    rclpy.logging.get_logger('serial_node').info(found_ver_msg + ', expected ' + str(protocol_ver_msgs[self.protocol_ver]))
                    continue

                # Read message length, checksum (3 bytes)
                read_step = 'message length'
                msg_len_bytes = self.tryRead(3)
                msg_length, _ = struct.unpack("<hB", msg_len_bytes)

                # Validate message length checksum.
                if sum(array.array("B", msg_len_bytes)) % 256 != 255:
                    rclpy.logging.get_logger('serial_node').info('Wrong checksum for msg length, length ' + str(msg.length) + ', dropping message.')
                    continue

                # Read topic id (2 bytes)
                read_step = 'topic id'
                topic_id_header = self.tryRead(2)
                topic_id, = struct.unpack("<H", topic_id_header)

                # Read serialized message data.
                read_step = 'data'
                try:
                    msg = self.tryRead(msg_length)
                except IOError:
                    self.sendDiagnostics(diagnostic_msgs.msg.DiagnosticStatus.ERROR, ERROR_PACKET_FAILED)
                    rclpy.logging.get_logger('serial_node').info('Packet Failed :  Failed to read msg data')
                    rclpy.logging.get_logger('serial_node').info('expected msg length is ' + str(msg_length))
                    raise

                # Reada checksum for topic id and msg
                read_step = 'data checksum'
                chk = self.tryRead(1)
                checksum = sum(array.array('B', topic_id_header + msg + chk))

                # Validate checksum.
                if checksum % 256 == 255:
                    self.synced = True
                    self.lastsync_success = Time()
                    try:
                        self.callbacks[topic_id](msg)
                    except KeyError:
                        rclpy.logging.get_logger('serial_node').error('Tried to publish before configured, topic id ' + str(topic_id))
                        self.requestTopics()
                    time.sleep(0.001)
                else:
                    rclpy.logging.get_logger('serial_node').info('wrong checksum for topic id and msg')

            except IOError as exc:
                rclpy.logging.get_logger('serial_node').warn('Last read step: ' + str(read_step))
                rclpy.logging.get_logger('serial_node').warn('Run loop error: ' + str(exc))
                # One of the read calls had an issue. Just to be safe, request that the client
                # reinitialize their topics.
                with self.read_lock:
                    self.port.flushInput()
                with self.write_lock:
                    self.port.flushOutput()
                self.requestTopics()
        self.txStopRequest()
        self.write_thread.join()

    def setPublishSize(self, size):
        if self.buffer_out < 0:
            self.buffer_out = size
            rclpy.logging.get_logger('serial_node').info('Note: publish buffer size is ' + self.buffer_out + ' bytes')

    def setSubscribeSize(self, size):
        if self.buffer_in < 0:
            self.buffer_in = size
            rclpy.logging.get_logger('serial_node').info('Note: subscribe buffer size is ' + self.buffer_in + ' bytes')

    def setupPublisher(self, data):
        """ Register a new publisher. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            pub = Publisher(msg)
            self.publishers[msg.topic_id] = pub
            self.callbacks[msg.topic_id] = pub.handlePacket
            self.setPublishSize(msg.buffer_size)
            rclpy.logging.get_logger('serial_node').info('Setup publisher on ' + str(msg.topic_name) + ' [' + str(msg.message_type) + ']')
        except Exception as e:
            rclpy.logging.get_logger('serial_node').error('Creation of publisher failed: ' + str(e))

    def setupSubscriber(self, data):
        """ Register a new subscriber. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            if not msg.topic_name in list(self.subscribers.keys()):
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                rclpy.logging.get_logger('serial_node').info('Setup subscriber on ' + str(msg.topic_name) + ' [' + str(msg.message_type) + ']')
            elif msg.message_type != self.subscribers[msg.topic_name].message._type:
                old_message_type = self.subscribers[msg.topic_name].message._type
                self.subscribers[msg.topic_name].unregister()
                sub = Subscriber(msg, self)
                self.subscribers[msg.topic_name] = sub
                self.setSubscribeSize(msg.buffer_size)
                rclpy.logging.get_logger('serial_node').info('Change the message type of subscriber on ' + str(msg.topic_name) + ' from [' + str(old_message_type) + '] to [' + str(msg.message_type) + ']')
        except Exception as e:
            rclpy.logging.get_logger('serial_node').error('Creation of subscriber failed: ' + str(e))

    def setupServiceServerPublisher(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                rclpy.logging.get_logger('serial_node').info('Setup service server on ' + str(msg.topic_name) + ' [' + str(msg.message_type) + ']' )
                self.services[msg.topic_name] = srv
            if srv.mres._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rclpy.logging.get_logger('serial_node').error('Creation of service server failed: ' + str(e))

    def setupServiceServerSubscriber(self, data):
        """ Register a new service server. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceServer(msg, self)
                rclpy.logging.get_logger('serial_node').info('Setup service server on ' + str(msg.topic_name) + ' [' + str(msg.message_type) + ']')
                self.services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rclpy.logging.get_logger('serial_node').error('Creation of service server failed: ' + str(e))

    def setupServiceClientPublisher(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setPublishSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                rclpy.logging.get_logger('serial_node').info('Setup service client on ' + str(msg.topic_name) + ' [' + str(msg.message_type) + ']')
                self.services[msg.topic_name] = srv
            if srv.mreq._md5sum == msg.md5sum:
                self.callbacks[msg.topic_id] = srv.handlePacket
            else:
                raise Exception('Checksum does not match: ' + srv.mreq._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rclpy.logging.get_logger('serial_node').error('Creation of service client failed: ' + str(e))

    def setupServiceClientSubscriber(self, data):
        """ Register a new service client. """
        try:
            msg = TopicInfo()
            msg.deserialize(data)
            self.setSubscribeSize(msg.buffer_size)
            try:
                srv = self.services[msg.topic_name]
            except KeyError:
                srv = ServiceClient(msg, self)
                rclpy.logging.get_logger('serial_node').info('Setup service client on ' + str(msg.topic_name) + '[' + str(msg.message_type) + ']')
                self.services[msg.topic_name] = srv
            if srv.mres._md5sum == msg.md5sum:
                srv.id = msg.topic_id
            else:
                raise Exception('Checksum does not match: ' + srv.mres._md5sum + ',' + msg.md5sum)
        except Exception as e:
            rclpy.logging.get_logger('serial_node').error('Creation of service client failed: ' + str(e))

    def handleTimeRequest(self, data):
        """ Respond to device with system time. """
        t = Time()
        data_buffer = io.BytesIO()
        t.serialize(data_buffer)
        self.send( TopicInfo.ID_TIME, data_buffer.getvalue() )
        self.lastsync = Time()

    def handleParameterRequest(self, data):
        """ Send parameters to device. Supports only simple datatypes and arrays of such. """
        req = RequestParamRequest()
        req.deserialize(data)
        resp = RequestParamResponse()
        try:
            param = rclpy.get_param(req.name)
        except KeyError:
            rclpy.logging.get_logger('serial_node').error('Parameter ' + str(req.name) + ' does not exist')
            return

        if param is None:
            rclpy.logging.get_logger('serial_node').error('Parameter ' + str(req.name) + ' does not exist')
            return

        if isinstance(param, dict):
            rclpy.logging.get_logger('serial_node').error('Cannot send param ' + str(req.name) + ' because it is a dictionary')
            return
        if not isinstance(param, list):
            param = [param]
        #check to make sure that all parameters in list are same type
        t = type(param[0])
        for p in param:
            if t!= type(p):
                rclpy.logging.get_logger('serial_node').error('All Paramers in the list ' + str(req.name) + ' must be of the same type')
                return
        if t == int or t == bool:
            resp.ints = param
        if t == float:
            resp.floats =param
        if t == str:
            resp.strings = param
        data_buffer = io.BytesIO()
        resp.serialize(data_buffer)
        self.send(TopicInfo.ID_PARAMETER_REQUEST, data_buffer.getvalue())

    def handleLoggingRequest(self, data):
        """ Forward logging information from serial device into ROS. """
        msg = Log()
        msg.deserialize(data)
        if msg.level == Log.ROSDEBUG:
            rclpy.logging.get_logger('serial_node').debug(str(msg.msg))
        elif msg.level == Log.INFO:
            rclpy.logging.get_logger('serial_node').info(str(msg.msg))
        elif msg.level == Log.WARN:
            rclpy.logging.get_logger('serial_node').warn(str(msg.msg))
        elif msg.level == Log.ERROR:
            rclpy.logging.get_logger('serial_node').error(str(msg.msg))
        elif msg.level == Log.FATAL:
            rclpy.logging.get_logger('serial_node').fatal(str(msg.msg))

    def send(self, topic, msg):
        """
        Queues data to be written to the serial port.
        """
        self.write_queue.put((topic, msg))

    def _write(self, data):
        """
        Writes raw data over the serial port. Assumes the data is formatting as a packet. http://wiki.ros.org/rosserial/Overview/Protocol
        """
        with self.write_lock:
            self.port.write(data)
            self.last_write = Time()

    def _send(self, topic, msg_bytes):
        """
        Send a message on a particular topic to the device.
        """
        length = len(msg_bytes)
        if self.buffer_in > 0 and length > self.buffer_in:
            rclpy.logging.get_logger('serial_node').error('Message from ROS network dropped: message larger than buffer.\n' + str(msg))
            return -1
        else:
            # frame : header (1b) + version (1b) + msg_len(2b) + msg_len_chk(1b) + topic_id(2b) + msg(nb) + msg_topic_id_chk(1b)
            length_bytes = struct.pack('<h', length)
            length_checksum = 255 - (sum(array.array('B', length_bytes)) % 256)
            length_checksum_bytes = struct.pack('B', length_checksum)

            topic_bytes = struct.pack('<h', topic)
            msg_checksum = 255 - (sum(array.array('B', topic_bytes + msg_bytes)) % 256)
            msg_checksum_bytes = struct.pack('B', msg_checksum)

            self._write(self.header + self.protocol_ver + length_bytes + length_checksum_bytes + topic_bytes + msg_bytes + msg_checksum_bytes)
            return length

    def processWriteQueue(self):
        """
        Main loop for the thread that processes outgoing data to write to the serial port.
        """
        while rclpy.ok():
            if self.write_queue.empty():
                time.sleep(0.01)
            else:
                data = self.write_queue.get()
                while True:
                    try:
                        if isinstance(data, tuple):
                            topic, msg = data
                            self._send(topic, msg)
                        elif isinstance(data, bytes):
                            self._write(data)
                        else:
                            rclpy.logging.get_logger('serial_node').error('Trying to write invalid data type: ' + type(data))
                        break
                    except SerialTimeoutException as exc:
                        rclpy.logging.get_logger('serial_node').error('Write timeout: ' + str(exc))
                        time.sleep(1)
                    except RuntimeError as exc:
                        rclpy.logging.get_logger('serial_node').error('Write thread exception: ' + str(exc))
                        break


    def sendDiagnostics(self, level, msg_text):
        msg = diagnostic_msgs.msg.DiagnosticArray()
        status = diagnostic_msgs.msg.DiagnosticStatus()
        status.name = "rosserial_python"
        msg.header.stamp = Time().to_msg()
        msg.status.append(status)

        status.message = msg_text
        status.level = level

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[0].key="last sync"
        if self.lastsync.to_sec()>0:
            status.values[0].value=time.ctime(self.lastsync.to_sec())
        else:
            status.values[0].value="never"

        status.values.append(diagnostic_msgs.msg.KeyValue())
        status.values[1].key="last sync lost"
        status.values[1].value=time.ctime(self.lastsync_lost.to_sec())

        self.pub_diagnostics.publish(msg)
