import numpy as np
import asyncio
import serial_asyncio
from threading import Thread
from functools import partial

from ros2serial_python.async_reader import Reader
from ros2serial_python.async_writer import Writer
from ros2serial_msgs.msg import Forward
from ros2serial_msgs.srv import Send

import rclpy
from rclpy.node import Node


class serial_forwarding(Node):


    def __init__(self, node_name):
        super().__init__(node_name)
        self.name = node_name
        self.get_logger().info('Initializing ' + node_name +' publishers and subscribers')
        self.pub_recv_ = self.create_publisher(Forward, node_name + '/received', 10)
        self.pub_sent_ = self.create_publisher(Forward, node_name + '/sent', 10)
        self.srv_send_ = self.create_service(Send, node_name + '/send', self.send_callback)
        
        self.get_logger().info('Declaring ROS parameters')
        self.declare_parameters(
                namespace='',
                parameters=[ ('port', '/dev/ttyUSB0'),
                             ('baudrate', 115200)])

        self.get_logger().info('Setting ROS paramters')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
       
        # init queue so they can be published
        self.read_queue = asyncio.Queue()
        self.write_queue = asyncio.Queue()
        
        reader_partial = partial(Reader, self.read_queue)
        write_partial = partial(Writer, self.write_queue)

        self.loop = asyncio.get_event_loop()
        
        self.reader = serial_asyncio.create_serial_connection(self.loop,
                                                         reader_partial,
                                                         self.port,
                                                         baudrate=self.baudrate)
        self.writer = serial_asyncio.create_serial_connection(self.loop,
                                                         write_partial,
                                                         self.port,
                                                         baudrate=self.baudrate)
        
        asyncio.ensure_future(self.reader)
        asyncio.ensure_future(self.writer)
        
        asyncio.ensure_future(self.pub_received(self.loop, self.read_queue))
        asyncio.ensure_future(self.pub_sent(self.loop, self.write_queue))

        thread = Thread(target=self.start_background_loop, args=(self.loop, ), daemon=True)
        thread.start()


    async def pub_received(self, loop, q):
        counter = 0
        while rclpy.ok():
            print('waiting for received queue')
            msg = await q.get()
            print(f'{msg}')
            self.message = Forward()
            self.message.header.stamp = self.get_clock().now().to_msg()
            self.message.header.frame_id = self.name
            self.message.data = str(msg)
            self.pub_recv_.publish(self.message)
            counter += 1
        loop.stop()

    
    async def pub_sent(self, loop, q):
        counter = 0
        while rclpy.ok():
            print('waiting for sent queue')
            msg = await q.get()
            print(f'{msg}')
            self.message = Forward()
            self.message.header.stamp = self.get_clock().now().to_msg()
            self.message.header.frame_id = self.name
            self.message.data = str(msg)
            self.pub_sent_.publish(self.message)
            counter += 1
        loop.stop()

    async def send_callback(self, request, response):
        self.get_logger().info('Sending ' + request.data)
        try:
            # add data to write queue to publish on ROS
            asyncio.ensure_future(self.write_queue.put(request.data))
        except StopIteration:
            print('STOP ITERATION')
            pass

        #self.writer.send(request.data)
        response.status = 0
        return response

    def start_background_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        asyncio.set_event_loop(loop)
        loop.run_forever()

def main():
    rclpy.init()
    node = serial_forwarding('serial_node')
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
