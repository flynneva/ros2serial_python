import asyncio

# https://tinkering.xyz/async-serial/
class Writer(asyncio.Protocol):

    def __init__(self, q):
        # Store the que
        super().__init__()
        self.transport = None
        self.buf = None
        self.msgs_sent = None
        self.q = q

        # init generator
        self.send(None)

    def connection_made(self, transport):
        """Store the serial transport and schedule the task to send data.
        """
        self.transport = transport
        self.buf = bytes()
        self.msgs_sent = 0
        print('Writer connection created')

    def connection_lost(self, exc):
        print('Writer closed')

    async def send(self, data):
        # Send data
        # TODO(flynneva): do some checking here to see if its just bytes
        # or multiple lines with \n
        
        # write data to serial
        self.transport.serial.write(bytes([b]))
        # increase msgs sent counter
        # TODO(flynneva): might be good to somehow publish this counter to ROS too?
        self.msgs_sent += 1
        # add sent data to write queue to publish on ROS
        asyncio.ensure_future(self.q.put(data))
        # print(f'Writer sent: {bytes([b])}')
        self.transport.close()
