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


    def connection_made(self, transport):
        """Store the serial transport and schedule the task to send data.
        """
        self.transport = transport
        self.buf = bytes()
        self.msgs_sent = 0

        loop = asyncio.get_event_loop()
        asyncio.ensure_future(self.send(loop, self.q))
        # send carriage return by default at startup
        #asyncio.ensure_future(self.q.put('\n'))
        print('Writer created')


    def connection_lost(self, exc):
        print('Writer closed')

    
    async def send(self, loop, q):
        # loop while queue is not empty
        while self.transport:
            print('waiting for queue')
            data = await q.get()
            print('sending ' + str(data))
            # write data to serial
            if(data != None):
                print('data not none')
                self.transport.serial.write(bytes([data]))
            else:
                print('data none')
                # send carraige return if message to send is blank
                for b in b'\n':
                    self.transport.serial.write(bytes([b]))
            # increase msgs sent counter
            # TODO(flynneva): might be good to somehow publish this counter to ROS too?
            print('increase msgs sent')
            self.msgs_sent += 1
            #print('adding sent to que')
            # add sent data to write queue to publish on ROS
            #asyncio.ensure_future(self.q.put(data))
            print(f'Writer sent: {bytes([b])}')
            self.transport.close()
