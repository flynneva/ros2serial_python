import asyncio

# https://tinkering.xyz/async-serial/
class Reader(asyncio.Protocol):
    def __init__(self, q):
        # Store the queue.
        super().__init__()
        self.transport = None
        self.buf = None
        self.msgs_recvd = None
        self.q = q

    def connection_made(self, transport):
        # Store the serial transport and prepare to receive data.
        self.transport = transport
        self.buf = bytes()
        self.msgs_recvd = 0
        print('Reader connection created')

    def data_received(self, data):
        # Store characters until a newline is received.
        self.buf += data
        num_lines = data.count(b'\n')
        if b'\n' in self.buf:
            lines = self.buf.split(b'\n')
            self.buf = lines[-1]  # whatever was left over
            for line in lines[:-1]:
                asyncio.ensure_future(self.q.put(line))
                self.msgs_recvd += 1
        if self.msgs_recvd == num_lines:
            self.transport.close()

    def connection_lost(self, exc):
        print('Reader closed')
