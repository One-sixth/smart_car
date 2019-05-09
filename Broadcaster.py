import socket

class Broadcaster:
    network = '<broadcast>'
    port = 12357

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', self.port))

    def send(self, data: bytes, port=None):
        if port != None and port != self.port:
            self.port = port
            self.sock.bind(('', self.port))
        self.sock.sendto(data, (self.network, self.port))

    def recv(self, port=None):
        if port != None and port != self.port:
            self.sock.bind(('', self.port))
        data, address = self.sock.recvfrom(65535)
        return data, address


if __name__ == '__main__':
    import time
    bc = Broadcaster()
    while True:
        bc.send('Hello'.encode('utf8'))
        time.sleep(1)
        data, addr = bc.recv()
        print(data.decode('utf8'), addr)
        time.sleep(0.5)
