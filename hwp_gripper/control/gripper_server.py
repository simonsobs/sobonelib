import socket
import os
import sys
import pickle as pkl

this_dir = os.path.dirname(__file__)
sys.path.append(this_dir)

import BBB as bb
import JXC831 as jx
import control as ct
import gripper as gp
import command_gripper as cg

class GripperServer(object):
    def __init__(self, port):
        self.port = port

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('', self.port))
        self.s.listen()

        self.data = b''

        self.BBB = bb.BBB()
        self.JXC = jx.JXC831(self.BBB)
        self.CTL = ct.Control(self.JXC)
        self.GPR = gp.Gripper(self.CTL)
        self.CMD = cg.Command(self.GPR)

    def process_command(self):
        conn, addr = self.s.accept()
        with conn:
            print(f'Connection from: {addr}')
            while True:
                self.data = conn.recv(1024)
                if not self.data:
                    print(f'Connection ended: {addr}')
                    break

                command = self.data.decode(encoding = 'UTF-8')
                return_dict = self.CMD.CMD(command)
                conn.sendall(pkl.dumps(return_dict))
                self.data = b''

    def __exit__(self):
        self.s.close()

server = GripperServer(8041)
print('Starting Server')
while True:
    server.process_command()
