import socket
import os
import sys
import ctypes
import multiprocessing
import time
import pickle as pkl
import numpy as np

this_dir = os.path.dirname(__file__)
sys.path.append(this_dir)

import BBB as bb
import JXC831 as jx
import control as ct
import gripper as gp
import command_gripper as cg

import gripper_collector as col


class GripperServer(object):
    def __init__(self, pru_port, control_port):
        self.pru_port = pru_port
        self.control_port = control_port

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(('', self.control_port))
        self.s.listen()

        self.data = b''

        self.BBB = bb.BBB()
        self.JXC = jx.JXC831(self.BBB)
        self.CTL = ct.Control(self.JXC)
        self.GPR = gp.Gripper(self.CTL)
        self.CMD = cg.Command(self.GPR)

        self.collector = col.GripperCollector(self.pru_port)
        self._run_collect_pru = multiprocessing.Value(ctypes.c_bool, False)
        self._stopped = multiprocessing.Value(ctypes.c_bool, False)

        self.last_encoder = 0
        self.last_limit = 0
        self.last_limit_time = 0
        self.limit_pos = [13.,10.,13.,10.,13.,10.]
        self.is_forced = False

        # The Beaglebone code periodically queries the status of six pins measuring the
        # encoder signal and sends that data to this process in the form of UDP packets.
        # Each actuator has two encoder signals

        # Similarly the Beaglebone code also periodically queries the status of six pins
        # hooked up to the warm and cold limit switches on each actuator and sends that
        # data to the agent as seperate UDP packets

        # Names of the encoder chains (currently the code does not use these, but it's still a
        # good reference to know which index corresponds to which chain)
        # self.encoder_names = ['Actuator 1 A', 'Actuator 1 B', 'Actuator 2 A',
        #                       'Actuator 2 B', 'Actuator 3 A', 'Actuator 3 B']

        # Which bits on Beaglebone PRU1 register are used for the encoder (these values shouldn't change)
        self.encoder_pru = [0, 1, 2, 3, 4, 5]

        # Array which holds how many rising/falling edges have been detected from the encoder signal
        self.encoder_edges = multiprocessing.Array(ctypes.c_int, (0, 0, 0, 0, 0, 0))

        # Array which tells the code whether it should use incomming data to change the number of
        # encoder edges. There are six values, one for each encoder chain
        self.encoder_edges_record = multiprocessing.Array(ctypes.c_int, (0, 0, 0, 0, 0, 0))

        # Array which tells the code what direction the actuator should be moving in. There are six
        # values, one for each encoder chain
        self.encoder_direction = multiprocessing.Array(ctypes.c_int, (1, 1, 1, 1, 1, 1))

        # Names of the limit chains
        self.limit_names = ['Actuator 1 Cold', 'Actuator 1 Warm', 'Actuator 2 Cold',
                            'Actuator 2 Warm', 'Actuator 3 Cold', 'Actuator 3 Warm']
        
        # Which bits on Beaglebone PRU0 register are used for the limit switches (these values shouldn't
        # change)
        self.limit_pru = [8, 9, 10, 11, 12, 13]
        
        # Array which holds the current status of each limit switch
        self.limit_state = [0, 0, 0, 0, 0, 0]

        # Variable to tell the code whether the cryostat is warm (False) or cold (True). Needs to be
        # given by the user
        self.is_cold = multiprocessing.Value(ctypes.c_bool, False)
        
        # Variable to tell the code whether it should ignore any flags sent by the limit switches. If
        # this variable is False the actuators will only move while none of the active limit switches
        # are triggered (which limit switches are chosen depends on self.is_cold)
        self.force = multiprocessing.Value(ctypes.c_bool, False)

        self.collector_process = multiprocessing.Process(target = self.collect_pru)
        self.collector_process.start()

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
                args = command.split(' ')
                cmd = args[0].upper()
                if cmd == 'MOVE':
                    return_dict = self.move(command)
                elif cmd == 'HOME':
                    return_dict = self.home()
                elif cmd == 'LIMIT':
                    return_dict = self.limit()
                elif cmd == 'POSITION':
                    return_dict = self.position()
                elif cmd == 'IS_COLD':
                    return_dict = self.is_cold_func(command)
                elif cmd == 'FORCE':
                    return_dict = self.force_func(command)
                else:
                    return_dict = self.CMD.CMD(command)
                conn.sendall(pkl.dumps(return_dict))
                self.data = b''

    def move(self, command):
        args = command.split(' ')
        log = []
        pru_chains = [2 * int(args[2]) - 2, 2 * int(args[2] -1)]
        with self.encoder_direction.get_lock():
            with self.encoder_edges_record.get_lock():
                for chain in pru_chains:
                    self.encoder_edges_record[chain] = 1
                    if float(args[3]) >= 0:
                        self.encoder_direction[chain] = 1
                    elif float(args[3]) < 0:
                        self.enocder_direction[chain] = -1

        cur_pos = self._get_pos()[2 * int(args[2]) - 2]
        warm_limit_pos = self.limit_pos[2 * int(args[2]) - 1]
        cold_limit_pos = self.limit_pos[2 * int(args[2]) - 2]
        if cur_pos + float(args[3]) > warm_limit_pos and not self.force and not self.is_cold:
            log.append('Move command beyond warm limit switch position')
            dist = (warm_limit_pos - cur_pos) - (warm_limit_pos - cur_pos) % 0.1
            if dist < 0:
                log.append('Actuator already beyond warm limit switch position')
                dist = 0
        elif cur_pos + float(args[3]) > cold_limit_pos and not self.force:
            log.append('Move command beyond cold limit switch position')
            dist = (cold_limit_pos - cur_pos) - (cold_limit_pos - cur_pos) % 0.1
            if dist < 0:
                log.append('Actuator already beyond cold limit switch position')
                dist = 0
        else:
            dist = float(args[3])

        return_dict = self.CMD.CMD(command)
        [log.append(line) for line in return_dict['log']]
        return_dict['log'] = log

        with self.encoder_edges_record.get_lock():
            for chain in pru_chains:
                self.encoder_edges_record[chain] = 0

        return return_dict

    def home(self):
        with self.encoder_edges_record.get_lock():
            for index, _ in enumerate(self.encoder_edges_record):
                self.encoder_edges_record[index] = 1

        return_dict = self.CMD.CMD('HOME')

        with self.encoder_edges.get_lock():
            with self.encoder_edges_record.get_lock():
                for index, _ in enumerate(self.encoder_edges):
                    self.encoder_edges_record[index] = 0
                    self.encoder_edges[index] = 0

        return return_dict

    def is_cold_func(self, command):
        log = []
        args = command.split(' ')
        log.append(f'Received request to change is_cold to {args[1]}')
        with self.is_cold.get_lock():
            self.is_cold.value = bool(args[1])
            log.append('force successfully changed')

        return {'result': True, 'log': log}

    def force_func(self, command):
        log = []
        args = command.split(' ')
        log.append(f'Received request to change force to {args[1]}')
        if bool(args[1]):
            self.is_forced = False
            log.append('force successfully changed')

        with self.force.get_lock():
            self.force.value = bool(args[1])

        return {'result': True, 'log': log}

    def limit(self):
        log = []
        log.append('Queried limit state')
        return {'return': self.limit_state, 'log', log}

    def position(self):
        log = []
        log.append('Queried position state')
        return {'result': self._get_pos(), 'log': log}

    def collect_pru(self):
        with self._run_collect_pru.get_lock():
            self._run_collect_pru.value = True

        while self._run_collect_pru:
            # Collect data packets
            self.collector.relay_gripper_data()

            # Use collected data packets to find changes in gripper positions
            encoder_data = self.collector.process_packets()
            if len(encoder_data['state']):
                edges = np.concatenate(([self.last_encoder ^ encoder_data['state'][0]],
                                        encoder_data['state'][1:] ^ encoder_data['state'][:-1]))

                with self.encoder_edges.get_lock():
                    with self.encoder_direction.get_lock():
                        with self.encoder_edges_record.get_lock():
                            for index, pru in enumerate(self.encoder_pru):
                                if self.encoder_edges_record[index]:
                                    self.encoder_edges[index] += \
                                        self.encoder_direction[index] * np.sum((edges >> pru) & 1)

                self.last_encoder = encoder_data['state'][-1]

            # Check if any of the limit switches have been triggered and prevent gripper movement if necessary
            clock, state = self.collector.limit_state[0], int(self.collector.limit_state[1])

            for index, pru in enumerate(self.limit_pru):
                self.limit_state[index] = ((state & (1 << pru)) >> pru)

            if (state and not self.is_cold.value) or self.limit_state[0] or self.limit_state[2] \
                    or self.limit_state[4]:
                self.last_limit_time = time.time()
                if self.force.value and not self.is_forced:
                    self.CMD.CMD('EMG ON')
                elif self.last_limit != state and not self.force.value:
                    self.last_limit = state

                    if (self.limit_state[1] and not self.is_cold.value) or self.limit_state[0]:
                        self.CMD.CMD('EMG OFF 1')
                    else:
                        self.CMD.CMD('EMG ON 1')

                    if (self.limit_state[3] and not self.is_cold.value) or self.limit_state[2]:
                        self.CMD.CMD('EMG OFF 2')
                    else:
                        self.CMD.CMD('EMG ON 2')

                    if (self.limit_state[5] and not self.is_cold.value) or self.limit_state[4]:
                        self.CMD.CMD('EMG OFF 3')
                    else:
                        self.CMD.CMD('EMG ON 3')

                    print('Limit switch activation at clock: {}'.format(clock))
                    for index, name in enumerate(self.limit_names):
                        if self.limit_state[index]:
                            print('{} activated'.format(name))
            else:
                if time.time() - self.last_limit_time > 5:
                    if self.last_limit:
                        self.CMD.CMD('EMG ON')
                        self.last_limit = 0

        with self._stopped.get_lock():
            stopped.value = True

    def _get_pos(self):
        slope = 1 / 160.
        return [rising_edges * slope for rising_edges in self.encoder_edges]

    def __exit__(self):
        with self._run_collect_pru.get_lock():
            self._run_collect_pru = False

        while not self._stopped.value:
            time.sleep(0.001)

        self.collector_process.terminate()
        self.collector_process.join()
        self.s.close()

server = GripperServer(8040, 8041)
print('Starting Server')
while True:
    server.process_command()
