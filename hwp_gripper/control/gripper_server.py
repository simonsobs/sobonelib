import socket
import os
import sys
import ctypes
import multiprocessing
import time
import pickle as pkl
import numpy as np
from pru_monitor import PruMonitor
from dataclasses import asdict

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

        self.pru_monitor = PruMonitor(self.pru_port)
        self.pru_monitor.start()

        self.collector = col.GripperCollector(self.pru_port)
        self._run_collect_pru = multiprocessing.Value(ctypes.c_bool, False)
        self._stopped = multiprocessing.Value(ctypes.c_bool, False)

        self.last_encoder = 0
        self.last_limit = 0
        self.last_limit_time = 0
        self.limit_pos = [13.,10.,13.,10.,13.,10.]
        self.is_forced = False

        # Variable to tell the code whether the cryostat is warm (False) or cold (True). Needs to be
        # given by the user
        self.is_cold = multiprocessing.Value(ctypes.c_bool, False)
        
        # Variable to tell the code whether it should ignore any flags sent by the limit switches. If
        # this variable is False the actuators will only move while none of the active limit switches
        # are triggered (which limit switches are chosen depends on self.is_cold)
        self.force = multiprocessing.Value(ctypes.c_bool, False)

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
                elif cmd == 'GET_STATE':
                    return_dict = self.get_state()
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

        actuator_idx = int(args[2]) - 1
        state = self.pru_monitor.get_state()
        act_state = state.actuator[actuator_idx]
        cur_pos = act_state.pos

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

        return return_dict

    def home(self):
        return_dict = self.CMD.CMD('HOME')

        ## ADD CHECK HERE TO MAKE SURE JXC SETUP PIN TOGGLES!
        self.pru_monitor.set_home()

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
    
    def get_state(self):
        state = self.pru_monitor.get_state()
        return {'result': asdict(state)}


    def monitor_limit_state(self):

        prev_state = self.pru_monitor.get_state()
        while True:
            time.sleep(0.2)
            # Collect data packets
            state = self.pru_monitor.get_state()
            for i, act in enumerate(state.actuators):
                prev_act = prev_state.actuators[i]

                # States haven't changed, don't do anything
                if (act.cold_grip.state == prev_act.cold_grip.state \
                    and act.warm_grip.state == prev_act.warm_grip.state):
                    continue

                # Else, we may need to take EMG action
                print(f"Limit switch activation for axis {act.axis} at time: {time.time()}")
                if act.limits['cold_grip'].state:
                    print("Cold grip limit triggered, turning EMG Off")
                    self.CMD.CMD(f'EMG OFF {act.axis}')
                elif act.limits['warm_grip'].state and (not self.is_cold.value):
                    print("Warm grip limit triggered while warm, turning EMG Off")
                    self.CMD.CMD(f'EMG OFF {act.axis}')
                else:
                    print("Limits ok, turning EMG on")
                    self.CMD.CMD(f'EMG ON {act.axis}')

            prev_state = state


if __name__ == '__main__':
    server = GripperServer(8040, 8041)
    print('Starting Server')
    while True:
        server.process_command()