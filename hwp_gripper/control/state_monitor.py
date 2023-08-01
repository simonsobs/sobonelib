"""

The Beaglebone code periodically queries the status of six pins measuring the
encoder signal and sends that data to this process in the form of UDP packets.
Each actuator has two encoder signals

Similarly the Beaglebone code also periodically queries the status of six pins
hooked up to the warm and cold limit switches on each actuator and sends that
data to the agent as separate UDP packets

Names of the encoder chains (currently the code does not use these, but it's still a
good reference to know which index corresponds to which chain)
self.encoder_names = ['Actuator 1 A', 'Actuator 1 B', 'Actuator 2 A',
                      'Actuator 2 B', 'Actuator 3 A', 'Actuator 3 B']

Names of the limit chains
self.limit_names = ['Actuator 1 Cold', 'Actuator 1 Warm', 'Actuator 2 Cold',
                    'Actuator 2 Warm', 'Actuator 3 Cold', 'Actuator 3 Warm']

"""
from dataclasses import dataclass, field
from copy import deepcopy
import numpy as np
import socket
import struct
import threading
import time
from typing import Optional, Tuple, Dict
import queue


ENCODER_COUNTER_SIZE = 120
MM_PER_NOTCH = 1./160


@dataclass
class JXCPacket:
    """Packet that contains update to JXC pins"""
    setup: bool = False
    svon: bool = False
    busy: bool = False
    seton: bool = False
    inp: bool = False
    svre: bool = False
    alarm: bool = False
    brake: Tuple[bool, bool, bool] = (False, False, False)
    emg: Tuple[bool, bool, bool] = (False, False, False)
    out: int = 0
    timestamp: float = field(default_factory=time.time)


@dataclass
class EncoderPacket:
    """
    Attributes
    ------------
    clock: np.ndarray
        Encoder clock values
    state: np.ndarray
        Array of encoder states. Each state is an unsigned long, where the
        2*i and (2*i + 1) bits are the state of the i-th quad encoders.
    """
    clock: np.ndarray
    state: np.ndarray
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, data):
        """Unpack packet from UDP data"""
        unpack_str = 3*ENCODER_COUNTER_SIZE*"L"
        x = np.array(struct.unpack(unpack_str, data))
        clock = x[:ENCODER_COUNTER_SIZE]
        # adds in the overflow bits
        clock += x[ENCODER_COUNTER_SIZE: 2*ENCODER_COUNTER_SIZE] << 32

        state_ints = x[2*ENCODER_COUNTER_SIZE: 3*ENCODER_COUNTER_SIZE]
        return cls(clock=clock, state=state_ints)


@dataclass
class LimitPacket:
    clock: int
    state: int
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, data):
        """Unpack packet from UDP data"""
        x = np.array(struct.unpack("LLL", data))
        clock = x[0] + (x[1] << 32)
        return cls(clock=clock, state=x[2])


@dataclass
class ErrorPacket:
    error_code: int
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, data):
        """Unpack packet from UDP data"""
        x = np.array(struct.unpack("L", data))
        return cls(error_code=x[0])


@dataclass
class TimeoutPacket:
    timeout_type: int
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, data):
        """Unpack packet from UDP data"""
        x = np.array(struct.unpack("L", data))
        return cls(timeout_type=x[0])


@dataclass
class PulsePacket:
    """Packet that is regularly sent to keep connection alive"""
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, _):
        """Unpack packet from UDP data"""
        return cls()


packet_headers = {
    0xBAD0: EncoderPacket,
    0xF00D: LimitPacket,
    0xE12A: ErrorPacket,
    0x1234: TimeoutPacket,
    0x8888: PulsePacket,
}


def parse_packet(data):
    """
    Parses a PRU packet sent from the beaglebone's PRU process

    Parameters
    ---------------
    data: bytes
        UDP packet data
    """
    header = struct.unpack("L", data[:4])[0]
    if header not in packet_headers:
        raise RuntimeError(f"Bad header: {header}")
    
    packet_type = packet_headers[header]
    return packet_type.from_data(data[4:])


########################################
# Gripper state definition
########################################

@dataclass
class LimitState:
    pru_bit: Optional[int] = None
    state: bool = False


@dataclass
class ActuatorState:
    axis: int
    limit_pru_bits: Tuple[int, int]

    limits: Optional[Dict[str, LimitState]] = None

    pos: float = 0

    brake: bool = True
    emg: bool = False

    def __post_init__(self):
        self.limits = {
            'cold_grip': LimitState(pru_bit=self.limit_pru_bits[0]),
            'warm_grip': LimitState(pru_bit=self.limit_pru_bits[1]),
        }

@dataclass
class JXCState:
    setup: bool = False
    svon: bool = False
    busy: bool = False
    seton: bool = False
    inp: bool = False
    svre: bool = False
    alarm: bool = False
    out: int = 0

    @property
    def status(self):
        """
        Current status of the controller determined from JXC input pins.
        Described on page 43 of the JXC manual: 
            https://www.smcworld.com/assets/manual/en-jp/files/SFOD-OMT0010.pdf
        """

        # Bit string that we can use to easily check current state
        status_bits = [self.busy, self.inp, self.svre, self.seton]
        bit_rep = "".join([str(s) for s in status_bits])

        if self.out == 0:
            if bit_rep == "0000":
                return 'powered_down_servo_off'
            elif bit_rep == "0010":
                return 'powered_down_servo_on'
            elif bit_rep == "0100":
                return 'return_to_origin'
            elif bit_rep == "0111":
                return 'home'
        else:
            # This contains a number of different cases, but from the docs they
            # don't seem to be 1-to-1 to bit_rep
            return f'output_step_{self.out}'

@dataclass
class GripperState:
    actuators: Tuple[ActuatorState, ActuatorState, ActuatorState] = (
        ActuatorState(axis=1, limit_pru_bits=(8, 9)),
        ActuatorState(axis=2, limit_pru_bits=(10, 11)),
        ActuatorState(axis=3, limit_pru_bits=(12, 13)),
    )
    jxc = JXCState()

    last_packet_received: float = 0.0
    last_limit_received: float = 0.0
    last_encoder_received: float = 0.0

    calibrated: bool = False
    calibrated_at: float = 0

    _expiration_time: float = 10.0
    _last_enc_state: Optional[int] = None
    _left_home: bool = False

    @property
    def expired(self):
        """Returns True if the gripper state has not been updated in a while"""
        return time.time() - self.last_packet_received > self._expiration_time

    def update(self, packet):
        """Updates the gripper state based on an incoming PRU packet"""
        self.last_packet_received = packet.timestamp

        if isinstance(packet, JXCPacket):
            self.jxc.setup = packet.setup
            self.jxc.svon = packet.svon
            self.jxc.busy = packet.busy
            self.jxc.seton = packet.seton
            self.jxc.inp = packet.inp
            self.jxc.svre = packet.svre
            self.jxc.alarm = packet.alarm
            for i, act in self.actuators:
                act.brake = packet.brake[i]
                act.emg = packet.emg[i]
            
            if self._left_home:
                if self.jxc.status == 'home':
                    for act in self.actuators:
                        act.pos = 0
                    self.calibrated = True
                    self.calibrated_at = time.time()
                    self._left_home = False

            if self.jxc.out != 0:
                self._left_home = True
            
        if isinstance(packet, EncoderPacket):
            # Update the position of each encoder
            for act in self.actuators:
                idx = act.axis - 1
                a_state = (packet.state >> 2*idx) & 1
                b_state = (packet.state >> (2*idx + 1)) & 1

                if self._last_enc_state is not None:
                    rising_edges = np.diff(
                        a_state, prepend=(self._last_enc_state >> 2*idx) & 1
                    ) == 1
                    dirs = (b_state[rising_edges] * 2 - 1)
                else:
                    rising_edges = np.diff(a_state) == 1
                    dirs = (b_state[1:][rising_edges] * 2 - 1)

                act.pos += np.sum(dirs) * MM_PER_NOTCH

            self._last_enc_state = packet.state[-1]

        elif isinstance(packet, LimitPacket):
            self.last_limit_received = packet.timestamp

            # Update the limit state of each actuator
            for act in self.actuators:
                for limit in act.limits.values():
                    limit.state = bool((packet.state >> limit.pru_bit) & 1)

        elif isinstance(packet, ErrorPacket):
            # Should we do anything else for errors?
            print(f"Error packet received!! Error code: {packet.error_code}")

        elif isinstance(packet, TimeoutPacket):
            # Should we do anything else for timeouts?
            print(
                f"Timeout packet received!! timeout type: {packet.timeout_type}")
        elif isinstance(packet, PulsePacket):
            pass
        else:
            raise RuntimeError(f"Unknown packet type: {packet}")


class StateMonitor:
    """
    This class monitors incoming UDP packets from the PRU, and uses incoming
    data to keep track of the Gripper state.

    Args
    ------
    port : int
        Port to listen for incoming UDP packets
    ip_address : str
        IP Address to listen for incoming UDP packets. Defaults to using all
        available interfaces.
    """

    def __init__(self, port, jxc=None, ip_address='', jxc_sample_time=0.5):
        self.port = port
        self.ip_address = ip_address

        self._gripper_state = GripperState()
        self._state_lock = threading.Lock()
        self.jxc = jxc
        self.jxc_sample_time = jxc_sample_time

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.ip_address, self.port))
        self.packet_queue = queue.Queue()

        self.monitor_pru_thread = threading.Thread(target=self._monitor_pru, daemon=True)
        self.update_thread = threading.Thread(target=self._update_state, daemon=True)
        self.monitor_jxc_thread = threading.Thread(target=self._monitor_jxc, daemon=True)

    def start(self):
        """Start threads"""
        print("Starting state monitor threads")
        self.update_thread.start()
        self.monitor_pru_thread.start()
        self.monitor_jxc_thread.start()

    def _monitor_pru(self):
        while True:
            data, _ = self.socket.recvfrom(2048)
            self.packet_queue.put((parse_packet(data)))

    def _update_state(self):
        while True:
            packet = self.packet_queue.get()
            with self._state_lock:
                self._gripper_state.update(packet)

    def _monitor_jxc(self):
        if self.jxc is None:
            return

        jxc_mapping = {
            'setup': self.jxc.SETUP,
            'svon': self.jxc.SVON,
            'busy': self.jxc.BUSY,
            'seton': self.jxc.SETON,
            'inp': self.jxc.INP,
            'svre': self.jxc.SVRE,
            'alarm': self.jxc.ALARM,
            'brake': [self.jxc.BRAKE1, self.jxc.BRAKE2, self.jxc.BRAKE3],
            'emg': [self.jxc.EMG1, self.jxc.EMG2, self.jxc.EMG3],
            'out': [getattr(self.jxc, f'OUT{i}') for i in range(1, 9)],
        }

        while True:
            d = {}
            for k, v in jxc_mapping.items():
                if k in ['brake', 'emg']:
                    d[k] = (self.jxc.read(vv) for vv in v)
                elif k == 'out':
                    bin_arr = [self.jxc.read(vv) for vv in v]
                    # Converts a list of 1s and 0s representing a bin number
                    # to an int
                    d[k] = int(map(str, bin_arr), 2) 
                else:
                    d[k] = self.jxc.read(v)
            
            self.packet_queue.put(JXCPacket(**d))
            time.sleep(self.jxc_sample_time)

    def get_state(self):
        """
        Gets the current gripper state
        """
        with self._state_lock:
            # Return a copy of the state object
            return deepcopy(self._gripper_state)
