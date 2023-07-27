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
from dataclasses import dataclass, field, replace
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
        unpack_str = 3*f"{ENCODER_COUNTER_SIZE}I"
        x = struct.unpack(unpack_str, data)
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
        x = struct.unpack("III", data)
        clock = x[0] + (x[1] << 32)
        return cls(clock=clock, state=x[2])


@dataclass
class ErrorPacket:
    error_code: int
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, data):
        """Unpack packet from UDP data"""
        x = struct.unpack("I", data)
        return cls(error_code=x[0])


@dataclass
class TimeoutPacket:
    timeout_type: int
    timestamp: float = field(default_factory=time.time)

    @classmethod
    def from_data(cls, data):
        """Unpack packet from UDP data"""
        x = struct.unpack("I", data)
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
    header = struct.unpack("H", data[:2])
    if header not in packet_headers:
        raise RuntimeError(f"Bad header: {header}")

    packet_type = packet_headers[header]
    return packet_type.from_data(data[2:])


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
    calibrated: bool = False

    def __post_init__(self):
        self.limits = {
            'cold_grip': LimitState(pru_bit=self.limit_pru_bits[0]),
            'warm_grip': LimitState(pru_bit=self.limit_pru_bits[1]),
        }


@dataclass
class GripperState:
    actuators: Tuple[ActuatorState, ActuatorState, ActuatorState] = (
        ActuatorState(axis=1, limit_pru_bits=(8, 9)),
        ActuatorState(axis=2, limit_pru_bits=(10, 11)),
        ActuatorState(axis=3, limit_pru_bits=(12, 13)),
    )

    last_packet_received: float = 0.0
    last_limit_received: float = 0.0
    last_encoder_received: float = 0.0

    _expiration_time: float = 10.0
    _last_enc_state: Optional[int] = None

    @property
    def expired(self):
        """Returns True if the gripper state has not been updated in a while"""
        return time.time() - self.last_packet_received > self._expiration_time

    def update(self, packet):
        """Updates the gripper state based on an incoming PRU packet"""
        self.last_packet_received = packet.timestamp

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

        else:
            raise RuntimeError(f"Unknown packet type: {packet}")

    def set_home(self):
        for act in self.actuators:
            act.pos = 0
            act.calibrated = True


class PruMonitor:
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

    def __init__(self, port, ip_address=''):
        self.port = port
        self.ip_address = ip_address

        self._gripper_state = GripperState()
        self._state_lock = threading.Lock()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.ip_address, self.port))
        self.packet_queue = queue.Queue()

        self.read_thread = threading.Thread(target=self._read_packets)
        self.update_thread = threading.Thread(target=self._update_state)

    def start(self):
        self.read_thread.start()
        self.update_thread.start()

    def _read_packets(self):
        while True:
            data, _ = self.socket.recvfrom(1024)
            self.packet_queue.put((parse_packet(data)))

    def _update_state(self):
        while True:
            packet = self.packet_queue.get()
            with self._state_lock:
                self._gripper_state.update(packet)

    def set_home(self):
        """
        Sets the current position as the home position
        """
        with self._state_lock:
            self._gripper_state.set_home()

    def get_state(self):
        """
        Gets the current gripper state
        """
        with self._state_lock:
            # Return a copy of the state object
            return replace(self._gripper_state)
