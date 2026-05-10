"""Thin SocketCAN wrapper.

Same interface as copley_bridge.raw_socket_can / epos2_bridge so the
Ingenia bridge can share the low-level CAN path without taking a
runtime dependency on a sibling bridge package.
"""

from __future__ import annotations

import socket
import struct
import threading
from typing import Optional, Tuple


class RawSocketCAN:
    def __init__(self, channel: str) -> None:
        self.channel = channel
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((channel,))
        self.sock.settimeout(0.02)
        self.lock = threading.Lock()

    def send(self, cob_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("CAN payload must be <= 8 bytes")
        frame = struct.pack("=IB3x8s", cob_id, len(data), data.ljust(8, b"\x00"))
        with self.lock:
            self.sock.send(frame)

    def recv(self) -> Optional[Tuple[int, bytes]]:
        try:
            raw = self.sock.recv(16)
        except (TimeoutError, socket.timeout):
            return None
        can_id, dlc, data = struct.unpack("=IB3x8s", raw)
        return can_id, data[:dlc]

    def close(self) -> None:
        self.sock.close()
