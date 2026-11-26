"""Local, read-only streams shared by the runner and the HTTP diagnostics service."""

import os
import pwd
import socket
import struct
from threading import Condition, Thread

TELEMETRY_SOCKET = "\0fb4-diagnostics-telemetry"
CAMERA_SOCKET = "\0fb4-diagnostics-camera"
KEEPALIVE = b"\0"
MAX_PACKET_SIZE = 256 * 1024


class DiagnosticStream:
    """Publish the latest sample without waiting on consumers or buffering old samples."""

    def __init__(self, address, encode, on_active_changed=None):
        self._encode = encode
        self._on_active_changed = on_active_changed
        self._changed = Condition()
        self._peers = 0
        self._sequence = 0
        self._latest = None
        self._allowed_uids = {os.geteuid()}
        try:
            self._allowed_uids.add(pwd.getpwnam("fb4-diagnostics").pw_uid)
        except KeyError:
            pass

        self._listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            self._listener.bind(address)
            self._listener.listen(8)
        except OSError:
            self._listener.close()
            raise
        Thread(target=self._accept, name="diagnostics-listener", daemon=True).start()

    @property
    def active(self):
        return self._peers > 0

    def publish(self, sample):
        with self._changed:
            if self._peers:
                self._latest = sample
                self._sequence += 1
                self._changed.notify_all()

    def _accept(self):
        while True:
            connection, _ = self._listener.accept()
            credentials = connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
            _, uid, _ = struct.unpack("=3i", credentials)
            if uid not in self._allowed_uids:
                connection.close()
                continue
            Thread(target=self._send, args=(connection,), daemon=True).start()

    def _send(self, connection):
        with connection:
            connection.settimeout(2)
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, MAX_PACKET_SIZE)
            with self._changed:
                self._peers += 1
                if self._peers == 1 and self._on_active_changed is not None:
                    self._on_active_changed(True)
                sequence = self._sequence - int(self._latest is not None)

            try:
                while True:
                    with self._changed:
                        changed = self._changed.wait_for(lambda: self._sequence != sequence, timeout=1)
                        sequence = self._sequence
                        sample = self._latest if changed else None
                    connection.send(KEEPALIVE if sample is None else self._encode(sample))
            except OSError:
                pass
            finally:
                with self._changed:
                    self._peers -= 1
                    if self._peers == 0:
                        self._latest = None
                        if self._on_active_changed is not None:
                            self._on_active_changed(False)
