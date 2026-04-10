#!/usr/bin/env python3
"""Fan out one MAVLink serial device to two local PTYs.

This lets jMAVSim own one virtual serial port while a control script or QGC
uses the second virtual serial port. Both PTYs share the same underlying
hardware serial device.
"""

from __future__ import annotations

import argparse
import atexit
import errno
import json
import os
import pty
import selectors
import signal
import subprocess
import sys
import termios
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial
from serial import SerialException
from pymavlink import mavutil


DEFAULT_SIM_PATH = Path("/tmp/px4_hitl_sim.pty")
DEFAULT_CTRL_PATH = Path("/tmp/px4_hitl_ctrl.pty")


@dataclass
class PtyEndpoint:
    name: str
    master_fd: int
    slave_fd: int
    slave_path: str
    link_path: Path


def _set_raw(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def make_endpoint(name: str, link_path: Path) -> PtyEndpoint:
    master_fd, slave_fd = pty.openpty()
    _set_raw(master_fd)
    _set_raw(slave_fd)
    os.set_blocking(master_fd, False)
    slave_path = os.ttyname(slave_fd)
    link_path.parent.mkdir(parents=True, exist_ok=True)
    if link_path.exists() or link_path.is_symlink():
        link_path.unlink()
    link_path.symlink_to(slave_path)
    # Do not keep the slave side open in the hub. If we hold it open without a
    # reader, the PTY buffer can fill and stall forwarding before the control
    # client ever attaches.
    os.close(slave_fd)
    return PtyEndpoint(
        name=name,
        master_fd=master_fd,
        slave_fd=-1,
        slave_path=slave_path,
        link_path=link_path,
    )


class SerialHub:
    def __init__(
        self,
        serial_port: str,
        baud: int,
        sim_path: Path,
        ctrl_path: Path | None,
        mavlink_log: Path | None = None,
    ) -> None:
        self._serial_port = serial_port
        self._baud = baud
        self._serial = self._open_serial()
        self._last_serial_rx = time.monotonic()
        self._idle_reopen_s = 2.0
        self._selector = selectors.DefaultSelector()
        self._running = True
        self._mavlink_log_path = mavlink_log
        self._mavlink_log_handle = None
        self._profile_enabled = os.environ.get("PX4_HIL_HUB_PROFILE", "0") == "1"
        profile_path = os.environ.get("PX4_HIL_HUB_PROFILE_PATH")
        self._profile_path = Path(profile_path) if profile_path else None
        self._stats = {
            "serial_rx_bytes": 0,
            "serial_tx_bytes": 0,
            "serial_backpressure_events": 0,
            "serial_max_queue_bytes": 0,
            "pty_rx_bytes": 0,
            "pty_tx_bytes": 0,
            "pty_backpressure_events": 0,
            "pty_drop_events": 0,
            "pty_max_queue_bytes": 0,
            "pty_no_reader_events": 0,
        }
        if mavlink_log is not None:
            mavlink_log.parent.mkdir(parents=True, exist_ok=True)
            self._mavlink_log_handle = mavlink_log.open("a", encoding="utf-8")
        self._parsers = {
            "serial_to_ptys": make_mavlink_parser(),
            "pty_to_serial": make_mavlink_parser(),
        }
        self._serial_tx_buffer = bytearray()
        self.sim = make_endpoint("sim", sim_path)
        self.ctrl = make_endpoint("ctrl", ctrl_path) if ctrl_path is not None else None
        self._endpoints = [self.sim]
        if self.ctrl is not None:
            self._endpoints.append(self.ctrl)
        self._pty_tx_buffers = {endpoint.master_fd: bytearray() for endpoint in self._endpoints}
        self._endpoint_reader_state = {endpoint.master_fd: False for endpoint in self._endpoints}
        self._endpoint_reader_check_deadline = {endpoint.master_fd: 0.0 for endpoint in self._endpoints}
        self._selector.register(self._serial.fileno(), selectors.EVENT_READ, ("serial", None))
        for endpoint in self._endpoints:
            self._selector.register(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))
        atexit.register(self.close)

    def _open_serial(self) -> serial.Serial:
        serial_dev = serial.Serial(self._serial_port, baudrate=self._baud, timeout=0, write_timeout=0)
        os.set_blocking(serial_dev.fileno(), False)
        return serial_dev

    def _reopen_serial(self) -> None:
        old_fd = None
        try:
            old_fd = self._serial.fileno()
        except Exception:
            pass

        if old_fd is not None:
            try:
                self._selector.unregister(old_fd)
            except Exception:
                pass

        try:
            self._serial.close()
        except Exception:
            pass

        while self._running:
            try:
                self._serial = self._open_serial()
                self._selector.register(self._serial.fileno(), selectors.EVENT_READ, ("serial", None))
                return
            except SerialException:
                time.sleep(0.5)

    def close(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._profile_enabled:
            summary = {
                "serial_port": self._serial_port,
                "baud": self._baud,
                **self._stats,
            }
            print(
                "Hub profile: "
                f"serial_rx_bytes={summary['serial_rx_bytes']} "
                f"serial_tx_bytes={summary['serial_tx_bytes']} "
                f"serial_backpressure_events={summary['serial_backpressure_events']} "
                f"serial_max_queue_bytes={summary['serial_max_queue_bytes']} "
                f"pty_rx_bytes={summary['pty_rx_bytes']} "
                f"pty_tx_bytes={summary['pty_tx_bytes']} "
                f"pty_backpressure_events={summary['pty_backpressure_events']} "
                f"pty_drop_events={summary['pty_drop_events']} "
                f"pty_max_queue_bytes={summary['pty_max_queue_bytes']} "
                f"pty_no_reader_events={summary['pty_no_reader_events']}",
                flush=True,
            )
            if self._profile_path is not None:
                self._profile_path.parent.mkdir(parents=True, exist_ok=True)
                self._profile_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        try:
            self._selector.close()
        except Exception:
            pass
        if self._mavlink_log_handle is not None:
            try:
                self._mavlink_log_handle.close()
            except Exception:
                pass
        for endpoint in self._endpoints:
            try:
                os.close(endpoint.master_fd)
            except OSError:
                pass
            try:
                os.close(endpoint.slave_fd)
            except OSError:
                pass
            if endpoint.link_path.exists() or endpoint.link_path.is_symlink():
                endpoint.link_path.unlink(missing_ok=True)
        try:
            self._serial.close()
        except Exception:
            pass

    def run(self) -> int:
        print(f"Hardware serial: {self._serial.port} @ {self._serial.baudrate}")
        print(f"Simulator PTY:   {self.sim.link_path} -> {self.sim.slave_path}")
        if self.ctrl is not None:
            print(f"Control PTY:     {self.ctrl.link_path} -> {self.ctrl.slave_path}")
        sys.stdout.flush()

        while self._running:
            for key, events in self._selector.select(timeout=0.5):
                kind, endpoint = key.data
                if kind == "serial":
                    if events & selectors.EVENT_READ:
                        self._handle_serial()
                    if events & selectors.EVENT_WRITE:
                        self._flush_serial()
                else:
                    if events & selectors.EVENT_READ:
                        self._handle_pty(endpoint)
                    if events & selectors.EVENT_WRITE:
                        self._flush_pty(endpoint)

            # HIL serial traffic should be continuous. If the CubeOrange USB CDC
            # path renumerates after an arm/reset event, the old fd can go quiet
            # without an immediate read error. Reopen proactively so the PTYs
            # keep following the current /dev/serial/by-id target.
            if time.monotonic() - self._last_serial_rx > self._idle_reopen_s:
                self._reopen_serial()
                self._last_serial_rx = time.monotonic()

        return 0

    def _handle_serial(self) -> None:
        try:
            data = os.read(self._serial.fileno(), 4096)
        except OSError:
            self._reopen_serial()
            return
        if not data:
            return
        self._last_serial_rx = time.monotonic()
        self._stats["serial_rx_bytes"] += len(data)
        self._log_mavlink_messages("serial_to_ptys", data)
        for endpoint in self._endpoints:
            self._queue_pty_write(endpoint, data)

    def _handle_pty(self, endpoint: PtyEndpoint) -> None:
        try:
            data = os.read(endpoint.master_fd, 4096)
        except BlockingIOError:
            return
        except OSError as exc:
            if exc.errno in (errno.EIO, errno.EPIPE):
                return
            raise
        if not data:
            return
        self._stats["pty_rx_bytes"] += len(data)
        self._endpoint_reader_state[endpoint.master_fd] = True
        self._endpoint_reader_check_deadline[endpoint.master_fd] = time.monotonic() + 1.0
        self._log_mavlink_messages("pty_to_serial", data)
        self._queue_serial_write(data)

    def _update_registration(self, fileobj: int, events: int, data: tuple[str, PtyEndpoint | None]) -> None:
        try:
            self._selector.modify(fileobj, events, data)
        except KeyError:
            self._selector.register(fileobj, events, data)

    def _queue_pty_write(self, endpoint: PtyEndpoint, data: bytes) -> None:
        if not self._endpoint_has_reader(endpoint):
            self._stats["pty_no_reader_events"] += 1
            self._pty_tx_buffers[endpoint.master_fd].clear()
            self._update_registration(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))
            return

        buffer = self._pty_tx_buffers[endpoint.master_fd]
        if not buffer:
            try:
                written = os.write(endpoint.master_fd, data)
            except BlockingIOError:
                written = 0
                self._stats["pty_backpressure_events"] += 1
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EPIPE):
                    self._stats["pty_drop_events"] += 1
                    return
                raise

            self._stats["pty_tx_bytes"] += max(written, 0)
            if written >= len(data):
                return
            buffer.extend(data[written:])
        else:
            buffer.extend(data)
            self._stats["pty_backpressure_events"] += 1

        self._stats["pty_max_queue_bytes"] = max(self._stats["pty_max_queue_bytes"], len(buffer))

        self._update_registration(
            endpoint.master_fd,
            selectors.EVENT_READ | selectors.EVENT_WRITE,
            ("pty", endpoint),
        )

    def _flush_pty(self, endpoint: PtyEndpoint) -> None:
        buffer = self._pty_tx_buffers[endpoint.master_fd]
        if not buffer:
            self._update_registration(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))
            return

        if not self._endpoint_has_reader(endpoint):
            self._stats["pty_no_reader_events"] += 1
            buffer.clear()
            self._update_registration(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))
            return

        try:
            written = os.write(endpoint.master_fd, buffer)
        except BlockingIOError:
            self._stats["pty_backpressure_events"] += 1
            return
        except OSError as exc:
            if exc.errno in (errno.EIO, errno.EPIPE):
                self._stats["pty_drop_events"] += 1
                buffer.clear()
                self._update_registration(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))
                return
            raise

        if written > 0:
            self._stats["pty_tx_bytes"] += written
            del buffer[:written]

        if not buffer:
            self._update_registration(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))

    def _queue_serial_write(self, data: bytes) -> None:
        if not self._serial_tx_buffer:
            try:
                written = os.write(self._serial.fileno(), data)
            except BlockingIOError:
                written = 0
                self._stats["serial_backpressure_events"] += 1
            except OSError:
                self._reopen_serial()
                return

            self._stats["serial_tx_bytes"] += max(written, 0)
            if written >= len(data):
                return
            self._serial_tx_buffer.extend(data[written:])
        else:
            self._serial_tx_buffer.extend(data)
            self._stats["serial_backpressure_events"] += 1

        self._stats["serial_max_queue_bytes"] = max(self._stats["serial_max_queue_bytes"], len(self._serial_tx_buffer))

        self._update_registration(
            self._serial.fileno(),
            selectors.EVENT_READ | selectors.EVENT_WRITE,
            ("serial", None),
        )

    def _flush_serial(self) -> None:
        if not self._serial_tx_buffer:
            self._update_registration(self._serial.fileno(), selectors.EVENT_READ, ("serial", None))
            return

        try:
            written = os.write(self._serial.fileno(), self._serial_tx_buffer)
        except BlockingIOError:
            self._stats["serial_backpressure_events"] += 1
            return
        except OSError:
            self._reopen_serial()
            return

        if written > 0:
            self._stats["serial_tx_bytes"] += written
            del self._serial_tx_buffer[:written]

        if not self._serial_tx_buffer:
            self._update_registration(self._serial.fileno(), selectors.EVENT_READ, ("serial", None))

    def _endpoint_has_reader(self, endpoint: PtyEndpoint) -> bool:
        if endpoint.name == "sim":
            return True

        now = time.monotonic()
        deadline = self._endpoint_reader_check_deadline[endpoint.master_fd]

        if now < deadline:
            return self._endpoint_reader_state[endpoint.master_fd]

        has_reader = False
        try:
            result = subprocess.run(
                ["lsof", "-t", endpoint.slave_path],
                capture_output=True,
                text=True,
                check=False,
            )
            current_pid = os.getpid()
            for line in result.stdout.splitlines():
                line = line.strip()
                if line.isdigit() and int(line) != current_pid:
                    has_reader = True
                    break
        except Exception:
            has_reader = True

        self._endpoint_reader_state[endpoint.master_fd] = has_reader
        self._endpoint_reader_check_deadline[endpoint.master_fd] = now + 1.0
        return has_reader

    def _log_mavlink_messages(self, direction: str, data: bytes) -> None:
        if self._mavlink_log_handle is None:
            return

        parser = self._parsers[direction]

        for byte in data:
            try:
                message = parser.parse_char(bytes([byte]))
            except Exception:
                continue

            if not message:
                continue

            record = {
                "ts": time.time(),
                "direction": direction,
                "name": message.get_type(),
                "msgid": int(message.get_msgId()),
                "sysid": int(message.get_srcSystem()),
                "compid": int(message.get_srcComponent()),
                "fields": sanitize_for_json(message.to_dict()),
            }
            self._mavlink_log_handle.write(json.dumps(record, sort_keys=True) + "\n")
            self._mavlink_log_handle.flush()


def make_mavlink_parser():
    parser = mavutil.mavlink.MAVLink(None)
    parser.robust_parsing = True
    return parser


def sanitize_for_json(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): sanitize_for_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_for_json(v) for v in value]
    if isinstance(value, (bytes, bytearray)):
        return list(value)
    return value


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial-port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--sim-link", type=Path, default=DEFAULT_SIM_PATH)
    ap.add_argument("--control-link", type=Path, default=None)
    ap.add_argument("--mavlink-log", type=Path, default=None)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    hub = SerialHub(args.serial_port, args.baud, args.sim_link, args.control_link, args.mavlink_log)

    def _stop(_sig, _frame) -> None:
        hub.close()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    return hub.run()


if __name__ == "__main__":
    raise SystemExit(main())
