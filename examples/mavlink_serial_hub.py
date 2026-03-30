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
import os
import pty
import selectors
import signal
import sys
import termios
import time
from dataclasses import dataclass
from pathlib import Path

import serial
from serial import SerialException


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
    def __init__(self, serial_port: str, baud: int, sim_path: Path, ctrl_path: Path) -> None:
        self._serial_port = serial_port
        self._baud = baud
        self._serial = self._open_serial()
        self._selector = selectors.DefaultSelector()
        self._running = True
        self.sim = make_endpoint("sim", sim_path)
        self.ctrl = make_endpoint("ctrl", ctrl_path)
        self._endpoints = [self.sim, self.ctrl]
        self._selector.register(self._serial.fileno(), selectors.EVENT_READ, ("serial", None))
        for endpoint in self._endpoints:
            self._selector.register(endpoint.master_fd, selectors.EVENT_READ, ("pty", endpoint))
        atexit.register(self.close)

    def _open_serial(self) -> serial.Serial:
        return serial.Serial(self._serial_port, baudrate=self._baud, timeout=0)

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
        try:
            self._selector.close()
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
        print(f"Control PTY:     {self.ctrl.link_path} -> {self.ctrl.slave_path}")
        sys.stdout.flush()

        while self._running:
            for key, _ in self._selector.select(timeout=0.5):
                kind, endpoint = key.data
                if kind == "serial":
                    self._handle_serial()
                else:
                    self._handle_pty(endpoint)

        return 0

    def _handle_serial(self) -> None:
        try:
            data = os.read(self._serial.fileno(), 4096)
        except OSError:
            self._reopen_serial()
            return
        if not data:
            return
        for endpoint in self._endpoints:
            try:
                os.write(endpoint.master_fd, data)
            except BlockingIOError:
                continue
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EPIPE):
                    continue
                raise

    def _handle_pty(self, endpoint: PtyEndpoint) -> None:
        try:
            data = os.read(endpoint.master_fd, 4096)
        except BlockingIOError:
            return
        if not data:
            return
        try:
            self._serial.write(data)
        except (OSError, SerialException):
            self._reopen_serial()


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("--serial-port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--sim-link", type=Path, default=DEFAULT_SIM_PATH)
    ap.add_argument("--control-link", type=Path, default=DEFAULT_CTRL_PATH)
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    hub = SerialHub(args.serial_port, args.baud, args.sim_link, args.control_link)

    def _stop(_sig, _frame) -> None:
        hub.close()

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    return hub.run()


if __name__ == "__main__":
    raise SystemExit(main())
