"""Immediate nonblocking terminal keys; no input thread or signal replacement.

端末の非ブロッキングキー入力と設定復元。測定や走行判断は持たない。"""

import atexit
import os
import select
import sys


class TerminalKeys:
    def __init__(self, stream=None):
        if stream is None:
            self.stream = sys.stdin
        else:
            self.stream = stream
        self.fd = None
        self.saved = None
        self.termios = None
        self.windows = None
        self.registered = False

    def open(self):
        if not self.stream.isatty():
            raise RuntimeError(
                "Interactive sensor test requires terminal stdin; use an SSH terminal (ssh -t)"
            )
        if sys.platform == "win32":
            import msvcrt

            self.windows = msvcrt
        else:
            import termios
            import tty

            self.fd = self.stream.fileno()
            self.termios = termios
            self.saved = termios.tcgetattr(self.fd)
            try:
                # cbreak retains ISIG: alpha's Ctrl+C/shutdown handling still applies.
                tty.setcbreak(self.fd)
                termios.tcflush(self.fd, termios.TCIFLUSH)
            except Exception:
                self.close()
                raise
        atexit.register(self.close)
        self.registered = True

    def poll(self):
        if self.windows is not None:
            keys = []
            while len(keys) < 32 and self.windows.kbhit():
                keys.append(self.windows.getwch())
            return keys
        if self.fd is None:
            return []
        ready, _, _ = select.select([self.fd], [], [], 0)
        if not ready:
            return []
        data = os.read(self.fd, 32)
        if data:
            return list(data.decode("ascii", errors="ignore"))
        else:
            return ["q"]

    def close(self):
        try:
            if self.saved is not None:
                self.termios.tcsetattr(self.fd, self.termios.TCSADRAIN, self.saved)
        finally:
            self.saved = None
            self.fd = None
            self.windows = None
            if self.registered:
                atexit.unregister(self.close)
                self.registered = False
