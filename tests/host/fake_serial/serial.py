"""Minimal stand-in for pyserial: 'opens' a file of captured UART bytes.

Lets the real tools/parse_imu.py run unmodified against bytes produced by the
host simulation:  PYTHONPATH=tests/host/fake_serial python3 tools/parse_imu.py <file> 0.5
"""
import time


class Serial:
    def __init__(self, port, baudrate=9600, timeout=None):
        self._f = open(port, "rb")

    def reset_input_buffer(self):
        pass

    def read(self, n=1):
        data = self._f.read(n)
        if not data:
            time.sleep(0.01)        # end of capture: behave like an idle line
        return data

    def close(self):
        self._f.close()
