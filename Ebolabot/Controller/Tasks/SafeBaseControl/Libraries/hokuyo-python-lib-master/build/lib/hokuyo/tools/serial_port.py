import struct
import sys

__author__ = 'paoolo'


class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0

    def close(self):
        self.__port.close()

    def get_checksum(self):
        return self.__checksum

    def read(self, size):
        char = self.__port.read(size)
        if sys.version_info >= (3,0,0):
            char = str(char, 'UTF-8')
        return char

    def write(self, char):
        if sys.version_info >= (3,0,0):
            char = bytes(char, 'UTF-8')
        self.__port.write(char)

    def send_command(self, address, command):
        self.__checksum = address
        self.__port.write(chr(address))
        self.__checksum += command
        self.__port.write(chr(command))

    def read_byte(self):
        res = self.__port.read(1)
        if len(res) > 0:
            val = struct.unpack('>B', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_sbyte(self):
        res = self.__port.read(1)
        if len(res) > 0:
            val = struct.unpack('>b', res)
            self.__checksum += val[0] & 0xFF
            return val[0]
        return None

    def read_word(self):
        res = self.__port.read(2)
        if len(res) > 0:
            val = struct.unpack('>H', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_sword(self):
        res = self.__port.read(2)
        if len(res) > 0:
            val = struct.unpack('>h', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            return val[0]
        return None

    def read_long(self):
        res = self.__port.read(4)
        if len(res) > 0:
            val = struct.unpack('>L', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def read_slong(self):
        res = self.__port.read(4)
        if len(res) > 0:
            val = struct.unpack('>l', res)
            self.__checksum += val[0] & 0xFF
            self.__checksum += (val[0] >> 8) & 0xFF
            self.__checksum += (val[0] >> 16) & 0xFF
            self.__checksum += (val[0] >> 24) & 0xFF
            return val[0]
        return None

    def write_byte(self, val):
        self.__checksum += val & 0xFF
        return self.__port.write(struct.pack('>B', val))

    def write_sbyte(self, val):
        self.__checksum += val & 0xFF
        return self.__port.write(struct.pack('>b', val))

    def write_word(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        return self.__port.write(struct.pack('>H', val))

    def write_sword(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        return self.__port.write(struct.pack('>h', val))

    def write_long(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        return self.__port.write(struct.pack('>L', val))

    def write_slong(self, val):
        self.__checksum += val & 0xFF
        self.__checksum += (val >> 8) & 0xFF
        self.__checksum += (val >> 16) & 0xFF
        self.__checksum += (val >> 24) & 0xFF
        return self.__port.write(struct.pack('>l', val))
