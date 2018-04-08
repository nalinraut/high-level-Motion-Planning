import threading
import traceback
import sys
import time


__author__ = 'paoolo'


def chunks(l, n):
    for i in range(0, len(l), n):
        yield l[i:i + n]


def decode(val):
    bin_str = '0b'
    for char in val:
        val = ord(char) - 0x30
        bin_str += '%06d' % int(bin(val)[2:])
    return int(bin_str, 2)


class Hokuyo(object):
    SHORT_COMMAND_LEN = 5
    MD_COMMAND_REPLY_LEN = 20

    LASER_ON = 'BM\n'
    LASER_OFF = 'QT\n'
    RESET = 'RS\n'

    VERSION_INFO = 'VV\n'
    SENSOR_STATE = 'II\n'
    SENSOR_SPECS = 'PP\n'
    SET_SCIP2    = 'SCIP2.0\n'

    CHARS_PER_VALUE = 3.0
    CHARS_PER_LINE = 66.0
    CHARS_PER_BLOCK = 64.0

    START_DEG = 314.875-90
    STEP_DEG = 0.25

    START_STEP = 0
    STOP_STEP = 1080

    VERSION_INFO_LINES = 6
    SENSOR_STATE_LINES = 8
    SENSOR_SPECS_LINES = 9

    def __init__(self, port):
        self.__port = port
        self.__port_lock = threading.RLock()

        self.__timestamp, self.__angles, self.__distances = 0, [], []
        self.__scan_lock = threading.Lock()

        self.__is_active = True
        self.__scanning_allowed = False

    def __offset(self):
        count = 2
        result = ''

        self.__port_lock.acquire()
        try:
            a = self.__port.read(1)
            b = self.__port.read(1)

            while not ((a == '\n' and b == '\n') or (a == '' and b == '')):
                result += a
                a = b
                b = self.__port.read(1)
                count += 1
        finally:
            self.__port_lock.release()

        result += a
        result += b

        sys.stderr.write('READ %d EXTRA BYTES: "%s"\n' % (count, str(result)))

    def __execute_command(self, command):
        self.__port_lock.acquire()
        try:
            self.__port.write(command)
            result = self.__port.read(len(command))
            assert result == command
        finally:
            self.__port_lock.release()
        return result

    def __short_command(self, command, check_response=True):
        result = ''
        self.__port_lock.acquire()
        try:
            try:
                result += self.__execute_command(command)
                result += self.__port.read(Hokuyo.SHORT_COMMAND_LEN)

                if check_response:
                    assert result[-5:-2] == '00P'
                assert result[-2:] == '\n\n'

                return result
            except BaseException as e:
                sys.stderr.write('RESULT: "%s"' % result)
                traceback.print_exc()
                self.__offset()
        finally:
            self.__port_lock.release()

    def __long_command(self, cmd, lines, check_response=True):
        result = ''
        self.__port_lock.acquire()
        try:
            try:
                result += self.__execute_command(cmd)

                result += self.__port.read(4)
                if check_response:
                    assert result[-4:-1] == '00P'
                assert result[-1:] == '\n'

                line = 0
                while line < lines:
                    char = self.__port.read_byte()
                    if not char is None:
                        char = chr(char)
                        result += char
                        if char == '\n':
                            line += 1
                    else:  # char is None
                        line += 1

                assert result[-2:] == '\n\n'

                return result
            except BaseException as e:
                sys.stderr.write('RESULT: "%s"' % result)
                traceback.print_exc()
                self.__offset()
        finally:
            self.__port_lock.release()

    def terminate(self):
        self.reset()

        self.__is_active = False
        self.__port_lock.acquire()
        try:
            self.__port.close()
        finally:
            self.__port_lock.release()

    def laser_on(self):
        return self.__short_command(Hokuyo.LASER_ON, check_response=True)


    def laser_off(self):
        return self.__short_command(Hokuyo.LASER_OFF)

    def reset(self):
        return self.__short_command(Hokuyo.RESET)

    def set_scip2(self):
        "for URG-04LX"
        return self.__short_command(Hokuyo.SET_SCIP2, check_response=False)

    def set_motor_speed(self, motor_speed=99):
        return self.__short_command('CR' + '%02d' % motor_speed + '\n', check_response=False)

    def set_high_sensitive(self, enable=True):
        return self.__short_command('HS' + ('1\n' if enable else '0\n'), check_response=False)

    def get_version_info(self):
        return self.__long_command(Hokuyo.VERSION_INFO, Hokuyo.VERSION_INFO_LINES)

    def get_sensor_state(self):
        return self.__long_command(Hokuyo.SENSOR_STATE, Hokuyo.SENSOR_STATE_LINES)

    def get_sensor_specs(self):
        return self.__long_command(Hokuyo.SENSOR_SPECS, Hokuyo.SENSOR_SPECS_LINES)

    def __get_and_parse_scan(self, cluster_count, start_step, stop_step):
        distances = {}
        result = ''

        count = ((stop_step - start_step) * Hokuyo.CHARS_PER_VALUE * Hokuyo.CHARS_PER_LINE)
        count /= (Hokuyo.CHARS_PER_BLOCK * cluster_count)
        count += 1.0 + 4.0  # paoolo(FIXME): why +4.0?
        count = int(count)

        self.__port_lock.acquire()
        try:
            result += self.__port.read(count)
        finally:
            self.__port_lock.release()

        assert result[-2:] == '\n\n'

        result = result.split('\n')
        result = [line[:-1] for line in result]
        result = ''.join(result)

        i = 0
        start = (-Hokuyo.START_DEG + Hokuyo.STEP_DEG * cluster_count * (start_step - Hokuyo.START_STEP))
        for chunk in chunks(result, 3):
            distances[- ((Hokuyo.STEP_DEG * cluster_count * i) + start)] = decode(chunk)
            i += 1

        return distances

    def get_single_scan(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1):
        self.__port_lock.acquire()
        try:
            cmd = 'GD%04d%04d%02d\n' % (start_step, stop_step, cluster_count)
            self.__port.write(cmd)

            result = self.__port.read(len(cmd))
            assert result == cmd

            result += self.__port.read(4)
            assert result[-4:-1] == '00P'
            assert result[-1] == '\n'

            result = self.__port.read(6)
            assert result[-1] == '\n'

            scan = self.__get_and_parse_scan(cluster_count, start_step, stop_step)
            return scan

        except BaseException as e:
            traceback.print_exc()
            self.__offset()

        finally:
            self.__port_lock.release()

    def __get_multiple_scans(self, start_step=START_STEP, stop_step=STOP_STEP, cluster_count=1,
                             scan_interval=0, number_of_scans=0):
        self.__port_lock.acquire()
        try:
            cmd = 'MD%04d%04d%02d%01d%02d\n' % (start_step, stop_step, cluster_count, scan_interval, number_of_scans)
            self.__port.write(cmd)

            result = self.__port.read(len(cmd))
            assert result == cmd

            result += self.__port.read(Hokuyo.SHORT_COMMAND_LEN)
            assert result[-2:] == '\n\n'

            index = 0
            while number_of_scans == 0 or index > 0:
                index -= 1

                result = self.__port.read(Hokuyo.MD_COMMAND_REPLY_LEN)
                assert result[:13] == cmd[:13]

                result = self.__port.read(6)
                assert result[-1] == '\n'

                scan = self.__get_and_parse_scan(cluster_count, start_step, stop_step)
                yield scan

        except BaseException as e:
            traceback.print_exc()
            self.__offset()

        finally:
            self.__port_lock.release()

    def enable_scanning(self, _enable_scanning):
        self.__scanning_allowed = _enable_scanning

    def __set_scan(self, scan):
        if scan is not None:
            timestamp = int(time.time() * 1000.0)
            angles, distances = Hokuyo.__parse_scan(scan)

            self.__scan_lock.acquire()
            try:
                self.__angles, self.__distances, self.__timestamp = angles, distances, timestamp
            finally:
                self.__scan_lock.release()

    def get_scan(self):
        if not self.__scanning_allowed:
            scan = self.get_single_scan()
            self.__set_scan(scan)

        self.__scan_lock.acquire()
        try:
            return self.__angles, self.__distances, self.__timestamp
        finally:
            self.__scan_lock.release()

    def scanning_loop(self):
        while self.__is_active:
            if self.__scanning_allowed:
                self.__port_lock.acquire()
                for scan in self.__get_multiple_scans():
                    self.__set_scan(scan)
                    if not self.__scanning_allowed or not self.__is_active:
                        self.laser_off()
                        self.laser_on()
                        self.__port_lock.release()
                        break
            time.sleep(0.1)

    @staticmethod
    def __parse_scan(scan):
        angles = sorted(scan.keys())
        distances = list(map(scan.get, angles))
        return angles, distances
