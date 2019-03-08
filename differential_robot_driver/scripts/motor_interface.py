#!/usr/bin/env python
import serial
import math
import time
from threading import Timer, Lock

class HubMotorInterface:

    _status_code = {
        0: 'stop',
        1: 'start',
        2: 'over current',
        3: 'over voltage',
        4: 'encoder error',
        5: 'over heat',
        6: 'under voltage',
        7: 'over load'
    }

    _config = dict(
        baudrate = 57600,
        cmd_pkg_size = 4,
        rpl_pkg_size = 32,
        heart_beat_dur = 0.05,
        default_acc = 0x64,
        serial_timeout = 0.02,
    )

    _checksum = lambda self, msg: sum(msg) & 0xff

    def __init__(self, name, port, debug = False):
        self.name = name
        self.serial_port = port
        self.serial = None
        self.debug_mode = debug
        self.feedback = dict(
            status = 0,
            vol = 0,
            out_c = 0,
            out_s = 0,
            posi_set = 0,
            posi_fee = 0
        )
        self.serial_lock = Lock()
        self.timer = Timer(self._config.get('heart_beat_dur'), self.__heart_beater)
        self.available = False

    def _debug_log(self, msg):
        if self.debug_mode:
            print('[DEBUG]' + self.name + ': ' + msg)
    
    def _err_log(self, msg):
        print('[ERROR]' + self.name + ': ' + msg)

    def begin(self):
        try:
            self.serial = serial.Serial(self.serial_port,57600)
        except Exception, e:
            self._err_log(repr(e))
            return
        self.available = True
        self.set_pcmode()
        self.set_acc(self._config.get('default_acc'))
        self.set_speed(0)
        self.enable()
        
        self.timer.start()
    
    def end(self):
        self.timer.cancel()
        if self.serial:
            self.set_speed(0)
            self.disable()
            self.serial.close()
            
    def _serialSend(log):
        def decorator(fn):
            def wrapper(self, *args, **kwds):
                if not self.available:
                    self._err_log('motor is not begin')
                    return
                self._debug_log(log)
                msg = fn(self, *args, **kwds)
                msg.append(self._checksum(msg))
                self.serial_lock.acquire()
                try:
                    self.serial.write(msg)
                except:
                    self._err_log('serial(' + self.serial_port + ') is not open')
                    pass
                self.serial_lock.release()
                self._debug_log(str(msg))
            return wrapper
        return decorator

    @_serialSend(log = 'enable motor')
    def enable(self):
        return [0x00, 0x00, 0x01]

    @_serialSend(log = 'disable motor')
    def disable(self):
        return [0x00, 0x00, 0x00]

    @_serialSend(log = 'select speed pc mode')
    def set_pcmode(self):
        return [0x02, 0x00, 0xc4]

    @_serialSend(log = 'set accel time')
    def set_acc(self,acc):
        return [0x0a, acc, acc]

    @_serialSend(log = 'set speed')
    def set_speed(self,rpm):
        rpm = int(rpm*10.0/60000.0*16384.0) & 0xffff
        rpm_h = (rpm >> 8) & 0xff
        rpm_l = rpm & 0xff
        return [0x06, rpm_h, rpm_l]
        
    @_serialSend(log = 'send heart beater package')
    def __heart_beat(self):
        return [0x80,0x00]

    def __heart_beater(self):
        self.timer = Timer(self._config.get('heart_beat_dur'), self.__heart_beater)
        self.timer.start()
        self.__heart_beat()
        self._read_feedback()

    def _read_feedback(self):
        b = 0
        rpl_size = self._config.get('rpl_pkg_size') - 1
        while self.serial.inWaiting() > 0 and b != 0x80:
            b = ord(self.serial.read())
        if b == 0x80:
            if self.serial.inWaiting() > rpl_size:
                self.serial.read(self.serial.inWaiting())
                self._debug_log('serial flush')
                return False
            t = time.time()
            while self.serial.inWaiting() < rpl_size:
                if time.time() - t > self._config.get('serial_timeout'):
                    self._debug_log('read serial out of time')
                    return False
                else:
                    time.sleep(0.001)
            try:
                received_bytes = self.serial.read(rpl_size)
            except:
                self._err_log('failed to read serial')
                pass
            received_bytes = [ord(c) for c in received_bytes]
            received_bytes.insert(0, b)
            self._debug_log(str(received_bytes))
            self._parse_package(received_bytes)

    def _parse_package(self, pkg):
        params = [0 for i in range(8)]
        for i in range(8):
            data = [pkg[i*4+j] for j in range(4)]
            if self._checksum(data[:3]) == data[3]:
                params[i] = data[1] << 8 | data[2]
        lowbit = lambda x: x & (-x)
        int16 = lambda n: -(~(n-1)&0xffff) if(n>>15) else n 
        int32 = lambda n: -(~(n-1)&0xffffffff) if(n>>31) else n
        self.feedback = dict(
            status = self._status_code[lowbit(params[0])],
            vol = params[1],
            out_c = params[2] / 100.0,
            out_s = int16(params[3])/16384.0*6000,
            posi_set = int32(params[4] << 16 | params[5]),
            posi_fee = int32(params[6] << 16 | params[7]),
        )
        self._debug_log(str(self.feedback))

    def get_status(self):
        return self.feedback.get('status')
    def get_posi_fee(self):
        return self.feedback.get('posi_fee')

if __name__=='__main__':
    motor = HubMotorInterface("motor0","/dev/ttyUSB0", debug = True)
    motor2 = HubMotorInterface("motor1","/dev/ttyUSB1", debug = False)
    motor.begin()
    motor2.begin()
    # motor2.set_speed(-60)
    motor.end()
    motor2.end()
