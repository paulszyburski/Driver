# -*- coding: utf-8 -*-

import glob
import signal
import struct
import sys
import threading
from time import sleep

import serial

try:
    import hid
except ImportError:
    hid = None


class mSerial:
    def __init__(self):
        self.ser = None

    def start(self, port, baud=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0,
            write_timeout=1
        )

        # mCore / Arduino usually resets when serial opens
        sleep(2)

        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def device(self):
        return self.ser

    def serialPorts(self):
        if sys.platform.startswith("win"):
            ports = ["COM%s" % (i + 1) for i in range(256)]
        elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        elif sys.platform.startswith("darwin"):
            ports = glob.glob("/dev/tty.*")
        else:
            raise EnvironmentError("Unsupported platform")

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except Exception:
                pass

        return result

    def writePackage(self, package):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Serial port is not open")

        self.ser.write(bytes(package))
        self.ser.flush()
        sleep(0.01)

    def read(self, size=1):
        return self.ser.read(size)

    def isOpen(self):
        return self.ser is not None and self.ser.is_open

    def inWaiting(self):
        if self.ser is None or not self.ser.is_open:
            return 0
        return self.ser.in_waiting

    def close(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()


class mHID:
    """
    HID support is kept only for compatibility.
    For your mBot over USB serial, do not use this.
    """

    def __init__(self):
        self.device = None
        self.buffer = []

    def start(self):
        if hid is None:
            raise RuntimeError("hid module is not installed")

        self.buffer = []

        if hasattr(hid, "device"):
            self.device = hid.device()
            self.device.open(0x0416, 0xffff)

            try:
                self.device.set_nonblocking(True)
            except Exception:
                pass

        elif hasattr(hid, "Device"):
            self.device = hid.Device(vid=0x0416, pid=0xffff)

        else:
            raise RuntimeError("Unsupported hid library API")

    def enumerate(self):
        return []

    def writePackage(self, package):
        if self.device is None:
            raise RuntimeError("HID device is not open")

        buf = bytearray()
        buf.append(0)
        buf.append(len(package))
        buf.extend(bytearray(package))

        self.device.write(bytes(buf))
        sleep(0.01)

    def read(self, size=1):
        if not self.buffer:
            return None

        value = self.buffer[0]
        self.buffer = self.buffer[1:]
        return value

    def isOpen(self):
        return self.device is not None

    def inWaiting(self):
        if self.device is None:
            return 0

        try:
            try:
                data = self.device.read(64, timeout_ms=1)
            except TypeError:
                data = self.device.read(64)

            if data is None:
                return len(self.buffer)

            data = list(data)

            if len(data) > 0:
                length = data[0]

                if length > 0:
                    for i in range(length):
                        if i + 1 < len(data):
                            self.buffer.append(data[i + 1])

        except Exception:
            pass

        return len(self.buffer)

    def close(self):
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass

        self.device = None


class mBot:
    def __init__(self):
        try:
            signal.signal(signal.SIGINT, self.exit)
        except Exception:
            pass

        self.__selectors = {}
        self.buffer = []
        self.bufferIndex = 0
        self.isParseStart = False
        self.exiting = False
        self.isParseStartIndex = 0
        self.device = None
        self._read_thread = None
        self._write_lock = threading.Lock()
        self._response_events = {}
        self._response_values = {}

    def startWithSerial(self, port):
        self.device = mSerial()
        self.device.start(port)
        self.start()

    def startWithHID(self):
        self.device = mHID()
        self.device.start()
        self.start()

    def excepthook(self, exctype, value, traceback):
        self.close()

    def start(self):
        sys.excepthook = self.excepthook

        self.exiting = False
        self._read_thread = threading.Thread(
            target=self.__onRead,
            args=(self.onParse,),
            daemon=True
        )
        self._read_thread.start()

    def close(self):
        self.exiting = True

        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass

    def exit(self, signal_value=None, frame=None):
        self.close()
        sys.exit(0)

    def __onRead(self, callback):
        while not self.exiting:
            try:
                if self.device is None or not self.device.isOpen():
                    sleep(0.1)
                    continue

                n = self.device.inWaiting()

                for _ in range(n):
                    raw = self.device.read()

                    if raw is None:
                        continue

                    if isinstance(raw, int):
                        byte = raw
                    elif isinstance(raw, (bytes, bytearray)):
                        if len(raw) == 0:
                            continue
                        byte = raw[0]
                    elif isinstance(raw, str):
                        byte = ord(raw)
                    else:
                        continue

                    callback(byte)

                sleep(0.01)

            except (OSError, serial.SerialException, ValueError):
                if not self.exiting:
                    sleep(0.05)

            except Exception as e:
                if not self.exiting:
                    print("mBot read thread error:", repr(e), file=sys.stderr)
                    sleep(0.05)

    def __writePackage(self, pack):
        if self.device is None:
            raise RuntimeError("mBot is not connected")

        with self._write_lock:
            self.device.writePackage(bytearray(pack))

    def _u8(self, value):
        return max(0, min(255, int(value)))

    def _speed(self, value):
        return max(-255, min(255, int(value)))

    def doRGBLed(self, port, slot, index, red, green, blue):
        self.__writePackage(bytearray([
            0xff, 0x55, 0x9, 0x0, 0x2, 0x8,
            self._u8(port),
            self._u8(slot),
            self._u8(index),
            self._u8(red),
            self._u8(green),
            self._u8(blue)
        ]))

    def doRGBLedOnBoard(self, index, red, green, blue):
        self.doRGBLed(0x7, 0x2, index, red, green, blue)

    def doMotor(self, port, speed):
        speed = self._speed(speed)
        self.__writePackage(bytearray([
            0xff, 0x55, 0x6, 0x0, 0x2, 0xa, self._u8(port)
        ] + self.short2bytes(speed)))

    def doMove(self, leftSpeed, rightSpeed):
        leftSpeed = self._speed(leftSpeed)
        rightSpeed = self._speed(rightSpeed)

        self.__writePackage(bytearray([
            0xff, 0x55, 0x7, 0x0, 0x2, 0x5
        ] + self.short2bytes(-leftSpeed) + self.short2bytes(rightSpeed)))

    def doServo(self, port, slot, angle):
        angle = max(0, min(180, int(angle)))

        self.__writePackage(bytearray([
            0xff, 0x55, 0x6, 0x0, 0x2, 0xb,
            self._u8(port),
            self._u8(slot),
            self._u8(angle)
        ]))

    def doBuzzer(self, buzzer, time=0):
        self.__writePackage(bytearray([
            0xff, 0x55, 0x7, 0x0, 0x2, 0x22
        ] + self.short2bytes(int(buzzer)) + self.short2bytes(int(time))))

    def doSevSegDisplay(self, port, display):
        self.__writePackage(bytearray([
            0xff, 0x55, 0x8, 0x0, 0x2, 0x9, self._u8(port)
        ] + self.float2bytes(float(display))))

    def doIROnBoard(self, message):
        if isinstance(message, str):
            msg = message.encode("utf-8")
        elif isinstance(message, int):
            msg = bytes([message])
        else:
            msg = bytes(message)

        self.__writePackage(bytearray([
            0xff, 0x55, len(msg) + 3, 0x0, 0x2, 0xd
        ] + list(msg)))

    def requestLightOnBoard(self, extID, callback):
        self.requestLight(extID, 8, callback)

    def requestLight(self, extID, port, callback):
        self.__doCallback(extID, callback)
        self.__writePackage(bytearray([
            0xff, 0x55, 0x4, self._u8(extID), 0x1, 0x3, self._u8(port)
        ]))

    def requestButtonOnBoard(self, extID, callback):
        self.__doCallback(extID, callback)
        self.__writePackage(bytearray([
            0xff, 0x55, 0x4, self._u8(extID), 0x1, 0x1f, 0x7
        ]))

    def requestIROnBoard(self, extID, callback):
        self.__doCallback(extID, callback)
        self.__writePackage(bytearray([
            0xff, 0x55, 0x3, self._u8(extID), 0x1, 0xd
        ]))

    def requestUltrasonicSensor(self, extID, port, callback=None, timeout=1.0):
        event = None

        if callback is None:
            event = threading.Event()
            self._response_events[extID] = event
            self._response_values.pop(extID, None)

            def _store_value(value):
                self._response_values[extID] = value
                event.set()

            self.__doCallback(extID, _store_value)
        else:
            def _callback_and_store(value):
                self._response_values[extID] = value
                callback(value)

            self.__doCallback(extID, _callback_and_store)

        self.__writePackage(bytearray([
            0xff, 0x55, 0x4, self._u8(extID), 0x1, 0x1, self._u8(port)
        ]))

        if event is None:
            return None

        if event.wait(timeout):
            self._response_events.pop(extID, None)
            return self._response_values.get(extID)

        self._response_events.pop(extID, None)
        return None

    def requestLineFollower(self, extID, port, callback):
        self.__doCallback(extID, callback)
        self.__writePackage(bytearray([
            0xff, 0x55, 0x4, self._u8(extID), 0x1, 0x11, self._u8(port)
        ]))

    def onParse(self, byte):
        self.buffer.append(byte)

        if len(self.buffer) > 256:
            self.buffer = self.buffer[-64:]

        bufferLength = len(self.buffer)

        if bufferLength >= 2:
            if self.buffer[bufferLength - 1] == 0x55 and self.buffer[bufferLength - 2] == 0xff:
                self.isParseStart = True
                self.isParseStartIndex = bufferLength - 2

            if (
                self.buffer[bufferLength - 1] == 0x0a
                and self.buffer[bufferLength - 2] == 0x0d
                and self.isParseStart is True
            ):
                self.isParseStart = False

                position = self.isParseStartIndex + 2

                if position + 2 > len(self.buffer):
                    self.buffer = []
                    return

                extID = self.buffer[position]
                position += 1

                value_type = self.buffer[position]
                position += 1

                value = 0

                try:
                    # 1 byte, 2 float, 3 short, 4 string, 5 double/float
                    if value_type == 1:
                        value = self.buffer[position]

                    elif value_type == 2:
                        value = self.readFloat(position)

                        if value < -255 or value > 1023:
                            value = 0

                    elif value_type == 3:
                        value = self.readShort(position)

                    elif value_type == 4:
                        value = self.readString(position)

                    elif value_type == 5:
                        value = self.readDouble(position)

                    if value_type <= 5:
                        self.responseValue(extID, value)

                except Exception as e:
                    print("mBot parse error:", repr(e), file=sys.stderr)

                self.buffer = []

    def readFloat(self, position):
        v = self.buffer[position:position + 4]
        return struct.unpack("<f", bytes(v))[0]

    def readShort(self, position):
        v = self.buffer[position:position + 2]
        return struct.unpack("<h", bytes(v))[0]

    def readString(self, position):
        length = self.buffer[position]
        position += 1

        raw = self.buffer[position:position + length]
        return bytes(raw).decode("utf-8", errors="replace")

    def readDouble(self, position):
        # Original Makeblock protocol often sends float here
        v = self.buffer[position:position + 4]
        return struct.unpack("<f", bytes(v))[0]

    def responseValue(self, extID, value):
        callback = self.__selectors.get("callback_" + str(extID))

        if callback is not None:
            callback(value)

    def __doCallback(self, extID, callback):
        self.__selectors["callback_" + str(extID)] = callback

    def float2bytes(self, fval):
        return list(struct.pack("<f", float(fval)))

    def short2bytes(self, sval):
        return list(struct.pack("<h", int(sval)))
