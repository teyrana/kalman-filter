#!/usr/bin/python3

# standard library
import csv
import struct
import sys
import time

# pip packages
import serial


class Command:
    def __init__(self, command=0, data=''):
        # print("Building command...")
        text = 'f7{:0>2X}{}'.format(command, data)
        # print("... from command #{:>3} := {}".format(int(command), text))

        # The checksum is computed as an arithmetic summation of all of the characters in the packet (except
        # the checksum value itself) modulus 256. The result is a single byte in the range [0, 255]
        checksum = sum(bytes.fromhex(text[2:]))
        # print("... from              := {}".format(text[2:]))
        text += '{:0>2X}'.format(checksum)
        # print("... with checksum     := {}".format(text))

        self._binary_command = bytes.fromhex(text)

    def __repr__(self):
        line = '{:X} '.format(self._binary_command[0])  # sync byte
        line += '{:0>2X} '.format(self._binary_command[1])  # command id
        line += ''.join(['{:0>2X}'.format(b) for b in self._binary_command[2:-1]])  # data bytes
        line += ' {:0>2X}'.format(self._binary_command[-1]) # checksum
        return line

    def bin(self) -> bytes:
        return self._binary_command

    def cmd(self):
        return str(hex(self._binary_command[1]))


class Sample:

    # Command    Description                                    Return    Return Data Details
    #                                                           Length
    # 0 (0x00)   Read filtered, tared orientation(Quaternion)   16        Quaternion (float x4)
    get_qgrad2_quaternion_command = Command(0x00)

    #            Normalized Sensor Data
    # 33(0x21)   Read gyros                                     12        Vector(float x3)
    get_gyros_command = Command(0x21)

    #            Normalized Sensor Data
    # 34(0x22)   Read accelerometer                             12        Vector(float x3)
    get_accel_command = Command(0x22)

    def row(self) -> dict:
        """
        :return: returns a row of data to write to csv file
        """
        self._row['time'] = str(self._time)
        self._row['quaternion//QGRAD2'] = str(self._quaternion_qgrad2)
        self._row['gyro'] = str(self._gyros)
        self._row['accel'] = str(self._accelerations)

        return self._row

    def __init__(self, source_: serial.serialwin32):
        self._source = source_

        self._row = {'time': '',
                     'quaternion//QGRAD2': '',
                     'gyro': '',
                     'accel': '',
                     'n': ''}
        self._time = 0
        self._quaternion_qgrad2 = [0, 0, 0, 1] # identity value
        self._gyros = [0, 0, 0]
        self._accelerations = [0, 0, 0]

    def __repr__(self) -> str:
        return '[{}]: {}'.format(self._time,
                                 self._quaternion_qgrad2)

    @property
    def time(self) -> float:
        return self._time

    def titles(self) -> list:
        return list(self._row.keys())

    def update(self) -> None:
        """
        source: https://yostlabs.com/wp/wp-content/uploads/pdf/3-Space-Sensor-Users-Manual-Nano.pdf
        """

        self._time = time.time()
        self._row['time'] = str(self._time)

        # Get Filtered(QGRAD2) Quaternion)
        self._source.write(Sample.get_qgrad2_quaternion_command.bin())
        try:
            qfbin = self._source.read(16)
            self._quaternion_qgrad2[0] = struct.unpack('>f', qfbin[0:4])[0]
            self._quaternion_qgrad2[1] = struct.unpack('>f', qfbin[4:8])[0]
            self._quaternion_qgrad2[2] = struct.unpack('>f', qfbin[8:12])[0]
            self._quaternion_qgrad2[3] = struct.unpack('>f', qfbin[12:16])[0]
        except serial.SerialTimeoutException:
            print("timeout!  bad command?: " + str(Sample.get_qgrad2_quaternion_command))

        # debug
        self._source.write(b":33\n")
        print("<< received gyros//raw: " + str(self._source.readline()))

        # Get Normalized Gyro Values
        self._source.write(Sample.get_gyros_command.bin())
        try:
            qfbin = self._source.read(12)
            self._gyros[0] = struct.unpack('>f', qfbin[0:4])[0]
            self._gyros[1] = struct.unpack('>f', qfbin[4:8])[0]
            self._gyros[2] = struct.unpack('>f', qfbin[8:12])[0]
            print(">> from command: " + str(Sample.get_gyros_command))
            print("<< received gyros//raw: " + str(self._gyros))
        except serial.SerialTimeoutException:
            print("timeout!  bad command?: " + str(Sample.get_gyros_command))

        # Get Normalized Accelerometer values
        self._source.write(Sample.get_accel_command.bin())
        try:
            qfbin = self._source.read(12)
            self._accelerations[0] = struct.unpack('>f', qfbin[0:4])[0]
            self._accelerations[1] = struct.unpack('>f', qfbin[4:8])[0]
            self._accelerations[2] = struct.unpack('>f', qfbin[8:12])[0]
        except serial.SerialTimeoutException:
            print("timeout!  bad command?: " + str(Sample.get_accel_command))


# Press Shift+F10 to execute it or replace it with your code.
# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    serial_port = "COM3"

    try:
        # open serial port
        with serial.Serial(serial_port, 115200, timeout=0.1) as ahrs:
            # check which port was really used
            print(">> success! Opening Serial Port: " + ahrs.name)

            with open("../ahrs.csv", "w") as outfilehandle:
                current_sample = Sample(ahrs)
                writer = csv.DictWriter(outfilehandle,
                                        dialect=csv.unix_dialect,
                                        fieldnames=current_sample.titles(),
                                        quoting=csv.QUOTE_MINIMAL)
                writer.writeheader()

                try:
                    while True:
                        current_sample.update()

                        writer.writerow(current_sample.row())

                        break

                        # run at @ ~100 Hz
                        time.sleep(0.01)

                except KeyboardInterrupt:
                    print('Break Signalled -- Exiting.')

            # close AHRS com port
            ahrs.close()

    except serial.serialutil.SerialException as ex:
        print(f"Could not open Serial Port: {serial_port}")
        print("... (under windows, try the 'mode' command)")
        print("... Exiting.")
        exit(-2)

    # exit main
    exit(0)
