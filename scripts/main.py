#!/usr/bin/python3

# standard library
import csv
import sys
import time

# pip packages
import serial


class Command:
    def __init__(self, command=0, data=''):
        # print("Building command...")
        text = 'f7{:0>2X}{}'.format(command, data)
        # print("... from command #{:>3} := {}".format(int(command), text))

        # The checksum is computed as an arithmetic summation of all of the characters in the packet (except the checksum value
        # itself) modulus 256. This gives a resulting checksum in the range 0 to 255. The checksum for binary packets is
        # transmitted as a single 8-bit byte value.
        checksum = sum(bytes.fromhex(text[2:]))
        # print("... from              := {}".format(text[2:]))
        text += '{:0>2X}'.format(checksum)
        # print("... with checksum     := {}".format(text))

        self._binary_command = bytes.fromhex(text)

    def __repr__(self):
        line = '{:X} '.format(self._binary_command[0])  # sync byte
        line += '{:0>2X}'.format(self._binary_command[1])  # command id
        line += ''.join(['{:0>2X}'.format(b) for b in self._binary_command[2:-1]])  # data bytes
        line += ' {:0>2X}'.format(self._binary_command[-1]) # checksum
        return line

    @property
    def cmd(self):
        return str(hex(self._binary_command[1]))


class Sample:

    def __init__(self, ahrs):
        self._source = ahrs

        self.time = 0
        self.placeholder = 0
        # self.raw =
        # self.qgrad2 =

        self.n = ''

    def __repr__(self):
        return f'[{self.time}]: {self.placeholder}'

    @staticmethod
    def titles():
        return Sample().__dict__.keys()

    def update(self):
        """
        source: https://yostlabs.com/wp/wp-content/uploads/pdf/3-Space-Sensor-Users-Manual-Nano.pdf
        """

        ## command:
        self._source.write(b'hello')

        self._source.readline()

        self.time = 1
        self.placeholder = "duck"


# Press Shift+F10 to execute it or replace it with your code.
# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    serial_port = "COM3"

    tc = Command(1, '1111')

    print(tc)
    print("== " + str(tc))

    exit(0)


    try:
        # open serial port
        with serial.Serial(serial_port, 115200) as ahrs:
            print("::success::")
            print(ahrs.name)  # check which port was really used

            with open("../ahrs.csv", "w") as outfilehandle:
                current_sample = SampleSet(ahrs)
                writer = csv.DictWriter(outfilehandle,
                                        dialect=csv.unix_dialect,
                                        fieldnames=current_sample.titles(),
                                        quoting=csv.QUOTE_NONE)
                writer.writeheader()

                try:
                    while True:
                        current_sample.update()

                        print(current_sample)

                        writer.writerow(current_sample.__dict__)

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
