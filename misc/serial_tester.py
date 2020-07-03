import serial
import time

if __name__ == "__main__":
    while True:
        for freq in [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 128000, 230400, 256000, 460800, 500000, 675000, 921600]:
            test_string = b"abcdefghijklmnopqrstuvwxyz1234567890ABCDEFGHIJKLMNOPQRSTUVWXYZ~!@#$%^&*();:'{}[]|,./'"
            for bytesize in [7,8]:
                for parity in [serial.PARITY_NONE, serial.PARITY_EVEN, serial.PARITY_ODD]:
                    for stopbits in [serial.STOPBITS_ONE, serial.STOPBITS_TWO]:

                        print(freq, end=' ')

                        print(bytesize, end="")

                        if parity == serial.PARITY_NONE:
                            print("N", end="")
                        elif parity == serial.PARITY_EVEN:
                            print("E", end="")
                        else:
                            print("O", end="")

                        if stopbits == serial.STOPBITS_ONE:
                            print("1", end="")
                        else:
                            print("2", end="")
                        print()

                        with serial.Serial('/dev/ttyUSB0', baudrate=freq, timeout=1, parity=parity,
                                           stopbits=stopbits, bytesize=bytesize) as ftdi:
                            with serial.Serial('/dev/ttyACM0', baudrate=freq, timeout=1, parity=parity,
                                               stopbits=stopbits, bytesize=bytesize) as turtle:

                                turtle.flushInput()
                                ftdi.flushInput()

                                ftdi.write(test_string)
                                result = turtle.read(len(test_string))
                                print("ftdi->turtle received:", result)
                                assert result == test_string

                                turtle.write(test_string)
                                result = ftdi.read(len(test_string))
                                print("turtle->ftdi received:", result)
                                assert result == test_string

                        print()