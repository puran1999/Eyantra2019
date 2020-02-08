import sys
import glob
import serial


def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

'''
if __name__ == '__main__':
    ports = serial_ports()
    print(ports)
    for port in ports:
        try:
            s = serial.Serial(port)
            print(s.get_settings())
            #s.reset_input_buffer()
            #s.reset_output_buffer()
            while True:
                s.write(
            num = 0
            ack = s.read(1).decode("utf-8")
            print(ack)
            if ack == 1:
                while num < 10:
                    s.write(bytes(str(num).encode("utf-8")))
                    num = num + 1
            else:
                s.write(b'x')
            #res = s.read(1).decode("utf-8")
            #print(res)
            print(s.in_waiting)
            print(s.out_waiting)
            s.close()
        except (OSError, serial.SerialException):
            pass
'''
