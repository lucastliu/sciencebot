import serial

class SerialMotor:
    def __init__(self, port, baud=9600):
        """Opens a serial port connection with the specified port."""
        self.port = serial.Serial(port, baud)

    def set_motor(self, motor_port, power):
        """Sends a message to this serial port of the form "z<motor_port> <power>\n", with power to two decimal places."""
        self.port.write(("z%d %.2f\n" % (motor_port, power)).encode("utf-8"))
