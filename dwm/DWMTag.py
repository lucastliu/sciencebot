import serial
import time
import datetime

class DWMTag():
    def __init__(self, port_name="/dev/ttyACM1"):
        #defualt port name from my computer. Different on differnt devices.
        '''
        Opens the serial connection at the given port and writes commands
        to read position from the tag.
        Waits until one position reading is successful before deeming connection successful
        '''
        self.DWM = serial.Serial(port=port_name, baudrate=115200)
        self.DWM.write("\r\r".encode())
        time.sleep(1)
        self.DWM.write("lec\r".encode())
        time.sleep(1)
        is_not_available = True
        while is_not_available:
            line=self.DWM.readline()
            if(line):
                if len(line)>=5:
                    parse=line.decode().split(",")
                    try:
                        pos_ind = parse.index("POS")
                        if pos_ind is not None:
                            is_not_available = False
                    except Exception as ex:
                        print('connecting...')
        print("Connected to " +self.DWM.name)
        self.x_position = 0.00
        self.y_position = 0.00
        self.curr_time = curr_time = datetime.datetime.now().strftime("%H:%M:%S.%f")

    def update_position(self):
        '''
        Calling this function reads the line over the serial connection
        and parses it to update the variables
        self.x_position
        self.y_position
        self.curr_time
        '''
        try:
            line=self.DWM.readline()
            if(line):
                if len(line)>=5:
                    parse=line.decode().split(",")
                    if parse.index("POS"):
                        pos_ind = parse.index("POS")
                        curr_time = datetime.datetime.now().strftime("%H:%M:%S.%f")
                        x_pos = parse[pos_ind+1]
                        y_pos = parse[pos_ind+2]

                        self.x_position = float(x_pos)
                        self.y_position = float(y_pos)
                        self.curr_time = curr_time
                        # print('updated_position to: ', self.x_position, self.y_position)

            else:
                print("Distance not calculated: ",line.decode())
        except Exception as ex:
            x = 3
            # print('position unavailable')
            # print(ex)

    def get_pos(self):
        '''
        return the most recently position of the tag in the form of a list:
        [x_pos, y_pos]
        where x_pos is a float, y_pos is a float.
        Ex:
        [1.79, 1.80]
        '''
        # print('get_pos DWMTAG: ', self.x_position, self.y_position)
        return [self.x_position, self.y_position]

    def get_time(self):
        '''
        returns the time at which the most recent position reading was taken
        '''
        return self.curr_time

    def close_serial(self):
        '''
        Closes the serial connection.
        '''
        self.DWM.write("\r".encode())
        self.DWM.close()
        print('Serial connection closed')
        return
