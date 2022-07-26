import os
import math
import copy
import time
import inspect
import numpy as np
import matplotlib.pyplot
from cmd import Cmd
from pyfiglet import Figlet
from colorama import Fore as color
from joint_drive_.joint_drive import JointDrive
from joint_drive_.dynamixel import Dynamixel 

# Constants
__NUMBER_OF_SERVO_IDS = 20

# Submenu 1
# ------------------------------------------------
class SubCmdSet(Cmd):
    prompt = 'root#@set# '

    def do_id(self, args):
        """Set id"""
        MyPrompt.servo = JointDrive(int(args))                                 
        f = open("demo.txt", "w+")
        f.write(args)
        f.close()

    def do_angle(self, args):
        """Set angle in radian -2.6 to +2.6"""
        MyPrompt.servo.set_goal_position(float(args))

    def do_speed(self, args):
        """Set speed in percent 0 - 100"""
        if int(args) > 100 or int(args) < 0:
            print("Invalid speed value")
            return True

        ticks = JointDrive._LIMITED_PERC_UNIT * float(args)
        rpm = math.floor(ticks * JointDrive._LIMITED_SPEED_UNIT)
        MyPrompt.servo.set_speed_rpm(rpm, True)

    def do_clear(self, args):
        """Clear screen"""
        MyPrompt.cmd_ui(MyPrompt)

    def do_quit(self, args):
        """Quit current submenu"""
        print()
        return True

# Submenu 2
# ------------------------------------------------
class SubCmdShow(Cmd):
    prompt = 'root#@show# '
    __NUMBER_OF_SERVO_IDS = 25

    def do_devices(self, args):
        """List avaiable devices"""
        id_lst = []

        for id in range(0, self.__NUMBER_OF_SERVO_IDS):
            id_ = JointDrive.ping(id)
            if len(id_) != 0:
                id_lst.append(id_[2])

        print()
        print("Devices found:")
        print("---------------")
        for id in id_lst:
            print("Servo ID: ", id)
        print()

    def do_clear(self, args):
        """clear screen"""
        MyPrompt.cmd_ui(MyPrompt)

    def do_quit(self, args):
        """Quit current submenu"""
        print()
        return True

# Submenu 3
# ------------------------------------------------
class SubCmdSync(Cmd):
    prompt = 'root#@sync# '

    def __init__(self):
        """Foo"""
        super().__init__()
        self.data = []
        self.sync_ui()
        self.exists = False

    def sync_ui(self, id = 'none'):
        """Foo"""
        # os.system('cls')
        f = Figlet(font='slant')
        print(f.renderText('Sync Mode'))

        default_len = 2
        table_content = ['ID', 'Angle', 'Speed']

        print("*" + (' ' * 5) + 'Data Packet No. x', end = '')
        [print((' ' * default_len) + '|' + (' ' * default_len) + content, end='') for content in table_content]
        print()
        print('-' * f.width)

        for index, data in enumerate(self.data):
            print("*" 
             + (' ' * 5) + 'Data Packet No. %s' % (index + 1)
             + (' ' * default_len) + '|' + (' ' * (default_len + (len(str(self.data[index][0]))) % default_len)) + '%s' % self.data[index][0]
             + (' ' * default_len) + '|' + (' ' * (default_len + 1 + (len(str(self.data[index][1]))) % default_len)) + '%s' % self.data[index][1]
             + (' ' * (default_len + 2)) + '|' + (' ' * (default_len + (len(str(self.data[index][2]))) % default_len)) + '%s' % self.data[index][2]
             )
        print('-' * f.width)
        print()
        print('Expected command -- \'build\' id angle speed')
        print()

    def do_run(self, args):
        """Execute sync write"""
        _data = copy.deepcopy(self.data)
        for data in _data:
            data[0] = JointDrive(data[0])
        JointDrive.set_angle_speed_sync(*_data)

    def do_build(self, args):
        """Build package: --build -id -angle -speed"""
        arg_lst = args.split(' ')

        for i in self.data:
            if i[0] == int(arg_lst[0]):
                self.exists = True
                i[1] = float(arg_lst[1])
                i[2] = float(arg_lst[2])

        if not self.exists:
            self.data.extend([[int(arg_lst[0]), float(arg_lst[1]), float(arg_lst[2])]])
            self.exists = False

    def do_update(self, args):
        """Update current screen"""
        os.system('cls')
        self.sync_ui()

    def do_quit(self, args):
        """Quit current submenu"""
        print()
        return True

# Main entry point menu
# ------------------------------------------------
class MyPrompt(Cmd):
    _MIN_RPM_ANGLE = -(150*2*math.pi)/360
    _MAX_RPM_ANGLE = (150*2*math.pi)/360
    servo = JointDrive(0) 

    # User Interface on CMD
    # ===============================================================================
    # ...
    # -------------------------------------------------------------------------------
    def cmd_ui(self, id = -1, firmware = -1, baud = -1, rdt = -1, srl = -1, gp = -1, speed = -1, temp = 0):
        """User interface"""
        os.system('cls')
        f = Figlet(font='slant')
        print(f.renderText('Servo Demo'))
        print('-' * f.width)
        print("*" + (' ' * 5) + 'Servo ID: %s' % id)
        print("*" + (' ' * 5) + 'Firmware Version: %s' % hex(firmware))
        print("*" + (' ' * 5) + 'Baudrate: %s Mbps' % baud)
        print("*" + (' ' * 5) + 'Return Delay Time: %s' % rdt)
        print("*" + (' ' * 5) + 'Status Return Level: %s' % srl)
        print("*" + (' ' * 5) + 'Goal Position (radian): %s' % gp)
        print("*" + (' ' * 5) + 'Moving Speed (rpm): %s' % speed)
        print("*" + (' ' * 5) + 'Temperature: %s°C' % temp)
        print('-' * f.width)
        print()
        print('Use \'help\' to see available commands')

    def action_ui(self, id = -1, firmware = -1, baud = -1, rdt = -1, srl = -1, gp = -1, speed = -1, temp = 0):
        """Updating user interface on action command"""
        os.system('cls')
        f = Figlet(font='slant')
        print(f.renderText('Active Demo'))
        print('-' * f.width)
        print("*" + (' ' * 5) + 'Servo ID: %s' % id)
        print("*" + (' ' * 5) + 'Firmware Version: %s' % hex(firmware))
        print("*" + (' ' * 5) + 'Baudrate: %s Mbps' % baud)
        print("*" + (' ' * 5) + 'Return Delay Time: %s' % rdt)
        print("*" + (' ' * 5) + 'Status Return Level: %s' % srl)
        print("*" + (' ' * 5) + 'Current Angle (radian): ' + (color.RED + '%s' + color.RESET) % gp)
        # print("*" + (' ' * 5) + 'Current Speed (rpm): ' + (color.RED + '%s' + color.RESET) % speed)
        print("*" + (' ' * 5) + 'Temperature: ' + (color.RED + '%s' + color.RESET + '°C') % temp)
        print('-' * f.width)
        print()
        time.sleep(1)

    # Do Functions
    # ===============================================================================
    # Commands displayed on CMD UI
    # -------------------------------------------------------------------------------
    def do_action(self, *args):                   
        """Execute servo commands"""
        self.servo.action()
        moving_status = self.servo._request_n_byte(0x2E)[0]

        while moving_status is 1:
            id = self.servo.id
            firmware = self.servo._request_n_byte(0x02)[0]
            baud = self.servo._request_n_byte(0x04)[0]
            rdt = self.servo.get_return_delay()
            srl = self.servo.get_return_level()
            gp = self.servo.get_present_position()
            speed = self.servo.get_present_speed()
            temp = self.servo.get_present_temperature()
            self.action_ui(id, firmware, baud, rdt, srl, gp, speed, temp)
            moving_status = self.servo._request_n_byte(0x2E)[0]
            time.sleep(0.2) 
        self.do_update()

    def do_update(self, args=0):
        """Update menu screen"""
        if self.servo.id == 0:
            self.do_clear(args) 
        else:
            id = self.servo.id
            firmware = self.servo._request_n_byte( 0x02)[0]
            baud = self.servo._request_n_byte(0x04)[0]
            rdt = self.servo.get_return_delay()
            srl = self.servo.get_return_level()
            gp = self.servo.get_goal_position()
            speed = self.servo.get_moving_speed()
            temp = self.servo.get_present_temperature()
            self.cmd_ui(id, firmware, baud, rdt, srl, gp, speed, temp)

    def do_set(self, args):
        """Set various things"""
        sub_cmd_set = SubCmdSet()
        sub_cmd_set.cmdloop()
        # self.do_update(args)
        # self.helper(inspect.currentframe().f_code.co_name)

    def do_show(self, args):
        """Show various things"""
        sub_cmd_show = SubCmdShow()
        sub_cmd_show.cmdloop()
        # self.cmd_ui()

    def do_sync(self, args):
        """Syn Write Mode"""
        sub_cmd_sync = SubCmdSync()
        sub_cmd_sync.cmdloop()
        self.cmd_ui()
        # self.do_update(args)

    def do_rpm(self, args):
        """RPM Messung und Plot"""
        fig = matplotlib.pyplot.figure()
        ax = fig.add_subplot(1,1,1)
        rpm = []
        t = []

        _speed = self.servo.get_moving_speed()
        time.sleep(2)
        # Startzustand einnehmen
        self.servo.set_desired_angle_speed(self._MIN_RPM_ANGLE, 114, True)      # Startposition
        moving_status = self.servo.get_moving_status()                          # Warte bis Start erreicht
        while moving_status is 1:
            moving_status = self.servo.get_moving_status()                      # moving_status = self.servo._request_n_byte(0x2E)[0]  
        time.sleep(2)

        # Messung
        self.servo.set_desired_angle_speed(self._MAX_RPM_ANGLE, _speed, True)   # Fahre an Endposition
        start_time = time.perf_counter()
        moving_status = self.servo.get_moving_status()
        while moving_status is 1:
            rpm.append(self.servo.get_present_speed())
            _t = time.perf_counter() - start_time
            t.append(format(_t, '.2f'))
            moving_status = self.servo.get_moving_status()
            time.sleep(0.02)
        
        # Plotting Data
        # ------------------------
        rpm_mean = [np.mean(rpm)] * len(t)
        ax.plot([x for x in t], [y for y in rpm],  'c-')
        ax.plot(t, rpm_mean, 'r--')

        # Legende
        ax.set_title('Servo RPM Verlauf')
        ax.legend(['current speed', 'rpm mean: %.2f' % rpm_mean[0]])
        ax.set_xlabel('Zeit in s')
        ax.set_ylabel('Geschwindigkeit in rpm')
        matplotlib.pyplot.savefig('rpm.png')
        matplotlib.pyplot.show()

    def do_pytest(self, args):
        """Run predefined pytest -- ! Use this only with two actuators connected !"""
        if Dynamixel._Dynamixel__serial_port.isOpen():
            Dynamixel._Dynamixel__serial_port.close()
        myCmd = 'py.test -v'
        os.system(myCmd)
        Dynamixel._Dynamixel__serial_port.open()

    def do_clear(self, args):
        """Clear screen"""
        self.cmd_ui()

    def do_quit(self, args):
        """Quits the program."""
        if os.path.exists('demo.txt'):
            os.remove('demo.txt')
        print("Quitting...")
        os.system('exit')
        return True
        # raise SystemExit

    # Helper Functions
    # ------------------------------------------------
    def helper(self, func_name, intern = 'none'):
        print("Invalid command")
        print()
        print("Use \'? %s\' to see available commands" % func_name[3:])
        # print("functions: ", intern)

    def help_show(self):
        print()
        print("Documented commands (type help <topic>)")
        print("========================================")
        lst = [(name, t) for name, t in SubCmdSet.__dict__.items() if type(t).__name__ == 'function' and not name.startswith('__')]
        [print(i[0][3:], end=' ') for i in lst]
        print()
        print()

    def help_set(self):
        print()
        print("Documented commands (type help <topic>)")
        print("========================================")
        lst = [(name, t) for name, t in SubCmdSet.__dict__.items() if type(t).__name__ == 'function' and not name.startswith('__')]
        [print(i[0][3:], end=' ') for i in lst]
        print()
        print()

if __name__ == '__main__':
    prompt = MyPrompt()
    prompt.cmd_ui()
    prompt.prompt = 'root# '
    prompt.cmdloop()