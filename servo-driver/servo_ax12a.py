from joint_drive_.dynamixel import Dynamixel

class ServoAx12a(Dynamixel):
    """Definition of Servo-Ax12a-controller class, defines control and status methods

    Implements communication commands to servo AX12A
    Uses the commands of Dynamixel class to send and receive the required servo values

    """

    # Definition of private class attributes
    # ----------------------------------------------------------------------
    # EEPROM communication registers addresses
    __RETURN_DELAY_TIME = 0x05                      # time of return delay (n * 2 in [uS]), 1 byte, read/write
    __RETURN_LEVEL = 0x10                           # status return level, 1 byte, read/write
    # RAM communication registers addresses
    __GOAL_POSITION = 0x1E                          # goal position, 2 byte, read/write
    __MOVING_SPEED = 0x20                           # moving speed, 2 byte, read/write
    __PRESENT_POSITION = 0x24                       # current position, 2 byte, read only
    __PRESENT_SPEED = 0x26                          # current speed, 2 byte, read only
    __CW_ANGLE_LIMIT = 0x06
    __CCW_ANGLE_LIMIT = 0x08
    __PRESENT_TEMPERATURE = 0x2B
    __PRESENT_VOLTAGE = 0x2A
    __MOVING_STATUS = 0x2E
    __PRESENT_VOLTAGE = 0x2A

    # Definition of protected class attributes
    # ----------------------------------------------------------------------
    # Servo constants
    _ANGLE_MAX_TICKS = 1023                         # ticks at highest position (330 degree)
    _ANGLE_MIN_TICKS = 0                            # ticks at lowest position (30 degree)
    _ANGLE_MAX_DEGREE = 300                         # highest angle reachable is 330 degree
    _ANGLE_MIN_DEGREE = 0                           # lowest angle reachable is 30 degree
    _SPEED_UNIT = 0.111                             # 0.111 rpm per tick
    _SPEED_MAX_TICKS = 1023                         # 1023
    _SPEED_MAX_RPM = 1023 * 0.111                   # 1023 * 0.111 = 113.5 rpm
    _SPEED_PERC_UNIT = _SPEED_MAX_TICKS / 100       # 10.23 ticks ^= 1%
    # User defined constants for limited rpm level
    _LIMITED_MAX_RPM = 80
    _LIMITED_MAX_TICKS = int((_ANGLE_MAX_TICKS / _SPEED_MAX_RPM) * _LIMITED_MAX_RPM)
    _LIMITED_SPEED_UNIT = _LIMITED_MAX_RPM / _LIMITED_MAX_TICKS
    _LIMITED_PERC_UNIT = _LIMITED_MAX_TICKS / 100   # 7.21 ticks ^= 1%


    # Definition of public class attributes
    # ----------------------------------------------------------------------
    # Definition of valid return levels
    RETURN_LEVEL_PING_COMMAND = 0                   # Status packet is only returned for pong
    RETURN_LEVEL_READ_COMMANDS = 1                  # Status packet is returned for ping and read requests
    RETURN_LEVEL_ALL_COMMANDS = 2                   # Status packet is returned for ping, read requests and writes
    # Defines the wait time the servo switches from receive to send mode
    # when a read request packet was received                                             
    RETURN_DELAY_VALUE = 10                         # Definition of return delay time, value * 2 -> [uS]

    # Definition of private methods
    # ----------------------------------------------------------------------
    def __init__(self, id_):
        """Constructor, return level and return delay are set"""
        super().__init__(id_)

    def __merge_two_bytes(self, a, b):
        """Merge two bytes"""
        a = a * 256
        a = a + b
        return a

    def __split_word_in_bytes(self, word):
        """Split word in bytes to be able to use servo commands"""
        a = word & 255
        b = word >> 8  
        return [a, b]

    # Getter methods for servo Ax12a
    # ----------------------------------------------------------------------
    def get_return_delay(self):
        """Get time of return delay

        Returns:
            delay time (int): value range 0 to 254 (0xFE) can be used, and the delay time per data value is 2 usec. 
        """
        return self._request_n_byte(self.__RETURN_DELAY_TIME)[0]
        
    def get_return_level(self):
        """Get status return level

        Returns:
            0 -> no return against all commands (except PING command)
            1 -> return only for the READ command
            2 -> return for all commands
        """
        return self._request_n_byte(self.__RETURN_LEVEL)[0]

    def get_goal_position(self):
        """Get goal position

        Returns:
            value range 0 to 1023, the unit is 0.29 degree.
        """
        return_data = self._request_n_word(self.__GOAL_POSITION)
        return self.__merge_two_bytes(return_data[1], return_data[0])

    def get_moving_speed(self):
        """Get moving speed set to reach the goal position

        Returns:
            value range 0 to 1023, the unit is about 0.111 rpm.
            If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
            If it is 1023, it is about 114rpm.
        """
        return_data = self._request_n_word(self.__MOVING_SPEED)
        return self.__merge_two_bytes(return_data[1], return_data[0])

    def get_present_position(self):
        """Get present position

        Returns:
            value range 0 to 1023, the unit is 0.29 degree
        """
        return_data = self._request_n_word(self.__PRESENT_POSITION)
        return self.__merge_two_bytes(return_data[1], return_data[0])

    def get_present_speed(self):
        """Get present speed

        Returns:
            value range 0 to 1023, the unit is about 0.111 rpm.
            If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
            If it is 1023, it is about 114rpm.
        """
        return_data = self._request_n_word(self.__PRESENT_SPEED)
        return self.__merge_two_bytes(return_data[1], return_data[0])
        
    def get_goal_pos_speed(self):
        """Get goal position and speed

        Returns:
            position (int): value range 0 ~ 1023
            speed (int):  value range 0 ~ 1023, the unit is about 0.111 rpm.
                    If it is 0, it means the maximum rpm of the motor is used without controlling the speed.
                    If it is 1023, it is about 114rpm.
        """
        return_data = self._request_n_word(self.__GOAL_POSITION)
        pos = self.__merge_two_bytes(return_data[1], return_data[0])

        return_data = self._request_n_word(self.__MOVING_SPEED)
        speed = self.__merge_two_bytes(return_data[1],return_data[0])

        return [pos, speed]
        
    def get_present_pos_speed(self):
        """Get present position and speed

        Returns:
            position (int): value range 0 ~ 1023
            speed (int):  value range 0 ~ 1023, the unit is about 0.111 rpm.
                    If it is 0, it means the maximum rpm of the motor is used without controlling the speed.
                    If it is 1023, it is about 114rpm.
        """
        return_data = self._request_n_word(self.__PRESENT_POSITION)
        pos = self.__merge_two_bytes(return_data[1], return_data[0])

        return_data = self._request_n_word(self.__PRESENT_SPEED)
        speed = self.__merge_two_bytes(return_data[1],return_data[0])
        return [pos, speed]

    def get_moving_status(self):
        """Get current moving status
        
        Returns: 
            1 -> moving, 0 -> not moving
        """
        return self._request_n_byte(self.__MOVING_STATUS)[0]
        
    def get_present_temperature(self):
        """Get present temperature of servo

        Returns:
            Temperature in degree celsius
        """
        return self._request_n_byte(self.__PRESENT_TEMPERATURE)[0]

    def get_present_voltage(self):
        """Get present voltage of servo

        Returns:
            Temperature in degree celsius
        """
        return self._request_n_byte(self.__PRESENT_VOLTAGE)[0]

    # Setter methods for servo Ax12a
    # ----------------------------------------------------------------------
    def set_return_delay(self, delay, trigger=True):
        """Set time of return delay

        Args:
            delay (int): 0 to 254 (0xFE) can be used, and the delay time per data value is 2 usec.
            trigger (boolean): (Default value = True, execute immediately)
            False: wait for trigger command
        """
        self._write_n_byte_pkt(self.__RETURN_DELAY_TIME, [delay], trigger)
        
    def set_return_level(self, level, trigger=True):
        """Set status return level

        Args:
            level (int):  
                0 -> no return against all commands (except PING command)
                1 -> return only for READ command
                2 -> return for all commands
            trigger (boolean): (Default value = True, execute immediately)
            False: wait for trigger command
        """
        if self.return_level == 1 and level == 2:
            switch_level = True
        else:
            switch_level = False 
        self._write_n_byte_pkt(self.__RETURN_LEVEL, [level], trigger, switch_level)
        self.return_level = level

    def set_goal_position(self, position, trigger=False):
        """Set goal position

        Args:
            position (int): 0 to 1023 is available. The unit is 0.29 degree.
            trigger (boolean): (Default value = False, wait for trigger command)
            True: execute immediately
        """
        self._write_n_word_pkt(self.__GOAL_POSITION, [position], trigger)

    def set_moving_speed(self, speed, trigger=False):
        """Set moving speed

        Args:
            speed (int):  
                0 to 1023 can be used, and the unit is about 0.111 rpm.
                If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
                If it is 1023, it is about 114rpm.
            trigger (boolean): (Default value = False, wait for trigger command)
            True: execute immediately
        """
        self._write_n_word_pkt(self.__MOVING_SPEED, [speed], trigger)

    def set_goal_pos_speed(self, position, speed, trigger=False):
        """Set goal position and speed

        Args:
            position (int): 0 to 1023 is available. The unit is 0.29 degree.
            speed (int): 0 to 1023 can be used, and the unit is about 0.111 rpm.
                   If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
                   If it is 1023, it is about 114rpm
            trigger (boolean): (Default value = False, wait for trigger command)
            True: execute immediately
        """
        self._write_n_word_pkt(self.__GOAL_POSITION, [position], trigger)
        self._write_n_word_pkt(self.__MOVING_SPEED, [speed], trigger)

    def set_cw_angle_limit(self, angle, trigger=True):
        """Set clockwise angle limit

        Args:
            angle (int): (Default value = 0), value range 0 to 1023
        """
        self._write_n_word_pkt(self.__CW_ANGLE_LIMIT, [angle], trigger)

    def set_ccw_angle_limit(self, angle, trigger=True):
        """Set counter-clockwise angle limit

        Args:
            angle (int): (Default value = 0), value range 0 to 1023
        """
        self._write_n_word_pkt(self.__CCW_ANGLE_LIMIT, [angle], trigger)