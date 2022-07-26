import math
import numpy
import inspect
import custom_exceptions as excep
from custom_decorator import *
from joint_drive_.servo_ax12a import ServoAx12a


class JointDrive(ServoAx12a):
    """Class definition of ax12a-controller class, defines interface to the robot

    Implements the interface between leg- and servo class

    Provides all required methods that allow the leg class to control the servo
    Implements all necessary codomain conversion between leg- and servo values
    Limits values too valid servo values
    Servo uses ticks from 0 to 1023 for angle and speed
    Leg uses angles in radian and rotation per minute for speed
    Defines zero angle as average of min- and max value -> positive and negative angles are allowed
    """

    # Definition of public class attributes
    # ----------------------------------------------------------------------
    # Zero angle offset of servo in radian (+150°)
    _ANGLE_RADIAN_ZERO = (ServoAx12a._ANGLE_MAX_DEGREE - ServoAx12a._ANGLE_MIN_DEGREE) * math.pi / 360
    # _ANGLE_RADIAN_OFFSET = 30 * math.pi / 180
    # Ticks per rad
    _ANGLE_UNIT = ServoAx12a._ANGLE_MAX_TICKS / ((ServoAx12a._ANGLE_MAX_DEGREE - ServoAx12a._ANGLE_MIN_DEGREE)
                                                 * math.pi * 2 / 360)

    # Private methods    
    # ----------------------------------------------------------------------
    def __init__(self, id_, ccw=False, a_offset=0.0, a_max=300*(2*math.pi/360), a_min=0, return_level=2):
        """Constructor, defines the following variables

        Args:
            id: id of servo

        Attributes:
            cw: counterclockwise, rotating direction
            a_offest: angle offset in radian
            a_max: maximum angle allowed
            a_min: minimum angle allowed
        """
        super().__init__(id_)
        self.ccw = ccw
        self.a_offset = self._ANGLE_RADIAN_ZERO + a_offset
        self.a_max = a_max
        self.a_min = a_min
        self.savedgoalposition = 0
        self.set_return_level(self.return_level, True)
        self.set_cw_angle_limit(self.__convert_angle_to_ticks(a_min, False))
        self.set_ccw_angle_limit(self.__convert_angle_to_ticks(a_max, False))

    def __convert_angle_to_ticks(self, angle, ccw, offset = 0):
        """Converts angle in radian to servo ticks

        Args:
            angle: in radian
            offset: (Default value = 0)

        Returns:
            ticks: servo ticks
        """
        angle = angle + offset
        ticks = int(angle * self._ANGLE_UNIT)
        if ccw:
            ticks = 1023 - 1 - ticks
        return ticks

    def __convert_ticks_to_angle(self, ticks, ccw, offset = 0):
        """Converts servo ticks to angle in radian

        Args:
            ticks: servo ticks
            offset: (Default value = 0)
        
        Returns:
            angle in radian
        """
        if ccw:
            ticks = 1023 - ticks
        angle = ticks / self._ANGLE_UNIT
        angle = angle - offset
        return angle

    def __convert_speed_to_ticks(self, speed):
        """Converts speed in rpm to servo ticks

        Args:
            speed: speed in rpm (0 to 113.5)

        Returns:
            servo ticks
        """
        speed = numpy.clip(speed, 0, ServoAx12a._SPEED_MAX_RPM)
        ticks = speed / ServoAx12a._SPEED_UNIT
        ticks = math.floor(ticks + 0.5)

        return ticks

    def __convert_ticks_to_speed(self, ticks):
        """Converts ticks to speed in rpm

        Args:
            ticks: servo ticks (0 to 1023)

        Returns:
            speed in rpm
        """
        ticks = numpy.clip(ticks, 0, ServoAx12a._SPEED_MAX_TICKS)
        speed = math.floor(ticks * ServoAx12a._SPEED_UNIT)
        return speed

    def __convert_percent_speed_to_ticks(self, perc):
        """Converts given speed value in percent to servo ticks

        Args:
            perc: value range 0 to 100

        Returns:
            ticks: servo ticks (0 to LIMITED_MAX_TICKS)
        """
        perc = numpy.clip(perc, 0, 100)
        ticks = math.floor(perc * ServoAx12a._LIMITED_PERC_UNIT)
        return ticks

    def __convert_ticks_to_voltage(self, ticks):
        return ticks / 10

    # Public Getter methods
    # ----------------------------------------------------------------------
    def get_present_position(self):
        """Get current angle of servo

        Returns:
            angle (float): angle in radian
        """
        ticks = super().get_present_position()
        return self.__convert_ticks_to_angle(ticks, self.ccw, self.a_offset)

    def get_present_voltage(self):
        """Get current voltage of servo

        Returns:
            voltage (float):
        """
        ticks = super().get_present_voltage()
        return self.__convert_ticks_to_voltage(ticks)


    def get_goal_position(self):
        """Get saved goal position

        Returns:
            angle (float): angle in radian
        """
        return_data = super().get_goal_position()
        angle = self.__convert_ticks_to_angle(return_data, self.ccw, self.a_offset)
        return angle

    def get_present_pos_speed(self):
        """Get present position and present speed

        Returns:
            angle, speed (lst): angle in radian, speed in rpm
        """
        return_data = super().get_present_pos_speed()
        angle = self.__convert_ticks_to_angle(return_data[0], self.ccw, self.a_offset)
        speed = self.__convert_ticks_to_speed(return_data[1], self.ccw)
        return [angle, speed]

    def get_goal_pos_speed(self):
        """Get saved goal position and speed

        Returns:
            angle, speed (lst): angle in radian, speed in rpm
        """
        return_data = super().get_goal_pos_speed()
        angle = self.__convert_ticks_to_angle(return_data[0], self.ccw, self.a_offset)
        speed = self.__convert_ticks_to_speed(return_data[1], self.ccw)
        return [angle, speed]

    def get_moving_speed(self):
        """Get moving speed

        Returns:
            speed (int): speed in rpm
        """
        return_data = super().get_moving_speed()
        speed = self.__convert_ticks_to_speed(return_data)
        return speed
    
    def get_present_speed(self):
        """Get present speed

        Returns:
            speed (int): speed in rpm
        """
        return_data = super().get_present_speed()
        speed = self.__convert_ticks_to_speed(return_data)
        return speed

    def get_saved_goal_position(self):
        """Get saved goal position

        Returns:
            angle (int): angle in radian
        """
        return self.savedgoalposition

    def get_present_voltage(self):
        ticks = super().get_present_voltage()
        return ticks / 10

    # Public Setter methods
    # ----------------------------------------------------------------------
    @decorator_error_handling
    def set_speed_rpm(self, speed, trigger=False):
        """Set speed value of a single servo

        Args:
            speed (int): speed in rpm (0 to 113.5)
        """
        if speed > ServoAx12a._SPEED_MAX_RPM or speed < 0:
            print("Invalid speed value")
            return True
        speed_ticks = self.__convert_speed_to_ticks(speed)
        self.set_moving_speed(speed_ticks, trigger)

    @decorator_error_handling
    def set_speed_percent(self, perc, trigger=False):
        """Set speed value of a single servo

        Args:
            speed (int): speed in percent (0 to 100)
        """
        if perc > 100 or perc < 0:
            print("Invalid speed value")
            return True
        ticks = self.__convert_percent_speed_to_ticks(perc)
        self.set_moving_speed(ticks, trigger)

    @decorator_error_handling
    def set_goal_position(self, angle, trigger=False):
        """Set servo to desired angle

        Args:
            angle (float): angle in radian
        """
        savedgoalposition = angle
        if angle > (self.a_max - self.a_offset):
            print("Angle zu groß")
            return True
        elif angle < (self.a_min - self.a_offset):
            print("Angle zu klein")
            return True
        ticks = self.__convert_angle_to_ticks(angle, self.ccw, self.a_offset)
        super().set_goal_position(ticks, trigger)

    @decorator_error_handling
    def set_desired_angle_speed(self, angle, speed=0, trigger=False):
        """Set servo to desired angle and speed

        Args:
            angle (float): angle in radian
            speed (int): speed of movement in percent, speed = 0 -> maximum speed
        """
        self.savedgoalposition = angle
        if angle > (self.a_max - self.a_offset):
            print("Angle zu groß")
            return True
        elif angle < (self.a_min - self.a_offset):
            print("Angle zu klein")
            return True

        if speed > ServoAx12a._SPEED_MAX_RPM or speed < 0:
            print("Invalid speed value")
            return True
        
        angle_ticks = self.__convert_angle_to_ticks(angle, self.ccw, self.a_offset)    
        speed_ticks = self.__convert_percent_speed_to_ticks(speed)
        self.set_goal_pos_speed(angle_ticks, speed_ticks, trigger)

    # Static methods
    # ----------------------------------------------------------------------
    @staticmethod
    @decorator_error_handling
    def set_angle_speed_sync(*packet):
        """Set goal position and speed value for multiple servo actuators at once.

        Args:
            angle (float): angle in radian
            speed (int): speed in rpm

        Notes:
            Expected packet format: [jointdrive_object, angle, speed]
            Use this function to transmit commands to multiple servo actuators at once.
            This sync function executes the command immediately and does not return any status packet. 

        Example:
            JointDrive.set_angle_speed_sync([leg1.alpha_joint, 1, 100], [leg1.beta_joint, 2.6, 70], ..., [leg_n.m_joint, i, j])
        """
        if len(packet) == 0:                                                # no packet 
            return -1

        for data in packet:                                                 # convert each element in ticks for servo usage
            if isinstance(data, list):                                      # only accept lists
                data[0].savedGoalPosition = data[1]
                if len(data) > 3:                                           # prevent writing in unwanted memory address 
                    raise excep.MemoryAccessError(inspect.currentframe().f_code.co_name)
                elif len(data) < 3:                                         # data length has to be the same for all servos
                    raise excep.PacketTypeError(inspect.currentframe().f_code.co_name)
                else:
                    data[1] = data[0].__convert_angle_to_ticks(data[1], data[0].ccw, data[0].a_offset)
                    data[2] = data[0].___convert_percent_speed_to_ticks(data[2])
            else:
                raise excep.PacketTypeError(inspect.currentframe().f_code.co_name)

        ServoAx12a._sync_write(ServoAx12a._ServoAx12a__GOAL_POSITION, packet)