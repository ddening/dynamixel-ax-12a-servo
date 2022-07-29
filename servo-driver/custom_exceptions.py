import logging
import sys

# Dynamixel Custom Exceptions
# ---------------------------------------------------------------------------
class DynamixelError(Exception):
    """Generic exception for dynamixel to log exceptions"""

    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    fh = logging.FileHandler('debug.log')
    fh.setLevel(logging.DEBUG)
    logger.addHandler(fh)
    formatter = logging.Formatter('%(message)s')
    fh.setFormatter(formatter)
    logger.info("--------------------------------------------------------")
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    def __init__(self):
        pass

    def log(self, c_name, f_name = "no info avaiable"):
        self.logger.debug(c_name)
        self.logger.debug("Error occured in function: " + f_name)

class InstructionError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class OverloadError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class ChecksumError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class RangeError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class OverheatingError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class AngleLimitError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class InputVoltageError(DynamixelError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

# JointDrive Custom Exceptions
# ---------------------------------------------------------------------------
class JointDriveError(Exception):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class MemoryAccessError(JointDriveError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])

class PacketTypeError(JointDriveError):
    def __init__(self, *args):
        self.log(self.__class__, args[0])
