import serial
import time
import inspect
import custom_exceptions as excep
from custom_decorator import *
from joint_drive_ import serial_ports

class Dynamixel:
    """Classdefinition to implement dynamixel protocol

    Implements the dynamixel protocol 1.0

    Assigns the class object to a dedicated servo by the servo id
    Initializes the serial connection to the servo bus
    Handles the transfer of all required packet types with 1..n data bytes or -words

    """

    # Definition of protected class attributes
    # Accessible only within own and derived classes 
    # ---------------------------------------------------------------------------
    _ID_BROADCAST = 0xFE

    # Definition of private class attributes, accessible only within own class
    # ---------------------------------------------------------------------------
    # Define dynamixel constants
    __DYNAMIXEL_PORT_NR = 0                                                     # Index of dynamixel line in list
    __BAUDRATE = 1000000                                                        # Baudrate of dynamixel serial line
    __TIME_OUT_DEFAULT = 0.2                                                    # Default time out
    __DIRECT_ACTION = 3                                                         # Direct action command
    __TRIGGERT_ACTION = 4                                                       # Triggered action command
    __STATUS_PACKET_BASE_LENGTH = 6                                             # Base length of status packet
    __lines = serial_ports.serial_port_list()                                   # Contains all available serial lines
    __serial_port = serial.Serial(
        __lines[__DYNAMIXEL_PORT_NR],
        __BAUDRATE, timeout=__TIME_OUT_DEFAULT)                                 # Serial line object

    # Create templates of command packets 
    __pkt_ping = [255, 255, 0, 0x02, 0x01, 0]                                   # Packet to ping a servo
    __pkt_action = [255, 255, 0, 2, 5, 0]                                       # Packet to invoke action
    __pkt_read_data = [255, 255, 0, 4, 2, 0, 0, 0]                              # Packet to request data
    __pkt_write_byte = [255, 255, 0, 4, 3, 0, 0, 0]                             # Packet to write byte
    __pkt_write_n_byte = [255, 255, 0, 0, 3, 0]                                 # Base-packet to write n-bytes
    __pkt_write_word = [255, 255, 0, 5, 3, 0, 0, 0, 0]                          # Packet to write word
    __pkt_sync_write = [0xFF, 0xFF, 0xFE, 0, 0x83, 0, 0]                        # Base-packet to sync write

    # ---------------------------------------------------------------------------
    # Definition of private methods with implicit servo-id
    # Accessible only within own class
    # ---------------------------------------------------------------------------
    def __init__(self, id_):
        """Constructor, sets id and defines error variable

        Args:
            id: id of attached servo
        """
        self.id = id_
        self.return_level = 2
        self.err = 0x00
     
    @decorator_error_handling
    def __log_excep(self, err):
        """Logs exceptions into a .log file
 
        Args:
            err (bytes): contains error bits. If a bit is set to 1 -> error

        Notes:
            * received err from dynamixel status packet is usually a hex 0xij value (i,j being any positive number)
              needs to be casted into a b'00000000' type form for list iteration
              use this function in read_status_pkt 
        """
        if err == 0: return True

        # Don't alter err_lst order
        err_lst = [excep.InputVoltageError, excep.AngleLimitError, excep.OverheatingError, excep.RangeError, excep.ChecksumError, excep.OverloadError, excep.InstructionError]

        _err = bin(err)
        _err = _err[2:].zfill(8)
        _err = _err[::-1]

        c = 0
        for bit in _err:
            if int(bit) is 1:
                raise err_lst[c](inspect.currentframe().f_code.co_name)
            c += 1

    def __send_packet(self, pkt, read_pkt = False):
        """Send packet to servo memory and calculate checksum

        Args:
            pkt: command packet
        """
        pkt[-1] = self.__check_sum(pkt)                                         # calculate checksum
        num_bytes = self.__serial_port.write(bytearray(pkt))                    # send command to serial line
        if self.return_level == 2 and read_pkt == False:
           self.__read_status_pkt(self.__STATUS_PACKET_BASE_LENGTH)

    def __do_action(self, id_=_ID_BROADCAST):
        """Start predefined action on servo

        Args:
            id: id of servo to ping, without id -> broadcast action
        """
        pkt = Dynamixel.__pkt_action.copy()
        pkt[2] = id_
        self.__send_packet(pkt)

    def __write_read_data_pkt(self, register, n_byte):
        """Prepares and sends packet to servo in order to read data from servo memory

        Args:
            register: register address of servo
            n_byte: number of bytes to read
        """
        pkt = Dynamixel.__pkt_read_data.copy()
        pkt[2] = self.id                                                        # set servo ID
        pkt[5] = register                                                       # address where the data is to be read (EEPROM/RAM Area)
        pkt[6] = n_byte                                                         # length of data to be read (usually 0x01 or 0x02)
        self.__send_packet(pkt, True)

    def __read_status_pkt(self, n_byte):
        """Read status packet, set error value and get return values from servo

        Args:
            n_byte: number of bytes to read

        Returns:
            return_packet: contains data byte(s) from servo memory 
        """
        status_packet = list(self.__serial_port.read(n_byte))                   # read status/response packet from dynamixel unit
        if len(status_packet) >= self.__STATUS_PACKET_BASE_LENGTH:
            self.err = status_packet[4]
        self.__log_excep(self.err)
        return status_packet

    @staticmethod
    def __check_sum(pkt):
        """Calculates checksum of packet list

        Args:
            pkt: contains sub elements of command packet to calculate checksum

        Returns:
            checksum
        """
        s = sum(pkt[2:-1])                                                      # add all values from servo-id to last parameter
        return (~s) & 0xFF                                                      # invert sum bit-wise and limit to byte range
        
    # Definition of protected methods
    # Accessible within own and derived classes
    # ---------------------------------------------------------------------------
    def _request_n_byte(self, register, dt_len=1):
        """Read data byte from servo memory

        Args:
            register: register address of servo
            dt_len: number of data bytes to read, default value = 1

        Returns:
            return_packet: contains data byte(s) from servo memory            
        """
        self.__write_read_data_pkt(register, dt_len)                            # send read command                        
        return_packet = self.__read_status_pkt(self.__STATUS_PACKET_BASE_LENGTH + dt_len)                          # receive status/response packet
        return return_packet[5:-1]

    def _request_n_word(self, register, dt_w_len=1):
        """Read data word from servo memory

        Args:
            register: register address of servo
            dt_w_len: number of data words to read, default value = 1

        Notes:
            one data word equals 2 byte = 16 bit 
        """
        dt_len = dt_w_len << 1
        return_packet = self._request_n_byte(register, dt_len)
        return return_packet

    def _write_n_byte_pkt(self, register, data, trigger, return_status = False):
        """Sends packet to servo in order to write n data bytes into servo memory
        
        Args:
            register: register address of servo
            data: list of bytes to write
            trigger: True -> command is directly executed, 
                     False-> command is delayed until action command
        """
        pkt = Dynamixel.__pkt_write_n_byte.copy()
        pkt[2] = self.id                                                        # set servo ID
        pkt[3] = len(data) + 3                                                  # instruction + mem_address + checksum = 3, + remaining data packet length
        pkt[5] = register                                                       # address where the data is to be written (EEPROM/RAM Area)
        if trigger:                                                         
            pkt[4] = self.__DIRECT_ACTION
        else:
            pkt[4] = self.__TRIGGERT_ACTION
        pkt.extend(data)                                                        # complete command packet with missing parameters/data
        pkt.append(0)                                                           # extend packet with checksum value 0 (dummy checksum)
        self.__send_packet(pkt, return_status)

    def _write_n_word_pkt(self, register, data, trigger):
        """Sends packet to servo in order to write data dword into servo memory
        
        Args:
            register: register address of servo
            data: list of words to write
            trigger: True -> command is directly executed,
                     False-> command is delayed until action command
        """
        data_lst = []

        for dword in data:
            low_byte = dword & 0xFF                                             # split dword in low and high byte
            high_byte = (dword >> 8) & 0xFF
            data_lst.append(low_byte)                                           # fill data_lst
            data_lst.append(high_byte)

        self._write_n_byte_pkt(register, data_lst, trigger)

    @staticmethod
    def _send_sync(pkt):
        pkt[-1] = Dynamixel.__check_sum(pkt)
        Dynamixel.__serial_port.write(bytearray(pkt))

    @staticmethod
    @decorator_error_handling
    def _sync_write(mem_address, packet):
        """Used for controlling many dynamixel actuators at the same time

        Args:
            *packet: [IDs, param_1, param_2, ..., param_n]
            *packet: [servo_ID, goal_pos (ticks), speed_val (ticks)]

        Notes:
            Use this instructions only when the lengths and addresses
            of the control table to be written to are the same.
        """
        pkt = Dynamixel.__pkt_sync_write.copy()                                 # [0xFF, 0xFF, 0xFE, 0, 0x83, 0, 0]

        id_lst = []
        data_lst = []

        for data in packet:
            id_lst.append(data[0].id)                                           # ID
            data_lst.extend(data[1:])                                           # parameters  
            if len(data_lst) % len(id_lst) != 0:                                # check if all data packets are the same length
                raise excep.PacketTypeError(inspect.currentframe().f_code.co_name)

        data_length = int((len(data_lst) / len(id_lst)) * 2)

        pkt[3] = (data_length + 1) * len(id_lst) + 4                            # packet length
        pkt[5] = mem_address
        pkt[6] = data_length                                                    # length of each data packet for each servo

        # build data packet 
        for param in packet:
            pkt.append(param[0].id)                                             # servo ID
            for dword in param[1:]:                                             # traverse parameters without ID
                pkt.append(dword & 0xFF)                                        # low byte of dword
                pkt.append((dword >> 8) & 0xFF)                                 # high byte of dword
        pkt.append(0)                                                           # extend packet with checksum value 0 (dummy checksum
        
        Dynamixel._send_sync(pkt)
    
    # Definition of public methods with implicit servo-id
    # Accessible from everywhere
    # ---------------------------------------------------------------------------
    @staticmethod
    def show_serial_lines():
        """Show avaible serial lines"""
        print(Dynamixel.__lines)

    @staticmethod    
    def action_all():
        """Broadcast action to all servo
        
        Usage:
            JointDrive.action_all() or Dynamixel.action_all()
        """
        Dynamixel.__do_action(Dynamixel)

    @staticmethod
    def ping(id):
        """Ping servo with specific id
        
        Args:
            id (int): id of servo

        Returns:
            status packet (lst)
        """
        ping = Dynamixel.__pkt_ping.copy()                                     
        ping[2] = id
        ping[-1] = Dynamixel.__check_sum(ping)
        Dynamixel.__serial_port.write(bytearray(ping))
        return list(Dynamixel.__serial_port.read(Dynamixel.__STATUS_PACKET_BASE_LENGTH))

    def action(self):
        """Start predefined action on servo with assigned id"""
        self.__do_action(self.id)

    def get_last_error(self):
        """Return last error"""
        return self.err

    def read_port(self, n_byte=1):
        """Read n byte from port

        Args:
            n_byte (int): (Default value = 0x01)
        """
        return self.__read_status_pkt(n_byte)
        
if __name__ == "__main__":
    main()
