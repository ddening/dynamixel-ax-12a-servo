from typing import Dict

from joint_drive_.joint_drive import JointDrive

class ServoMaster:

    SPEED = 70

    joint_drive_positions: Dict[JointDrive, float] = {}

    @staticmethod
    def send_positions():
        position_parameters = [
            [joint_drive, position, ServoMaster.SPEED] for joint_drive, position in ServoMaster.joint_drive_positions.items()
        ]

        JointDrive.set_angle_speed_sync(*position_parameters)
        ServoMaster.joint_drive_positions.clear()
