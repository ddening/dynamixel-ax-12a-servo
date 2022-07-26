import os
import time
import random
import pytest
from joint_drive_.joint_drive import JointDrive
from joint_drive_.servo_ax12a import ServoAx12a

# use py.test -v

# Constants
DEFAULT_ID_A = 6
DEFAULT_ID_B = 14
NUMBER_OF_SERVO_IDS = 18
NUMBER_OF_TEST_CASES = 10

# find all avaiable servos
id_lst = []
for id in range(0, NUMBER_OF_SERVO_IDS):
    id_ = JointDrive.ping(id)
    if len(id_) != 0:
        id_lst.append(id_[2])

# get ID when running pytest from demo.py
if os.path.exists("demo.txt"):
    f = open("demo.txt", "r")
    if f.mode == 'r':
        content = f.read()
        if len(content) is 0:
            if len(id_lst) is 0:
                servoID = DEFAULT_ID_A
            else:
                servoID = id_lst[0]
        else:
            servoID = int(content)
else:
    if len(id_lst) is 0:
        servoID = DEFAULT_ID_A
    else:
        servoID = id_lst[0]

SYNC_ID_1 = servoID
SYNC_ID_2 = next((e for e in id_lst if e != servoID), DEFAULT_ID_B)

# Test Objects
servo = ServoAx12a(servoID)
joint = JointDrive(servoID)

# ====== RANDOM TEST CASES FROM A GIVEN POOL ======
RANDOM_SERVO_POSITION = [random.randrange(0, 1024) for n in range(0, NUMBER_OF_TEST_CASES)]
RANDOM_SERVO_SPEED = [random.randrange(0, 1024) for n in range(0, NUMBER_OF_TEST_CASES)]

RANDOM_JOINTDRIVE_POSITION = [round(random.uniform(-2.6, 2.6), 2) for n in range(0, NUMBER_OF_TEST_CASES)]
RANDOM_JOINTDRIVE_SPEED_RPM = [random.randrange(0, ServoAx12a._LIMITED_MAX_RPM) for n in range(0, NUMBER_OF_TEST_CASES)]
RANDOM_JOINTDRIVE_SPEED_PERC = [random.randrange(0, 101) for n in range(0, NUMBER_OF_TEST_CASES)]

RANDOM_SYNC_POSITION = [round(random.uniform(-2.6, 2.6), 2) for n in range(0, NUMBER_OF_TEST_CASES)]

# ====== PREDEFINED TEST CASES ======
SERVO_DELAY = [5, 10, 2]
SERVO_RETURN_LEVEL = [1, 2]
SERVO_GOAL_POSITION = RANDOM_SERVO_POSITION  + [0, 100, 222, 1023]
SERVO_MOVING_SPEED = RANDOM_SERVO_SPEED + [10, 1000, 1023, 0]

JOINTDRIVE_GOAL_POSITION = RANDOM_JOINTDRIVE_POSITION + [0, 2.6, -2.6]
JOINTDRIVE_SPEED_RPM = RANDOM_JOINTDRIVE_SPEED_RPM + [50, ServoAx12a._LIMITED_MAX_RPM]
JOINTDRIVE_SPEED_PERC = RANDOM_JOINTDRIVE_SPEED_PERC + [0, 50, 100]
JOINTDRIVE_EXPECTED_RPM = [joint._JointDrive__convert_ticks_to_speed(e) for e in 
                          [joint._JointDrive__convert_percent_speed_to_ticks(e) for e in JOINTDRIVE_SPEED_PERC]]
JOINTDRIVE_CCW = [2.6]
JOINTDRIVE_CW  = [2.6]

SYNC_OBJ_ALPHA = JointDrive(SYNC_ID_1)
SYNC_OBJ_BETA  = JointDrive(SYNC_ID_2)
SYNC_GOAL_POSITION = RANDOM_SYNC_POSITION + [0, 2.6]
SYNC_SPEED = 100

# ====== ServoAX12a Testumgebung ======

def test_return_delay():
    for delay in SERVO_DELAY:
        time.sleep(1)
        servo.set_return_delay(delay, True)
        time.sleep(1)
        assert servo.get_return_delay() == delay
     
def test_return_level():
    for return_level in SERVO_RETURN_LEVEL:
        time.sleep(1)
        servo.set_return_level(return_level, True)
        time.sleep(1)
        assert servo.get_return_level() == return_level

def test_goal_position():
    for gPosition in SERVO_GOAL_POSITION:
        time.sleep(1)
        servo.set_goal_position(gPosition, True)
        time.sleep(1)
        assert servo.get_goal_position() == gPosition

def test_moving_speed():
    for speed in SERVO_MOVING_SPEED:
        servo.set_moving_speed(speed, True)
        assert servo.get_moving_speed() == speed

def test_goal_pos_speed():
    servo.set_goal_pos_speed(100, 100, True)
    assert servo.get_goal_pos_speed() == [100, 100]
    servo.set_goal_pos_speed(1023, 1023, True)
    assert servo.get_goal_pos_speed() == [1023, 1023]

def test_moving_status():
    time.sleep(2)
    assert servo.get_moving_status() == 0
    servo.set_goal_pos_speed(0, 500, True)
    time.sleep(0.2)
    assert servo.get_moving_status() == 1
    time.sleep(1)

def test_temperature():
    assert servo.get_present_temperature() > 20 and servo.get_present_temperature() < 60

# ====== JointDrive Testumgebung ======

def test_set_joint_angle():
    for gPosition in JOINTDRIVE_GOAL_POSITION:
        joint.set_goal_position(gPosition, True)
        assert joint.get_goal_position() == pytest.approx(gPosition, abs=1e-1)
        time.sleep(2)

def test_set_speed_rpm():
    for rpm in JOINTDRIVE_SPEED_RPM:
        joint.set_speed_rpm(rpm, True)
        assert joint.get_moving_speed() == pytest.approx(rpm, abs=2e-0)

def test_set_speed_perc():
    for (perc, rpm) in zip(JOINTDRIVE_SPEED_PERC, JOINTDRIVE_EXPECTED_RPM):
        joint.set_speed_percent(perc, True)
        assert joint.get_moving_speed() == pytest.approx(rpm, abs=2)

def test_ccw():
    # Nullstellung
    joint.set_goal_position(0, True)
    assert joint.get_goal_position() == pytest.approx(0, abs=1e-1)
    time.sleep(2)

    joint.ccw = True

    for pos in JOINTDRIVE_CCW:
        joint.set_goal_position(pos, True)
        assert joint.get_goal_position() == pytest.approx(pos, abs=1e-1)
        time.sleep(2)

def test_cw():
    # Nullstellung
    joint.set_goal_position(0, True)
    assert joint.get_goal_position() == pytest.approx(0, abs=1e-1)
    time.sleep(2)

    joint.ccw = False

    for pos in JOINTDRIVE_CW:
        joint.set_goal_position(pos, True)
        assert joint.get_goal_position() == pytest.approx(pos, abs=1e-1)
        time.sleep(2)

def test_sync_function():
    for pos in SYNC_GOAL_POSITION:
        JointDrive.set_angle_speed_sync([SYNC_OBJ_ALPHA, pos, SYNC_SPEED], [SYNC_OBJ_BETA, pos, SYNC_SPEED])
        assert SYNC_OBJ_ALPHA.get_goal_position() == pytest.approx(pos, abs=1e-1) 
        assert SYNC_OBJ_BETA.get_goal_position() == pytest.approx(pos, abs=1e-1)
        time.sleep(2)

def test_sync_ccw():
    JointDrive.set_angle_speed_sync([SYNC_OBJ_ALPHA, pos, SYNC_SPEED], [SYNC_OBJ_BETA, pos, SYNC_SPEED])
    assert SYNC_OBJ_ALPHA.get_goal_position() == pytest.approx(0, abs=1e-1) 
    assert SYNC_OBJ_BETA.get_goal_position() == pytest.approx(0, abs=1e-1)
    time.sleep(2)

    beta.ccw = True

    JointDrive.set_angle_speed_sync([SYNC_OBJ_ALPHA, 2.6, SYNC_SPEED], [SYNC_OBJ_BETA, 2.6, SYNC_SPEED])
    assert SYNC_OBJ_ALPHA.get_goal_position() == pytest.approx(2.6, abs=1e-1) 
    assert SYNC_OBJ_BETA.get_goal_position() == pytest.approx(2.6, abs=1e-1)
    time.sleep(2)

def test_present_position():
    time.sleep(1)
    old_val = 0
    joint.set_goal_position(0, True)
    time.sleep(1)
    joint.set_goal_position(2.6, True)

    moving_status = joint._request_n_byte(0x2E)[0]

    while moving_status is 1:
         moving_status = joint._request_n_byte(0x2E)[0]
         current_val = joint.get_present_position()
         assert current_val >= old_val 
    