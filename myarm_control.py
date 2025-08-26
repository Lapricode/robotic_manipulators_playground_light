from pymycobot.myarm import MyArm
import time
import numpy as np


mc = MyArm('/dev/ttyAMA0', 115200)
reset_angles = np.array([0, 0, 0, 0, 0, 0, 0])
offset_angles = np.array([0, 0, 0, 0, 0, 0, 0])
# offset_angles = np.array([0, 1, 0, -2, 0, 0, 0])


def decode_command(command, old_joints_angles, joints_motors_list):
    motors_names = []
    for motor in joints_motors_list:
        motors_names.append(motor[0])
    new_joints_angles = []
    for angle in old_joints_angles:
        new_joints_angles.append(np.rad2deg(angle))
    for item in command.split():
        i = 0
        while i < len(item) and item[i].isalpha():
            i += 1
        if item[:i] in motors_names:
            joint_index = motors_names.index(item[:i])
            new_joints_angles[joint_index] = float(item[i:])
    return new_joints_angles  # in degrees

def send_joints_angles(joints_angles, speed = 30, offset_angles = offset_angles):
    # joints_angles is in degrees
    mc.send_angles((np.array(joints_angles) + np.array(offset_angles)).tolist(), speed)
    while mc.is_moving(): pass
    print(f"coords: {mc.get_coords()}")
