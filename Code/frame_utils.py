import math
import numpy as np
def ned_to_enu_quaternion(ned_quat):
    enu_quat = np.array([ned_quat[1], ned_quat[0], -ned_quat[2], -ned_quat[3]])
    return enu_quat

def enu_to_ned_quaternion(enu_quat):
    ned_quat = np.array([enu_quat[1], enu_quat[0], -enu_quat[2], -enu_quat[3]])
    return ned_quat

def RotMatrixFromQuat(quaternion):
    """Construct a 3x3 rotation matrix given a rotation quaternion"""
    q0 = quaternion[0][0]
    q1 = quaternion[1][0]
    q2 = quaternion[2][0]
    q3 = quaternion[3][0]
    return np.array([
        [2*q0**2 + 2*q1**2 - 1, 2*(q1*q2 - q3*q0),      2*(q1*q3 + q2*q0)],
        [2*(q1*q2 + q3*q0),     2*q0**2 + 2*q2**2 - 1,  2*(q2*q3 - q1*q0)],
        [2*(q1*q3 - q2*q0),     2*(q2*q3 + q1*q0),      2*q0**2 + 2*q3**2 - 1]])
         
def enu2ned(enu_vector):
    enu2ned_quaternion = np.array([
    [0],
    [math.sqrt(2)/2],
    [math.sqrt(2)/2],
    [0]
    ])
    enu2ned_rotation_matrix = RotMatrixFromQuat(enu2ned_quaternion)
    
    ned_vector = np.matmul(enu2ned_rotation_matrix, enu_vector)
    
    return ned_vector

def ned2enu(ned_vector):
    ned2enu_quaternion = np.array([
    [0],
    [-math.sqrt(2)/2],
    [-math.sqrt(2)/2],
    [0]
    ])
    
    ned2enu_rotation_matrix = RotMatrixFromQuat(ned2enu_quaternion)
    
    enu_vector = np.matmul(ned2enu_rotation_matrix, ned_vector)
    
    return enu_vector

def enu2ned_fullstate(enu_state):
    enu_xyz = np.array(enu_state[0:3]).reshape(3,1)
    enu_vxyz = np.array(enu_state[3:6]).reshape(3,1)
    enu_q_wxyz = np.roll(np.array(enu_state[6:10]),1)
    pqr = np.array(enu_state[10:13]).reshape(3,1)
    
    ned_xyz = enu2ned(enu_xyz)
    ned_vxyz = enu2ned(enu_vxyz)
    ned_q_xyzw = np.roll(enu_to_ned_quaternion(enu_q_wxyz),-1)
    
    ned_state = np.concatenate((ned_xyz.flatten(),ned_vxyz.flatten(),ned_q_xyzw.flatten(),pqr.flatten()))
    return ned_state

def ned2enu_fullstate(ned_state):
    # TODO Verify every thing and refactor. for now xyz is correct and verified.
    ned_xyz = np.array(ned_state[0:3]).reshape(3,1)
    ned_vxyz = np.array(ned_state[3:6]).reshape(3,1)
    ned_q_xyzw = np.roll(np.array(ned_state[6:10]),1)
    pqr = np.array(ned_state[10:13]).reshape(3,1)
    
    # enu_xyz = ned2enu(ned_xyz)
    # TODO JUGAADING IT FOR NOW
    enu_xyz[0] = ned_xyz[0]
    enu_xyz[1] = -ned_xyz[1]
    enu_xyz[2] = -ned_xyz[2]

    enu_vxyz = ned2enu(ned_vxyz)
    enu_q_wxyz = np.roll(ned_to_enu_quaternion(ned_q_xyzw),-1)
    
    enu_state = np.concatenate((enu_xyz.flatten(),enu_vxyz.flatten(),enu_q_wxyz.flatten(),pqr.flatten()))
    return enu_state

def quat_xyzw2wxyz(q_xyzw):
    return np.roll(q_xyzw,1)

def quat_wxyz2xyzw(q_wxyz):
    return np.roll(q_wxyz,-1)
