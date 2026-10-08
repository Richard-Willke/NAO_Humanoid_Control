import numpy as np
import math


""" Utility functions to convert one from of rotation to another """


def to_quat(roll, pitch, yaw):
        
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return np.array([qw, qx, qy, qz])

def to_rot(orientation = np.zeros(3)):
    Rx = np.array([[1, 0, 0], 
                    [0, math.cos(orientation[0]), -math.sin(orientation[0])], 
                    [0, math.sin(orientation[0]),  math.cos(orientation[0])]])


    Ry = np.array([[math.cos(orientation[1]),  0, math.sin(orientation[1])], 
                    [0, 1, 0], 
                    [-math.sin(orientation[1]), 0, math.cos(orientation[1])]])

    
    Rz = np.array([[math.cos(orientation[2]), -math.sin(orientation[2]), 0], 
                    [math.sin(orientation[2]),  math.cos(orientation[2]), 0], 
                    [0, 0, 1]])
    

    """ Note by Rohan : Check order of multiplication """
    R = np.dot(Rz,np.dot(Ry,Rx))

    return R


def rotation_matrix_to_euler(R):
    """
    Convert a 3x3 rotation matrix to roll, pitch, yaw angles.

    :param R: 3x3 rotation matrix (numpy array)
    :return: (roll, pitch, yaw) in radians
    """
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)

    singular = sy < 1e-6  # Check for singularity

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0  # Set yaw to 0 in singular case

    return [roll, pitch, yaw]  


def quaternion_to_euler(q = None):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]


def rotation_matrix_to_xzy_quaternion(R):
    """
    Convert a 3x3 rotation matrix to a quaternion in XZY order.
    
    Parameters:
        R (numpy.ndarray): 3x3 rotation matrix.
    
    Returns:
        numpy.ndarray: Quaternion (x, z, y, w).
    """
    trace = np.trace(R)
    
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        z = (R[0, 2] - R[2, 0]) / s
        y = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        x = 0.25 * s
        w = (R[2, 1] - R[1, 2]) / s
        z = (R[0, 1] + R[1, 0]) / s
        y = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        y = 0.25 * s
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        z = 0.25 * s
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
    
    return np.array([x, z, y, w])



def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[3] #w
    q1 = Q[0] #x
    q2 = Q[1] #y
    q3 = Q[2] #z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix