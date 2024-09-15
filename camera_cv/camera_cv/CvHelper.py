import math
import numpy as np
from scipy import signal
import numpy as np

# Euler angles to quaternion
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        #OUTPUT --> i : yaw, j : roll, k : pitch 
        return roll_x, pitch_y, yaw_z # in radians

# Transform 3D coordinates from camera frame to base frame
def tramform2base(x,y,z,angles):
    x = x-200 # 200 mm offset from camera to type-c board
    pitch = angles[2]
    roll = angles[1]
    yaw = angles[0]

    #transform z --> x, x --> y, y --> -z to standard TF
    R_pitch = np.array([[math.cos(pitch ), 0, math.sin(pitch ),0],
                    [0, 1, 0,0],
                    [-math.sin(pitch ), 0, math.cos(pitch ),0],
                    [0,0,0,1]])
    
    R_yaw = np.array([[math.cos(yaw), -math.sin(yaw), 0,0],
                    [math.sin(yaw), math.cos(yaw), 0,0],
                    [0, 0, 1,0],
                    [0,0,0,1]])
    
    xyz = np.array([x,y,z,1]).transpose()
    xyz_final = np.dot(R_yaw,np.dot(R_pitch,xyz)).transpose()
    
    #calculate pitch yaw roll final
    yaw_final = np.arctan2(y, x)
    pitch_final = np.arctan2(-z, np.sqrt(x**2 + y**2))
    roll_final = np.arctan2(np.sin(yaw)*z - np.cos(yaw)*y, np.cos(pitch)*x + np.sin(pitch)*y)
    
    # Convert angles to degrees

    return [roll_final, pitch_final, yaw_final]

# spherical to cartesian coordinates transformation
def spherical_to_cartesian(r, phi, theta):
    phi = ((phi + np.pi) % (2*np.pi)) - np.pi

    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return x, -y, z

# Wrap angle to range -pi to pi
def wrap_angle(angle):
    # Wrap angle to range -pi to pi
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Transform 3D coordinates from camera frame to base frame
def tramform2base(x,y,z,angles):

    x = x+20 # 200 mm offset from camera to type-c board
    pitch = round(angles[1], 3)
    roll = round(angles[0], 3)
    yaw = -round(angles[2], 3)
    #transform z --> x, x --> y, y --> -z to standard TF
    R_pitch = np.array([[math.cos(pitch ), 0, math.sin(pitch )],
                    [0, 1, 0],
                    [-math.sin(pitch ), 0, math.cos(pitch )]])
    
    R_yaw = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                    [math.sin(yaw), math.cos(yaw), 0],
                    [0, 0, 1]])
    
    xyz = np.array([x,y,z])
    xyz_final = (R_yaw.dot(R_pitch.dot(xyz)))
    # print(xyz_final)
    x_final = xyz_final[0]
    y_final = xyz_final[1]
    z_final = xyz_final[2]
    
    #calculate pitch yaw roll final
    yaw_final = np.arctan2(y_final, x_final)
    # if yaw_final <= 0:
    #     yaw_final += math.pi
    # else:
    #     yaw_final -= math.pi
    
    pitch_final = np.arctan2(z_final, np.sqrt(x_final**2 + y_final**2))
    roll_final = np.arctan2(np.sin(yaw)*z_final - np.cos(yaw)*y_final, np.cos(pitch)*x_final + np.sin(pitch)*y_final)
    # print([roll_final, pitch_final, yaw_final])
    # Convert angles to degree
    return [round(roll_final, 2), 
            round(pitch_final, 2), 
            round(yaw_final, 2)]

# Apply Filtfilt filter to data collection
# input : data collection list
def ellip_filter(pitch_record, yaw_record):

    # Create a elliptic filter
    b,a= signal.ellip(3, 0.02, 120, 0.125)

    # Apply filter to data
    yaw_record_filtered = signal.filtfilt(b, a, yaw_record, method="gust",irlen=50)
    pitch_record_filtered = signal.filtfilt(b, a, pitch_record, method="gust",irlen=50)

    return [pitch_record_filtered[-1], yaw_record_filtered[-1]]

def normalize_angle(x):
    """
    Normalize the angle to be within the range of -pi to pi.
    """
    return np.arctan2(np.sin(x), np.cos(x))

def fx(x, dt):
    """
    State transition function for the EKF.
    x[0] is the angle, x[1] is the angular velocity.
    """
    angle = normalize_angle(x[0] + x[1] * dt)
    angular_velocity = x[1]
    return np.array([angle, angular_velocity])

def jfx(x, dt):
    """
    Jacobian of the state transition function.
    """
    return np.array([[1, dt], [0, 1]])

def hx(x):
    """
    Measurement function.
    Only the angle is measured.
    """
    return np.array([x[0]])

def jhx(x):
    """
    Jacobian of the measurement function.
    """
    return np.array([[1, 0]])


