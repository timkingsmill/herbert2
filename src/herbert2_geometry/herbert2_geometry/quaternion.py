# -------------------------------------------------------------------------

from math import degrees, sin, cos, atan2, pi, sqrt, asin, acos, fabs
import attr
import geometry_msgs.msg

# -------------------------------------------------------------------------

rad2deg: float = 180.0 / pi
deg2rad: float = pi / 180.0

# -------------------------------------------------------------------------

@attr.s(slots = True, frozen = True)
class EulerAngles():
    roll  = attr.ib(type = float, default = 0.0)
    pitch = attr.ib(type = float, default = 0.0)
    yaw   = attr.ib(type = float, default = 0.0)

    # ---------------------------------------------------------------------

    def __init__(self) -> None:
        pass

    # ---------------------------------------------------------------------

# -------------------------------------------------------------------------

@attr.s(slots = True, frozen = True)
class Quaternion():
    """
    Based on code from:
    https://github.com/MomsFriendlyRobotCompany/squaternion/blob/master/squaternion/squaternion.py

    This class should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    # ---------------------------------------------------------------------

    w = attr.ib(type = float, default = 1.0)
    x = attr.ib(type = float, default = 0.0)
    y = attr.ib(type = float, default = 0.0)
    z = attr.ib(type = float, default = 0.0)

    # ---------------------------------------------------------------------

    def __init__(self) -> None:
        pass

    # ---------------------------------------------------------------------

    def __len__(self):
        """
            Enables the length function to work: len(q) => 4
        """
        return 4

    # ---------------------------------------------------------------------

    def get_yaw(self, use_degrees = False) -> float:
        """
            Calculate yaw (rotation around the z-axis)
            Range is -180.0 .. +180.0
            Positive yaw is an anti-clockwise rotation of the robot.
            Negative yaw is a clockwise rotation of the robot
        """
        siny_cosp: float =       2.0 * ((self.w * self.z) + (self.x * self.y))
        cosy_cosp: float = 1.0 - 2.0 * ((self.y * self.y) + (self.z * self.z))
        yaw_radians: float = atan2(siny_cosp, cosy_cosp)

        if use_degrees:
            yaw = degrees(yaw_radians)

        return yaw

    # ---------------------------------------------------------------------

    def is_valid(self) -> bool:
        """
        Return true if the fields of the quaternion are valid.
        """
        valid: bool = True

        # Test the value range of each quaternion field.
        valid &= (fabs(self.w) <= 1.0)
        valid &= (fabs(self.x) <= 1.0)
        valid &= (fabs(self.y) <= 1.0)
        valid &= (fabs(self.z) <= 1.0)
        
        return valid

    # ---------------------------------------------------------------------

    @staticmethod
    def from_msg(quat: geometry_msgs.msg.Quaternion):
        return(Quaternion(quat.w, quat.x, quat.y, quat.z)) 

    # ---------------------------------------------------------------------

    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float, use_degrees = False):
        """
        Euler angles euler2quat(roll, pitch, yaw, degrees=False). 
        default is radians, but set degrees True if giving degrees.
        This is a modified version of this:
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
        """
        if use_degrees:
            roll  *= deg2rad
            pitch *= deg2rad
            yaw   *= deg2rad
        
        # Abbreviations for the various angular functions
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        w: float = (cy * cr * cp) + (sy * sr * sp)
        x: float = (cy * sr * cp) - (sy * cr * sp)
        y: float = (cy * cr * sp) + (sy * sr * cp)
        z: float = (sy * cr * cp) - (cy * sr * sp)

        return Quaternion(w, x, y, z)

    # ---------------------------------------------------------------------

    def to_euler(self, use_degrees = False) -> EulerAngles:
        """
            Returns the Euler angles as a tuple(roll, pitch, yaw)
            This is a modified version of this:
            https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        """
        ysqr = self.y * self.y
        
        # Calculate roll
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + ysqr)
        roll = atan2(t0, t1)

        # Calculate pitch
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)

        # Calculate yaw
        yaw: float = self.get_yaw(use_degrees)

        if use_degrees:
            roll  *= rad2deg
            pitch *= rad2deg
            #yaw   *= rad2deg

        """
        To Do: Return EulerAngles class
        """
        return ([roll, pitch, yaw])
        #return EulerAngles(roll, pitch, yaw)

    # -------------------------------------------------------------------------

    @property
    def magnitude(self):
        """Returns the magnitude of the quaternion"""
        return sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    # -------------------------------------------------------------------------

    @property
    def get_normalized(self):
        """
        Returns a normalized a quaterion (unit quaternion) so its
        magnitude is 1.0
        """
        m = self.magnitude

        w, x, y, z = self.w, self.x, self.y, self.z

        if m == 1.0:
            return Quaternion(w, x, y, z)
        elif m < 1e-6:
            raise ZeroDivisionError('Quaternion.normalize: div by {}'.format(m))

        w /= m
        x /= m
        y /= m
        z /= m

        return Quaternion(w, x, y, z)

    # -------------------------------------------------------------------------

    @property
    def is_normalized(self) -> bool:
        return fabs(self.magnitude - 1.0) < 0.001

    # -------------------------------------------------------------------------
