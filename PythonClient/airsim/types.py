from __future__ import print_function
import msgpackrpc  # install as admin: pip install msgpack-rpc-python
import numpy as np  # pip install numpy
import math


class MsgpackMixin:
    attribute_order = []

    def __repr__(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def to_msgpack(self, *args, **kwargs):
        encoded = []
        for attr_name, attr_type in self.attribute_order:
            value = getattr(self, attr_name)
            if isinstance(value, list) and value and isinstance(value[0], MsgpackMixin):
                encoded.append([v.to_msgpack() for v in value])
            elif isinstance(value, MsgpackMixin):
                encoded.append(value.to_msgpack())
            else:
                encoded.append(value)
        return encoded

    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        if len(encoded) != len(cls.attribute_order):
            raise ValueError("Length of encoded data does not match number of attributes")

        for index, (attr_name, attr_type) in enumerate(cls.attribute_order):
            value = encoded[index]
            if isinstance(value, list) and value and isinstance(value[0], MsgpackMixin):
                value = [v.from_msgpack() for v in value]
            elif issubclass(attr_type, MsgpackMixin):
                value = attr_type.from_msgpack(value)
            setattr(obj, attr_name, value)

        return obj


class _ImageType(type):
    @property
    def Scene(cls):
        return 0

    def DepthPlanar(cls):
        return 1

    def DepthPerspective(cls):
        return 2

    def DepthVis(cls):
        return 3

    def DisparityNormalized(cls):
        return 4

    def Segmentation(cls):
        return 5

    def SurfaceNormals(cls):
        return 6

    def Infrared(cls):
        return 7

    def OpticalFlow(cls):
        return 8

    def OpticalFlowVis(cls):
        return 9

    def __getattr__(self, key):
        if key == 'DepthPlanner':
            print(
                '\033[31m' + "DepthPlanner has been (correctly) renamed to DepthPlanar. Please use ImageType.DepthPlanar instead." + '\033[0m')
            raise AttributeError


class ImageType(metaclass=_ImageType):
    Scene = 0
    DepthPlanar = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7
    OpticalFlow = 8
    OpticalFlowVis = 9


class DrivetrainType:
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1


class LandedState:
    Landed = 0
    Flying = 1


class WeatherParameter:
    Rain = 0
    Roadwetness = 1
    Snow = 2
    RoadSnow = 3
    MapleLeaf = 4
    RoadLeaf = 5
    Dust = 6
    Fog = 7
    Enabled = 8


class Vector2r(MsgpackMixin):
    x_val = 0.0
    y_val = 0.0

    attribute_order = [
        ('x_val', float),
        ('y_val', float)
    ]

    def __init__(self, x_val=0.0, y_val=0.0):
        self.x_val = x_val
        self.y_val = y_val


class Vector3r(MsgpackMixin):
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    attribute_order = [
        ('x_val', float),
        ('y_val', float),
        ('z_val', float)
    ]

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

    @staticmethod
    def nanVector3r():
        return Vector3r(np.nan, np.nan, np.nan)

    def containsNan(self):
        return (math.isnan(self.x_val) or math.isnan(self.y_val) or math.isnan(self.z_val))

    def __add__(self, other):
        return Vector3r(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val)

    def __sub__(self, other):
        return Vector3r(self.x_val - other.x_val, self.y_val - other.y_val, self.z_val - other.z_val)

    def __truediv__(self, other):
        if type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Vector3r(self.x_val / other, self.y_val / other, self.z_val / other)
        else:
            raise TypeError('unsupported operand type(s) for /: %s and %s' % (str(type(self)), str(type(other))))

    def __mul__(self, other):
        if type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Vector3r(self.x_val * other, self.y_val * other, self.z_val * other)
        else:
            raise TypeError('unsupported operand type(s) for *: %s and %s' % (str(type(self)), str(type(other))))

    def dot(self, other):
        if type(self) == type(other):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val
        else:
            raise TypeError('unsupported operand type(s) for \'dot\': %s and %s' % (str(type(self)), str(type(other))))

    def cross(self, other):
        if type(self) == type(other):
            cross_product = np.cross(self.to_numpy_array(), other.to_numpy_array())
            return Vector3r(cross_product[0], cross_product[1], cross_product[2])
        else:
            raise TypeError(
                'unsupported operand type(s) for \'cross\': %s and %s' % (str(type(self)), str(type(other))))

    def get_length(self):
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2) ** 0.5

    def distance_to(self, other):
        return ((self.x_val - other.x_val) ** 2 + (self.y_val - other.y_val) ** 2 + (
                self.z_val - other.z_val) ** 2) ** 0.5

    def to_Quaternionr(self):
        return Quaternionr(self.x_val, self.y_val, self.z_val, 0)

    def to_numpy_array(self):
        return np.array([self.x_val, self.y_val, self.z_val], dtype=np.float32)

    def __iter__(self):
        return iter((self.x_val, self.y_val, self.z_val))


class Quaternionr(MsgpackMixin):
    w_val = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    attribute_order = [
        ('w_val', float),
        ('x_val', float),
        ('y_val', float),
        ('z_val', float)
    ]

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
        self.w_val = w_val

    @staticmethod
    def nanQuaternionr():
        return Quaternionr(np.nan, np.nan, np.nan, np.nan)

    def containsNan(self):
        return (math.isnan(self.w_val) or math.isnan(self.x_val) or math.isnan(self.y_val) or math.isnan(self.z_val))

    def __add__(self, other):
        if type(self) == type(other):
            return Quaternionr(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val,
                               self.w_val + other.w_val)
        else:
            raise TypeError('unsupported operand type(s) for +: %s and %s' % (str(type(self)), str(type(other))))

    def __mul__(self, other):
        if type(self) == type(other):
            t, x, y, z = self.w_val, self.x_val, self.y_val, self.z_val
            a, b, c, d = other.w_val, other.x_val, other.y_val, other.z_val
            return Quaternionr(w_val=a * t - b * x - c * y - d * z,
                               x_val=b * t + a * x + d * y - c * z,
                               y_val=c * t + a * y + b * z - d * x,
                               z_val=d * t + z * a + c * x - b * y)
        else:
            raise TypeError('unsupported operand type(s) for *: %s and %s' % (str(type(self)), str(type(other))))

    def __truediv__(self, other):
        if type(other) == type(self):
            return self * other.inverse()
        elif type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Quaternionr(self.x_val / other, self.y_val / other, self.z_val / other, self.w_val / other)
        else:
            raise TypeError('unsupported operand type(s) for /: %s and %s' % (str(type(self)), str(type(other))))

    def dot(self, other):
        if type(self) == type(other):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val + self.w_val * other.w_val
        else:
            raise TypeError('unsupported operand type(s) for \'dot\': %s and %s' % (str(type(self)), str(type(other))))

    def cross(self, other):
        if type(self) == type(other):
            return (self * other - other * self) / 2
        else:
            raise TypeError(
                'unsupported operand type(s) for \'cross\': %s and %s' % (str(type(self)), str(type(other))))

    def outer_product(self, other):
        if type(self) == type(other):
            return (self.inverse() * other - other.inverse() * self) / 2
        else:
            raise TypeError(
                'unsupported operand type(s) for \'outer_product\': %s and %s' % (str(type(self)), str(type(other))))

    def rotate(self, other):
        if type(self) == type(other):
            if other.get_length() == 1:
                return other * self * other.inverse()
            else:
                raise ValueError('length of the other Quaternionr must be 1')
        else:
            raise TypeError(
                'unsupported operand type(s) for \'rotate\': %s and %s' % (str(type(self)), str(type(other))))

    def conjugate(self):
        return Quaternionr(-self.x_val, -self.y_val, -self.z_val, self.w_val)

    def star(self):
        return self.conjugate()

    def inverse(self):
        return self.star() / self.dot(self)

    def sgn(self):
        return self / self.get_length()

    def get_length(self):
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2 + self.w_val ** 2) ** 0.5

    def to_numpy_array(self):
        return np.array([self.x_val, self.y_val, self.z_val, self.w_val], dtype=np.float32)

    def __iter__(self):
        return iter((self.x_val, self.y_val, self.z_val, self.w_val))


class Pose(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()

    attribute_order = [
        ('position', Vector3r),
        ('orientation', Quaternionr)
    ]

    def __init__(self, position_val=None, orientation_val=None):
        position_val = position_val if position_val is not None else Vector3r()
        orientation_val = orientation_val if orientation_val is not None else Quaternionr()
        self.position = position_val
        self.orientation = orientation_val

    @staticmethod
    def nanPose():
        return Pose(Vector3r.nanVector3r(), Quaternionr.nanQuaternionr())

    def containsNan(self):
        return (self.position.containsNan() or self.orientation.containsNan())

    def __iter__(self):
        return iter((self.position, self.orientation))


class CollisionInfo(MsgpackMixin):
    has_collided = False
    normal = Vector3r()
    impact_point = Vector3r()
    position = Vector3r()
    penetration_depth = 0.0
    time_stamp = 0.0
    object_name = ""
    object_id = -1

    attribute_order = [
        ('has_collided', bool),
        ('penetration_depth', float),
        ('time_stamp', np.uint64),
        ('normal', Vector3r),
        ('impact_point', Vector3r),
        ('position', Vector3r),
        ('object_name', str),
        ('object_id', int)
    ]


class GeoPoint(MsgpackMixin):
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0

    attribute_order = [
        ('latitude', float),
        ('longitude', float),
        ('altitude', float)
    ]


class YawMode(MsgpackMixin):
    is_rate = True
    yaw_or_rate = 0.0

    attribute_order = [
        ('is_rate', bool),
        ('yaw_or_rate', float)
    ]

    def __init__(self, is_rate=True, yaw_or_rate=0.0):
        self.is_rate = is_rate
        self.yaw_or_rate = yaw_or_rate


class RCData(MsgpackMixin):
    timestamp = 0
    pitch, roll, throttle, yaw = (0.0,) * 4  # init 4 variable to 0.0
    left_z, right_z = (0.0,) * 2
    switches = 0
    vendor_id = ''
    is_initialized = False
    is_valid = False

    attribute_order = [
        ('timestamp', np.uint64),
        ('pitch', float),
        ('roll', float),
        ('throttle', float),
        ('yaw', float),
        ('left_z', float),
        ('right_z', float),
        ('switches', int),
        ('vendor_id',str),
        ('is_initialized', bool),
        ('is_valid', bool)
    ]

    def __init__(self, timestamp=0, pitch=0.0, roll=0.0, throttle=0.0, yaw=0.0, left_z=0.0, right_z=0.0, switches=0,
                 vendor_id='', is_initialized=False,
                 is_valid=False):
        self.timestamp = timestamp
        self.pitch = pitch
        self.roll = roll
        self.throttle = throttle
        self.yaw = yaw
        self.left_z = left_z
        self.right_z = right_z
        self.switches = switches
        self.vendor_id = vendor_id
        self.is_initialized = is_initialized
        self.is_valid = is_valid


class ImageRequest(MsgpackMixin):
    camera_name = '0'
    image_type = ImageType.Scene
    pixels_as_float = False
    compress = False

    attribute_order = [
        ('camera_name', str),
        ('image_type', int),
        ('pixels_as_float', bool),
        ('compress', bool)
    ]

    def __init__(self, camera_name, image_type, pixels_as_float=False, compress=True):
        # todo: in future remove str(), it's only for compatibility to pre v1.2
        self.camera_name = str(camera_name)
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress


class ImageResponse(MsgpackMixin):
    image_data_uint8 = np.array([], dtype=np.uint8)
    image_data_float = np.array([], dtype=float)
    camera_name = ''
    camera_position = Vector3r()
    camera_orientation = Quaternionr()
    time_stamp = np.uint64(0)
    message = ''
    pixels_as_float = 0.0
    compress = True
    width = 0
    height = 0
    image_type = ImageType.Scene

    attribute_order = [
        ('image_data_uint8', np.ndarray),
        ('image_data_float', np.ndarray),
        ('camera_position', Vector3r),
        ('camera_name', str),
        ('camera_orientation', Quaternionr),
        ('time_stamp', np.uint64),
        ('message', str),
        ('pixels_as_float', bool),
        ('compress', bool),
        ('width', int),
        ('height', int),
        ('image_type', int)
    ]


class CarControls(MsgpackMixin):
    throttle = 0.0
    steering = 0.0
    brake = 0.0
    handbrake = False
    is_manual_gear = False
    manual_gear = 0
    gear_immediate = True

    attribute_order = [
        ('throttle', float),
        ('steering', float),
        ('brake', float),
        ('handbrake', bool),
        ('is_manual_gear', bool),
        ('manual_gear', int),
        ('gear_immediate', bool)
    ]

    def __init__(self, throttle=0, steering=0, brake=0,
                 handbrake=False, is_manual_gear=False, manual_gear=0, gear_immediate=True):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake
        self.handbrake = handbrake
        self.is_manual_gear = is_manual_gear
        self.manual_gear = manual_gear
        self.gear_immediate = gear_immediate

    def set_throttle(self, throttle_val, forward):
        if (forward):
            self.is_manual_gear = False
            self.manual_gear = 0
            self.throttle = abs(throttle_val)
        else:
            self.is_manual_gear = False
            self.manual_gear = -1
            self.throttle = - abs(throttle_val)


class KinematicsState(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()
    linear_velocity = Vector3r()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()
    angular_acceleration = Vector3r()

    attribute_order = [
        ('position', Vector3r),
        ('orientation', Quaternionr),
        ('linear_velocity', Vector3r),
        ('angular_velocity', Vector3r),
        ('linear_acceleration', Vector3r),
        ('angular_acceleration', Vector3r)
    ]


class EnvironmentState(MsgpackMixin):
    position = Vector3r()
    geo_point = GeoPoint()
    gravity = Vector3r()
    air_pressure = 0.0
    temperature = 0.0
    air_density = 0.0

    attribute_order = [
        ('position', Vector3r),
        ('geo_point', GeoPoint),
        ('gravity', Vector3r),
        ('air_pressure', float),
        ('temperature', float),
        ('air_density', float)
    ]


class CarState(MsgpackMixin):
    speed = 0.0
    gear = 0
    rpm = 0.0
    maxrpm = 0.0
    handbrake = False
    kinematics_estimated = KinematicsState()
    timestamp = np.uint64(0)

    attribute_order = [
        ('speed', float),
        ('gear', int),
        ('rpm', float),
        ('maxrpm', float),
        ('handbrake', bool),
        ('kinematics_estimated', KinematicsState),
        ('timestamp', np.uint64)
    ]


class MultirotorState(MsgpackMixin):
    collision = CollisionInfo()
    kinematics_estimated = KinematicsState()
    gps_location = GeoPoint()
    timestamp = np.uint64(0)
    landed_state = LandedState.Landed
    rc_data = RCData()
    ready = False
    ready_message = ""
    can_arm = False

    attribute_order = [
        ('collision', CollisionInfo),
        ('kinematics_estimated', KinematicsState),
        ('gps_location', GeoPoint),
        ('timestamp', np.uint64),
        ('landed_state', int),
        ('rc_data', RCData)
    ]


class RotorParameters(MsgpackMixin):
    thrust = 0.0
    torque_scaler = 0.0
    speed = 0.0

    attribute_order = [
        ('thrust', float),
        ('torque_scaler', float),
        ('speed', float)
    ]


class RotorStates(MsgpackMixin):
    timestamp = np.uint64(0)
    rotors: list[RotorParameters] = []

    attribute_order = [
        ('rotors', list[RotorParameters]),
        ('timestamp', np.uint64)
    ]


class ProjectionMatrix(MsgpackMixin):
    matrix = []
    attribute_order = [
        ('matrix', list)
    ]


class CameraInfo(MsgpackMixin):
    pose = Pose()
    fov = -1
    proj_mat = ProjectionMatrix()

    attribute_order = [
        ('pose', Pose),
        ('fov', float),  # Field of View as Float
        ('proj_mat', ProjectionMatrix)
    ]


class LidarData(MsgpackMixin):
    time_stamp = np.uint64(0)
    point_cloud = 0.0
    pose = Pose()
    segmentation = 0

    attribute_order = [
        ('time_stamp', np.uint64),
        ('point_cloud', float),
        ('pose', Pose),
        ('segmentation', int)
    ]


class ImuData(MsgpackMixin):
    time_stamp = np.uint64(0)
    orientation = Quaternionr()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()

    attribute_order = [
        ('time_stamp', np.uint64),
        ('orientation', Quaternionr),
        ('angular_velocity', Vector3r),
        ('linear_acceleration', Vector3r)
    ]


class BarometerData(MsgpackMixin):
    time_stamp = np.uint64(0)
    altitude = Quaternionr()
    pressure = Vector3r()
    qnh = Vector3r()

    attribute_order = [
        ('time_stamp', np.uint64),
        ('altitude', float),
        ('pressure', float),
        ('qnh', float)
    ]


class MagnetometerData(MsgpackMixin):
    time_stamp = np.uint64(0)
    magnetic_field_body = Vector3r()
    magnetic_field_covariance = 0.0

    attribute_order = [
        ('time_stamp', np.uint64),
        ('magnetic_field_body', Vector3r),
        ('magnetic_field_covariance', float)
    ]


class GnssFixType(MsgpackMixin):
    GNSS_FIX_NO_FIX = 0
    GNSS_FIX_TIME_ONLY = 1
    GNSS_FIX_2D_FIX = 2
    GNSS_FIX_3D_FIX = 3


class GnssReport(MsgpackMixin):
    geo_point = GeoPoint()
    eph = 0.0
    epv = 0.0
    velocity = Vector3r()
    fix_type = GnssFixType()
    time_utc = np.uint64(0)

    attribute_order = [
        ('geo_point', GeoPoint),
        ('eph', float),
        ('epv', float),
        ('velocity', Vector3r),
        ('fix_type', int),
        ('time_utc', np.uint64)
    ]


class GpsData(MsgpackMixin):
    time_stamp = np.uint64(0)
    gnss = GnssReport()
    is_valid = False

    attribute_order = [
        ('time_stamp', np.uint64),
        ('gnss', GnssReport),
        ('is_valid', bool)
    ]


class DistanceSensorData(MsgpackMixin):
    time_stamp = np.uint64(0)
    distance = 0.0
    min_distance = 0.0
    max_distance = 0.0
    relative_pose = Pose()

    attribute_order = [
        ('time_stamp', np.uint64),
        ('distance', float),
        ('min_distance', float),
        ('max_distance', float),
        ('relative_pose', Pose)
    ]


class Box2D(MsgpackMixin):
    min = Vector2r()
    max = Vector2r()

    attribute_order = [
        ('min', Vector2r),
        ('max', Vector2r)
    ]


class Box3D(MsgpackMixin):
    min = Vector3r()
    max = Vector3r()

    attribute_order = [
        ('min', Vector3r),
        ('max', Vector3r)
    ]


class DetectionInfo(MsgpackMixin):
    name = ''
    geo_point = GeoPoint()
    box2D = Box2D()
    box3D = Box3D()
    relative_pose = Pose()

    attribute_order = [
        ('name', str),
        ('geo_point', GeoPoint),
        ('box2D', Box2D),
        ('box3D', Box3D),
        ('relative_pose', Pose)
    ]


class PIDGains():
    """
    Struct to store values of PID gains. Used to transmit controller gain values while instantiating
    AngleLevel/AngleRate/Velocity/PositionControllerGains objects.

    Attributes:
        kP (float): Proportional gain
        kI (float): Integrator gain
        kD (float): Derivative gain
    """

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def to_list(self):
        return [self.kp, self.ki, self.kd]


class AngleRateControllerGains():
    """
    Struct to contain controller gains used by angle level PID controller

    Attributes:
        roll_gains (PIDGains): kP, kI, kD for roll axis
        pitch_gains (PIDGains): kP, kI, kD for pitch axis
        yaw_gains (PIDGains): kP, kI, kD for yaw axis
    """

    def __init__(self, roll_gains=PIDGains(0.25, 0, 0),
                 pitch_gains=PIDGains(0.25, 0, 0),
                 yaw_gains=PIDGains(0.25, 0, 0)):
        self.roll_gains = roll_gains
        self.pitch_gains = pitch_gains
        self.yaw_gains = yaw_gains

    def to_lists(self):
        return [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp], [self.roll_gains.ki, self.pitch_gains.ki,
                                                                              self.yaw_gains.ki], [self.roll_gains.kd,
                                                                                                   self.pitch_gains.kd,
                                                                                                   self.yaw_gains.kd]


class AngleLevelControllerGains():
    """
    Struct to contain controller gains used by angle rate PID controller

    Attributes:
        roll_gains (PIDGains): kP, kI, kD for roll axis
        pitch_gains (PIDGains): kP, kI, kD for pitch axis
        yaw_gains (PIDGains): kP, kI, kD for yaw axis
    """

    def __init__(self, roll_gains=PIDGains(2.5, 0, 0),
                 pitch_gains=PIDGains(2.5, 0, 0),
                 yaw_gains=PIDGains(2.5, 0, 0)):
        self.roll_gains = roll_gains
        self.pitch_gains = pitch_gains
        self.yaw_gains = yaw_gains

    def to_lists(self):
        return [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp], [self.roll_gains.ki, self.pitch_gains.ki,
                                                                              self.yaw_gains.ki], [self.roll_gains.kd,
                                                                                                   self.pitch_gains.kd,
                                                                                                   self.yaw_gains.kd]


class VelocityControllerGains():
    """
    Struct to contain controller gains used by velocity PID controller

    Attributes:
        x_gains (PIDGains): kP, kI, kD for X axis
        y_gains (PIDGains): kP, kI, kD for Y axis
        z_gains (PIDGains): kP, kI, kD for Z axis
    """

    def __init__(self, x_gains=PIDGains(0.2, 0, 0),
                 y_gains=PIDGains(0.2, 0, 0),
                 z_gains=PIDGains(2.0, 2.0, 0)):
        self.x_gains = x_gains
        self.y_gains = y_gains
        self.z_gains = z_gains

    def to_lists(self):
        return [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp], [self.x_gains.ki, self.y_gains.ki,
                                                                     self.z_gains.ki], [self.x_gains.kd,
                                                                                        self.y_gains.kd,
                                                                                        self.z_gains.kd]


class PositionControllerGains():
    """
    Struct to contain controller gains used by position PID controller

    Attributes:
        x_gains (PIDGains): kP, kI, kD for X axis
        y_gains (PIDGains): kP, kI, kD for Y axis
        z_gains (PIDGains): kP, kI, kD for Z axis
    """

    def __init__(self, x_gains=PIDGains(0.25, 0, 0),
                 y_gains=PIDGains(0.25, 0, 0),
                 z_gains=PIDGains(0.25, 0, 0)):
        self.x_gains = x_gains
        self.y_gains = y_gains
        self.z_gains = z_gains

    def to_lists(self):
        return [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp], [self.x_gains.ki, self.y_gains.ki,
                                                                     self.z_gains.ki], [self.x_gains.kd,
                                                                                        self.y_gains.kd,
                                                                                        self.z_gains.kd]


class MeshPositionVertexBuffersResponse(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()
    vertices = 0.0
    indices = 0.0
    name = ''

    attribute_order = [
        ('position', Vector3r),
        ('orientation', Quaternionr),
        ('vertices', float),
        ('indices', float),
        ('name', str)
    ]
