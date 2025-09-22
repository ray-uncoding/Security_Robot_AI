# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ackermann_msgs:msg/AckermannDrive.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AckermannDrive(type):
    """Metaclass of message 'AckermannDrive'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ackermann_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ackermann_msgs.msg.AckermannDrive')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ackermann_drive
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ackermann_drive
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ackermann_drive
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ackermann_drive
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ackermann_drive

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AckermannDrive(metaclass=Metaclass_AckermannDrive):
    """Message class 'AckermannDrive'."""

    __slots__ = [
        '_steering_angle',
        '_steering_angle_velocity',
        '_speed',
        '_acceleration',
        '_jerk',
    ]

    _fields_and_field_types = {
        'steering_angle': 'float',
        'steering_angle_velocity': 'float',
        'speed': 'float',
        'acceleration': 'float',
        'jerk': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.steering_angle = kwargs.get('steering_angle', float())
        self.steering_angle_velocity = kwargs.get('steering_angle_velocity', float())
        self.speed = kwargs.get('speed', float())
        self.acceleration = kwargs.get('acceleration', float())
        self.jerk = kwargs.get('jerk', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.steering_angle != other.steering_angle:
            return False
        if self.steering_angle_velocity != other.steering_angle_velocity:
            return False
        if self.speed != other.speed:
            return False
        if self.acceleration != other.acceleration:
            return False
        if self.jerk != other.jerk:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def steering_angle(self):
        """Message field 'steering_angle'."""
        return self._steering_angle

    @steering_angle.setter
    def steering_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'steering_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._steering_angle = value

    @builtins.property
    def steering_angle_velocity(self):
        """Message field 'steering_angle_velocity'."""
        return self._steering_angle_velocity

    @steering_angle_velocity.setter
    def steering_angle_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'steering_angle_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'steering_angle_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._steering_angle_velocity = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed = value

    @builtins.property
    def acceleration(self):
        """Message field 'acceleration'."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acceleration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'acceleration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._acceleration = value

    @builtins.property
    def jerk(self):
        """Message field 'jerk'."""
        return self._jerk

    @jerk.setter
    def jerk(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'jerk' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'jerk' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._jerk = value
