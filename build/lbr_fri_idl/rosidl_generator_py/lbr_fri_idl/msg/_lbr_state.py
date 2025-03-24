# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lbr_fri_idl:msg/LBRState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'measured_joint_position'
# Member 'commanded_joint_position'
# Member 'measured_torque'
# Member 'commanded_torque'
# Member 'external_torque'
# Member 'ipo_joint_position'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LBRState(type):
    """Metaclass of message 'LBRState'."""

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
            module = import_type_support('lbr_fri_idl')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'lbr_fri_idl.msg.LBRState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__lbr_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__lbr_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__lbr_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__lbr_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__lbr_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LBRState(metaclass=Metaclass_LBRState):
    """Message class 'LBRState'."""

    __slots__ = [
        '_sample_time',
        '_session_state',
        '_connection_quality',
        '_safety_state',
        '_operation_mode',
        '_drive_state',
        '_client_command_mode',
        '_overlay_type',
        '_control_mode',
        '_time_stamp_sec',
        '_time_stamp_nano_sec',
        '_measured_joint_position',
        '_commanded_joint_position',
        '_measured_torque',
        '_commanded_torque',
        '_external_torque',
        '_ipo_joint_position',
        '_tracking_performance',
    ]

    _fields_and_field_types = {
        'sample_time': 'double',
        'session_state': 'int8',
        'connection_quality': 'int8',
        'safety_state': 'int8',
        'operation_mode': 'int8',
        'drive_state': 'int8',
        'client_command_mode': 'int8',
        'overlay_type': 'int8',
        'control_mode': 'int8',
        'time_stamp_sec': 'uint32',
        'time_stamp_nano_sec': 'uint32',
        'measured_joint_position': 'double[7]',
        'commanded_joint_position': 'double[7]',
        'measured_torque': 'double[7]',
        'commanded_torque': 'double[7]',
        'external_torque': 'double[7]',
        'ipo_joint_position': 'double[7]',
        'tracking_performance': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 7),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sample_time = kwargs.get('sample_time', float())
        self.session_state = kwargs.get('session_state', int())
        self.connection_quality = kwargs.get('connection_quality', int())
        self.safety_state = kwargs.get('safety_state', int())
        self.operation_mode = kwargs.get('operation_mode', int())
        self.drive_state = kwargs.get('drive_state', int())
        self.client_command_mode = kwargs.get('client_command_mode', int())
        self.overlay_type = kwargs.get('overlay_type', int())
        self.control_mode = kwargs.get('control_mode', int())
        self.time_stamp_sec = kwargs.get('time_stamp_sec', int())
        self.time_stamp_nano_sec = kwargs.get('time_stamp_nano_sec', int())
        if 'measured_joint_position' not in kwargs:
            self.measured_joint_position = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.measured_joint_position = numpy.array(kwargs.get('measured_joint_position'), dtype=numpy.float64)
            assert self.measured_joint_position.shape == (7, )
        if 'commanded_joint_position' not in kwargs:
            self.commanded_joint_position = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.commanded_joint_position = numpy.array(kwargs.get('commanded_joint_position'), dtype=numpy.float64)
            assert self.commanded_joint_position.shape == (7, )
        if 'measured_torque' not in kwargs:
            self.measured_torque = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.measured_torque = numpy.array(kwargs.get('measured_torque'), dtype=numpy.float64)
            assert self.measured_torque.shape == (7, )
        if 'commanded_torque' not in kwargs:
            self.commanded_torque = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.commanded_torque = numpy.array(kwargs.get('commanded_torque'), dtype=numpy.float64)
            assert self.commanded_torque.shape == (7, )
        if 'external_torque' not in kwargs:
            self.external_torque = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.external_torque = numpy.array(kwargs.get('external_torque'), dtype=numpy.float64)
            assert self.external_torque.shape == (7, )
        if 'ipo_joint_position' not in kwargs:
            self.ipo_joint_position = numpy.zeros(7, dtype=numpy.float64)
        else:
            self.ipo_joint_position = numpy.array(kwargs.get('ipo_joint_position'), dtype=numpy.float64)
            assert self.ipo_joint_position.shape == (7, )
        self.tracking_performance = kwargs.get('tracking_performance', float())

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
        if self.sample_time != other.sample_time:
            return False
        if self.session_state != other.session_state:
            return False
        if self.connection_quality != other.connection_quality:
            return False
        if self.safety_state != other.safety_state:
            return False
        if self.operation_mode != other.operation_mode:
            return False
        if self.drive_state != other.drive_state:
            return False
        if self.client_command_mode != other.client_command_mode:
            return False
        if self.overlay_type != other.overlay_type:
            return False
        if self.control_mode != other.control_mode:
            return False
        if self.time_stamp_sec != other.time_stamp_sec:
            return False
        if self.time_stamp_nano_sec != other.time_stamp_nano_sec:
            return False
        if all(self.measured_joint_position != other.measured_joint_position):
            return False
        if all(self.commanded_joint_position != other.commanded_joint_position):
            return False
        if all(self.measured_torque != other.measured_torque):
            return False
        if all(self.commanded_torque != other.commanded_torque):
            return False
        if all(self.external_torque != other.external_torque):
            return False
        if all(self.ipo_joint_position != other.ipo_joint_position):
            return False
        if self.tracking_performance != other.tracking_performance:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sample_time(self):
        """Message field 'sample_time'."""
        return self._sample_time

    @sample_time.setter
    def sample_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'sample_time' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'sample_time' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._sample_time = value

    @builtins.property
    def session_state(self):
        """Message field 'session_state'."""
        return self._session_state

    @session_state.setter
    def session_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'session_state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'session_state' field must be an integer in [-128, 127]"
        self._session_state = value

    @builtins.property
    def connection_quality(self):
        """Message field 'connection_quality'."""
        return self._connection_quality

    @connection_quality.setter
    def connection_quality(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'connection_quality' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'connection_quality' field must be an integer in [-128, 127]"
        self._connection_quality = value

    @builtins.property
    def safety_state(self):
        """Message field 'safety_state'."""
        return self._safety_state

    @safety_state.setter
    def safety_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'safety_state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'safety_state' field must be an integer in [-128, 127]"
        self._safety_state = value

    @builtins.property
    def operation_mode(self):
        """Message field 'operation_mode'."""
        return self._operation_mode

    @operation_mode.setter
    def operation_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'operation_mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'operation_mode' field must be an integer in [-128, 127]"
        self._operation_mode = value

    @builtins.property
    def drive_state(self):
        """Message field 'drive_state'."""
        return self._drive_state

    @drive_state.setter
    def drive_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'drive_state' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'drive_state' field must be an integer in [-128, 127]"
        self._drive_state = value

    @builtins.property
    def client_command_mode(self):
        """Message field 'client_command_mode'."""
        return self._client_command_mode

    @client_command_mode.setter
    def client_command_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'client_command_mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'client_command_mode' field must be an integer in [-128, 127]"
        self._client_command_mode = value

    @builtins.property
    def overlay_type(self):
        """Message field 'overlay_type'."""
        return self._overlay_type

    @overlay_type.setter
    def overlay_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'overlay_type' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'overlay_type' field must be an integer in [-128, 127]"
        self._overlay_type = value

    @builtins.property
    def control_mode(self):
        """Message field 'control_mode'."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'control_mode' field must be an integer in [-128, 127]"
        self._control_mode = value

    @builtins.property
    def time_stamp_sec(self):
        """Message field 'time_stamp_sec'."""
        return self._time_stamp_sec

    @time_stamp_sec.setter
    def time_stamp_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_stamp_sec' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_stamp_sec' field must be an unsigned integer in [0, 4294967295]"
        self._time_stamp_sec = value

    @builtins.property
    def time_stamp_nano_sec(self):
        """Message field 'time_stamp_nano_sec'."""
        return self._time_stamp_nano_sec

    @time_stamp_nano_sec.setter
    def time_stamp_nano_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_stamp_nano_sec' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_stamp_nano_sec' field must be an unsigned integer in [0, 4294967295]"
        self._time_stamp_nano_sec = value

    @builtins.property
    def measured_joint_position(self):
        """Message field 'measured_joint_position'."""
        return self._measured_joint_position

    @measured_joint_position.setter
    def measured_joint_position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'measured_joint_position' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'measured_joint_position' numpy.ndarray() must have a size of 7"
            self._measured_joint_position = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'measured_joint_position' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._measured_joint_position = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def commanded_joint_position(self):
        """Message field 'commanded_joint_position'."""
        return self._commanded_joint_position

    @commanded_joint_position.setter
    def commanded_joint_position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'commanded_joint_position' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'commanded_joint_position' numpy.ndarray() must have a size of 7"
            self._commanded_joint_position = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'commanded_joint_position' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._commanded_joint_position = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def measured_torque(self):
        """Message field 'measured_torque'."""
        return self._measured_torque

    @measured_torque.setter
    def measured_torque(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'measured_torque' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'measured_torque' numpy.ndarray() must have a size of 7"
            self._measured_torque = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'measured_torque' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._measured_torque = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def commanded_torque(self):
        """Message field 'commanded_torque'."""
        return self._commanded_torque

    @commanded_torque.setter
    def commanded_torque(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'commanded_torque' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'commanded_torque' numpy.ndarray() must have a size of 7"
            self._commanded_torque = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'commanded_torque' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._commanded_torque = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def external_torque(self):
        """Message field 'external_torque'."""
        return self._external_torque

    @external_torque.setter
    def external_torque(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'external_torque' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'external_torque' numpy.ndarray() must have a size of 7"
            self._external_torque = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'external_torque' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._external_torque = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def ipo_joint_position(self):
        """Message field 'ipo_joint_position'."""
        return self._ipo_joint_position

    @ipo_joint_position.setter
    def ipo_joint_position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'ipo_joint_position' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 7, \
                "The 'ipo_joint_position' numpy.ndarray() must have a size of 7"
            self._ipo_joint_position = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 7 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'ipo_joint_position' field must be a set or sequence with length 7 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._ipo_joint_position = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def tracking_performance(self):
        """Message field 'tracking_performance'."""
        return self._tracking_performance

    @tracking_performance.setter
    def tracking_performance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tracking_performance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tracking_performance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tracking_performance = value
