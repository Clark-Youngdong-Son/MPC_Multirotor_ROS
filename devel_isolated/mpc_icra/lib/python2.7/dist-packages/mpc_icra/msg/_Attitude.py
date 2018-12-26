# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mpc_icra/Attitude.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class Attitude(genpy.Message):
  _md5sum = "157ec3ce1e65802d81d0f3d7eae23348"
  _type = "mpc_icra/Attitude"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64 t_now
float64 roll_d
float64 pitch_d
float64 yaw_d
float64 p_d
float64 q_d
float64 r_d
float64 z_old
float64 z_d
float64 z_dot_old
float64 z_dot_d
"""
  __slots__ = ['t_now','roll_d','pitch_d','yaw_d','p_d','q_d','r_d','z_old','z_d','z_dot_old','z_dot_d']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       t_now,roll_d,pitch_d,yaw_d,p_d,q_d,r_d,z_old,z_d,z_dot_old,z_dot_d

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Attitude, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.t_now is None:
        self.t_now = 0.
      if self.roll_d is None:
        self.roll_d = 0.
      if self.pitch_d is None:
        self.pitch_d = 0.
      if self.yaw_d is None:
        self.yaw_d = 0.
      if self.p_d is None:
        self.p_d = 0.
      if self.q_d is None:
        self.q_d = 0.
      if self.r_d is None:
        self.r_d = 0.
      if self.z_old is None:
        self.z_old = 0.
      if self.z_d is None:
        self.z_d = 0.
      if self.z_dot_old is None:
        self.z_dot_old = 0.
      if self.z_dot_d is None:
        self.z_dot_d = 0.
    else:
      self.t_now = 0.
      self.roll_d = 0.
      self.pitch_d = 0.
      self.yaw_d = 0.
      self.p_d = 0.
      self.q_d = 0.
      self.r_d = 0.
      self.z_old = 0.
      self.z_d = 0.
      self.z_dot_old = 0.
      self.z_dot_d = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_11d.pack(_x.t_now, _x.roll_d, _x.pitch_d, _x.yaw_d, _x.p_d, _x.q_d, _x.r_d, _x.z_old, _x.z_d, _x.z_dot_old, _x.z_dot_d))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 88
      (_x.t_now, _x.roll_d, _x.pitch_d, _x.yaw_d, _x.p_d, _x.q_d, _x.r_d, _x.z_old, _x.z_d, _x.z_dot_old, _x.z_dot_d,) = _struct_11d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_11d.pack(_x.t_now, _x.roll_d, _x.pitch_d, _x.yaw_d, _x.p_d, _x.q_d, _x.r_d, _x.z_old, _x.z_d, _x.z_dot_old, _x.z_dot_d))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 88
      (_x.t_now, _x.roll_d, _x.pitch_d, _x.yaw_d, _x.p_d, _x.q_d, _x.r_d, _x.z_old, _x.z_d, _x.z_dot_old, _x.z_dot_d,) = _struct_11d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_11d = struct.Struct("<11d")