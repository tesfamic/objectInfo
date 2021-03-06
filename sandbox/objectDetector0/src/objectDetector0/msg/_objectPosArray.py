"""autogenerated by genpy from objectDetector0/objectPosArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import objectDetector0.msg

class objectPosArray(genpy.Message):
  _md5sum = "80d8fbab3d3cc04c5f7737732f132d93"
  _type = "objectDetector0/objectPosArray"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """objectPos[] position

================================================================================
MSG: objectDetector0/objectPos
int16 width
int16 height

"""
  __slots__ = ['position']
  _slot_types = ['objectDetector0/objectPos[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       position

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(objectPosArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.position is None:
        self.position = []
    else:
      self.position = []

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
      length = len(self.position)
      buff.write(_struct_I.pack(length))
      for val1 in self.position:
        _x = val1
        buff.write(_struct_2h.pack(_x.width, _x.height))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.position is None:
        self.position = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.position = []
      for i in range(0, length):
        val1 = objectDetector0.msg.objectPos()
        _x = val1
        start = end
        end += 4
        (_x.width, _x.height,) = _struct_2h.unpack(str[start:end])
        self.position.append(val1)
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
      length = len(self.position)
      buff.write(_struct_I.pack(length))
      for val1 in self.position:
        _x = val1
        buff.write(_struct_2h.pack(_x.width, _x.height))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.position is None:
        self.position = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.position = []
      for i in range(0, length):
        val1 = objectDetector0.msg.objectPos()
        _x = val1
        start = end
        end += 4
        (_x.width, _x.height,) = _struct_2h.unpack(str[start:end])
        self.position.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2h = struct.Struct("<2h")
