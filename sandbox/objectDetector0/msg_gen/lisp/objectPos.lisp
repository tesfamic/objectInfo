; Auto-generated. Do not edit!


(cl:in-package objectDetector0-msg)


;//! \htmlinclude objectPos.msg.html

(cl:defclass <objectPos> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0))
)

(cl:defclass objectPos (<objectPos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objectPos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objectPos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objectDetector0-msg:<objectPos> is deprecated: use objectDetector0-msg:objectPos instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <objectPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objectDetector0-msg:width-val is deprecated.  Use objectDetector0-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <objectPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objectDetector0-msg:height-val is deprecated.  Use objectDetector0-msg:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objectPos>) ostream)
  "Serializes a message object of type '<objectPos>"
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objectPos>) istream)
  "Deserializes a message object of type '<objectPos>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objectPos>)))
  "Returns string type for a message object of type '<objectPos>"
  "objectDetector0/objectPos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objectPos)))
  "Returns string type for a message object of type 'objectPos"
  "objectDetector0/objectPos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objectPos>)))
  "Returns md5sum for a message object of type '<objectPos>"
  "5d7f0e302480e684afa7baf49b775bf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objectPos)))
  "Returns md5sum for a message object of type 'objectPos"
  "5d7f0e302480e684afa7baf49b775bf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objectPos>)))
  "Returns full string definition for message of type '<objectPos>"
  (cl:format cl:nil "int16 width~%int16 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objectPos)))
  "Returns full string definition for message of type 'objectPos"
  (cl:format cl:nil "int16 width~%int16 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objectPos>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objectPos>))
  "Converts a ROS message object to a list"
  (cl:list 'objectPos
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
