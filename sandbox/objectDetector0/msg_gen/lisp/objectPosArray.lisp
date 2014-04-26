; Auto-generated. Do not edit!


(cl:in-package objectDetector0-msg)


;//! \htmlinclude objectPosArray.msg.html

(cl:defclass <objectPosArray> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type (cl:vector objectDetector0-msg:objectPos)
   :initform (cl:make-array 0 :element-type 'objectDetector0-msg:objectPos :initial-element (cl:make-instance 'objectDetector0-msg:objectPos))))
)

(cl:defclass objectPosArray (<objectPosArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <objectPosArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'objectPosArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objectDetector0-msg:<objectPosArray> is deprecated: use objectDetector0-msg:objectPosArray instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <objectPosArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objectDetector0-msg:position-val is deprecated.  Use objectDetector0-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <objectPosArray>) ostream)
  "Serializes a message object of type '<objectPosArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'position))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <objectPosArray>) istream)
  "Deserializes a message object of type '<objectPosArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'objectDetector0-msg:objectPos))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<objectPosArray>)))
  "Returns string type for a message object of type '<objectPosArray>"
  "objectDetector0/objectPosArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'objectPosArray)))
  "Returns string type for a message object of type 'objectPosArray"
  "objectDetector0/objectPosArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<objectPosArray>)))
  "Returns md5sum for a message object of type '<objectPosArray>"
  "80d8fbab3d3cc04c5f7737732f132d93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'objectPosArray)))
  "Returns md5sum for a message object of type 'objectPosArray"
  "80d8fbab3d3cc04c5f7737732f132d93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<objectPosArray>)))
  "Returns full string definition for message of type '<objectPosArray>"
  (cl:format cl:nil "objectPos[] position~%~%================================================================================~%MSG: objectDetector0/objectPos~%int16 width~%int16 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'objectPosArray)))
  "Returns full string definition for message of type 'objectPosArray"
  (cl:format cl:nil "objectPos[] position~%~%================================================================================~%MSG: objectDetector0/objectPos~%int16 width~%int16 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <objectPosArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <objectPosArray>))
  "Converts a ROS message object to a list"
  (cl:list 'objectPosArray
    (cl:cons ':position (position msg))
))
