; Auto-generated. Do not edit!


(cl:in-package objectLocation0-msg)


;//! \htmlinclude coordinateArray.msg.html

(cl:defclass <coordinateArray> (roslisp-msg-protocol:ros-message)
  ((coordinates
    :reader coordinates
    :initarg :coordinates
    :type (cl:vector objectLocation0-msg:coordinateContent)
   :initform (cl:make-array 0 :element-type 'objectLocation0-msg:coordinateContent :initial-element (cl:make-instance 'objectLocation0-msg:coordinateContent))))
)

(cl:defclass coordinateArray (<coordinateArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <coordinateArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'coordinateArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name objectLocation0-msg:<coordinateArray> is deprecated: use objectLocation0-msg:coordinateArray instead.")))

(cl:ensure-generic-function 'coordinates-val :lambda-list '(m))
(cl:defmethod coordinates-val ((m <coordinateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader objectLocation0-msg:coordinates-val is deprecated.  Use objectLocation0-msg:coordinates instead.")
  (coordinates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <coordinateArray>) ostream)
  "Serializes a message object of type '<coordinateArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coordinates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'coordinates))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <coordinateArray>) istream)
  "Deserializes a message object of type '<coordinateArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coordinates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coordinates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'objectLocation0-msg:coordinateContent))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<coordinateArray>)))
  "Returns string type for a message object of type '<coordinateArray>"
  "objectLocation0/coordinateArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'coordinateArray)))
  "Returns string type for a message object of type 'coordinateArray"
  "objectLocation0/coordinateArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<coordinateArray>)))
  "Returns md5sum for a message object of type '<coordinateArray>"
  "5850328c4be1cfc40d068810a54cfe4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'coordinateArray)))
  "Returns md5sum for a message object of type 'coordinateArray"
  "5850328c4be1cfc40d068810a54cfe4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<coordinateArray>)))
  "Returns full string definition for message of type '<coordinateArray>"
  (cl:format cl:nil "coordinateContent[] coordinates~%~%================================================================================~%MSG: objectLocation0/coordinateContent~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'coordinateArray)))
  "Returns full string definition for message of type 'coordinateArray"
  (cl:format cl:nil "coordinateContent[] coordinates~%~%================================================================================~%MSG: objectLocation0/coordinateContent~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <coordinateArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coordinates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <coordinateArray>))
  "Converts a ROS message object to a list"
  (cl:list 'coordinateArray
    (cl:cons ':coordinates (coordinates msg))
))
