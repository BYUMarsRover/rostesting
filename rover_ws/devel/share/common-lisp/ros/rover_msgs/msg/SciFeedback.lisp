; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude SciFeedback.msg.html

(cl:defclass <SciFeedback> (roslisp-msg-protocol:ros-message)
  ((temp
    :reader temp
    :initarg :temp
    :type cl:fixnum
    :initform 0)
   (humidity
    :reader humidity
    :initarg :humidity
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SciFeedback (<SciFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SciFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SciFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<SciFeedback> is deprecated: use rover_msgs-msg:SciFeedback instead.")))

(cl:ensure-generic-function 'temp-val :lambda-list '(m))
(cl:defmethod temp-val ((m <SciFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:temp-val is deprecated.  Use rover_msgs-msg:temp instead.")
  (temp m))

(cl:ensure-generic-function 'humidity-val :lambda-list '(m))
(cl:defmethod humidity-val ((m <SciFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:humidity-val is deprecated.  Use rover_msgs-msg:humidity instead.")
  (humidity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SciFeedback>) ostream)
  "Serializes a message object of type '<SciFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'temp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'humidity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'humidity)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SciFeedback>) istream)
  "Deserializes a message object of type '<SciFeedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'temp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'humidity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'humidity)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SciFeedback>)))
  "Returns string type for a message object of type '<SciFeedback>"
  "rover_msgs/SciFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SciFeedback)))
  "Returns string type for a message object of type 'SciFeedback"
  "rover_msgs/SciFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SciFeedback>)))
  "Returns md5sum for a message object of type '<SciFeedback>"
  "5129e2448ce73c4ae0b8d277356e0494")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SciFeedback)))
  "Returns md5sum for a message object of type 'SciFeedback"
  "5129e2448ce73c4ae0b8d277356e0494")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SciFeedback>)))
  "Returns full string definition for message of type '<SciFeedback>"
  (cl:format cl:nil "uint16 temp~%uint16 humidity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SciFeedback)))
  "Returns full string definition for message of type 'SciFeedback"
  (cl:format cl:nil "uint16 temp~%uint16 humidity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SciFeedback>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SciFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'SciFeedback
    (cl:cons ':temp (temp msg))
    (cl:cons ':humidity (humidity msg))
))
