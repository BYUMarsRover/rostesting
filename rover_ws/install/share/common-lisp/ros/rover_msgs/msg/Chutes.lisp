; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude Chutes.msg.html

(cl:defclass <Chutes> (roslisp-msg-protocol:ros-message)
  ((chutes
    :reader chutes
    :initarg :chutes
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Chutes (<Chutes>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Chutes>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Chutes)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<Chutes> is deprecated: use rover_msgs-msg:Chutes instead.")))

(cl:ensure-generic-function 'chutes-val :lambda-list '(m))
(cl:defmethod chutes-val ((m <Chutes>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:chutes-val is deprecated.  Use rover_msgs-msg:chutes instead.")
  (chutes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Chutes>) ostream)
  "Serializes a message object of type '<Chutes>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chutes)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'chutes)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Chutes>) istream)
  "Deserializes a message object of type '<Chutes>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chutes)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'chutes)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Chutes>)))
  "Returns string type for a message object of type '<Chutes>"
  "rover_msgs/Chutes")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Chutes)))
  "Returns string type for a message object of type 'Chutes"
  "rover_msgs/Chutes")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Chutes>)))
  "Returns md5sum for a message object of type '<Chutes>"
  "735dabca75806217d50ebaff1a183d1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Chutes)))
  "Returns md5sum for a message object of type 'Chutes"
  "735dabca75806217d50ebaff1a183d1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Chutes>)))
  "Returns full string definition for message of type '<Chutes>"
  (cl:format cl:nil "uint16 chutes~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Chutes)))
  "Returns full string definition for message of type 'Chutes"
  (cl:format cl:nil "uint16 chutes~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Chutes>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Chutes>))
  "Converts a ROS message object to a list"
  (cl:list 'Chutes
    (cl:cons ':chutes (chutes msg))
))
