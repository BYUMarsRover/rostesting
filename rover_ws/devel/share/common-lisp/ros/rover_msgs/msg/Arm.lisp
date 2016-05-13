; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude Arm.msg.html

(cl:defclass <Arm> (roslisp-msg-protocol:ros-message)
  ((tur
    :reader tur
    :initarg :tur
    :type cl:fixnum
    :initform 0)
   (sh
    :reader sh
    :initarg :sh
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Arm (<Arm>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Arm>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Arm)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<Arm> is deprecated: use rover_msgs-msg:Arm instead.")))

(cl:ensure-generic-function 'tur-val :lambda-list '(m))
(cl:defmethod tur-val ((m <Arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:tur-val is deprecated.  Use rover_msgs-msg:tur instead.")
  (tur m))

(cl:ensure-generic-function 'sh-val :lambda-list '(m))
(cl:defmethod sh-val ((m <Arm>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:sh-val is deprecated.  Use rover_msgs-msg:sh instead.")
  (sh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Arm>) ostream)
  "Serializes a message object of type '<Arm>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tur)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tur)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sh)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sh)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Arm>) istream)
  "Deserializes a message object of type '<Arm>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tur)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tur)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sh)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sh)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Arm>)))
  "Returns string type for a message object of type '<Arm>"
  "rover_msgs/Arm")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Arm)))
  "Returns string type for a message object of type 'Arm"
  "rover_msgs/Arm")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Arm>)))
  "Returns md5sum for a message object of type '<Arm>"
  "4a106f4b259cd6c1addd28eb4eb5100f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Arm)))
  "Returns md5sum for a message object of type 'Arm"
  "4a106f4b259cd6c1addd28eb4eb5100f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Arm>)))
  "Returns full string definition for message of type '<Arm>"
  (cl:format cl:nil "uint16 tur~%uint16 sh~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Arm)))
  "Returns full string definition for message of type 'Arm"
  (cl:format cl:nil "uint16 tur~%uint16 sh~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Arm>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Arm>))
  "Converts a ROS message object to a list"
  (cl:list 'Arm
    (cl:cons ':tur (tur msg))
    (cl:cons ':sh (sh msg))
))
