; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude Pololu.msg.html

(cl:defclass <Pololu> (roslisp-msg-protocol:ros-message)
  ((q1
    :reader q1
    :initarg :q1
    :type cl:fixnum
    :initform 0)
   (q2
    :reader q2
    :initarg :q2
    :type cl:fixnum
    :initform 0)
   (q3
    :reader q3
    :initarg :q3
    :type cl:fixnum
    :initform 0)
   (q4
    :reader q4
    :initarg :q4
    :type cl:fixnum
    :initform 0)
   (q5
    :reader q5
    :initarg :q5
    :type cl:fixnum
    :initform 0)
   (q6
    :reader q6
    :initarg :q6
    :type cl:fixnum
    :initform 0)
   (grip
    :reader grip
    :initarg :grip
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Pololu (<Pololu>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pololu>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pololu)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<Pololu> is deprecated: use rover_msgs-msg:Pololu instead.")))

(cl:ensure-generic-function 'q1-val :lambda-list '(m))
(cl:defmethod q1-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q1-val is deprecated.  Use rover_msgs-msg:q1 instead.")
  (q1 m))

(cl:ensure-generic-function 'q2-val :lambda-list '(m))
(cl:defmethod q2-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q2-val is deprecated.  Use rover_msgs-msg:q2 instead.")
  (q2 m))

(cl:ensure-generic-function 'q3-val :lambda-list '(m))
(cl:defmethod q3-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q3-val is deprecated.  Use rover_msgs-msg:q3 instead.")
  (q3 m))

(cl:ensure-generic-function 'q4-val :lambda-list '(m))
(cl:defmethod q4-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q4-val is deprecated.  Use rover_msgs-msg:q4 instead.")
  (q4 m))

(cl:ensure-generic-function 'q5-val :lambda-list '(m))
(cl:defmethod q5-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q5-val is deprecated.  Use rover_msgs-msg:q5 instead.")
  (q5 m))

(cl:ensure-generic-function 'q6-val :lambda-list '(m))
(cl:defmethod q6-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q6-val is deprecated.  Use rover_msgs-msg:q6 instead.")
  (q6 m))

(cl:ensure-generic-function 'grip-val :lambda-list '(m))
(cl:defmethod grip-val ((m <Pololu>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:grip-val is deprecated.  Use rover_msgs-msg:grip instead.")
  (grip m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pololu>) ostream)
  "Serializes a message object of type '<Pololu>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q5)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q6)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'grip)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'grip)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pololu>) istream)
  "Deserializes a message object of type '<Pololu>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q5)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'q6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'q6)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'grip)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'grip)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pololu>)))
  "Returns string type for a message object of type '<Pololu>"
  "rover_msgs/Pololu")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pololu)))
  "Returns string type for a message object of type 'Pololu"
  "rover_msgs/Pololu")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pololu>)))
  "Returns md5sum for a message object of type '<Pololu>"
  "981b3dedffc3376c9ed31b487f6742c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pololu)))
  "Returns md5sum for a message object of type 'Pololu"
  "981b3dedffc3376c9ed31b487f6742c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pololu>)))
  "Returns full string definition for message of type '<Pololu>"
  (cl:format cl:nil "uint16 q1~%uint16 q2~%uint16 q3~%uint16 q4~%uint16 q5~%uint16 q6~%uint16 grip~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pololu)))
  "Returns full string definition for message of type 'Pololu"
  (cl:format cl:nil "uint16 q1~%uint16 q2~%uint16 q3~%uint16 q4~%uint16 q5~%uint16 q6~%uint16 grip~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pololu>))
  (cl:+ 0
     2
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pololu>))
  "Converts a ROS message object to a list"
  (cl:list 'Pololu
    (cl:cons ':q1 (q1 msg))
    (cl:cons ':q2 (q2 msg))
    (cl:cons ':q3 (q3 msg))
    (cl:cons ':q4 (q4 msg))
    (cl:cons ':q5 (q5 msg))
    (cl:cons ':q6 (q6 msg))
    (cl:cons ':grip (grip msg))
))
