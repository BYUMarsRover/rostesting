; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude All.msg.html

(cl:defclass <All> (roslisp-msg-protocol:ros-message)
  ((lw
    :reader lw
    :initarg :lw
    :type cl:fixnum
    :initform 0)
   (rw
    :reader rw
    :initarg :rw
    :type cl:fixnum
    :initform 0)
   (pan
    :reader pan
    :initarg :pan
    :type cl:fixnum
    :initform 0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:fixnum
    :initform 0)
   (camnum
    :reader camnum
    :initarg :camnum
    :type cl:fixnum
    :initform 0)
   (q1
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
    :initform 0)
   (chutes
    :reader chutes
    :initarg :chutes
    :type cl:fixnum
    :initform 0)
   (shovel
    :reader shovel
    :initarg :shovel
    :type cl:fixnum
    :initform 0))
)

(cl:defclass All (<All>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <All>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'All)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<All> is deprecated: use rover_msgs-msg:All instead.")))

(cl:ensure-generic-function 'lw-val :lambda-list '(m))
(cl:defmethod lw-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:lw-val is deprecated.  Use rover_msgs-msg:lw instead.")
  (lw m))

(cl:ensure-generic-function 'rw-val :lambda-list '(m))
(cl:defmethod rw-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:rw-val is deprecated.  Use rover_msgs-msg:rw instead.")
  (rw m))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:pan-val is deprecated.  Use rover_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:tilt-val is deprecated.  Use rover_msgs-msg:tilt instead.")
  (tilt m))

(cl:ensure-generic-function 'camnum-val :lambda-list '(m))
(cl:defmethod camnum-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:camnum-val is deprecated.  Use rover_msgs-msg:camnum instead.")
  (camnum m))

(cl:ensure-generic-function 'q1-val :lambda-list '(m))
(cl:defmethod q1-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q1-val is deprecated.  Use rover_msgs-msg:q1 instead.")
  (q1 m))

(cl:ensure-generic-function 'q2-val :lambda-list '(m))
(cl:defmethod q2-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q2-val is deprecated.  Use rover_msgs-msg:q2 instead.")
  (q2 m))

(cl:ensure-generic-function 'q3-val :lambda-list '(m))
(cl:defmethod q3-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q3-val is deprecated.  Use rover_msgs-msg:q3 instead.")
  (q3 m))

(cl:ensure-generic-function 'q4-val :lambda-list '(m))
(cl:defmethod q4-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q4-val is deprecated.  Use rover_msgs-msg:q4 instead.")
  (q4 m))

(cl:ensure-generic-function 'q5-val :lambda-list '(m))
(cl:defmethod q5-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q5-val is deprecated.  Use rover_msgs-msg:q5 instead.")
  (q5 m))

(cl:ensure-generic-function 'q6-val :lambda-list '(m))
(cl:defmethod q6-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:q6-val is deprecated.  Use rover_msgs-msg:q6 instead.")
  (q6 m))

(cl:ensure-generic-function 'grip-val :lambda-list '(m))
(cl:defmethod grip-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:grip-val is deprecated.  Use rover_msgs-msg:grip instead.")
  (grip m))

(cl:ensure-generic-function 'chutes-val :lambda-list '(m))
(cl:defmethod chutes-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:chutes-val is deprecated.  Use rover_msgs-msg:chutes instead.")
  (chutes m))

(cl:ensure-generic-function 'shovel-val :lambda-list '(m))
(cl:defmethod shovel-val ((m <All>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:shovel-val is deprecated.  Use rover_msgs-msg:shovel instead.")
  (shovel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <All>) ostream)
  "Serializes a message object of type '<All>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pan)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pan)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tilt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tilt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camnum)) ostream)
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chutes)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shovel)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shovel)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <All>) istream)
  "Deserializes a message object of type '<All>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pan)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pan)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tilt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tilt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camnum)) (cl:read-byte istream))
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'chutes)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'shovel)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'shovel)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<All>)))
  "Returns string type for a message object of type '<All>"
  "rover_msgs/All")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'All)))
  "Returns string type for a message object of type 'All"
  "rover_msgs/All")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<All>)))
  "Returns md5sum for a message object of type '<All>"
  "d58546379b54b62019e6cb68da5f967c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'All)))
  "Returns md5sum for a message object of type 'All"
  "d58546379b54b62019e6cb68da5f967c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<All>)))
  "Returns full string definition for message of type '<All>"
  (cl:format cl:nil "uint16 lw~%uint16 rw~%uint16 pan~%uint16 tilt~%uint8 camnum~%uint16 q1~%uint16 q2~%uint16 q3~%uint16 q4~%uint16 q5~%uint16 q6~%uint16 grip~%uint8 chutes~%uint16 shovel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'All)))
  "Returns full string definition for message of type 'All"
  (cl:format cl:nil "uint16 lw~%uint16 rw~%uint16 pan~%uint16 tilt~%uint8 camnum~%uint16 q1~%uint16 q2~%uint16 q3~%uint16 q4~%uint16 q5~%uint16 q6~%uint16 grip~%uint8 chutes~%uint16 shovel~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <All>))
  (cl:+ 0
     2
     2
     2
     2
     1
     2
     2
     2
     2
     2
     2
     2
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <All>))
  "Converts a ROS message object to a list"
  (cl:list 'All
    (cl:cons ':lw (lw msg))
    (cl:cons ':rw (rw msg))
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
    (cl:cons ':camnum (camnum msg))
    (cl:cons ':q1 (q1 msg))
    (cl:cons ':q2 (q2 msg))
    (cl:cons ':q3 (q3 msg))
    (cl:cons ':q4 (q4 msg))
    (cl:cons ':q5 (q5 msg))
    (cl:cons ':q6 (q6 msg))
    (cl:cons ':grip (grip msg))
    (cl:cons ':chutes (chutes msg))
    (cl:cons ':shovel (shovel msg))
))
