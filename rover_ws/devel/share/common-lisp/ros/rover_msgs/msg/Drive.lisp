; Auto-generated. Do not edit!


(cl:in-package rover_msgs-msg)


;//! \htmlinclude Drive.msg.html

(cl:defclass <Drive> (roslisp-msg-protocol:ros-message)
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
    :initform 0))
)

(cl:defclass Drive (<Drive>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Drive>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Drive)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_msgs-msg:<Drive> is deprecated: use rover_msgs-msg:Drive instead.")))

(cl:ensure-generic-function 'lw-val :lambda-list '(m))
(cl:defmethod lw-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:lw-val is deprecated.  Use rover_msgs-msg:lw instead.")
  (lw m))

(cl:ensure-generic-function 'rw-val :lambda-list '(m))
(cl:defmethod rw-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:rw-val is deprecated.  Use rover_msgs-msg:rw instead.")
  (rw m))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:pan-val is deprecated.  Use rover_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:tilt-val is deprecated.  Use rover_msgs-msg:tilt instead.")
  (tilt m))

(cl:ensure-generic-function 'camnum-val :lambda-list '(m))
(cl:defmethod camnum-val ((m <Drive>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_msgs-msg:camnum-val is deprecated.  Use rover_msgs-msg:camnum instead.")
  (camnum m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Drive>) ostream)
  "Serializes a message object of type '<Drive>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rw)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pan)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pan)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tilt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tilt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camnum)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Drive>) istream)
  "Deserializes a message object of type '<Drive>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'lw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'lw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'rw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'rw)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pan)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pan)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tilt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'tilt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camnum)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Drive>)))
  "Returns string type for a message object of type '<Drive>"
  "rover_msgs/Drive")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Drive)))
  "Returns string type for a message object of type 'Drive"
  "rover_msgs/Drive")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Drive>)))
  "Returns md5sum for a message object of type '<Drive>"
  "6dbf1ff82d4e5af7bd6088c316a10468")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Drive)))
  "Returns md5sum for a message object of type 'Drive"
  "6dbf1ff82d4e5af7bd6088c316a10468")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Drive>)))
  "Returns full string definition for message of type '<Drive>"
  (cl:format cl:nil "uint16 lw~%uint16 rw~%uint16 pan~%uint16 tilt~%uint8 camnum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Drive)))
  "Returns full string definition for message of type 'Drive"
  (cl:format cl:nil "uint16 lw~%uint16 rw~%uint16 pan~%uint16 tilt~%uint8 camnum~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Drive>))
  (cl:+ 0
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Drive>))
  "Converts a ROS message object to a list"
  (cl:list 'Drive
    (cl:cons ':lw (lw msg))
    (cl:cons ':rw (rw msg))
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
    (cl:cons ':camnum (camnum msg))
))
