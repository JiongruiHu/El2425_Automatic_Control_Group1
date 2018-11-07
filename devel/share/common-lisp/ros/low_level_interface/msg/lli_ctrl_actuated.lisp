; Auto-generated. Do not edit!


(cl:in-package low_level_interface-msg)


;//! \htmlinclude lli_ctrl_actuated.msg.html

(cl:defclass <lli_ctrl_actuated> (roslisp-msg-protocol:ros-message)
  ((steering
    :reader steering
    :initarg :steering
    :type cl:fixnum
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:fixnum
    :initform 0)
   (transmission
    :reader transmission
    :initarg :transmission
    :type cl:fixnum
    :initform 0)
   (differential_front
    :reader differential_front
    :initarg :differential_front
    :type cl:fixnum
    :initform 0)
   (differential_rear
    :reader differential_rear
    :initarg :differential_rear
    :type cl:fixnum
    :initform 0)
   (ctrl_code
    :reader ctrl_code
    :initarg :ctrl_code
    :type cl:fixnum
    :initform 0)
   (voltage
    :reader voltage
    :initarg :voltage
    :type cl:fixnum
    :initform 0)
   (current
    :reader current
    :initarg :current
    :type cl:fixnum
    :initform 0)
   (energy_consumed
    :reader energy_consumed
    :initarg :energy_consumed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass lli_ctrl_actuated (<lli_ctrl_actuated>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lli_ctrl_actuated>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lli_ctrl_actuated)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name low_level_interface-msg:<lli_ctrl_actuated> is deprecated: use low_level_interface-msg:lli_ctrl_actuated instead.")))

(cl:ensure-generic-function 'steering-val :lambda-list '(m))
(cl:defmethod steering-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:steering-val is deprecated.  Use low_level_interface-msg:steering instead.")
  (steering m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:velocity-val is deprecated.  Use low_level_interface-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'transmission-val :lambda-list '(m))
(cl:defmethod transmission-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:transmission-val is deprecated.  Use low_level_interface-msg:transmission instead.")
  (transmission m))

(cl:ensure-generic-function 'differential_front-val :lambda-list '(m))
(cl:defmethod differential_front-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:differential_front-val is deprecated.  Use low_level_interface-msg:differential_front instead.")
  (differential_front m))

(cl:ensure-generic-function 'differential_rear-val :lambda-list '(m))
(cl:defmethod differential_rear-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:differential_rear-val is deprecated.  Use low_level_interface-msg:differential_rear instead.")
  (differential_rear m))

(cl:ensure-generic-function 'ctrl_code-val :lambda-list '(m))
(cl:defmethod ctrl_code-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:ctrl_code-val is deprecated.  Use low_level_interface-msg:ctrl_code instead.")
  (ctrl_code m))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:voltage-val is deprecated.  Use low_level_interface-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:current-val is deprecated.  Use low_level_interface-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'energy_consumed-val :lambda-list '(m))
(cl:defmethod energy_consumed-val ((m <lli_ctrl_actuated>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader low_level_interface-msg:energy_consumed-val is deprecated.  Use low_level_interface-msg:energy_consumed instead.")
  (energy_consumed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lli_ctrl_actuated>) ostream)
  "Serializes a message object of type '<lli_ctrl_actuated>"
  (cl:let* ((signed (cl:slot-value msg 'steering)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'transmission)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'differential_front)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'differential_rear)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ctrl_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'voltage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'energy_consumed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lli_ctrl_actuated>) istream)
  "Deserializes a message object of type '<lli_ctrl_actuated>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'steering) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'transmission) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'differential_front) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'differential_rear) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ctrl_code) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'voltage) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'energy_consumed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lli_ctrl_actuated>)))
  "Returns string type for a message object of type '<lli_ctrl_actuated>"
  "low_level_interface/lli_ctrl_actuated")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lli_ctrl_actuated)))
  "Returns string type for a message object of type 'lli_ctrl_actuated"
  "low_level_interface/lli_ctrl_actuated")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lli_ctrl_actuated>)))
  "Returns md5sum for a message object of type '<lli_ctrl_actuated>"
  "6c801b9e27d9831fee1704e6d7985df4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lli_ctrl_actuated)))
  "Returns md5sum for a message object of type 'lli_ctrl_actuated"
  "6c801b9e27d9831fee1704e6d7985df4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lli_ctrl_actuated>)))
  "Returns full string definition for message of type '<lli_ctrl_actuated>"
  (cl:format cl:nil "int8 steering ~%int8 velocity~%int8 transmission~%int8 differential_front~%int8 differential_rear~%int8 ctrl_code~%int8 voltage~%int8 current~%int8 energy_consumed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lli_ctrl_actuated)))
  "Returns full string definition for message of type 'lli_ctrl_actuated"
  (cl:format cl:nil "int8 steering ~%int8 velocity~%int8 transmission~%int8 differential_front~%int8 differential_rear~%int8 ctrl_code~%int8 voltage~%int8 current~%int8 energy_consumed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lli_ctrl_actuated>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lli_ctrl_actuated>))
  "Converts a ROS message object to a list"
  (cl:list 'lli_ctrl_actuated
    (cl:cons ':steering (steering msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':transmission (transmission msg))
    (cl:cons ':differential_front (differential_front msg))
    (cl:cons ':differential_rear (differential_rear msg))
    (cl:cons ':ctrl_code (ctrl_code msg))
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':current (current msg))
    (cl:cons ':energy_consumed (energy_consumed msg))
))
