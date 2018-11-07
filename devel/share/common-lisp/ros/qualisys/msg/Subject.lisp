; Auto-generated. Do not edit!


(cl:in-package qualisys-msg)


;//! \htmlinclude Subject.msg.html

(cl:defclass <Subject> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (occluded
    :reader occluded
    :initarg :occluded
    :type cl:boolean
    :initform cl:nil)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (markers
    :reader markers
    :initarg :markers
    :type (cl:vector qualisys-msg:Marker)
   :initform (cl:make-array 0 :element-type 'qualisys-msg:Marker :initial-element (cl:make-instance 'qualisys-msg:Marker))))
)

(cl:defclass Subject (<Subject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Subject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Subject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name qualisys-msg:<Subject> is deprecated: use qualisys-msg:Subject instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Subject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qualisys-msg:header-val is deprecated.  Use qualisys-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Subject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qualisys-msg:name-val is deprecated.  Use qualisys-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'occluded-val :lambda-list '(m))
(cl:defmethod occluded-val ((m <Subject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qualisys-msg:occluded-val is deprecated.  Use qualisys-msg:occluded instead.")
  (occluded m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <Subject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qualisys-msg:position-val is deprecated.  Use qualisys-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <Subject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qualisys-msg:orientation-val is deprecated.  Use qualisys-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'markers-val :lambda-list '(m))
(cl:defmethod markers-val ((m <Subject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader qualisys-msg:markers-val is deprecated.  Use qualisys-msg:markers instead.")
  (markers m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Subject>) ostream)
  "Serializes a message object of type '<Subject>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'occluded) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'markers))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'markers))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Subject>) istream)
  "Deserializes a message object of type '<Subject>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'occluded) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'markers) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'markers)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'qualisys-msg:Marker))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Subject>)))
  "Returns string type for a message object of type '<Subject>"
  "qualisys/Subject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Subject)))
  "Returns string type for a message object of type 'Subject"
  "qualisys/Subject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Subject>)))
  "Returns md5sum for a message object of type '<Subject>"
  "678d5c79c0845b2430769c75a38d4e56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Subject)))
  "Returns md5sum for a message object of type 'Subject"
  "678d5c79c0845b2430769c75a38d4e56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Subject>)))
  "Returns full string definition for message of type '<Subject>"
  (cl:format cl:nil "Header header~%string name~%bool occluded~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%qualisys/Marker[] markers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: qualisys/Marker~%string name~%string subject_name~%geometry_msgs/Point position~%bool occluded~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Subject)))
  "Returns full string definition for message of type 'Subject"
  (cl:format cl:nil "Header header~%string name~%bool occluded~%geometry_msgs/Point position~%geometry_msgs/Quaternion orientation~%qualisys/Marker[] markers~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: qualisys/Marker~%string name~%string subject_name~%geometry_msgs/Point position~%bool occluded~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Subject>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'markers) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Subject>))
  "Converts a ROS message object to a list"
  (cl:list 'Subject
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':occluded (occluded msg))
    (cl:cons ':position (position msg))
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':markers (markers msg))
))
