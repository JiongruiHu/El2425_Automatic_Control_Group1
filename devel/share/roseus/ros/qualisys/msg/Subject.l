;; Auto-generated. Do not edit!


(when (boundp 'qualisys::Subject)
  (if (not (find-package "QUALISYS"))
    (make-package "QUALISYS"))
  (shadow 'Subject (find-package "QUALISYS")))
(unless (find-package "QUALISYS::SUBJECT")
  (make-package "QUALISYS::SUBJECT"))

(in-package "ROS")
;;//! \htmlinclude Subject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass qualisys::Subject
  :super ros::object
  :slots (_header _name _occluded _position _orientation _markers ))

(defmethod qualisys::Subject
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:occluded __occluded) nil)
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:orientation __orientation) (instance geometry_msgs::Quaternion :init))
    ((:markers __markers) (let (r) (dotimes (i 0) (push (instance qualisys::Marker :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _occluded __occluded)
   (setq _position __position)
   (setq _orientation __orientation)
   (setq _markers __markers)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:occluded
   (&optional __occluded)
   (if __occluded (setq _occluded __occluded)) _occluded)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:markers
   (&rest __markers)
   (if (keywordp (car __markers))
       (send* _markers __markers)
     (progn
       (if __markers (setq _markers (car __markers)))
       _markers)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; bool _occluded
    1
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Quaternion _orientation
    (send _orientation :serialization-length)
    ;; qualisys/Marker[] _markers
    (apply #'+ (send-all _markers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _occluded
       (if _occluded (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Quaternion _orientation
       (send _orientation :serialize s)
     ;; qualisys/Marker[] _markers
     (write-long (length _markers) s)
     (dolist (elem _markers)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _occluded
     (setq _occluded (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Quaternion _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;; qualisys/Marker[] _markers
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _markers (let (r) (dotimes (i n) (push (instance qualisys::Marker :init) r)) r))
     (dolist (elem- _markers)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get qualisys::Subject :md5sum-) "678d5c79c0845b2430769c75a38d4e56")
(setf (get qualisys::Subject :datatype-) "qualisys/Subject")
(setf (get qualisys::Subject :definition-)
      "Header header
string name
bool occluded
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
qualisys/Marker[] markers

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: qualisys/Marker
string name
string subject_name
geometry_msgs/Point position
bool occluded

")



(provide :qualisys/Subject "678d5c79c0845b2430769c75a38d4e56")


