;; Auto-generated. Do not edit!


(when (boundp 'qualisys::Markers)
  (if (not (find-package "QUALISYS"))
    (make-package "QUALISYS"))
  (shadow 'Markers (find-package "QUALISYS")))
(unless (find-package "QUALISYS::MARKERS")
  (make-package "QUALISYS::MARKERS"))

(in-package "ROS")
;;//! \htmlinclude Markers.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass qualisys::Markers
  :super ros::object
  :slots (_header _markers ))

(defmethod qualisys::Markers
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:markers __markers) (let (r) (dotimes (i 0) (push (instance qualisys::Marker :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _markers __markers)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
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
    ;; qualisys/Marker[] _markers
    (apply #'+ (send-all _markers :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
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

(setf (get qualisys::Markers :md5sum-) "6ba87eb5111a9609174c0acf8542f67d")
(setf (get qualisys::Markers :datatype-) "qualisys/Markers")
(setf (get qualisys::Markers :definition-)
      "Header header
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
MSG: qualisys/Marker
string name
string subject_name
geometry_msgs/Point position
bool occluded

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :qualisys/Markers "6ba87eb5111a9609174c0acf8542f67d")


