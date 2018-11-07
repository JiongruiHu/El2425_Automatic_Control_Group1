;; Auto-generated. Do not edit!


(when (boundp 'qualisys::Marker)
  (if (not (find-package "QUALISYS"))
    (make-package "QUALISYS"))
  (shadow 'Marker (find-package "QUALISYS")))
(unless (find-package "QUALISYS::MARKER")
  (make-package "QUALISYS::MARKER"))

(in-package "ROS")
;;//! \htmlinclude Marker.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass qualisys::Marker
  :super ros::object
  :slots (_name _subject_name _position _occluded ))

(defmethod qualisys::Marker
  (:init
   (&key
    ((:name __name) "")
    ((:subject_name __subject_name) "")
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:occluded __occluded) nil)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _subject_name (string __subject_name))
   (setq _position __position)
   (setq _occluded __occluded)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:subject_name
   (&optional __subject_name)
   (if __subject_name (setq _subject_name __subject_name)) _subject_name)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:occluded
   (&optional __occluded)
   (if __occluded (setq _occluded __occluded)) _occluded)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _subject_name
    4 (length _subject_name)
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; bool _occluded
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _subject_name
       (write-long (length _subject_name) s) (princ _subject_name s)
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; bool _occluded
       (if _occluded (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _subject_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _subject_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; bool _occluded
     (setq _occluded (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get qualisys::Marker :md5sum-) "a16c57ea269b234761b832931693cc90")
(setf (get qualisys::Marker :datatype-) "qualisys/Marker")
(setf (get qualisys::Marker :definition-)
      "string name
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



(provide :qualisys/Marker "a16c57ea269b234761b832931693cc90")


