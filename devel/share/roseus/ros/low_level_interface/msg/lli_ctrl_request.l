;; Auto-generated. Do not edit!


(when (boundp 'low_level_interface::lli_ctrl_request)
  (if (not (find-package "LOW_LEVEL_INTERFACE"))
    (make-package "LOW_LEVEL_INTERFACE"))
  (shadow 'lli_ctrl_request (find-package "LOW_LEVEL_INTERFACE")))
(unless (find-package "LOW_LEVEL_INTERFACE::LLI_CTRL_REQUEST")
  (make-package "LOW_LEVEL_INTERFACE::LLI_CTRL_REQUEST"))

(in-package "ROS")
;;//! \htmlinclude lli_ctrl_request.msg.html


(defclass low_level_interface::lli_ctrl_request
  :super ros::object
  :slots (_steering _velocity _transmission _differential_front _differential_rear _ctrl_code ))

(defmethod low_level_interface::lli_ctrl_request
  (:init
   (&key
    ((:steering __steering) 0)
    ((:velocity __velocity) 0)
    ((:transmission __transmission) 0)
    ((:differential_front __differential_front) 0)
    ((:differential_rear __differential_rear) 0)
    ((:ctrl_code __ctrl_code) 0)
    )
   (send-super :init)
   (setq _steering (round __steering))
   (setq _velocity (round __velocity))
   (setq _transmission (round __transmission))
   (setq _differential_front (round __differential_front))
   (setq _differential_rear (round __differential_rear))
   (setq _ctrl_code (round __ctrl_code))
   self)
  (:steering
   (&optional __steering)
   (if __steering (setq _steering __steering)) _steering)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:transmission
   (&optional __transmission)
   (if __transmission (setq _transmission __transmission)) _transmission)
  (:differential_front
   (&optional __differential_front)
   (if __differential_front (setq _differential_front __differential_front)) _differential_front)
  (:differential_rear
   (&optional __differential_rear)
   (if __differential_rear (setq _differential_rear __differential_rear)) _differential_rear)
  (:ctrl_code
   (&optional __ctrl_code)
   (if __ctrl_code (setq _ctrl_code __ctrl_code)) _ctrl_code)
  (:serialization-length
   ()
   (+
    ;; int8 _steering
    1
    ;; int8 _velocity
    1
    ;; int8 _transmission
    1
    ;; int8 _differential_front
    1
    ;; int8 _differential_rear
    1
    ;; int8 _ctrl_code
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _steering
       (write-byte _steering s)
     ;; int8 _velocity
       (write-byte _velocity s)
     ;; int8 _transmission
       (write-byte _transmission s)
     ;; int8 _differential_front
       (write-byte _differential_front s)
     ;; int8 _differential_rear
       (write-byte _differential_rear s)
     ;; int8 _ctrl_code
       (write-byte _ctrl_code s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _steering
     (setq _steering (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _steering 127) (setq _steering (- _steering 256)))
   ;; int8 _velocity
     (setq _velocity (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _velocity 127) (setq _velocity (- _velocity 256)))
   ;; int8 _transmission
     (setq _transmission (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _transmission 127) (setq _transmission (- _transmission 256)))
   ;; int8 _differential_front
     (setq _differential_front (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _differential_front 127) (setq _differential_front (- _differential_front 256)))
   ;; int8 _differential_rear
     (setq _differential_rear (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _differential_rear 127) (setq _differential_rear (- _differential_rear 256)))
   ;; int8 _ctrl_code
     (setq _ctrl_code (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ctrl_code 127) (setq _ctrl_code (- _ctrl_code 256)))
   ;;
   self)
  )

(setf (get low_level_interface::lli_ctrl_request :md5sum-) "89a86354005db1fc65181bcc6d834320")
(setf (get low_level_interface::lli_ctrl_request :datatype-) "low_level_interface/lli_ctrl_request")
(setf (get low_level_interface::lli_ctrl_request :definition-)
      "int8 steering
int8 velocity
int8 transmission
int8 differential_front
int8 differential_rear
int8 ctrl_code

")



(provide :low_level_interface/lli_ctrl_request "89a86354005db1fc65181bcc6d834320")


