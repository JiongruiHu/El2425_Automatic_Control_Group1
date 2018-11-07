
(cl:in-package :asdf)

(defsystem "low_level_interface-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "lli_ctrl_actuated" :depends-on ("_package_lli_ctrl_actuated"))
    (:file "_package_lli_ctrl_actuated" :depends-on ("_package"))
    (:file "lli_ctrl_request" :depends-on ("_package_lli_ctrl_request"))
    (:file "_package_lli_ctrl_request" :depends-on ("_package"))
  ))